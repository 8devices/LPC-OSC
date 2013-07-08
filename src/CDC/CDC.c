/**
 * @file	CDC.c
 * @author  Giedrius Medzevicius <giedrius@8devices.com>
 *
 * @section LICENSE
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Giedrius Medzevicius
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 */

#include "CDC/CDC.h"

#include "string.h"


/* Function declarations */
uint32_t CDC_getPacketSize();

void CDC_readPacket(uint8_t *buf);

void CDC_writePacket(uint8_t *buf, uint32_t size);

void USB_pin_clk_init(void);

/* Private variable declaration */
USBD_API_T   *pUsbApi;
USBD_HANDLE_T pUsbHandle;

uint8_t		*tmpRxBuf;

#define RX_BUFFER_SIZE_N	7
#define RX_BUFFER_MASK		((1 << RX_BUFFER_SIZE_N) - 1)
volatile uint8_t  rxBuffer[1 << RX_BUFFER_SIZE_N];
volatile uint32_t rxBufferWritePos;
volatile uint32_t rxBufferReadPos;

#define rxBufferAvailable()  ((rxBufferWritePos-rxBufferReadPos) & RX_BUFFER_MASK)
#define rxBufferFree()       ((rxBufferReadPos-1-rxBufferWritePos) & RX_BUFFER_MASK)

ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	return LPC_OK;
}

ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {

	switch (event) {
		case USB_EVT_OUT: {
			uint32_t rxLen = pUsbApi->hw->ReadEP(hUsb, USB_CDC_EP_BULK_OUT, tmpRxBuf);

			if (rxBufferFree() < rxLen) {	// Buffer overflow
				rxBufferWritePos = 0;
				rxBufferReadPos = 0;
			} else {
				uint8_t *ptr = tmpRxBuf;
				while (rxLen--) {
					rxBuffer[rxBufferWritePos & RX_BUFFER_MASK] = *ptr++;
					rxBufferWritePos++;
				}
			}
			break;
		}
		default:
			break;
	}
	return LPC_OK;
}

void USB_IRQHandler(void) {
	pUsbApi->hw->ISR(pUsbHandle);
}

ErrorCode_t CDC_Init(OSCPacketStream *stream) {
	USBD_API_INIT_PARAM_T usb_param;
	USBD_CDC_INIT_PARAM_T cdc_param;
	USB_CORE_DESCS_T desc;
	USBD_HANDLE_T hUsb, hCdc;
	ErrorCode_t ret = LPC_OK;
	uint32_t ep_indx;

	/* get USB API table pointer */
	pUsbApi = (USBD_API_T*) ((*(ROM **) (0x1FFF1FF8))->pUSBD);

	/* enable clocks and pinmux for usb0 */
	USB_pin_clk_init();

	/* initialize call back structures */
	memset((void*) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB_BASE;
	usb_param.mem_base = 0x10001000;
	usb_param.mem_size = 0x1000;
	usb_param.max_num_ep = 3;

	/* init CDC params */
	memset((void*) &cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));

	/* Initialize Descriptor pointers */
	memset((void*) &desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &VCOM_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &VCOM_StringDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];
	desc.high_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];

	/* USB Initialization */
	ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);

	if (ret != LPC_OK)
		return ret;

	// init CDC params
	cdc_param.mem_base = 0x10001500;
	cdc_param.mem_size = 0x300;
	cdc_param.cif_intf_desc = (uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE];
	cdc_param.dif_intf_desc = (uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE+ USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE];

	ret = pUsbApi->cdc->init(hUsb, &cdc_param, &hCdc);

	if (ret != LPC_OK)
		return ret;

	/* Initialize private data */
	pUsbHandle = hUsb;

	rxBufferReadPos = 0;
	rxBufferWritePos = 0;

	tmpRxBuf = (uint8_t*) (cdc_param.mem_base + (0 * USB_HS_MAX_BULK_PACKET));
	cdc_param.mem_size -= (4 * USB_HS_MAX_BULK_PACKET);

	/* register endpoint interrupt handler */
	ep_indx = (((USB_CDC_EP_BULK_IN & 0x0F) << 1) + 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_in_hdlr, NULL);

	if (ret != LPC_OK)
		return ret;

	/* register endpoint interrupt handler */
	ep_indx = ((USB_CDC_EP_BULK_OUT & 0x0F) << 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_out_hdlr, NULL);

	if (ret != LPC_OK)
		return ret;

	/* enable IRQ */
	NVIC_EnableIRQ(USB_IRQn); //  enable USB0 interrrupts

	/* USB Connect */
	pUsbApi->hw->Connect(hUsb, 1);

	stream->getPacketSize = CDC_getPacketSize;
	stream->readPacket = CDC_readPacket;
	stream->writePacket = CDC_writePacket;

	return LPC_OK;
}

uint32_t CDC_getPacketSize() {
	uint32_t size = 0;
	uint32_t available = rxBufferAvailable();

	while (available > 4) { // while at least header + 1 byte is available
		if (rxBuffer[rxBufferReadPos & RX_BUFFER_MASK] == 'S' && rxBuffer[(rxBufferReadPos + 1) & RX_BUFFER_MASK] == 'T') {
			size = (rxBuffer[(rxBufferReadPos + 2) & RX_BUFFER_MASK] << 8)
				  | rxBuffer[(rxBufferReadPos + 3) & RX_BUFFER_MASK];
		}

		if (size == 0) {
			rxBufferReadPos++;
			available--;
		} else {
			if (available + 4 >= size) // packet (+header) is ready
				return size;
			else 	// packet not yet arrived
				return 0;
		}
	}

	return 0;
}

void CDC_readPacket(uint8_t *buf) {
	uint32_t size = CDC_getPacketSize();

	rxBufferReadPos += 4;	// skip the header

	while (size--) {
		*buf++ = rxBuffer[rxBufferReadPos & RX_BUFFER_MASK];
		rxBufferReadPos++;
	}
}

void inline CDC_writeData(uint8_t *buf, uint32_t size) {
	while (size > 0) {
		uint32_t sent = pUsbApi->hw->WriteEP(pUsbHandle, USB_CDC_EP_BULK_IN, buf, size);
		buf += sent;
		size -= sent;
	}
}

void CDC_writePacket(uint8_t *buf, uint32_t size) {
	/*uint8_t header[] = { 'S', 'T', '\0', '\0' };

	header[2] = (size >> 8) & 0xFF;
	header[3] = size & 0xFF;

	CDC_writeData(header, 4); // This is bugged at the moment */
	CDC_writeData(buf, size);
}


void USB_pin_clk_init(void) {
	/* Enable AHB clock to the GPIO domain. */
	LPC_SYSCON ->SYSAHBCLKCTRL |= (1 << 6);

	/* Enable AHB clock to the USB block and USB RAM. */
	LPC_SYSCON ->SYSAHBCLKCTRL |= ((0x1 << 14) | (0x1 << 27));

	/* Pull-down is needed, or internally, VBUS will be floating. This is to
	 address the wrong status in VBUSDebouncing bit in CmdStatus register. It
	 happens on the NXP Validation Board only that a wrong ESD protection chip is used. */
	LPC_IOCON ->PIO0_3 &= ~0x1F;
//  LPC_IOCON->PIO0_3   |= ((0x1<<3)|(0x01<<0));	/* Secondary function VBUS */
	LPC_IOCON ->PIO0_3 |= (0x01 << 0); /* Secondary function VBUS */
	LPC_IOCON ->PIO0_6 &= ~0x07;
	LPC_IOCON ->PIO0_6 |= (0x01 << 0); /* Secondary function SoftConn */

	return;
}
