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
#include "main.h"

#include "string.h"


/* Function declarations */
void UART_Init(uint32_t baudrate);
void UART_Close();
void UART_Flush();

uint32_t CDC_getPacketSize();
void CDC_readPacket(uint8_t *buf);
void CDC_writePacket(uint8_t *buf, uint32_t size);

void USB_pin_clk_init(void);

/* Private variables */
USBD_API_T   *pUsbApi;
USBD_HANDLE_T pUsbHandle;

//uint8_t		*tmpRxBuf;
//uint8_t		*tmpTxBuf;
volatile uint8_t tmpRxBuf[USB_HS_MAX_BULK_PACKET];
volatile uint8_t tmpTxBuf[USB_HS_MAX_BULK_PACKET];

/* UART Bridge variables */
volatile uint8_t CDC_UART_txBuffer[USB_HS_MAX_BULK_PACKET];
volatile uint32_t CDC_UART_txBufferSize;
volatile uint32_t CDC_UART_txBufferSent;

#define UART_RX_BUFFER_SIZE	(USB_HS_MAX_BULK_PACKET-1)	// -1 saves us from needing to send ZLP
volatile uint8_t UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
volatile uint16_t UART_RX_Index;

/* OSC variables */
#define CDC_OSC_RX_BUFFER_SIZE_N	7
#define CDC_OSC_RX_BUFFER_MASK		((1 << CDC_OSC_RX_BUFFER_SIZE_N) - 1)
volatile uint8_t  CDC_OSC_rxBuffer[1 << CDC_OSC_RX_BUFFER_SIZE_N];
volatile uint32_t CDC_OSC_rxBufferWritePos;
volatile uint32_t CDC_OSC_rxBufferReadPos;

#define CDC_OSC_rxBufferAvailable()  ((CDC_OSC_rxBufferWritePos-CDC_OSC_rxBufferReadPos) & CDC_OSC_RX_BUFFER_MASK)
#define CDC_OSC_rxBufferFree()       ((CDC_OSC_rxBufferReadPos-1-CDC_OSC_rxBufferWritePos) & CDC_OSC_RX_BUFFER_MASK)

volatile uint8_t CDC_OSC_txReady;

/* End of private variables */


/* USB and ISR handlers */
volatile uint8_t receiveLineCoding = 0;
ErrorCode_t EP0_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	if (receiveLineCoding) {
		receiveLineCoding = 3;
	}


	if (event == USB_EVT_SETUP) {
		USB_SETUP_PACKET packet;
		uint32_t len = pUsbApi->hw->ReadSetupPkt(hUsb, USB_ENDPOINT_OUT(0), (uint32_t*)&packet);

		if (receiveLineCoding) {
			receiveLineCoding = 0;
		}

		if ((packet.bmRequestType.B & 0x7F) == 0x21) { // Type=Class, Recipient=Interface
			if (packet.wIndex.W == 2) { // OSC CDC CIF interface (2)
				switch (packet.bRequest) {
					case 0x20: { // SET_LINE_CODING
						if (packet.wLength != 7)
							return ERR_USBD_INVALID_REQ;
						receiveLineCoding = 1;
						pUsbApi->hw->EnableEvent(pUsbHandle, 0x00, USB_EVT_OUT, 1);
						pUsbApi->hw->EnableEvent(pUsbHandle, 0x00, USB_EVT_IN, 1);
						//uint8_t lcsLen = pUsbApi->hw->ReadReqEP(pUsbHandle, USB_ENDPOINT_OUT(0), (uint8_t*)tmpRxBuf, 7);
						//return ERR_USBD_SEND_DATA;

						//uint8_t lcsLen = pUsbApi->hw->ReadReqEP(pUsbHandle, USB_ENDPOINT_OUT(0), (uint8_t*)tmpRxBuf, 64);
						//pUsbApi->hw->WriteEP(pUsbHandle, USB_ENDPOINT_IN(0), (uint8_t*)tmpRxBuf, 0);
						//return ERR_USBD_UNHANDLED;

						//pUsbApi->hw->WriteEP(pUsbHandle, USB_ENDPOINT_IN(0), NULL, 0);
						return ERR_USBD_UNHANDLED;
					}
					case 0x21: { // GET_LINE_CODING
						if (packet.wLength != 7)
							return ERR_USBD_INVALID_REQ;

						uint8_t lcs[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };
						pUsbApi->hw->WriteEP(pUsbHandle, USB_ENDPOINT_IN(0), lcs, 7);
						return LPC_OK;
					}
					case 0x22: { // SET_CONTROL_LINE_STATE
						if (packet.wLength != 0)
							return ERR_USBD_INVALID_REQ;

						pUsbApi->hw->WriteEP(pUsbHandle, USB_ENDPOINT_IN(0), NULL, 0);
						return LPC_OK;
					}
				}
			}
		}
	} else if (event == USB_EVT_OUT) {
		receiveLineCoding = 0;
		if (receiveLineCoding) {
			//uint8_t lcsLen = pUsbApi->hw->ReadEP(pUsbHandle, USB_ENDPOINT_OUT(0), (uint8_t*)tmpRxBuf);
			receiveLineCoding = 0;
			return LPC_OK;
		}
	} else if (event == USB_EVT_IN) {
		if (receiveLineCoding) {
			receiveLineCoding = 0;
		}
	}

	return ERR_USBD_UNHANDLED;
}


ErrorCode_t UART_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	return LPC_OK;
}

ErrorCode_t UART_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	switch (event) {
		case USB_EVT_OUT: {
			if (CDC_UART_txBufferSent == CDC_UART_txBufferSize) {
				CDC_UART_txBufferSize = pUsbApi->hw->ReadEP(hUsb, CDC_DIF1_BULK_OUT_EP, (uint8_t*)CDC_UART_txBuffer);
				CDC_UART_txBufferSent = 0;

				if (LPC_USART->LSR & BIT5) // THR register is empty
					LPC_USART->THR = CDC_UART_txBuffer[CDC_UART_txBufferSent++];
			}
			break;
		}
		default:
			break;
	}
	return LPC_OK;
}

ErrorCode_t OSC_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	CDC_OSC_txReady = 1;
	return LPC_OK;
}

ErrorCode_t OSC_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	switch (event) {
		case USB_EVT_OUT: {
			uint32_t rxLen = pUsbApi->hw->ReadEP(hUsb, CDC_DIF2_BULK_OUT_EP, (uint8_t*)tmpRxBuf);

			if (CDC_OSC_rxBufferFree() < rxLen) {	// Buffer overflow
				CDC_OSC_rxBufferWritePos = 0;
				CDC_OSC_rxBufferReadPos = 0;
			} else {
				uint8_t *ptr = (uint8_t*)tmpRxBuf;
				while (rxLen--) {
					CDC_OSC_rxBuffer[CDC_OSC_rxBufferWritePos & CDC_OSC_RX_BUFFER_MASK] = *ptr++;
					CDC_OSC_rxBufferWritePos++;
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

void UART_IRQHandler() {
	uint32_t flags = (LPC_USART->IIR >> 1) & 0x7; // parse interrupt flags

	// Combined code for efficiency? and fault handling
	if (flags == 0x3 || flags == 0x02 || flags == 0x06) {	// RLS, CTI or RDA interrupts
		// TODO: remove timer_serial and add native CTI support
		uint32_t lsr;
		while ((lsr = LPC_USART->LSR) & 0x9F) {	// while data is available
			if (lsr & 0x9E) {	// if there's any error - drop the byte
				LPC_USART->RBR;
			} else {			// else - buffer it
				UART_RX_BUFFER[UART_RX_Index++] = LPC_USART ->RBR;

				if (UART_RX_Index == UART_RX_BUFFER_SIZE) {	// if the buffer is full - send it out
					UART_Flush();
					timer_serial = TIMER_STOP;
				} else {	// else (re)set the timer
					timer_serial = 10;	// 10ms
				}
			}
		}
	} else if (flags == 0x01) {		// THRE interrupt
		if (CDC_UART_txBufferSent < CDC_UART_txBufferSize) {
			// assuming that THR is empty at interrupt
			LPC_USART->THR = CDC_UART_txBuffer[CDC_UART_txBufferSent++];
		}
	}
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
	usb_param.max_num_ep = 5;

	/* init CDC params */
	memset((void*) &cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));

	/* Initialize Descriptor pointers */
	memset((void*) &desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &VCOM_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &VCOM_StringDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];
	desc.high_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];

	/* USB Initialization */
	//uint32_t usbMemSize = pUsbApi->hw->GetMemSize(&usb_param);
	ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);

	if (ret != LPC_OK)
		return ret;

	// init CDC params
	cdc_param.mem_base = 0x10001500;
	cdc_param.mem_size = 0x300;
	cdc_param.cif_intf_desc = (uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE+8];
	cdc_param.dif_intf_desc = (uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE+8+ USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE];
	//cdc_param.cif_intf_desc = (uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE + 8 + USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE + USB_INTERFACE_DESC_SIZE + 2*USB_ENDPOINT_DESC_SIZE + 8];
	//cdc_param.dif_intf_desc = (uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE + 8 + USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE + USB_INTERFACE_DESC_SIZE + 2*USB_ENDPOINT_DESC_SIZE + 8 + USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE];


	//uint32_t cdcMemSize = pUsbApi->cdc->GetMemSize(&cdc_param);
	ret = pUsbApi->cdc->init(hUsb, &cdc_param, &hCdc);

	if (ret != LPC_OK)
		return ret;

	/* Initialize private data */
	pUsbHandle = hUsb;

	CDC_UART_txBufferSent = 0;
	CDC_UART_txBufferSize = 0;

	CDC_OSC_rxBufferReadPos = 0;
	CDC_OSC_rxBufferWritePos = 0;

	//tmpRxBuf = (uint8_t*) (cdc_param.mem_base + (0 * USB_HS_MAX_BULK_PACKET));
	//tmpTxBuf = (uint8_t*) (cdc_param.mem_base + (1 * USB_HS_MAX_BULK_PACKET));
	cdc_param.mem_size -= (4 * USB_HS_MAX_BULK_PACKET);

	CDC_OSC_txReady = 1;

	/* register UART Bridge endpoint interrupt handler */
	ep_indx = (((CDC_DIF1_BULK_IN_EP & 0x0F) << 1) + 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, UART_bulk_in_hdlr, NULL);

	if (ret != LPC_OK)
		return ret;

	/* register UART Bridge endpoint interrupt handler */
	ep_indx = ((CDC_DIF1_BULK_OUT_EP & 0x0F) << 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, UART_bulk_out_hdlr, NULL);

	if (ret != LPC_OK)
		return ret;

	/* register OSC endpoint interrupt handler */
	ep_indx = (((CDC_DIF2_BULK_IN_EP & 0x0F) << 1) + 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, OSC_bulk_in_hdlr, NULL);

	if (ret != LPC_OK)
		return ret;

	/* register OSC endpoint interrupt handler */
	ep_indx = ((CDC_DIF2_BULK_OUT_EP & 0x0F) << 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, OSC_bulk_out_hdlr, NULL);

	if (ret != LPC_OK)
		return ret;


	ret = pUsbApi->core->RegisterClassHandler(hUsb, EP0_hdlr, NULL);
	if (ret != LPC_OK)
		return ret;

	/* enable IRQ */
	NVIC_SetPriority(USB_IRQn, 0); // give highest priority to USB
	NVIC_EnableIRQ(USB_IRQn); //  enable USB0 interrrupts

	/* USB Connect */
	pUsbApi->hw->Connect(hUsb, 1);

	UART_Init(9600);

	stream->getPacketSize = CDC_getPacketSize;
	stream->readPacket = CDC_readPacket;
	stream->writePacket = CDC_writePacket;

	return LPC_OK;
}

/* Part 1: Functions for UART Bridge */

void UART_Init(uint32_t baudrate) {
	NVIC_DisableIRQ(UART_IRQn);

	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12); // enable AHB clock for UART
	LPC_SYSCON->UARTCLKDIV = 1; // 48MHz

	LPC_USART->LCR = 0x83; 		// 8bit, 1stop, no parity, enable DLAB
	uint32_t divider = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate;	// divider settings for baudrate
	LPC_USART->DLM = divider / 256;
	LPC_USART->DLL = divider % 256;
	LPC_USART->FDR = 0x10;		// no fractional divider, TODO: implement it
	LPC_USART->LCR &= ~0x80;	// disable DLAB
	LPC_USART->FCR = 0x07;		// enable and reset FIFO buffers
	LPC_USART->IER = 0; 		// All USART interrupts disabled

	while ((LPC_USART->LSR & (BIT5 | BIT6)) != (BIT5 | BIT6)); //clear TX

	while (LPC_USART->LSR & 0x9F) { // clear RX buffer and line errors
		LPC_USART->RBR;
	}
	UART_RX_Index = 0;

	LPC_USART->IER = BIT0 | BIT1 | BIT2;	// Enable RDA(+CRT), THRE and RLS interrupts
	NVIC_SetPriority(UART_IRQn, 2);
	NVIC_EnableIRQ(UART_IRQn);
}

void UART_Close() {
	NVIC_DisableIRQ(UART_IRQn);
	LPC_USART->IER = 0;

	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 12); // disable AHB clock for UART
}

void UART_Flush() {
	NVIC_DisableIRQ(UART_IRQn);

	if (UART_RX_Index != 0) {	// if the buffer is not empty
		pUsbApi->hw->WriteEP(pUsbHandle, CDC_DIF1_BULK_IN_EP, (uint8_t*)UART_RX_BUFFER, UART_RX_Index);
		UART_RX_Index = 0;
	}

	NVIC_EnableIRQ(UART_IRQn);
}


/* Part 2: Functions for OSC CDC port */

uint32_t CDC_getPacketSize() {
	uint32_t size = 0;
	uint32_t available = CDC_OSC_rxBufferAvailable();

	while (available > 4) { // while at least header + 1 byte is available
		if (CDC_OSC_rxBuffer[CDC_OSC_rxBufferReadPos & CDC_OSC_RX_BUFFER_MASK] == 'S' && CDC_OSC_rxBuffer[(CDC_OSC_rxBufferReadPos + 1) & CDC_OSC_RX_BUFFER_MASK] == 'T') {
			size = (CDC_OSC_rxBuffer[(CDC_OSC_rxBufferReadPos + 2) & CDC_OSC_RX_BUFFER_MASK] << 8)
				  | CDC_OSC_rxBuffer[(CDC_OSC_rxBufferReadPos + 3) & CDC_OSC_RX_BUFFER_MASK];
		}

		if (size == 0) {
			CDC_OSC_rxBufferReadPos++;
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

	CDC_OSC_rxBufferReadPos += 4;	// skip the header

	while (size--) {
		*buf++ = CDC_OSC_rxBuffer[CDC_OSC_rxBufferReadPos & CDC_OSC_RX_BUFFER_MASK];
		CDC_OSC_rxBufferReadPos++;
	}
}

void CDC_writePacket(uint8_t *buf, uint32_t size) {
	tmpTxBuf[0] = 'S';
	tmpTxBuf[1] = 'T';
	tmpTxBuf[2] = (size >> 8);
	tmpTxBuf[3] = size & 0xFF;

	memcpy((uint8_t*)(tmpTxBuf+4), buf, (size > 60 ? 60 : size));

	size += 4;

	uint32_t sent = 0;
	while (size > 0) {
		CDC_OSC_txReady = 0;
		sent = pUsbApi->hw->WriteEP(pUsbHandle, CDC_DIF2_BULK_IN_EP, (uint8_t*)tmpTxBuf, (size > 64 ? 64 : size));
		buf += sent;
		size -= sent;
		while (!CDC_OSC_txReady); // block until data is sent

		if (size > 0)
			memcpy((uint8_t*)tmpTxBuf, buf, (size > 64 ? 64 : size));
	}

	if (sent == 64) { // if final packet was full
		pUsbApi->hw->WriteEP(pUsbHandle, CDC_DIF2_BULK_IN_EP, NULL, 0); // send ZLP
	}
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

	LPC_IOCON->PIO0_18 &= ~0x07;
	LPC_IOCON->PIO0_18 |= 0x01;
	LPC_IOCON->PIO0_19 &= ~0x07;
	LPC_IOCON->PIO0_19 |= 0x01;

	return;
}
