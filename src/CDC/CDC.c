/**
 * @file	CDC.c
 * @author  Giedrius Medzevicius <giedrius@8devices.com>
 *
 * @section LICENSE
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 UAB 8devices
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

/* Type declarations */
typedef enum {
	STOP_BIT_1 = 0,
	STOP_BIT_1_5 = 1,
	STOP_BIT_2 = 2,
} stop_bits_t;

typedef enum {
	PARITY_NONE = 0,
	PARITY_ODD  = 1,
	PARITY_EVEN = 2,
	PARITY_MARK = 3,
	PARITY_SPACE= 4,
} parity_t;


/* Function declarations */
void UART_Init(uint32_t baudrate, uint8_t dataBits, parity_t parity, stop_bits_t stopBits);
void UART_Close();
void UART_Flush();

uint32_t CDC_getPacketSize();
void CDC_readPacket(uint8_t *buf);
void CDC_writePacket(uint8_t *buf, uint32_t size);

void USB_pin_clk_init(void);

/* Private variables */
USBD_API_T   *pUsbApi;
USBD_HANDLE_T pUsbHandle;

volatile uint8_t tmpRxBuf[USB_HS_MAX_BULK_PACKET];
volatile uint8_t tmpTxBuf[USB_HS_MAX_BULK_PACKET];

/* UART Bridge variables */

volatile struct {
	uint32_t baudrate;
	stop_bits_t stopBits;
	parity_t parity;
	uint8_t dataBits;
} CDC_UART_Config;

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
#define REQ_TYPE( direction, type, recipient )  ((direction<<7)|(type<<5)|recipient)

ErrorCode_t EP0_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {

	USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
	static USB_SETUP_PACKET packet;
	pUsbApi->hw->ReadSetupPkt( hUsb, USB_ENDPOINT_OUT(0), (uint32_t *)&packet );

	switch (event) {
		case USB_EVT_SETUP:

			if (packet.bmRequestType.B == 0x80 // Setup Device to Host
					&& packet.bRequest == 0x06 // Get descriptor
					&& packet.wValue.WB.H == 0x06 // Get Device Qualifier Descriptor
				) {
				uint8_t dq[] = { 0x0A, USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE,
						WBVAL(0x0200), 0xEF, 0x02, 0x01, USB_MAX_PACKET0, 0x01,
						0x00 };
				pUsbApi->hw->WriteEP(pUsbHandle, USB_ENDPOINT_IN(0), dq, 10);
				return LPC_OK;
			}


			pCtrl->EP0Data.Count = packet.wLength;   // Number of bytes to transfer

			if ( (packet.bmRequestType.B == REQ_TYPE(REQUEST_HOST_TO_DEVICE,REQUEST_CLASS,REQUEST_TO_INTERFACE) )
				  && (packet.bRequest        == 0x20 ) // SetLineCoding
				  && (packet.wValue.W        == (0<<8) )    // descriptor type | index
				  && ((packet.wIndex.W == 0) || (packet.wIndex.W == 2))
				) {

				pCtrl->EP0Data.pData = pCtrl->EP0Buf;
				pCtrl->EP0Data.Count = 7;
				//pUsbApi->core->DataOutStage( hUsb );
				pUsbApi->core->StatusInStage(hUsb);
				return LPC_OK;
			}

			if ( (packet.bmRequestType.B == REQ_TYPE(REQUEST_DEVICE_TO_HOST,REQUEST_CLASS,REQUEST_TO_INTERFACE) )
				  && (packet.bRequest        == 0x21 ) // GetLineCoding
				  && (packet.wValue.W        == (0<<8) )  // Report type | Report ID
				  && ((packet.wIndex.W == 0) || (packet.wIndex.W == 2))
				) {
				uint8_t lcs[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 }; // Default 9600 8n1

				if (packet.wIndex.W == 0) {
					lcs[0] = CDC_UART_Config.baudrate;
					lcs[1] = CDC_UART_Config.baudrate >> 8;
					lcs[2] = CDC_UART_Config.baudrate >> 16;
					lcs[3] = CDC_UART_Config.baudrate >> 24;

					lcs[4] = CDC_UART_Config.stopBits;

					lcs[5] = CDC_UART_Config.parity;

					lcs[6] = CDC_UART_Config.dataBits;
				}

				pCtrl->EP0Data.Count = 7;
				pCtrl->EP0Data.pData = (uint8_t*)&lcs;
				pUsbApi->core->DataInStage(hUsb);

				return LPC_OK;
			}

			if ( (packet.bmRequestType.B == REQ_TYPE(REQUEST_HOST_TO_DEVICE,REQUEST_CLASS,REQUEST_TO_INTERFACE) )
				  && (packet.bRequest        == 0x22 ) // SetControlLineState
				  && ((packet.wIndex.W == 0) || (packet.wIndex.W == 2)) // Both interfaces
				) {
				pUsbApi->core->StatusInStage(hUsb);
				return LPC_OK;
			}

			break;
		case USB_EVT_OUT:
			if (pCtrl->EP0Data.Count > 0) {
				pUsbApi->core->DataOutStage(hUsb);
			} else {
				pUsbApi->core->StatusInStage(hUsb);

				if ( (packet.bmRequestType.B == REQ_TYPE(REQUEST_HOST_TO_DEVICE,REQUEST_CLASS,REQUEST_TO_INTERFACE) )
					  && (packet.bRequest        == 0x20 ) // SetLineCoding
					  && (packet.wValue.W        == (0<<8) )    // descriptor type | index
					  && ((packet.wIndex.W == 0))
					) {
					uint8_t *ptr = pCtrl->EP0Buf;
					uint32_t baudrate = *ptr | (*(ptr+1) << 8) | (*(ptr+2) << 16) | (*(ptr+3) << 24);
					ptr += 4;
					uint8_t stopbits = *ptr++;
					uint8_t parity = *ptr++;
					uint8_t dataBits = *ptr++;

					UART_Init(baudrate, dataBits, parity, stopbits);
				}
			}

			return LPC_OK;
			break;

		case USB_EVT_IN:
			/*if (pCtrl->EP0Data.Count > 0) {
				pUsbApi->core->DataInStage( hUsb );
				return LPC_OK;
			} else {
				pUsbApi->core->StatusOutStage( hUsb );
				return LPC_OK;
			}*/

			break;

		default:
			break;
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
	USB_CORE_DESCS_T desc;
	USBD_HANDLE_T hUsb;
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
	usb_param.max_num_ep = 10;

	/* Initialize Descriptor pointers */
	memset((void*) &desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &VCOM_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &VCOM_StringDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];
	desc.high_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];
	//desc.device_qualifier = (uint8_t*) &VCOM_DeviceQualifier[0];

	/* USB Initialization */
	//uint32_t usbMemSize = pUsbApi->hw->GetMemSize(&usb_param);
	ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);

	if (ret != LPC_OK)
		return ret;

	/* Initialize private data */
	pUsbHandle = hUsb;

	CDC_UART_txBufferSent = 0;
	CDC_UART_txBufferSize = 0;

	CDC_OSC_rxBufferReadPos = 0;
	CDC_OSC_rxBufferWritePos = 0;

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

	/* register EP0 handler */
	ret = pUsbApi->core->RegisterClassHandler(hUsb, EP0_hdlr, NULL);
	if (ret != LPC_OK)
		return ret;

	/* enable IRQ */
	NVIC_SetPriority(USB_IRQn, 0); // give highest priority to USB
	NVIC_EnableIRQ(USB_IRQn); //  enable USB0 interrrupts

	/* USB Connect */
	pUsbApi->hw->Connect(hUsb, 1);

	//UART_Init(9600, 8, PARITY_NONE, STOP_BIT_1); // 9600 8n1

	stream->getPacketSize = CDC_getPacketSize;
	stream->readPacket = CDC_readPacket;
	stream->writePacket = CDC_writePacket;

	return LPC_OK;
}

/* Part 1: Functions for UART Bridge */

void UART_Init(uint32_t baudrate, uint8_t dataBits, parity_t parity, stop_bits_t stopBits) {
	NVIC_DisableIRQ(UART_IRQn);

	if (baudrate < 46 || baudrate > 3000000)
		baudrate = 9600;

	if ((stopBits != STOP_BIT_1) && (stopBits != STOP_BIT_2)) // only 1 and 2 stop bits supported
		stopBits = STOP_BIT_1;

	if (dataBits > 8 || dataBits < 5)
		dataBits = 8;

	CDC_UART_Config.baudrate = baudrate;
	CDC_UART_Config.stopBits = stopBits;
	CDC_UART_Config.parity = parity;
	CDC_UART_Config.dataBits = dataBits;


	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12); // enable AHB clock for UART
	LPC_SYSCON->UARTCLKDIV = 1; // 48MHz

	LPC_USART->LCR = (dataBits - 5) | ((stopBits == STOP_BIT_1 ? 0 : 1) << 2)
			| ((parity == PARITY_NONE ? 0 : 1) << 3) | (((parity - 1) & 0x3) << 4)
			| BIT7;
	//LPC_USART->LCR = 0x83; 		// 8bit, 1stop, no parity, enable DLAB
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
