/**
 * @file	LPC_UART.c
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

#include "Modules/LPC_UART.h"

volatile uint8_t UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
volatile uint16_t UART_RX_Index;


void lpc_serial_begin(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i')
		return;

	uint32_t baudrate = OSCMessage_getArgument_int32(msg, 0);

	NVIC_DisableIRQ(UART_IRQn);

	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12); // enable AHB clock for UART
	LPC_SYSCON->UARTCLKDIV = 1; // 48MHz

	LPC_USART->LCR = 0x83; 		// 8bit, 1stop, no parity, enable DLAB
	uint32_t divider = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate;	// divider settings for baudrate
	LPC_USART->DLM = divider / 256;
	LPC_USART->DLL = divider % 256;
	LPC_USART->FDR = 0x10;		// no fractional divider, TODO: implement it
	LPC_USART->LCR &= ~0x80;	// disable DLAB
	LPC_USART->FCR = 0x07;		// enable FIFO buffers and reset XXX: debugger reads this (write only) register as 0xC1
	LPC_USART->IER = 0; 		// All USART interrupts disabled


	while ((LPC_USART->LSR & (BIT5 | BIT6)) != (BIT5 | BIT6)); //clear TX

	while (LPC_USART->LSR & 0x9F) { // clear RX buffer and line errors
		LPC_USART->RBR;
	}
	UART_RX_Index = 0;

	LPC_USART->IER = BIT0 | BIT2;	// Enable RDA and RLS interrupts
	NVIC_EnableIRQ(UART_IRQn);

}

void lpc_serial_write(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1)
		return;

	char type = OSCMessage_getArgumentType(msg, 0);
	switch (type) {
		case 'i':
			while (!(LPC_USART ->LSR & BIT5));
			LPC_USART ->THR = OSCMessage_getArgument_int32(msg, 0) & 0xFF;
			break;
		case 's': {
			char *str = OSCMessage_getArgument_string(msg, 0);
			while (*str != '\0') {
				while (!(LPC_USART ->LSR & BIT5));
				LPC_USART->THR = *str++ & 0xFF;
			}
			break;
		}
		case 'b': {
			uint32_t size;
			uint8_t *buf = OSCMessage_getArgument_blob(msg, 0, &size);
			while (size--) {
				while (!(LPC_USART->LSR & BIT5));
				LPC_USART->THR = *buf++ & 0xFF;
			}
			break;
		}
	}
}

void lpc_serial_end(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 0)
		return;

	NVIC_DisableIRQ(UART_IRQn);
	LPC_USART->IER = 0;

	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 12); // disable AHB clock for UART
}

void UART_Flush() {
	NVIC_DisableIRQ(UART_IRQn);

	if (UART_RX_Index != 0) {	// if the buffer is not empty
		OSCMessage *msg = OSCMessage_new();
		if (msg != NULL ) {
			OSCMessage_setAddress(msg, "/lpc/serial/read");
			OSCMessage_addArgument_blob(msg, (uint8_t*) &UART_RX_BUFFER, UART_RX_Index);
			OSCMessage_sendMessage(msg, &stream);
			OSCMessage_delete(msg);
		}
		UART_RX_Index = 0;
	}

	NVIC_EnableIRQ(UART_IRQn);
}

void UART_IRQHandler() {
	uint32_t flags = (LPC_USART->IIR >> 1) & 0x7; // parse interrupt flags

	/*if (flags == 0x3) {	// Receive Line Status interrupts
		while (LPC_USART->LSR & 0x9E) {
			LPC_USART->RBR;
		}
		if (LPC_USART->LSR & 0x1) {
			while (1);	// TODO: fix it
		}
	} else if (flags == 0x2) {	// Receive Data Available interrupt
		if (LPC_USART->LSR & 0x9E) {
			while (1);	// TODO: fix it
		}
		while (LPC_USART->LSR & BIT0) {	// while data is available
			UART_RX_BUFFER[UART_RX_Index++] = LPC_USART->RBR;

			if (UART_RX_Index == UART_RX_BUFFER_SIZE) {	// if the buffer is full - send it out
				OSCMessage *msg = OSCMessage_new();	// XXX: bad (slow) thing to do in the interrupt
				if (msg != NULL) {
					OSCMessage_setAddress(msg, "/lpc/uart/read");
					OSCMessage_addArgument_blob(msg, (uint8_t*)&UART_RX_BUFFER, UART_RX_BUFFER_SIZE);
					OSCMessage_sendMessage(msg, &stream);
					OSCMessage_delete(msg);
				}
				UART_RX_Index = 0;
			}
		}
	}*/

	// Combined code for efficiency? and fault handling
	if (flags == 0x3 || flags == 0x02) {	// RLS or RDA
		uint32_t lsr;
		while ((lsr = LPC_USART ->LSR) & 0x9F) {	// while data is available
			if (lsr & 0x9E) {	// if there's any error - drop the byte
				LPC_USART ->RBR;
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
	}
}
