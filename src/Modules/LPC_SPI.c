/**
 * @file	LPC_SPI.c
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


#include "Modules/LPC_SPI.h"

void lpc_spi0_begin(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 2)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' || OSCMessage_getArgumentType(msg, 1) != 'i')
		return;

	uint32_t divider = (OSCMessage_getArgument_int32(msg, 0)-1) & 0xFF;
	uint32_t mode = OSCMessage_getArgument_int32(msg, 1) & 0x3;

/*	LPC_IOCON->PIO0_2 &= ~0x7;
	LPC_IOCON->PIO0_2 |= 0 | (2 << 3); // SPI0 SSEL as GPIO output, pull-up
	LPC_GPIO->DIR[0] |= BIT2;	// GPIO0_2 - output
	LPC_GPIO->SET[0] |= BIT2;	// Set high

	LPC_IOCON->PIO0_8 &= ~0x7;
	LPC_IOCON->PIO0_8 |= 1; // SPI0 MISO
	LPC_IOCON->PIO0_9 &= ~0x7;
	LPC_IOCON->PIO0_9 |= 1; // SPI0 MOSI
	LPC_IOCON->PIO1_29 &= ~0x7;
	LPC_IOCON->PIO1_29 |= 1; // SPI0 MISO
*/

	LPC_SYSCON->PRESETCTRL |= 1; 		// de-assert SPI0
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT11;	// enable SPI0 clock
	LPC_SYSCON->SSP0CLKDIV = 1; //48MHz


	LPC_SSP0->CR1 = 0;		// Master mode, SPI disabled
	LPC_SSP0->CPSR = 24;	// 48MHz/24 = 2MHz
	LPC_SSP0->CR0 = (0x7) | (0 << 4) | (mode << 6) | (divider << 8); // 8bits, SPI mode x
	LPC_SSP0->IMSC = 0;		// Interrupts disabled
	LPC_SSP0->CR1 = BIT1;	//Master mode, SPI enabled

	while (LPC_SSP0->SR & BIT4);	// wait while BUSY (reading or writing)

	while (LPC_SSP0->SR & BIT2) {	// Read while Rx FIFO not empty
		LPC_SSP0->DR;
	}
}

void lpc_spi0_trans(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 2)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'b' || OSCMessage_getArgumentType(msg, 1) != 'i') return;

	uint32_t dataSize, writeSize;
	uint8_t *data = OSCMessage_getArgument_blob(msg, 0, &dataSize);

	uint32_t readSize = writeSize = dataSize;
	uint8_t *readBuf = NULL, *readPtr = NULL;

	uint8_t requestRead =  OSCMessage_getArgument_int32(msg, 1) & 0x1;
	if (requestRead) {
		readBuf = (uint8_t*)MemoryManager_malloc(writeSize);
		readPtr = readBuf;
	}

	//LPC_GPIO->CLR[0] |= BIT2;	// Set low - slave select

	while (writeSize || readSize) {
		while (writeSize && (LPC_SSP0->SR & BIT1)) { // Tx FIFO not full
			LPC_SSP0->DR = *data++;
			writeSize--;
		}

		while (readSize && (LPC_SSP0->SR & BIT2)) { // Rx FIFO not empty
			uint32_t tmp = LPC_SSP0->DR;
			readSize--;
			if (readBuf != NULL)
				*readPtr++ = tmp;
		}
	}
	//LPC_GPIO->SET[0] |= BIT2;	// Set high - deselect slave

	if (readBuf != NULL) {
		OSCMessage *msg2 = OSCMessage_new();
		OSCMessage_setAddress(msg2, "/lpc/spi0/trans");
		OSCMessage_addArgument_blob(msg2, readBuf, dataSize);
		OSCMessage_sendMessage(msg2, &stream);
		OSCMessage_delete(msg2);
		MemoryManager_free(readBuf);
	}
}

void lpc_spi0_end(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 0)
		return;

	LPC_SSP0->CR1 = 0;		// SPI disabled
	LPC_SYSCON->SYSAHBCLKCTRL &= ~BIT11;	// disable SPI0 clock
	LPC_SYSCON->PRESETCTRL &= ~1; 			// assert SPI0
}
