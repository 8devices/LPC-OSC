/**
 * @file	LPC_I2C.c
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


#include "Modules/LPC_I2C.h"


struct {
	enum {
		I2C_IDLE=0,
		I2C_START, 	// First start bit sent
		I2C_SLAW,	// Slave Address + W sent
		I2C_DATAW,	// Data write
		I2C_RESTART,// Second start bit sent
		I2C_SLAR,	// Slave Address + R sent
		I2C_DATAR,	// Data read
	} status;
	uint32_t error;

	uint8_t slaveAddress;

	uint32_t writeSize;	// Number of (remaining) bytes to write
	uint32_t readSize;	// Number of (remaining) bytes to  read
	uint32_t readCount;	// Number of actually read bytes

	uint8_t *writePtr;
	uint8_t *readPtr;
} I2CHandler;

static inline void I2C_ERROR() {
	I2CHandler.error = 1;
	I2CHandler.status = I2C_IDLE;
	LPC_I2C->CONSET = BIT4; // send STOP
}

volatile uint8_t steps[32];
volatile uint32_t stepInd = 0;

void I2C_IRQHandler(void) {
	uint8_t status = LPC_I2C->STAT;

	if (stepInd < 32)
		steps[stepInd++] = status;

	//LPC_I2C->CONCLR = BIT3 | BIT4 | BIT5; // Clear interrupt flag , start and stop bits

	if (status == 0xF8) {
		LPC_I2C->CONCLR = BIT3;
		return;
	}

	switch (status) {
		case 0x08: {	// START
			/*LPC_I2C->CONCLR = BIT5;//clear start;
			if (I2CHandler.status != I2C_START) { // deny unknown i2c sources
				I2C_ERROR();
			} else {
				if (I2CHandler.writeSize > 0) {
					I2CHandler.status = I2C_SLAW;
					LPC_I2C->DAT = I2CHandler.slaveAddress << 1; // Send SLA+W
				} else {
					I2CHandler.status = I2C_SLAR;
					LPC_I2C->DAT = (I2CHandler.slaveAddress << 1) | 1; // Send SLA+R
				}
			}*/
			if (I2CHandler.writeSize > 0) {
				I2CHandler.status = I2C_SLAW;
				LPC_I2C ->DAT = I2CHandler.slaveAddress << 1; // Send SLA+W
			} else {
				I2CHandler.status = I2C_SLAR;
				LPC_I2C ->DAT = (I2CHandler.slaveAddress << 1) | 1; // Send SLA+R
			}
			LPC_I2C->CONCLR = BIT5;
			break;
		}
		case 0x18: {	// SLAW + ACK
			if (I2CHandler.status == I2C_SLAW) {
				I2CHandler.status = I2C_DATAW;
				LPC_I2C->DAT = *I2CHandler.writePtr++;
				I2CHandler.writeSize--;
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x20: {	// SLAW + NACK
			if (I2CHandler.status == I2C_SLAW) {
				I2CHandler.error = 0x20;
				I2CHandler.status = I2C_IDLE;
				LPC_I2C->CONSET = BIT4;//set stop
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x28: {	// DATAW + ACK
			if (I2CHandler.status == I2C_DATAW) {
				if (I2CHandler.writeSize > 0) {
					LPC_I2C->DAT = *I2CHandler.writePtr++;
					I2CHandler.writeSize--;
				} else {
					if (I2CHandler.readSize > 0) {
						I2CHandler.status = I2C_RESTART;
						LPC_I2C->CONSET = BIT5;		// Repeat START
					} else {
						I2CHandler.status = I2C_IDLE;
						LPC_I2C->CONSET = BIT4;		// send STOP
					}
				}
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x30: {	// DATAW + NACK
			if (I2CHandler.status == I2C_DATAW) {
				I2CHandler.error = 0x30;
				I2CHandler.status = I2C_IDLE;
				LPC_I2C->CONSET = BIT4;	// send STOP
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x10: {	// Repeated START XXX: clear START bit??
			if (I2CHandler.status == I2C_RESTART) {
				I2CHandler.status = I2C_SLAR;
				LPC_I2C->DAT = I2CHandler.slaveAddress << 1 | 1; // Send SLA+R
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x40: {	// SLAR + ACK
			if (I2CHandler.status == I2C_SLAR) {
				I2CHandler.status = I2C_DATAR;
				if (I2CHandler.readSize == 1) {	// if it's the last byte
					LPC_I2C->CONSET = BIT2;	// Return NACK
				}
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x48: {	// SLAR + NACK
			if (I2CHandler.status == I2C_SLAR) {
				I2CHandler.error = 0x48;
				I2CHandler.status = I2C_IDLE;
				//LPC_I2C->CONCLR = BIT5;
				LPC_I2C->CONSET = BIT4;	// send STOP
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x50: {	// DATAR + ACK
			if (I2CHandler.status == I2C_DATAR) {
				*I2CHandler.readPtr++ = LPC_I2C->DAT;
				I2CHandler.readCount++;
				I2CHandler.readSize--;

				if (I2CHandler.readSize == 1) {	// if it's the last byte
					LPC_I2C->CONSET = BIT2;	// Return NACK
				}
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x58: {	// DATAR + NACK
			if (I2CHandler.status == I2C_DATAR) {
				*I2CHandler.readPtr++ = LPC_I2C->DAT;
				I2CHandler.readCount++;
				I2CHandler.readSize--;

				I2CHandler.status = I2C_IDLE;
				LPC_I2C->CONSET = BIT4;	// send STOP

				if (I2CHandler.readSize != 0) {	// if it's not the last byte
					I2CHandler.error = 0x58;
				}
			} else {
				I2C_ERROR();
			}
			break;
		}
		case 0x38: {	// Arbitration lost
			I2CHandler.error = 0x38;
			I2CHandler.status = I2C_IDLE;
			LPC_I2C->CONSET = BIT4;//set stop
			break;
		}
		case 0xF8:
			break;
		default: {
			I2C_ERROR();
		}
	}

	LPC_I2C->CONCLR = BIT3;

}

void lpc_i2c_begin(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 0)
			return;

/*	LPC_IOCON->PIO0_4 &= 0x7;
	LPC_IOCON->PIO0_4 |= 1;		// Set I2C SCL mode
	LPC_IOCON->PIO0_5 &= 0x7;
	LPC_IOCON->PIO0_5 |= 1;		// Set I2C SDA mode
*/

	LPC_SYSCON->PRESETCTRL |= BIT1; 	// de-assert I2C
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT5;	// enable I2C clock

	LPC_I2C->CONCLR = BIT6 | BIT5 | BIT3 | BIT2;	// Clear all
	LPC_I2C->SCLH = 240; // Set I2C speed 48MHz/(240+240) = 100kHz
	LPC_I2C->SCLL = 240;

	NVIC_EnableIRQ(I2C_IRQn);
	LPC_I2C->CONSET = BIT6; //Enable I2C Master mode

}

void lpc_i2c_trans(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 3)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' || OSCMessage_getArgumentType(msg, 1) != 'b' || OSCMessage_getArgumentType(msg, 2) != 'i') return;


	/* Initialize I2C Transfer parameters */
	I2CHandler.error = 0;
	I2CHandler.slaveAddress = OSCMessage_getArgument_int32(msg, 0) & 0x7F;
	I2CHandler.writePtr = OSCMessage_getArgument_blob(msg, 1, &I2CHandler.writeSize);
	I2CHandler.readSize = OSCMessage_getArgument_int32(msg, 2);
	I2CHandler.readCount = 0;

	if (I2CHandler.writeSize == 0 && I2CHandler.readSize == 0) return;

	uint8_t *bundleBuf = NULL;
	if (I2CHandler.readSize != 0) {
		bundleBuf = (uint8_t*)MemoryManager_malloc(I2CHandler.readSize);
		if (bundleBuf == NULL) // XXX: add error message?
			return;
	}
	I2CHandler.readPtr = bundleBuf;

	/* Start I2C Transfer */
	I2CHandler.status = I2C_START;
	LPC_I2C->CONCLR = BIT4; //clear stop
	LPC_I2C->CONSET = BIT5; // Initiate START

	while (I2CHandler.status != I2C_IDLE); // Wait for transfer to complete

	OSCMessage *msg2 = OSCMessage_new();
	OSCMessage_setAddress(msg2, "/lpc/i2c/trans");
	OSCMessage_addArgument_int32(msg2, I2CHandler.slaveAddress);
	if (I2CHandler.error != 0) {
		OSCMessage_addArgument_int32(msg2, I2CHandler.error);
	} else {
		OSCMessage_addArgument_blob(msg2, bundleBuf, I2CHandler.readCount);
	}
	OSCMessage_sendMessage(msg2, &stream);
	OSCMessage_delete(msg2);

	MemoryManager_free(bundleBuf);
}

void lpc_i2c_end(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 0)
		return;

	NVIC_DisableIRQ(I2C_IRQn);

	LPC_I2C->CONCLR = BIT6 | BIT5 | BIT3 | BIT2;	// Clear all
	LPC_SYSCON->SYSAHBCLKCTRL &= ~BIT5;	// disable I2C clock
	LPC_SYSCON->PRESETCTRL &= ~BIT1; 	// assert I2C
}
