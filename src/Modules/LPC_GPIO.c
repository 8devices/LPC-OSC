/**
 * @file	LPC_GPIO.c
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

#include "Modules/LPC_GPIO.h"


#define LPC_PIN_COUNT	34

uint8_t const LPC_PIN_IDS[] = {
		0+20,	0+2,	24+26,	24+27,	24+20,	0+21,	24+23,	24+24,	// 8
		0+7,	24+28,	24+31,	24+21,	0+8,	0+9,	0+10,	24+29,	// 16
		24+19,	24+25,	24+16,	0+19,	0+18,	0+17,	24+15,	0+23,	// 24
		0+22,	0+16,	0+15,	24+22,	24+14,	24+13,	0+14,	0+13,	// 32
		0+12,	0+11,	// 34
};

volatile uint32_t * const LPC_PIN_REGISTERS[] = {
		&LPC_IOCON->PIO0_20, 	&LPC_IOCON->PIO0_2 , 	&LPC_IOCON->PIO1_26, 	&LPC_IOCON->PIO1_27,
		&LPC_IOCON->PIO1_20, 	&LPC_IOCON->PIO0_21, 	&LPC_IOCON->PIO1_23, 	&LPC_IOCON->PIO1_24,		// 8
		&LPC_IOCON->PIO0_7 , 	&LPC_IOCON->PIO1_28, 	&LPC_IOCON->PIO1_31, 	&LPC_IOCON->PIO1_21,
		&LPC_IOCON->PIO0_8 ,	&LPC_IOCON->PIO0_9 , 	&LPC_IOCON->SWCLK_PIO0_10, &LPC_IOCON->PIO1_29,		// 16
		&LPC_IOCON->PIO1_19, 	&LPC_IOCON->PIO1_25,	&LPC_IOCON->PIO1_16, 	&LPC_IOCON->PIO0_19,
		&LPC_IOCON->PIO0_18,	&LPC_IOCON->PIO0_17,	&LPC_IOCON->PIO1_15, 	&LPC_IOCON->PIO0_23, 		// 24
		&LPC_IOCON->PIO0_22,	&LPC_IOCON->PIO0_16,	&LPC_IOCON->SWDIO_PIO0_15,&LPC_IOCON->PIO1_22,
		&LPC_IOCON->PIO1_14, 	&LPC_IOCON->PIO1_13, 	&LPC_IOCON->TRST_PIO0_14,&LPC_IOCON->TDO_PIO0_13,	// 32
		&LPC_IOCON->TMS_PIO0_12,&LPC_IOCON->TDI_PIO0_11,	// 34
};

#define LPC_PIN_FUNCTION_MASK	(BIT7 | 7)		// FUNC bits + AD bit
#define LPC_PIN_MODE_MASK		(3 << 3)

uint8_t const LPC_PIN_PRIMARY_FUNCTION[] = {	// GPIO function, All AD bits = 1
		0x80,	0x80,	0x80,	0x80,		0x80,	0x80,	0x80,	0x80,	// 8
		0x80,	0x80,	0x80,	0x80,		0x80,	0x80,	0x81,	0x80,	// 16
		0x80,	0x80,	0x80,	0x80,		0x80,	0x80,	0x80,	0x80,	// 24
		0x80,	0x80,	0x81,	0x80,		0x80,	0x80,	0x81,	0x81,	// 32
		0x81,	0x81,	// 34
};

uint8_t const LPC_PIN_SECONDARY_FUNCTION[] = {
		0x80 /* GPIO */,	0x80 /* GPIO */,		0x81 /* CT32B0 MAT2 */,	0x80 /* GPIO */,
		0x82 /* SPI1 SCK */,0x82 /* SPI1 MOSI */,	0x80 /* GPIO */,		0x81 /* CT32B0 MAT0 */,	// 8
		0x80 /* GPIO */,	0x80 /* GPIO */,		0x80 /* GPIO */,		0x82 /* SPI1 MISO */,
		0x81 /* SPI0 MISO*/,0x81 /* SPI0 MOSI */,	0x82 /* SPI0 SCK */,	0x80 /* GPIO */,	// 16
		0x80 /* GPIO */,	0x81 /* CT32B0 MAT1 */,	0x80 /* GPIO */,		0x81 /* UART TX */,
		0x81 /* UART RX */,	0x80 /* GPIO */,		0x82 /* PWM16_2 */,		0x01 /* ADC7 */,	// 24
		0x01 /* ADC6 */,	0x01 /* ADC5 */,		0x02 /* ADC4 */,		0x80 /* GPIO */,
		0x82 /* PWM16_1 */,	0x82 /* PWM16_0 */,		0x02 /* ADC3 */,		0x02 /* ADC2 */,	// 32
		0x02 /* ADC1 */,	0x02 /* ADC0 */,	// 34
};

void lpc_config_setPrimary(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1) return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' ) return;

	uint8_t pin = OSCMessage_getArgument_int32(msg, 0);

	if (pin >= LPC_PIN_COUNT) return;

	*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_PRIMARY_FUNCTION[pin];
}

void lpc_config_setSecondary(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1) return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i') return;

	uint8_t pin = OSCMessage_getArgument_int32(msg, 0);

	if (pin >= LPC_PIN_COUNT) return;

	*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_SECONDARY_FUNCTION[pin];
}


void lpc_pinMode(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 2) return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' || OSCMessage_getArgumentType(msg, 1) != 'i') return;

	uint8_t pin = OSCMessage_getArgument_int32(msg, 0);
	uint8_t mode = OSCMessage_getArgument_int32(msg, 1);

	if (pin >= LPC_PIN_COUNT) return;
	if (mode > 4 || mode == 3) return;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	*LPC_PIN_REGISTERS[pin] &= ~LPC_PIN_MODE_MASK;	// Remove pull-up/down resistors

	if (mode == 1) {
		LPC_GPIO->DIR[port] |= (1 << pinNum);	// Set direction bit (output)
	} else {
		*LPC_PIN_REGISTERS[pin] |= (mode << 2) & LPC_PIN_MODE_MASK;		// Setup resistors
		LPC_GPIO->DIR[port] &= ~(1 << pinNum);	// Clear direction bit (input)
	}
}

void lpc_digitalWrite(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 2)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' || OSCMessage_getArgumentType(msg, 1) != 'i')
		return;

	uint8_t pin = OSCMessage_getArgument_int32(msg, 0);
	uint8_t value = OSCMessage_getArgument_int32(msg, 1);

	if (pin >= LPC_PIN_COUNT) return;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	if (value == 0) {
		LPC_GPIO->CLR[port] = (1 << pinNum);
	} else {
		LPC_GPIO->SET[port] = (1 << pinNum);
	}
}

void lpc_digitalRead(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i')
		return;

	uint8_t pin = OSCMessage_getArgument_int32(msg, 0);

	if (pin >= LPC_PIN_COUNT)
		return;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	uint8_t val = 0;

	if (LPC_GPIO->PIN[port] & (1 << pinNum))
		val = 1;

	OSCMessage *msg2 = OSCMessage_new();
	OSCMessage_setAddress(msg2, "/lpc/digitalRead");
	OSCMessage_addArgument_int32(msg2, pin);
	OSCMessage_addArgument_int32(msg2, val);
	OSCMessage_sendMessage(msg2, &stream);
	OSCMessage_delete(msg2);
}

void lpc_attachInterrupt(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 3)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' || OSCMessage_getArgumentType(msg, 1) != 'i' || OSCMessage_getArgumentType(msg, 2) != 'i') return;

	uint8_t p_intID = OSCMessage_getArgument_int32(msg, 0);	// interrupt ID
	uint8_t p_pin = OSCMessage_getArgument_int32(msg, 1);	// pin ID
	uint8_t p_mode = OSCMessage_getArgument_int32(msg, 2);	// interrupt mode

	if (p_pin >= LPC_PIN_COUNT || p_intID > 7 || p_mode > 4) return;

	NVIC_DisableIRQ(p_intID);	// Disable interrupt. XXX: Luckily FLEX_INTx_IRQn == x, so it can be used this way, otherwise BE AWARE!

	LPC_SYSCON->PINTSEL[p_intID] = LPC_PIN_IDS[p_pin]; 	// select which pin will cause the interrupts

	// XXX: using SI/CI ENF and ENR registers could probably save few instructions
	switch (p_mode) {
		case 0: {	// LOW level mode
			LPC_GPIO_PIN_INT ->ISEL |= (1 << p_intID);	// Set PMODE=level sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable level interrupt.
			LPC_GPIO_PIN_INT ->IENF &= ~(1 << p_intID);	// Set active level LOW.
			break;
		}
		case 1: {	// HIGH level mode
			LPC_GPIO_PIN_INT ->ISEL |= (1 << p_intID);	// Set PMODE=level sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable level interrupt.
			LPC_GPIO_PIN_INT ->IENF |= (1 << p_intID);	// Set active level HIGH.
			break;
		}
		case 2: {	// Edge CHANGE mode
			LPC_GPIO_PIN_INT ->ISEL &= ~(1 << p_intID);	// Set PMODE=edge sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable rising edge.
			LPC_GPIO_PIN_INT ->IENF |= (1 << p_intID);	// Enable falling edge.
			break;
		}
		case 3: {	// RISING edge mode
			LPC_GPIO_PIN_INT ->ISEL &= ~(1 << p_intID);	// Set PMODE=edge sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable rising edge.
			LPC_GPIO_PIN_INT ->IENF &= ~(1 << p_intID);	// Disable falling edge.
			break;
		}
		case 4: {	// FALLING edge mode
			LPC_GPIO_PIN_INT ->ISEL &= ~(1 << p_intID);	// Set PMODE=edge sensitive
			LPC_GPIO_PIN_INT ->IENR &= ~(1 << p_intID);	// Disable rising edge.
			LPC_GPIO_PIN_INT ->IENF |= (1 << p_intID);	// Enable falling edge.
			break;
		}
	}

	LPC_GPIO_PIN_INT->RISE = (1 << p_intID);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1 << p_intID);	// Clear falling edge (sort of) flag
	NVIC_EnableIRQ(p_intID);	// Enable interrupt. XXX: Luckily FLEX_INTx_IRQn == x, so it can be used this way, otherwise BE AWARE!
}

void lpc_detachInterrupt(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i') return;

	uint8_t p_intID = OSCMessage_getArgument_int32(msg, 0);	// interrupt ID

	NVIC_DisableIRQ(p_intID);	// Disable interrupt. XXX: Luckily FLEX_INTx_IRQn == x, so it can be used this way, otherwise BE AWARE!
	LPC_GPIO_PIN_INT->CIENR = (1 << p_intID);	// Disable rising edge or level interrupt
	LPC_GPIO_PIN_INT->CIENF = (1 << p_intID);	// Disable falling edge interrupt
	LPC_GPIO_PIN_INT->RISE = (1 << p_intID);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1 << p_intID);	// Clear falling edge (sort of) flag
}

inline void GPIO_EnableInt0() {
	LPC_GPIO_PIN_INT->RISE = BIT0;	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = BIT0;	// Clear falling edge (sort of) flag
	NVIC_EnableIRQ(FLEX_INT0_IRQn);	// Enable ISR
}

void FLEX_INT0_IRQHandler() {
	timer_interrupt0 = TIMER_STOP;
	NVIC_DisableIRQ(FLEX_INT0_IRQn);		// Disable ISR

	if (LPC_GPIO_PIN_INT->IST & BIT0) {
		uint8_t interruptEvent = 0xFF;

		if ((LPC_GPIO_PIN_INT->ISEL & BIT0)) {	// if LEVEL mode
			if (LPC_GPIO_PIN_INT->IENR & BIT0) {	// if LEVEL interrupts are enabled
				if (LPC_GPIO_PIN_INT->IENF & BIT0) {	// HIGH mode
					interruptEvent = 1;
				} else {								// LOW mode
					interruptEvent = 0;
				}
			}
		} else {	// EDGE mode
			if ((LPC_GPIO_PIN_INT->RISE & BIT0) && (LPC_GPIO_PIN_INT->IENR & BIT0)) {	// Rising edge interrupt
				interruptEvent = 3;
			}
			if ((LPC_GPIO_PIN_INT->FALL & BIT0) && (LPC_GPIO_PIN_INT->IENF & BIT0)) {	// Falling edge interrupt
				if (interruptEvent == 3)
					interruptEvent = 2;				// Edge CHANGE (RISE+FALL)
				else
					interruptEvent = 4;				// Falling edge
			}
		}

		OSCMessage *msg = OSCMessage_new();
		if (msg != NULL) {
			OSCMessage_setAddress(msg, "/lpc/interrupt0");
			OSCMessage_addArgument_int32(msg, interruptEvent);
			OSCMessage_sendMessage(msg, &stream);
			OSCMessage_delete(msg);
		}

		timer_interrupt0 = 50; // 50ms delay
		return;
	}

	GPIO_EnableInt0();
}
