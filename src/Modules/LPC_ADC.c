/**
 * @file	LPC_ADC.c
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


#include "Modules/LPC_ADC.h"

volatile uint32_t* LPC_ADC_PINS[] = {
		&LPC_IOCON ->TDI_PIO0_11,
		&LPC_IOCON ->TMS_PIO0_12, &LPC_IOCON ->TDO_PIO0_13,
		&LPC_IOCON ->TRST_PIO0_14, &LPC_IOCON ->SWDIO_PIO0_15,
		&LPC_IOCON ->PIO0_16, &LPC_IOCON ->PIO0_22, &LPC_IOCON ->PIO0_23
};

volatile const uint32_t LPC_ADC_MODE[] = {
		0x2, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01
};

void lpc_analogRead(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1)
		return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i')
		return;

	uint8_t pin = OSCMessage_getArgument_int32(msg, 0);

	if (pin > 7)
		return;

	*(LPC_ADC_PINS[pin]) = LPC_ADC_MODE[pin];	// alternate function, analog, no-pull

	LPC_SYSCON->PDRUNCFG &= ~BIT4;			// power up ADC
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT13;		// enable ADC clock
	LPC_ADC->CR = (1 << pin) | (15 << 8) | (1 << 24);	//ADn, clock divider = 15+1, and start conversion

	while (!(LPC_ADC->DR[pin] & BIT31));	// wait for conversion to end
	uint16_t val = (LPC_ADC->DR[pin] >> 6) & 0x3FF;		// read value

	LPC_ADC->CR = 0;						// stop ADC
	LPC_SYSCON->SYSAHBCLKCTRL &= ~BIT13;	// stop ADC clock
	LPC_SYSCON->PDRUNCFG |= BIT4;			// power down ADC

	OSCMessage *msg2 = OSCMessage_new();
	OSCMessage_setAddress(msg2, "/lpc/analogRead");
	OSCMessage_addArgument_int32(msg2, pin);
	OSCMessage_addArgument_int32(msg2, val);
	OSCMessage_sendMessage(msg2, &stream);
	OSCMessage_delete(msg2);
}
