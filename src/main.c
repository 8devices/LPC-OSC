/**
 * @file	main.c
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

#include "main.h"

#include "CDC/CDC.h"

#include "Modules/LPC_GPIO.h"
#include "Modules/LPC_ADC.h"
#include "Modules/LPC_UART.h"
#include "Modules/LPC_SPI.h"
#include "Modules/LPC_I2C.h"
#include "Modules/LPC_PWM.h"

volatile OSCTimetag system_time;

uint64_t getTime() {
	return system_time.raw;
}

void SysTick_Handler() {
	system_time.raw += 4294967; // approx 2^32/10^3

	/* Decrement UART timer and flush RX buffer on timeout */
	if (timer_serial != TIMER_STOP) {
		timer_serial--;
		if (timer_serial == 0) {
			UART_Flush();
		}
	}

	if (timer_interrupt0 != TIMER_STOP) {
		timer_interrupt0--;
		if (timer_interrupt0 == 0) {
			GPIO_EnableInt0();
		}
	}
}

inline uint8_t isValidRegisterAddress(uint32_t addr) {

	if (addr >= 0x40000000 && addr < 0x40020000) return 1;	// I2C, WWDT, USART, timers, ADC

	if (addr >= 0x40038000 && addr < 0x40050000) return 1;	// PMU, flash/EEPROM, SSP0, IOCON, system control, GPIO interrupts

	if (addr >= 0x40058000 && addr < 0x40064000) return 1;	// SSP1, GPIO Group interrupts

	if (addr >= 0x40080000 && addr < 0x40084000) return 1;	// USB

	if (addr >= 0x50000000 && addr < 0x50004000) return 1;	// GPIO

	return 0;
}

void lpc_system_registerWrite(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 2) return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i' || OSCMessage_getArgumentType(msg, 1) != 'i') return;

	uint32_t p_addr = OSCMessage_getArgument_int32(msg, 0);
	uint32_t p_value = OSCMessage_getArgument_int32(msg, 1);

	if (!isValidRegisterAddress(p_addr)) return;

	*((volatile uint32_t *)p_addr) = p_value;
}

void lpc_system_registerRead(OSCMessage *msg) {
	if (OSCMessage_getArgumentCount(msg) != 1) return;

	if (OSCMessage_getArgumentType(msg, 0) != 'i') return;

	uint32_t p_addr = OSCMessage_getArgument_int32(msg, 0);

	if (!isValidRegisterAddress(p_addr)) return;

	uint32_t value = *((volatile uint32_t *)p_addr);

	OSCMessage *msg2 = OSCMessage_new();
	if (msg != NULL ) {
		OSCMessage_setAddress(msg2, "/lpc/system/registerRead");
		OSCMessage_addArgument_int32(msg2, p_addr);
		OSCMessage_addArgument_int32(msg2, value);
		OSCMessage_sendMessage(msg2, &stream);
		OSCMessage_delete(msg2);
	}
}

void LedCallback(OSCMessage *msg) {
	LPC_GPIO->NOT[0] |= BIT7;
}

int main(void) {
	SystemCoreClockUpdate();

	timer_serial = TIMER_STOP;
	timer_interrupt0 = TIMER_STOP;

	system_time.raw = OSCTimetag_immediately;
	SysTick_Config(SystemCoreClock/1000);	// Configure Systick to run at 1kHz (1ms)

	//while (CDCPacketStream_init(&stream) != 0);
	while (CDC_Init(&stream) != LPC_OK); // Load OSCPacketStream

	LPC_SYSCON->SYSAHBCLKCTRL |= BIT6 | BIT16 | BIT19; // Enable clock for GPIO, IOConfig and Pin Interrupts

#ifndef DEBUG
	// Disabled for debugging (JTAG)
	uint8_t pin;
	for (pin=0; pin<LPC_GPIO_PIN_COUNT; pin++)
		*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_PRIMARY_FUNCTION[pin];
#endif

	// PIO0_4 and PIO0_5 forced to I2C
	LPC_IOCON->PIO0_4 |= 1;	// I2C SCL
	LPC_IOCON->PIO0_5 |= 1;	// I2C SDA

	/* Temporary and test configs */
	// XXX: LED config - leave it for now
	LPC_GPIO->DIR[0] |= BIT7;
	LPC_GPIO->CLR[0] |= BIT7;


	/* OSC initialization, configuration and launch */
	OSCServer *server = OSCServer_new(getTime);

	/* Test functions (temporary) */
	OSCServer_addMessageHandler(server, "/led", LedCallback);

	/* GPIO/Pin functions */
	OSCServer_addMessageHandler(server, "/lpc/setPrimary", lpc_config_setPrimary);
	OSCServer_addMessageHandler(server, "/lpc/setSecondary", lpc_config_setSecondary);

	OSCServer_addMessageHandler(server, "/lpc/pinMode", lpc_pinMode);
	OSCServer_addMessageHandler(server, "/lpc/digitalWrite", lpc_digitalWrite);
	OSCServer_addMessageHandler(server, "/lpc/digitalRead", lpc_digitalRead);

	OSCServer_addMessageHandler(server, "/lpc/attachInterrupt", lpc_attachInterrupt);
	OSCServer_addMessageHandler(server, "/lpc/detachInterrupt", lpc_detachInterrupt);

	/* ADC functions */
	OSCServer_addMessageHandler(server, "/lpc/analogRead", lpc_analogRead);

	/* USART (Serial) functions */
	OSCServer_addMessageHandler(server, "/lpc/serial/begin", lpc_serial_begin);
	OSCServer_addMessageHandler(server, "/lpc/serial/write", lpc_serial_write);
	OSCServer_addMessageHandler(server, "/lpc/serial/end", lpc_serial_end);

	/* SPI functions */
	OSCServer_addMessageHandler(server, "/lpc/spi0/begin", lpc_spi0_begin);
	OSCServer_addMessageHandler(server, "/lpc/spi0/trans", lpc_spi0_trans);
	OSCServer_addMessageHandler(server, "/lpc/spi0/end", lpc_spi0_end);

	/* I2C functions */
	OSCServer_addMessageHandler(server, "/lpc/i2c/begin", lpc_i2c_begin);
	OSCServer_addMessageHandler(server, "/lpc/i2c/trans", lpc_i2c_trans);
	OSCServer_addMessageHandler(server, "/lpc/i2c/end", lpc_i2c_end);

	/* PWM functions */
	OSCServer_addMessageHandler(server, "/lpc/pwm0/begin", lpc_pwm0_begin);
	OSCServer_addMessageHandler(server, "/lpc/pwm0/set", lpc_pwm0_set);
	OSCServer_addMessageHandler(server, "/lpc/pwm0/end", lpc_pwm0_end);

	OSCServer_addMessageHandler(server, "/lpc/pwm1/begin", lpc_pwm1_begin);
	OSCServer_addMessageHandler(server, "/lpc/pwm1/set", lpc_pwm1_set);
	OSCServer_addMessageHandler(server, "/lpc/pwm1/end", lpc_pwm1_end);


	/* Advanced functions */
	OSCServer_addMessageHandler(server, "/lpc/system/registerWrite", lpc_system_registerWrite);
	OSCServer_addMessageHandler(server, "/lpc/system/registerRead", lpc_system_registerRead);


	OSCServer_loop(server, &stream);

	OSCServer_delete(server);


	while (1);
}
