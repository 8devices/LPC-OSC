/**
 * @file	CDCPacketStream.c
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

#include "CDC/CDCPacketStream.h"

#include "CDC/CDC.h"

static VCOM_DATA_T vCom;

uint32_t getPacketSize() {
	uint32_t i;
	for (i=vCom.ser_pos; i<vCom.rxlen; i++)
		if (vCom.rxBuf[i] == '\n')
			return i-vCom.ser_pos;

	return 0;
}

void readPacket(uint8_t *buf) {
	uint32_t i = 0;
	uint8_t data;
	while ((data=vCom.rxBuf[vCom.ser_pos++]) != '\n')
		buf[i++] = data;

	if (vCom.ser_pos == vCom.rxlen) {
		vCom.ser_pos = 0;
		vCom.rxlen = 0;
		if (vCom.usbrx_pend) {
			vCom.usbrx_pend = 0;
			VCOM_bulk_out_hdlr(vCom.hUsb, (void*) &vCom, USB_EVT_OUT);
		}
	}
}

void writePacket(uint8_t *buf, uint32_t size) {
	VCOM_usb_send2(&vCom, buf, size);
}

uint8_t	CDCPacketStream_init(OSCPacketStream *stream) {
	ErrorCode_t ret = InitCDC(&vCom);

	if (ret != LPC_OK) return 1;

	stream->getPacketSize = getPacketSize;
	stream->readPacket = readPacket;
	stream->writePacket = writePacket;

	return 0;
}
