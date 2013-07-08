/*
 * CDC.h
 *
 *  Created on: 2013.06.25
 *      Author: Giedrius
 */

#ifndef CDC_H_
#define CDC_H_

#include "LPC11Uxx.h"
#include "mw_usbd_rom_api.h"
#include "power_api.h"

#include <OSC/OSCPacketStream.h>

extern uint8_t VCOM_DeviceDescriptor[];
extern uint8_t VCOM_StringDescriptor[];
extern uint8_t VCOM_ConfigDescriptor[];

ErrorCode_t CDC_Init(OSCPacketStream *stream);

#endif /* CDC_H_ */

