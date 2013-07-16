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

#define CDC_CIF1_INT_IN_EP		USB_ENDPOINT_IN(1)
#define CDC_DIF1_BULK_OUT_EP	USB_ENDPOINT_OUT(2)
#define CDC_DIF1_BULK_IN_EP		USB_ENDPOINT_IN(2)

#define CDC_CIF2_INT_IN_EP		USB_ENDPOINT_IN(3)
#define CDC_DIF2_BULK_OUT_EP	USB_ENDPOINT_OUT(4)
#define CDC_DIF2_BULK_IN_EP		USB_ENDPOINT_IN(4)

extern uint8_t VCOM_DeviceDescriptor[];
extern uint8_t VCOM_StringDescriptor[];
extern uint8_t VCOM_ConfigDescriptor[];

ErrorCode_t CDC_Init(OSCPacketStream *stream);

#endif /* CDC_H_ */

