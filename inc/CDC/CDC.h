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

extern uint8_t VCOM_DeviceDescriptor[];
extern uint8_t VCOM_StringDescriptor[];
extern uint8_t VCOM_ConfigDescriptor[];

USBD_API_T* pUsbApi;

/* UART defines */
#define TX_FIFO_SIZE 16

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

/* VCOM defines */
#define VCOM_BUFFERS    4
#define VCOM_BUF_EMPTY_INDEX  (0xFF)
#define VCOM_BUF_FREE   0
#define VCOM_BUF_ALLOC  1
#define VCOM_BUF_USBTXQ  2
#define VCOM_BUF_UARTTXQ  3
#define VCOM_BUF_ALLOCU  4

struct VCOM_DATA;
typedef void (*VCOM_SEND_T) (struct VCOM_DATA* pVcom);

typedef struct VCOM_DATA {
  USBD_HANDLE_T hUsb;
  USBD_HANDLE_T hCdc;
  uint8_t* rxBuf;
  uint8_t* txBuf;
  volatile uint8_t ser_pos;
  volatile uint16_t rxlen;
  volatile uint16_t txlen;
  VCOM_SEND_T send_fn;
  volatile uint32_t sof_counter;
  volatile uint32_t last_ser_rx;
  volatile uint16_t break_time;
  volatile uint16_t usbrx_pend;
} VCOM_DATA_T;


ErrorCode_t	VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event);
void 		VCOM_usb_send2(VCOM_DATA_T* pVcom, uint8_t *buf, uint32_t size);

ErrorCode_t InitCDC(VCOM_DATA_T *vcom);

#endif /* CDC_H_ */
