/**
 * @file	CDC.c
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

#include "CDC/CDC.h"

#include "string.h"

static VCOM_DATA_T *g_vCOM;

/**********************************************************************
 ** Function prototyping
 **********************************************************************/
void USB_pin_clk_init(void) {
	/* Enable AHB clock to the GPIO domain. */
	LPC_SYSCON ->SYSAHBCLKCTRL |= (1 << 6);

	/* Enable AHB clock to the USB block and USB RAM. */
	LPC_SYSCON ->SYSAHBCLKCTRL |= ((0x1 << 14) | (0x1 << 27));

	/* Pull-down is needed, or internally, VBUS will be floating. This is to
	 address the wrong status in VBUSDebouncing bit in CmdStatus register. It
	 happens on the NXP Validation Board only that a wrong ESD protection chip is used. */
	LPC_IOCON ->PIO0_3 &= ~0x1F;
//  LPC_IOCON->PIO0_3   |= ((0x1<<3)|(0x01<<0));	/* Secondary function VBUS */
	LPC_IOCON ->PIO0_3 |= (0x01 << 0); /* Secondary function VBUS */
	LPC_IOCON ->PIO0_6 &= ~0x07;
	LPC_IOCON ->PIO0_6 |= (0x01 << 0); /* Secondary function SoftConn */

	return;
}

void VCOM_usb_send(VCOM_DATA_T* pVcom) {
	/* data received send it back */
	pVcom->txlen -= pUsbApi->hw->WriteEP(pVcom->hUsb, USB_CDC_EP_BULK_IN,
			pVcom->txBuf, pVcom->txlen);
}

void VCOM_usb_send2(VCOM_DATA_T* pVcom, uint8_t *buf, uint32_t size) {
	while (size > 0) {
		uint32_t sent = pUsbApi->hw->WriteEP(pVcom->hUsb, USB_CDC_EP_BULK_IN, buf, size);
		buf += sent;
		size -= sent;
	}
}

ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	return LPC_OK;
}

ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	VCOM_DATA_T* pVcom = (VCOM_DATA_T*) data;

	switch (event) {
	case USB_EVT_OUT:

		if (pVcom->rxlen == 0) {
			pVcom->rxlen = pUsbApi->hw->ReadEP(hUsb, USB_CDC_EP_BULK_OUT,
					pVcom->rxBuf);
			pVcom->send_fn(pVcom);
		} else {
			/* indicate bridge write buffer pending in USB buf */
			pVcom->usbrx_pend = 1;
		}
		break;
	default:
		break;
	}
	return LPC_OK;
}

void USB_IRQHandler(void) {
	pUsbApi->hw->ISR(g_vCOM->hUsb);
}

ErrorCode_t InitCDC(VCOM_DATA_T *vcom) {
	USBD_API_INIT_PARAM_T usb_param;
	USBD_CDC_INIT_PARAM_T cdc_param;
	USB_CORE_DESCS_T desc;
	USBD_HANDLE_T hUsb, hCdc;
	ErrorCode_t ret = LPC_OK;
	uint32_t ep_indx;

	g_vCOM = vcom;

	/* get USB API table pointer */
	pUsbApi = (USBD_API_T*) ((*(ROM **) (0x1FFF1FF8))->pUSBD);

	/* enable clocks and pinmux for usb0 */
	USB_pin_clk_init();

	/* initilize call back structures */
	memset((void*) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB_BASE;
	usb_param.mem_base = 0x10001000;
	usb_param.mem_size = 0x1000;
	usb_param.max_num_ep = 3;

	/* init CDC params */
	memset((void*) &cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));
	memset((void*) vcom, 0, sizeof(VCOM_DATA_T));

	/* Initialize Descriptor pointers */
	memset((void*) &desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &VCOM_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &VCOM_StringDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];
	desc.high_speed_desc = (uint8_t *) &VCOM_ConfigDescriptor[0];

	/* USB Initialization */
	ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);

	if (ret != LPC_OK)
		return ret;

	// init CDC params
	cdc_param.mem_base = 0x10001500;
	cdc_param.mem_size = 0x300;
	cdc_param.cif_intf_desc =
			(uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE];
	cdc_param.dif_intf_desc =
			(uint8_t *) &VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE+ USB_INTERFACE_DESC_SIZE + 0x0013
			+ USB_ENDPOINT_DESC_SIZE];

	ret = pUsbApi->cdc->init(hUsb, &cdc_param, &hCdc);

	if (ret != LPC_OK)
		return ret;

	/* store USB handle */
	vcom->hUsb = hUsb;
	vcom->hCdc = hCdc;
	vcom->send_fn = VCOM_usb_send;

	/* allocate transfer buffers */
	vcom->rxBuf =
			(uint8_t*) (cdc_param.mem_base + (0 * USB_HS_MAX_BULK_PACKET));
	vcom->txBuf =
			(uint8_t*) (cdc_param.mem_base + (1 * USB_HS_MAX_BULK_PACKET));
	cdc_param.mem_size -= (4 * USB_HS_MAX_BULK_PACKET);

	/* register endpoint interrupt handler */
	ep_indx = (((USB_CDC_EP_BULK_IN & 0x0F) << 1) + 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_in_hdlr,
			vcom);

	if (ret != LPC_OK)
		return ret;

	/* register endpoint interrupt handler */
	ep_indx = ((USB_CDC_EP_BULK_OUT & 0x0F) << 1);
	ret = pUsbApi->core->RegisterEpHandler(hUsb, ep_indx, VCOM_bulk_out_hdlr,
			vcom);

	if (ret != LPC_OK)
		return ret;

	/* enable IRQ */
	NVIC_EnableIRQ(USB_IRQn); //  enable USB0 interrrupts

	/* USB Connect */
	pUsbApi->hw->Connect(hUsb, 1);

	return LPC_OK;
}
