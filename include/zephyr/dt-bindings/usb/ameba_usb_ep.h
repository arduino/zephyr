/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_USB_AMEBA_USB_EP_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_USB_AMEBA_USB_EP_H_

#define AMEBA_USB_EP_TYPE_MASK 0x0F

#define AMEBA_USB_EP_TYPE_CTRL 0x01
#define AMEBA_USB_EP_TYPE_ISOC 0x02
#define AMEBA_USB_EP_TYPE_BULK 0x04
#define AMEBA_USB_EP_TYPE_INTR 0x08

#define AMEBA_USB_EP_TYPE_NON_CTRL                                                                 \
	(AMEBA_USB_EP_TYPE_ISOC | AMEBA_USB_EP_TYPE_BULK | AMEBA_USB_EP_TYPE_INTR)
#define AMEBA_USB_EP_TYPE_ALL (AMEBA_USB_EP_TYPE_MASK)

#define AMEBA_USB_EP_DIR_OUT 0
#define AMEBA_USB_EP_DIR_IN  1

#define AMEBA_USB_BUS_SPEED_UNKNOWN 0
#define AMEBA_USB_BUS_SPEED_FS      1
#define AMEBA_USB_BUS_SPEED_HS      2
#define AMEBA_USB_BUS_SPEED_SS      3

/*
	revert     31:20bit
	epnum      19:16bit
	direction  15:15bit
	type       14:11bit
	mps        10:0bit
*/
#define AMEBA_USB_EP(epnum, direction, type, mps)                                                  \
	(((epnum) << 16) | ((direction) << 15) | ((type) << 11) | ((mps) << 0))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_USB_AMEBA_USB_EP_H_ */
