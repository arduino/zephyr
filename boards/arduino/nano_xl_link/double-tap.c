/*
 * Copyright (c) Arduino s.r.l. and/or its affiliated companies
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Double tap of the reset button into MCUboot's USB DFU, built into MCUboot
 * only. The first reset arms a cookie in retained RAM and keeps it armed for
 * DOUBLE_TAP_WINDOW_MS; a reset inside that window finds it and sets the
 * retention boot mode, which CONFIG_BOOT_USB_DFU_BOOT_MODE turns into USB DFU.
 *
 * The cookie sits at offset 0 of the "retained_ram" node, which the Arduino
 * core variant provides together with the zephyr,boot-mode area.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/retention/bootmode.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(retained_ram))

#define DOUBLE_TAP_WINDOW_MS 500
#define DOUBLE_TAP_OFFSET    0

static const uint8_t double_tap_armed[] = {0x44, 0x54, 0x41, 0x50}; /* "DTAP" */
static const uint8_t double_tap_clear[sizeof(double_tap_armed)];

static const struct device *const retained_ram = DEVICE_DT_GET(DT_NODELABEL(retained_ram));
static bool tapped;

/*
 * Everything between the reset and the moment the cookie is armed is a dead
 * zone where a second tap goes unnoticed, and on this SoC clock bring-up alone
 * takes hundreds of milliseconds. So arm before it, through the retained-RAM
 * driver (CONFIG_RETAINED_MEM_INIT_PRIORITY=0).
 */
static int double_tap_arm(void)
{
	uint8_t value[sizeof(double_tap_armed)];

	(void)retained_mem_read(retained_ram, DOUBLE_TAP_OFFSET, value, sizeof(value));
	tapped = (memcmp(value, double_tap_armed, sizeof(value)) == 0);
	(void)retained_mem_write(retained_ram, DOUBLE_TAP_OFFSET,
				 tapped ? double_tap_clear : double_tap_armed, sizeof(value));

	return 0;
}
SYS_INIT(double_tap_arm, PRE_KERNEL_1, 1);
BUILD_ASSERT(CONFIG_RETAINED_MEM_INIT_PRIORITY < 1,
	     "the retained-RAM driver must be up before double_tap_arm()");

/* Before main(), which is where MCUboot reads (and clears) the boot mode. */
static int double_tap_check(void)
{
	if (tapped) {
		(void)bootmode_set(BOOT_MODE_TYPE_BOOTLOADER);
	} else {
		/* Hold the window open so a second tap finds the cookie armed. */
		k_msleep(DOUBLE_TAP_WINDOW_MS);
		(void)retained_mem_write(retained_ram, DOUBLE_TAP_OFFSET, double_tap_clear,
					 sizeof(double_tap_clear));
	}

	return 0;
}
SYS_INIT(double_tap_check, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(retained_ram)) */
