
/*
 * Copyright (c) 2025 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/retention/retention.h>
#include <zephyr/retention/bootmode.h>
#include <zephyr/sys/reboot.h>

#define DOUBLE_TAP_CMD      0x4B
#define DOUBLE_TAP_DELAY_ms 500

LOG_MODULE_REGISTER(double_tap, LOG_LEVEL_INF);

const struct device *retention0 = DEVICE_DT_GET(DT_NODELABEL(retention0));

static void write_tap_cmd()
{
	uint8_t magic = DOUBLE_TAP_CMD;
	if (retention_write(retention0, 0, &magic, 1)) {
		LOG_ERR("Unable to write retention0\n");
	}
}

static bool check_tap_cmd()
{
	uint8_t magic = 0;
	if (retention_read(retention0, 0, &magic, 1)) {
		LOG_ERR("Unable to read retention0\n");
	} else {
		if (magic == DOUBLE_TAP_CMD) {
			return true;
		}
	}
	return false;
}

/**
 * @brief Double TAP detection function called by zephyr at Init before
 *        application (in this case application is mcuboot)
 */
static int double_tap_detection(void)
{
	int ret;

	ret = retention_is_valid(retention0);

	if (ret == 1) {
		LOG_INF("RETENTION-0 is valid\n");
		if (check_tap_cmd()) {
			LOG_INF("Setting bootmode\n");
			ret = bootmode_set(BOOT_MODE_TYPE_BOOTLOADER);
			retention_clear(retention0);
			if (ret < 0) {
				LOG_ERR("ERROR: Unable to set boot mode (%d)\n", ret);
			}
		}
	} else if (ret == 0) {
		LOG_INF("RETENTION-0 is NOT valid\n");
		write_tap_cmd();
		k_msleep(DOUBLE_TAP_DELAY_ms);
		retention_clear(retention0);
	} else {
		LOG_ERR("ERROR while checking for RETENTION-0 validity");
	}

	return 0;
}

/* Register the function to run at the APPLICATION level */
SYS_INIT(double_tap_detection, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
