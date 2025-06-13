/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/hwinfo.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint8_t uuid[8];

	EFUSE_GetUUID((u32 *)uuid);

	if (length > sizeof(uuid)) {
		length = sizeof(uuid);
	}

	_memcpy(buffer, uuid, length);

	return length;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = (RESET_POR | RESET_SOFTWARE | RESET_WATCHDOG | RESET_LOW_POWER_WAKE |
		      RESET_BROWNOUT);

	return 0;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t reason = BOOT_Reason();

	switch (reason) {
	case AON_BIT_RSTF_KM0_SYS:
	case AON_BIT_RSTF_KM4_SYS:
		*cause = RESET_SOFTWARE;
		break;
	case AON_BIT_RSTF_IWDG:
	case AON_BIT_RSTF_WDG0:
	case AON_BIT_RSTF_WDG1:
	case AON_BIT_RSTF_WDG2:
		*cause = RESET_WATCHDOG;
		break;
	case AON_BIT_RSTF_DSLP:
		*cause = RESET_LOW_POWER_WAKE;
		break;
	case AON_BIT_RSTF_BOR:
		*cause = RESET_BROWNOUT;
		break;
	default:
		*cause = RESET_POR;
		break;
	}

	return 0;
}
