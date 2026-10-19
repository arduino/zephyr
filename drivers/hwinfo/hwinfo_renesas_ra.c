/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	bsp_unique_id_t const *unique_id = R_BSP_UniqueIdGet();
	size_t len = MIN(length, sizeof(unique_id->unique_id_bytes));

#if defined(CONFIG_HWINFO_ARDUINO_LEGACY_ID)
	/* Arduino legacy order: each 32-bit word stored big-endian */
	uint8_t id[sizeof(unique_id->unique_id_bytes)];

	for (size_t i = 0; i < ARRAY_SIZE(unique_id->unique_id_words); i++) {
		sys_put_be32(unique_id->unique_id_words[i], &id[i * sizeof(uint32_t)]);
	}

	memcpy(buffer, id, len);
#else
	memcpy(buffer, &unique_id->unique_id_bytes[0], len);
#endif /* CONFIG_HWINFO_ARDUINO_LEGACY_ID */

	return len;
}
