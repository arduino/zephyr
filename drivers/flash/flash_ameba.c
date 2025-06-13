/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT realtek_ameba_flash_controller

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_ameba, CONFIG_FLASH_LOG_LEVEL);

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#define FLASH_SEM_TIMEOUT (k_is_in_isr() ? K_NO_WAIT : K_FOREVER)

struct flash_ameba_dev_config {
	uint32_t base_addr;
};

struct flash_ameba_dev_data {
#ifdef CONFIG_MULTITHREADING
	struct k_sem sem;
#endif
};

static const struct flash_parameters flash_ameba_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

#ifdef CONFIG_MULTITHREADING
static inline void flash_ameba_sem_take(const struct device *dev)
{
	struct flash_ameba_dev_data *data = dev->data;

	k_sem_take(&data->sem, FLASH_SEM_TIMEOUT);
}

static inline void flash_ameba_sem_give(const struct device *dev)
{
	struct flash_ameba_dev_data *data = dev->data;

	k_sem_give(&data->sem);
}
#else

#define flash_ameba_sem_take(dev)                                                                  \
	do {                                                                                       \
	} while (0)
#define flash_ameba_sem_give(dev)                                                                  \
	do {                                                                                       \
	} while (0)

#endif /* CONFIG_MULTITHREADING */

static int flash_ameba_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	int ret = 0;

	flash_ameba_sem_take(dev);

	FLASH_ReadStream(address, length, buffer);

	flash_ameba_sem_give(dev);
	return ret;
}

static int flash_ameba_write(const struct device *dev, off_t address, const void *buffer,
			     size_t length)
{
	int ret = 0;

	flash_ameba_sem_take(dev);

	FLASH_WriteStream(address, length, (u8 *)buffer);

	flash_ameba_sem_give(dev);
	return ret;
}

static int flash_ameba_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_pages_info info;
	uint32_t start_sector_offset, end_sector_offset;
	uint32_t i;
	int ret = 0;

	flash_ameba_sem_take(dev);

	ret = flash_get_page_info_by_offs(dev, offset, &info);
	if (ret) {
		return ret;
	}
	start_sector_offset = info.start_offset;
	ret = flash_get_page_info_by_offs(dev, offset + len - 1, &info);
	if (ret) {
		return ret;
	}
	end_sector_offset = info.start_offset;

	for (i = start_sector_offset; i <= end_sector_offset; i++) {
		FLASH_EraseXIP(EraseSector, start_sector_offset);
	}

	flash_ameba_sem_give(dev);

	return ret;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_ameba_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

void flash_ameba_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	*layout = &flash_ameba_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *flash_ameba_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_ameba_parameters;
}

static int flash_ameba_init(const struct device *dev)
{
	struct flash_ameba_dev_data *const dev_data = dev->data;

#ifdef CONFIG_MULTITHREADING
	k_sem_init(&dev_data->sem, 1, 1);
#endif /* CONFIG_MULTITHREADING */

	return 0;
}

static const struct flash_driver_api flash_ameba_driver_api = {
	.read = flash_ameba_read,
	.write = flash_ameba_write,
	.erase = flash_ameba_erase,
	.get_parameters = flash_ameba_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_ameba_page_layout,
#endif
};

static struct flash_ameba_dev_data flash_ameba_data;

static const struct flash_ameba_dev_config flash_ameba_config = {
	.base_addr = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, flash_ameba_init, NULL, &flash_ameba_data, &flash_ameba_config,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &flash_ameba_driver_api);
