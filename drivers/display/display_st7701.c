/*
 * Copyright (c) 2024 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sitronix_st7701

#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(st7701, CONFIG_DISPLAY_LOG_LEVEL);

#include "display_st7701.h"

struct st7701_config {
	const struct device *mipi_dsi;
	const struct gpio_dt_spec reset;
	const struct gpio_dt_spec backlight;
	uint8_t data_lanes;
	uint16_t width;
	uint16_t height;
	uint8_t channel;
	uint16_t rotation;
};

struct st7701_data {
	uint16_t xres;
	uint16_t yres;
	uint8_t dsi_pixel_format;
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;
};

static inline int st7701_dcs_write(const struct device *dev, uint8_t cmd, const void *buf,
				     size_t len)
{
	const struct st7701_config *cfg = dev->config;
	int ret;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, cmd, buf, len);
	if (ret < 0) {
		LOG_ERR("DCS 0x%x write failed! (%d)", cmd, ret);
		return ret;
	}

	return 0;
}

static int st7701_short_write_1p(const struct device *dev, uint8_t cmd, uint8_t val)
{
	const struct st7701_config *cfg = dev->config;
	int ret;
	uint8_t buf[] = {cmd, val};

	ret = mipi_dsi_generic_write(cfg->mipi_dsi, cfg->channel, buf, sizeof(val));
	if (ret < 0) {
		return ret;
	}

	return 0;
}


static int st7701_generic_write(const struct device *dev, const void *buf, size_t len)
{
	const struct st7701_config *cfg = dev->config;
	int ret;

	ret = mipi_dsi_generic_write(cfg->mipi_dsi, cfg->channel, buf, len);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int st7701_check_id(const struct device *dev)
{
	const struct st7701_config *cfg = dev->config;
	uint32_t id = 0;
	int ret;

	ret = mipi_dsi_dcs_read(cfg->mipi_dsi, cfg->channel, ST7701_CMD_ID1, &id, sizeof(id));
	if (ret != sizeof(id)) {
		LOG_ERR("Read panel ID failed! (%d)", ret);
		return -EIO;
	}

	if (id != 0xFF) {
		LOG_ERR("ID 0x%x (should 0x%x)", id, 0xFF);
		return -EINVAL;
	}

	return 0;
}

static int st7701_configure(const struct device *dev)
{
	struct st7701_data *data = dev->data;
	uint8_t buf[4];
	int ret;

	const uint8_t Display_Control_0[] = {0xFF,0x77,0x01,0x00,0x00,0x10};  
	const uint8_t Display_Control_1[] = {0xC0,0x63,0x00};
	const uint8_t Display_Control_2[] = {0xC1,0x11,0x02};
	const uint8_t Display_Control_3[] = {0xC2,0x01,0x08};
	const uint8_t Display_Control_4[] = {0xCC,0x18};

	st7701_generic_write(dev, Display_Control_0, sizeof(Display_Control_0));
	st7701_generic_write(dev, Display_Control_1, sizeof(Display_Control_1));
	st7701_generic_write(dev, Display_Control_2, sizeof(Display_Control_2));
	st7701_generic_write(dev, Display_Control_3, sizeof(Display_Control_3));
	st7701_generic_write(dev, Display_Control_4, sizeof(Display_Control_4));

	//-------------------------------------Gamma Cluster Setting---------------------------------------------//
	const uint8_t _B0[] = {0xB0, 0x40, 0xc9, 0x91, 0x0d,
						0x12, 0x07, 0x02, 0x09, 0x09, 
						0x1f, 0x04, 0x50, 0x0f, 0xe4, 
						0x29, 0xdf};

	const uint8_t _B1[] = {0xB1, 0x40, 0xcb, 0xd0, 0x11,
						0x92, 0x07, 0x00, 0x08, 0x07, 
						0x1c, 0x06, 0x53, 0x12, 0x63, 
						0xeb, 0xdf};

	st7701_generic_write(dev, _B0, sizeof(_B0));
	st7701_generic_write(dev, _B1, sizeof(_B1));

	//---------------------------------------End Gamma Setting-----------------------------------------------//
	//------------------------------------End Display Control setting----------------------------------------//
	//-----------------------------------------Bank0 Setting End---------------------------------------------//

	//-------------------------------------------Bank1 Setting-----------------------------------------------//
	//-------------------------------- Power Control Registers Initial --------------------------------------//
	const uint8_t _FF1[] = {DSI_CMD2BKX_SEL,0x77,0x01,0x00,0x00,DSI_CMD2BK1_SEL};
	st7701_generic_write(dev, _FF1, sizeof(_FF1));

	st7701_short_write_1p(dev, DSI_CMD2_BK1_VRHS, 0x65);
	//-------------------------------------------Vcom Setting------------------------------------------------//
	st7701_short_write_1p(dev, DSI_CMD2_BK1_VCOM, 0x34);
	//-----------------------------------------End Vcom Setting----------------------------------------------//
	st7701_short_write_1p(dev, DSI_CMD2_BK1_VGHSS, 0x87);
	st7701_short_write_1p(dev, DSI_CMD2_BK1_TESTCMD, 0x80);

	st7701_short_write_1p(dev, DSI_CMD2_BK1_VGLS, 0x49);
	st7701_short_write_1p(dev, DSI_CMD2_BK1_PWCTLR1, 0x85);

	st7701_short_write_1p(dev, DSI_CMD2_BK1_PWCTLR2, 0x20);
	st7701_short_write_1p(dev, 0xB9, 0x10);
	st7701_short_write_1p(dev, DSI_CMD2_BK1_SPD1, 0x78);
	st7701_short_write_1p(dev, DSI_CMD2_BK1_SPD2, 0x78);
	st7701_short_write_1p(dev, DSI_CMD2_BK1_MIPISET1, 0x88);
	//---------------------------------End Power Control Registers Initial ----------------------------------//
	k_msleep(100);

	//---------------------------------------------GIP Setting-----------------------------------------------//
	const uint8_t _E0[] = {0xE0,0x00,0x00,0x02};
	st7701_generic_write(dev, _E0, sizeof(_E0));
	//----------------------------------GIP------------------------------------------------------------------//
	const uint8_t _E1[] = {0xE1,0x08,0x00,0x0A,0x00,0x07,0x00,0x09,0x00,0x00,0x33,0x33};
	const uint8_t _E2[] = {0xE2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	st7701_generic_write(dev, _E1, sizeof(_E1));
	st7701_generic_write(dev, _E2, sizeof(_E2));

	//-------------------------------------------------------------------------------------------------------//
	const uint8_t _E3[] = {0xE3,0x00,0x00,0x33,0x33};
	const uint8_t _E4[] = {0xE4,0x44,0x44};
	st7701_generic_write(dev, _E3, sizeof(_E3));
	st7701_generic_write(dev, _E4, sizeof(_E4));

	const uint8_t _E5[] = {0xE5,0x0E,0x60,0xA0,0xa0,0x10,0x60,0xA0,0xA0,0x0A,0x60,0xA0,0xA0,0x0C,0x60,0xA0,0xA0};
	st7701_generic_write(dev, _E5, sizeof(_E5));

	const uint8_t _E6[] = {0xE6,0x00,0x00,0x33,0x33};
	const uint8_t _E7[] = {0xE7,0x44,0x44};
	st7701_generic_write(dev, _E6, sizeof(_E6));
	st7701_generic_write(dev, _E7, sizeof(_E7));

	const uint8_t _E8[] = {0xE8,0x0D,0x60,0xA0,0xA0,0x0F,0x60,0xA0,0xA0,0x09,0x60,0xA0,0xA0,0x0B,0x60,0xA0,0xA0};
	st7701_generic_write(dev, _E8, sizeof(_E8));

	const uint8_t _EB[] = {0xEB,0x02,0x01,0xE4,0xE4,0x44,0x00,0x40};
	const uint8_t _EC[] = {0xEC,0x02,0x01};
	st7701_generic_write(dev, _EB, sizeof(_EB));
	st7701_generic_write(dev, _EC, sizeof(_EC));

	const uint8_t _ED[17] = {0xED,0xAB,0x89,0x76,0x54,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x10,0x45,0x67,0x98,0xBA};
	st7701_generic_write(dev, _ED, sizeof(_ED));

	//--------------------------------------------End GIP Setting-----------------------------------------------//
	//------------------------------ Power Control Registers Initial End----------------------------------------//
	//------------------------------------------Bank1 Setting---------------------------------------------------//
	const uint8_t _FF2[] = {DSI_CMD2BKX_SEL,0x77,0x01,0x00,0x00,DSI_CMD2BKX_SEL_NONE};
	st7701_generic_write(dev, _FF2, sizeof(_FF2));

	/* exit sleep mode */
	ret = st7701_dcs_write(dev, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	k_msleep(50);

	/* set pixel color format */
	switch (data->dsi_pixel_format) {
	case MIPI_DSI_PIXFMT_RGB565:
		buf[0] = MIPI_DCS_PIXEL_FORMAT_16BIT;
		break;
	case MIPI_DSI_PIXFMT_RGB888:
		buf[0] = MIPI_DCS_PIXEL_FORMAT_24BIT;
		break;
	default:
		LOG_ERR("Unsupported pixel format 0x%x!", data->dsi_pixel_format);
		return -ENOTSUP;
	}

	ret = st7701_dcs_write(dev, MIPI_DCS_SET_PIXEL_FORMAT, buf, 1);
	if (ret < 0) {
		return ret;
	}

#if 0
	/* configure address mode */
	if (data->orientation == DISPLAY_ORIENTATION_NORMAL) {
		buf[0] = 0x00;
	} else if (data->orientation == DISPLAY_ORIENTATION_ROTATED_90) {
		buf[0] = MIPI_DCS_ADDRESS_MODE_MIRROR_X | MIPI_DCS_ADDRESS_MODE_SWAP_XY;
	} else if (data->orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		buf[0] = MIPI_DCS_ADDRESS_MODE_MIRROR_X | MIPI_DCS_ADDRESS_MODE_MIRROR_Y;
	} else if (data->orientation == DISPLAY_ORIENTATION_ROTATED_270) {
		buf[0] = MIPI_DCS_ADDRESS_MODE_MIRROR_Y | MIPI_DCS_ADDRESS_MODE_SWAP_XY;
	}

	ret = st7701_dcs_write(dev, MIPI_DCS_SET_ADDRESS_MODE, buf, 1);
	if (ret < 0) {
		return ret;
	}
#endif

	buf[0] = 0x00;
	buf[1] = 0x00;
	sys_put_be16(data->xres, (uint8_t *)&buf[2]);
	ret = st7701_dcs_write(dev, MIPI_DCS_SET_COLUMN_ADDRESS, buf, 4);
	if (ret < 0) {
		return ret;
	}

	buf[0] = 0x00;
	buf[1] = 0x00;
	sys_put_be16(data->yres, (uint8_t *)&buf[2]);
	ret = st7701_dcs_write(dev, MIPI_DCS_SET_PAGE_ADDRESS, buf, 4);
	if (ret < 0) {
		return ret;
	}

	/* backlight control */
	buf[0] = ST7701_WRCTRLD_BCTRL | ST7701_WRCTRLD_DD | ST7701_WRCTRLD_BL;
	ret = st7701_dcs_write(dev, MIPI_DCS_WRITE_CONTROL_DISPLAY, buf, 1);
	if (ret < 0) {
		return ret;
	}

	/* adaptive brightness control */
	buf[0] = ST7701_WRCABC_UI;
	ret = st7701_dcs_write(dev, MIPI_DCS_WRITE_POWER_SAVE, buf, 1);
	if (ret < 0) {
		return ret;
	}

	/* adaptive brightness control minimum brightness */
	buf[0] = 0xFF;
	ret = st7701_dcs_write(dev, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS, buf, 1);
	if (ret < 0) {
		return ret;
	}

	/* brightness */
	buf[0] = 0xFF;
	ret = st7701_dcs_write(dev, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, buf, 1);
	if (ret < 0) {
		return ret;
	}

	/* Display On */
	ret = st7701_dcs_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int st7701_blanking_on(const struct device *dev)
{
	const struct st7701_config *cfg = dev->config;
	int ret;

	if (cfg->backlight.port != NULL) {
		ret = gpio_pin_set_dt(&cfg->backlight, 0);
		if (ret) {
			LOG_ERR("Disable backlight failed! (%d)", ret);
			return ret;
		}
	}

	return st7701_dcs_write(dev, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
}

static int st7701_blanking_off(const struct device *dev)
{
	const struct st7701_config *cfg = dev->config;
	int ret;

	if (cfg->backlight.port != NULL) {
		ret = gpio_pin_set_dt(&cfg->backlight, 1);
		if (ret) {
			LOG_ERR("Enable backlight failed! (%d)", ret);
			return ret;
		}
	}

	return st7701_dcs_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0);
}

static int st7701_write(const struct device *dev, uint16_t x, uint16_t y,
			  const struct display_buffer_descriptor *desc, const void *buf)
{
	return -ENOTSUP;
}

static int st7701_set_brightness(const struct device *dev, uint8_t brightness)
{
	return st7701_dcs_write(dev, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, &brightness, 1);
}

static void st7701_get_capabilities(const struct device *dev,
				      struct display_capabilities *capabilities)
{
	const struct st7701_config *cfg = dev->config;
	struct st7701_data *data = dev->data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = cfg->width;
	capabilities->y_resolution = cfg->height;
	capabilities->supported_pixel_formats = data->pixel_format;
	capabilities->current_pixel_format = data->pixel_format;
	capabilities->current_orientation = data->orientation;
}

static const struct display_driver_api st7701_api = {
	.blanking_on = st7701_blanking_on,
	.blanking_off = st7701_blanking_off,
	.write = st7701_write,
	.set_brightness = st7701_set_brightness,
	.get_capabilities = st7701_get_capabilities,
};

static int st7701_init(const struct device *dev)
{
	const struct st7701_config *cfg = dev->config;
	struct st7701_data *data = dev->data;
	struct mipi_dsi_device mdev;
	int ret;

	if (cfg->reset.port) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("Reset GPIO device is not ready!");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Reset display failed! (%d)", ret);
			return ret;
		}
		k_msleep(10);
		ret = gpio_pin_set_dt(&cfg->reset, 1);
		if (ret < 0) {
			LOG_ERR("Enable display failed! (%d)", ret);
			return ret;
		}
		k_msleep(100);
	}

	/* store x/y resolution & rotation */
	if (cfg->rotation == 0) {
		data->xres = cfg->width;
		data->yres = cfg->height;
		data->orientation = DISPLAY_ORIENTATION_NORMAL;
	} else if (cfg->rotation == 90) {
		data->xres = cfg->height;
		data->yres = cfg->width;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_90;
	} else if (cfg->rotation == 180) {
		data->xres = cfg->width;
		data->yres = cfg->height;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_180;
	} else if (cfg->rotation == 270) {
		data->xres = cfg->height;
		data->yres = cfg->width;
		data->orientation = DISPLAY_ORIENTATION_ROTATED_270;
	}

	/* attach to MIPI-DSI host */
	mdev.data_lanes = cfg->data_lanes;
	mdev.pixfmt = data->dsi_pixel_format;
	mdev.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_LPM;

	mdev.timings.hactive = cfg->width;
	mdev.timings.hbp = 20;
	mdev.timings.hsync = 20;
	mdev.timings.hfp = 20;
	mdev.timings.vactive = cfg->height;
	mdev.timings.vbp = 60;
	mdev.timings.vsync = 2;
	mdev.timings.vfp = 10;

	ret = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &mdev);
	if (ret < 0) {
		LOG_ERR("MIPI-DSI attach failed! (%d)", ret);
		return ret;
	}

	ret = st7701_check_id(dev);
	if (ret) {
		LOG_ERR("Panel ID check failed! (%d)", ret);
		return ret;
	}

	ret = st7701_configure(dev);
	if (ret) {
		LOG_ERR("DSI init sequence failed! (%d)", ret);
		return ret;
	}

	ret = st7701_blanking_off(dev);
	if (ret) {
		LOG_ERR("Display blanking off failed! (%d)", ret);
		return ret;
	}

	return 0;
}

#define ST7701_DEVICE(inst)									\
	static const struct st7701_config st7701_config_##inst = {				\
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(inst)),					\
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),			\
		.backlight = GPIO_DT_SPEC_INST_GET_OR(inst, bl_gpios, {0}),			\
		.data_lanes = DT_INST_PROP_BY_IDX(inst, data_lanes, 0),				\
		.width = DT_INST_PROP(inst, width),						\
		.height = DT_INST_PROP(inst, height),						\
		.channel = DT_INST_REG_ADDR(inst),						\
		.rotation = DT_INST_PROP(inst, rotation),					\
	};											\
	static struct st7701_data st7701_data_##inst = {					\
		.dsi_pixel_format = DT_INST_PROP(inst, pixel_format),				\
	};											\
	DEVICE_DT_INST_DEFINE(inst, &st7701_init, NULL, &st7701_data_##inst,		\
			      &st7701_config_##inst, POST_KERNEL,				\
			      CONFIG_DISPLAY_ST7701_INIT_PRIORITY, &st7701_api);		\

DT_INST_FOREACH_STATUS_OKAY(ST7701_DEVICE)
