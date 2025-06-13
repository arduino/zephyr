/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba Led Controller
 */

#define DT_DRV_COMPAT   realtek_ameba_ledc
#define ENABLE_DMA_MODE 0

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/led/led.h>

#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ledc_ameba);

#define RESULT_RUNNING  0
#define RESULT_COMPLETE 1
#define RESULT_ERR      2

struct ameba_ledc_data_struct {
	LEDC_InitTypeDef ledc_init_struct;
	/* GDMA_InitTypeDef ledc_dma_struct; */

	uint32_t *tx_data;     /* tx data handle */
	uint16_t tx_total_len; /* tx total length */
	uint16_t tx_len;       /* tx len that has been wrote to the FIFO */
	uint8_t irq_result: 2; /* tx status */
};

struct ameba_ledc_cfg_struct {
	const struct pinctrl_dev_config *pinctrl_dev;
	const struct device *clock_dev;
	const uint8_t *color_mapping;
	const clock_control_subsys_t clock_subsys;

	uint32_t wait_data_time_ns;
	uint32_t reset_ns;

	uint32_t t0h_ns;
	uint32_t t0l_ns;
	uint32_t t1h_ns;
	uint32_t t1l_ns;

	uint16_t led_num_cfg; /* max 1024 */

	uint8_t num_colors;
	uint8_t output_RGB_mode;
	uint8_t dma_mode;
};

static void ameba_ledc_isr_handle(const struct device *dev)
{
	struct ameba_ledc_data_struct *pdata = dev->data;
	uint32_t intr_status;
	uint32_t ledc_fifothr;
	uint32_t *start_addr;

	LEDC_INTConfig(LEDC_DEV, LEDC_BIT_GLOBAL_INT_EN, DISABLE);

	intr_status = LEDC_GetINT(LEDC_DEV);
	/* LOG_DBG("ISR status %x-0x%08x", intr_status,LEDC_DEV->LEDC_LED_INTERRUPT_CTRL_REG); */

	if (intr_status & LEDC_BIT_FIFO_CPUREQ_INT) {
		LEDC_ClearINT(LEDC_DEV, LEDC_BIT_FIFO_CPUREQ_INT);

		ledc_fifothr = LEDC_GetFIFOLevel(LEDC_DEV);
		start_addr = pdata->tx_data + pdata->tx_len;

		if ((pdata->tx_total_len - pdata->tx_len) >= ledc_fifothr) {
			pdata->tx_len += LEDC_SendData(LEDC_DEV, start_addr, ledc_fifothr);
		} else {
			pdata->tx_len += LEDC_SendData(LEDC_DEV, start_addr,
						       pdata->tx_total_len - pdata->tx_len);
		}

		LEDC_INTConfig(LEDC_DEV, LEDC_BIT_GLOBAL_INT_EN, ENABLE);
		return;
	}

	if (intr_status & LEDC_BIT_LED_TRANS_FINISH_INT) {
		LEDC_ClearINT(LEDC_DEV, LEDC_BIT_LED_TRANS_FINISH_INT);
		/* LOG_DBG("TX DONE %x", intr_status); */

		pdata->irq_result = RESULT_COMPLETE;
		LEDC_SoftReset(LEDC_DEV);
	}

	if (intr_status & LEDC_BIT_WAITDATA_TIMEOUT_INT) {
		LEDC_ClearINT(LEDC_DEV, LEDC_BIT_WAITDATA_TIMEOUT_INT);

		pdata->irq_result = RESULT_ERR;
		LEDC_SoftReset(LEDC_DEV);
	}

	if (intr_status & LEDC_BIT_FIFO_OVERFLOW_INT) {
		LEDC_ClearINT(LEDC_DEV, LEDC_BIT_FIFO_OVERFLOW_INT);

		pdata->irq_result = RESULT_ERR;
		LEDC_SoftReset(LEDC_DEV);
	}

	LEDC_INTConfig(LEDC_DEV, LEDC_BIT_GLOBAL_INT_EN, ENABLE);
}

static int ameba_ledc_update_rgb(const struct device *dev, struct led_rgb *pixels,
				 size_t num_pixels)
{
	const struct ameba_ledc_cfg_struct *cfg = dev->config;
	struct ameba_ledc_data_struct *pdata = dev->data;
	uint16_t data_len = (uint16_t)num_pixels;
	uint8_t i;

	/* LEDC_MAX_DATA_LENGTH 0x2000 */
	if (!IS_LEDC_DATA_LENGTH(num_pixels)) {
		LOG_WRN("Total data length too long, force to Max %d", LEDC_MAX_DATA_LENGTH);
		data_len = LEDC_MAX_DATA_LENGTH;
	}

	pdata->tx_len = 0;
	pdata->tx_data = (uint32_t *)pixels;
	pdata->tx_total_len = data_len;
	pdata->irq_result = RESULT_RUNNING;

	pdata->ledc_init_struct.data_length = data_len;
	LEDC_SetTotalLength(LEDC_DEV, pdata->ledc_init_struct.data_length);

	LOG_DBG("Write %d data/0x%08x cnt %d", pdata->ledc_init_struct.data_length,
		pdata->tx_data[0], cfg->num_colors);

	/* Convert from RGB to on-wire format (e.g. GRB, GRBW, RGB, etc) */
	for (i = 0; i < num_pixels; i++) {
		uint8_t j;
		struct led_rgb pixel_tmp = {0, 0, 0, 0};
		uint8_t *ptr = (uint8_t *)&pixel_tmp;

		for (j = 0; j < cfg->num_colors; j++) {
			switch (cfg->color_mapping[j]) {
			case LED_COLOR_ID_RED:
				*ptr++ = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				*ptr++ = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				*ptr++ = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
		}

		memcpy(pixels + i, &pixel_tmp, sizeof(struct led_rgb));
	}
	LOG_DBG("Write %d data 0x%08x", num_pixels, pdata->tx_data[0]);

	LEDC_Cmd(LEDC_DEV, ENABLE);

	while (pdata->irq_result == RESULT_RUNNING) {
		/* Sleep to release cpu, wait the interrupt change the irq_status */
		k_msleep(10);
	};

	if (pdata->irq_result == RESULT_COMPLETE) {
		LOG_DBG("Ledc TX done!\n");
		return 0;
	}

	if (pdata->irq_result == RESULT_ERR) {
		LOG_WRN("Ledc err!\n");
		return -EFAULT;
	}

	LOG_WRN("Ledc exit %d\n", pdata->irq_result);
	return -EFAULT;
}

static int ameba_ledc_update_channels(const struct device *dev, uint8_t *channels,
				      size_t num_channels)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channels);
	ARG_UNUSED(num_channels);
	LOG_WRN("Update_channels not support");
	return -ENOTSUP;
}

static const struct led_strip_driver_api ameba_ledc_api = {
	.update_rgb = ameba_ledc_update_rgb,
	.update_channels = ameba_ledc_update_channels,
};

/*
 */
static int ameba_ledc_init(const struct device *dev)
{
	const struct ameba_ledc_cfg_struct *cfg = dev->config;
	struct ameba_ledc_data_struct *data = dev->data;
	LEDC_InitTypeDef *pledc_init_struct = &(data->ledc_init_struct);
	uint16_t led_num;
	int err = 0;

	/* enable clock */
	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (err != 0) {
		LOG_ERR("Enable clk %d err %d\n", (uint32_t)cfg->clock_subsys, err);
		return err;
	}

	/* enable pinctrl */
	if (pinctrl_apply_state(cfg->pinctrl_dev, PINCTRL_STATE_DEFAULT)) {
		LOG_ERR("Pinctrl device not ready");
		return -ENODEV;
	}

	/* check the dts config valid
	 * LEDC_MAX_LED_NUM 1024
	 */
	if (!IS_LEDC_LED_NUM(cfg->led_num_cfg)) {
		LOG_ERR("Illegal parameter: LED cnt %d, force to Max %d\n", cfg->led_num_cfg,
			LEDC_MAX_LED_NUM);
		led_num = LEDC_MAX_LED_NUM;
	} else {
		led_num = cfg->led_num_cfg;
	}

	/* ledc init */
	LEDC_StructInit(pledc_init_struct);

	pledc_init_struct->led_count = led_num;
#if ENABLE_DMA_MODE
	pledc_init_struct->ledc_trans_mode = cfg->dma_mode;
#else
	pledc_init_struct->ledc_trans_mode = 0;
#endif
	pledc_init_struct->t1h_ns = cfg->t1h_ns;
	pledc_init_struct->t1l_ns = cfg->t1l_ns;
	pledc_init_struct->t0h_ns = cfg->t0h_ns;
	pledc_init_struct->t0l_ns = cfg->t0l_ns;
	pledc_init_struct->reset_ns = cfg->reset_ns;
	pledc_init_struct->wait_data_time_ns = cfg->wait_data_time_ns;
	pledc_init_struct->output_RGB_mode = cfg->output_RGB_mode;
	pledc_init_struct->data_length = LEDC_DEFAULT_LED_NUM;
	pledc_init_struct->ledc_fifo_level = 0xF;
	pledc_init_struct->ledc_polarity = LEDC_IDLE_POLARITY_LOW;
	pledc_init_struct->wait_time0_en = ENABLE;
	pledc_init_struct->wait_time1_en = ENABLE;
	pledc_init_struct->wait_time0_ns = 0xEF;      /* 6us */
	pledc_init_struct->wait_time1_ns = 0x2625A00; /* 1000000000ns */

	LEDC_Init(LEDC_DEV, pledc_init_struct);

	/* LEDC_SetInputMode(LEDC_DEV, 0);
	 * LEDC_SetOutputMode(LEDC_DEV, 0);
	 */

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), ameba_ledc_isr_handle,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	LOG_DBG("Ledc init finish");

	return 0;
}

PINCTRL_DT_INST_DEFINE(0);
static const uint8_t ameba_ledc_color_mapping[] = DT_INST_PROP(0, color_mapping);
static struct ameba_ledc_data_struct ameba_ledc_data;
static const struct ameba_ledc_cfg_struct ameba_ledc_cfg = {
	.pinctrl_dev = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, idx),
	.num_colors = DT_INST_PROP_LEN(0, color_mapping),
	.color_mapping = ameba_ledc_color_mapping,

	.dma_mode = DT_PROP(DT_NODELABEL(ledc), dma_mode),
	.led_num_cfg = DT_PROP(DT_NODELABEL(ledc), chain_length),
	.output_RGB_mode = DT_PROP(DT_NODELABEL(ledc), output_rgb_mode),
	.wait_data_time_ns = DT_PROP(DT_NODELABEL(ledc), wait_data_timeout),
	.t0h_ns = DT_PROP(DT_NODELABEL(ledc), data_tx_time0h),
	.t0l_ns = DT_PROP(DT_NODELABEL(ledc), data_tx_time0l),
	.t1h_ns = DT_PROP(DT_NODELABEL(ledc), data_tx_time1h),
	.t1l_ns = DT_PROP(DT_NODELABEL(ledc), data_tx_time1l),
	.reset_ns = DT_PROP(DT_NODELABEL(ledc), refresh_time),
};

DEVICE_DT_INST_DEFINE(0, &ameba_ledc_init, NULL, &ameba_ledc_data, &ameba_ledc_cfg, POST_KERNEL,
		      CONFIG_LED_STRIP_INIT_PRIORITY, &ameba_ledc_api);
