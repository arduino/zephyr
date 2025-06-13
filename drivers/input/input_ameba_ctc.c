/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_captouch

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <stdbool.h>
#include <zephyr/input/input.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ctc_ameba, CONFIG_INPUT_LOG_LEVEL);

/* Device config structure */
struct ctc_ameba_config {
	CAPTOUCH_TypeDef *captouch;
	/* the number of captouch channels */
	const uint8_t channel_count;
	/* clock device */
	const struct device *clock;
	const clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
};

static void ctc_ameba_isr(const struct device *dev)
{
	const struct ctc_ameba_config *config = dev->config;
	CAPTOUCH_TypeDef *pctc = config->captouch;
	uint8_t i;
	uint32_t int_sts = CapTouch_GetISR(pctc);
	static int ctc_input_key[] = DT_INST_PROP(0, input_key);

	for (i = 0; i < config->channel_count; i++) {
		if (int_sts & CT_CHX_PRESS_INT(i)) {
			LOG_INF("Key %d press\n", i);
			input_report_key(dev, ctc_input_key[i], 1, true, K_FOREVER);
		} else if (int_sts & CT_CHX_RELEASE_INT(i)) {
			LOG_INF("Key %d release", i);
			input_report_key(dev, ctc_input_key[i], 0, true, K_FOREVER);
		}
	}

	CapTouch_INTClearPendingBit(pctc, int_sts);
}

static int ctc_ameba_init(const struct device *dev)
{
	const struct ctc_ameba_config *config = dev->config;
	CAPTOUCH_TypeDef *pctc = config->captouch;
	CapTouch_InitTypeDef ctc_init_struct;
	uint8_t i;

	uint8_t ctc_ch_num = DT_INST_PROP(0, channel_count);
	int ctc_diff_th[] = DT_INST_PROP(0, diff_thre);
	int ctc_bias[] = DT_INST_PROP(0, mbias_current);
	int ctc_nnoise_th[] = DT_INST_PROP(0, nnoise_thre);
	int ctc_pnoise_th[] = DT_INST_PROP(0, pnoise_thre);
	int ctc_ch_sts[] = DT_INST_PROP(0, channel_status);

	if (ARRAY_SIZE(ctc_diff_th) != ctc_ch_num) {
		LOG_ERR("Unmatched diff_thre count!");
		return -EIO;
	}

	if (ARRAY_SIZE(ctc_bias) != ctc_ch_num) {
		LOG_ERR("Unmatched mbias_current count!");
		return -EIO;
	}

	if (ARRAY_SIZE(ctc_nnoise_th) != ctc_ch_num) {
		LOG_ERR("Unmatched nnoise_thre count!");
		return -EIO;
	}

	if (ARRAY_SIZE(ctc_pnoise_th) != ctc_ch_num) {
		LOG_ERR("Unmatched pnoise_thre count!");
		return -EIO;
	}

	if (ARRAY_SIZE(ctc_ch_sts) != ctc_ch_num) {
		LOG_ERR("Unmatched channel_status count!");
		return -EIO;
	}

	if (!device_is_ready(config->clock)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_on(config->clock, config->clock_subsys)) {
		LOG_ERR("Could not enable CAPTOUCH clock");
		return -EIO;
	}

	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}

	CapTouch_StructInit(&ctc_init_struct);

	for (i = 0; i < CT_CHANNEL_NUM; i++) {
		ctc_init_struct.CT_Channel[i].CT_DiffThrehold = ctc_diff_th[i];
		ctc_init_struct.CT_Channel[i].CT_MbiasCurrent = ctc_bias[i];
		ctc_init_struct.CT_Channel[i].CT_ETCNNoiseThr = ctc_nnoise_th[i];
		ctc_init_struct.CT_Channel[i].CT_ETCPNoiseThr = ctc_pnoise_th[i];
		if (ctc_ch_sts[i] == 1) {
			ctc_init_struct.CT_Channel[i].CT_CHEnable = ENABLE;
		}
	}

	CapTouch_Init(pctc, &ctc_init_struct);
	CapTouch_Cmd(pctc, ENABLE);
	CapTouch_INTConfig(pctc, CT_ALL_INT_EN, ENABLE);
	/* Data will be sent to FIFO automatically even when debug mode is off since BCut */
	CapTouch_INTConfig(pctc, CT_BIT_AFIFO_OVERLVL_INTR_EN, DISABLE);
	CapTouch_INTConfig(pctc, CT_BIT_AFIFO_OVERFLOW_INTR_EN, DISABLE);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), ctc_ameba_isr, DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

PINCTRL_DT_INST_DEFINE(0);

static const struct ctc_ameba_config ctc_config = {
	.captouch = (CAPTOUCH_TypeDef *)DT_INST_REG_ADDR(0),
	.channel_count = DT_INST_PROP(0, channel_count),
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, idx),
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

DEVICE_DT_INST_DEFINE(0, ctc_ameba_init, NULL, NULL, &ctc_config, POST_KERNEL,
		      CONFIG_INPUT_INIT_PRIORITY, NULL);
