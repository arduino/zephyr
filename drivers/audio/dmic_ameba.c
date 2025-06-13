/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_codec

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#define LOG_LEVEL CONFIG_AUDIO_DMIC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_ameba);

struct dmic_ameba_config {
	const struct device *comm_master;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
};

struct dmic_ameba_data {
	enum dmic_state state;
	size_t pcm_mem_size;
	struct k_mem_slab *pcm_mem_slab;
};

static int dmic_ameba_enable_clock(const struct device *dev)
{
	const struct dmic_ameba_config *config = dev->config;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enables I2S peripheral */
	if (clock_control_on(config->clock_dev, config->clock_subsys)) {
		LOG_ERR("Could not enable CODEC clock");
		return -EIO;
	}

	return 0;
}

int dmic_ameba_configure(const struct device *dev, struct dmic_cfg *cfg)
{
	int ret;
	const struct dmic_ameba_config *config = dev->config;
	struct dmic_ameba_data *const data = dev->data;

	/* PCM buffer size */
	data->pcm_mem_slab = cfg->streams->mem_slab;
	data->pcm_mem_size = cfg->streams->block_size;

	/* configure I2S channels */
	struct i2s_config i2s_cfg;

	i2s_cfg.word_size = cfg->streams->pcm_width;
	i2s_cfg.channels = cfg->channel.req_num_chan;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = cfg->streams->pcm_rate;
	i2s_cfg.block_size = data->pcm_mem_size;
	i2s_cfg.mem_slab = data->pcm_mem_slab;
	i2s_cfg.timeout = 2000;

	ret = i2s_configure(config->comm_master, I2S_DIR_RX, &i2s_cfg);
	if (ret != 0) {
		LOG_ERR("I2S device configuration error");
		return ret;
	}

	I2S_InitTypeDef I2S_InitStruct;
	uint8_t codec_rx_sr;
	uint8_t tdm_mode;

	switch (i2s_cfg.channels) {
	case 1:
	case 2:
		tdm_mode = 0x00;
		break;

	case 3:
	case 4:
		tdm_mode = 0x01;
		break;

	case 5:
	case 6:
		tdm_mode = 0x02;
		break;

	case 7:
	case 8:
		tdm_mode = 0x03;
		break;

	default:
		LOG_ERR("invalid chnnels");
		return -EINVAL;
	}

	switch (i2s_cfg.frame_clk_freq) {
	case SP_48K:
		codec_rx_sr = SR_48K;
		break;

	case SP_96K:
		codec_rx_sr = SR_96K;
		break;

	case SP_192K:
		codec_rx_sr = SR_192K;
		break;

	case SP_32K:
		codec_rx_sr = SR_32K;
		break;

	case SP_176P4K:
		codec_rx_sr = SR_176P4K;
		break;

	case SP_16K:
		codec_rx_sr = SR_16K;
		break;

	case SP_8K:
		codec_rx_sr = SR_8K;
		break;

	case SP_44P1K:
		codec_rx_sr = SR_44P1K;
		break;

	case SP_88P2K:
		codec_rx_sr = SR_88P2K;
		break;

	case SP_24K:
		codec_rx_sr = SR_24K;
		break;

	case SP_12K:
		codec_rx_sr = SR_12K;
		break;

	case SP_22P05K:
		codec_rx_sr = SR_22P05K;
		break;

	case SP_11P025K:
		codec_rx_sr = SR_11P025K;
		break;

	default:
		LOG_ERR("invalid sample rate");
		return -EINVAL;
	}

	AUDIO_CODEC_I2S_StructInit(&I2S_InitStruct);
	I2S_InitStruct.CODEC_SelRxI2STdm = tdm_mode;
	I2S_InitStruct.CODEC_SelI2SRxSR = codec_rx_sr;
	AUDIO_CODEC_Record(I2S0, APP_DMIC_RECORD, &I2S_InitStruct);

	/* when sample rate is 96k,the clock need 5M, otherwise the waveform will have burrs */
	if (codec_rx_sr == SR_96K) {
		AUDIO_CODEC_SetDmicClk(DMIC_5M, ENABLE);
	} else {
		AUDIO_CODEC_SetDmicClk(DMIC_2P5M, ENABLE);
	}

	/* ADC digital volume gain*/
	AUDIO_CODEC_SetADCVolume(ADC1, 0x2f);
	AUDIO_CODEC_SetADCVolume(ADC2, 0x2f);

	data->state = DMIC_STATE_CONFIGURED;

	return 0;
}

int dmic_ameba_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	int ret;
	const struct dmic_ameba_config *config = dev->config;
	struct dmic_ameba_data *const data = dev->data;

	enum i2s_trigger_cmd i2s_cmd;
	enum dmic_state tmp_state;

	switch (cmd) {
	case DMIC_TRIGGER_START:
		if (data->state == DMIC_STATE_CONFIGURED) {
			tmp_state = DMIC_STATE_ACTIVE;
			i2s_cmd = I2S_TRIGGER_START;
		} else {
			return 0;
		}
		break;
	case DMIC_TRIGGER_STOP:
		if (data->state == DMIC_STATE_ACTIVE) {
			tmp_state = DMIC_STATE_CONFIGURED;
			i2s_cmd = I2S_TRIGGER_DROP;
		} else {
			return 0;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = i2s_trigger(config->comm_master, I2S_DIR_RX, i2s_cmd);
	if (ret != 0) {
		LOG_ERR("trigger failed with %d error", ret);
		return ret;
	}

	data->state = tmp_state;
	return 0;
}

int dmic_ameba_read(const struct device *dev, uint8_t stream, void **buffer, size_t *size,
		    int32_t timeout)
{
	int ret;
	const struct dmic_ameba_config *config = dev->config;

	ret = i2s_read(config->comm_master, buffer, size);
	if (ret != 0) {
		LOG_ERR("read failed (%d)", ret);
		return ret;
	}

	return 0;
}

#define I2S(idx)      DT_NODELABEL(i2s##idx)
#define I2S_NODE(idx) I2S(idx)
#define I2S_BINDING   I2S_NODE(DT_INST_PROP(0, i2s_index))

static const struct _dmic_ops dmic_ameba_api = {
#if DT_NODE_HAS_STATUS(I2S_BINDING, okay)
	.configure = dmic_ameba_configure,
	.trigger = dmic_ameba_trigger,
	.read = dmic_ameba_read,
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2s) */
};

static int dmic_ameba_init(const struct device *dev)
{
	int ret;
	const struct dmic_ameba_config *config = dev->config;
	struct dmic_ameba_data *const data = dev->data;

	if (!device_is_ready(config->comm_master)) {
		return -ENODEV;
	}

	/* Enable I2S clock propagation */
	ret = dmic_ameba_enable_clock(dev);
	if (ret < 0) {
		LOG_ERR("%s: clock enabling failed: %d", __func__, ret);
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("I2S pinctrl setup failed (%d)", ret);
		return ret;
	}

	data->state = DMIC_STATE_INITIALIZED;
	return 0;
}

#define DMIC_AMEBA_INIT(n)                                                                         \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct dmic_ameba_config dmic_ameba_config_##n = {                            \
		.comm_master = DEVICE_DT_GET(I2S_BINDING),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
                                                                                                   \
	static struct dmic_ameba_data dmic_ameba_data_##n;                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, dmic_ameba_init, NULL, &dmic_ameba_data_##n,                      \
			      &dmic_ameba_config_##n, POST_KERNEL,                                 \
			      CONFIG_AUDIO_DMIC_INIT_PRIORITY, &dmic_ameba_api);

DMIC_AMEBA_INIT(0)
