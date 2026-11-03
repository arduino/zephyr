/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <stdbool.h>
#include <zephyr/dt-bindings/adc/mchp_pic32ck_gc_adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define DT_DRV_COMPAT microchip_adc_g2

LOG_MODULE_REGISTER(adc_mchp_g2, CONFIG_ADC_LOG_LEVEL);

#define ADC_CALC_TAD_NS(gclk_adc_hz, ctrl_clock_div, adc_div_ratio)                                \
	(((ctrl_clock_div + 1) * 2 * adc_div_ratio * 1000000000ULL) / gclk_adc_hz)

#define ADC_CALC_SAMPLE_COUNT(tad, res, acq_time_ns) ((((uint64_t)(acq_time_ns)) / tad) - res - 3)

/* ADC resolution options (in bits) */
#define ADC_RESOLUTION_6BIT  6
#define ADC_RESOLUTION_8BIT  8
#define ADC_RESOLUTION_10BIT 10
#define ADC_RESOLUTION_12BIT 12

#define CHNCFG4_CHANNEL_MAX         7
#define CHNCFG_BITS_PER_TRGSRC      4
#define CHNCFG_SOFTWARE_TRIGGER_Val 1

#define ADC_SAMPLE_COUNT_MIN     3
#define ADC_SAMPLE_COUNT_DEFAULT 5
#define ADC_SAMPLE_COUNT_MAX     1025

#define ADC_MAX_OVERSAMPLING_VAL 8

#define ADC_WKUPEXP_DELAY     10000
#define ADC_VREF_STABLE_DELAY 20000
#define TIMEOUT_VALUE_US      1000
#define DELAY_US              2

struct adc_mchp_dev_data {
	struct adc_context ctx;
	const struct device *dev;
	int16_t *buffer;
	int16_t *repeat_buffer;
	uint32_t channels;
	uint32_t freq; /* ADC operating gclk frequency in Hz. */
	uint32_t tad;  /* ADC Clock Period */
	uint8_t channel_id;
	uint32_t ch_initialized;
	struct adc_channel_cfg *saved_cfg;
};

struct mchp_adc_clock {
	const struct device *clock_dev;
	clock_control_subsys_t mclk_sys;
	clock_control_subsys_t gclk_sys;
};

struct adc_mchp_dev_config {
	adc_registers_t *regs;
	fuses_calotp_registers_t *fuses;
	const struct pinctrl_dev_config *pcfg;
	struct mchp_adc_clock adc_clock;
	uint8_t ctrl_clock_div;       /* divides the GCLK_ADC input clock into ADC control clock */
	uint8_t adc_div_ratio;        /* Division Ratio for ADC Sampling Clock */
	uint8_t adc_wkup_clock_count; /* Wake-Up TAD Clock Count */
	uint8_t num_channels;         /* Maximum number of ADC channels. */
	void (*config_func)(const struct device *dev);
};

/* Wait for synchronization */
static inline void adc_wait_synchronization(adc_registers_t *adc_reg)
{
	if (WAIT_FOR(((adc_reg->ADC_SYNCBUSY & ADC_SYNCBUSY_Msk) == 0),
		     TIMEOUT_VALUE_US,                  /* 1 ms timeout */
		     k_busy_wait(DELAY_US)) == false) { /* 2 µs delay between polls */
		LOG_ERR("Timeout waiting for ADC_SYNCBUSY to clear");
	}
}

/* Calculate sample count from acquisition time */
static uint32_t adc_get_sample_count(uint32_t tad, uint8_t res, uint16_t acq_time)
{
	uint32_t sample_count, acq_time_ns;

	switch (ADC_ACQ_TIME_UNIT(acq_time)) {
	case ADC_ACQ_TIME_TICKS:
		acq_time_ns = k_ticks_to_ns_floor64(ADC_ACQ_TIME_VALUE(acq_time));
		break;
	case ADC_ACQ_TIME_MICROSECONDS:
		acq_time_ns = (ADC_ACQ_TIME_VALUE(acq_time)) * 1000;
		break;
	case ADC_ACQ_TIME_NANOSECONDS:
		acq_time_ns = ADC_ACQ_TIME_VALUE(acq_time);
		break;
	default:
		/* Unsupported acquisition time unit or ADC_ACQ_TIME_DEFAULT */
		return ADC_SAMPLE_COUNT_DEFAULT;
	}
	sample_count = ADC_CALC_SAMPLE_COUNT(acq_time_ns, tad, res);

	/* Clip if value went out of range */
	sample_count = (sample_count < ADC_SAMPLE_COUNT_MIN) ? ADC_SAMPLE_COUNT_MIN : sample_count;
	sample_count = (sample_count > ADC_SAMPLE_COUNT_MAX) ? ADC_SAMPLE_COUNT_MAX : sample_count;

	return sample_count;
}

static void adc_start_channel(const struct device *dev, struct adc_context *ctx)
{
	const struct adc_mchp_dev_config *const dev_cfg = dev->config;
	struct adc_mchp_dev_data *dev_data = dev->data;
	adc_registers_t *adc_reg = dev_cfg->regs;
	struct adc_channel_cfg *cfg;
	uint32_t reg_val;
	uint16_t sample_count;

	/* Determine the next channel to process by finding the least significant bit set */
	dev_data->channel_id = find_lsb_set(dev_data->channels) - 1;

	/* Get the configuration for the selected channel, which is already validated. */
	cfg = &(dev_data->saved_cfg[dev_data->channel_id]);

	/* Set Acquisition sample count */
	sample_count = adc_get_sample_count(dev_data->tad, ctx->sequence.resolution,
					    cfg->acquisition_time);
	reg_val = adc_reg->CONFIG[0].ADC_CORCTRL & (~ADC_CORCTRL_SAMC_Msk);
	adc_reg->CONFIG[0].ADC_CORCTRL = reg_val | ADC_CORCTRL_SAMC(sample_count);

	/* Apply reference selection */
	reg_val = adc_reg->ADC_CTRLD & (~ADC_CTRLD_VREFSEL_Msk);
	if (cfg->reference == ADC_REF_VDD_1) {
		adc_reg->ADC_CTRLD = reg_val | ADC_CTRLD_VREFSEL_AVDD_AVSS;
	} else {
		adc_reg->ADC_CTRLD = reg_val | ADC_CTRLD_VREFSEL_EXTERNAL_VREFH_AVSS;
	}

	/* Select the channel(s) to be used in this conversion */
	if (cfg->differential == true) {
		reg_val = adc_reg->CONFIG[0].ADC_CHNCFG3 & ~(ADC_CHNCFG3_DIFF_Msk);
		adc_reg->CONFIG[0].ADC_CHNCFG3 = reg_val | ADC_CHNCFG3_DIFF(cfg->channel_id);
	}

	/* Enable the ADC controller. */
	adc_reg->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;
	adc_wait_synchronization(adc_reg);

	/* Wait for voltage reference to be stable. It will only be updated if CTRLA.ENABLE is on */
	if (WAIT_FOR(((adc_reg->ADC_CTLINTFLAG & ADC_CTLINTFLAG_VREFRDY_Msk) ==
		      ADC_CTLINTFLAG_VREFRDY_Msk),
		     ADC_VREF_STABLE_DELAY, k_busy_wait(DELAY_US)) == false) {
		LOG_ERR("Timeout waiting for ADC_VREF_STABLE_DELAY");
	}

	reg_val = adc_reg->ADC_CTRLB & ~(ADC_CTRLB_ADCHSEL_Msk);
	adc_reg->ADC_CTRLB = reg_val | ADC_CTRLB_ADCHSEL(cfg->channel_id);
	adc_wait_synchronization(adc_reg);

	adc_reg->INT[0].ADC_INTENSET |= ADC_INTENSET_CHRDY(1 << cfg->channel_id);

	/* Start the ADC conversion */
	adc_reg->ADC_CTRLB |= ADC_CTRLB_GSWTRG_Msk;
	adc_wait_synchronization(adc_reg);
}

static int adc_check_buffer_size(const struct adc_sequence *sequence, uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);
	if (sequence->options != NULL) {
		needed_buffer_size *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)", sequence->buffer_size,
			needed_buffer_size);
		return -ENOMEM;
	}

	return 0;
}

/* Function required to be implemented for adc_context. And will be called when a sampling (of one
 * or more channels, depending on the realized sequence) is to be started.
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_mchp_dev_data *dev_data = CONTAINER_OF(ctx, struct adc_mchp_dev_data, ctx);

	dev_data->channels = ctx->sequence.channels;

	adc_start_channel(dev_data->dev, ctx);
}

static int adc_set_oversampling(adc_registers_t *adc_reg, uint8_t oversampling, uint8_t channel)
{
	uint8_t reg_val;

	/*
	 * Oversampling configuration:
	 * 0x0 = 1 sample
	 * 0x1 = 2 samples
	 * 0x2 = 4 samples
	 * 0x3 = 8 samples
	 * 0x4 = 16 samples
	 * 0x5 = 32 samples
	 * 0x6 = 64 samples
	 * 0x7 = 128 samples
	 * 0x8 = 256 samples
	 *
	 * Valid range: 0 to 8 (inclusive)
	 */
	if (oversampling > ADC_MAX_OVERSAMPLING_VAL) {
		LOG_ERR("Invalid oversampling: %d\n", oversampling);
		return -EINVAL;
	}
	if (oversampling > 0) {
		uint8_t oversam_reg_val[ADC_MAX_OVERSAMPLING_VAL] = {
			ADC_FLTCTRL_OVRSAM_2_SAMPLES_Val,   ADC_FLTCTRL_OVRSAM_4_SAMPLES_Val,
			ADC_FLTCTRL_OVRSAM_8_SAMPLES_Val,   ADC_FLTCTRL_OVRSAM_16_SAMPLES_Val,
			ADC_FLTCTRL_OVRSAM_32_SAMPLES_Val,  ADC_FLTCTRL_OVRSAM_64_SAMPLES_Val,
			ADC_FLTCTRL_OVRSAM_128_SAMPLES_Val, ADC_FLTCTRL_OVRSAM_256_SAMPLES_Val};
		reg_val =
			adc_reg->ADC_FLTCTRL & ~(ADC_FLTCTRL_OVRSAM_Msk | ADC_FLTCTRL_FLTCHNID_Msk);
		reg_val |= (ADC_FLTCTRL_FLTEN_Msk | ADC_FLTCTRL_FMODE_Msk |
			    ADC_FLTCTRL_FLTCHNID(channel) |
			    ADC_FLTCTRL_OVRSAM(oversam_reg_val[oversampling - 1]));
		adc_reg->ADC_FLTCTRL = reg_val;
	}

	return 0;
}

static int adc_set_resolution(adc_registers_t *adc_reg, uint8_t resolution)
{
	uint16_t resolution_val;
	uint16_t reg_val;

	reg_val = adc_reg->CONFIG[0].ADC_CORCTRL & ~(ADC_CORCTRL_SELRES_Msk);
	switch (resolution) {
	case ADC_RESOLUTION_6BIT:
		resolution_val = ADC_CORCTRL_SELRES_6_BITS_Val;
		break;
	case ADC_RESOLUTION_8BIT:
		resolution_val = ADC_CORCTRL_SELRES_8_BITS_Val;
		break;
	case ADC_RESOLUTION_10BIT:
		resolution_val = ADC_CORCTRL_SELRES_10_BITS_Val;
		break;
	case ADC_RESOLUTION_12BIT:
		resolution_val = ADC_CORCTRL_SELRES_12_BITS_Val;
		break;
	default:
		LOG_ERR("Invalid resolution: %d\n", resolution);
		return -EINVAL;
	}
	adc_reg->CONFIG[0].ADC_CORCTRL = reg_val | ADC_CORCTRL_SELRES(resolution_val);

	return 0;
}

static int adc_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_mchp_dev_config *const dev_cfg = dev->config;
	struct adc_mchp_dev_data *dev_data = dev->data;
	int ret;
	uint32_t channels, channel_count, index;

	if (sequence->channels == 0) {
		LOG_ERR("No channels selected!\n");
		return -EINVAL;
	}

	/* Set Resolution */
	ret = adc_set_resolution(ADC_REGS, sequence->resolution);
	if (ret != 0) {
		LOG_ERR("Invalid resolution : %d\n", sequence->resolution);
		return ret;
	}

	/* Verify all requested channels are initialized and store resolution */
	channels = sequence->channels;
	channel_count = 0;
	while (channels != 0) {
		/* Iterate through all channels and check if they are initialized */
		index = find_lsb_set(channels) - 1;
		if (index >= dev_cfg->num_channels) {
			LOG_ERR("Invalid channel number : %d", index);
			return -EINVAL;
		}
		/* If the channels is not initialized return invalid */
		if ((dev_data->ch_initialized & (1 << index)) == 0) {
			LOG_ERR("Channel is not initialized");
			return -EINVAL;
		}

		/* Set oversampling */
		ret = adc_set_oversampling(ADC_REGS, sequence->oversampling, index);
		if (ret != 0) {
			LOG_ERR("Invalid oversampling : %d\n", sequence->oversampling);
			return ret;
		}

		channel_count++;
		channels &= ~BIT(index);
	}

	/* Check buffer */
	ret = adc_check_buffer_size(sequence, channel_count);
	if (ret != 0) {
		LOG_ERR("Check buffer size invalid\n");
		return ret;
	}

	/* Store buffer references for use during sampling */
	dev_data->buffer = sequence->buffer;
	dev_data->repeat_buffer = sequence->buffer;

	/* At this point we allow the scheduler to do other things while
	 * we wait for the conversions to complete. This is provided by the
	 * adc_context functions. However, the caller of this function is
	 * blocked until the results are in.
	 * adc_context_start_read --> adc_context_start_sampling() --> adc_start_channel()
	 */
	adc_context_start_read(&dev_data->ctx, sequence);

	/* Wait for all ADC conversions to complete, if it's a synchronous call */
	ret = adc_context_wait_for_completion(&dev_data->ctx);

	return ret;
}

/* Function required to be implemented for adc_context. And will be called when the sample buffer
 * pointer should be prepared for writing of next sampling results, the "repeat_sampling" parameter
 * indicates if the results should be written in the same place as before (when true) or as
 * consecutive ones (otherwise).
 */
static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_mchp_dev_data *data = CONTAINER_OF(ctx, struct adc_mchp_dev_data, ctx);

	if (repeat_sampling == true) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_mchp_isr(const struct device *dev)
{
	const struct adc_mchp_dev_config *const dev_cfg = dev->config;
	struct adc_mchp_dev_data *dev_data = dev->data;
	adc_registers_t *adc_reg = dev_cfg->regs;
	uint16_t result;

	/* Clear interrupt. */
	adc_reg->INT[0].ADC_INTFLAG |= ADC_INTFLAG_CHRDYC(1 << dev_data->channel_id);
	adc_reg->ADC_CORCHDATAID = (adc_reg->ADC_CORCHDATAID & ~(ADC_CORCHDATAID_CHRDYID_Msk)) |
				   (dev_data->channel_id);
	result = adc_reg->ADC_CHRDYDAT & ADC_CHRDYDAT_CHRDYDAT_Msk;
	*dev_data->buffer = result;
	dev_data->buffer++;
	dev_data->channels &= ~BIT(dev_data->channel_id);

	if (dev_data->channels != 0) {
		/* If multiple channels are configured, continue sampling the next channel */
		adc_start_channel(dev, &dev_data->ctx);
	} else {
		/* Disable the ADC controller. */
		adc_reg->ADC_CTRLA &= ~ADC_CTRLA_ENABLE_Msk;
		adc_wait_synchronization(adc_reg);

		/* If no additional channels, notify that sampling is complete */
		adc_context_on_sampling_done(&dev_data->ctx, dev);
	}
}

static int adc_mchp_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_mchp_dev_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);
	ret = adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

#ifdef CONFIG_ADC_ASYNC

static int adc_mchp_read_async(const struct device *dev, const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_mchp_dev_data *data = dev->data;
	int ret = 0;

	adc_context_lock(&data->ctx, true, async);
	ret = adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}
#endif /* CONFIG_ADC_ASYNC */

static int adc_mchp_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	struct adc_mchp_dev_data *dev_data = dev->data;
	const struct adc_mchp_dev_config *const dev_cfg = dev->config;
	adc_registers_t *adc_reg = dev_cfg->regs;
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_id >= dev_cfg->num_channels) {
		LOG_ERR("Invalid Channel id : %d\n", channel_id);
		return -EINVAL;
	}

	/* Mark as, not initialized */
	dev_data->ch_initialized &= ~(1 << channel_id);

	/* ADC doesn't have programmable gain */
	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported gain setting");
		return -ENOTSUP;
	}

	/* Validate reference */
	if ((channel_cfg->reference != ADC_REF_VDD_1) &&
	    (channel_cfg->reference != ADC_REF_EXTERNAL0)) {
		LOG_ERR("Invalid reference : %d\n", channel_cfg->reference);
		return -EINVAL;
	}

	/* Validate differential input */
	if ((channel_cfg->differential == true) && (channel_id != MCHP_ADC_AIN0) &&
	    (channel_id != MCHP_ADC_AIN2) && (channel_id != MCHP_ADC_AIN4)) {
		LOG_ERR("Invalid differential inputs\n");
		return -EINVAL;
	}

	/* identify if CHNCFG4 or CHNCFG5 and set trigger source */
	__IO uint32_t *chncfg = (channel_id > CHNCFG4_CHANNEL_MAX)
					? (&(adc_reg->CONFIG[0].ADC_CHNCFG5))
					: (&(adc_reg->CONFIG[0].ADC_CHNCFG4));
	*chncfg = CHNCFG_SOFTWARE_TRIGGER_Val
		  << ((channel_id % (CHNCFG4_CHANNEL_MAX + 1)) * CHNCFG_BITS_PER_TRGSRC);

	/* Add the channel config to the saved_cfg array */
	dev_data->saved_cfg[channel_id] = *channel_cfg;

	/* If individual channel configuration supported, to do during channel sequencing. */
	dev_data->ch_initialized |= (1 << channel_id);

	return 0;
}

static int adc_mchp_init(const struct device *dev)
{
	const struct adc_mchp_dev_config *const dev_cfg = dev->config;
	struct adc_mchp_dev_data *dev_data = dev->data;
	adc_registers_t *adc_reg = dev_cfg->regs;
	fuses_calotp_registers_t *fuses = dev_cfg->fuses;
	int ret = 0;
	uint32_t reg_val;

	dev_data->dev = dev;
	/* Switch on ADC gclock */
	ret = clock_control_on(dev_cfg->adc_clock.clock_dev, dev_cfg->adc_clock.gclk_sys);
	if ((ret != 0) && (ret != -EALREADY)) {
		LOG_ERR("Failed to enable the GCLK for ADC: %d", ret);
		return ret;
	}

	/* Switch on ADC mclock */
	ret = clock_control_on(dev_cfg->adc_clock.clock_dev, dev_cfg->adc_clock.mclk_sys);
	if ((ret != 0) && (ret != -EALREADY)) {
		LOG_ERR("Failed to enable the MCLK for ADC: %d", ret);
		return ret;
	}

	/* Get ADC Clock Frequency */
	ret = clock_control_get_rate(dev_cfg->adc_clock.clock_dev, dev_cfg->adc_clock.gclk_sys,
				     &dev_data->freq);
	if (ret != 0) {
		LOG_ERR("Failed to get the clock rate for ADC: %d", ret);
		return ret;
	}

	/* Configure pins */
	pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);

	/* Software reset */
	adc_reg->ADC_CTRLA |= ADC_CTRLA_SWRST_Msk;
	adc_wait_synchronization(adc_reg);

	/* Copy calibration value for all the enabled ADC cores */
	adc_reg->CONFIG[0].ADC_CALCTRL = fuses->FUSES_FCCFG65;

	/* Enables and powers up the ADC module */
	reg_val = adc_reg->ADC_CTRLA & ~(ADC_CTRLA_ONDEMAND_Msk);
	adc_reg->ADC_CTRLA = reg_val | ADC_CTRLA_ANAEN_Msk;

	/* Configure ctrl_clock_div and wkupexp */
	reg_val = adc_reg->ADC_CTRLD & ~(ADC_CTRLD_CTLCKDIV_Msk | ADC_CTRLD_WKUPEXP_Msk);
	reg_val |= (ADC_CTRLD_CTLCKDIV(dev_cfg->ctrl_clock_div) |
		    ADC_CTRLD_WKUPEXP(dev_cfg->adc_wkup_clock_count));
	adc_reg->ADC_CTRLD = reg_val;

	/* Configure adc_div_ratio and global sw trigger */
	reg_val = adc_reg->CONFIG[0].ADC_CORCTRL &
		  ~(ADC_CORCTRL_ADCDIV_Msk | ADC_CORCTRL_STRGSRC_Msk);
	reg_val |= ADC_CORCTRL_ADCDIV(dev_cfg->adc_div_ratio) |
		   ADC_CORCTRL_STRGSRC(ADC_CORCTRL_STRGSRC_GLOBAL_SOFTWARE_TRIGGER_Val);
	adc_reg->CONFIG[0].ADC_CORCTRL = reg_val;

	/* Calculate tad and store */
	dev_data->tad =
		ADC_CALC_TAD_NS(dev_data->freq, dev_cfg->ctrl_clock_div, dev_cfg->adc_div_ratio);

	/* Configure and enable IRQ */
	dev_cfg->config_func(dev);

	/* Analog and bias circuitry enable for the ADC SAR Core n (ANLEN) */
	adc_reg->ADC_CTRLD |= ADC_CTRLD_ANLEN_Msk;

	/* Enable the ADC Core n modules digital interface (CHNEN) */
	adc_reg->ADC_CTRLD |= ADC_CTRLD_CHNEN_Msk;

	reg_val = adc_reg->ADC_CTRLD & (~ADC_CTRLD_VREFSEL_Msk);
	adc_reg->ADC_CTRLD = reg_val | ADC_CTRLD_VREFSEL_AVDD_AVSS;

	/* Enable the ADC controller. */
	adc_reg->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;
	adc_wait_synchronization(adc_reg);

	/* Wait for WKUPEXP delay to expire after which ADC SAR Core n is Ready (CRDY) */
	if (WAIT_FOR(((adc_reg->ADC_CTLINTFLAG & (1 << ADC_CTLINTFLAG_CRRDY_Pos)) != 0),
		     ADC_WKUPEXP_DELAY, k_busy_wait(DELAY_US)) == false) {
		LOG_ERR("Timeout waiting for ADC_WKUPEXP_DELAY");
	}

	/* Disable the ADC controller. */
	adc_reg->ADC_CTRLA &= ~ADC_CTRLA_ENABLE_Msk;
	adc_wait_synchronization(adc_reg);

	/* Initialize ADC context */
	adc_context_unlock_unconditionally(&dev_data->ctx);

	return 0;
}

static DEVICE_API(adc, adc_mchp_api) = {
	.channel_setup = adc_mchp_channel_setup,
	.read = adc_mchp_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_mchp_read_async,
#endif
};

#define ADC_MCHP_DEFINE_CONFIG_FUNC(n)                                                             \
	static void adc_mchp_config_##n(const struct device *dev)                                  \
	{                                                                                          \
		/* Placeholder for IRQ and calibration configuration */                            \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, core0, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, core0, priority), adc_mchp_isr,                 \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, core0, irq));                                    \
		return;                                                                            \
	}

#define ADC_MCHP_DATA_DEFN(n)                                                                      \
	static struct adc_channel_cfg adc_channel_config_##n[DT_INST_PROP(n, num_channels)];       \
	static struct adc_mchp_dev_data adc_mchp_data_##n = {                                      \
		ADC_CONTEXT_INIT_TIMER(adc_mchp_data_##n, ctx),                                    \
		ADC_CONTEXT_INIT_LOCK(adc_mchp_data_##n, ctx),                                     \
		ADC_CONTEXT_INIT_SYNC(adc_mchp_data_##n, ctx),                                     \
		.saved_cfg = adc_channel_config_##n,                                               \
		.freq = 0,                                                                         \
	}

#define ADC_MCHP_CONFIG_DEFN(n)                                                                    \
	static void adc_mchp_config_##n(const struct device *dev);                                 \
	static struct adc_mchp_dev_config adc_mchp_cfg_##n = {                                     \
		.regs = (adc_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, adc),                       \
		.fuses = (fuses_calotp_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, fuses),           \
		.config_func = adc_mchp_config_##n,                                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.ctrl_clock_div = DT_INST_PROP(n, ctrl_clock_div),                                 \
		.adc_div_ratio = DT_INST_PROP(n, adc_div_ratio),                                   \
		.adc_wkup_clock_count = DT_INST_PROP(n, adc_wkup_clock_count),                     \
		.num_channels = DT_INST_PROP(n, num_channels),                                     \
		.adc_clock.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),                         \
		.adc_clock.mclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, subsystem)),   \
		.adc_clock.gclk_sys = (void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, subsystem))}

#define ADC_MCHP_DEVICE(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ADC_MCHP_CONFIG_DEFN(n);                                                                   \
	ADC_MCHP_DATA_DEFN(n);                                                                     \
	DEVICE_DT_INST_DEFINE(n, adc_mchp_init, NULL, &adc_mchp_data_##n, &adc_mchp_cfg_##n,       \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &adc_mchp_api);               \
	ADC_MCHP_DEFINE_CONFIG_FUNC(n);

DT_INST_FOREACH_STATUS_OKAY(ADC_MCHP_DEVICE)
