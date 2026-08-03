/*
 * Copyright (c) Arduino s.r.l. and/or its affiliated companies
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RP2040 PIO-based DMIC driver.
 *
 *   - PIO state machine generates PDM clock and samples bits into RX FIFO
 *   - DMA transfers raw PDM bits from PIO RX FIFO to a ping-pong buffer
 *   - DMA completion ISR (DMA_IRQ_0) runs the OpenPDM decimation filter
 *   - Resulting PCM blocks are posted to a k_msgq for dmic_read() callers
 */

#define DT_DRV_COMPAT raspberrypi_rp2040_pdm

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/irq.h>

#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>
#include <hardware/regs/intctrl.h> /* DMA_IRQ_0 = 11 */

#include "dmic_rp2040_pio.pio.h"
#include "OpenPDMFilter.h"

/* RAW_BUFFER_SIZE must be a multiple of (decimation / 8).
 * For decimation=128: multiple of 16; for decimation=64: multiple of 8.
 * 512 satisfies both. */
#define RAW_BUFFER_SIZE  512
#define DECIMATION       128
#define QUEUE_DEPTH      4

struct rp2040_pdm_config {
	uint8_t clk_pin;
	uint8_t din_pin;
	uint8_t pio_num;
	int     gain;
};

struct rp2040_pdm_data {
	/* PIO */
	PIO  pio;
	uint sm;
	uint pio_offset;

	/* DMA */
	int  dma_ch;

	/* Raw PDM ping-pong buffers */
	uint8_t raw_buf[2][RAW_BUFFER_SIZE];
	volatile int raw_buf_idx;

	/* OpenPDM filter */
	TPDMFilter_InitStruct filter;
	int cut_frames;  /* PCM blocks to mute at startup */

	/* DMIC API contract: caller provides these in configure() */
	struct k_mem_slab *mem_slab;
	uint32_t block_size;

	/* Queue of completed PCM blocks returned by dmic_read() */
	struct k_msgq rx_queue;
	void *rx_queue_buf[QUEUE_DEPTH];

	bool configured;
	bool running;
};

/* ── ISR ─────────────────────────────────────────────────────────────────── */

static void rp2040_pdm_dma_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct rp2040_pdm_data *data = dev->data;

	/* Acknowledge DMA_IRQ_0 for our channel */
	if (!(dma_hw->ints0 & (1u << data->dma_ch))) {
		return; /* Not our channel — spurious or shared IRQ */
	}
	dma_hw->ints0 = 1u << data->dma_ch;

	/* Restart DMA to the alternate buffer before processing.
	 * Must reload TRANS_COUNT explicitly — after completion it is 0. */
	int shadow = data->raw_buf_idx ^ 1;
	dma_channel_set_trans_count(data->dma_ch, RAW_BUFFER_SIZE, false);
	dma_channel_set_write_addr(data->dma_ch, data->raw_buf[shadow], true);

	/* Allocate a PCM block from the caller-supplied slab */
	void *pcm_buf;
	if (k_mem_slab_alloc(data->mem_slab, &pcm_buf, K_NO_WAIT) != 0) {
		/* Drop frame if no buffer available */
		data->raw_buf_idx = shadow;
		return;
	}

	/* Decimate raw PDM bits → PCM samples */
	int16_t *dst = (int16_t *)pcm_buf;
	if (data->filter.Decimation == 128) {
		Open_PDM_Filter_128(data->raw_buf[data->raw_buf_idx], dst, 1, &data->filter);
	} else {
		Open_PDM_Filter_64(data->raw_buf[data->raw_buf_idx], dst, 1, &data->filter);
	}

	/* Mute the first few frames (mic startup noise) */
	if (data->cut_frames > 0) {
		memset(pcm_buf, 0, data->block_size);
		data->cut_frames--;
	}

	data->raw_buf_idx = shadow;

	/* Post to queue; drop if full */
	if (k_msgq_put(&data->rx_queue, &pcm_buf, K_NO_WAIT) != 0) {
		k_mem_slab_free(data->mem_slab, pcm_buf);
	}
}

/* ── Driver ops ──────────────────────────────────────────────────────────── */

static int rp2040_pdm_configure(const struct device *dev, struct dmic_cfg *cfg)
{
	const struct rp2040_pdm_config *hw = dev->config;
	struct rp2040_pdm_data *data = dev->data;

	if (cfg->channel.req_num_streams < 1 ||
	    cfg->streams[0].pcm_width != 16) {
		return -EINVAL;
	}

	/* Release resources from a previous configure() */
	if (data->configured) {
		pio_sm_set_enabled(data->pio, data->sm, false);
		pio_sm_unclaim(data->pio, data->sm);
		pio_remove_program(data->pio, &pdm_pio_program, data->pio_offset);
		dma_channel_unclaim(data->dma_ch);
		data->configured = false;
	}

	uint32_t sample_rate = cfg->streams[0].pcm_rate;

	/* Store parameters for trigger/read */
	data->mem_slab  = cfg->streams[0].mem_slab;
	data->block_size = cfg->streams[0].block_size;

	/* Select decimation factor based on sample rate and clock constraints.
	 * PDM clock = sample_rate × decimation × 2.
	 * Mic accepts 1.2 – 3.25 MHz → prefer 128, fall back to 64. */
	int decimation = DECIMATION;
	if ((uint64_t)sample_rate * decimation * 2 > 3250000U) {
		decimation = 64;
	}

	int raw_buf_len = RAW_BUFFER_SIZE / (decimation / 8);
	int pcm_samples = raw_buf_len;
	uint32_t expected_block_size = (uint32_t)pcm_samples * sizeof(int16_t);

	/* The caller's block_size must match what the filter produces */
	if (data->block_size != expected_block_size) {
		data->block_size = expected_block_size;
	}

	/* Initialise OpenPDM filter (mbed-compatible defaults). */
	data->filter.Fs            = sample_rate;
	data->filter.MaxVolume     = 1;
	data->filter.nSamples      = pcm_samples;
	data->filter.LP_HZ         = sample_rate / 2;
	data->filter.HP_HZ         = 10;
	data->filter.In_MicChannels  = 1;
	data->filter.Out_MicChannels = 1;
	data->filter.Decimation    = decimation;
	data->filter.filterGain    = hw->gain;
	Open_PDM_Filter_Init(&data->filter);

	/* Select PIO block */
	data->pio = (hw->pio_num == 0) ? pio0 : pio1;

	/* Add PIO program */
	if (!pio_can_add_program(data->pio, &pdm_pio_program)) {
		return -ENODEV;
	}
	data->pio_offset = pio_add_program(data->pio, &pdm_pio_program);

	/* Claim a free state machine */
	int sm = pio_claim_unused_sm(data->pio, false);
	if (sm < 0) {
		pio_remove_program(data->pio, &pdm_pio_program, data->pio_offset);
		return -EBUSY;
	}
	data->sm = (uint)sm;

	/* Compute clock divider: sys_clk / (sample_rate × decimation × 2).
	 * Note: pico-sdk's clock_get_hz(clk_sys) returns 0 because Zephyr's
	 * clock_control driver bring up the PLLs but never populates pico-sdk's
	 * internal configured_freq[] table. Read the rate from devicetree. */
	const uint32_t sys_clk_hz =
		DT_PROP(DT_NODELABEL(clk_sys), clock_frequency);
	float clk_div = (float)sys_clk_hz /
	                (float)(sample_rate * (uint32_t)decimation * 2u);
	pdm_pio_program_init(data->pio, data->sm, data->pio_offset,
	                     hw->clk_pin, hw->din_pin, clk_div);

	/* Claim DMA channel */
	data->dma_ch = dma_claim_unused_channel(true);

	data->configured = true;
	return 0;
}

static int rp2040_pdm_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct rp2040_pdm_data *data = dev->data;

	if (!data->configured && cmd == DMIC_TRIGGER_START) {
		return -EIO;
	}

	switch (cmd) {
	case DMIC_TRIGGER_START: {
		if (data->running) {
			return 0;
		}

		/* Flush the queue */
		k_msgq_purge(&data->rx_queue);

		data->raw_buf_idx = 0;
		data->cut_frames = 5; /* mute first 5 PCM blocks at startup */

		/* Set up DMA: PIO RX FIFO → raw_buf[0], 8-bit transfers */
		dma_channel_config c = dma_channel_get_default_config(data->dma_ch);
		channel_config_set_read_increment(&c, false);
		channel_config_set_write_increment(&c, true);
		channel_config_set_dreq(&c, pio_get_dreq(data->pio, data->sm, false));
		channel_config_set_transfer_data_size(&c, DMA_SIZE_8);

		/* Enable DMA_IRQ_0 for our channel */
		dma_hw->ints0 = 1u << data->dma_ch; /* clear any pending */
		dma_channel_set_irq0_enabled(data->dma_ch, true);

		dma_channel_configure(data->dma_ch, &c,
		                      data->raw_buf[0],   /* destination */
		                      &data->pio->rxf[data->sm], /* source */
		                      RAW_BUFFER_SIZE,
		                      true);              /* start immediately */

		irq_enable(DMA_IRQ_0);

		/* Small delay for mic to stabilise */
		k_sleep(K_MSEC(100));

		data->running = true;
		break;
	}

	case DMIC_TRIGGER_STOP:
		if (!data->running) {
			return 0;
		}
		irq_disable(DMA_IRQ_0);
		dma_channel_set_irq0_enabled(data->dma_ch, false);
		dma_channel_abort(data->dma_ch);
		pio_sm_set_enabled(data->pio, data->sm, false);
		data->running = false;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int rp2040_pdm_read(const struct device *dev, uint8_t stream,
                           void **buf, size_t *size, int32_t timeout)
{
	struct rp2040_pdm_data *data = dev->data;
	void *block = NULL;

	ARG_UNUSED(stream);

	/* SYS_FOREVER_MS = -1 must map to K_FOREVER, not K_MSEC(-1). */
	k_timeout_t t = (timeout == SYS_FOREVER_MS) ? K_FOREVER : K_MSEC(timeout);
	int ret = k_msgq_get(&data->rx_queue, &block, t);
	if (ret < 0) {
		return ret;
	}

	*buf  = block;
	*size = data->block_size;
	return 0;
}

static const struct _dmic_ops rp2040_pdm_ops = {
	.configure = rp2040_pdm_configure,
	.trigger   = rp2040_pdm_trigger,
	.read      = rp2040_pdm_read,
};

/* ── Driver instance init ─────────────────────────────────────────────────── */

static int rp2040_pdm_init(const struct device *dev)
{
	struct rp2040_pdm_data *data = dev->data;

	/* Initialise the PCM block message queue */
	k_msgq_init(&data->rx_queue, (char *)data->rx_queue_buf,
	            sizeof(void *), QUEUE_DEPTH);

	data->configured = false;
	data->running    = false;

	/* Connect DMA_IRQ_0 to our ISR.  The IRQ is left disabled here;
	 * rp2040_pdm_trigger(START) enables it. */
	IRQ_CONNECT(DMA_IRQ_0, 3,
	            rp2040_pdm_dma_isr, DEVICE_DT_INST_GET(0), 0);

	return 0;
}

/* ── DT instance macro ────────────────────────────────────────────────────── */

#define RP2040_PDM_INIT(inst)                                           \
	static struct rp2040_pdm_data rp2040_pdm_data_##inst;           \
	static const struct rp2040_pdm_config rp2040_pdm_cfg_##inst = { \
		.clk_pin = DT_INST_PROP(inst, clk_pin),                 \
		.din_pin = DT_INST_PROP(inst, din_pin),                 \
		.pio_num = DT_INST_PROP(inst, pio_num),                 \
		.gain    = DT_INST_PROP(inst, gain),                    \
	};                                                               \
	DEVICE_DT_INST_DEFINE(inst, rp2040_pdm_init, NULL,              \
	                      &rp2040_pdm_data_##inst,                   \
	                      &rp2040_pdm_cfg_##inst,                    \
	                      POST_KERNEL,                               \
	                      CONFIG_AUDIO_DMIC_INIT_PRIORITY,           \
	                      &rp2040_pdm_ops);

DT_INST_FOREACH_STATUS_OKAY(RP2040_PDM_INIT)
