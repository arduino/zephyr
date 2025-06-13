/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba SPI interface
 */

#define DT_DRV_COMPAT realtek_ameba_spi

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ameba_spi, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

struct spi_ameba_data {
	struct spi_context ctx;
	const struct device *dev;
	bool initialized;
	uint32_t datasize; /* real dfs */
	uint8_t fifo_diff; /* cannot be bigger than FIFO depth */
#ifdef CONFIG_SPI_AMEBA_DMA
	/* TODO */
#endif
};

struct spi_ameba_config {
	/* spi info */
	uint32_t reg;
	uint16_t clkid;

	/* rcc info */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;

	/* pinctrl info */
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_AMEBA_INTERRUPT
	void (*irq_configure)();
#endif
};

static inline bool spi_ameba_is_slave(struct spi_ameba_data *spi)
{
	return (IS_ENABLED(CONFIG_SPI_SLAVE) && spi_context_is_slave(&spi->ctx));
}

static bool spi_ameba_transfer_ongoing(struct spi_ameba_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static int spi_ameba_get_err(const struct spi_ameba_config *cfg)
{
	SPI_TypeDef *spi = (SPI_TypeDef *)cfg->reg;

	if (SSI_GetStatus(spi) & SPI_BIT_DCOL) {
		LOG_ERR("spi%p Data Collision Error status detected", spi);
		return -EIO;
	}

	return 0;
}

static int spi_ameba_frame_exchange(const struct device *dev)
{
	struct spi_ameba_data *data = dev->data;
	const struct spi_ameba_config *dev_config = dev->config;
	SPI_TypeDef *spi = (SPI_TypeDef *)dev_config->reg;
	struct spi_context *ctx = &data->ctx;
	uint32_t datalen = data->datasize;
	int dfs = ((datalen - 1) >> 3) + 1; /* frame bytes */
	uint16_t tx_frame = 0U, rx_frame = 0U;

	while (!SSI_Writeable(spi)) {
		/* NOP */
	}

	if (spi_context_tx_buf_on(ctx)) {
		if (datalen <= 8) {
			tx_frame = ctx->tx_buf ? *(uint8_t *)(data->ctx.tx_buf) : 0;
		} else if (datalen <= 16) {
			tx_frame = ctx->tx_buf ? *(uint16_t *)(data->ctx.tx_buf) : 0;
		} else { /* if (datalen <= 32)  */
			/* tx_frame = ctx->tx_buf ? *(uint32_t *)(data->ctx.tx_buf) : 0; */
			LOG_ERR("Data Frame Size is supported from 4 ~ 16 \r\n");
			return EINVAL;
		}
	}

	SSI_WriteData(spi, tx_frame);
	spi_context_update_tx(ctx, dfs, 1);

	while (!SSI_Readable(spi)) {
		/* NOP */
	}

	rx_frame = SSI_ReadData(spi);

	if (spi_context_rx_buf_on(ctx)) {
		if (datalen <= 8) {
			*(uint8_t *)data->ctx.rx_buf = rx_frame;
		} else if (datalen <= 16) {
			*(uint16_t *)data->ctx.rx_buf = rx_frame;
		} else { /* if (datalen <= 32) */
			/* (uint32_t *)data->ctx.rx_buf = rx_frame; */
			LOG_ERR("Data Frame Size is supported from 4 ~ 16 \r\n");
			return EINVAL;
		}
	}
	spi_context_update_rx(ctx, dfs, 1);

	return spi_ameba_get_err(dev_config);
}

#ifdef CONFIG_SPI_AMEBA_INTERRUPT
static void spi_ameba_complete(const struct device *dev, int status)
{
	struct spi_ameba_data *dev_data = dev->data;
	const struct spi_ameba_config *dev_config = dev->config;
	SPI_TypeDef *spi = (SPI_TypeDef *)dev_config->reg;

	uint32_t int_mask = SPI_GetINTConfig(spi);

	SSI_INTConfig(spi, int_mask, DISABLE);

#ifdef CONFIG_SPI_AMEBA_DMA
	/* TODO */
#endif

	spi_context_complete(&dev_data->ctx, dev, status);
}

#ifdef CONFIG_SPI_AMEBA_INTERRUPT
static void spi_ameba_receive_data(const struct device *dev)
{
	const struct spi_ameba_config *cfg = dev->config;
	struct spi_ameba_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	SPI_TypeDef *spi = (SPI_TypeDef *)cfg->reg;

	uint32_t rxlevel;
	int err = 0;
	uint32_t datalen = data->datasize;  /* data bit 4 ~ 16 */
	int dfs = ((datalen - 1) >> 3) + 1; /* data number 1, 2 bytes */

	volatile uint32_t readable = SSI_Readable(spi);

	while (readable) {
		rxlevel = SSI_GetRxCount(spi);

		while (rxlevel--) {
			if (spi_context_rx_buf_on(ctx)) {
				if (data->ctx.rx_buf != NULL) {
					if (datalen <= 8) {
						/* 8~4 bits mode */
						*(uint8_t *)data->ctx.rx_buf =
							(uint8_t)SSI_ReadData(spi);
					} else {
						/* 16~9 bits mode */
						*(uint16_t *)data->ctx.rx_buf =
							(uint16_t)SSI_ReadData(spi);
					}
				} else {
					/* for Master mode, doing TX also will got RX data,
					 * so drop the dummy data
					 */
					(void)SSI_ReadData(spi);
				}
				/* spi_context_update_rx(ctx, dfs, 1); */
			} else if (spi_context_rx_on(ctx)) {
				/* fix for case: rx half end: buf1 is null
				 * but len1 !=0, skip len1 and rx into buf2
				 */
				(void)SSI_ReadData(spi);
			}

			spi_context_update_rx(ctx, dfs, 1);
			data->fifo_diff--;

			if (!spi_context_rx_on(ctx)) {
				break;
			}
		}

		if (!spi_context_rx_on(ctx)) {
			break;
		}

		readable = SSI_Readable(spi);
	}
}

static void spi_ameba_send_data(const struct device *dev)
{
	const struct spi_ameba_config *cfg = dev->config;
	struct spi_ameba_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int err = 0;
	uint32_t txdata = 0U;
	uint32_t txmax;
	SPI_TypeDef *spi = (SPI_TypeDef *)cfg->reg;

	uint32_t datalen = data->datasize;  /* data bit 4 ~ 16 */
	int dfs = ((datalen - 1) >> 3) + 1; /* data number 1, 2 bytes */

	u32 writeable = SSI_Writeable(spi);

	if (spi_context_rx_on(ctx)) { /* fix for rx bigger than tx */
		txmax = SSI_TX_FIFO_DEPTH - SSI_GetTxCount(spi) - SSI_GetRxCount(spi);
		if ((int)txmax < 0) {
			txmax = 0U; /* if rx-fifo is full, hold off tx */
		}
	} else {
		txmax = SSI_TX_FIFO_DEPTH - SSI_GetTxCount(spi);
	}

	if (writeable) {
		/* Disable Tx FIFO Empty IRQ */
		SSI_INTConfig(spi, SPI_BIT_TXEIM, DISABLE);

		while (txmax) {
			if (spi_context_tx_buf_on(ctx)) {
				if (datalen <= 8) {
					/* 8~4 bits mode */
					if (data->ctx.tx_buf != NULL) {
						txdata = *((uint8_t *)(data->ctx.tx_buf));
					} else if (!spi_ameba_is_slave(data)) {
						/* For master mode: Push a dummy to TX FIFO for Read
						 */
						txdata = (uint8_t)0; /* Dummy byte */
					}
				} else {
					/* 16~9 bits mode */
					if (data->ctx.tx_buf != NULL) {
						txdata = *((uint16_t *)(data->ctx.tx_buf));
					} else if (!spi_ameba_is_slave(data)) {
						/* For master mode: Push a dummy to TX FIFO for Read
						 */
						txdata = (uint16_t)0; /* Dummy byte */
					}
				}
				/* spi_context_update_tx(ctx, dfs, 1); */
			} else if (spi_context_rx_on(ctx)) { /* rx bigger than tx */
				/* No need to push more than necessary */
				if ((int)(data->ctx.rx_len - data->fifo_diff) <= 0) {
					break;
				}
				txdata = 0U;

			} else if (spi_context_tx_on(&data->ctx)) {
				/* fix for txbuf is NULL but txlen != 0 */
				txdata = 0U;
			} else {
				/* Nothing to push anymore */
				break;
			}

			SSI_WriteData(spi, txdata);
			spi_context_update_tx(ctx, dfs, 1);
			data->fifo_diff++;

			txmax--;
		}

		/* Enable Tx FIFO Empty IRQ */
		SSI_INTConfig(spi, SPI_BIT_TXEIM, ENABLE);
	}
}
#endif

static void spi_ameba_isr(struct device *dev)
{
	const struct spi_ameba_config *cfg = dev->config;
	struct spi_ameba_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	SPI_TypeDef *spi = (SPI_TypeDef *)cfg->reg;
	int err = 0;

	uint32_t int_mask = SPI_GetINTConfig(spi);

	LOG_INF("[ISR] int_mask 0X%x \r\n", int_mask);

#if 0
	err = spi_ameba_get_err(cfg);
	if (err) {
		spi_ameba_complete(dev, err);
		return;
	}

	if (spi_ameba_transfer_ongoing(data)) {
		err = spi_ameba_frame_exchange(dev);
	}

	if (err || !spi_ameba_transfer_ongoing(data)) {
		spi_ameba_complete(dev, err);
	}
#else

	uint32_t int_status = SSI_GetIsr(spi);

	SSI_SetIsrClean(spi, int_status);

	if (int_status & (SPI_BIT_TXOIS | SPI_BIT_RXUIS | SPI_BIT_RXOIS | SPI_BIT_TXUIS)) {
		LOG_INF("[INT] InterruptStatus %x\n", int_status);
	}

	if (int_status & SPI_BIT_RXFIS) {
		LOG_INF("[ISR] RXFIS\r\n");
		spi_ameba_receive_data(dev);

		if (!spi_context_rx_on(ctx)) {
			SSI_INTConfig(spi, SPI_BIT_RXFIM, DISABLE);
		}
	}

	if (int_status & SPI_BIT_TXEIS) {
		LOG_INF("[ISR] TXEIS\r\n");
		spi_ameba_send_data(dev);

		if (!spi_context_tx_on(ctx)) {
			SSI_INTConfig(spi, SPI_BIT_TXEIM, DISABLE);
		}
	}

	if (!spi_ameba_transfer_ongoing(data)) {
		spi_ameba_complete(dev, err);
	}
#endif
}
#endif /* CONFIG_SPI_AMEBA_INTERRUPT */

static int spi_ameba_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_ameba_data *data = dev->data;
	const struct spi_ameba_config *config = dev->config;
	SPI_TypeDef *spi = (SPI_TypeDef *)config->reg;
	struct spi_context *ctx = &data->ctx;
	SSI_InitTypeDef spi_init_struct;
	uint32_t bus_freq;

#ifdef CONFIG_SPI_AMEBA_DMA
	int dma_datasize;
#endif

	/* LOG_INF("[spi_ameba_configure] gspi_cfg = freq=%d, opertion=0x%x, slave_num=%d \r\n",
	 * spi_cfg->frequency, spi_cfg->operation, spi_cfg->slave);
	 */
	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enables SPI peripheral */
	if (clock_control_on(config->clock_dev, config->clock_subsys)) {
		LOG_ERR("Could not enable SPI clock");
		return -EIO;
	}

	if (data->initialized && spi_context_configured(ctx, spi_cfg)) {
		LOG_INF("Already configured. No need to do it again \r\n");
		return 0;
	}

	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode is not supported on %s", dev->name);
		return -EINVAL;
	}

	if (spi_cfg->operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -EINVAL;
	}

	if (spi_cfg->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB mode is supported");
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only single line mode is supported");
		return -EINVAL;
	}

	if (data->initialized) {
		SSI_Cmd(spi, DISABLE);
		data->initialized = false;
	}

	/* init spi */
	SSI_StructInit(&spi_init_struct);

	/* ameba config spi role */
	/* if (IS_ENABLED(CONFIG_SPI_SLAVE) && spi_context_is_slave(ctx)) {*/
	if (spi_ameba_is_slave(data)) {
		SSI_SetRole(spi, SSI_SLAVE);
		spi_init_struct.SPI_Role = SSI_SLAVE;
		LOG_INF("SPI ROLE: SSI_SLAVE \r\n");
	} else {
		SSI_SetRole(spi, SSI_MASTER);
		spi_init_struct.SPI_Role = SSI_MASTER;
		LOG_INF(" SPI ROLE: SSI_MASTER \r\n");
	}

#ifdef CONFIG_SPI_AMEBA_DMA
	/* TODO */
#endif

	SSI_Init(spi, &spi_init_struct);

#ifdef CONFIG_SPI_AMEBA_DMA
	/* TODO */
#endif

	/* set format */
	SSI_SetSclkPhase(spi, (((spi_cfg->operation) & SPI_MODE_CPHA) ? SCPH_TOGGLES_AT_START
								      : SCPH_TOGGLES_IN_MIDDLE));
	SSI_SetSclkPolarity(spi, (((spi_cfg->operation) & SPI_MODE_CPOL) ? SCPOL_INACTIVE_IS_HIGH
									 : SCPOL_INACTIVE_IS_LOW));
	SSI_SetDataFrameSize(spi, (SPI_WORD_SIZE_GET(spi_cfg->operation) - 1)); /* DataFrameSize */

	/* set frequency */
	bus_freq = 100000000;
	SSI_SetBaudDiv(spi, bus_freq / spi_cfg->frequency);

	/* DiagPrintf("spi_ameba_configure line%d cfgfreq %d div %d \r\n", __LINE__,
	 * spi_cfg->frequency, clk_div);
	 */

	data->datasize = SPI_WORD_SIZE_GET(spi_cfg->operation);
	data->initialized = true;

	ctx->config = spi_cfg;

	return 0;
}

static int spi_ameba_transceive_impl(const struct device *dev, const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, bool asynchronous,
				     spi_callback_t cb, void *userdata)
{
	struct spi_ameba_data *data = dev->data;
	const struct spi_ameba_config *config = dev->config;
	SPI_TypeDef *spi = (SPI_TypeDef *)config->reg;
	int ret;

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);
	ret = spi_ameba_configure(dev, spi_cfg);
	if (ret < 0) {
		goto error;
	}

	SSI_Cmd(spi, ENABLE);
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, ((data->datasize - 1) >> 3) + 1);
	data->fifo_diff = 0U;

	spi_context_cs_control(&data->ctx, true);

#ifdef CONFIG_SPI_AMEBA_INTERRUPT
#ifdef CONFIG_SPI_AMEBA_DMA
	if (data->dma_rx.dma_dev && data->dma_tx.dma_dev) {
		/* TODO */
	} else
#endif
	{
		uint32_t int_mask; /* fix warning */

		if (tx_bufs) { /* && tx_bufs->buffers*/
			int_mask = SPI_BIT_TXEIM;
		}

		if (rx_bufs) { /* && rx_bufs->buffers*/
			int_mask |= SPI_BIT_RXFIM;
		}

		DiagPrintf("Set int_mask 0x%x \r\n", int_mask);
		SSI_INTConfig(spi, SPI_BIT_TXEIM | SPI_BIT_RXFIM, ENABLE);
	}

	/* slave take sema forever, master take sema by timeout */
	ret = spi_context_wait_for_completion(&data->ctx);
#else

	do {
		ret = spi_ameba_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_ameba_transfer_ongoing(data));

#ifdef CONFIG_SPI_ASYNC
	spi_context_complete(&data->ctx, dev, ret);
#endif
#endif

	while ((!(SSI_GetStatus(spi) & SPI_BIT_TFE)) || (SSI_GetStatus(spi) & SPI_BIT_BUSY)) {
		/* Wait until last frame transfer complete. */
	}

#ifdef CONFIG_SPI_AMEBA_DMA
dma_error:
#endif
	spi_context_cs_control(&data->ctx, false);
	SSI_Cmd(spi, DISABLE);

error:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_ameba_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_ameba_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_ameba_init(const struct device *dev)
{
	struct spi_ameba_data *data = dev->data;
	const struct spi_ameba_config *cfg = dev->config;
	int ret;

	if (!cfg->clock_dev) {
		return -EINVAL;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

#ifdef CONFIG_SPI_AMEBA_DMA
	/* TODO */
#endif

	ret = spi_context_cs_configure_all(&data->ctx);

	if (ret < 0) {
		LOG_ERR("Failed to spi_context_cs_configure_all");
		return ret;
	}

#ifdef CONFIG_SPI_AMEBA_INTERRUPT
	cfg->irq_configure(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_ameba_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return spi_ameba_transceive_impl(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_ameba_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				      void *userdata)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static const struct spi_driver_api ameba_spi_api = {.transceive = spi_ameba_transceive,
#ifdef CONFIG_SPI_ASYNC
						    .transceive_async = spi_ameba_transceive_async,
#endif
						    .release = spi_ameba_release};

#define AMEBA_SPI_IRQ_CONFIGURE(n)                                                                 \
	static void spi_ameba_irq_configure_##n(void)                                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_ameba_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define SPI_DMA_CHANNEL(n, dir)

#define AMEBA_SPI_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	IF_ENABLED(CONFIG_SPI_AMEBA_INTERRUPT, \
				(AMEBA_SPI_IRQ_CONFIGURE(n)));                       \
                                                                                                   \
	static struct spi_ameba_data spi_ameba_data_##n = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_ameba_data_##n, ctx),                                    \
		SPI_CONTEXT_INIT_SYNC(spi_ameba_data_##n, ctx),                                    \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx).dev = DEVICE_DT_INST_GET(n),  \
		.initialized = false, SPI_DMA_CHANNEL(n, rx) SPI_DMA_CHANNEL(n, tx)};              \
                                                                                                   \
	static struct spi_ameba_config spi_ameba_config_##n = {                                    \
		.reg = DT_INST_REG_ADDR(n),                                                        \
		.clkid = DT_INST_CLOCKS_CELL(n, idx),                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),               \
		IF_ENABLED(CONFIG_SPI_AMEBA_INTERRUPT, \
			(.irq_configure = spi_ameba_irq_configure_##n)) };                        \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &spi_ameba_init, NULL, &spi_ameba_data_##n,                       \
			      &spi_ameba_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,        \
			      &ameba_spi_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_SPI_INIT)
