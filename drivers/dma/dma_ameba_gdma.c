/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_gdma

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/dma/dma_ameba_gdma.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_ameba_gdma, CONFIG_DMA_LOG_LEVEL);

enum dma_reload_type {
	GDMA_Single = 0,
	GDMA_ReloadDst,
	GDMA_ReloadSrc,
	GDMA_ReloadSrcDst,
};

struct dma_ameba_channel {
	uint32_t block_size;
	uint32_t block_num;
	uint32_t block_id;
	uint32_t src_trans_width;
	uint32_t dst_trans_width;
	uint32_t src_addr;
	uint32_t dst_addr;
	dma_callback_t callback;
	void *user_data;
	struct GDMA_CH_LLI *link_node;
	uint8_t relead_type;
	uint8_t chnl_direction;
	bool busy;
};

struct dma_ameba_data {
	struct dma_context dma_ctx;
	/* define Bitmap variable for channel request */
	ATOMIC_DEFINE(channels_atomic, DT_INST_PROP(0, dma_channels));
	struct dma_ameba_channel *channel_status;
};

struct dma_ameba_config {
	uint32_t base;
	uint8_t channel_num;
	uint8_t instane_id;
	void (*config_irq)(const struct device *dev);
	struct device *src_dev;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

static inline int dma_ameba_data_size_convert(uint32_t data_size)
{
	int para = 0;

	switch (data_size) {
	case 1:
		para = TrWidthOneByte;
		break;
	case 2:
		para = TrWidthTwoBytes;
		break;
	case 4:
		para = TrWidthFourBytes;
		break;
	default:
		para = -EINVAL;
		LOG_ERR("data_size must be 1, 2, or 4 (size = %d)", data_size);
		break;
	}
	return para;
}

static inline int dma_ameba_burst_size_convert(uint32_t burst_size)
{
	int para = 0;

	switch (burst_size) {
	case 1:
		para = MsizeOne;
		break;
	case 4:
		para = MsizeFour;
		break;
	case 8:
		para = MsizeEight;
		break;
	case 16:
		para = MsizeSixteen;
		break;
	default:
		LOG_ERR("burst size must be 1, 4, 8, 16 (size = %d)", burst_size);
		para = -EINVAL;
		break;
	}
	return para;
}

static inline int dma_ameba_reload_type_get(struct dma_config *config_dma)
{
	enum dma_reload_type type = 0;

	if (config_dma->head_block->source_reload_en && config_dma->head_block->dest_reload_en) {
		type = GDMA_ReloadSrcDst;
	} else if (config_dma->head_block->source_reload_en) {
		type = GDMA_ReloadSrc;
	} else if (config_dma->head_block->dest_reload_en) {
		type = GDMA_ReloadDst;
	} else {
		type = GDMA_Single;
	}
	return type;
}

static inline void dma_ameba_cache_handle(const struct device *dev, uint32_t channel)
{
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;
	uint32_t cache_line_mask = sys_cache_data_line_size_get() - 1;
	uint32_t temp_chnl_dir = data->channel_status[channel].chnl_direction;
	uint32_t temp_block_size = data->channel_status[channel].block_size;
	uint32_t *temp_src_addr = (uint32_t *)(data->channel_status[channel].src_addr);
	uint32_t *temp_dst_addr = (uint32_t *)(data->channel_status[channel].dst_addr);

	if (((u32)temp_src_addr & cache_line_mask) || ((u32)temp_dst_addr & cache_line_mask)) {
		LOG_WRN("The transfer address and cache line must be aligned, src: %x; dst: %x; "
			"cache line size= %d\n",

			(u32)temp_src_addr, (u32)temp_dst_addr, cache_line_mask + 1);
	}
	/* If the source or destination is memory, data consistency needs to be ensured. */
	if (temp_chnl_dir == MEMORY_TO_MEMORY) {
		sys_cache_data_flush_range(temp_src_addr, temp_block_size);
		sys_cache_data_flush_and_invd_range(temp_dst_addr, temp_block_size);
	} else if (temp_chnl_dir == MEMORY_TO_PERIPHERAL) {
		sys_cache_data_flush_range(temp_src_addr, temp_block_size);
	} else if (temp_chnl_dir == PERIPHERAL_TO_MEMORY) {
		sys_cache_data_flush_and_invd_range(temp_dst_addr, temp_block_size);
	}
}

static void dma_ameba_isr_handler(const struct device *dev, uint32_t channel)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;
	uint32_t cache_line_mask = ~(sys_cache_data_line_size_get() - 1);
	int err = 0;
	uint32_t isr_type = 0;

	/* If multiple blocks perform reload transmission, the reload bit needs to be cleared
	 * when transmitting to the second block in reverse order.
	 */
	if (data->channel_status[channel].block_num == data->channel_status[channel].block_id + 2) {
		GDMA_ChCleanAutoReload(config->instane_id, channel, CLEAN_RELOAD_SRC_DST);
	}
	/* 1.DMA interrupt type.*/
	isr_type = GDMA_ClearINT(config->instane_id, channel);

	if (isr_type & BlockType) {
		LOG_DBG("dma block %d transfer complete.\n",
			data->channel_status[channel].block_id);
		data->channel_status[channel].block_id++;
	}
	/* transfer complete*/
	if (isr_type == TransferType) {
		data->channel_status[channel].busy = false;
	}
	/* transfer error(protocol error)*/
	if (isr_type == ErrType) {
		err = -EIO;
	}
	/* 2. If the cache is opened, we need to ensure that the cache data and memory data
	 * on the destination address are consistent.
	 */
	if (data->channel_status[channel].chnl_direction == MEMORY_TO_MEMORY ||
	    data->channel_status[channel].chnl_direction == PERIPHERAL_TO_MEMORY) {
		sys_cache_data_flush_range(
			(void *)(data->channel_status[channel].dst_addr & cache_line_mask),
			data->channel_status[channel].block_size);
	}
	/* 3. Execute user call back function.*/
	if (data->channel_status[channel].callback) {
		data->channel_status[channel].callback(dev, data->channel_status[channel].user_data,
						       channel, err);
	}
}

static int dma_ameba_configure(const struct device *dev, uint32_t channel,
			       struct dma_config *config_dma)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;
	GDMA_InitTypeDef dma_init_struct;

	int ret = 0;
	/*1. check parameters */
	if (!config_dma) {
		LOG_ERR("invalid dma config parameters\n");
		return -EINVAL;
	}

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}

	if (config_dma->channel_direction > PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction (%d) is not supported\n", config_dma->channel_direction);
		return -ENOTSUP;
	}

	if (config_dma->head_block->fifo_mode_control) {
		LOG_ERR("fifo_mode_control settings is not supported\n");
		return -ENOTSUP;
	}

	if (config_dma->channel_priority >= config->channel_num) {
		LOG_ERR("channel_priority must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}

	if (config_dma->source_chaining_en || config_dma->dest_chaining_en) {
		LOG_WRN("realtek block chain is linked-list mode\n");
	}
	/* 2. config channel */
	GDMA_StructInit(&dma_init_struct);
	dma_init_struct.GDMA_Index = config->instane_id;
	dma_init_struct.GDMA_ChNum = channel;
	dma_init_struct.GDMA_DIR = config_dma->channel_direction;
	/*2.1 configuration about direction  */
	if (config_dma->channel_direction == MEMORY_TO_PERIPHERAL) {
		/* Data transfer depends on destination peripheral request */
		if (config_dma->head_block->flow_control_mode) {
			dma_init_struct.GDMA_DIR = TTFCMemToPeri_PerCtrl;
		}
		/* config dest ip handshake interface */
		if (!config_dma->dest_handshake) {
			dma_init_struct.GDMA_DstHandshakeInterface = config_dma->dma_slot;
		} else {
			LOG_ERR("dma does not support software handshake\n");
			return -ENOTSUP;
		}
	} else if (config_dma->channel_direction == PERIPHERAL_TO_MEMORY) {
		/* Data transfer depends on whether the source peripheral data is ready */
		if (config_dma->head_block->flow_control_mode) {
			dma_init_struct.GDMA_DIR = TTFCPeriToMem_PerCtrl;
		}
		if (!config_dma->source_handshake) {
			/* config src ip handshake interface */
			dma_init_struct.GDMA_SrcHandshakeInterface = config_dma->dma_slot;
		} else {
			LOG_ERR("dma does not support software handshake\n");
			return -ENOTSUP;
		}
	}
	/* 2.2 config channel width/burst length/addr change mode/block size */
	dma_init_struct.GDMA_SrcAddr = config_dma->head_block->source_address;
	dma_init_struct.GDMA_DstAddr = config_dma->head_block->dest_address;
	if (config_dma->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT ||
	    config_dma->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("dma does not support addr decrement mode\n");
		return -ENOTSUP;
	}
	dma_init_struct.GDMA_SrcInc = config_dma->head_block->source_addr_adj;
	dma_init_struct.GDMA_DstInc = config_dma->head_block->dest_addr_adj;

	ret = dma_ameba_data_size_convert(config_dma->source_data_size);
	if (ret != -EINVAL) {
		dma_init_struct.GDMA_SrcDataWidth = ret;
	} else {
		return ret;
	}
	ret = dma_ameba_data_size_convert(config_dma->dest_data_size);
	if (ret != -EINVAL) {
		dma_init_struct.GDMA_DstDataWidth = ret;
	} else {
		return ret;
	}

	ret = dma_ameba_burst_size_convert(config_dma->source_burst_length);
	if (ret != -EINVAL) {
		dma_init_struct.GDMA_SrcMsize = ret;
	} else {
		return ret;
	}
	ret = dma_ameba_burst_size_convert(config_dma->dest_burst_length);
	if (ret != -EINVAL) {
		dma_init_struct.GDMA_DstMsize = ret;
	} else {
		return ret;
	}

	/* if data block size (or source addr, destination addr) is unaligned,
	 * fix data width and burst length
	 */
	if (((config_dma->head_block->block_size & 0x03) != 0) ||
	    (((config_dma->head_block->source_address) & 0x03) != 0) ||
	    (((config_dma->head_block->dest_address) & 0x03) != 0)) {

		LOG_WRN("Data buffer or address is unaligned, block size = %d, src_addr = 0x%x, "
			"dst_addr = 0x%x. Dma will auto-fix.\n",

			config_dma->head_block->block_size, config_dma->head_block->source_address,
			config_dma->head_block->dest_address);
		/* move (1 byte * burst size)each transfer */
		dma_init_struct.GDMA_SrcDataWidth = TrWidthOneByte;
		dma_init_struct.GDMA_DstDataWidth = TrWidthOneByte;
	}

	/* set single block size = data block size / src_data_width */
	dma_init_struct.GDMA_BlockSize =
		config_dma->head_block->block_size >> dma_init_struct.GDMA_SrcDataWidth;

	/* 2.3 Continuous block(Adjacent address spaces) mode*/
	if (config_dma->head_block->dest_reload_en || config_dma->head_block->source_reload_en) {
		dma_init_struct.GDMA_ReloadSrc = config_dma->head_block->source_reload_en;
		dma_init_struct.GDMA_ReloadSrc = config_dma->head_block->dest_reload_en;
	}

	/* 2.4 link list mode (Discontinuous address).*/
	if (config_dma->source_chaining_en || config_dma->dest_chaining_en) {
		if (config_dma->head_block->next_block && (config_dma->block_count <= 1)) {
			LOG_ERR("The block_count does not match the dma_block_cfg "
				"configuration.\n");
			return -EINVAL;
		}
#ifdef CONFIG_DMA_AMEBA_LLI
		BUILD_ASSERT(CONFIG_HEAP_MEM_POOL_SIZE > 0,
			     "CONFIG_HEAP_MEM_POOL_SIZE must be configured in the .conf file");
		struct dma_block_config *cur_block = config_dma->head_block;

		dma_init_struct.GDMA_LlpSrcEn = config_dma->source_chaining_en;
		dma_init_struct.GDMA_LlpDstEn = config_dma->dest_chaining_en;

		/** @note malloc link list node for initialization, to ponit to link-block, */
		data->channel_status[channel].link_node = (struct GDMA_CH_LLI *)k_malloc(
			config_dma->block_count * sizeof(struct GDMA_CH_LLI));

		for (int i = 0; ((i < config_dma->block_count) && cur_block); i++) {
			if (dma_init_struct.GDMA_LlpSrcEn) {
				data->channel_status[channel].link_node[i].LliEle.Sarx =
					cur_block->source_address;
			}

			if (dma_init_struct.GDMA_LlpDstEn) {
				data->channel_status[channel].link_node[i].LliEle.Darx =
					cur_block->dest_address;
			}
			/* linked list concatenation */

			/*if this is last block but cyclic*/
			if ((i == config_dma->block_count - 1) && (config_dma->cyclic == 1)) {
				data->channel_status[channel].link_node[i].pNextLli =
					&data->channel_status[channel].link_node[0];
				/*if this is last block but not cyclic*/
			} else if ((i == config_dma->block_count - 1) &&
				   (config_dma->cyclic == 0)) {
				data->channel_status[channel].link_node[i].pNextLli = NULL;
			} else {
				data->channel_status[channel].link_node[i].pNextLli =
					&data->channel_status[channel].link_node[i + 1];
			}

			data->channel_status[channel].link_node[i].BlockSize =
				cur_block->block_size >> dma_init_struct.GDMA_SrcDataWidth;

			cur_block = cur_block->next_block;
		}
#endif
	}

	/* 2.5 Interrupt type config*/
	dma_init_struct.GDMA_IsrType = (TransferType | ErrType);

	/* 3. Initialization. */
	GDMA_Init(config->instane_id, channel, &dma_init_struct);
	GDMA_SetChnlPriority(config->instane_id, channel, config_dma->channel_priority);
	if (data->channel_status[channel].link_node != NULL) {
		GDMA_SetLLP(config->instane_id, channel, config_dma->block_count,
			    data->channel_status[channel].link_node, config_dma->cyclic);
	}
	/* 4. recording channel status */
	data->channel_status[channel].block_num = config_dma->block_count;
	data->channel_status[channel].block_size = config_dma->head_block->block_size;
	data->channel_status[channel].chnl_direction = config_dma->channel_direction;
	data->channel_status[channel].src_trans_width = dma_init_struct.GDMA_SrcDataWidth;
	data->channel_status[channel].dst_trans_width = dma_init_struct.GDMA_DstDataWidth;
	data->channel_status[channel].dst_addr = config_dma->head_block->dest_address;
	data->channel_status[channel].src_addr = config_dma->head_block->source_address;
	data->channel_status[channel].callback = config_dma->dma_callback;
	data->channel_status[channel].user_data = config_dma->user_data;
	data->channel_status[channel].relead_type = dma_ameba_reload_type_get(config_dma);
	return 0;
}

static int dma_ameba_start(const struct device *dev, uint32_t channel)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}
	dma_ameba_cache_handle(dev, channel);
	/* channel enable */
	GDMA_Cmd(config->instane_id, channel, ENABLE);

	data->channel_status[channel].busy = true;

	return 0;
}

static int dma_ameba_stop(const struct device *dev, uint32_t channel)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}
	/* channel enable */
	GDMA_Cmd(config->instane_id, channel, DISABLE);

	data->channel_status[channel].busy = false;

#ifdef CONFIG_DMA_AMEBA_LLI
	/** @note free link list node */
	if (data->channel_status[channel].link_node) {
		k_free(data->channel_status[channel].link_node);
		data->channel_status[channel].link_node = NULL;
	}
#endif

	return 0;
}

static int dma_ameba_get_status(const struct device *dev, uint32_t channel,
				struct dma_status *status)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}

	status->busy = data->channel_status[channel].busy;
	/* In the actual test, only the total amount of data can be read
	 * if it is greater than 2048 Bytes.
	 */
	status->pending_length = data->channel_status[channel].block_size -
				 GDMA_GetBlkSize(config->instane_id, channel);
	status->dir = data->channel_status[channel].chnl_direction;

	return 0;
}

static int dma_ameba_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			    size_t size)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}
	/* update current channel's parameters. */
	data->channel_status[channel].src_addr = src;
	data->channel_status[channel].dst_addr = dst;
	data->channel_status[channel].block_size = size;

	dma_ameba_cache_handle(dev, channel);

	GDMA_Cmd(config->instane_id, channel, DISABLE);
	GDMA_SetSrcAddr(config->instane_id, channel, src);
	GDMA_SetDstAddr(config->instane_id, channel, dst);
	/* The size required by the gdma hardware must be aligned
	 * with the source transmission width.
	 */
	size = size >> data->channel_status[channel].src_trans_width;
	GDMA_SetBlkSize(config->instane_id, channel, (u32)size);

	return 0;
}

static int dma_ameba_suspend(const struct device *dev, uint32_t channel)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}

	if (data->channel_status[channel].busy == false) {
		LOG_ERR("suspend channel not busy");
		return -EINVAL;
	}

	GDMA_Suspend(config->instane_id, channel);

	data->channel_status[channel].busy = false;

	return 0;
}

static int dma_ameba_resume(const struct device *dev, uint32_t channel)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	if (channel >= config->channel_num) {
		LOG_ERR("channel id must be < (%d)\n", config->channel_num);
		return -EINVAL;
	}

	if (data->channel_status[channel].busy == true) {
		LOG_ERR("resume channel is busy");
		return -EINVAL;
	}

	GDMA_Resume(config->instane_id, channel);

	data->channel_status[channel].busy = true;

	return 0;
}

static bool dma_ameba_chan_filter(const struct device *dev, int channel, void *filter_param)
{
	uint32_t requested_channel;

	if (!filter_param) {
		return true;
	}

	requested_channel = *(uint32_t *)filter_param;

	if (channel == requested_channel) {
		return true;
	}

	return false;
}

static int dma_ameba_init(const struct device *dev)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	struct dma_ameba_data *data = (struct dma_ameba_data *)dev->data;

	int ret = 0;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	for (uint32_t i = 0; i < config->channel_num; i++) {
		data->channel_status[i].busy = false;
		data->channel_status[i].callback = NULL;
		data->channel_status[i].user_data = NULL;
		data->channel_status[i].block_size = 0;
		data->channel_status[i].block_num = 0;
		data->channel_status[i].block_id = 0;
		data->channel_status[i].src_trans_width = 0;
		data->channel_status[i].dst_trans_width = 0;
		data->channel_status[i].dst_addr = 0;
		data->channel_status[i].src_addr = 0;
		data->channel_status[i].chnl_direction = 0;
		data->channel_status[i].link_node = NULL;
		data->channel_status[i].relead_type = GDMA_Single;
	}

	data->dma_ctx.dma_channels = config->channel_num;
	data->dma_ctx.magic = DMA_MAGIC;
	data->dma_ctx.atomic = data->channels_atomic;
	config->config_irq(dev);

	return 0;
}

static const struct dma_driver_api dma_ameba_api = {
	.config = dma_ameba_configure,
	.start = dma_ameba_start,
	.stop = dma_ameba_stop,
	.suspend = dma_ameba_suspend,
	.resume = dma_ameba_resume,
	.get_status = dma_ameba_get_status,
	.reload = dma_ameba_reload,
	.chan_filter = dma_ameba_chan_filter,
};

/*
 * Macro to CONNECT and enable each irq (order is given by the 'listify')
 * chan: channel of the DMA instance (assuming one irq per channel)
 * dma : dma instance (one GDMA instance on ameba)
 */
#define DMA_AMEBA_IRQ_CONNECT_CHANNEL(chan, dma)                                                   \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(dma, chan, irq),                                    \
			    DT_INST_IRQ_BY_IDX(dma, chan, priority), dma_ameba_irq_##dma##_##chan, \
			    DEVICE_DT_INST_GET(dma), 0);                                           \
		irq_enable(DT_INST_IRQ_BY_IDX(dma, chan, irq));                                    \
	} while (0)

/*
 * Macro to configure the irq for each dma instance (n)
 * Loop to CONNECT and enable each irq for each channel
 * Expecting as many irq as property <dma_channels>
 */
#define DMA_AMEBA_IRQ_CONNECT(n)                                                                   \
	static void dma_ameba_config_irq_##n(const struct device *dev)                             \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		LISTIFY(DT_INST_PROP(n, dma_channels), \
				DMA_AMEBA_IRQ_CONNECT_CHANNEL, \
				(;), \
				n);         \
	}

/*
 * Macro to instantiate the irq handler (order is given by the 'listify')
 * chan: channel of the DMA instance (assuming one irq per channel)
 * dma : dma instance (one GDMA instance on ameba)
 */
#define DMA_AMEBA_DEFINE_IRQ_HANDLER(chan, dma)                                                    \
	static void dma_ameba_irq_##dma##_##chan(const struct device *dev)                         \
	{                                                                                          \
		dma_ameba_isr_handler(dev, chan);                                                  \
	}

#define DMA_AMEBA_INIT(n)                                                                          \
	BUILD_ASSERT(DT_INST_PROP(n, dma_channels) == DT_NUM_IRQS(DT_DRV_INST(n)),                 \
		     "Nb of Channels and IRQ mismatch");                                           \
                                                                                                   \
	LISTIFY(DT_INST_PROP(n, dma_channels), \
			DMA_AMEBA_DEFINE_IRQ_HANDLER, \
			(;), \
			n);                  \
                                                                                                   \
	DMA_AMEBA_IRQ_CONNECT(n);                                                                  \
                                                                                                   \
	static const struct dma_ameba_config dma_config_##n = {                                    \
		.instane_id = n,                                                                   \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.channel_num = DT_INST_PROP(n, dma_channels),                                      \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (void *)DT_INST_CLOCKS_CELL(n, idx),                               \
		.config_irq = dma_ameba_config_irq_##n,                                            \
	};                                                                                         \
	static struct dma_ameba_channel dma_ameba_##n##_channels[DT_INST_PROP(n, dma_channels)];   \
	static struct dma_ameba_data dma_data_##n = {                                              \
		.channel_status = dma_ameba_##n##_channels,                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &dma_ameba_init, NULL, &dma_data_##n, &dma_config_##n,            \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, &dma_ameba_api);

/* Init GDMA instane 0*/
#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)
DMA_AMEBA_INIT(0)
#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay) */
