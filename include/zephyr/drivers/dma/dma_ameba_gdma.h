/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_AMEBA_GDMA_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_AMEBA_GDMA_H_

#define AMEBA_DT_INST_DMA_CTLR(n, name)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                                                \
		    (DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, name))), (NULL))

#define AMEBA_DT_INST_DMA_CELL(n, name, cell)                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas), (DT_INST_DMAS_CELL_BY_NAME(n, name, cell)),    \
		    (0xff))

/* macros for getting channel-configuration */

/**
 * @brief direction defined on bits 0-1
 *
 * @param value, 0 -> MEM_TO_MEM, 1 -> MEM_TO_PERIPH, 2 -> PERIPH_TO_MEM, 3 ->PERIPH_TO_PERIPH
 */

#define AMEBA_DMA_CH_DIR_GET(value) ((value >> 0) & 0x3)

/**
 * @brief source increment defined on bit 2-3
 *
 * @param value, 0 -> increment, 1 -> decrement(Not support), 2 -> no change
 */
#define AMEBA_DMA_SRC_ADDR_ADJ_GET(value) ((value >> 2) & 0x3)

/**
 * @brief destination increment defined on bit 4-5
 *
 * @param value, 0 -> increment, 1 -> decrement(Not support), 2 -> no change
 */
#define AMEBA_DMA_DST_ADDR_ADJ_GET(value) ((value >> 4) & 0x3)

/**
 * @brief source data size defined on bits 6-8
 *
 * @param value, data size(width) can be 0x1 byte/0x2 byte/0x4 byte
 */
#define AMEBA_DMA_SRC_DATA_SIZE_GET(value) ((value >> 6) & 0x7)

/**
 * @brief destination data size defined on bits 9-11
 *
 * @param value, data size(width) can be 0x1 byte/0x2 byte/0x4 byte
 */
#define AMEBA_DMA_DST_DATA_SIZE_GET(value) ((value >> 9) & 0x7)

/**
 * @brief source msize defined on bits 12-16
 *
 * @param value, msize(burst size) can be 0x1, 0x4, 0x8, 0x10
 */

#define AMEBA_DMA_SRC_BST_SIZE_GET(value) ((value >> 12) & 0x1F)
/**
 * @brief destination msize defined on bits 17-21
 *
 * @param value, msize(burst size) can be 0x1, 0x4, 0x8, 0x10
 */
#define AMEBA_DMA_DST_BST_SIZE_GET(value) ((value >> 17) & 0x1F)

/**
 * @brief priority defined on bits 22-24 as 0-7
 *
 * @param value, priority can be 0-7 if set. Otherwise, ch-0's priority is highest.
 */
#define AMEBA_DMA_CH_PRIORITY_GET(value) ((value >> 22) & 0x7)

#define AMEBA_DMA_CONFIG(index, dir, block_number, cb)                                             \
	{                                                                                          \
		.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),                           \
		.channel_direction =                                                               \
			AMEBA_DMA_CH_DIR_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),          \
		.channel_priority =                                                                \
			AMEBA_DMA_CH_PRIORITY_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),     \
		.source_data_size =                                                                \
			AMEBA_DMA_SRC_DATA_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),   \
		.dest_data_size =                                                                  \
			AMEBA_DMA_DST_DATA_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),   \
		.source_burst_length =                                                             \
			AMEBA_DMA_SRC_BST_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),    \
		.dest_burst_length =                                                               \
			AMEBA_DMA_DST_BST_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),    \
		.block_count = block_number,                                                       \
		.dma_callback = cb,                                                                \
	}

#endif
