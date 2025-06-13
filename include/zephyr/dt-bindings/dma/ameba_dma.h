/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_AMEBA_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_AMEBA_DMA_H_

/**
 * @name custom DMA flags for channel configuration
 * @{
 */
/** DMA  transfer direction config on bits 0-1 */
#define AMEBA_DMA_CH_DIR_SET(val)  ((val & 0x3) << 0)
#define AMEBA_DMA_MEMORY_TO_MEMORY AMEBA_DMA_CH_DIR_SET(0)
#define AMEBA_DMA_MEMORY_TO_PERIPH AMEBA_DMA_CH_DIR_SET(1)
#define AMEBA_DMA_PERIPH_TO_MEMORY AMEBA_DMA_CH_DIR_SET(2)
#define AMEBA_DMA_PERIPH_TO_PERIPH AMEBA_DMA_CH_DIR_SET(3)

/** DMA  Soure Mem/Periph increment Address config on bit 2-3 */
#define AMEBA_DMA_SRC_ADDR_ADJ_SET(val)  ((val & 0x3) << 2)
#define AMEBA_DMA_SRC_ADDR_ADJ_INC       AMEBA_DMA_SRC_ADDR_ADJ_SET(0)
#define AMEBA_DMA_SRC_ADDR_ADJ_NO_CHANGE AMEBA_DMA_SRC_ADDR_ADJ_SET(2)

/** DMA  Destination Mem/Periph increment Address config on bit 4-5 */
#define AMEBA_DMA_DST_ADDR_ADJ_SET(val)  ((val & 0x3) << 4)
#define AMEBA_DMA_DST_ADDR_ADJ_INC       AMEBA_DMA_DST_ADDR_ADJ_SET(0)
#define AMEBA_DMA_DST_ADDR_ADJ_NO_CHANGE AMEBA_DMA_DST_ADDR_ADJ_SET(2)

/** DMA  Source data size config on bits 6-8 */
#define AMEBA_DMA_SRC_DATA_SIZE_SET(val) ((val & 0x7) << 6)
#define AMEBA_DMA_SRC_WIDTH_8BITS        AMEBA_DMA_SRC_DATA_SIZE_SET(0)
#define AMEBA_DMA_SRC_WIDTH_16BITS       AMEBA_DMA_SRC_DATA_SIZE_SET(1)
#define AMEBA_DMA_SRC_WIDTH_32BITS       AMEBA_DMA_SRC_DATA_SIZE_SET(2)

/** DMA  Destination data size config on bits 9-11 */
#define AMEBA_DMA_DST_DATA_SIZE_SET(val) ((val & 0x7) << 9)
#define AMEBA_DMA_DST_WIDTH_8BITS        AMEBA_DMA_DST_DATA_SIZE_SET(0)
#define AMEBA_DMA_DST_WIDTH_16BITS       AMEBA_DMA_DST_DATA_SIZE_SET(1)
#define AMEBA_DMA_DST_WIDTH_32BITS       AMEBA_DMA_DST_DATA_SIZE_SET(2)

/** DMA  Source burst size config on bits 12-16 */
#define AMEBA_DMA_SRC_BURST_SET(val) ((val & 0x1F) << 12)
#define AMEBA_DMA_SRC_BURST_ONE      AMEBA_DMA_SRC_BURST_SET(0)
#define AMEBA_DMA_SRC_BURST_FOUR     AMEBA_DMA_SRC_BURST_SET(1)
#define AMEBA_DMA_SRC_BURST_EIGHT    AMEBA_DMA_SRC_BURST_SET(2)
#define AMEBA_DMA_SRC_BURST_SIXTEEN  AMEBA_DMA_SRC_BURST_SET(3)
/** DMA  Destination burst size config on bits 17-21 */
#define AMEBA_DMA_DST_BURST_SET(val) ((val & 0x1F) << 17)
#define AMEBA_DMA_DST_BURST_ONE      AMEBA_DMA_DST_BURST_SET(0)
#define AMEBA_DMA_DST_BURST_FOUR     AMEBA_DMA_DST_BURST_SET(1)
#define AMEBA_DMA_DST_BURST_EIGHT    AMEBA_DMA_DST_BURST_SET(2)
#define AMEBA_DMA_DST_BURST_SIXTEEN  AMEBA_DMA_DST_BURST_SET(3)

/** DMA  Channel priority config on bit 22-24, and value can be 0~7 */
#define AMEBA_DMA_CH_PRIORITY_SET(val) ((val & 0x7) << 22)

/* DMA  usual combination for peripheral transfer */
#define AMEBA_DMA_PERIPH_TX                                                                        \
	(AMEBA_DMA_MEMORY_TO_PERIPH | AMEBA_DMA_SRC_ADDR_ADJ_INC | AMEBA_DMA_DST_ADDR_ADJ_NO_CHANGE)

#define AMEBA_DMA_PERIPH_RX                                                                        \
	(AMEBA_DMA_PERIPH_TO_MEMORY | AMEBA_DMA_SRC_ADDR_ADJ_NO_CHANGE | AMEBA_DMA_DST_ADDR_ADJ_INC)

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_AMEBA_DMA_H_ */
