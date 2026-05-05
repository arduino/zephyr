/*
 * Copyright (c) 2022 Vestas Wind Systems A/S
 * Copyright (c) 2021 Alexander Wachter
 * Copyright (c) 2022 Kamil Serwus
 * Copyright (c) 2023 Sebastian Schlupp
 * Copyright (c) 2024 Gerson Fernando Budke <nandojve@gmail.com>
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Microchip M_CAN CAN driver
 *
 * Unified CAN driver for all Microchip MCUs using the Bosch M_CAN IP core
 * (ISO 11898-1:2015, CAN 2.0 part A/B, CAN FD).
 *
 * Supported families:
 * - PIC32CK SG/GC series
 */

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/can_mcan.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <soc.h>

LOG_MODULE_REGISTER(can_mchp_mcan, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT microchip_mcan_g1

/*
 * Get SRAM details from Devicetree
 */
#define SRAM_NODE			DT_CHOSEN(zephyr_sram)
#define MRAM_BASE_ADDR		DT_REG_ADDR(SRAM_NODE)
#define MRAM_SIZE			DT_REG_SIZE(SRAM_NODE)
#define MRAM_ADDR_MAX		(MRAM_BASE_ADDR + MRAM_SIZE - 1)

struct can_mchp_config {
	mm_reg_t base;
	mem_addr_t mram;
	void (*config_irq)(void);
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	uint32_t mclk_id;
	uint32_t gclk_id;
};

static int can_mchp_read_reg(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *cfg = mcan_config->custom;

	return can_mcan_sys_read_reg(cfg->base, reg, val);
}

static int can_mchp_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *cfg = mcan_config->custom;

	switch (reg) {
	case CAN_MCAN_ILS:
		/* All interrupts are assigned to MCAN_INT0 */
		val = 0;
		break;
	case CAN_MCAN_ILE:
		/* Microchip devices have only one interrupt line */
		val = CAN_MCAN_ILE_EINT0;
		break;
	default:
		/* No field remap needed */
		break;
	}

	return can_mcan_sys_write_reg(cfg->base, reg, val);
}

static int can_mchp_read_mram(const struct device *dev, uint16_t offset,
							  void *dst, size_t len)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *cfg = mcan_config->custom;

	/* Ensure M_CAN writes are visible to CPU before reading */
	barrier_dsync_fence_full();

	return can_mcan_sys_read_mram(cfg->mram, offset, dst, len);
}

static int can_mchp_write_mram(const struct device *dev, uint16_t offset,
							   const void *src, size_t len)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *cfg = mcan_config->custom;
	int ret;

	ret = can_mcan_sys_write_mram(cfg->mram, offset, src, len);

	/* Ensure CPU writes are visible to M_CAN before it accesses MRAM */
	barrier_dsync_fence_full();

	return ret;
}

static int can_mchp_clear_mram(const struct device *dev, uint16_t offset,
							   size_t len)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *cfg = mcan_config->custom;
	int ret;

	ret = can_mcan_sys_clear_mram(cfg->mram, offset, len);

	/* Ensure CPU writes are visible to M_CAN */
	barrier_dsync_fence_full();

	return ret;
}

static void can_mchp_line_x_isr(const struct device *dev)
{
	can_mcan_line_0_isr(dev);
	can_mcan_line_1_isr(dev);
}

static int can_mchp_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_mcan_config *mcan_cfg = dev->config;
	const struct can_mchp_config *cfg = mcan_cfg->custom;

	return clock_control_get_rate(cfg->clock_dev,
			(clock_control_subsys_t)(uintptr_t)cfg->gclk_id, rate);
}

static int can_mchp_clock_enable(const struct can_mchp_config *cfg)
{
	int ret;

	/* Enable MCLK (AHB bus clock) first - required before GCLK */
	ret = clock_control_on(cfg->clock_dev,
				(clock_control_subsys_t)(uintptr_t)cfg->mclk_id);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to enable MCLK: %d", ret);
		return ret;
	}

	/* Enable GCLK (peripheral clock) */
	ret = clock_control_on(cfg->clock_dev,
				(clock_control_subsys_t)(uintptr_t)cfg->gclk_id);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to enable GCLK: %d", ret);
		return ret;
	}

	return 0;
}

static int can_mchp_init(const struct device *dev)
{
	const struct can_mcan_config *mcan_cfg = dev->config;
	const struct can_mchp_config *cfg = mcan_cfg->custom;
	int ret;

	/* Validate MRAM address is in the addressable range */
	if (cfg->mram < MRAM_BASE_ADDR || cfg->mram > MRAM_ADDR_MAX) {
		LOG_ERR("MRAM address 0x%08lx outside SRAM range (0x%08x - 0x%08x)",
				(unsigned long)cfg->mram, MRAM_BASE_ADDR, MRAM_ADDR_MAX);
		return -EINVAL;
	}

	/* Enable clocks */
	ret = can_mchp_clock_enable(cfg);
	if (ret < 0) {
		return ret;
	}

	/* Apply pinctrl if configured */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl: %d", ret);
			return ret;
		}
	}

	/* Configure Message RAM. */
	ret = can_mcan_configure_mram(dev, MRAM_BASE_ADDR, cfg->mram);
	if (ret != 0) {
		LOG_ERR("Failed to configure message RAM: %d", ret);
		return ret;
	}

	/* Initialize MCAN core */
	ret = can_mcan_init(dev);
	if (ret != 0) {
		LOG_ERR("Failed to initialize MCAN: %d", ret);
		return ret;
	}

	/* Configure and enable interrupts */
	cfg->config_irq();

	return 0;
}

static DEVICE_API(can, can_mchp_driver_api) = {
	.get_capabilities = can_mcan_get_capabilities,
	.start = can_mcan_start,
	.stop = can_mcan_stop,
	.set_mode = can_mcan_set_mode,
	.set_timing = can_mcan_set_timing,
	.send = can_mcan_send,
	.add_rx_filter = can_mcan_add_rx_filter,
	.remove_rx_filter = can_mcan_remove_rx_filter,
	.get_state = can_mcan_get_state,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_mcan_recover,
#endif
	.get_core_clock = can_mchp_get_core_clock,
	.get_max_filters = can_mcan_get_max_filters,
	.set_state_change_callback = can_mcan_set_state_change_callback,
	.timing_min = CAN_MCAN_TIMING_MIN_INITIALIZER,
	.timing_max = CAN_MCAN_TIMING_MAX_INITIALIZER,
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_mcan_set_timing_data,
	.timing_data_min = CAN_MCAN_TIMING_DATA_MIN_INITIALIZER,
	.timing_data_max = CAN_MCAN_TIMING_DATA_MAX_INITIALIZER,
#endif
};

static const struct can_mcan_ops can_mchp_ops = {
	.read_reg = can_mchp_read_reg,
	.write_reg = can_mchp_write_reg,
	.read_mram = can_mchp_read_mram,
	.write_mram = can_mchp_write_mram,
	.clear_mram = can_mchp_clear_mram,
};

#define CAN_MCHP_IRQ_CFG_FUNCTION(inst)						\
static void config_can_mchp_##inst##_irq(void)					\
{										\
	LOG_DBG("Enable CAN%d IRQ", inst);					\
	IRQ_CONNECT(DT_INST_IRQN(inst),						\
			DT_INST_IRQ(inst, priority),				\
			can_mchp_line_x_isr,					\
			DEVICE_DT_INST_GET(inst), 0);				\
	irq_enable(DT_INST_IRQN(inst));						\
}

#define CAN_MCHP_PINCTRL_DEFINE(inst)						\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),			\
		   (PINCTRL_DT_INST_DEFINE(inst);))

#define CAN_MCHP_PINCTRL_GET(inst)						\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),			\
		(PINCTRL_DT_INST_DEV_CONFIG_GET(inst)))

#define CAN_MCHP_CFG_INST(inst)							\
	CAN_MCAN_DT_INST_CALLBACKS_DEFINE(inst, can_mchp_cbs_##inst);		\
	CAN_MCAN_DT_INST_MRAM_DEFINE(inst, can_mchp_mram_##inst);		\
										\
	static const struct can_mchp_config can_mchp_cfg_##inst = {		\
		.base = CAN_MCAN_DT_INST_MCAN_ADDR(inst),			\
		.mram = (mem_addr_t)POINTER_TO_UINT(&can_mchp_mram_##inst),	\
		.clock_dev = DEVICE_DT_GET(DT_NODELABEL(clock)),		\
		.mclk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, subsystem),	\
		.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, subsystem),	\
		.pcfg = CAN_MCHP_PINCTRL_GET(inst),				\
		.config_irq = config_can_mchp_##inst##_irq,			\
	};									\
										\
	static const struct can_mcan_config can_mcan_cfg_##inst =		\
		CAN_MCAN_DT_CONFIG_INST_GET(inst, &can_mchp_cfg_##inst,		\
						&can_mchp_ops,			\
						&can_mchp_cbs_##inst);

#define CAN_MCHP_DATA_INST(inst)						\
	static struct can_mcan_data can_mcan_data_##inst =			\
		CAN_MCAN_DATA_INITIALIZER(NULL);

#define CAN_MCHP_DEVICE_INST(inst)						\
	CAN_DEVICE_DT_INST_DEFINE(inst, can_mchp_init, NULL,			\
				&can_mcan_data_##inst,			\
				&can_mcan_cfg_##inst,				\
				POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,	\
				&can_mchp_driver_api);

#define CAN_MCHP_INST(inst)							\
	CAN_MCAN_DT_INST_BUILD_ASSERT_MRAM_CFG(inst);				\
	CAN_MCHP_PINCTRL_DEFINE(inst)						\
	CAN_MCHP_IRQ_CFG_FUNCTION(inst)						\
	CAN_MCHP_CFG_INST(inst)							\
	CAN_MCHP_DATA_INST(inst)						\
	CAN_MCHP_DEVICE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_MCHP_INST)
