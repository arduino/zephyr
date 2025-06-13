/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_watchdog

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_ameba, CONFIG_WDT_LOG_LEVEL);

struct wdt_ameba_data {
	uint32_t timeout;
	uint32_t window;
	wdt_callback_t callback;
};

struct wdt_ameba_config {
	WDG_TypeDef *WDG;
	int irq_source;
	void (*irq_config_func)(const struct device *dev);
	uint32_t eicnt;
};

static void wdt_ameba_isr(const struct device *dev);

static int wdt_ameba_disable(const struct device *dev)
{
	const struct wdt_ameba_config *config = dev->config;

	WDG_Cmd(config->WDG, DISABLE);

	return 0;
}

static int wdt_ameba_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(channel_id);
	const struct wdt_ameba_config *config = dev->config;

	WDG_Refresh(config->WDG);

	return 0;
}

static int wdt_ameba_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_ameba_config *config = dev->config;
	struct wdt_ameba_data *data = dev->data;
	WDG_InitTypeDef WDG_initstruct;

	if (options == WDT_OPT_PAUSE_HALTED_BY_DBG) {
		LOG_ERR("pausing watchdog by debugger is not supported\n");
		return -ENOTSUP;
	}
	if (data->timeout == 0) {
		LOG_ERR("timeout illegal!\n");
		return -ENOTSUP;
	}

	WDG_StructMemValueSet(&WDG_initstruct, data->window, data->timeout, config->eicnt);
	WDG_Init(config->WDG, &WDG_initstruct);
	WDG_Enable(config->WDG);

	return 0;
}

static int wdt_ameba_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct wdt_ameba_data *data = dev->data;

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		return -EINVAL;
	}
	if (cfg->flags == WDT_FLAG_RESET_NONE) {
		return -ENOTSUP;
	}

	data->timeout = cfg->window.max;
	data->window = (cfg->window.max - cfg->window.min);
	data->callback = cfg->callback;

	return 0;
}

static int wdt_ameba_init(const struct device *dev)
{
	const struct wdt_ameba_config *config = dev->config;

	config->irq_config_func(dev);

	return 0;
}

static void wdt_ameba_isr(const struct device *dev)
{
	struct wdt_ameba_data *data = dev->data;
	const struct wdt_ameba_config *config = dev->config;
	wdt_callback_t cb;

	cb = data->callback;
	if (cb) {
		cb(dev, 0);
	}

	WDG_INTConfig(config->WDG, WDG_BIT_EIE, DISABLE);
	WDG_ClearINT(config->WDG, WDG_BIT_EIC);
}

static const struct wdt_driver_api wdt_api = {.setup = wdt_ameba_setup,
					      .disable = wdt_ameba_disable,
					      .install_timeout = wdt_ameba_install_timeout,
					      .feed = wdt_ameba_feed};

#define WDT_IRQ_CONFIG(n)                                                                          \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), wdt_ameba_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define AMEBA_WDT_INIT(n)                                                                          \
	WDT_IRQ_CONFIG(n)                                                                          \
	static struct wdt_ameba_data wdt##n##_data;                                                \
	static const struct wdt_ameba_config wdt_ameba_config##n = {                               \
		.WDG = (WDG_TypeDef *)DT_INST_REG_ADDR(n),                                         \
		.irq_source = DT_INST_IRQN(n),                                                     \
		.irq_config_func = irq_config_##n,                                                 \
		.eicnt = DT_INST_PROP(n, early_int_cnt)};                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, wdt_ameba_init, NULL, &wdt##n##_data, &wdt_ameba_config##n,       \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_WDT_INIT)
