/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UHC_DWC2_STM32_OTGHS_H
#define ZEPHYR_DRIVERS_USB_UHC_DWC2_STM32_OTGHS_H

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <stm32_ll_bus.h>
#include <stm32_usb_common.h>

struct stm32_otghs_uhc_config {
	const struct stm32_pclken *pclken;
	size_t num_clocks;
	const struct pinctrl_dev_config *pinctrl;
};

static int stm32_otghs_uhc_use_fs_phy(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static int stm32_otghs_uhc_pre_init(const struct device *dev)
{
	const struct stm32_otghs_uhc_config *const cfg = UHC_DWC2_QUIRK_CONFIG(dev);
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	struct usb_dwc2_reg *const base = uhc_dwc2_get_base(dev);
	int ret;

	ret = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	ret = stm32_usb_pwr_enable();
	if (ret != 0) {
		return ret;
	}

	if (cfg->num_clocks > 1) {
		ret = clock_control_configure(clk, (void *)&cfg->pclken[1], NULL);
		if (ret != 0) {
			return ret;
		}
	}

	ret = clock_control_on(clk, (void *)&cfg->pclken[0]);
	if (ret != 0) {
		return ret;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	/* The ULPI clock must not sleep when the embedded FS PHY is used */
	LL_AHB1_GRP1_DisableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
#endif

	/* PHY must be selected before the core reset */
	sys_set_bits((mem_addr_t)&base->gusbcfg, USB_DWC2_GUSBCFG_PHYSEL_USB11);
	sys_set_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_PWRDWN);
	sys_clear_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

static int stm32_otghs_uhc_pre_enable(const struct device *dev)
{
	struct usb_dwc2_reg *const base = uhc_dwc2_get_base(dev);

	/* Not guaranteed to survive the core reset done in init */
	sys_set_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_PWRDWN);
	sys_clear_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

static int stm32_otghs_uhc_shutdown(const struct device *dev)
{
	const struct stm32_otghs_uhc_config *const cfg = UHC_DWC2_QUIRK_CONFIG(dev);
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	struct usb_dwc2_reg *const base = uhc_dwc2_get_base(dev);
	int ret;

	sys_clear_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_PWRDWN);

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_AHB1_GRP1_EnableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
#endif

	ret = clock_control_off(clk, (void *)&cfg->pclken[0]);
	if (ret != 0) {
		LOG_ERR("Unable to disable the clock: %d", ret);
		return ret;
	}

	return stm32_usb_pwr_disable();
}

#define UHC_DWC2_STM32_IS_OTGHS(n) DT_INST_NODE_HAS_COMPAT(n, st_stm32_otghs_uhc)

/* The node has several interrupts, the controller one is named "otghs" */
#define UHC_DWC2_STM32_IRQN(n)                                                 \
	COND_CODE_1(UHC_DWC2_STM32_IS_OTGHS(n),                                \
		    (DT_INST_IRQ_BY_NAME(n, otghs, irq)), (DT_INST_IRQN(n)))

#define UHC_DWC2_STM32_IRQ_PRIO(n)                                             \
	COND_CODE_1(UHC_DWC2_STM32_IS_OTGHS(n),                                \
		    (DT_INST_IRQ_BY_NAME(n, otghs, priority)),                 \
		    (DT_INST_IRQ(n, priority)))

#define UHC_DWC2_IRQ_DT_INST_DEFINE(n)                                         \
	static void uhc_dwc2_irq_enable_func_##n(const struct device *dev)     \
	{                                                                      \
		ARG_UNUSED(dev);                                               \
		IRQ_CONNECT(UHC_DWC2_STM32_IRQN(n),                            \
			    UHC_DWC2_STM32_IRQ_PRIO(n),                        \
			    uhc_dwc2_isr_handler,                              \
			    DEVICE_DT_INST_GET(n),                             \
			    0);                                                \
									       \
		irq_enable(UHC_DWC2_STM32_IRQN(n));                            \
	}                                                                      \
									       \
	static void uhc_dwc2_irq_disable_func_##n(const struct device *dev)    \
	{                                                                      \
		irq_disable(UHC_DWC2_STM32_IRQN(n));                           \
	}

#define QUIRK_STM32_OTGHS_UHC_INST(n)                                          \
	PINCTRL_DT_INST_DEFINE(n);                                             \
									       \
	static const struct stm32_pclken stm32_otghs_uhc_pclken_##n[] =       \
		STM32_DT_INST_CLOCKS(n);                                       \
									       \
	static const struct stm32_otghs_uhc_config uhc_dwc2_quirk_config_##n = { \
		.pclken = stm32_otghs_uhc_pclken_##n,                          \
		.num_clocks = DT_INST_NUM_CLOCKS(n),                           \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                  \
	};                                                                     \
									       \
	static uint8_t uhc_dwc2_quirk_data_##n;                                \
									       \
	static const struct uhc_dwc2_vendor_quirks uhc_dwc2_vendor_quirks_##n = { \
		.pre_init = stm32_otghs_uhc_pre_init,                          \
		.pre_enable = stm32_otghs_uhc_pre_enable,                      \
		.shutdown = stm32_otghs_uhc_shutdown,                          \
		.use_fs_phy = stm32_otghs_uhc_use_fs_phy,                      \
	};

#define QUIRK_STM32_OTGHS_UHC_DEFINE(n)                                        \
	COND_CODE_1(UHC_DWC2_STM32_IS_OTGHS(n),                                \
		    (QUIRK_STM32_OTGHS_UHC_INST(n)), ())

DT_INST_FOREACH_STATUS_OKAY(QUIRK_STM32_OTGHS_UHC_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UHC_DWC2_STM32_OTGHS_H */
