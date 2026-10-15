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

/* Same as the STM32 HAL: the IN response time depends on the AHB clock */
static void stm32_otghs_uhc_set_trdt(struct usb_dwc2_reg *const base)
{
	const uint32_t trdt_pos = 10U;
	const uint32_t trdt_mask = 0xFU << trdt_pos;
	uint32_t gusbcfg;

	if (sys_clock_hw_cycles_per_sec() < 32000000U) {
		LOG_WRN("AHB clock below 32 MHz, keeping the default turnaround time");
		return;
	}

	gusbcfg = sys_read32((mem_addr_t)&base->gusbcfg);
	gusbcfg &= ~trdt_mask;
	gusbcfg |= 6U << trdt_pos;
	sys_write32(gusbcfg, (mem_addr_t)&base->gusbcfg);
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
	stm32_otghs_uhc_set_trdt(base);

	return 0;
}

static int stm32_otghs_uhc_pre_enable(const struct device *dev)
{
	struct usb_dwc2_reg *const base = uhc_dwc2_get_base(dev);

	sys_set_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_PWRDWN);
	sys_clear_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

/* The core soft reset restores the turnaround time (TRDT reads 5 afterwards) */
static int stm32_otghs_uhc_post_reset(const struct device *dev)
{
	struct usb_dwc2_reg *const base = uhc_dwc2_get_base(dev);

	stm32_otghs_uhc_set_trdt(base);
	sys_set_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_PWRDWN);
	sys_clear_bits((mem_addr_t)&base->ggpio, USB_DWC2_GGPIO_STM32_VBDEN);
	/* The embedded PHY is full-speed only, as the STM32 HAL host init sets it */
	sys_set_bits((mem_addr_t)&base->hcfg, USB_DWC2_HCFG_FSLSSUPP);

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
#define UHC_DWC2_STM32_IS_OTGFS(n) DT_INST_NODE_HAS_COMPAT(n, st_stm32_otgfs_uhc)
#define UHC_DWC2_STM32_IS_OTG(n)                                               \
	UTIL_OR(UHC_DWC2_STM32_IS_OTGHS(n), UHC_DWC2_STM32_IS_OTGFS(n))

/* The node may have several interrupts, the controller one is named "otghs"/"otgfs" */
#define UHC_DWC2_STM32_IRQN(n)                                                 \
	COND_CODE_1(UHC_DWC2_STM32_IS_OTGHS(n),                                \
		    (DT_INST_IRQ_BY_NAME(n, otghs, irq)),                      \
		    (COND_CODE_1(UHC_DWC2_STM32_IS_OTGFS(n),                   \
				 (DT_INST_IRQ_BY_NAME(n, otgfs, irq)),         \
				 (DT_INST_IRQN(n)))))

#define UHC_DWC2_STM32_IRQ_PRIO(n)                                             \
	COND_CODE_1(UHC_DWC2_STM32_IS_OTGHS(n),                                \
		    (DT_INST_IRQ_BY_NAME(n, otghs, priority)),                 \
		    (COND_CODE_1(UHC_DWC2_STM32_IS_OTGFS(n),                   \
				 (DT_INST_IRQ_BY_NAME(n, otgfs, priority)),    \
				 (DT_INST_IRQ(n, priority)))))

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
		.post_reset = stm32_otghs_uhc_post_reset,                      \
		.shutdown = stm32_otghs_uhc_shutdown,                          \
		.use_fs_phy = stm32_otghs_uhc_use_fs_phy,                      \
	};

#define QUIRK_STM32_OTGHS_UHC_DEFINE(n)                                        \
	COND_CODE_1(UHC_DWC2_STM32_IS_OTG(n),                                  \
		    (QUIRK_STM32_OTGHS_UHC_INST(n)), ())

DT_INST_FOREACH_STATUS_OKAY(QUIRK_STM32_OTGHS_UHC_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UHC_DWC2_STM32_OTGHS_H */
