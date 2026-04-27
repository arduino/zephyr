/*
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * STM32 U5OTGHS embedded HS PHY driver
 *
 * Covers the unnamed PHY first found in STM32U5 series,
 * and later used in other series such as STM32WBA, ...
 *
 * Note: the HAL SYSCFG used is provided by stm32XXX_hal.c
 * which is always included in the build, and the __HAL_RCC
 * function is actually an in-header/LL-like macro.
 */
#include <soc.h>
#include <stm32_ll_system.h>

#include <stm32_bitops.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "stm32_usb_common.h"

#include <stm32h5xx_ll_rcc.h>
#include <stm32h5xx_hal_pwr_ex.h>

/* Even though we won't use this macro, define it for grep-ability */
#define DT_DRV_COMPAT st_stm32h5_otghs_phy

struct stm32_h5otghs_phy_config {
	uint32_t num_clocks;
	struct stm32_pclken clocks[];
};

static const struct device *rcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

static int stm32_h5otghs_phy_enable(const struct stm32_usb_phy *phy)
{
	const struct stm32_h5otghs_phy_config *cfg = phy->pcfg;
	int res;

	/* Configure PHY input mux (if provided) */
	if (cfg->num_clocks > 1) {
		res = clock_control_configure(rcc, (clock_control_subsys_t)&cfg->clocks[1], NULL);
		if (res != 0) {
			return res;
		}
	}

	/* Turn on the PHY's clock */
	res = clock_control_on(rcc, (clock_control_subsys_t)&cfg->clocks[0]);

	__HAL_RCC_USB_OTG_HS_CLK_ENABLE();

	LL_RCC_SetOTGPHYClockSource(LL_RCC_OTGPHYREFCKCLKSOURCE_24M);
	__HAL_RCC_OTGHS_CONFIG(RCC_OTGHSCLKSOURCE_HSE_DIV2);

	stm32_reg_set_bits(&PWR->USBSCR, PWR_USBSCR_OTGHSEN);

	return res;
}

static int stm32_h5otghs_phy_disable(const struct stm32_usb_phy *phy)
{
	const struct stm32_h5otghs_phy_config *cfg = phy->pcfg;

	stm32_reg_clear_bits(&PWR->USBSCR, PWR_USBSCR_OTGHSEN);

	return clock_control_off(rcc, (clock_control_subsys_t)&cfg->clocks[0]);
}

#define DEFINE_H5OTGHS_PHY(usb_node, phy_node)							\
	static const struct stm32_h5otghs_phy_config CONCAT(phy, DT_DEP_ORD(phy_node), _cfg) = {\
		.num_clocks = DT_NUM_CLOCKS(phy_node),						\
		.clocks = STM32_DT_CLOCKS(phy_node)						\
	};											\
	const struct stm32_usb_phy USB_STM32_PHY_PSEUDODEV_NAME(usb_node) = {			\
		.enable = stm32_h5otghs_phy_enable,						\
		.disable = stm32_h5otghs_phy_disable,						\
		.pcfg = &CONCAT(phy, DT_DEP_ORD(phy_node), _cfg),				\
	};

/*
 * Iterate all USB nodes and instantiate PHY when appropriate.
 * We could implement a stricter check but this should suffice;
 * there are no series with different type of embedded HS PHYs.
 */
#define _FOREACH_NODE(usb_node)									\
	IF_ENABLED(USB_STM32_NODE_PHY_IS_EMBEDDED_HS(usb_node),					\
		(DEFINE_H5OTGHS_PHY(usb_node, USB_STM32_PHY(usb_node))))
#define _FOREACH_COMPAT(compat) DT_FOREACH_STATUS_OKAY(compat, _FOREACH_NODE)
FOR_EACH(_FOREACH_COMPAT, (), STM32_USB_COMPATIBLES)
