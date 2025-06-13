/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AMEBA_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AMEBA_CLOCK_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_SOC_SERIES_AMEBADPLUS)
#include <zephyr/dt-bindings/clock/amebadplus_clock.h>
#elif defined(CONFIG_SOC_SERIES_AMEBAD)
#include <zephyr/dt-bindings/clock/amebad_clock.h>
#else
#error : please choose the right chip type.
#endif

#define AMEBA_RCC_DIV_VALUE_INVALID 0xFFFFFFFF
#define AMEBA_RCC_SRC_VALUE_INVALID 0xFFFFFFFF

/** Common clock control device node for all ameba chips */
#define AMEBA_CLOCK_CONTROL_NODE DT_NODELABEL(rcc)
#define AMEBA_CLOCK_CONTROL_DEV  DEVICE_DT_GET(AMEBA_CLOCK_CONTROL_NODE)

#define AMEBA_RCC_PRAM_CFG(src_val, div_val)                                                       \
	{                                                                                          \
		.div = div_val,                                                                    \
		.src = src_val,                                                                    \
	}

#define AMEBA_RCC_CFG_ONLY_SRC(src) AMEBA_RCC_PRAM_CFG(src, AMEBA_RCC_DIV_VALUE_INVALID)
#define AMEBA_RCC_CFG_ONLY_DIV(div) AMEBA_RCC_PRAM_CFG(AMEBA_RCC_SRC_VALUE_INVALID, div)

/*
 * struct ameba_clock_config clk_ctrl = AMEBA_RCC_PRAM_CFG(0,1);
 * clock_control_configure(data->clock, idx,&clk_ctrl);
 */

/* clock configure params */
struct ameba_clock_config {
	uint32_t div; /* clock div value */
	uint32_t src; /* clock source value */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AMEBA_CLOCK_CONTROL_H_ */
