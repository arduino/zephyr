/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBAD_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBAD_CLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Domain clocks
 *
 * AON Domain
 * AON_OTP Domain
 * SYSON Domain
 * SOC Domain
 * BT Domain
 */

/* AON Domain clocks */
#define AMEBA_ATIM_CLK 0
#define AMEBA_RTC_CLK  1

/* SYSON Domain clocks */
#define AMEBA_PWM0_CLK    2
#define AMEBA_UART0_CLK   3
#define AMEBA_LOGUART_CLK 4
#define AMEBA_UART3_CLK   5
#define AMEBA_ADC_CLK     6
#define AMEBA_GPIO_CLK    7
#define AMEBA_LTIM0_CLK   8
#define AMEBA_LTIM1_CLK   9
#define AMEBA_LTIM2_CLK   10
#define AMEBA_LTIM3_CLK   11

/* SOC Domain clocks */
#define AMEBA_PERI_HCLK 12
#define AMEBA_GDMA0_CLK 13
#define AMEBA_SPI0_CLK  14
#define AMEBA_SPI1_CLK  15
#define AMEBA_FLASH_CLK 16
#define AMEBA_PSRAM_CLK 17
#define AMEBA_I2C0_CLK  18
#define AMEBA_PRNG_CLK  19

#define AMEBA_CLK_MAX 20 /* clk idx max */

#define AMEBA_NUMERICAL_PERIPH(name, n)                                                            \
	[AMEBA_##name##n##_CLK] = {                                                                \
		.parent = AMEBA_RCC_NO_PARENT,                                                     \
		.cke = APBPeriph_##name##n##_CLOCK,                                                \
		.fen = APBPeriph_##name##n,                                                        \
	},

#define AMEBA_SINGLE_PERIPH(name)                                                                  \
	[AMEBA_##name##_CLK] = {                                                                   \
		.parent = AMEBA_RCC_NO_PARENT,                                                     \
		.cke = APBPeriph_##name##_CLOCK,                                                   \
		.fen = APBPeriph_##name,                                                           \
	},

#define AMEBA_REMAP_PERIPH(clk_index, remap_name)                                                  \
	[clk_index] = {                                                                            \
		.parent = AMEBA_RCC_NO_PARENT,                                                     \
		.cke = APBPeriph_##remap_name##_CLOCK,                                             \
		.fen = APBPeriph_##remap_name,                                                     \
	},

#define AMEBA_LTIM_PERIPHS                                                                         \
	AMEBA_REMAP_PERIPH(AMEBA_LTIM0_CLK, GTIMER)                                                \
	AMEBA_REMAP_PERIPH(AMEBA_LTIM1_CLK, GTIMER)                                                \
	AMEBA_REMAP_PERIPH(AMEBA_LTIM2_CLK, GTIMER)                                                \
	AMEBA_REMAP_PERIPH(AMEBA_LTIM3_CLK, GTIMER)

#define AMEBA_PWM_PERIPHS AMEBA_REMAP_PERIPH(AMEBA_PWM0_CLK, GTIMER)

#define AMEBA_SPI_PERIPHS                                                                          \
	AMEBA_NUMERICAL_PERIPH(SPI, 0) /* AMEBA_SPI0_CLK */                                        \
	AMEBA_NUMERICAL_PERIPH(SPI, 1) /* AMEBA_SPI1_CLK */

#define AMEBA_I2C_PERIPHS AMEBA_NUMERICAL_PERIPH(I2C, 0) /* AMEBA_I2C0_CLK */

#define AMEBA_UART_PERIPHS                                                                         \
	AMEBA_NUMERICAL_PERIPH(UART, 0) /* AMEBA_UART0_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(UART, 3) /* AMEBA_UART3_CLK */

#define AMEBA_GDMA0_PERIPHS AMEBA_SINGLE_PERIPH(GDMA0) /* AMEBA_GDMA0_CLK */
#define AMEBA_PSRAM_PERIPHS AMEBA_SINGLE_PERIPH(PSRAM) /* AMEBA_PSRAM_CLK */
#define AMEBA_RTC_PERIPHS   AMEBA_SINGLE_PERIPH(RTC)   /* AMEBA_RTC_CLK */

/* TODO: Enabled in KM0 */
#define AMEBA_LOGUART_PERIPHS AMEBA_REMAP_PERIPH(AMEBA_LOGUART_CLK, NULL)
#define AMEBA_FLASH_PERIPHS   AMEBA_REMAP_PERIPH(AMEBA_FLASH_CLK, NULL)
#define AMEBA_GPIO_PERIPHS    AMEBA_REMAP_PERIPH(AMEBA_GPIO_CLK, NULL)
#define AMEBA_ADC_PERIPHS     AMEBA_REMAP_PERIPH(AMEBA_ADC_CLK, NULL)
#define AMEBA_PRNG_PERIPHS    AMEBA_REMAP_PERIPH(AMEBA_PRNG_CLK, NULL)

#define APBPeriph_NULL_CLOCK APBPeriph_CLOCK_NULL

#define AMEBA_CORE_PERIPHS                                                                         \
	AMEBA_RTC_PERIPHS                                                                          \
	AMEBA_PWM_PERIPHS                                                                          \
	AMEBA_UART_PERIPHS                                                                         \
	AMEBA_LOGUART_PERIPHS                                                                      \
	AMEBA_ADC_PERIPHS                                                                          \
	AMEBA_GPIO_PERIPHS                                                                         \
	AMEBA_LTIM_PERIPHS                                                                         \
	AMEBA_GDMA0_PERIPHS                                                                        \
	AMEBA_SPI_PERIPHS                                                                          \
	AMEBA_FLASH_PERIPHS                                                                        \
	AMEBA_I2C_PERIPHS                                                                          \
	AMEBA_PRNG_PERIPHS

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBAD_CLOCK_H_ */
