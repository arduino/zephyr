/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBADPLUS_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBADPLUS_CLOCK_H_

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
#define AMEBA_ATIM_CLK 1
#define AMEBA_RTC_CLK  2

/* SYSON Domain clocks */
#define AMEBA_PWM0_CLK    3
#define AMEBA_PWM1_CLK    4
#define AMEBA_HTIM0_CLK   5
#define AMEBA_HTIM1_CLK   6
#define AMEBA_LEDC_CLK    7
#define AMEBA_UART0_CLK   8
#define AMEBA_UART1_CLK   9
#define AMEBA_UART2_CLK   10
#define AMEBA_LOGUART_CLK 11
#define AMEBA_DTIM_CLK    12
#define AMEBA_ADC_CLK     13
#define AMEBA_GPIO_CLK    14
#define AMEBA_LTIM0_CLK   15
#define AMEBA_LTIM1_CLK   16
#define AMEBA_LTIM2_CLK   17
#define AMEBA_LTIM3_CLK   18
#define AMEBA_LTIM4_CLK   19
#define AMEBA_LTIM5_CLK   20
#define AMEBA_LTIM6_CLK   21
#define AMEBA_LTIM7_CLK   22
#define AMEBA_PTIM0_CLK   23
#define AMEBA_PTIM1_CLK   24
#define AMEBA_KSCAN_CLK   25

/* SOC Domain clocks */
#define AMEBA_DMAC_CLK   26
#define AMEBA_SDIO_CLK   27
#define AMEBA_SPI0_CLK   28
#define AMEBA_SPI1_CLK   29
#define AMEBA_USB_CLK    30
#define AMEBA_FLASH_CLK  31
#define AMEBA_PSRAM_CLK  32
#define AMEBA_SPORT0_CLK 33
#define AMEBA_SPORT1_CLK 34
#define AMEBA_AC_CLK     35
#define AMEBA_IRDA_CLK   36
#define AMEBA_I2C0_CLK   37
#define AMEBA_I2C1_CLK   38
#define AMEBA_TRNG_CLK   39

/* misc clocks */
#define AMEBA_BTON_CLK 40
#define AMEBA_AES_CLK  42

#define AMEBA_CLK_MAX 43 /* clk idx max */

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

#define AMEBA_LTIM_PERIPHS                                                                         \
	AMEBA_NUMERICAL_PERIPH(LTIM, 0) /* AMEBA_LTIM0_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 1) /* AMEBA_LTIM1_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 2) /* AMEBA_LTIM2_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 3) /* AMEBA_LTIM3_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 4) /* AMEBA_LTIM4_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 5) /* AMEBA_LTIM5_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 6) /* AMEBA_LTIM6_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(LTIM, 7) /* AMEBA_LTIM7_CLK */

#define AMEBA_PTIM_PERIPHS                                                                         \
	AMEBA_NUMERICAL_PERIPH(PTIM, 0) /* AMEBA_PTIM0_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(PTIM, 1) /* AMEBA_PTIM1_CLK */

#define AMEBA_SPI_PERIPHS                                                                          \
	AMEBA_NUMERICAL_PERIPH(SPI, 0) /* AMEBA_SPI0_CLK */                                        \
	AMEBA_NUMERICAL_PERIPH(SPI, 1) /* AMEBA_SPI1_CLK */

#define AMEBA_SPORT_PERIPHS                                                                        \
	AMEBA_NUMERICAL_PERIPH(SPORT, 0) /* AMEBA_SPORT0_CLK */                                    \
	AMEBA_NUMERICAL_PERIPH(SPORT, 1) /* AMEBA_SPORT1_CLK */

#define AMEBA_I2C_PERIPHS                                                                          \
	AMEBA_NUMERICAL_PERIPH(I2C, 0) /* AMEBA_I2C0_CLK */                                        \
	AMEBA_NUMERICAL_PERIPH(I2C, 1) /* AMEBA_I2C1_CLK */

#define AMEBA_PWM_PERIPHS                                                                          \
	AMEBA_NUMERICAL_PERIPH(PWM, 0) /* AMEBA_PWM0_CLK */                                        \
	AMEBA_NUMERICAL_PERIPH(PWM, 1) /* AMEBA_PWM1_CLK */

#define AMEBA_HTIM_PERIPHS                                                                         \
	AMEBA_NUMERICAL_PERIPH(HTIM, 0) /* AMEBA_HTIM0_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(HTIM, 1) /* AMEBA_HTIM1_CLK */

#define AMEBA_UART_PERIPHS                                                                         \
	AMEBA_NUMERICAL_PERIPH(UART, 0) /* AMEBA_UART0_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(UART, 1) /* AMEBA_UART1_CLK */                                      \
	AMEBA_NUMERICAL_PERIPH(UART, 2) /* AMEBA_UART2_CLK */

#define AMEBA_LOGUART_PERIPHS AMEBA_SINGLE_PERIPH(LOGUART) /* AMEBA_LOGUART_CLK */
#define AMEBA_KSCAN_PERIPHS   AMEBA_SINGLE_PERIPH(KSCAN)   /* AMEBA_KSCAN_CLK */
#define AMEBA_DMAC_PERIPHS    AMEBA_SINGLE_PERIPH(DMAC)    /* AMEBA_DMAC_CLK */
#define AMEBA_SDIO_PERIPHS    AMEBA_SINGLE_PERIPH(SDIO)    /* AMEBA_SDIO_CLK */
#define AMEBA_USB_PERIPHS     AMEBA_SINGLE_PERIPH(USB)     /* AMEBA_USB_CLK */
#define AMEBA_FLASH_PERIPHS   AMEBA_SINGLE_PERIPH(FLASH)   /* AMEBA_FLASH_CLK */
#define AMEBA_PSRAM_PERIPHS   AMEBA_SINGLE_PERIPH(PSRAM)   /* AMEBA_PSRAM_CLK */
#define AMEBA_AC_PERIPHS      AMEBA_SINGLE_PERIPH(AC)      /* AMEBA_AC_CLK */
#define AMEBA_IRDA_PERIPHS    AMEBA_SINGLE_PERIPH(IRDA)    /* AMEBA_IRDA_CLK */
#define AMEBA_TRNG_PERIPHS    AMEBA_SINGLE_PERIPH(TRNG)    /* AMEBA_TRNG_CLK */
#define AMEBA_RTC_PERIPHS     AMEBA_SINGLE_PERIPH(RTC)     /* AMEBA_RTC_CLK */
#define AMEBA_LEDC_PERIPHS    AMEBA_SINGLE_PERIPH(LEDC)    /* AMEBA_LEDC_CLK */
#define AMEBA_ADC_PERIPHS     AMEBA_SINGLE_PERIPH(ADC)     /* AMEBA_ADC_CLK */
#define AMEBA_GPIO_PERIPHS    AMEBA_SINGLE_PERIPH(GPIO)    /* AMEBA_GPIO_CLK */
#define AMEBA_BTON_PERIPHS    AMEBA_SINGLE_PERIPH(BTON)    /* AMEBA_BTON_CLK */

#define AMEBA_CORE_PERIPHS                                                                         \
	AMEBA_RTC_PERIPHS                                                                          \
	AMEBA_PWM_PERIPHS                                                                          \
	AMEBA_HTIM_PERIPHS                                                                         \
	AMEBA_LEDC_PERIPHS                                                                         \
	AMEBA_UART_PERIPHS                                                                         \
	AMEBA_LOGUART_PERIPHS                                                                      \
	AMEBA_ADC_PERIPHS                                                                          \
	AMEBA_GPIO_PERIPHS                                                                         \
	AMEBA_LTIM_PERIPHS                                                                         \
	AMEBA_PTIM_PERIPHS                                                                         \
	AMEBA_KSCAN_PERIPHS                                                                        \
	AMEBA_DMAC_PERIPHS                                                                         \
	AMEBA_SDIO_PERIPHS                                                                         \
	AMEBA_SPI_PERIPHS                                                                          \
	AMEBA_USB_PERIPHS                                                                          \
	AMEBA_FLASH_PERIPHS                                                                        \
	AMEBA_SPORT_PERIPHS                                                                        \
	AMEBA_AC_PERIPHS                                                                           \
	AMEBA_I2C_PERIPHS                                                                          \
	AMEBA_TRNG_PERIPHS                                                                         \
	AMEBA_BTON_PERIPHS

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBADPLUS_CLOCK_H_ */
