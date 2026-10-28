/*
 * Copyright (c) Arduino s.r.l. and/or its affiliated companies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_RENESAS_RA_COMMON_RA_ICU_DYN_H_
#define ZEPHYR_SOC_RENESAS_RA_COMMON_RA_ICU_DYN_H_

#include <stdint.h>

/**
 * Link an ICU event to the first NVIC slot that has no handler, install @p isr on it and enable it.
 *
 * @return the NVIC slot, or a negative errno if none is free.
 */
int ra_icu_dyn_irq_connect(uint32_t event, uint32_t priority, void (*isr)(const void *),
			   const void *arg);

/** Release a slot returned by ra_icu_dyn_irq_connect(). */
void ra_icu_dyn_irq_disconnect(int slot);

#endif /* ZEPHYR_SOC_RENESAS_RA_COMMON_RA_ICU_DYN_H_ */
