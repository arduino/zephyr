/*
 * Copyright (c) Arduino s.r.l. and/or its affiliated companies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>
#include <bsp_api.h>
#include "ra_icu_dyn.h"

int ra_icu_dyn_irq_connect(uint32_t event, uint32_t priority, void (*isr)(const void *),
			   const void *arg)
{
	unsigned int key = irq_lock();

	for (int slot = 0; slot < BSP_ICU_VECTOR_MAX_ENTRIES; slot++) {
		/* A slot without a handler is free, statically connected ones are skipped */
		if (_sw_isr_table[slot].isr != z_irq_spurious) {
			continue;
		}

		irq_connect_dynamic(slot, priority, isr, arg, 0);
		R_ICU->IELSR[slot] = event;
		irq_enable(slot);
		irq_unlock(key);
		return slot;
	}

	irq_unlock(key);
	return -ENOMEM;
}

void ra_icu_dyn_irq_disconnect(int slot)
{
	unsigned int key = irq_lock();

	irq_disable(slot);
	R_ICU->IELSR[slot] = 0;
	z_isr_install(slot, z_irq_spurious, NULL);
	irq_unlock(key);
}
