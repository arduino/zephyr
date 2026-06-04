/*
 * Copyright (c) 2024-2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm/nmi.h>
#include <zephyr/cache.h>
#include <bsp_api.h>

extern void NMI_Handler(void);

void soc_early_init_hook(void)
{
#ifdef CONFIG_RUNTIME_NMI
	z_arm_nmi_set_handler(NMI_Handler);
#endif /* CONFIG_RUNTIME_NMI */

#if defined(CONFIG_SOC_SERIES_RA6M5)
	/* The first-stage bootloader may hand off with stale ICU event links still
	 * programmed in R_ICU->IELSR (they are not cleared by a software reset).
	 * FSP's bsp_irq_cfg() only writes the non-zero entries of
	 * g_interrupt_event_link_select[] (empty under dynamic interrupt numbering),
	 * so leftover links survive. Such a link fires on an NVIC line with no
	 * connected ISR as soon as the peripheral generates the event (e.g. SCI3
	 * TXI/TEI driven by the PF1550 PMIC) -> spurious "Unhandled IRQn" fatal.
	 * Wipe the whole table before any driver configures its own vectors. */
	for (int i = 0; i < BSP_ICU_VECTOR_MAX_ENTRIES; i++) {
		R_ICU->IELSR[i] = 0;
	}
#endif /* CONFIG_SOC_SERIES_RA6M5 */

#if defined(CONFIG_DCACHE)
	sys_cache_data_enable();
#endif /* CONFIG_DCACHE */

#if defined(CONFIG_ICACHE)
	/* Invalidate I-Cache after initializing the .ram_from_flash section. */
	sys_cache_instr_invd_all();
#endif /* CONFIG_ICACHE */
}

void soc_late_init_hook(void)
{
#ifdef CONFIG_SOC_RA_ENABLE_START_SECOND_CORE
	R_BSP_SecondaryCoreStart();
#endif /* CONFIG_SOC_RA_ENABLE_START_SECOND_CORE */
}
