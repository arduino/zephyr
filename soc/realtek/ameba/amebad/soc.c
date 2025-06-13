/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <ameba_soc.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include "ameba_system.h"

void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {z_arm_reset, NULL, /* BOOT_RAM_WakeFromPG, */
				    (uint32_t)NewVectorTable};

static void app_vdd1833_detect(void)
{
	u32 temp;

	if (FALSE == is_power_supply18()) {
		temp = HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HS_RFAFE_IND_VIO1833);
		temp |= BIT_RFAFE_IND_VIO1833;
		HAL_WRITE32(SYSTEM_CTRL_BASE_HP, REG_HS_RFAFE_IND_VIO1833, temp);
	}

	printk("REG_HS_RFAFE_IND_VIO1833 (0 is 1.8V): %x\n",
	       HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HS_RFAFE_IND_VIO1833));
}

/* Disable KM0 Loguart Interrupt */
void shell_loguratRx_Ipc_Tx(u32 ipc_dir, u32 ipc_ch)
{
	IPC_MSG_STRUCT ipc_msg_temp;

	ipc_msg_temp.msg_type = IPC_USER_POINT;
	ipc_msg_temp.msg = 0;
	ipc_msg_temp.msg_len = 1;
	ipc_msg_temp.rsvd = 0; /* for coverity init issue */
	ipc_send_message(ipc_dir, ipc_ch, &ipc_msg_temp);
}

void soc_early_init_hook(void)
{
	/*
	 * Cache is enabled by default at reset, disable it before
	 * sys_cache*-functions can enable them.
	 */
	Cache_Enable(DISABLE);
	sys_cache_data_enable();
	sys_cache_instr_enable();

	shell_loguratRx_Ipc_Tx((u32)NULL, IPC_INT_CHAN_SHELL_SWITCH);

	SystemSetCpuClk(CONFIG_CPU_CLOCK_SEL_VALUE);

	/* Register IPC interrupt */
	IRQ_CONNECT(IPC_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCM0_DEV, 0);
	irq_enable(IPC_IRQ);

	/* IPC table initialization */
	ipc_table_init(IPCM4_DEV);

	app_vdd1833_detect();
}
