/*
 * Copyright (c) 2025 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/hwinfo.h>
#include <soc.h>

/* Magic number to indicate a double-reset */
#define BOOT_MAGIC_NUMBER           0xDEADBEEF
#define STM32H5_ROM_BOOTLOADER_ADDR 0x0BFAC000

/* Get the base address of the backup SRAM from Devicetree */
#define BACKUP_SRAM_NODE DT_INST(0, st_stm32_backup_sram)
#define BACKUP_SRAM_ADDR DT_REG_ADDR(BACKUP_SRAM_NODE)

struct backup_store {
	uint32_t wait_for_app_magic;
	uint32_t magic;
};
volatile __stm32_backup_sram_section struct backup_store backup;

#ifdef RESET_RCC_ENABLED
static void Reset_RCC_To_HSI(void)
{
	/* Turn on the High-Speed Internal (HSI) oscillator */
	RCC->CR |= RCC_CR_HSION;

	/* Wait until HSI is stable and ready */
	while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
		/* Spin block */
	}

	/* Switch the System Clock back to HSI by resetting the Clock Configuration
	 * Register
	 */
	RCC->CFGR1 = 0x00000000;

	/* Wait until HSI is actively being used as the system clock.
	 * The SWS (System clock switch status) bits will read as 0 when HSI is
	 * active.
	 */
	while ((RCC->CFGR1 & RCC_CFGR1_SWS) != 0) {
		/* Spin block */
	}

	/* Disable the High-Speed External (HSE) oscillator and all PLLs */
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);

	/* Clear all clock-related interrupts and pending flags */
	RCC->CICR = 0xFFFFFFFF;
}
#endif
/**
 * @brief Standard ARM Cortex-M routine to jump to a ROM Bootloader
 */
void JumpToBootloader(void)
{
	printk("JUMP............\n");
	k_msleep(100);

	uint32_t i = 0;
	void (*SysMemBootJump)(void);
	/* * STM32H5 System Memory (ROM Bootloader) Address */
	const uint32_t boot_addr = STM32H5_ROM_BOOTLOADER_ADDR;
	/* Disable all interrupts */
	__disable_irq();
	/* Disable Systick timer */
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	MPU->CTRL = 0;
	/* Set the clock to the default state */
	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	for (i = 0; i < 5; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
#ifdef RESET_RCC_ENABLED
	Reset_RCC_To_HSI();
#endif
	/* The ROM bootloader must receive interrupts through its own vector table. */
	SCB->VTOR = boot_addr;
	__DSB();
	__ISB();
	/* Set up the jump to boot loader address + 4 */
	SysMemBootJump = (void (*)(void))(*((uint32_t *)(boot_addr + 4)));
	/*
	 * Zephyr runs application threads on PSP and may configure both stack
	 * limits. Restore the reset-state stack configuration expected by ROM.
	 */
	__set_CONTROL(0);
	__set_MSPLIM(0);
	__set_PSPLIM(0);
	__ISB();
	__set_MSP(*(uint32_t *)boot_addr);
	__enable_irq();
	/* Call the function to jump to boot loader location */
	SysMemBootJump();

	/* Jump is done successfully */
	while (1) {
		/* Code should never reach this loop */
	}
}

/**
 * @brief Initialization function called by Zephyr during boot
 */
static int mini_bootloader_init(void)
{
	uint32_t reset_cause_id = 0;

	hwinfo_get_reset_cause(&reset_cause_id);
	printk("RESET CAUSE: %i\n", reset_cause_id);
	/* Verify and initialize the Backup SRAM device */
	const struct device *const backup_memory = DEVICE_DT_GET_ONE(st_stm32_backup_sram);

	if (!device_is_ready(backup_memory)) {
		printk("ERROR: BackUp SRAM device is not ready\n");
		return 0; /* Return 0 to continue booting the main app anyway */
	}

	/* Check the contents of the backup register */
	if (backup.magic == BOOT_MAGIC_NUMBER) {
		/* * CONDITION MET: Board was reset within the 500ms window!
		 */
		printk("call JumpToBootloader\n");

		/* Clear the magic number so we don't get permanently stuck in bootloader mode */
		backup.magic = 0x00000000;

		/* Jump to ST System Memory */
		JumpToBootloader();
		while (1) {
			printk("this should never happen!!!");
			k_msleep(1000);
		}
	} else {
		/* * NORMAL BOOT: Write magic number and start the countdown
		 */
		printk("Starting 500ms boot window...\n");
		backup.magic = BOOT_MAGIC_NUMBER;

		/* * Wait 500ms. Because we are using the APPLICATION init level,
		 * the kernel is running and we can safely use k_msleep().
		 * If the user resets the board now, the backup RAM retains the magic number.
		 */
		k_msleep(500);

		/* Window expired without a reset. Clear magic number and continue normal boot. */
		backup.magic = 0x00000000;
		printk("Boot window expired. Continuing to application main()...\n");
	}

	/* Returning 0 automatically hands control back to Zephyr to finish booting and call main()
	 */
	return 0;
}

/* Register the function to run at the APPLICATION level */
SYS_INIT(mini_bootloader_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
