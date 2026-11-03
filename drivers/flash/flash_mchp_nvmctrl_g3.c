/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_nvmctrl_g3

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/clock.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/drivers/flash/mchp_flash.h>

LOG_MODULE_REGISTER(flash_mchp_nvmctrl_g3, CONFIG_FLASH_LOG_LEVEL);

#define SOC_NV_FLASH_COMPAT(node_id)                                                               \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, soc_nv_flash), (node_id), ())

/* The single soc-nv-flash child of controller n, or empty if it has none */
#define SOC_NV_FLASH_NODE(n)                                                                       \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, SOC_NV_FLASH_COMPAT)

#define FLASH_BANK_BASE_ADDR(node)            DT_REG_ADDR(node)
#define FLASH_BANK_SIZE(node)                 DT_REG_SIZE(node)
#define FLASH_BANK_WRITE_BLOCK_SIZE(node)     DT_PROP(node, write_block_size)
#define FLASH_BANK_WRITE_BLOCK_SIZE_QDW(node) DT_PROP(node, write_block_size_quad_dw)
#define FLASH_BANK_WRITE_BLOCK_SIZE_ROW(node) DT_PROP(node, write_block_size_row)
#define FLASH_BANK_ERASE_BLOCK_SIZE(node)     DT_PROP(node, erase_block_size)

#define FLASH_BANK_UNIMPLEMENTED_REGION_COUNT(node)                                                \
	COND_CODE_1(DT_NODE_HAS_PROP(node, unimplemented_region),                                  \
		    (DT_PROP_LEN(node, unimplemented_region) / 2), (0))

/*
 * Optional APB clocks: PIC32CK SG/GC and PIC32CZ CA have dedicated FCW and FCR
 * APB clocks (clock-names entries "mclk-fcw-apb" and "mclk-fcr-apb"), while
 * PIC32CM SG/GC only has AHB clocks.  Detect at build time via DTS
 * clock-names presence.
 */
#define NVMCTRL_HAS_FCW_APB_CLOCK DT_INST_CLOCKS_HAS_NAME(0, mclk_fcw_apb)
#define NVMCTRL_HAS_FCR_APB_CLOCK DT_INST_CLOCKS_HAS_NAME(0, mclk_fcr_apb)

#define TIMEOUT_VALUE_US     100000U
#define TIMEOUT_DONE_WAIT_MS 50U
#define DELAY_US             2U

#define IS_MCHP_FLASH_G3_ALIGNED(value, alignment) (((value) & ((alignment) - 1)) == 0)

enum fcw_operation_mode {
	PROGRAM_ERASE_OPERATION = 0x7U,
	PAGE_ERASE_OPERATION = 0x4U,
	ROW_PROGRAM_OPERATION = 0x3U,
	QUAD_DOUBLE_WORD_PROGRAM_OPERATION = 0x2U,
	SINGLE_DOUBLE_WORD_PROGRAM_OPERATION = 0x1U,
	NO_OPERATION = 0x0U,
};

enum fcw_unlock_key {
	FCW_UNLOCK_WRKEY = 0x91C32C01U,
	FCW_UNLOCK_SWAPKEY = 0x91C32C02U,
	FCW_UNLOCK_CFGKEY = 0x91C32C04U,
};

enum flash_operation {
	OPERATION_READ = 0U,
	OPERATION_WRITE = 1U,
	OPERATION_ERASE = 2U,
};

struct flash_mchp_g3_clock {
	clock_control_subsys_t mclk_ahb; /* AHB clock control subsystem */
	clock_control_subsys_t mclk_apb; /* APB clock control subsystem */
};

struct nvmctrl_data {
	struct k_sem fcw_sem_lock;  /* Semaphore used for locking mechanism */
	struct k_sem done_flag_sem; /* Semaphore used for synching flash write done */
	uint32_t interrupt_status;  /* Flash controller interrupt status flags */
};

/* Flash controller (NVMCTRL) configuration, shared by all banks it drives */
struct nvmctrl_config {
	const struct device *clock_dev;                     /* Clock driver device */
	void (*irq_config_func)(const struct nvmctrl_config *config); /* IRQ Function ptr */
	struct nvmctrl_data *data;                          /* Controller run-time state */
	struct fcw_config {
		fcw_registers_t *regs;            /* Pointer to Flash peripheral registers */
		struct flash_mchp_g3_clock clock; /* Flash clock control configuration */
	} fcw;

	struct fcr_config {
		fcr_registers_t *regs;            /* Pointer to Flash peripheral registers */
		struct flash_mchp_g3_clock clock; /* Flash clock control configuration */
	} fcr;
};

/*
 * Flash bank configuration. A controller with a single soc-nv-flash child
 * exposes that bank as the controller device itself; a controller with
 * several microchip,flash-g3 children (PIC32CK: BFM and PFM) exposes each of
 * them as its own flash device, all sharing the controller.
 */
struct flash_mchp_g3_bank_config {
	const struct nvmctrl_config *nvmctrl;
	struct flash_parameters parameters;
#if CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_layout page_layout;
#endif
	uint32_t base_addr;
	uint32_t size;
	uint32_t write_block_size;
	uint32_t write_block_size_qdw;
	uint32_t write_block_size_row;
	uint32_t erase_block_size;
	uint32_t unimplemented_region_count;
	const uint32_t *unimplemented_regions;
};

static inline void fcw_ready_wait(fcw_registers_t *regs)
{
	if (WAIT_FOR(((regs->FCW_STATUS & FCW_STATUS_BUSY_Msk) == 0), TIMEOUT_VALUE_US,
		     k_busy_wait(DELAY_US)) == false) {
		LOG_WRN("FCW_STATUS_BUSY wait timed out");
	}
}

static inline void fcr_ready_wait(fcr_registers_t *regs)
{
	if (WAIT_FOR(((regs->FCR_STATUS & FCR_STATUS_PRM_Msk) == 0), TIMEOUT_VALUE_US,
		     k_busy_wait(DELAY_US)) == false) {
		LOG_WRN("FCR_STATUS_PRM wait timed out");
	}
}

static void enable_fcw_interrupts(fcw_registers_t *regs)
{
	regs->FCW_INTENSET |=
		(FCW_INTENSET_KEYERR_Msk | FCW_INTENSET_CFGERR_Msk | FCW_INTENSET_FIFOERR_Msk |
		 FCW_INTENSET_BUSERR_Msk | FCW_INTENSET_WPERR_Msk | FCW_INTENSET_OPERR_Msk |
#ifdef FCW_INTENSET_SECERR_Msk
		 FCW_INTENSET_SECERR_Msk |
#endif
#ifdef FCW_INTENSET_HTDPGM_Msk
		 FCW_INTENSET_HTDPGM_Msk |
#endif
#ifdef FCW_INTENSET_BORERR_Msk
		 FCW_INTENSET_BORERR_Msk |
#endif
#ifdef FCW_INTENSET_RSTERR_Msk
		 FCW_INTENSET_RSTERR_Msk |
#endif
		 FCW_INTENSET_WRERR_Msk |
		 FCW_INTENSET_DONE_Msk);
}

static void disable_fcr_interrupts(fcr_registers_t *regs)
{
	regs->FCR_INTENCLR = FCR_INTENCLR_Msk;
}

static void clear_fcw_error_status(fcw_registers_t *regs, uint32_t error_mask)
{
	regs->FCW_INTFLAG |= error_mask;
}

static void clear_fcr_error_status(fcr_registers_t *regs, uint32_t error_mask)
{
	regs->FCR_INTFLAG |= error_mask;
}

static uint32_t get_fcw_interrupt_status_flags(fcw_registers_t *fcw_regs)
{
	uint32_t fcw_status =
		(fcw_regs->FCW_INTFLAG &
		 (FCW_INTFLAG_KEYERR_Msk | FCW_INTFLAG_CFGERR_Msk | FCW_INTFLAG_FIFOERR_Msk |
		  FCW_INTFLAG_BUSERR_Msk | FCW_INTFLAG_WPERR_Msk | FCW_INTFLAG_OPERR_Msk |
#ifdef FCW_INTFLAG_SECERR_Msk
		  FCW_INTFLAG_SECERR_Msk |
#endif
#ifdef FCW_INTFLAG_HTDPGM_Msk
		  FCW_INTFLAG_HTDPGM_Msk |
#endif
#ifdef FCW_INTFLAG_BORERR_Msk
		  FCW_INTFLAG_BORERR_Msk |
#endif
#ifdef FCW_INTFLAG_RSTERR_Msk
		  FCW_INTFLAG_RSTERR_Msk |
#endif
		  FCW_INTFLAG_WRERR_Msk |
		  FCW_INTFLAG_DONE_Msk));

	return fcw_status;
}

static int validate_flash_parameters(const struct flash_mchp_g3_bank_config *const config,
				     off_t offset,
				     size_t size, enum flash_operation op)
{
	if ((offset < 0) || (offset > config->size) || (size > config->size) ||
	    (offset > (config->size - size))) {
		LOG_WRN("Offset+Size is beyond flash addressable range."
			" Off: %#08x, Sz: %#08x, MaxSz: %#08x",
			(uint32_t)offset, size, config->size);
		return -EINVAL;
	}

	for (int i = 0; i < config->unimplemented_region_count; i++) {
		uint32_t unimp_region_start = config->unimplemented_regions[i * 2];
		uint32_t unimp_region_end = config->unimplemented_regions[(i * 2) + 1];

		if ((config->base_addr + offset + size) > unimp_region_start &&
		    (config->base_addr + offset + size) <= unimp_region_end) {
			LOG_WRN("Offset+Size lies in unimplemented flash range."
				" Off: %#08x, Sz: %#08x",
				(uint32_t)offset, size);
			return -EINVAL;
		}
	}

	switch (op) {
	case OPERATION_READ:
		break;
	case OPERATION_WRITE:
		if (!(IS_MCHP_FLASH_G3_ALIGNED(offset, config->write_block_size))) {
			LOG_WRN("WRITE: Offset should be multiples of write-block size."
				" Off: %#08x",
				(uint32_t)offset);
			return -EINVAL;
		}
		if (!(IS_MCHP_FLASH_G3_ALIGNED(size, config->write_block_size))) {
			LOG_WRN("WRITE: Size should be multiples of write-block size."
				" Sz: %#08x",
				size);
			return -EINVAL;
		}
		break;
	case OPERATION_ERASE:
		if (size < config->erase_block_size) {
			LOG_WRN("ERASE: Cannot erase less than the size of an erase-block."
				" Sz: %#08x",
				size);
			return -EINVAL;
		}
		if (!IS_MCHP_FLASH_G3_ALIGNED(size, config->erase_block_size)) {
			LOG_WRN("ERASE: Size should be multiples of erase-block size."
				" Sz: %#08x",
				size);
			return -EINVAL;
		}
		if (!IS_MCHP_FLASH_G3_ALIGNED(offset, config->erase_block_size)) {
			LOG_WRN("ERASE: Offset should be multiples of erase-block size."
				" Sz: %#08x",
				size);
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unsupported operation");
		return -EINVAL;
	}

	return 0;
}

static int exec_flash_operation(fcw_registers_t *regs, struct nvmctrl_data *nvmctr_data,
				uint32_t address, enum fcw_operation_mode operation)
{
	int wait_err;

	fcw_ready_wait(regs);
	regs->FCW_MUTEX = (FCW_MUTEX_LOCK(1) | FCW_MUTEX_OWNER(1));
	regs->FCW_ADDR = address;

	clear_fcw_error_status(regs, FCW_INTFLAG_Msk);
	nvmctr_data->interrupt_status = 0;

	regs->FCW_KEY = (uint32_t)FCW_UNLOCK_WRKEY;
	k_sem_init(&nvmctr_data->done_flag_sem, 0, 1);

#ifdef FCW_CTRLOP_NVMOP_Pos
	/* PIC32CM SG/GC uses CTRLOP register instead of CTRLA */
	regs->FCW_CTRLOP = FCW_CTRLOP_PREPG_Msk | FCW_CTRLOP_NVMOP(operation);
#else
	regs->FCW_CTRLA = FCW_CTRLA_PREPG_Msk | FCW_CTRLA_NVMOP(operation);
#endif

	wait_err = k_sem_take(&nvmctr_data->done_flag_sem, K_MSEC(TIMEOUT_DONE_WAIT_MS));
	if (wait_err != 0) {
		LOG_ERR("Error waiting for Done flag");
		return -EIO;
	}

	fcw_ready_wait(regs);
	regs->FCW_MUTEX = (FCW_MUTEX_LOCK(0) | FCW_MUTEX_OWNER(0));

	nvmctr_data->interrupt_status &= ~FCW_INTENSET_DONE_Msk;
	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_KEYERR_Msk) != 0) {
		LOG_ERR("Key Error Flag Bit is set");
	}

	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_CFGERR_Msk) != 0) {
		LOG_ERR("Configuration Error Flag Bit is set");
	}

	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_FIFOERR_Msk) != 0) {
		LOG_ERR("FIFO Underrun During Row Write Flag Bit is set");
	}

	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_BUSERR_Msk) != 0) {
		LOG_ERR("AHB Bus Error During Row Write Flag Bit is set");
	}

	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_WPERR_Msk) != 0) {
		LOG_ERR("Write Protection Error Flag Bit is set");
	}

	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_OPERR_Msk) != 0) {
		LOG_ERR("NVMOP Error Flag Bit is set");
	}

#ifdef FCW_INTFLAG_SECERR_Msk
	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_SECERR_Msk) != 0) {
		LOG_ERR("Security Violation Error Bit is set");
	}
#endif
#ifdef FCW_INTFLAG_HTDPGM_Msk
	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_HTDPGM_Msk) != 0) {
		LOG_ERR("High Temperature Detect Error Flag Bit is set");
	}
#endif
#ifdef FCW_INTFLAG_BORERR_Msk
	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_BORERR_Msk) != 0) {
		LOG_ERR("Reset or Brown Out Detect Error Flag Bit is set");
	}
#endif
#ifdef FCW_INTFLAG_RSTERR_Msk
	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_RSTERR_Msk) != 0) {
		LOG_ERR("Reset During Program Error Flag Bit is set");
	}
#endif

	if ((nvmctr_data->interrupt_status & FCW_INTFLAG_WRERR_Msk) != 0) {
		LOG_ERR("Write Error Flag Bit is set");
	}

	return ((nvmctr_data->interrupt_status == 0) ? 0 : -EIO);
}

static int flash_mchp_read(const struct device *dev, off_t offset, void *data_buff,
			   size_t no_of_bytes)
{
	const struct flash_mchp_g3_bank_config *const config = dev->config;
	struct nvmctrl_data *data = config->nvmctrl->data;
	int ret = validate_flash_parameters(config, offset, no_of_bytes, OPERATION_READ);

	if (ret != 0) {
		LOG_ERR("Error occurred in validating Flash read arguments");
		return ret;
	}

	if (data_buff == NULL) {
		LOG_ERR("Data Buffer is NULL");
		return -EINVAL;
	}

	k_sem_take(&data->fcw_sem_lock, K_FOREVER);
	(void)memcpy(data_buff, (uint8_t *)(config->base_addr + offset), no_of_bytes);
	k_sem_give(&data->fcw_sem_lock);

	return 0;
}

/*
 * FCW alignment traps (neither is documented in DS60001795):
 *
 * 1. SRCADDR alignment: ROW_PROGRAM_OPERATION reads its source from SRAM via
 *    SRCADDR, but the hardware ignores SRCADDR[4:0] and silently rounds the
 *    source down to a 32-byte boundary, so a misaligned caller buffer is
 *    programmed shifted, with no error. Rows are therefore staged through a
 *    bounce buffer aligned to the row size.
 *
 * 2. FCW_ADDR alignment: QDW and ROW programming need a naturally aligned
 *    destination (32 B / 1 KB). A misaligned one silently corrupts data, so
 *    the write loop only picks QDW/ROW on aligned offsets and falls back to
 *    DW for the leading unaligned part.
 */
#define FLASH_MCHP_G3_ROW_BUF_SIZE 1024U

static uint8_t __aligned(FLASH_MCHP_G3_ROW_BUF_SIZE) row_bounce_buf[FLASH_MCHP_G3_ROW_BUF_SIZE];

static int flash_mchp_write(const struct device *dev, off_t offset, const void *data_buff,
			    size_t no_of_bytes)
{
	const struct flash_mchp_g3_bank_config *const config = dev->config;
	struct nvmctrl_data *data = config->nvmctrl->data;
	fcw_registers_t *regs = config->nvmctrl->fcw.regs;
	int nvm_error = -EIO;
	const uint8_t *src_data_buff_read_ptr = (const uint8_t *)data_buff;
	uint32_t word;

	if (no_of_bytes == 0) {
		return 0;
	}

	nvm_error = validate_flash_parameters(config, offset, no_of_bytes, OPERATION_WRITE);
	if (nvm_error != 0) {
		LOG_ERR("Error occurred in validating Flash write arguments");
		return nvm_error;
	}

	if (data_buff == NULL) {
		LOG_ERR("Data Buffer is NULL");
		return -EINVAL;
	}

	k_sem_take(&data->fcw_sem_lock, K_FOREVER);
	while (no_of_bytes > 0) {
		fcw_ready_wait(regs);
		LOG_DBG("Number of bytes yet to be written %u", no_of_bytes);

		if ((no_of_bytes >= config->write_block_size_row) &&
		    IS_MCHP_FLASH_G3_ALIGNED(offset, config->write_block_size_row)) {
			memcpy(row_bounce_buf, src_data_buff_read_ptr, config->write_block_size_row);
			regs->FCW_SRCADDR = (uint32_t)row_bounce_buf;
			LOG_DBG("Writing row-block at address %#08x",
				(uint32_t)(config->base_addr + offset));
			nvm_error = exec_flash_operation(regs, data, (config->base_addr + offset),
							 ROW_PROGRAM_OPERATION);
			if (nvm_error != 0) {
				LOG_ERR("Flash Write Operation Failed");
				break;
			}

			no_of_bytes -= config->write_block_size_row;
			offset += config->write_block_size_row;
			src_data_buff_read_ptr += config->write_block_size_row;
		} else if ((no_of_bytes >= config->write_block_size_qdw) &&
			   IS_MCHP_FLASH_G3_ALIGNED(offset, config->write_block_size_qdw)) {
			for (uint32_t i = 0; i < (config->write_block_size_qdw / sizeof(uint32_t));
			     i++) {
				memcpy(&word, src_data_buff_read_ptr + (i * sizeof(uint32_t)),
				       sizeof(uint32_t));
				regs->FCW_DATA[i] = word;
			}
			LOG_DBG("Writing quad-word (%u B) at address %#08x",
				config->write_block_size_qdw,
				(uint32_t)(config->base_addr + offset));
			nvm_error = exec_flash_operation(regs, data, (config->base_addr + offset),
							 QUAD_DOUBLE_WORD_PROGRAM_OPERATION);
			if (nvm_error != 0) {
				LOG_ERR("Flash Write Operation Failed");
				break;
			}

			no_of_bytes -= config->write_block_size_qdw;
			offset += config->write_block_size_qdw;
			src_data_buff_read_ptr += config->write_block_size_qdw;
		} else if (no_of_bytes >= config->write_block_size) {
			for (uint32_t i = 0; i < (config->write_block_size / sizeof(uint32_t));
			     i++) {
				memcpy(&word, src_data_buff_read_ptr + (i * sizeof(uint32_t)),
				       sizeof(uint32_t));
				regs->FCW_DATA[i] = word;
			}
			LOG_DBG("Writing double-word at address %#08x",
				(uint32_t)(config->base_addr + offset));
			nvm_error = exec_flash_operation(regs, data,
							 (uint32_t)(config->base_addr + offset),
							 SINGLE_DOUBLE_WORD_PROGRAM_OPERATION);
			if (nvm_error != 0) {
				LOG_ERR("Flash Write Operation Failed");
				break;
			}

			no_of_bytes -= config->write_block_size;
			offset += config->write_block_size;
			src_data_buff_read_ptr += config->write_block_size;
		} else {
			LOG_ERR("Alignment error - no_of_bytes is not aligned to write-block size "
				"%#08x",
				no_of_bytes);
			nvm_error = -EIO;
			break;
		}
	}

	k_sem_give(&data->fcw_sem_lock);

	return nvm_error;
}

static int flash_mchp_erase(const struct device *dev, off_t offset, size_t no_of_bytes)
{
	const struct flash_mchp_g3_bank_config *const config = dev->config;
	struct nvmctrl_data *data = config->nvmctrl->data;
	fcw_registers_t *regs = config->nvmctrl->fcw.regs;
	int nvm_error = -EIO;

	if (no_of_bytes == 0) {
		return 0;
	}

	nvm_error = validate_flash_parameters(config, offset, no_of_bytes, OPERATION_ERASE);
	if (nvm_error != 0) {
		LOG_ERR("Error occurred in validating Flash erase arguments");
		return nvm_error;
	}

	while (no_of_bytes > 0U) {
		LOG_DBG("Erasing block at address %#08x", (uint32_t)(config->base_addr + offset));
		k_sem_take(&data->fcw_sem_lock, K_FOREVER);
		nvm_error = exec_flash_operation(regs, data, (uint32_t)(config->base_addr + offset),
						 PAGE_ERASE_OPERATION);
		k_sem_give(&data->fcw_sem_lock);
		if (nvm_error != 0) {
			LOG_ERR("Error erasing the block. Err code: %d", nvm_error);
			break;
		}

		no_of_bytes -= config->erase_block_size;
		offset += config->erase_block_size;
	}

	return nvm_error;
}

#if CONFIG_FLASH_EX_OP_ENABLED
static int flash_mchp_ex_op(const struct device *dev, uint16_t opr_code, const uintptr_t exop_info,
			    void *out)
{
	ARG_UNUSED(out);
	const struct flash_mchp_g3_bank_config *const config = dev->config;
	struct nvmctrl_data *data = config->nvmctrl->data;
	fcw_registers_t *regs = config->nvmctrl->fcw.regs;
	struct pfm_exop *pfm_exop_data;
	struct bfm_exop *bfm_exop_data;
	uint8_t region;
	int nvm_err = -EINVAL;

	k_sem_take(&data->fcw_sem_lock, K_FOREVER);
	switch (opr_code) {
	case FLASH_EX_OP_PFM_REGION_WRITE_PROTECT_ENABLE:
		pfm_exop_data = (struct pfm_exop *)exop_info;
		fcw_ready_wait(regs);
		regs->FCW_KEY = FCW_UNLOCK_CFGKEY;
		region = (uint8_t)(pfm_exop_data->region);
		regs->FCW_PWP[region] = (FCW_PWP_PWPBASE(pfm_exop_data->base_addr) |
					 FCW_PWP_PWPSIZE(pfm_exop_data->region_size) |
					 FCW_PWP_PWPMIR(pfm_exop_data->mirror_enable));
		regs->FCW_PWP[region] |= FCW_PWP_PWPEN_Msk;
		nvm_err = 0;
		break;
	case FLASH_EX_OP_PFM_REGION_WRITE_PROTECT_DISABLE:
		pfm_exop_data = (struct pfm_exop *)(exop_info);
		fcw_ready_wait(regs);
		regs->FCW_KEY = FCW_UNLOCK_CFGKEY;
		region = (uint8_t)(pfm_exop_data->region);
		regs->FCW_PWP[region] = (FCW_PWP_PWPBASE(pfm_exop_data->base_addr) |
					 FCW_PWP_PWPSIZE(pfm_exop_data->region_size) |
					 FCW_PWP_PWPMIR(pfm_exop_data->mirror_enable));
		regs->FCW_PWP[region] |= FCW_PWP_PWPLOCK_Msk;
		nvm_err = 0;
		break;
	case FLASH_EX_OP_PFM_REGION_WRITE_PROTECT_LOCK:
		pfm_exop_data = (struct pfm_exop *)(exop_info);
		fcw_ready_wait(regs);
		regs->FCW_KEY = FCW_UNLOCK_CFGKEY;
		region = (uint8_t)(pfm_exop_data->region);
		regs->FCW_PWP[region] = (FCW_PWP_PWPBASE(pfm_exop_data->base_addr) |
					 FCW_PWP_PWPSIZE(pfm_exop_data->region_size) |
					 FCW_PWP_PWPMIR(pfm_exop_data->mirror_enable));
		regs->FCW_PWP[region] &= ~FCW_PWP_PWPEN_Msk;
		nvm_err = 0;
		break;
	case FLASH_EX_OP_BFM_WRITE_PROTECT_ENABLE:
		bfm_exop_data = (struct bfm_exop *)(exop_info);
		fcw_ready_wait(regs);
		regs->FCW_KEY = FCW_UNLOCK_CFGKEY;
		if (bfm_exop_data->boot_bank == BOOT_FLASH_BANK_1) {
			regs->FCW_LBWP |= bfm_exop_data->wp_page;
		} else {
			regs->FCW_UBWP |= bfm_exop_data->wp_page;
		}
		nvm_err = 0;
		break;
	case FLASH_EX_OP_BFM_WRITE_PROTECT_DISABLE:
		bfm_exop_data = (struct bfm_exop *)(exop_info);
		fcw_ready_wait(regs);
		regs->FCW_KEY = FCW_UNLOCK_CFGKEY;
		if (bfm_exop_data->boot_bank == BOOT_FLASH_BANK_1) {
			regs->FCW_LBWP &= ~bfm_exop_data->wp_page;
		} else {
			regs->FCW_UBWP &= ~bfm_exop_data->wp_page;
		}
		nvm_err = 0;
		break;
	case FLASH_EX_OP_BFM_WRITE_PROTECT_LOCK:
		bfm_exop_data = (struct bfm_exop *)(exop_info);
		fcw_ready_wait(regs);
		regs->FCW_KEY = FCW_UNLOCK_CFGKEY;
		if (bfm_exop_data->boot_bank == BOOT_FLASH_BANK_1) {
			regs->FCW_LBWP |= FCW_LBWP_LBWPLOCK_Msk;
		} else {
			regs->FCW_UBWP |= FCW_UBWP_UBWPLOCK_Msk;
		}
		nvm_err = 0;
		break;
	default:
		LOG_ERR("Unsupported operation");
		nvm_err = -EINVAL;
	}
	k_sem_give(&data->fcw_sem_lock);

	return nvm_err;
}
#endif /* CONFIG_FLASH_EX_OP_ENABLED */

static void nvmctrl_isr(const struct nvmctrl_config *const config)
{
	struct nvmctrl_data *data = config->data;

	data->interrupt_status = get_fcw_interrupt_status_flags(config->fcw.regs);
	clear_fcw_error_status(config->fcw.regs, data->interrupt_status);

	if ((data->interrupt_status & FCW_INTFLAG_DONE_Msk) == FCW_INTFLAG_DONE_Msk) {
		k_sem_give(&data->done_flag_sem);
	}
}

static int nvmctrl_init(const struct nvmctrl_config *const config)
{
	struct nvmctrl_data *data = config->data;
	int nvm_err;

	nvm_err = clock_control_on(config->clock_dev, config->fcw.clock.mclk_ahb);
	if ((nvm_err != 0) && (nvm_err != -EALREADY)) {
		LOG_ERR("Failed to enable FCW AHB Clock: %d", nvm_err);
		return nvm_err;
	}

#if NVMCTRL_HAS_FCW_APB_CLOCK
	/* FCW APB clock exists on PIC32CK SG/GC and PIC32CZ CA but not on PIC32CM SG/GC */
	nvm_err = clock_control_on(config->clock_dev, config->fcw.clock.mclk_apb);
	if ((nvm_err != 0) && (nvm_err != -EALREADY)) {
		LOG_ERR("Failed to enable FCW APB Clock: %d", nvm_err);
		return nvm_err;
	}
#endif

	nvm_err = clock_control_on(config->clock_dev, config->fcr.clock.mclk_ahb);
	if ((nvm_err != 0) && (nvm_err != -EALREADY)) {
		LOG_ERR("Failed to enable FCR AHB Clock: %d", nvm_err);
		return nvm_err;
	}

#if NVMCTRL_HAS_FCR_APB_CLOCK
	/* FCR APB clock exists on PIC32CK SG/GC and PIC32CZ CA but not on PIC32CM SG/GC */
	nvm_err = clock_control_on(config->clock_dev, config->fcr.clock.mclk_apb);
	if ((nvm_err != 0) && (nvm_err != -EALREADY)) {
		LOG_ERR("Failed to enable FCR APB Clock: %d", nvm_err);
		return nvm_err;
	}
#endif

	k_sem_init(&data->fcw_sem_lock, 1, 1);
	k_sem_init(&data->done_flag_sem, 0, 1);
	fcw_ready_wait(config->fcw.regs);
	fcr_ready_wait(config->fcr.regs);
	config->irq_config_func(config);
	enable_fcw_interrupts(config->fcw.regs);
	disable_fcr_interrupts(config->fcr.regs);
	clear_fcw_error_status(config->fcw.regs, FCW_INTFLAG_Msk);
	clear_fcr_error_status(config->fcr.regs, FCR_INTFLAG_Msk);

	nvm_err = exec_flash_operation(config->fcw.regs, data, config->fcw.regs->FCW_ADDR,
				       NO_OPERATION);
	if (nvm_err != 0) {
		LOG_ERR("Failed to Initialize NVMCTRL. Err: %d", nvm_err);
	}

	return nvm_err;
}

#if CONFIG_FLASH_PAGE_LAYOUT
#define FLASH_MCHP_G3_CONFIG_PAGE_LAYOUT(node)                                                     \
	.page_layout.pages_count = FLASH_BANK_SIZE(node) / FLASH_BANK_ERASE_BLOCK_SIZE(node),      \
	.page_layout.pages_size = FLASH_BANK_ERASE_BLOCK_SIZE(node),
#else
#define FLASH_MCHP_G3_CONFIG_PAGE_LAYOUT(node)
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if CONFIG_FLASH_PAGE_LAYOUT
void flash_mchp_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
			    size_t *layout_size)
{
	const struct flash_mchp_g3_bank_config *const config = dev->config;

	*layout = &config->page_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *flash_mchp_get_parameters(const struct device *dev)
{
	const struct flash_mchp_g3_bank_config *const config = dev->config;

	return &config->parameters;
}

static DEVICE_API(flash, flash_mchp_g3_driver_api) = {
	.read = flash_mchp_read,
	.write = flash_mchp_write,
	.erase = flash_mchp_erase,
	.get_parameters = flash_mchp_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mchp_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_mchp_ex_op,
#endif /*CONFIG_FLASH_EX_OP_ENABLED*/
};

static int __maybe_unused nvmctrl_bank_dev_init(const struct device *dev)
{
	const struct flash_mchp_g3_bank_config *const config = dev->config;

	return nvmctrl_init(config->nvmctrl);
}

static int __maybe_unused nvmctrl_ctrl_dev_init(const struct device *dev)
{
	return nvmctrl_init(dev->config);
}

#define NVMCTRL_CONFIG_NAME(ctrl_node) _CONCAT(nvmctrl_g3_config_, DT_DEP_ORD(ctrl_node))

/* Flash bank configuration for the flash node 'node' of controller 'ctrl_node' */
#define FLASH_MCHP_G3_BANK_CONFIG_DEFINE(node, ctrl_node)                                          \
	BUILD_ASSERT(FLASH_BANK_WRITE_BLOCK_SIZE_ROW(node) <= FLASH_MCHP_G3_ROW_BUF_SIZE,          \
		     "row size exceeds the row bounce buffer");                                 \
                                                                                                   \
	IF_ENABLED(DT_NODE_HAS_PROP(node, unimplemented_region),                                   \
		   (static const uint32_t _CONCAT(dt_unimplemented_regions_, DT_DEP_ORD(node))[] = \
			    DT_PROP(node, unimplemented_region);))                                 \
                                                                                                   \
	static const struct flash_mchp_g3_bank_config _CONCAT(flash_mchp_g3_bank_,                 \
							      DT_DEP_ORD(node)) = {                \
		.nvmctrl = &NVMCTRL_CONFIG_NAME(ctrl_node),                                        \
		.parameters =                                                                      \
			{                                                                          \
				.write_block_size = FLASH_BANK_WRITE_BLOCK_SIZE(node),             \
				.erase_value = 0xff,                                               \
			},                                                                         \
		.base_addr = FLASH_BANK_BASE_ADDR(node),                                           \
		.size = FLASH_BANK_SIZE(node),                                                     \
		.write_block_size = FLASH_BANK_WRITE_BLOCK_SIZE(node),                             \
		.erase_block_size = FLASH_BANK_ERASE_BLOCK_SIZE(node),                             \
		.write_block_size_qdw = FLASH_BANK_WRITE_BLOCK_SIZE_QDW(node),                     \
		.write_block_size_row = FLASH_BANK_WRITE_BLOCK_SIZE_ROW(node),                     \
		.unimplemented_region_count = FLASH_BANK_UNIMPLEMENTED_REGION_COUNT(node),         \
		.unimplemented_regions = COND_CODE_1(DT_NODE_HAS_PROP(node, unimplemented_region), \
			(_CONCAT(dt_unimplemented_regions_, DT_DEP_ORD(node))), (NULL)),           \
		FLASH_MCHP_G3_CONFIG_PAGE_LAYOUT(node)};

/* A microchip,flash-g3 child that is not soc-nv-flash is a flash device of its own */
#define FLASH_MCHP_G3_BANK_DEVICE_DEFINE(node)                                                     \
	FLASH_MCHP_G3_BANK_CONFIG_DEFINE(node, DT_PARENT(node))                                    \
	DEVICE_DT_DEFINE(node, NULL, NULL, NULL, &_CONCAT(flash_mchp_g3_bank_, DT_DEP_ORD(node)),  \
			 POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &flash_mchp_g3_driver_api);

#define FLASH_MCHP_G3_CHILD_BANK(node)                                                             \
	COND_CODE_1(UTIL_AND(DT_NODE_HAS_COMPAT(node, microchip_flash_g3),                         \
			     UTIL_NOT(DT_NODE_HAS_COMPAT(node, soc_nv_flash))),                    \
		    (FLASH_MCHP_G3_BANK_DEVICE_DEFINE(node)), ())

/* Single soc-nv-flash child: the controller device is the flash device */
#define MCHP_NVMCTRL_G3_SINGLE_BANK(n)                                                             \
	FLASH_MCHP_G3_BANK_CONFIG_DEFINE(SOC_NV_FLASH_NODE(n), DT_DRV_INST(n))                     \
	DEVICE_DT_INST_DEFINE(n, nvmctrl_bank_dev_init, NULL, &nvmctrl_g3_data##n,                 \
			      &_CONCAT(flash_mchp_g3_bank_, DT_DEP_ORD(SOC_NV_FLASH_NODE(n))),     \
			      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &flash_mchp_g3_driver_api);

/*
 * Several microchip,flash-g3 children: the controller is set up first and
 * each child bank is a flash device of its own. The banks have no init of
 * their own and are only used after the controller has initialized.
 */
#define MCHP_NVMCTRL_G3_MULTI_BANK(n)                                                              \
	DEVICE_DT_INST_DEFINE(n, nvmctrl_ctrl_dev_init, NULL, &nvmctrl_g3_data##n,                 \
			      &NVMCTRL_CONFIG_NAME(DT_DRV_INST(n)), POST_KERNEL,                 \
			      CONFIG_FLASH_INIT_PRIORITY, NULL);                                    \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, FLASH_MCHP_G3_CHILD_BANK)

#define MCHP_NVMCTRL_G3_INIT(n)                                                                    \
	static const struct nvmctrl_config NVMCTRL_CONFIG_NAME(DT_DRV_INST(n));                    \
	static void nvmctrl_g3_irq_config##n(const struct nvmctrl_config *config)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, fcw, irq),                                      \
			    DT_INST_IRQ_BY_NAME(n, fcw, priority), nvmctrl_isr,                    \
			    &NVMCTRL_CONFIG_NAME(DT_DRV_INST(n)), 0);                              \
		irq_enable(DT_INST_IRQ_BY_NAME(n, fcw, irq));                                      \
	}                                                                                          \
                                                                                                   \
	static struct nvmctrl_data nvmctrl_g3_data##n;                                             \
                                                                                                   \
	static const struct nvmctrl_config NVMCTRL_CONFIG_NAME(DT_DRV_INST(n)) = {                 \
		.clock_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, clock_parent)),                      \
		.irq_config_func = nvmctrl_g3_irq_config##n,                                       \
		.data = &nvmctrl_g3_data##n,                                                       \
                                                                                                   \
		.fcw.regs = (fcw_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, fcw),                   \
		.fcw.clock.mclk_ahb =                                                              \
			(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_fcw_ahb, subsystem)),         \
		IF_ENABLED(NVMCTRL_HAS_FCW_APB_CLOCK,                                              \
			(.fcw.clock.mclk_apb =                                                     \
				(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_fcw_apb,              \
								     subsystem)),))         \
                                                                                                   \
		.fcr.regs = (fcr_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, fcr),                   \
		.fcr.clock.mclk_ahb =                                                              \
			(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_fcr_ahb, subsystem)),         \
		IF_ENABLED(NVMCTRL_HAS_FCR_APB_CLOCK,                                              \
			(.fcr.clock.mclk_apb =                                                     \
				(void *)(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk_fcr_apb,              \
								     subsystem)),))         \
	};                                                                                         \
                                                                                                   \
	COND_CODE_1(IS_EMPTY(SOC_NV_FLASH_NODE(n)), (MCHP_NVMCTRL_G3_MULTI_BANK(n)),               \
		    (MCHP_NVMCTRL_G3_SINGLE_BANK(n)))

DT_INST_FOREACH_STATUS_OKAY(MCHP_NVMCTRL_G3_INIT)
