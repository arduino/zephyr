/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Flash Read/Write sample for Microchip NVMCTRL G2 (FCW) devices.
 *
 * This sample demonstrates the Zephyr Flash API by running a generic
 * erase/write/read/verify sequence on two different flash regions:
 *
 *   - PFM storage partition (flash1): safe area for runtime data.
 *   - BFM last page (flash0): upper boundary of Boot Flash Memory.
 *
 * The core test logic lives in a single reusable function
 * (flash_region_test) that receives the flash device, offset, length
 * and a human-readable region name.  All geometry (page size, write
 * block size) is obtained at runtime from the flash driver, so the
 * sample works across PIC32CK and PIC32CZ devices without
 * modification.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

/* ---- Partition configuration ---- */

#define PFM_PARTITION       storage_partition
#define PFM_DEVICE          PARTITION_DEVICE(PFM_PARTITION)
#define PFM_OFFSET          PARTITION_OFFSET(PFM_PARTITION)

#define BFM_PARTITION       boot_partition
#define BFM_DEVICE          PARTITION_DEVICE(BFM_PARTITION)
#define BFM_OFFSET          PARTITION_OFFSET(BFM_PARTITION)
#define BFM_SIZE            PARTITION_SIZE(BFM_PARTITION)

/*
 * Maximum test buffer size.  The actual test size is the smaller of
 * the requested length and this cap, to limit RAM usage.
 */
#define MAX_TEST_SIZE       4096U
#define MAX_BUFFER_WORDS    (MAX_TEST_SIZE / sizeof(uint32_t))

/* Buffers -- word-aligned for flash programming. */
static uint32_t write_buf[MAX_BUFFER_WORDS];
static uint32_t read_buf[MAX_BUFFER_WORDS];

/* Running test counter shared across all invocations. */
static unsigned int test_num;

/* ---- Helpers ---- */

/**
 * Fill @p buf with a simple pattern seeded by @p seed so that each
 * test region gets a unique, recognisable data pattern.
 */
static void populate_buffer(uint32_t *buf, uint32_t count, uint32_t seed)
{
	for (uint32_t i = 0; i < count; i++) {
		buf[i] = seed + i;
	}
}

/**
 * Check that a memory region is filled with a given byte value.
 * Returns 0 if all bytes match, -1 otherwise.
 */
static int check_erased(const uint8_t *buf, size_t len, uint8_t erase_val)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] != erase_val) {
			return -1;
		}
	}
	return 0;
}

/* ---- Generic flash region test ---- */

/**
 * Run a full erase / erase-verify / write / read / data-verify cycle
 * on an arbitrary flash region.
 *
 * @param flash_dev  Flash device (must be ready).
 * @param offset     Byte offset from the flash device base.
 * @param len        Number of bytes to test (capped to MAX_TEST_SIZE).
 *                   Must be aligned to the erase-block size.
 * @param name       Short human-readable region name for log messages.
 * @param seed       Pattern seed so each region gets unique data.
 *
 * @return 0 on success, negative errno on failure.
 */
static int flash_region_test(const struct device *flash_dev,
			     off_t offset, size_t len,
			     const char *name, uint32_t seed)
{
	const struct flash_parameters *fp;
	size_t write_block_size;
	size_t erase_block_size;
	size_t test_size;
	uint32_t buffer_words;
	int rc;

	printf("\n--- %s test ---\n\n", name);

	/* --- Validate device --- */
	if (!device_is_ready(flash_dev)) {
		printf("%s: flash device not ready!\n", name);
		return -ENODEV;
	}

	/* --- Query flash parameters --- */
	fp = flash_get_parameters(flash_dev);
	write_block_size = fp->write_block_size;

	printf("%s: flash device is ready.\n", name);
	printf("%s parameters:\n", name);
	printf("  Write block size : %zu bytes\n", write_block_size);
	printf("  Erase value      : 0x%02x\n", fp->erase_value);

#ifdef CONFIG_FLASH_PAGE_LAYOUT
	struct flash_pages_info page_info;

	rc = flash_get_page_info_by_offs(flash_dev, offset, &page_info);
	if (rc != 0) {
		printf("  Failed to get page info (rc=%d)\n", rc);
		return rc;
	}
	erase_block_size = page_info.size;
	printf("  Erase block size : %zu bytes\n", erase_block_size);
	printf("  Page start offset: 0x%lx\n", (unsigned long)page_info.start_offset);
	printf("  Page index       : %" PRIu32 "\n", page_info.index);
	printf("  Total pages      : %zu\n", flash_get_page_count(flash_dev));
#else
	erase_block_size = 4096U;
	printf("  Erase block size : %zu bytes (default)\n", erase_block_size);
#endif

	printf("  Test offset      : 0x%lx\n", (unsigned long)offset);

	/* Cap test size to buffer capacity and requested length. */
	test_size = (len <= MAX_TEST_SIZE) ? len : MAX_TEST_SIZE;
	test_size = (test_size <= erase_block_size) ? test_size : erase_block_size;
	buffer_words = test_size / sizeof(uint32_t);

	printf("  Test size        : %zu bytes\n", test_size);

	/* --- Erase --- */
	test_num++;
	printf("\nTest %u: %s erase\n", test_num, name);
	printf("  Erasing %zu bytes at offset 0x%lx ...\n",
	       erase_block_size, (unsigned long)offset);

	rc = flash_erase(flash_dev, offset, erase_block_size);
	if (rc != 0) {
		printf("  %s erase FAILED (rc=%d)\n", name, rc);
		return rc;
	}
	printf("  %s erase succeeded!\n", name);

	/* --- Erase verification --- */
	test_num++;
	printf("\nTest %u: %s erase verification\n", test_num, name);

	rc = flash_read(flash_dev, offset, read_buf, test_size);
	if (rc != 0) {
		printf("  %s read after erase FAILED (rc=%d)\n", name, rc);
		return rc;
	}

	if (check_erased((const uint8_t *)read_buf, test_size,
			 fp->erase_value) != 0) {
		printf("  %s erase verification FAILED: region not blank\n",
		       name);
		return -EIO;
	}
	printf("  %s erase verification succeeded!\n", name);

	/* --- Write --- */
	populate_buffer(write_buf, buffer_words, seed);

	test_num++;
	printf("\nTest %u: %s write\n", test_num, name);
	printf("  Writing %zu bytes (write block = %zu bytes) ...\n",
	       test_size, write_block_size);

	rc = flash_write(flash_dev, offset, write_buf, test_size);
	if (rc != 0) {
		printf("  %s write FAILED (rc=%d)\n", name, rc);
		return rc;
	}
	printf("  %s write succeeded!\n", name);

	/* --- Read back --- */
	test_num++;
	printf("\nTest %u: %s read\n", test_num, name);
	printf("  Reading back %zu bytes ...\n", test_size);

	memset(read_buf, 0, test_size);
	rc = flash_read(flash_dev, offset, read_buf, test_size);
	if (rc != 0) {
		printf("  %s read FAILED (rc=%d)\n", name, rc);
		return rc;
	}
	printf("  %s read succeeded!\n", name);

	/* --- Data verification --- */
	test_num++;
	printf("\nTest %u: %s data verification\n", test_num, name);

	if (memcmp(write_buf, read_buf, test_size) == 0) {
		printf("  %s data read matches data written. Good!!\n", name);
		return 0;
	}

	/* Find first mismatch for debug output. */
	for (uint32_t i = 0; i < buffer_words; i++) {
		if (write_buf[i] != read_buf[i]) {
			printf("  %s VERIFY FAILED at word %u: "
			       "wrote 0x%08x, read 0x%08x\n",
			       name, i, write_buf[i], read_buf[i]);
			break;
		}
	}
	return -EIO;
}

/* ---- Main ---- */

int main(void)
{
	int ret;

	printf("\n");
	printf("Flash Read/Write sample for %s\n", CONFIG_BOARD);
	printf("==========================\n");

	/*
	 * PFM storage partition (flash1).
	 * Uses the storage_partition offset defined in the board DTS.
	 */
	ret = flash_region_test(PFM_DEVICE,
				PFM_OFFSET,
				MAX_TEST_SIZE,
				"PFM storage",
				0x00000000U);
	if (ret != 0) {
		printf("\n==========================\n");
		printf("Flash test FAILED (rc=%d).\n", ret);
		return 0;
	}

	/*
	 * BFM last page (flash0).
	 * Compute the offset of the last erase-page within the
	 * boot_partition so the test exercises the upper address
	 * boundary of the Boot Flash Memory.
	 */
	{
		size_t erase_blk = 4096U; /* updated below if page layout available */

#ifdef CONFIG_FLASH_PAGE_LAYOUT
		struct flash_pages_info pi;

		if (flash_get_page_info_by_offs(BFM_DEVICE, BFM_OFFSET, &pi) == 0) {
			erase_blk = pi.size;
		}
#endif
		off_t bfm_last_page = (off_t)(BFM_OFFSET + BFM_SIZE - erase_blk);

		ret = flash_region_test(BFM_DEVICE,
					bfm_last_page,
					erase_blk,
					"BFM top",
					0xFFFF0000U);
	}

	printf("\n==========================\n");
	if (ret == 0) {
		printf("All flash tests PASSED.\n");
	} else {
		printf("Flash test FAILED (rc=%d).\n", ret);
	}

	return 0;
}
