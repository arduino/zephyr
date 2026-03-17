/*
 * Copyright (c) 2024 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARC custom LLEXT settings
 *
 * This file contains custom definition for ARC boards running LLEXT
 *
 */

#ifndef ZEPHYR_ARCH_ARC_INCLUDE_LLEXT_ARCH_CUSTOM_H_
#define ZEPHYR_ARCH_ARC_INCLUDE_LLEXT_ARCH_CUSTOM_H_

#ifndef LLEXT_INSTR_HEAP_SECTION
#define LLEXT_INSTR_HEAP_SECTION Z_GENERIC_DOT_SECTION(".rodata.llext_instr_heap")
#endif /* LLEXT_INSTR_HEAP_SECTION */

#ifndef LLEXT_DATA_HEAP_SECTION
#define LLEXT_DATA_HEAP_SECTION Z_GENERIC_DOT_SECTION(".data.llext_data_heap")
#endif /* LLEXT_DATA_HEAP_SECTION */

#endif /* ZEPHYR_ARCH_ARC_INCLUDE_LLEXT_ARCH_CUSTOM_H_ */
