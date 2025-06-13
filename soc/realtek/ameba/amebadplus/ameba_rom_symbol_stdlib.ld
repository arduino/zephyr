/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

SECTIONS
{
    PROVIDE(__rom_stdlib_text_start__ = 0x33000);
    PROVIDE(stdlib_loguart_relocation = 0x337bd);
    PROVIDE(_atoi = 0x337c9);
    PROVIDE(_stratoi = 0x337c9);
    PROVIDE(__wrap_atoi = 0x337c9);
    PROVIDE(__wrap_stratoi = 0x337c9);
    PROVIDE(__wrap_printf = 0x337f5);
    PROVIDE(_printf = 0x337f5);
    PROVIDE(__wrap_putchar = 0x3381f);
    PROVIDE(_putchar = 0x3381f);
    PROVIDE(__wrap_puts = 0x33935);
    PROVIDE(_puts = 0x33935);
    PROVIDE(__wrap_snprintf = 0x339a3);
    PROVIDE(_snprintf = 0x339a3);
    PROVIDE(__wrap_sprintf = 0x33a41);
    PROVIDE(_sprintf = 0x33a41);
    PROVIDE(__wrap_sscanf = 0x33a81);
    PROVIDE(_sscanf = 0x33a81);
    PROVIDE(__wrap_strcat = 0x33bb7);
    PROVIDE(_strcat = 0x33bb7);
    PROVIDE(__wrap_strchr = 0x33be9);
    PROVIDE(_strchr = 0x33be9);
    PROVIDE(__wrap_strcpy = 0x33c75);
    PROVIDE(_strcpy = 0x33c75);
    PROVIDE(__wrap_strncat = 0x33d8d);
    PROVIDE(_strncat = 0x33d8d);
    PROVIDE(__wrap_strncmp = 0x33dcf);
    PROVIDE(_strncmp = 0x33dcf);
    PROVIDE(__wrap_strncpy = 0x33e2b);
    PROVIDE(_strncpy = 0x33e2b);
    PROVIDE(__wrap_strpbrk = 0x33e7f);
    PROVIDE(_strpbrk = 0x33e7f);
    PROVIDE(__wrap_strsep = 0x33eb3);
    PROVIDE(_strsep = 0x33eb3);
    PROVIDE(__wrap_strstr = 0x340d7);
    PROVIDE(_strstr = 0x340d7);
    PROVIDE(__wrap_strtok = 0x34215);
    PROVIDE(_strtok = 0x34215);
    PROVIDE(__wrap_strtol = 0x34395);
    PROVIDE(_strtol = 0x34395);
    PROVIDE(__wrap_vsnprintf = 0x37fd7);
    PROVIDE(_vsnprintf = 0x37fd7);
    PROVIDE(__rom_stdlib_text_end__ = 0x40a40);
    PROVIDE(__rom_stdlib_data_start__ = 0x20006000);
    PROVIDE(__rom_stdlib_data_end__ = 0x200069ae);
    PROVIDE(__rom_stdlib_bss_start__ = 0x200069c0);
    PROVIDE(rom_customer_log = 0x200069c0);
    PROVIDE(__rom_stdlib_bss_end__ = 0x20006a14);
    PROVIDE(__rom_stdlib_heap_start__ = 0x20006a20);
}
