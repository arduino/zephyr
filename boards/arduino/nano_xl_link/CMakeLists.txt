# Copyright (c) Arduino s.r.l. and/or its affiliated companies
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MCUBOOT AND CONFIG_RETENTION_BOOT_MODE)
  zephyr_library()
  zephyr_library_sources(double-tap.c)
endif()
