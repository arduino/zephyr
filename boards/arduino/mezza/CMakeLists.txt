if(CONFIG_BOARD_ARDUINO_MEZZA)
  zephyr_library()
  zephyr_library_sources(mini-bootloader.c)
endif()
