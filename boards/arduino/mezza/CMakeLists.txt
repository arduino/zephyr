if(CONFIG_BOARD_ARDUINO_MEZZA AND CONFIG_MCUBOOT)
  zephyr_library()
  zephyr_library_sources(double-tap.c)
endif()
