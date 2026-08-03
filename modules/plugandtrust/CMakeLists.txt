# SPDX-License-Identifier: Apache-2.0

if(CONFIG_PLUGANDTRUST)
  message(STATUS "Building Plug And Trust")

  FILE(
    GLOB
    plugandtrust_sources
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/se05x_APDU_impl.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/smCom.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/se05x_tlv.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/scp03/se05x_scp03.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/eckey/se05x_ec_key_auth.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/scp03/se05x_auth_utils.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/scp03/tc/se05x_scp03_crypto_tc.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/t1oi2c/phNxpEse_Api.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/t1oi2c/phNxpEsePal_i2c.c
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/t1oi2c/phNxpEseProto7816_3.c
    sm_timer.c
  )

  zephyr_include_directories(
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/apdu/scp03
    ${ZEPHYR_CURRENT_MODULE_DIR}/lib/t1oi2c
    .
  )

  zephyr_library()
  zephyr_library_sources(${plugandtrust_sources})

  add_definitions(-DT1oI2C)
  add_definitions(-DT1oI2C_UM11225)

  if(CONFIG_PLUGANDTRUST_SCP03)
    add_definitions(-DWITH_PLATFORM_SCP03)
  endif()

  if(CONFIG_PLUGANDTRUST_ECKEY)
    add_definitions(-DWITH_ECKEY_SESSION)
  endif()

  if(CONFIG_PLUGANDTRUST_ECKEY_SCP03)
    add_definitions(-DWITH_ECKEY_SCP03_SESSION)
  endif()

  if(CONFIG_PLUGANDTRUST_SM_APDU_MUTEX)
     add_definitions(-DENABLE_SM_APDU_MUTEX)
  endif()
endif()
