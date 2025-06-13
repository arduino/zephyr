/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_crypto

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/crypto/crypto.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto_ameba);

#if DT_HAS_COMPAT_STATUS_OKAY(realtek_ameba_crypto)
#else
#error No AMEBA HW Crypto Accelerator in device tree
#endif

#define CRYP_SUPPORT    (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)
#define BLOCK_LEN_BYTES 16
#define BLOCK_LEN_WORDS (BLOCK_LEN_BYTES / sizeof(uint32_t))

struct crypto_ameba_config {
	int ameba_todo;
};

struct crypto_ameba_data {
	struct k_sem device_sem;
	struct k_sem session_sem;
};

static int crypto_ameba_session_setup(const struct device *dev, struct cipher_ctx *ctx,
				      enum cipher_algo algo, enum cipher_mode mode,
				      enum cipher_op op_type)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);
	ARG_UNUSED(algo);
	ARG_UNUSED(op_type);

	/* The CRYP peripheral supports the AES ECB, CBC, CTR, CCM and GCM
	 * modes of operation, of which ECB, CBC, CTR and CCM are supported
	 * through the crypto API. However, in CCM mode, although the AMEBACube
	 * HAL driver follows the documentation (cf. RM0090, par. 23.3) by
	 * padding incomplete input data blocks in software prior encryption,
	 * incorrect authentication tags are returned for input data which is
	 * not a multiple of 128 bits. Therefore, CCM mode is not supported by
	 * this driver.
	 */
	if ((mode != CRYPTO_CIPHER_MODE_ECB) && (mode != CRYPTO_CIPHER_MODE_CBC) &&
	    (mode != CRYPTO_CIPHER_MODE_CTR)) {
		LOG_ERR("Unsupported mode");
		return -EINVAL;
	}

	return 0;
}

static int crypto_ameba_session_free(const struct device *dev, struct cipher_ctx *ctx)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);

	/*
	 * k_sem_take(&data->session_sem, K_FOREVER);
	 * k_sem_give(&data->session_sem);
	 */

	return 0;
}

static int crypto_ameba_query_caps(const struct device *dev)
{
	return CRYP_SUPPORT;
}

static int crypto_ameba_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	/*
	 * const struct device *const clk = DEVICE_DT_GET(AMEBA_CLOCK_CONTROL_NODE);
	 * struct crypto_ameba_data *data = CRYPTO_AMEBA_DATA(dev);
	 * const struct crypto_ameba_config *cfg = CRYPTO_AMEBA_CFG(dev);

	 * if (!device_is_ready(clk)) {
	 *     LOG_ERR("clock control device not ready");
	 *     return -ENODEV;
	 * }
	 * k_sem_init(&data->device_sem, 1, 1);
	 * k_sem_init(&data->session_sem, 1, 1);
	 */

	return 0;
}

static const struct crypto_driver_api crypto_enc_funcs = {
	.cipher_begin_session = crypto_ameba_session_setup,
	.cipher_free_session = crypto_ameba_session_free,
	.cipher_async_callback_set = NULL,
	.query_hw_caps = crypto_ameba_query_caps,
};

static struct crypto_ameba_data crypto_ameba_dev_data = {};

static const struct crypto_ameba_config crypto_ameba_dev_config = {};

DEVICE_DT_INST_DEFINE(0, crypto_ameba_init, NULL, &crypto_ameba_dev_data, &crypto_ameba_dev_config,
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY, (void *)&crypto_enc_funcs);
