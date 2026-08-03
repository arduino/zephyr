/*
 * Copyright (c) 2026 ARDUINO SRL
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_se05x
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/drivers/gpio.h>
#include <sm_i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(se05x, CONFIG_CRYPTO_LOG_LEVEL);

struct se05x_device_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec en_gpios;
};

static inline int se05x_en_gpios_set(const struct device *dev, int value)
{
	struct se05x_device_config *cfg = (struct se05x_device_config*) dev->config;
	int ret;

	if (cfg->en_gpios.port == NULL) {
		return 0;
	}

	ret = gpio_pin_set_dt(&cfg->en_gpios, value);
	if (ret < 0) {
		LOG_ERR("%s: can't enable se05x!", dev->name);
	}

	return 0;
}


i2c_error_t axI2CInit(void **conn_ctx, const char *pDevName)
{
	struct device *dev = DEVICE_DT_INST_GET(0);
	*conn_ctx = dev;

	return I2C_OK;
}

void axI2CTerm(void *conn_ctx, int mode)
{
	struct device *dev = (struct device *)conn_ctx;
	int ret = device_deinit(dev);

	if (ret < 0) {
		LOG_ERR("%s: can't deinit se05x (%d)", dev->name, ret);
	}
}

i2c_error_t axI2CWrite(void *conn_ctx, unsigned char bus, unsigned char addr, unsigned char *pTx,
		       unsigned short txLen)
{
	struct device *dev = (struct device *)conn_ctx;
	struct se05x_device_config *cfg = (struct se05x_device_config*) dev->config;
	int ret = 0;

	if (ret = i2c_write_dt(&cfg->i2c, pTx, txLen)) {
		return I2C_FAILED;
	}

	return I2C_OK;
}

i2c_error_t axI2CRead(void *conn_ctx, unsigned char bus, unsigned char addr, unsigned char *pRx,
		      unsigned short rxLen)
{
	struct device *dev = (struct device *)conn_ctx;
	struct se05x_device_config *cfg = (struct se05x_device_config*) dev->config;
	int ret = 0;

	if (ret = i2c_read_dt(&cfg->i2c, pRx, rxLen)) {
		return I2C_FAILED;
	}

	return I2C_OK;
}

static int se05x_init(const struct device *dev)
{
	const struct se05x_device_config *cfg = dev->config;
	uint32_t i2c_cfg;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	if (cfg->en_gpios.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->en_gpios)) {
			LOG_ERR("%s: gpio pin: %s not ready", dev->name, cfg->en_gpios.port->name);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->en_gpios, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
		if (ret < 0) {
			LOG_ERR("%s: can't configure enable pin (%d) as output", dev->name,
				cfg->en_gpios.pin);
			return ret;
		}

		se05x_en_gpios_set(dev, 1);
	}

	return 0;
}

static int se05x_deinit(const struct device *dev)
{
	const struct se05x_device_config *cfg = dev->config;
	int ret = 0;
	LOG_INF("%s: deinit", dev->name);

	ret = device_deinit(cfg->i2c.bus);
	if (ret < 0) {
		LOG_ERR("%s: can't deinit se05x i2c (%d)", cfg->i2c.bus->name, ret);
		return ret;
	}

	se05x_en_gpios_set(dev, 0);
	ret = device_deinit(cfg->en_gpios.port);
	if (ret < 0) {
		LOG_ERR("%s: can't deinit se05x en_gpios (%d)", cfg->en_gpios.port->name, ret);
		return ret;
	}

	return 0;
}

static DEVICE_API(crypto, crypto_enc_funcs) = {
	.cipher_begin_session = NULL,
	.cipher_free_session = NULL,
	.cipher_async_callback_set = NULL,
	.query_hw_caps = NULL,
};

#define SE05X_DEFINE(inst)                                                                         \
	static const struct se05x_device_config se05x_config_##inst = {                            \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.en_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, se05x_en_gpios, {0}),                   \
	};                                                                                         \
	DEVICE_DT_INST_DEINIT_DEFINE(inst, &se05x_init, &se05x_deinit,                             \
		NULL, NULL, &se05x_config_##inst,                                                  \
		POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,                                          \
		(void *)&crypto_enc_funcs);

DT_INST_FOREACH_STATUS_OKAY(SE05X_DEFINE)
