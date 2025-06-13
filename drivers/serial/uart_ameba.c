/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba UART interface
 */
#define DT_DRV_COMPAT realtek_ameba_uart

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_ameba, CONFIG_UART_LOG_LEVEL);

struct uart_ameba_config {
	UART_TypeDef *uart;
	int uart_idx;
	const struct device *clock_dev;
	const struct pinctrl_dev_config *pcfg;
	const clock_control_subsys_t clock_subsys;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	uart_irq_config_func_t irq_config_func;
#endif
#if CONFIG_UART_ASYNC_API
	const struct device *dma_dev;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_channel;
	bool uart_id;
#endif
};

/* driver data */
struct uart_ameba_data {
	struct uart_config uart_config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
	bool tx_int_en;
	bool rx_int_en;
#endif
#if CONFIG_UART_ASYNC_API
	struct uart_ameba_async_data async;
	uhci_dev_t *uhci_dev;
	const struct device *uart_dev;
#endif
};

static int uart_ameba_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;

	if (!UART_Readable(uart)) {
		return -1;
	}

	UART_CharGet(uart, (uint8_t *)c);

	return 0;
}

static void uart_ameba_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;

	while (!UART_Writable(uart)) {
	}

	UART_CharPut(uart, (uint8_t)c);
}

static int uart_ameba_err_check(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;
	uint32_t lsr = UART_LineStatusGet(uart);
	uint32_t err = 0U;

	/* Check for errors */
	if (lsr & RUART_BIT_OVR_ERR) {
		err |= UART_ERROR_OVERRUN;
	}

	if (lsr & RUART_BIT_PAR_ERR) {
		err |= UART_ERROR_PARITY;
	}

	if (lsr & RUART_BIT_FRM_ERR) {
		err |= UART_ERROR_FRAMING;
	}

	if (lsr & RUART_BIT_BREAK_INT) {
		err |= UART_BREAK;
	}

	/* Clear errors */
	if (lsr & UART_ALL_RX_ERR) {
		UART_INT_Clear(uart, RUART_BIT_RLSICF);
	}

	return err;
}

static int rtl_cfg_parity(UART_InitTypeDef *uart_struct, uint32_t parity)
{
	switch (parity) {
	case UART_CFG_PARITY_NONE:
		uart_struct->Parity = RUART_PARITY_DISABLE;
		break;
	case UART_CFG_PARITY_ODD:
		uart_struct->Parity = RUART_PARITY_ENABLE;
		uart_struct->ParityType = RUART_ODD_PARITY;
		uart_struct->StickParity = RUART_STICK_PARITY_DISABLE;
		break;
	case UART_CFG_PARITY_EVEN:
		uart_struct->Parity = RUART_PARITY_ENABLE;
		uart_struct->ParityType = RUART_EVEN_PARITY;
		uart_struct->StickParity = RUART_STICK_PARITY_DISABLE;
		break;
	case UART_CFG_PARITY_MARK:
		uart_struct->Parity = RUART_PARITY_ENABLE;
		uart_struct->ParityType = RUART_ODD_PARITY;
		uart_struct->StickParity = RUART_STICK_PARITY_ENABLE;
		break;
	case UART_CFG_PARITY_SPACE:
		uart_struct->Parity = RUART_PARITY_ENABLE;
		uart_struct->ParityType = RUART_EVEN_PARITY;
		uart_struct->StickParity = RUART_STICK_PARITY_ENABLE;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int rtl_cfg_data_bits(UART_InitTypeDef *uart_struct, uint32_t data_bits)
{
	switch (data_bits) {
	case UART_CFG_DATA_BITS_7:
		uart_struct->WordLen = RUART_WLS_7BITS;
		break;
	case UART_CFG_DATA_BITS_8:
		uart_struct->WordLen = RUART_WLS_8BITS;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int rtl_cfg_stop_bits(UART_InitTypeDef *uart_struct, uint32_t stop_bits)
{
	switch (stop_bits) {
	case UART_CFG_STOP_BITS_1:
		uart_struct->StopBit = RUART_STOP_BIT_1;
		break;
	case UART_CFG_STOP_BITS_2:
		uart_struct->StopBit = RUART_STOP_BIT_2;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int rtl_cfg_flow_ctrl(UART_InitTypeDef *uart_struct, uint32_t flow_ctrl)
{
	switch (flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		uart_struct->FlowControl = DISABLE;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		uart_struct->FlowControl = ENABLE;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int uart_ameba_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	int ret;

	UART_InitTypeDef UART_InitStruct;
	UART_TypeDef *uart = config->uart;

	UART_StructInit(&UART_InitStruct);

	ret = rtl_cfg_parity(&UART_InitStruct, cfg->parity);
	if (ret < 0) {
		LOG_ERR("Unsupported UART Parity Type (%d)", ret);
		return ret;
	}

	ret = rtl_cfg_data_bits(&UART_InitStruct, cfg->data_bits);
	if (ret < 0) {
		LOG_ERR("Unsupported UART Data Bit (%d)", ret);
		return ret;
	}

	ret = rtl_cfg_stop_bits(&UART_InitStruct, cfg->stop_bits);
	if (ret < 0) {
		LOG_ERR("Unsupported UART Stop Bit (%d)", ret);
		return ret;
	}

	ret = rtl_cfg_flow_ctrl(&UART_InitStruct, cfg->flow_ctrl);
	if (ret < 0) {
		LOG_ERR("Unsupported UART Flow Control (%d)", ret);
		return ret;
	}

	UART_Init(uart, &UART_InitStruct);
	UART_SetBaud(uart, cfg->baudrate);
	UART_RxCmd(uart, ENABLE);

	data->uart_config = *cfg;

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_ameba_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_ameba_data *data = dev->data;

	*cfg = data->uart_config;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_ameba_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;
	uint8_t num_tx = 0U;
	unsigned int key;

	if (!UART_Writable(uart)) {
		return num_tx;
	}

	/* Lock interrupts to prevent nested interrupts or thread switch */

	key = irq_lock();

	while ((len - num_tx > 0) && UART_Writable(uart)) {
		UART_CharPut(uart, (uint8_t)tx_data[num_tx++]);
	}

	irq_unlock(key);

	return num_tx;
}

static int uart_ameba_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;
	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) && UART_Readable(uart)) {
		UART_CharGet(uart, (uint8_t *)(rx_data + num_rx++));
	}

	/* Clear timeout int flag */
	if (UART_LineStatusGet(uart) & RUART_BIT_TIMEOUT_INT) {
		UART_INT_Clear(uart, RUART_BIT_TOICF);
	}

	return num_rx;
}

static void uart_ameba_irq_tx_enable(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;
	u32 sts;

	/* Disable IRQ Interrupts and Save Previous Status. */
	sts = irq_disable_save();

	data->tx_int_en = true;
	UART_INTConfig(uart, RUART_BIT_ETBEI, ENABLE);

	/* Enable IRQ Interrupts according to Previous Status. */
	irq_enable_restore(sts);
}

static void uart_ameba_irq_tx_disable(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;
	u32 sts;

	/* Disable IRQ Interrupts and Save Previous Status. */
	sts = irq_disable_save();

	data->tx_int_en = false;
	UART_INTConfig(uart, RUART_BIT_ETBEI, DISABLE);

	/* Enable IRQ Interrupts according to Previous Status. */
	irq_enable_restore(sts);
}

static int uart_ameba_irq_tx_ready(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;

	return (UART_LineStatusGet(uart) & RUART_BIT_TX_EMPTY) && data->tx_int_en;
}

static int uart_ameba_irq_tx_complete(const struct device *dev)
{
	return uart_ameba_irq_tx_ready(dev);
}

static void uart_ameba_irq_rx_enable(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;

	data->rx_int_en = true;
	UART_INTConfig(uart, RUART_BIT_ERBI | RUART_BIT_ETOI, ENABLE);
}

static void uart_ameba_irq_rx_disable(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;

	data->rx_int_en = false;
	UART_INTConfig(uart, RUART_BIT_ERBI | RUART_BIT_ETOI, DISABLE);
}

static int uart_ameba_irq_rx_ready(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;

	return (UART_LineStatusGet(uart) & (RUART_BIT_RXFIFO_INT | RUART_BIT_TIMEOUT_INT)) &&
	       data->rx_int_en;
}

static void uart_ameba_irq_err_enable(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;

	UART_INTConfig(uart, RUART_BIT_ELSI, ENABLE);
}

static void uart_ameba_irq_err_disable(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	UART_TypeDef *uart = config->uart;

	UART_INTConfig(uart, RUART_BIT_ELSI, DISABLE);
}

static int uart_ameba_irq_is_pending(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;
	UART_TypeDef *uart = config->uart;

	return ((UART_LineStatusGet(uart) & RUART_BIT_TX_EMPTY) && data->tx_int_en) ||
	       ((UART_LineStatusGet(uart) & (RUART_BIT_RXFIFO_INT | RUART_BIT_TIMEOUT_INT)) &&
		data->rx_int_en);
}

static int uart_ameba_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_ameba_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct uart_ameba_data *data = dev->data;

	data->user_cb = cb;
	data->user_data = cb_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_ameba_init(const struct device *dev)
{
	const struct uart_ameba_config *config = dev->config;
	struct uart_ameba_data *data = dev->data;

	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_on(config->clock_dev, config->clock_subsys)) {
		LOG_ERR("Could not enable UART clock");
		return -EIO;
	}

	ret = uart_ameba_configure(dev, &data->uart_config);

	if (ret < 0) {
		LOG_ERR("Error configuring UART (%d)", ret);
		return ret;
	}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

#if CONFIG_UART_ASYNC_API
	if (config->dma_dev) {
		if (!device_is_ready(config->dma_dev)) {
			LOG_ERR("DMA device is not ready");
			return -ENODEV;
		}

		clock_control_on(config->clock_dev, (clock_control_subsys_t)ESP32_UHCI0_MODULE);
		uhci_ll_init(data->uhci_dev);
		uhci_ll_set_eof_mode(data->uhci_dev, UHCI_RX_IDLE_EOF | UHCI_RX_LEN_EOF);
		uhci_ll_attach_uart_port(data->uhci_dev, config->uart_id);
		data->uart_dev = dev;

		k_work_init_delayable(&data->async.tx_timeout_work, uart_ameba_async_tx_timeout);
		k_work_init_delayable(&data->async.rx_timeout_work, uart_ameba_async_rx_timeout);
	}
#endif
	return 0;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define AMEBA_UART_IRQ_HANDLER_DECL(n)                                                             \
	static void uart_ameba_irq_config_func_##n(const struct device *dev);
#define AMEBA_UART_IRQ_HANDLER(n)                                                                  \
	static void uart_ameba_irq_config_func_##n(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_ameba_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
#define AMEBA_UART_IRQ_HANDLER_FUNC(n) .irq_config_func = uart_ameba_irq_config_func_##n,
#else
#define AMEBA_UART_IRQ_HANDLER_DECL(n) /* Not used */
#define AMEBA_UART_IRQ_HANDLER(n)      /* Not used */
#define AMEBA_UART_IRQ_HANDLER_FUNC(n) /* Not used */
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

static void uart_ameba_isr(const struct device *dev)
{
	struct uart_ameba_data *data = dev->data;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

static const struct uart_driver_api uart_ameba_api = {
	.poll_in = uart_ameba_poll_in,
	.poll_out = uart_ameba_poll_out,
	.err_check = uart_ameba_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_ameba_configure,
	.config_get = uart_ameba_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ameba_fifo_fill,
	.fifo_read = uart_ameba_fifo_read,
	.irq_tx_enable = uart_ameba_irq_tx_enable,
	.irq_tx_disable = uart_ameba_irq_tx_disable,
	.irq_tx_ready = uart_ameba_irq_tx_ready,
	.irq_rx_enable = uart_ameba_irq_rx_enable,
	.irq_rx_disable = uart_ameba_irq_rx_disable,
	.irq_tx_complete = uart_ameba_irq_tx_complete,
	.irq_rx_ready = uart_ameba_irq_rx_ready,
	.irq_err_enable = uart_ameba_irq_err_enable,
	.irq_err_disable = uart_ameba_irq_err_disable,
	.irq_is_pending = uart_ameba_irq_is_pending,
	.irq_update = uart_ameba_irq_update,
	.irq_callback_set = uart_ameba_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#if CONFIG_UART_ASYNC_API
	.callback_set = uart_ameba_async_callback_set,
	.tx = uart_ameba_async_tx,
	.tx_abort = uart_ameba_async_tx_abort,
	.rx_enable = uart_ameba_async_rx_enable,
	.rx_buf_rsp = uart_ameba_async_rx_buf_rsp,
	.rx_disable = uart_ameba_async_rx_disable,
#endif /*CONFIG_UART_ASYNC_API*/
};

#define AMEBA_UART_INIT(n)                                                                         \
	AMEBA_UART_IRQ_HANDLER_DECL(n)                                                             \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct uart_ameba_config uart_ameba_config##n = {                             \
		.uart = (UART_TypeDef *)DT_INST_REG_ADDR(n),                                       \
		.uart_idx = n,                                                                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),               \
		AMEBA_UART_IRQ_HANDLER_FUNC(n)};                                                   \
                                                                                                   \
	static struct uart_ameba_data uart_ameba_data_##n = {                                      \
		.uart_config = {.baudrate = DT_INST_PROP(n, current_speed),                        \
				.parity = UART_CFG_PARITY_NONE,                                    \
				.stop_bits = UART_CFG_STOP_BITS_1,                                 \
				.data_bits = UART_CFG_DATA_BITS_8,                                 \
				.flow_ctrl = UART_CFG_FLOW_CTRL_NONE},                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &uart_ameba_init, NULL, &uart_ameba_data_##n,                     \
			      &uart_ameba_config##n, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,    \
			      &uart_ameba_api);                                                    \
                                                                                                   \
	AMEBA_UART_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(AMEBA_UART_INIT);
