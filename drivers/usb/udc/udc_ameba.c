/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba UDC
 */

#define DT_DRV_COMPAT realtek_ameba_udc

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/dt-bindings/usb/ameba_usb_ep.h>
#include <zephyr/usb/usb_device.h>
#include "udc_common.h"

#include "usbd.h"
#include "usbd_hal.h"
#include "usbd_pcd.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_ameba, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define AMEBA_GET_USB_EP_NUM(x)    (((x) & 0x000F0000) >> 16)
#define AMEBA_GET_USB_EP_IS_OUT(x) (((x) & 0x00008000) >> 15)
#define AMEBA_GET_USB_EP_TYPE(x)   (((x) & 0x00007800) >> 11)
#define AMEBA_GET_USB_EP_MPS(x)    (((x) & 0x000007FF) >> 0)

#define AMEBA_USB_EP_TYPE_CHECK(x, mask) ((x & mask) ? (1) : (0))
#define EP0_MPS                          64U

K_THREAD_STACK_DEFINE(udc_isr_thread_stack, USBD_ISR_THREAD_STACK_SIZE);
static struct k_thread udc_isr_thread;
static struct k_sem udc_sema;

struct udc_ameba_data {
	usbd_pcd_t *pcd;   /* usbd_pcd_t* */
	usb_dev_t usb_dev; /* usb_dev_t */

	uint32_t irq;

	const struct device *dev; /* zephyr udc driver */
};

struct udc_ameba_config {
	uint16_t ep0_mps;
	enum udc_bus_speed max_speed;
};

static const usbd_config_t udc_cfg = {.speed = USB_SPEED_FULL,
				      .dma_enable = 0U,
				      .intr_use_ptx_fifo = 0U,
				      .nptx_max_epmis_cnt = 10U,
				      .ext_intr_en = 0,
				      .nptx_max_err_cnt = {
					      0U,
					      0U,
					      0U,
					      2000U,
				      }};

static struct udc_ameba_data udc0_priv;
static struct udc_ep_config ep_cfg_in[DT_INST_PROP(0, in_ep_cnt)];
static struct udc_ep_config ep_cfg_out[DT_INST_PROP(0, out_ep_cnt)];

static struct udc_ameba_data *udc_ameba_get_data_handle(usbd_pcd_t *pcd)
{
	ARG_UNUSED(pcd);
	return &udc0_priv;
}

static int udc_ameba_ctrl_feed_dout(const struct device *dev, const size_t length)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_put(&cfg->fifo, buf);

	usbd_pcd_ep_receive(priv->pcd, cfg->addr, buf->data, buf->size);

	return 0;
}

static int udc_ameba_tx(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	const struct udc_ameba_config *cfg = dev->config;
	uint8_t *data;
	uint32_t len;
	int status;

	if (udc_ep_is_busy(dev, ep)) {
		LOG_DBG("EP%02x busy", ep);
		return 0;
	}

	data = buf->data;
	len = buf->len;

	if (ep == USB_CONTROL_EP_IN) {
		len = MIN(cfg->ep0_mps, buf->len);
	}

	buf->data += len;
	buf->len -= len;

	status = usbd_pcd_ep_transmit(priv->pcd, ep, data, len);
	if (status != HAL_OK) {
		LOG_ERR("EP%02x tx fail %d", ep, status);
		return -EIO;
	}

	udc_ep_set_busy(dev, ep, true);

	if (ep == USB_CONTROL_EP_IN && len > 0) {
		/* Wait for an empty package from the host.
		 * This also flushes the TX FIFO to the host.
		 */
		udc_ameba_ctrl_feed_dout(dev, 0);
	}

	return 0;
}

static int udc_ameba_rx(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	if (udc_ep_is_busy(dev, ep)) {
		return 0;
	}

	status = usbd_pcd_ep_receive(priv->pcd, ep, buf->data, buf->size);
	if (status != HAL_OK) {
		LOG_ERR("EP%02x Rx fail %d", ep, status);
		return -EIO;
	}

	udc_ep_set_busy(dev, ep, true);

	return 0;
}

static int udc_ameba_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	status = usbd_hal_set_device_address(priv->pcd, addr);
	if (status != HAL_OK) {
		LOG_ERR("Set addr %d error %d", addr, status);
		return -EIO;
	}

	return 0;
}

static void udc_ameba_handle_reset_interrupt(usbd_pcd_t *pcd)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);

	usbd_pcd_handle_reset_interrupt(pcd);

	udc_submit_event(priv->dev, UDC_EVT_RESET, 0);
}

#if 0
void udc_ameba_con_cb(usbd_pcd_t *pcd)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);

	udc_submit_event(priv->dev, UDC_EVT_VBUS_READY, 0);
}

void udc_ameba_discon_cb(usbd_pcd_t *pcd)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);

	udc_submit_event(priv->dev, UDC_EVT_VBUS_REMOVED, 0);
}
#endif

static void udc_ameba_handle_suspend_interrupt(usbd_pcd_t *pcd)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);

	udc_set_suspended(priv->dev, true);
	udc_submit_event(priv->dev, UDC_EVT_SUSPEND, 0);
}

static void udc_ameba_handle_wakeup_interrupt(usbd_pcd_t *pcd)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);

	udc_set_suspended(priv->dev, false);
	udc_submit_event(priv->dev, UDC_EVT_RESUME, 0);
}

static void udc_ameba_handle_enum_done_interrupt(usbd_pcd_t *pcd)
{
	usbd_pcd_handle_enum_done_interrupt(pcd);
}

static void udc_ameba_handle_sof_interrupt(usbd_pcd_t *pcd)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);

	udc_submit_event(priv->dev, UDC_EVT_SOF, 0);
}

/* weak */
void usbd_pcd_handle_ep_out_setup_packet_interrupt(usbd_pcd_t *pcd, u8 ep_num)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);
	struct usb_setup_packet *setup = (struct usb_setup_packet *)priv->pcd->setup;
	const struct device *dev = priv->dev;
	struct net_buf *buf;
	int err;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, sizeof(struct usb_setup_packet));
	if (buf == NULL) {
		LOG_ERR("Alloc setup buf fail");
		return;
	}

	udc_ep_buf_set_setup(buf);
	memcpy(buf->data, setup, 8);
	net_buf_add(buf, 8);

	udc_ctrl_update_stage(dev, buf);

	if (!buf->len) {
		return;
	}

	if (setup->bRequest == USB_SREQ_SET_ADDRESS) {
		/* HAL requires we set the address before submitting status */
		udc_ameba_set_address(dev, setup->wValue);
	}

	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		err = udc_ameba_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		udc_ctrl_submit_s_in_status(dev);
	} else {
		udc_ctrl_submit_s_status(dev);
	}
}

/* weak */
void usbd_pcd_handle_ep_out_transfer_complete_interrupt(usbd_pcd_t *pcd, uint8_t epnum)
{
	uint32_t rx_count = usbd_pcd_ep_get_rx_data_size(pcd, epnum);
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);
	const struct device *dev = priv->dev;
	uint8_t ep = epnum | USB_EP_DIR_OUT;
	struct net_buf *buf;

	udc_ep_set_busy(dev, ep, false);

	buf = udc_buf_get(dev, ep);
	if (unlikely(buf == NULL)) {
		LOG_ERR("EP%02x queue is empty", ep);
		return;
	}

	net_buf_add(buf, rx_count);

	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev)) {
			udc_ctrl_update_stage(dev, buf);
			udc_ctrl_submit_status(dev, buf);
		} else {
			udc_ctrl_update_stage(dev, buf);
		}

		if (udc_ctrl_stage_is_status_in(dev)) {
			udc_ctrl_submit_s_out_status(dev, buf);
		}
	} else {
		udc_submit_ep_event(dev, buf, 0);
	}

	buf = udc_buf_peek(dev, ep);
	if (buf) {
		udc_ameba_rx(dev, ep, buf);
	}
}

/* weak */
void usbd_pcd_handle_data_in_ep_stage(usbd_pcd_t *pcd, u8 epnum, u8 *pbuf, u8 status)
{
	struct udc_ameba_data *priv = udc_ameba_get_data_handle(pcd);
	const struct device *dev = priv->dev;
	uint8_t ep = epnum | USB_EP_DIR_IN;
	struct net_buf *buf;

	udc_ep_set_busy(dev, ep, false);

	buf = udc_buf_peek(dev, ep);
	if (unlikely(buf == NULL)) {
		return;
	}

	if (ep == USB_CONTROL_EP_IN && buf->len) {
		const struct udc_ameba_config *cfg = dev->config;
		uint32_t len = MIN(cfg->ep0_mps, buf->len);

		usbd_pcd_ep_transmit(priv->pcd, ep, buf->data, len);

		buf->len -= len;
		buf->data += len;

		return;
	}

	udc_buf_get(dev, ep);

	if (ep == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * IN transfer finished, release buffer,
			 * control OUT buffer should be already fed.
			 */
			net_buf_unref(buf);
		}

		return;
	}

	udc_submit_ep_event(dev, buf, 0);

	buf = udc_buf_peek(dev, ep);
	if (buf) {
		udc_ameba_tx(dev, ep, buf);
	}
}

/* USB interrupt */
static void udc_ameba_handle_interrupt(usbd_pcd_t *pcd)
{
	/* ensure that we are in device mode */
	if (usb_hal_get_otg_mode() == USB_OTG_MODE_DEVICE) {

		usb_os_lock(pcd->lock);

		u32 gintsts = usb_hal_read_interrupts();

		/* avoid spurious interrupt */
		if (gintsts == 0U) {
			usb_os_unlock(pcd->lock);
			return;
		}

		if (gintsts & (USB_OTG_GINTSTS_IEPINT)) {
			usbd_pcd_handle_in_ep_interrupt(pcd);
		}

		if (gintsts & (USB_OTG_GINTSTS_OEPINT)) {
			usbd_pcd_handle_out_ep_interrupt(pcd);
		}

		if (gintsts & (USB_OTG_GINTSTS_IISOIXFR)) {
			usbd_pcd_handle_incomplete_in_isoc(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_IISOIXFR);
		}

		/* Handle Resume Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_WKUINT)) {
			udc_ameba_handle_wakeup_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_WKUINT);
		}

		/* Handle Suspend Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_USBSUSP)) {
			udc_ameba_handle_suspend_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_USBSUSP);
		}
		/* Handle Reset Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_USBRST)) {
			udc_ameba_handle_reset_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_USBRST);
		}

		/* Handle Enumeration done Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_ENUMDNE)) {
			udc_ameba_handle_enum_done_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_ENUMDNE);
		}

		/* Handle RxQLevel Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_RXFLVL)) {
			usbd_pcd_handle_rx_fifo_non_empty_interrupt(pcd);
		}

		/* Handle NP TxFIFO Empty Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_NPTXFE)) {
			usbd_pcd_handle_np_tx_fifo_empty_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_NPTXFE);
		}

		/* Handle EPMIS Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_EPMIS)) {
			usbd_pcd_handle_ep_mismatch_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_EPMIS);
		}

		/* Handle GINAKEFF Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_GINAKEFF)) {
			usbd_pcd_handle_in_nak_effective(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_GINAKEFF);
		}

		/* Handle SOF Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_SOF)) {
			udc_ameba_handle_sof_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_SOF);
		}

		/* Handle EOPF Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_EOPF)) {
			usbd_pcd_handle_eopf_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_EOPF);
		}

		/* Handle Connection event Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_SRQINT)) {
			usbd_pcd_handle_srq_interrupt(pcd);
			USB_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_SRQINT);
		}

		/* Handle Disconnection event Interrupt */
		if (gintsts & (USB_OTG_GINTSTS_OTGINT)) {
			usbd_pcd_handle_otg_interrupt(pcd);
		}

		usb_os_unlock(pcd->lock);
	}
}

static void udc_ameba_irq(const struct device *dev)
{
	const struct udc_ameba_data *priv = udc_get_private(dev);
	usbd_pcd_t *pcd = priv->pcd;

	irq_disable(priv->irq);

	usb_os_sema_give(pcd->isr_sema);
}

static void udc_ameba_isr_task(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = (struct device *)arg1;
	const struct udc_ameba_data *priv = udc_get_private(dev);
	usbd_pcd_t *pcd = priv->pcd;
	u32 gintsts;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		if (usb_os_sema_take(pcd->isr_sema, USB_OS_SEMA_TIMEOUT) == HAL_OK) {

			gintsts = usb_hal_read_interrupts();

			udc_ameba_handle_interrupt(pcd);
			irq_enable(priv->irq);

			if ((gintsts != 0U) && (usb_hal_get_otg_mode() == USB_OTG_MODE_DEVICE)) {
				if ((gintsts & USB_OTG_GINTSTS_USBSUSP) != 0) {
					if (pcd->dev->dev_attach_status ==
					    USBD_ATTACH_STATUS_ATTACHED) {
						pcd->dev->dev_attach_status =
							USBD_ATTACH_STATUS_DETACHED;
					}
				} else {
					if ((pcd->dev->dev_attach_status !=
					     USBD_ATTACH_STATUS_ATTACHED) &&
					    ((gintsts & USB_OTG_GINTSTS_ESUSP) == 0)) {
						pcd->dev->dev_attach_status =
							USBD_ATTACH_STATUS_ATTACHED;
					}
				}
			}
		}
	}
}

static int udc_ameba_ep_flush(const struct device *dev, struct udc_ep_config *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	return 0;
}

/* struct udc_api */
static int udc_ameba_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_ameba_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static int udc_ameba_init(const struct device *dev)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	usb_dev_t *usb_dev = &(priv->usb_dev);
	int status;

	usb_dev->driver = NULL;
	usb_dev->dev_state = USBD_STATE_DEFAULT;
	usb_dev->ctrl_buf = (u8 *)usb_os_malloc(USB_OTG_HS_MAX_PACKET_SIZE);
	if (usb_dev->ctrl_buf == NULL) {
		return -ENOMEM;
	}

	status = usbd_pcd_init(usb_dev, (usbd_config_t *)&udc_cfg);
	if (status != 0) {
		LOG_ERR("PCD_Init fail %d", status);
		return -EIO;
	}

	k_thread_start(&udc_isr_thread);

	return 0;
}

static int udc_ameba_shutdown(const struct device *dev)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	usb_dev_t *usb_dev = &(priv->usb_dev);
	int status;

	usb_dev->dev_state = USBD_STATE_DEFAULT;
	usb_hal_disable_global_interrupt();

	if ((usb_dev->driver != NULL) && (usb_dev->driver->clear_config != NULL)) {
		usb_dev->driver->clear_config(usb_dev, usb_dev->dev_config);
		usb_dev->driver = NULL;
	}
	usb_dev->driver = NULL;

	status = usbd_pcd_deinit(usb_dev);

	if (usb_dev->ctrl_buf != NULL) {
		usb_os_mfree(usb_dev->ctrl_buf);
		usb_dev->ctrl_buf = NULL;
	}

	if (status != HAL_OK) {
		LOG_ERR("PCD_DeInit fail %d", status);
		/* continue anyway */
	}

	return 0;
}

static int udc_ameba_enable(const struct device *dev)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	status = usbd_pcd_start(priv->pcd);
	if (status != HAL_OK) {
		LOG_ERR("PCD_Start fail %d", status);
		return -EIO;
	}

	irq_enable(priv->irq);

	return 0;
}

static int udc_ameba_disable(const struct device *dev)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	irq_disable(priv->irq);

	status = usbd_pcd_stop(priv->pcd);
	if (status != HAL_OK) {
		LOG_ERR("PCD_Stop fail %d", status);
		return -EIO;
	}

	return 0;
}

static int udc_ameba_host_wakeup(const struct device *dev)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	status = usbd_hal_wake_host(priv->pcd);
	if (status != HAL_OK) {
		LOG_ERR("Activate Remote Wakeup fail %d", status);
		return -EIO;
	}

	return 0;
}

static int udc_ameba_ep_enable(const struct device *dev, struct udc_ep_config *ep)
{
	enum usb_dc_ep_transfer_type type = ep->attributes & USB_EP_TRANSFER_TYPE_MASK;
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	status = usbd_pcd_ep_init(priv->pcd, ep->addr, ep->mps, type);
	if (status != HAL_OK) {
		LOG_ERR("EP%02x init fail %d", ep->addr, status);
		return -EIO;
	}

	return 0;
}

static int udc_ameba_ep_disable(const struct device *dev, struct udc_ep_config *ep)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status;

	status = usbd_pcd_ep_deinit(priv->pcd, ep->addr);
	if (status != HAL_OK) {
		LOG_ERR("EP%02x deinit fail %d", ep->addr, status);
		return -EIO;
	}

	return 0;
}

static int udc_ameba_ep_set_halt(const struct device *dev, struct udc_ep_config *cfg)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status = 0;

	status = usbd_pcd_ep_set_stall(priv->pcd, cfg->addr);
	if (status != HAL_OK) {
		LOG_ERR("EP%02x set hal fail %d", cfg->addr, status);
		return -EIO;
	}

	return 0;
}

static int udc_ameba_ep_clear_halt(const struct device *dev, struct udc_ep_config *cfg)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	int status = 0;

	status = usbd_pcd_ep_clear_stall(priv->pcd, cfg->addr);
	if (status != HAL_OK) {
		LOG_ERR("EP%02x clear halt fail %d", cfg->addr, status);
		return -EIO;
	}

	return 0;
}

static int udc_ameba_ep_enqueue(const struct device *dev, struct udc_ep_config *epcfg,
				struct net_buf *buf)
{
	unsigned int lock_key;
	int ret;

	udc_buf_put(epcfg, buf);

	lock_key = irq_lock();

	if (USB_EP_DIR_IS_IN(epcfg->addr)) {
		ret = udc_ameba_tx(dev, epcfg->addr, buf);
	} else {
		ret = udc_ameba_rx(dev, epcfg->addr, buf);
	}

	irq_unlock(lock_key);

	return ret;
}

static int udc_ameba_ep_dequeue(const struct device *dev, struct udc_ep_config *epcfg)
{
	struct net_buf *buf;

	udc_ameba_ep_flush(dev, epcfg);

	buf = udc_buf_get_all(dev, epcfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(dev, epcfg->addr, false);

	return 0;
}

static enum udc_bus_speed udc_ameba_get_device_speed(const struct device *dev)
{
	/* AmebadPlus just support FS */
	const struct udc_ameba_config *cfg = dev->config;

	return cfg->max_speed;
}

static const struct udc_api udc_ameba_api = {
	.device_speed = udc_ameba_get_device_speed,
	.ep_enqueue = udc_ameba_ep_enqueue,
	.ep_dequeue = udc_ameba_ep_dequeue,
	.ep_set_halt = udc_ameba_ep_set_halt,
	.ep_clear_halt = udc_ameba_ep_clear_halt,
	.ep_try_config = NULL,
	.ep_enable = udc_ameba_ep_enable,
	.ep_disable = udc_ameba_ep_disable,
	.host_wakeup = udc_ameba_host_wakeup,
	.set_address = udc_ameba_set_address,
	.enable = udc_ameba_enable,
	.disable = udc_ameba_disable,
	.init = udc_ameba_init,
	.shutdown = udc_ameba_shutdown,
	.lock = udc_ameba_lock,
	.unlock = udc_ameba_unlock,
};

/* ----------------- Instance/Device specific data ----------------- */
static int udc_ameba_ep_init(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	int err;
	err = udc_register_ep(dev, ep_cfg);
	if (err != 0) {
		LOG_ERR("Register ep fail");
		return err;
	}
	return 0;
}

static int udc_ameba_driver_init(const struct device *dev)
{
	struct udc_ameba_data *priv = udc_get_private(dev);
	const struct udc_ameba_config *cfg = dev->config;
	struct udc_data *data = dev->data;
	struct udc_ep_config *p_ep_cfg_handle = NULL;

	/* Parse dts to get all ep information */
	uint8_t ep_in_idx = 0;
	uint8_t ep_out_idx = 0;
	uint8_t ep_num = 0;
	uint8_t ep_direction = 0;
	uint8_t ep_type = 0;
	uint16_t ep_mps = 0;

	uint8_t ep_cnt = DT_PROP_LEN(DT_DRV_INST(0), ep_ranges);
	const int ep_info_array[] = DT_PROP(DT_DRV_INST(0), ep_ranges);

	p_ep_cfg_handle = &(ep_cfg_in[0]);
	p_ep_cfg_handle->caps.in = 1;
	p_ep_cfg_handle->caps.control = 1;
	p_ep_cfg_handle->caps.mps = cfg->ep0_mps;
	p_ep_cfg_handle->addr = USB_EP_DIR_IN | 0;
	if (udc_ameba_ep_init(dev, p_ep_cfg_handle) == 0) {
		ep_in_idx++;
	}

	p_ep_cfg_handle = &(ep_cfg_out[0]);
	p_ep_cfg_handle->caps.out = 1;
	p_ep_cfg_handle->caps.control = 1;
	p_ep_cfg_handle->caps.mps = cfg->ep0_mps;
	p_ep_cfg_handle->addr = USB_EP_DIR_OUT | 0;
	if (udc_ameba_ep_init(dev, p_ep_cfg_handle) == 0) {
		ep_out_idx++;
	}

	for (unsigned int i = 0; i < ep_cnt; i++) {

		ep_num = AMEBA_GET_USB_EP_NUM(ep_info_array[i]);
		ep_direction = AMEBA_GET_USB_EP_IS_OUT(ep_info_array[i]);
		ep_type = AMEBA_GET_USB_EP_TYPE(ep_info_array[i]);
		ep_mps = AMEBA_GET_USB_EP_MPS(ep_info_array[i]);

		if (ep_direction == AMEBA_USB_EP_DIR_IN) {
			if (ep_in_idx < ARRAY_SIZE(ep_cfg_in)) {
				p_ep_cfg_handle = &(ep_cfg_in[ep_in_idx]);
				p_ep_cfg_handle->caps.in = 1;
				p_ep_cfg_handle->caps.bulk =
					AMEBA_USB_EP_TYPE_CHECK(ep_type, AMEBA_USB_EP_TYPE_BULK);
				p_ep_cfg_handle->caps.interrupt =
					AMEBA_USB_EP_TYPE_CHECK(ep_type, AMEBA_USB_EP_TYPE_INTR);
				p_ep_cfg_handle->caps.iso =
					AMEBA_USB_EP_TYPE_CHECK(ep_type, AMEBA_USB_EP_TYPE_ISOC);
				p_ep_cfg_handle->caps.mps = ep_mps;
				p_ep_cfg_handle->addr = USB_EP_DIR_IN | ep_num;

				if (udc_ameba_ep_init(dev, p_ep_cfg_handle) == 0) {
					ep_in_idx++;
				} else {
					LOG_ERR("Register in ep fail");
				}
			} else {
				LOG_WRN("In ep array has no space");
			}
		} else if (ep_direction == AMEBA_USB_EP_DIR_OUT) {
			if (ep_out_idx < ARRAY_SIZE(ep_cfg_out)) {
				p_ep_cfg_handle = &(ep_cfg_out[ep_out_idx]);
				p_ep_cfg_handle->caps.out = 1;
				p_ep_cfg_handle->caps.bulk =
					AMEBA_USB_EP_TYPE_CHECK(ep_type, AMEBA_USB_EP_TYPE_BULK);
				p_ep_cfg_handle->caps.interrupt =
					AMEBA_USB_EP_TYPE_CHECK(ep_type, AMEBA_USB_EP_TYPE_INTR);
				p_ep_cfg_handle->caps.iso =
					AMEBA_USB_EP_TYPE_CHECK(ep_type, AMEBA_USB_EP_TYPE_ISOC);
				p_ep_cfg_handle->caps.mps = ep_mps;
				p_ep_cfg_handle->addr = USB_EP_DIR_OUT | ep_num;

				if (udc_ameba_ep_init(dev, p_ep_cfg_handle) == 0) {
					ep_out_idx++;
				}
			} else {
				LOG_ERR("Register out ep fail");
			}
		} else {
			LOG_WRN("Out ep array has no space");
		}
	}

	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.mps0 = UDC_MPS0_64;

	priv->dev = dev;
	priv->pcd = (usbd_pcd_t *)usbd_get_pcd_handle();

	priv->pcd->isr_sema = &udc_sema;
	usb_os_sema_create(&(priv->pcd->isr_sema));
	priv->pcd->isr_initialized = 1;

	/* create the task to handle the usb isr message */
	k_thread_create(&udc_isr_thread, udc_isr_thread_stack, USBD_ISR_THREAD_STACK_SIZE,
			udc_ameba_isr_task, (void *)dev, NULL, NULL, -1, K_USER, K_MSEC(0));

	k_thread_name_set(&udc_isr_thread, "udc_isr_thread");

	priv->irq = DT_INST_IRQN(0);
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), udc_ameba_irq, DEVICE_DT_INST_GET(0),
		    0);

	return 0;
}

static struct udc_data udc0_data = {
	.mutex = Z_MUTEX_INITIALIZER(udc0_data.mutex),
	.priv = &udc0_priv,
};

static const struct udc_ameba_config udc0_cfg = {
	.ep0_mps = EP0_MPS,
	.max_speed = DT_INST_PROP(0, udc_max_speed),
};

DEVICE_DT_INST_DEFINE(0, udc_ameba_driver_init, NULL, &udc0_data, &udc0_cfg, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_ameba_api);
