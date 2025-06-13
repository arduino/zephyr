/*
 * Copyright (c) 2024 Realtek Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "os_wrapper_semaphore.h"
#include "rtw_wifi_defs.h"
#include "wifi_intf_drv_to_app_basic.h"
#include "os_wrapper_memory.h"
#include "wifi_api_event.h"

#define DT_DRV_COMPAT         realtek_ameba_wifi
#define WIFI_EVENT_STACK_SIZE 4096
#define DHCPV4_MASK           (NET_EVENT_IPV4_DHCP_BOUND | NET_EVENT_IPV4_DHCP_STOP)
#define MAX_IP_ADDR_LEN       16

#if defined(__IAR_SYSTEMS_ICC__) || defined(__GNUC__) || defined(__CC_ARM) ||                      \
	(defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
/* SET pack mode 1-alignment for the following area. */
#pragma pack(1)
#endif

struct rtk_wifi_status {
	char ssid[WIFI_SSID_MAX_LEN + 1];
	char pass[WIFI_PSK_MAX_LEN + 1];
	bool connected;
	uint8_t channel;
	int rssi;
};

struct ameba_wifi_runtime {
	uint8_t mac_addr[2][6];
	uint8_t frame_buf[2][NET_ETH_MAX_FRAME_SIZE];
#if defined(CONFIG_NET_STATISTICS_WIFI)
	struct net_stats_wifi stats;
#endif
	struct rtk_wifi_status status;
	scan_result_cb_t scan_cb;
	uint8_t state;
};

typedef enum {
	SYSTEM_EVENT_WIFI_READY = 0,
	SYSTEM_EVENT_SCAN_DONE,
	SYSTEM_EVENT_STA_START,
	SYSTEM_EVENT_STA_STOP,
	SYSTEM_EVENT_STA_CONNECTED,
	SYSTEM_EVENT_STA_DISCONNECTED,

	RTK_WIFI_EVENT_STA_START,
	RTK_WIFI_EVENT_STA_STOP,
	RTK_WIFI_EVENT_STA_CONNECTED,
	RTK_WIFI_EVENT_STA_DISCONNECTED,
	RTK_WIFI_EVENT_SCAN_DONE,
	RTK_WIFI_EVENT_AP_STOP,
	RTK_WIFI_EVENT_AP_STACONNECTED,
	RTK_WIFI_EVENT_AP_STADISCONNECTED,
} system_event_id_t;

typedef struct {
	uint8_t bssid[6]; /*!< BSSID of the connected AP */
	uint8_t ssid[32]; /*!< SSID of the connected AP */
	uint8_t ssid_len; /*!< Length of the SSID */
	uint8_t channel;  /*!< Channel of the connected AP */
} wifi_event_sta_connected_t;

typedef union {
	wifi_event_sta_connected_t sta_connected; /*!< station connected to AP */
} system_event_info_t;

struct ameba_system_event {
	system_event_id_t event_id;
	system_event_info_t event_info;
};

enum rtk_state_flag {
	RTK_STA_STOPPED,
	RTK_STA_STARTED,
	RTK_STA_CONNECTING,
	RTK_STA_CONNECTED,
	RTK_AP_CONNECTED,
	RTK_AP_DISCONNECTED,
	RTK_AP_STOPPED,
};

void wlan_initialize(void);
void wifi_init(void);
int wifi_get_setting(unsigned char wlan_idx, struct _rtw_wifi_setting_t *psetting);
int wifi_get_scan_records(unsigned int *AP_num, char *scan_buf);
int inic_host_send(int idx, void *pkt_addr, uint32_t len);
int wifi_get_mac_address(int idx, struct _rtw_mac_t *mac, u8 efuse);
int eth_rtk_rx(uint8_t idx, void *buffer, uint16_t len);
int rltk_wlan_send(int idx, void *pkt_addr, uint32_t len);
int whc_host_send_zephyr(int idx, void *pkt_addr, uint32_t len);

