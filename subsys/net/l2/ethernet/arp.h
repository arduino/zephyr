/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ARP_H
#define __ARP_H

#include <zephyr/sys/slist.h>
#include <zephyr/net/ethernet.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Address resolution (ARP) library
 * @defgroup arp ARP Library
 * @ingroup networking
 * @{
 */

/** @brief Address resolution complete, destination link address injected */
#define NET_ARP_COMPLETE     0
/** @brief Destination link address unknown, ARP request to be sent instead */
#define NET_ARP_PKT_REPLACED 1
/** @brief Destination link address unknown, ARP request pending, packet queued */
#define NET_ARP_PKT_QUEUED   2

#if defined(CONFIG_NET_ARP) && defined(CONFIG_NET_NATIVE)

#define NET_ARP_HDR(pkt) ((struct net_arp_hdr *)net_pkt_data(pkt))

struct net_arp_hdr {
	uint16_t hwtype;			/* HTYPE */
	uint16_t protocol;			/* PTYPE */
	uint8_t hwlen;				/* HLEN */
	uint8_t protolen;			/* PLEN */
	uint16_t opcode;
	struct net_eth_addr src_hwaddr;		/* SHA */
	uint8_t src_ipaddr[NET_IPV4_ADDR_SIZE];	/* SPA */
	struct net_eth_addr dst_hwaddr;		/* THA */
	uint8_t dst_ipaddr[NET_IPV4_ADDR_SIZE];	/* TPA */
} __packed;

#define NET_ARP_HTYPE_ETH 1
#define NET_ARP_IPV4_PTYPE_SIZE 4

#define NET_ARP_REQUEST 1
#define NET_ARP_REPLY   2

/**
 * @brief Prepare an ARP request, if required
 *
 * @param pkt Packet that wants to be sent
 * @param request_ip Destination address of the packet
 * @param current_ip Sending IP (Can be NULL)
 * @param arp_pkt ARP packet that should be sent in place of @a pkt, if not NULL
 *
 * @retval NET_ARP_COMPLETE ARP information populated in @a pkt
 * @retval NET_ARP_PKT_REPLACED ARP request to be sent existing in @a arp_pkt
 * @retval NET_ARP_PKT_QUEUED @a pkt was queued for transmission on ARP resolution
 * @retval <0 on failure
 */
int net_arp_prepare(struct net_pkt *pkt,
			struct in_addr *request_ip,
			struct in_addr *current_ip,
			struct net_pkt **arp_pkt);
enum net_verdict net_arp_input(struct net_pkt *pkt,
			       struct net_eth_addr *src,
			       struct net_eth_addr *dst);

int net_arp_clear_pending(struct net_if *iface,
				struct in_addr *dst);

struct arp_entry {
	sys_snode_t node;
	uint32_t req_start;
	struct net_if *iface;
	struct in_addr ip;
	struct net_eth_addr eth;
	struct k_fifo pending_queue;
};

typedef void (*net_arp_cb_t)(struct arp_entry *entry,
			     void *user_data);
int net_arp_foreach(net_arp_cb_t cb, void *user_data);

void net_arp_clear_cache(struct net_if *iface);
void net_arp_init(void);
void net_arp_update(struct net_if *iface, struct in_addr *src,
		    struct net_eth_addr *hwaddr, bool gratuitous,
		    bool force);


#else /* CONFIG_NET_ARP */
#define net_arp_prepare(_kt, _u1, _u2, _arp) NET_ARP_COMPLETE
#define net_arp_input(...) NET_OK
#define net_arp_clear_cache(...)
#define net_arp_foreach(...) 0
#define net_arp_init(...)
#define net_arp_clear_pending(...) 0
#define net_arp_update(...)

#endif /* CONFIG_NET_ARP */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ARP_H */
