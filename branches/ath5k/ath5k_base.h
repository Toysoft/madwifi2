/*-
 * Copyright (c) 2002-2007 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * This file is released under GPLv2
 */

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_ATHVAR_H
#define _DEV_ATH_ATHVAR_H

#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/wireless.h>
#include <linux/if_ether.h>

#include "ath5k.h"

#define	ATH_TIMEOUT		1000

#define ATH_LONG_CALIB		30 /* seconds */
#define ATH_SHORT_CALIB		1

/*
 * Maximum acceptable MTU
 * MAXFRAMEBODY - WEP - QOS - RSN/WPA:
 * 2312 - 8 - 2 - 12 = 2290
 */
#define ATH_MAX_MTU	2290
#define ATH_MIN_MTU	32

#define	ATH_RXBUF	40		/* number of RX buffers */
#define	ATH_TXBUF	200		/* number of TX buffers */
#define	ATH_TXDESC	1		/* number of descriptors per buffer */
#define ATH_BCBUF	1		/* number of beacon buffers */
#define	ATH_TXMAXTRY	11		/* max number of transmit attempts */
#define	ATH_TXINTR_PERIOD 5		/* max number of batched tx descriptors */

#define ATH_BEACON_AIFS_DEFAULT  0	/* default aifs for ap beacon q */
#define ATH_BEACON_CWMIN_DEFAULT 0	/* default cwmin for ap beacon q */
#define ATH_BEACON_CWMAX_DEFAULT 0	/* default cwmax for ap beacon q */

#define ATH_RSSI_LPF_LEN	10
#define ATH_RSSI_DUMMY_MARKER	0x127
#define ATH_EP_MUL(x, mul)	((x) * (mul))
#define ATH_RSSI_IN(x)		(ATH_EP_MUL((x), AR5K_RSSI_EP_MULTIPLIER))
#define ATH_LPF_RSSI(x, y, len) \
    ((x != ATH_RSSI_DUMMY_MARKER) ? (((x) * ((len) - 1) + (y)) / (len)) : (y))
#define ATH_RSSI_LPF(x, y) do {						\
	if ((y) >= -20)							\
		x = ATH_LPF_RSSI((x), ATH_RSSI_IN((y)), ATH_RSSI_LPF_LEN); \
} while (0)

struct ath_buf {
	struct list_head	list;
	unsigned int		flags;	/* tx descriptor flags */
	struct ath_desc		*desc;	/* virtual addr of desc */
	dma_addr_t		daddr;	/* physical addr of desc */
	struct sk_buff		*skb;	/* skbuff for buf */
	dma_addr_t		skbaddr;/* physical addr of skb data */
	struct ieee80211_tx_control ctl;
};

/*
 * Data transmit queue state.  One of these exists for each
 * hardware transmit queue.  Packets sent to us from above
 * are assigned to queues based on their priority.  Not all
 * devices support a complete set of hardware transmit queues.
 * For those devices the array sc_ac2q will map multiple
 * priorities to fewer hardware queues (typically all to one
 * hardware queue).
 */
struct ath_txq {
	unsigned int	qnum;		/* hardware q number */
	u32		*link;		/* link ptr in last TX desc */
	struct list_head q;		/* transmit queue */
	spinlock_t	lock;		/* lock on q and link */
	bool		setup;
};

#if CHAN_DEBUG
#define ATH_CHAN_MAX	(26+26+26+200+200)
#else
#define ATH_CHAN_MAX	(14+14+14+252+20)	/* XXX what's the max? */
#endif

struct ath_softc {
	struct pci_dev		*pdev;		/* for dma mapping */
	void __iomem		*iobase;	/* address of the device */
	struct mutex		lock;		/* dev-level lock */
	struct ieee80211_tx_queue_stats tx_stats;
	struct ieee80211_low_level_stats ll_stats;
	struct ieee80211_hw	*hw;		/* IEEE 802.11 common */
	struct ieee80211_hw_mode modes[NUM_IEEE80211_MODES];
	struct ieee80211_channel channels[ATH_CHAN_MAX];
	struct ieee80211_rate	rates[AR5K_MAX_RATES * NUM_IEEE80211_MODES];
	enum ieee80211_if_types	opmode;
	struct ath_hw		*ah;		/* Atheros HW */

	int			debug;

	struct ath_buf		*bufptr;	/* allocated buffer ptr */
	struct ath_desc		*desc;		/* TX/RX descriptors */
	dma_addr_t		desc_daddr;	/* DMA (physical) address */
	size_t			desc_len;	/* size of TX/RX descriptors */
	u16			cachelsz;	/* cache line size */

	DECLARE_BITMAP(status, 6);
#define ATH_STAT_INVALID	0		/* disable hardware accesses */
#define ATH_STAT_MRRETRY	1		/* multi-rate retry support */
#define ATH_STAT_PROMISC	2
#define ATH_STAT_LEDBLINKING	3		/* LED blink operation active */
#define ATH_STAT_LEDENDBLINK	4		/* finish LED blink operation */
#define ATH_STAT_LEDSOFT	5		/* enable LED gpio status */

	unsigned int		curmode;	/* current phy mode */
	struct ieee80211_channel *curchan;	/* current h/w channel */

	int 			iface_id;	/* add/remove_interface id */

	struct {
		u8	rxflags;	/* radiotap rx flags */
		u8	txflags;	/* radiotap tx flags */
		u16	ledon;		/* softled on time */
		u16	ledoff;		/* softled off time */
	} hwmap[32];				/* h/w rate ix mappings */

	enum ath5k_int		imask;		/* interrupt mask copy */

	DECLARE_BITMAP(keymap, AR5K_KEYCACHE_SIZE); /* key use bit map */

	u8			bssidmask[ETH_ALEN];

	unsigned int		led_pin,	/* GPIO pin for driving LED */
				led_on,		/* pin setting for LED on */
				led_off;	/* off time for current blink */
	struct timer_list	led_tim;	/* led off timer */
	u8			led_rxrate;	/* current rx rate for LED */
	u8			led_txrate;	/* current tx rate for LED */

	struct tasklet_struct	restq;		/* reset tasklet */

	unsigned int		rxbufsize;	/* rx size based on mtu */
	struct list_head	rxbuf;		/* receive buffer */
	spinlock_t		rxbuflock;
	u32			*rxlink;	/* link ptr in last RX desc */
	struct tasklet_struct	rxtq;		/* rx intr tasklet */

	struct list_head	txbuf;		/* transmit buffer */
	spinlock_t		txbuflock;
	unsigned int		txbuf_len;	/* buf count in txbuf list */
	struct ath_txq		txqs[2];	/* beacon and tx */

	struct ath_txq		*txq;		/* beacon and tx*/
	struct tasklet_struct	txtq;		/* tx intr tasklet */

	struct ath_buf		*bbuf;		/* beacon buffer */
	unsigned int		bhalq,		/* HAL q for outgoing beacons */
				bmisscount,	/* missed beacon transmits */
				bintval,	/* beacon interval */
				bsent;

	struct timer_list	calib_tim;	/* calibration timer */
};

#define ath5k_hw_hasbssidmask(_ah) \
	(ath5k_hw_get_capability(_ah, AR5K_CAP_BSSIDMASK, 0, NULL) == 0)
#define ath5k_hw_hasveol(_ah) \
	(ath5k_hw_get_capability(_ah, AR5K_CAP_VEOL, 0, NULL) == 0)

#endif
