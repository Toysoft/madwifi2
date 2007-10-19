/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * Copyright (c) 2004-2005 Atheros Communications, Inc.
 * Copyright (c) 2006 Devicescape Software, Inc.
 * Copyright (c) 2007 Jiri Slaby <jirislaby@gmail.com>
 * Copyright (c) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */
#define	ATH_PCI_VERSION	"0.9.5.0-BSD"

#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/cache.h>
#include <linux/pci.h>
#include <linux/ethtool.h>
#include <linux/uaccess.h>

#include <net/ieee80211_radiotap.h>

#include <asm/unaligned.h>

#include "base.h"
#include "reg.h"

#define ATH_DEBUG_MODES		0 /* Show found modes in the log? */
#define ATH_DUMP_SKB		0 /* show skb contents */
#define AR_DEBUG		1

/* unaligned little endian access */
#define LE_READ_2(_p) (le16_to_cpu(get_unaligned((__le16 *)(_p))))
#define LE_READ_4(_p) (le32_to_cpu(get_unaligned((__le32 *)(_p))))

#if AR_DEBUG
#define	DPRINTF(sc, _m, _fmt...) do {				\
	if (unlikely(((sc)->debug & (_m)) && net_ratelimit()))	\
		printk(KERN_DEBUG _fmt);			\
} while (0)
#else
static inline int __attribute__ ((format (printf, 3, 4)))
DPRINTF(struct ath_softc *sc, unsigned int m, const char *fmt, ...)
{
	return 0;
}
#endif
enum {
	ATH_DEBUG_XMIT		= 0x00000001,	/* basic xmit operation */
	ATH_DEBUG_RESET		= 0x00000020,	/* reset processing */
	ATH_DEBUG_MODE		= 0x00000040,	/* mode init/setup */
	ATH_DEBUG_BEACON	= 0x00000080,	/* beacon handling */
	ATH_DEBUG_INTR		= 0x00001000,	/* ISR */
	ATH_DEBUG_BEACON_PROC	= 0x00008000,	/* beacon ISR proc */
	ATH_DEBUG_CALIBRATE	= 0x00010000,	/* periodic calibration */
	ATH_DEBUG_LED		= 0x00100000,	/* led management */
	ATH_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	ATH_DEBUG_ANY		= 0xffffffff
};

enum {
	ATH_LED_TX,
	ATH_LED_RX,
};

static int ath_calinterval = 1; /* Calibrate PHY every 1 sec (TODO: Fixme) */

#if AR_DEBUG
static unsigned int ath_debug;
module_param_named(debug, ath_debug, uint, 0);
#endif

/*
 * User a static table of PCI id's for now.  While this is the
 * "new way" to do things, we may want to switch back to having
 * the HAL check them by defining a probe method.
 */
static struct pci_device_id ath_pci_id_table[] __devinitdata = {
	{ PCI_VDEVICE(ATHEROS, 0x0207), .driver_data = AR5K_AR5210 }, /* 5210 early */
	{ PCI_VDEVICE(ATHEROS, 0x0007), .driver_data = AR5K_AR5210 }, /* 5210 */
	{ PCI_VDEVICE(ATHEROS, 0x0011), .driver_data = AR5K_AR5211 }, /* 5311 */
	{ PCI_VDEVICE(ATHEROS, 0x0012), .driver_data = AR5K_AR5211 }, /* 5211 */
	{ PCI_VDEVICE(ATHEROS, 0x0013), .driver_data = AR5K_AR5212 }, /* 5212 */
	{ PCI_VDEVICE(3COM_2,  0x0013), .driver_data = AR5K_AR5212 }, /* 3com 5212 */
	{ PCI_VDEVICE(3COM,    0x0013), .driver_data = AR5K_AR5212 }, /* 3com 3CRDAG675 5212 */
	{ PCI_VDEVICE(ATHEROS, 0x1014), .driver_data = AR5K_AR5212 }, /* IBM minipci 5212 */
	{ PCI_VDEVICE(ATHEROS, 0x0014), .driver_data = AR5K_AR5212 },
	{ PCI_VDEVICE(ATHEROS, 0x0015), .driver_data = AR5K_AR5212 },
	{ PCI_VDEVICE(ATHEROS, 0x0016), .driver_data = AR5K_AR5212 },
	{ PCI_VDEVICE(ATHEROS, 0x0017), .driver_data = AR5K_AR5212 },
	{ PCI_VDEVICE(ATHEROS, 0x0018), .driver_data = AR5K_AR5212 },
	{ PCI_VDEVICE(ATHEROS, 0x0019), .driver_data = AR5K_AR5212 },
	{ PCI_VDEVICE(ATHEROS, 0x001a), .driver_data = AR5K_AR5212 }, /* 2413 Griffin-lite */
	{ PCI_VDEVICE(ATHEROS, 0x001b), .driver_data = AR5K_AR5212 }, /* 5413 Eagle */
	{ PCI_VDEVICE(ATHEROS, 0x001c), .driver_data = AR5K_AR5212 }, /* 5424 Condor (PCI-E)*/
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ath_pci_id_table);

static void ath_led_event(struct ath_softc *, int);
static int ath_reset(struct ieee80211_hw *);

#if AR_DEBUG
static void ath_printrxbuf(struct ath_buf *bf, int done)
{
	struct ath_desc *ds = bf->desc;

	printk(KERN_DEBUG "R (%p %llx) %08x %08x %08x %08x %08x %08x %c\n",
		ds, (unsigned long long)bf->daddr,
		ds->ds_link, ds->ds_data, ds->ds_ctl0, ds->ds_ctl1,
		ds->ds_hw[0], ds->ds_hw[1],
		!done ? ' ' : (ds->ds_rxstat.rs_status == 0) ? '*' : '!');
}

static void ath_printtxbuf(struct ath_buf *bf, int done)
{
	struct ath_desc *ds = bf->desc;

	printk(KERN_DEBUG "T (%p %llx) %08x %08x %08x %08x %08x %08x %08x "
		"%08x %c\n", ds, (unsigned long long)bf->daddr, ds->ds_link,
		ds->ds_data, ds->ds_ctl0, ds->ds_ctl1,
		ds->ds_hw[0], ds->ds_hw[1], ds->ds_hw[2], ds->ds_hw[3],
		!done ? ' ' : (ds->ds_txstat.ts_status == 0) ? '*' : '!');
}
#endif

#if ATH_DUMP_SKB
static inline void ath_dump_skb(struct sk_buff *skb, const char *prefix)
{
	print_hex_dump_bytes(prefix, DUMP_PREFIX_NONE, skb->data,
			min(200U, skb->len));
}
#else
static inline void ath_dump_skb(struct sk_buff *skb, const char *prefix) {}
#endif

static inline void ath_cleanup_txbuf(struct ath_softc *sc, struct ath_buf *bf)
{
	BUG_ON(!bf);
	if (!bf->skb)
		return;
	pci_unmap_single(sc->pdev, bf->skbaddr, bf->skb->len,
			PCI_DMA_TODEVICE);
	dev_kfree_skb(bf->skb);
	bf->skb = NULL;
}

static void ath_tasklet_reset(unsigned long data)
{
	struct ath_softc *sc = (void *)data;

	ath_reset(sc->hw);
}

static void ath_tx_processq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ieee80211_tx_status txs = {};
	struct ath_buf *bf, *bf0;
	struct ath_desc *ds;
	struct sk_buff *skb;
	int ret;

	spin_lock(&txq->lock);
	list_for_each_entry_safe(bf, bf0, &txq->q, list) {
		ds = bf->desc;

		/* TODO only one segment */
		pci_dma_sync_single_for_cpu(sc->pdev, sc->desc_daddr,
				sc->desc_len, PCI_DMA_FROMDEVICE);
		ret = sc->ah->ah_proc_tx_desc(sc->ah, ds);
		if (unlikely(ret == -EINPROGRESS))
			break;
		else if (unlikely(ret)) {
			printk(KERN_ERR "ath: error %d while processing "
				"queue %u\n", ret, txq->qnum);
			break;
		}

		skb = bf->skb;
		bf->skb = NULL;
		pci_unmap_single(sc->pdev, bf->skbaddr, skb->len,
				PCI_DMA_TODEVICE);

		txs.control = bf->ctl;
		txs.retry_count = ds->ds_txstat.ts_shortretry +
			ds->ds_txstat.ts_longretry / 6;
		if (unlikely(ds->ds_txstat.ts_status)) {
			sc->ll_stats.dot11ACKFailureCount++;
			if (ds->ds_txstat.ts_status & AR5K_TXERR_XRETRY)
				txs.excessive_retries = 1;
			else if (ds->ds_txstat.ts_status & AR5K_TXERR_FILT)
				txs.flags |= IEEE80211_TX_STATUS_TX_FILTERED;
		} else {
			txs.flags |= IEEE80211_TX_STATUS_ACK;
			txs.ack_signal = ds->ds_txstat.ts_rssi;
		}

		ieee80211_tx_status(sc->hw, skb, &txs);
		sc->tx_stats.data[txq->qnum].count++;

		spin_lock(&sc->txbuflock);
		sc->tx_stats.data[txq->qnum].len--;
		list_move_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		spin_unlock(&sc->txbuflock);
	}
	if (likely(list_empty(&txq->q)))
		txq->link = NULL;
	spin_unlock(&txq->lock);
	if (sc->txbuf_len > ATH_TXBUF / 5)
		ieee80211_wake_queues(sc->hw);
}

static void ath_tasklet_tx(unsigned long data)
{
	struct ath_softc *sc = (void *)data;

	ath_tx_processq(sc, sc->txq);

	ath_led_event(sc, ATH_LED_TX);
}

static int ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hw *ah = sc->ah;
	struct sk_buff *skb = bf->skb;
	struct ath_desc *ds;

	if (likely(skb == NULL)) {
		unsigned int off;

		/*
		 * Allocate buffer with headroom_needed space for the
		 * fake physical layer header at the start.
		 */
		skb = dev_alloc_skb(sc->rxbufsize + sc->cachelsz - 1);
		if (unlikely(skb == NULL)) {
			printk(KERN_ERR "ath: can't alloc skbuff of size %u\n",
					sc->rxbufsize + sc->cachelsz - 1);
			return -ENOMEM;
		}
		/*
		 * Cache-line-align.  This is important (for the
		 * 5210 at least) as not doing so causes bogus data
		 * in rx'd frames.
		 */
		off = ((unsigned long)skb->data) % sc->cachelsz;
		if (off != 0)
			skb_reserve(skb, sc->cachelsz - off);

		bf->skb = skb;
		bf->skbaddr = pci_map_single(sc->pdev,
			skb->data, sc->rxbufsize, PCI_DMA_FROMDEVICE);
		if (unlikely(pci_dma_mapping_error(bf->skbaddr))) {
			printk(KERN_ERR "%s: DMA mapping failed\n", __func__);
			dev_kfree_skb(skb);
			bf->skb = NULL;
			return -ENOMEM;
		}
	}

	/*
	 * Setup descriptors.  For receive we always terminate
	 * the descriptor list with a self-linked entry so we'll
	 * not get overrun under high load (as can happen with a
	 * 5212 when ANI processing enables PHY error frames).
	 *
	 * To insure the last descriptor is self-linked we create
	 * each descriptor as self-linked and add it to the end.  As
	 * each additional descriptor is added the previous self-linked
	 * entry is ``fixed'' naturally.  This should be safe even
	 * if DMA is happening.  When processing RX interrupts we
	 * never remove/process the last, self-linked, entry on the
	 * descriptor list.  This insures the hardware always has
	 * someplace to write a new frame.
	 */
	ds = bf->desc;
	ds->ds_link = bf->daddr;	/* link to self */
	ds->ds_data = bf->skbaddr;
	ath5k_hw_setup_rx_desc(ah, ds,
		skb_tailroom(skb),	/* buffer size */
		0);

	if (sc->rxlink != NULL)
		*sc->rxlink = bf->daddr;
	sc->rxlink = &ds->ds_link;
	return 0;
}

static unsigned int ath_rx_decrypted(struct ath_softc *sc,
		struct ath_desc *ds, struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (void *)skb->data;
	unsigned int keyix, hlen = ieee80211_get_hdrlen_from_skb(skb);

	if (!(ds->ds_rxstat.rs_status & AR5K_RXERR_DECRYPT) &&
			ds->ds_rxstat.rs_keyix != AR5K_RXKEYIX_INVALID)
		return RX_FLAG_DECRYPTED;

	/* Apparently when a default key is used to decrypt the packet
	   the hal does not set the index used to decrypt.  In such cases
	   get the index from the packet. */
	if ((le16_to_cpu(hdr->frame_control) & IEEE80211_FCTL_PROTECTED) &&
			!(ds->ds_rxstat.rs_status & AR5K_RXERR_DECRYPT) &&
			skb->len >= hlen + 4) {
		keyix = skb->data[hlen + 3] >> 6;

		if (test_bit(keyix, sc->keymap))
			return RX_FLAG_DECRYPTED;
	}

	return 0;
}

static inline u64 ath_extend_tsf(struct ath_hw *ah, u32 rstamp)
{
	u64 tsf = ath5k_hw_get_tsf64(ah);

	if ((tsf & 0x7fff) < rstamp)
		tsf -= 0x8000;

	return (tsf & ~0x7fff) | rstamp;
}

static void ath_tasklet_rx(unsigned long data)
{
	struct ieee80211_rx_status rxs = {};
	struct sk_buff *skb;
	struct ath_softc *sc = (void *)data;
	struct ath_buf *bf;
	struct ath_desc *ds;
	u16 len;
	u8 stat;
	int ret;
	int hdrlen;
	int pad;

	spin_lock(&sc->rxbuflock);
	do {
		if (unlikely(list_empty(&sc->rxbuf))) {
			if (net_ratelimit())
				printk(KERN_WARNING "ath: empty rx buf pool\n");
			break;
		}
		bf = list_first_entry(&sc->rxbuf, struct ath_buf, list);
		BUG_ON(bf->skb == NULL);
		skb = bf->skb;
		ds = bf->desc;

		/* TODO only one segment */
		pci_dma_sync_single_for_cpu(sc->pdev, sc->desc_daddr,
				sc->desc_len, PCI_DMA_FROMDEVICE);

		if (unlikely(ds->ds_link == bf->daddr)) /* this is the end */
			break;

		ret = sc->ah->ah_proc_rx_desc(sc->ah, ds);
		if (unlikely(ret == -EINPROGRESS))
			break;
		else if (unlikely(ret)) {
			if (net_ratelimit())
				printk(KERN_ERR "ath: error in processing rx "
					"descriptor\n");
			return;
		}

		if (unlikely(ds->ds_rxstat.rs_more)) {
			if (net_ratelimit())
				printk(KERN_INFO "ath: unsupported jumbo\n");
			goto next;
		}

		stat = ds->ds_rxstat.rs_status;
		if (unlikely(stat)) {
			if (stat & AR5K_RXERR_PHY)
				goto next;
			if (stat & AR5K_RXERR_DECRYPT) {
				/*
				 * Decrypt error.  If the error occurred
				 * because there was no hardware key, then
				 * let the frame through so the upper layers
				 * can process it.  This is necessary for 5210
				 * parts which have no way to setup a ``clear''
				 * key cache entry.
				 *
				 * XXX do key cache faulting
				 */
				if (ds->ds_rxstat.rs_keyix ==
						AR5K_RXKEYIX_INVALID &&
						!(stat & AR5K_RXERR_CRC))
					goto accept;
			}
			if (stat & AR5K_RXERR_MIC) {
				rxs.flag |= RX_FLAG_MMIC_ERROR;
				goto accept;
			}

			/* let crypto-error packets fall through in MNTR */
			if ((stat & ~(AR5K_RXERR_DECRYPT|AR5K_RXERR_MIC)) ||
					sc->opmode != IEEE80211_IF_TYPE_MNTR)
				goto next;
		}
accept:
		len = ds->ds_rxstat.rs_datalen;
		pci_dma_sync_single_for_cpu(sc->pdev, bf->skbaddr, len,
				PCI_DMA_FROMDEVICE);
		pci_unmap_single(sc->pdev, bf->skbaddr, sc->rxbufsize,
				PCI_DMA_FROMDEVICE);
		bf->skb = NULL;

		skb_put(skb, len);

		/*
		 * the hardware adds a padding to 4 byte boundaries between
		 * the header and the payload data if the header length is
		 * not multiples of 4 - remove it
		 */
		hdrlen = ieee80211_get_hdrlen_from_skb(skb);
		if (hdrlen & 3) {
			pad = hdrlen % 4;
			memmove(skb->data + pad, skb->data, hdrlen);
			skb_pull(skb, pad);
		}

		if (sc->opmode == IEEE80211_IF_TYPE_MNTR)
			rxs.mactime = ath_extend_tsf(sc->ah,
					ds->ds_rxstat.rs_tstamp);
		else
			rxs.mactime = ds->ds_rxstat.rs_tstamp;
		rxs.freq = sc->curchan->freq;
		rxs.channel = sc->curchan->chan;
		rxs.phymode = sc->curmode;
		rxs.ssi = ds->ds_rxstat.rs_rssi;
		rxs.antenna = ds->ds_rxstat.rs_antenna;
		rxs.rate = ds->ds_rxstat.rs_rate;
		rxs.flag |= ath_rx_decrypted(sc, ds, skb);

		ath_dump_skb(skb, "RX  ");

		__ieee80211_rx(sc->hw, skb, &rxs);
		sc->led_rxrate = ds->ds_rxstat.rs_rate;
		ath_led_event(sc, ATH_LED_RX);
next:
		list_move_tail(&bf->list, &sc->rxbuf);
	} while (ath_rxbuf_init(sc, bf) == 0);
	spin_unlock(&sc->rxbuflock);
}

/*
 * Setup the beacon frame for transmit.
 */
static int ath_beacon_setup(struct ath_softc *sc, struct ath_buf *bf,
		struct ieee80211_tx_control *ctl)
{
	struct sk_buff *skb = bf->skb;
	struct ath_hw *ah = sc->ah;
	struct ath_desc *ds;
	int ret, antenna = 0;
	u32 flags;

	bf->skbaddr = pci_map_single(sc->pdev, skb->data, skb->len,
			PCI_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_BEACON, "%s: skb %p [data %p len %u] "
			"skbaddr %llx\n", __func__, skb, skb->data, skb->len,
			(unsigned long long)bf->skbaddr);
	if (pci_dma_mapping_error(bf->skbaddr)) {
		printk(KERN_ERR "ath: beacon DMA mapping failed\n");
		return -EIO;
	}

	ds = bf->desc;

	flags = AR5K_TXDESC_NOACK;
	if (sc->opmode == IEEE80211_IF_TYPE_IBSS && ath5k_hw_hasveol(ah)) {
		ds->ds_link = bf->daddr;	/* self-linked */
		flags |= AR5K_TXDESC_VEOL;
		/*
		 * Let hardware handle antenna switching if txantenna is not set
		 */
	} else {
		ds->ds_link = 0;
		/*
		 * Switch antenna every 4 beacons if txantenna is not set
		 * XXX assumes two antennas
		 */
		if (antenna == 0)
			antenna = sc->bsent & 4 ? 2 : 1;
	}

	ds->ds_data = bf->skbaddr;
	ret = ah->ah_setup_tx_desc(ah, ds, skb->len + FCS_LEN,
			ieee80211_get_hdrlen_from_skb(skb),
			AR5K_PKT_TYPE_BEACON, 0xffff, ctl->tx_rate, 1,
			AR5K_TXKEYIX_INVALID, antenna, flags, 0, 0);
	if (ret)
		goto err_unmap;
	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	ret = ah->ah_fill_tx_desc(ah, ds, roundup(skb->len, 4), true, true);
	if (ret)
		goto err_unmap;

	return 0;
err_unmap:
	pci_unmap_single(sc->pdev, bf->skbaddr, skb->len, PCI_DMA_TODEVICE);
	return ret;
}

/*
 * Transmit a beacon frame at SWBA.  Dynamic updates to the
 * frame contents are done as needed and the slot time is
 * also adjusted based on current state.
 *
 * this is usually called from interrupt context (ath_intr())
 * but also from ath_beacon_config() in IBSS mode which in turn
 * can be called from a tasklet and user context
 */
static void ath_beacon_send(struct ath_softc *sc)
{
	struct ath_buf *bf = sc->bbuf;
	struct ath_hw *ah = sc->ah;

	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s\n", __func__);

	if (unlikely(bf->skb == NULL || sc->opmode == IEEE80211_IF_TYPE_STA ||
			sc->opmode == IEEE80211_IF_TYPE_MNTR)) {
		printk(KERN_WARNING "ath: bf=%p bf_skb=%p\n", bf,
				bf ? bf->skb : NULL);
		return;
	}
	/*
	 * Check if the previous beacon has gone out.  If
	 * not don't don't try to post another, skip this
	 * period and wait for the next.  Missed beacons
	 * indicate a problem and should not occur.  If we
	 * miss too many consecutive beacons reset the device.
	 */
	if (unlikely(ath5k_hw_num_tx_pending(ah, sc->bhalq) != 0)) {
		sc->bmisscount++;
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: missed %u consecutive beacons\n",
			__func__, sc->bmisscount);
		if (sc->bmisscount > 3) {		/* NB: 3 is a guess */
			DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
				"%s: stuck beacon time (%u missed)\n",
				__func__, sc->bmisscount);
			tasklet_schedule(&sc->restq);
		}
		return;
	}
	if (unlikely(sc->bmisscount != 0)) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: resume beacon xmit after %u misses\n",
			__func__, sc->bmisscount);
		sc->bmisscount = 0;
	}

	/*
	 * Stop any current dma and put the new frame on the queue.
	 * This should never fail since we check above that no frames
	 * are still pending on the queue.
	 */
	if (unlikely(ath5k_hw_stop_tx_dma(ah, sc->bhalq))) {
		printk(KERN_WARNING "ath: beacon queue %u didn't stop?\n",
				sc->bhalq);
		/* NB: the HAL still stops DMA, so proceed */
	}
	pci_dma_sync_single_for_cpu(sc->pdev, bf->skbaddr, bf->skb->len,
			PCI_DMA_TODEVICE);

	ath5k_hw_put_tx_buf(ah, sc->bhalq, bf->daddr);
	ath5k_hw_tx_start(ah, sc->bhalq);
	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: TXDP[%u] = %llx (%p)\n",
		__func__, sc->bhalq, (unsigned long long)bf->daddr, bf->desc);

	sc->bsent++;
}

static int ath_beaconq_config(struct ath_softc *sc)
{
	struct ath_hw *ah = sc->ah;
	struct ath5k_txq_info qi;
	int ret;

	ret = ath5k_hw_get_tx_queueprops(ah, sc->bhalq, &qi);
	if (ret)
		return ret;
	if (sc->opmode == IEEE80211_IF_TYPE_AP ||
			sc->opmode == IEEE80211_IF_TYPE_IBSS) {
		/*
		 * Always burst out beacon and CAB traffic
		 * (aifs = cwmin = cwmax = 0)
		 */
		qi.tqi_aifs = 0;
		qi.tqi_cw_min = 0;
		qi.tqi_cw_max = 0;
	}

	ret = ath5k_hw_setup_tx_queueprops(ah, sc->bhalq, &qi);
	if (ret) {
		printk(KERN_ERR "%s: unable to update parameters for beacon "
			"hardware queue!\n", __func__);
		return ret;
	}

	return ath5k_hw_reset_tx_queue(ah, sc->bhalq); /* push to h/w */;
}

/*
 * Configure the beacon and sleep timers.
 *
 * When operating as an AP this resets the TSF and sets
 * up the hardware to notify us when we need to issue beacons.
 *
 * When operating in station mode this sets up the beacon
 * timers according to the timestamp of the last received
 * beacon and the current TSF, configures PCF and DTIM
 * handling, programs the sleep registers so the hardware
 * will wakeup in time to receive beacons, and configures
 * the beacon miss handling so we'll receive a BMISS
 * interrupt when we stop seeing beacons from the AP
 * we've associated with.
 */
static void ath_beacon_config(struct ath_softc *sc)
{
#define TSF_TO_TU(_h, _l)	(((_h) << 22) | ((_l) >> 10))
	struct ath_hw *ah = sc->ah;
	u32 uninitialized_var(nexttbtt), intval, tsftu;
	u64 tsf;

	intval = sc->bintval & AR5K_BEACON_PERIOD;
	if (WARN_ON(!intval))
		return;

	/* current TSF converted to TU */
	tsf = ath5k_hw_get_tsf64(ah);
	tsftu = TSF_TO_TU((u32)(tsf >> 32), (u32)tsf);

	DPRINTF(sc, ATH_DEBUG_BEACON, "%s: intval %u hw tsftu %u\n", __func__,
			intval, tsftu);

	if (sc->opmode == IEEE80211_IF_TYPE_STA ||
			(sc->opmode == IEEE80211_IF_TYPE_IBSS &&
				!sc->bbuf->skb)) {
		ath5k_hw_set_intr(ah, 0);
		sc->imask |= AR5K_INT_BMISS;
		sc->bmisscount = 0;
		ath5k_hw_set_intr(ah, sc->imask);
	} else if (sc->opmode == IEEE80211_IF_TYPE_IBSS /* TODO || AP */) {
		ath5k_hw_set_intr(ah, 0);
		if (sc->opmode == IEEE80211_IF_TYPE_IBSS) {
			/*
			 * Pull nexttbtt forward to reflect the current
			 * TSF. Add one intval otherwise the timespan
			 * can be too short for ibss merges.
			 */
			nexttbtt = tsftu + 2 * intval;

			DPRINTF(sc, ATH_DEBUG_BEACON, "%s: nexttbtt %u "
				"intval %u\n", __func__, nexttbtt, intval);

			/*
			 * In IBSS mode enable the beacon timers but only
			 * enable SWBA interrupts if we need to manually
			 * prepare beacon frames.  Otherwise we use a
			 * self-linked tx descriptor and let the hardware
			 * deal with things.
			 */
			if (!ath5k_hw_hasveol(ah))
				sc->imask |= AR5K_INT_SWBA;
		} /* TODO else AP */

		intval |= AR5K_BEACON_ENA;

		ath_beaconq_config(sc);
		ath5k_hw_init_beacon(ah, nexttbtt, intval);

		sc->bmisscount = 0;
		ath5k_hw_set_intr(ah, sc->imask);
		/*
		 * When using a self-linked beacon descriptor in
		 * ibss mode load it once here.
		 */
		if (sc->opmode == IEEE80211_IF_TYPE_IBSS &&
				ath5k_hw_hasveol(ah))
			ath_beacon_send(sc);
	}
#undef TSF_TO_TU
}

static void ath_mode_init(struct ath_softc *sc)
{
	struct ath_hw *ah = sc->ah;
	u32 rfilt;

	/* configure rx filter */
	rfilt = sc->filter_flags;
	ath5k_hw_set_rx_filter(ah, rfilt);

	if (ath5k_hw_hasbssidmask(ah))
		ath5k_hw_set_bssid_mask(ah, sc->bssidmask);

	/* configure operational mode */
	ath5k_hw_set_opmode(ah);

	ath5k_hw_set_mcast_filter(ah, 0, 0);
	DPRINTF(sc, ATH_DEBUG_MODE, "%s: RX filter 0x%x\n", __func__, rfilt);
}

/*
 * Enable the receive h/w following a reset.
 */
static int ath_startrecv(struct ath_softc *sc)
{
	struct ath_hw *ah = sc->ah;
	struct ath_buf *bf;
	int ret;

	sc->rxbufsize = roundup(IEEE80211_MAX_LEN, sc->cachelsz);

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: cachelsz %u rxbufsize %u\n",
		__func__, sc->cachelsz, sc->rxbufsize);

	sc->rxlink = NULL;

	spin_lock_bh(&sc->rxbuflock);
	list_for_each_entry(bf, &sc->rxbuf, list) {
		ret = ath_rxbuf_init(sc, bf);
		if (ret != 0) {
			spin_unlock_bh(&sc->rxbuflock);
			goto err;
		}
	}
	bf = list_first_entry(&sc->rxbuf, struct ath_buf, list);
	spin_unlock_bh(&sc->rxbuflock);

	ath5k_hw_put_rx_buf(ah, bf->daddr);
	ath5k_hw_start_rx(ah);		/* enable recv descriptors */
	ath_mode_init(sc);		/* set filters, etc. */
	ath5k_hw_start_rx_pcu(ah);	/* re-enable PCU/DMA engine */

	return 0;
err:
	return ret;
}

static inline void ath_update_txpow(struct ath_softc *sc)
{
	ath5k_hw_set_txpower_limit(sc->ah, 0);
}

static int ath_stop_locked(struct ath_softc *);

static int ath_init(struct ath_softc *sc)
{
	int ret;

	mutex_lock(&sc->lock);

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: mode %d\n", __func__, sc->opmode);

	/*
	 * Stop anything previously setup.  This is safe
	 * no matter this is the first time through or not.
	 */
	ath_stop_locked(sc);

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	sc->curchan = sc->hw->conf.chan;
	ret = ath5k_hw_reset(sc->ah, sc->opmode, sc->curchan, false);
	if (ret) {
		printk(KERN_ERR "unable to reset hardware: %d\n", ret);
		goto done;
	}
	/*
	 * This is needed only to setup initial state
	 * but it's best done after a reset.
	 */
	ath_update_txpow(sc);

	/*
	 * Setup the hardware after reset: the key cache
	 * is filled as needed and the receive engine is
	 * set going.  Frame transmit is handled entirely
	 * in the frame output path; there's nothing to do
	 * here except setup the interrupt mask.
	 */
	ret = ath_startrecv(sc);
	if (ret)
		goto done;

	/*
	 * Enable interrupts.
	 */
	sc->imask = AR5K_INT_RX | AR5K_INT_TX | AR5K_INT_RXEOL |
		AR5K_INT_RXORN | AR5K_INT_FATAL | AR5K_INT_GLOBAL;

	ath5k_hw_set_intr(sc->ah, sc->imask);

	mod_timer(&sc->calib_tim, round_jiffies(jiffies +
			msecs_to_jiffies(ath_calinterval * 1000)));

	ret = 0;
done:
	mutex_unlock(&sc->lock);
	return ret;
}

/*
 * Disable the receive h/w in preparation for a reset.
 */
static void ath_stoprecv(struct ath_softc *sc)
{
	struct ath_hw *ah = sc->ah;

	ath5k_hw_stop_pcu_recv(ah);	/* disable PCU */
	ath5k_hw_set_rx_filter(ah, 0);	/* clear recv filter */
	ath5k_hw_stop_rx_dma(ah);	/* disable DMA engine */
	mdelay(3);			/* 3ms is long enough for 1 frame */
#if AR_DEBUG
	if (unlikely(sc->debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL))) {
		struct ath_desc *ds;
		struct ath_buf *bf;
		int status;

		printk(KERN_DEBUG "%s: rx queue %x, link %p\n", __func__,
			ath5k_hw_get_rx_buf(ah), sc->rxlink);

		spin_lock_bh(&sc->rxbuflock);
		list_for_each_entry(bf, &sc->rxbuf, list) {
			ds = bf->desc;
			status = ah->ah_proc_rx_desc(ah, ds);
			if (!status || (sc->debug & ATH_DEBUG_FATAL))
				ath_printrxbuf(bf, status == 0);
		}
		spin_unlock_bh(&sc->rxbuflock);
	}
#endif
	sc->rxlink = NULL;		/* just in case */
}

static void ath_tx_draintxq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_buf *bf, *bf0;

	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath_tx_tasklet
	 */
	spin_lock_bh(&txq->lock);
	list_for_each_entry_safe(bf, bf0, &txq->q, list) {
#if AR_DEBUG
		if (sc->debug & ATH_DEBUG_RESET)
			ath_printtxbuf(bf, !sc->ah->ah_proc_tx_desc(sc->ah,
						bf->desc));
#endif
		ath_cleanup_txbuf(sc, bf);

		spin_lock_bh(&sc->txbuflock);
		sc->tx_stats.data[txq->qnum].len--;
		list_move_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		spin_unlock_bh(&sc->txbuflock);
	}
	txq->link = NULL;
	spin_unlock_bh(&txq->lock);
}

/*
 * Drain the transmit queues and reclaim resources.
 */
static void ath_draintxq(struct ath_softc *sc)
{
	struct ath_hw *ah = sc->ah;
	int i;

	/* XXX return value */
	if (likely(!test_bit(ATH_STAT_INVALID, sc->status))) {
		/* don't touch the hardware if marked invalid */
		(void)ath5k_hw_stop_tx_dma(ah, sc->bhalq);
		DPRINTF(sc, ATH_DEBUG_RESET, "%s: beacon queue %x\n", __func__,
			ath5k_hw_get_tx_buf(ah, sc->bhalq));
		for (i = 0; i < ARRAY_SIZE(sc->txqs); i++)
			if (sc->txqs[i].setup) {
				ath5k_hw_stop_tx_dma(ah, sc->txqs[i].qnum);
				DPRINTF(sc, ATH_DEBUG_RESET, "%s: txq [%u] %x, "
					"link %p\n", __func__,
					sc->txqs[i].qnum,
					ath5k_hw_get_tx_buf(ah,
							sc->txqs[i].qnum),
					sc->txqs[i].link);
			}
	}
	ieee80211_start_queues(sc->hw); /* XXX move to callers */

	for (i = 0; i < ARRAY_SIZE(sc->txqs); i++)
		if (sc->txqs[i].setup)
			ath_tx_draintxq(sc, &sc->txqs[i]);
}

static int ath_stop_locked(struct ath_softc *sc)
{
	struct ath_hw *ah = sc->ah;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: invalid %u\n", __func__,
			test_bit(ATH_STAT_INVALID, sc->status));

	/*
	 * Shutdown the hardware and driver:
	 *    stop output from above
	 *    disable interrupts
	 *    turn off timers
	 *    turn off the radio
	 *    clear transmit machinery
	 *    clear receive machinery
	 *    drain and release tx queues
	 *    reclaim beacon resources
	 *    power down hardware
	 *
	 * Note that some of this work is not possible if the
	 * hardware is gone (invalid).
	 */
	ieee80211_stop_queues(sc->hw);

	if (!test_bit(ATH_STAT_INVALID, sc->status)) {
		if (test_bit(ATH_STAT_LEDSOFT, sc->status)) {
			del_timer_sync(&sc->led_tim);
			ath5k_hw_set_gpio(ah, sc->led_pin, !sc->led_on);
			__clear_bit(ATH_STAT_LEDBLINKING, sc->status);
		}
		ath5k_hw_set_intr(ah, 0);
	}
	ath_draintxq(sc);
	if (!test_bit(ATH_STAT_INVALID, sc->status)) {
		ath_stoprecv(sc);
		ath5k_hw_phy_disable(ah);
	} else
		sc->rxlink = NULL;

	return 0;
}

/*
 * Stop the device, grabbing the top-level lock to protect
 * against concurrent entry through ath_init (which can happen
 * if another thread does a system call and the thread doing the
 * stop is preempted).
 */
static int ath_stop_hw(struct ath_softc *sc)
{
	int ret;

	mutex_lock(&sc->lock);
	ret = ath_stop_locked(sc);
	if (ret == 0 && !test_bit(ATH_STAT_INVALID, sc->status)) {
		/*
		 * Set the chip in full sleep mode.  Note that we are
		 * careful to do this only when bringing the interface
		 * completely to a stop.  When the chip is in this state
		 * it must be carefully woken up or references to
		 * registers in the PCI clock domain may freeze the bus
		 * (and system).  This varies by chip and is mostly an
		 * issue with newer parts that go to sleep more quickly.
		 */
		if (sc->ah->ah_mac_version >= 7 &&
				sc->ah->ah_mac_revision >= 8) {
			/*
			 * XXX
			 * don't put newer MAC revisions > 7.8 to sleep because
			 * of the above mentioned problems
			 */
			DPRINTF(sc, ATH_DEBUG_RESET, "%s: mac version > 7.8, "
				"not putting device to sleep\n", __func__);
		} else {
			DPRINTF(sc, ATH_DEBUG_RESET,
				"%s: putting device to full sleep\n", __func__);
			ath5k_hw_set_power(sc->ah, AR5K_PM_FULL_SLEEP, true, 0);
		}
	}
	ath_cleanup_txbuf(sc, sc->bbuf);
	mutex_unlock(&sc->lock);

	del_timer_sync(&sc->calib_tim);

	return ret;
}

static void ath_setcurmode(struct ath_softc *sc, unsigned int mode)
{
	if (unlikely(test_bit(ATH_STAT_LEDSOFT, sc->status))) {
		/* from Atheros NDIS driver, w/ permission */
		static const struct {
			u16 rate;	/* tx/rx 802.11 rate */
			u16 timeOn;	/* LED on time (ms) */
			u16 timeOff;	/* LED off time (ms) */
		} blinkrates[] = {
			{ 108,  40,  10 },
			{  96,  44,  11 },
			{  72,  50,  13 },
			{  48,  57,  14 },
			{  36,  67,  16 },
			{  24,  80,  20 },
			{  22, 100,  25 },
			{  18, 133,  34 },
			{  12, 160,  40 },
			{  10, 200,  50 },
			{   6, 240,  58 },
			{   4, 267,  66 },
			{   2, 400, 100 },
			{   0, 500, 130 }
		};
		const struct ath5k_rate_table *rt =
				ath5k_hw_get_rate_table(sc->ah, mode);
		unsigned int i, j;

		BUG_ON(rt == NULL);

		memset(sc->hwmap, 0, sizeof(sc->hwmap));
		for (i = 0; i < 32; i++) {
			u8 ix = rt->rate_code_to_index[i];
			if (ix == 0xff) {
				sc->hwmap[i].ledon = msecs_to_jiffies(500);
				sc->hwmap[i].ledoff = msecs_to_jiffies(130);
				continue;
			}
			sc->hwmap[i].txflags = IEEE80211_RADIOTAP_F_DATAPAD;
			if (SHPREAMBLE_FLAG(ix) || rt->rates[ix].modulation ==
					IEEE80211_RATE_OFDM)
				sc->hwmap[i].txflags |=
						IEEE80211_RADIOTAP_F_SHORTPRE;
			/* receive frames include FCS */
			sc->hwmap[i].rxflags = sc->hwmap[i].txflags |
					IEEE80211_RADIOTAP_F_FCS;
			/* setup blink rate table to avoid per-packet lookup */
			for (j = 0; j < ARRAY_SIZE(blinkrates) - 1; j++)
				if (blinkrates[j].rate == /* XXX why 7f? */
						(rt->rates[ix].dot11_rate&0x7f))
					break;

			sc->hwmap[i].ledon = msecs_to_jiffies(blinkrates[j].
					timeOn);
			sc->hwmap[i].ledoff = msecs_to_jiffies(blinkrates[j].
					timeOff);
		}
	}

	sc->curmode = mode;
}

/*
 * Set/change channels.  If the channel is really being changed,
 * it's done by reseting the chip.  To accomplish this we must
 * first cleanup any pending DMA, then restart stuff after a la
 * ath_init.
 */
static int ath_chan_set(struct ath_softc *sc, struct ieee80211_channel *chan)
{
	struct ath_hw *ah = sc->ah;
	int ret;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %u (%u MHz) -> %u (%u MHz)\n",
		__func__, sc->curchan->chan, sc->curchan->freq,
		chan->chan, chan->freq);

	if (chan->freq != sc->curchan->freq || chan->val != sc->curchan->val) {
		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath5k_hw_set_intr(ah, 0);	/* disable interrupts */
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		ret = ath5k_hw_reset(ah, sc->opmode, chan, true);
		if (ret) {
			printk(KERN_ERR "%s: unable to reset channel %u "
				"(%u Mhz)\n", __func__, chan->chan, chan->freq);
			return ret;
		}
		sc->curchan = chan;
		ath_update_txpow(sc);

		/*
		 * Re-enable rx framework.
		 */
		ret = ath_startrecv(sc);
		if (ret) {
			printk(KERN_ERR "%s: unable to restart recv logic\n",
					__func__);
			return ret;
		}

		/*
		 * Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g.
		 *
		 * XXX needed?
		 */
/*		ath_chan_change(sc, chan); */

		/*
		 * Re-enable interrupts.
		 */
		ath5k_hw_set_intr(ah, sc->imask);
	}

	return 0;
}

static int ath_tx_bf(struct ath_softc *sc, struct ath_buf *bf,
		struct ieee80211_tx_control *ctl)
{
	struct ath_hw *ah = sc->ah;
	struct ath_txq *txq = sc->txq;
	struct ath_desc *ds = bf->desc;
	struct sk_buff *skb = bf->skb;
	unsigned int pktlen, flags, keyidx = AR5K_TXKEYIX_INVALID;
	int ret;

	flags = AR5K_TXDESC_INTREQ | AR5K_TXDESC_CLRDMASK;
	bf->ctl = *ctl;
	/* XXX endianness */
	bf->skbaddr = pci_map_single(sc->pdev, skb->data, skb->len,
			PCI_DMA_TODEVICE);

	if (ctl->flags & IEEE80211_TXCTL_NO_ACK)
		flags |= AR5K_TXDESC_NOACK;

	pktlen = skb->len + FCS_LEN;

	if (!(ctl->flags & IEEE80211_TXCTL_DO_NOT_ENCRYPT)) {
		keyidx = ctl->key_idx;
		pktlen += ctl->icv_len;
	}

	ret = ah->ah_setup_tx_desc(ah, ds, pktlen,
		ieee80211_get_hdrlen_from_skb(skb), AR5K_PKT_TYPE_NORMAL,
		0xffff, ctl->tx_rate, ctl->retry_limit, keyidx, 0, flags, 0, 0);
	if (ret)
		goto err_unmap;

	ds->ds_link = 0;
	ds->ds_data = bf->skbaddr;

	ret = ah->ah_fill_tx_desc(ah, ds, skb->len, true, true);
	if (ret)
		goto err_unmap;

	spin_lock_bh(&txq->lock);
	list_add_tail(&bf->list, &txq->q);
	sc->tx_stats.data[txq->qnum].len++;
	if (txq->link == NULL) /* is this first packet? */
		ath5k_hw_put_tx_buf(ah, txq->qnum, bf->daddr);
	else /* no, so only link it */
		*txq->link = bf->daddr;

	txq->link = &ds->ds_link;
	ath5k_hw_tx_start(ah, txq->qnum);
	spin_unlock_bh(&txq->lock);

	return 0;
err_unmap:
	pci_unmap_single(sc->pdev, bf->skbaddr, skb->len, PCI_DMA_TODEVICE);
	return ret;
}

static int ath_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
		struct ieee80211_tx_control *ctl)
{
	struct ath_softc *sc = hw->priv;
	struct ath_buf *bf;
	unsigned long flags;
	int hdrlen;
	int pad;

	ath_dump_skb(skb, "TX  ");

	if (sc->opmode == IEEE80211_IF_TYPE_MNTR)
		DPRINTF(sc, ATH_DEBUG_XMIT, "tx in monitor (scan?)\n");

	/*
	 * the hardware expects the header padded to 4 byte boundaries
	 * if this is not the case we add the padding after the header
	 */
	hdrlen = ieee80211_get_hdrlen_from_skb(skb);
	if (hdrlen & 3) {
		pad = hdrlen % 4;
		if (skb_headroom(skb) < pad) {
			if (net_ratelimit())
				printk(KERN_ERR "ath: tx hdrlen not %%4: %d "
					"not enough headroom to pad %d\n",
					hdrlen, pad);
			return -1;
		}
		skb_push(skb, pad);
		memmove(skb->data, skb->data+pad, hdrlen);
	}

	sc->led_txrate = ctl->tx_rate;

	spin_lock_irqsave(&sc->txbuflock, flags);
	if (list_empty(&sc->txbuf)) {
		if (net_ratelimit())
			printk(KERN_ERR "ath: no further txbuf available, "
				"dropping packet\n");
		spin_unlock_irqrestore(&sc->txbuflock, flags);
		ieee80211_stop_queue(hw, ctl->queue);
		return -1;
	}
	bf = list_first_entry(&sc->txbuf, struct ath_buf, list);
	list_del(&bf->list);
	sc->txbuf_len--;
	if (list_empty(&sc->txbuf))
		ieee80211_stop_queues(hw);
	spin_unlock_irqrestore(&sc->txbuflock, flags);

	bf->skb = skb;

	if (ath_tx_bf(sc, bf, ctl)) {
		bf->skb = NULL;
		spin_lock_irqsave(&sc->txbuflock, flags);
		list_add_tail(&bf->list, &sc->txbuf);
		sc->txbuf_len++;
		spin_unlock_irqrestore(&sc->txbuflock, flags);
		dev_kfree_skb_any(skb);
		return 0;
	}

	return 0;
}

static int ath_reset(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hw *ah = sc->ah;
	int ret;

	DPRINTF(sc, ATH_DEBUG_RESET, "resetting\n");
	/*
	 * Convert to a HAL channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	sc->curchan = hw->conf.chan;

	ath5k_hw_set_intr(ah, 0);
	ath_draintxq(sc);
	ath_stoprecv(sc);

	ret = ath5k_hw_reset(ah, sc->opmode, sc->curchan, true);
	if (unlikely(ret)) {
		printk(KERN_ERR "ath: can't reset hardware (%d)\n", ret);
		goto err;
	}
	ath_update_txpow(sc);

	ret = ath_startrecv(sc);
	if (unlikely(ret)) {
		printk(KERN_ERR "ath: can't start recv logic\n");
		goto err;
	}
	/*
	 * We may be doing a reset in response to an ioctl
	 * that changes the channel so update any state that
	 * might change as a result.
	 *
	 * XXX needed?
	 */
/*	ath_chan_change(sc, c); */
	ath_beacon_config(sc);
	/* intrs are started by ath_beacon_config */

	ieee80211_wake_queues(hw);

	return 0;
err:
	return ret;
}

static int ath_start(struct ieee80211_hw *hw)
{
	return ath_init(hw->priv);
}

void ath_stop(struct ieee80211_hw *hw)
{
	ath_stop_hw(hw->priv);
}

static int ath_add_interface(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	int ret;

	mutex_lock(&sc->lock);
	if (sc->iface_id) {
		ret = 0;
		goto end;
	}

	sc->iface_id = conf->if_id;

	switch (conf->type) {
	case IEEE80211_IF_TYPE_STA:
	case IEEE80211_IF_TYPE_IBSS:
	case IEEE80211_IF_TYPE_MNTR:
		sc->opmode = conf->type;
		break;
	default:
		ret = -EOPNOTSUPP;
		goto end;
	}
	ret = 0;
end:
	mutex_unlock(&sc->lock);
	return ret;
}

static void ath_remove_interface(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = hw->priv;

	mutex_lock(&sc->lock);
	if (sc->iface_id != conf->if_id)
		goto end;

	sc->iface_id = 0;
end:
	mutex_unlock(&sc->lock);
}

static int ath_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	struct ath_softc *sc = hw->priv;

	sc->bintval = conf->beacon_int * 1000 / 1024;
	ath_setcurmode(sc, conf->phymode);

	return ath_chan_set(sc, conf->chan);
}

static int ath_config_interface(struct ieee80211_hw *hw, int if_id,
		struct ieee80211_if_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hw *ah = sc->ah;
	int ret;

	/* Set to a reasonable value. Note that this will
	 * be set to mac80211's value at ath_config(). */
	sc->bintval = 1000 * 1000 / 1024;
	mutex_lock(&sc->lock);
	if (sc->iface_id != if_id) {
		ret = -EIO;
		goto unlock;
	}
	if (conf->bssid) {
		/* Cache for later use during resets */
		memcpy(ah->bssid, conf->bssid, ETH_ALEN);
		/* XXX: assoc id is set to 0 for now, mac80211 doesn't have
		 * a clean way of letting us retrieve this yet. */
		ath5k_hw_set_associd(ah, ah->bssid, 0);
	}
	mutex_unlock(&sc->lock);

	return ath_reset(hw);
unlock:
	mutex_unlock(&sc->lock);
	return ret;
}

#define SUPPORTED_FIF_FLAGS \
	FIF_PROMISC_IN_BSS |  FIF_ALLMULTI | FIF_FCSFAIL | \
	FIF_PLCPFAIL | FIF_CONTROL | FIF_OTHER_BSS | \
	FIF_BCN_PRBRESP_PROMISC
/*
 * o always accept unicast, broadcast, and multicast traffic
 * o multicast traffic for all BSSIDs will be enabled if mac80211
 *   says it should be
 * o maintain current state of phy ofdm or phy cck error reception.
 *   If the hardware detects any of these type of errors then
 *   ath5k_hw_get_rx_filter() will pass to us the respective
 *   hardware filters to be able to receive these type of frames.
 * o probe request frames are accepted only when operating in
 *   hostap, adhoc, or monitor modes
 * o enable promiscuous mode according to the interface state
 * o accept beacons:
 *   - when operating in adhoc mode so the 802.11 layer creates
 *     node table entries for peers,
 *   - when operating in station mode for collecting rssi data when
 *     the station is otherwise quiet, or
 *   - when scanning
 */
static void ath_configure_filter(struct ieee80211_hw *hw,
               unsigned int changed_flags,
               unsigned int *new_flags,
               int mc_count, struct dev_mc_list *mclist)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hw *ah = sc->ah;
	u32 mfilt[2], val, rfilt;
	u8 pos;
	int i;

	mfilt[0] = 0;
	mfilt[1] = 0;

	/* Only deal with supported flags */
	changed_flags &= SUPPORTED_FIF_FLAGS;
	*new_flags &= SUPPORTED_FIF_FLAGS;

	/* If HW detects any phy or radar errors, leave those filters on.
	 * Also, always enable Unicast, Broadcasts and Multicast
	 * XXX: move unicast, bssid broadcasts and multicast to mac80211 */
	rfilt = (ath5k_hw_get_rx_filter(ah) & (AR5K_RX_FILTER_PHYERR)) |
		(AR5K_RX_FILTER_UCAST | AR5K_RX_FILTER_BCAST |
		AR5K_RX_FILTER_MCAST);

	if (changed_flags & (FIF_PROMISC_IN_BSS | FIF_OTHER_BSS)) {
		if (*new_flags & FIF_PROMISC_IN_BSS) {
			rfilt |= AR5K_RX_FILTER_PROM;
			__set_bit(ATH_STAT_PROMISC, sc->status);
		}
		else
			__clear_bit(ATH_STAT_PROMISC, sc->status);
	}

	/* Note, AR5K_RX_FILTER_MCAST is already enabled */
	if (*new_flags & FIF_ALLMULTI) {
		mfilt[0] =  ~0;
		mfilt[1] =  ~0;
	} else {
		for (i = 0; i < mc_count; i++) {
			if (!mclist)
				break;
			/* calculate XOR of eight 6-bit values */
			val = LE_READ_4(mclist->dmi_addr + 0);
			pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			val = LE_READ_4(mclist->dmi_addr + 3);
			pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			pos &= 0x3f;
			mfilt[pos / 32] |= (1 << (pos % 32));
			/* XXX: we might be able to just do this instead,
			* but not sure, needs testing, if we do use this we'd
			* neet to inform below to not reset the mcast */
			/* ath5k_hw_set_mcast_filterindex(ah,
			 *      mclist->dmi_addr[5]); */
			mclist = mclist->next;
		}
	}

	/* This is the best we can do */
	if (*new_flags & (FIF_FCSFAIL | FIF_PLCPFAIL))
		rfilt |= AR5K_RX_FILTER_PHYERR;

	/* FIF_BCN_PRBRESP_PROMISC really means to enable beacons
	* and probes for any BSSID, this needs testing */
	if (*new_flags & FIF_BCN_PRBRESP_PROMISC)
		rfilt |= AR5K_RX_FILTER_BEACON | AR5K_RX_FILTER_PROBEREQ;

	/* FIF_CONTROL doc says that if FIF_PROMISC_IN_BSS is not
	 * set we should only pass on control frames for this
	 * station. This needs testing. I believe right now this
	 * enables *all* control frames, which is OK.. but
	 * but we should see if we can improve on granularity */
	if (*new_flags & FIF_CONTROL)
		rfilt |= AR5K_RX_FILTER_CONTROL;

	/* Additional settings per mode -- this is per ath5k */

	/* XXX move these to mac80211, and add a beacon IFF flag to mac80211 */

	if (sc->opmode == IEEE80211_IF_TYPE_MNTR)
		rfilt |= AR5K_RX_FILTER_CONTROL | AR5K_RX_FILTER_BEACON |
			AR5K_RX_FILTER_PROBEREQ | AR5K_RX_FILTER_PROM;
	if (sc->opmode != IEEE80211_IF_TYPE_STA)
		rfilt |= AR5K_RX_FILTER_PROBEREQ;
	if (sc->opmode != IEEE80211_IF_TYPE_AP &&
		test_bit(ATH_STAT_PROMISC, sc->status))
		rfilt |= AR5K_RX_FILTER_PROM;
	if (sc->opmode == IEEE80211_IF_TYPE_STA ||
		sc->opmode == IEEE80211_IF_TYPE_IBSS) {
		rfilt |= AR5K_RX_FILTER_BEACON;
	}

	/* Set multicast bits */
	ath5k_hw_set_mcast_filter(ah, mfilt[0], mfilt[1]);
	/* Set the cached hw filter flags, this will alter actually
	 * be set in HW */
	sc->filter_flags = rfilt;
}

static int ath_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		const u8 *local_addr, const u8 *addr,
		struct ieee80211_key_conf *key)
{
	struct ath_softc *sc = hw->priv;
	int ret = 0;

	switch(key->alg) {
	case ALG_WEP:
		break;
	case ALG_TKIP:
	case ALG_CCMP:
		return -EOPNOTSUPP;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	mutex_lock(&sc->lock);

	switch (cmd) {
	case SET_KEY:
		ret = ath5k_hw_set_key(sc->ah, key->keyidx, key, addr);
		if (ret) {
			printk(KERN_ERR "ath: can't set the key\n");
			goto unlock;
		}
		__set_bit(key->keyidx, sc->keymap);
		key->hw_key_idx = key->keyidx;
		break;
	case DISABLE_KEY:
		ath5k_hw_reset_key(sc->ah, key->keyidx);
		__clear_bit(key->keyidx, sc->keymap);
		break;
	default:
		ret = -EINVAL;
		goto unlock;
	}

unlock:
	mutex_unlock(&sc->lock);
	return ret;
}

static int ath_get_stats(struct ieee80211_hw *hw,
		struct ieee80211_low_level_stats *stats)
{
	struct ath_softc *sc = hw->priv;

	memcpy(stats, &sc->ll_stats, sizeof(sc->ll_stats));

	return 0;
}

static int ath_get_tx_stats(struct ieee80211_hw *hw,
		struct ieee80211_tx_queue_stats *stats)
{
	struct ath_softc *sc = hw->priv;

	memcpy(stats, &sc->tx_stats, sizeof(sc->tx_stats));

	return 0;
}

static u64 ath_get_tsf(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;

	return ath5k_hw_get_tsf64(sc->ah);
}

static void ath_reset_tsf(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;

	ath5k_hw_reset_tsf(sc->ah);
}

static int ath_beacon_update(struct ieee80211_hw *hw, struct sk_buff *skb,
		struct ieee80211_tx_control *ctl)
{
	struct ath_softc *sc = hw->priv;
	int ret;

	ath_dump_skb(skb, "BC  ");

	mutex_lock(&sc->lock);

	if (sc->opmode != IEEE80211_IF_TYPE_IBSS) {
		ret = -EIO;
		goto end;
	}

	ath_cleanup_txbuf(sc, sc->bbuf);
	sc->bbuf->skb = skb;
	ret = ath_beacon_setup(sc, sc->bbuf, ctl);
	if (ret)
		sc->bbuf->skb = NULL;

end:
	mutex_unlock(&sc->lock);
	return ret;
}

static struct ieee80211_ops ath_hw_ops = {
	.tx = ath_tx,
	.start = ath_start,
	.stop = ath_stop,
	.add_interface = ath_add_interface,
	.remove_interface = ath_remove_interface,
	.config = ath_config,
	.config_interface = ath_config_interface,
	.configure_filter = ath_configure_filter,
	.set_key = ath_set_key,
	.get_stats = ath_get_stats,
	.conf_tx = NULL,
	.get_tx_stats = ath_get_tx_stats,
	.get_tsf = ath_get_tsf,
	.reset_tsf = ath_reset_tsf,
	.beacon_update = ath_beacon_update,
};

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
static void ath_calibrate(unsigned long data)
{
	struct ath_softc *sc = (void *)data;
	struct ath_hw *ah = sc->ah;

	DPRINTF(sc, ATH_DEBUG_CALIBRATE, "ath: channel %u/%x\n",
		sc->curchan->chan, sc->curchan->val);

	if (ath5k_hw_get_rf_gain(ah) == AR5K_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		DPRINTF(sc, ATH_DEBUG_RESET, "calibration, resetting\n");
		ath_reset(sc->hw);
	}
	if (ath5k_hw_phy_calibrate(ah, sc->curchan))
		printk(KERN_ERR "ath: calibration of channel %u failed\n",
				sc->curchan->chan);

	mod_timer(&sc->calib_tim, round_jiffies(jiffies +
			msecs_to_jiffies(ath_calinterval * 1000)));
}

static void ath_led_off(unsigned long data)
{
	struct ath_softc *sc = (void *)data;

	if (test_bit(ATH_STAT_LEDENDBLINK, sc->status))
		__clear_bit(ATH_STAT_LEDBLINKING, sc->status);
	else {
		__set_bit(ATH_STAT_LEDENDBLINK, sc->status);
		ath5k_hw_set_gpio(sc->ah, sc->led_pin, !sc->led_on);
		mod_timer(&sc->led_tim, jiffies + sc->led_off);
	}
}

/*
 * Blink the LED according to the specified on/off times.
 */
static void ath_led_blink(struct ath_softc *sc, unsigned int on,
		unsigned int off)
{
	DPRINTF(sc, ATH_DEBUG_LED, "%s: on %u off %u\n", __func__, on, off);
	ath5k_hw_set_gpio(sc->ah, sc->led_pin, sc->led_on);
	__set_bit(ATH_STAT_LEDBLINKING, sc->status);
	__clear_bit(ATH_STAT_LEDENDBLINK, sc->status);
	sc->led_off = off;
	mod_timer(&sc->led_tim, jiffies + on);
}

static void ath_led_event(struct ath_softc *sc, int event)
{
	if (likely(!test_bit(ATH_STAT_LEDSOFT, sc->status)))
		return;
	if (unlikely(test_bit(ATH_STAT_LEDBLINKING, sc->status)))
		return; /* don't interrupt active blink */
	switch (event) {
	case ATH_LED_TX:
		ath_led_blink(sc, sc->hwmap[sc->led_txrate].ledon,
			sc->hwmap[sc->led_txrate].ledoff);
		break;
	case ATH_LED_RX:
		ath_led_blink(sc, sc->hwmap[sc->led_rxrate].ledon,
			sc->hwmap[sc->led_rxrate].ledoff);
		break;
	}
}

static irqreturn_t ath_intr(int irq, void *dev_id)
{
	struct ath_softc *sc = dev_id;
	struct ath_hw *ah = sc->ah;
	enum ath5k_int status;
	unsigned int counter = 1000;

	if (unlikely(test_bit(ATH_STAT_INVALID, sc->status) ||
				!ath5k_hw_is_intr_pending(ah)))
		return IRQ_NONE;

	do {
		/*
		* Figure out the reason(s) for the interrupt.  Note
		* that the hal returns a pseudo-ISR that may include
		* bits we haven't explicitly enabled so we mask the
		* value to insure we only process bits we requested.
		*/
		ath5k_hw_get_isr(ah, &status);		/* NB: clears IRQ too */
		DPRINTF(sc, ATH_DEBUG_INTR, "%s: status 0x%x/0x%x\n", __func__,
				status, sc->imask);
		status &= sc->imask; /* discard unasked for bits */
		if (unlikely(status & AR5K_INT_FATAL)) {
			/*
			* Fatal errors are unrecoverable.  Typically
			* these are caused by DMA errors.  Unfortunately
			* the exact reason is not (presently) returned
			* by the hal.
			*/
			tasklet_schedule(&sc->restq);
		} else if (unlikely(status & AR5K_INT_RXORN)) {
			tasklet_schedule(&sc->restq);
		} else {
			if (status & AR5K_INT_SWBA) {
				/*
				* Software beacon alert--time to send a beacon.
				* Handle beacon transmission directly; deferring
				* this is too slow to meet timing constraints
				* under load.
				*/
				ath_beacon_send(sc);
			}
			if (status & AR5K_INT_RXEOL) {
				/*
				* NB: the hardware should re-read the link when
				*     RXE bit is written, but it doesn't work at
				*     least on older hardware revs.
				*/
				sc->rxlink = NULL;
			}
			if (status & AR5K_INT_TXURN) {
				/* bump tx trigger level */
				ath5k_hw_update_tx_triglevel(ah, true);
			}
			if (status & AR5K_INT_RX)
				tasklet_schedule(&sc->rxtq);
			if (status & AR5K_INT_TX)
				tasklet_schedule(&sc->txtq);
			if (status & AR5K_INT_BMISS) {
			}
			if (status & AR5K_INT_MIB) {
				/* TODO */
			}
		}
	} while (ath5k_hw_is_intr_pending(ah) && counter-- > 0);

	if (unlikely(!counter && net_ratelimit()))
		printk(KERN_WARNING "ath: too many interrupts, giving up for "
				"now\n");

	return IRQ_HANDLED;
}

/*
 * Convert IEEE channel number to MHz frequency.
 */
static inline short ath_ieee2mhz(short chan)
{
	if (chan <= 14 || chan >= 27)
		return ieee80211chan2mhz(chan);
	else
		return 2212 + chan * 20;
}

static unsigned int ath_copy_rates(struct ieee80211_rate *rates,
		const struct ath5k_rate_table *rt, unsigned int max)
{
	unsigned int i, count;

	if (rt == NULL)
		return 0;

	for (i = 0, count = 0; i < rt->rate_count && max > 0; i++) {
		if (!rt->rates[i].valid)
			continue;
		rates->rate = rt->rates[i].rate_kbps / 100;
		rates->val = rt->rates[i].rate_code;
		rates->flags = rt->rates[i].modulation;
		rates++;
		count++;
		max--;
	}

	return count;
}

static unsigned int ath_copy_channels(struct ath_hw *ah,
		struct ieee80211_channel *channels, unsigned int mode,
		unsigned int max)
{
	static const struct { unsigned int mode, mask, chan; } map[] = {
		[MODE_IEEE80211A] = { CHANNEL_OFDM, CHANNEL_OFDM | CHANNEL_TURBO, CHANNEL_A },
		[MODE_ATHEROS_TURBO] = { CHANNEL_OFDM|CHANNEL_TURBO, CHANNEL_OFDM | CHANNEL_TURBO, CHANNEL_T },
		[MODE_IEEE80211B] = { CHANNEL_CCK, CHANNEL_CCK, CHANNEL_B },
		[MODE_IEEE80211G] = { CHANNEL_OFDM, CHANNEL_OFDM, CHANNEL_G },
		[MODE_ATHEROS_TURBOG] = { CHANNEL_OFDM | CHANNEL_TURBO, CHANNEL_OFDM | CHANNEL_TURBO, CHANNEL_TG },
	};
	static const struct ath5k_regchannel chans_2ghz[] =
		IEEE80211_CHANNELS_2GHZ;
	static const struct ath5k_regchannel chans_5ghz[] =
		IEEE80211_CHANNELS_5GHZ;
	const struct ath5k_regchannel *chans;
	enum ath5k_regdom dmn;
	unsigned int i, count, size, chfreq, all, f, ch;

	if (!test_bit(mode, ah->ah_modes))
		return 0;

	all = ah->ah_regdomain == DMN_DEFAULT || CHAN_DEBUG == 1;

	switch (mode) {
	case MODE_IEEE80211A:
	case MODE_ATHEROS_TURBO:
		/* 1..220, but 2GHz frequencies are filtered by check_channel */
		size = all ? 220 : ARRAY_SIZE(chans_5ghz);
		chans = chans_5ghz;
		dmn = ath5k_regdom2flag(ah->ah_regdomain,
				IEEE80211_CHANNELS_5GHZ_MIN);
		chfreq = CHANNEL_5GHZ;
		break;
	case MODE_IEEE80211B:
	case MODE_IEEE80211G:
	case MODE_ATHEROS_TURBOG:
		size = all ? 26 : ARRAY_SIZE(chans_2ghz);
		chans = chans_2ghz;
		dmn = ath5k_regdom2flag(ah->ah_regdomain,
				IEEE80211_CHANNELS_2GHZ_MIN);
		chfreq = CHANNEL_2GHZ;
		break;
	default:
		printk(KERN_WARNING "bad mode, not copying channels\n");
		return 0;
	}

	for (i = 0, count = 0; i < size && max > 0; i++) {
		ch = all ? i + 1 : chans[i].chan;
		f = ath_ieee2mhz(ch);
		/* Check if channel is supported by the chipset */
		if (!ath5k_channel_ok(ah, f, chfreq))
			continue;

		/* Match regulation domain */
		if (!all && !(IEEE80211_DMN(chans[i].domain) &
							IEEE80211_DMN(dmn)))
			continue;

		if (!all && (chans[i].mode & map[mode].mask) != map[mode].mode)
			continue;

		/* Write channel and increment counter */
		channels->chan = ch;
		channels->freq = f;
		channels->val = map[mode].chan;
		channels++;
		count++;
		max--;
	}

	return count;
}

#if ATH_DEBUG_MODES
static void ath_dump_modes(struct ieee80211_hw_mode *modes)
{
	unsigned int m, i;

	for (m = 0; m < NUM_DRIVER_MODES; m++) {
		printk(KERN_DEBUG "Mode %u: channels %d, rates %d\n", m,
				modes[m].num_channels, modes[m].num_rates);
		printk(KERN_DEBUG " channels:\n");
		for (i = 0; i < modes[m].num_channels; i++)
			printk(KERN_DEBUG "  %3d %d %.4x %.4x\n",
					modes[m].channels[i].chan,
					modes[m].channels[i].freq,
					modes[m].channels[i].val,
					modes[m].channels[i].flag);
		printk(KERN_DEBUG " rates:\n");
		for (i = 0; i < modes[m].num_rates; i++)
			printk(KERN_DEBUG "  %4d %.4x %.4x %.4x\n",
					modes[m].rates[i].rate,
					modes[m].rates[i].val,
					modes[m].rates[i].flags,
					modes[m].rates[i].val2);
	}
}
#else
static inline void ath_dump_modes(struct ieee80211_hw_mode *modes) {}
#endif

static inline int ath5k_register_mode(struct ieee80211_hw *hw, u8 m)
{
	struct ath_softc *sc = hw->priv;
	struct ieee80211_hw_mode *modes = sc->modes;
	int i, ret;

	for (i = 0; i < NUM_DRIVER_MODES; i++) {
		if (modes[i].mode != m || !modes[i].num_channels)
			continue;
		ret = ieee80211_register_hwmode(hw, &modes[i]);
		if (ret) {
			printk(KERN_ERR "can't register hwmode %u\n", m);
			return ret;
		}
		return 0;
	}
	return 1;
}

/* Only tries to register modes our EEPROM says it can support */
#define REGISTER_MODE(m) do { \
	if (test_bit(m, ah->ah_capabilities.cap_mode)) { \
		ret = ath5k_register_mode(hw, m); \
		if (ret) \
			return ret; \
	} \
} while (0) \

static int ath_getchannels(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hw *ah = sc->ah;
	struct ieee80211_hw_mode *modes = sc->modes;
	unsigned int i, max_r, max_c;
	int ret;

	BUILD_BUG_ON(ARRAY_SIZE(sc->modes) < 3);

	/* The order here does not matter */
	modes[0].mode = MODE_IEEE80211G;
	modes[1].mode = MODE_IEEE80211B;
	modes[2].mode = MODE_IEEE80211A;

	max_r = ARRAY_SIZE(sc->rates);
	max_c = ARRAY_SIZE(sc->channels);

	for (i = 0; i < NUM_DRIVER_MODES; i++) {
		struct ieee80211_hw_mode *mode = &modes[i];
		const struct ath5k_rate_table *hw_rates;

		if (i == 0) {
			modes[0].rates	= sc->rates;
			modes->channels	= sc->channels;
		} else {
			struct ieee80211_hw_mode *prev_mode = &modes[i-1];
			int prev_num_r	= prev_mode->num_rates;
			int prev_num_c	= prev_mode->num_channels;
			mode->rates	= &prev_mode->rates[prev_num_r];
			mode->channels	= &prev_mode->channels[prev_num_c];
		}

		hw_rates = ath5k_hw_get_rate_table(ah, mode->mode);
		if (!hw_rates)
			return -EINVAL;

		mode->num_rates    = ath_copy_rates(mode->rates, hw_rates,
			max_r);
		mode->num_channels = ath_copy_channels(ah, mode->channels,
			mode->mode, max_c);
		max_r -= mode->num_rates;
		max_c -= mode->num_channels;
	}

	/* We try to register all modes this driver supports. We don't bother
	 * with MODE_IEEE80211B for AR5212 as MODE_IEEE80211G already accounts
	 * for that as per mac80211. Then, REGISTER_MODE() will will actually
	 * check the eeprom reading for more reliable capability information.
	 * Order matters here as per mac80211's latest preference. This will
	 * all hopefullly soon go away. */

	REGISTER_MODE(MODE_IEEE80211G);
	if (ah->ah_version != AR5K_AR5212)
		REGISTER_MODE(MODE_IEEE80211B);
	REGISTER_MODE(MODE_IEEE80211A);

	ath_dump_modes(modes);

	return ret;
}

static int ath_desc_alloc(struct ath_softc *sc, struct pci_dev *pdev)
{
	struct ath_desc *ds;
	struct ath_buf *bf;
	dma_addr_t da;
	unsigned int i;
	int ret;

	/* allocate descriptors */
	sc->desc_len = sizeof(struct ath_desc) *
			(ATH_TXBUF + ATH_RXBUF + ATH_BCBUF + 1);
	sc->desc = pci_alloc_consistent(pdev, sc->desc_len, &sc->desc_daddr);
	if (sc->desc == NULL) {
		dev_err(&pdev->dev, "can't allocate descriptors\n");
		ret = -ENOMEM;
		goto err;
	}
	ds = sc->desc;
	da = sc->desc_daddr;
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: DMA map: %p (%zu) -> %llx\n",
		__func__, ds, sc->desc_len, (unsigned long long)sc->desc_daddr);

	bf = kcalloc(1 + ATH_TXBUF + ATH_RXBUF + ATH_BCBUF,
			sizeof(struct ath_buf), GFP_KERNEL);
	if (bf == NULL) {
		dev_err(&pdev->dev, "can't allocate bufptr\n");
		ret = -ENOMEM;
		goto err_free;
	}
	sc->bufptr = bf;

	INIT_LIST_HEAD(&sc->rxbuf);
	for (i = 0; i < ATH_RXBUF; i++, bf++, ds++, da += sizeof(*ds)) {
		bf->desc = ds;
		bf->daddr = da;
		list_add_tail(&bf->list, &sc->rxbuf);
	}

	INIT_LIST_HEAD(&sc->txbuf);
	sc->txbuf_len = ATH_TXBUF;
	for (i = 0; i < ATH_TXBUF; i++, bf++, ds++,
			da += sizeof(*ds)) {
		bf->desc = ds;
		bf->daddr = da;
		list_add_tail(&bf->list, &sc->txbuf);
	}

	/* beacon buffer */
	bf->desc = ds;
	bf->daddr = da;
	sc->bbuf = bf;

	return 0;
err_free:
	pci_free_consistent(pdev, sc->desc_len, sc->desc, sc->desc_daddr);
err:
	sc->desc = NULL;
	return ret;
}

static void ath_desc_free(struct ath_softc *sc, struct pci_dev *pdev)
{
	struct ath_buf *bf;

	ath_cleanup_txbuf(sc, sc->bbuf);
	list_for_each_entry(bf, &sc->txbuf, list)
		ath_cleanup_txbuf(sc, bf);
	list_for_each_entry(bf, &sc->rxbuf, list)
		ath_cleanup_txbuf(sc, bf);

	/* Free memory associated with all descriptors */
	pci_free_consistent(pdev, sc->desc_len, sc->desc, sc->desc_daddr);

	kfree(sc->bufptr);
	sc->bufptr = NULL;
}

static int ath_beaconq_setup(struct ath_hw *ah)
{
	struct ath5k_txq_info qi = {
		.tqi_aifs = AR5K_TXQ_USEDEFAULT,
		.tqi_cw_min = AR5K_TXQ_USEDEFAULT,
		.tqi_cw_max = AR5K_TXQ_USEDEFAULT,
		/* NB: for dynamic turbo, don't enable any other interrupts */
		.tqi_flags = AR5K_TXQ_FLAG_TXDESCINT_ENABLE
	};

	return ath5k_hw_setup_tx_queue(ah, AR5K_TX_QUEUE_BEACON, &qi);
}

static struct ath_txq *ath_txq_setup(struct ath_softc *sc, int qtype,
		int subtype)
{
	struct ath_hw *ah = sc->ah;
	struct ath_txq *txq;
	struct ath5k_txq_info qi = {
		.tqi_subtype = subtype,
		.tqi_aifs = AR5K_TXQ_USEDEFAULT,
		.tqi_cw_min = AR5K_TXQ_USEDEFAULT,
		.tqi_cw_max = AR5K_TXQ_USEDEFAULT
	};
	int qnum;

	/*
	 * Enable interrupts only for EOL and DESC conditions.
	 * We mark tx descriptors to receive a DESC interrupt
	 * when a tx queue gets deep; otherwise waiting for the
	 * EOL to reap descriptors.  Note that this is done to
	 * reduce interrupt load and this only defers reaping
	 * descriptors, never transmitting frames.  Aside from
	 * reducing interrupts this also permits more concurrency.
	 * The only potential downside is if the tx queue backs
	 * up in which case the top half of the kernel may backup
	 * due to a lack of tx descriptors.
	 */
	qi.tqi_flags = AR5K_TXQ_FLAG_TXEOLINT_ENABLE |
				AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
	qnum = ath5k_hw_setup_tx_queue(ah, qtype, &qi);
	if (qnum < 0) {
		/*
		 * NB: don't print a message, this happens
		 * normally on parts with too few tx queues
		 */
		return ERR_PTR(qnum);
	}
	if (qnum >= ARRAY_SIZE(sc->txqs)) {
		printk(KERN_ERR "hal qnum %u out of range, max %tu!\n",
			qnum, ARRAY_SIZE(sc->txqs));
		ath5k_hw_release_tx_queue(ah, qnum);
		return ERR_PTR(-EINVAL);
	}
	txq = &sc->txqs[qnum];
	if (!txq->setup) {
		txq->qnum = qnum;
		txq->link = NULL;
		INIT_LIST_HEAD(&txq->q);
		spin_lock_init(&txq->lock);
		txq->setup = true;
	}
	return &sc->txqs[qnum];
}

static void ath_tx_cleanup(struct ath_softc *sc)
{
	struct ath_txq *txq = sc->txqs;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sc->txqs); i++, txq++)
		if (txq->setup) {
			ath5k_hw_release_tx_queue(sc->ah, txq->qnum);
			txq->setup = false;
		}
}

static int ath_attach(struct pci_dev *pdev, struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hw *ah = sc->ah;
	u8 mac[ETH_ALEN];
	unsigned int i;
	int ret;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: devid 0x%x\n", __func__, pdev->device);

	/*
	 * Check if the MAC has multi-rate retry support.
	 * We do this by trying to setup a fake extended
	 * descriptor.  MAC's that don't have support will
	 * return false w/o doing anything.  MAC's that do
	 * support it will return true w/o doing anything.
	 */
	if (ah->ah_setup_xtx_desc(ah, NULL, 0, 0, 0, 0, 0, 0))
		__set_bit(ATH_STAT_MRRETRY, sc->status);

	/*
	 * Reset the key cache since some parts do not
	 * reset the contents on initial power up.
	 */
	for (i = 0; i < AR5K_KEYCACHE_SIZE; i++)
		ath5k_hw_reset_key(ah, i);

	/*
	 * Collect the channel list.  The 802.11 layer
	 * is resposible for filtering this list based
	 * on settings like the phy mode and regulatory
	 * domain restrictions.
	 */
	ret = ath_getchannels(hw);
	if (ret) {
		dev_err(&pdev->dev, "can't get channels\n");
		goto err;
	}

	/* NB: setup here so ath_rate_update is happy */
	if (test_bit(MODE_IEEE80211A, ah->ah_modes))
		ath_setcurmode(sc, MODE_IEEE80211A);
	else
		ath_setcurmode(sc, MODE_IEEE80211B);

	/*
	 * Allocate tx+rx descriptors and populate the lists.
	 */
	ret = ath_desc_alloc(sc, pdev);
	if (ret) {
		dev_err(&pdev->dev, "can't allocate descriptors\n");
		goto err;
	}

	/*
	 * Allocate hardware transmit queues: one queue for
	 * beacon frames and one data queue for each QoS
	 * priority.  Note that the hal handles reseting
	 * these queues at the needed time.
	 */
	ret = ath_beaconq_setup(ah);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't setup a beacon xmit queue\n");
		goto err_desc;
	}
	sc->bhalq = ret;

	sc->txq = ath_txq_setup(sc, AR5K_TX_QUEUE_DATA, AR5K_WME_AC_BK);
	if (IS_ERR(sc->txq)) {
		dev_err(&pdev->dev, "can't setup xmit queue\n");
		ret = PTR_ERR(sc->txq);
		goto err_bhal;
	}

	tasklet_init(&sc->rxtq, ath_tasklet_rx, (unsigned long)sc);
	tasklet_init(&sc->txtq, ath_tasklet_tx, (unsigned long)sc);
	tasklet_init(&sc->restq, ath_tasklet_reset, (unsigned long)sc);
	setup_timer(&sc->calib_tim, ath_calibrate, (unsigned long)sc);
	setup_timer(&sc->led_tim, ath_led_off, (unsigned long)sc);

	sc->led_on = 0; /* low true */
	/*
	 * Auto-enable soft led processing for IBM cards and for
	 * 5211 minipci cards.  Users can also manually enable/disable
	 * support with a sysctl.
	 */
	if (pdev->device == PCI_DEVICE_ID_ATHEROS_AR5212_IBM ||
			pdev->device == PCI_DEVICE_ID_ATHEROS_AR5211) {
		__set_bit(ATH_STAT_LEDSOFT, sc->status);
		sc->led_pin = 0;
	}
	/* Enable softled on PIN1 on HP Compaq nc6xx, nc4000 & nx5000 laptops */
	if (pdev->subsystem_vendor == PCI_VENDOR_ID_COMPAQ) {
		__set_bit(ATH_STAT_LEDSOFT, sc->status);
		sc->led_pin = 0;
	}
	if (test_bit(ATH_STAT_LEDSOFT, sc->status)) {
		ath5k_hw_set_gpio_output(ah, sc->led_pin);
		ath5k_hw_set_gpio(ah, sc->led_pin, !sc->led_on);
	}

	ath5k_hw_get_lladdr(ah, mac);
	SET_IEEE80211_PERM_ADDR(hw, mac);
	/* All MAC address bits matter for ACKs */
	memset(sc->bssidmask, 0xff, ETH_ALEN);
	ath5k_hw_set_bssid_mask(sc->ah, sc->bssidmask);

	ret = ieee80211_register_hw(hw);
	if (ret) {
		dev_err(&pdev->dev, "can't register ieee80211 hw\n");
		goto err_queues;
	}

	return 0;
err_queues:
	ath_tx_cleanup(sc);
err_bhal:
	ath5k_hw_release_tx_queue(ah, sc->bhalq);
err_desc:
	ath_desc_free(sc, pdev);
err:
	return ret;
}

static void ath_detach(struct pci_dev *pdev, struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;

	/*
	 * NB: the order of these is important:
	 * o call the 802.11 layer before detaching the hal to
	 *   insure callbacks into the driver to delete global
	 *   key cache entries can be handled
	 * o reclaim the tx queue data structures after calling
	 *   the 802.11 layer as we'll get called back to reclaim
	 *   node state and potentially want to use them
	 * o to cleanup the tx queues the hal is called, so detach
	 *   it last
	 * Other than that, it's straightforward...
	 */
	ieee80211_unregister_hw(hw);
	ath_desc_free(sc, pdev);
	ath_tx_cleanup(sc);
	ath5k_hw_release_tx_queue(sc->ah, sc->bhalq);

	/*
	 * NB: can't reclaim these until after ieee80211_ifdetach
	 * returns because we'll get called back to reclaim node
	 * state and potentially want to use them.
	 */
}

static const char *ath_chip_name(u8 mac_version)
{
	switch (mac_version) {
	case AR5K_AR5210:
		return "AR5210";
	case AR5K_AR5211:
		return "AR5211";
	case AR5K_AR5212:
		return "AR5212";
	}
	return "Unknown";
}

static int __devinit ath_pci_probe(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	void __iomem *mem;
	struct ath_softc *sc;
	struct ieee80211_hw *hw;
	int ret;
	u8 csz;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "can't enable device\n");
		goto err;
	}

	/* XXX 32-bit addressing only */
	ret = pci_set_dma_mask(pdev, DMA_32BIT_MASK);
	if (ret) {
		dev_err(&pdev->dev, "32-bit DMA not available\n");
		goto err_dis;
	}

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &csz);
	if (csz == 0) {
		/*
		 * Linux 2.4.18 (at least) writes the cache line size
		 * register as a 16-bit wide register which is wrong.
		 * We must have this setup properly for rx buffer
		 * DMA to work so force a reasonable value here if it
		 * comes up zero.
		 */
		csz = L1_CACHE_BYTES / sizeof(u32);
		pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, csz);
	}
	/*
	 * The default setting of latency timer yields poor results,
	 * set it to the value used by other systems.  It may be worth
	 * tweaking this setting more.
	 */
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0xa8);

	pci_set_master(pdev);

	/*
	 * Disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_write_config_byte(pdev, 0x41, 0);

	ret = pci_request_region(pdev, 0, "ath");
	if (ret) {
		dev_err(&pdev->dev, "cannot reserve PCI memory region\n");
		goto err_dis;
	}

	mem = pci_iomap(pdev, 0, 0);
	if (!mem) {
		dev_err(&pdev->dev, "cannot remap PCI memory region\n") ;
		ret = -EIO;
		goto err_reg;
	}

	hw = ieee80211_alloc_hw(sizeof(*sc), &ath_hw_ops);
	if (hw == NULL) {
		dev_err(&pdev->dev, "cannot allocate ieee80211_hw\n");
		ret = -ENOMEM;
		goto err_map;
	}

	SET_IEEE80211_DEV(hw, &pdev->dev);
	hw->flags = IEEE80211_HW_RX_INCLUDES_FCS;
	hw->extra_tx_headroom = 2;
	hw->channel_change_time = 5000;
	hw->max_rssi = 127; /* FIXME: get a real value for this. */
	sc = hw->priv;
	sc->hw = hw;
	sc->pdev = pdev;

	/*
	 * Mark the device as detached to avoid processing
	 * interrupts until setup is complete.
	 */
#if AR_DEBUG
	sc->debug = ath_debug;
#endif
	__set_bit(ATH_STAT_INVALID, sc->status);
	sc->iobase = mem;
	sc->cachelsz = csz * sizeof(u32); /* convert to bytes */
	sc->opmode = IEEE80211_IF_TYPE_STA;
	mutex_init(&sc->lock);
	spin_lock_init(&sc->rxbuflock);
	spin_lock_init(&sc->txbuflock);

	pci_set_drvdata(pdev, hw);

	pci_enable_msi(pdev);

	ret = request_irq(pdev->irq, ath_intr, IRQF_SHARED, "ath", sc);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_free;
	}

	sc->ah = ath5k_hw_attach(pdev->device, id->driver_data, sc, sc->iobase);
	if (IS_ERR(sc->ah)) {
		ret = PTR_ERR(sc->ah);
		goto err_irq;
	}

	ret = ath_attach(pdev, hw);
	if (ret)
		goto err_ah;

	dev_info(&pdev->dev, "%s chip found: mac %d.%d phy %d.%d\n",
			ath_chip_name(id->driver_data), sc->ah->ah_mac_version,
			sc->ah->ah_mac_version, sc->ah->ah_phy_revision >> 4,
			sc->ah->ah_phy_revision & 0xf);

	/* ready to process interrupts */
	__clear_bit(ATH_STAT_INVALID, sc->status);

	return 0;
err_ah:
	ath5k_hw_detach(sc->ah);
err_irq:
	free_irq(pdev->irq, sc);
err_free:
	pci_disable_msi(pdev);
	ieee80211_free_hw(hw);
err_map:
	pci_iounmap(pdev, mem);
err_reg:
	pci_release_region(pdev, 0);
err_dis:
	pci_disable_device(pdev);
err:
	return ret;
}

static void __devexit ath_pci_remove(struct pci_dev *pdev)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	struct ath_softc *sc = hw->priv;

	ath_detach(pdev, hw);
	ath5k_hw_detach(sc->ah);
	free_irq(pdev->irq, sc);
	pci_disable_msi(pdev);
	pci_iounmap(pdev, sc->iobase);
	pci_release_region(pdev, 0);
	pci_disable_device(pdev);
	ieee80211_free_hw(hw);
}

#ifdef CONFIG_PM
static int ath_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	struct ath_softc *sc = hw->priv;

	if (test_bit(ATH_STAT_LEDSOFT, sc->status))
		ath5k_hw_set_gpio(sc->ah, sc->led_pin, 1);

	ath_stop_hw(sc);
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);

	return 0;
}

static int ath_pci_resume(struct pci_dev *pdev)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	struct ath_softc *sc = hw->priv;
	int err;

	err = pci_set_power_state(pdev, PCI_D0);
	if (err)
		return err;

	err = pci_enable_device(pdev);
	if (err)
		return err;

	pci_restore_state(pdev);
	/*
	 * Suspend/Resume resets the PCI configuration space, so we have to
	 * re-disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state
	 */
	pci_write_config_byte(pdev, 0x41, 0);

	ath_init(sc);
	if (test_bit(ATH_STAT_LEDSOFT, sc->status)) {
		ath5k_hw_set_gpio_output(sc->ah, sc->led_pin);
		ath5k_hw_set_gpio(sc->ah, sc->led_pin, 0);
	}

	return 0;
}
#else
#define ath_pci_suspend NULL
#define ath_pci_resume NULL
#endif /* CONFIG_PM */

static struct pci_driver ath_pci_drv_id = {
	.name		= "ath_pci",
	.id_table	= ath_pci_id_table,
	.probe		= ath_pci_probe,
	.remove		= __devexit_p(ath_pci_remove),
	.suspend	= ath_pci_suspend,
	.resume		= ath_pci_resume,
};

static int mincalibrate = 1;
static int maxcalibrate = INT_MAX / 1000;
#define	CTL_AUTO	-2	/* cannot be CTL_ANY or CTL_NONE */

static ctl_table ath_static_sysctls[] = {
#if AR_DEBUG
	{
	  .procname	= "debug",
	  .mode		= 0644,
	  .data		= &ath_debug,
	  .maxlen	= sizeof(ath_debug),
	  .proc_handler	= proc_dointvec
	},
#endif
	{
	  .procname	= "calibrate",
	  .mode		= 0644,
	  .data		= &ath_calinterval,
	  .maxlen	= sizeof(ath_calinterval),
	  .extra1	= &mincalibrate,
	  .extra2	= &maxcalibrate,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ 0 }
};
static ctl_table ath_ath_table[] = {
	{
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_static_sysctls
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{
	  .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_sysctl_header;

static int __init init_ath_pci(void)
{
	int ret;

	ret = pci_register_driver(&ath_pci_drv_id);
	if (ret) {
		printk(KERN_ERR "ath_pci: can't register pci driver\n");
		return ret;
	}
	ath_sysctl_header = register_sysctl_table(ath_root_table);

	return 0;
}

static void __exit exit_ath_pci(void)
{
	if (ath_sysctl_header)
		unregister_sysctl_table(ath_sysctl_header);
	pci_unregister_driver(&ath_pci_drv_id);
}

module_init(init_ath_pci);
module_exit(exit_ath_pci);

MODULE_AUTHOR("Jiri Slaby");
MODULE_DESCRIPTION("Support for Atheros 802.11 wireless LAN cards.");
MODULE_SUPPORTED_DEVICE("Atheros WLAN cards");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(ATH_PCI_VERSION " (EXPERIMENTAL)");
