/*
 * Copyright (c) 2002, 2003 Sam Leffler.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Sam Leffler ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL John Hay BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Driver for the Atheros Wireless LAN controller.
 */
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/uaccess.h>

#define	AR_DEBUG
#include "if_athvar.h"
#include "if_ethersubr.h"		/* for ETHER_IS_MULTICAST */
#include "ah_desc.h"

/* unalligned little endian access */     
#define LE_READ_2(p)							\
	((u_int16_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8)))
#define LE_READ_4(p)							\
	((u_int32_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8) |	\
	  (((u_int8_t *)(p))[2] << 16) | (((u_int8_t *)(p))[3] << 24)))

static int	ath_init(struct net_device *);
static int	ath_stop(struct net_device *);
static void	ath_start(struct net_device *);
static void	ath_watchdog(struct net_device *);
static int	ath_ioctl(struct net_device *, struct ifreq *, int);
static int	ath_media_change(struct net_device *);
static void	ath_media_status(struct net_device *, struct ifmediareq *);
static void	ath_mode_init(struct net_device *);
static int	ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
static void	ath_beacon_int(struct net_device *);
static void	ath_beacon_free(struct ath_softc *);
static int	ath_desc_alloc(struct ath_softc *);
static void	ath_desc_free(struct ath_softc *);
static void	ath_node_free(struct ieee80211com *, struct ieee80211_node *);
static int	ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
static void	ath_rx_int(struct net_device *);
static int	ath_hardstart(struct sk_buff *, struct net_device *);
static int	ath_tx_start(struct ath_softc *, struct ieee80211_node *,
			     struct ath_buf *, struct sk_buff *);
static void	ath_tx_int(struct net_device *);
static HAL_CHANNEL *ath_chan_find(struct ath_softc *, u_int);
static int	ath_chan_set(struct ath_softc *, u_int8_t);
static void	ath_draintxq(struct ath_softc *);
static void	ath_stoprecv(struct ath_softc *);
static int	ath_startrecv(struct ath_softc *);
static void	ath_next_scan(unsigned long);
static void	ath_calibrate(unsigned long);
static int	ath_newstate(void *, enum ieee80211_state);
static void	ath_rate_ctl(struct ath_softc *, struct ieee80211_node *);

#ifdef AR_DEBUG
int	ath_debug = 2;		/*XXX*/
#define	IFF_DUMPPKTS(_ic)	(ath_debug || netif_msg_dumppkts(_ic))
static	void ath_printrxbuf(struct ath_buf *bf);
static	void ath_printtxbuf(struct ath_buf *bf);
#else
#define	IFF_DUMPPKTS(_ic)	netif_msg_dumppkts(_ic)
#endif

/*
 * Supported Rates.
 * The step of the table is 3Mbps.
 * Note that we won't use 9Mbps, which is worse than 6Mbps.
 */
static u_int8_t	ath_rate_tbl[] = {
	0 /*0M*/,	0 /*3M*/,	AR_Rate_6M,	0 /*9M*/,
	AR_Rate_12M,	0 /*15M*/,	AR_Rate_18M,	0 /*21M*/,
	AR_Rate_24M,	0 /*27M*/,	0 /*30M*/,	0 /*33M*/,
	AR_Rate_36M, 	0 /*39M*/,	0 /*42M*/,	0 /*45M*/,
	AR_Rate_48M,	0 /*51M*/,	AR_Rate_54M,
};

int
ath_attach(uint16_t devid, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah;
	int i, error = 0;
	u_int8_t *r;
	struct ifmediareq imr;
	HAL_STATUS status;

	DPRINTF(("ath_attach: unit %d devid 0x%x\n", sc->sc_unit, devid));

	spin_lock_init(&sc->sc_lock);
	skb_queue_head_init(&sc->sc_sndq);

	ah = ath_hal_attach(devid, sc, 0, (void *) dev->mem_start, &status);
	if (ah == NULL) {
		printk(KERN_ERR "%s: unable to attach hardware; HAL status %u\n",
			dev->name, status);
		error = ENXIO;
		goto bad;
	}
	sc->sc_ah = ah;

	/* XXX where does the country code, et. al. come from? */
	if (!ath_hal_init_channels(ah, sc->sc_channels, ATH_MAXCHAN,
#ifdef notdef
		CTRY_DEFAULT, MODE_SELECT_11A|MODE_SELECT_11B, 1)) {
#else
		CTRY_DEFAULT, MODE_SELECT_11A, 1)) {
#endif
		printk(KERN_ERR "%s: unable to initialize channel list\n",
			dev->name);
		error = EINVAL;
		goto bad;
	}
	/* start scanning from first channel */
	sc->sc_cur_chan = &sc->sc_channels[0];

	error = ath_desc_alloc(sc);
	if (error != 0) {
		printk("%s: failed to allocate descriptors: %d\n",
			dev->name, error);
		goto bad;
	}
	init_timer(&sc->sc_scan_ch);
	sc->sc_scan_ch.function = ath_next_scan;
	sc->sc_scan_ch.data = (unsigned long) sc;
	init_timer(&sc->sc_cal_ch);
	sc->sc_cal_ch.function = ath_calibrate;
	sc->sc_cal_ch.data = (unsigned long) sc;

	ether_setup(dev);
	dev->open = ath_init;
	dev->hard_start_xmit = ath_hardstart;
	dev->set_multicast_list = ath_mode_init;
	dev->do_ioctl = ath_ioctl;

	ic->ic_watchdog = ath_watchdog;
	ic->ic_start = ath_start;

	ic->ic_newstate = ath_newstate;
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_flags =
	    IEEE80211_F_HASWEP | IEEE80211_F_HASIBSS | IEEE80211_F_HASHOSTAP;
	ic->ic_node_privlen = sizeof(struct ath_nodestat);
	ic->ic_node_free = ath_node_free;
	ic->ic_bss.ni_private = &sc->sc_bss_stat;

	/* get mac address from hardware */
	ath_hal_getmac(ah, dev->dev_addr);

	/*
	 * Fill in 802.11 available channel set.
	 */
	memset(ic->ic_chan_avail, 0, sizeof(ic->ic_chan_avail));
	for (i = 0; i < ATH_MAXCHAN; i++) {
		HAL_CHANNEL *c = &sc->sc_channels[i];
		if (c->channelFlags)
			setbit(ic->ic_chan_avail,
				ath_hal_ghz2ieee(c->channel, c->channelFlags));
	}
	r = ic->ic_sup_rates;
	for (i = 0; i < sizeof(ath_rate_tbl); i++) {
		if (r == ic->ic_sup_rates + IEEE80211_RATE_SIZE)
			break;
		/* IEEE802.11 supported rate is in 0.5Mbps */
		switch (ath_rate_tbl[i]) {
		case 0:
			/* unsupported by hardware */
			break;
		case AR_Rate_6M:
		case AR_Rate_12M:
		case AR_Rate_24M:
			/* required rate in 802.11a */
			*r++ = i * 3 * 2 | 0x80;
			break;
		default:
			*r++ = i * 3 * 2;
			break;
		}
	}

	/* call MI attach routine. */
	ieee80211_ifattach(dev);

	printk("%s: 802.11 address: %s\n", dev->name, ether_sprintf(dev->dev_addr));

	ifmedia_init(&sc->sc_media, 0, ath_media_change, ath_media_status);
	ifmedia_add(&sc->sc_media,
	    IFM_MAKEWORD(IFM_IEEE80211, IFM_AUTO, 0, 0),
	    0, NULL);
	ifmedia_add(&sc->sc_media,
	    IFM_MAKEWORD(IFM_IEEE80211, IFM_AUTO, IFM_IEEE80211_ADHOC, 0),
	    0, NULL);
	ifmedia_add(&sc->sc_media,
	    IFM_MAKEWORD(IFM_IEEE80211, IFM_AUTO, IFM_IEEE80211_HOSTAP, 0),
	    0, NULL);
	ath_media_status(dev, &imr);
	ifmedia_set(&sc->sc_media, imr.ifm_active);

	sc->sc_attached = 1;

	return 0;
bad:
	if (ah)
		ath_hal_detach(ah);
	sc->sc_invalid = 1;
	return error;
}

int
ath_detach(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(("ath_detach: sc_attached %u\n", sc->sc_attached));
	if (sc->sc_attached) {
		sc->sc_invalid = 1;
		ath_stop(dev);
		ath_desc_free(sc);
		ath_hal_detach(sc->sc_ah);
		ifmedia_removeall(&sc->sc_media);
		ieee80211_ifdetach(dev);
	}
	return 0;
}

void
ath_suspend(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(("ath_suspend: sc_attached %u\n", sc->sc_attached));
	if (sc->sc_attached) {
		ath_stop(dev);
	}
}

void
ath_resume(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(("ath_resume: sc_attached %u\n", sc->sc_attached));
	if (sc->sc_attached) {
		if (netif_running(dev))
			ath_init(dev);
	}
}

void
ath_shutdown(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(("ath_shutdown: sc_attached %u\n", sc->sc_attached));
	if (sc->sc_attached)
		ath_stop(dev);
}

void
ath_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	HAL_INT status;

	ATH_LOCK(sc);
	while (ath_hal_intrpend(ah)) {
		ath_hal_getisr(ah, &status);
		DPRINTF2(("ath_intr: status 0x%x\n", status));
		if (status & HAL_INT_FATAL) {
			printk("%s: hardware error (0x%x); resetting\n",
				dev->name, status);
			ath_hal_dumpstate(ah);	/*XXX*/
			ath_init(dev);
			continue;
		}
		if (status & HAL_INT_RXORN) {
			printk("%s: rx FIFO overrun (0x%x); resetting\n",
				dev->name, status);
			ath_hal_dumpstate(ah);	/*XXX*/
			ath_init(dev);
			continue;
		}
		if (status & HAL_INT_RXEOL) {
			/*
			 * XXX The hardware should re-read the link when
			 * RXE bit is written, but it doesn't work at least
			 * on older revision of the hardware.
			 */
			sc->sc_rxlink = NULL;
		}

		if (status & HAL_INT_RX)
			ath_rx_int(dev);
		if (status & HAL_INT_TX)
			ath_tx_int(dev);
		if (status & HAL_INT_SWBA)
			ath_beacon_int(dev);
#if 0
		if (status & (HAL_INT_BMISS | HAL_INT_RXNOFRM)) {
DPRINTF(("ath_intr: beacon miss | rxnofrm: status 0x%x\n", status));
			if (ic->ic_opmode == IEEE80211_M_STA &&
			    ic->ic_state == IEEE80211_S_RUN)
				ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
		}
#endif
	}
	ATH_UNLOCK(sc);
}

/*
 * Fill the hardware key cache with key entries.
 */
static void
ath_initkeytable(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	int i;

	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
		struct ieee80211_wepkey *k = &ic->ic_nw_keys[i];
		if (k->wk_len == 0)
			ath_hal_keyreset(ah, i);
		else
			/* XXX return value */
			ath_hal_keyset(ah, i, (const HAL_KEYVAL *) k);
	}
}

static int
ath_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = &ic->ic_bss;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t val;
	HAL_STATUS status;

	DPRINTF(("ath_init\n"));

	ic->ic_state = IEEE80211_S_INIT;

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	if (!ath_hal_reset(ah, ic->ic_opmode, sc->sc_cur_chan, AH_FALSE, &status)) {
		printk("%s: unable to reset hardware; HAL status %u\n",
			dev->name, status);
		return EIO;
	}
	/*
	 * Setup the hardware after reset: the key cache
	 * is filled as needed and the receive engine is
	 * set going.  Frame transmit is handled entirely
	 * in the frame output path; there's nothing to do
	 * here except setup the interrupt mask.
	 */
	if (ic->ic_flags & IEEE80211_F_WEPON)
		ath_initkeytable(sc);
	if (ath_startrecv(sc) != 0) {
		printk("%s: unable to start recv logic\n", dev->name);
		return EIO;
	}
	/*
	 * Enable interrupts.
	 */
	val = HAL_INT_RX | HAL_INT_TX |
	      HAL_INT_RXEOL | HAL_INT_RXORN | HAL_INT_FATAL;
	if (ic->ic_opmode == IEEE80211_M_STA)		/* beacon miss */
		val |= HAL_INT_BMISS;
	else
		val |= HAL_INT_SWBA;			/* beacon prepare */
	ath_hal_intrset(ah, val | HAL_INT_GLOBAL);

	netif_start_queue(dev);

	/*
	 * The hardware should be ready to go now so it's safe
	 * to kick the 802.11 state machine as it's likely to
	 * immediately call back to us to send mgmt frames.
	 */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		int i;
		ni->ni_chan = ic->ic_ibss_chan;
		ni->ni_intval = ic->ic_lintval;
		ni->ni_rssi = 0;
		ni->ni_rstamp = 0;
		memset(ni->ni_tstamp, 0, sizeof(ni->ni_tstamp));
		ni->ni_nrate = 0;
		for (i = 0; i < IEEE80211_RATE_SIZE; i++) {
			if (ic->ic_sup_rates[i])
				ni->ni_rates[ni->ni_nrate++] =
				    ic->ic_sup_rates[i];
		}
		memcpy(ni->ni_macaddr, dev->dev_addr, IEEE80211_ADDR_LEN);
		memcpy(ni->ni_bssid, dev->dev_addr, IEEE80211_ADDR_LEN);
		ni->ni_esslen = ic->ic_des_esslen;
		memcpy(ni->ni_essid, ic->ic_des_essid, ni->ni_esslen);
		ni->ni_capinfo = IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_WEPON)
			ni->ni_capinfo |= IEEE80211_CAPINFO_PRIVACY;
		ic->ic_flags |= IEEE80211_F_SIBSS;
		ic->ic_state = IEEE80211_S_SCAN;	/*XXX*/
		ieee80211_new_state(dev, IEEE80211_S_RUN, -1);
	} else {
		HAL_CHANNEL *c = sc->sc_cur_chan;
		ni->ni_chan = ath_hal_ghz2ieee(c->channel, c->channelFlags);
		ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
	}

	return 0;
}

static int
ath_stop(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;

	DPRINTF(("ath_stop: sc_invalid %u\n", sc->sc_invalid));

	ATH_LOCK(sc);
	ieee80211_new_state(dev, IEEE80211_S_INIT, -1);
	if (!sc->sc_invalid) {
		/* 
		 * Disable interrupts and stop everything.
		 */
		ath_hal_intrset(ah, 0);
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		ath_hal_setpower(ah, PM_FULL_SLEEP, 0);
	}
	ath_beacon_free(sc);
	sc->sc_txlink = sc->sc_rxlink = NULL;
	sc->sc_oactive = 0;
	sc->sc_ic.ic_timer = 0;
	sc->sc_tx_timer = 0;
	skb_queue_purge(&sc->sc_sndq);

	/* free transmit queue */
	while ((bf = TAILQ_FIRST(&sc->sc_txq)) != NULL) {
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, bf->bf_skb->len, PCI_DMA_FROMDEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
		TAILQ_REMOVE(&sc->sc_txq, bf, bf_list);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
	}
	ATH_UNLOCK(sc);

	return 0;
}

/*
 * Accept packets from above; we queue them and kick the common
 * start code.  This is very un-Linux like but insures any management
 * frames get priority w/o totally mangling the existing logic.
 */
static int
ath_hardstart(struct sk_buff *skb, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	skb_queue_tail(&sc->sc_sndq, skb);
	if (!sc->sc_oactive)
		ath_start(dev);

	return 0;
}

static void
ath_start(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_buf *bf;
	struct sk_buff *skb;
	struct ieee80211_frame *wh;

	ATH_LOCK(sc);
	if (sc->sc_invalid) {
		ATH_UNLOCK(sc);
		return;
	}
	for (;;) {
		/*
		 * Grab a TX buffer and associated resources.
		 */
		bf = TAILQ_FIRST(&sc->sc_txbuf);
		/*
		 * Poll the management queue for frames; they
		 * have priority over normal data frames.
		 */
		skb = skb_peek(&ic->ic_mgtq);
		if (skb != NULL) {
			if (bf == NULL) {
				DPRINTF(("ath_start: out of xmit buffers (mgtq)\n"));
				sc->sc_oactive = 0;
				break;
			}
			skb = skb_dequeue(&ic->ic_mgtq);
			wh = (struct ieee80211_frame *) skb->data;
			if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) ==
			    IEEE80211_FC0_SUBTYPE_PROBE_RESP) {
				/* fill time stamp */
				u_int64_t tsf;
				u_int32_t *tstamp;

				tsf = ath_hal_gettsf(ah);
				/* XXX: adjust 100us delay to xmit */
				tsf += 100;
				tstamp = (u_int32_t *)&wh[1];
				tstamp[0] = cpu_to_le32(tsf & 0xffffffff);
				tstamp[1] = cpu_to_le32(tsf >> 32);
			}
		} else {
			/*
			 * No data frames go out unless we're associated.
			 */
			if (ic->ic_state != IEEE80211_S_RUN) {
				DPRINTF(("ath_start: discard data packet, "
					"state %u\n", ic->ic_state));
				break;
			}
			skb = skb_peek(&sc->sc_sndq);
			if (skb == NULL)
				break;
			if (bf == NULL) {
				DPRINTF(("ath_start: out of xmit buffers (data)\n"));
				sc->sc_oactive = 1;
				break;
			}
			skb = skb_dequeue(&sc->sc_sndq);
			ic->ic_stats.tx_packets++;
			/*
			 * Encapsulate the packet in prep for transmission.
			 */
			skb = ieee80211_encap(dev, skb);
			if (skb == NULL) {
				DPRINTF(("ath_start: encapsulation failure\n"));
				sc->sc_stats.ast_tx_encap++;
				ic->ic_stats.tx_errors++;
				continue;
			}
			wh = (struct ieee80211_frame *) skb->data;
			if (ic->ic_flags & IEEE80211_F_WEPON)
				wh->i_fc[1] |= IEEE80211_FC1_WEP;
		}

		ni = ieee80211_find_node(ic, wh->i_addr1);
		if (ni == NULL &&
		    !IEEE80211_IS_MULTICAST(wh->i_addr1) &&
		    ic->ic_opmode != IEEE80211_M_STA &&
		    (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
		    IEEE80211_FC0_TYPE_DATA) {
			dev_kfree_skb(skb);
			sc->sc_stats.ast_tx_nonode++;
			ic->ic_stats.tx_errors++;
			continue;
		}
		if (ni == NULL)
			ni = &ic->ic_bss;

		/*
		 * TODO:
		 * The duration field of 802.11 header should be filled.
		 * XXX This may be done in the ieee80211 layer, but the upper 
		 *     doesn't know the detail of parameters such as IFS
		 *     for now..
		 */

		if (IFF_DUMPPKTS(ic))
			ieee80211_dump_pkt(skb->data, skb->len,
			    ni->ni_rates[ni->ni_txrate] & IEEE80211_RATE_VAL,
			    -1);

		if (ath_tx_start(sc, ni, bf, skb)) {
			ic->ic_stats.tx_errors++;
			continue;
		}

		sc->sc_tx_timer = 5;
		ic->ic_timer = 1;
	}
	ATH_UNLOCK(sc);
}

static void
ath_watchdog(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;

	ic->ic_timer = 0;
	if (sc->sc_invalid)
		return;
	if (sc->sc_tx_timer) {
		if (--sc->sc_tx_timer == 0) {
			printk("%s: device timeout\n", dev->name);
			ath_hal_dumpstate(sc->sc_ah);	/*XXX*/
			ath_init(dev);
			ic->ic_stats.tx_errors++;
			sc->sc_stats.ast_watchdog++;
			return;
		}
		ic->ic_timer = 1;
	}
	if (sc->sc_ic.ic_opmode == IEEE80211_M_STA)
		ath_rate_ctl(sc, &sc->sc_ic.ic_bss);
	else {
		TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
			ath_rate_ctl(sc, ni);
		}
	}
	ieee80211_watchdog(dev);
}

static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ath_softc *sc = dev->priv;
	int error = 0;

	switch (cmd) {
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(dev, ifr, &sc->sc_media, cmd);
		break;
	default:
		/* XXX ifr vs data */
		error = ieee80211_ioctl(dev, cmd, (caddr_t) ifr);
		break;
	}
	if (error == ENETRESET) {
		ath_init(dev);			/* XXX lose error */
		error = 0;
	}
	return error;
}

static int
ath_media_change(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifmedia_entry *ime;
	int error;

	error = 0;
	ime = sc->sc_media.ifm_cur;
	switch (IFM_SUBTYPE(ime->ifm_media)) {
	case IFM_AUTO:
		break;
	default:
		error = EIO;
		break;		/* XXX ??? */
	}
	if (ime->ifm_media & IFM_IEEE80211_ADHOC) {
		ic->ic_opmode = IEEE80211_M_IBSS;
		ic->ic_flags |= IEEE80211_F_IBSSON;
	} else if (ime->ifm_media & IFM_IEEE80211_HOSTAP) {
		ic->ic_opmode = IEEE80211_M_HOSTAP;
		ic->ic_flags &= ~IEEE80211_F_IBSSON;
	} else {
		ic->ic_opmode = IEEE80211_M_STA;
		ic->ic_flags &= ~IEEE80211_F_IBSSON;
	}
	ath_init(dev);			/* XXX lose error */
	return error;
}

static void
ath_media_status(struct net_device *dev, struct ifmediareq *imr)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211_node *ni = NULL;

	imr->ifm_status = IFM_AVALID;
	imr->ifm_active = IFM_IEEE80211;
	if (sc->sc_ic.ic_state == IEEE80211_S_RUN)
		imr->ifm_status |= IFM_ACTIVE;
	imr->ifm_active |= IFM_AUTO;
	switch (sc->sc_ic.ic_opmode) {
	case IEEE80211_M_STA:
		ni = &sc->sc_ic.ic_bss;
		break;
	case IEEE80211_M_IBSS:
		imr->ifm_active |= IFM_IEEE80211_ADHOC;
		break;
	case IEEE80211_M_AHDEMO:
		/* should not come here */
		break;
	case IEEE80211_M_HOSTAP:
		imr->ifm_active |= IFM_IEEE80211_HOSTAP;
		break;			/* NB: skip media line rate */
	}
	if (ni == NULL)
		return;
	/* XXX long/short preamble??? */
	switch (ni->ni_rates[ni->ni_txrate] & IEEE80211_RATE_VAL) {
	case 0:
		imr->ifm_active |= IFM_AUTO;
		break;
	case 2:
		imr->ifm_active |= IFM_IEEE80211_DS1;
		break;
	case 4:
		imr->ifm_active |= IFM_IEEE80211_DS2;
		break;
	case 11:
		imr->ifm_active |= IFM_IEEE80211_DS5;
		break;
	case 12:
		imr->ifm_active |= IFM_IEEE80211_ODFM6;
		break;
	case 18:
		imr->ifm_active |= IFM_IEEE80211_ODFM9;
		break;
	case 22:
		imr->ifm_active |= IFM_IEEE80211_DS11;
		break;
	case 24:
		imr->ifm_active |= IFM_IEEE80211_ODFM12;
		break;
	case 36:
		imr->ifm_active |= IFM_IEEE80211_ODFM18;
		break;
	case 44:
		imr->ifm_active |= IFM_IEEE80211_DS22;
		break;
	case 48:
		imr->ifm_active |= IFM_IEEE80211_ODFM24;
		break;
	case 72:
		imr->ifm_active |= IFM_IEEE80211_ODFM36;
		break;
	case 108:
		imr->ifm_active |= IFM_IEEE80211_ODFM54;
		break;
	case 144:
		imr->ifm_active |= IFM_IEEE80211_ODFM72;
		break;
	}
}

static void
ath_mode_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt, mfilt[2], val;
	u_int8_t pos;
	struct dev_mc_list *mc;

	/* configure operational mode */
	ath_hal_setopmode(ah, ic->ic_opmode);

	/* receive filter */
	rfilt = HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST | HAL_RX_FILTER_MCAST;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP && (dev->flags & IFF_PROMISC))
		rfilt |= HAL_RX_FILTER_PROM;
	if (ic->ic_state == IEEE80211_S_SCAN)
		rfilt |= HAL_RX_FILTER_BEACON;
	ath_hal_setrxfilter(ah, rfilt);

	/* calculate and install multicast filter */
	if ((dev->flags & IFF_ALLMULTI) == 0) {
		mfilt[0] = mfilt[1] = 0;
		for (mc = dev->mc_list; mc; mc = mc->next) {
			/* calculate XOR of eight 6bit values */
			val = LE_READ_4(mc->dmi_addr + 0);
			pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			val = LE_READ_4(mc->dmi_addr + 3);
			pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			pos &= 0x3f;
			mfilt[pos / 32] |= (1 << (pos % 32));
		}
	} else {
		mfilt[0] = mfilt[1] = ~0;
	}
	ath_hal_setmcastfilter(ah, mfilt[0], mfilt[1]);
	DPRINTF(("ath_mode_init: RX filter 0x%x, MC filter %08x:%08x\n",
	    rfilt, mfilt[0], mfilt[1]));
}

static int
ath_beacon_alloc(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = &ic->ic_dev;
#if 0
	struct ath_hal *ah = sc->sc_ah;
#endif
	struct ieee80211_frame *wh;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct sk_buff *skb;
	int arate, pktlen;
	u_int8_t *frm;
	u_int16_t capinfo;

	bf = sc->sc_bcbuf;
	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
	}
	pktlen = + 8
		  + sizeof(u_int16_t)
		  + sizeof(u_int16_t)
		  + 2 + ni->ni_esslen
		  + 2 + ni->ni_nrate
		  + 6;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;

	wh = (struct ieee80211_frame *)
		skb_put(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_BEACON;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(u_int16_t *)wh->i_dur = 0;
	memcpy(wh->i_addr1, dev->broadcast, IEEE80211_ADDR_LEN);
	memcpy(wh->i_addr2, dev->dev_addr, IEEE80211_ADDR_LEN);
	memcpy(wh->i_addr3, ni->ni_bssid, IEEE80211_ADDR_LEN);
	*(u_int16_t *)wh->i_seq = 0;

	/*
	 * beacon frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[tlv] parameter set (IBSS)
	 */
	frm = (u_int8_t *) skb_put(skb, pktlen);
	memset(frm, 0, 8);	/* timestamp is set by hardware */
	frm += 8;
	*(u_int16_t *)frm = cpu_to_le16(ni->ni_intval);
	frm += 2;
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = ni->ni_esslen;
	memcpy(frm, ni->ni_essid, ni->ni_esslen);
	frm += ni->ni_esslen;
	*frm++ = IEEE80211_ELEMID_RATES;
	*frm++ = ni->ni_nrate;
	memcpy(frm, ni->ni_rates, ni->ni_nrate);
	frm += ni->ni_nrate;
	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		*frm++ = IEEE80211_ELEMID_IBSSPARMS;
		*frm++ = 2;
		*frm++ = 0; *frm++ = 0;		/* TODO: ATIM window */
	} else {
		/* TODO: TIM */
		*frm++ = IEEE80211_ELEMID_TIM;
		*frm++ = 4;	/* length */
		*frm++ = 0;	/* DTIM count */ 
		*frm++ = 1;	/* DTIM period */
		*frm++ = 0;	/* bitmap control */
		*frm++ = 0;	/* Partial Virtual Bitmap (variable length) */
	}
	skb_trim(skb, frm - skb->data);

	DPRINTF2(("ath_beacon_alloc: skb %p len %u\n", skb, skb->len));
	bf->bf_daddr = pci_map_single(sc->sc_pci_dev,
		skb->data, skb->len, PCI_DMA_TODEVICE);
	bf->bf_skb = skb;

	/* setup descriptors */
	arate = ath_rate_tbl[(ni->ni_rates[0] & IEEE80211_RATE_VAL) / 6];
	if (arate == 0)
		arate = AR_Rate_6M;
	ds = bf->bf_desc;

	ds->ds_link = 0;
	ds->ds_data = bf->bf_daddr;
	ds->ds_ctl0 = (skb->len + IEEE80211_CRC_LEN) |
	    (sizeof(struct ieee80211_frame) << AR_HdrLen_S) |
	    (arate << AR_XmitRate_S);
	ds->ds_ctl1 = roundup(skb->len, 4);
	ds->ds_status0 = ds->ds_status1 = 0;
#if 0
	(*ah->ah_setTxDescFrmLen)(ah, ds, m->m_pkthdr.len + IEEE80211_CRC_LEN);
	(*ah->ah_setTxDescHdrLen)(ah, ds, sizeof(struct ieee80211_frame));
	(*ah->ah_setTxDescRate)(ah, ds, ni->ni_rates[0] & IEEE80211_RATE_VAL);
	(*ah->ah_setTxDescBufLen)(ah, ds, roundup(bf->bf_segs[0].ds_len, 4));
#endif
	return 0;
}

static void
ath_beacon_int(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_buf *bf = sc->sc_bcbuf;
	struct ath_hal *ah = sc->sc_ah;

	if (!ath_hal_waitforbeacon(ah, bf)) {
		DPRINTF(("ath_beacon_int: TXQ1F busy"));
		return;				/* busy */
	}
	if (sc->sc_ic.ic_opmode == IEEE80211_M_STA ||
	    bf == NULL || bf->bf_skb == NULL) {
		DPRINTF(("ath_beacon_int: ic_flags=%x bf=%p bf_m=%p\n",
		    sc->sc_ic.ic_flags, bf, bf ? bf->bf_skb : NULL));
		return;
	}
	pci_dma_sync_single(sc->sc_pci_dev,
		bf->bf_daddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
	ath_hal_qbeacon(ah, bf);
	DPRINTF2(("ath_beacon_int: TXDP1 = %p (%p)\n",
	    (caddr_t)bf->bf_daddr, bf->bf_desc));
#if 0
	ATH_REG_WRITE(sc, AR_DMA_BCR,
	    (sc->sc_ic.ic_opmode == IEEE80211_M_IBSS ? AR_BCR_BCMD : 0) |
	    AR_BCR_BDMAE | AR_BCR_TQ1V);
#endif
	/* TODO power management */
}

static void
ath_beacon_free(struct ath_softc *sc)
{
	struct ath_buf *bf = sc->sc_bcbuf;

	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
	}
}

static int
ath_desc_alloc(struct ath_softc *sc)
{
	int i, bsize;
	struct ath_desc *ds;
	struct ath_buf *bf;

	/* allocate descriptors */
	sc->sc_desc_len = sizeof(struct ath_desc) *
				(ATH_TXBUF * ATH_TXDESC + ATH_RXBUF + 1);
	sc->sc_desc = pci_alloc_consistent(sc->sc_pci_dev,
				sc->sc_desc_len, &sc->sc_desc_daddr);
	if (sc->sc_desc == NULL)
		return ENOMEM;
	ds = sc->sc_desc;
	DPRINTF(("ath_desc_alloc: DMA map: %p (%d) -> %p (%lu)\n",
	    ds, sc->sc_desc_len,
	    (caddr_t) sc->sc_desc_daddr, /*XXX*/ (u_long) sc->sc_desc_len));

	/* allocate buffers */
	bsize = sizeof(struct ath_buf) * (ATH_TXBUF + ATH_RXBUF + 1);
	bf = kmalloc(bsize, GFP_KERNEL);
	if (bf == NULL)
		goto bad;
	memset(bf, 0, bsize);
	sc->sc_bufptr = bf;
	TAILQ_INIT(&sc->sc_rxbuf);
	for (i = 0; i < ATH_RXBUF; i++, bf++, ds++) {
		bf->bf_desc = ds;
		bf->bf_daddr = sc->sc_desc_daddr +
		    ((caddr_t)ds - (caddr_t)sc->sc_desc);
		TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
	}
	TAILQ_INIT(&sc->sc_txbuf);
	for (i = 0; i < ATH_TXBUF; i++, bf++, ds += ATH_TXDESC) {
		bf->bf_desc = ds;
		bf->bf_daddr = sc->sc_desc_daddr +
		    ((caddr_t)ds - (caddr_t)sc->sc_desc);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
	}
	TAILQ_INIT(&sc->sc_txq);
	/* beacon buffer */
	bf->bf_desc = ds;
	bf->bf_daddr = sc->sc_desc_daddr + ((caddr_t)ds - (caddr_t)sc->sc_desc);
	sc->sc_bcbuf = bf;
	return 0;

bad:
	pci_free_consistent(sc->sc_pci_dev, sc->sc_desc_len,
		sc->sc_desc, sc->sc_desc_daddr);
	sc->sc_desc = NULL;
	return ENOMEM;
}

static void
ath_desc_free(struct ath_softc *sc)
{
	struct ath_buf *bf;

	TAILQ_FOREACH(bf, &sc->sc_txq, bf_list) {
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
	}
	TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
		if (bf->bf_skb != NULL) {
			pci_unmap_single(sc->sc_pci_dev,
				bf->bf_daddr, bf->bf_skb->len,
				PCI_DMA_FROMDEVICE);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
		}
	if (sc->sc_bcbuf != NULL) {
		bf = sc->sc_bcbuf;
		pci_unmap_single(sc->sc_pci_dev, bf->bf_daddr,
			bf->bf_skb->len, PCI_DMA_TODEVICE);
		sc->sc_bcbuf = NULL;
	}

	pci_free_consistent(sc->sc_pci_dev, sc->sc_desc_len,
		sc->sc_desc, sc->sc_desc_daddr);

	TAILQ_INIT(&sc->sc_rxbuf);
	TAILQ_INIT(&sc->sc_txbuf);
	TAILQ_INIT(&sc->sc_txq);
	kfree(sc->sc_bufptr);
	sc->sc_bufptr = NULL;
}

static void
ath_node_free(struct ieee80211com *ic, struct ieee80211_node *ni)
{
        struct ath_softc *sc = ic->ic_dev.priv;
	struct ath_buf *bf;

	TAILQ_FOREACH(bf, &sc->sc_txq, bf_list) {
		if (bf->bf_node == ni)
			bf->bf_node = NULL;
	}
}

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct sk_buff *skb;
	struct ath_desc *ds;

	skb = bf->bf_skb;
	if (skb == NULL) {
		skb = dev_alloc_skb(IEEE80211_MAX_LEN);
		if (skb == NULL)
			return ENOMEM;
		skb->dev = &sc->sc_ic.ic_dev;
		bf->bf_skb = skb;
	}
	bf->bf_daddr = pci_map_single(sc->sc_pci_dev,
		skb->data, skb->len, PCI_DMA_FROMDEVICE);

	/* setup descriptors */
	ds = bf->bf_desc;
	ds->ds_link = 0;
	ds->ds_data = cpu_to_le32(bf->bf_daddr);
	ds->ds_ctl0 = 0;
	ds->ds_ctl1 = skb->len;
	ds->ds_status0 = ds->ds_status1 = 0;

	if (sc->sc_rxlink != NULL)
		*sc->sc_rxlink = cpu_to_le32(bf->bf_daddr);
	sc->sc_rxlink = &ds->ds_link;
	return 0;
}

static void
ath_rx_int(struct net_device *dev)
{
	struct ath_buf *bf;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	struct sk_buff *skb;
	struct ieee80211_frame *wh, whbuf;
	int rssi, len;
	u_int32_t rstamp, now;
	u_int8_t arate, rate;

	for (;;) {
		bf = TAILQ_FIRST(&sc->sc_rxbuf);
		skb = bf->bf_skb;
		if (skb == NULL)
			goto rx_next;
		ds = bf->bf_desc;
		if ((ds->ds_status1 & AR_Done) == 0)
			break;
		len = ds->ds_status0 & AR_DataLen;
		rssi = ATH_BITVAL(ds->ds_status0, AR_RcvSigStrength);
		rstamp = ATH_BITVAL(ds->ds_status1, AR_RcvTimestamp);
		arate = ATH_BITVAL(ds->ds_status0, AR_RcvRate);
		rate = ((arate & 0x4) ? 72 : 48) >> (arate & 0x3);
		if (rate == 72)
			rate = 54;
		pci_dma_sync_single(sc->sc_pci_dev,
			bf->bf_daddr, skb->len, PCI_DMA_FROMDEVICE);

		/* expand AR_RcvTimestamp(13bit) to 16bit */
		now = (ath_hal_gettsf(ah) >> 10) & 0xffff;
		if ((now & 0x1fff) < rstamp)
			rstamp |= (now - 0x2000) & 0xffff;
		else
			rstamp |= now;

		wh = (struct ieee80211_frame *) skb->data;
		if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
		    IEEE80211_FC0_TYPE_CTL) {
			/*
			 * Ignore control frame received in promisc mode.
			 */
			goto rx_next;
		}
		if (len < sizeof(struct ieee80211_frame)) {
#ifdef AR_DEBUG
			if (ath_debug) {
				printk("%s: ath_rx_int: short packet %d\n",
					dev->name, len);
				ieee80211_dump_pkt(skb->data, len, rate*2, rssi);
			}
#endif
			goto rx_next;
		}
		DPRINTF2(("R  (%p %p) %08x %08x %08x %08x %08x %08x %c\n",
		    ds, (struct ath_desc *)bf->bf_daddr,
		    ds->ds_link, ds->ds_data, ds->ds_ctl0, ds->ds_ctl1,
		    ds->ds_status0, ds->ds_status1,
		    (ds->ds_status1 & AR_FrmRcvOK) ? '*' : '!'));
		if ((ds->ds_status1 & AR_FrmRcvOK) == 0) {
			sc->sc_ic.ic_stats.rx_errors++;
			if (ds->ds_status1 & AR_CRCErr)
				sc->sc_stats.ast_rx_crcerr++;
			if (ds->ds_status1 & AR_FIFOOverrun)
				sc->sc_stats.ast_rx_fifoerr++;
			if (ds->ds_status1 & AR_DecryptCRCErr)
				sc->sc_stats.ast_rx_badcrypt++;
			if (ds->ds_status1 & AR_PHYErr) {
				sc->sc_stats.ast_rx_phyerr++;
				if (ds->ds_status1 & AR_PHYErr_Tim)
					sc->sc_stats.ast_rx_phy_tim++;
				if (ds->ds_status1 & AR_PHYErr_Par)
					sc->sc_stats.ast_rx_phy_par++;
				if (ds->ds_status1 & AR_PHYErr_Rate)
					sc->sc_stats.ast_rx_phy_rate++;
				if (ds->ds_status1 & AR_PHYErr_Len)
					sc->sc_stats.ast_rx_phy_len++;
				if (ds->ds_status1 & AR_PHYErr_QAM)
					sc->sc_stats.ast_rx_phy_qam++;
				if (ds->ds_status1 & AR_PHYErr_Srv)
					sc->sc_stats.ast_rx_phy_srv++;
				if (ds->ds_status1 & AR_PHYErr_TOR)
					sc->sc_stats.ast_rx_phy_tor++;
			}
			goto rx_next;
		}
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, skb->len, PCI_DMA_FROMDEVICE);
		bf->bf_skb = NULL;
		skb_put(skb, len);
		skb->protocol = ETH_P_CONTROL;		/* XXX */
		if (IFF_DUMPPKTS(&sc->sc_ic))
			ieee80211_dump_pkt(skb->data, len, rate * 2, rssi);
		skb_trim(skb, skb->len - IEEE80211_CRC_LEN);
		if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
			/*
			 * WEP is decrypted by hardware. Clear WEP bit
			 * and trim WEP header for ieee80211_input().
			 */
			wh->i_fc[1] &= ~IEEE80211_FC1_WEP;
			memcpy(&whbuf, wh, sizeof(whbuf));
			skb_pull(skb, IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN);
			memcpy(skb->data, &whbuf, sizeof(whbuf));
			/*
			 * Also trim WEP ICV from the tail.
			 */
			skb_trim(skb, skb->len - IEEE80211_WEP_CRCLEN);
		}
		ieee80211_input(dev, skb, rssi, rstamp);
  rx_next:
		TAILQ_REMOVE(&sc->sc_rxbuf, bf, bf_list);
		TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
		if (ath_rxbuf_init(sc, bf))
			break;
	}
	ath_hal_rxena(ah);			/* in case of RXEOL */
}

static int
ath_tx_start(struct ath_softc *sc, struct ieee80211_node *ni, struct ath_buf *bf,
    struct sk_buff *skb)
{
	struct ath_hal *ah = sc->sc_ah;
	int i, iswep, hdrlen, pktlen;
	u_int8_t rate, arate;
	struct ath_desc *ds;
	struct ieee80211_frame *wh;
	u_int32_t iv;
	u_int8_t *ivp;
	u_int8_t hdrbuf[sizeof(struct ieee80211_frame) +
	    IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN];
	u_int subtype;
	HAL_DESC_PKT_TYPE atype;

	wh = (struct ieee80211_frame *) skb->data;
	iswep = wh->i_fc[1] & IEEE80211_FC1_WEP;
	hdrlen = sizeof(struct ieee80211_frame);
	pktlen = skb->len;

	if (iswep) {
		memcpy(hdrbuf, skb->data, hdrlen);
		skb_pull(skb, hdrlen);
		skb_push(skb, sizeof(hdrbuf));
		ivp = hdrbuf + hdrlen;
		/*
		 * XXX
		 * IV must not duplicate during the lifetime of the key.
		 * But no mechanism to renew keys is defined in IEEE 802.11
		 * WEP.  And IV may be duplicated between other stations
		 * because of the session key itself is shared.
		 * So we use pseudo random IV for now, though it is not the
		 * right way.
		 */
		get_random_bytes(&iv, sizeof(iv));
		for (i = 0; i < IEEE80211_WEP_IVLEN; i++) {
			ivp[i] = iv;
			iv >>= 8;
		}
		ivp[i] = sc->sc_ic.ic_wep_txkey << 6;	/* Key ID and pad */
		memcpy(skb->data, hdrbuf, sizeof(hdrbuf));
		/*
		 * The ICV length must be included into hdrlen and pktlen.
		 */
		hdrlen = sizeof(hdrbuf) + IEEE80211_WEP_CRCLEN;
		pktlen = skb->len + IEEE80211_WEP_CRCLEN;
	}
	pktlen += IEEE80211_CRC_LEN;

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
	bf->bf_daddr = pci_map_single(sc->sc_pci_dev,
		skb->data, skb->len, PCI_DMA_TODEVICE);
	DPRINTF2(("ath_tx_start: skb %p len %u daddr %x\n",
		skb, pktlen, bf->bf_daddr));
	bf->bf_skb = skb;
	bf->bf_node = ni;

	/* setup descriptors */
	rate = ni->ni_rates[ni->ni_txrate] & IEEE80211_RATE_VAL;
	arate = (rate % 6 == 0) ? ath_rate_tbl[rate / 6] : 0;
	if (arate == 0)
		arate = AR_Rate_24M;

	ds = bf->bf_desc;
	/* first descriptor only */
	ds->ds_ctl0 = pktlen | (arate << AR_XmitRate_S);
	ds->ds_ctl1 = 0;
	ath_hal_settxdeschdrlen(ah, ds, hdrlen);
	/*
	 * Calculate Atheros packet type from IEEE80211 packet header.
	 */
	atype = HAL_DESC_PKT_TYPE_NORMAL;		/* default */
	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_MGT:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_BEACON)
			atype = HAL_DESC_PKT_TYPE_BEACON;
		else if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			atype = HAL_DESC_PKT_TYPE_PROBE_RESP;
		else if (subtype == IEEE80211_FC0_SUBTYPE_ATIM)
			atype = HAL_DESC_PKT_TYPE_ATIM;
		break;
	case IEEE80211_FC0_TYPE_CTL:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_PS_POLL)
			atype = HAL_DESC_PKT_TYPE_PSPOLL;
		break;
	}
	ath_hal_settxdescpkttype(ah, ds, atype);
	if (iswep)
		ath_hal_settxdesckey(ah, ds, sc->sc_ic.ic_wep_txkey);

	ds->ds_link = 0;
	ds->ds_data = bf->bf_daddr;
	ds->ds_ctl1 |= skb->len | 0;
	ds->ds_status0 = ds->ds_status1 = 0;
	DPRINTF2(("ath_tx_start: %08x %08x %08x %08x\n",
	    ds->ds_link, ds->ds_data, ds->ds_ctl0, ds->ds_ctl1));

	TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
	TAILQ_INSERT_TAIL(&sc->sc_txq, bf, bf_list);
	if (sc->sc_txlink == NULL) {
		ath_hal_puttxbuf(ah, bf->bf_daddr);
		DPRINTF2(("ath_tx_start: TXDP0 = %p (%p)\n",
		    (caddr_t)bf->bf_daddr, bf->bf_desc));
	} else {
		*sc->sc_txlink = bf->bf_daddr;
		DPRINTF2(("ath_tx_start: link(%p)=%p (%p)\n",
		    sc->sc_txlink, (caddr_t)bf->bf_daddr, bf->bf_desc));
	}
	sc->sc_txlink = &bf->bf_desc[0].ds_link;
	ath_hal_txstart(ah);

	return 0;
}

static void
ath_tx_int(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct ath_nodestat *st;

#ifdef AR_DEBUG
	if (ath_debug > 1) {
		printk("ath_tx_int: tx queue %p, link %p\n",
		    (caddr_t) ath_hal_gettxbuf(sc->sc_ah), sc->sc_txlink);
		TAILQ_FOREACH(bf, &sc->sc_txq, bf_list) {
			ath_printtxbuf(bf);
		}
	}
#endif /* AR_DEBUG */
	for (;;) {
		bf = TAILQ_FIRST(&sc->sc_txq);
		if (bf == NULL) {
			sc->sc_txlink = NULL;
			break;
		}
		/* only the last descriptor is needed */
		ds = &bf->bf_desc[0];
		if ((ds->ds_status1 & AR_Done) == 0)
			break;
		if (bf->bf_node != NULL) {
			st = bf->bf_node->ni_private;
			if (ds->ds_status0 & AR_FrmXmitOK)
				st->st_tx_ok++;
			else {
				st->st_tx_err++;
				sc->sc_ic.ic_stats.tx_errors++;
				sc->sc_stats.ast_tx_descerr++;
			}
			st->st_tx_retr +=
			    ATH_BITVAL(ds->ds_status0, AR_ShortRetryCnt) +
			    ATH_BITVAL(ds->ds_status0, AR_LongRetryCnt);
		}
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
		TAILQ_REMOVE(&sc->sc_txq, bf, bf_list);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
	}
	sc->sc_oactive = 0;
	sc->sc_tx_timer = 0;
	ath_start(dev);
}

/*
 * Locate the channel for the specified frequency.
 */
static HAL_CHANNEL *
ath_chan_find(struct ath_softc *sc, u_int freq)
{
	HAL_CHANNEL *c;

	for (c = sc->sc_cur_chan+1; c < &sc->sc_channels[64]; c++)
		if (c->channel == freq)
			return c;
	for (c = &sc->sc_channels[0]; c < sc->sc_cur_chan; c++)
		if (c->channel == freq)
			return c;
	return NULL;
}

/*
 * Drain the transmit queue and reclaim resources.
 */
static void
ath_draintxq(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;

	/* XXX return value */
	(void) ath_hal_stoptxdma(ah, HAL_TX_QUEUE_DATA);
	(void) ath_hal_stoptxdma(ah, HAL_TX_QUEUE_BEACON);
	DPRINTF(("ath_draintxq: tx queue %p, link %p\n",
	    (caddr_t) ath_hal_gettxbuf(ah), sc->sc_txlink));
	for (;;) {
		bf = TAILQ_FIRST(&sc->sc_txq);
		if (bf == NULL) {
			sc->sc_txlink = NULL;
			break;
		}
#ifdef AR_DEBUG
		if (ath_debug)
			ath_printtxbuf(bf);
#endif /* AR_DEBUG */
		pci_unmap_single(sc->sc_pci_dev,
			bf->bf_daddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
		TAILQ_REMOVE(&sc->sc_txq, bf, bf_list);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
	}
	sc->sc_oactive = 0;
	sc->sc_tx_timer = 0;
}

/*
 * Disable the receive h/w in preparation for a reset.
 */
static void
ath_stoprecv(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	ath_hal_stoppcurecv(ah);	/* disable PCU */
	ath_hal_setrxfilter(ah, 0);	/* clear recv filter */
	ath_hal_stopdmarecv(ah);	/* disable DMA engine */
	udelay(3000);			/* long enough for 1 frame */
#ifdef AR_DEBUG
	if (ath_debug) {
		struct ath_buf *bf;

		DPRINTF(("ath_stoprecv: rx queue %p, link %p\n",
		    (caddr_t) ath_hal_getrxbuf(ah), sc->sc_rxlink));
		TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			struct ath_desc *ds = bf->bf_desc;
			if (ds->ds_status1 & AR_Done)
				ath_printrxbuf(bf);
		}
	}
#endif
	sc->sc_rxlink = NULL;		/* just in case */
}

/*
 * Enable the receive h/w following a reset.
 */
static int
ath_startrecv(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;

	sc->sc_rxlink = NULL;
	TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		if (error != 0) {
			DPRINTF(("ath_startrecv: ath_rxbuf_init failed %d\n",
				error));
			return error;
		}
	}
	ath_hal_putrxbuf(ah, TAILQ_FIRST(&sc->sc_rxbuf)->bf_daddr);
	ath_hal_rxena(ah);		/* enable recv descriptors */
	ath_mode_init(&sc->sc_ic.ic_dev);/* set filters, etc. */
	ath_hal_startpcurecv(ah);	/* re-enable PCU/DMA engine */
	return 0;
}

/*
 * Set/change channels.  If the channel is really being changed,
 * it's done by resetting the chip.  To accomplish this we must
 * first cleanup any pending DMA, then restart stuff after a la
 * ath_init.
 */
static int
ath_chan_set(struct ath_softc *sc, u_int8_t chan)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = &ic->ic_dev;
	u_int freq = ath_hal_ieee2ghz(chan, 0);
	HAL_CHANNEL *cc = sc->sc_cur_chan;

	DPRINTF(("ath_chan_set: %u %uMHz -> %u %uMHz\n",
	    ath_hal_ghz2ieee(cc->channel, cc->channelFlags), cc->channel,
	    chan, freq));
	if (freq != cc->channel) {
		HAL_CHANNEL *nchan;
		HAL_STATUS status;
		int val;

		/*
		 * Locate the channel for the specified frequency.
		 */
		nchan = ath_chan_find(sc, freq);
		if (nchan == NULL) {
			printk("%s: ath_chan_find: channel %u not found!\n",
				dev->name, freq);
			return EINVAL;
		}

		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		if (!ath_hal_reset(ah, ic->ic_opmode, nchan, AH_TRUE, &status)) {
			printk("%s: ath_chan_set: unable to reset "
				"channel %u (%uMhz)\n", dev->name, chan, freq);
			return EIO;
		}

		/* XXX necessary??? doesn't look so */
		/* initialize WEP key in keytable */
		if (ic->ic_flags & IEEE80211_F_WEPON) {
			int i;
			for (i = 0; i < IEEE80211_WEP_NKID; i++) {
				struct ieee80211_wepkey *k = &ic->ic_nw_keys[i];
				if (k->wk_len == 0)
					ath_hal_keyreset(ah, i);
				else
					/* XXX return value */
					ath_hal_keyset(ah, i, (const HAL_KEYVAL *) k);
			}
		}

		/*
		 * Re-enable rx framework.
		 */
		if (ath_startrecv(sc) != 0) {
			printk("%s: ath_chan_set: unable to restart recv logic\n", dev->name);
			return EIO;
		}

		/*
		 * Enable interrupts.
		 */
		val = HAL_INT_RX | HAL_INT_TX |
		      HAL_INT_RXEOL | HAL_INT_RXORN | HAL_INT_FATAL;
		if (ic->ic_opmode == IEEE80211_M_STA)
			val |= HAL_INT_BMISS;		/* beacon miss */
		else
			val |= HAL_INT_SWBA;		/* beacon prepare */
		ath_hal_intrset(ah, val | HAL_INT_GLOBAL);

		sc->sc_cur_chan = nchan;
	}
	return 0;
}

static void
ath_next_scan(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *) arg;

	if (sc->sc_ic.ic_state != IEEE80211_S_SCAN)
		return;
	ieee80211_next_scan(&sc->sc_ic.ic_dev);
}

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
static void
ath_calibrate(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *) arg;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(("ath_calibrate: channel %u/%x\n",
		sc->sc_cur_chan->channel, sc->sc_cur_chan->channelFlags));
	if (!ath_hal_calibrate(ah, sc->sc_cur_chan))
		printk("%s: ath_calibrate: calibration of channel %u failed\n",
			sc->sc_ic.ic_dev.name, sc->sc_cur_chan->channel);

	sc->sc_scan_ch.expires = jiffies + HZ;
	add_timer(&sc->sc_scan_ch);
}

static int
ath_newstate(void *arg, enum ieee80211_state nstate)
{
	struct net_device *dev = (struct net_device *) arg;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = &ic->ic_bss;
	struct ath_nodestat *st;
	int i, error;
	u_int8_t *bssid;
	u_int32_t rfilt, val, now;
	enum ieee80211_state ostate;
#ifdef AR_DEBUG
	static const char *stname[] =
	    { "INIT", "SCAN", "AUTH", "ASSOC", "RUN" };
#endif /* AR_DEBUG */
	static const HAL_LED_STATE leds[] = {
	    HAL_LED_INIT,	/* IEEE80211_S_INIT */
	    HAL_LED_SCAN,	/* IEEE80211_S_SCAN */
	    HAL_LED_AUTH,	/* IEEE80211_S_AUTH */
	    HAL_LED_ASSOC, 	/* IEEE80211_S_ASSOC */
	    HAL_LED_RUN, 	/* IEEE80211_S_RUN */
	};

	ostate = ic->ic_state;
	now = (ath_hal_gettsf(ah) >> 10) & 0xffff;

	DPRINTF(("ath_newstate: %s -> %s (%u)\n",
	    stname[ostate], stname[nstate], now));

	ath_hal_setledstate(ah, leds[nstate]);		/* set LED */

	if (nstate == IEEE80211_S_INIT) {
		error = 0;			/* cheat + use error return */
		goto bad;
	}
	error = ath_chan_set(sc, ni->ni_chan);
	if (error != 0)
		goto bad;
	rfilt = HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST | HAL_RX_FILTER_MCAST;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP && (dev->flags & IFF_PROMISC))
		rfilt |= HAL_RX_FILTER_PROM;
	if (nstate == IEEE80211_S_SCAN) {
		mod_timer(&sc->sc_scan_ch, jiffies + (HZ / 5));
		bssid = dev->broadcast;
		rfilt |= HAL_RX_FILTER_BEACON;
	} else {
		del_timer(&sc->sc_scan_ch);
		bssid = ni->ni_bssid;
	}
	ath_hal_setrxfilter(ah, rfilt);
	DPRINTF(("ath_newstate: RX filter 0x%x bssid %s\n",
		 rfilt, ether_sprintf(bssid)));

	if (nstate == IEEE80211_S_RUN && ic->ic_opmode != IEEE80211_M_IBSS)
		ath_hal_setassocid(ah, bssid, ni->ni_associd);
	else
		ath_hal_setassocid(ah, bssid, 0);
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, bssid);
	}

	if (nstate == IEEE80211_S_RUN) {
		DPRINTF(("ath_newstate(RUN): ic_flags=0x%08x iv=%d bssid=%s "
			 "capinfo=0x%04x chan=%d\n"
			 , ic->ic_flags
			 , ni->ni_intval
			 , ether_sprintf(ni->ni_bssid)
			 , ni->ni_capinfo
			 , ni->ni_chan));
		if (ic->ic_opmode != IEEE80211_M_STA) {
			error = ath_beacon_alloc(sc, ni);
			if (error != 0)
				goto bad;
		}

		/* set beacon timers */
		val = (LE_READ_4(ni->ni_tstamp + 4) << 22) |
		    (LE_READ_4(ni->ni_tstamp) >> 10);
		DPRINTF(("ath_newstate: btime=%u rtime=%u now=%u\n",
		    val, ni->ni_rstamp, now));
		if (val != 0)
			val += roundup(now - ni->ni_rstamp, ni->ni_intval);
		val += ni->ni_intval;
		ath_hal_beaconinit(ah, ic->ic_opmode, val, ni->ni_intval,
			ic->ic_opmode != IEEE80211_M_STA ?
				sc->sc_bcbuf : NULL);

		/* start periodic recalibration timer */
		/* XXX make timer configurable */
		mod_timer(&sc->sc_cal_ch, jiffies + HZ);

		/* enable no frame received timeout interrupt */
		ath_hal_intrset(ah,  ath_hal_intrget(ah) | HAL_INT_RXNOFRM);
	} else {
		ath_hal_beaconreset(ah);	/* reset beacon timers */
		del_timer(&sc->sc_cal_ch);	/* no calibration */
	}
	st = &sc->sc_bss_stat;
	st->st_tx_ok = st->st_tx_err = st->st_tx_retr = st->st_tx_upper = 0;
	if (ic->ic_opmode != IEEE80211_M_STA) {
		TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
			st = ni->ni_private;
			st->st_tx_ok = st->st_tx_err = st->st_tx_retr =
			    st->st_tx_upper = 0;
		}
	}
	return 0;
bad:
	del_timer(&sc->sc_scan_ch);
	del_timer(&sc->sc_cal_ch);
	return error;
}

static void
ath_rate_ctl(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ath_nodestat *st = ni->ni_private;
	int mod = 0, orate, enough;

	/*
	 * Rate control
	 * XXX: very primitive version.
	 */

	enough = (st->st_tx_ok + st->st_tx_err >= 10);

	/* no packet reached -> down */
	if (st->st_tx_err > 0 && st->st_tx_ok == 0)
		mod = -1;

	/* all packets needs retry in average -> down */
	if (enough && st->st_tx_ok < st->st_tx_retr)
		mod = -1;

	/* no error and less than 10% of packets needs retry -> up */
	if (enough && st->st_tx_err == 0 && st->st_tx_ok > st->st_tx_retr * 10)
		mod = 1;

	orate = ni->ni_txrate;
	switch (mod) {
	case 0:
		if (enough && st->st_tx_upper > 0)
			st->st_tx_upper--;
		break;
	case -1:
		if (ni->ni_txrate > 0)
			ni->ni_txrate--;
		st->st_tx_upper = 0;
		break;
	case 1:
		if (++st->st_tx_upper < 2)
			break;
		st->st_tx_upper = 0;
		if (ni->ni_txrate + 1 < ni->ni_nrate)
			ni->ni_txrate++;
		break;
	}

	if (ni->ni_txrate != orate) {
		DPRINTF(("ath_rate_ctl: %dM -> %dM (%d ok, %d err, %d retr)\n",
		    (ni->ni_rates[orate] & IEEE80211_RATE_VAL) / 2,
		    (ni->ni_rates[ni->ni_txrate] & IEEE80211_RATE_VAL) / 2,
		    st->st_tx_ok, st->st_tx_err, st->st_tx_retr));
	}
	if (ni->ni_txrate != orate || enough)
		st->st_tx_ok = st->st_tx_err = st->st_tx_retr = 0;
}

#ifdef AR_DEBUG
static void
ath_printrxbuf(struct ath_buf *bf)
{
	struct ath_desc *ds;
	int i;

	for (i = 0, ds = bf->bf_desc; i < bf->bf_nseg; i++, ds++) {
		printk("R%d (%p %p) %08x %08x %08x %08x %08x %08x %c\n",
		    i, ds, (struct ath_desc *)bf->bf_daddr + i,
		    ds->ds_link, ds->ds_data,
		    ds->ds_ctl0, ds->ds_ctl1,
		    ds->ds_status0, ds->ds_status1,
		    !(ds->ds_status1 & AR_Done) ? ' ' :
		    (ds->ds_status0 & AR_FrmRcvOK) ? '*' : '!');
	}
}

static void
ath_printtxbuf(struct ath_buf *bf)
{
	struct ath_desc *ds;
	int i;

	for (i = 0, ds = bf->bf_desc; i < bf->bf_nseg; i++, ds++) {
		printk("T%d (%p %p) %08x %08x %08x %08x %08x %08x %c\n",
		    i, ds, (struct ath_desc *)bf->bf_daddr + i,
		    ds->ds_link, ds->ds_data,
		    ds->ds_ctl0, ds->ds_ctl1,
		    ds->ds_status0, ds->ds_status1,
		    !(ds->ds_status1 & AR_Done) ? ' ' :
		    (ds->ds_status0 & AR_FrmXmitOK) ? '*' : '!');
	}
}
#endif /* AR_DEBUG */
