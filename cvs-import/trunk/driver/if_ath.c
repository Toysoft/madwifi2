/*-
 * Copyright (c) 2002, 2003 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id$
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
#include <linux/cache.h>

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
static void	ath_fatal_tasklet(void *);
static void	ath_rxorn_tasklet(void *);
static void	ath_bmiss_tasklet(void *);
static int	ath_stop(struct net_device *);
static void	ath_ratectl(unsigned long);
static void	ath_initkeytable(struct ath_softc *);
static void	ath_mode_init(struct net_device *);
static int	ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
static void	ath_beacon_tasklet(void *);
static void	ath_beacon_free(struct ath_softc *);
static int	ath_desc_alloc(struct ath_softc *);
static void	ath_desc_free(struct ath_softc *);
static void	ath_node_free(struct ieee80211com *, struct ieee80211_node *);
static int	ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
static void	ath_rx_tasklet(void *);
static int	ath_hardstart(struct sk_buff *, struct net_device *);
static int	ath_mgtstart(struct sk_buff *, struct net_device *);
static int	ath_tx_start(struct net_device *, struct ieee80211_node *,
			     struct ath_buf *, struct sk_buff *);
static void	ath_tx_tasklet(void *);
static void	ath_tx_timeout(struct net_device *);
static int	ath_chan_set(struct ath_softc *, struct ieee80211channel *);
static void	ath_draintxq(struct ath_softc *);
static void	ath_stoprecv(struct ath_softc *);
static int	ath_startrecv(struct net_device *);
static void	ath_next_scan(unsigned long);
static void	ath_calibrate(unsigned long);
static int	ath_newstate(void *, enum ieee80211_state);
static struct net_device_stats *ath_getstats(struct net_device *);

static int	ath_rate_setup(struct net_device *, u_int mode);
static void	ath_rate_ctl(void *, struct ieee80211_node *);

#ifdef CONFIG_PROC_FS
static void	ath_proc_init(struct ath_softc *sc);
static void	ath_proc_remove(struct ath_softc *sc);
#endif /* CONFIG_PROC_FS */

#ifdef AR_DEBUG
int	ath_debug = 0;
#define	IFF_DUMPPKTS(_ic)	(ath_debug || netif_msg_dumppkts(_ic))
static	void ath_printrxbuf(struct ath_buf *bf, int);
static	void ath_printtxbuf(struct ath_buf *bf, int);
#else
#define	IFF_DUMPPKTS(_ic)	netif_msg_dumppkts(_ic)
#endif

int
ath_attach(u_int16_t devid, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah;
	int i, error = 0, ix, nchan;
	u_int8_t csz;
#define	ATH_MAXCHAN	32		/* number of potential channels */
	HAL_CHANNEL chans[ATH_MAXCHAN];		/* XXX get off stack */
	HAL_STATUS status;

	DPRINTF(("ath_attach: devid 0x%x\n", devid));

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(sc->sc_pdev, PCI_CACHE_LINE_SIZE, &csz);
	/* XXX assert csz is non-zero */
	sc->sc_cachelsz = csz << 2;		/* convert to bytes */

	spin_lock_init(&sc->sc_txbuflock);
	spin_lock_init(&sc->sc_txqlock);

	INIT_TQUEUE(&sc->sc_rxtq,	ath_rx_tasklet,		dev);
	INIT_TQUEUE(&sc->sc_txtq,	ath_tx_tasklet,		dev);
	INIT_TQUEUE(&sc->sc_swbatq,	ath_beacon_tasklet,	dev);
	INIT_TQUEUE(&sc->sc_bmisstq,	ath_bmiss_tasklet,	dev);
	INIT_TQUEUE(&sc->sc_rxorntq,	ath_rxorn_tasklet,	dev);
	INIT_TQUEUE(&sc->sc_fataltq,	ath_fatal_tasklet,	dev);

	ah = ath_hal_attach(devid, sc, 0, (void *) dev->mem_start, &status);
	if (ah == NULL) {
		printk(KERN_ERR "%s: unable to attach hardware; HAL status %u\n",
			dev->name, status);
		error = ENXIO;
		goto bad;
	}
	sc->sc_ah = ah;

	/* XXX where does the country code, et. al. come from? */
	if (!ath_hal_init_channels(ah, chans, ATH_MAXCHAN, &nchan,
#ifdef notdef
		CTRY_DEFAULT, HAL_MODE_ALL, AH_TRUE)) {
#else
		CTRY_DEFAULT, HAL_MODE_11A|HAL_MODE_11B, AH_FALSE)) {
#endif
		printk(KERN_ERR "%s: unable to initialize channel list\n",
			dev->name);
		error = EINVAL;
		goto bad;
	}
	/*
	 * Convert HAL channels to ieee80211 ones and insert
	 * them in the table according to their channel number.
	 */
	for (i = 0; i < nchan; i++) {
		HAL_CHANNEL *c = &chans[i];
		ix = ath_hal_mhz2ieee(c->channel, c->channelFlags);
		if (ix > IEEE80211_CHAN_MAX) {
			printk(KERN_ERR "%s: bad HAL channel %u/%x ignored\n",
				dev->name, c->channel, c->channelFlags);
			continue;
		}
		if (ic->ic_channels[ix].ic_freq != 0) {
			printk(KERN_ERR "%s: chan %u already (freq %u)\n",
				dev->name, ix, ic->ic_channels[ix].ic_freq);
			continue;
		}
		ic->ic_channels[ix].ic_freq = c->channel;
		/* NB: flags are known to be compatible */
		ic->ic_channels[ix].ic_flags = c->channelFlags;
	}
	/*
	 * Setup rate tables for all potential media types.
	 */
	ath_rate_setup(dev, IEEE80211_MODE_11A);
	ath_rate_setup(dev, IEEE80211_MODE_11B);
	ath_rate_setup(dev, IEEE80211_MODE_11G);

	error = ath_desc_alloc(sc);
	if (error != 0) {
		printk("%s: failed to allocate descriptors: %d\n",
			dev->name, error);
		goto bad;
	}

	/*
	 * For now just pre-allocate one data queue and one
	 * beacon queue.  Note that the HAL handles resetting
	 * them at the needed time.  Eventually we'll want to
	 * allocate more tx queues for splitting management
	 * frames and for QOS support.
	 */
	sc->sc_txhalq = ath_hal_setuptxqueue(ah,
		HAL_TX_QUEUE_DATA,
		AH_TRUE			/* enable interrupts */
	);
	if (sc->sc_txhalq == (u_int) -1) {
		printk("%s: unable to setup a data xmit queue!\n", dev->name);
		goto bad;
	}
	sc->sc_bhalq = ath_hal_setuptxqueue(ah,
		HAL_TX_QUEUE_BEACON,
		AH_TRUE			/* enable interrupts */
	);
	if (sc->sc_bhalq == (u_int) -1) {
		printk("%s: unable to setup a beacon xmit queue!\n", dev->name);
		goto bad;
	}

	init_timer(&sc->sc_rate_ctl);
	sc->sc_rate_ctl.data = (unsigned long) dev;
	sc->sc_rate_ctl.function = ath_ratectl;

	init_timer(&sc->sc_scan_ch);
	sc->sc_scan_ch.function = ath_next_scan;
	sc->sc_scan_ch.data = (unsigned long) dev;

	init_timer(&sc->sc_cal_ch);
	sc->sc_cal_ch.function = ath_calibrate;
	sc->sc_cal_ch.data = (unsigned long) dev;

	ether_setup(dev);
	dev->open = ath_init;
	dev->hard_start_xmit = ath_hardstart;
	dev->tx_timeout = ath_tx_timeout;
	dev->watchdog_timeo = 5 * HZ;			/* XXX */
	dev->set_multicast_list = ath_mode_init;
	dev->get_stats = ath_getstats;
	dev->tx_queue_len = ATH_TXBUF-1;		/* 1 for mgmt frame */

	ic->ic_mgtstart = ath_mgtstart;
	ic->ic_init = ath_init;

	ic->ic_newstate = ath_newstate;
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_caps = IEEE80211_C_WEP | IEEE80211_C_IBSS | IEEE80211_C_HOSTAP;
	ic->ic_node_privlen = sizeof(struct ath_nodestat);
	ic->ic_node_free = ath_node_free;
	ic->ic_bss.ni_private = &sc->sc_bss_stat;

	/* get mac address from hardware */
	ath_hal_getmac(ah, dev->dev_addr);

	/* call MI attach routine. */
	ieee80211_ifattach(dev);

	/* don't accept xmit's until we are associated */
	netif_stop_queue(dev);

	printk("%s: 802.11 address: %s\n",
		dev->name, ether_sprintf(dev->dev_addr));

#ifdef CONFIG_PROC_FS
	ath_proc_init(sc);
#endif
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

	DPRINTF(("ath_detach flags %x\n", dev->flags));
	sc->sc_invalid = 1;
	ath_stop(dev);
	ath_desc_free(sc);
	ath_hal_detach(sc->sc_ah);
#ifdef CONFIG_PROC_FS
	ath_proc_remove(sc);
#endif
	ieee80211_ifdetach(dev);

	return 0;
}

void
ath_suspend(struct net_device *dev)
{
	DPRINTF(("ath_suspend flags %x\n", dev->flags));
	ath_stop(dev);
}

void
ath_resume(struct net_device *dev)
{
	DPRINTF(("ath_resume %x\n", dev->flags));
	ath_init(dev);
}

void
ath_shutdown(struct net_device *dev)
{
	DPRINTF(("ath_shutdown %x\n", dev->flags));
	ath_stop(dev);
}

/*
 * Interrupt handler.  All the actual processing is
 * deferred to tasklets.
 */
void
ath_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	HAL_INT status;
	int needmark;

	if (sc->sc_invalid) {
		/*
		 * The hardware is gone, don't touch anything.
		 * XXX can this happen?
		 */
		return;
	}
	if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
		DPRINTF(("ath_intr: flags 0x%x\n", dev->flags));
		ath_hal_getisr(ah, &status);	/* clear ISR */
		ath_hal_intrset(ah, 0);		/* disable further intr's */
		return;
	}
	needmark = 0;
	ath_hal_getisr(ah, &status);
	DPRINTF2(("%s: interrupt, status 0x%x\n", dev->name, status));
#ifdef AR_DEBUG
	if (ath_debug &&
	    (status & (HAL_INT_FATAL|HAL_INT_RXORN|HAL_INT_BMISS))) {
		printk("%s: ath_intr: status 0x%x\n", dev->name, status);
		ath_hal_dumpstate(ah);
	}
#endif /* AR_DEBUG */
	if (status & HAL_INT_FATAL) {
		sc->sc_stats.ast_hardware++;
		needmark |= queue_task(&sc->sc_fataltq, &tq_immediate);
	} else if (status & HAL_INT_RXORN) {
		sc->sc_stats.ast_rxorn++;
		needmark |= queue_task(&sc->sc_rxorntq, &tq_immediate);
	} else {
		if (status & HAL_INT_RXEOL) {
			/*
			 * XXX The hardware should re-read the link when
			 * RXE bit is written, but it doesn't work at least
			 * on older revision of the hardware.
			 */
			sc->sc_stats.ast_rxeol++;
			sc->sc_rxlink = NULL;
		}
		if (status & HAL_INT_TXURN) {
			sc->sc_stats.ast_txurn++;
			/* bump tx trigger level */
			ath_hal_updatetxtriglevel(ah, AH_TRUE);
		}
		if (status & HAL_INT_RX)
			needmark |= queue_task(&sc->sc_rxtq, &tq_immediate);
		if (status & HAL_INT_TX)
			needmark |= queue_task(&sc->sc_txtq, &tq_immediate);
		if (status & HAL_INT_SWBA)
			needmark |= queue_task(&sc->sc_swbatq, &tq_immediate);
		if (status & HAL_INT_BMISS) {
			sc->sc_stats.ast_bmiss++;
			needmark |= queue_task(&sc->sc_bmisstq, &tq_immediate);
		}
	}
	if (needmark)
		mark_bh(IMMEDIATE_BH);
}

static void
ath_fatal_tasklet(void *data)
{
	struct net_device *dev = data;

	printk("%s: hardware error; resetting\n", dev->name);
	ath_init(dev);
}

static void
ath_rxorn_tasklet(void *data)
{
	struct net_device *dev = data;

	printk("%s: rx FIFO overrun; resetting\n", dev->name);
	ath_init(dev);
}

static void
ath_bmiss_tasklet(void *data)
{
	struct net_device *dev = data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
			
	DPRINTF(("ath_bmiss_tasklet\n"));
	/* XXX probably better to send a probe before rescanning */
	if (ic->ic_opmode == IEEE80211_M_STA && ic->ic_state == IEEE80211_S_RUN)
		ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
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
	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop(dev);

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	if (!ath_hal_reset(ah, ic->ic_opmode, (HAL_CHANNEL *) ic->ic_ibss_chan,
	     AH_FALSE, &status)) {
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
	if (ath_startrecv(dev) != 0) {
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

	dev->flags |= IFF_RUNNING;
	ic->ic_state = IEEE80211_S_INIT;

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
			if (ic->ic_sup_rates[ni->ni_mode][i])
				ni->ni_rates[ni->ni_nrate++] =
				    ic->ic_sup_rates[ni->ni_mode][i];
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
		ni->ni_chan = ic->ic_ibss_chan;
		ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
	}

	/*
	 * Start the background rate control thread.
	 */
	mod_timer(&sc->sc_rate_ctl, jiffies + HZ);

	return 0;
}

static int
ath_stop(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(("ath_stop: invalid %u flags 0x%x\n",
		sc->sc_invalid, dev->flags));

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Shutdown the hardware and driver:
		 *    stop output from above
		 *    disable interrupts
		 *    turn off timers
		 *    clear transmit machinery
		 *    clear receive machinery
		 *    drain and release tx queues
		 *    reclaim beacon resources
		 *    reset 802.11 state machine
		 *    power down hardware
		 *
		 * Note that some of this work is not possible if the
		 * hardware is gone (invalid).
		 */
		netif_stop_queue(dev);
		dev->flags &= ~IFF_RUNNING;
		/* XXX how/when to stop ieee80211 timer? */
		if (!sc->sc_invalid)
			ath_hal_intrset(ah, 0);
		ath_draintxq(sc);
		if (!sc->sc_invalid)
			ath_stoprecv(sc);
		else
			sc->sc_rxlink = NULL;
		ath_beacon_free(sc);
		ieee80211_new_state(dev, IEEE80211_S_INIT, -1);
		del_timer(&sc->sc_rate_ctl);
		if (!sc->sc_invalid)
			ath_hal_setpower(ah, HAL_PM_FULL_SLEEP, 0);
	}

	return 0;
}

/*
 * Transmit a data packet.  On failure caller is
 * assumed to reclaim the resources.
 */
static int
ath_hardstart(struct sk_buff *skb, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_buf *bf = NULL;
	struct ieee80211_frame *wh;
	int error;

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(("ath_hardstart: discard, invalid %d flags %x\n",
			sc->sc_invalid, dev->flags));
		sc->sc_stats.ast_tx_invalid++;
		error = -ENETDOWN;
		goto bad;
	}
	/*
	 * No data frames go out unless we're associated; this
	 * should not happen as the 802.11 layer does not enable
	 * the xmit queue until we enter the RUN state.
	 */
	if (ic->ic_state != IEEE80211_S_RUN) {
		DPRINTF(("ath_hardstart: discard, state %u\n", ic->ic_state));
		sc->sc_stats.ast_tx_discard++;
		error = -ENETDOWN;
		goto bad;
	}
	/*
	 * Grab a TX buffer and associated resources.
	 */
	spin_lock_bh(&sc->sc_txbuflock);
	bf = TAILQ_FIRST(&sc->sc_txbuf);
	if (bf != NULL)
		TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
	/* XXX use a counter and leave at least one for mgmt frames */
	if (TAILQ_EMPTY(&sc->sc_txbuf)) {
		DPRINTF(("ath_hardstart: stop queue\n"));
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(dev);
	}
	spin_unlock_bh(&sc->sc_txbuflock);
	if (bf == NULL) {		/* NB: should not happen */
		printk("ath_hardstart: discard, no xmit buf\n");
		sc->sc_stats.ast_tx_nobuf++;
		error = -ENOBUFS;
		goto bad;
	}
	/*
	 * Encapsulate the packet for transmission.
	 */
	skb = ieee80211_encap(dev, skb);
	if (skb == NULL) {
		DPRINTF(("ath_hardstart: discard, encapsulation failure\n"));
		sc->sc_stats.ast_tx_encap++;
#ifdef notdef
		error = -ENOMEM;
#else
		/*
		 * Can't return an error code here because the caller
		 * reclaims the skbuff on error and it's already gone.
		 */
		error = 0;
#endif
		goto bad;
	}
	wh = (struct ieee80211_frame *) skb->data;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		wh->i_fc[1] |= IEEE80211_FC1_WEP;

	ni = ieee80211_find_node(ic, wh->i_addr1);
	if (ni == NULL &&
	    !IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    ic->ic_opmode != IEEE80211_M_STA) {
		DPRINTF(("ath_hardstart: discard, no destination state\n"));
		sc->sc_stats.ast_tx_nonode++;
		error = -EHOSTUNREACH;
		goto bad;
	}
	if (ni == NULL)
		ni = ieee80211_ref_node(&ic->ic_bss);

	/*
	 * TODO:
	 * The duration field of 802.11 header should be filled.
	 * XXX This may be done in the ieee80211 layer, but the upper 
	 *     doesn't know the detail of parameters such as IFS
	 *     for now..
	 */

	if (IFF_DUMPPKTS(ic))
		ieee80211_dump_pkt(skb->data, skb->len,
		    ni->ni_rates[ni->ni_txrate] & IEEE80211_RATE_VAL, -1);

	error = ath_tx_start(dev, ni, bf, skb);
	ieee80211_unref_node(&ni);
	if (error == 0)
		return 0;
	/* fall thru... */
bad:
	if (bf != NULL) {
		spin_lock_bh(&sc->sc_txbuflock);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock_bh(&sc->sc_txbuflock);
	}
	return error;
}

/*
 * Transmit a management frame.  On failure we reclaim the skbuff.
 * Note that management frames come directly from the 802.11 layer
 * and do not honor the send queue flow control.  Need to investigate
 * using priority queueing so management frames can bypass data.
 */
static int
ath_mgtstart(struct sk_buff *skb, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_buf *bf = NULL;
	struct ieee80211_frame *wh;
	int error;

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(("ath_mgtstart: discard, invalid %d flags %x\n",
			sc->sc_invalid, dev->flags));
		sc->sc_stats.ast_tx_invalid++;
		error = -ENETDOWN;
		goto bad;
	}
	/*
	 * Grab a TX buffer and associated resources.
	 */
	spin_lock_bh(&sc->sc_txbuflock);
	bf = TAILQ_FIRST(&sc->sc_txbuf);
	if (bf != NULL)
		TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
	if (TAILQ_EMPTY(&sc->sc_txbuf))	{
		DPRINTF(("ath_hardstart: stop queue\n"));
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(dev);
	}
	spin_unlock_bh(&sc->sc_txbuflock);
	if (bf == NULL) {
		printk("ath_mgtstart: discard, no xmit buf\n");
		sc->sc_stats.ast_tx_nobufmgt++;
		error = -ENOBUFS;
		goto bad;
	}
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
	ni = ieee80211_find_node(ic, wh->i_addr1);
	if (ni == NULL)
		ni = ieee80211_ref_node(&ic->ic_bss);

	/*
	 * TODO:
	 * The duration field of 802.11 header should be filled.
	 * XXX This may be done in the ieee80211 layer, but the upper 
	 *     doesn't know the detail of parameters such as IFS
	 *     for now..
	 */

	if (IFF_DUMPPKTS(ic))
		ieee80211_dump_pkt(skb->data, skb->len,
		    ni->ni_rates[ni->ni_txrate] & IEEE80211_RATE_VAL, -1);

	error = ath_tx_start(dev, ni, bf, skb);
	ieee80211_unref_node(&ni);
	if (error == 0) {
		sc->sc_stats.ast_tx_mgmt++;
		/*
		 * XXX probably shouldn't be mucking with this
		 * stuff, but we need to timeout the xmit.
		 */
		dev->trans_start = jiffies;
		__netdev_watchdog_up(dev);
		return 0;
	}
	/* fall thru... */
bad:
	if (bf != NULL) {
		spin_lock_bh(&sc->sc_txbuflock);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock_bh(&sc->sc_txbuflock);
	}
	dev_kfree_skb(skb);
	return error;
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

static struct sk_buff *
ath_alloc_skb(u_int size, u_int align)
{
	struct sk_buff *skb;
	u_int off;

	skb = dev_alloc_skb(size + align-1);
	if (skb != NULL) {
		off = ((unsigned long) skb->data) % align;
		if (off != 0)
			skb_reserve(skb, off);
	}
	return skb;
}

static int
ath_beacon_alloc(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = &ic->ic_dev;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_frame *wh;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct sk_buff *skb;
	int pktlen;
	u_int8_t *frm, rate;
	u_int16_t capinfo;
	const HAL_RATE_TABLE *rt;

	bf = sc->sc_bcbuf;
	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
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
	/*
	 * Beacon frames must be aligned to a 32-bit boundary and
	 * the buffer length must be a multiple of 4 bytes.  Allocate
	 * an skbuff large enough for us to insure this.
	 */
	skb = ath_alloc_skb(roundup(sizeof(struct ieee80211_frame)+pktlen, 4),
		sizeof(u_int32_t));
	if (skb == NULL) {
		DPRINTF(("ath_beacon_alloc: cannot allocate sk_buff; size %u\n",
			roundup(sizeof(struct ieee80211_frame)+pktlen, 4)));
		sc->sc_stats.ast_be_nobuf++;
		return ENOMEM;
	}

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

	bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
		skb->data, skb->len, PCI_DMA_TODEVICE);
	DPRINTF2(("ath_beacon_alloc: skb %p [data %p len %u] skbaddr %p\n",
		skb, skb->data, skb->len, (caddr_t) bf->bf_skbaddr));
	bf->bf_skb = skb;

	/* setup descriptors */
	ds = bf->bf_desc;

	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	/* XXX verify mbuf data area covers this roundup */
	/* XXX compression processing */
	/*
	 * Calculate rate code.  CCK beacons need a long premable.
	 * XXX everything at 6Mbs
	 */
	rt = sc->sc_rates[ni->ni_mode];
	if (!IEEE80211_IS_CHAN_PUREG(ni->ni_chan))
		rate = rt->info[0].rateCode | rt->info[0].shortPreamble;
	else
		rate = rt->info[0].rateCode;
	ath_hal_setuptxdesc(ah, ds
		, skb->len + IEEE80211_CRC_LEN	/* frame length */
		, sizeof(struct ieee80211_frame)/* header length */
		, HAL_PKT_TYPE_BEACON		/* Atheros packet type */
		, 0x20				/* txpower XXX */
		, rate, 4 /*XXX*/		/* series 0 rate/tries */
		, HAL_TXKEYIX_INVALID		/* no encryption */
		, 0				/* antenna mode */
		, HAL_TXDESC_NOACK		/* no ack for beacons */
		| HAL_TXDESC_VEOL		/* veol-style re-xmit */
		, 0				/* rts/cts rate */
		, 0				/* rts/cts duration */
	);
	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	ath_hal_filltxdesc(ah, ds
		, roundup(skb->len, 4)		/* buffer length */
		, AH_TRUE			/* first segment */
		, AH_TRUE			/* last segment */
	);
	return 0;
}

static void
ath_beacon_tasklet(void *data)
{
	struct net_device *dev = data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_buf *bf = sc->sc_bcbuf;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(("ath_beacon_tasklet\n"));
	if (!ath_hal_waitforbeacon(ah, bf)) {
		DPRINTF(("ath_beacon_int: TXQ1F busy"));
		return;				/* busy */
	}
	if (ic->ic_opmode == IEEE80211_M_STA ||
	    bf == NULL || bf->bf_skb == NULL) {
		DPRINTF(("ath_beacon_int: ic_flags=%x bf=%p bf_m=%p\n",
		    ic->ic_flags, bf, bf ? bf->bf_skb : NULL));
		return;
	}
	pci_dma_sync_single(sc->sc_pdev,
		bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);

	/*
	 * NB: For parts other than the 5210 this will happen
	 * only once.  Because the descriptor is marked with
	 * VEOL the frame will be retransmitted automatically
	 * at the next beacon time interval w/o interrupting
	 * the host.  The 5210 doesn't have VEOL support so for
	 * that part we take a real interrupt and re-submit the
	 * beacon frame here.
	 */
	ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
	ath_hal_txstart(ah, sc->sc_bhalq);
	DPRINTF2(("%s: TXDP%u = %p (%p)\n", __func__,
		sc->sc_bhalq, (caddr_t)bf->bf_daddr, bf->bf_desc));
	/* TODO power management */
}

static void
ath_beacon_free(struct ath_softc *sc)
{
	struct ath_buf *bf = sc->sc_bcbuf;

	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
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
	sc->sc_desc = pci_alloc_consistent(sc->sc_pdev,
				sc->sc_desc_len, &sc->sc_desc_daddr);
	if (sc->sc_desc == NULL)
		return ENOMEM;
	ds = sc->sc_desc;
	DPRINTF(("ath_desc_alloc: DMA map: %p (%d) -> %p\n",
	    ds, sc->sc_desc_len, (caddr_t) sc->sc_desc_daddr));

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
	pci_free_consistent(sc->sc_pdev, sc->sc_desc_len,
		sc->sc_desc, sc->sc_desc_daddr);
	sc->sc_desc = NULL;
	return ENOMEM;
}

static void
ath_desc_free(struct ath_softc *sc)
{
	struct ath_buf *bf;

	TAILQ_FOREACH(bf, &sc->sc_txq, bf_list) {
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
	}
	TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
		if (bf->bf_skb != NULL) {
			pci_unmap_single(sc->sc_pdev,
				bf->bf_skbaddr, bf->bf_skb->len,
				PCI_DMA_FROMDEVICE);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
		}
	if (sc->sc_bcbuf != NULL) {
		bf = sc->sc_bcbuf;
		pci_unmap_single(sc->sc_pdev, bf->bf_skbaddr,
			bf->bf_skb->len, PCI_DMA_TODEVICE);
		sc->sc_bcbuf = NULL;
	}

	pci_free_consistent(sc->sc_pdev, sc->sc_desc_len,
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

	/* XXX do we need to block bh? */
	spin_lock_bh(&sc->sc_txqlock);
	TAILQ_FOREACH(bf, &sc->sc_txq, bf_list) {
		if (bf->bf_node == ni)
			bf->bf_node = NULL;
	}
	spin_unlock_bh(&sc->sc_txqlock);
}

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;
	struct ath_desc *ds;

	skb = bf->bf_skb;
	if (skb == NULL) {
		/*
		 * Cache-line-align.  This is important (for the 5210 at
		 * least) as not doing so causes bogus data in rx'd frames.
		 */
		skb = ath_alloc_skb(sc->sc_rxbufsize, sc->sc_cachelsz);
		if (skb == NULL) {
			DPRINTF(("ath_rxbuf_init: skbuff allocation failed; "
				"size %u\n", sc->sc_rxbufsize));
			sc->sc_stats.ast_rx_nobuf++;
			return ENOMEM;
		}
		skb->dev = &sc->sc_ic.ic_dev;
		bf->bf_skb = skb;
		bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
			skb->data, skb->len, PCI_DMA_FROMDEVICE);
	}

	/* setup descriptors */
	ds = bf->bf_desc;
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath_hal_setuprxdesc(ah, ds
		, skb_tailroom(skb)		/* buffer size */
		, 0
	);

	if (sc->sc_rxlink != NULL)
		*sc->sc_rxlink = bf->bf_daddr;
	sc->sc_rxlink = &ds->ds_link;
	return 0;
}

static void
ath_rx_tasklet(void *data)
{
	struct net_device *dev = data;
	struct ath_buf *bf;
	struct ath_softc *sc = dev->priv;
	struct net_device_stats *stats = &sc->sc_ic.ic_stats;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	struct sk_buff *skb;
	struct ieee80211_frame *wh, whbuf;
	int len;
	u_int8_t phyerr;
	HAL_STATUS status;

	DPRINTF2(("ath_rx_tasklet\n"));
	do {
		bf = TAILQ_FIRST(&sc->sc_rxbuf);
		if (bf == NULL) {		/* XXX ??? can this happen */
			printk("ath_rx_tasklet: no buffer\n");
			break;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {		/* XXX ??? can this happen */
			printk("ath_rx_tasklet: no skbuff\n");
			continue;
		}
		ds = bf->bf_desc;
		status = ath_hal_rxprocdesc(ah, ds);
#ifdef AR_DEBUG
		if (ath_debug > 1)
			ath_printrxbuf(bf, status == HAL_OK); 
#endif
		if (status == HAL_EINPROGRESS)
			break;
		TAILQ_REMOVE(&sc->sc_rxbuf, bf, bf_list);

		if (ds->ds_rxstat.rs_status != 0) {
			if (ds->ds_rxstat.rs_status & HAL_RXERR_CRC)
				sc->sc_stats.ast_rx_crcerr++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_FIFO)
				sc->sc_stats.ast_rx_fifoerr++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_DECRYPT)
				sc->sc_stats.ast_rx_badcrypt++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_PHY) {
				sc->sc_stats.ast_rx_phyerr++;
				phyerr = ds->ds_rxstat.rs_phyerr & 0x1f;
				sc->sc_stats.ast_rx_phy[phyerr]++;
			}
			goto rx_next;
		}

		len = ds->ds_rxstat.rs_datalen;
		if (len < sizeof(struct ieee80211_frame)) {
			DPRINTF(("%s: ath_rx_tasklet: short packet %d\n",
			    dev->name, len));
			sc->sc_stats.ast_rx_tooshort++;
			goto rx_next;
		}

		pci_dma_sync_single(sc->sc_pdev,
			bf->bf_skbaddr, len, PCI_DMA_FROMDEVICE);

		wh = (struct ieee80211_frame *) skb->data;
		if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
		    IEEE80211_FC0_TYPE_CTL) {
			/*
			 * Ignore control frame received in promisc mode.
			 */
			DPRINTF(("%s: ath_rx_tasklet: discard ctl frame, "
				"fc %x\n", dev->name, wh->i_fc[0]));
			goto rx_next;
		}
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, skb->len, PCI_DMA_FROMDEVICE);
		bf->bf_skb = NULL;
		skb_put(skb, len);
		skb->protocol = ETH_P_CONTROL;		/* XXX */
		if (IFF_DUMPPKTS(&sc->sc_ic)) {
			struct ieee80211com *ic = &sc->sc_ic;
			const HAL_RATE_TABLE *rt = sc->sc_rates[ic->ic_curmode];
			ieee80211_dump_pkt(skb->data, len,
				rt->info[rt->rateCodeToIndex[ds->ds_rxstat.rs_rate]].dot11Rate & IEEE80211_RATE_VAL,
				ds->ds_rxstat.rs_rssi);
		}
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
		stats->rx_packets++;
		stats->rx_bytes += len;
		ieee80211_input(dev, skb, ds->ds_rxstat.rs_rssi,
			ds->ds_rxstat.rs_tstamp);
  rx_next:
		TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
	} while (ath_rxbuf_init(sc, bf) == 0);

	ath_hal_rxena(ah);			/* in case of RXEOL */
}

/*
 * XXX Size of an ACK control frame in bytes.
 */
#define	IEEE80211_ACK_SIZE	(2+2+IEEE80211_ADDR_LEN+4)

static int
ath_tx_start(struct net_device *dev, struct ieee80211_node *ni, struct ath_buf *bf,
    struct sk_buff *skb)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct net_device_stats *stats = &sc->sc_ic.ic_stats;
	int i, iswep, hdrlen, pktlen;
	u_int8_t rix, txrate, ctsrate;
	struct ath_desc *ds;
	struct ieee80211_frame *wh;
	u_int32_t iv;
	u_int8_t *ivp;
	u_int8_t hdrbuf[sizeof(struct ieee80211_frame) +
	    IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN];
	u_int subtype, flags, ctsduration;
	HAL_PKT_TYPE atype;
	const HAL_RATE_TABLE *rt;

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
		pktlen += IEEE80211_WEP_CRCLEN;
	}
	pktlen += IEEE80211_CRC_LEN;

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
	bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
		skb->data, skb->len, PCI_DMA_TODEVICE);
	DPRINTF2(("ath_tx_start: skb %p [data %p len %u] skbaddr %x\n",
		skb, skb->data, skb->len, bf->bf_skbaddr));
	bf->bf_skb = skb;
	bf->bf_node = ni;

	ds = bf->bf_desc;
	rt = sc->sc_rates[ni->ni_mode];

	/*
	 * Calculate Atheros packet type from IEEE80211 packet header
	 * and setup for rate calculations.
	 */
	atype = HAL_PKT_TYPE_NORMAL;			/* default */
	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_MGT:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_BEACON)
			atype = HAL_PKT_TYPE_BEACON;
		else if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			atype = HAL_PKT_TYPE_PROBE_RESP;
		else if (subtype == IEEE80211_FC0_SUBTYPE_ATIM)
			atype = HAL_PKT_TYPE_ATIM;
		rix = 0;			/* XXX lowest rate */
		break;
	case IEEE80211_FC0_TYPE_CTL:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_PS_POLL)
			atype = HAL_PKT_TYPE_PSPOLL;
		rix = 0;			/* XXX lowest rate */
		break;
	default:
		/* 
		 * XXX insert rate control work here.
		 */
		txrate = ni->ni_rates[ni->ni_txrate];
		for (rix = 0; rix < rt->rateCount; rix++)
			if (txrate == rt->info[rix].dot11Rate)
				break;
		break;
	}
	if (IEEE80211_IS_CHAN_PUREG(ni->ni_chan)) {
		/* 11g always uses long preamble */
		txrate = rt->info[rix].rateCode;
		flags &= ~HAL_TXDESC_SHORTPRE;
	} else {
		/* XXX no control over long/short preamble */
		txrate = rt->info[rix].rateCode | rt->info[rix].shortPreamble;
		flags |= HAL_TXDESC_SHORTPRE;
	}

	/*
	 * Calculate miscellaneous flags.
	 */
	flags = 0;
	if (IEEE80211_ADDR_EQ(wh->i_addr1, dev->broadcast))
		flags |= HAL_TXDESC_NOACK;

	/*
	 * Calculate RTS/CTS rate and duration if needed.
	 */
	ctsduration = 0;
	if (flags & (HAL_TXDESC_RTSENA|HAL_TXDESC_CTSENA)) {
		ctsrate = rt->info[rix].rateCode;
		if (flags & HAL_TXDESC_RTSENA) {	/* SIFS + CTS */
			ctsduration += ath_hal_computetxtime(ah,
				rt, IEEE80211_ACK_SIZE,
				ctsrate, (flags & HAL_TXDESC_SHORTPRE) != 0);
		}
		/* SIFS + data */
		ctsduration += ath_hal_computetxtime(ah, rt, pktlen,
			txrate, (flags & HAL_TXDESC_SHORTPRE) != 0);
		if ((flags & HAL_TXDESC_NOACK) == 0) {	/* SIFS + ACK */
			ctsduration += ath_hal_computetxtime(ah,
				rt, IEEE80211_ACK_SIZE,
				ctsrate, (flags & HAL_TXDESC_SHORTPRE) != 0);
		}
	} else
		ctsrate = 0;

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	ath_hal_setuptxdesc(ah, ds
		, pktlen		/* packet length */
		, hdrlen		/* header length */
		, atype			/* Atheros packet type */
		, 60			/* txpower XXX */
		, txrate, 1+10		/* series 0 rate/tries */
		, iswep ? sc->sc_ic.ic_wep_txkey : HAL_TXKEYIX_INVALID
		, 0			/* antenna mode */
		, flags			/* flags */
		, ctsrate		/* rts/cts rate */
		, ctsduration		/* rts/cts duration */
	);
#ifdef notyet
	ath_hal_setupxtxdesc(ah, ds
		, AH_FALSE		/* short preamble */
		, 0, 0			/* series 1 rate/tries */
		, 0, 0			/* series 2 rate/tries */
		, 0, 0			/* series 3 rate/tries */
	);
#endif

	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath_hal_filltxdesc(ah, ds
		, skb->len		/* segment length */
		, AH_TRUE		/* first segment */
		, AH_TRUE		/* last segment */
	);
	DPRINTF2(("ath_tx_start: %d: %08x %08x %08x %08x %08x %08x\n",
	    0, ds->ds_link, ds->ds_data, ds->ds_ctl0, ds->ds_ctl1,
	    ds->ds_hw[0], ds->ds_hw[1]));

	spin_lock_bh(&sc->sc_txqlock);
	TAILQ_INSERT_TAIL(&sc->sc_txq, bf, bf_list);
	if (sc->sc_txlink == NULL) {
		ath_hal_puttxbuf(ah, sc->sc_txhalq, bf->bf_daddr);
		DPRINTF2(("ath_tx_start: TXDP0 = %p (%p)\n",
		    (caddr_t)bf->bf_daddr, bf->bf_desc));
	} else {
		*sc->sc_txlink = bf->bf_daddr;
		DPRINTF2(("ath_tx_start: link(%p)=%p (%p)\n",
		    sc->sc_txlink, (caddr_t)bf->bf_daddr, bf->bf_desc));
	}
	sc->sc_txlink = &ds->ds_link;
	spin_unlock_bh(&sc->sc_txqlock);

	ath_hal_txstart(ah, sc->sc_txhalq);

	stats->tx_packets++;
	stats->tx_bytes += pktlen;

	return 0;
}

static void
ath_tx_tasklet(void *data)
{
	struct net_device *dev = data;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct ath_nodestat *st;
	int sr, lr;
	HAL_STATUS status;

	DPRINTF2(("ath_tx_tasklet: tx queue %p, link %p\n",
		(caddr_t) ath_hal_gettxbuf(sc->sc_ah, sc->sc_txhalq),
		sc->sc_txlink));
	for (;;) {
		spin_lock(&sc->sc_txqlock);
		bf = TAILQ_FIRST(&sc->sc_txq);
		if (bf == NULL) {
			sc->sc_txlink = NULL;
			spin_unlock(&sc->sc_txqlock);
			break;
		}
		ds = bf->bf_desc;		/* NB: last decriptor */
		status = ath_hal_txprocdesc(ah, ds);
#ifdef AR_DEBUG
		if (ath_debug > 1)
			ath_printtxbuf(bf, status == HAL_OK);
#endif
		if (status == HAL_EINPROGRESS) {
			spin_unlock(&sc->sc_txqlock);
			break;
		}
		TAILQ_REMOVE(&sc->sc_txq, bf, bf_list);
		spin_unlock(&sc->sc_txqlock);

		if (bf->bf_node != NULL) {
			st = bf->bf_node->ni_private;
			if (ds->ds_txstat.ts_status == 0)
				st->st_tx_ok++;
			else {
				st->st_tx_err++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_XRETRY)
					sc->sc_stats.ast_tx_xretries++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_FIFO)
					sc->sc_stats.ast_tx_fifoerr++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_FILT)
					sc->sc_stats.ast_tx_filtered++;
			}
			sr = ds->ds_txstat.ts_shortretry;
			lr = ds->ds_txstat.ts_longretry;
			sc->sc_stats.ast_tx_shortretry += sr;
			sc->sc_stats.ast_tx_longretry += lr;
			st->st_tx_retr += sr + lr;
		}
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		spin_lock(&sc->sc_txbuflock);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock(&sc->sc_txbuflock);
	}
	netif_wake_queue(dev);
}

static void
ath_tx_timeout(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	sc->sc_stats.ast_watchdog++;
	ath_hal_dumpstate(sc->sc_ah);	/*XXX*/
	ath_init(dev);
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
	if (!sc->sc_invalid) {
		(void) ath_hal_stoptxdma(ah, sc->sc_txhalq);
		DPRINTF(("ath_draintxq: tx queue %p, link %p\n",
		    (caddr_t) ath_hal_gettxbuf(ah, sc->sc_txhalq),
		    sc->sc_txlink));
		(void) ath_hal_stoptxdma(ah, sc->sc_bhalq);
		DPRINTF(("ath_draintxq: beacon queue %p\n",
		    (caddr_t) ath_hal_gettxbuf(ah, sc->sc_bhalq)));
	}
	for (;;) {
		spin_lock_bh(&sc->sc_txqlock);
		bf = TAILQ_FIRST(&sc->sc_txq);
		if (bf == NULL) {
			sc->sc_txlink = NULL;
			spin_unlock_bh(&sc->sc_txqlock);
			break;
		}
		TAILQ_REMOVE(&sc->sc_txq, bf, bf_list);
		spin_unlock_bh(&sc->sc_txqlock);
#ifdef AR_DEBUG
		if (ath_debug)
			ath_printtxbuf(bf,
				ath_hal_txprocdesc(ah, bf->bf_desc) == HAL_OK);
#endif /* AR_DEBUG */
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		spin_lock_bh(&sc->sc_txbuflock);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock_bh(&sc->sc_txbuflock);
	}
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

		printk("ath_stoprecv: rx queue %p, link %p\n",
		    (caddr_t) ath_hal_getrxbuf(ah), sc->sc_rxlink);
		TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			if (ath_hal_rxprocdesc(ah, bf->bf_desc) == HAL_OK)
				ath_printrxbuf(bf, 1);
		}
	}
#endif
	sc->sc_rxlink = NULL;		/* just in case */
}

/*
 * Enable the receive h/w following a reset.
 */
static int
ath_startrecv(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;

	sc->sc_rxbufsize = roundup(dev->mtu + IEEE80211_CRC_LEN +
		(IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN +
		 IEEE80211_WEP_CRCLEN), sc->sc_cachelsz);
	DPRINTF(("ath_startrecv: mtu %u cachelsz %u rxbufsize %u\n",
		dev->mtu, sc->sc_cachelsz, sc->sc_rxbufsize));

	sc->sc_rxlink = NULL;
	TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		if (error != 0)
			return error;
	}

	bf = TAILQ_FIRST(&sc->sc_rxbuf);
	ath_hal_putrxbuf(ah, bf->bf_daddr);
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
ath_chan_set(struct ath_softc *sc, struct ieee80211channel *chan)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = &ic->ic_dev;

	DPRINTF(("ath_chan_set: %u %uMHz -> %u %uMHz\n",
	    ieee80211_chan2ieee(ic, ic->ic_ibss_chan),
	    	ic->ic_ibss_chan->ic_freq,
	    ieee80211_chan2ieee(ic, chan), chan->ic_freq));
	if (chan != ic->ic_ibss_chan) {
		HAL_STATUS status;
		int val;

		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		if (!ath_hal_reset(ah, ic->ic_opmode,
		     (HAL_CHANNEL *) chan, AH_TRUE, &status)) {
			printk("%s: ath_chan_set: unable to reset "
				"channel %u (%uMhz)\n", dev->name,
				ieee80211_chan2ieee(ic, chan), chan->ic_freq);
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
		if (ath_startrecv(dev) != 0) {
			printk("%s: ath_chan_set: unable to restart recv "
				"logic\n", dev->name);
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

		ic->ic_ibss_chan = chan;
	}
	return 0;
}

static void
ath_next_scan(unsigned long arg)
{
	struct net_device *dev = (struct net_device *) arg;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	if (ic->ic_state == IEEE80211_S_SCAN)
		ieee80211_next_scan(dev);
}

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
static void
ath_calibrate(unsigned long arg)
{
	struct net_device *dev = (struct net_device *) arg;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211channel *c;

	c = ic->ic_ibss_chan;
	DPRINTF2(("ath_calibrate: channel %u/%x\n", c->ic_freq, c->ic_flags));
	if (!ath_hal_calibrate(ah, (HAL_CHANNEL *) c))
		printk("%s: ath_calibrate: calibration of channel %u failed\n",
			dev->name, c->ic_freq);

	sc->sc_cal_ch.expires = jiffies + HZ;
	add_timer(&sc->sc_cal_ch);
}

static int
ath_newstate(void *arg, enum ieee80211_state nstate)
{
	struct net_device *dev = (struct net_device *) arg;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = &ic->ic_bss;	/* NB: no reference */
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

	ath_hal_setledstate(ah, leds[nstate]);	/* set LED */
	netif_stop_queue(dev);			/* before we do anything else */

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
			 , ieee80211_chan2ieee(ic, ni->ni_chan)));
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

		netif_start_queue(dev);
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

static int
ath_rate_setup(struct net_device *dev, u_int mode)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const HAL_RATE_TABLE *rt;
	u_int8_t *r;
	int i, maxrates;

	switch (mode) {
	case IEEE80211_MODE_11A:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11A);
		break;
	case IEEE80211_MODE_11B:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11B);
		break;
	case IEEE80211_MODE_11G:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11B);
		break;
	case IEEE80211_MODE_TURBO:
		sc->sc_rates[mode] =
			ath_hal_getratetable(ah, HAL_MODE_TURBO);
		break;
	default:
		DPRINTF(("%s: invalid mode %u\n", __func__, mode));
		return 0;
	}
	r = ic->ic_sup_rates[mode];
	rt = sc->sc_rates[mode];
	if (rt->rateCount > IEEE80211_RATE_SIZE) {
		DPRINTF(("%s: rate table too small (%u > %u)\n",
			__func__, rt->rateCount, IEEE80211_RATE_SIZE));
		maxrates = IEEE80211_RATE_SIZE;
	} else
		maxrates = rt->rateCount;
	/* XXX must mark basic rates for 11g */
	for (i = 0; i < maxrates; i++)
		*r++ = rt->info[i].dot11Rate;
	return 1;
}

static void
ath_rate_ctl(void *arg, struct ieee80211_node *ni)
{
	struct ath_softc *sc = arg;
	struct ath_nodestat *st = ni->ni_private;
	int mod = 0, orate, enough;

	/*
	 * Rate control
	 * XXX: very primitive version.
	 */
	(void) sc;		/* NB: silence compiler */

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

static void
ath_ratectl(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	if (dev->flags & IFF_RUNNING) {
		if (ic->ic_opmode == IEEE80211_M_STA)
			ath_rate_ctl(sc, &ic->ic_bss);	/* NB: no reference */
		else
			ieee80211_iterate_nodes(ic, ath_rate_ctl, sc);
	}
	sc->sc_rate_ctl.expires = jiffies + HZ;		/* once a second */
	add_timer(&sc->sc_rate_ctl);
}

#ifdef AR_DEBUG
static void
ath_printrxbuf(struct ath_buf *bf, int done)
{
	struct ath_desc *ds = bf->bf_desc;

	printk("R (%p %p) %08x %08x %08x %08x %08x %08x %c\n",
	    ds, (struct ath_desc *)bf->bf_daddr,
	    ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1,
	    ds->ds_hw[0], ds->ds_hw[1],
	    !done ? ' ' : (ds->ds_rxstat.rs_status == 0) ? '*' : '!');
}

static void
ath_printtxbuf(struct ath_buf *bf, int done)
{
	struct ath_desc *ds = bf->bf_desc;

	printk("T (%p %p) %08x %08x %08x %08x %08x %08x %08x %08x %c\n",
	    ds, (struct ath_desc *)bf->bf_daddr,
	    ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1,
	    ds->ds_hw[0], ds->ds_hw[1], ds->ds_hw[2], ds->ds_hw[3],
	    !done ? ' ' : (ds->ds_txstat.ts_status == 0) ? '*' : '!');
}
#endif /* AR_DEBUG */

/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ath_getstats(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct net_device_stats *stats = &sc->sc_ic.ic_stats;

	/* update according to private statistics */
	stats->tx_errors = sc->sc_stats.ast_tx_encap
			 + sc->sc_stats.ast_tx_nonode
			 + sc->sc_stats.ast_tx_xretries
			 + sc->sc_stats.ast_tx_fifoerr
			 + sc->sc_stats.ast_tx_filtered
			 ;
	stats->tx_dropped = sc->sc_stats.ast_tx_nobuf
			+ sc->sc_stats.ast_tx_nobufmgt;
	stats->rx_errors = sc->sc_stats.ast_rx_tooshort
			+ sc->sc_stats.ast_rx_crcerr
			+ sc->sc_stats.ast_rx_fifoerr
			+ sc->sc_stats.ast_rx_badcrypt
			+ sc->sc_stats.ast_rx_phyerr
			;
	stats->rx_length_errors = sc->sc_stats.ast_rx_phy[HAL_PHYERR_LEN];
	stats->rx_crc_errors = sc->sc_stats.ast_rx_crcerr;
	/* XXX??? */
	stats->rx_fifo_errors = sc->sc_stats.ast_rx_phy[HAL_PHYERR_TIM];
	/* XXX??? */
	stats->rx_missed_errors = sc->sc_stats.ast_rx_phy[HAL_PHYERR_TOR];

	return stats;
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>

static int
ath_proc_debug_read(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	if (off != 0) {
		*eof = 1;
		return 0;
	}
	return sprintf(page, "%d\n", ath_debug);
}

static int
ath_proc_debug_write(struct file *file, const char *buf,
	unsigned long count, void *data)
{
	int v;
	
	if (sscanf(buf, "%d", &v) == 1) {
		ath_debug = v;
		return count;
	} else
		return -EINVAL;
}

static int
ath_proc_stats(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct ath_softc *sc = (struct ath_softc *) data;
	char *cp = page;

	if (off != 0) {
		*eof = 1;
		return 0;
	}
#define	STAT(x) do {							\
	if (sc->sc_stats.ast_##x != 0)					\
		cp += sprintf(cp, #x "=%u\n", sc->sc_stats.ast_##x);	\
} while (0)
	STAT(watchdog);	  STAT(hardware);   STAT(bmiss);
	STAT(rxorn);	  STAT(rxeol);

	STAT(tx_mgmt);	  STAT(tx_qstop);   STAT(tx_discard); STAT(tx_invalid);
	STAT(tx_encap);	  STAT(tx_nonode);  STAT(tx_nobuf);   STAT(tx_nobufmgt);
	STAT(tx_xretries);STAT(tx_fifoerr); STAT(tx_filtered);
	STAT(tx_shortretry); STAT(tx_longretry);

	STAT(rx_orn);	  STAT(rx_crcerr);  STAT(rx_fifoerr); STAT(rx_badcrypt);
#define	PHYSTAT(x) do {							\
	if (sc->sc_stats.ast_rx_phy[HAL_PHYERR_##x] != 0)		\
		cp += sprintf(cp, "PHYERR_##x=%u\n",			\
			sc->sc_stats.ast_rx_phy[HAL_PHYERR_##x]);	\
} while (0)
	PHYSTAT(UND); PHYSTAT(TIM); PHYSTAT(PAR); PHYSTAT(RATE);
	PHYSTAT(LEN); PHYSTAT(QAM); PHYSTAT(SRV); PHYSTAT(TOR);
	STAT(rx_nobuf);

	STAT(be_nobuf);

	return cp - page;
#undef PHYSTAT
#undef STAT
}

static void
ath_proc_init(struct ath_softc *sc)
{
	struct proc_dir_entry *dp;

	/* XXX this uses /proc/net, but we want /proc/net/drivers */
	sc->sc_proc = proc_mkdir(sc->sc_ic.ic_dev.name, proc_net);
	if (sc->sc_proc == NULL) {
		printk(KERN_INFO "/proc/net/drivers/%s: failed to create\n",
			sc->sc_ic.ic_dev.name);
		return;
	}
	dp = create_proc_entry("debug", 0644, sc->sc_proc);
	if (dp) {
		dp->read_proc = ath_proc_debug_read;
		dp->write_proc = ath_proc_debug_write;
		dp->data = sc;
	}
	create_proc_read_entry("stats", 0644, sc->sc_proc, ath_proc_stats, sc);
}

static void
ath_proc_remove(struct ath_softc *sc)
{
	if (sc->sc_proc != NULL) {
		remove_proc_entry("stats", sc->sc_proc);
		remove_proc_entry("debug", sc->sc_proc);
		remove_proc_entry(sc->sc_ic.ic_dev.name, proc_net);
		sc->sc_proc = NULL;
	}
}
#endif /* CONFIG_PROC_FS */
