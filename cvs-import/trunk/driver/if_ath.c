/*-
 * Copyright (c) 2002, 2003 Sam Leffler, Errno Consulting
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
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>

#include <asm/uaccess.h>

/*#define	AR_DEBUG*/
#include "if_athvar.h"
#include "if_ethersubr.h"		/* for ETHER_IS_MULTICAST */
#include "ah_desc.h"
#include "if_llc.h"

/* unalligned little endian access */     
#define LE_READ_2(p)							\
	((u_int16_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8)))
#define LE_READ_4(p)							\
	((u_int32_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8) |	\
	  (((u_int8_t *)(p))[2] << 16) | (((u_int8_t *)(p))[3] << 24)))

static int	ath_init(struct net_device *);
static void	ath_reset(struct net_device *);
static void	ath_fatal_tasklet(void *);
static void	ath_rxorn_tasklet(void *);
static void	ath_bmiss_tasklet(void *);
static int	ath_stop(struct net_device *);
static int	ath_media_change(struct net_device *);
static void	ath_ratectl(unsigned long);
static void	ath_initkeytable(struct ath_softc *);
static void	ath_mode_init(struct net_device *);
static int	ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
static void	ath_beacon_tasklet(void *);
static void	ath_beacon_free(struct ath_softc *);
static void	ath_beacon_config(struct ath_softc *);
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
static int	ath_getchannels(struct net_device *, u_int cc, HAL_BOOL outdoor);

static int	ath_rate_setup(struct net_device *, u_int mode);
static void	ath_setcurmode(struct ath_softc *, enum ieee80211_phymode);
static void	ath_rate_ctl_reset(struct ath_softc *, enum ieee80211_state);
static void	ath_rate_ctl(void *, struct ieee80211_node *);
static int      ath_change_mtu(struct net_device *, int);

static	int ath_dwelltime = 200;		/* 5 channels/second */
static	int ath_calinterval = 30;		/* calibrate every 30 secs */
static	int ath_rateinterval = 1000;		/* rate ctl interval (ms)  */
static	int ath_countrycode = CTRY_DEFAULT;	/* country code */
static	int ath_regdomain = 0;			/* regulatory domain */
static	int ath_outdoor = AH_TRUE;		/* enable outdoor use */

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
	int error = 0;
	u_int8_t csz;
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

	ah = _ath_hal_attach(devid, sc, 0, (void *) dev->mem_start, &status);
	if (ah == NULL) {
		printk(KERN_ERR "%s: unable to attach hardware; HAL status %u\n",
			dev->name, status);
		error = ENXIO;
		goto bad;
	}
	if (ah->ah_abi != HAL_ABI_VERSION) {
		printk(KERN_ERR "%s: HAL ABI msmatch; "
			"driver expects 0x%x, HAL reports 0x%x\n",
			dev->name, HAL_ABI_VERSION, ah->ah_abi);
		error = ENXIO;		/* XXX */
		goto bad;
	}
	sc->sc_ah = ah;

	/*
	 * Collect the channel list using the default country
	 * code and including outdoor channels.  The 802.11 layer
	 * is resposible for filtering this list to a set of
	 * channels that it considers ok to use.
	 */
	error = ath_getchannels(dev, ath_countrycode, ath_outdoor);
	if (error != 0)
		goto bad;
	ath_regdomain = ath_hal_getregdomain(ah);	/* for users */

	/*
	 * Setup rate tables for all potential media types.
	 */
	ath_rate_setup(dev, IEEE80211_MODE_11A);
	ath_rate_setup(dev, IEEE80211_MODE_11B);
	ath_rate_setup(dev, IEEE80211_MODE_11G);
	ath_rate_setup(dev, IEEE80211_MODE_TURBO);

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
	dev->stop = ath_stop;
	dev->hard_start_xmit = ath_hardstart;
	dev->tx_timeout = ath_tx_timeout;
	dev->watchdog_timeo = 5 * HZ;			/* XXX */
	dev->set_multicast_list = ath_mode_init;
	dev->get_stats = ath_getstats;
 	dev->change_mtu = &ath_change_mtu;
	dev->tx_queue_len = ATH_TXBUF-1;		/* 1 for mgmt frame */

	ic->ic_mgtstart = ath_mgtstart;
	ic->ic_init = ath_init;
	ic->ic_reset = ath_reset;

	ic->ic_newstate = ath_newstate;
	/* XXX not right but it's not used anywhere important */
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_caps = IEEE80211_C_WEP | IEEE80211_C_IBSS | IEEE80211_C_HOSTAP | IEEE80211_C_MONITOR | IEEE80211_C_SHPREAMBLE;
	ic->ic_flags |= IEEE80211_F_DATAPAD;

	ic->ic_node_privlen = sizeof(struct ath_nodestat);
	ic->ic_node_free = ath_node_free;
	ic->ic_bss.ni_private = &sc->sc_bss_stat;

	/* get mac address from hardware */
	ath_hal_getmac(ah, dev->dev_addr);

	/* call MI attach routine. */
	ieee80211_ifattach(dev);
	ieee80211_media_init(dev, ath_media_change, ieee80211_media_status);

	/* don't accept xmit's until we are associated */
	netif_stop_queue(dev);

	printk("%s: 802.11 address: %s\n",
		dev->name, ether_sprintf(dev->dev_addr));
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
	ath_stop(dev);
	sc->sc_invalid = 1;
	ath_desc_free(sc);
	ath_hal_detach(sc->sc_ah);
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

static int      
ath_change_mtu(struct net_device *dev, int new_mtu) 
{
	if (new_mtu > ATH_MAX_MTU || new_mtu <= ATH_MIN_MTU)
		return -EINVAL;
 	DPRINTF(("ath_change_mtu: %d\n", new_mtu));
 	dev->mtu = new_mtu;
 	ath_reset(dev);

 	return 0;
}

/*
 * Interrupt handler.  All the actual processing is
 * deferred to tasklets.
 */
irqreturn_t
ath_intr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	HAL_INT status;
	int needmark;

	if (sc->sc_invalid) {
		/*
		 * The hardware is gone, or HAL isn't yet initialized.
		 * Don't touch anything.
		 */
		return IRQ_NONE;
	}
	if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
		DPRINTF(("ath_intr: flags 0x%x\n", dev->flags));
		ath_hal_getisr(ah, &status);	/* clear ISR */
		ath_hal_intrset(ah, 0);		/* disable further intr's */
		return IRQ_NONE;
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
	status &= sc->sc_imask;			/* discard unasked for bits */
	if (status & HAL_INT_FATAL) {
		sc->sc_stats.ast_hardware++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		needmark |= queue_task(&sc->sc_fataltq, &tq_immediate);
	} else if (status & HAL_INT_RXORN) {
		sc->sc_stats.ast_rxorn++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
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
	return IRQ_HANDLED;
}

static void
ath_fatal_tasklet(void *data)
{
	struct net_device *dev = data;

	printk("%s: hardware error; resetting\n", dev->name);
	ath_reset(dev);
}

static void
ath_rxorn_tasklet(void *data)
{
	struct net_device *dev = data;

	printk("%s: rx FIFO overrun; resetting\n", dev->name);
	ath_reset(dev);
}

static void
ath_bmiss_tasklet(void *data)
{
	struct net_device *dev = data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
			
	DPRINTF(("ath_bmiss_tasklet\n"));
	KASSERT(ic->ic_opmode == IEEE80211_M_STA,
		("unexpect operating mode %u", ic->ic_opmode));
	if (ic->ic_state == IEEE80211_S_RUN)
		ieee80211_new_state(dev, IEEE80211_S_ASSOC, 0);
}

static u_int
ath_chan2flags(struct ieee80211com *ic, struct ieee80211channel *chan)
{
	static const u_int modeflags[] = {
		0,			/* IEEE80211_MODE_AUTO */
		CHANNEL_A,		/* IEEE80211_MODE_11A */
		CHANNEL_B,		/* IEEE80211_MODE_11B */
		CHANNEL_PUREG,		/* IEEE80211_MODE_11G */
		CHANNEL_T		/* IEEE80211_MODE_TURBO */
	};
	return modeflags[ieee80211_chan2mode(ic, chan)];
}

static int
ath_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	enum ieee80211_phymode mode;
	struct ath_hal *ah = sc->sc_ah;
	HAL_STATUS status;
	HAL_CHANNEL hchan;

	DPRINTF(("ath_init: mode=%d\n",ic->ic_opmode));
	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop(dev);
	
	/*
	 * Resize receive skb's if changing to or from monitor mode
	 */
	if ((dev->type == ARPHRD_ETHER && ic->ic_opmode == IEEE80211_M_MONITOR) ||
	    (dev->type == ARPHRD_IEEE80211_PRISM && ic->ic_opmode != IEEE80211_M_MONITOR)) {
		struct ath_buf *bf;
		TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
			if (bf->bf_skb != NULL) {
				pci_unmap_single(sc->sc_pdev,
					bf->bf_skbaddr, sc->sc_rxbufsize,
					PCI_DMA_FROMDEVICE);
				dev_kfree_skb(bf->bf_skb);
				bf->bf_skb = NULL;
			}
	}

	/*
	 * Change our interface type if we are in monitor mode.
	 */
	dev->type = (ic->ic_opmode == IEEE80211_M_MONITOR) ?
		ARPHRD_IEEE80211_PRISM : ARPHRD_ETHER;

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	hchan.channel = ic->ic_ibss_chan->ic_freq;
	hchan.channelFlags = ath_chan2flags(ic, ic->ic_ibss_chan);
	if (!ath_hal_reset(ah, ic->ic_opmode, &hchan, AH_FALSE, &status)) {
		printk("%s: unable to reset hardware; hal status %u\n",
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
	sc->sc_imask = HAL_INT_RX | HAL_INT_TX
		  | HAL_INT_RXEOL | HAL_INT_RXORN
		  | HAL_INT_FATAL | HAL_INT_GLOBAL;
	ath_hal_intrset(ah, sc->sc_imask);

	dev->flags |= IFF_RUNNING;
	ic->ic_state = IEEE80211_S_INIT;

	/*
	 * The hardware should be ready to go now so it's safe
	 * to kick the 802.11 state machine as it's likely to
	 * immediately call back to us to send mgmt frames.
	 */
	ni = &ic->ic_bss;
	ni->ni_chan = ic->ic_ibss_chan;
	mode = ieee80211_chan2mode(ic, ni->ni_chan);
	if (mode != sc->sc_curmode)
		ath_setcurmode(sc, mode);
	if (ic->ic_opmode != IEEE80211_M_MONITOR)
		ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
	else
		ieee80211_new_state(dev, IEEE80211_S_RUN, -1);
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
 * Reset the hardware w/o losing operational state.  This is
 * basically a more efficient way of doing ath_stop, ath_init,
 * followed by state transitions to the current 802.11
 * operational state.  Used to recover from errors rx overrun
 * and to reset the hardware when rf gain settings must be reset.
 */
static void
ath_reset(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211channel *c;
	HAL_STATUS status;
	HAL_CHANNEL hchan;

	/*
	 * Convert to a HAL channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	c = ic->ic_ibss_chan;
	hchan.channel = c->ic_freq;
	hchan.channelFlags = ath_chan2flags(ic, c);

	ath_hal_intrset(ah, 0);		/* disable interrupts */
	ath_draintxq(sc);		/* stop xmit side */
	ath_stoprecv(sc);		/* stop recv side */
	/* NB: indicate channel change so we do a full reset */
	if (!ath_hal_reset(ah, ic->ic_opmode, &hchan, AH_TRUE, &status))
		printk("%s: %s: unable to reset hardware; hal status %u\n",
			dev->name, __func__, status);
	ath_hal_intrset(ah, sc->sc_imask);
	if (ath_startrecv(dev) != 0)	/* restart recv */
		printk("%s: %s: unable to start recv logic\n",
			dev->name, __func__);
	if (ic->ic_state == IEEE80211_S_RUN) {
		ath_beacon_config(sc);	/* restart beacons */
		netif_wake_queue(dev);	/* restart xmit */
	}
}

/*
 * XXX System may not be
 * configured to leave enough headroom for us to push the
 * 802.11 frame.  In that case fallback on reallocating
 * the frame with enough space.  Alternatively we can carry
 * the frame separately and use s/g support in the hardware.
 */
int ieee80211_skbhdr_adjust(struct sk_buff *skb, struct net_device *dev)
{
	struct ieee80211com *ic = (void*)dev->priv;
	int len = sizeof(struct ieee80211_qosframe) + sizeof(struct llc) +
		IEEE80211_ADDR_LEN - sizeof(struct ether_header);
	if (ic->ic_flags & IEEE80211_F_WEPON)
		len += IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;

	if ((skb_headroom(skb) < len) &&
	    pskb_expand_head(skb, len - skb_headroom(skb), 0, GFP_ATOMIC)) {
		dev_kfree_skb(skb);
		return -ENOMEM;
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
		DPRINTF(("%s: discard, invalid %d flags %x\n", __func__,
			sc->sc_invalid, dev->flags));
		sc->sc_stats.ast_tx_invalid++;
		return -ENETDOWN;
	}
	
	error = ieee80211_skbhdr_adjust(skb, dev);
	if (error != 0)
		return error;

	/*
	 * No data frames go out unless we're associated; this
	 * should not happen as the 802.11 layer does not enable
	 * the xmit queue until we enter the RUN state.
	 */
	if (ic->ic_state != IEEE80211_S_RUN) {
		DPRINTF(("%s: discard, state %u\n", __func__, ic->ic_state));
		sc->sc_stats.ast_tx_discard++;
		/*
		 * Someone outside the driver started the queue;
		 * turn it off until we asociate (yech).
		 */
		netif_stop_queue(dev);
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
		DPRINTF(("%s: stop queue\n", __func__));
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(dev);
	}
	spin_unlock_bh(&sc->sc_txbuflock);
	if (bf == NULL) {		/* NB: should not happen */
		printk("%s: discard, no xmit buf\n", __func__);
		sc->sc_stats.ast_tx_nobuf++;
		goto bad;
	}

	/*
	 * Encapsulate the packet for transmission.
	 */
	skb = ieee80211_encap(dev, skb);
	if (skb == NULL) {
		DPRINTF(("%s: discard, encapsulation failure\n", __func__));
		sc->sc_stats.ast_tx_encap++;
		goto bad;
	}
	wh = (struct ieee80211_frame *) skb->data;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
	/*
	 * Locate node state.  When operating
	 * in station mode we always use ic_bss.
	 */
	if (ic->ic_opmode != IEEE80211_M_STA) {
		ni = ieee80211_find_node(ic, wh->i_addr1);
		if (ni == NULL && !IEEE80211_IS_MULTICAST(wh->i_addr1)) {
			DPRINTF(("%s: discard, no destination state\n",
				__func__));
			sc->sc_stats.ast_tx_nonode++;
			goto bad;
		}
		if (ni == NULL)
			ni = ieee80211_ref_node(&ic->ic_bss);
	} else
		ni = ieee80211_ref_node(&ic->ic_bss);
	if (IFF_DUMPPKTS(ic))
		ieee80211_dump_pkt(skb->data, skb->len,
		    ni->ni_rates.rs_rates[ni->ni_txrate] & IEEE80211_RATE_VAL, -1);

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
	return 0;	/* NB: return !0 only in a ``hard error condition'' */
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
	if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_PROBE_RESP) {
		/* fill time stamp */
		u_int64_t tsf;
		u_int32_t *tstamp;

		tsf = ath_hal_gettsf64(ah);
		/* XXX: adjust 100us delay to xmit */
		tsf += 100;
		tstamp = (u_int32_t *)&wh[1];
		tstamp[0] = cpu_to_le32(tsf & 0xffffffff);
		tstamp[1] = cpu_to_le32(tsf >> 32);
	}
	/*
	 * Locate node state.  When operating
	 * in station mode we always use ic_bss.
	 */
	if (ic->ic_opmode != IEEE80211_M_STA) {
		ni = ieee80211_find_node(ic, wh->i_addr1);
		if (ni == NULL)
			ni = ieee80211_ref_node(&ic->ic_bss);
	} else
		ni = ieee80211_ref_node(&ic->ic_bss);

	if (IFF_DUMPPKTS(ic))
		ieee80211_dump_pkt(skb->data, skb->len,
		    ni->ni_rates.rs_rates[ni->ni_txrate] & IEEE80211_RATE_VAL, -1);

	error = ath_tx_start(dev, ni, bf, skb);
	ieee80211_unref_node(&ni);
	if (error == 0) {
		sc->sc_stats.ast_tx_mgmt++;
		/*
		 * XXX probably shouldn't be mucking with this
		 * stuff, but we need to timeout the xmit.
		 */
		dev->trans_start = jiffies;
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

static int
ath_media_change(struct net_device *dev)
{
	int error;

	error = ieee80211_media_change(dev);
	if (error == ENETRESET) {
		if ((dev->flags & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))
			error = ath_init(dev);
		else
			error = 0;
	}
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
	rfilt = (ath_hal_getrxfilter(ah) & HAL_RX_FILTER_PHYERR)
	      | HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST | HAL_RX_FILTER_MCAST;
	if (ic->ic_opmode != IEEE80211_M_STA)
		rfilt |= HAL_RX_FILTER_PROBEREQ;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP && (dev->flags & IFF_PROMISC))
		rfilt |= HAL_RX_FILTER_PROM;
	if (ic->ic_state == IEEE80211_S_SCAN ||
	    ic->ic_opmode == IEEE80211_M_STA)
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
			skb_reserve(skb, align - off);
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
	struct ieee80211_rateset *rs;
	const HAL_RATE_TABLE *rt;

	bf = sc->sc_bcbuf;
	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
	}
	rs = &ni->ni_rates;
	pktlen = + 8
		  + sizeof(u_int16_t)
		  + sizeof(u_int16_t)
		  + 2 + ni->ni_esslen
		  + 2 + rs->rs_nrates
		  + 3
		  + 6;
	if (rs->rs_nrates > IEEE80211_RATE_SIZE)
		pktlen += 2;
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
	 *	[tlv] extended supported rates
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
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = ni->ni_esslen;
	memcpy(frm, ni->ni_essid, ni->ni_esslen);
	frm += ni->ni_esslen;
	frm = ieee80211_add_rates(frm, rs);
	*frm++ = IEEE80211_ELEMID_DSPARMS;
	*frm++ = 1;
	*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
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
	frm = ieee80211_add_xrates(frm, &ni->ni_rates);
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
	/*
	 * Calculate rate code.
	 * XXX everything at min xmit rate
	 */
	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));
	if (ic->ic_flags & IEEE80211_F_SHPREAMBLE)
		rate = rt->info[0].rateCode | rt->info[0].shortPreamble;
	else
		rate = rt->info[0].rateCode;
	ath_hal_setuptxdesc(ah, ds
		, skb->len + IEEE80211_CRC_LEN	/* frame length */
		, sizeof(struct ieee80211_frame)/* header length */
		, HAL_PKT_TYPE_BEACON		/* Atheros packet type */
		, 0x20				/* txpower XXX */
		, rate, 1			/* series 0 rate/tries */
		, HAL_TXKEYIX_INVALID		/* no encryption */
		, 0				/* antenna mode */
		, HAL_TXDESC_NOACK		/* no ack for beacons */
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
	if (ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_MONITOR ||
	    bf == NULL || bf->bf_skb == NULL) {
		DPRINTF(("%s: ic_flags=%x bf=%p bf_m=%p\n",
			 __func__, ic->ic_flags, bf, bf ? bf->bf_skb : NULL));
		return;
	}
	/* update beacon to reflect PS poll state */
	if (!ath_hal_stoptxdma(ah, sc->sc_bhalq)) {
		DPRINTF(("%s: beacon queue %u did not stop?",
			__func__, sc->sc_bhalq));
		return;			/* busy, XXX is this right? */
	}
	pci_dma_sync_single(sc->sc_pdev,
		bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);

	ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
	ath_hal_txstart(ah, sc->sc_bhalq);
	DPRINTF2(("%s: TXDP%u = %p (%p)\n", __func__,
		sc->sc_bhalq, (caddr_t)bf->bf_daddr, bf->bf_desc));
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
static void
ath_beacon_config(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = &ic->ic_bss;
	u_int32_t nexttbtt;
	
	nexttbtt = (LE_READ_4(ni->ni_tstamp + 4) << 22) |
	    (LE_READ_4(ni->ni_tstamp) >> 10);
	DPRINTF(("%s: nexttbtt=%u\n", __func__, nexttbtt));
	nexttbtt += ni->ni_intval;
	if (ic->ic_opmode == IEEE80211_M_STA) {
		HAL_BEACON_STATE bs;
		u_int32_t bmisstime;

		/* NB: no PCF support right now */
		memset(&bs, 0, sizeof(bs));
		bs.bs_intval = ni->ni_intval;
		bs.bs_nexttbtt = nexttbtt;
		bs.bs_dtimperiod = bs.bs_intval;
		bs.bs_nextdtim = nexttbtt;
		/*
		 * Calculate the number of consecutive beacons to miss
		 * before taking a BMISS interrupt.  The configuration
		 * is specified in ms, so we need to convert that to
		 * TU's and then calculate based on the beacon interval.
		 * Note that we clamp the result to at most 10 beacons.
		 */
		bmisstime = (ic->ic_bmisstimeout * 1000) / 1024;
		bs.bs_bmissthreshold = howmany(bmisstime,ni->ni_intval);
		if (bs.bs_bmissthreshold > 10)
			bs.bs_bmissthreshold = 10;
		else if (bs.bs_bmissthreshold <= 0)
			bs.bs_bmissthreshold = 1;

		/*
		 * Calculate sleep duration.  The configuration is
		 * given in ms.  We insure a multiple of the beacon
		 * period is used.  Also, if the sleep duration is
		 * greater than the DTIM period then it makes senses
		 * to make it a multiple of that.
		 *
		 * XXX fixed at 100ms
		 */
		bs.bs_sleepduration =
			roundup((100 * 1000) / 1024, bs.bs_intval);
		if (bs.bs_sleepduration > bs.bs_dtimperiod)
			bs.bs_sleepduration = roundup(bs.bs_sleepduration, bs.bs_dtimperiod);

		DPRINTF(("%s: intval %u nexttbtt %u dtim %u nextdtim %u bmiss %u sleep %u\n"
			, __func__
			, bs.bs_intval
			, bs.bs_nexttbtt
			, bs.bs_dtimperiod
			, bs.bs_nextdtim
			, bs.bs_bmissthreshold
			, bs.bs_sleepduration
		));
		ath_hal_intrset(ah, 0);
		/*
		 * Reset our tsf so the hardware will update the
		 * tsf register to reflect timestamps found in
		 * received beacons.
		 */
		ath_hal_resettsf(ah);
		ath_hal_beacontimers(ah, &bs, 0/*XXX*/, 0, 0);
		sc->sc_imask |= HAL_INT_BMISS;
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
		DPRINTF(("%s: intval %u nexttbtt %u\n",
			__func__, ni->ni_intval, nexttbtt));
		ath_hal_intrset(ah, 0);
		ath_hal_beaconinit(ah, ic->ic_opmode,
			nexttbtt, ni->ni_intval);
		if (ic->ic_opmode != IEEE80211_M_MONITOR)
			sc->sc_imask |= HAL_INT_SWBA;	/* beacon prepare */
		ath_hal_intrset(ah, sc->sc_imask);
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

	/*
	 * NB: TX queues have already been freed in ath_draintxq(),
	 * which must be called before calling this function.
	 */

	/* Free all pre-allocated RX skb */
	TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
		if (bf->bf_skb != NULL) {
			pci_unmap_single(sc->sc_pdev,
				bf->bf_skbaddr, sc->sc_rxbufsize,
				PCI_DMA_FROMDEVICE);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
		}

	/* If beacon skb has not been freed yet, do it now */
	if (sc->sc_bcbuf != NULL) {
		bf = sc->sc_bcbuf;
		if (bf->bf_skb != NULL) {
			pci_unmap_single(sc->sc_pdev, bf->bf_skbaddr,
					 bf->bf_skb->len, PCI_DMA_TODEVICE);
		}
		sc->sc_bcbuf = NULL;
	}

	/* Free memory associated with all descriptors */
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

/*
 * For packet capture, define the same physical layer packet header 
 * structure as used in the wlan-ng driver 
 */
enum {
	DIDmsg_lnxind_wlansniffrm		= 0x00000044,
	DIDmsg_lnxind_wlansniffrm_hosttime	= 0x00010044,
	DIDmsg_lnxind_wlansniffrm_mactime	= 0x00020044,
	DIDmsg_lnxind_wlansniffrm_channel	= 0x00030044,
	DIDmsg_lnxind_wlansniffrm_rssi		= 0x00040044,
	DIDmsg_lnxind_wlansniffrm_sq		= 0x00050044,
	DIDmsg_lnxind_wlansniffrm_signal	= 0x00060044,
	DIDmsg_lnxind_wlansniffrm_noise		= 0x00070044,
	DIDmsg_lnxind_wlansniffrm_rate		= 0x00080044,
	DIDmsg_lnxind_wlansniffrm_istx		= 0x00090044,
	DIDmsg_lnxind_wlansniffrm_frmlen	= 0x000A0044
};
enum {
	P80211ENUM_msgitem_status_no_value	= 0x00
};
enum {
	P80211ENUM_truth_false			= 0x00
};

typedef struct {
	u_int32_t did;
	u_int16_t status;
	u_int16_t len;
	u_int32_t data;
} p80211item_uint32_t;

typedef struct {
	u_int32_t msgcode;
	u_int32_t msglen;
#define WLAN_DEVNAMELEN_MAX 16
	u_int8_t devname[WLAN_DEVNAMELEN_MAX];
	p80211item_uint32_t hosttime;
	p80211item_uint32_t mactime;
	p80211item_uint32_t channel;
	p80211item_uint32_t rssi;
	p80211item_uint32_t sq;
	p80211item_uint32_t signal;
	p80211item_uint32_t noise;
	p80211item_uint32_t rate;
	p80211item_uint32_t istx;
	p80211item_uint32_t frmlen;
} wlan_ng_prism2_header;

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;
	struct ath_desc *ds;

	skb = bf->bf_skb;
	if (skb == NULL) {
 		if (sc->sc_ic.ic_opmode == IEEE80211_M_MONITOR) {
 			u_int off;
 			/*
 			 * Allocate buffer for monitor mode with space for the
			 * wlan-ng style physical layer header at the start.
 			 */
 			skb = dev_alloc_skb(sc->sc_rxbufsize +
					    sizeof(wlan_ng_prism2_header) +
					    sc->sc_cachelsz - 1);
 			if (skb == NULL) {
 				DPRINTF(("%s: skbuff alloc of size %u failed\n",
					__func__,
					sc->sc_rxbufsize
					+ sizeof(wlan_ng_prism2_header)
					+ sc->sc_cachelsz -1));
 				sc->sc_stats.ast_rx_nobuf++;
 				return ENOMEM;
 			}
 			/*
			 * Reserve space for the Prism header.
 			 */
 			skb_reserve(skb, sizeof(wlan_ng_prism2_header));
			/*
 			 * Align to cache line.
			 */
 			off = ((unsigned long) skb->data) % sc->sc_cachelsz;
 			if (off != 0)
 				skb_reserve(skb, sc->sc_cachelsz - off);
		} else {
			/*
			 * Cache-line-align.  This is important (for the
			 * 5210 at least) as not doing so causes bogus data
			 * in rx'd frames.
			 */
			skb = ath_alloc_skb(sc->sc_rxbufsize, sc->sc_cachelsz);
			if (skb == NULL) {
				DPRINTF(("%s: skbuff alloc of size %u failed\n",
					__func__, sc->sc_rxbufsize));
				sc->sc_stats.ast_rx_nobuf++;
				return ENOMEM;
			}
		}
		skb->dev = &sc->sc_ic.ic_dev;
		bf->bf_skb = skb;
		bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
			skb->data, sc->sc_rxbufsize, PCI_DMA_FROMDEVICE);
	}

	/*
	 * Setup descriptors.  For receive we always terminate
	 * the descriptor list with a self-linked entry so we'll
	 * not get overrun under high load (as can happen with a
	 * 5212 when ANI processing enables PHY errors).
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
	ds = bf->bf_desc;
	ds->ds_link = bf->bf_daddr;		/* link to self */
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

/*
 * Add a prism2 header to a received frame and
 * dispatch it to capture tools like kismet.
 */
static void
ath_rx_capture(struct net_device *dev, struct ath_desc *ds, struct sk_buff *skb)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int len = ds->ds_rxstat.rs_datalen;
	int rssi = ds->ds_rxstat.rs_rssi;
	const HAL_RATE_TABLE *rt = sc->sc_currates;
	struct ieee80211_frame *wh;
	uint8_t qosframe[sizeof(struct ieee80211_qosframe) + IEEE80211_ADDR_LEN];
	int rate = (rt != NULL ?
	    (rt->info[rt->rateCodeToIndex[
	    	ds->ds_rxstat.rs_rate]].dot11Rate & IEEE80211_RATE_VAL) : 2);
 	wlan_ng_prism2_header *ph;
	int headersize, padbytes;

	skb->protocol = ETH_P_CONTROL;
	skb->pkt_type = PACKET_OTHERHOST;

	skb_put(skb, len);
	
	/* Check to see if we need to remove pad bytes */
	if (ic->ic_flags & IEEE80211_F_DATAPAD) {
	    wh = (struct ieee80211_frame *) skb->data;
	    if (((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) &&
		(wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK & IEEE80211_FC0_SUBTYPE_QOS)) {
		headersize = sizeof(struct ieee80211_qosframe);
		if ((wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_DSTODS) {
		    headersize += IEEE80211_ADDR_LEN;
		}
		padbytes = roundup(headersize,4) - headersize;
		if (padbytes > 0) {
		    memcpy(&qosframe[0],skb->data,headersize);
		    skb_pull(skb,padbytes);
		    memcpy(skb->data,&qosframe[0],headersize);
		}
	    }
	}

	ph = (wlan_ng_prism2_header *)
		skb_push(skb, sizeof(wlan_ng_prism2_header));
	memset(ph, 0, sizeof(wlan_ng_prism2_header));

	ph->msgcode = DIDmsg_lnxind_wlansniffrm;
	ph->msglen = sizeof(wlan_ng_prism2_header);
	strcpy(ph->devname, dev->name);

	ph->hosttime.did = DIDmsg_lnxind_wlansniffrm_hosttime;
	ph->hosttime.status = 0;
	ph->hosttime.len = 4;
	ph->hosttime.data = jiffies;

	/* Pass up tsf clock in mactime */
	ph->mactime.did = DIDmsg_lnxind_wlansniffrm_mactime;
//	ph->mactime.status = P80211ENUM_msgitem_status_no_value;
	ph->mactime.status = 0;
	ph->mactime.len = 4;
//	ph->mactime.data = 0;
	ph->mactime.data = ath_hal_gettsf32(sc->sc_ah);

	ph->istx.did = DIDmsg_lnxind_wlansniffrm_istx;
	ph->istx.status = 0;
	ph->istx.len = 4;
	ph->istx.data = P80211ENUM_truth_false;

	ph->frmlen.did = DIDmsg_lnxind_wlansniffrm_frmlen;
	ph->frmlen.status = 0;
	ph->frmlen.len = 4;
	ph->frmlen.data = len;

	ph->channel.did = DIDmsg_lnxind_wlansniffrm_channel;
	ph->channel.status = 0;
	ph->channel.len = 4;
	ph->channel.data = ieee80211_mhz2ieee(ic->ic_ibss_chan->ic_freq,0);

	ph->rssi.did = DIDmsg_lnxind_wlansniffrm_rssi;
	ph->rssi.status = P80211ENUM_msgitem_status_no_value;
	ph->rssi.len = 4;
	ph->rssi.data = 0;

	ph->signal.did = DIDmsg_lnxind_wlansniffrm_signal;
	ph->signal.status = 0;
	ph->signal.len = 4;
	ph->signal.data = rssi;

	ph->rate.did = DIDmsg_lnxind_wlansniffrm_rate;
	ph->rate.status = 0;
	ph->rate.len = 4;
	ph->rate.data = rate;

	skb->dev = dev;
	skb->mac.raw = skb->data ;
	skb->ip_summed = CHECKSUM_NONE;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = __constant_htons(0x0019);  /* ETH_P_80211_RAW */

	netif_rx(skb);
}

static void
ath_rx_tasklet(void *data)
{
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_desc + \
		((_pa) - (_sc)->sc_desc_daddr)))
	struct net_device *dev = data;
	struct ath_buf *bf;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
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
		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at the end */
			break;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {		/* XXX ??? can this happen */
			printk("ath_rx_tasklet: no skbuff\n");
			continue;
		}
		/* XXX sync descriptor memory */
		/*
		 * Must provide the virtual address of the current
		 * descriptor, the physical address, and the virtual
		 * address of the next descriptor in the h/w chain.
		 * This allows the HAL to look ahead to see if the
		 * hardware is done with a descriptor by checking the
		 * done bit in the following descriptor and the address
		 * of the current descriptor the DMA engine is working
		 * on.  All this is necessary because of our use of
		 * a self-linked list to avoid rx overruns.
		 */
		status = ath_hal_rxprocdesc(ah, ds,
				bf->bf_daddr, PA2DESC(sc, ds->ds_link));
#ifdef AR_DEBUG
		if (ath_debug > 1)
			ath_printrxbuf(bf, status == HAL_OK); 
#endif
		if (status == HAL_EINPROGRESS)
			break;
		TAILQ_REMOVE(&sc->sc_rxbuf, bf, bf_list);

		if (ds->ds_rxstat.rs_more) {
			/*
			 * Frame spans multiple descriptors; this
			 * cannot happen yet as we don't support
			 * jumbograms.  If not in monitor mode,
			 *  discard the frame.
			 */

			/* enable this if you want to see error frames in Monitor mode */
#ifdef ERROR_FRAMES
			if (ic->ic_opmode != IEEE80211_M_MONITOR) {
				/* XXX statistic */
				goto rx_next;
			}
#endif
			/* fall thru for monitor mode handling... */
		} else if (ds->ds_rxstat.rs_status != 0) {
			if (ds->ds_rxstat.rs_status & HAL_RXERR_CRC) {
				sc->sc_stats.ast_rx_crcerr++;
				/*
				 * Record the rssi for crc errors; it
				 * should still be valid.
				 */
				sc->sc_stats.ast_rx_rssidelta =
					ds->ds_rxstat.rs_rssi -
					sc->sc_stats.ast_rx_rssi;
				sc->sc_stats.ast_rx_rssi =
					ds->ds_rxstat.rs_rssi;
			}
			if (ds->ds_rxstat.rs_status & HAL_RXERR_FIFO)
				sc->sc_stats.ast_rx_fifoerr++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_DECRYPT)
				sc->sc_stats.ast_rx_badcrypt++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_PHY) {
				sc->sc_stats.ast_rx_phyerr++;
				phyerr = ds->ds_rxstat.rs_phyerr & 0x1f;
				sc->sc_stats.ast_rx_phy[phyerr]++;
			}

			/*
			 * reject error frames, we normally don't want
			 * to see them in monitor mode.
			 */
			if ((ds->ds_rxstat.rs_status & HAL_RXERR_DECRYPT ) ||
			    (ds->ds_rxstat.rs_status & HAL_RXERR_PHY))
			    goto rx_next;

			/*
			 * In monitor mode, allow through packets that
			 * cannot be decrypted
			 */
			if ((ds->ds_rxstat.rs_status & ~HAL_RXERR_DECRYPT) ||
			    sc->sc_ic.ic_opmode != IEEE80211_M_MONITOR)
				goto rx_next;
		}

		sc->sc_stats.ast_rx_rssidelta =
			ds->ds_rxstat.rs_rssi - sc->sc_stats.ast_rx_rssi;
		sc->sc_stats.ast_rx_rssi = ds->ds_rxstat.rs_rssi;

		len = ds->ds_rxstat.rs_datalen;
		if (len < sizeof(struct ieee80211_frame)) {
			DPRINTF(("%s: ath_rx_tasklet: short packet %d\n",
			    dev->name, len));
			sc->sc_stats.ast_rx_tooshort++;
			goto rx_next;
		}

		pci_dma_sync_single(sc->sc_pdev,
			bf->bf_skbaddr, len, PCI_DMA_FROMDEVICE);

		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
			/*
			 * Monitor mode: clean the skbuff, fabricate
			 * the Prism header existing tools expect,
			 * and dispatch.
			 */
			pci_unmap_single(sc->sc_pdev, bf->bf_skbaddr,
					sc->sc_rxbufsize, PCI_DMA_FROMDEVICE);
			bf->bf_skb = NULL;

			ath_rx_capture(dev, ds, skb);
			goto rx_done;
		}

		/*
		 * Normal receive.
		 */
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
		pci_unmap_single(sc->sc_pdev, bf->bf_skbaddr,
			sc->sc_rxbufsize, PCI_DMA_FROMDEVICE);
		bf->bf_skb = NULL;
		skb_put(skb, len);
		skb->protocol = ETH_P_CONTROL;		/* XXX */
		if (IFF_DUMPPKTS(&sc->sc_ic)) {
			const HAL_RATE_TABLE *rt = sc->sc_currates;
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

		ieee80211_input(dev, skb,
				ds->ds_rxstat.rs_rssi,
				ds->ds_rxstat.rs_tstamp,
				ds->ds_rxstat.rs_antenna);
rx_done:
		stats->rx_packets++;
		stats->rx_bytes += len;
rx_next:
		TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
	} while (ath_rxbuf_init(sc, bf) == 0);

	ath_hal_rxmonitor(ah);			/* rx signal state monitoring */
	ath_hal_rxena(ah);			/* in case of RXEOL */
#undef PA2DESC
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
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct net_device_stats *stats = &ic->ic_stats;
	int i, iswep, hdrlen, pktlen;
	u_int8_t rix, cix, txrate, ctsrate;
	struct ath_desc *ds;
	struct ieee80211_frame *wh;
	u_int32_t iv;
	u_int8_t *ivp;
	u_int8_t hdrbuf[sizeof(struct ieee80211_frame) +
	    IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN];
	u_int subtype, flags, ctsduration, antenna;
	HAL_PKT_TYPE atype;
	const HAL_RATE_TABLE *rt;
	HAL_BOOL shortPreamble;
	struct ath_nodestat *st;

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
                iv = ic->ic_iv;

		/*
		 * Skip 'bad' IVs from Fluhrer/Mantin/Shamir:
		 * (B, 255, N) with 3 <= B < 8
		 */
		if (iv >= 0x03ff00 &&
		    (iv & 0xf8ff00) == 0x00ff00)
			iv += 0x000100;
		ic->ic_iv = iv + 1;

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
		pktlen += IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN+
			IEEE80211_WEP_CRCLEN;

		/* Packet header has moved, reset our local pointer */
		wh = (struct ieee80211_frame *) skb->data;
	}

	pktlen += IEEE80211_CRC_LEN;

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
	bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
		skb->data, pktlen, PCI_DMA_TODEVICE);
	DPRINTF2(("ath_tx_start: skb %p [data %p len %u] skbaddr %x\n",
		skb, skb->data, skb->len, bf->bf_skbaddr));
	bf->bf_skb = skb;
	bf->bf_node = ni;

	/* setup descriptors */
	ds = bf->bf_desc;
	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

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
		rix = sc->sc_rixmap[ni->ni_rates.rs_rates[ni->ni_txrate] &
				IEEE80211_RATE_VAL];
		if (rix == 0xff) {
			printk("%s: %s: bogus xmit rate 0x%x\n",
				dev->name, __func__,
				ni->ni_rates.rs_rates[ni->ni_txrate]);
			sc->sc_stats.ast_tx_badrate++;
			return -EIO;
		}
		break;
	}
	/*
	 * NB: the 802.11 layer marks whether or not we should
	 * use short preamble based on the current mode and
	 * negotiated parameters.
	 */
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)) {
		txrate = rt->info[rix].rateCode | rt->info[rix].shortPreamble;
		shortPreamble = AH_TRUE;
		sc->sc_stats.ast_tx_shortpre++;
	} else {
		txrate = rt->info[rix].rateCode;
		shortPreamble = AH_FALSE;
	}

	/*
	 * Calculate miscellaneous flags.
	 */
	flags = HAL_TXDESC_CLRDMASK;		/* XXX needed for wep errors */
	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		flags |= HAL_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
	} else if (pktlen > ic->ic_rtsthreshold) {
		flags |= HAL_TXDESC_RTSENA;	/* RTS based on frame length */
		sc->sc_stats.ast_tx_rts++;
	}

	/*
	 * Calculate duration.  This logically belongs in the 802.11
	 * layer but it lacks sufficient information to calculate it.
	 */
	if ((flags & HAL_TXDESC_NOACK) == 0 &&
	    (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL) {
		u_int16_t dur;
		/*
		 * XXX not right with fragmentation.
		 */
		dur = ath_hal_computetxtime(ah, rt, IEEE80211_ACK_SIZE,
				rix, shortPreamble);
		*(u_int16_t *)wh->i_dur = cpu_to_le16(dur);
	}

	/*
	 * Calculate RTS/CTS rate and duration if needed.
	 */
	ctsduration = 0;
	if (flags & (HAL_TXDESC_RTSENA|HAL_TXDESC_CTSENA)) {
		/*
		 * CTS transmit rate is derived from the transmit rate
		 * by looking in the h/w rate table.  We must also factor
		 * in whether or not a short preamble is to be used.
		 */
		cix = rt->info[rix].controlRate;
		ctsrate = rt->info[cix].rateCode;
		if (shortPreamble)
			ctsrate |= rt->info[cix].shortPreamble;
		/*
		 * Compute the transmit duration based on the size
		 * of an ACK frame.  We call into the HAL to do the
		 * computation since it depends on the characteristics
		 * of the actual PHY being used.
		 */
		if (flags & HAL_TXDESC_RTSENA) {	/* SIFS + CTS */
			ctsduration += ath_hal_computetxtime(ah,
				rt, IEEE80211_ACK_SIZE, cix, shortPreamble);
		}
		/* SIFS + data */
		ctsduration += ath_hal_computetxtime(ah,
			rt, pktlen, rix, shortPreamble);
		if ((flags & HAL_TXDESC_NOACK) == 0) {	/* SIFS + ACK */
			ctsduration += ath_hal_computetxtime(ah,
				rt, IEEE80211_ACK_SIZE, cix, shortPreamble);
		}
	} else
		ctsrate = 0;

	/*
	 * For now use the antenna on which the last good
	 * frame was received on.  We assume this field is
	 * initialized to 0 which gives us ``auto'' or the
	 * ``default'' antenna.
	 */
	st = ni->ni_private;
	if (st->st_tx_antenna)
		antenna = st->st_tx_antenna;
	else
		antenna = ni->ni_recv_hist[ni->ni_hist_cur].hi_rantenna;

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	/* XXX check return value? */
	ath_hal_setuptxdesc(ah, ds
		, pktlen		/* packet length */
		, hdrlen		/* header length */
		, atype			/* Atheros packet type */
		, 60			/* txpower XXX */
		, txrate, 1+10		/* series 0 rate/tries */
		, iswep ? sc->sc_ic.ic_wep_txkey : HAL_TXKEYIX_INVALID
		, antenna		/* antenna mode */
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

	dev->trans_start = jiffies;

	return 0;
}

static void
ath_tx_tasklet(void *data)
{
	struct net_device *dev = data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct ieee80211_node *ni;
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

		ni = bf->bf_node;
		if (ni != NULL) {
			st = ni->ni_private;
			if (ds->ds_txstat.ts_status == 0) {
				st->st_tx_ok++;
				st->st_tx_antenna = ds->ds_txstat.ts_antenna;
				sc->sc_stats.ast_tx_rssidelta =
					ds->ds_txstat.ts_rssi -
					sc->sc_stats.ast_tx_rssi;
				sc->sc_stats.ast_tx_rssi =
					ds->ds_txstat.ts_rssi;
			} else {
				st->st_tx_err++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_XRETRY)
					sc->sc_stats.ast_tx_xretries++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_FIFO)
					sc->sc_stats.ast_tx_fifoerr++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_FILT)
					sc->sc_stats.ast_tx_filtered++;
				st->st_tx_antenna = 0;	/* invalidate */
			}
			sr = ds->ds_txstat.ts_shortretry;
			lr = ds->ds_txstat.ts_longretry;
			sc->sc_stats.ast_tx_shortretry += sr;
			sc->sc_stats.ast_tx_longretry += lr;
			if (sr + lr)
				st->st_tx_retr++;
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
	/*
	 * Don't wakeup unless we're associated; this insures we don't
	 * signal the upper layer it's ok to start sending data frames.
	 */
	/* XXX use a low watermark to reduce wakeups */
	if (ic->ic_state == IEEE80211_S_RUN)
		netif_wake_queue(dev);
}

static void
ath_tx_timeout(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	sc->sc_stats.ast_watchdog++;
#ifdef AR_DEBUG
	if (ath_debug)
		ath_hal_dumpstate(sc->sc_ah);
#endif
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
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_desc + \
		((_pa) - (_sc)->sc_desc_daddr)))
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
			struct ath_desc *ds = bf->bf_desc;
			if (ath_hal_rxprocdesc(ah, ds, bf->bf_daddr,
			    PA2DESC(sc, ds->ds_link)) == HAL_OK)
				ath_printrxbuf(bf, 1);
		}
	}
#endif
	sc->sc_rxlink = NULL;		/* just in case */
#undef PA2DESC
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

	/*
	 * Cisco's VPN software requires that drivers be able to
	 * receive encapsulated frames that are larger than the MTU.
	 * Since we can't be sure how large a frame we'll get, setup
	 * to handle the larges on possible.  If you're not using the
	 * VPN driver and want to save memory, define ATH_ENFORCE_MTU
	 * and you'll allocate only what you really need.
	 */
#ifdef ATH_ENFORCE_MTU
	if (sc->sc_ic.ic_opmode == IEEE80211_M_MONITOR) {
		sc->sc_rxbufsize = roundup(IEEE80211_MAX_LEN, sc->sc_cachelsz);
	} else {
		sc->sc_rxbufsize = roundup(sizeof(struct ieee80211_frame) +
			dev->mtu + IEEE80211_CRC_LEN +
			(IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN +
			 IEEE80211_WEP_CRCLEN), sc->sc_cachelsz);
	}
#else
	sc->sc_rxbufsize = roundup(IEEE80211_MAX_LEN, sc->sc_cachelsz);
#endif
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

	DPRINTF(("%s: %u (%u MHz) -> %u (%u MHz)\n", __func__,
	    ieee80211_chan2ieee(ic, ic->ic_ibss_chan),
	    	ic->ic_ibss_chan->ic_freq,
	    ieee80211_chan2ieee(ic, chan), chan->ic_freq));
	if (chan != ic->ic_ibss_chan) {
		HAL_STATUS status;
		HAL_CHANNEL hchan;
		enum ieee80211_phymode mode;

		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		/*
		 * Convert to a HAL channel description with
		 * the flags constrained to reflect the current
		 * operating mode.
		 */
		hchan.channel = chan->ic_freq;
		hchan.channelFlags = ath_chan2flags(ic, chan);
		if (!ath_hal_reset(ah, ic->ic_opmode, &hchan, AH_TRUE, &status)) {
			printk("%s: %s: unable to reset channel %u (%uMhz)\n",
				dev->name, __func__,
				ieee80211_chan2ieee(ic, chan), chan->ic_freq);
			return EIO;
		}

		/*
		 * Re-enable rx framework.
		 */
		if (ath_startrecv(dev) != 0) {
			printk("%s: %s: unable to restart recv logic\n",
				dev->name, __func__);
			return EIO;
		}

		/*
		 * Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g.
		 */
		ic->ic_ibss_chan = chan;
		mode = ieee80211_chan2mode(ic, chan);
		if (mode != sc->sc_curmode)
			ath_setcurmode(sc, mode);

		/*
		 * Re-enable interrupts.
		 */
		ath_hal_intrset(ah, sc->sc_imask);
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
	HAL_CHANNEL hchan;

	sc->sc_stats.ast_per_cal++;

	/*
	 * Convert to a HAL channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	c = ic->ic_ibss_chan;
	hchan.channel = c->ic_freq;
	hchan.channelFlags = ath_chan2flags(ic, c);

	DPRINTF2(("%s: channel %u/%x\n", __func__, c->ic_freq, c->ic_flags));

	if (ath_hal_getrfgain(ah) == HAL_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		sc->sc_stats.ast_per_rfgain++;
		ath_reset(dev);
	}
	if (!ath_hal_calibrate(ah, &hchan)) {
		DPRINTF(("%s: %s: calibration of channel %u failed\n",
			dev->name, __func__, c->ic_freq));
		sc->sc_stats.ast_per_calfail++;
	}
	sc->sc_cal_ch.expires = jiffies + (ath_calinterval * HZ);
	add_timer(&sc->sc_cal_ch);
}

static int
ath_newstate(void *arg, enum ieee80211_state nstate)
{
	struct net_device *dev = (struct net_device *) arg;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	int i, error;
	u_int8_t *bssid;
	u_int32_t rfilt;
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

	DPRINTF(("%s: %s -> %s\n", __func__, stname[ostate], stname[nstate]));

	ath_hal_setledstate(ah, leds[nstate]);	/* set LED */
	netif_stop_queue(dev);			/* before we do anything else */

	if (nstate == IEEE80211_S_INIT) {
		sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
		ath_hal_intrset(ah, sc->sc_imask);
		error = 0;			/* cheat + use error return */
		goto bad;
	}
	ni = &ic->ic_bss;
	error = ath_chan_set(sc, ni->ni_chan);
	if (error != 0)
		goto bad;
	rfilt = (ath_hal_getrxfilter(ah) & HAL_RX_FILTER_PHYERR)
	      | HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST | HAL_RX_FILTER_MCAST;
	if (ic->ic_opmode != IEEE80211_M_STA)
		rfilt |= HAL_RX_FILTER_PROBEREQ;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP && (dev->flags & IFF_PROMISC))
		rfilt |= HAL_RX_FILTER_PROM;
	if (ic->ic_state == IEEE80211_S_SCAN ||
	    ic->ic_opmode == IEEE80211_M_STA)
		rfilt |= HAL_RX_FILTER_BEACON;
	if (nstate == IEEE80211_S_SCAN) {
		mod_timer(&sc->sc_scan_ch,
			jiffies + ((HZ * ath_dwelltime) / 1000));
		bssid = dev->broadcast;
	} else {
		del_timer(&sc->sc_scan_ch);
		bssid = ni->ni_bssid;
	}
	ath_hal_setrxfilter(ah, rfilt);
	DPRINTF(("%s: RX filter 0x%x bssid %s\n",
		 __func__, rfilt, ether_sprintf(bssid)));

	if (nstate == IEEE80211_S_RUN && ic->ic_opmode == IEEE80211_M_STA)
		ath_hal_setassocid(ah, bssid, ni->ni_associd);
	else
		ath_hal_setassocid(ah, bssid, 0);
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, bssid);
	}

	if (ic->ic_opmode == IEEE80211_M_MONITOR) {
		/* start periodic recalibration timer */
		mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));
		netif_start_queue(dev);
	} else if (nstate == IEEE80211_S_RUN) {
		DPRINTF(("%s(RUN): ic_flags=0x%08x iv=%d bssid=%s "
			"capinfo=0x%04x chan=%d\n"
			 , __func__
			 , ic->ic_flags
			 , ni->ni_intval
			 , ether_sprintf(ni->ni_bssid)
			 , ni->ni_capinfo
			 , ieee80211_chan2ieee(ic, ni->ni_chan)));

		/*
		 * Allocate and setup the beacon frame for AP or adhoc mode.
		 */
		if (ic->ic_opmode == IEEE80211_M_HOSTAP ||
		    ic->ic_opmode == IEEE80211_M_IBSS) {
			error = ath_beacon_alloc(sc, ni);
			if (error != 0)
				goto bad;
		}

		/*
		 * Configure the beacon and sleep timers.
		 */
		ath_beacon_config(sc);

		/* start periodic recalibration timer */
		mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));

		netif_start_queue(dev);
	} else {
		sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
		ath_hal_intrset(ah, sc->sc_imask);
		del_timer(&sc->sc_cal_ch);	/* no calibration */
	}

	/*
	 * Reset the rate control state.
	 */
	ath_rate_ctl_reset(sc, nstate);
	return 0;
bad:
	del_timer(&sc->sc_scan_ch);
	del_timer(&sc->sc_cal_ch);
	return error;
}

static int
ath_getchannels(struct net_device *dev, u_int cc, HAL_BOOL outdoor)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	HAL_CHANNEL *chans;
	int i, ix, nchan;

	sc->sc_have11g = 0;
	chans = kmalloc(IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL), GFP_KERNEL);
	if (chans == NULL) {
		printk("%s: unable to allocate channel table\n", dev->name);
		return ENOMEM;
	}
	if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, &nchan,
	    cc, HAL_MODE_ALL, outdoor)) {
		printk("%s: unable to collect channel list from hal\n",
			dev->name);
		kfree(chans);
		return EINVAL;
	}

	/*
	 * Convert HAL channels to ieee80211 ones and insert
	 * them in the table according to their channel number.
	 */
	for (i = 0; i < nchan; i++) {
		HAL_CHANNEL *c = &chans[i];
		ix = ath_hal_mhz2ieee(c->channel, c->channelFlags);
		if (ix > IEEE80211_CHAN_MAX) {
			printk("%s: bad hal channel %u (%u/%x) ignored\n",
				dev->name, ix, c->channel, c->channelFlags);
			continue;
		}
		/* NB: flags are known to be compatible */
		if (ic->ic_channels[ix].ic_freq == 0) {
			ic->ic_channels[ix].ic_freq = c->channel;
			ic->ic_channels[ix].ic_flags = c->channelFlags;
		} else {
			/* channels overlap; e.g. 11g and 11b */
			ic->ic_channels[ix].ic_flags |= c->channelFlags;
		}
		if ((c->channelFlags & CHANNEL_G) == CHANNEL_G)
			sc->sc_have11g = 1;
	}
	kfree(chans);
	return 0;
}

static int
ath_rate_setup(struct net_device *dev, u_int mode)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const HAL_RATE_TABLE *rt;
	struct ieee80211_rateset *rs;
	int i, maxrates;

	switch (mode) {
	case IEEE80211_MODE_11A:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11A);
		break;
	case IEEE80211_MODE_11B:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11B);
		break;
	case IEEE80211_MODE_11G:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11G);
		break;
	case IEEE80211_MODE_TURBO:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_TURBO);
		break;
	default:
		DPRINTF(("%s: invalid mode %u\n", __func__, mode));
		return 0;
	}
	rt = sc->sc_rates[mode];
	if (rt == NULL)
		return 0;
	if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
		DPRINTF(("%s: rate table too small (%u > %u)\n",
			__func__, rt->rateCount, IEEE80211_RATE_MAXSIZE));
		maxrates = IEEE80211_RATE_MAXSIZE;
	} else
		maxrates = rt->rateCount;
	rs = &ic->ic_sup_rates[mode];
	for (i = 0; i < maxrates; i++)
		rs->rs_rates[i] = rt->info[i].dot11Rate;
	rs->rs_nrates = maxrates;
	return 1;
}

static void
ath_setcurmode(struct ath_softc *sc, enum ieee80211_phymode mode)
{
	const HAL_RATE_TABLE *rt;
	int i;

	memset(sc->sc_rixmap, 0xff, sizeof(sc->sc_rixmap));
	rt = sc->sc_rates[mode];
	KASSERT(rt != NULL, ("no h/w rate set for phy mode %u", mode));
	for (i = 0; i < rt->rateCount; i++)
		sc->sc_rixmap[rt->info[i].dot11Rate & IEEE80211_RATE_VAL] = i;
	sc->sc_currates = rt;
	sc->sc_curmode = mode;
}

/*
 * Reset the rate control state for each 802.11 state transition.
 */
static void
ath_rate_ctl_reset(struct ath_softc *sc, enum ieee80211_state state)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_nodestat *st;

	if (ic->ic_fixed_rate != -1)
		return;
	st = &sc->sc_bss_stat;
	st->st_tx_ok = st->st_tx_err = st->st_tx_retr = st->st_tx_upper = 0;
	if (ic->ic_opmode == IEEE80211_M_STA) {
		ni = &ic->ic_bss;
		if (state == IEEE80211_S_RUN) {
			/* start with highest negotiated rate */
			KASSERT(ni->ni_rates.rs_nrates > 0,
				("transition to RUN state w/ no rates!"));
			ni->ni_txrate = ni->ni_rates.rs_nrates - 1;
		} else {
			/* use lowest rate */
			ni->ni_txrate = 0;
		}
	} else {
		TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
			ni->ni_txrate = 0;		/* use lowest rate */
			st = ni->ni_private;
			st->st_tx_ok = st->st_tx_err = st->st_tx_retr =
			    st->st_tx_upper = 0;
		}
	}
	if (state == IEEE80211_S_RUN) {
		int interval;
		/*
		 * Start the background rate control thread if we
		 * are not configured to use a fixed xmit rate.
		 */
		interval = ath_rateinterval;
		if (ic->ic_opmode == IEEE80211_M_STA)
			interval /= 2;
		mod_timer(&sc->sc_rate_ctl, jiffies + ((HZ * interval) / 1000));
	}
}

/* 
 * Examine and potentially adjust the transmit rate.
 */
static void
ath_rate_ctl(void *arg, struct ieee80211_node *ni)
{
	struct ath_softc *sc = arg;
	struct ath_nodestat *st = ni->ni_private;
	struct ieee80211_rateset *rs = &ni->ni_rates;
	int mod = 0, orate, enough;

	/*
	 * Rate control
	 * XXX: very primitive version.
	 */
	sc->sc_stats.ast_rate_calls++;

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
		if (ni->ni_txrate > 0) {
			ni->ni_txrate--;
			sc->sc_stats.ast_rate_drop++;
		}
		st->st_tx_upper = 0;
		break;
	case 1:
		if (++st->st_tx_upper < 10)
			break;
		st->st_tx_upper = 0;
		if (ni->ni_txrate + 1 < rs->rs_nrates) {
			ni->ni_txrate++;
			sc->sc_stats.ast_rate_raise++;
		}
		break;
	}

	if (ni->ni_txrate != orate) {
		DPRINTF(("%s: %dM -> %dM (%d ok, %d err, %d retr)\n",
		    __func__,
		    (rs->rs_rates[orate] & IEEE80211_RATE_VAL) / 2,
		    (rs->rs_rates[ni->ni_txrate] & IEEE80211_RATE_VAL) / 2,
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
	int interval;

	if (dev->flags & IFF_RUNNING) {
		if (ic->ic_opmode == IEEE80211_M_STA)
			ath_rate_ctl(sc, &ic->ic_bss);	/* NB: no reference */
		else
			ieee80211_iterate_nodes(ic, ath_rate_ctl, sc);
	}
	interval = ath_rateinterval;
	if (ic->ic_opmode == IEEE80211_M_STA)
		interval /= 2;
	sc->sc_rate_ctl.expires = jiffies + ((HZ * interval) / 1000);
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
			;
	stats->rx_crc_errors = sc->sc_stats.ast_rx_crcerr;

	return stats;
}

#ifdef CONFIG_SYSCTL
enum {
	ATH_STATS	= 1,
	ATH_DEBUG	= 2,
	ATH_DWELLTIME	= 3,
	ATH_CALIBRATE	= 4,
	ATH_RATEINTERVAL= 5,
	ATH_DUMP	= 8,
	ATH_CC		= 9,
	ATH_OUTDOOR	= 10,
	ATH_REGDOMAIN	= 11,
};
static	char ath_info[512];
static	char ath_dump[10];

static int
ath_sysctl_stats(ctl_table *ctl, int write, struct file *filp,
	void *buffer, size_t *lenp)
{
	struct net_device *dev;
	struct ath_softc *sc;
	char *cp = ath_info;

	if (!*lenp || (filp->f_pos && !write)) {
		*lenp = 0;
		return 0;
	}
	dev = dev_get_by_name("ath0");		/* XXX */
	if (!dev)
		return EINVAL;
	sc = dev->priv;
#define	STAT(x) do {							\
	if (sc->sc_stats.ast_##x != 0)					\
		cp += sprintf(cp, #x "=%u\n", sc->sc_stats.ast_##x);	\
} while (0)
#define	ISTAT(x) do {							\
	cp += sprintf(cp, #x "=%d\n", sc->sc_stats.ast_##x);		\
} while (0)
	*cp = '\0';
	STAT(watchdog);	  STAT(hardware);   STAT(bmiss);
	STAT(rxorn);	  STAT(rxeol);

	STAT(tx_mgmt);	  STAT(tx_qstop);   STAT(tx_discard); STAT(tx_invalid);
	STAT(tx_encap);	  STAT(tx_nonode);  STAT(tx_nobuf);   STAT(tx_nobufmgt);
	STAT(tx_xretries);STAT(tx_fifoerr); STAT(tx_filtered);
	STAT(tx_shortretry);		    STAT(tx_longretry);
	STAT(tx_badrate); STAT(tx_noack);   STAT(tx_rts);     STAT(tx_cts);
	STAT(tx_shortpre);ISTAT(tx_rssi);   ISTAT(tx_rssidelta);

	STAT(rx_orn);	  STAT(rx_crcerr);  STAT(rx_fifoerr); STAT(rx_badcrypt);
#define	PHYSTAT(x) do {							\
	if (sc->sc_stats.ast_rx_phy[HAL_PHYERR_##x] != 0)		\
		cp += sprintf(cp, "PHYERR_" #x "=%u\n",			\
			sc->sc_stats.ast_rx_phy[HAL_PHYERR_##x]);	\
} while (0)
	PHYSTAT(UNDERRUN);	PHYSTAT(TIMING);	PHYSTAT(PARITY);
	PHYSTAT(RATE);		PHYSTAT(LENGTH);	PHYSTAT(RADAR);
	PHYSTAT(SERVICE);	PHYSTAT(TOR);

	PHYSTAT(OFDM_TIMING);
	PHYSTAT(OFDM_SIGNAL_PARITY);
	PHYSTAT(OFDM_RATE_ILLEGAL);
	PHYSTAT(OFDM_LENGTH_ILLEGAL);
	PHYSTAT(OFDM_POWER_DROP);
	PHYSTAT(OFDM_SERVICE);
	PHYSTAT(OFDM_RESTART);
	PHYSTAT(CCK_TIMING);
	PHYSTAT(CCK_HEADER_CRC);
	PHYSTAT(CCK_RATE_ILLEGAL);
	PHYSTAT(CCK_SERVICE);
	PHYSTAT(CCK_RESTART);

	STAT(rx_nobuf);		ISTAT(rx_rssi);		ISTAT(rx_rssidelta);

	STAT(be_nobuf);

	STAT(per_cal);		STAT(per_rfgain);
	STAT(rate_calls);	STAT(rate_raise);	STAT(rate_drop);

	dev_put(dev);

	return proc_dostring(ctl, write, filp, buffer, lenp);
#undef PHYSTAT
#undef STAT
}

static int
ath_sysctl_handler(ctl_table *ctl, int write, struct file *filp,
	void *buffer, size_t *lenp)
{
	int *valp = ctl->data;
	int val = *valp;			/* save old value */
	int ret = proc_dointvec(ctl, write, filp, buffer, lenp);
	if (write && *valp != val) {
		/* XXX this is wrong, need to intercept writes */
		switch (ctl->ctl_name) {
		case ATH_DWELLTIME:
			if (*valp < 100)	/* 100ms min */
				*valp = 100;
			break;
		case ATH_CALIBRATE:
			if (*valp < 1)		/* 1/second min */
				*valp = 1;
			break;
		case ATH_RATEINTERVAL:
			if (*valp < 500)	/* 500ms min */
				*valp = 500;
			break;
		}
	}
	return ret;
}

static int
ath_sysctl_dump(ctl_table *ctl, int write, struct file *filp,
	void *buffer, size_t *lenp)
{
	int ret = proc_dostring(ctl, write, filp, buffer, lenp);
	/* NB: should always be a write */
	if (ret == 0) {
		struct net_device *dev;
		struct ath_softc *sc;

		dev = dev_get_by_name("ath0");		/* XXX */
		if (!dev) {
			printk("%s: no ath0 device\n", __func__);
			return EINVAL;
		}
		sc = dev->priv;
		if (*lenp >= 3 && strncmp(buffer, "hal", 3) == 0)
			ath_hal_dumpstate(sc->sc_ah);
		else {
			printk("%s: don't grok \"%.*s\"\n",
				__func__, *lenp, (char*) buffer);
			ret = -EINVAL;
		}
		dev_put(dev);
	}
	return ret;
}

enum {
	DEV_ATH		= 9,			/* XXX */
};
static ctl_table ath_sysctls[] = {
	{ ATH_STATS, 		"stats",	ath_info,
	  sizeof(ath_info),	0444,	NULL,	ath_sysctl_stats },
#ifdef AR_DEBUG
	{ ATH_DEBUG, 		"debug",	&ath_debug,
	  sizeof(ath_debug),	0644,	NULL,	ath_sysctl_handler },
#endif
	{ ATH_DWELLTIME,	"dwelltime",	&ath_dwelltime,
	  sizeof(ath_dwelltime),0644,	NULL,	ath_sysctl_handler },
	{ ATH_CALIBRATE,	"calibrate",	&ath_calinterval,
	  sizeof(ath_calinterval),0644,	NULL,	ath_sysctl_handler },
	{ ATH_RATEINTERVAL,	"rateinterval",	&ath_rateinterval,
	  sizeof(ath_rateinterval),0644,NULL,	ath_sysctl_handler },
	{ ATH_DUMP,		"dump",		ath_dump,
	  sizeof(ath_dump),	0200,	NULL,	ath_sysctl_dump },
	{ ATH_CC,		"countrycode",	&ath_countrycode,
	  sizeof(ath_countrycode),0444,	NULL,	ath_sysctl_handler },
	{ ATH_OUTDOOR,		"outdoor",	&ath_outdoor,
	  sizeof(ath_outdoor),	0444,	NULL,	ath_sysctl_handler },
	{ ATH_REGDOMAIN,	"regdomain",	&ath_regdomain,
	  sizeof(ath_regdomain),0444,	NULL,	ath_sysctl_handler },
	{ 0 }
};
static ctl_table ath_ath_table[] = {
	{ DEV_ATH, "ath", NULL, 0, 055, ath_sysctls },
	{ 0 }
};
static ctl_table ath_root_table[] = {
#ifdef CONFIG_PROC_FS
	{ CTL_NET, "net", NULL, 0, 0555, ath_ath_table },
#endif /* CONFIG_PROC_FS */
	{ 0 }
};
static struct ctl_table_header *ath_sysctl_header;

void
ath_sysctl_register(void)
{
	static int initialized = 0;

	if (!initialized) {
		ath_sysctl_header = register_sysctl_table(ath_root_table, 1);
		initialized = 1;
	}
}

void
ath_sysctl_unregister(void)
{
	if (ath_sysctl_header)
		unregister_sysctl_table(ath_sysctl_header);
}
#endif /* CONFIG_SYSCTL */
