/*-
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
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
 *
 * This software is derived from work of Atsushi Onoe; his contribution
 * is greatly appreciated.
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

#include "if_ethersubr.h"		/* for ETHER_IS_MULTICAST */
#include "if_media.h"
#include "if_llc.h"

#include <net80211/ieee80211_var.h>

#define	AR_DEBUG
#include "if_athvar.h"
#include "ah_desc.h"
#include "ah_devid.h"			/* XXX to identify IBM cards */

/* unalligned little endian access */     
#define LE_READ_2(p)							\
	((u_int16_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8)))
#define LE_READ_4(p)							\
	((u_int32_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8) |	\
	  (((u_int8_t *)(p))[2] << 16) | (((u_int8_t *)(p))[3] << 24)))

static void	ath_announce(struct net_device *);
static int	ath_init(struct net_device *);
static int	ath_reset(struct net_device *);
static void	ath_fatal_tasklet(TQUEUE_ARG);
static void	ath_rxorn_tasklet(TQUEUE_ARG);
static void	ath_bmiss_tasklet(TQUEUE_ARG);
static int	ath_stop_locked(struct net_device *);
static int	ath_stop(struct net_device *);
static int	ath_media_change(struct net_device *);
static void	ath_ratectl(unsigned long);
static void	ath_initkeytable(struct ath_softc *);
static int	ath_key_alloc(struct ieee80211com *,
			const struct ieee80211_key *);
static int	ath_key_delete(struct ieee80211com *,
			const struct ieee80211_key *);
static int	ath_key_set(struct ieee80211com *, const struct ieee80211_key *,
			const u_int8_t mac[IEEE80211_ADDR_LEN]);
static void	ath_key_update_begin(struct ieee80211com *);
static void	ath_key_update_end(struct ieee80211com *);
static void	ath_mode_init(struct net_device *);
static void	ath_setslottime(struct ath_softc *);
static void	ath_updateslot(struct net_device *);
static int	ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
static void	ath_beacon_tasklet(struct net_device *);
static void	ath_beacon_free(struct ath_softc *);
static void	ath_beacon_config(struct ath_softc *);
static int	ath_desc_alloc(struct ath_softc *);
static void	ath_desc_free(struct ath_softc *);
static struct ieee80211_node *ath_node_alloc(struct ieee80211com *);
static void	ath_node_free(struct ieee80211com *, struct ieee80211_node *);
static void	ath_node_copy(struct ieee80211com *,
			struct ieee80211_node *, const struct ieee80211_node *);
static u_int8_t	ath_node_getrssi(struct ieee80211com *,
			struct ieee80211_node *);
static int	ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
static void	ath_rx_tasklet(TQUEUE_ARG data);
static int	ath_hardstart(struct sk_buff *, struct net_device *);
static int	ath_mgtstart(struct ieee80211com *ic, struct sk_buff *skb);
static int	ath_tx_setup(struct ath_softc *, int ac, int txq);
static int	ath_tx_start(struct net_device *, struct ieee80211_node *,
			     struct ath_buf *, struct sk_buff *);
static void	ath_tx_tasklet_q0(TQUEUE_ARG data);
static void	ath_tx_tasklet_q0123(TQUEUE_ARG data);
static void	ath_tx_tasklet(TQUEUE_ARG data);
static void	ath_tx_timeout(struct net_device *);
static int	ath_chan_set(struct ath_softc *, struct ieee80211_channel *);
static void	ath_draintxq(struct ath_softc *);
static void	ath_stoprecv(struct ath_softc *);
static int	ath_startrecv(struct ath_softc *);
static void	ath_chan_change(struct ath_softc *, struct ieee80211_channel *);
static void	ath_next_scan(unsigned long);
static void	ath_calibrate(unsigned long);
static int	ath_newstate(struct ieee80211com *, enum ieee80211_state, int);
#if IEEE80211_VLAN_TAG_USED
static void	ath_vlan_register(struct net_device *, struct vlan_group *);
static void	ath_vlan_kill_vid(struct net_device *, unsigned short );
#endif
static struct net_device_stats *ath_getstats(struct net_device *);
#ifdef CONFIG_NET_WIRELESS
static struct iw_statistics *ath_iw_getstats(struct net_device *);
static struct iw_handler_def ath_iw_handler_def;
#endif
static void	ath_newassoc(struct ieee80211com *,
			struct ieee80211_node *, int);
static int	ath_getchannels(struct net_device *, u_int cc,
			HAL_BOOL outdoor, HAL_BOOL xchanmode);
static void	ath_update_led(struct ath_softc *);

static int	ath_set_mac_address(struct net_device *, void *);
static int	ath_change_mtu(struct net_device *, int);
static int	ath_ioctl(struct net_device *, struct ifreq *, int);

static int	ath_rate_setup(struct net_device *, u_int mode);
static void	ath_setcurmode(struct ath_softc *, enum ieee80211_phymode);
static void	ath_rate_update(struct ath_softc *, struct ieee80211_node *,
			int rate);
static void	ath_rate_ctl_start(struct ath_softc *, struct ieee80211_node *);
static void	ath_rate_ctl_reset(struct ath_softc *, enum ieee80211_state);
static void	ath_rate_ctl(void *, struct ieee80211_node *);

#ifdef CONFIG_SYSCTL
static void	ath_dynamic_sysctl_register(struct ath_softc *);
static void	ath_dynamic_sysctl_unregister(struct ath_softc *);
#endif /* CONFIG_SYSCTL */

static const char *acnames[] = {
	"WME_AC_BE",
	"WME_AC_BK",
	"WME_AC_VI",
	"WME_AC_VO",
	"WME_UPSD",
};

static	int ath_dwelltime = 200;		/* 5 channels/second */
static	int ath_calinterval = 30;		/* calibrate every 30 secs */
static	int ath_rateinterval = 1000;		/* rate ctl interval (ms)  */
static	int ath_countrycode = CTRY_DEFAULT;	/* country code */
static	int ath_regdomain = 0;			/* regulatory domain */
static	int ath_outdoor = AH_FALSE;		/* enable outdoor use */
static	int ath_xchanmode = AH_TRUE;		/* enable extended channels */

#ifdef AR_DEBUG
int	ath_debug = 0;
#define	IFF_DUMPPKTS(_ic, _m)	((ath_debug & _m) || ieee80211_msg_dumppkts(_ic))
static	void ath_printrxbuf(struct ath_buf *bf, int);
static	void ath_printtxbuf(struct ath_buf *bf, int);
enum {
	ATH_DEBUG_XMIT		= 0x00000001,	/* basic xmit operation */
	ATH_DEBUG_XMIT_DESC	= 0x00000002,	/* xmit descriptors */
	ATH_DEBUG_RECV		= 0x00000004,	/* basic recv operation */
	ATH_DEBUG_RECV_DESC	= 0x00000008,	/* recv descriptors */
	ATH_DEBUG_RATE		= 0x00000010,	/* rate control */
	ATH_DEBUG_RESET		= 0x00000020,	/* reset processing */
	ATH_DEBUG_MODE		= 0x00000040,	/* mode init/setup */
	ATH_DEBUG_BEACON 	= 0x00000080,	/* beacon handling */
	ATH_DEBUG_WATCHDOG 	= 0x00000100,	/* watchdog timeout */
	ATH_DEBUG_INTR		= 0x00001000,	/* ISR */
	ATH_DEBUG_TX_PROC	= 0x00002000,	/* tx ISR proc */
	ATH_DEBUG_RX_PROC	= 0x00004000,	/* rx ISR proc */
	ATH_DEBUG_BEACON_PROC	= 0x00008000,	/* beacon ISR proc */
	ATH_DEBUG_CALIBRATE	= 0x00010000,	/* periodic calibration */
	ATH_DEBUG_KEYCACHE	= 0x00020000,	/* key cache management */
	ATH_DEBUG_STATE		= 0x00040000,	/* 802.11 state transitions */
	ATH_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	ATH_DEBUG_ANY		= 0xffffffff
};
#define	DPRINTF(_m, _fmt, ...) do {				\
	if (ath_debug & _m)					\
		printk(_fmt, __VA_ARGS__);			\
} while (0)
#define	KEYPRINTF(sc, ix, hk, mac) do {				\
	if (ath_debug & ATH_DEBUG_KEYCACHE)			\
		ath_keyprint(__func__, ix, hk, mac);		\
} while (0)
#else
#define	IFF_DUMPPKTS(_ic, _m)	netif_msg_dumppkts(_ic)
#define	DPRINTF(_m, _fmt, ...)
#define	KEYPRINTF(sc, k, ix, mac)
#endif

static	int countrycode = -1;
MODULE_PARM(countrycode, "i");
MODULE_PARM_DESC(countrycode, "Override default country code");
static	int outdoor = -1;
MODULE_PARM(outdoor, "i");
MODULE_PARM_DESC(outdoor, "Enable/disable outdoor use");
static	int xchanmode = -1;
MODULE_PARM(xchanmode, "i");
MODULE_PARM_DESC(xchanmode, "Enable/disable extended channel mode");

int
ath_attach(u_int16_t devid, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah;
	HAL_STATUS status;
	int error = 0, i;
	u_int8_t csz;

	DPRINTF(ATH_DEBUG_ANY, "%s: devid 0x%x\n", __func__, devid);

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(sc->sc_pdev, PCI_CACHE_LINE_SIZE, &csz);
	/* XXX assert csz is non-zero */
	sc->sc_cachelsz = csz << 2;		/* convert to bytes */

	ATH_LOCK_INIT(sc);
	ATH_TXBUF_LOCK_INIT(sc);

	ATH_INIT_TQUEUE(&sc->sc_rxtq,	ath_rx_tasklet,		dev);
	ATH_INIT_TQUEUE(&sc->sc_txtq,	ath_tx_tasklet,		dev);
	ATH_INIT_TQUEUE(&sc->sc_bmisstq,ath_bmiss_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_rxorntq,ath_rxorn_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_fataltq,ath_fatal_tasklet,	dev);

	/*
	 * Attach the hal and verify ABI compatibility by checking
	 * the hal's ABI signature against the one the driver was
	 * compiled with.  A mismatch indicates the driver was
	 * built with an ah.h that does not correspond to the hal
	 * module loaded in the kernel.
	 */
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
	 * Check if the MAC has multi-rate retry support.
	 * We do this by trying to setup a fake extended
	 * descriptor.  MAC's that don't have support will
	 * return false w/o doing anything.  MAC's that do
	 * support it will return true w/o doing anything.
	 */
	sc->sc_mrretry = ath_hal_setupxtxdesc(ah, NULL, 0,0, 0,0, 0,0);

	/*
	 * Get the hardware key cache size.
	 */
	sc->sc_keymax = ath_hal_keycachesize(ah);
	if (sc->sc_keymax > sizeof(sc->sc_keymap) * NBBY) {
		printk("%s: Warning, using only %u entries in %u key cache\n",
			dev->name, sizeof(sc->sc_keymap) * NBBY, sc->sc_keymax);
		sc->sc_keymax = sizeof(sc->sc_keymap) * NBBY;
	}
	/*
	 * Reset the key cache since some parts do not
	 * reset the contents on initial power up.
	 */
	for (i = 0; i < sc->sc_keymax; i++)
		ath_hal_keyreset(ah, i);
	/*
	 * Mark key cache slots associated with global keys
	 * as in use.  If we knew TKIP was not to be used we
	 * could leave the +32, +64, and +32+64 slots free.
	 * XXX only for splitmic.
	 */
	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
		setbit(sc->sc_keymap, i);
		setbit(sc->sc_keymap, i+32);
		setbit(sc->sc_keymap, i+64);
		setbit(sc->sc_keymap, i+32+64);
	}

	/*
	 * Collect the channel list using the default country
	 * code and including outdoor channels.  The 802.11 layer
	 * is resposible for filtering this list to a set of
	 * channels that it considers ok to use.
	 */
	if (countrycode != -1)
		ath_countrycode = countrycode;
	if (outdoor != -1)
		ath_outdoor = outdoor;
	if (xchanmode != -1)
		ath_xchanmode = xchanmode;
	error = ath_getchannels(dev, ath_countrycode,
			ath_outdoor, ath_xchanmode);
	if (error != 0)
		goto bad;
	/*
	 * Copy these back; they are set as a side effect
	 * of constructing the channel list.
	 */
	ath_hal_getregdomain(ah, &ath_regdomain);
	ath_hal_getcountrycode(ah, &ath_countrycode);

	/*
	 * Setup rate tables for all potential media types.
	 */
	ath_rate_setup(dev, IEEE80211_MODE_11A);
	ath_rate_setup(dev, IEEE80211_MODE_11B);
	ath_rate_setup(dev, IEEE80211_MODE_11G);
	ath_rate_setup(dev, IEEE80211_MODE_TURBO);
	/* NB: setup here so ath_rate_update is happy */
	ath_setcurmode(sc, IEEE80211_MODE_11A);

	/*
	 * Allocate tx+rx descriptors and populate the lists.
	 */
	error = ath_desc_alloc(sc);
	if (error != 0) {
		printk(KERN_ERR "%s: failed to allocate descriptors: %d\n",
			dev->name, error);
		goto bad;
	}

	/*
	 * Allocate hardware transmit queues: one queue for
	 * beacon frames and one data queue for each QoS
	 * priority.  Note that the hal handles reseting
	 * these queues at the needed time.
	 *
	 * XXX PS-Poll
	 */
	sc->sc_bhalq = ath_hal_setuptxqueue(ah, HAL_TX_QUEUE_BEACON, NULL);
	if (sc->sc_bhalq == (u_int) -1) {
		printk(KERN_ERR "%s: unable to setup a beacon xmit queue!\n",
			dev->name);
		goto bad2;
	}
	if (!ath_tx_setup(sc, WME_AC_BE, HAL_WME_AC_BE))
		goto bad2;
	if (!ath_tx_setup(sc, WME_AC_BK, HAL_WME_AC_BK))
		goto bad2;
	if (!ath_tx_setup(sc, WME_AC_VI, HAL_WME_AC_VI))
		goto bad2;
	if (!ath_tx_setup(sc, WME_AC_VO, HAL_WME_AC_VO))
		goto bad2;
	/* 
	 * Special case certain configurations.
	 */
	switch (sc->sc_txqsetup) {
	case 0x01:
		ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0, dev);
		break;
	case 0x0f:
		ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0123, dev);
		break;
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

	sc->sc_ledstate = 1;
	/*
	 * Auto-enable soft led processing for IBM cards and for
	 * 5211 minipci cards.  Users can also manually enable/disable
	 * support with a sysctl.
	 */
	sc->sc_softled = (devid == AR5212_DEVID_IBM || devid == AR5211_DEVID);
	if (sc->sc_softled) {
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
		ath_hal_gpioset(ah, sc->sc_ledpin, 0);
	}

	ether_setup(dev);
	dev->open = ath_init;
	dev->stop = ath_stop;
	dev->hard_start_xmit = ath_hardstart;
	dev->tx_timeout = ath_tx_timeout;
	dev->watchdog_timeo = 5 * HZ;			/* XXX */
	dev->set_multicast_list = ath_mode_init;
	dev->do_ioctl = ath_ioctl;
	dev->get_stats = ath_getstats;
	dev->set_mac_address = ath_set_mac_address;
 	dev->change_mtu = &ath_change_mtu;
	dev->tx_queue_len = ATH_TXBUF-1;		/* 1 for mgmt frame */
#ifdef CONFIG_NET_WIRELESS
	dev->get_wireless_stats = ath_iw_getstats;
	ieee80211_ioctl_iwsetup(&ath_iw_handler_def);
	dev->wireless_handlers = &ath_iw_handler_def;
#endif /* CONFIG_NET_WIRELESS */
#if IEEE80211_VLAN_TAG_USED
	dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
	dev->vlan_rx_register = ath_vlan_register;
	dev->vlan_rx_kill_vid = ath_vlan_kill_vid;
#endif /* IEEE80211_VLAN_TAG_USED */

	ic->ic_dev = dev;
	ic->ic_devstats = &sc->sc_devstats;
	ic->ic_mgtstart = ath_mgtstart;
	ic->ic_init = ath_init;
	ic->ic_reset = ath_reset;
	ic->ic_newassoc = ath_newassoc;
	ic->ic_updateslot = ath_updateslot;
	/* XXX not right but it's not used anywhere important */
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_caps =
		  IEEE80211_C_IBSS		/* ibss, nee adhoc, mode */
		| IEEE80211_C_HOSTAP		/* hostap mode */
		| IEEE80211_C_MONITOR		/* monitor mode */
		| IEEE80211_C_SHPREAMBLE	/* short preamble supported */
		| IEEE80211_C_SHSLOT		/* short slot time supported */
		| IEEE80211_C_TXPMGT		/* transmit power control */
		;
	/*
	 * Query the hal to figure out h/w crypto support.
	 */
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_WEP))
		ic->ic_caps |= IEEE80211_C_WEP;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_AES_OCB))
		ic->ic_caps |= IEEE80211_C_AES;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_AES_CCM))
		ic->ic_caps |= IEEE80211_C_AES_CCM;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_CKIP))
		ic->ic_caps |= IEEE80211_C_CKIP;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_TKIP)) {
		ic->ic_caps |= IEEE80211_C_TKIP;
		/*
		 * Check if h/w does the MIC and/or whether the
		 * separate key cache entries are required to
		 * handle both tx+rx MIC keys.
		 */
		if (ath_hal_ciphersupported(ah, HAL_CIPHER_MIC))
			ic->ic_caps |= IEEE80211_C_TKIPMIC;
		if (ath_hal_tkipsplit(ah))
			sc->sc_splitmic = 1;
	}
	/*
	 * Indicate we need the 802.11 header padded to a
	 * 32-bit boundary for 4-address and QoS frames.
	 */
	ic->ic_flags |= IEEE80211_F_DATAPAD;

	/* get mac address from hardware */
	ath_hal_getmac(ah, ic->ic_myaddr);
	IEEE80211_ADDR_COPY(dev->dev_addr, ic->ic_myaddr);

	/* call MI attach routine. */
	ieee80211_ifattach(ic);
	/* override default methods */
	ic->ic_node_alloc = ath_node_alloc;
	sc->sc_node_free = ic->ic_node_free;
	ic->ic_node_free = ath_node_free;
	sc->sc_node_copy = ic->ic_node_copy;
	ic->ic_node_copy = ath_node_copy;
	ic->ic_node_getrssi = ath_node_getrssi;
	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = ath_newstate;
	ic->ic_crypto.cs_key_alloc = ath_key_alloc;
	ic->ic_crypto.cs_key_delete = ath_key_delete;
	ic->ic_crypto.cs_key_set = ath_key_set;
	ic->ic_crypto.cs_key_update_begin = ath_key_update_begin;
	ic->ic_crypto.cs_key_update_end = ath_key_update_end;

	/* complete initialization */
	ieee80211_media_init(ic, ath_media_change, ieee80211_media_status);

	if (register_netdev(dev)) {
		printk(KERN_ERR "%s: unable to register device\n", dev->name);
		goto bad3;
	}
	/*
	 * Attach dynamic MIB vars and announce support
	 * now that we have a device name with unit number.
	 */
#ifdef CONFIG_SYSCTL
	ath_dynamic_sysctl_register(sc);
	ieee80211_sysctl_register(ic);
#endif /* CONFIG_SYSCTL */
	ieee80211_announce(ic);
	ath_announce(dev);
	return 0;
bad3:
	ieee80211_ifdetach(ic);
bad2:
	if (sc->sc_txq[WME_AC_BK].axq_qnum != (u_int) -1)
		ATH_TXQ_LOCK_DESTROY(&sc->sc_txq[WME_AC_BK]);
	if (sc->sc_txq[WME_AC_BE].axq_qnum != (u_int) -1)
		ATH_TXQ_LOCK_DESTROY(&sc->sc_txq[WME_AC_BE]);
	if (sc->sc_txq[WME_AC_VI].axq_qnum != (u_int) -1)
		ATH_TXQ_LOCK_DESTROY(&sc->sc_txq[WME_AC_VI]);
	if (sc->sc_txq[WME_AC_VO].axq_qnum != (u_int) -1)
		ATH_TXQ_LOCK_DESTROY(&sc->sc_txq[WME_AC_VO]);
	ath_desc_free(sc);
bad:
	if (ah)
		ath_hal_detach(ah);
	ATH_TXBUF_LOCK_DESTROY(sc);
	ATH_LOCK_DESTROY(sc);
	sc->sc_invalid = 1;
	return error;
}

int
ath_detach(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int i;

	DPRINTF(ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_stop(dev);
	sc->sc_invalid = 1;
	/*
	 * NB: Must do this before detaching the hal to insure
	 *     callbacks into the driver to delete global key
	 *     entries can be handled.
	 */
	ieee80211_ifdetach(ic);
	ath_desc_free(sc);
	ath_hal_detach(sc->sc_ah);

	/*
	 * NB: can't reclaim these until after ieee80211_ifdetach
	 * returns because we'll get called back to reclaim node
	 * state and potentially want to use them.
	 */
	ATH_TXBUF_LOCK_DESTROY(sc);
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ATH_TXQ_LOCK_DESTROY(&sc->sc_txq[i]);

#ifdef CONFIG_SYSCTL
	ath_dynamic_sysctl_unregister(sc);
#endif /* CONFIG_SYSCTL */
	ATH_LOCK_DESTROY(sc);
	unregister_netdev(dev);

	return 0;
}

/*
 * Announce various information on device/driver attach.
 */
static void
ath_announce(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	int i;

	printk("%s: mac %d.%d phy %d.%d", dev->name,
		ah->ah_macVersion, ah->ah_macRev,
		ah->ah_phyRev >> 4, ah->ah_phyRev & 0xf);
	if (ah->ah_analog5GhzRev)
		printk(" 5ghz radio %d.%d",
			ah->ah_analog5GhzRev >> 4, ah->ah_analog5GhzRev & 0xf);
	if (ah->ah_analog2GhzRev)
		printk(" 2ghz radio %d.%d",
			ah->ah_analog2GhzRev >> 4, ah->ah_analog2GhzRev & 0xf);
	printk("\n");
	printk("%s: 802.11 address: %s\n",
		dev->name, ether_sprintf(dev->dev_addr));
	for (i = 0; i <= WME_AC_VO; i++) {
		struct ath_txq *txq = sc->sc_ac2q[i];
		printk("%s: Use hw queue %u for %s traffic\n",
			dev->name, txq->axq_qnum, acnames[i]);
	}
}

void
ath_suspend(struct net_device *dev)
{
	DPRINTF(ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_stop(dev);
}

void
ath_resume(struct net_device *dev)
{
	DPRINTF(ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_init(dev);
}

void
ath_shutdown(struct net_device *dev)
{
	DPRINTF(ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_stop(dev);
}

/*
 * Interrupt handler.  Most of the actual processing is deferred.
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
		 * The hardware is not ready/present, don't touch anything.
		 * Note this can happen early on if the IRQ is shared.
		 */
		return IRQ_NONE;
	}
	if (!ath_hal_intrpend(ah))		/* shared irq, not for us */
		return IRQ_NONE;
	if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
		DPRINTF(ATH_DEBUG_INTR, "%s: flags 0x%x\n",
			__func__, dev->flags);
		ath_hal_getisr(ah, &status);	/* clear ISR */
		ath_hal_intrset(ah, 0);		/* disable further intr's */
		return IRQ_HANDLED;
	}
	needmark = 0;
	ath_hal_getisr(ah, &status);		/* NB: clears ISR too */
	DPRINTF(ATH_DEBUG_INTR, "%s: status 0x%x\n", __func__, status);
	status &= sc->sc_imask;			/* discard unasked for bits */
	if (status & HAL_INT_FATAL) {
		sc->sc_stats.ast_hardware++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		ATH_SCHEDULE_TQUEUE(&sc->sc_fataltq, &needmark);
	} else if (status & HAL_INT_RXORN) {
		sc->sc_stats.ast_rxorn++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		ATH_SCHEDULE_TQUEUE(&sc->sc_rxorntq, &needmark);
	} else {
		if (status & HAL_INT_RXEOL) {
			/*
			 * NB: the hardware should re-read the link when
			 *     RXE bit is written, but it doesn't work at
			 *     least on older hardware revs.
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
			ATH_SCHEDULE_TQUEUE(&sc->sc_rxtq, &needmark);
		if (status & HAL_INT_TX)
			ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, &needmark);
		if (status & HAL_INT_SWBA) {
			/*
			 * Handle beacon transmission directly; deferring
			 * this is too slow to meet timing constraints
			 * under load.
			 */
			ath_beacon_tasklet(dev);
		}
		if (status & HAL_INT_BMISS) {
			sc->sc_stats.ast_bmiss++;
			ATH_SCHEDULE_TQUEUE(&sc->sc_bmisstq, &needmark);
		}
	}
	if (needmark)
		mark_bh(IMMEDIATE_BH);
	return IRQ_HANDLED;
}

static void
ath_fatal_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;

	printk("%s: hardware error; reseting\n", dev->name);
	ath_reset(dev);
}

static void
ath_rxorn_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;

	printk("%s: rx FIFO overrun; reseting\n", dev->name);
	ath_reset(dev);
}

static void
ath_bmiss_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	DPRINTF(ATH_DEBUG_ANY, "%s\n", __func__);
	KASSERT(ic->ic_opmode == IEEE80211_M_STA,
		("unexpect operating mode %u", ic->ic_opmode));
	if (ic->ic_state == IEEE80211_S_RUN) {
		/*
		 * Rather than go directly to scan state, try to
		 * reassociate first.  If that fails then the state
		 * machine will drop us into scanning after timing
		 * out waiting for a probe response.
		 */
		ieee80211_new_state(ic, IEEE80211_S_ASSOC, -1);
	}
}

static u_int
ath_chan2flags(struct ieee80211com *ic, struct ieee80211_channel *chan)
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
	int error = 0;

	ATH_LOCK(sc);

	DPRINTF(ATH_DEBUG_RESET, "%s: mode %d\n", __func__, ic->ic_opmode);

	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop_locked(dev);
	
	/*
	 * Resize receive skb's if changing to or from monitor mode
	 */
	if ((dev->type == ARPHRD_ETHER &&
	     ic->ic_opmode == IEEE80211_M_MONITOR) ||
	    (dev->type == ARPHRD_IEEE80211_PRISM &&
	     ic->ic_opmode != IEEE80211_M_MONITOR)) {
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
	sc->sc_curchan.channel = ic->ic_ibss_chan->ic_freq;
	sc->sc_curchan.channelFlags = ath_chan2flags(ic, ic->ic_ibss_chan);
	if (!ath_hal_reset(ah, ic->ic_opmode, &sc->sc_curchan, AH_FALSE, &status)) {
		printk("%s: unable to reset hardware; hal status %u\n",
			dev->name, status);
		error = -EIO;
		goto done;
	}

	/*
	 * Setup the hardware after reset: the key cache
	 * is filled as needed and the receive engine is
	 * set going.  Frame transmit is handled entirely
	 * in the frame output path; there's nothing to do
	 * here except setup the interrupt mask.
	 */
	ath_initkeytable(sc);		/* XXX still needed? */
	if (ath_startrecv(sc) != 0) {
		printk("%s: unable to start recv logic\n", dev->name);
		error = -EIO;
		goto done;
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
	ni = ic->ic_bss;
	ni->ni_chan = ic->ic_ibss_chan;
	mode = ieee80211_chan2mode(ic, ni->ni_chan);
	if (mode != sc->sc_curmode)
		ath_setcurmode(sc, mode);
	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
		if (ic->ic_roaming != IEEE80211_ROAMING_MANUAL)
			ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
	} else
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
done:
	ATH_UNLOCK(sc);
	return error;
}

static int
ath_stop_locked(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(ATH_DEBUG_RESET, "%s: invalid %u flags 0x%x\n",
		__func__, sc->sc_invalid, dev->flags);

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Shutdown the hardware and driver:
		 *    reset 802.11 state machine (do first so
		 *	station deassoc/deauth frames can be sent)
		 *    stop output from above
		 *    disable interrupts
		 *    turn off timers
		 *    turn off the radio
		 *    clear transmit machinery
		 *    clear receive machinery
		 *    reclaim beacon resources
		 *    power down hardware
		 *
		 * Note that some of this work is not possible if the
		 * hardware is gone (invalid).
		 */
		ieee80211_new_state(ic, IEEE80211_S_INIT, -1);
		netif_stop_queue(dev);
		dev->flags &= ~IFF_RUNNING;
		if (!sc->sc_invalid) {
			if (sc->sc_softled)
				ath_hal_gpioset(ah, sc->sc_ledpin, 1);
			ath_hal_intrset(ah, 0);
			ath_hal_phydisable(ah);
		}
		ath_draintxq(sc);
		if (!sc->sc_invalid)
			ath_stoprecv(sc);
		else
			sc->sc_rxlink = NULL;
		ath_beacon_free(sc);
		if (!sc->sc_invalid)
			ath_hal_setpower(ah, HAL_PM_FULL_SLEEP, 0);
	}
	return 0;
}

/*
 * Stop the device, grabbing the top-level lock to protect
 * against concurrent entry through ath_init (which can happen
 * if another thread does a system call and the thread doing the
 * stop is preempted).
 */
static int
ath_stop(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	int error;

	ATH_LOCK(sc);
	error = ath_stop_locked(dev);
	ATH_UNLOCK(sc);
	return error;
}

/*
 * Reset the hardware w/o losing operational state.  This is
 * basically a more efficient way of doing ath_stop, ath_init,
 * followed by state transitions to the current 802.11
 * operational state.  Used to recover from errors rx overrun
 * and to reset the hardware when rf gain settings must be reset.
 */
static int
ath_reset(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_channel *c;
	HAL_STATUS status;

	/*
	 * Convert to a HAL channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	c = ic->ic_ibss_chan;
	sc->sc_curchan.channel = c->ic_freq;
	sc->sc_curchan.channelFlags = ath_chan2flags(ic, c);

	ath_hal_intrset(ah, 0);		/* disable interrupts */
	ath_draintxq(sc);		/* stop xmit side */
	ath_stoprecv(sc);		/* stop recv side */
	/* NB: indicate channel change so we do a full reset */
	if (!ath_hal_reset(ah, ic->ic_opmode, &sc->sc_curchan, AH_TRUE, &status))
		printk("%s: %s: unable to reset hardware; hal status %u\n",
			dev->name, __func__, status);
	if (ath_startrecv(sc) != 0)	/* restart recv */
		printk("%s: %s: unable to start recv logic\n",
			dev->name, __func__);
	/*
	 * We may be doing a reset in response to an ioctl
	 * that changes the channel so update any state that
	 * might change as a result.
	 */
	ath_chan_change(sc, c);
	if (ic->ic_state == IEEE80211_S_RUN)
		ath_beacon_config(sc);	/* restart beacons */
	ath_hal_intrset(ah, sc->sc_imask);

	if (ic->ic_state == IEEE80211_S_RUN)
		netif_wake_queue(dev);	/* restart xmit */
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
	struct ieee80211_node *ni = NULL;
	struct ath_buf *bf = NULL;
	int pktlen;

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(ATH_DEBUG_XMIT, "%s: discard, invalid %d flags %x\n",
			__func__, sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		return -ENETDOWN;
	}
	/*
	 * No data frames go out unless we're associated; this
	 * should not happen as the 802.11 layer does not enable
	 * the xmit queue until we enter the RUN state.
	 */
	if (ic->ic_state != IEEE80211_S_RUN) {
		DPRINTF(ATH_DEBUG_XMIT, "%s: discard, state %u\n",
			__func__, ic->ic_state);
		sc->sc_stats.ast_tx_discard++;
		goto bad;
	}

	/*
	 * Grab a TX buffer and associated resources.
	 */
	ATH_TXBUF_LOCK_BH(sc);
	bf = TAILQ_FIRST(&sc->sc_txbuf);
	if (bf != NULL)
		TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
	/* XXX use a counter and leave at least one for mgmt frames */
	if (TAILQ_EMPTY(&sc->sc_txbuf)) {
		DPRINTF(ATH_DEBUG_XMIT, "%s: stop queue\n", __func__);
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(dev);
	}
	ATH_TXBUF_UNLOCK_BH(sc);
	if (bf == NULL) {		/* NB: should not happen */
		printk("%s: discard, no xmit buf\n", __func__);
		sc->sc_stats.ast_tx_nobuf++;
		goto bad;
	}

	/*
	 * Encapsulate the packet for transmission.
	 */
	skb = ieee80211_encap(ic, skb, &ni);
	if (skb == NULL) {
		DPRINTF(ATH_DEBUG_XMIT,
			"%s: discard, encapsulation failure\n", __func__);
		sc->sc_stats.ast_tx_encap++;
		goto bad;
	}
	pktlen = skb->len;		/* NB: don't reference skb below */
	if (ath_tx_start(dev, ni, bf, skb) == 0) {
		sc->sc_devstats.tx_packets++;
		sc->sc_devstats.tx_bytes += pktlen;
		return 0;
	}
	/* fall thru... */
bad:
	if (ni && ni != ic->ic_bss)
		ieee80211_free_node(ic, ni);
	if (bf != NULL) {
		ATH_TXBUF_LOCK_BH(sc);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK_BH(sc);
	}
	if (skb)
		dev_kfree_skb(skb);
	return 0;	/* NB: return !0 only in a ``hard error condition'' */
}

/*
 * Transmit a management frame.  On failure we reclaim the skbuff.
 * Note that management frames come directly from the 802.11 layer
 * and do not honor the send queue flow control.  Need to investigate
 * using priority queueing so management frames can bypass data.
 */
static int
ath_mgtstart(struct ieee80211com *ic, struct sk_buff *skb)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni = NULL;
	struct ath_buf *bf = NULL;
	struct ieee80211_frame *wh;
	struct ieee80211_cb *cb;
	int error;

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(ATH_DEBUG_XMIT, "%s: discard, invalid %d flags %x\n",
			__func__, sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		error = -ENETDOWN;
		goto bad;
	}
	/*
	 * Grab a TX buffer and associated resources.
	 */
	ATH_TXBUF_LOCK_BH(sc);
	bf = TAILQ_FIRST(&sc->sc_txbuf);
	if (bf != NULL)
		TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
	if (TAILQ_EMPTY(&sc->sc_txbuf))	{
		DPRINTF(ATH_DEBUG_XMIT, "%s: stop queue\n", __func__);
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(dev);
	}
	ATH_TXBUF_UNLOCK_BH(sc);
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
	 * NB: the referenced node pointer is in the
	 * control block of the sk_buff.  This is
	 * placed there by ieee80211_mgmt_output because
	 * we need to hold the reference with the frame.
	 */
	cb = (struct ieee80211_cb *)skb->cb;
	ni = cb->ni;
	error = ath_tx_start(dev, ni, bf, skb);
	if (error == 0) {
		sc->sc_stats.ast_tx_mgmt++;
		return 0;
	}
	/* fall thru... */
bad:
	if (ni && ni != ic->ic_bss)
		ieee80211_free_node(ic, ni);
	if (bf != NULL) {
		ATH_TXBUF_LOCK_BH(sc);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK_BH(sc);
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


#ifdef AR_DEBUG
static void
ath_keyprint(const char *tag, u_int ix,
	const HAL_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
	static const char *ciphers[] = {
		"WEP",
		"AES-OCB",
		"AES-CCM",
		"CKIP",
		"TKIP",
		"CLR",
	};
	int i, n;

	printk("%s: [%02u] %-7s ", tag, ix, ciphers[hk->kv_type]);
	for (i = 0, n = hk->kv_len; i < n; i++)
		printk("%02x", hk->kv_val[i]);
	printk(" mac %s", ether_sprintf(mac));
	if (hk->kv_type == HAL_CIPHER_TKIP) {
		printk(" mic ");
		for (i = 0; i < sizeof(hk->kv_mic); i++)
			printk("%02x", hk->kv_mic[i]);
	}
	printk("\n");
}
#endif

/*
 * Set a TKIP key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP.
 */
static int
ath_keyset_tkip(struct ath_softc *sc, const struct ieee80211_key *k,
	HAL_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
#define	IEEE80211_KEY_XR	(IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV)
	static const u_int8_t zerobssid[IEEE80211_ADDR_LEN];
	struct ath_hal *ah = sc->sc_ah;

	KASSERT(k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP,
		("got a non-TKIP key, cipher %u", k->wk_cipher->ic_cipher));
	KASSERT(sc->sc_splitmic, ("key cache !split"));
	if ((k->wk_flags & IEEE80211_KEY_XR) == IEEE80211_KEY_XR) {
		/*
		 * TX key goes at first index, RX key at +32.
		 * The hal handles the MIC keys at index+64.
		 */
		memcpy(hk->kv_mic, k->wk_txmic, sizeof(hk->kv_mic));
		KEYPRINTF(sc, k->wk_keyix, hk, zerobssid);
		if (!ath_hal_keyset(ah, k->wk_keyix, hk, zerobssid))
			return 0;

		memcpy(hk->kv_mic, k->wk_rxmic, sizeof(hk->kv_mic));
		KEYPRINTF(sc, k->wk_keyix+32, hk, mac);
		/* XXX delete tx key on failure? */
		return ath_hal_keyset(ah, k->wk_keyix+32, hk, mac);
	} else if (k->wk_flags & IEEE80211_KEY_XR) {
		/*
		 * TX/RX key goes at first index.
		 * The hal handles the MIC keys are index+64.
		 */
		KASSERT(k->wk_keyix < IEEE80211_WEP_NKID,
			("group key at index %u", k->wk_keyix));
		memcpy(hk->kv_mic, k->wk_flags & IEEE80211_KEY_XMIT ?
			k->wk_txmic : k->wk_rxmic, sizeof(hk->kv_mic));
		KEYPRINTF(sc, k->wk_keyix, hk, zerobssid);
		return ath_hal_keyset(ah, k->wk_keyix, hk, zerobssid);
	}
	/* XXX key w/o xmit/recv; need this for compression? */
	return 0;
#undef IEEE80211_KEY_XR
}

/*
 * Set a net80211 key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP with hardware MIC support.
 */
static int
ath_keyset(struct ath_softc *sc, const struct ieee80211_key *k,
	const u_int8_t mac[IEEE80211_ADDR_LEN])
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	static const u_int8_t ciphermap[] = {
		HAL_CIPHER_WEP,		/* IEEE80211_CIPHER_WEP */
		HAL_CIPHER_TKIP,	/* IEEE80211_CIPHER_TKIP */
		HAL_CIPHER_AES_OCB,	/* IEEE80211_CIPHER_AES_OCB */
		HAL_CIPHER_AES_CCM,	/* IEEE80211_CIPHER_AES_CCM */
		(u_int8_t) -1,		/* 4 is not allocated */
		HAL_CIPHER_CKIP,	/* IEEE80211_CIPHER_CKIP */
		HAL_CIPHER_CLR,		/* IEEE80211_CIPHER_NONE */
	};
	struct ath_hal *ah = sc->sc_ah;
	const struct ieee80211_cipher *cip = k->wk_cipher;
	HAL_KEYVAL hk;

	memset(&hk, 0, sizeof(hk));
	/*
	 * Software crypto uses a "clear key" so non-crypto
	 * state kept in the key cache are maintained and
	 * so that rx frames have an entry to match.
	 */
	if ((k->wk_flags & IEEE80211_KEY_SWCRYPT) == 0) {
		KASSERT(cip->ic_cipher < N(ciphermap),
			("invalid cipher type %u", cip->ic_cipher));
		hk.kv_type = ciphermap[cip->ic_cipher];
		hk.kv_len = k->wk_keylen;
		memcpy(hk.kv_val, k->wk_key, k->wk_keylen);
	} else
		hk.kv_type = HAL_CIPHER_CLR;

	if (hk.kv_type == HAL_CIPHER_TKIP &&
	    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0 &&
	    sc->sc_splitmic) {
		return ath_keyset_tkip(sc, k, &hk, mac);
	} else {
		KEYPRINTF(sc, k->wk_keyix, &hk, mac);
		return ath_hal_keyset(ah, k->wk_keyix, &hk, mac);
	}
#undef N
}

/*
 * Fill the hardware key cache with key entries.
 */
static void
ath_initkeytable(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = ic->ic_dev;
	struct ath_hal *ah = sc->sc_ah;
	u_int8_t *bssid;
	int i;

	/* XXX maybe should reset all keys when !PRIVACY */
	if (ic->ic_state == IEEE80211_S_SCAN)
		bssid = dev->broadcast;
	else
		bssid = ic->ic_bss->ni_bssid;
	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
		struct ieee80211_key *k = &ic->ic_nw_keys[i];

		if (k->wk_keylen == 0) {
			ath_hal_keyreset(ah, i);
			DPRINTF(ATH_DEBUG_KEYCACHE, "%s: reset key %u\n",
				__func__, i);
		} else {
			ath_keyset(sc, k, bssid);
		}
	}
}

/*
 * Allocate tx/rx key slots for TKIP.  We allocate two slots for
 * each key, one for decrypt/encrypt and the other for the MIC.
 */
static u_int16_t
key_alloc_2pair(struct ath_softc *sc)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	u_int i, keyix;

	KASSERT(sc->sc_splitmic, ("key cache !split"));
	/* XXX could optimize */
	for (i = 0; i < N(sc->sc_keymap)/4; i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots in this byte are free.
			 */
			keyix = i*NBBY;
			while (b & 1) {
		again:
				keyix++;
				b >>= 1;
			}
			/* XXX IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV */
			if (isset(sc->sc_keymap, keyix+32) ||
			    isset(sc->sc_keymap, keyix+64) ||
			    isset(sc->sc_keymap, keyix+32+64)) {
				/* full pair unavailable */
				/* XXX statistic */
				if (keyix == (i+1)*NBBY) {
					/* no slots were appropriate, advance */
					continue;
				}
				goto again;
			}
			setbit(sc->sc_keymap, keyix);
			setbit(sc->sc_keymap, keyix+64);
			setbit(sc->sc_keymap, keyix+32);
			setbit(sc->sc_keymap, keyix+32+64);
			DPRINTF(ATH_DEBUG_KEYCACHE,
				"%s: key pair %u,%u %u,%u\n",
				__func__, keyix, keyix+64,
				keyix+32, keyix+32+64);
			return keyix;
		}
	}
	DPRINTF(ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
	return IEEE80211_KEYIX_NONE;
#undef N
}

/*
 * Allocate a single key cache slot.
 */
static u_int16_t
key_alloc_single(struct ath_softc *sc)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	u_int i, keyix;

	/* XXX try i,i+32,i+64,i+32+64 to minimize key pair conflicts */
	for (i = 0; i < N(sc->sc_keymap); i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots are free.
			 */
			keyix = i*NBBY;
			while (b & 1)
				keyix++, b >>= 1;
			setbit(sc->sc_keymap, keyix);
			DPRINTF(ATH_DEBUG_KEYCACHE, "%s: key %u\n",
				__func__, keyix);
			return keyix;
		}
	}
	DPRINTF(ATH_DEBUG_KEYCACHE, "%s: out of space\n", __func__);
	return IEEE80211_KEYIX_NONE;
#undef N
}

/*
 * Allocate one or more key cache slots for a uniacst key.  The
 * key itself is needed only to identify the cipher.  For hardware
 * TKIP with split cipher+MIC keys we allocate two key cache slot
 * pairs so that we can setup separate TX and RX MIC keys.  Note
 * that the MIC key for a TKIP key at slot i is assumed by the
 * hardware to be at slot i+64.  This limits TKIP keys to the first
 * 64 entries.
 */
static int
ath_key_alloc(struct ieee80211com *ic, const struct ieee80211_key *k)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	/*
	 * We allocate two pair for TKIP when using the h/w to do
	 * the MIC.  For everything else, including software crypto,
	 * we allocate a single entry.  Note that s/w crypto requires
	 * a pass-through slot on the 5211 and 5212.  The 5210 does
	 * not support pass-through cache entries and we map all
	 * those requests to slot 0.
	 */
	if (k->wk_flags & IEEE80211_KEY_SWCRYPT) {
		return key_alloc_single(sc);
	} else if (k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP &&
	    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0 && sc->sc_splitmic) {
		return key_alloc_2pair(sc);
	} else {
		return key_alloc_single(sc);
	}
}

/*
 * Delete an entry in the key cache allocated by ath_key_alloc.
 */
static int
ath_key_delete(struct ieee80211com *ic, const struct ieee80211_key *k)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	const struct ieee80211_cipher *cip = k->wk_cipher;
	u_int keyix = k->wk_keyix;

	DPRINTF(ATH_DEBUG_KEYCACHE, "%s: delete key %u\n", __func__, keyix);

	ath_hal_keyreset(ah, keyix);
	/*
	 * Handle split tx/rx keying required for TKIP with h/w MIC.
	 */
	if (cip->ic_cipher == IEEE80211_CIPHER_TKIP &&
	    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0 && sc->sc_splitmic)
		ath_hal_keyreset(ah, keyix+32);		/* RX key */
	if (keyix >= IEEE80211_WEP_NKID) {
		/*
		 * Don't touch keymap entries for global keys so
		 * they are never considered for dynamic allocation.
		 */
		clrbit(sc->sc_keymap, keyix);
		if (cip->ic_cipher == IEEE80211_CIPHER_TKIP &&
		    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0 &&
		    sc->sc_splitmic) {
			clrbit(sc->sc_keymap, keyix+64);	/* TX key MIC */
			clrbit(sc->sc_keymap, keyix+32);	/* RX key */
			clrbit(sc->sc_keymap, keyix+32+64);	/* RX key MIC */
		}
	}
	return 1;
}

/*
 * Set the key cache contents for the specified key.  Key cache
 * slot(s) must already have been allocated by ath_key_alloc.
 */
static int
ath_key_set(struct ieee80211com *ic, const struct ieee80211_key *k,
	const u_int8_t mac[IEEE80211_ADDR_LEN])
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	return ath_keyset(sc, k, mac);
}

/*
 * Block/unblock tx+rx processing while a key change is done.
 * We assume the caller serializes key management operations
 * so we only need to worry about synchronization with other
 * uses that originate in the driver.
 */
static void
ath_key_update_begin(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	/*
	 * When called from the rx tasklet we cannot use
	 * tasklet_disable because it will block waiting
	 * for us to complete execution.
	 *
	 * XXX Using in_softirq is not right since we might
	 * be called from other soft irq contexts than
	 * ath_rx_tasklet.
	 */
	if (!in_softirq())
		tasklet_disable(&sc->sc_rxtq);
	netif_stop_queue(dev);
}

static void
ath_key_update_end(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	netif_start_queue(dev);
	if (!in_softirq())		/* NB: see above */
		tasklet_enable(&sc->sc_rxtq);
}

/*
 * Calculate the receive filter according to the
 * operating mode and state:
 *
 * o always accept unicast, broadcast, and multicast traffic
 * o maintain current state of phy error reception
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
static u_int32_t
ath_calcrxfilter(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = ic->ic_dev;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	rfilt = (ath_hal_getrxfilter(ah) & HAL_RX_FILTER_PHYERR)
	      | HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST | HAL_RX_FILTER_MCAST;
	if (ic->ic_opmode != IEEE80211_M_STA)
		rfilt |= HAL_RX_FILTER_PROBEREQ;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP && (dev->flags & IFF_PROMISC))
		rfilt |= HAL_RX_FILTER_PROM;
	if (ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_IBSS ||
	    ic->ic_state == IEEE80211_S_SCAN)
		rfilt |= HAL_RX_FILTER_BEACON;
	return rfilt;
}

static void
ath_mode_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt, mfilt[2], val;
	u_int8_t pos;
	struct dev_mc_list *mc;

	/* configure rx filter */
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);

	/* configure operational mode and slot time */
	ath_hal_setopmode(ah);
	ath_setslottime(sc);

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
	DPRINTF(ATH_DEBUG_MODE, "%s: RX filter 0x%x, MC filter %08x:%08x\n",
		__func__, rfilt, mfilt[0], mfilt[1]);
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

/*
 * Setup the beacon frame.  The frame is mapped for DMA
 * and the transmit descriptor is filled in.
 */
static int
ath_beacon_setup(struct ath_softc *sc, struct ath_buf *bf, struct sk_buff *skb)
{
#define	USE_SHPREAMBLE(_ic) \
	(((_ic)->ic_flags & (IEEE80211_F_SHPREAMBLE | IEEE80211_F_USEBARKER))\
		== IEEE80211_F_SHPREAMBLE)
#define	MIN(a,b)	((a) < (b) ? (a) : (b))
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = bf->bf_node;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_node *an = ATH_NODE(ni);
	struct ath_desc *ds;
	u_int8_t rate;

	bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
		skb->data, skb->len, PCI_DMA_TODEVICE);
	DPRINTF(ATH_DEBUG_BEACON, "%s: skb %p [data %p len %u] skbaddr %p\n",
		__func__, skb, skb->data, skb->len, (caddr_t) bf->bf_skbaddr);

	/* setup descriptors */
	ds = bf->bf_desc;

	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	/*
	 * Calculate rate code.
	 * XXX everything at min xmit rate
	 */
	if (USE_SHPREAMBLE(ic))
		rate = an->an_tx_mgtratesp;
	else
		rate = an->an_tx_mgtrate;
	ath_hal_setuptxdesc(ah, ds
		, skb->len + IEEE80211_CRC_LEN	/* frame length */
		, sizeof(struct ieee80211_frame)/* header length */
		, HAL_PKT_TYPE_BEACON		/* Atheros packet type */
		, MIN(ni->ni_txpower,60)	/* txpower XXX */
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
#undef MIN
#undef USE_SHPREAMBLE
}

/*
 * Set the slot time based on the current setting.
 */
static void
ath_setslottime(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		ath_hal_setslottime(ah, HAL_SLOT_TIME_9);
	else
		ath_hal_setslottime(ah, HAL_SLOT_TIME_20);
	sc->sc_updateslot = OK;
}

/*
 * Callback from the 802.11 layer to update the
 * slot time based on the current setting.
 */
static void
ath_updateslot(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	/*
	 * In station mode we change the hardware immediately.
	 * For other operation we defer the change until beacon
	 * updates have propagated to the stations.
	 */
	if (ic->ic_curmode == IEEE80211_M_STA)
		ath_setslottime(sc);
	else
		sc->sc_updateslot = UPDATE;
}

/*
 * Allocate and setup an initial beacon frame.
 */
static int
ath_beacon_alloc(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_buf *bf;
	struct sk_buff *skb;
	int error;

	bf = sc->sc_bcbuf;
	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
	}
	/*
	 * NB: the beacon data buffer must be 32-bit aligned;
	 * we assume the mbuf routines will return us something
	 * with this alignment (perhaps should assert).
	 */
	skb = ieee80211_beacon_alloc(ic, ni, &sc->sc_boff);
	if (skb == NULL) {
		DPRINTF(ATH_DEBUG_BEACON, "%s: cannot get sk_buff\n", __func__);
		sc->sc_stats.ast_be_nobuf++;
		return ENOMEM;
	}
	bf->bf_node = ni;
	error = ath_beacon_setup(sc, bf, skb);
	if (error == 0)
		bf->bf_skb = skb;
	else
		dev_kfree_skb(skb);
	return error;
}

/*
 * Transmit a beacon frame at SWBA.  Dynamic updates to the
 * frame contents are done as needed and the slot time is
 * also adjusted based on current state.
 */
static void
ath_beacon_tasklet(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_buf *bf = sc->sc_bcbuf;
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;

	DPRINTF(ATH_DEBUG_BEACON_PROC, "%s\n", __func__);
	if (ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_MONITOR ||
	    bf == NULL || bf->bf_skb == NULL) {
		DPRINTF(ATH_DEBUG_ANY, "%s: ic_flags=%x bf=%p bf_m=%p\n",
			 __func__, ic->ic_flags, bf, bf ? bf->bf_skb : NULL);
		return;
	}

	/*
	 * Update dynamic beacon contents.  If this returns non-zero
	 * then we need to update the descriptor state because the
	 * beacon frame changed size and/or was re-allocated.
	 */
	skb = bf->bf_skb;
	if (ieee80211_beacon_update(ic, bf->bf_node, &sc->sc_boff, &skb)) {
		/* NB: the old sk_buff is free'd */
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		bf->bf_skb = NULL;
		if (ath_beacon_setup(sc, bf, skb) != 0) {
			dev_kfree_skb(skb);
			/* XXX statistic */
			return;		/* XXX??? */
		}
		bf->bf_skb = skb;
	}

	/*
	 * Handle slot time change when a non-ERP station joins/leaves
	 * an 11g network.  The 802.11 layer notifies us via callback,
	 * we mark updateslot, then wait one beacon before effecting
	 * the change.  This gives associated stations at least one
	 * beacon interval to note the state change.
	 */
	/* XXX locking */
	if (sc->sc_updateslot == UPDATE)
		sc->sc_updateslot = COMMIT;	/* commit next beacon */
	else if (sc->sc_updateslot == COMMIT)
		ath_setslottime(sc);		/* commit change to h/w */

	/*
	 * Stop any current dma and put the new frame on the queue.
	 */
	if (!ath_hal_stoptxdma(ah, sc->sc_bhalq)) {
		DPRINTF(ATH_DEBUG_ANY, "%s: beacon queue %u did not stop?\n",
			__func__, sc->sc_bhalq);
		/* NB: the HAL still stops DMA, so proceed */
	}
	pci_dma_sync_single(sc->sc_pdev,
		bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);

	ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
	ath_hal_txstart(ah, sc->sc_bhalq);
	DPRINTF(ATH_DEBUG_BEACON_PROC, "%s: TXDP%u = %p (%p)\n", __func__,
		sc->sc_bhalq, (caddr_t)bf->bf_daddr, bf->bf_desc);
}

/*
 * Reclaim beacon resources.
 */
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
	struct ieee80211_node *ni = ic->ic_bss;
	u_int32_t nexttbtt, intval;
	
	nexttbtt = (LE_READ_4(ni->ni_tstamp + 4) << 22) |
	    (LE_READ_4(ni->ni_tstamp) >> 10);
	DPRINTF(ATH_DEBUG_BEACON, "%s: nexttbtt=%u\n", __func__, nexttbtt);
	nexttbtt += ni->ni_intval;
	intval = ni->ni_intval & HAL_BEACON_PERIOD;
	if (ic->ic_opmode == IEEE80211_M_STA) {
		HAL_BEACON_STATE bs;
		u_int32_t bmisstime;

		/* NB: no PCF support right now */
		memset(&bs, 0, sizeof(bs));
		/*
		 * Reset our tsf so the hardware will update the
		 * tsf register to reflect timestamps found in
		 * received beacons.
		 */
		bs.bs_intval = intval | HAL_BEACON_RESET_TSF;
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

		DPRINTF(ATH_DEBUG_BEACON, 
			"%s: intval %u nexttbtt %u dtim %u nextdtim %u bmiss %u sleep %u\n"
			, __func__
			, bs.bs_intval
			, bs.bs_nexttbtt
			, bs.bs_dtimperiod
			, bs.bs_nextdtim
			, bs.bs_bmissthreshold
			, bs.bs_sleepduration
		);
		ath_hal_intrset(ah, 0);
		ath_hal_beacontimers(ah, &bs, 0/*XXX*/, 0, 0);
		sc->sc_imask |= HAL_INT_BMISS;
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
		DPRINTF(ATH_DEBUG_BEACON, "%s: intval %u nexttbtt %u\n",
			__func__, ni->ni_intval, nexttbtt);
		ath_hal_intrset(ah, 0);
		if (nexttbtt == ni->ni_intval)
			intval |= HAL_BEACON_RESET_TSF;
		if (ic->ic_opmode != IEEE80211_M_MONITOR) {
			intval |= HAL_BEACON_ENA;
			sc->sc_imask |= HAL_INT_SWBA;	/* beacon prepare */
		}
		ath_hal_beaconinit(ah, nexttbtt, intval);
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
	DPRINTF(ATH_DEBUG_ANY, "%s: DMA map: %p (%d) -> %p\n",
	    __func__, ds, sc->sc_desc_len, (caddr_t) sc->sc_desc_daddr);

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
	kfree(sc->sc_bufptr);
	sc->sc_bufptr = NULL;
}

static struct ieee80211_node *
ath_node_alloc(struct ieee80211com *ic)
{
	struct ath_node *an = kmalloc(sizeof(struct ath_node), GFP_ATOMIC);
	if (an) {
		struct ath_softc *sc = ic->ic_dev->priv;
		int i;

		memset(an, 0, sizeof(struct ath_node));
		for (i = 0; i < ATH_RHIST_SIZE; i++)
			an->an_rx_hist[i].arh_ticks = ATH_RHIST_NOTIME;
		an->an_rx_hist_next = ATH_RHIST_SIZE-1;
		ath_rate_update(sc, &an->an_node, 0);
		return &an->an_node;
	} else
		return NULL;
}

/*
 * Clear any references to a node in a transmit queue.
 * This happens when the node is removed so we don't
 * need to worry about reclaiming reference counts; we
 * just null the pointer and the right thing will happen
 * when the buffer is cleaned.
 */
static void
ath_tx_cleanq(struct ath_txq *txq, struct ieee80211_node *ni)
{
	struct ath_buf *bf;

	ATH_TXQ_LOCK_BH(txq);
	TAILQ_FOREACH(bf, &txq->axq_q, bf_list) {
		if (bf->bf_node == ni)
			bf->bf_node = NULL;
	}
	ATH_TXQ_UNLOCK_BH(txq);
}

static void
ath_node_free(struct ieee80211com *ic, struct ieee80211_node *ni)
{
        struct ath_softc *sc = ic->ic_dev->priv;
	int i;

	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_cleanq(&sc->sc_txq[i], ni);
	(*sc->sc_node_free)(ic, ni);
}

static void
ath_node_copy(struct ieee80211com *ic,
	struct ieee80211_node *dst, const struct ieee80211_node *src)
{
        struct ath_softc *sc = ic->ic_dev->priv;

	memcpy(&dst[1], &src[1],
		sizeof(struct ath_node) - sizeof(struct ieee80211_node));
	(*sc->sc_node_copy)(ic, dst, src);
}

static u_int8_t
ath_node_getrssi(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	struct ath_node *an = ATH_NODE(ni);
	int i, now, nsamples, rssi;

	/*
	 * Calculate the average over the last second of sampled data.
	 */
	now = jiffies;
	nsamples = 0;
	rssi = 0;
	i = an->an_rx_hist_next;
	do {
		struct ath_recv_hist *rh = &an->an_rx_hist[i];
		if (rh->arh_ticks == ATH_RHIST_NOTIME)
			goto done;
		if (now - rh->arh_ticks > 1*HZ)
			goto done;
		rssi += rh->arh_rssi;
		nsamples++;
		if (i == 0)
			i = ATH_RHIST_SIZE-1;
		else
			i--;
	} while (i != an->an_rx_hist_next);
done:
	/*
	 * Return either the average or the last known
	 * value if there is no recent data.
	 */
	if (nsamples)
		rssi /= nsamples;
	else {
		rssi = an->an_rx_hist[i].arh_rssi;
		/*
		 * XXX Nodes created during scans may only have one
		 * sample and this isn't entered into the history
		 * buffer so we need to get it directly from the node
		 * structure.
		 */
		if (rssi == 0)
			rssi = ni->ni_rssi;
	}
	return rssi;
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
 				DPRINTF(ATH_DEBUG_ANY,
					"%s: skbuff alloc of size %u failed\n",
					__func__,
					sc->sc_rxbufsize
					+ sizeof(wlan_ng_prism2_header)
					+ sc->sc_cachelsz -1);
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
				DPRINTF(ATH_DEBUG_ANY,
					"%s: skbuff alloc of size %u failed\n",
					__func__, sc->sc_rxbufsize);
				sc->sc_stats.ast_rx_nobuf++;
				return ENOMEM;
			}
		}
		skb->dev = &sc->sc_dev;
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
	int rate = (rt != NULL ?
	    (rt->info[rt->rateCodeToIndex[
	    	ds->ds_rxstat.rs_rate]].dot11Rate & IEEE80211_RATE_VAL) : 2);
 	wlan_ng_prism2_header *ph;

	skb->protocol = ETH_P_CONTROL;
	skb->pkt_type = PACKET_OTHERHOST;
	skb_put(skb, len);
	
	/* Check to see if we need to remove pad bytes */
	if (ic->ic_flags & IEEE80211_F_DATAPAD) {
		struct ieee80211_frame *wh =
			(struct ieee80211_frame *) skb->data;

		if (((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
		    IEEE80211_FC0_TYPE_DATA) &&
		    (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS)) {
			int headersize = sizeof(struct ieee80211_qosframe);
			int padbytes;

			if ((wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) ==
			    IEEE80211_FC1_DIR_DSTODS)
				headersize += IEEE80211_ADDR_LEN;
			padbytes = roundup(headersize,4) - headersize;
			if (padbytes > 0) {
				uint8_t qos[sizeof(struct ieee80211_qosframe) +
					IEEE80211_ADDR_LEN];
				memcpy(&qos[0], skb->data, headersize);
				skb_pull(skb, padbytes);
				memcpy(skb->data, &qos[0], headersize);
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
ath_rx_tasklet(TQUEUE_ARG data)
{
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_desc + \
		((_pa) - (_sc)->sc_desc_daddr)))
	struct net_device *dev = (struct net_device *)data;
	struct ath_buf *bf;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	struct sk_buff *skb;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	struct ath_node *an;
	struct ath_recv_hist *rh;
	int len;
	u_int phyerr;
	HAL_STATUS status;

	DPRINTF(ATH_DEBUG_RX_PROC, "%s\n", __func__);
	do {
		bf = TAILQ_FIRST(&sc->sc_rxbuf);
		if (bf == NULL) {		/* XXX ??? can this happen */
			printk("%s: no buffer (%s)\n", dev->name, __func__);
			break;
		}
		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at the end */
			break;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {		/* XXX ??? can this happen */
			printk("%s: no skbuff (%s)\n", dev->name, __func__);
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
		if (ath_debug & ATH_DEBUG_RECV_DESC)
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
			 * discard the frame.
			 */
#ifndef ERROR_FRAMES
			/*
			 * Enable this if you want to see
			 * error frames in Monitor mode.
			 */
			if (ic->ic_opmode != IEEE80211_M_MONITOR) {
				sc->sc_stats.ast_rx_toobig++;
				goto rx_next;
			}
#endif
			/* fall thru for monitor mode handling... */
		} else if (ds->ds_rxstat.rs_status != 0) {
			if (ds->ds_rxstat.rs_status & HAL_RXERR_CRC)
				sc->sc_stats.ast_rx_crcerr++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_FIFO)
				sc->sc_stats.ast_rx_fifoerr++;
			if (ds->ds_rxstat.rs_status & HAL_RXERR_PHY) {
				sc->sc_stats.ast_rx_phyerr++;
				phyerr = ds->ds_rxstat.rs_phyerr & 0x1f;
				sc->sc_stats.ast_rx_phy[phyerr]++;
				goto rx_next;
			}
			if (ds->ds_rxstat.rs_status & HAL_RXERR_DECRYPT) {
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
				if (ds->ds_rxstat.rs_keyix == HAL_RXKEYIX_INVALID)
					goto rx_accept;
				sc->sc_stats.ast_rx_badcrypt++;
			}
			if (ds->ds_rxstat.rs_status & HAL_RXERR_MIC) {
				sc->sc_stats.ast_rx_badmic++;
				/*
				 * Do minimal work required to hand off
				 * the 802.11 header for notifcation.
				 */
				/* XXX frag's and qos frames */
				len = ds->ds_rxstat.rs_datalen;
				if (len >= sizeof (struct ieee80211_frame)) {
					pci_dma_sync_single(sc->sc_pdev,
					    bf->bf_skbaddr, len,
					    PCI_DMA_FROMDEVICE);
					ieee80211_notify_michael_failure(ic,
						(struct ieee80211_frame *)
						    skb->data,
						ds->ds_rxstat.rs_keyix);
				}
			}
			/*
			 * Reject error frames, we normally don't want
			 * to see them in monitor mode (in monitor mode
			 * allow through packets that have crypto problems).
			 */
			if ((ds->ds_rxstat.rs_status &~
				(HAL_RXERR_DECRYPT|HAL_RXERR_MIC)) ||
			    sc->sc_ic.ic_opmode != IEEE80211_M_MONITOR)
				goto rx_next;
		}
rx_accept:
		len = ds->ds_rxstat.rs_datalen;
		if (len < IEEE80211_MIN_LEN) {
			DPRINTF(ATH_DEBUG_RECV, "%s: short packet %d\n",
				__func__, len);
			sc->sc_stats.ast_rx_tooshort++;
			goto rx_next;
		}

		/*
		 * Sync and unmap the frame.  At this point we're
		 * committed to passing the sk_buff somewhere so
		 * clear buf_skb; this means a new sk_buff must be
		 * allocated when the rx descriptor is setup again
		 * to receive another frame.
		 */
		pci_dma_sync_single(sc->sc_pdev,
			bf->bf_skbaddr, len, PCI_DMA_FROMDEVICE);
		pci_unmap_single(sc->sc_pdev, bf->bf_skbaddr,
			sc->sc_rxbufsize, PCI_DMA_FROMDEVICE);
		bf->bf_skb = NULL;

		if (sc->sc_softled)
			ath_update_led(sc);

		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
			/*
			 * Monitor mode: clean the skbuff, fabricate
			 * the Prism header existing tools expect,
			 * and dispatch.
			 */
			ath_rx_capture(dev, ds, skb);
			goto rx_next;
		}

		/*
		 * Normal receive.
		 */
		skb_put(skb, len);
		skb->protocol = ETH_P_CONTROL;		/* XXX */

		if (IFF_DUMPPKTS(ic, ATH_DEBUG_RECV)) {
			ieee80211_dump_pkt(skb->data, len,
				   sc->sc_hwmap[ds->ds_rxstat.rs_rate] &
				   	IEEE80211_RATE_VAL,
				   ds->ds_rxstat.rs_rssi);
		}

		skb_trim(skb, skb->len - IEEE80211_CRC_LEN);
		/* XXX move up to ieee80211_input */
		wh = (struct ieee80211_frame *) skb->data;

		/*
		 * Locate the node for sender, track state, and then
		 * pass the (referenced) node up to the 802.11 layer
		 * for its use.  We are required to pass some node so
		 * we fall back to ic_bss when this frame is from an
		 * unknown sender.  The 802.11 layer knows this means the
		 * sender wasn't in the node table and acts accordingly.
		 * Note also that by convention we do not reference
		 * count ic_bss, only other nodes (ic_bss is never free'd).
		 */
		if (ic->ic_opmode != IEEE80211_M_STA) {
			ni = ieee80211_find_node(ic, wh->i_addr2);
			if (ni == NULL)
				ni = ic->ic_bss;
		} else
			ni = ic->ic_bss;

		/*
		 * Record driver-specific state.
		 */
		an = ATH_NODE(ni);
		if (++(an->an_rx_hist_next) == ATH_RHIST_SIZE)
			an->an_rx_hist_next = 0;
		rh = &an->an_rx_hist[an->an_rx_hist_next];
		rh->arh_ticks = jiffies;
		rh->arh_rssi = ds->ds_rxstat.rs_rssi;
		rh->arh_antenna = ds->ds_rxstat.rs_antenna;

		/*
		 * Send frame up for processing.
		 */
		ieee80211_input(ic, skb, ni,
			ds->ds_rxstat.rs_rssi, ds->ds_rxstat.rs_tstamp);

		/*
		 * Reclaim node reference.
		 */
		if (ni != ic->ic_bss)
			ieee80211_free_node(ic, ni);
rx_next:
		TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
	} while (ath_rxbuf_init(sc, bf) == 0);

	ath_hal_rxmonitor(ah);			/* rx signal state monitoring */
#if 0
	/* XXX with the final descriptor self-linked this is unneeded */
	ath_hal_rxena(ah);			/* in case of RXEOL */
#endif
#undef PA2DESC
}

/*
 * Setup a hardware data transmit queue for the specified
 * access control.  The hal may not support all requested
 * queues in which case it will return a reference to a
 * previously setup queue.  We record the mapping from ac's
 * to h/w queues for use by ath_tx_start and also track
 * the set of h/w queues being used to optimize work in the
 * transmit interrupt handler and related routines.
 */
static int
ath_tx_setup(struct ath_softc *sc, int ac, int haltype)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;
	int qnum;

	if (ac >= N(sc->sc_ac2q)) {
		printk("%s: AC %u out of range, max %u!\n",
			sc->sc_dev.name, ac, N(sc->sc_ac2q));
		return 0;
	}
	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = haltype;
	qnum = ath_hal_setuptxqueue(ah, HAL_TX_QUEUE_DATA, &qi);
	if (qnum == -1) {
		printk("%s: Unable to setup hardware queue for %s traffic!\n",
			sc->sc_dev.name, acnames[ac]);
		return 0;
	}
	if (qnum >= N(sc->sc_txq)) {
		printk("%s: hal qnum %u out of range, max %u!\n",
			sc->sc_dev.name, qnum, N(sc->sc_txq));
		return 0;
	}
	if (!ATH_TXQ_SETUP(sc, qnum)) {
		struct ath_txq *txq = &sc->sc_txq[qnum];

		txq->axq_qnum = qnum;
		txq->axq_link = NULL;
		TAILQ_INIT(&txq->axq_q);
		ATH_TXQ_LOCK_INIT(txq);
		sc->sc_txqsetup |= 1<<qnum;
	}
	sc->sc_ac2q[ac] = &sc->sc_txq[qnum];
	return 1;
#undef N
}

/*
 * XXX Size of an ACK control frame in bytes.
 */
#define	IEEE80211_ACK_SIZE	(2+2+IEEE80211_ADDR_LEN+4)

static int
ath_tx_start(struct net_device *dev, struct ieee80211_node *ni, struct ath_buf *bf,
    struct sk_buff *skb)
{
#define	MIN(a,b)	((a) < (b) ? (a) : (b))
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	int iswep, ismcast, keyix, hdrlen, pktlen, try0;
	u_int8_t rix, txrate, ctsrate;
	u_int8_t cix = 0xff;		/* NB: silence compiler */
	struct ath_desc *ds;
	struct ath_txq *txq;
	struct ieee80211_frame *wh;
	u_int subtype, flags, ctsduration, antenna;
	HAL_PKT_TYPE atype;
	const HAL_RATE_TABLE *rt;
	HAL_BOOL shortPreamble;
	struct ath_node *an;

	wh = (struct ieee80211_frame *) skb->data;
	iswep = wh->i_fc[1] & IEEE80211_FC1_WEP;
	ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
	hdrlen = sizeof(struct ieee80211_frame);
	pktlen = skb->len;
	/*
	 * Select hardware transmit queue and adjust header
	 * length if this is a QoS frame.  Default all non-QoS
	 * traffic to the background queue.  Management frames
	 * get reassigned below.
	 */
	if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
		/* XXX validate skb->priority */
		txq = sc->sc_ac2q[skb->priority];
		hdrlen += sizeof(u_int16_t);
	} else
		txq = sc->sc_ac2q[WME_AC_BK];

	if (iswep) {
		const struct ieee80211_cipher *cip;
		struct ieee80211_key *k;

		/*
		 * Construct the 802.11 header+trailer for an encrypted
		 * frame. The only reason this can fail is because of an
		 * unknown or unsupported cipher/key type.
		 */
		k = ieee80211_crypto_encap(ic, ni, skb);
		if (k == NULL) {
			/*
			 * This can happen when the key is yanked after the
			 * frame was queued.  Just discard the frame; the
			 * 802.11 layer counts failures and provides
			 * debugging/diagnostics.
			 */
			return -EIO;
		}
		/*
		 * Adjust the packet + header lengths for the crypto
		 * additions and calculate the h/w key index.  When
		 * a s/w mic is done the frame will have had any mic
		 * added to it prior to entry so skb->len above will
		 * account for it. Otherwise we need to add it to the
		 * packet length.
		 */
		cip = k->wk_cipher;
		hdrlen += cip->ic_header;
		pktlen += cip->ic_header + cip->ic_trailer;
		if ((k->wk_flags & IEEE80211_KEY_SWMIC) == 0)
			pktlen += cip->ic_miclen;
		keyix = k->wk_keyix;

		/* packet header may have moved, reset our local pointer */
		wh = (struct ieee80211_frame *) skb->data;
	} else
		keyix = HAL_TXKEYIX_INVALID;

	pktlen += IEEE80211_CRC_LEN;

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
	bf->bf_skbaddr = pci_map_single(sc->sc_pdev,
		skb->data, pktlen, PCI_DMA_TODEVICE);
	DPRINTF(ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %x\n",
		__func__, skb, skb->data, skb->len, bf->bf_skbaddr);
	bf->bf_skb = skb;
	bf->bf_node = ni;

	/* setup descriptors */
	ds = bf->bf_desc;
	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	/*
	 * NB: the 802.11 layer marks whether or not we should
	 * use short preamble based on the current mode and
	 * negotiated parameters.
	 */
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)) {
		shortPreamble = AH_TRUE;
		sc->sc_stats.ast_tx_shortpre++;
	} else {
		shortPreamble = AH_FALSE;
	}

	an = ATH_NODE(ni);
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
		try0 = ATH_TXMAXTRY;
		if (shortPreamble)
			txrate = an->an_tx_mgtratesp;
		else
			txrate = an->an_tx_mgtrate;
		/* NB: force all management frames to highest queue */
		txq = sc->sc_ac2q[WME_AC_VO];
		break;
	case IEEE80211_FC0_TYPE_CTL:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_PS_POLL)
			atype = HAL_PKT_TYPE_PSPOLL;
		rix = 0;			/* XXX lowest rate */
		try0 = ATH_TXMAXTRY;
		if (shortPreamble)
			txrate = an->an_tx_mgtratesp;
		else
			txrate = an->an_tx_mgtrate;
		break;
	default:
		rix = an->an_tx_rix0;
		try0 = an->an_tx_try0;
		if (rix == 0xff) {
			printk("%s: bogus xmit rate 0x%x (%s)\n", dev->name,
				ni->ni_rates.rs_rates[ni->ni_txrate], __func__);
			sc->sc_stats.ast_tx_badrate++;
			return -EIO;
		}
		if (shortPreamble)
			txrate = an->an_tx_rate0sp;
		else
			txrate = an->an_tx_rate0;
		break;
	}

	/*
	 * Calculate miscellaneous flags.
	 */
	flags = HAL_TXDESC_CLRDMASK;		/* XXX needed for crypto errs */
	if (ismcast) {
		flags |= HAL_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
	} else if (pktlen > ic->ic_rtsthreshold) {
		flags |= HAL_TXDESC_RTSENA;	/* RTS based on frame length */
		cix = rt->info[rix].controlRate;
		sc->sc_stats.ast_tx_rts++;
	}

	/*
	 * If 802.11g protection is enabled, determine whether
	 * to use RTS/CTS or just CTS.  Note that this is only
	 * done for OFDM unicast frames.
	 */
	if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
	    rt->info[rix].phy == IEEE80211_T_OFDM &&
	    (flags & HAL_TXDESC_NOACK) == 0) {
		/* XXX fragments must use CCK rates w/ protection */
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
			flags |= HAL_TXDESC_RTSENA;
		else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
			flags |= HAL_TXDESC_CTSENA;
		cix = rt->info[sc->sc_protrix].controlRate;
		sc->sc_stats.ast_tx_protect++;
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
		/* XXX optimize a la data transmit rate */
		/*
		 * CTS transmit rate is derived from the transmit rate
		 * by looking in the h/w rate table.  We must also factor
		 * in whether or not a short preamble is to be used.
		 */
		/* NB: cix is set above where RTS/CTS is enabled */
		KASSERT(cix != 0xff, ("cix not setup"));
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
		/*
		 * Must disable multi-rate retry when using RTS/CTS.
		 */
		try0 = ATH_TXMAXTRY;
	} else
		ctsrate = 0;

	/*
	 * For now use the antenna on which the last good
	 * frame was received on.  We assume this field is
	 * initialized to 0 which gives us ``auto'' or the
	 * ``default'' antenna.
	 */
	if (an->an_tx_antenna)
		antenna = an->an_tx_antenna;
	else
		antenna = an->an_rx_hist[an->an_rx_hist_next].arh_antenna;

	if (IFF_DUMPPKTS(ic, ATH_DEBUG_XMIT))
		ieee80211_dump_pkt(skb->data, skb->len,
		    ni->ni_rates.rs_rates[ni->ni_txrate] & IEEE80211_RATE_VAL,
		    -1);

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	/* XXX check return value? */
	ath_hal_setuptxdesc(ah, ds
		, pktlen		/* packet length */
		, hdrlen		/* header length */
		, atype			/* Atheros packet type */
		, MIN(ni->ni_txpower,60)/* txpower */
		, txrate, try0		/* series 0 rate/tries */
		, keyix			/* key cache index */
		, antenna		/* antenna mode */
		, flags			/* flags */
		, ctsrate		/* rts/cts rate */
		, ctsduration		/* rts/cts duration */
	);
	/*
	 * Setup the multi-rate retry state only when we're
	 * going to use it.  This assumes ath_hal_setuptxdesc
	 * initializes the descriptors (so we don't have to)
	 * when the hardware supports multi-rate retry and
	 * we don't use it.
	 */
	if (try0 != ATH_TXMAXTRY)
		ath_hal_setupxtxdesc(ah, ds
			, an->an_tx_rate1sp, 2	/* series 1 */
			, an->an_tx_rate2sp, 2	/* series 2 */
			, an->an_tx_rate3sp, 2	/* series 3 */
		);

	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath_hal_filltxdesc(ah, ds
		, skb->len		/* segment length */
		, AH_TRUE		/* first segment */
		, AH_TRUE		/* last segment */
	);
	DPRINTF(ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
	    __func__, txq->axq_qnum, ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);

	/*
	 * Insert the frame on the outbound list and
	 * pass it on to the hardware.
	 */
	ATH_TXQ_LOCK_BH(txq);
	TAILQ_INSERT_TAIL(&txq->axq_q, bf, bf_list);
	if (txq->axq_link == NULL) {
		ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
		DPRINTF(ATH_DEBUG_XMIT, "%s: TXDP0[%u] = %p (%p)\n", __func__,
		    txq->axq_qnum, (caddr_t)bf->bf_daddr, bf->bf_desc);
	} else {
		*txq->axq_link = bf->bf_daddr;
		DPRINTF(ATH_DEBUG_XMIT, "%s: link[%u](%p)=%p (%p)\n", __func__,
		    txq->axq_qnum, txq->axq_link,
		    (caddr_t)bf->bf_daddr, bf->bf_desc);
	}
	txq->axq_link = &ds->ds_link;
	ATH_TXQ_UNLOCK_BH(txq);

	if (sc->sc_softled)
		ath_update_led(sc);

	ath_hal_txstart(ah, txq->axq_qnum);
	dev->trans_start = jiffies;
	return 0;
#undef MIN
}

/*
 * Process completed xmit descriptors from the specified queue.
 */
static void
ath_tx_processq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct ieee80211_node *ni;
	struct ath_node *an;
	int sr, lr;
	HAL_STATUS status;

	DPRINTF(ATH_DEBUG_TX_PROC, "%s: tx queue %p, link %p\n", __func__,
		(caddr_t) ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum),
		txq->axq_link);
	for (;;) {
		ATH_TXQ_LOCK(txq);
		bf = TAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK(txq);
			break;
		}
		ds = bf->bf_desc;		/* NB: last decriptor */
		status = ath_hal_txprocdesc(ah, ds);
#ifdef AR_DEBUG
		if (ath_debug & ATH_DEBUG_XMIT_DESC)
			ath_printtxbuf(bf, status == HAL_OK);
#endif
		if (status == HAL_EINPROGRESS) {
			ATH_TXQ_UNLOCK(txq);
			break;
		}
		TAILQ_REMOVE(&txq->axq_q, bf, bf_list);
		ATH_TXQ_UNLOCK(txq);

		ni = bf->bf_node;
		if (ni != NULL) {
			an = ATH_NODE(ni);
			if (ds->ds_txstat.ts_status == 0) {
				an->an_tx_ok++;
				an->an_tx_antenna = ds->ds_txstat.ts_antenna;
				if (ds->ds_txstat.ts_rate & HAL_TXSTAT_ALTRATE)
					sc->sc_stats.ast_tx_altrate++;
				sc->sc_stats.ast_tx_rssidelta =
					ds->ds_txstat.ts_rssi -
					sc->sc_stats.ast_tx_rssi;
				sc->sc_stats.ast_tx_rssi =
					ds->ds_txstat.ts_rssi;
			} else {
				an->an_tx_err++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_XRETRY)
					sc->sc_stats.ast_tx_xretries++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_FIFO)
					sc->sc_stats.ast_tx_fifoerr++;
				if (ds->ds_txstat.ts_status & HAL_TXERR_FILT)
					sc->sc_stats.ast_tx_filtered++;
				an->an_tx_antenna = 0;	/* invalidate */
			}
			sr = ds->ds_txstat.ts_shortretry;
			lr = ds->ds_txstat.ts_longretry;
			sc->sc_stats.ast_tx_shortretry += sr;
			sc->sc_stats.ast_tx_longretry += lr;
			an->an_tx_retr += sr + lr;
			/*
			 * Reclaim reference to node.
			 *
			 * NB: the node may be reclaimed here if, for example
			 *     this is a DEAUTH message that was sent and the
			 *     node was timed out due to inactivity.
			 */
			if (ni != ic->ic_bss)
				ieee80211_free_node(ic, ni);
		}
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		ATH_TXBUF_LOCK(sc);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK(sc);
	}
}

/*
 * Deferred processing of transmit interrupt; special-cased
 * for a single hardware transmit queue (e.g. 5210 and 5211).
 */
static void
ath_tx_tasklet_q0(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	ath_tx_processq(sc, &sc->sc_txq[0]);
	/*
	 * Don't wakeup unless we're associated; this insures we don't
	 * signal the upper layer it's ok to start sending data frames.
	 */
	/* XXX use a low watermark to reduce wakeups */
	if (ic->ic_state == IEEE80211_S_RUN)
		netif_wake_queue(dev);
}

/*
 * Deferred processing of transmit interrupt; special-cased
 * for four hardware queues, 0-3 (e.g. 5212 w/ WME support).
 */
static void
ath_tx_tasklet_q0123(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	/*
	 * Process each active queue.
	 */
	ath_tx_processq(sc, &sc->sc_txq[0]);
	ath_tx_processq(sc, &sc->sc_txq[1]);
	ath_tx_processq(sc, &sc->sc_txq[2]);
	ath_tx_processq(sc, &sc->sc_txq[3]);
	/*
	 * Don't wakeup unless we're associated; this insures we don't
	 * signal the upper layer it's ok to start sending data frames.
	 */
	/* XXX use a low watermark to reduce wakeups */
	if (ic->ic_state == IEEE80211_S_RUN)
		netif_wake_queue(dev);
}

/*
 * Deferred processing of transmit interrupt.
 */
static void
ath_tx_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int i;

	/*
	 * Process each active queue.
	 */
	/* XXX faster to read ISR_S0_S and ISR_S1_S to determine q's? */
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_processq(sc, &sc->sc_txq[i]);
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
	ath_init(dev);
}

static void
ath_tx_draintxq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_buf *bf;

	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath_tx_tasklet
	 */
	for (;;) {
		ATH_TXQ_LOCK(txq);
		bf = TAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK(txq);
			break;
		}
		TAILQ_REMOVE(&txq->axq_q, bf, bf_list);
		ATH_TXQ_UNLOCK(txq);
#ifdef AR_DEBUG
		if (ath_debug & ATH_DEBUG_RESET)
			ath_printtxbuf(bf,
				ath_hal_txprocdesc(ah, bf->bf_desc) == HAL_OK);
#endif /* AR_DEBUG */
		pci_unmap_single(sc->sc_pdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		ni = bf->bf_node;
		bf->bf_node = NULL;
		if (ni != NULL && ni != ic->ic_bss) {
			/*
			 * Reclaim node reference.
			 */
			ieee80211_free_node(ic, ni);
		}
		ATH_TXBUF_LOCK(sc);
		TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK(sc);
	}
}

static void
ath_tx_stopdma(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;

	(void) ath_hal_stoptxdma(ah, txq->axq_qnum);
	DPRINTF(ATH_DEBUG_RESET, "%s: tx queue [%u] %p, link %p\n",
	    __func__, txq->axq_qnum,
	    (caddr_t) ath_hal_gettxbuf(ah, txq->axq_qnum), txq->axq_link);
}

/*
 * Drain the transmit queues and reclaim resources.
 */
static void
ath_draintxq(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	int i;

	/* XXX return value */
	if (!sc->sc_invalid) {
		(void) ath_hal_stoptxdma(ah, sc->sc_bhalq);
		DPRINTF(ATH_DEBUG_RESET, "%s: beacon queue %p\n", __func__,
		    (caddr_t) ath_hal_gettxbuf(ah, sc->sc_bhalq));
		for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
			if (ATH_TXQ_SETUP(sc, i))
				ath_tx_stopdma(sc, &sc->sc_txq[i]);
	}
	sc->sc_dev.trans_start = jiffies;
	netif_start_queue(&sc->sc_dev);
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_draintxq(sc, &sc->sc_txq[i]);
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
	mdelay(3);			/* 3ms is long enough for 1 frame */
#ifdef AR_DEBUG
	if (ath_debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL)) {
		struct ath_buf *bf;

		printk("ath_stoprecv: rx queue %p, link %p\n",
		    (caddr_t) ath_hal_getrxbuf(ah), sc->sc_rxlink);
		TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			struct ath_desc *ds = bf->bf_desc;
			HAL_STATUS status = ath_hal_rxprocdesc(ah, ds,
				bf->bf_daddr, PA2DESC(sc, ds->ds_link));
			if (status == HAL_OK || (ath_debug & ATH_DEBUG_FATAL))
				ath_printrxbuf(bf, status == HAL_OK);
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
ath_startrecv(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = ic->ic_dev;
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
	DPRINTF(ATH_DEBUG_RESET, "%s: mtu %u cachelsz %u rxbufsize %u\n",
		__func__, dev->mtu, sc->sc_cachelsz, sc->sc_rxbufsize);

	sc->sc_rxlink = NULL;
	TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		if (error != 0)
			return error;
	}

	bf = TAILQ_FIRST(&sc->sc_rxbuf);
	ath_hal_putrxbuf(ah, bf->bf_daddr);
	ath_hal_rxena(ah);		/* enable recv descriptors */
	ath_mode_init(&sc->sc_dev);	/* set filters, etc. */
	ath_hal_startpcurecv(ah);	/* re-enable PCU/DMA engine */
	return 0;
}

/* 
 * Update internal state after a channel change.
 */
static void
ath_chan_change(struct ath_softc *sc, struct ieee80211_channel *chan)
{
	struct ieee80211com *ic = &sc->sc_ic;
	enum ieee80211_phymode mode;

	/*
	 * Change channels and update the h/w rate map
	 * if we're switching; e.g. 11a to 11b/g.
	 */
	mode = ieee80211_chan2mode(ic, chan);
	if (mode != sc->sc_curmode)
		ath_setcurmode(sc, mode);
#ifdef notyet
	/*
	 * Update BPF state.
	 */
	sc->sc_tx_th.wt_chan_freq = sc->sc_rx_th.wr_chan_freq =
		htole16(chan->ic_freq);
	sc->sc_tx_th.wt_chan_flags = sc->sc_rx_th.wr_chan_flags =
		htole16(chan->ic_flags);
#endif
}

/*
 * Set/change channels.  If the channel is really being changed,
 * it's done by reseting the chip.  To accomplish this we must
 * first cleanup any pending DMA, then restart stuff after a la
 * ath_init.
 */
static int
ath_chan_set(struct ath_softc *sc, struct ieee80211_channel *chan)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = ic->ic_dev;
	HAL_CHANNEL hchan;

	/*
	 * Convert to a HAL channel description with
	 * the flags constrained to reflect the current
	 * operating mode.
	 */
	hchan.channel = chan->ic_freq;
	hchan.channelFlags = ath_chan2flags(ic, chan);

	DPRINTF(ATH_DEBUG_RESET, "%s: %u (%u MHz) -> %u (%u MHz)\n", __func__,
	    ath_hal_mhz2ieee(sc->sc_curchan.channel,
		sc->sc_curchan.channelFlags),
	    	sc->sc_curchan.channel,
	    ath_hal_mhz2ieee(hchan.channel, hchan.channelFlags), hchan.channel);
	if (hchan.channel != sc->sc_curchan.channel ||
	    hchan.channelFlags != sc->sc_curchan.channelFlags) {
		HAL_STATUS status;

		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		if (!ath_hal_reset(ah, ic->ic_opmode, &hchan, AH_TRUE, &status)) {
			printk("%s: %s: unable to reset channel %u (%uMhz) "
				"hal status %u\n", dev->name, __func__,
				ieee80211_chan2ieee(ic, chan), chan->ic_freq,
				status);
			return EIO;
		}
		sc->sc_curchan = hchan;

		/*
		 * Re-enable rx framework.
		 */
		if (ath_startrecv(sc) != 0) {
			printk("%s: %s: unable to restart recv logic\n",
				dev->name, __func__);
			return EIO;
		}

		/*
		 * Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g.
		 */
		ic->ic_ibss_chan = chan;
		ath_chan_change(sc, chan);

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
		ieee80211_next_scan(ic);
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

	sc->sc_stats.ast_per_cal++;

	DPRINTF(ATH_DEBUG_CALIBRATE, "%s: channel %u/%x\n",
		__func__, sc->sc_curchan.channel, sc->sc_curchan.channelFlags);

	if (ath_hal_getrfgain(ah) == HAL_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		sc->sc_stats.ast_per_rfgain++;
		ath_reset(dev);
	}
	if (!ath_hal_calibrate(ah, &sc->sc_curchan)) {
		DPRINTF(ATH_DEBUG_ANY, "%s: calibration of channel %u failed\n",
			__func__, sc->sc_curchan.channel);
		sc->sc_stats.ast_per_calfail++;
	}
	sc->sc_cal_ch.expires = jiffies + (ath_calinterval * HZ);
	add_timer(&sc->sc_cal_ch);
}

static int
ath_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni;
	int i, error;
	u_int8_t *bssid;
	u_int32_t rfilt;
	static const HAL_LED_STATE leds[] = {
	    HAL_LED_INIT,	/* IEEE80211_S_INIT */
	    HAL_LED_SCAN,	/* IEEE80211_S_SCAN */
	    HAL_LED_AUTH,	/* IEEE80211_S_AUTH */
	    HAL_LED_ASSOC, 	/* IEEE80211_S_ASSOC */
	    HAL_LED_RUN, 	/* IEEE80211_S_RUN */
	};

	DPRINTF(ATH_DEBUG_STATE, "%s: %s -> %s\n", __func__,
		ieee80211_state_name[ic->ic_state],
		ieee80211_state_name[nstate]);

	del_timer(&sc->sc_scan_ch);		/* ap/neighbor scan timer */
	del_timer(&sc->sc_cal_ch);		/* periodic calibration timer */
	ath_hal_setledstate(ah, leds[nstate]);	/* set LED */
	netif_stop_queue(dev);			/* before we do anything else */

	if (nstate == IEEE80211_S_INIT) {
		del_timer(&sc->sc_rate_ctl);
		sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
		ath_hal_intrset(ah, sc->sc_imask);
		goto done;
	}
	ni = ic->ic_bss;
	error = ath_chan_set(sc, ni->ni_chan);
	if (error != 0)
		goto bad;
	rfilt = ath_calcrxfilter(sc);
	if (nstate == IEEE80211_S_SCAN)
		bssid = dev->broadcast;
	else
		bssid = ni->ni_bssid;
	ath_hal_setrxfilter(ah, rfilt);
	DPRINTF(ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s\n",
		 __func__, rfilt, ether_sprintf(bssid));

	if (nstate == IEEE80211_S_RUN && ic->ic_opmode == IEEE80211_M_STA)
		ath_hal_setassocid(ah, bssid, ni->ni_associd);
	else
		ath_hal_setassocid(ah, bssid, 0);
	if (ic->ic_flags & IEEE80211_F_PRIVACY) {
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, bssid);
	}

	if (ic->ic_opmode == IEEE80211_M_MONITOR) {
		/* nothing to do */;
	} else if (nstate == IEEE80211_S_RUN) {
		DPRINTF(ATH_DEBUG_STATE,
			"%s(RUN): ic_flags=0x%08x iv=%d bssid=%s "
			"capinfo=0x%04x chan=%d\n"
			 , __func__
			 , ic->ic_flags
			 , ni->ni_intval
			 , ether_sprintf(ni->ni_bssid)
			 , ni->ni_capinfo
			 , ieee80211_chan2ieee(ic, ni->ni_chan));

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
	} else {
		sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
		ath_hal_intrset(ah, sc->sc_imask);
	}

	/*
	 * Reset the rate control state.
	 */
	ath_rate_ctl_reset(sc, nstate);
done:
	/*
	 * Invoke the parent method to complete the work.
	 */
	error = (*sc->sc_newstate)(ic, nstate, arg);
	/*
	 * Finally, start any timers.
	 */
	if (nstate == IEEE80211_S_RUN) {
		/* start periodic recalibration timer */
		mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));
	} else if (nstate == IEEE80211_S_SCAN) {
		/* start ap/neighbor scan timer */
		mod_timer(&sc->sc_scan_ch,
			jiffies + ((HZ * ath_dwelltime) / 1000));
	}
bad:
	netif_start_queue(dev);
	return error;
}

/*
 * Setup driver-specific state for a newly associated node.
 * Note that we're called also on a re-associate, the isnew
 * param tells us if this is the first time or not.
 */
static void
ath_newassoc(struct ieee80211com *ic, struct ieee80211_node *ni, int isnew)
{
	if (isnew)
		ath_rate_ctl_start(ic->ic_dev->priv, ni);
}

static int
ath_getchannels(struct net_device *dev, u_int cc,
	HAL_BOOL outdoor, HAL_BOOL xchanmode)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	HAL_CHANNEL *chans;
	int i, ix, nchan;

	chans = kmalloc(IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL), GFP_KERNEL);
	if (chans == NULL) {
		printk("%s: unable to allocate channel table\n", dev->name);
		return ENOMEM;
	}
	if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, &nchan,
	    cc, HAL_MODE_ALL, outdoor, xchanmode)) {
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
	}
	kfree(chans);
	return 0;
}

static void
ath_update_led(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t threshold;

	/*
	 * When not associated, flash LED on for 5s, off for 200ms.
	 * XXX this assumes 100ms beacon interval.
	 */
	if (ic->ic_state != IEEE80211_S_RUN) {
		threshold = 2 + sc->sc_ledstate * 48;
	} else {
		threshold = 2 + sc->sc_ledstate * 18;
	}
	if (ic->ic_stats.is_rx_beacon - sc->sc_beacons >= threshold) {
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
		ath_hal_gpioset(ah, sc->sc_ledpin, sc->sc_ledstate);
		sc->sc_ledstate ^= 1;
		sc->sc_beacons = ic->ic_stats.is_rx_beacon;
	}
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
		DPRINTF(ATH_DEBUG_ANY, "%s: invalid mode %u\n", __func__, mode);
		return 0;
	}
	rt = sc->sc_rates[mode];
	if (rt == NULL)
		return 0;
	if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
		DPRINTF(ATH_DEBUG_ANY, "%s: rate table too small (%u > %u)\n",
			__func__, rt->rateCount, IEEE80211_RATE_MAXSIZE);
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
	memset(sc->sc_hwmap, 0, sizeof(sc->sc_hwmap));
	for (i = 0; i < 32; i++) {
		u_int8_t ix = rt->rateCodeToIndex[i];
		if (ix != 0xff)
			sc->sc_hwmap[i] = rt->info[ix].dot11Rate;
	}
	sc->sc_currates = rt;
	sc->sc_curmode = mode;
	/*
	 * All protection frames are transmited at 2Mb/s for
	 * 11g, otherwise at 1Mb/s.
	 * XXX select protection rate index from rate table.
	 */
	sc->sc_protrix = (mode == IEEE80211_MODE_11G ? 1 : 0);
	/* NB: caller is responsible for reseting rate control state */
}

static void
ath_rate_update(struct ath_softc *sc, struct ieee80211_node *ni, int rate)
{
	struct ath_node *an = ATH_NODE(ni);
	const HAL_RATE_TABLE *rt = sc->sc_currates;
	u_int8_t rix;

	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	DPRINTF(ATH_DEBUG_RATE, "%s: set xmit rate for %s to %dM\n",
	    __func__, ether_sprintf(ni->ni_macaddr),
	    ni->ni_rates.rs_nrates > 0 ?
		(ni->ni_rates.rs_rates[rate] & IEEE80211_RATE_VAL) / 2 : 0);

	ni->ni_txrate = rate;
	/* XXX management/control frames always go at the lowest speed */
	an->an_tx_mgtrate = rt->info[0].rateCode;
	an->an_tx_mgtratesp = an->an_tx_mgtrate | rt->info[0].shortPreamble;
	/*
	 * Before associating a node has no rate set setup
	 * so we can't calculate any transmit codes to use.
	 * This is ok since we should never be sending anything
	 * but management frames and those always go at the
	 * lowest hardware rate.
	 */
	if (ni->ni_rates.rs_nrates == 0)
		goto done;
	an->an_tx_rix0 = sc->sc_rixmap[
		ni->ni_rates.rs_rates[rate] & IEEE80211_RATE_VAL];
	an->an_tx_rate0 = rt->info[an->an_tx_rix0].rateCode;
	an->an_tx_rate0sp = an->an_tx_rate0 |
		rt->info[an->an_tx_rix0].shortPreamble;
	if (sc->sc_mrretry) {
		/*
		 * Hardware supports multi-rate retry; setup two
		 * step-down retry rates and make the lowest rate
		 * be the ``last chance''.  We use 4, 2, 2, 2 tries
		 * respectively (4 is set here, the rest are fixed
		 * in the xmit routine).
		 */
		an->an_tx_try0 = 1 + 3;		/* 4 tries at rate 0 */
		if (--rate >= 0) {
			rix = sc->sc_rixmap[
				ni->ni_rates.rs_rates[rate]&IEEE80211_RATE_VAL];
			an->an_tx_rate1 = rt->info[rix].rateCode;
			an->an_tx_rate1sp = an->an_tx_rate1 |
				rt->info[rix].shortPreamble;
		} else {
			an->an_tx_rate1 = an->an_tx_rate1sp = 0;
		}
		if (--rate >= 0) {
			rix = sc->sc_rixmap[
				ni->ni_rates.rs_rates[rate]&IEEE80211_RATE_VAL];
			an->an_tx_rate2 = rt->info[rix].rateCode;
			an->an_tx_rate2sp = an->an_tx_rate2 |
				rt->info[rix].shortPreamble;
		} else {
			an->an_tx_rate2 = an->an_tx_rate2sp = 0;
		}
		if (rate > 0) {
			/* NB: only do this if we didn't already do it above */
			an->an_tx_rate3 = rt->info[0].rateCode;
			an->an_tx_rate3sp =
				an->an_tx_mgtrate | rt->info[0].shortPreamble;
		} else {
			an->an_tx_rate3 = an->an_tx_rate3sp = 0;
		}
	} else {
		an->an_tx_try0 = ATH_TXMAXTRY;	/* max tries at rate 0 */
		an->an_tx_rate1 = an->an_tx_rate1sp = 0;
		an->an_tx_rate2 = an->an_tx_rate2sp = 0;
		an->an_tx_rate3 = an->an_tx_rate3sp = 0;
	}
done:
	an->an_tx_ok = an->an_tx_err = an->an_tx_retr = an->an_tx_upper = 0;
}

/*
 * Set the starting transmit rate for a node.
 */
static void
ath_rate_ctl_start(struct ath_softc *sc, struct ieee80211_node *ni)
{
#define	RATE(_ix)	(ni->ni_rates.rs_rates[(_ix)] & IEEE80211_RATE_VAL)
	struct ieee80211com *ic = &sc->sc_ic;
	int srate;

	KASSERT(ni->ni_rates.rs_nrates > 0, ("no rates"));
	if (ic->ic_fixed_rate == -1) {
		/*
		 * No fixed rate is requested. For 11b start with
		 * the highest negotiated rate; otherwise, for 11g
		 * and 11a, we start "in the middle" at 24Mb or 36Mb.
		 */
		srate = ni->ni_rates.rs_nrates - 1;
		if (sc->sc_curmode != IEEE80211_MODE_11B) {
			/*
			 * Scan the negotiated rate set to find the
			 * closest rate.
			 */
			/* NB: the rate set is assumed sorted */
			for (; srate >= 0 && RATE(srate) > 72; srate--)
				;
			KASSERT(srate >= 0, ("bogus rate set"));
		}
	} else {
		/*
		 * A fixed rate is to be used; ic_fixed_rate is an
		 * index into the supported rate set.  Convert this
		 * to the index into the negotiated rate set for
		 * the node.  We know the rate is there because the
		 * rate set is checked when the station associates.
		 */
		const struct ieee80211_rateset *rs =
			&ic->ic_sup_rates[ic->ic_curmode];
		int r = rs->rs_rates[ic->ic_fixed_rate] & IEEE80211_RATE_VAL;
		/* NB: the rate set is assumed sorted */
		srate = ni->ni_rates.rs_nrates - 1;
		for (; srate >= 0 && RATE(srate) != r; srate--)
			;
		KASSERT(srate >= 0,
			("fixed rate %d not in rate set", ic->ic_fixed_rate));
	}
	ath_rate_update(sc, ni, srate);
#undef RATE
}

/*
 * Reset the rate control state for each 802.11 state transition.
 */
static void
ath_rate_ctl_reset(struct ath_softc *sc, enum ieee80211_state state)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;

	if (ic->ic_opmode == IEEE80211_M_STA) {
		/*
		 * Reset local xmit state; this is really only
		 * meaningful when operating in station mode.
		 */
		ni = ic->ic_bss;
		if (state == IEEE80211_S_RUN) {
			ath_rate_ctl_start(sc, ni);
		} else {
			ath_rate_update(sc, ni, 0);
		}
	} else {
		/*
		 * When operating as a station the node table holds
		 * the AP's that were discovered during scanning.
		 * For any other operating mode we want to reset the
		 * tx rate state of each node.
		 */
		TAILQ_FOREACH(ni, &ic->ic_node, ni_list)
			ath_rate_update(sc, ni, 0);	/* use lowest rate */
		ath_rate_update(sc, ic->ic_bss, 0);
	}
	if (ic->ic_fixed_rate == -1 && state == IEEE80211_S_RUN) {
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
	struct ath_node *an = ATH_NODE(ni);
	struct ieee80211_rateset *rs = &ni->ni_rates;
	int mod = 0, nrate, enough;

	/*
	 * Rate control
	 * XXX: very primitive version.
	 */
	sc->sc_stats.ast_rate_calls++;

	enough = (an->an_tx_ok + an->an_tx_err >= 10);

	/* no packet reached -> down */
	if (an->an_tx_err > 0 && an->an_tx_ok == 0)
		mod = -1;

	/* all packets needs retry in average -> down */
	if (enough && an->an_tx_ok < an->an_tx_retr)
		mod = -1;

	/* no error and less than 10% of packets needs retry -> up */
	if (enough && an->an_tx_err == 0 && an->an_tx_ok > an->an_tx_retr * 10)
		mod = 1;

	nrate = ni->ni_txrate;
	switch (mod) {
	case 0:
		if (enough && an->an_tx_upper > 0)
			an->an_tx_upper--;
		break;
	case -1:
		if (nrate > 0) {
			nrate--;
			sc->sc_stats.ast_rate_drop++;
		}
		an->an_tx_upper = 0;
		break;
	case 1:
		if (++an->an_tx_upper < 10)
			break;
		an->an_tx_upper = 0;
		if (nrate + 1 < rs->rs_nrates) {
			nrate++;
			sc->sc_stats.ast_rate_raise++;
		}
		break;
	}

	if (nrate != ni->ni_txrate) {
		DPRINTF(ATH_DEBUG_RATE,
		    "%s: %dM -> %dM (%d ok, %d err, %d retr)\n",
		    __func__,
		    (rs->rs_rates[ni->ni_txrate] & IEEE80211_RATE_VAL) / 2,
		    (rs->rs_rates[nrate] & IEEE80211_RATE_VAL) / 2,
		    an->an_tx_ok, an->an_tx_err, an->an_tx_retr);
		ath_rate_update(sc, ni, nrate);
	} else if (enough)
		an->an_tx_ok = an->an_tx_err = an->an_tx_retr = 0;
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
			ath_rate_ctl(sc, ic->ic_bss);	/* NB: no reference */
		else
			ieee80211_iterate_nodes(ic, ath_rate_ctl, sc);
	}
	interval = ath_rateinterval;
	if (ic->ic_opmode == IEEE80211_M_STA)
		interval /= 2;
	sc->sc_rate_ctl.expires = jiffies + ((HZ * interval) / 1000);
	add_timer(&sc->sc_rate_ctl);
}

#if IEEE80211_VLAN_TAG_USED
static void
ath_vlan_register(struct net_device *dev, struct vlan_group *grp)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	ieee80211_vlan_register(ic, grp);
}

static void
ath_vlan_kill_vid(struct net_device *dev, unsigned short vid)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	ieee80211_vlan_kill_vid(ic, vid);
}
#endif /* IEEE80211_VLAN_TAG_USED */

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
	struct net_device_stats *stats = &sc->sc_devstats;

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

#ifdef CONFIG_NET_WIRELESS
/*
 * Return wireless extensions statistics.
 */
static struct iw_statistics *
ath_iw_getstats(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct iw_statistics *is = &sc->sc_iwstats;

	ieee80211_iw_getstats(ic, is);
	/* add in statistics maintained by the driver */
	is->discard.code += sc->sc_stats.ast_rx_badcrypt;
	is->discard.retries += sc->sc_stats.ast_tx_xretries;
	is->discard.misc += sc->sc_stats.ast_tx_encap
			 + sc->sc_stats.ast_tx_nonode
			 + sc->sc_stats.ast_tx_xretries
			 + sc->sc_stats.ast_tx_fifoerr
			 + sc->sc_stats.ast_tx_filtered
			 + sc->sc_stats.ast_tx_nobuf
			 + sc->sc_stats.ast_tx_nobufmgt;
			 ;
	is->miss.beacon = sc->sc_stats.ast_bmiss;

	return &sc->sc_iwstats;
}

#include <net/iw_handler.h>
/*
 * Bounce functions to get to the 802.11 code. These are
 * necessary for now because wireless extensions operations
 * are done on the underlying device and not the 802.11 instance.
 * This will change when there can be multiple 802.11 instances
 * associated with a device and we must have a net_device for
 * each so we can manipulate them individually.
 */
#define	ATH_CHAR_BOUNCE(name)						\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 char *erq, char *extra)				\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}
#define	ATH_POINT_BOUNCE(name)						\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 struct iw_point *erq, char *extra)			\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}
#define	ATH_PARAM_BOUNCE(name)						\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 struct iw_param *erq, char *extra)			\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}
#define	ATH_SOCKADDR_BOUNCE(name)					\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 struct sockaddr *erq, char *extra)			\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}
#define	ATH_FREQ_BOUNCE(name)						\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 struct iw_freq *erq, char *extra)			\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}
#define	ATH_U32_BOUNCE(name)						\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 __u32 *erq, char *extra)				\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}
#define	ATH_VOID_BOUNCE(name)						\
static int								\
ath_ioctl_##name(struct net_device *dev, struct iw_request_info *info,	\
	     	 void *erq, char *extra)				\
{									\
	struct ath_softc *sc = dev->priv;				\
	return ieee80211_ioctl_##name(&sc->sc_ic, info, erq, extra);	\
}

ATH_CHAR_BOUNCE(giwname)
ATH_POINT_BOUNCE(siwencode)
ATH_POINT_BOUNCE(giwencode)
ATH_PARAM_BOUNCE(siwrate)
ATH_PARAM_BOUNCE(giwrate)
ATH_PARAM_BOUNCE(siwsens)
ATH_PARAM_BOUNCE(giwsens)
ATH_PARAM_BOUNCE(siwrts)
ATH_PARAM_BOUNCE(giwrts)
ATH_PARAM_BOUNCE(siwfrag)
ATH_PARAM_BOUNCE(giwfrag)
ATH_SOCKADDR_BOUNCE(siwap)
ATH_SOCKADDR_BOUNCE(giwap)
ATH_POINT_BOUNCE(siwnickn)
ATH_POINT_BOUNCE(giwnickn)
ATH_FREQ_BOUNCE(siwfreq)
ATH_FREQ_BOUNCE(giwfreq)
ATH_POINT_BOUNCE(siwessid)
ATH_POINT_BOUNCE(giwessid)
ATH_POINT_BOUNCE(giwrange)
ATH_U32_BOUNCE(siwmode)
ATH_U32_BOUNCE(giwmode)
ATH_PARAM_BOUNCE(siwpower)
ATH_PARAM_BOUNCE(giwpower)
ATH_PARAM_BOUNCE(siwretry)
ATH_PARAM_BOUNCE(giwretry)
ATH_PARAM_BOUNCE(siwtxpow)
ATH_PARAM_BOUNCE(giwtxpow)
ATH_POINT_BOUNCE(iwaplist)
#ifdef SIOCGIWSCAN
ATH_POINT_BOUNCE(siwscan)
ATH_POINT_BOUNCE(giwscan)
#endif
ATH_VOID_BOUNCE(setparam)
ATH_VOID_BOUNCE(getparam)
ATH_VOID_BOUNCE(setkey)
ATH_VOID_BOUNCE(getkey)
ATH_VOID_BOUNCE(delkey)
ATH_VOID_BOUNCE(setmlme)
ATH_VOID_BOUNCE(setoptie)
ATH_VOID_BOUNCE(getoptie)
ATH_VOID_BOUNCE(addmac)
ATH_VOID_BOUNCE(delmac)
ATH_VOID_BOUNCE(chanlist)

/* Structures to export the Wireless Handlers */
static const iw_handler ath_handlers[] = {
	(iw_handler) NULL,				/* SIOCSIWCOMMIT */
	(iw_handler) ath_ioctl_giwname,			/* SIOCGIWNAME */
	(iw_handler) NULL,				/* SIOCSIWNWID */
	(iw_handler) NULL,				/* SIOCGIWNWID */
	(iw_handler) ath_ioctl_siwfreq,			/* SIOCSIWFREQ */
	(iw_handler) ath_ioctl_giwfreq,			/* SIOCGIWFREQ */
	(iw_handler) ath_ioctl_siwmode,			/* SIOCSIWMODE */
	(iw_handler) ath_ioctl_giwmode,			/* SIOCGIWMODE */
	(iw_handler) ath_ioctl_siwsens,			/* SIOCSIWSENS */
	(iw_handler) ath_ioctl_giwsens,			/* SIOCGIWSENS */
	(iw_handler) NULL /* not used */,		/* SIOCSIWRANGE */
	(iw_handler) ath_ioctl_giwrange,		/* SIOCGIWRANGE */
	(iw_handler) NULL /* not used */,		/* SIOCSIWPRIV */
	(iw_handler) NULL /* kernel code */,		/* SIOCGIWPRIV */
	(iw_handler) NULL /* not used */,		/* SIOCSIWSTATS */
	(iw_handler) NULL /* kernel code */,		/* SIOCGIWSTATS */
	(iw_handler) NULL,				/* SIOCSIWSPY */
	(iw_handler) NULL,				/* SIOCGIWSPY */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) ath_ioctl_siwap,			/* SIOCSIWAP */
	(iw_handler) ath_ioctl_giwap,			/* SIOCGIWAP */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) ath_ioctl_iwaplist,		/* SIOCGIWAPLIST */
#ifdef SIOCGIWSCAN
	(iw_handler) ath_ioctl_siwscan,			/* SIOCSIWSCAN */
	(iw_handler) ath_ioctl_giwscan,			/* SIOCGIWSCAN */
#else
	(iw_handler) NULL,				/* SIOCSIWSCAN */
	(iw_handler) NULL,				/* SIOCGIWSCAN */
#endif /* SIOCGIWSCAN */
	(iw_handler) ath_ioctl_siwessid,		/* SIOCSIWESSID */
	(iw_handler) ath_ioctl_giwessid,		/* SIOCGIWESSID */
	(iw_handler) ath_ioctl_siwnickn,		/* SIOCSIWNICKN */
	(iw_handler) ath_ioctl_giwnickn,		/* SIOCGIWNICKN */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) ath_ioctl_siwrate,			/* SIOCSIWRATE */
	(iw_handler) ath_ioctl_giwrate,			/* SIOCGIWRATE */
	(iw_handler) ath_ioctl_siwrts,			/* SIOCSIWRTS */
	(iw_handler) ath_ioctl_giwrts,			/* SIOCGIWRTS */
	(iw_handler) ath_ioctl_siwfrag,			/* SIOCSIWFRAG */
	(iw_handler) ath_ioctl_giwfrag,			/* SIOCGIWFRAG */
	(iw_handler) ath_ioctl_siwtxpow,		/* SIOCSIWTXPOW */
	(iw_handler) ath_ioctl_giwtxpow,		/* SIOCGIWTXPOW */
	(iw_handler) ath_ioctl_siwretry,		/* SIOCSIWRETRY */
	(iw_handler) ath_ioctl_giwretry,		/* SIOCGIWRETRY */
	(iw_handler) ath_ioctl_siwencode,		/* SIOCSIWENCODE */
	(iw_handler) ath_ioctl_giwencode,		/* SIOCGIWENCODE */
	(iw_handler) ath_ioctl_siwpower,		/* SIOCSIWPOWER */
	(iw_handler) ath_ioctl_giwpower,		/* SIOCGIWPOWER */
};
static const iw_handler ath_priv_handlers[] = {
	(iw_handler) ath_ioctl_setparam,		/* SIOCWFIRSTPRIV+0 */
	(iw_handler) ath_ioctl_getparam,		/* SIOCWFIRSTPRIV+1 */
	(iw_handler) ath_ioctl_setkey,			/* SIOCWFIRSTPRIV+2 */
	(iw_handler) ath_ioctl_getkey,			/* SIOCWFIRSTPRIV+3 */
	(iw_handler) ath_ioctl_delkey,			/* SIOCWFIRSTPRIV+4 */
	(iw_handler) NULL,				/* SIOCWFIRSTPRIV+5 */
	(iw_handler) ath_ioctl_setmlme,			/* SIOCWFIRSTPRIV+6 */
	(iw_handler) NULL,				/* SIOCWFIRSTPRIV+7 */
	(iw_handler) ath_ioctl_setoptie,		/* SIOCWFIRSTPRIV+8 */
	(iw_handler) ath_ioctl_getoptie,		/* SIOCWFIRSTPRIV+9 */
	(iw_handler) ath_ioctl_addmac,			/* SIOCWFIRSTPRIV+10 */
	(iw_handler) NULL,				/* SIOCWFIRSTPRIV+11 */
	(iw_handler) ath_ioctl_delmac,			/* SIOCWFIRSTPRIV+12 */
	(iw_handler) NULL,				/* SIOCWFIRSTPRIV+13 */
	(iw_handler) ath_ioctl_chanlist,		/* SIOCWFIRSTPRIV+14 */
};

static struct iw_handler_def ath_iw_handler_def = {
#define	N(a)	(sizeof (a) / sizeof (a[0]))
	.standard		= (iw_handler *) ath_handlers,
	.num_standard		= N(ath_handlers),
	.private		= (iw_handler *) ath_priv_handlers,
	.num_private		= N(ath_priv_handlers),
#undef N
};
#endif /* CONFIG_NET_WIRELESS */

static int
ath_set_mac_address(struct net_device *dev, void *addr)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct sockaddr *mac = addr;
	int error;

	if (netif_running(dev)) {
		DPRINTF(ATH_DEBUG_ANY,
			"%s: cannot set address; device running\n", __func__);
		return -EBUSY;
	}
	DPRINTF(ATH_DEBUG_ANY, "%s: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n", __func__,
		mac->sa_data[0], mac->sa_data[1], mac->sa_data[2],
		mac->sa_data[3], mac->sa_data[4], mac->sa_data[5]);

	ATH_LOCK(sc);
	/* XXX not right for multiple vap's */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, mac->sa_data);
	IEEE80211_ADDR_COPY(dev->dev_addr, mac->sa_data);
	ath_hal_setmac(ah, dev->dev_addr);
	error = -ath_reset(dev);
	ATH_UNLOCK(sc);

	return error;
}

static int      
ath_change_mtu(struct net_device *dev, int mtu) 
{
	struct ath_softc *sc = dev->priv;
	int error;

	if (!(ATH_MIN_MTU < mtu && mtu <= ATH_MAX_MTU)) {
		DPRINTF(ATH_DEBUG_ANY, "%s: invalid %d, min %u, max %u\n",
			__func__, mtu, ATH_MIN_MTU, ATH_MAX_MTU);
		return -EINVAL;
	}
	DPRINTF(ATH_DEBUG_ANY, "%s: %d\n", __func__, mtu);

	ATH_LOCK(sc);
	dev->mtu = mtu;
	/* NB: the rx buffers may need to be reallocated */
	tasklet_disable(&sc->sc_rxtq);
	error = -ath_reset(dev);
	tasklet_enable(&sc->sc_rxtq);
	ATH_UNLOCK(sc);

	return error;
}

/*
 * Diagnostic interface to the HAL.  This is used by various
 * tools to do things like retrieve register contents for
 * debugging.  The mechanism is intentionally opaque so that
 * it can change frequently w/o concern for compatiblity.
 */
static int
ath_ioctl_diag(struct ath_softc *sc, struct ath_diag *ad)
{
	struct ath_hal *ah = sc->sc_ah;
	u_int id = ad->ad_id & ATH_DIAG_ID;
	void *indata = NULL;
	void *outdata = NULL;
	u_int32_t insize = ad->ad_in_size;
	u_int32_t outsize = ad->ad_out_size;
	int error = 0;

	if (ad->ad_id & ATH_DIAG_IN) {
		/*
		 * Copy in data.
		 */
		indata = kmalloc(insize, GFP_KERNEL);
		if (indata == NULL) {
			error = -ENOMEM;
			goto bad;
		}
		if (copy_from_user(indata, ad->ad_in_data, insize)) {
			error = -EFAULT;
			goto bad;
		}
	}
	if (ad->ad_id & ATH_DIAG_DYN) {
		/*
		 * Allocate a buffer for the results (otherwise the HAL
		 * returns a pointer to a buffer where we can read the
		 * results).  Note that we depend on the HAL leaving this
		 * pointer for us to use below in reclaiming the buffer;
		 * may want to be more defensive.
		 */
		outdata = kmalloc(outsize, GFP_KERNEL);
		if (outdata == NULL) {
			error = -ENOMEM;
			goto bad;
		}
	}
	if (ath_hal_getdiagstate(ah, id, indata, insize, &outdata, &outsize)) {
		if (outsize < ad->ad_out_size)
			ad->ad_out_size = outsize;
		if (outdata &&
		     copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
			error = -EFAULT;
	} else {
		error = -EINVAL;
	}
bad:
	if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
		kfree(indata);
	if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
		kfree(outdata);
	return error;
}

extern	int ath_ioctl_ethtool(struct ath_softc *sc, int cmd, void *addr);

static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int error;

	ATH_LOCK(sc);
	switch (cmd) {
	case SIOCGATHSTATS:
		sc->sc_stats.ast_rx_rssi =
			(*ic->ic_node_getrssi)(ic, sc->sc_ic.ic_bss);
		if (copy_to_user(ifr->ifr_data, &sc->sc_stats,
		    sizeof (sc->sc_stats)))
			error = -EFAULT;
		else
			error = 0;
		break;
	case SIOCGATHDIAG:
		if (!capable(CAP_NET_ADMIN))
			error = -EPERM;
		else
			error = ath_ioctl_diag(sc, (struct ath_diag *) ifr);
		break;
	case SIOCETHTOOL:
		if (copy_from_user(&cmd, ifr->ifr_data, sizeof(cmd)))
			error = -EFAULT;
		else
			error = ath_ioctl_ethtool(sc, cmd, ifr->ifr_data);
			break;
	default:
		error = ieee80211_ioctl(ic, ifr, cmd);
		break;
	}
	ATH_UNLOCK(sc);
	return error;
}

#ifdef CONFIG_SYSCTL
/*
 * Sysctls are split into ``static'' and ``dynamic'' tables.
 * The former are defined at module load time and are used
 * control parameters common to all devices.  The latter are
 * tied to particular device instances and come and go with
 * each device.  The split is currently a bit tenuous; many of
 * the static ones probably should be dynamic but having them
 * static (e.g. debug) means they can be set after a module is
 * loaded and before bringing up a device.  The alternative
 * is to add module parameters.
 */

/*
 * Dynamic (i.e. per-device) sysctls.  These are automatically
 * mirrored in /proc/sys.
 */
enum {
	ATH_SLOTTIME	= 1,
	ATH_ACKTIMEOUT	= 2,
	ATH_CTSTIMEOUT	= 3,
	ATH_SOFTLED	= 4,
	ATH_LEDPIN	= 5,
};

static int
ath_sysctl_halparam(ctl_table *ctl, int write, struct file *filp,
	void *buffer, size_t *lenp)
{
	struct ath_softc *sc = ctl->extra1;
	u_int val;
	int ret;

	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) {
		ret = proc_dointvec(ctl, write, filp, buffer, lenp);
		if (ret == 0) {
			switch (ctl->ctl_name) {
			case ATH_SLOTTIME:
				if (!ath_hal_setslottime(sc->sc_ah, val))
					ret = -EINVAL;
				break;
			case ATH_ACKTIMEOUT:
				if (!ath_hal_setacktimeout(sc->sc_ah, val))
					ret = -EINVAL;
				break;
			case ATH_CTSTIMEOUT:
				if (!ath_hal_setctstimeout(sc->sc_ah, val))
					ret = -EINVAL;
				break;
			case ATH_SOFTLED:
				if (val != sc->sc_softled) {
					if (val)
						ath_hal_gpioCfgOutput(sc->sc_ah,
							sc->sc_ledpin);
					ath_hal_gpioset(sc->sc_ah,
						sc->sc_ledpin,!val);
					sc->sc_softled = val;
				}
				break;
			case ATH_LEDPIN:
				sc->sc_ledpin = val;
				break;
			default:
				return -EINVAL;
			}
		}
	} else {
		switch (ctl->ctl_name) {
		case ATH_SLOTTIME:
			val = ath_hal_getslottime(sc->sc_ah);
			break;
		case ATH_ACKTIMEOUT:
			val = ath_hal_getacktimeout(sc->sc_ah);
			break;
		case ATH_CTSTIMEOUT:
			val = ath_hal_getctstimeout(sc->sc_ah);
			break;
		case ATH_SOFTLED:
			val = sc->sc_softled;
			break;
		case ATH_LEDPIN:
			val = sc->sc_ledpin;
			break;
		default:
			return -EINVAL;
		}
		ret = proc_dointvec(ctl, write, filp, buffer, lenp);
	}
	return ret;
}

static	int mindwelltime = 100;			/* 100ms */
static	int mincalibrate = 1;			/* once a second */
static	int minrateinterval = 500;		/* 500ms */
static	int maxint = 0x7fffffff;		/* 32-bit big */

#define	CTL_AUTO	-2	/* cannot be CTL_ANY or CTL_NONE */

static const ctl_table ath_sysctl_template[] = {
	{ .ctl_name	= ATH_SLOTTIME,
	  .procname	= "slottime",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_ACKTIMEOUT,
	  .procname	= "acktimeout",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_CTSTIMEOUT,
	  .procname	= "ctstimeout",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_SOFTLED,
	  .procname	= "softled",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_LEDPIN,
	  .procname	= "ledpin",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ 0 }
};

static void
ath_dynamic_sysctl_register(struct ath_softc *sc)
{
	int i, space;

	space = 5*sizeof(struct ctl_table) + sizeof(ath_sysctl_template);
	sc->sc_sysctls = kmalloc(space, GFP_KERNEL);
	if (sc->sc_sysctls == NULL) {
		printk("%s: no memory for sysctl table!\n", __func__);
		return;
	}

	/* setup the table */
	memset(sc->sc_sysctls, 0, space);
	sc->sc_sysctls[0].ctl_name = CTL_DEV;
	sc->sc_sysctls[0].procname = "dev";
	sc->sc_sysctls[0].mode = 0555;
	sc->sc_sysctls[0].child = &sc->sc_sysctls[2];
	/* [1] is NULL terminator */
	sc->sc_sysctls[2].ctl_name = CTL_AUTO;
	sc->sc_sysctls[2].procname = sc->sc_dev.name;
	sc->sc_sysctls[2].mode = 0555;
	sc->sc_sysctls[2].child = &sc->sc_sysctls[4];
	/* [3] is NULL terminator */
	/* copy in pre-defined data */
	memcpy(&sc->sc_sysctls[4], ath_sysctl_template,
		sizeof(ath_sysctl_template));

	/* add in dynamic data references */
	for (i = 4; sc->sc_sysctls[i].ctl_name; i++)
		if (sc->sc_sysctls[i].extra1 == NULL)
			sc->sc_sysctls[i].extra1 = sc;

	/* and register everything */
	sc->sc_sysctl_header = register_sysctl_table(sc->sc_sysctls, 1);
	if (!sc->sc_sysctl_header) {
		printk("%s: failed to register sysctls!\n", sc->sc_dev.name);
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}
}

static void
ath_dynamic_sysctl_unregister(struct ath_softc *sc)
{
	if (sc->sc_sysctl_header) {
		unregister_sysctl_table(sc->sc_sysctl_header);
		sc->sc_sysctl_header = NULL;
	}
	if (sc->sc_sysctls) {
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}
}

/*
 * Static (i.e. global) sysctls.  Note that the hal sysctls
 * are located under ours by sharing the setting for DEV_ATH.
 */
enum {
	DEV_ATH		= 9,			/* XXX known by hal */
};

static ctl_table ath_static_sysctls[] = {
#ifdef AR_DEBUG
	{ .ctl_name	= CTL_AUTO,
	   .procname	= "debug",
	  .mode		= 0644,
	  .data		= &ath_debug,
	  .maxlen	= sizeof(ath_debug),
	  .proc_handler	= proc_dointvec
	},
#endif
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "countrycode",
	  .mode		= 0444,
	  .data		= &ath_countrycode,
	  .maxlen	= sizeof(ath_countrycode),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "regdomain",
	  .mode		= 0444,
	  .data		= &ath_regdomain,
	  .maxlen	= sizeof(ath_regdomain),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "outdoor",
	  .mode		= 0444,
	  .data		= &ath_outdoor,
	  .maxlen	= sizeof(ath_outdoor),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "xchanmode",
	  .mode		= 0444,
	  .data		= &ath_xchanmode,
	  .maxlen	= sizeof(ath_xchanmode),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "dwelltime",
	  .mode		= 0644,
	  .data		= &ath_dwelltime,
	  .maxlen	= sizeof(ath_dwelltime),
	  .extra1	= &mindwelltime,
	  .extra2	= &maxint,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "calibrate",
	  .mode		= 0644,
	  .data		= &ath_calinterval,
	  .maxlen	= sizeof(ath_calinterval),
	  .extra1	= &mincalibrate,
	  .extra2	= &maxint,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "rateinterval",
	  .mode		= 0644,
	  .data		= &ath_rateinterval,
	  .maxlen	= sizeof(ath_rateinterval),
	  .extra1	= &minrateinterval,
	  .extra2	= &maxint,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ 0 }
};
static ctl_table ath_ath_table[] = {
	{ .ctl_name	= DEV_ATH,
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_static_sysctls
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{ .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_sysctl_header;

void
ath_sysctl_register(void)
{
	static int initialized = 0;

	if (!initialized) {
		ath_sysctl_header =
			register_sysctl_table(ath_root_table, 1);
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
