/*
 * Copyright (C) 2002-2007 Sam Leffler, Errno Consulting
 * Copyright (C) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu> 
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

/*
 * Driver for the Atheros Wireless LAN controller.
 *
 * This software is derived from work of Atsushi Onoe; his contribution
 * is greatly appreciated.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/pci.h>
#include <net/iw_handler.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>

/*
 * #define AR_DEBUG here if you need to debug the ath_pci module (athdebug)
 * disable this if not needed because it adds an amount of load
 */
#define AR_DEBUG

#include "ath5k_hw.h"

/* unaligned little endian access */
#define LE_READ_2(_p) (le16_to_cpu(get_unaligned((__le16 *)(_p))))
#define LE_READ_4(_p) (le32_to_cpu(get_unaligned((__le32 *)(_p))))

enum {
	ATH_LED_TX,
	ATH_LED_RX,
	ATH_LED_POLL,
};

static int	ath_init(struct net_device *);
static int	ath_reset(struct net_device *);
static int	ath_stop_locked(struct net_device *);
static int	ath_stop(struct net_device *);
static int	ath_media_change(struct net_device *);
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
static int	ath_beaconq_setup(struct ath_hal *);
static int	ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
static void	ath_beacon_setup(struct ath_softc *, struct ath_buf *);
static void	ath_beacon_send(struct net_device *);
static void	ath_beacon_free(struct ath_softc *);
static void	ath_beacon_config(struct ath_softc *);
static void	ath_descdma_cleanup(struct ath_softc *sc, ath_bufhead *);
static int	ath_desc_alloc(struct ath_softc *);
static void	ath_desc_free(struct ath_softc *);
static struct ieee80211_node *ath_node_alloc(struct ieee80211_node_table *);
static void	ath_node_free(struct ieee80211_node *);
static u_int8_t	ath_node_getrssi(const struct ieee80211_node *);
static int	ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
static void	ath_recv_mgmt(struct ieee80211com *, struct sk_buff *,
			struct ieee80211_node *,
			int subtype, int rssi, u_int32_t rstamp);
static void	ath_setdefantenna(struct ath_softc *, u_int);
static struct ath_txq *ath_txq_setup(struct ath_softc*, int qtype, int subtype);
static int	ath_tx_setup(struct ath_softc *, int, int);
static int	ath_wme_update(struct ieee80211com *);
static void	ath_tx_cleanupq(struct ath_softc *, struct ath_txq *);
static void	ath_tx_cleanup(struct ath_softc *);
static int	ath_start(struct sk_buff *, struct net_device *);

static int	ath_tx_start(struct net_device *, struct ieee80211_node *,
			     struct ath_buf *, struct sk_buff *);
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
static struct iw_statistics *ath_iw_getstats(struct net_device *);
static struct iw_handler_def ath_iw_handler_def;
static void	ath_setup_stationkey(struct ieee80211_node *);
static void	ath_newassoc(struct ieee80211_node *, int);
static int	ath_getchannels(struct net_device *, u_int cc,
			AR5K_BOOL outdoor, AR5K_BOOL xchanmode);
static void	ath_led_event(struct ath_softc *, int);
static void     ath_led_off(unsigned long arg);
static void	ath_update_txpow(struct ath_softc *);

static int	ath_set_mac_address(struct net_device *, void *);
static int	ath_change_mtu(struct net_device *, int);
static int	ath_ioctl(struct net_device *, struct ifreq *, int);

static int	ath_rate_setup(struct net_device *, u_int mode);
static void	ath_setcurmode(struct ath_softc *, u_int);

static void	ath_announce(struct ath_softc *);

static const char *hal_status_desc[] = {
	"Everything went O.K.",
	"Unable to allocate memory for ath_hal",
	"Hardware I/O Error",
	"Unable to access EEPROM",
	"Invalid EEPROM checksum",
	"Unable to get device caps from EEPROM",
	"Unable to read MAC address from EEPROM",
	"Invalid parameter to function",
	"Hardware revision not supported",
	"Unexpected error ocured during process"
};

static	int ath_dwelltime = 200;		/* 5 channels/second */
static	int ath_calinterval = 30;		/* calibrate every 30 secs */
static	int ath_countrycode = CTRY_DEFAULT;	/* country code */
static	int ath_regdomain = 0;			/* regulatory domain */
static	int ath_outdoor = 1;		/* enable outdoor use */
static	int ath_xchanmode = 1;		/* enable extended channels */

#ifdef AR_DEBUG
static	int ath_debug = 0;
#define	IFF_DUMPPKTS(sc, _m) \
	((sc->sc_debug & _m) || ieee80211_msg_dumppkts(&sc->sc_ic))
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
	ATH_DEBUG_NODE		= 0x00080000,	/* node management */
	ATH_DEBUG_LED		= 0x00100000,	/* led management */
	ATH_DEBUG_MAC80211	= 0x02000000,   /* mac80211 interface */
	ATH_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	ATH_DEBUG_ANY		= 0xffffffff
};
#define	DPRINTF(sc, _m, _fmt, ...) do {		\
	if (sc->sc_debug & (_m))			\
		printk(_fmt, __VA_ARGS__);		\
} while (0)
#define	KEYPRINTF(sc, ix, hk, mac) do {		\
	if (sc->sc_debug & ATH_DEBUG_KEYCACHE)		\
		ath_keyprint(__func__, ix, hk, mac);	\
} while (0)
static	void ath_printrxbuf(struct ath_buf *bf, int);
static	void ath_printtxbuf(struct ath_buf *bf, int);
#else
#define	IFF_DUMPPKTS(sc, _m)	0
#define	DPRINTF(sc, _m, _fmt, ...)
#define	KEYPRINTF(sc, k, ix, mac)
#endif

static	int countrycode = -1;
static	int outdoor = -1;
static	int xchanmode = -1;
module_param(countrycode, int, 0);
module_param(outdoor, int, 0);
module_param(xchanmode, int, 0);
MODULE_PARM_DESC(countrycode, "Override default country code");
MODULE_PARM_DESC(outdoor, "Enable/disable outdoor use");
MODULE_PARM_DESC(xchanmode, "Enable/disable extended channel mode");


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
	DIDmsg_lnxind_wlansniffrm_noise	= 0x00070044,
	DIDmsg_lnxind_wlansniffrm_rate		= 0x00080044,
	DIDmsg_lnxind_wlansniffrm_istx		= 0x00090044,
	DIDmsg_lnxind_wlansniffrm_frmlen	= 0x000A0044
};
enum {
	P80211ENUM_msgitem_status_no_value	= 0x00
};
enum {
	P80211ENUM_truth_false			= 0x00,
	P80211ENUM_truth_true			= 0x01
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

/* XXX: consider changing arg2 to struct ath_softc */
int
ath_attach(u_int16_t devid, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211_hw *hw = sc->sc_hw;
	struct ath_hal *ah;
	AR5K_STATUS status;
	int error = 0, i;
	u_int8_t csz;
	sc->devid = devid;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: devid 0x%x\n", __func__, devid);

	init_mutex(&(sc)->sc_lock);
	spin_lock_init(&(sc)->sc_txbuflock);

	tasklet_init(&sc->sc_resettq,	ath_reset_tasklet, (unsigned long) dev);
	tasklet_init(&sc->sc_rxtq,	ath_rx_tasklet, (unsigned long) dev);
	tasklet_init(&sc->sc_rxtq,	ath_beacon_tasklet, (unsigned long) sc);
	
	/* Initialize the hardware structure */
	ah = ath5k_hw_init(devid, sc->sc_iobase, &status);
	if (ah == NULL) {
		printk(KERN_ERR "%s: unable to initialize hardware structure: '%s' (HW status %u)\n",
			__func__, hal_status_desc[status], status);
		error = ENXIO;
		goto bad1;
	}
	sc->sc_ah = ah;

	/*
	 * Reset the key cache since some parts do not
	 * reset the contents on initial power up.
	 */
	for (i = 0; i < AR5K_KEYCACHE_SIZE; i++)
		ath5k_hw_reset_key(ah, i);
	/*
	 * Mark key cache slots associated with global keys
	 * as in use.  If we knew TKIP was not to be used we
	 * could leave the +32, +64, and +32+64 slots free.
	 * XXX only for splitmic.
	 * XXX: is this necessary?
	 */
	for (i = 0; i < AR5K_WEP_NKID; i++) {
		setbit(sc->sc_keymap, i);
		setbit(sc->sc_keymap, i+32);
		setbit(sc->sc_keymap, i+64);
		setbit(sc->sc_keymap, i+32+64);
	}

	/*
	 * Collect the channel list using the default country
	 * code and including outdoor channels.  The 802.11 layer
	 * is resposible for filtering this list based on settings
	 * like the phy mode.
	 */
	if (countrycode != -1)
		ath_countrycode = countrycode;
	if (outdoor != -1)
		ath_outdoor = outdoor;
	if (xchanmode != -1)
		ath_xchanmode = xchanmode;
	/* XXX remove countrycode, outdoor, and get this from nl80211 later. */
	error = ath_getchannels(dev, ath_countrycode,
			ath_outdoor, ath_xchanmode);
	if (error != 0) {
		goto bad1;
	}
	
	/*
	 * Setup rate tables for all potential media types.
	 */
	ath_rate_setup(dev, IEEE80211_MODE_11A);
	ath_rate_setup(dev, IEEE80211_MODE_11B);
	ath_rate_setup(dev, IEEE80211_MODE_11G);

	/* NB: setup here so ath_rate_update is happy */
	ath_setcurmode(sc, IEEE80211_MODE_11A);

	/*
	 * Allocate tx+rx descriptors and populate the lists.
	 */
	error = ath_desc_alloc(sc);
	if (error != 0) {
		if_printf(dev, "failed to allocate descriptors: %d\n", error);
		goto bad1;
	}

	/*
	 * Allocate hardware transmit queues: one queue for
	 * beacon frames and one data queue for each QoS
	 * priority.  Note that the hal handles reseting
	 * these queues at the needed time.
	 *
	 * XXX PS-Poll
	 */
	sc->sc_bhalq = ath_beaconq_setup(ah);
	if (sc->sc_bhalq == (u_int) -1) {
		if_printf(dev, "unable to setup a beacon xmit queue!\n");
		goto bad2;
	}
	sc->sc_cabq = ath_txq_setup(sc, AR5K_TX_QUEUE_CAB, 0);
	if (sc->sc_cabq == NULL) {
		if_printf(dev, "unable to setup CAB xmit queue!\n");
		error = EIO;
		goto bad2;
	}
	/* NB: insure BK queue is the lowest priority h/w queue */
	if (!ath_tx_setup(sc, WME_AC_BK, AR5K_WME_AC_BK)) {
		if_printf(dev, "unable to setup xmit queue for %s traffic!\n",
			ieee80211_wme_acnames[WME_AC_BK]);
		error = EIO;
		goto bad2;
	}
	if (!ath_tx_setup(sc, WME_AC_BE, AR5K_WME_AC_BE) ||
	    !ath_tx_setup(sc, WME_AC_VI, AR5K_WME_AC_VI) ||
	    !ath_tx_setup(sc, WME_AC_VO, AR5K_WME_AC_VO)) {
		/* 
		 * Not enough hardware tx queues to properly do WME;
		 * just punt and assign them all to the same h/w queue.
		 * We could do a better job of this if, for example,
		 * we allocate queues when we switch from station to
		 * AP mode.
		 */
		if (sc->sc_ac2q[WME_AC_VI] != NULL)
			ath_tx_cleanupq(sc, sc->sc_ac2q[WME_AC_VI]);
		if (sc->sc_ac2q[WME_AC_BE] != NULL)
			ath_tx_cleanupq(sc, sc->sc_ac2q[WME_AC_BE]);
		sc->sc_ac2q[WME_AC_BE] = sc->sc_ac2q[WME_AC_BK];
		sc->sc_ac2q[WME_AC_VI] = sc->sc_ac2q[WME_AC_BK];
		sc->sc_ac2q[WME_AC_VO] = sc->sc_ac2q[WME_AC_BK];
	}

	/* 
	 * Special case certain configurations.  Note the
	 * CAB queue is handled by these specially so don't
	 * include them when checking the txq setup mask.
	 */
	switch (sc->sc_txqsetup &~ (1<<sc->sc_cabq->axq_qnum)) {
	case 0x01:
		tasklet_init(&sc->sc_txtq, ath_tx_tasklet_q0, 
			(unsigned long) dev);
		break;
	case 0x0f:
		tasklet_init(&sc->sc_txtq, ath_tx_tasklet_q0123,
			(unsigned long) dev);
		break;
	default:
		tasklet_init(&sc->sc_txtq, ath_tx_tasklet,
			(unsigned long) dev);
		break;
	}

	/*
	 * Setup rate control.  Some rate control modules
	 * call back to change the anntena state so expose
	 * the necessary entry points.
	 * XXX maybe belongs in struct ath_ratectrl?
	 */
	sc->sc_setdefantenna = ath_setdefantenna;
	sc->sc_rc = ath_rate_attach(sc);
	if (sc->sc_rc == NULL) {
		error = EIO;
		goto bad2;
	}

	init_timer(&sc->sc_scan_ch);
	sc->sc_scan_ch.function = ath_next_scan;
	sc->sc_scan_ch.data = (unsigned long) dev;

	init_timer(&sc->sc_cal_ch);
	sc->sc_cal_ch.function = ath_calibrate;
	sc->sc_cal_ch.data = (unsigned long) dev;

	sc->sc_blinking = 0;
	sc->sc_ledstate = 1;
	sc->sc_ledon = 0;			/* low true */
	sc->sc_ledidle = (2700*HZ)/1000;	/* 2.7sec */

	init_timer(&sc->sc_ledtimer);
	sc->sc_ledtimer.function = ath_led_off;
	sc->sc_ledtimer.data = (unsigned long) sc;
	/* Auto-enable soft led processing for IBM cards and for
	 * 5211 minipci cards. */
	sc->sc_softled = (devid == PCI_PRODUCT_ATHEROS_AR5212_IBM || 
		devid == PCI_PRODUCT_ATHEROS_AR5211);
	if (sc->sc_softled) {
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
		ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);
	}

	ether_setup(dev);
	dev->open = ath_init;
	dev->stop = ath_stop;
	dev->hard_start_xmit = ath_start;
	dev->tx_timeout = ath_tx_timeout;
	dev->watchdog_timeo = 5 * HZ;			/* XXX */
	dev->set_multicast_list = ath_mode_init;
	dev->do_ioctl = ath_ioctl;
	dev->get_stats = ath_getstats;
	dev->set_mac_address = ath_set_mac_address;
 	dev->change_mtu = &ath_change_mtu;
	dev->tx_queue_len = ATH_TXBUF;			/* TODO? 1 for mgmt frame */
/*get_wireless_stats moved from net_device to iw_handler_def*/
# if IW_HANDLER_VERSION < 7
	dev->get_wireless_stats = ath_iw_getstats;
# endif
	ieee80211_ioctl_iwsetup(&ath_iw_handler_def);
	dev->wireless_handlers = &ath_iw_handler_def;
#if IEEE80211_VLAN_TAG_USED
	dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
	dev->vlan_rx_register = ath_vlan_register;
	dev->vlan_rx_kill_vid = ath_vlan_kill_vid;
#endif /* IEEE80211_VLAN_TAG_USED */

	ic->ic_dev = dev;
	ic->ic_devstats = &sc->sc_devstats;
	ic->ic_init = ath_init;
	ic->ic_reset = ath_reset;
	ic->ic_newassoc = ath_newassoc;
	ic->ic_updateslot = ath_updateslot;
	ic->ic_wme.wme_update = ath_wme_update;
	/* XXX not right but it's not used anywhere important */
	ic->ic_phytype = IEEE80211_RATE_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_caps =
		  IEEE80211_C_IBSS		/* ibss, nee adhoc, mode */
		| IEEE80211_C_AHDEMO		/* adhoc demo (pseudo_ibss) mode */
		| IEEE80211_C_HOSTAP		/* hostap mode */
		| IEEE80211_C_MONITOR		/* monitor mode */
		| IEEE80211_C_SHPREAMBLE	/* short preamble supported */
		| IEEE80211_C_SHSLOT		/* short slot time supported */
		| IEEE80211_C_WPA		/* capable of WPA1+WPA2 */
		;

	/*
	 * initialize management queue
	 */
	skb_queue_head_init(&ic->ic_mgtq);

	/*
	 * Query the hal to figure out h/w crypto support.
	 */
	if (ath_hal_ciphersupported(ah, AR5K_CIPHER_WEP))
		ic->ic_caps |= IEEE80211_C_WEP;
	if (ath_hal_ciphersupported(ah, AR5K_CIPHER_AES_OCB))
		ic->ic_caps |= IEEE80211_C_AES;
	if (ath_hal_ciphersupported(ah, AR5K_CIPHER_AES_CCM))
		ic->ic_caps |= IEEE80211_C_AES_CCM;
	if (ath_hal_ciphersupported(ah, AR5K_CIPHER_CKIP))
		ic->ic_caps |= IEEE80211_C_CKIP;
	if (ath_hal_ciphersupported(ah, AR5K_CIPHER_TKIP)) {
		ic->ic_caps |= IEEE80211_C_TKIP;
		/*
		 * Check if h/w does the MIC and/or whether the
		 * separate key cache entries are required to
		 * handle both tx+rx MIC keys.
		 */
		if (ath_hal_ciphersupported(ah, AR5K_CIPHER_MIC))
			ic->ic_caps |= IEEE80211_C_TKIPMIC;
		if (ath_hal_tkipsplit(ah))
			sc->sc_splitmic = 1;
	}
	sc->sc_hasclrkey = ath_hal_ciphersupported(ah, AR5K_CIPHER_CLR);
	sc->sc_mcastkey = ath_hal_getmcastkeysearch(ah);
	/*
	 * TPC support can be done either with a global cap or
	 * per-packet support.  The latter is not available on
	 * all parts.  We're a bit pedantic here as all parts
	 * support a global cap.
	 */
	if (ath_hal_hastpc(ah) || ath_hal_hastxpowlimit(ah))
		ic->ic_caps |= IEEE80211_C_TXPMGT;

	/*
	 * Mark WME capability only if we have sufficient
	 * hardware queues to do proper priority scheduling.
	 */
	if (sc->sc_ac2q[WME_AC_BE] != sc->sc_ac2q[WME_AC_BK])
		ic->ic_caps |= IEEE80211_C_WME;
	/*
	 * Check for misc other capabilities.
	 */
	if (ath_hal_hasbursting(ah))
		ic->ic_caps |= IEEE80211_C_BURST;

	/*
	 * Indicate we need the 802.11 header padded to a
	 * 32-bit boundary for 4-address and QoS frames.
	 */
	ic->ic_flags |= IEEE80211_F_DATAPAD;

	/*
	 * Query the hal about antenna support.
	 */
	sc->sc_defant = ath_hal_getdefantenna(ah);

	/*
	 * Not all chips have the VEOL support we want to
	 * use with IBSS beacons; check here for it.
	 */
	sc->sc_hasveol = ath_hal_hasveol(ah);

	sc->sc_rxfilter = 0;
	sc->sc_rawdev_enabled = 0;
	sc->sc_rawdev.type = ARPHRD_IEEE80211;

	/* get mac address from hardware */
	ath_hal_getmac(ah, ic->ic_myaddr);
	IEEE80211_ADDR_COPY(dev->dev_addr, ic->ic_myaddr);

	/* call MI attach routine. */
	ieee80211_ifattach(ic);
	/* override default methods */
	ic->ic_node_alloc = ath_node_alloc;
	sc->sc_node_free = ic->ic_node_free;
	ic->ic_node_free = ath_node_free;
	ic->ic_node_getrssi = ath_node_getrssi;
	sc->sc_recv_mgmt = ic->ic_recv_mgmt;
	ic->ic_recv_mgmt = ath_recv_mgmt;
	sc->sc_newstate = ic->ic_newstate;
	ic->ic_newstate = ath_newstate;
	ic->ic_crypto.cs_key_alloc = ath_key_alloc;
	ic->ic_crypto.cs_key_delete = ath_key_delete;
	ic->ic_crypto.cs_key_set = ath_key_set;
	ic->ic_crypto.cs_key_update_begin = ath_key_update_begin;
	ic->ic_crypto.cs_key_update_end = ath_key_update_end;

	radar_init(ic);

	/* complete initialization */
	ieee80211_media_init(ic, ath_media_change, ieee80211_media_status);

	if (register_netdev(dev)) {
		printk(KERN_ERR "%s: unable to register device\n", dev->name);
		goto bad3;
	}

	ieee80211_announce(ic);
	ath_announce(sc);
	return 0;
bad3:
	ieee80211_ifdetach(ic);
	ath_rate_detach(sc->sc_rc);
bad2:
	ath_tx_cleanup(sc);
	ath_desc_free(sc);
bad1:
	if (ah) {
		ath_hal_detach(ah);
	}
bad0:
	sc->sc_invalid = 1;
	return error;
}

int
ath_detach(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_stop(dev);
	sc->sc_invalid = 1;
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
	ieee80211_ifdetach(ic);
	ath_rate_detach(sc->sc_rc);
	ath_desc_free(sc);
	ath_tx_cleanup(sc);
	ath_hal_detach(sc->sc_ah);

	/*
	 * NB: can't reclaim these until after ieee80211_ifdetach
	 * returns because we'll get called back to reclaim node
	 * state and potentially want to use them.
	 */
	unregister_netdev(dev);

	return 0;
}

void
ath_suspend(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	if (sc->sc_softled) 
	    	ath_hal_gpioset(ah, sc->sc_ledpin, 1);

	ath_stop(dev);
}

void
ath_resume(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_init(dev);
	if (sc->sc_softled) {
	    	ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
	    	ath_hal_gpioset(ah, sc->sc_ledpin, 0);
	}
}

static int
ath_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_STATUS status;
	int error = 0;
	int opmode;
	
	down(&(sc)->sc_lock)

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: mode %d\n", __func__, ic->ic_opmode);

	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop_locked(dev);

	/*
	 * Change our interface type if we are in monitor mode.
	 */
	dev->type = (ic->ic_opmode == IEEE80211_M_MONITOR) ?
		ARPHRD_IEEE80211_RADIOTAP : ARPHRD_ETHER;

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	sc->sc_curchan.freq = ic->ic_ibss_chan->ic_freq;
	sc->sc_curchan.channel_flags = ath_chan2flags(ic, ic->ic_ibss_chan);
	opmode = (ic->ic_opmode == IEEE80211_M_AHDEMO) ? 0 : ic->ic_opmode;
	if (!ath_hal_reset(ah, opmode, &sc->sc_curchan, FALSE, &status)) {
		if_printf(dev, "unable to reset hardware; hal status %u\n",
			status);
		error = -EIO;
		goto done;
	}

	/*
	 * This is needed only to setup initial state
	 * but it's best done after a reset.
	 */
	ath_update_txpow(sc);

	/*
	 * Likewise this is set during reset so update
	 * state cached in the driver.
	 */
	sc->sc_diversity = ath_hal_getdiversity(ah);

	/*
	 * Setup the hardware after reset: the key cache
	 * is filled as needed and the receive engine is
	 * set going.  Frame transmit is handled entirely
	 * in the frame output path; there's nothing to do
	 * here except setup the interrupt mask.
	 */
	if (ath_startrecv(sc) != 0) {
		if_printf(dev, "unable to start recv logic\n");
		error = -EIO;
		goto done;
	}

	/*
	 * Enable interrupts.
	 */
	sc->sc_imask = AR5K_INT_RX | AR5K_INT_TX
		  | AR5K_INT_RXEOL | AR5K_INT_RXORN
		  | AR5K_INT_FATAL | AR5K_INT_GLOBAL;	// TODO: compiler warning integer overflow in expression

	/* Check if the device has hardware counters for PHY
	 * errors. If so we need to enable the MIB interrupt
	 * so we can act on stat triggers. We only do this currently
	 * for station mode */
	if (ath5k_hw_get_cap_phycounters(ah))
		sc->sc_needmib = 1;

	if (ath5k_hw_get_cap_phycounters(ah) && 
		ic->ic_opmode == IEEE80211_M_STA)
		sc->sc_imask |= AR5K_INT_MIB;
	ath5k_hw_set_intr(ah, sc->sc_imask);

	dev->flags |= IFF_RUNNING;
	ic->ic_state = IEEE80211_S_INIT;

	/*
	 * The hardware should be ready to go now so it's safe
	 * to kick the 802.11 state machine as it's likely to
	 * immediately call back to us to send mgmt frames.
	 */
	ni = ic->ic_bss;
	ni->ni_chan = ic->ic_ibss_chan;
	ath_chan_change(sc, ni->ni_chan);
	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
		if (ic->ic_roaming != IEEE80211_ROAMING_MANUAL)
			ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
	} else
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
done:
	up(&(sc)->sc_lock);
	return error;
}

static int
ath_stop_locked(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: invalid %u flags 0x%x\n",
		__func__, sc->sc_invalid, dev->flags);

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Shutdown the hardware and driver:
		 *    reset 802.11 state machine
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
		ieee80211_new_state(ic, IEEE80211_S_INIT, -1);
		netif_stop_queue(dev);
		if (sc->sc_rawdev_enabled) 
			netif_stop_queue(&sc->sc_rawdev);

		dev->flags &= ~IFF_RUNNING;
		if (!sc->sc_invalid) {
			if (sc->sc_softled) {
				del_timer(&sc->sc_ledtimer);
				ath_hal_gpioset(ah, sc->sc_ledpin,
					!sc->sc_ledon);
				sc->sc_blinking = 0;
			}
			ath5k_hw_set_intr(ah, 0);
		}
		ath_draintxq(sc);
		if (!sc->sc_invalid) {
			ath_stoprecv(sc);
			ath_hal_phydisable(ah);
		} else
			sc->sc_rxlink = NULL;
		ath_beacon_free(sc);
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

	down(&(sc)->sc_lock);
	error = ath_stop_locked(dev);
	if (error == 0 && !sc->sc_invalid) {
		/*
		 * Set the chip in full sleep mode.  Note that we are
		 * careful to do this only when bringing the interface
		 * completely to a stop.  When the chip is in this state
		 * it must be carefully woken up or references to
		 * registers in the PCI clock domain may freeze the bus
		 * (and system).  This varies by chip and is mostly an
		 * issue with newer parts that go to sleep more quickly.
		 */
		if (sc->sc_ah->ah_mac_version >= 7 && sc->sc_ah->ah_mac_revision >= 8) {
			/*
			 * XXX
			 * don't put newer MAC revisions > 7.8 to sleep because
			 * of the above mentioned problems
			 */
			DPRINTF(sc, ATH_DEBUG_RESET,
				"%s: mac version > 7.8, not putting device to sleep\n",
				__func__);
		}
		else {
			DPRINTF(sc, ATH_DEBUG_RESET,
				"%s: putting device to full sleep\n", __func__);
			ath_hal_setpower(sc->sc_ah, AR5K_PM_FULL_SLEEP, 0);
		}
	}
	up(&(sc)->sc_lock);
	return error;
}

static int
ar_device(int devid)
{
	switch (devid) {
	case PCI_PRODUCT_ATHEROS_AR5210_DEFAULT:
	case PCI_PRODUCT_ATHEROS_AR5210:
	case PCI_PRODUCT_ATHEROS_AR5210_AP:
		return 5210;
	case PCI_PRODUCT_ATHEROS_AR5211_DEFAULT:
	case PCI_PRODUCT_ATHEROS_AR5311:
	case PCI_PRODUCT_ATHEROS_AR5211_LEGACY:
	case PCI_PRODUCT_ATHEROS_AR5211_FPGA11B:
		return 5211;
	case PCI_PRODUCT_ATHEROS_AR5212_DEFAULT:
	case PCI_PRODUCT_ATHEROS_AR5212:
	case PCI_PRODUCT_ATHEROS_AR5212_FPGA:
	case PCI_PRODUCT_ATHEROS_AR5212_IBM:
	case PCI_PRODUCT_ATHEROS_AR5212_REV2:
	case PCI_PRODUCT_ATHEROS_AR5212_REV7:
	case PCI_PRODUCT_ATHEROS_AR5212_REV8:
	/*      case AR5212_AR2315_REV6:
	*      case AR5212_AR2315_REV7:
	*      case AR5212_AR2317_REV1: */
	case PCI_PRODUCT_ATHEROS_AR5212_0014:
	case PCI_PRODUCT_ATHEROS_AR5212_0015:
	case PCI_PRODUCT_ATHEROS_AR5212_0016:
	case PCI_PRODUCT_ATHEROS_AR5212_0017:
	case PCI_PRODUCT_ATHEROS_AR5212_0018:
	case PCI_PRODUCT_ATHEROS_AR5212_0019:
	case PCI_PRODUCT_ATHEROS_AR2413:
	case PCI_PRODUCT_ATHEROS_AR5413:
	case PCI_PRODUCT_ATHEROS_AR5424:
	/*      case AR5212_DEVID_FF19: */
		return 5212;
	/*Not-supported by OpenHAL*/

	/*      case AR5213_SREV_1_0:
	*      case AR5213_SREV_REG:
	*      case AR_SUBVENDOR_ID_NOG:
	*      case AR_SUBVENDOR_ID_NEW_A:
	*              return 5213; */
	default:
		return 0; /* unknown */
	}
}


static int 
ath_set_ack_bitrate(struct ath_softc *sc, int high)
{
	struct ath_hal *hal = sc->sc_ah;
	if (ar_device(sc->devid) == 5212 || ar_device(sc->devid) == 5213) {
		/* set ack to be sent at low bit-rate */
		/* registers taken from the OpenBSD 5212 HAL */
#define AR5K_AR5212_STA_ID1                     0x8004
#define AR5K_AR5212_STA_ID1_ACKCTS_6MB          0x01000000
#define AR5K_AR5212_STA_ID1_BASE_RATE_11B       0x02000000
		u_int32_t v = AR5K_AR5212_STA_ID1_BASE_RATE_11B | 
			AR5K_AR5212_STA_ID1_ACKCTS_6MB;
		if (high) {
			AR5K_REG_WRITE(AR5K_AR5212_STA_ID1, 
				AR5K_REG_READ(AR5K_AR5212_STA_ID1) & ~v);
		} else {
			AR5K_REG_WRITE(AR5K_AR5212_STA_ID1, 
				AR5K_REG_READ(AR5K_AR5212_STA_ID1) | v);
		}
		return 0;
	}
	return 1;
}


/*
 * Reset the hardware w/o losing operational state.  This is
 * basically a more efficient way of doing ath_stop, ath_init,
 * followed by state transitions to the current 802.11
 * operational state.  Used to recover from various errors and
 * to reset or reload hardware state.
 */
static int
ath_reset(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211_conf *conf = &sc->sc_hw->conf;
	enum ar5k_opmode opmode = sc->sc_opmode;;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_STATUS status;
	
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: resetting\n", dev->name);
	
	/*
	 * Convert to a Hardware channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	sc->sc_curchan.freq = conf->freq;
	sc->sc_curchan.channel_flags = conf->channel_val;

	ath5k_hw_set_intr(ah, 0);	/* disable interrupts */
	ath_draintxq(sc);		/* stop xmit side */
	ath_stoprecv(sc);		/* stop recv side */

	/* NB: indicate channel change so we do a full reset */
	if (!ath5k_hw_reset(ah, opmode, &sc->sc_curchan, TRUE, &status))
		if_printf(dev, "%s: unable to reset hardware: '%s' (%u)\n",
			__func__, hal_status_desc[status], status);
	ath_update_txpow(sc);		/* update tx power state */

	if (ath_startrecv(sc) != 0)	/* restart recv */
		if_printf(dev, "%s: unable to start recv logic\n", __func__);

	if (sc->sc_softled)
		ath5k_hw_set_gpio_output(ah, sc->sc_ledpin);

	/*
	 * We may be doing a reset in response to an ioctl
	 * that changes the channel so update any state that
	 * might change as a result.
	 */
	ath_chan_change(sc,  &sc->sc_curchan);
	if (sc->sc_beacons)
		ath_beacon_config(sc);	/* restart beacons */
	ath5k_hw_set_intr(ah, sc->sc_imask);

	ath_set_ack_bitrate(sc, sc->sc_ackrate);

        for (i = 0; i < sc->sc_hw->queues; i++)
		ieee80211_wake_queue(sc->sc_hw, i);
	return 0;
}


#define NUM_RADIOTAP_ELEMENTS 18

static int radiotap_elem_to_bytes[NUM_RADIOTAP_ELEMENTS] = 
	{8, /* IEEE80211_RADIOTAP_TSFT */
	 1, /* IEEE80211_RADIOTAP_FLAGS */
	 1, /* IEEE80211_RADIOTAP_RATE */
	 4, /* IEEE80211_RADIOTAP_CHANNEL */
	 2, /* IEEE80211_RADIOTAP_FHSS */
	 1, /* IEEE80211_RADIOTAP_DBM_ANTSIGNAL */
	 1, /* IEEE80211_RADIOTAP_DBM_ANTNOISE */
	 2, /* IEEE80211_RADIOTAP_LOCK_QUALITY */
	 2, /* IEEE80211_RADIOTAP_TX_ATTENUATION */
	 2, /* IEEE80211_RADIOTAP_DB_TX_ATTENUATION */
	 1, /* IEEE80211_RADIOTAP_DBM_TX_POWER */
	 1, /* IEEE80211_RADIOTAP_ANTENNA */
	 1, /* IEEE80211_RADIOTAP_DB_ANTSIGNAL */
	 1, /* IEEE80211_RADIOTAP_DB_ANTNOISE */
	 2, /* IEEE80211_RADIOTAP_RX_FLAGS */
	 2, /* IEEE80211_RADIOTAP_TX_FLAGS */
	 1, /* IEEE80211_RADIOTAP_RTS_RETRIES */
	 1, /* IEEE80211_RADIOTAP_DATA_RETRIES */
	};


/*
 * the following rt_* functions deal with verifying that a valid
 * radiotap header is on a packet as well as functions to extracting
 * what information is included.
 * XXX maybe these should go in ieee_radiotap.c
 */
static int rt_el_present(struct ieee80211_radiotap_header *th, u_int32_t element)
{
	if (element > NUM_RADIOTAP_ELEMENTS)
		return 0;
	return le32_to_cpu(th->it_present) & (1 << element);
}

static int rt_check_header(struct ieee80211_radiotap_header *th, int len) 
{
	int bytes = 0;
	int x = 0;
	if (th->it_version != 0) 
		return 0;

	if (le16_to_cpu(th->it_len) < sizeof(struct ieee80211_radiotap_header))
		return 0;
	
	for (x = 0; x < NUM_RADIOTAP_ELEMENTS; x++) {
		if (rt_el_present(th, x))
		    bytes += radiotap_elem_to_bytes[x];
	}

	if (le16_to_cpu(th->it_len) < sizeof(struct ieee80211_radiotap_header) + bytes) 
		return 0;
	
	if (le16_to_cpu(th->it_len) > len)
		return 0;

	return 1;
}

static u_int8_t *rt_el_offset(struct ieee80211_radiotap_header *th, u_int32_t element) {
	unsigned int x = 0;
	u_int8_t *offset = ((u_int8_t *) th) + sizeof(struct ieee80211_radiotap_header);
	for (x = 0; x < NUM_RADIOTAP_ELEMENTS && x < element; x++) {
		if (rt_el_present(th, x))
			offset += radiotap_elem_to_bytes[x];
	}

	return offset;
}

/* mcgrof XXX: Old pre-ng ath_start, remove */
/*
 * ath_start for raw 802.11 packets.
 */
static int
ath_start_raw(struct sk_buff *skb, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_txq *txq;
	struct ath_buf *bf;
	AR5K_PKT_TYPE atype;
	int pktlen, hdrlen, try0, pri, dot11_rate, txpower; 
	u_int8_t ctsrate, ctsduration, txrate;
	u_int flags = 0;
	struct ieee80211_frame *wh; 
	struct ath_desc *ds;
	const struct ar5k_rate_table *rt;
	uint8_t testmac[IEEE80211_ADDR_LEN];

	if ((sc->sc_dev.flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		/* device is not up... silently discard packet */
		dev_kfree_skb(skb);
		return 0;
	}

	/*
	 * Grab a TX buffer and associated resources.
	 */

	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));
	flags = AR5K_TXDESC_INTREQ | AR5K_TXDESC_CLRDMASK;
	try0 = ATH_TXMAXTRY;
	dot11_rate = 0;
	ctsrate = 0;
	ctsduration = 0;
	pri = 0;
	txpower = 60;
	txq = sc->sc_ac2q[pri];
	txrate = rt->rates[0].rate_code;
	atype =  AR5K_PKT_TYPE_NORMAL;

	/*
	 * strip any physical layer header off the skb, if it is
	 * present, and read out any settings we can, like what txrate
	 * to send this packet at.
	 */
	switch(dev->type) {
	case ARPHRD_IEEE80211:
		break;
	case ARPHRD_IEEE80211_PRISM: {
		wlan_ng_prism2_header *ph = NULL;
		ph = (wlan_ng_prism2_header *) skb->data;
		/* does it look like there is a prism header here? */
		if (skb->len > sizeof (wlan_ng_prism2_header) &&
		    ph->msgcode == DIDmsg_lnxind_wlansniffrm &&
		    ph->rate.did == DIDmsg_lnxind_wlansniffrm_rate) {
			dot11_rate = ph->rate.data;
			skb_pull(skb, sizeof(wlan_ng_prism2_header));
		} 
		break;
	}
	case ARPHRD_IEEE80211_RADIOTAP: {
		struct ieee80211_radiotap_header *th = (struct ieee80211_radiotap_header *) skb->data;
		if (rt_check_header(th, skb->len)) {
			if (rt_el_present(th, IEEE80211_RADIOTAP_RATE)) {
				dot11_rate = *((u_int8_t *) rt_el_offset(th, 
					      IEEE80211_RADIOTAP_RATE));
			}
			if (rt_el_present(th, IEEE80211_RADIOTAP_DATA_RETRIES)) {
				try0 = 1 + *((u_int8_t *) rt_el_offset(th, 
					      IEEE80211_RADIOTAP_DATA_RETRIES));
			}
			if (rt_el_present(th, IEEE80211_RADIOTAP_DBM_TX_POWER)) {
				txpower = *((u_int8_t *) rt_el_offset(th, 
					      IEEE80211_RADIOTAP_DBM_TX_POWER));
				if (txpower > 60) 
					txpower = 60;
				
			}

			skb_pull(skb, le16_to_cpu(th->it_len));
		}
		break;
	}
	default:
		/* nothing */
		break;
	}
	
	if (dot11_rate != 0) {
		int index = sc->sc_rixmap[dot11_rate & IEEE80211_RATE_VAL];
		if (index >= 0 && index < rt->rate_count) {
			txrate = rt->rates[index].rate_code;
		}
	}

	wh = (struct ieee80211_frame *) skb->data;
	pktlen = skb->len + IEEE80211_CRC_LEN;
	hdrlen = sizeof(struct ieee80211_frame);

	if (hdrlen < pktlen) 
		hdrlen = pktlen;

	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		flags |= AR5K_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
	}
	
	testmac[0] = 0x41; /* A */
	testmac[1] = 0x54; /* T */
	testmac[2] = 0x48; /* H */
	testmac[3] = 0x54; /* T */
	testmac[4] = 0x53; /* S */
	testmac[5] = 0x54; /* T */

	if (IEEE80211_ADDR_EQ(wh->i_addr1, testmac)) {
		flags |= AR5K_TXDESC_NOACK;	/* no ack for test packets */
		sc->sc_stats.ast_tx_noack++;
		DPRINTF(sc, ATH_DEBUG_XMIT, "%s: output testpacket (len %i)\n",
		    __func__, skb->len);
	}

	if (IFF_DUMPPKTS(sc, ATH_DEBUG_XMIT))
		ieee80211_dump_pkt(skb->data, skb->len,
				   sc->sc_hwmap[txrate].ieeerate, -1);

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
	bf->bf_skbaddr = pci_map_single(sc->sc_bdev,
		skb->data, pktlen, PCI_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %lx\n",
		__func__, skb, skb->data, skb->len, (long unsigned int) bf->bf_skbaddr);
	if (pci_dma_mapping_error(bf->bf_skbaddr)) {
		if_printf(dev, "%s: DMA mapping failed\n", __func__);
		dev_kfree_skb(skb);
		bf->bf_skb = NULL;
		sc->sc_stats.ast_tx_busdma++;

		spin_lock_bh(&(sc)->sc_txbuflock);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock_bh(&(sc)->sc_txbuflock);
		return -EIO;
	}
	bf->bf_skb = skb;
	bf->bf_node = NULL;

	/* setup descriptors */
	ds = bf->bf_desc;
	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	/* XXX check return value? */
	ath5k_hw_tx_desc_setup(ah, ds
		, pktlen		/* packet length */
		, hdrlen		/* header length */
		, atype			/* Atheros packet type */
		, txpower     	        /* txpower */
		, txrate, try0		/* series 0 rate/tries */
		, AR5K_TXKEYIX_INVALID	/* key cache index */
		, sc->sc_txantenna	/* antenna mode */
		, flags			/* flags */
		, ctsrate		/* rts/cts rate */
		, ctsduration		/* rts/cts duration */
	);

	/*
	 * Fillin the remainder of the descriptor info.
	 */
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath5k_hw_tx_desc_fill(ah, ds
		, skb->len		/* segment length */
		, TRUE		/* first segment */
		, TRUE		/* last segment */
		, ds			/* first descriptor */
	);

	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
	    __func__, txq->axq_qnum, ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
	/*
	 * Insert the frame on the outbound list and
	 * pass it on to the hardware.
	 */
	spin_lock_bh(&(txq)->axq_lock);
//	if (flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)) {
//		u_int32_t txopLimit = IEEE80211_TXOP_TO_US(
//			cap->cap_wmeParams[pri].wmep_txopLimit);
		/*
		 * When bursting, potentially extend the CTS duration
		 * of a previously queued frame to cover this frame
		 * and not exceed the txopLimit.  If that can be done
		 * then disable RTS/CTS on this frame since it's now
		 * covered (burst extension).  Otherwise we must terminate
		 * the burst before this frame goes out so as not to
		 * violate the WME parameters.  All this is complicated
		 * as we need to update the state of packets on the
		 * (live) hardware queue.  The logic is buried in the hal
		 * because it's highly chip-specific.
		 */
//		if (txopLimit != 0) {
//			sc->sc_stats.ast_tx_ctsburst++;
//			if (updateCTSForBursting(ah, ds, txq) == 0) {
				/*
				 * This frame was not covered by RTS/CTS from
				 * the previous frame in the burst; update the
				 * descriptor pointers so this frame is now
				 * treated as the last frame for extending a
				 * burst.
				 */
//				txq->axq_lastdsWithCTS = ds;
				/* set gating Desc to final desc */
//				txq->axq_gatingds =
//					(struct ath_desc *)txq->axq_link;
//			} else
//				sc->sc_stats.ast_tx_ctsext++;
//		}
//	}
	ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
	DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: txq depth = %d\n",
		__func__, txq->axq_depth);
	if (txq->axq_link == NULL) {
		ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: TXDP[%u] = %llx (%p) depth %d\n", __func__,
			txq->axq_qnum, ito64(bf->bf_daddr), bf->bf_desc,
			txq->axq_depth);
	} else {
		*txq->axq_link = bf->bf_daddr;
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: link[%u](%p)=%llx (%p) depth %d\n", __func__,
			txq->axq_qnum, txq->axq_link,
			ito64(bf->bf_daddr), bf->bf_desc, txq->axq_depth);
	}
	txq->axq_link = &bf->bf_desc->ds_link;
	/*
	 * The CAB queue is started from the SWBA handler since
	 * frames only go out on DTIM and to avoid possible races.
	 */
	if (txq != sc->sc_cabq)
		ath_hal_txstart(ah, txq->axq_qnum);
	spin_unlock_bh(&(txq)->axq_lock);

	sc->sc_devstats.tx_packets++;
	sc->sc_devstats.tx_bytes += skb->len;
	sc->sc_dev.trans_start = jiffies;
	sc->sc_rawdev.trans_start = jiffies;

	return 0;
}

/* From dadwifi */
static void
ath_tx_startraw(struct ath_softc *sc, struct ath_buf *bf, struct sk_buff *skb,
	       	struct ieee80211_tx_control *control, struct ath_txq *txq) 
{
	struct ath_hal *ah = sc->sc_ah;
	const struct ar5k_rate_table *rt;
	int pktlen;
	int hdrlen;
	AR5K_PKT_TYPE atype;
	u_int flags;
	int keyix;
	int try0;
	int power;
	u_int8_t antenna, txrate;
	struct ath_desc *ds=NULL;
	int header_len, header_pad;
	/* The Atheros hardware requires that the 802.11 header be
	 * padded out to a multiple of 4 bytes in length. */
	header_len = ieee80211_get_hdrlen_from_skb(skb);
	header_pad = (4 - (header_len & 3)) & 3;
	if (unlikely(header_pad)) {
		u8 *pos;
		pos = skb_push(skb, header_pad);
		memmove(pos, pos + header_pad, header_len);
        }
	try0 = control->retry_limit;
	rt = sc->sc_currates;
	txrate = control->tx_rate;
	/* FIXME : what unit does the hal need power is? */
	power = control->power_level > 60 ? 60 : control->power_level;
	bf->control = *control;
	hdrlen = ieee80211_get_hdrlen_from_skb(skb);
	pktlen = skb->len - header_pad + FCS_LEN;
	if (control->key_idx == HW_KEY_IDX_INVALID)
		keyix = AR5K_TXKEYIX_INVALID;
	else {
		keyix = control->key_idx;
		if (sc->sc_ath_keys[keyix].ak_alg == ALG_TKIP) {
			struct ieee80211_hdr *hdr;
			u16 fctl;
			u16 sctl;

			hdr = (struct ieee80211_hdr *) skb->data;
			fctl = le16_to_cpu(hdr->frame_control);
			sctl = le16_to_cpu(hdr->seq_ctrl);

			if (likely(!(fctl & IEEE80211_FCTL_MOREFRAGS) &&
				    (sctl & IEEE80211_SCTL_FRAG) == 0)) {

				/* hwaccel adds Michael MIC to the end of the
				 * payload of unfragmented frames. */
				pktlen += 8;
			}
		}
		pktlen += control->icv_len;
	}
	flags = AR5K_TXDESC_INTREQ | AR5K_TXDESC_CLRDMASK; /* XXX needed for crypto errs */
	bf->bf_skbaddr = pci_map_single(sc->sc_bdev,
					skb->data, pktlen, PCI_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %llx\n",
		__func__, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));
	bf->bf_skb = skb;
	bf->bf_node = NULL;
	/* setup descriptors */
	ds = bf->bf_desc;
	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_mode));
	if (control->flags & IEEE80211_TXCTL_NO_ACK)
		flags |= AR5K_TXDESC_NOACK;
	atype = AR5K_PKT_TYPE_NORMAL;		/* default */
	flags |= AR5K_TXDESC_INTREQ;
	antenna = sc->sc_txantenna;
	/* XXX check return value? */
	ath5k_hw_tx_desc_setup(ah, ds
			    , pktlen	/* packet length */
			    , hdrlen	/* header length */
			    , atype	/* Atheros packet type */
			    , power	/* txpower */
			    , txrate, try0 /* series 0 rate/tries */
			    , keyix	/* key cache index */
			    , antenna	/* antenna mode */
			    , flags	/* flags */
			    , 0		/* rts/cts rate */
			    , 0		/* rts/cts duration */
			   );
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath5k_hw_tx_desc_fill(ah, ds
			   , skb->len	/* segment length */
			   , TRUE	/* first segment */
			   , TRUE	/* last segment */
			   , ds		/* first descriptor */
			   );
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
		__func__, txq->axq_qnum, ds->ds_link, ds->ds_data,
		ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
	ath_tx_txqaddbuf(sc, txq, bf, ds, pktlen);
}

static int
ath_start(struct sk_buff *skb, struct net_device *dev)
{
#define CLEANUP()	\
	do{ \
		spin_lock_bh(&(sc)->sc_txbuflock); \
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list); \
		spin_unlock_bh(&(sc)->sc_txbuflock); \
		if (ni != NULL)	\
			ieee80211_free_node(ni);	\
	} while (0)

	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct ath_buf *bf;
	struct ieee80211_cb *cb;
	struct sk_buff *skb0;
	struct ieee80211_frame *wh;
	struct ether_header *eh;
	
	int ret = 0;
	int counter = 0;

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: discard, invalid %d flags %x\n",
			__func__, sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		return -ENETDOWN;
	}
	
	for (;;) {
		/*
		 * Grab a TX buffer and associated resources.
		 */
		spin_lock_bh(&(sc)->sc_txbuflock);
		bf = STAILQ_FIRST(&sc->sc_txbuf);
		if (bf != NULL)
			STAILQ_REMOVE_HEAD(&sc->sc_txbuf, bf_list);
		/* XXX use a counter and leave at least one for mgmt frames */
		if (STAILQ_EMPTY(&sc->sc_txbuf)) {
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: stop queue\n", __func__);
			sc->sc_stats.ast_tx_qstop++;
			netif_stop_queue(dev);
			if (sc->sc_rawdev_enabled)
				netif_stop_queue(&sc->sc_rawdev);
		}
		spin_unlock_bh(&(sc)->sc_txbuflock);

		if (bf == NULL) {		/* NB: should not happen */
			DPRINTF(sc, ATH_DEBUG_ANY, "%s: out of xmit buffers\n",
				__func__);
			sc->sc_stats.ast_tx_nobuf++;
			break;
		}

		/*
		 * Poll the management queue for frames; they
		 * have priority over normal data frames.
		 */
		IF_DEQUEUE(&ic->ic_mgtq, skb0);
		
		if (skb0 == NULL) {
			/*
			 * data frames
			 */
			if (counter++ > 200)
			    DPRINTF(sc, ATH_DEBUG_FATAL, "%s (%s): endlessloop (data) (counter=%i)\n", __func__, dev->name, counter);

			if (!skb) {		/* NB: no data (called for mgmt) */
				spin_lock_bh(&(sc)->sc_txbuflock);
				STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
				spin_unlock_bh(&(sc)->sc_txbuflock);
				break;
			}
			/*
			 * No data frames go out unless we're associated; this
			 * should not happen as the 802.11 layer does not enable
			 * the xmit queue until we enter the RUN state.
			 */
			if (ic->ic_state != IEEE80211_S_RUN) {
				DPRINTF(sc, ATH_DEBUG_ANY,
					"%s: ignore data packet, state %u\n",
					__func__, ic->ic_state);
				sc->sc_stats.ast_tx_discard++;
				spin_lock_bh(&(sc)->sc_txbuflock);
				STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
				spin_unlock_bh(&(sc)->sc_txbuflock);
				break;
			}
			/* 
			 * Find the node for the destination so we can do
			 * things like power save and fast frames aggregation.
			 */
			if (skb->len < sizeof(struct ether_header)) {
				ic->ic_stats.is_tx_nobuf++;   /* XXX */
				ni = NULL;

				ret = 0; /* error return */
				CLEANUP();
				break;
			}
			eh = (struct ether_header *)skb->data;
			ni = ieee80211_find_txnode(ic, eh->ether_dhost);
			if (ni == NULL) {
				/* NB: ieee80211_find_txnode does stat+msg */

				ret = 0; /* error return */
				CLEANUP();
				break;
			}
			cb = (struct ieee80211_cb *)skb->cb;
			if ((ni->ni_flags & IEEE80211_NODE_PWR_MGT) &&
			    (cb->flags & M_PWR_SAV) == 0) {
				/*
				 * Station in power save mode; pass the frame
				 * to the 802.11 layer and continue.  We'll get
				 * the frame back when the time is right.
				 */
				ieee80211_pwrsave(ic, ni, skb);
				/* don't free this on function exit point */
				skb = NULL;
				CLEANUP();
				break;
			}
			/* calculate priority so we can find the tx queue */
			if (ieee80211_classify(ic, skb, ni)) {
				DPRINTF(sc, ATH_DEBUG_XMIT,
					"%s: discard, classification failure\n",
					__func__);

				ret = 0; /* error return */
				CLEANUP();
				break;
			}
			sc->sc_devstats.tx_packets++;
			sc->sc_devstats.tx_bytes += skb->len;

			/*
			 * Encapsulate the packet for transmission.
			 */
			skb = ieee80211_encap(ic, skb, ni);
			if (skb == NULL) {
				DPRINTF(sc, ATH_DEBUG_ANY,
					"%s: encapsulation failure\n",
					__func__);
				sc->sc_stats.ast_tx_encap++;

				ret = 0; /* error return */
				CLEANUP();
				break;
			}

			/*
			 * using unified sk_buff for transmit data and management frames
			 */
			skb0 = skb;
		} else {
			/*
			 * management frames
			 */
			cb = (struct ieee80211_cb *)skb0->cb;
			ni = cb->ni;

			if (counter++ > 200)
			    DPRINTF(sc, ATH_DEBUG_FATAL, "%s (%s): endlessloop (mgnt) (counter=%i)\n", __func__, dev->name, counter);

			wh = (struct ieee80211_frame *) skb0->data;
			if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) == 
				IEEE80211_FC0_SUBTYPE_PROBE_RESP) {
				/* fill time stamp */
				u_int64_t tsf;
				__le32 *tstamp;

				tsf = ath_hal_gettsf64(ah);
				/* XXX: adjust 100us delay to xmit */
				tsf += 100;
				tstamp = (__le32 *)&wh[1];
				tstamp[0] = htole32(tsf & 0xffffffff);
				tstamp[1] = htole32(tsf >> 32);
			}
			sc->sc_stats.ast_tx_mgmt++;
		}

		if (ath_tx_start(dev, ni, bf, skb0)) {
			ret = 0; 	/* TODO: error value */
			skb = NULL;	/* ath_tx_start() already freed this */
			CLEANUP();
			continue;
		}
		/*
		 * the data frame is last
		 */
		if (skb0 == skb) {
			skb = NULL;	/* will be released by tx_processq */
			break; 
		}
		sc->sc_tx_timer = 5;
		mod_timer(&ic->ic_slowtimo, jiffies + HZ);
	}
	if (skb)
		dev_kfree_skb(skb);
	return ret;	/* NB: return !0 only in a ``hard error condition'' */
#undef CLEANUP
}

static int
ath_media_change(struct net_device *dev)
{
#define	IS_UP(dev) \
	((dev->flags & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))
	int error;

	error = ieee80211_media_change(dev);
	if (error == ENETRESET) {
		if (IS_UP(dev))
			error = ath_init(dev);
		else
			error = 0;
	}
	return error;
#undef IS_UP
}

#ifdef AR_DEBUG
static void
ath_keyprint(const char *tag, u_int ix,
	const AR5K_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
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

	printk("%s: [%02u] %-7s ", tag, ix, ciphers[hk->wk_type]);
	for (i = 0, n = hk->wk_len; i < n; i++)
		printk("%02x", hk->wk_key[i]);
	printk(" mac %s", ether_sprintf(mac));
	if (hk->wk_type == AR5K_CIPHER_TKIP) {
		printk(" mic ");
		for (i = 0; i < sizeof(hk->wk_mic); i++)
			printk("%02x", hk->wk_mic[i]);
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
	AR5K_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
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
		memcpy(hk->wk_mic, k->wk_txmic, sizeof(hk->wk_mic));
		KEYPRINTF(sc, k->wk_keyix, hk, zerobssid);
		if (!ath_hal_keyset(ah, k->wk_keyix, hk, zerobssid))
			return 0;

		memcpy(hk->wk_mic, k->wk_rxmic, sizeof(hk->wk_mic));
		KEYPRINTF(sc, k->wk_keyix+32, hk, mac);
		/* XXX delete tx key on failure? */
		return ath_hal_keyset(ah, k->wk_keyix+32, hk, mac);
	} else if (k->wk_flags & IEEE80211_KEY_XR) {
		/*
		 * TX/RX key goes at first index.
		 * The hal handles the MIC keys are index+64.
		 */
		memcpy(hk->wk_mic, k->wk_flags & IEEE80211_KEY_XMIT ?
			k->wk_txmic : k->wk_rxmic, sizeof(hk->wk_mic));
		KEYPRINTF(sc, k->wk_keyix, hk, mac);
		return ath_hal_keyset(ah, k->wk_keyix, hk, mac);
	}
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
	const u_int8_t mac0[IEEE80211_ADDR_LEN],
	struct ieee80211_node *bss)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	static const u_int8_t ciphermap[] = {
		AR5K_CIPHER_WEP,		/* IEEE80211_CIPHER_WEP */
		AR5K_CIPHER_TKIP,	/* IEEE80211_CIPHER_TKIP */
		AR5K_CIPHER_AES_OCB,	/* IEEE80211_CIPHER_AES_OCB */
		AR5K_CIPHER_AES_CCM,	/* IEEE80211_CIPHER_AES_CCM */
		(u_int8_t) -1,		/* 4 is not allocated */
		AR5K_CIPHER_CKIP,	/* IEEE80211_CIPHER_CKIP */
		AR5K_CIPHER_CLR,		/* IEEE80211_CIPHER_NONE */
	};
	struct ath_hal *ah = sc->sc_ah;
	const struct ieee80211_cipher *cip = k->wk_cipher;
	u_int8_t gmac[IEEE80211_ADDR_LEN];
	const u_int8_t *mac;
	AR5K_KEYVAL hk;

	memset(&hk, 0, sizeof(hk));
	/*
	 * Software crypto uses a "clear key" so non-crypto
	 * state kept in the key cache are maintained and
	 * so that rx frames have an entry to match.
	 */
	if ((k->wk_flags & IEEE80211_KEY_SWCRYPT) == 0) {
		KASSERT(cip->ic_cipher < N(ciphermap),
			("invalid cipher type %u", cip->ic_cipher));
		hk.wk_type = ciphermap[cip->ic_cipher];
		hk.wk_len = k->wk_keylen;
		memcpy(hk.wk_key, k->wk_key, k->wk_keylen);
	} else
		hk.wk_type = AR5K_CIPHER_CLR;

        if ((k->wk_flags & IEEE80211_KEY_GROUP) && sc->sc_mcastkey) {
                /*
                 * Group keys on hardware that supports multicast frame
                 * key search use a mac that is the sender's address with
                 * the high bit set instead of the app-specified address.
                 */
                IEEE80211_ADDR_COPY(gmac, bss->ni_macaddr);
                gmac[0] |= 0x80;
                mac = gmac;
        } else
                mac = mac0;

	if (hk.wk_type == AR5K_CIPHER_TKIP &&
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
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: key pair %u,%u %u,%u\n",
				__func__, keyix, keyix+64,
				keyix+32, keyix+32+64);
			return keyix;
		}
	}
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
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
			DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: key %u\n",
				__func__, keyix);
			return keyix;
		}
	}
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of space\n", __func__);
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
	 * Group key allocation must be handled specially for
	 * parts that do not support multicast key cache search
	 * functionality.  For those parts the key id must match
	 * the h/w key index so lookups find the right key.  On
	 * parts w/ the key search facility we install the sender's
	 * mac address (with the high bit set) and let the hardware
	 * find the key w/o using the key id.  This is preferred as
	 * it permits us to support multiple users for adhoc and/or
	 * multi-station operation.
	 */
	if ((k->wk_flags & IEEE80211_KEY_GROUP) && !sc->sc_mcastkey) {
		u_int keyix;

		if (!(&ic->ic_nw_keys[0] <= k &&
		      k < &ic->ic_nw_keys[AR5K_WEP_NKID])) {
			/* should not happen */
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: bogus group key\n", __func__);
			return IEEE80211_KEYIX_NONE;
		}
		keyix = k - ic->ic_nw_keys;
		/*
		 * XXX we pre-allocate the global keys so
		 * have no way to check if they've already been allocated.
		 */
		return keyix;
	}

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
        struct ieee80211_node *ni;
	u_int keyix = k->wk_keyix;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: delete key %u\n", __func__, keyix);

	ath_hal_keyreset(ah, keyix);
        /*
         * Check the key->node map and flush any ref.
         */
        ni = sc->sc_keyixmap[keyix];
        if (ni != NULL) {
                ieee80211_free_node(ni);
                sc->sc_keyixmap[keyix] = NULL;
        }
	/*
	 * Handle split tx/rx keying required for TKIP with h/w MIC.
	 */
	if (cip->ic_cipher == IEEE80211_CIPHER_TKIP &&
            (k->wk_flags & IEEE80211_KEY_SWMIC) == 0 && sc->sc_splitmic) {
		ath_hal_keyreset(ah, keyix+32);		/* RX key */
                ni = sc->sc_keyixmap[keyix+32];
                if (ni != NULL) {                       /* as above... */
                        ieee80211_free_node(ni);
                        sc->sc_keyixmap[keyix+32] = NULL;
                }
        }
	if (keyix >= AR5K_WEP_NKID) {
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

	return ath_keyset(sc, k, mac, ic->ic_bss);
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
	
	DPRINTF(sc, ATH_DEBUG_FATAL, "%lu %s (%s)\n", jiffies, __func__, dev->name);
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	/*
	 * When called from the rx tasklet we cannot use
	 * tasklet_disable because it will block waiting
	 * for us to complete execution.
	 *
	 * XXX Using in_softirq is not right since we might
	 * be called from other soft irq contexts than
	 * ath_rx_tasklet.
	 * TODO: can cause bugs
	 */
#if 1
	if (!in_softirq())
		tasklet_disable(&sc->sc_rxtq);
#endif
	netif_stop_queue(dev);	// TODO: find a way to not block mgmt frames
	if (sc->sc_rawdev_enabled) 
		netif_stop_queue(&sc->sc_rawdev);
}

static void
ath_key_update_end(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_FATAL, "%lu %s (%s)\n", jiffies, __func__, dev->name);
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	netif_start_queue(dev);
	if (sc->sc_rawdev_enabled) 
		netif_start_queue(&sc->sc_rawdev);
#if 1
	if (!in_softirq())		/* NB: see above */
		tasklet_enable(&sc->sc_rxtq);
#endif
}

/*
 * Calculate the receive filter according to the
 * operating mode and state:
 *
 * o always accept unicast, broadcast, and multicast traffic
 * o maintain current state of phy error reception (the hal
 *   may enable phy error frames for noise immunity work)
 * o probe request frames are accepted only when operating in
 *   hostap, adhoc, or monitor modes
 * o enable promiscuous mode according to the interface state
 * o accept beacons:
 *   - when operating in adhoc mode so the 802.11 layer creates
 *     node table entries for peers,
 *   - when operating in station mode for collecting rssi data when
 *     the station is otherwise quiet, or
 *   - when scanning
 * o accept any additional packets specified by sc_rxfilter
 */
static u_int32_t
ath_calcrxfilter(struct ath_softc *sc, enum ieee80211_state state)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct net_device *dev = ic->ic_dev;
	u_int32_t rfilt;

	rfilt = (ath_hal_getrxfilter(ah) & AR5K_RX_FILTER_PHYERROR)
	      | AR5K_RX_FILTER_UCAST | AR5K_RX_FILTER_BCAST | AR5K_RX_FILTER_MCAST | AR5K_RX_FILTER_PHYRADAR;
	if (ic->ic_opmode != IEEE80211_M_STA && 
	    ic->ic_opmode != IEEE80211_M_AHDEMO)
		rfilt |= AR5K_RX_FILTER_PROBEREQ;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP &&
	    (dev->flags & IFF_PROMISC))
		rfilt |= AR5K_RX_FILTER_PROM;
	if (ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_IBSS ||
	    state == IEEE80211_S_SCAN)
		rfilt |= AR5K_RX_FILTER_BEACON;

	if (sc->sc_rawdev_enabled && (sc->sc_rawdev.flags & IFF_UP)) {
		/* if the rawdev is up, accept all normal packets */
		rfilt |= (AR5K_RX_FILTER_CONTROL | AR5K_RX_FILTER_BEACON | AR5K_RX_FILTER_PROM |
			AR5K_RX_FILTER_PROBEREQ);
	}

	rfilt |= sc->sc_rxfilter;

	return rfilt;
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

	/* configure rx filter */
	rfilt = ath_calcrxfilter(sc, ic->ic_state);
	ath_hal_setrxfilter(ah, rfilt);

	/* configure operational mode */
	ath_hal_setopmode(ah);

	/*
	 * Handle any link-level address change.  Note that we only
	 * need to force ic_myaddr; any other addresses are handled
	 * as a byproduct of the ifnet code marking the interface
	 * down then up.
	 *
	 * XXX should get from lladdr instead of arpcom but that's more work
	 */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, dev->dev_addr);
	ath_hal_setmac(ah, ic->ic_myaddr);

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
	DPRINTF(sc, ATH_DEBUG_MODE, "%s: RX filter 0x%x, MC filter %08x:%08x\n",
		__func__, rfilt, mfilt[0], mfilt[1]);
}

/*
 * Set the slot time based on the current setting.
 * This is called by ath_updateslot below and when a non-ERP node
 * joins the network
 */
static void
ath_setslottime(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	/* If the user has asked to lock the slot-time, ignore the
	 * slot time adjustment. This is useful in longer-range networks
	 * that require a slot time larger than the standard ones. -TvE
	 */
	if (!sc->sc_lockslottime) {
		if (ic->ic_flags & IEEE80211_F_SHSLOT)
			ath_hal_setslottime(ah, AR5K_SLOT_TIME_9);
		else
			ath_hal_setslottime(ah, AR5K_SLOT_TIME_20);
	}
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
	 * When not coordinating the BSS, change the hardware
	 * immediately.  For other operation we defer the change
	 * until beacon updates have propagated to the stations.
	 */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		sc->sc_updateslot = UPDATE;
	else
		ath_setslottime(sc);
}

/*
 * Setup a h/w transmit queue for beacons.
 */
static int
ath_beaconq_setup(struct ath_hal *ah)
{
	AR5K_TXQ_INFO qi;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_aifs = AR5K_TXQ_USEDEFAULT;
	qi.tqi_cw_min = AR5K_TXQ_USEDEFAULT;
	qi.tqi_cw_max = AR5K_TXQ_USEDEFAULT;
        /* NB: for dynamic turbo, don't enable any other interrupts */
        qi.tqi_flags = AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
        return ath_hal_setuptxqueue(ah, AR5K_TX_QUEUE_BEACON, &qi);
}

/*
 * Setup the IFS parameters for the beacon queue.
 */
static int
ath_beaconq_config(struct ath_softc *sc)
{
        struct ath_hal *ah = sc->sc_ah;
        AR5K_TXQ_INFO qi;

        ath5k_hw_get_tx_queueprops(ah, sc->sc_bhalq, &qi);
        if (sc->sc_opmode == AR5K_M_HOSTAP) {
                /*
                 * Always burst out beacon and CAB traffic.
                 */
                qi.tqi_aifs = ATH_BEACON_AIFS_DEFAULT;
                qi.tqi_cw_min = ATH_BEACON_CWMIN_DEFAULT;
                qi.tqi_cw_max = ATH_BEACON_CWMAX_DEFAULT;
        }

        if (!ath5k_hw_setup_tx_queueprops(ah, sc->sc_bhalq, &qi)) {
                printk("%s: unable to update h/w beacon queue parameters\n"
                        "beacon hardware queue! (%s)\n", __func__, 
			sc->sc_dev.name);
                return 0;
        } else {
                ath5k_hw_reset_tx_queue(ah, sc->sc_bhalq); /* push to h/w */
                return 1;
        }
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

	bf = STAILQ_FIRST(&sc->sc_bbuf);
	if (bf == NULL) {
		DPRINTF(sc, ATH_DEBUG_BEACON, "%s: no dma buffers\n", __func__);
		sc->sc_stats.ast_be_nobuf++;	/* XXX */
		return ENOMEM;			/* XXX */
	}
	/*
	 * NB: the beacon data buffer must be 32-bit aligned;
	 * we assume the mbuf routines will return us something
	 * with this alignment (perhaps should assert).
	 */
	skb = ieee80211_beacon_alloc(ic, ni, &sc->sc_boff);
	if (skb == NULL) {
		DPRINTF(sc, ATH_DEBUG_BEACON, "%s: cannot get sk_buff\n",
			__func__);
		sc->sc_stats.ast_be_nobuf++;
		return ENOMEM;
	}

	if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
	}
	bf->bf_skb = skb;
	bf->bf_node = ieee80211_ref_node(ni);

	return 0; // TODO: return value
}

/*
 * Setup the beacon frame for transmit.
 */
static void
ath_beacon_setup(struct ath_softc *sc, struct ath_buf *bf)
{
#define	USE_SHPREAMBLE(_ic) \
	(((_ic)->ic_flags & (IEEE80211_F_SHPREAMBLE | IEEE80211_F_USEBARKER))\
		== IEEE80211_F_SHPREAMBLE)
	struct ieee80211_node *ni = bf->bf_node;
	struct ieee80211com *ic = ni->ni_ic;
	struct sk_buff *skb = bf->bf_skb;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_node *an = ATH_NODE(ni);
	struct ath_desc *ds;
	int antenna = sc->sc_txantenna;
	int flags;
	u_int8_t rate;

	bf->bf_skbaddr = pci_map_single(sc->sc_bdev,
		skb->data, skb->len, PCI_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_BEACON,
		"%s: skb %p [data %p len %u] skbaddr %llx\n",
		__func__, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));
	if (pci_dma_mapping_error(bf->bf_skbaddr)) {
		if_printf(&sc->sc_dev, "%s: DMA mapping failed\n", __func__);
		return;
	}
	
	/* setup descriptors */
	ds = bf->bf_desc;

	flags = AR5K_TXDESC_NOACK;
	if (ic->ic_opmode == IEEE80211_M_IBSS && sc->sc_hasveol) {
		ds->ds_link = bf->bf_daddr;	/* self-linked */
		flags |= AR5K_TXDESC_VEOL;
		/*
		 * Let hardware handle antenna switching if txantenna is not set
		 */
	} else {
		ds->ds_link = 0;
		/*
		 * Switch antenna every 4 beacons if txantenna is not set
		 * XXX assumes two antenna
		 */
		if (antenna == 0) {
			antenna = (sc->sc_stats.ast_be_xmit & 4 ? 2 : 1);
		}
	}

	ds->ds_data = bf->bf_skbaddr;
	/*
	 * Calculate rate code.
	 * XXX everything at min xmit rate
	 */
	if (USE_SHPREAMBLE(ic))
		rate = an->an_tx_mgtratesp;
	else
		rate = an->an_tx_mgtrate;
	ath5k_hw_tx_desc_setup(ah, ds
		, skb->len + IEEE80211_CRC_LEN	/* frame length */
		, sizeof(struct ieee80211_frame)/* header length */
		, AR5K_PKT_TYPE_BEACON		/* Atheros packet type */
		, ni->ni_txpower		/* txpower XXX */
		, rate, 1			/* series 0 rate/tries */
		, AR5K_TXKEYIX_INVALID		/* no encryption */
		, antenna			/* antenna mode */
		, flags				/* no ack, veol for beacons */
		, 0				/* rts/cts rate */
		, 0				/* rts/cts duration */
	);
	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	ath5k_hw_tx_desc_fill(ah, ds
		, roundup(skb->len, 4)		/* buffer length */
		, TRUE			/* first segment */
		, TRUE			/* last segment */
		, ds				/* first descriptor */
	);
#undef USE_SHPREAMBLE
}

/*
 * Transmit a beacon frame at SWBA.  Dynamic updates to the
 * frame contents are done as needed and the slot time is
 * also adjusted based on current state.
 *
 * this is usually called from interrupt context (ath5k_intr())
 * but also from ath_beacon_config() in IBSS mode which in turn
 * can be called from a tasklet and user context
 */
static void
ath_beacon_send(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_buf *bf = STAILQ_FIRST(&sc->sc_bbuf);
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;
	int ncabq, otherant;

	BUG_ON(ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_MONITOR)
	/*
	 * Check if the previous beacon has gone out.  If
	 * not don't don't try to post another, skip this
	 * period and wait for the next.  Missed beacons
	 * indicate a problem and should not occur.  If we
	 * miss too many consecutive beacons reset the device.
	 */
	if (ath5k_hw_num_tx_pending(ah, sc->sc_bhalq) != 0) {
		sc->sc_bmisscount++;
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: missed %u consecutive beacons\n",
			__func__, sc->sc_bmisscount);
		if (sc->sc_bmisscount > 10) { /* this is a guess */
			DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: stuck beacon time (%u missed)\n",
			__func__, sc->sc_bmisscount);
			tasklet_schedule(&sc->sc_resettq);
                }
		return;
	}
	if (sc->sc_bmisscount != 0) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: resume beacon xmit after %u misses\n",
			__func__, sc->sc_bmisscount);
		sc->sc_bmisscount = 0;
	}

	/*
	 * Update dynamic beacon contents.  If this returns
	 * non-zero then we need to remap the memory because
	 * the beacon frame changed size (probably because
	 * of the TIM bitmap).
	 */
	skb = bf->bf_skb;
	ncabq = sc->sc_cabq->axq_depth;

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
	 * Check recent per-antenna transmit statistics and flip
	 * the default antenna if noticeably more frames went out
	 * on the non-default antenna.
	 * XXX assumes 2 anntenae
	 */
	otherant = sc->sc_defant & 1 ? 2 : 1;
	if (sc->sc_ant_tx[otherant] > sc->sc_ant_tx[sc->sc_defant] + 2)
		ath_setdefantenna(sc, otherant);
	sc->sc_ant_tx[1] = sc->sc_ant_tx[2] = 0;

	/*
	 * Construct tx descriptor.
	 */
	ath_beacon_setup(sc, bf);

	/*
	 * Stop any current dma and put the new frame on the queue.
	 * This should never fail since we check above that no frames
	 * are still pending on the queue.
	 */
	if (ath5k_hw_stop_rx_dma(ah, sc->sc_bhalq)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: beacon queue %u did not stop?\n",
			__func__, sc->sc_bhalq);
		/* NB: the HAL still stops DMA, so proceed */
	}
	pci_dma_sync_single_for_cpu(sc->sc_bdev,
		bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);

	/*
	 * Enable the CAB queue before the beacon queue to
	 * insure CAB frames are triggered by this beacon.
	 * The CAB queue holds multicast traffic for stations in 
	 * power-save mode.
	 *
	 * NB: only at DTIM
	 */
	ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
	ath_hal_txstart(ah, sc->sc_bhalq);
	DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
		"%s: TXDP[%u] = %llx (%p)\n", __func__,
		sc->sc_bhalq, ito64(bf->bf_daddr), bf->bf_desc);

	sc->sc_stats.ast_be_xmit++;
}

/*
 * Reclaim beacon resources.
 */
static void
ath_beacon_free(struct ath_softc *sc)
{
	struct ath_buf *bf;

	STAILQ_FOREACH(bf, &sc->sc_bbuf, bf_list) {
            if (bf->bf_skb != NULL) {
		pci_unmap_single(sc->sc_bdev,
                    bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
                dev_kfree_skb(bf->bf_skb);
                bf->bf_skb = NULL;
            }
            if (bf->bf_node != NULL) {
                ieee80211_free_node(bf->bf_node);
                bf->bf_node = NULL;
            }
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
#define TSF_TO_TU(_h,_l)        (((_h) << 22) | ((_l) >> 10))
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni = ic->ic_bss;
	u_int32_t nexttbtt, intval;
	u_int64_t tsf;
	u_int32_t tsftu;
	
	/* extract tstamp from last beacon and convert to TU */
	nexttbtt = TSF_TO_TU(LE_READ_4(ni->ni_tstamp.data + 4),
			     LE_READ_4(ni->ni_tstamp.data));
	/* NB: the beacon interval is kept internally in TU's */
	intval = ni->ni_intval & AR5K_BEACON_PERIOD;
	/* current TSF converted to TU */
	tsf = ath_hal_gettsf64(ah);
	tsftu = TSF_TO_TU((u_int32_t)(tsf>>32), (u_int32_t)tsf);
	
	DPRINTF(sc, ATH_DEBUG_BEACON, "%s: last beacon %u intval %u (%u) hw tsftu %u\n",
		__func__, nexttbtt, intval, ni->ni_intval, tsftu);
	
	if (ic->ic_opmode == IEEE80211_M_STA) {
		AR5K_BEACON_STATE bs;
		int dtimperiod, dtimcount;
		int cfpperiod, cfpcount;

		/*
		 * Setup dtim and cfp parameters according to
		 * last beacon we received (which may be none).
		 */
		dtimperiod = ni->ni_dtim_period;
		if (dtimperiod <= 0)            /* NB: 0 if not known */
			dtimperiod = 1;
		dtimcount = ni->ni_dtim_count;
		if (dtimcount >= dtimperiod)    /* NB: sanity check */
			dtimcount = 0;          /* XXX? */
		cfpperiod = 1;                  /* NB: no PCF support yet */
		cfpcount = 0;
#define FUDGE   2
		/*
		 * Pull nexttbtt forward to reflect the current
		 * TSF and calculate dtim+cfp state for the result.
		 */
		do {
			nexttbtt += intval;
			if (--dtimcount < 0) {
				dtimcount = dtimperiod - 1;
				if (--cfpcount < 0)
					cfpcount = cfpperiod - 1;
			}
		} while (nexttbtt < tsftu + FUDGE);
#undef FUDGE
		memset(&bs, 0, sizeof(bs));
		bs.bs_interval = intval;
		bs.bs_next_beacon = nexttbtt;
		bs.bs_dtim_period = dtimperiod*intval;
		bs.bs_next_dtim = bs.bs_next_beacon + dtimcount*intval;
		bs.bs_cfp_period = cfpperiod*bs.bs_dtim_period;
		bs.bs_cfp_next = bs.bs_next_dtim + cfpcount*bs.bs_dtim_period;
		bs.bs_cfp_max_duration = 0;
#if 0
		/*
		 * The 802.11 layer records the offset to the DTIM
		 * bitmap while receiving beacons; use it here to
		 * enable h/w detection of our AID being marked in
		 * the bitmap vector (to indicate frames for us are
		 * pending at the AP).
		 * XXX do DTIM handling in s/w to WAR old h/w bugs
		 * XXX enable based on h/w rev for newer chips
		 */
		bs.bs_tim_offset = ni->ni_timoff;
#endif
		/*
		 * Calculate the number of consecutive beacons to miss
		 * before taking a BMISS interrupt.  The configuration
		 * is specified in ms, so we need to convert that to
		 * TU's and then calculate based on the beacon interval.
		 * Note that we clamp the result to at most 10 beacons.
		 */
		bs.bs_bmiss_threshold = howmany(ic->ic_bmisstimeout, intval);
		if (bs.bs_bmiss_threshold > 10)
			bs.bs_bmiss_threshold = 10;
		else if (bs.bs_bmiss_threshold <= 0)
			bs.bs_bmiss_threshold = 1;

		/*
		 * Calculate sleep duration.  The configuration is
		 * given in ms.  We insure a multiple of the beacon
		 * period is used.  Also, if the sleep duration is
		 * greater than the DTIM period then it makes senses
		 * to make it a multiple of that.
		 *
		 * XXX fixed at 100ms
		 */
		bs.bs_sleep_duration =
			roundup(IEEE80211_MS_TO_TU(100), bs.bs_interval);
		if (bs.bs_sleep_duration > bs.bs_dtim_period)
			bs.bs_sleep_duration = roundup(bs.bs_sleep_duration, bs.bs_dtim_period);

		DPRINTF(sc, ATH_DEBUG_BEACON, 
			"%s: tsf %llx tsf:tu %u intval %u nexttbtt %u dtim %u nextdtim %u bmiss %u sleep %u cfp:period %u maxdur %u next %u timoffset %u\n"
			, __func__
			, tsf, tsftu
			, bs.bs_interval
			, bs.bs_next_beacon
			, bs.bs_dtim_period
			, bs.bs_next_dtim
			, bs.bs_bmiss_threshold
			, bs.bs_sleep_duration
			, bs.bs_cfp_period
			, bs.bs_cfp_max_duration
			, bs.bs_cfp_next
			, bs.bs_tim_offset
		);
		ath5k_hw_set_intr(ah, 0);
		ath_hal_beacontimers(ah, &bs);
		sc->sc_imask |= AR5K_INT_BMISS;
		sc->sc_bmisscount = 0;
		ath5k_hw_set_intr(ah, sc->sc_imask);
	} else { /* IBSS or HOSTAP */
		ath5k_hw_set_intr(ah, 0);
		
		if (ic->ic_opmode == IEEE80211_M_IBSS) {
			/*
			 * Pull nexttbtt forward to reflect the current
			 * TSF. Add one intval otherwise the timespan
			 * can be too short for ibss merges.
			 */
			do {
				nexttbtt += intval;
			} while (nexttbtt < tsftu+intval);
			nexttbtt += intval;
		
			DPRINTF(sc, ATH_DEBUG_BEACON, "%s: nexttbtt %u intval %u\n",
				__func__, nexttbtt, intval & AR5K_BEACON_PERIOD );
			
			/*
			 * In IBSS mode enable the beacon timers but only
			 * enable SWBA interrupts if we need to manually
			 * prepare beacon frames.  Otherwise we use a
			 * self-linked tx descriptor and let the hardware
			 * deal with things.
			 */
			if (!sc->sc_hasveol)
				sc->sc_imask |= AR5K_INT_SWBA;
		
		} else if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			if (nexttbtt == 0) {
				/*
				 * starting a new BSS: we can reset the TSF 
				 * and start with zero
				 */
				nexttbtt = intval;
				intval |= AR5K_BEACON_RESET_TSF;
			} else {
				nexttbtt += intval;
			}
			/*
			 * In AP mode we enable the beacon timers and
			 * SWBA interrupts to prepare beacon frames.
			 */
			sc->sc_imask |= AR5K_INT_SWBA;	/* beacon prepare */
		}
		
		intval |= AR5K_BEACON_ENA;
		
		ath_beaconq_config(sc);
		ath_hal_beaconinit(ah, nexttbtt, intval);
		
		sc->sc_bmisscount = 0;
		ath5k_hw_set_intr(ah, sc->sc_imask);
		/*
		 * When using a self-linked beacon descriptor in
		 * ibss mode load it once here.
		 */
		if (ic->ic_opmode == IEEE80211_M_IBSS && sc->sc_hasveol)
			ath_beacon_send(&sc->sc_dev);
	}
#undef TSF_TO_TU
}

static void
ath_beacon_config(struct ath_softc *sc)
{
        struct ath_hal *ah = sc->sc_ah;
        u_int32_t nexttbtt, intval;

        ath5k_hw_set_intr(ah, 0);
        intval = sc->sc_beacon_interval;

        nexttbtt = intval; /* for ap mode */

        intval |= AR5K_BEACON_RESET_TSF;

        /* In AP mode we enable the beacon timers and
	 * SWBA interrupts to prepare beacon frames. */

        intval |= AR5K_BEACON_ENA;
        sc->sc_imask |= AR5K_INT_SWBA;  /* beacon prepare */
        ath_beaconq_config(sc);
        ath_hal_beaconinit(ah, nexttbtt, intval);
        sc->sc_bmisscount = 0;
        ath5k_hw_set_intr(ah, sc->sc_imask);
}

static void
ath_descdma_cleanup(struct ath_softc *sc, ath_bufhead *head)
{
	struct ath_buf *bf;
	struct ieee80211_node *ni;

	STAILQ_FOREACH(bf, head, bf_list){
		if (bf->bf_skb) {
			pci_unmap_single(sc->sc_bdev,
				bf->bf_skbaddr, sc->sc_rxbufsize,
				PCI_DMA_FROMDEVICE);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
		}
		ni = bf->bf_node;
		bf->bf_node = NULL;
		if (ni != NULL) {
			/*
			 * Reclaim node reference.
			 */
			ieee80211_free_node(ni);
		}
	}

	STAILQ_INIT(head);
}

static int
ath_desc_alloc(struct ath_softc *sc)
{
#define	DS2PHYS(_sc, _ds) \
	((_sc)->sc_desc_daddr + ((caddr_t)(_ds) - (caddr_t)(_sc)->sc_desc))
	int bsize;
	struct ath_desc *ds;
	struct ath_buf *bf;
	int i;

	/* allocate descriptors */
	sc->sc_desc_len = sizeof(struct ath_desc) *
				(ATH_TXBUF * ATH_TXDESC + ATH_RXBUF + ATH_BCBUF + 1);
	barrier();	/* get_order() rounds down constants before Linux 2.6.21 */
	sc->sc_desc = pci_alloc_consistent(sc->sc_bdev,
				sc->sc_desc_len, &sc->sc_desc_daddr);
	if (sc->sc_desc == NULL) {
		if_printf(&sc->sc_dev, "%s, could not allocate descriptors\n", __func__);
		return ENOMEM;
	}
	ds = sc->sc_desc;
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: DMA map: %p (%u) -> %llx\n",
	    __func__, ds, (unsigned int) sc->sc_desc_len, ito64(sc->sc_desc_daddr));

	/* allocate buffers */
	bsize = sizeof(struct ath_buf) * (ATH_TXBUF + ATH_RXBUF + ATH_BCBUF + 1);
	bf = kmalloc(bsize, GFP_KERNEL);
	if (bf == NULL)
		goto bad;
	memset(bf, 0, bsize);
	sc->sc_bufptr = bf;

	STAILQ_INIT(&sc->sc_rxbuf);
	for (i = 0; i < ATH_RXBUF; i++, bf++, ds++) {
		bf->bf_desc = ds;
		bf->bf_daddr = DS2PHYS(sc, ds);
		STAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
	}

	STAILQ_INIT(&sc->sc_txbuf);
	for (i = 0; i < ATH_TXBUF; i++, bf++, ds += ATH_TXDESC) {
		bf->bf_desc = ds;
		bf->bf_daddr = DS2PHYS(sc, ds);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
	}

	STAILQ_INIT(&sc->sc_bbuf);
	for (i = 0; i < ATH_BCBUF; i++, bf++, ds++) {
		bf->bf_desc = ds;
		bf->bf_daddr = DS2PHYS(sc, ds);
		STAILQ_INSERT_TAIL(&sc->sc_bbuf, bf, bf_list);
	}

	return 0;
bad:
	pci_free_consistent(sc->sc_bdev, sc->sc_desc_len,
		sc->sc_desc, sc->sc_desc_daddr);
	sc->sc_desc = NULL;
	return ENOMEM;
#undef DS2PHYS
}

static void
ath_desc_free(struct ath_softc *sc)
{
        ath_descdma_cleanup(sc, &sc->sc_bbuf);
        ath_descdma_cleanup(sc, &sc->sc_txbuf);
        ath_descdma_cleanup(sc, &sc->sc_rxbuf);

	/* Free memory associated with all descriptors */
	pci_free_consistent(sc->sc_bdev, sc->sc_desc_len,
		sc->sc_desc, sc->sc_desc_daddr);

	kfree(sc->sc_bufptr);
	sc->sc_bufptr = NULL;
}

static struct ieee80211_node *
ath_node_alloc(struct ieee80211_node_table *nt)
{
	struct ieee80211com *ic = nt->nt_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	const size_t space = sizeof(struct ath_node) + sc->sc_rc->arc_space;
	struct ath_node *an;

	an = kmalloc(space, GFP_ATOMIC);
	if (an == NULL) {
		/* XXX stat+msg */
		return NULL;
	}
	memset(an, 0, space);
	an->an_avgrssi = ATH_RSSI_DUMMY_MARKER;
	an->an_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
	an->an_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
	an->an_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
	ath_rate_node_init(sc, an);

	DPRINTF(sc, ATH_DEBUG_NODE, "%s: an %p\n", __func__, an);
	return &an->an_node;
}

static void
ath_node_free(struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;
        struct ath_softc *sc = ic->ic_dev->priv;
	DPRINTF(sc, ATH_DEBUG_NODE, "%s: ni %p\n", __func__, ni);
/* 
	for (i = 0; i < AR5K_NUM_TX_QUEUES; i++)		// TODO: seems we need this still
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_cleanq(&sc->sc_txq[i], ni);
*/
	ath_rate_node_cleanup(sc, ATH_NODE(ni));
	sc->sc_node_free(ni);
}

static u_int8_t
ath_node_getrssi(const struct ieee80211_node *ni)
{
#define	AR5K_EP_RND(x, mul) \
	((((x)%(mul)) >= ((mul)/2)) ? ((x) + ((mul) - 1)) / (mul) : (x)/(mul))
	u_int32_t avgrssi = ATH_NODE_CONST(ni)->an_avgrssi;
	int32_t rssi;

	/*
	 * When only one frame is received there will be no state in
	 * avgrssi so fallback on the value recorded by the 802.11 layer.
	 */
	if (avgrssi != ATH_RSSI_DUMMY_MARKER)
		rssi = AR5K_EP_RND(avgrssi, AR5K_RSSI_EP_MULTIPLIER);
	else
		rssi = ni->ni_rssi;
	/* NB: theoretically we shouldn't need this, but be paranoid */
	return rssi < 0 ? 0 : rssi > 127 ? 127 : rssi;
#undef AR5K_EP_RND
}

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;
	struct ath_desc *ds;
	int headroom_needed = 0;
	
	if (sc->sc_ic.ic_opmode == IEEE80211_M_MONITOR) {
		headroom_needed = sizeof(wlan_ng_prism2_header);
	} else if (sc->sc_rawdev.type == ARPHRD_IEEE80211_PRISM) {
		headroom_needed = sizeof(wlan_ng_prism2_header);
	} else if (sc->sc_rawdev.type == ARPHRD_IEEE80211_RADIOTAP) {
		headroom_needed = sizeof(struct ath_rx_radiotap_header);
	}

	/* 
	 * Check if we have enough headroom. If not, just free the skb
	 * and we'll alloc another one below.
	 */
	if (bf->bf_skb && skb_headroom(bf->bf_skb) < headroom_needed) {
		pci_unmap_single(sc->sc_bdev,
				 bf->bf_skbaddr, sc->sc_rxbufsize,
				 PCI_DMA_FROMDEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
	}

	skb = bf->bf_skb;

	if (skb == NULL) {
		u_int off;

		/*
		 * Allocate buffer with headroom_needed space for the
		 * fake physical layer header at the start.
		 */
		skb = dev_alloc_skb(sc->sc_rxbufsize +
				    headroom_needed + 
				    sc->sc_cachelsz - 1);
		if (skb == NULL) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: skbuff alloc of size %d failed\n",
				__func__,
				(int)(sc->sc_rxbufsize
				      + headroom_needed
				      + sc->sc_cachelsz - 1));
			sc->sc_stats.ast_rx_nobuf++;
			return ENOMEM;
		}
		/*
		 * Reserve space for the fake physical layer header.
		 */
		skb_reserve(skb, headroom_needed);
		/*
		 * Cache-line-align.  This is important (for the
		 * 5210 at least) as not doing so causes bogus data
		 * in rx'd frames.
		 */
		off = ((unsigned long) skb->data) % sc->sc_cachelsz;
		if (off != 0)
			skb_reserve(skb, sc->sc_cachelsz - off);

		skb->dev = &sc->sc_dev;
		bf->bf_skb = skb;
		bf->bf_skbaddr = pci_map_single(sc->sc_bdev,
			skb->data, sc->sc_rxbufsize, PCI_DMA_FROMDEVICE);
		if (pci_dma_mapping_error(bf->bf_skbaddr)) {
			if_printf(&sc->sc_dev, "%s: DMA mapping failed\n", __func__);
			dev_kfree_skb(skb);
			bf->bf_skb = NULL;
			sc->sc_stats.ast_rx_busdma++;
			return ENOMEM;
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
	ds = bf->bf_desc;
	ds->ds_link = bf->bf_daddr;	/* link to self */
	ds->ds_data = bf->bf_skbaddr;
	ath_hal_setuprxdesc(ah, ds
		, skb_tailroom(skb)	/* buffer size */
		, 0
	);

	if (sc->sc_rxlink != NULL)
		*sc->sc_rxlink = bf->bf_daddr;
	sc->sc_rxlink = &ds->ds_link;
	return 0;
}

/*
 * Add additional headers to a transmitted frame and netif_rx it on
 * a monitor or raw device
 */
static void
ath_tx_capture(struct net_device *dev, struct ath_desc *ds, struct sk_buff *skb)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	u_int32_t tsf;

	/* 
	 * release the owner of this skb since we're basically
	 * recycling it 
	 */
	if (atomic_read(&skb->users) != 1) {
		struct sk_buff *skb2 = skb;
		skb = skb_clone(skb, GFP_ATOMIC);
		if (skb == NULL) {
			dev_kfree_skb(skb2);
			return;
		}
		kfree_skb(skb2);
	} else {
		skb_orphan(skb);
	}

	switch (dev->type) {
	case ARPHRD_IEEE80211:
		break;
	case ARPHRD_IEEE80211_PRISM: {
		wlan_ng_prism2_header *ph;
		if (skb_headroom(skb) < sizeof(wlan_ng_prism2_header) &&
		    pskb_expand_head(skb, 
				     sizeof(wlan_ng_prism2_header), 
				     0, GFP_ATOMIC)) {
			DPRINTF(sc, ATH_DEBUG_RECV, 
				"%s: couldn't pskb_expand_head\n", __func__);
			goto bad;
		}

		ph = (wlan_ng_prism2_header *)
			skb_push(skb, sizeof(wlan_ng_prism2_header));
		memset(ph, 0, sizeof(wlan_ng_prism2_header));
		
		ph->msgcode = DIDmsg_lnxind_wlansniffrm;
		ph->msglen = sizeof(wlan_ng_prism2_header);
		strcpy(ph->devname, sc->sc_dev.name);
		
		ph->hosttime.did = DIDmsg_lnxind_wlansniffrm_hosttime;
		ph->hosttime.status = 0;
		ph->hosttime.len = 4;
		ph->hosttime.data = jiffies;
		
		/* Pass up tsf clock in mactime */
		ph->mactime.did = DIDmsg_lnxind_wlansniffrm_mactime;
		ph->mactime.status = 0;
		ph->mactime.len = 4;
		/*
		 * Rx descriptor has the low 15 bits of the tsf at
		 * the time the frame was received.  Use the current
		 * tsf to extend this to 32 bits.
		 */
		tsf = ath_hal_gettsf32(sc->sc_ah);
		if ((tsf & 0x7fff) < ds->ds_rxstat.rs_tstamp)
			tsf -= 0x8000;
		ph->mactime.data = ds->ds_rxstat.rs_tstamp | (tsf &~ 0x7fff);
		
		ph->istx.did = DIDmsg_lnxind_wlansniffrm_istx;
		ph->istx.status = 0;
		ph->istx.len = 4;
		ph->istx.data = P80211ENUM_truth_true;
		
		ph->frmlen.did = DIDmsg_lnxind_wlansniffrm_frmlen;
		ph->frmlen.status = 0;
		ph->frmlen.len = 4;
		ph->frmlen.data = ds->ds_rxstat.rs_datalen;
		
		ph->channel.did = DIDmsg_lnxind_wlansniffrm_channel;
		ph->channel.status = 0;
		ph->channel.len = 4;
		ph->channel.data = ieee80211_mhz2ieee(ic->ic_ibss_chan->ic_freq,0);
		
		ph->rssi.did = DIDmsg_lnxind_wlansniffrm_rssi;
		ph->rssi.status = 0;
		ph->rssi.len = 4;
		ph->rssi.data = ds->ds_rxstat.rs_rssi;
		
		ph->noise.did = DIDmsg_lnxind_wlansniffrm_noise;
		ph->noise.status = 0;
		ph->noise.len = 4;
		ph->noise.data = -95;
		
		ph->signal.did = DIDmsg_lnxind_wlansniffrm_signal;
		ph->signal.status = 0;
		ph->signal.len = 4;
		ph->signal.data = -95 + ds->ds_rxstat.rs_rssi;
		
		ph->rate.did = DIDmsg_lnxind_wlansniffrm_rate;
		ph->rate.status = 0;
		ph->rate.len = 4;
		ph->rate.data = sc->sc_hwmap[ds->ds_txstat.ts_rate &~ AR5K_TXSTAT_ALTRATE].ieeerate;
		break;
	}
	case ARPHRD_IEEE80211_RADIOTAP: {
		struct ath_tx_radiotap_header *th;

		if (skb_headroom(skb) < sizeof(struct ath_tx_radiotap_header) &&
		    pskb_expand_head(skb, 
				     sizeof(struct ath_tx_radiotap_header), 
				     0, GFP_ATOMIC)) {
			DPRINTF(sc, ATH_DEBUG_RECV, 
				"%s: couldn't pskb_expand_head\n", __func__);
			goto bad;
		}
		
		th = (struct ath_tx_radiotap_header *) skb_push(skb, sizeof(struct ath_tx_radiotap_header));
		memset(th, 0, sizeof(struct ath_tx_radiotap_header));
		th->wt_ihdr.it_version = 0;
		th->wt_ihdr.it_len = cpu_to_le16(sizeof(struct ath_tx_radiotap_header));
		th->wt_ihdr.it_present = cpu_to_le32(ATH_TX_RADIOTAP_PRESENT);
		th->wt_flags = 0;
		th->wt_rate = sc->sc_hwmap[ds->ds_txstat.ts_rate &~ AR5K_TXSTAT_ALTRATE].ieeerate;
		th->wt_txpower = 0;
		th->wt_antenna = ds->ds_txstat.ts_antenna;
		th->wt_tx_flags = 0;
		if (ds->ds_txstat.ts_status) 
			th->wt_tx_flags |= cpu_to_le16(IEEE80211_RADIOTAP_F_TX_FAIL);
		th->wt_rts_retries = ds->ds_txstat.ts_shortretry;
		th->wt_data_retries = ds->ds_txstat.ts_longretry;
		
		break;
	}
	default:
		break;
	}

	skb->dev = dev;
	skb_reset_mac_header(skb);
	skb->ip_summed = CHECKSUM_NONE;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = __constant_htons(0x0019);  /* ETH_P_80211_RAW */
	netif_rx(skb);
	return;

 bad:
	dev_kfree_skb(skb);
	return;
}

/*
 * Add additional headers to a received frame and netif_rx it on
 * a monitor or raw device
 */
static void
ath_rx_capture(struct net_device *dev, struct ath_desc *ds, struct sk_buff *skb)
{
#define	IS_QOS_DATA(wh) \
	((wh->i_fc[0] & (IEEE80211_FC0_TYPE_MASK|IEEE80211_FC0_SUBTYPE_MASK))==\
		(IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS))
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int len = ds->ds_rxstat.rs_datalen;
	struct ieee80211_frame *wh;
	u_int32_t tsf;

	KASSERT(ic->ic_flags & IEEE80211_F_DATAPAD,
		("data padding not enabled?"));
	/* Remove pad bytes */
	wh = (struct ieee80211_frame *) skb->data;
	if (IS_QOS_DATA(wh)) {
		int headersize = ieee80211_hdrsize(wh);
		int padbytes = roundup(headersize,4) - headersize;

		/*
		 * Copy up 802.11 header and strip h/w padding.
		 */
		if (padbytes > 0) {
			memmove(skb->data + padbytes, skb->data, headersize);
			skb_pull(skb, padbytes);
			len -= padbytes;
		}
	}

	switch (dev->type) {
	case ARPHRD_IEEE80211:
		break;
	case ARPHRD_IEEE80211_PRISM: {
		wlan_ng_prism2_header *ph;
		if (skb_headroom(skb) < sizeof(wlan_ng_prism2_header)) {
			DPRINTF(sc, ATH_DEBUG_RECV,
				"%s: prism not enough headroom %d/%d\n",
				__func__, skb_headroom(skb), 
				(int)sizeof(wlan_ng_prism2_header));
			goto bad;
		}
		ph = (wlan_ng_prism2_header *)
			skb_push(skb, sizeof(wlan_ng_prism2_header));
		memset(ph, 0, sizeof(wlan_ng_prism2_header));
		
		ph->msgcode = DIDmsg_lnxind_wlansniffrm;
		ph->msglen = sizeof(wlan_ng_prism2_header);
		strcpy(ph->devname, sc->sc_dev.name);
		
		ph->hosttime.did = DIDmsg_lnxind_wlansniffrm_hosttime;
		ph->hosttime.status = 0;
		ph->hosttime.len = 4;
		ph->hosttime.data = jiffies;
		
		/* Pass up tsf clock in mactime */
		ph->mactime.did = DIDmsg_lnxind_wlansniffrm_mactime;
		ph->mactime.status = 0;
		ph->mactime.len = 4;
		/*
		 * Rx descriptor has the low 15 bits of the tsf at
		 * the time the frame was received.  Use the current
		 * tsf to extend this to 32 bits.
		 */
		tsf = ath_hal_gettsf32(sc->sc_ah);
		if ((tsf & 0x7fff) < ds->ds_rxstat.rs_tstamp)
			tsf -= 0x8000;
		ph->mactime.data = ds->ds_rxstat.rs_tstamp | (tsf &~ 0x7fff);
		
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
		ph->rssi.status = 0;
		ph->rssi.len = 4;
		ph->rssi.data = ds->ds_rxstat.rs_rssi;
		
		ph->noise.did = DIDmsg_lnxind_wlansniffrm_noise;
		ph->noise.status = 0;
		ph->noise.len = 4;
		ph->noise.data = -95;
		
		ph->signal.did = DIDmsg_lnxind_wlansniffrm_signal;
		ph->signal.status = 0;
		ph->signal.len = 4;
		ph->signal.data = -95 + ds->ds_rxstat.rs_rssi;
		
		ph->rate.did = DIDmsg_lnxind_wlansniffrm_rate;
		ph->rate.status = 0;
		ph->rate.len = 4;
		ph->rate.data = sc->sc_hwmap[ds->ds_rxstat.rs_rate].ieeerate;
		break;
	}
	case ARPHRD_IEEE80211_RADIOTAP: {
		struct ath_rx_radiotap_header *th;
		if (skb_headroom(skb) < sizeof(struct ath_rx_radiotap_header)) {
			DPRINTF(sc, ATH_DEBUG_RECV,
				"%s: radiotap not enough headroom %d/%d\n",
				__func__, skb_headroom(skb), 
				(int)sizeof(struct ath_rx_radiotap_header));
			goto bad;
		}
		th = (struct ath_rx_radiotap_header  *) skb_push(skb, sizeof(struct ath_rx_radiotap_header));
		memset(th, 0, sizeof(struct ath_rx_radiotap_header));

		th->wr_ihdr.it_version = 0;
		th->wr_ihdr.it_len = cpu_to_le16(sizeof(struct ath_rx_radiotap_header));
		th->wr_ihdr.it_present = cpu_to_le32(ATH_RX_RADIOTAP_PRESENT);
		th->wr_flags = IEEE80211_RADIOTAP_F_FCS;
		th->wr_rate = sc->sc_hwmap[ds->ds_rxstat.rs_rate].ieeerate;
		th->wr_chan_freq = cpu_to_le16(ic->ic_ibss_chan->ic_freq);
		th->wr_chan_flags = cpu_to_le16(ic->ic_ibss_chan->ic_flags);
		th->wr_antenna = ds->ds_rxstat.rs_antenna;
		th->wr_antsignal = ds->ds_rxstat.rs_rssi;
		if (ds->ds_rxstat.rs_status & AR5K_RXERR_CRC)
			th->wr_flags |= IEEE80211_RADIOTAP_F_BADFCS;

		break;
	}
	default:
		break;
	}
	
	skb->dev = dev;
	skb_reset_mac_header(skb);
	skb->ip_summed = CHECKSUM_NONE;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = __constant_htons(0x0019);  /* ETH_P_80211_RAW */

	netif_rx(skb);
	return;

 bad:
	dev_kfree_skb(skb);
	return;
#undef IS_QOS_DATA
}

/*
 * Extend 15-bit time stamp from rx descriptor to
 * a full 64-bit TSF using the current h/w TSF.
 */
static inline uint64_t
ath_tsf_extend(struct ath_hal *ah, uint32_t rstamp)
{
	uint64_t tsf;

	tsf = ath_hal_gettsf64(ah);

	/* Compensate for rollover. */
	if ((tsf & 0x7fff) < rstamp)
		tsf -= 0x8000;
	
	return ((tsf & ~(uint64_t)0x7fff) | rstamp);
}

/*
 * Intercept management frames to collect beacon rssi data
 * and to do ibss merges.
 */
static void
ath_recv_mgmt(struct ieee80211com *ic, struct sk_buff *skb,
	struct ieee80211_node *ni,
	int subtype, int rssi, u_int32_t rstamp)
{
	struct ath_softc *sc = ic->ic_dev->priv;

	/*
	 * Call up first so subsequent work can use information
	 * potentially stored in the node (e.g. for ibss merge).
	 */
	sc->sc_recv_mgmt(ic, skb, ni, subtype, rssi, rstamp);
	
	switch (subtype) {
	case IEEE80211_FC0_SUBTYPE_BEACON:
		/* update rssi statistics for use by the hal */
		ATH_RSSI_LPF((ATH_NODE(ni))->an_halstats.ns_avgbrssi, rssi);
		/* fall thru... */
	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
		if (ic->ic_opmode == IEEE80211_M_IBSS &&
		    ic->ic_state == IEEE80211_S_RUN) {
			/* Extend rstamp with the current tsf to 64 bit */
			u_int64_t tsf = ath_tsf_extend(sc->sc_ah, rstamp);
			/*
			 * Handle ibss merge as needed; check the tsf on the
			 * frame before attempting the merge.  The 802.11 spec
			 * says the station should change it's bssid to match
			 * the oldest station with the same ssid, where oldest
			 * is determined by the tsf.  Note that hardware
			 * reconfiguration happens through callback to
			 * ath_newstate as the state machine will go from
			 * RUN -> RUN when this happens.
			 */
			if (le64toh(ni->ni_tstamp.tsf) >= tsf) {
				DPRINTF(sc, ATH_DEBUG_STATE,
				    "ibss merge, rstamp %u tsf %llx "
				    "tstamp %llx\n", rstamp, tsf,
				    ni->ni_tstamp.tsf);
				ieee80211_ibss_merge(ni);
			}
		}
		if (ic->ic_opmode == IEEE80211_M_STA &&
			ic->ic_state == IEEE80211_S_RUN &&
			sc->sc_bmisscount > 0) {
				struct ieee80211_frame *wh;
				wh = (struct ieee80211_frame *) skb->data;
				if (IEEE80211_ADDR_EQ(wh->i_addr2, ic->ic_bss->ni_bssid)) {
					DPRINTF(sc, ATH_DEBUG_BEACON,
						"[%s] received %s after beacon miss - clear\n",
						ether_sprintf(wh->i_addr2),
						(subtype == IEEE80211_FC0_SUBTYPE_BEACON) ?
						"beacon" : "probe response");
						sc->sc_bmisscount = 0;
					ic->ic_mgt_timer = 0;
				}
		}
		break;
	}
}

/*
 * Set the default antenna.
 */
static void
ath_setdefantenna(struct ath_softc *sc, u_int antenna)
{
	struct ath_hal *ah = sc->sc_ah;

	/* XXX block beacon interrupts */
	ath_hal_setdefantenna(ah, antenna);
	if (sc->sc_defant != antenna)
		sc->sc_stats.ast_ant_defswitch++;
	sc->sc_defant = antenna;
	sc->sc_rxotherant = 0;
}

/*
 * Callback from the 802.11 layer to update WME parameters.
 */
static int
ath_wme_update(struct ieee80211com *ic)
{
	struct ath_softc *sc = ic->ic_dev->priv;

	return !ath_txq_update(sc, WME_AC_BE) ||
	    !ath_txq_update(sc, WME_AC_BK) ||
	    !ath_txq_update(sc, WME_AC_VI) ||
	    !ath_txq_update(sc, WME_AC_VO) ? EIO : 0;
}

/*
 * Reclaim resources for a setup queue.
 */
static void
ath_tx_cleanupq(struct ath_softc *sc, struct ath_txq *txq)
{

	ath_hal_releasetxqueue(sc->sc_ah, txq->axq_qnum);
	sc->sc_txqsetup &= ~(1<<txq->axq_qnum);
}

/*
 * Reclaim all tx queue resources.
 */
static void
ath_tx_cleanup(struct ath_softc *sc)
{
	int i;

	for (i = 0; i < AR5K_NUM_TX_QUEUES; i++) {
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_cleanupq(sc, &sc->sc_txq[i]);
	}
}

static int
ath_tx_start(struct net_device *dev, struct ieee80211_node *ni, struct ath_buf *bf,
    struct sk_buff *skb)
{
#define	CTS_DURATION \
	ath_hal_computetxtime(ah, rt, IEEE80211_ACK_LEN, cix, TRUE)
/*#define	updateCTSForBursting(_ah, _ds, _txq) \
	ath_hal_updateCTSForBursting(_ah, _ds, \
	    _txq->axq_linkbuf != NULL ? _txq->axq_linkbuf->bf_desc : NULL, \
	    _txq->axq_lastdsWithCTS, _txq->axq_gatingds, \
	    txopLimit, CTS_DURATION)*/
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	const struct chanAccParams *cap = &ic->ic_wme.wme_chanParams;
	int iswep, ismcast, keyix, hdrlen, pktlen, try0;
	u_int8_t rix, txrate, ctsrate;
	u_int8_t cix = 0xff;		/* NB: silence compiler */
	struct ath_desc *ds;
	struct ath_txq *txq;
	struct ieee80211_frame *wh;
	u_int subtype, flags, ctsduration;
	AR5K_PKT_TYPE atype;
	const struct ar5k_rate_table *rt;
	AR5K_BOOL short_preamble;
	struct ath_node *an;
	struct llc *llc;
	int eapol;
	u_int pri;

	wh = (struct ieee80211_frame *) skb->data;
	iswep = wh->i_fc[1] & IEEE80211_FC1_WEP;
	ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
	hdrlen = ieee80211_anyhdrsize(wh);
	// TODO: not so correct (WDS)
	llc = (struct llc *) (skb->data + sizeof(struct ieee80211_frame));
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: ether_type: 0x%x\n",
		__func__, ntohs(llc->llc_snap.ether_type));
	if (llc->llc_snap.ether_type == __constant_htons(ETHERTYPE_PAE))
		eapol = 1;
	else
		eapol = 0;
	llc = NULL;

	/*
	 * Packet length must not include any
	 * pad bytes; deduct them here.
	 */
	//TODO: ??? pktlen = m0->m_pkthdr.len - (hdrlen & 3);
	pktlen = skb->len - (hdrlen & 3);
	
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
			dev_kfree_skb(skb);
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
	} else if (ni->ni_ucastkey.wk_cipher == &ieee80211_cipher_none) {
                /*
                 * Use station key cache slot, if assigned.
                 */
                keyix = ni->ni_ucastkey.wk_keyix;
                if (keyix == IEEE80211_KEYIX_NONE)
                        keyix = AR5K_TXKEYIX_INVALID;
	} else
		keyix = AR5K_TXKEYIX_INVALID;

	pktlen += IEEE80211_CRC_LEN;

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
	bf->bf_skbaddr = pci_map_single(sc->sc_bdev,
		skb->data, pktlen, PCI_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %lx\n",
		__func__, skb, skb->data, skb->len, (long unsigned int) bf->bf_skbaddr);
	if (pci_dma_mapping_error(bf->bf_skbaddr)) {
		if_printf(dev, "%s: DMA mapping failed\n", __func__);
		dev_kfree_skb(skb);
		bf->bf_skb = NULL;
		sc->sc_stats.ast_tx_busdma++;
		return -EIO;
	}
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
		short_preamble = TRUE;
		sc->sc_stats.ast_tx_shortpre++;
	} else {
		short_preamble = FALSE;
	}

	an = ATH_NODE(ni);
	flags = AR5K_TXDESC_CLRDMASK;		/* XXX needed for crypto errs */
	/*
	 * Calculate Atheros packet type from IEEE80211 packet header,
	 * setup for rate calculations, and select h/w transmit queue.
	 */
	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_MGT:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_BEACON)
			atype = AR5K_PKT_TYPE_BEACON;
		else if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			atype = AR5K_PKT_TYPE_PROBE_RESP;
		else if (subtype == IEEE80211_FC0_SUBTYPE_ATIM)
			atype = AR5K_PKT_TYPE_ATIM;
		else
			atype = AR5K_PKT_TYPE_NORMAL;	/* XXX */
		rix = 0;			/* XXX lowest rate */
		try0 = ATH_TXMAXTRY;
		if (short_preamble)
			txrate = an->an_tx_mgtratesp;
		else
			txrate = an->an_tx_mgtrate;
		/* NB: force all management frames to highest queue */
		if (ni->ni_flags & IEEE80211_NODE_QOS) {
			pri = WME_AC_VO;
		} else
			pri = WME_AC_BE;
		flags |= AR5K_TXDESC_INTREQ;	/* force interrupt */
		break;
	case IEEE80211_FC0_TYPE_CTL:
		atype = AR5K_PKT_TYPE_PSPOLL;	/* stop setting of duration */
		rix = 0;			/* XXX lowest rate */
		try0 = ATH_TXMAXTRY;
		if (short_preamble)
			txrate = an->an_tx_mgtratesp;
		else
			txrate = an->an_tx_mgtrate;
		/* NB: force all ctl frames to highest queue */
		if (ni->ni_flags & IEEE80211_NODE_QOS) {
			pri = WME_AC_VO;
		} else
			pri = WME_AC_BE;
		flags |= AR5K_TXDESC_INTREQ;	/* force interrupt */
		break;
	case IEEE80211_FC0_TYPE_DATA:
		atype = AR5K_PKT_TYPE_NORMAL;		/* default */
		if (ismcast) {
			rix = 0;			/* XXX lowest rate */
			try0 = 0;
			if (short_preamble)
				txrate = an->an_tx_mgtratesp;
			else
				txrate = an->an_tx_mgtrate;
		}
		else if (eapol) {
			rix = 0;			/* XXX lowest rate */
			try0 = 0;			// TODO: userspace or hardware retry?
			if (short_preamble)
				txrate = an->an_tx_mgtratesp;
			else
				txrate = an->an_tx_mgtrate;
			flags |= AR5K_TXDESC_INTREQ;	/* force interrupt */
		} else {
			if (ic->ic_fixed_rate == -1) {
				/*
				 * Data frames; consult the rate control module.
				 */
				ath_rate_findrate(sc, an, short_preamble, pktlen,
					  &rix, &try0, &txrate);
			}
			else {
				rix = ic->ic_fixed_rate;
				try0 = ATH_TXMAXTRY; //XXX: should be configurabe
				if (short_preamble)
					txrate = rt->rates[rix].rate_code | SHPREAMBLE_FLAG(rix);
				else
					txrate = rt->rates[rix].rate_code;
			}
		}
		sc->sc_txrate = txrate;			/* for LED blinking */
		/*
		 * Default all non-QoS traffic to the background queue.
		 */
		if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
			pri = M_WME_GETAC(skb);
			if (cap->cap_wmeParams[pri].wmep_noackPolicy) {
				flags |= AR5K_TXDESC_NOACK;
				sc->sc_stats.ast_tx_noack++;
			}
		} else
			pri = WME_AC_BE;
		break;
	default:
		if_printf(dev, "bogus frame type 0x%x (%s)\n",
			wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK, __func__);
		/* XXX statistic */
		DPRINTF(sc, ATH_DEBUG_FATAL, "%s: kfree_skb: skb %p [data %p len %u] skbaddr %lx\n",
			__func__, skb, skb->data, skb->len, (long unsigned int) bf->bf_skbaddr);
		dev_kfree_skb(skb);
		bf->bf_skb = NULL;
		return -EIO;
	}
	txq = sc->sc_ac2q[pri];

	DPRINTF(sc, ATH_DEBUG_RATE, "%s: [%s] rix %d, try %d, txrate %d (rs_nrates %d)\n",
		__func__, ether_sprintf(ni->ni_macaddr), 
		rix, try0, txrate, ni->ni_rates.rs_nrates);
	
	/*
	 * When servicing one or more stations in power-save mode
	 * multicast frames must be buffered until after the beacon.
	 * We use the CAB queue for that.
	 */
	if (ismcast && ic->ic_ps_sta) {
		txq = sc->sc_cabq;
		/* XXX? more bit in 802.11 frame header */
	}

	/*
	 * Calculate miscellaneous flags.
	 */
	if (ismcast) {
		flags |= AR5K_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
	} else if (pktlen > ic->ic_rtsthreshold) {
		flags |= AR5K_TXDESC_RTSENA;	/* RTS based on frame length */
		cix = rt->rates[rix].control_rate;
		sc->sc_stats.ast_tx_rts++;
	}

	/*
	 * If 802.11g protection is enabled, determine whether
	 * to use RTS/CTS or just CTS.  Note that this is only
	 * done for OFDM unicast frames.
	 */
	if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
	    rt->rates[rix].modulation == IEEE80211_RATE_OFDM &&
	    (flags & AR5K_TXDESC_NOACK) == 0) {
		/* XXX fragments must use CCK rates w/ protection */
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
			flags |= AR5K_TXDESC_RTSENA;
		else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
			flags |= AR5K_TXDESC_CTSENA;
		cix = rt->rates[sc->sc_protrix].control_rate;
		sc->sc_stats.ast_tx_protect++;
	}

	/*
	 * Calculate duration.  This logically belongs in the 802.11
	 * layer but it lacks sufficient information to calculate it.
	 */
	if ((flags & AR5K_TXDESC_NOACK) == 0 &&
	    (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL) {
		u_int16_t dur;
		/*
		 * XXX not right with fragmentation.
		 */
		if (short_preamble)
			dur = rt->rates[rix].sp_ack_duration;
		else
			dur = rt->rates[rix].lp_ack_duration;
		wh->i_dur = htole16(dur);
	}

	/*
	 * Calculate RTS/CTS rate and duration if needed.
	 */
	ctsduration = 0;
	if (flags & (AR5K_TXDESC_RTSENA|AR5K_TXDESC_CTSENA)) {
		/*
		 * CTS transmit rate is derived from the transmit rate
		 * by looking in the h/w rate table.  We must also factor
		 * in whether or not a short preamble is to be used.
		 */
		/* NB: cix is set above where RTS/CTS is enabled */
		KASSERT(cix != 0xff, ("cix not setup"));
		ctsrate = rt->rates[cix].rate_code;
		/*
		 * Compute the transmit duration based on the frame
		 * size and the size of an ACK frame.  We call into the
		 * HAL to do the computation since it depends on the
		 * characteristics of the actual PHY being used.
		 *
		 * NB: CTS is assumed the same size as an ACK so we can
		 *     use the precalculated ACK durations.
		 */
		if (short_preamble) {
			ctsrate |= SHPREAMBLE_FLAG(cix);
			if (flags & AR5K_TXDESC_RTSENA)		/* SIFS + CTS */
				ctsduration += rt->rates[cix].sp_ack_duration;
			ctsduration += ath_hal_computetxtime(ah,
				rt, pktlen, rix, TRUE);
			if ((flags & AR5K_TXDESC_NOACK) == 0)	/* SIFS + ACK */
				ctsduration += rt->rates[cix].sp_ack_duration;
		} else {
			if (flags & AR5K_TXDESC_RTSENA)		/* SIFS + CTS */
				ctsduration += rt->rates[cix].lp_ack_duration;
			ctsduration += ath_hal_computetxtime(ah,
				rt, pktlen, rix, FALSE);
			if ((flags & AR5K_TXDESC_NOACK) == 0)	/* SIFS + ACK */
				ctsduration += rt->rates[cix].lp_ack_duration;
		}
		/*
		 * Must disable multi-rate retry when using RTS/CTS.
		 */
		try0 = ATH_TXMAXTRY;
	} else
		ctsrate = 0;

	if (IFF_DUMPPKTS(sc, ATH_DEBUG_XMIT))
		ieee80211_dump_pkt(skb->data, skb->len,
			sc->sc_hwmap[txrate].ieeerate, -1);

	/* 
	 * Determine if a tx interrupt should be generated for
	 * this descriptor.  We take a tx interrupt to reap
	 * descriptors when the h/w hits an EOL condition or
	 * when the descriptor is specifically marked to generate
	 * an interrupt.  We periodically mark descriptors in this
	 * way to insure timely replenishing of the supply needed
	 * for sending frames.  Defering interrupts reduces system
	 * load and potentially allows more concurrent work to be
	 * done but if done to aggressively can cause senders to
	 * backup.
	 *
	 * NB: use >= to deal with sc_txintrperiod changing
	 *     dynamically later through nl80211.
	 */
	if (flags & AR5K_TXDESC_INTREQ) {
		txq->axq_intrcnt = 0;
	} else if (++txq->axq_intrcnt >= sc->sc_txintrperiod) {
		flags |= AR5K_TXDESC_INTREQ;
		txq->axq_intrcnt = 0;
	}

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	/* XXX check return value? */
	ath5k_hw_tx_desc_setup(ah, ds
		, pktlen		/* packet length */
		, hdrlen		/* header length */
		, atype			/* Atheros packet type */
		, ni->ni_txpower	/* txpower */
		, txrate, try0		/* series 0 rate/tries */
		, keyix			/* key cache index */
		, sc->sc_txantenna	/* antenna mode */
		, flags			/* flags */
		, ctsrate		/* rts/cts rate */
		, ctsduration		/* rts/cts duration */
	);
	bf->bf_flags = flags;
	/*
	 * Setup the multi-rate retry state only when we're
	 * going to use it.  This assumes ath5k_hw_tx_desc_setup 
	 * initializes the descriptors (so we don't have to)
	 * when the hardware supports multi-rate retry and
	 * we don't use it.
	 */
	if (try0 != ATH_TXMAXTRY)
		ath_rate_setupxtxdesc(sc, an, ds, short_preamble, rix);

	/*
	 * Fillin the remainder of the descriptor info.
	 */
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath5k_hw_tx_desc_fill(ah, ds
		, skb->len		/* segment length */
		, TRUE		/* first segment */
		, TRUE		/* last segment */
		, ds			/* first descriptor */
	);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
	    __func__, txq->axq_qnum, ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
	/*
	 * Insert the frame on the outbound list and
	 * pass it on to the hardware.
	 */
	spin_lock_bh(&(txq)->axq_lock);
	if (flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)) {
//		u_int32_t txopLimit = IEEE80211_TXOP_TO_US(
//			cap->cap_wmeParams[pri].wmep_txopLimit);
		/*
		 * When bursting, potentially extend the CTS duration
		 * of a previously queued frame to cover this frame
		 * and not exceed the txopLimit.  If that can be done
		 * then disable RTS/CTS on this frame since it's now
		 * covered (burst extension).  Otherwise we must terminate
		 * the burst before this frame goes out so as not to
		 * violate the WME parameters.  All this is complicated
		 * as we need to update the state of packets on the
		 * (live) hardware queue.  The logic is buried in the hal
		 * because it's highly chip-specific.
		 */
//		if (txopLimit != 0) {
//			sc->sc_stats.ast_tx_ctsburst++;
//			if (updateCTSForBursting(ah, ds, txq) == 0) {
				/*
				 * This frame was not covered by RTS/CTS from
				 * the previous frame in the burst; update the
				 * descriptor pointers so this frame is now
				 * treated as the last frame for extending a
				 * burst.
				 */
//				txq->axq_lastdsWithCTS = ds;
				/* set gating Desc to final desc */
//				txq->axq_gatingds =
//					(struct ath_desc *)txq->axq_link;
//			} else
//				sc->sc_stats.ast_tx_ctsext++;
//		}
	}
	ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
	DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: txq depth = %d\n",
		__func__, txq->axq_depth);
	if (txq->axq_link == NULL) {
		ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: TXDP[%u] = %llx (%p) depth %d\n", __func__,
			txq->axq_qnum, ito64(bf->bf_daddr), bf->bf_desc,
			txq->axq_depth);
	} else {
		*txq->axq_link = bf->bf_daddr;
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: link[%u](%p)=%llx (%p) depth %d\n", __func__,
			txq->axq_qnum, txq->axq_link,
			ito64(bf->bf_daddr), bf->bf_desc, txq->axq_depth);
	}
	txq->axq_link = &bf->bf_desc->ds_link;
	/*
	 * The CAB queue is started from the SWBA handler since
	 * frames only go out on DTIM and to avoid possible races.
	 */
	if (txq != sc->sc_cabq)
		ath_hal_txstart(ah, txq->axq_qnum);
	spin_unlock_bh(&(txq)->axq_lock);

	dev->trans_start = jiffies;
	sc->sc_rawdev.trans_start = jiffies;
	return 0;
//#undef updateCTSForBursting
#undef CTS_DURATION
}

/*
 * Process completed xmit descriptors from the specified queue.
 */
static void
ath_tx_processq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct ieee80211_node *ni;
	struct ath_node *an;
	int sr, lr, pri;
	AR5K_STATUS status;

	DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: tx queue %u head %p link %p\n",
		__func__, txq->axq_qnum,
		(caddr_t)(uintptr_t) ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum),
		txq->axq_link);
	for (;;) {
		spin_lock(&(txq)->axq_lock)
		txq->axq_intrcnt = 0;	/* reset periodic desc intr count */
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			spin_unlock(&(txq)->axq_lock);
			break;
		}
		ds = bf->bf_desc;		/* NB: last decriptor */
		status = ath5k_hw_tx_desc_proc(ah, ds);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_XMIT_DESC)
			ath_printtxbuf(bf, status == AR5K_OK);
#endif
		if (status == AR5K_EINPROGRESS) {
			spin_unlock(&(txq)->axq_lock);
			break;
		}
		if (ds == txq->axq_lastdsWithCTS)
			txq->axq_lastdsWithCTS = NULL;
		if (ds == txq->axq_gatingds)
			txq->axq_gatingds = NULL;
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		spin_unlock(&(txq)->axq_lock);

		ni = bf->bf_node;
		if (ni != NULL) {
			an = ATH_NODE(ni);
			if (ds->ds_txstat.ts_status == 0) {
				u_int8_t txant = ds->ds_txstat.ts_antenna;
				sc->sc_stats.ast_ant_tx[txant]++;
				sc->sc_ant_tx[txant]++;
				if (ds->ds_txstat.ts_rate & AR5K_TXSTAT_ALTRATE)
					sc->sc_stats.ast_tx_altrate++;
				sc->sc_stats.ast_tx_rssi =
					ds->ds_txstat.ts_rssi;
				ATH_RSSI_LPF(an->an_halstats.ns_avgtxrssi,
					ds->ds_txstat.ts_rssi);
				pri = M_WME_GETAC(bf->bf_skb);
				if (pri >= WME_AC_VO)
					ic->ic_wme.wme_hipri_traffic++;
				ni->ni_inact = ni->ni_inact_reload;
			} else {
				if (ds->ds_txstat.ts_status & AR5K_TXERR_XRETRY)
					sc->sc_stats.ast_tx_xretries++;
				if (ds->ds_txstat.ts_status & AR5K_TXERR_FIFO)
					sc->sc_stats.ast_tx_fifoerr++;
				if (ds->ds_txstat.ts_status & AR5K_TXERR_FILT)
					sc->sc_stats.ast_tx_filtered++;
			}
			sr = ds->ds_txstat.ts_shortretry;
			lr = ds->ds_txstat.ts_longretry;
			sc->sc_stats.ast_tx_shortretry += sr;
			sc->sc_stats.ast_tx_longretry += lr;
			/*
			 * Hand the descriptor to the rate control algorithm.
			 */
			if ((ds->ds_txstat.ts_status & AR5K_TXERR_FILT) == 0 &&
			    (bf->bf_flags & AR5K_TXDESC_NOACK) == 0)
				ath_rate_tx_complete(sc, an, ds);
			/*
			 * Reclaim reference to node.
			 *
			 * NB: the node may be reclaimed here if, for example
			 *     this is a DEAUTH message that was sent and the
			 *     node was timed out due to inactivity.
			 */
			ieee80211_free_node(ni);
		}
		pci_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);

		if (sc->sc_rawdev_enabled) {
			ath_tx_capture(&sc->sc_rawdev, ds, bf->bf_skb);
		} else {
			dev_kfree_skb(bf->bf_skb);
		}
		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		spin_lock(&(sc)->sc_txbuflock);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock(&(_sc)->sc_txbuflock);
	}
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
#ifdef AR_DEBUG
	struct ath_hal *ah = sc->sc_ah;
#endif
	struct ieee80211_node *ni;
	struct ath_buf *bf;

	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath_tx_tasklet
	 */
	for (;;) {
		spin_lock_bh(&(txq)->axq_lock);
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			spin_unlock_bh(&(txq)->axq_lock);
			break;
		}
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		spin_unlock_bh(&(txq)->axq_lock);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RESET)
			ath_printtxbuf(bf,
				ath5k_hw_tx_desc_proc(ah, bf->bf_desc) == AR5K_OK);
#endif /* AR_DEBUG */
		pci_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, PCI_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		ni = bf->bf_node;
		bf->bf_node = NULL;
		if (ni != NULL) {
			/*
			 * Reclaim node reference.
			 */
			ieee80211_free_node(ni);
		}
		spin_lock_bh(&(sc)->sc_txbuflock);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		spin_unlock_bh(&(sc)->sc_txbuflock);
	}
}

static void
ath_tx_stopdma(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;

	(void) ath5k_hw_stop_rx_dma(ah, txq->axq_qnum);
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: tx queue [%u] %p, link %p\n",
	    __func__, txq->axq_qnum,
	    (caddr_t)(uintptr_t) ath_hal_gettxbuf(ah, txq->axq_qnum),
	    txq->axq_link);
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
		/* don't touch the hardware if marked invalid */
		(void) ath5k_hw_stop_rx_dma(ah, sc->sc_bhalq);
		DPRINTF(sc, ATH_DEBUG_RESET,
		    "%s: beacon queue %p\n", __func__,
		    (caddr_t)(uintptr_t) ath_hal_gettxbuf(ah, sc->sc_bhalq));
		for (i = 0; i < AR5K_NUM_TX_QUEUES; i++) {
			if (ATH_TXQ_SETUP(sc, i))
				ath_tx_stopdma(sc, &sc->sc_txq[i]);
		}
	}
	sc->sc_dev.trans_start = jiffies;
	sc->sc_rawdev.trans_start = jiffies;
	
	netif_start_queue(&sc->sc_dev);		// TODO: needed here?
	if (sc->sc_rawdev_enabled)
		netif_start_queue(&sc->sc_rawdev);
	
	for (i = 0; i < AR5K_NUM_TX_QUEUES; i++) {
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_draintxq(sc, &sc->sc_txq[i]);
	}
	sc->sc_tx_timer = 0;
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
	if (sc->sc_debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL)) {	// TODO: compiler warns integer overflow
		struct ath_buf *bf;

		printk("%s: rx queue %p, link %p\n", __func__,
			(caddr_t)(uintptr_t) ath_hal_getrxbuf(ah), sc->sc_rxlink);
		STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			struct ath_desc *ds = bf->bf_desc;
			AR5K_STATUS status = ath5k_hw_rx_desc_proc(ah, ds,
				bf->bf_daddr, PA2DESC(sc, ds->ds_link));
			if (status == AR5K_OK || (sc->sc_debug & ATH_DEBUG_FATAL))
				ath_printrxbuf(bf, status == AR5K_OK);
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
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: mtu %u cachelsz %u rxbufsize %u\n",
		__func__, dev->mtu, sc->sc_cachelsz, sc->sc_rxbufsize);

	sc->sc_rxlink = NULL;
	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		if (error != 0) {
			DPRINTF(sc, ATH_DEBUG_RECV,
				"%s: ath_rxbuf_init failed %d\n",
				__func__, error);
			return error;
		}
	}

	bf = STAILQ_FIRST(&sc->sc_rxbuf);
	ath_hal_putrxbuf(ah, bf->bf_daddr);
	ath_hal_rxena(ah);		/* enable recv descriptors */
	ath_mode_init(dev);		/* set filters, etc. */
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
	u_int16_t flags;

	/*
	 * Change channels and update the h/w rate map
	 * if we're switching; e.g. 11a to 11b/g.
	 */
	mode = ieee80211_chan2mode(ic, chan);
	if (mode != sc->sc_curmode)
		ath_setcurmode(sc, mode);
	/*
	 * Update BPF state.  NB: ethereal et. al. don't handle
	 * merged flags well so pick a unique mode for their use.
	 */
	if (IEEE80211_IS_CHAN_A(chan))
		flags = IEEE80211_CHAN_A;
	/* XXX 11g schizophrenia */
	else if (IEEE80211_IS_CHAN_G(chan) ||
	    IEEE80211_IS_CHAN_PUREG(chan))
		flags = IEEE80211_CHAN_G;
	else
		flags = IEEE80211_CHAN_B;
	if (IEEE80211_IS_CHAN_T(chan))
		flags |= CHANNEL_TURBO;
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
	AR5K_CHANNEL hchan;
	int opmode;
	
	/*
	 * Convert to a HAL channel description with
	 * the flags constrained to reflect the current
	 * operating mode.
	 */
	if (chan == IEEE80211_CHAN_ANYC) {
		return 0;
	}
	hchan.freq = chan->ic_freq;
	hchan.channel_flags = ath_chan2flags(ic, chan);

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %u (%u MHz) -> %u (%u MHz)\n",
	    __func__,
	    ath_hal_mhz2ieee(sc->sc_curchan.freq,
		sc->sc_curchan.channel_flags),
	    	sc->sc_curchan.freq,
	    ath_hal_mhz2ieee(hchan.freq, hchan.channel_flags), hchan.freq);
	if (hchan.freq != sc->sc_curchan.freq ||
	    hchan.channel_flags != sc->sc_curchan.channel_flags) {
		AR5K_STATUS status;

		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath5k_hw_set_intr(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* clear pending tx frames */
		ath_stoprecv(sc);		/* turn off frame recv */
		opmode = (ic->ic_opmode == IEEE80211_M_AHDEMO) ? 0 : ic->ic_opmode;
		if (!ath_hal_reset(ah, opmode, &hchan, TRUE, &status)) {
			if_printf(ic->ic_dev, "ath_chan_set: unable to reset "
				"channel %u (%u Mhz)\n",
				ieee80211_chan2ieee(ic, chan), chan->ic_freq);
			return EIO;
		}
		sc->sc_curchan = hchan;
		ath_update_txpow(sc);		/* update tx power state */
		sc->sc_diversity = ath_hal_getdiversity(ah);

		/*
		 * Re-enable rx framework.
		 */
		if (ath_startrecv(sc) != 0) {
			if_printf(ic->ic_dev,
				"ath_chan_set: unable to restart recv logic\n");
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
		ath5k_hw_set_intr(ah, sc->sc_imask);
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

	DPRINTF(sc, ATH_DEBUG_CALIBRATE, "%s: channel %u/%x\n",
		__func__, sc->sc_curchan.freq, sc->sc_curchan.channel_flags);

	if (ath_hal_getrfgain(ah) == AR5K_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		sc->sc_stats.ast_per_rfgain++;
		DPRINTF(sc, ATH_DEBUG_RESET,
			"%s: calibration, resetting\n", dev->name);
		ath_reset(dev);
	}
	if (!ath_hal_calibrate(ah, &sc->sc_curchan)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: calibration of channel %u failed\n",
			__func__, sc->sc_curchan.freq);
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
	const u_int8_t *bssid;
	u_int32_t rfilt;
	static const AR5K_LED_STATE leds[] = {
	    AR5K_LED_INIT,	/* IEEE80211_S_INIT */
	    AR5K_LED_SCAN,	/* IEEE80211_S_SCAN */
	    AR5K_LED_AUTH,	/* IEEE80211_S_AUTH */
	    AR5K_LED_ASSOC, 	/* IEEE80211_S_ASSOC */
	    AR5K_LED_RUN, 	/* IEEE80211_S_RUN */
	};

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s -> %s\n", __func__,
		ieee80211_state_name[ic->ic_state],
		ieee80211_state_name[nstate]);

	del_timer(&sc->sc_scan_ch);		/* ap/neighbor scan timer */
	del_timer(&sc->sc_cal_ch);		/* periodic calibration timer */
	ath_hal_setledstate(ah, leds[nstate]);	/* set LED */
	netif_stop_queue(dev);			/* before we do anything else */
	if (sc->sc_rawdev_enabled)
		netif_stop_queue(&sc->sc_rawdev);

	if (nstate == IEEE80211_S_INIT) {
		sc->sc_imask &= ~(AR5K_INT_SWBA | AR5K_INT_BMISS);
		/*
		 * NB: disable interrupts so we don't rx frames.
		 */
    		ath5k_hw_set_intr(ah, sc->sc_imask &~ AR5K_INT_GLOBAL);
		/*
		 * Notify the rate control algorithm.
		 */
		ath_rate_newstate(sc, nstate);
		goto done;
	}
	ni = ic->ic_bss;
	error = ath_chan_set(sc, ni->ni_chan);
	if (error != 0)
		goto bad;
	rfilt = ath_calcrxfilter(sc, nstate);
	if (nstate == IEEE80211_S_SCAN)
		bssid = dev->broadcast;
	else
		bssid = ni->ni_bssid;
	ath_hal_setrxfilter(ah, rfilt);
	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s\n",
		 __func__, rfilt, ether_sprintf(bssid));

	if (nstate == IEEE80211_S_RUN && ic->ic_opmode == IEEE80211_M_STA)
		ath_hal_setassocid(ah, bssid, ni->ni_associd);
	else
		ath_hal_setassocid(ah, bssid, 0);
	if (ic->ic_flags & IEEE80211_F_PRIVACY) {
		for (i = 0; i < AR5K_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, bssid);
	}

	/*
	 * Notify the rate control algorithm so rates
	 * are setup should ath_beacon_alloc be called.
	 */
	ath_rate_newstate(sc, nstate);

	if (ic->ic_opmode == IEEE80211_M_MONITOR) {
		/* nothing to do */;
	} else if (nstate == IEEE80211_S_RUN) {
		DPRINTF(sc, ATH_DEBUG_STATE,
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
                switch (ic->ic_opmode) {
                case IEEE80211_M_HOSTAP:
                case IEEE80211_M_IBSS:
			/*
			 * Allocate and setup the beacon frame.
			 *
			 * Stop any previous beacon DMA.  This may be
			 * necessary, for example, when an ibss merge
			 * causes reconfiguration; there will be a state
			 * transition from RUN->RUN that means we may
			 * be called with beacon transmission active.
			 */
			ath5k_hw_stop_rx_dma(ah, sc->sc_bhalq);
			ath_beacon_free(sc);
			error = ath_beacon_alloc(sc, ni);
			if (error != 0)
				goto bad;
                       break;
                case IEEE80211_M_STA:
                        /*
                         * Allocate a key cache slot to the station.
                         */
                        if ((ic->ic_flags & IEEE80211_F_PRIVACY) == 0 &&
                            sc->sc_hasclrkey &&
                            ni->ni_ucastkey.wk_keyix == IEEE80211_KEYIX_NONE)
                                ath_setup_stationkey(ni);
                        break;
                default:
                        break;
		}

		/*
		 * Configure the beacon and sleep timers.
		 */
		ath_beacon_config(sc);
	} else {
		ath5k_hw_set_intr(ah,
			sc->sc_imask &~ (AR5K_INT_SWBA | AR5K_INT_BMISS));
		sc->sc_imask &= ~(AR5K_INT_SWBA | AR5K_INT_BMISS);
	}
done:
	/*
	 * Invoke the parent method to complete the work.
	 */
	error = sc->sc_newstate(ic, nstate, arg);
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
	if (sc->sc_rawdev_enabled) 
		netif_start_queue(&sc->sc_rawdev);

	return error;
}

/*
 * Allocate a key cache slot to the station so we can
 * setup a mapping from key index to node. The key cache
 * slot is needed for managing antenna state and for
 * compression when stations do not use crypto.  We do
 * it uniliaterally here; if crypto is employed this slot
 * will be reassigned.
 */
static void
ath_setup_stationkey(struct ieee80211_node *ni)
{
        struct ieee80211com *ic = ni->ni_ic;
        struct ath_softc *sc = ic->ic_dev->priv;
        u_int16_t keyix;

        keyix = ath_key_alloc(ic, &ni->ni_ucastkey);
        if (keyix == IEEE80211_KEYIX_NONE) {
                /*
                 * Key cache is full; we'll fall back to doing
                 * the more expensive lookup in software.  Note
                 * this also means no h/w compression.
                 */
                /* XXX msg+statistic */
        } else {
                ni->ni_ucastkey.wk_keyix = keyix;
                /* NB: this will create a pass-thru key entry */
                ath_keyset(sc, &ni->ni_ucastkey, ni->ni_macaddr, ic->ic_bss);
        }
}

/*
 * Setup driver-specific state for a newly associated node.
 * Note that we're called also on a re-associate, the isnew
 * param tells us if this is the first time or not.
 */
static void
ath_newassoc(struct ieee80211_node *ni, int isnew)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ath_softc *sc = ic->ic_dev->priv;

	ath_rate_newassoc(sc, ATH_NODE(ni), isnew);
        if (isnew &&
            (ic->ic_flags & IEEE80211_F_PRIVACY) == 0 && sc->sc_hasclrkey) {
                KASSERT(ni->ni_ucastkey.wk_keyix == IEEE80211_KEYIX_NONE,
                    ("new assoc with a unicast key already setup (keyix %u)",
                    ni->ni_ucastkey.wk_keyix));
                ath_setup_stationkey(ni);
        }
}

/*
 * Initialize channels array. Comes from hal, verify name usage, this
 * used to be ath_hal_init_channels()
 */
int
ath5k_init_channels(struct ath_hal *hal, AR5K_CHANNEL *channels,
    u_int max_channels, u_int *channels_size, AR5K_CTRY_CODE country, u_int16_t mode,
    int outdoor, int extended)
{
	u_int i, c;
	u_int32_t domain_current;
	u_int domain_5ghz, domain_2ghz;
	AR5K_CHANNEL *all_channels;
	AR5K_CTRY_CODE country_current;

	/* Allocate and initialize channel array */
	if ((all_channels = kmalloc(sizeof(AR5K_CHANNEL) * max_channels,
	    GFP_KERNEL)) == NULL)
		return (FALSE);
	memset(all_channels, 0, sizeof(AR5K_CHANNEL) * max_channels);

	i = c = 0;
	domain_current = hal->ah_regdomain;
	hal->ah_country_code = country;
	country_current = hal->ah_country_code;

	/*
	 * In debugging mode, enable all channels supported by the chipset
	 */
	if (domain_current == DMN_DEFAULT || CHAN_DEBUG == 1) {
		int min, max, freq;
		u_int flags;

		min = 1; /* 2GHz channel 1 -2412Mhz */
		max = 26;/* 2GHz channel 26 (non-ieee) -2732Mhz */

		flags = CHANNEL_B | CHANNEL_G;

/* XXX: This is nasty move label to inline */
 debugchan:
		for (i = min; (i <= max) && (c < max_channels); i++) {
			freq = ath_hal_ieee2mhz(i, flags);
			if (ath5k_check_channel(hal, freq, flags) == FALSE)
				continue;
			all_channels[c].freq = freq;
			all_channels[c].channel_flags = flags;
			c++;
		}

		/* If is there to protect from infinite loop */
		if (flags & CHANNEL_2GHZ) {
/* ath_hal_mhz2ieee returns 1 for IEEE80211_CHANNELS_5GHZ_MIN 
for loop starts from 1 and all channels are marked as 5GHz M.F.*/
//			min = ath_hal_mhz2ieee(IEEE80211_CHANNELS_5GHZ_MIN,
//			    CHANNEL_5GHZ);
/* Continue from where we stoped, skip last 2GHz channel */
			min = max + 1;
			max = ath_hal_mhz2ieee(IEEE80211_CHANNELS_5GHZ_MAX,
						CHANNEL_5GHZ);
			flags = CHANNEL_A | CHANNEL_T | CHANNEL_XR;
			goto debugchan;
		}

		goto done;
	}

	domain_5ghz = ieee80211_regdomain2flag(domain_current,
	    IEEE80211_CHANNELS_5GHZ_MIN);
	domain_2ghz = ieee80211_regdomain2flag(domain_current,
	    IEEE80211_CHANNELS_2GHZ_MIN);

	/*
	 * Create channel list based on chipset capabilities, regulation domain
	 * and mode. 5GHz...
	 */

	/* This iterates over all possible 5GHz channels. This is pulled from
	 * ieee80211_regdomain.h, we can migrate this to use our own regualtory
	 * agent */
	for (i = 0; (hal->ah_capabilities.cap_range.range_5ghz_max > 0) &&
		 (i < ARRAY_SIZE(ath5k_5ghz_channels)) &&
		 (c < max_channels); i++) {
		/* Check if channel is supported by the chipset */
		if (ath5k_check_channel(hal,
		    ath5k_5ghz_channels[i].rc_channel,
		    CHANNEL_5GHZ) == FALSE)
			continue;

		/* Match regulation domain */
		if ((IEEE80211_DMN(ath5k_5ghz_channels[i].rc_domain) &
			IEEE80211_DMN(domain_5ghz)) == 0)
			continue;

		/* Match modes */
		if (ath5k_5ghz_channels[i].rc_mode & CHANNEL_TURBO)
			all_channels[c].channel_flags = CHANNEL_T;
		else if (ath5k_5ghz_channels[i].rc_mode &
		    CHANNEL_OFDM)
			all_channels[c].channel_flags = CHANNEL_A;
		else
			continue;

		/* Write channel and increment counter */
		all_channels[c++].freq = ath5k_5ghz_channels[i].rc_channel;
	}

	/*
	 * ...and 2GHz.
	 */
	for (i = 0; (hal->ah_capabilities.cap_range.range_2ghz_max > 0) &&
			(i < ARRAY_SIZE(ath5k_2ghz_channels)) &&
			(c < max_channels); i++) {
		
		/* Check if channel is supported by the chipset */
		if (ath5k_check_channel(hal,
		    ath5k_2ghz_channels[i].rc_channel,
		    CHANNEL_2GHZ) == FALSE)
			continue;

		/* Match regulation domain */
		if ((IEEE80211_DMN(ath5k_2ghz_channels[i].rc_domain) &
			IEEE80211_DMN(domain_2ghz)) == 0)
			continue;

		/* Match modes */
		if ((hal->ah_capabilities.cap_mode & AR5K_MODE_11B) &&
		   (ath5k_2ghz_channels[i].rc_mode & CHANNEL_CCK))
			all_channels[c].channel_flags = CHANNEL_B;

		if ((hal->ah_capabilities.cap_mode & AR5K_MODE_11G) &&
		   (ath5k_2ghz_channels[i].rc_mode & CHANNEL_OFDM)) {
			all_channels[c].channel_flags |= CHANNEL_G;
/*			if (ath5k_2ghz_channels[i].rc_mode &
			    CHANNEL_TURBO)
				all_channels[c].channel_flags |= CHANNEL_TG;*/
		}

		/* Write channel and increment counter */
		all_channels[c++].freq = ath5k_2ghz_channels[i].rc_channel;
	}

 done:
	memcpy(channels, all_channels, sizeof(AR5K_CHANNEL) * max_channels);
	*channels_size = c;
	kfree(all_channels);
	return (TRUE);
}


static int
ath_getchannels(struct net_device *dev, u_int cc,
	AR5K_BOOL outdoor, AR5K_BOOL xchanmode)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_CHANNEL *chans;
	u_int nchan;

	chans = kmalloc(IEEE80211_CHAN_MAX * sizeof(AR5K_CHANNEL), GFP_KERNEL);
	if (chans == NULL) {
		if_printf(dev, "unable to allocate channel table\n");
		return ENOMEM;
	}
	if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, &nchan,
	    cc, AR5K_MODE_ALL, outdoor, xchanmode)) {
		u_int32_t rd;

		ath_hal_getregdomain(ah, &rd);
		if_printf(dev, "unable to collect channel list from hal; "
			"regdomain likely %u country code %u\n", rd, cc);
		kfree(chans);
		return EINVAL;
	}

	/* Convert hardware channels to mac80211 ones */
	ath5k_add_channels(sc, MODE_IEEE80211A, chans, nchan, CHANNEL_A);
	ath5k_add_channels(sc, MODE_IEEE80211B, chans, nchan, CHANNEL_B);
	ath5k_add_channels(sc, MODE_IEEE80211G, chans, nchan, CHANNEL_G);
	kfree(chans);
	return 0;
}

/*
 * Turn the LED off: flip the pin and then set a timer so no
 * update will happen for the specified duration.
 */
static void
ath_led_off(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *) arg;
        
        if(sc->sc_endblink == 1){
            /* part of ath_led_done() */
            sc->sc_blinking = 0;
        } else{
            ath_hal_gpioset(sc->sc_ah, sc->sc_ledpin, !sc->sc_ledon);
            sc->sc_endblink = 1;
            mod_timer(&sc->sc_ledtimer, jiffies + sc->sc_ledoff);
        }
}

/*
 * Blink the LED according to the specified on/off times.
 */
static void
ath_led_blink(struct ath_softc *sc, int on, int off)
{
	DPRINTF(sc, ATH_DEBUG_LED, "%s: on %u off %u\n", __func__, on, off);
	ath_hal_gpioset(sc->sc_ah, sc->sc_ledpin, sc->sc_ledon);
	sc->sc_blinking = 1;
	sc->sc_endblink = 0;
	sc->sc_ledoff = off;
	mod_timer(&sc->sc_ledtimer, jiffies + on);
}

static void
ath_led_event(struct ath_softc *sc, int event)
{

	sc->sc_ledevent = jiffies;	/* time of last event */
	if (sc->sc_blinking)		/* don't interrupt active blink */
		return;
	switch (event) {
	case ATH_LED_POLL:
		ath_led_blink(sc, sc->sc_hwmap[0].ledon,
			sc->sc_hwmap[0].ledoff);
		break;
	case ATH_LED_TX:
		ath_led_blink(sc, sc->sc_hwmap[sc->sc_txrate].ledon,
			sc->sc_hwmap[sc->sc_txrate].ledoff);
		break;
	case ATH_LED_RX:
		ath_led_blink(sc, sc->sc_hwmap[sc->sc_rxrate].ledon,
			sc->sc_hwmap[sc->sc_rxrate].ledoff);
		break;
	}
}

static void
ath_update_txpow(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t txpow;

	if (sc->sc_curtxpow != ic->ic_txpowlimit) {
		ath_hal_settxpowlimit(ah, ic->ic_txpowlimit);
		/* read back in case value is clamped */
		(void)ath_hal_gettxpowlimit(ah, &txpow);
		ic->ic_txpowlimit = sc->sc_curtxpow = txpow;
	}
	/* 
	 * Fetch max tx power level for status requests.
	 */
	(void)ath_hal_getmaxtxpow(sc->sc_ah, &txpow);
	ic->ic_bss->ni_txpower = txpow;
}

static int
ath_rate_setup(struct net_device *dev, u_int mode)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	const struct ar5k_rate_table *rt;
	struct ieee80211_rateset *rs;
	int i, maxrates;

	switch (mode) {
	case IEEE80211_MODE_11A:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, AR5K_MODE_11A);
		break;
	case IEEE80211_MODE_11B:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, AR5K_MODE_11B);
		break;
	case IEEE80211_MODE_11G:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, AR5K_MODE_11G);
		break;
	case IEEE80211_MODE_TURBO_A:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, AR5K_MODE_TURBO);
		break;
	case IEEE80211_MODE_TURBO_G:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, AR5K_MODE_108G);
		break;
	default:
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid mode %u\n",
			__func__, mode);
		return 0;
	}
	rt = sc->sc_rates[mode];
	if (rt == NULL)
		return 0;
	if (rt->rate_count > IEEE80211_RATE_MAXSIZE) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: rate table too small (%u > %u)\n",
			__func__, rt->rate_count, IEEE80211_RATE_MAXSIZE);
		maxrates = IEEE80211_RATE_MAXSIZE;
	} else
		maxrates = rt->rate_count;
	rs = &ic->ic_sup_rates[mode];
	for (i = 0; i < maxrates; i++)
		rs->rs_rates[i] = rt->rates[i].dot11_rate;
	rs->rs_nrates = maxrates;
	return 1;
}

static void
ath_setcurmode(struct ath_softc *sc, u_int mode)
{
	/* NB: on/off times from the Atheros NDIS driver, w/ permission */
	static const struct {
		u_int		rate;		/* tx/rx 802.11 rate */
		u_int16_t	timeOn;		/* LED on time (ms) */
		u_int16_t	timeOff;	/* LED off time (ms) */
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
		{   0, 500, 130 },
	};
	const struct ar5k_rate_table *rt;
	int i, j;

	memset(sc->sc_rixmap, 0xff, sizeof(sc->sc_rixmap));
	rt = sc->sc_rates[mode];
	KASSERT(rt != NULL, ("no h/w rate set for phy mode %u", mode));

	memset(sc->sc_hwmap, 0, sizeof(sc->sc_hwmap));
	for (i = 0; i < 32; i++) {
		u_int8_t ix = rt->rate_code_to_index[i];
		if (ix == 0xff) {
			sc->sc_hwmap[i].ledon = msecs_to_jiffies(500);
			sc->sc_hwmap[i].ledoff = msecs_to_jiffies(130);
			continue;
		}
		/* setup blink rate table to avoid per-packet lookup */
		for (j = 0; j < ARRAY_SIZE(blinkrates)-1; j++)
			if (blinkrates[j].rate == sc->sc_hwmap[i].ieeerate)
				break;
		/* NB: this uses the last entry if the rate isn't found */
		/* XXX beware of overlow */
		sc->sc_hwmap[i].ledon = msecs_to_jiffies(blinkrates[j].timeOn);
		sc->sc_hwmap[i].ledoff = msecs_to_jiffies(blinkrates[j].timeOff);
	}
	sc->sc_currates = rt;
	sc->sc_curmode = mode;
	/*
	 * All protection frames are transmited at 2Mb/s for
	 * 11g, otherwise at 1Mb/s.
	 * XXX select protection rate index from rate table.
	 */
	sc->sc_protrix = ((mode == IEEE80211_MODE_11G || mode == IEEE80211_MODE_TURBO_G) ? 1 : 0);
	/* NB: caller is responsible for reseting rate control state */
}


static int
ath_rawdev_attach(struct ath_softc *sc) 
{
	struct net_device *rawdev;
	unsigned t;
	rawdev = &sc->sc_rawdev;
	strcpy(rawdev->name, sc->sc_dev.name);
	strcat(rawdev->name, "raw");
	rawdev->priv = sc;

	/* ether_setup clobbers type, so save it */
	t = rawdev->type;
	ether_setup(rawdev);
	rawdev->type = t;

	rawdev->stop = NULL;
	rawdev->hard_start_xmit = ath_start_raw;
	rawdev->set_multicast_list = NULL;
	rawdev->get_stats = ath_getstats;
	rawdev->tx_queue_len = ATH_TXBUF;
	rawdev->flags |= IFF_NOARP;
	rawdev->flags &= ~IFF_MULTICAST;
	rawdev->mtu = IEEE80211_MAX_LEN;

	if (register_netdev(rawdev)) {
		goto bad;
	}

	if ((sc->sc_dev.flags & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))
		netif_start_queue(&sc->sc_rawdev);

	sc->sc_rawdev_enabled = 1;

	return 0;
 bad:
	return -1;
}

static void
ath_rawdev_detach(struct ath_softc *sc) 
{
	if (sc->sc_rawdev_enabled) {
		sc->sc_rawdev_enabled = 0;
		netif_stop_queue(&sc->sc_rawdev);
		unregister_netdev(&sc->sc_rawdev);
	}
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

	printk("R (%p %llx) %08x %08x %08x %08x %08x %08x %c\n",
	    ds, ito64(bf->bf_daddr),
	    ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1,
	    ds->ds_hw[0], ds->ds_hw[1],
	    !done ? ' ' : (ds->ds_rxstat.rs_status == 0) ? '*' : '!');
}

static void
ath_printtxbuf(struct ath_buf *bf, int done)
{
	struct ath_desc *ds = bf->bf_desc;

	printk("T (%p %llx) %08x %08x %08x %08x %08x %08x %08x %08x %c\n",
	    ds, ito64(bf->bf_daddr),
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
ATH_FREQ_BOUNCE(siwfreqx)
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
	(iw_handler) ath_ioctl_siwfreqx,		/* SIOCSIWFREQ */
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
	(iw_handler) NULL,				/* SIOCWFIRSTPRIV+3 */
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
/*get_wireless_stats moved from net_device to iw_handler_def*/
# if IW_HANDLER_VERSION >= 7
	.get_wireless_stats	= ath_iw_getstats,
# endif
	.standard		= (iw_handler *) ath_handlers,
	.num_standard		= N(ath_handlers),
	.private		= (iw_handler *) ath_priv_handlers,
	.num_private		= N(ath_priv_handlers),
#undef N
};

static int
ath_set_mac_address(struct net_device *dev, void *addr)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct sockaddr *mac = addr;
	int error = 0;

	if (netif_running(dev)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: cannot set address; device running\n", __func__);
		return -EBUSY;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		__func__,
		mac->sa_data[0], mac->sa_data[1], mac->sa_data[2],
		mac->sa_data[3], mac->sa_data[4], mac->sa_data[5]);

	down(&(sc)->sc_lock);
	/* XXX not right for multiple vap's */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, mac->sa_data);
	IEEE80211_ADDR_COPY(dev->dev_addr, mac->sa_data);
	ath_hal_setmac(ah, dev->dev_addr);
	if ((dev->flags & IFF_RUNNING) && !sc->sc_invalid) {
		error = -ath_reset(dev);
	}
	up(&(sc)->sc_lock);

	return error;
}

static int      
ath_change_mtu(struct net_device *dev, int mtu) 
{
	struct ath_softc *sc = dev->priv;
	int error = 0;

	if (!(ATH_MIN_MTU < mtu && mtu <= ATH_MAX_MTU)) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid %d, min %u, max %u\n",
			__func__, mtu, ATH_MIN_MTU, ATH_MAX_MTU);
		return -EINVAL;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: %d\n", __func__, mtu);

	down(&(sc)->sc_lock);
	dev->mtu = mtu;
	if ((dev->flags & IFF_RUNNING) && !sc->sc_invalid) {
		/* NB: the rx buffers may need to be reallocated */
		tasklet_disable(&sc->sc_rxtq);
		error = -ath_reset(dev);
		tasklet_enable(&sc->sc_rxtq);
	}
	up(&(sc)->sc_lock);

	return error;
}

static	int mindwelltime = 100;			/* 100ms */
static	int mincalibrate = 1;			/* once a second */
static	int maxint = 0x7fffffff;		/* 32-bit big */

/*
 * Announce various information on device/driver attach.
 */
static void
ath_announce(struct ath_softc *sc)
{
#define	AR5K_MODE_DUALBAND	(AR5K_MODE_11A|AR5K_MODE_11B)
	struct net_device *dev = &sc->sc_dev;
	struct ath_hal *ah = sc->sc_ah;
	u_int modes, cc;
	int i;

	if_printf(dev, "mac %d.%d phy %d.%d",
		ah->ah_mac_version, ah->ah_mac_revision,
		ah->ah_phy_revision >> 4, ah->ah_phy_revision & 0xf);
	/*
	 * Print radio revision(s).  We check the wireless modes
	 * to avoid falsely printing revs for inoperable parts.
	 * Dual-band radio revs are returned in the 5Ghz rev number.
	 */
	ath_hal_getcountrycode(ah, &cc);
	modes = ath_hal_getwirelessmodes(ah, cc);
	if ((modes & AR5K_MODE_DUALBAND) == AR5K_MODE_DUALBAND) {
		if (ah->ah_radio_5ghz_revision && ah->ah_radio_2ghz_revision)
			printk(" 5ghz radio %d.%d 2ghz radio %d.%d",
				ah->ah_radio_5ghz_revision >> 4,
				ah->ah_radio_5ghz_revision & 0xf,
				ah->ah_radio_2ghz_revision >> 4,
				ah->ah_radio_2ghz_revision & 0xf);
		else
			printk(" radio %d.%d", ah->ah_radio_5ghz_revision >> 4,
				ah->ah_radio_5ghz_revision & 0xf);
	} else
		printk(" radio %d.%d", ah->ah_radio_5ghz_revision >> 4,
			ah->ah_radio_5ghz_revision & 0xf);
	printk("\n");
	for (i = 0; i <= WME_AC_VO; i++) {
		struct ath_txq *txq = sc->sc_ac2q[i];
                    if_printf(dev, "Use hw queue %u for %s traffic\n",
                            txq->axq_qnum, ieee80211_wme_acnames[i]);
	}
        if_printf(dev, "Use hw queue %u for CAB traffic\n",
                sc->sc_cabq->axq_qnum);
        if_printf(dev, "Use hw queue %u for beacons\n", sc->sc_bhalq);
	
#ifdef AR_DEBUG
	printk("Debugging version (ATH)\n");
#endif
#undef AR5K_MODE_DUALBAND
}
