/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * Copyright (c) 2006 Devicescape Software, Inc.
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
#include "opt_ah.h"

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <asm/uaccess.h>


#ifdef USE_HEADERLEN_RESV
#include <net80211/if_llc.h>
#endif


#include "if_athvar.h"
#include "ah_desc.h"
#include "ah_devid.h"			/* XXX to identify chipset */

#ifdef ATH_PCI		/* PCI BUS */
#include "if_ath_pci.h"
#endif			/* PCI BUS */
#ifdef ATH_AHB		/* AHB BUS */
#include "if_ath_ahb.h"
#endif			/* AHB BUS */

#ifdef ATH_TX99_DIAG
#include "ath_tx99.h"
#endif
#include "if_ath_d80211.h"	
#include "if_ath.h"	

#include "ath_hw.h"

/* unaligned little endian access */
#define LE_READ_2(p)							\
	((u_int16_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8)))
#define LE_READ_4(p)							\
	((u_int32_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8) |	\
	  (((u_int8_t *)(p))[2] << 16) | (((u_int8_t *)(p))[3] << 24)))

enum {
	ATH_LED_TX,
	ATH_LED_RX,
	ATH_LED_POLL,
};

#if 0
static struct ieee80211vap *ath_vap_create(struct ieee80211com *,
	const char *, int, int, int, struct net_device *);
static void ath_vap_delete(struct ieee80211vap *);
#endif
static int ath_set_ack_bitrate(struct ath_softc *, int);
static void ath_fatal_tasklet(TQUEUE_ARG);
static void ath_rxorn_tasklet(TQUEUE_ARG);
#if 0
static void ath_bmiss_tasklet(TQUEUE_ARG);
#endif
static void ath_bstuck_tasklet(TQUEUE_ARG);
static void ath_beacon_tasklet(TQUEUE_ARG);
#if 0
static void ath_radar_task(struct ATH_WORK_THREAD *);
static void ath_dfs_test_return(unsigned long);

#endif
static int ath_stop_locked(struct ath_softc *);
#if 0
#if 0
static void ath_initkeytable(struct ath_softc *);
#endif
static void ath_key_update_begin(struct ieee80211vap *);
static void ath_key_update_end(struct ieee80211vap *);
#endif
static void ath_mode_init(struct ath_softc *);
#if 0
static void ath_setslottime(struct ath_softc *);
static void ath_updateslot(struct net_device *);
#endif
static int ath_beaconq_setup(struct ath_hal *);
#ifdef ATH_SUPERG_DYNTURBO
static void ath_beacon_dturbo_update(struct ieee80211vap *, int *, u_int8_t);
static void ath_beacon_dturbo_config(struct ieee80211vap *, u_int32_t);
static void ath_turbo_switch_mode(unsigned long);
static int ath_check_beacon_done(struct ath_softc *);
#endif
static void ath_beacon_send(struct ath_softc *, int *);
#if 0
static void ath_beacon_start_adhoc(struct ath_softc *, struct ieee80211vap *);
static void ath_beacon_return(struct ath_softc *, struct ath_buf *);
#endif
static void ath_beacon_free(struct ath_softc *);
static void ath_beacon_config(struct ath_softc *);
static int ath_desc_alloc(struct ath_softc *);
static void ath_desc_free(struct ath_softc *);
#if 0
static void ath_desc_swap(struct ath_desc *);
static struct ieee80211_node *ath_node_alloc(struct ieee80211_node_table *,
	struct ieee80211vap *);
static void ath_node_cleanup(struct ieee80211_node *);
static void ath_node_free(struct ieee80211_node *);
static u_int8_t ath_node_getrssi(const struct ieee80211_node *);
#endif
static int ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
#if 0
static void ath_recv_mgmt(struct ieee80211_node *, struct sk_buff *, int,
	int, u_int32_t);
#endif
static void ath_setdefantenna(struct ath_softc *, u_int);
static struct ath_txq *ath_txq_setup(struct ath_softc *, int, int);
static void ath_rx_tasklet(TQUEUE_ARG);
#if 0
static int ath_hardstart(struct sk_buff *, struct net_device *);
static int ath_mgtstart(struct ieee80211com *, struct sk_buff *);
#ifdef ATH_SUPERG_COMP
static u_int32_t ath_get_icvlen(struct ieee80211_key *);
static u_int32_t ath_get_ivlen(struct ieee80211_key *);
static void ath_setup_comp(struct ieee80211_node *, int);
static void ath_comp_set(struct ieee80211vap *, struct ieee80211_node *, int);	
#endif
#endif
static int ath_tx_setup(struct ath_softc *, int, int);
#if 0
static int ath_wme_update(struct ieee80211com *);
static void ath_uapsd_flush(struct ieee80211_node *);
#endif
static void ath_tx_cleanupq(struct ath_softc *, struct ath_txq *);
static void ath_tx_cleanup(struct ath_softc *);
#if 0
static void ath_tx_uapsdqueue(struct ath_softc *, struct ath_node *,
	struct ath_buf *);

static int ath_tx_start(struct net_device *, struct ieee80211_node *,
	struct ath_buf *, struct sk_buff *, int);
#endif
static void ath_tx_tasklet_q0(TQUEUE_ARG);
#if 0
static void ath_tx_tasklet_q0123(TQUEUE_ARG);
static void ath_tx_tasklet(TQUEUE_ARG);
static void ath_tx_timeout(struct net_device *);
#endif
static void ath_tx_draintxq(struct ath_softc *, struct ath_txq *);
static void ath_draintxq(struct ath_softc *);
static __inline void ath_tx_txqaddbuf(struct ath_softc *,
	struct ath_txq *, struct ath_buf *, struct ath_desc *, int);
static void ath_stoprecv(struct ath_softc *);
static int ath_startrecv(struct ath_softc *);
static void ath_flushrecv(struct ath_softc *);
static void ath_chan_change(struct ath_softc *, AR5K_CHANNEL *);
static void ath_calibrate(unsigned long);
#if 0
static int ath_newstate(struct ieee80211vap *, enum ieee80211_state, int);

static void ath_scan_start(struct ieee80211com *);
static void ath_scan_end(struct ieee80211com *);
static void ath_set_channel(struct ieee80211com *);
static void ath_set_coverageclass(struct ieee80211com *);
static u_int ath_mhz2ieee(struct ieee80211com *, u_int, u_int);
#ifdef ATH_SUPERG_FF
static int athff_can_aggregate(struct ath_softc *, struct ether_header *,
	struct ath_node *, struct sk_buff *, u_int16_t, int *);
#endif
static struct net_device_stats *ath_getstats(struct net_device *);
static void ath_setup_stationkey(struct ieee80211_node *);
static void ath_setup_stationwepkey(struct ieee80211_node *);
static void ath_setup_keycacheslot(struct ath_softc *, struct ieee80211_node *);
static void ath_newassoc(struct ieee80211_node *, int);
#endif
static int ath_getchannels(struct ath_softc *, u_int, AR5K_BOOL, AR5K_BOOL);
static void ath_led_event(struct ath_softc *, int);
static void ath_update_txpow(struct ath_softc *);
#if 0

static int ath_set_mac_address(struct net_device *, void *);
static int ath_change_mtu(struct net_device *, int);
static int ath_ioctl(struct net_device *, struct ifreq *, int);
#endif

static int ath_rate_setup(struct ath_softc *, u_int);
#if 0
static void ath_setup_subrates(struct net_device *);
#ifdef ATH_SUPERG_XR
static int ath_xr_rate_setup(struct net_device *);
static void ath_grppoll_txq_setup(struct ath_softc *, int, int);
static void ath_grppoll_start(struct ieee80211vap *, int);
static void ath_grppoll_stop(struct ieee80211vap *);
static u_int8_t ath_node_move_data(const struct ieee80211_node *);
static void ath_grppoll_txq_update(struct ath_softc *, int);
static void ath_grppoll_period_update(struct ath_softc *);
#endif
#endif
static void ath_setcurmode(struct ath_softc *, u_int);

static void ath_dynamic_sysctl_register(struct ath_softc *);
static void ath_dynamic_sysctl_unregister(struct ath_softc *);
static void ath_announce(struct ath_softc *sc);
#if 0
static void ath_check_dfs_clear(unsigned long);
#endif
static const char *ath_get_hal_status_desc(AR5K_STATUS status);
	
static int ath_calinterval = ATH_SHORT_CALINTERVAL;		/*
								 * calibrate every 30 secs in steady state
								 * but check every second at first.
								 */
static int ath_countrycode = CTRY_DEFAULT;	/* country code */
static int ath_outdoor = FALSE;		/* enable outdoor use */
static int ath_xchanmode = TRUE;		/* enable extended channels */
static char *autocreate = NULL;
static int rfkill = -1;
static int countrycode = -1;
static int outdoor = -1;
static int xchanmode = -1;

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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(countrycode, "i");
MODULE_PARM(outdoor, "i");
MODULE_PARM(xchanmode, "i");
MODULE_PARM(rfkill, "i");
MODULE_PARM(autocreate, "s");
#else
#include <linux/moduleparam.h>
module_param(countrycode, int, 0600);
module_param(outdoor, int, 0600);
module_param(xchanmode, int, 0600);
module_param(rfkill, int, 0600);
module_param(autocreate, charp, 0600);
#endif
MODULE_PARM_DESC(countrycode, "Override default country code");
MODULE_PARM_DESC(outdoor, "Enable/disable outdoor use");
MODULE_PARM_DESC(xchanmode, "Enable/disable extended channel mode");
MODULE_PARM_DESC(rfkill, "Enable/disable RFKILL capability");
MODULE_PARM_DESC(autocreate, "Create ath device in [sta|ap|wds|adhoc|ahdemo|monitor] mode. defaults to sta, use 'none' to disable");

int	ath_debug = 0;
#ifdef AR_DEBUG
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(ath_debug, "i");
#else
module_param(ath_debug, int, 0600);
#endif
MODULE_PARM_DESC(ath_debug, "Load-time debug output enable");

#define	IFF_DUMPPKTS(sc, _m) \
	((sc->sc_debug & _m))
static void ath_printrxbuf(struct ath_buf *, int);
static void ath_printtxbuf(struct ath_buf *, int);
#define	KEYPRINTF(sc, ix, hk, mac) do {				\
	if (sc->sc_debug & ATH_DEBUG_KEYCACHE)			\
		ath_keyprint(sc, __func__, ix, hk, mac);	\
} while (0)
#else /* defined(AR_DEBUG) */
#define	IFF_DUMPPKTS(sc, _m)	netif_msg_dumppkts(&sc->sc_ic)
#define	KEYPRINTF(sc, k, ix, mac)
#endif /* defined(AR_DEBUG) */

#define ATH_SETUP_XR_VAP(sc,vap,rfilt) \
	do { \
		if (sc->sc_curchan.private_flags & CHANNEL_4MS_LIMIT) \
			vap->iv_fragthreshold = XR_4MS_FRAG_THRESHOLD; \
		else \
			vap->iv_fragthreshold = vap->iv_xrvap->iv_fragthreshold; \
		if (!sc->sc_xrgrppoll) { \
			ath_grppoll_txq_setup(sc, AR5K_TX_QUEUE_DATA, GRP_POLL_PERIOD_NO_XR_STA(sc)); \
			ath_grppoll_start(vap, sc->sc_xrpollcount); \
			ath_hal_setrxfilter(sc->sc_ah, rfilt|AR5K_RX_FILTER_XRPOLL); \
		} \
   	} while(0)

/*
 * Define the scheme that we select MAC address for multiple BSS on the same radio.
 * The very first VAP will just use the MAC address from the EEPROM.
 * For the next 3 VAPs, we set the U/L bit (bit 1) in MAC address,
 * and use the next two bits as the index of the VAP.
 */
#define ATH_SET_VAP_BSSID_MASK(bssid_mask)      ((bssid_mask)[0] &= ~(((ATH_BCBUF-1)<<2)|0x02))
#define ATH_GET_VAP_ID(bssid)                   ((bssid)[0] >> 2)
#define ATH_SET_VAP_BSSID(bssid, id)            \
    	do {                                    \
		if (id)                            \
            		(bssid)[0] |= (((id) << 2) | 0x02); \
	} while(0)

int
ath_attach(u_int16_t devid, struct ath_softc *sc)
{
	struct ieee80211_hw *hw = sc->sc_hw;
	struct ath_hal *ah;
	AR5K_STATUS status;
	int error = 0, i;
	u_int8_t csz;
	sc->devid = devid;
	sc->sc_debug = ath_debug;
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: devid 0x%x\n", __func__, devid);

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	bus_read_cachesize(sc, &csz);
	/* XXX assert csz is non-zero */
	sc->sc_cachelsz = csz << 2;		/* convert to bytes */

	ATH_LOCK_INIT(sc);
	ATH_TXBUF_LOCK_INIT(sc);
	ATH_RXBUF_LOCK_INIT(sc);

	ATH_INIT_TQUEUE(&sc->sc_rxtq,	 ath_rx_tasklet,	sc);
#if 0
	ATH_INIT_TQUEUE(&sc->sc_txtq,	 ath_tx_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_bmisstq, ath_bmiss_tasklet,	dev);
#endif
	ATH_INIT_TQUEUE(&sc->sc_bstucktq,ath_bstuck_tasklet,	sc);
	ATH_INIT_TQUEUE(&sc->sc_beacontq, ath_beacon_tasklet,	sc);
	ATH_INIT_TQUEUE(&sc->sc_rxorntq, ath_rxorn_tasklet,	sc);
	ATH_INIT_TQUEUE(&sc->sc_fataltq, ath_fatal_tasklet,	sc);
#if 0
	INIT_WORK(&sc->sc_radartask, ath_radar_task);
#endif

	/*
	 * Attach the HAL and verify ABI compatibility by checking
	 * the HAL's ABI signature against the one the driver was
	 * compiled with.  A mismatch indicates the driver was
	 * built with an ah.h that does not correspond to the HAL
	 * module loaded in the kernel.
	 */
	ah = _ath_hal_attach(devid, sc, 0,
			     (__force AR5K_BUS_HANDLE) sc->sc_iobase,
			     &status);
	if (ah == NULL) {
		printk(KERN_ERR "%s: unable to attach hardware: '%s' (HAL status %u)\n",
			sc->name, ath_get_hal_status_desc(status), status);
		error = ENXIO;
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
	 * Check if the device has hardware counters for PHY
	 * errors.  If so we need to enable the MIB interrupt
	 * so we can act on stat triggers.
	 */
	if (ath_hal_hwphycounters(ah))
		sc->sc_needmib = 1;

	/*
	 * Get the hardware key cache size.
	 */
	sc->sc_keymax = ath_hal_keycachesize(ah);
	if (sc->sc_keymax > ATH_KEYMAX) {
		printk("%s: Warning, using only %u entries in %u key cache\n",
			sc->name, ATH_KEYMAX, sc->sc_keymax);
		sc->sc_keymax = ATH_KEYMAX;
	}
	/*
	 * Reset the key cache since some parts do not
	 * reset the contents on initial power up.
	 */
	for (i = 0; i < sc->sc_keymax; i++)
		ath_hal_keyreset(ah, i);

	/*
	 * Collect the channel list using the default country
	 * code and including outdoor channels.  The 802.11 layer
	 * is responsible for filtering this list based on settings
	 * like the phy mode.
	 */
	if (countrycode != -1)
		ath_countrycode = countrycode;
	if (outdoor != -1)
		ath_outdoor = outdoor;
	if (xchanmode != -1)
		ath_xchanmode = xchanmode;
	error = ath_getchannels(sc, ath_countrycode,
			ath_outdoor, ath_xchanmode);
	if (error != 0)
		goto bad;

#if 0
	ic->ic_country_code = ath_countrycode;
	ic->ic_country_outdoor = ath_outdoor;
#endif

	if (rfkill != -1) {
		printk(KERN_INFO "ath_pci: switching rfkill capability %s\n",
			rfkill ? "on" : "off");	
		ath_hal_setrfsilent(ah, rfkill);
	}

	/*
	 * Setup rate tables for all potential media types.
	 */
	ath_rate_setup(sc, AR5K_MODE_11A);
	ath_rate_setup(sc, AR5K_MODE_11B);
	ath_rate_setup(sc, AR5K_MODE_11G);
#if 0
	/* FIXME: hostapd does not support turbo modes. */
	ath_rate_setup(dev, AR5K_MODE_TURBO);
	ath_rate_setup(dev, AR5K_MODE_108G);
#endif
#if 0
	/* Setup for half/quarter rates */
	ath_setup_subrates(dev);
#endif

	/* NB: setup here so ath_rate_update is happy */
	ath_setcurmode(sc, AR5K_MODE_11A);

	/*
	 * Allocate tx+rx descriptors and populate the lists.
	 */
	error = ath_desc_alloc(sc);
	if (error != 0) {
		printk(KERN_ERR "%s: failed to allocate descriptors: %d\n",
			sc->name, error);
		goto bad;
	}

#if 0
	/*
	 * Init ic_caps prior to queue init, since WME cap setting
	 * depends on queue setup.
	 */
	ic->ic_caps = 0;
#endif

	/*
	 * Allocate hardware transmit queues: one queue for
	 * beacon frames and one data queue for each QoS
	 * priority.  Note that the HAL handles resetting
	 * these queues at the needed time.
	 *
	 * XXX PS-Poll
	 */
	sc->sc_bhalq = ath_beaconq_setup(ah);
	if (sc->sc_bhalq == (u_int) -1) {
		printk(KERN_ERR "%s: unable to setup a beacon xmit queue!\n",
			sc->name);
		error = EIO;
		goto bad2;
	}
	sc->sc_cabq = ath_txq_setup(sc, AR5K_TX_QUEUE_CAB, 0);
	if (sc->sc_cabq == NULL) {
		printk(KERN_ERR "%s: unable to setup CAB xmit queue!\n",
			sc->name);
		error = EIO;
		goto bad2;
	}
#if 0
	/* NB: ensure BK queue is the lowest priority h/w queue */
	if (!ath_tx_setup(sc, WME_AC_BK, AR5K_WME_AC_BK)) {
		printk(KERN_ERR "%s: unable to setup xmit queue for %s traffic!\n",
			dev->name, ieee80211_wme_acnames[WME_AC_BK]);
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
	} else {
		/*
		 * Mark WME capability since we have sufficient
		 * hardware queues to do proper priority scheduling.
		 */
		ic->ic_caps |= IEEE80211_C_WME;
		sc->sc_uapsdq = ath_txq_setup(sc, AR5K_TX_QUEUE_UAPSD, 0);
		if (sc->sc_uapsdq == NULL)
			DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: unable to setup UAPSD xmit queue!\n",
				__func__);
		else {
			ic->ic_caps |= IEEE80211_C_UAPSD;
			/*
			 * default UAPSD on if HW capable
			 */
			IEEE80211_COM_UAPSD_ENABLE(ic);
		}
	}
#ifdef ATH_SUPERG_XR
	ath_xr_rate_setup(dev);
	sc->sc_xrpollint = XR_DEFAULT_POLL_INTERVAL;
	sc->sc_xrpollcount = XR_DEFAULT_POLL_COUNT;
	strcpy(sc->sc_grppoll_str, XR_DEFAULT_GRPPOLL_RATE_STR);
	sc->sc_grpplq.axq_qnum = -1;
	sc->sc_xrtxq = ath_txq_setup(sc, AR5K_TX_QUEUE_DATA, AR5K_XR_DATA);
#endif

	/*
	 * Special case certain configurations.  Note the
	 * CAB queue is handled by these specially so don't
	 * include them when checking the txq setup mask.
	 */
	switch (sc->sc_txqsetup &~ ((1<<sc->sc_cabq->axq_qnum) |
				(sc->sc_uapsdq ? (1<<sc->sc_uapsdq->axq_qnum) : 0))) {
	case 0x01:
		ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0, dev);
		break;
	case 0x0f:
		ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0123, dev);
		break;
	}

	sc->sc_setdefantenna = ath_setdefantenna;
	sc->sc_rc = ath_rate_attach(sc);
	if (sc->sc_rc == NULL) {
		error = EIO;
		goto bad2;
	}

#endif
	ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0, sc);
	/* FIXME: we are only using a single hardware queue. */
	if (!ath_tx_setup(sc, WME_AC_BK, AR5K_WME_AC_BK)) {
		printk(KERN_ERR "unable to setup xmit queue\n");
		error = -EIO;
		goto bad2;
	}
	init_timer(&sc->sc_cal_ch);
	sc->sc_cal_ch.function = ath_calibrate;
	sc->sc_cal_ch.data = (unsigned long) sc;

#ifdef ATH_SUPERG_DYNTURBO
	init_timer(&sc->sc_dturbo_switch_mode);
	sc->sc_dturbo_switch_mode.function = ath_turbo_switch_mode;
	sc->sc_dturbo_switch_mode.data = (unsigned long) dev;
#endif

	sc->sc_blinking = 0;
	sc->sc_ledstate = 1;
	sc->sc_ledon = 0;			/* low true */
	sc->sc_ledidle = msecs_to_jiffies(2700);	/* 2.7 sec */
	sc->sc_dfstesttime = ATH_DFS_TEST_RETURN_PERIOD;
	init_timer(&sc->sc_ledtimer);
#if 0
	init_timer(&sc->sc_dfswaittimer);
	init_timer(&sc->sc_dfstesttimer);
#endif
	sc->sc_ledtimer.data = (unsigned long) sc;
	if (sc->sc_softled) {
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
		ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);
	}
#if 0

	/* NB: ether_setup is done by bus-specific code */
	dev->open = ath_init;
	dev->stop = ath_stop;
	dev->hard_start_xmit = ath_hardstart;
	dev->tx_timeout = ath_tx_timeout;
	dev->watchdog_timeo = 5 * HZ;			/* XXX */
	dev->set_multicast_list = ath_mode_init;
	dev->do_ioctl = ath_ioctl;
	dev->get_stats = ath_getstats;
	dev->set_mac_address = ath_set_mac_address;
 	dev->change_mtu = ath_change_mtu;
	dev->tx_queue_len = ATH_TXBUF - 1;		/* 1 for mgmt frame */
#ifdef USE_HEADERLEN_RESV
	dev->hard_header_len += sizeof(struct ieee80211_qosframe) +
				sizeof(struct llc) +
				IEEE80211_ADDR_LEN +
				IEEE80211_WEP_IVLEN +
				IEEE80211_WEP_KIDLEN;
#ifdef ATH_SUPERG_FF
	dev->hard_header_len += ATH_FF_MAX_HDR;
#endif
#endif
	ic->ic_dev = dev;
	ic->ic_mgtstart = ath_mgtstart;
	ic->ic_init = ath_init;
	ic->ic_reset = ath_reset;
	ic->ic_newassoc = ath_newassoc;
	ic->ic_updateslot = ath_updateslot;

	ic->ic_wme.wme_update = ath_wme_update;
	ic->ic_uapsd_flush = ath_uapsd_flush;

	/* XXX not right but it's not used anywhere important */
	ic->ic_phytype = MODULATION_CCK;
	ic->ic_opmode = IEEE80211_IF_TYPE_STA;
	sc->sc_opmode = IEEE80211_IF_TYPE_STA;
	/* 
	 * Set the Atheros Advanced Capabilities from station config before 
	 * starting 802.11 state machine.  Currently, set only fast-frames 
	 * capability.
	 */
	ic->ic_ath_cap = 0;
	sc->sc_fftxqmin = ATH_FF_TXQMIN;
#ifdef ATH_SUPERG_FF
	ic->ic_ath_cap |= (ath_hal_fastframesupported(ah) ? IEEE80211_ATHC_FF : 0);
#endif
	ic->ic_ath_cap |= (ath_hal_burstsupported(ah) ? IEEE80211_ATHC_BURST : 0);

#ifdef ATH_SUPERG_COMP
	ic->ic_ath_cap |= (ath_hal_compressionsupported(ah) ? IEEE80211_ATHC_COMP : 0); 
#endif

#ifdef ATH_SUPERG_DYNTURBO
	ic->ic_ath_cap |= (ath_hal_turboagsupported(ah) ? (IEEE80211_ATHC_TURBOP |
							IEEE80211_ATHC_AR) : 0);
#endif
#ifdef ATH_SUPERG_XR
	ic->ic_ath_cap |= (ath_hal_xrsupported(ah) ? IEEE80211_ATHC_XR : 0);
#endif

	ic->ic_caps |=
		  IEEE80211_C_IBSS		/* ibss, nee adhoc, mode */
		| IEEE80211_C_HOSTAP		/* hostap mode */
		| IEEE80211_C_MONITOR		/* monitor mode */
		| IEEE80211_C_AHDEMO		/* adhoc demo mode */
		| IEEE80211_C_SHPREAMBLE	/* short preamble supported */
		| IEEE80211_C_SHSLOT		/* short slot time supported */
		| IEEE80211_C_WPA		/* capable of WPA1+WPA2 */
		| IEEE80211_C_BGSCAN		/* capable of bg scanning */
		;
#endif
	/*
	 * Query the HAL to figure out h/w crypto support.
	 */
	if (ath_hal_ciphersupported(ah, AR5K_CIPHER_TKIP)) {
		/*
		 * Check if h/w does the MIC and/or whether the
		 * separate key cache entries are required to
		 * handle both tx+rx MIC keys.
		 */
		if (ath_hal_ciphersupported(ah, AR5K_CIPHER_MIC) &&
		    ath_hal_hastkipmic(ah)) {

			/*
			 * Check if h/w does MIC correctly when
			 * WMM is turned on.
			 */
			if (!ath_hal_wmetkipmic(ah))
				/* FIXME: add a flag to d80211 so we can
				   support hardware TKIP and software MIC
				   for WME? */
				hw->flags |= IEEE80211_HW_NO_TKIP_WMM_HWACCEL;
		} else {
			hw->flags |= IEEE80211_HW_TKIP_INCLUDE_MMIC;
		}

		/*
		 * If the h/w supports storing tx+rx MIC keys
		 * in one cache slot automatically enable use.
		 */
		if (ath_hal_hastkipsplit(ah) ||
		    !ath_hal_settkipsplit(ah, FALSE))
			sc->sc_splitmic = 1;
	}
#if 0
	sc->sc_hasclrkey = ath_hal_ciphersupported(ah, AR5K_CIPHER_CLR);
#if 0
	sc->sc_mcastkey = ath_hal_getmcastkeysearch(ah);
#endif
	/*
	 * TPC support can be done either with a global cap or
	 * per-packet support.  The latter is not available on
	 * all parts.  We're a bit pedantic here as all parts
	 * support a global cap.
	 */
	sc->sc_hastpc = ath_hal_hastpc(ah);
	if (sc->sc_hastpc || ath_hal_hastxpowlimit(ah))
		ic->ic_caps |= IEEE80211_C_TXPMGT;

	/*
	 * Default 11.h to start enabled.
	 */
	ic->ic_flags |= IEEE80211_F_DOTH;
	
	/*
	 * Check for misc other capabilities.
	 */
	if (ath_hal_hasbursting(ah))
		ic->ic_caps |= IEEE80211_C_BURST;
#endif
	sc->sc_hasbmask = ath_hal_hasbssidmask(ah);
	sc->sc_hastsfadd = ath_hal_hastsfadjust(ah);
#if 0
	/*
	 * Indicate we need the 802.11 header padded to a
	 * 32-bit boundary for 4-address and QoS frames.
	 */
	ic->ic_flags |= IEEE80211_F_DATAPAD;

#endif
	/*
	 * Query the HAL about antenna support
	 * Enable rx fast diversity if HAL has support
	 */
	if (ath_hal_hasdiversity(ah)) {
		sc->sc_hasdiversity = 1;
		ath_hal_setdiversity(ah, TRUE);
		sc->sc_diversity = 1;
	} else {
		sc->sc_hasdiversity = 0;
		sc->sc_diversity = 0;
		ath_hal_setdiversity(ah, FALSE);
	}
	sc->sc_defant = ath_hal_getdefantenna(ah);

	/*
	 * Not all chips have the VEOL support we want to
	 * use with IBSS beacons; check here for it.
	 */
	sc->sc_hasveol = ath_hal_hasveol(ah);

	/* get mac address from hardware */
	ath_hal_getmac(ah, sc->sc_hw->wiphy->perm_addr);
	if (sc->sc_hasbmask) {
		memset(sc->sc_bssidmask, 0xff, ETH_ALEN);
		ath_hal_setbssidmask(ah, sc->sc_bssidmask);
	}

#if 0
	/* call MI attach routine. */
	ieee80211_ifattach(ic);
	/* override default methods */
	ic->ic_node_alloc = ath_node_alloc;
	sc->sc_node_free = ic->ic_node_free;
	ic->ic_node_free = ath_node_free;
	ic->ic_node_getrssi = ath_node_getrssi;
#ifdef ATH_SUPERG_XR
	ic->ic_node_move_data = ath_node_move_data;
#endif
	sc->sc_node_cleanup = ic->ic_node_cleanup;
	ic->ic_node_cleanup = ath_node_cleanup;
	sc->sc_recv_mgmt = ic->ic_recv_mgmt;
	ic->ic_recv_mgmt = ath_recv_mgmt;

	ic->ic_vap_create = ath_vap_create;
	ic->ic_vap_delete = ath_vap_delete;

	ic->ic_scan_start = ath_scan_start;
	ic->ic_scan_end = ath_scan_end;
	ic->ic_set_channel = ath_set_channel;

	ic->ic_set_coverageclass = ath_set_coverageclass;
	ic->ic_mhz2ieee = ath_mhz2ieee;

	if (register_netdev(dev)) {
		printk(KERN_ERR "%s: unable to register device\n", dev->name);
		goto bad3;
	}
	/*
	 * Attach dynamic MIB vars and announce support
	 * now that we have a device name with unit number.
	 */
#endif
	if (ath_d80211_attach(sc)) {
		error = -EIO;
		goto bad;
	}
	strlcpy(sc->name, wiphy_name(sc->sc_hw->wiphy), sizeof(sc->name));
	ath_dynamic_sysctl_register(sc);
	ath_announce(sc);
#ifdef ATH_TX99_DIAG
	printk("%s: TX99 support enabled\n", dev->name);
#endif
	sc->sc_invalid = 0;
	
	if ((sc->sc_num_modes > 0) && (sc->sc_hw_modes[0].num_channels > 0)) {
	    sc->sc_hw->conf.freq = sc->sc_hw_modes[0].channels[0].freq;
	    sc->sc_hw->conf.channel_val = sc->sc_hw_modes[0].channels[0].val;
	}
	
	return 0;
bad2:
	ath_tx_cleanup(sc);
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
ath_detach(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	AR5K_INT tmp;
	DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);
	ath_stop(sc);

	ath_hal_setpower(sc->sc_ah, AR5K_PM_AWAKE,0);
	/* Flush the radar task if it's scheduled */
#if 0
	if (sc->sc_rtasksched == 1)
		flush_scheduled_work();
#endif

	sc->sc_invalid = 1;

	/*
	 * NB: the order of these is important:
	 * o call the 802.11 layer before detaching the HAL to
	 *   ensure callbacks into the driver to delete global
	 *   key cache entries can be handled
	 * o reclaim the tx queue data structures after calling
	 *   the 802.11 layer as we'll get called back to reclaim
	 *   node state and potentially want to use them
	 * o to cleanup the tx queues the HAL is called, so detach
	 *   it last
	 * Other than that, it's straightforward...
	 */
	ieee80211_unregister_hw(sc->sc_hw);	

	ath_hal_intrset(ah, 0);		/* disable further intr's */
	ath_hal_getisr(ah, &tmp);	/* clear ISR */
#ifdef ATH_TX99_DIAG
	if (sc->sc_tx99 != NULL)
		sc->sc_tx99->detach(sc->sc_tx99);
#endif
	ath_desc_free(sc);
	ath_tx_cleanup(sc);
	ath_hal_detach(ah);

	ath_dynamic_sysctl_unregister(sc);
	ATH_LOCK_DESTROY(sc);
	ath_d80211_detach(sc);
	return 0;
}

#if 0
static struct ieee80211vap *
ath_vap_create(struct ieee80211com *ic, const char *name, int unit,
	int opmode, int flags, struct net_device *mdev)
{
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct net_device *dev;
	struct ath_vap *avp;
	struct ieee80211vap *vap;
	int ic_opmode;

	if (ic->ic_dev->flags & IFF_RUNNING) {
		/* needs to disable hardware too */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* stop xmit side */
		ath_stoprecv(sc);		/* stop recv side */
	}
	/* XXX ic unlocked and race against add */
	switch (opmode) {
	case IEEE80211_M_STA:	/* ap+sta for repeater application */
		if (sc->sc_nstavaps != 0)  /* only one sta regardless */
			return NULL;
		if ((sc->sc_nvaps != 0) && (!(flags & IEEE80211_NO_STABEACONS)))
			return NULL;   /* If using station beacons, must first up */
		if (flags & IEEE80211_NO_STABEACONS) {
			sc->sc_nostabeacons = 1;
			ic_opmode = IEEE80211_M_HOSTAP;	/* Run with chip in AP mode */
		} else 
			ic_opmode = opmode;
		break;
	case IEEE80211_M_IBSS:
		if (sc->sc_nvaps != 0)		/* only one */
			return NULL;
		ic_opmode = opmode;
		break;
	case IEEE80211_M_AHDEMO:
	case IEEE80211_M_MONITOR:
		if (sc->sc_nvaps != 0 && ic->ic_opmode != opmode) {
			/* preserve existing mode */
			ic_opmode = ic->ic_opmode;
		} else
			ic_opmode = opmode;
		break;
	case IEEE80211_M_HOSTAP:
	case IEEE80211_M_WDS:
		/* permit multiple ap's and/or wds links */
		/* XXX sta+ap for repeater/bridge application */
		if ((sc->sc_nvaps != 0) && (ic->ic_opmode == IEEE80211_M_STA))
			return NULL;
		/* XXX not right, beacon buffer is allocated on RUN trans */
		if (opmode == IEEE80211_M_HOSTAP && STAILQ_EMPTY(&sc->sc_bbuf))
			return NULL;
		/*
		 * XXX Not sure if this is correct when operating only
		 * with WDS links.
		 */
		ic_opmode = IEEE80211_M_HOSTAP;

		break;
	default:
		return NULL;
	}

	if (sc->sc_nvaps >= ATH_BCBUF) {
		printk(KERN_WARNING "too many virtual ap's (already got %d)\n", sc->sc_nvaps);
		return NULL;
	}

	dev = alloc_etherdev(sizeof(struct ath_vap) + sc->sc_rc->arc_vap_space);
	if (dev == NULL) {
		/* XXX msg */
		return NULL;
	}
	
	avp = dev->priv;
	ieee80211_vap_setup(ic, dev, name, unit, opmode, flags);
	/* override with driver methods */
	vap = &avp->av_vap;
	avp->av_newstate = vap->iv_newstate;
	vap->iv_newstate = ath_newstate;
	vap->iv_key_update_begin = ath_key_update_begin;
	vap->iv_key_update_end = ath_key_update_end;
#ifdef ATH_SUPERG_COMP
	vap->iv_comp_set = ath_comp_set;
#endif

	/* Let rate control register proc entries for the VAP */
	ath_rate_dynamic_proc_register(vap);

	/*
	 * Change the interface type for monitor mode.
	 */
	if (opmode == IEEE80211_M_MONITOR)
		dev->type = ARPHRD_IEEE80211_PRISM;
	if ((flags & IEEE80211_CLONE_BSSID) &&
	    sc->sc_nvaps != 0 && opmode != IEEE80211_M_WDS && sc->sc_hasbmask) {
		struct ieee80211vap *v;
		int id_mask, id;
		
		/*
		 * Hardware supports the bssid mask and a unique
		 * bssid was requested.  Assign a new mac address
		 * and expand our bssid mask to cover the active
		 * virtual ap's with distinct addresses.
		 */
		
		/* do a full search to mark all the allocated VAPs */
		id_mask = 0;
		TAILQ_FOREACH(v, &ic->ic_vaps, iv_next)
			id_mask |= (1 << ATH_GET_VAP_ID(v->iv_myaddr));
		
		for (id = 0; id < ATH_BCBUF; id++) {
			/* get the first available slot */
			if ((id_mask & (1 << id)) == 0) {
				ATH_SET_VAP_BSSID(vap->iv_myaddr, id);
				break;
			}
		}
	}
	avp->av_bslot = -1;
	STAILQ_INIT(&avp->av_mcastq.axq_q);
	ATH_TXQ_LOCK_INIT(&avp->av_mcastq);
	if (opmode == IEEE80211_M_HOSTAP || opmode == IEEE80211_M_IBSS) {
		/*
		 * Allocate beacon state for hostap/ibss.  We know
		 * a buffer is available because of the check above.
		 */
		avp->av_bcbuf = STAILQ_FIRST(&sc->sc_bbuf);
		STAILQ_REMOVE_HEAD(&sc->sc_bbuf, bf_list);
		if (opmode == IEEE80211_M_HOSTAP || !sc->sc_hasveol) {
			int slot;
			/*
			 * Assign the VAP to a beacon xmit slot.  As
			 * above, this cannot fail to find one.
			 */
			avp->av_bslot = 0;
			for (slot = 0; slot < ATH_BCBUF; slot++)
				if (sc->sc_bslot[slot] == NULL) {
					/*
					 * XXX hack, space out slots to better
					 * deal with misses
					 */
					if (slot + 1 < ATH_BCBUF &&
					    sc->sc_bslot[slot+1] == NULL) {
						avp->av_bslot = slot + 1;
						break;
					}
					avp->av_bslot = slot;
					/* NB: keep looking for a double slot */
				}
			KASSERT(sc->sc_bslot[avp->av_bslot] == NULL,
				("beacon slot %u not empty?", avp->av_bslot));
			sc->sc_bslot[avp->av_bslot] = vap;
			sc->sc_nbcnvaps++;
		}
		if ((opmode == IEEE80211_M_HOSTAP) && (sc->sc_hastsfadd)) {
			/*
			 * Multiple VAPs are to transmit beacons and we
			 * have h/w support for TSF adjusting; enable use
			 * of staggered beacons.
			 */
			/* XXX check for beacon interval too small */
			sc->sc_stagbeacons = 1;
		}
	}
	if (sc->sc_hastsfadd)
		ath_hal_settsfadjust(sc->sc_ah, sc->sc_stagbeacons);
	SET_NETDEV_DEV(dev, mdev->class_dev.dev);
	/* complete setup */
	(void) ieee80211_vap_attach(vap,
		ieee80211_media_change, ieee80211_media_status);

	ic->ic_opmode = ic_opmode;
	
	if (opmode != IEEE80211_M_WDS)
		sc->sc_nvaps++;
		
	if (opmode == IEEE80211_M_STA)
		sc->sc_nstavaps++;
	else if (opmode == IEEE80211_M_MONITOR)
		sc->sc_nmonvaps++;
	/*
	 * Adhoc demo mode is a pseudo mode; to the HAL it's
	 * just ibss mode and the driver doesn't use management
	 * frames.  Other modes carry over directly to the HAL.
	 */
	if (ic->ic_opmode == IEEE80211_M_AHDEMO)
		sc->sc_opmode = AR5K_M_IBSS;
	else
		sc->sc_opmode = (AR5K_OPMODE) ic->ic_opmode;	/* NB: compatible */

#ifdef ATH_SUPERG_XR
	if ( vap->iv_flags & IEEE80211_F_XR ) {
		if (ath_descdma_setup(sc, &sc->sc_grppolldma, &sc->sc_grppollbuf,
			"grppoll", (sc->sc_xrpollcount+1) * AR5K_ANTENNA_MAX_MODE, 1) != 0)
			printk("%s:grppoll Buf allocation failed \n",__func__);
		if (!sc->sc_xrtxq)
			sc->sc_xrtxq = ath_txq_setup(sc, AR5K_TX_QUEUE_XR_DATA, AR5K_WME_AC_BE)
		if (sc->sc_hasdiversity) {
			/* Save current diversity state if user destroys XR VAP */
			sc->sc_olddiversity = sc->sc_diversity;
			ath_hal_setdiversity(sc->sc_ah, 0);
			sc->sc_diversity = 0;
		}
	}
#endif
	if (ic->ic_dev->flags & IFF_RUNNING) {
		/* restart hardware */
		if (ath_startrecv(sc) != 0)	/* restart recv */
			printk("%s: %s: unable to start recv logic\n",
				dev->name, __func__);
		if (sc->sc_beacons)
			ath_beacon_config(sc, NULL);	/* restart beacons */
		ath_hal_intrset(ah, sc->sc_imask);
	}

	return vap;
}

static void
ath_vap_delete(struct ieee80211vap *vap)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_vap *avp = ATH_VAP(vap);
	int decrease = 1;
	int i;
	KASSERT(vap->iv_state == IEEE80211_S_INIT, ("VAP not stopped"));

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Quiesce the hardware while we remove the VAP.  In
		 * particular we need to reclaim all references to the
		 * VAP state by any frames pending on the tx queues.
		 *
		 * XXX can we do this w/o affecting other VAPs?
		 */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* stop xmit side */
		ath_stoprecv(sc);		/* stop recv side */
	}

	/*
	 * Reclaim any pending mcast bufs on the VAP.
	 */
	ath_tx_draintxq(sc, &avp->av_mcastq);
	ATH_TXQ_LOCK_DESTROY(&avp->av_mcastq);

	/*
	 * Reclaim beacon state.  Note this must be done before
	 * VAP instance is reclaimed as we may have a reference
	 * to it in the buffer for the beacon frame.
	 */
	if (avp->av_bcbuf != NULL) {
		if (avp->av_bslot != -1) {
			sc->sc_bslot[avp->av_bslot] = NULL;
			sc->sc_nbcnvaps--;
		}
		ath_beacon_return(sc, avp->av_bcbuf);
		avp->av_bcbuf = NULL;
		if (sc->sc_nbcnvaps == 0)
			sc->sc_stagbeacons = 0;
	}
	if (vap->iv_opmode == IEEE80211_M_STA) {
		sc->sc_nstavaps--;
		if (sc->sc_nostabeacons)
			sc->sc_nostabeacons = 0;
	} else if (vap->iv_opmode == IEEE80211_M_MONITOR) {
		sc->sc_nmonvaps--;
	} else if (vap->iv_opmode == IEEE80211_M_WDS) {
		decrease = 0;
	}
	ieee80211_vap_detach(vap);
	/* NB: memory is reclaimed through dev->destructor callback */
	if (decrease)
		sc->sc_nvaps--;

#ifdef ATH_SUPERG_XR 
	/*
	 * If it's an XR VAP, free the memory allocated explicitly.
	 * Since the XR VAP is not registered, OS cannot free the memory.
	 */
	if (vap->iv_flags & IEEE80211_F_XR) {
		ath_grppoll_stop(vap);
		ath_descdma_cleanup(sc, &sc->sc_grppolldma, &sc->sc_grppollbuf, BUS_DMA_FROMDEVICE);
		memset(&sc->sc_grppollbuf, 0, sizeof(sc->sc_grppollbuf));
		memset(&sc->sc_grppolldma, 0, sizeof(sc->sc_grppolldma));
		if (vap->iv_xrvap)
			vap->iv_xrvap->iv_xrvap = NULL;
		kfree(vap->iv_dev);
		ath_tx_cleanupq(sc,sc->sc_xrtxq);
		sc->sc_xrtxq = NULL;
		if (sc->sc_hasdiversity) {
			/* Restore diversity setting to old diversity setting */
			ath_hal_setdiversity(ah, sc->sc_olddiversity);
			sc->sc_diversity = sc->sc_olddiversity;
		}
	}
#endif

	for (i = 0; i < IEEE80211_APPIE_NUM_OF_FRAME; ++ i) {
		if (vap->app_ie[i].ie != NULL) {
			FREE(vap->app_ie[i].ie, M_DEVBUF);
			vap->app_ie[i].ie = NULL;
			vap->app_ie[i].length = 0;
		}
	}

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Restart rx+tx machines if device is still running.
		 */
		if (ath_startrecv(sc) != 0)	/* restart recv */
			printk("%s: %s: unable to start recv logic\n",
				dev->name, __func__);
		if (sc->sc_beacons)
			ath_beacon_config(sc, NULL);	/* restart beacons */
		ath_hal_intrset(ah, sc->sc_imask);
	}
}

#endif
void
ath_suspend(struct ath_softc *sc)
{
	DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);
	ath_stop(sc);
}

void
ath_resume(struct ath_softc *sc)
{
	DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);
	ath_init(sc);
}

static void
ath_uapsd_processtriggers(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct sk_buff *skb;
#if 0
	struct ieee80211_node *ni;
	struct ath_node *an;
	struct ieee80211_qosframe *qwh;
	struct ath_txq *uapsd_xmit_q = sc->sc_uapsdq;
	struct ieee80211com *ic = &sc->sc_ic;
#endif
	int retval;
#if 0
	int ac;
	u_int8_t tid;
	u_int16_t frame_seq;
#endif
	u_int64_t tsf;
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc + \
		((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))

	/* XXXAPSD: build in check against max triggers we could see
	 *          based on ic->ic_uapsdmaxtriggers.
	 */

	tsf = ath_hal_gettsf64(ah);
	ATH_RXBUF_LOCK(sc);
	if (sc->sc_rxbufcur == NULL)
		sc->sc_rxbufcur = STAILQ_FIRST(&sc->sc_rxbuf);
	for (bf = sc->sc_rxbufcur; bf; bf = STAILQ_NEXT(bf, bf_list)) {
		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at the end */
			break;
		}
		if (bf->bf_status & ATH_BUFSTATUS_DONE) {
			/* 
			 * already processed this buffer (shouldn't occur if
			 * we change code to always process descriptors in
			 * rx intr handler - as opposed to sometimes processing
			 * in the rx tasklet).
			 */
			continue;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {		/* XXX ??? can this happen */
			printk("%s: no skbuff\n", __func__);
			continue;
		}

		/* 
		 * NB: descriptor memory doesn't need to be sync'd
		 *     due to the way it was allocated. 
		 */

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
		retval = ath_hal_rxprocdesc(ah, ds, bf->bf_daddr, PA2DESC(sc, ds->ds_link));
		if (AR5K_EINPROGRESS == retval)
			break;

		/* XXX: we do not support frames spanning multiple descriptors */
		bf->bf_status |= ATH_BUFSTATUS_DONE;

#if 0
		/* errors? */
		if (ds->ds_rxstat.rs_status)
			continue;

		/* prepare wireless header for examination */
		bus_dma_sync_single(sc->sc_bdev, bf->bf_skbaddr, 
							sizeof(struct ieee80211_qosframe), 
							BUS_DMA_FROMDEVICE);
		qwh = (struct ieee80211_qosframe *) skb->data;

		/* find the node. it MUST be in the keycache. */
		if (ds->ds_rxstat.rs_keyix == AR5K_RXKEYIX_INVALID ||
		    (ni = sc->sc_keyixmap[ds->ds_rxstat.rs_keyix]) == NULL) {
			/* 
			 * XXX: this can occur if WEP mode is used for non-Atheros clients
			 *      (since we do not know which of the 4 WEP keys will be used
			 *      at association time, so cannot setup a key-cache entry.
			 *      The Atheros client can convey this in the Atheros IE.)
			 *
			 * TODO: The fix is to use the hash lookup on the node here.
			 */
#if 0
			/*
			 * This print is very chatty, so removing for now.
			 */
			DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: U-APSD node (%s) has invalid keycache entry\n",
				__func__, ether_sprintf(qwh->i_addr2));
#endif
			continue;
		}
		
		if (!(ni->ni_flags & IEEE80211_NODE_UAPSD))
			continue;
		
		/*
		 * Must deal with change of state here, since otherwise there would
		 * be a race (on two quick frames from STA) between this code and the
		 * tasklet where we would:
		 *   - miss a trigger on entry to PS if we're already trigger hunting
		 *   - generate spurious SP on exit (due to frame following exit frame)
		 */
		if (((qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) ^
		     (ni->ni_flags & IEEE80211_NODE_PWR_MGT))) {
			/*
			 * NB: do not require lock here since this runs at intr
			 * "proper" time and cannot be interrupted by rx tasklet
			 * (code there has lock). May want to place a macro here
			 * (that does nothing) to make this more clear.
			 */
			ni->ni_flags |= IEEE80211_NODE_PS_CHANGED;
			ni->ni_pschangeseq = *(__le16 *)(&qwh->i_seq[0]);
			ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
			ni->ni_flags ^= IEEE80211_NODE_PWR_MGT;
			if (qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) {
				ni->ni_flags |= IEEE80211_NODE_UAPSD_TRIG;
				ic->ic_uapsdmaxtriggers++;
				WME_UAPSD_NODE_TRIGSEQINIT(ni);
				DPRINTF(sc, ATH_DEBUG_UAPSD,
					"%s: Node (%s) became U-APSD triggerable (%d)\n", 
					__func__, ether_sprintf(qwh->i_addr2),
					ic->ic_uapsdmaxtriggers);
			} else {
				ni->ni_flags &= ~IEEE80211_NODE_UAPSD_TRIG;
				ic->ic_uapsdmaxtriggers--;
				DPRINTF(sc, ATH_DEBUG_UAPSD,
					"%s: Node (%s) no longer U-APSD triggerable (%d)\n", 
					__func__, ether_sprintf(qwh->i_addr2),
					ic->ic_uapsdmaxtriggers);
				/* 
				 * XXX: rapidly thrashing sta could get 
				 * out-of-order frames due this flush placing
				 * frames on backlogged regular AC queue and
				 * re-entry to PS having fresh arrivals onto
				 * faster UPSD delivery queue. if this is a
				 * big problem we may need to drop these.
				 */
				ath_uapsd_flush(ni);
			}
			
			continue;
		}

		if (ic->ic_uapsdmaxtriggers == 0)
			continue;
		
		/* make sure the frame is QoS data/null */
		/* NB: with current sub-type definitions, the 
		 * IEEE80211_FC0_SUBTYPE_QOS check, below, covers the 
		 * QoS null case too.
		 */
		if (((qwh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_DATA) ||
		     !(qwh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS))
			continue;
		
		/*
		 * To be a trigger:
		 *   - node is in triggerable state
		 *   - QoS data/null frame with triggerable AC
		 */
		tid = qwh->i_qos[0] & IEEE80211_QOS_TID;
		ac = TID_TO_WME_AC(tid);
		if (!WME_UAPSD_AC_CAN_TRIGGER(ac, ni))
			continue;
		
		DPRINTF(sc, ATH_DEBUG_UAPSD, 
			"%s: U-APSD trigger detected for node (%s) on AC %d\n",
			__func__, ether_sprintf(ni->ni_macaddr), ac);
		if (ni->ni_flags & IEEE80211_NODE_UAPSD_SP) {
			/* have trigger, but SP in progress, so ignore */
			DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s:   SP already in progress - ignoring\n",
				__func__);
			continue;
		}

		/*
		 * Detect duplicate triggers and drop if so.
		 */
		frame_seq = le16toh(*(__le16 *)qwh->i_seq);
		if ((qwh->i_fc[1] & IEEE80211_FC1_RETRY) &&
		    frame_seq == ni->ni_uapsd_trigseq[ac]) {
			DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: dropped dup trigger, ac %d, seq %d\n",
				__func__, ac, frame_seq);
			continue;
		}

		an = ATH_NODE(ni);

		/* start the SP */
		ATH_NODE_UAPSD_LOCK(an);
		ni->ni_stats.ns_uapsd_triggers++;
		ni->ni_flags |= IEEE80211_NODE_UAPSD_SP;
		ni->ni_uapsd_trigseq[ac] = frame_seq;
		ATH_NODE_UAPSD_UNLOCK(an);

		ATH_TXQ_LOCK(uapsd_xmit_q);
		if (STAILQ_EMPTY(&an->an_uapsd_q)) {
			DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s: Queue empty, generating QoS NULL to send\n",
				__func__);
			/* 
			 * Empty queue, so need to send QoS null on this ac. Make a
			 * call that will dump a QoS null onto the node's queue, then
			 * we can proceed as normal.
			 */
			ieee80211_send_qosnulldata(ni, ac);
		}

		if (STAILQ_FIRST(&an->an_uapsd_q)) {
			struct ath_buf *last_buf = STAILQ_LAST(&an->an_uapsd_q, ath_buf, bf_list);
			struct ath_desc *last_desc = last_buf->bf_desc;
			struct ieee80211_qosframe *qwhl = (struct ieee80211_qosframe *)last_buf->bf_skb->data;
			/* 
			 * NB: flip the bit to cause intr on the EOSP desc,
			 * which is the last one
			 */
			ath_hal_txreqintrdesc(sc->sc_ah, last_desc);
			qwhl->i_qos[0] |= IEEE80211_QOS_EOSP;

			if (IEEE80211_VAP_EOSPDROP_ENABLED(ni->ni_vap)) {
				/* simulate lost EOSP */
				qwhl->i_addr1[0] |= 0x40;
			}
			
			/* more data bit only for EOSP frame */
			if (an->an_uapsd_overflowqdepth)
				qwhl->i_fc[1] |= IEEE80211_FC1_MORE_DATA;
			else if (IEEE80211_NODE_UAPSD_USETIM(ni))
				ni->ni_vap->iv_set_tim(ni, 0);

			ni->ni_stats.ns_tx_uapsd += an->an_uapsd_qdepth;

			bus_dma_sync_single(sc->sc_bdev, last_buf->bf_skbaddr,
				sizeof(*qwhl), BUS_DMA_TODEVICE);
			
			if (uapsd_xmit_q->axq_link) {
#ifdef AH_NEED_DESC_SWAP
				*uapsd_xmit_q->axq_link = cpu_to_le32(STAILQ_FIRST(&an->an_uapsd_q)->bf_daddr);
#else
				*uapsd_xmit_q->axq_link = STAILQ_FIRST(&an->an_uapsd_q)->bf_daddr;
#endif
			}
			/* below leaves an_uapsd_q NULL */
			STAILQ_CONCAT(&uapsd_xmit_q->axq_q, &an->an_uapsd_q);
			uapsd_xmit_q->axq_link = &last_desc->ds_link;
			ath_hal_puttxbuf(sc->sc_ah, 
				uapsd_xmit_q->axq_qnum, 
				(STAILQ_FIRST(&uapsd_xmit_q->axq_q))->bf_daddr);
			ath_hal_txstart(sc->sc_ah, uapsd_xmit_q->axq_qnum);
		}
		an->an_uapsd_qdepth = 0;

		ATH_TXQ_UNLOCK(uapsd_xmit_q);
#endif
	}
	sc->sc_rxbufcur = bf;
	ATH_RXBUF_UNLOCK(sc);
#undef PA2DESC
}

/*
 * Interrupt handler.  Most of the actual processing is deferred.
 */
irqreturn_t
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
ath_intr(int irq, void *dev_id)
#else
ath_intr(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	struct ath_softc *sc = dev_id;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_INT status;
	int needmark;

	if (sc->sc_invalid) {
		/*
		 * The hardware is not ready/present, don't touch anything.
		 * Note this can happen early on if the IRQ is shared.
		 */
		return IRQ_NONE;
	}
	if (!ath_hw_irq_pending(ah))		/* shared irq, not for us */
		return IRQ_NONE;

	needmark = 0;
	/*
	 * Figure out the reason(s) for the interrupt.  Note
	 * that the HAL returns a pseudo-ISR that may include
	 * bits we haven't explicitly enabled so we mask the
	 * value to ensure we only process bits we requested.
	 */
	ath_hal_getisr(ah, &status);		/* NB: clears ISR too */
	DPRINTF(sc, ATH_DEBUG_INTR, "%s: status 0x%x\n", __func__, status);
	status &= sc->sc_imask;			/* discard unasked for bits */
	if (status & AR5K_INT_FATAL) {
		sc->sc_stats.ast_hardware++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		ATH_SCHEDULE_TQUEUE(&sc->sc_fataltq, &needmark);
	} else if (status & AR5K_INT_RXORN) {
		sc->sc_stats.ast_rxorn++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		ATH_SCHEDULE_TQUEUE(&sc->sc_rxorntq, &needmark);
	} else {
		if (status & AR5K_INT_SWBA) {
			/*
			 * Software beacon alert--time to send a beacon.
			 */
			ATH_SCHEDULE_TQUEUE(&sc->sc_beacontq, &needmark);
		}
		if (status & AR5K_INT_RXEOL) {
			/*
			 * NB: the hardware should re-read the link when
			 *     RXE bit is written, but it doesn't work at
			 *     least on older hardware revs.
			 */
			sc->sc_stats.ast_rxeol++;
		}
		if (status & AR5K_INT_TXURN) {
			sc->sc_stats.ast_txurn++;
			/* bump tx trigger level */
			ath_hal_updatetxtriglevel(ah, TRUE);
		}
		if (status & AR5K_INT_RX) {
			ath_uapsd_processtriggers(sc);
			/* Get the noise floor data in interrupt context as we can't get it
			 * per frame, so we need to get it as soon as possible (i.e. the tasklet
			 * might take too long to fire */
			//ath_hal_process_noisefloor(ah);
			//sc->sc_channoise = ath_hal_get_channel_noise(ah, &(sc->sc_curchan));
			ATH_SCHEDULE_TQUEUE(&sc->sc_rxtq, &needmark);
		}
		if (status & AR5K_INT_TX) {
#ifdef ATH_SUPERG_DYNTURBO
			/*
			 * Check if the beacon queue caused the interrupt 
			 * when a dynamic turbo switch
			 * is pending so we can initiate the change. 
			 * XXX must wait for all VAPs' beacons
			 */

			if (sc->sc_dturbo_switch) {
				u_int32_t txqs = (1 << sc->sc_bhalq);
				ath_hal_gettxintrtxqs(ah, &txqs);
				if(txqs & (1 << sc->sc_bhalq)) {
					sc->sc_dturbo_switch = 0;
					/*
					 * Hack: defer switch for 10ms to permit slow
					 * clients time to track us.  This especially
					 * noticeable with Windows clients.
					 */
					mod_timer(&sc->sc_dturbo_switch_mode,
							  jiffies + msecs_to_jiffies(10));
				}
			} 
#endif
			ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, &needmark);
		}
		if (status & AR5K_INT_BMISS) {
			sc->sc_stats.ast_bmiss++;
			ATH_SCHEDULE_TQUEUE(&sc->sc_bmisstq, &needmark);
		}
		if (status & AR5K_INT_MIB) {
			sc->sc_stats.ast_mib++;
			/*
			 * Disable interrupts until we service the MIB
			 * interrupt; otherwise it will continue to fire.
			 */
			ath_hal_intrset(ah, 0);
			/*
			 * Let the HAL handle the event.  We assume it will
			 * clear whatever condition caused the interrupt.
			 */
//			ath_hal_mibevent(ah, &sc->sc_halstats);
			ath_hal_intrset(ah, sc->sc_imask);
		}
	}
	if (needmark)
		mark_bh(IMMEDIATE_BH);
	return IRQ_HANDLED;
}

#if 0
static void
ath_radar_task(struct work_struct *thr)
{
	struct ath_softc *sc = container_of(thr, struct ath_softc, sc_radartask);
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel ichan;
	AR5K_CHANNEL hchan;

	sc->sc_rtasksched = 0;
	if (ath_hal_procdfs(ah, &hchan)) {
		/*
		 * DFS was found, initiate channel change
		 */
		ichan.ic_ieee = ath_hal_mhz2ieee(hchan.freq, hchan.channel_flags);
		ichan.ic_freq = hchan.freq;
		ichan.ic_flags = hchan.channel_flags;

		if ((sc->sc_curchan.freq == hchan.freq) &&
		    (sc->sc_curchan.channel_flags == hchan.freq)) {
			if (hchan.private_flags & CHANNEL_INTERFERENCE)
				sc->sc_curchan.private_flags |= CHANNEL_INTERFERENCE;
		}
		ieee80211_mark_dfs(ic, &ichan);
		if (((ic->ic_flags_ext & IEEE80211_FEXT_MARKDFS) == 0) &&
		    (ic->ic_opmode == IEEE80211_M_HOSTAP)) {
			sc->sc_dfstest_ieeechan = ic->ic_curchan->ic_ieee;
			sc->sc_dfstesttimer.function = ath_dfs_test_return;
			sc->sc_dfstesttimer.expires = jiffies + (sc->sc_dfstesttime * HZ);
			sc->sc_dfstesttimer.data = (unsigned long)sc;
			if (sc->sc_dfstest == 0) {
				sc->sc_dfstest = 1;
				add_timer(&sc->sc_dfstesttimer);
			}
		}
	}
}

static void
ath_dfs_test_return(unsigned long data)
{
	struct ath_softc *sc = (struct ath_softc *)data; 
	struct ieee80211com *ic = &sc->sc_ic;

	sc->sc_dfstest = 0;
	ieee80211_dfs_test_return(ic, sc->sc_dfstest_ieeechan);
}
#endif

static void
ath_fatal_tasklet(TQUEUE_ARG data)
{
	struct ath_softc *sc = (struct ath_softc *)data;

	printk("%s: hardware error; resetting\n", sc->name);
	ath_reset(sc);
}

static void
ath_rxorn_tasklet(TQUEUE_ARG data)
{
	struct ath_softc *sc = (struct ath_softc *)data;

	printk("%s: rx FIFO overrun; resetting\n", sc->name);
	ath_reset(sc);
}

#if 0
static void
ath_bmiss_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);

	ieee80211_beacon_miss(&sc->sc_ic);
}

static u_int
ath_chan2flags(struct ieee80211_channel *chan)
{
	u_int flags;
	static const u_int modeflags[] = {
		0,		/* IEEE80211_MODE_AUTO    */
		CHANNEL_A,	/* IEEE80211_MODE_11A     */
		CHANNEL_B,	/* IEEE80211_MODE_11B     */
		CHANNEL_PUREG,	/* IEEE80211_MODE_11G     */
		0,		/* IEEE80211_MODE_FH      */
		CHANNEL_108A,	/* IEEE80211_MODE_TURBO_A */
		CHANNEL_108G,	/* IEEE80211_MODE_TURBO_G */
	};

	flags = modeflags[ieee80211_chan2mode(chan)];

	if (IEEE80211_IS_CHAN_HALF(chan))
		flags |= CHANNEL_HALF;
	else if (IEEE80211_IS_CHAN_QUARTER(chan))
		flags |= CHANNEL_QUARTER;

	return flags;
}
#endif

/*
 * Context: process context
 */

int
ath_init(struct ath_softc *sc)
{
	struct ieee80211_conf *conf = &sc->sc_hw->conf;
	AR5K_OPMODE opmode = sc->sc_opmode;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_STATUS status;
	int error = 0;

	ATH_LOCK(sc);

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: mode %d\n", __func__, opmode);

	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop_locked(sc);

#ifdef ATH_CAP_TPC
	ath_hal_setcapability(sc->sc_ah, AR5K_CAP_TPC, 0, 1, NULL);
#endif

#if 0
	/* Whether we should enable h/w TKIP MIC */
	if ((ic->ic_caps & IEEE80211_C_WME) == 0)
		ath_hal_setcapability(sc->sc_ah, AR5K_CAP_TKIP_MIC, 0, 0, NULL);
	else {
		if (((ic->ic_caps & IEEE80211_C_WME_TKIPMIC) == 0) &&
		    (ic->ic_flags & IEEE80211_F_WME))
			ath_hal_setcapability(sc->sc_ah, AR5K_CAP_TKIP_MIC, 0, 0, NULL);
		else
			ath_hal_setcapability(sc->sc_ah, AR5K_CAP_TKIP_MIC, 0, 1, NULL);
	}
#endif
		
	/*
	 * Flush the skb's allocated for receive in case the rx
	 * buffer size changes.  This could be optimized but for
	 * now we do it each time under the assumption it does
	 * not happen often.
	 */
	ath_flushrecv(sc);

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	sc->sc_curchan.freq = conf->freq;
	sc->sc_curchan.channel_flags = conf->channel_val;
	if (!ath_hal_reset(ah, opmode, &sc->sc_curchan, FALSE, &status)) {
		printk("%s: unable to reset hardware: '%s' (HAL status %u) "
			"(freq %u flags 0x%x)\n", sc->name,
			ath_get_hal_status_desc(status), status,
			sc->sc_curchan.freq, sc->sc_curchan.channel_flags);
		error = -EIO;
		goto done;
	}

	if (sc->sc_softled)
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
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
#if 0
	ath_initkeytable(sc);		/* XXX still needed? */
#endif
	if (ath_startrecv(sc) != 0) {
		printk("%s: unable to start recv logic\n", sc->name);
		error = -EIO;
		goto done;
	}
	/*
	 * Enable interrupts.
	 */
	sc->sc_imask = AR5K_INT_RX | AR5K_INT_TX
		  | AR5K_INT_RXEOL | AR5K_INT_RXORN
		  | AR5K_INT_FATAL | AR5K_INT_GLOBAL;
	/*
	 * Enable MIB interrupts when there are hardware phy counters.
	 * Note we only do this (at the moment) for station mode.
	 */
	if (sc->sc_needmib && sc->sc_opmode == AR5K_M_STA)
		sc->sc_imask |= AR5K_INT_MIB;
	ath_hal_intrset(ah, sc->sc_imask);

	/*
	 * The hardware should be ready to go now so it's safe
	 * to kick the 802.11 state machine as it's likely to
	 * immediately call back to us to send mgmt frames.
	 */
	ath_chan_change(sc, &sc->sc_curchan);
	ath_set_ack_bitrate(sc, sc->sc_ackrate);
#ifdef ATH_TX99_DIAG
	if (sc->sc_tx99 != NULL)
		sc->sc_tx99->start(sc->sc_tx99);
#endif
	mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));
done:
	ATH_UNLOCK(sc);
	return error;
}

/* Caller must lock ATH_LOCK 
 *
 * Context: softIRQ
 */ 
static int
ath_stop_locked(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: invalid %u\n",
		__func__, sc->sc_invalid);

	if (1) {
		/*
		 * Shutdown the hardware and driver:
		 *    stop output from above
		 *    reset 802.11 state machine
		 *	(sends station deassoc/deauth frames)
		 *    turn off timers
		 *    disable interrupts
		 *    clear transmit machinery
		 *    clear receive machinery
		 *    turn off the radio
		 *    reclaim beacon resources
		 *
		 * Note that some of this work is not possible if the
		 * hardware is gone (invalid).
		 */
#ifdef ATH_TX99_DIAG
		if (sc->sc_tx99 != NULL)
			sc->sc_tx99->stop(sc->sc_tx99);
#endif
		ieee80211_stop_queues(sc->sc_hw);
		if (!sc->sc_invalid) {
			ath_hal_intrset(ah, 0);
			if (sc->sc_softled) {
				del_timer(&sc->sc_ledtimer);
				ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);
				sc->sc_blinking = 0;
				sc->sc_ledstate = 1;
			}
		}
		ath_draintxq(sc);
		if (!sc->sc_invalid) {
			ath_stoprecv(sc);
			ath_hal_phydisable(ah);
		} else
			sc->sc_rxlink = NULL;
		ath_beacon_free(sc);		/* XXX needed? */
	}
	if (sc->sc_softled)
		ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);
	
	return 0;
}

/*
 * Stop the device, grabbing the top-level lock to protect
 * against concurrent entry through ath_init (which can happen
 * if another thread does a system call and the thread doing the
 * stop is preempted).
 */
int
ath_stop(struct ath_softc *sc)
{
	int error;

	ATH_LOCK(sc);

	if (!sc->sc_invalid)
		ath_hal_setpower(sc->sc_ah, AR5K_PM_AWAKE,0);

	error = ath_stop_locked(sc);
#if 0
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
		ath_hal_setpower(sc->sc_ah, AR5K_PM_FULL_SLEEP,0);
	}
#endif
	ATH_UNLOCK(sc);

	del_timer(&sc->sc_cal_ch);

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
//	case AR5212_AR2315_REV6:
//	case AR5212_AR2315_REV7:
//	case AR5212_AR2317_REV1:
	case PCI_PRODUCT_ATHEROS_AR5212_0014:
	case PCI_PRODUCT_ATHEROS_AR5212_0015:
	case PCI_PRODUCT_ATHEROS_AR5212_0016:
	case PCI_PRODUCT_ATHEROS_AR5212_0017:
	case PCI_PRODUCT_ATHEROS_AR5212_0018:
	case PCI_PRODUCT_ATHEROS_AR5212_0019:
	case PCI_PRODUCT_ATHEROS_AR2413:
	case PCI_PRODUCT_ATHEROS_AR5413:
	case PCI_PRODUCT_ATHEROS_AR5424:
//	case AR5212_DEVID_FF19:
		return 5212;
	/*Not-supported by OpenHAL*/
//	case AR5213_SREV_1_0:
//	case AR5213_SREV_REG:
//	case AR_SUBVENDOR_ID_NOG:
//	case AR_SUBVENDOR_ID_NEW_A:
//		return 5213;
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
		u_int32_t v = AR5K_AR5212_STA_ID1_BASE_RATE_11B | AR5K_AR5212_STA_ID1_ACKCTS_6MB;
		if (high) {
			AR5K_REG_WRITE(AR5K_AR5212_STA_ID1, AR5K_REG_READ(AR5K_AR5212_STA_ID1) & ~v);
		} else {
			AR5K_REG_WRITE(AR5K_AR5212_STA_ID1, AR5K_REG_READ(AR5K_AR5212_STA_ID1) | v);
		}
		return 0;
	}
	return 1;
}

/*
 * Reset the hardware w/o losing operational state.  This is
 * basically a more efficient way of doing ath_stop, ath_init,
 * followed by state transitions to the current 802.11
 * operational state.  Used to recover from errors rx overrun
 * and to reset the hardware when rf gain settings must be reset.
 */
int
ath_reset(struct ath_softc *sc)
{
	struct ieee80211_conf *conf = &sc->sc_hw->conf;
	AR5K_OPMODE opmode = sc->sc_opmode;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_STATUS status;
	int i;

	/*
	 * Convert to a HAL channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	sc->sc_curchan.freq = conf->freq;
	sc->sc_curchan.channel_flags = conf->channel_val;

	ath_hal_intrset(ah, 0);		/* disable interrupts */
	ath_draintxq(sc);		/* stop xmit side */
	ath_stoprecv(sc);		/* stop recv side */
	/* NB: indicate channel change so we do a full reset */
	if (!ath_hal_reset(ah, opmode, &sc->sc_curchan, TRUE, &status))
		printk("%s: %s: unable to reset hardware: '%s' (HAL status %u)\n",
			sc->name, __func__, ath_get_hal_status_desc(status), status);
	ath_update_txpow(sc);		/* update tx power state */
	if (ath_startrecv(sc) != 0)	/* restart recv */
		printk("%s: %s: unable to start recv logic\n",
			sc->name, __func__);
	if (sc->sc_softled)
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);

	/*
	 * We may be doing a reset in response to an ioctl
	 * that changes the channel so update any state that
	 * might change as a result.
	 */
	ath_chan_change(sc, &sc->sc_curchan);
	if (sc->sc_beacons)
		ath_beacon_config(sc);	/* restart beacons */
	ath_hal_intrset(ah, sc->sc_imask);
	ath_set_ack_bitrate(sc, sc->sc_ackrate);

	for (i = 0; i < sc->sc_hw->queues; i++)
		ieee80211_wake_queue(sc->sc_hw, i);
#ifdef ATH_SUPERG_XR
	/*
	 * restart the group polls.
	 */
	if (sc->sc_xrgrppoll) {
		struct ieee80211vap *vap;
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
			if (vap && (vap->iv_flags & IEEE80211_F_XR))
				break;
		ath_grppoll_stop(vap);
		ath_grppoll_start(vap, sc->sc_xrpollcount);
	}
#endif
	return 0;
}


/* Swap transmit descriptor.
 * if AH_NEED_DESC_SWAP flag is not defined this becomes a "null"
 * function.
 */
static __inline void
ath_desc_swap(struct ath_desc *ds)
{
#ifdef AH_NEED_DESC_SWAP
	ds->ds_link = cpu_to_le32(ds->ds_link);
	ds->ds_data = cpu_to_le32(ds->ds_data);
	ds->ds_ctl0 = cpu_to_le32(ds->ds_ctl0);
	ds->ds_ctl1 = cpu_to_le32(ds->ds_ctl1);
	ds->ds_hw[0] = cpu_to_le32(ds->ds_hw[0]);
	ds->ds_hw[1] = cpu_to_le32(ds->ds_hw[1]);
#endif
}

/*
 * Insert a buffer on a txq 
 * 
 */
static __inline void
ath_tx_txqaddbuf(struct ath_softc *sc,
	struct ath_txq *txq, struct ath_buf *bf, 
	struct ath_desc *lastds, int framelen)
{
	struct ath_hal *ah = sc->sc_ah;

	/*
	 * Insert the frame on the outbound list and
	 * pass it on to the hardware.
	 */
	ATH_TXQ_LOCK(txq);
#if 0
	if (ni && ni->ni_vap && txq == &ATH_VAP(ni->ni_vap)->av_mcastq) {
		/*
		 * The CAB queue is started from the SWBA handler since
		 * frames only go out on DTIM and to avoid possible races.
		 */
		ath_hal_intrset(ah, sc->sc_imask & ~AR5K_INT_SWBA);
		ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: txq depth = %d\n", __func__, txq->axq_depth);
		if (txq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
			*txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
			*txq->axq_link = bf->bf_daddr;
#endif
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u](%p)=%llx (%p)\n",
				__func__,
				txq->axq_qnum, txq->axq_link,
				ito64(bf->bf_daddr), bf->bf_desc);
		}
		txq->axq_link = &lastds->ds_link;
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
#endif
		ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: txq depth = %d\n", __func__, txq->axq_depth);
		if (txq->axq_link == NULL) {
			ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: TXDP[%u] = %llx (%p)\n",
				__func__,
				txq->axq_qnum, ito64(bf->bf_daddr), bf->bf_desc);
		} else {
#ifdef AH_NEED_DESC_SWAP
			*txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
			*txq->axq_link = bf->bf_daddr;
#endif
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u] (%p)=%llx (%p)\n",
				__func__,
				txq->axq_qnum, txq->axq_link,
				ito64(bf->bf_daddr), bf->bf_desc);
		}
		txq->axq_link = &lastds->ds_link;
		ath_hal_txstart(ah, txq->axq_qnum);
#if 0
	}
#endif
	ATH_TXQ_UNLOCK(txq);

}

#if 0
static int 
dot11_to_ratecode(struct ath_softc *sc, const AR5K_RATE_TABLE *rt, int dot11)
{
	int index = sc->sc_rixmap[dot11 & IEEE80211_RATE_VAL];
	if (index >= 0 && index < rt->rate_count)
		return rt->rates[index].rateCode;
	
	return rt->rates[sc->sc_minrateix].rateCode;
}
#endif


static void
ath_tx_startraw(struct ath_softc *sc, struct ath_buf *bf, struct sk_buff *skb,
	       	struct ieee80211_tx_control *control, struct ath_txq *txq) 
{
	struct ath_hal *ah = sc->sc_ah;
	const AR5K_RATE_TABLE *rt;
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
	
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
					skb->data, pktlen, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %llx\n",
		__func__, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));
	
	
	bf->bf_skb = skb;
	bf->bf_node = NULL;
	
#ifdef ATH_SUPERG_FF
	bf->bf_numdesc = 1;
#endif
	
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
	ath_hal_setuptxdesc(ah, ds
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
//			    , 0		/* comp icv len */
//			    , 0		/* comp iv len */
//			    , ATH_COMP_PROC_NO_COMP_NO_CCS /* comp scheme */
			   );

#if 0
	if (ph->try1) {
		ath_hal_setupxtxdesc(sc->sc_ah, ds
			, dot11_to_ratecode(sc, rt, ph->rate1), ph->try1 /* series 1 */
			, dot11_to_ratecode(sc, rt, ph->rate2), ph->try2 /* series 2 */
			, dot11_to_ratecode(sc, rt, ph->rate3), ph->try3 /* series 3 */
			);	
	}
#endif
#ifdef ATH_SUPERG_FF
	bf->bf_flags = flags;			/* record for post-processing */
#endif

	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	
	ath_hal_filltxdesc(ah, ds
			   , skb->len	/* segment length */
			   , TRUE	/* first segment */
			   , TRUE	/* last segment */
			   , ds		/* first descriptor */
			   );
	
	/* NB: The desc swap function becomes void, 
	 * if descriptor swapping is not enabled
	 */
	ath_desc_swap(ds);
	
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
		__func__, txq->axq_qnum, ds->ds_link, ds->ds_data,
		ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);

	ath_tx_txqaddbuf(sc, txq, bf, ds, pktlen);
}

#ifdef ATH_SUPERG_FF
/*
 * Flush FF staging queue.
 */
static int
ath_ff_neverflushtestdone(struct ath_txq *txq, struct ath_buf *bf)
{
	return 0;
}

static int
ath_ff_ageflushtestdone(struct ath_txq *txq, struct ath_buf *bf)
{
	if ( (txq->axq_totalqueued - bf->bf_queueage) < ATH_FF_STAGEQAGEMAX )
		return 1;

	return 0;
}

/* Caller must not hold ATH_TXQ_LOCK and ATH_TXBUF_LOCK
 *
 * Context: softIRQ
 */
static void
ath_ffstageq_flush(struct ath_softc *sc, struct ath_txq *txq,
	int (*ath_ff_flushdonetest)(struct ath_txq *txq, struct ath_buf *bf))
{
	struct ath_buf *bf_ff = NULL;
	struct ieee80211_node *ni = NULL;
	int pktlen;
	int framecnt;

	for (;;) {
		ATH_TXQ_LOCK(txq);

		bf_ff = TAILQ_LAST(&txq->axq_stageq, axq_headtype);
		if ((!bf_ff) || ath_ff_flushdonetest(txq, bf_ff))
		{
			ATH_TXQ_UNLOCK(txq);
			break;
		}

		ni = bf_ff->bf_node;
		KASSERT(ATH_NODE(ni)->an_tx_ffbuf[bf_ff->bf_skb->priority],
			("no bf_ff on staging queue %p", bf_ff));
		ATH_NODE(ni)->an_tx_ffbuf[bf_ff->bf_skb->priority] = NULL;
		TAILQ_REMOVE(&txq->axq_stageq, bf_ff, bf_stagelist);

		ATH_TXQ_UNLOCK(txq);

		/* encap and xmit */
		bf_ff->bf_skb = ieee80211_encap(ni, bf_ff->bf_skb, &framecnt);
		if (bf_ff->bf_skb == NULL) {
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"%s: discard, encapsulation failure\n", __func__);
			sc->sc_stats.ast_tx_encap++;
			goto bad;
		}
		pktlen = bf_ff->bf_skb->len;	/* NB: don't reference skb below */
		if (ath_tx_start(sc->sc_dev, ni, bf_ff, bf_ff->bf_skb, 0) == 0)
			continue;
	bad:
		ieee80211_free_node(ni);
		if (bf_ff->bf_skb != NULL) {
			dev_kfree_skb(bf_ff->bf_skb);
			bf_ff->bf_skb = NULL;
		}
		bf_ff->bf_node = NULL;

		ATH_TXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf_ff, bf_list);
		ATH_TXBUF_UNLOCK_IRQ(sc);
	}
}
#endif


static struct ath_buf *ath_get_tx_buf(struct ath_softc *sc) {
	struct ath_buf *bf;

	ATH_TXBUF_LOCK_IRQ(sc);
	bf = STAILQ_FIRST(&sc->sc_txbuf);
	if (bf != NULL) {
		STAILQ_REMOVE_HEAD(&sc->sc_txbuf, bf_list);
	}
	/* XXX use a counter and leave at least one for mgmt frames */
	if (STAILQ_EMPTY(&sc->sc_txbuf)) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: stop queue\n", __func__);
		sc->sc_stats.ast_tx_qstop++;
		ieee80211_stop_queues(sc->sc_hw);
		sc->sc_devstopped = 1;
		ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, NULL);
	}
	ATH_TXBUF_UNLOCK_IRQ(sc);

	if (bf == NULL) {		/* NB: should not happen */
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: discard, no xmit buf\n", __func__);
		sc->sc_stats.ast_tx_nobuf++;
	}

	return bf;
}

#define ATH_HARDSTART_GET_TX_BUF_WITH_LOCK				\
	ATH_TXBUF_LOCK_IRQ(sc);						\
	bf = STAILQ_FIRST(&sc->sc_txbuf);				\
	if (bf != NULL) {						\
		STAILQ_REMOVE_HEAD(&sc->sc_txbuf, bf_list);		\
		STAILQ_INSERT_TAIL(&bf_head, bf, bf_list);              \
	}                                                               \
	/* XXX use a counter and leave at least one for mgmt frames */	\
	if (STAILQ_EMPTY(&sc->sc_txbuf)) {				\
		DPRINTF(sc, ATH_DEBUG_XMIT,				\
			"%s: stop queue\n", __func__);			\
		sc->sc_stats.ast_tx_qstop++;				\
		ieee80211_stop_queues(sc->sc_hw);			\
		sc->sc_devstopped = 1;					\
		ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, NULL); 		\
	}								\
	ATH_TXBUF_UNLOCK_IRQ(sc);					\
	if (bf == NULL) {		/* NB: should not happen */	\
		DPRINTF(sc,ATH_DEBUG_XMIT,				\
			"%s: discard, no xmit buf\n", __func__);	\
		sc->sc_stats.ast_tx_nobuf++;				\
		goto hardstart_fail;					\
	}


int
ath_d80211_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
	      struct ieee80211_tx_control *control)
{
	struct ath_softc *sc = hw->priv;
	struct ath_buf *bf = NULL;
	STAILQ_HEAD(tmp_bf_head, ath_buf) bf_head;

	STAILQ_INIT(&bf_head);
	
	ATH_HARDSTART_GET_TX_BUF_WITH_LOCK;
	/* FIXME: we are only using a single hardware queue. */
	ath_tx_startraw(sc, bf, skb, control, sc->sc_ac2q[WME_AC_BK]);
	return 0;
		
hardstart_fail:
	return 1;
}

#ifdef AR_DEBUG
static void
ath_keyprint(struct ath_softc *sc, const char *tag, u_int ix,
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
	printk(" mac " MAC_FMT, MAC_ARG(mac));
	if (hk->wk_type == AR5K_CIPHER_TKIP) {
		printk(" %s ", sc->sc_splitmic ? "mic" : "rxmic");
		for (i = 0; i < sizeof(hk->wk_mic); i++)
			printk("%02x", hk->wk_mic[i]);
#if AR5K_ABI_VERSION > 0x06052200
		if (!sc->sc_splitmic) {
			printk(" txmic ");
			for (i = 0; i < sizeof(hk->kv_txmic); i++)
				printk("%02x", hk->kv_txmic[i]);
		}
#endif
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
ath_keyset_tkip(struct ath_softc *sc, struct ieee80211_key_conf *key,
		AR5K_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
	static const u_int8_t zerobssid[IEEE80211_ADDR_LEN];
	struct ath_hal *ah = sc->sc_ah;

	KASSERT(hk->wk_type == AR5K_CIPHER_TKIP,
		("got a non-TKIP key, cipher %u", hk->wk_type));
	if (!(sc->sc_opmode == AR5K_M_STA && key->hw_key_idx < IEEE80211_WEP_NKID)) {
		if (sc->sc_splitmic) {
			/*
			 * TX key goes at first index, RX key at the rx index. 
			 * The hal handles the MIC keys at index+64.
			 */
			memcpy(hk->wk_mic, key->key + 16 /* ALG_TKIP_TEMP_AUTH_TX_MIC_KEY */,
			       8 /* FIXME: define a constant */);
			KEYPRINTF(sc, key->hw_key_idx, hk, zerobssid);
			if (!ath_hal_keyset(ah, key->hw_key_idx, hk, zerobssid))
				return 0;
			memcpy(hk->wk_mic, key->key + 24 /* ALG_TKIP_TEMP_AUTH_RX_MIC_KEY */,
			       8 /* FIXME: define a constant */);
			KEYPRINTF(sc, key->hw_key_idx + 32, hk, mac);
			/* XXX delete tx key on failure? */
			return ath_hal_keyset(ah, key->hw_key_idx + 32, hk, mac);
		} else {
			/*
			 * Room for both TX+RX MIC keys in one key cache
			 * slot, just set key at the first index; the HAL
			 * will handle the reset.
			 */
			memcpy(hk->wk_mic, key->key + 24 /* ALG_TKIP_TEMP_AUTH_RX_MIC_KEY */,
			       8 /* FIXME: define a constant */);
#if AR5K_ABI_VERSION > 0x06052200
			memcpy(hk->kv_txmic, key->key + 16 /* ALG_TKIP_TEMP_AUTH_TX_MIC_KEY */,
			       8 /* FIXME: define a constant */);
#endif
			KEYPRINTF(sc, key->hw_key_idx, hk, mac);
			return ath_hal_keyset(ah, key->hw_key_idx, hk, mac);
		}
	} else {
		/*
		 * RX key goes at first index.
		 * The HAL handles the MIC keys are index+64.
		 */
		memcpy(hk->wk_mic, key->key + 24 /* ALG_TKIP_TEMP_AUTH_RX_MIC_KEY */,
		       8 /* FIXME: define a constant */);
		KEYPRINTF(sc, key->hw_key_idx, hk, mac);
		return ath_hal_keyset(ah, key->hw_key_idx, hk, mac);
	}
	return 0;
}

/*
 * Set a key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP with hardware MIC support.
 */
int
ath_keyset(struct ath_softc *sc, struct ieee80211_key_conf *key,
	const u_int8_t mac[IEEE80211_ADDR_LEN])
{
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
	static const u_int8_t ciphermap[] = {
		AR5K_CIPHER_CLR,		/* ALG_NONE */
		AR5K_CIPHER_WEP,		/* ALG_WEP */
		AR5K_CIPHER_TKIP,	/* ALG_TKIP */
		AR5K_CIPHER_AES_CCM,	/* ALG_CCMP */
		AR5K_CIPHER_CLR,		/* ALG_NULL */
	};
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_hw *hw = sc->sc_hw;
	AR5K_KEYVAL hk;

	memset(&hk, 0, sizeof(hk));
	/*
	 * Software crypto uses a "clear key" so non-crypto
	 * state kept in the key cache are maintained and
	 * so that rx frames have an entry to match.
	 */
	if (key->alg != ALG_NULL) {
		if (key->alg >= N(ciphermap)) {
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: invalid cipher type %u\n", __func__, key->alg);
			return 0;
		}
		hk.wk_type = ciphermap[key->alg];
		if (key->alg == ALG_TKIP)
			hk.wk_len = 16; /* FIXME: define a constant */
		else
			hk.wk_len = key->keylen;
		memcpy(hk.wk_key, key->key, hk.wk_len);
	} else
		hk.wk_type = AR5K_CIPHER_CLR;

	if (hk.wk_type == AR5K_CIPHER_TKIP &&
	    (hw->flags & IEEE80211_HW_TKIP_INCLUDE_MMIC) == 0) {
		return ath_keyset_tkip(sc, key, &hk, mac);
	} else {
		KEYPRINTF(sc, key->hw_key_idx, &hk, mac);
		return ath_hal_keyset(ah, key->hw_key_idx, &hk, mac);
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
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
	u_int i, keyix;

	KASSERT(sc->sc_splitmic, ("key cache !split"));
	/* XXX could optimize */
	for (i = 0; i < N(sc->sc_keymap) / 4; i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots in this byte are free.
			 */
			keyix = i * NBBY;
			while (b & 1) {
		again:
				keyix++;
				b >>= 1;
			}
			/* XXX IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV */
			if (isset(sc->sc_keymap, keyix + 32) ||
			    isset(sc->sc_keymap, keyix + 64) ||
			    isset(sc->sc_keymap, keyix + 32 + 64) ||
			    keyix < IEEE80211_WEP_NKID) {
				/* index unavailable */
				/* XXX statistic */
				if (keyix == (i + 1) * NBBY) {
					/* no slots were appropriate, advance */
					continue;
				}
				goto again;
			}
			setbit(sc->sc_keymap, keyix);
			setbit(sc->sc_keymap, keyix + 64);
			setbit(sc->sc_keymap, keyix + 32);
			setbit(sc->sc_keymap, keyix + 32 + 64);
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: key pair %u,%u %u,%u\n",
				__func__, keyix, keyix + 64,
				keyix + 32, keyix + 32 + 64);
			return keyix;
		}
	}
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
	return IEEE80211_KEYIX_NONE;
#undef N
}

/*
 * Allocate tx/rx key slots for TKIP.  We allocate two slots for
 * each key, one for decrypt/encrypt and the other for the MIC.
 */
static u_int16_t
key_alloc_pair(struct ath_softc *sc)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	u_int i, keyix;

	KASSERT(!sc->sc_splitmic, ("key cache split"));
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
			if (keyix < IEEE80211_WEP_NKID ||
			    isset(sc->sc_keymap, keyix+64)) {
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
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: key pair %u,%u\n",
				__func__, keyix, keyix+64);
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
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
	u_int i, keyix;

	/* XXX try i,i+32,i+64,i+32+64 to minimize key pair conflicts */
	for (i = 0; i < N(sc->sc_keymap); i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots are free.
			 */
			keyix = i * NBBY;
			while (b & 1) {
		again:
				keyix++, b >>= 1;
			}
			if (keyix < IEEE80211_WEP_NKID ||
			    (keyix >= 32 && keyix < (32 + IEEE80211_WEP_NKID)) ||
			    (keyix >= 64 && keyix < (64 + IEEE80211_WEP_NKID)) ||
			    (keyix >= (32 + 64) &&
			     keyix < (32 + 64 + IEEE80211_WEP_NKID))) {
				/* never alloc a default key or default TKIP
				   key pair */
				if (keyix == (i + 1) * NBBY) {
					/* no slots were appropriate, advance */
					continue;
				}
				goto again;
			}
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
 * Allocate one or more key cache slots for a unicast key.  The
 * key itself is needed only to identify the cipher.  For hardware
 * TKIP with split cipher+MIC keys we allocate two key cache slot
 * pairs so that we can setup separate TX and RX MIC keys.  Note
 * that the MIC key for a TKIP key at slot i is assumed by the
 * hardware to be at slot i+64.  This limits TKIP keys to the first
 * 64 entries.
 */
int
ath_key_alloc(struct ath_softc *sc, struct ieee80211_key_conf *key,
	      const u_int8_t addr[IEEE80211_ADDR_LEN])
{
	struct ieee80211_hw *hw = sc->sc_hw;
	u_int keyix;

	if ((key->flags & IEEE80211_KEY_DEFAULT_WEP_ONLY) &&
	    (addr == NULL ||
	     memcmp(addr, "\xff\xff\xff\xff\xff\xff", ETH_ALEN) == 0)) {
		keyix = key->keyidx;

		if (key->alg == ALG_TKIP) {
			setbit(sc->sc_keymap, keyix);
			setbit(sc->sc_keymap, keyix + 64);
			setbit(sc->sc_keymap, keyix + 32);
			setbit(sc->sc_keymap, keyix + 32 + 64);
		} else
			setbit(sc->sc_keymap, keyix);

		return keyix;
	}
	/*
	 * We allocate two pair for TKIP when using the h/w to do
	 * the MIC.  For everything else, including software crypto,
	 * we allocate a single entry.  Note that s/w crypto requires
	 * a pass-through slot on the 5211 and 5212.  The 5210 does
	 * not support pass-through cache entries and we map all
	 * those requests to slot 0.
	 *
	 * Allocate 1 pair of keys for WEP case. Make sure the key
	 * is not a shared-key.
	 */
	if (key->alg == ALG_TKIP &&
	    (hw->flags & IEEE80211_HW_TKIP_INCLUDE_MMIC) == 0) {
		if (sc->sc_splitmic)
			return key_alloc_2pair(sc);
		else
			return key_alloc_pair(sc);
	} else
		return key_alloc_single(sc);
}

/*
 * Delete an entry in the key cache allocated by ath_key_alloc.
 */
int
ath_key_delete(struct ath_softc *sc, struct ieee80211_key_conf *key)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_hw *hw = sc->sc_hw;
	u_int keyix = key->hw_key_idx;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: delete key %u\n", __func__, keyix);

	keyix = key->hw_key_idx;

	if (keyix >= sc->sc_keymax)
	    return 0;
	    
	ath_hal_keyreset(ah, keyix);
	/*
	 * Handle split tx/rx keying required for TKIP with h/w MIC.
	 */
	if (key->alg == ALG_TKIP &&
	    (hw->flags & IEEE80211_HW_TKIP_INCLUDE_MMIC) == 0 &&
	    sc->sc_splitmic) {
		ath_hal_keyreset(ah, keyix + 32);	/* RX key */
	}

	clrbit(sc->sc_keymap, keyix);
	if (key->alg == ALG_TKIP &&
	    (hw->flags & IEEE80211_HW_TKIP_INCLUDE_MMIC) == 0) {
		if ((keyix + 64) < sc->sc_keymax)
			clrbit(sc->sc_keymap, keyix + 64);	/* TX key MIC */
		if (sc->sc_splitmic) {
			/* +32 for RX key, +32+64 for RX key MIC */
			if ((keyix + 32) < sc->sc_keymax)
			    clrbit(sc->sc_keymap, keyix + 32);
			if ((keyix + 32 + 64) < sc->sc_keymax)
			    clrbit(sc->sc_keymap, keyix + 32 + 64);
		}
	}

	return 1;
}

#if 0
/*
 * Block/unblock tx+rx processing while a key change is done.
 * We assume the caller serializes key management operations
 * so we only need to worry about synchronization with other
 * uses that originate in the driver.
 */
static void
ath_key_update_begin(struct ieee80211vap *vap)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
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
ath_key_update_end(struct ieee80211vap *vap)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	netif_start_queue(dev);
	if (!in_softirq())		/* NB: see above */
		tasklet_enable(&sc->sc_rxtq);
}

#endif
/*
 * Calculate the receive filter according to the
 * operating mode and state:
 *
 * o always accept unicast, broadcast, and multicast traffic
 * o maintain current state of phy error reception (the HAL
 *   may enable phy error frames for noise immunity work)
 * o probe request frames are accepted only when operating in
 *   hostap, adhoc, or monitor modes
 * o enable promiscuous mode according to the interface state
 * o accept beacons:
 *   - when operating in adhoc mode so the 802.11 layer creates
 *     node table entries for peers,
 *   - when operating in station mode for collecting rssi data when
 *     the station is otherwise quiet, or
 *   - when operating as a repeater so we see repeater-sta beacons
 *   - when scanning
 */
static u_int32_t
ath_calcrxfilter(struct ath_softc *sc)
{
#define	RX_FILTER_PRESERVE	(AR5K_RX_FILTER_PHYERR | AR5K_RX_FILTER_PHYRADAR)
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	rfilt = (ath_hal_getrxfilter(ah) & RX_FILTER_PRESERVE) |
		 AR5K_RX_FILTER_UCAST | AR5K_RX_FILTER_BCAST |
		 AR5K_RX_FILTER_MCAST;
	if (sc->sc_opmode != AR5K_M_STA)
		rfilt |= AR5K_RX_FILTER_PROBEREQ;
	if (sc->sc_opmode != AR5K_M_HOSTAP)
		rfilt |= AR5K_RX_FILTER_PROM;
	if (sc->sc_opmode == AR5K_M_STA ||
	    sc->sc_opmode == AR5K_M_IBSS ||	/* NB: AHDEMO too */
	    (sc->sc_nostabeacons) || sc->sc_scanning)
		rfilt |= AR5K_RX_FILTER_BEACON;
	if (sc->sc_opmode == AR5K_M_MONITOR)
		rfilt |= (AR5K_RX_FILTER_CONTROL | AR5K_RX_FILTER_BEACON | 
			  AR5K_RX_FILTER_PROBEREQ | AR5K_RX_FILTER_PROM);
	return rfilt;
#undef RX_FILTER_PRESERVE
}

#if 0
/*
 * Merge multicast addresses from all VAPs to form the
 * hardware filter.  Ideally we should only inspect our
 * own list and the 802.11 layer would merge for us but
 * that's a bit difficult so for now we put the onus on
 * the driver.
 */
static void
ath_merge_mcast(struct ath_softc *sc, u_int32_t mfilt[2])
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap;
	struct dev_mc_list *mc;
	u_int32_t val;
	u_int8_t pos;

	mfilt[0] = mfilt[1] = 0;
	/* XXX locking */
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
		struct net_device *dev = vap->iv_dev;
		for (mc = dev->mc_list; mc; mc = mc->next) {
			/* calculate XOR of eight 6-bit values */
			val = LE_READ_4(mc->dmi_addr + 0);
			pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			val = LE_READ_4(mc->dmi_addr + 3);
			pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			pos &= 0x3f;
			mfilt[pos / 32] |= (1 << (pos % 32));
		}
	}
}

#endif
static void
ath_mode_init(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt, mfilt[2];

	/* configure rx filter */
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);

	/* configure bssid mask */
	if (sc->sc_hasbmask)
		ath_hal_setbssidmask(ah, sc->sc_bssidmask);

	/* configure operational mode */
	ath_hal_setopmode(ah);

#if 0
	/* calculate and install multicast filter */
	if ((dev->flags & IFF_ALLMULTI) == 0)
		ath_merge_mcast(sc, mfilt);
	else
		mfilt[0] = mfilt[1] = ~0;
#else
	mfilt[0] = mfilt[1] = 0;
#endif
	ath_hal_setmcastfilter(ah, mfilt[0], mfilt[1]);
	DPRINTF(sc, ATH_DEBUG_STATE,
	     "%s: RX filter 0x%x, MC filter %08x:%08x\n",
	     __func__, rfilt, mfilt[0], mfilt[1]);
}

/*
 * Set the slot time based on the current setting.
 */
static void
ath_setslottime(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	if (sc->sc_slottimeconf > 0) /* manual override */
		ath_hal_setslottime(ah, sc->sc_slottimeconf);
	else if (sc->sc_shortslottime)
		ath_hal_setslottime(ah, AR5K_SLOT_TIME_9);
	else
		ath_hal_setslottime(ah, AR5K_SLOT_TIME_20);
	sc->sc_updateslot = OK;
}

#if 0
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
	else if (dev->flags & IFF_RUNNING)
		ath_setslottime(sc);
}

#ifdef ATH_SUPERG_DYNTURBO
/*
 * Dynamic turbo support.
 * XXX much of this could be moved up to the net80211 layer.
 */

/*
 * Configure dynamic turbo state on beacon setup.
 */
static void
ath_beacon_dturbo_config(struct ieee80211vap *vap, u_int32_t intval)
{
#define	IS_CAPABLE(vap) \
	(vap->iv_bss && (vap->iv_bss->ni_ath_flags & (IEEE80211_ATHC_TURBOP )) == \
		(IEEE80211_ATHC_TURBOP))
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;

	if (ic->ic_opmode == IEEE80211_M_HOSTAP && IS_CAPABLE(vap)) {

		/* Dynamic Turbo is supported on this channel. */
		sc->sc_dturbo = 1;
		sc->sc_dturbo_tcount = 0;
		sc->sc_dturbo_switch = 0;
		sc->sc_ignore_ar = 0;

		/* Set the initial ATHC_BOOST capability. */
		if (ic->ic_bsschan->ic_flags & CHANNEL_TURBO)
			ic->ic_ath_cap |=  IEEE80211_ATHC_BOOST;
		else
			ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;

		/*
		 * Calculate time & bandwidth thresholds
		 *
		 * sc_dturbo_base_tmin  :  ~70 seconds
		 * sc_dturbo_turbo_tmax : ~120 seconds
		 *
		 * NB: scale calculated values to account for staggered
		 *     beacon handling
		 */
		sc->sc_dturbo_base_tmin  = 70  * 1024 / ic->ic_lintval;
		sc->sc_dturbo_turbo_tmax = 120 * 1024 / ic->ic_lintval;
		sc->sc_dturbo_turbo_tmin = 5 * 1024 / ic->ic_lintval;
		/* convert the thresholds from BW/sec to BW/beacon period */
		sc->sc_dturbo_bw_base    = ATH_TURBO_DN_THRESH/(1024/ic->ic_lintval);  
		sc->sc_dturbo_bw_turbo   = ATH_TURBO_UP_THRESH/(1024/ic->ic_lintval); 
		/* time in hold state in number of beacon */
		sc->sc_dturbo_hold_max   = (ATH_TURBO_PERIOD_HOLD * 1024)/ic->ic_lintval;
	} else {
		sc->sc_dturbo = 0;
		ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;
	}
#undef IS_CAPABLE
}

/*
 * Update dynamic turbo state at SWBA.  We assume we care
 * called only if dynamic turbo has been enabled (sc_turbo).
 */
static void
ath_beacon_dturbo_update(struct ieee80211vap *vap, int *needmark,u_int8_t dtim)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	u_int32_t bss_traffic;

	/* TBD: Age out CHANNEL_INTERFERENCE */
	if (sc->sc_ignore_ar) {
		/* 
		 * Ignore AR for this beacon; a dynamic turbo
		 * switch just happened and the information
		 * is invalid.  Notify AR support of the channel
		 * change.
		 */
		sc->sc_ignore_ar = 0;
		ath_hal_ar_enable(sc->sc_ah);
	}
	sc->sc_dturbo_tcount++;
	/*
	 * Calculate BSS traffic over the previous interval.
	 */
	bss_traffic = (sc->sc_devstats.tx_bytes + sc->sc_devstats.rx_bytes)
		    - sc->sc_dturbo_bytes;
	sc->sc_dturbo_bytes = sc->sc_devstats.tx_bytes
			    + sc->sc_devstats.rx_bytes;
	if (ic->ic_ath_cap & IEEE80211_ATHC_BOOST) {
 		/* 
  		* before switching to base mode,
  		* make sure that the conditions( low rssi, low bw) to switch mode 
  		* hold for some time and time in turbo exceeds minimum turbo time.
  		*/
 
		if (sc->sc_dturbo_tcount >= sc->sc_dturbo_turbo_tmin && 
		   sc->sc_dturbo_hold ==0 &&
		   (bss_traffic < sc->sc_dturbo_bw_base || !sc->sc_rate_recn_state)) {
			sc->sc_dturbo_hold = 1;
		} else {
			if (sc->sc_dturbo_hold &&
			   bss_traffic >= sc->sc_dturbo_bw_turbo && sc->sc_rate_recn_state) {
				/* out of hold state */
				sc->sc_dturbo_hold = 0;
				sc->sc_dturbo_hold_count = sc->sc_dturbo_hold_max;
			}
		}
		if (sc->sc_dturbo_hold && sc->sc_dturbo_hold_count)
			sc->sc_dturbo_hold_count--;
		/*
		 * Current Mode: Turbo (i.e. BOOST)
		 *
		 * Transition to base occurs when one of the following
		 * is true:
		 *    1. its a DTIM beacon. 
		 *    2. Maximum time in BOOST has elapsed (120 secs).
		 *    3. Channel is marked with interference
		 *    4. Average BSS traffic falls below 4Mbps 
		 *    5. RSSI cannot support at least 18 Mbps rate 
		 * XXX do bw checks at true beacon interval?
		 */
		if (dtim && 
			(sc->sc_dturbo_tcount >= sc->sc_dturbo_turbo_tmax ||
			 ((vap->iv_bss->ni_ath_flags & IEEE80211_ATHC_AR) && 
			  (sc->sc_curchan.private_flags & CHANNEL_INTERFERENCE) &&
			  IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan)) || 
			 !sc->sc_dturbo_hold_count)) {
			DPRINTF(sc, ATH_DEBUG_TURBO, "%s: Leaving turbo\n",
					sc->sc_dev->name);
			ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;
			vap->iv_bss->ni_ath_flags &= ~IEEE80211_ATHC_BOOST;
			sc->sc_dturbo_tcount = 0;
			sc->sc_dturbo_switch = 1;
		}
	} else {
		/*
		 * Current Mode: BASE
		 *
		 * Transition to Turbo (i.e. BOOST) when all of the
		 * following are true:
		 *
		 * 1. its a DTIM beacon. 
		 * 2. Dwell time at base has exceeded minimum (70 secs)
		 * 3. Only DT-capable stations are associated
		 * 4. Channel is marked interference-free.
		 * 5. BSS data traffic averages at least 6Mbps 
		 * 6. RSSI is good enough to support 36Mbps 
		 * XXX do bw+rssi checks at true beacon interval?
		 */
		if (dtim && 
			(sc->sc_dturbo_tcount >= sc->sc_dturbo_base_tmin &&
			 (ic->ic_dt_sta_assoc != 0 &&
			  ic->ic_sta_assoc == ic->ic_dt_sta_assoc) &&
			 ((vap->iv_bss->ni_ath_flags & IEEE80211_ATHC_AR) == 0 || 
			  (sc->sc_curchan.private_flags & CHANNEL_INTERFERENCE) == 0) &&
			 bss_traffic >= sc->sc_dturbo_bw_turbo && 
			 sc->sc_rate_recn_state)) {
			DPRINTF(sc, ATH_DEBUG_TURBO, "%s: Entering turbo\n",
					sc->sc_dev->name);
			ic->ic_ath_cap |= IEEE80211_ATHC_BOOST;
			vap->iv_bss->ni_ath_flags |= IEEE80211_ATHC_BOOST;
			sc->sc_dturbo_tcount = 0;
			sc->sc_dturbo_switch = 1;
			sc->sc_dturbo_hold = 0;
			sc->sc_dturbo_hold_count = sc->sc_dturbo_hold_max;
		}
	}
}


static int 
ath_check_beacon_done(struct ath_softc *sc)
{
	struct ieee80211vap *vap=NULL;
	struct ath_vap *avp;
	struct ath_buf *bf;
	struct sk_buff *skb;
	struct ath_desc *ds;
	struct ath_hal *ah = sc->sc_ah;
	int slot;

	/*
	 * check if the last beacon went out with the mode change flag set.
	 */
	for (slot = 0; slot < ATH_BCBUF; slot++) {
		if(sc->sc_bslot[slot]) { 
			vap = sc->sc_bslot[slot];
			break;
		}
	}
	if (!vap)
		 return 0;
	avp = ATH_VAP(vap);
	bf = avp->av_bcbuf;
	skb = bf->bf_skb;
	ds = bf->bf_desc;

	return (ath_hal_txprocdesc(ah, ds) != AR5K_EINPROGRESS);

}

/*
 * Effect a turbo mode switch when operating in dynamic
 * turbo mode. wait for beacon to go out before switching.
 */
static void
ath_turbo_switch_mode(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int newflags;

	KASSERT(ic->ic_opmode == IEEE80211_M_HOSTAP,
		("unexpected operating mode %d", ic->ic_opmode));

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: dynamic turbo switch to %s mode\n",
		dev->name,
		ic->ic_ath_cap & IEEE80211_ATHC_BOOST ? "turbo" : "base");

	if (!ath_check_beacon_done(sc)) {
		/* 
		 * beacon did not go out. reschedule tasklet.
		 */
		mod_timer(&sc->sc_dturbo_switch_mode, jiffies + msecs_to_jiffies(2));
		return;
	}

	/* TBD: DTIM adjustments, delay CAB queue tx until after transmit */
	newflags = ic->ic_bsschan->ic_flags;
	if (ic->ic_ath_cap & IEEE80211_ATHC_BOOST) {
		if (IEEE80211_IS_CHAN_2GHZ(ic->ic_bsschan)) {
			/*
			 * Ignore AR next beacon. the AR detection
			 * code detects the traffic in normal channel
			 * from stations during transition delays
			 * between AP and station.
			 */
			sc->sc_ignore_ar = 1;
			ath_hal_ar_disable(sc->sc_ah);
		}
		newflags |= IEEE80211_CHAN_TURBO;
	} else
		newflags &= ~IEEE80211_CHAN_TURBO;
	ieee80211_dturbo_switch(ic, newflags);
	/* XXX ieee80211_reset_erp? */
}
#endif /* ATH_SUPERG_DYNTURBO */
#endif

/*
 * Setup a h/w transmit queue for beacons.
 */
static int
ath_beaconq_setup(struct ath_hal *ah)
{
	AR5K_TXQ_INFO qi;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_aifs = 1;
	qi.tqi_cw_min = 0;
	qi.tqi_cw_max = 0;
#ifdef ATH_SUPERG_DYNTURBO
	qi.tqi_flags = AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
#endif
	/* NB: don't enable any interrupts */
	return ath_hal_setuptxqueue(ah, AR5K_TX_QUEUE_BEACON, &qi);
}

/*
 * Configure IFS parameter for the beacon queue.
 */
static int
ath_beaconq_config(struct ath_softc *sc)
{
#define	ATH_EXPONENT_TO_VALUE(v)	((1<<v)-1)
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;

	ath_hal_gettxqueueprops(ah, sc->sc_bhalq, &qi);
	if (sc->sc_opmode == AR5K_M_HOSTAP) {
		/*
		 * Always burst out beacon and CAB traffic.
		 */
		qi.tqi_aifs = 1;
		qi.tqi_cw_min = 0;
		qi.tqi_cw_max = 0;
	} else {
#if 0
		struct wmeParams *wmep =
			&ic->ic_wme.wme_chanParams.cap_wmeParams[WME_AC_BE];
		/*
		 * Adhoc mode; important thing is to use 2x cwmin.
		 */
		qi.tqi_aifs = wmep->wmep_aifsn;
		qi.tqi_cw_min = 2 * ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmin);
		qi.tqi_cw_max = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmax);
#endif
	}

	if (!ath_hal_settxqueueprops(ah, sc->sc_bhalq, &qi)) {
		printk("%s: unable to update h/w beacon queue parameters\n",
			sc->name);
		return 0;
	} else {
		ath_hal_resettxqueue(ah, sc->sc_bhalq);	/* push to h/w */
		return 1;
	}
#undef ATH_EXPONENT_TO_VALUE
}


/*
 * Setup the beacon frame for transmit.
 */
static void
ath_beacon_setup(struct ath_softc *sc, struct ath_buf *bf,
		 struct ieee80211_tx_control *control)
{
	struct sk_buff *skb = bf->bf_skb;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	int flags;
	int antenna = sc->sc_txantenna;
	const AR5K_RATE_TABLE *rt;
	u_int8_t rix, rate;
	int ctsrate = 0;
	int ctsduration = 0;
	int hdrlen;
	int power;

	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: m %p len %u\n",
		__func__, skb, skb->len);

	/* setup descriptors */
	ds = bf->bf_desc;

	flags = AR5K_TXDESC_NOACK;
#ifdef ATH_SUPERG_DYNTURBO
	if (sc->sc_dturbo_switch)
		flags |= AR5K_TXDESC_INTREQ;
#endif

	if (sc->sc_opmode == AR5K_M_IBSS && sc->sc_hasveol) {
		ds->ds_link = bf->bf_daddr;	/* self-linked */
		flags |= AR5K_TXDESC_VEOL;
		/*
		 * Let hardware handle antenna switching if txantenna is not set
		 */
	} else {
		ds->ds_link = 0;
		/*
		 * Switch antenna every beacon if txantenna is not set
		 * Should only switch every beacon period, not for every
		 * SWBA's
		 * XXX assumes two antenna
		 */
		if (antenna == 0) {
			if (sc->sc_stagbeacons)
				antenna = ((sc->sc_stats.ast_be_xmit / sc->sc_nbcnvaps) & 1 ? 2 : 1);
			else
				antenna = (sc->sc_stats.ast_be_xmit & 1 ? 2 : 1);
		}
	}

	ds->ds_data = bf->bf_skbaddr;
	/*
	 * Calculate rate code.
	 * XXX everything at min xmit rate
	 */
	rix = sc->sc_minrateix;
	rt = sc->sc_currates;
	rate = control->tx_rate;
#ifdef ATH_SUPERG_XR
	if (bf->bf_node->ni_vap->iv_flags & IEEE80211_F_XR) {
		u_int8_t cix;
		int pktlen;
		pktlen = skb->len + IEEE80211_CRC_LEN;
		cix = rt->rates[sc->sc_protrix].controlRate;
		/* for XR VAP use different RTSCTS rates and calculate duration */
		ctsrate = rt->rates[cix].rateCode;
		if (USE_SHPREAMBLE(ic))
			ctsrate |= rt->rates[cix].shortPreamble;
		flags |= AR5K_TXDESC_CTSENA;
		rt = sc->sc_xr_rates;
		ctsduration = ath_hal_computetxtime(ah,rt, pktlen,
			IEEE80211_XR_DEFAULT_RATE_INDEX, FALSE);
		rate = rt->rates[IEEE80211_XR_DEFAULT_RATE_INDEX].rateCode;
	}
#endif

	hdrlen = ieee80211_get_hdrlen_from_skb(skb);
	/* FIXME : what unit does the hal need power is? */
	power = control->power_level > 60 ? 60 : control->power_level;

	ath_hal_setuptxdesc(ah, ds
		, skb->len + IEEE80211_CRC_LEN	/* frame length */
		, hdrlen			/* header length */
		, AR5K_PKT_TYPE_BEACON		/* Atheros packet type */
		, power				/* txpower XXX */
		, rate, 1			/* series 0 rate/tries */
		, AR5K_TXKEYIX_INVALID		/* no encryption */
		, antenna			/* antenna mode */
		, flags				/* no ack, veol for beacons */
		, ctsrate			/* rts/cts rate */
		, ctsduration			/* rts/cts duration */
//		, 0				/* comp icv len */
//		, 0				/* comp iv len */
//		, ATH_COMP_PROC_NO_COMP_NO_CCS	/* comp scheme */
	);

	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	ath_hal_filltxdesc(ah, ds
		, roundup(skb->len, 4)	/* buffer length */
		, TRUE		/* first segment */
		, TRUE		/* last segment */
		, ds			/* first descriptor */
	);

	/* NB: The desc swap function becomes void, 
	 * if descriptor swapping is not enabled
	 */
	ath_desc_swap(ds);
}

/*
 * Generate beacon frame and queue cab data for a bss.
 */
static struct ath_buf *
ath_beacon_generate(struct ath_softc *sc, struct ath_bss *bss)
{
	struct ath_buf *bf;
	struct sk_buff *skb;
	struct ieee80211_tx_control control;
	struct ieee80211_tx_control cab_control;
	struct ath_buf *cab_bf;

#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR) {
		vap->iv_xrbcnwait++;
		/* wait for XR_BEACON_FACTOR times before sending the beacon */
		if (vap->iv_xrbcnwait < IEEE80211_XR_BEACON_FACTOR)
			return NULL;
		vap->iv_xrbcnwait = 0;
	}
#endif
#ifdef ATH_SUPERG_DYNTURBO
	/* 
	 * If we are using dynamic turbo, update the
	 * capability info and arrange for a mode change
	 * if needed.
	 */
	if (sc->sc_dturbo) {
		u_int8_t dtim;
		dtim = ((avp->av_boff.bo_tim[2] == 1) ||
			(avp->av_boff.bo_tim[3] == 1));
		ath_beacon_dturbo_update(vap, needmark, dtim);
	}
#endif
	bf = bss->ab_bcbuf;

	memset(&control, 0, sizeof(control));
	skb = ieee80211_beacon_get(sc->sc_hw, bss->ab_if_id, &control);
	if (!skb) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: failed to get beacon for if_id %d\n", __func__,
		       	bss->ab_if_id);
		return NULL;
	}

	dev_kfree_skb(bf->bf_skb);

	bf->bf_skb = skb;

	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, skb->len, BUS_DMA_TODEVICE);
#if 0
	/*
	 * if the CABQ traffic from previous DTIM is pending and the current
	 * beacon is also a DTIM. 
	 *  1) if there is only one VAP let the cab traffic continue. 
	 *  2) if there are more than one VAP and we are using staggered
	 *     beacons, then drain the cabq by dropping all the frames in
	 *     the cabq so that the current VAP's cab traffic can be scheduled.
	 * XXX: Need to handle the last MORE_DATA bit here.
	 */
	/* FIXME: why drop the packets? */
	if (ncabq && (avp->av_boff.bo_tim[4] & 1) && sc->sc_cabq->axq_depth) {
		if (sc->sc_nvaps > 1 && sc->sc_stagbeacons) {
			ath_tx_draintxq(sc, sc->sc_cabq);
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: flush previous cabq traffic\n", __func__);
		}
	}
#endif

	/*
	 * Construct tx descriptor.
	 */
	ath_beacon_setup(sc, bf, &control);

	bus_dma_sync_single(sc->sc_bdev,
		bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);

	/*
	 * Enable the CAB queue before the beacon queue to
	 * ensure cab frames are triggered by this beacon.
	 */
	skb = ieee80211_get_buffered_bc(sc->sc_hw, bss->ab_if_id, &cab_control);

	while (skb) {

		cab_bf = ath_get_tx_buf(sc);
		if (!cab_bf) {
			kfree_skb(skb);
			break;
		}
		/* NB: gated by beacon so safe to start here */
		ath_tx_startraw(sc, cab_bf, skb, &cab_control, sc->sc_cabq);

		skb = ieee80211_get_buffered_bc(sc->sc_hw, bss->ab_if_id, &cab_control);
	}

	return bf;
}

/*
 * Transmit one or more beacon frames at SWBA.  Dynamic
 * updates to the frame contents are done as needed and
 * the slot time is also adjusted based on current state.
 */
static void
ath_beacon_send(struct ath_softc *sc, int *needmark)
{
#define	TSF_TO_TU(_h,_l) \
	((((u_int32_t)(_h)) << 22) | (((u_int32_t)(_l)) >> 10))
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	int slot;

	u_int32_t bfaddr;

	/*
	 * Check if the previous beacon has gone out.  If
	 * not don't try to post another, skip this period
	 * and wait for the next.  Missed beacons indicate
	 * a problem and should not occur.  If we miss too
	 * many consecutive beacons reset the device.
	 */
	if (ath_hal_numtxpending(ah, sc->sc_bhalq) != 0) {
		sc->sc_bmisscount++;
		/* XXX: 802.11h needs the chanchange IE countdown decremented.
		 *      We should consider adding a net80211 call to indicate
		 *      a beacon miss so appropriate action could be taken
		 *      (in that layer).
		 */
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: missed %u consecutive beacons\n",
			__func__, sc->sc_bmisscount);
		if (sc->sc_bmisscount > BSTUCK_THRESH)
			ATH_SCHEDULE_TQUEUE(&sc->sc_bstucktq, needmark);
		return;
	}
	if (sc->sc_bmisscount != 0) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: resume beacon xmit after %u misses\n",
			__func__, sc->sc_bmisscount);
		sc->sc_bmisscount = 0;
	}

	{
		u_int32_t *bflink;

		bflink = &bfaddr;

		spin_lock(&sc->sc_bss_lock);

		for (slot = 0; slot < sc->sc_num_bss; slot++) {

			bf = ath_beacon_generate(sc, &sc->sc_bss[slot]);

			if (bf != NULL) {
#ifdef AH_NEED_DESC_SWAP
				if (bflink != &bfaddr)
					*bflink = cpu_to_le32(bf->bf_daddr);
				else
					*bflink = bf->bf_daddr;
#else
				*bflink = bf->bf_daddr;
#endif
				bflink = &bf->bf_desc->ds_link;
			}
		}
		*bflink = 0;			/* link of last frame */
		spin_unlock(&sc->sc_bss_lock);
	}

	/*
	 * Handle slot time change when a non-ERP station joins/leaves
	 * an 11g network.  The 802.11 layer notifies us via callback,
	 * we mark updateslot, then wait one beacon before effecting
	 * the change.  This gives associated stations at least one
	 * beacon interval to note the state change.
	 *
	 * NB: The slot time change state machine is clocked according
	 *     to whether we are bursting or staggering beacons.  We
	 *     recognize the request to update and record the current
	 *     slot then don't transition until that slot is reached
	 *     again.  If we miss a beacon for that slot then we'll be
	 *     slow to transition but we'll be sure at least one beacon
	 *     interval has passed.  When bursting slot is always left
	 *     set to sc_num_bss so this check is a no-op.
	 */
	/* XXX locking */
	if (sc->sc_updateslot == UPDATE) {
		sc->sc_updateslot = COMMIT;	/* commit next beacon */
		sc->sc_slotupdate = slot;
	} else if (sc->sc_updateslot == COMMIT && sc->sc_slotupdate == slot)
		ath_setslottime(sc);		/* commit change to hardware */

#if 0
	if ((!sc->sc_stagbeacons || slot == 0) && (!sc->sc_diversity)) {
		int otherant;
		/*
		 * Check recent per-antenna transmit statistics and flip
		 * the default rx antenna if noticeably more frames went out
		 * on the non-default antenna.  Only do this if rx diversity
		 * is off.
		 * XXX assumes 2 antennae
		 */
		otherant = sc->sc_defant & 1 ? 2 : 1;
		if (sc->sc_ant_tx[otherant] > sc->sc_ant_tx[sc->sc_defant] + ATH_ANTENNA_DIFF) {
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: flip defant to %u, %u > %u\n",
				__func__, otherant, sc->sc_ant_tx[otherant],
				sc->sc_ant_tx[sc->sc_defant]);
			ath_setdefantenna(sc, otherant);
		}
		sc->sc_ant_tx[1] = sc->sc_ant_tx[2] = 0;
	}
#endif

	if (bfaddr != 0) {
		/*
		 * Stop any current DMA and put the new frame(s) on the queue.
		 * This should never fail since we check above that no frames
		 * are still pending on the queue.
		 */
		if (!ath_hal_stoptxdma(ah, sc->sc_bhalq)) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: beacon queue %u did not stop?\n",
				__func__, sc->sc_bhalq);
			/* NB: the HAL still stops DMA, so proceed */
		}
		/* NB: cabq traffic should already be queued and primed */
		ath_hal_puttxbuf(ah, sc->sc_bhalq, bfaddr);
		ath_hal_txstart(ah, sc->sc_bhalq);

		sc->sc_stats.ast_be_xmit++;		/* XXX per-VAP? */
	}
#undef TSF_TO_TU
}

/*
 * Reset the hardware after detecting beacons have stopped.
 */
static void
ath_bstuck_tasklet(TQUEUE_ARG data)
{
	struct ath_softc *sc = (struct ath_softc *)data;
	/*
	 * XXX:if the bmisscount is cleared while the 
	 *     tasklet execution is pending, the following
	 *     check will be true, in which case return 
	 *     without resetting the driver.
	 */
	if (sc->sc_bmisscount <= BSTUCK_THRESH) 
		return;
	printk("%s: stuck beacon; resetting (bmiss count %u)\n",
		sc->name, sc->sc_bmisscount);
	ath_reset(sc);
}

static void
ath_beacon_tasklet(TQUEUE_ARG data)
{
	struct ath_softc *sc = (struct ath_softc *)data;
	int needmark;

	ath_beacon_send(sc, &needmark);
}

#if 0
/*
 * Startup beacon transmission for adhoc mode when
 * they are sent entirely by the hardware using the
 * self-linked descriptor + veol trick.
 */
static void
ath_beacon_start_adhoc(struct ath_softc *sc, struct ieee80211vap *vap)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ieee80211_node *ni;
	struct ath_vap *avp;
	struct sk_buff *skb;

	avp = ATH_VAP(vap);
	if (avp->av_bcbuf == NULL) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: avp=%p av_bcbuf=%p\n",
			 __func__, avp, avp != NULL ? avp->av_bcbuf : NULL);
		return;
	}
	bf = avp->av_bcbuf;
	ni = bf->bf_node;

	/*
	 * Update dynamic beacon contents.  If this returns
	 * non-zero then we need to remap the memory because
	 * the beacon frame changed size (probably because
	 * of the TIM bitmap).
	 */
	skb = bf->bf_skb;
	if (ieee80211_beacon_update(ni, &avp->av_boff, skb, 0)) {
		bus_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);
		bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
			skb->data, skb->len, BUS_DMA_TODEVICE);
	}

	/*
	 * Construct tx descriptor.
	 */
	ath_beacon_setup(sc, bf);

	bus_dma_sync_single(sc->sc_bdev,
		bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);

	/* NB: caller is known to have already stopped tx DMA */
	ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
	ath_hal_txstart(ah, sc->sc_bhalq);
	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: TXDP%u = %llx (%p)\n", __func__,
		sc->sc_bhalq, ito64(bf->bf_daddr), bf->bf_desc);
}
#endif

#if 0
/*
 * Reclaim beacon resources and return buffer to the pool.
 */
static void
ath_beacon_return(struct ath_softc *sc, struct ath_buf *bf)
{
	if (bf->bf_skb != NULL) {
		bus_unmap_single(sc->sc_bdev,
		    bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
	}
	if (bf->bf_node != NULL) {
		ieee80211_free_node(bf->bf_node);
		bf->bf_node = NULL;
	}
	STAILQ_INSERT_TAIL(&sc->sc_bbuf, bf, bf_list);
}
#endif

/*
 * Reclaim all beacon resources.
 */
static void
ath_beacon_free(struct ath_softc *sc)
{
	struct ath_buf *bf;

	STAILQ_FOREACH(bf, &sc->sc_bbuf, bf_list) {
		if (bf->bf_skb != NULL) {
			bus_unmap_single(sc->sc_bdev,
				bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
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
 * will wake up in time to receive beacons, and configures
 * the beacon miss handling so we'll receive a BMISS
 * interrupt when we stop seeing beacons from the AP
 * we've associated with.
 */
static void
#if 0
ath_beacon_config(struct ath_softc *sc, struct ieee80211vap *vap)
#else
ath_beacon_config(struct ath_softc *sc)
#endif
{
#if 0
#define	TSF_TO_TU(_h,_l) \
	((((u_int32_t)(_h)) << 22) | (((u_int32_t)(_l)) >> 10))
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni;
	u_int32_t nexttbtt, intval;

	if (vap == NULL)
		vap = TAILQ_FIRST(&ic->ic_vaps);   /* XXX */

	ni = vap->iv_bss;

	/* extract tstamp from last beacon and convert to TU */
	nexttbtt = TSF_TO_TU(LE_READ_4(ni->ni_tstamp.data + 4),
			     LE_READ_4(ni->ni_tstamp.data));
	/* XXX conditionalize multi-bss support? */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/*
		 * For multi-bss ap support beacons are either staggered
		 * evenly over N slots or burst together.  For the former
		 * arrange for the SWBA to be delivered for each slot.
		 * Slots that are not occupied will generate nothing. 
		 */
		/* NB: the beacon interval is kept internally in TU's */
		intval = ic->ic_lintval & AR5K_BEACON_PERIOD;
		if (sc->sc_stagbeacons)
			intval /= ATH_BCBUF;	/* for staggered beacons */
		if ((sc->sc_nostabeacons) &&
		    (vap->iv_opmode == IEEE80211_M_HOSTAP))
			nexttbtt = 0;
	} else
		intval = ni->ni_intval & AR5K_BEACON_PERIOD;
	if (nexttbtt == 0)		/* e.g. for ap mode */
		nexttbtt = intval;
	else if (intval)		/* NB: can be 0 for monitor mode */
		nexttbtt = roundup(nexttbtt, intval);
	DPRINTF(sc, ATH_DEBUG_BEACON, "%s: nexttbtt %u intval %u (%u)\n",
		__func__, nexttbtt, intval, ni->ni_intval);
	if (ic->ic_opmode == IEEE80211_M_STA &&	!(sc->sc_nostabeacons)) {
		AR5K_BEACON_STATE bs;
		u_int64_t tsf;
		u_int32_t tsftu;
		int dtimperiod, dtimcount;
		int cfpperiod, cfpcount;

		/*
		 * Setup dtim and cfp parameters according to
		 * last beacon we received (which may be none).
		 */
		dtimperiod = vap->iv_dtim_period;
		if (dtimperiod <= 0)		/* NB: 0 if not known */
			dtimperiod = 1;
		dtimcount = vap->iv_dtim_count;
		if (dtimcount >= dtimperiod)	/* NB: sanity check */
			dtimcount = 0;		/* XXX? */
		cfpperiod = 1;			/* NB: no PCF support yet */
		cfpcount = 0;
#define	FUDGE	2
		/*
		 * Pull nexttbtt forward to reflect the current
		 * TSF and calculate dtim+cfp state for the result.
		 */
		tsf = ath_hal_gettsf64(ah);
		tsftu = TSF_TO_TU(tsf>>32, tsf) + FUDGE;
		do {
			nexttbtt += intval;
			if (--dtimcount < 0) {
				dtimcount = dtimperiod - 1;
				if (--cfpcount < 0)
					cfpcount = cfpperiod - 1;
			}
		} while (nexttbtt < tsftu);
#undef FUDGE
		memset(&bs, 0, sizeof(bs));
		bs.bs_intval = intval;
		bs.bs_nexttbtt = nexttbtt;
		bs.bs_dtimperiod = dtimperiod * intval;
		bs.bs_nextdtim = bs.bs_nexttbtt + dtimcount * intval;
		bs.bs_cfpperiod = cfpperiod * bs.bs_dtimperiod;
		bs.bs_cfpnext = bs.bs_nextdtim + cfpcount * bs.bs_dtimperiod;
		bs.bs_cfpmaxduration = 0;
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
		bs.bs_timoffset = ni->ni_timoff;
#endif
		/*
		 * Calculate the number of consecutive beacons to miss
		 * before taking a BMISS interrupt.  The configuration
		 * is specified in TU so we only need calculate based
		 * on the beacon interval.  Note that we clamp the
		 * result to at most 10 beacons.
		 */
		bs.bs_bmissthreshold = howmany(ic->ic_bmisstimeout, intval);
		if (bs.bs_bmissthreshold > 10)
			bs.bs_bmissthreshold = 10;
		else if (bs.bs_bmissthreshold <= 0)
			bs.bs_bmissthreshold = 1;

		/*
		 * Calculate sleep duration.  The configuration is
		 * given in ms.  We ensure a multiple of the beacon
		 * period is used.  Also, if the sleep duration is
		 * greater than the DTIM period then it makes senses
		 * to make it a multiple of that.
		 *
		 * XXX fixed at 100ms
		 */
		bs.bs_sleepduration =
			roundup(IEEE80211_MS_TO_TU(100), bs.bs_intval);
		if (bs.bs_sleepduration > bs.bs_dtimperiod)
			bs.bs_sleepduration = roundup(bs.bs_sleepduration, bs.bs_dtimperiod);

		DPRINTF(sc, ATH_DEBUG_BEACON, 
			"%s: tsf %llu tsf:tu %u intval %u nexttbtt %u dtim %u nextdtim %u bmiss %u sleep %u cfp:period %u maxdur %u next %u timoffset %u\n"
			, __func__
			, (long long) tsf, tsftu
			, bs.bs_intval
			, bs.bs_nexttbtt
			, bs.bs_dtimperiod
			, bs.bs_nextdtim
			, bs.bs_bmissthreshold
			, bs.bs_sleepduration
			, bs.bs_cfpperiod
			, bs.bs_cfpmaxduration
			, bs.bs_cfpnext
			, bs.bs_timoffset
		);

		ath_hal_intrset(ah, 0);
		ath_hal_beacontimers(ah, &bs);
		sc->sc_imask |= AR5K_INT_BMISS;
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
		ath_hal_intrset(ah, 0);
		if (nexttbtt == intval)
			intval |= AR5K_BEACON_RESET_TSF;
		if (ic->ic_opmode == IEEE80211_M_IBSS) {
			/*
			 * In IBSS mode enable the beacon timers but only
			 * enable SWBA interrupts if we need to manually
			 * prepare beacon frames.  Otherwise we use a
			 * self-linked tx descriptor and let the hardware
			 * deal with things.
			 */
			intval |= AR5K_BEACON_ENA;
			if (!sc->sc_hasveol)
				sc->sc_imask |= AR5K_INT_SWBA;
			ath_beaconq_config(sc);
		} else if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			/*
			 * In AP mode we enable the beacon timers and
			 * SWBA interrupts to prepare beacon frames.
			 */
			intval |= AR5K_BEACON_ENA;
			sc->sc_imask |= AR5K_INT_SWBA;	/* beacon prepare */
			ath_beaconq_config(sc);
		}
#ifdef ATH_SUPERG_DYNTURBO
		ath_beacon_dturbo_config(vap, intval & 
				~(AR5K_BEACON_RESET_TSF | AR5K_BEACON_ENA));
#endif
		ath_hal_beaconinit(ah, nexttbtt, intval);
		sc->sc_bmisscount = 0;
		ath_hal_intrset(ah, sc->sc_imask);
		/*
		 * When using a self-linked beacon descriptor in
		 * ibss mode load it once here.
		 */
		if (ic->ic_opmode == IEEE80211_M_IBSS && sc->sc_hasveol)
			ath_beacon_start_adhoc(sc, vap);
	}
	sc->sc_syncbeacon = 0;
#undef TSF_TO_TU
#else
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t nexttbtt, intval;

	ath_hal_intrset(ah, 0);
	intval = sc->sc_beacon_interval;

	nexttbtt = intval; /* for ap mode */

	intval |= AR5K_BEACON_RESET_TSF;

	/*
	 * In AP mode we enable the beacon timers and
	 * SWBA interrupts to prepare beacon frames.
	 */
	intval |= AR5K_BEACON_ENA;
	sc->sc_imask |= AR5K_INT_SWBA;	/* beacon prepare */
	ath_beaconq_config(sc);
	ath_hal_beaconinit(ah, nexttbtt, intval);
	sc->sc_bmisscount = 0;
	ath_hal_intrset(ah, sc->sc_imask);

#endif
}

int
ath_descdma_setup(struct ath_softc *sc,
	struct ath_descdma *dd, ath_bufhead *head,
	const char *name, int nbuf, int ndesc)
{
#define	DS2PHYS(_dd, _ds) \
	((_dd)->dd_desc_paddr + ((caddr_t)(_ds) - (caddr_t)(_dd)->dd_desc))
	struct ath_desc *ds;
	struct ath_buf *bf;
	int i, bsize, error;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %s DMA: %u buffers %u desc/buf\n",
		__func__, name, nbuf, ndesc);

	dd->dd_name = name;
	dd->dd_desc_len = sizeof(struct ath_desc) * nbuf * ndesc;

	/* allocate descriptors */
	dd->dd_desc = bus_alloc_consistent(sc->sc_bdev,
		dd->dd_desc_len, &dd->dd_desc_paddr);
	if (dd->dd_desc == NULL) {
		error = -ENOMEM;
		goto fail;
	}
	ds = dd->dd_desc;
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %s DMA map: %p (%lu) -> %llx (%lu)\n",
		__func__, dd->dd_name, ds, (u_long) dd->dd_desc_len,
		ito64(dd->dd_desc_paddr), /*XXX*/ (u_long) dd->dd_desc_len);

	/* allocate buffers */
	bsize = sizeof(struct ath_buf) * nbuf;
	bf = kmalloc(bsize, GFP_KERNEL);
	if (bf == NULL) {
		error = -ENOMEM;		/* XXX different code */
		goto fail2;
	}
	memset(bf, 0, bsize);
	dd->dd_bufptr = bf;

	STAILQ_INIT(head);
	for (i = 0; i < nbuf; i++, bf++, ds += ndesc) {
		bf->bf_desc = ds;
		bf->bf_daddr = DS2PHYS(dd, ds);
		STAILQ_INSERT_TAIL(head, bf, bf_list);
	}
	return 0;
fail2:
	bus_free_consistent(sc->sc_bdev, dd->dd_desc_len,
		dd->dd_desc, dd->dd_desc_paddr);
fail:
	memset(dd, 0, sizeof(*dd));
	return error;
#undef DS2PHYS
}

void
ath_descdma_cleanup(struct ath_softc *sc,
	struct ath_descdma *dd, ath_bufhead *head, int dir)
{
	struct ath_buf *bf;

	STAILQ_FOREACH(bf, head, bf_list) {
		if (bf->bf_skb != NULL) {
			/* XXX skb->len is not good enough for rxbuf */
			if (dd == &sc->sc_rxdma)
				bus_unmap_single(sc->sc_bdev,
					bf->bf_skbaddr, sc->sc_rxbufsize, dir);
			else
				bus_unmap_single(sc->sc_bdev,
					bf->bf_skbaddr, bf->bf_skb->len, dir);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
		}
	}

	/* Free memory associated with descriptors */
	bus_free_consistent(sc->sc_bdev, dd->dd_desc_len,
		dd->dd_desc, dd->dd_desc_paddr);

	STAILQ_INIT(head);
	kfree(dd->dd_bufptr);
	memset(dd, 0, sizeof(*dd));
}

static int
ath_desc_alloc(struct ath_softc *sc)
{
	int error;

	error = ath_descdma_setup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			"rx", ATH_RXBUF, 1);
	if (error != 0)
		return error;

	error = ath_descdma_setup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			"tx", ATH_TXBUF, ATH_TXDESC);
	if (error != 0) {
		ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			BUS_DMA_FROMDEVICE);
		return error;
	}

	if (error != 0) {
		ath_descdma_cleanup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			BUS_DMA_TODEVICE);
		ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			BUS_DMA_FROMDEVICE);
		return error;
	}
	return 0;
}

static void
ath_desc_free(struct ath_softc *sc)
{
	if (sc->sc_bdma.dd_desc_len != 0)
		ath_descdma_cleanup(sc, &sc->sc_bdma, &sc->sc_bbuf,
			BUS_DMA_TODEVICE);
	if (sc->sc_txdma.dd_desc_len != 0)
		ath_descdma_cleanup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			BUS_DMA_TODEVICE);
	if (sc->sc_rxdma.dd_desc_len != 0)
		ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			BUS_DMA_FROMDEVICE);
}

#if 0
static struct ieee80211_node *
ath_node_alloc(struct ieee80211_node_table *nt,struct ieee80211vap *vap)
{
	struct ath_softc *sc = nt->nt_ic->ic_dev->priv;
	const size_t space = sizeof(struct ath_node) + sc->sc_rc->arc_space;
	struct ath_node *an;

	an = kmalloc(space, GFP_ATOMIC);
	if (an == NULL)
		return NULL;
	memset(an, 0, space);
	an->an_decomp_index = INVALID_DECOMP_INDEX;
	an->an_avgrssi = ATH_RSSI_DUMMY_MARKER;
	an->an_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
	an->an_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
	an->an_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
	/*
	 * ath_rate_node_init needs a VAP pointer in node
	 * to decide which mgt rate to use
	 */
	an->an_node.ni_vap = vap;
	ath_rate_node_init(sc, an);

	/* U-APSD init */
	STAILQ_INIT(&an->an_uapsd_q);
	an->an_uapsd_qdepth = 0;
	STAILQ_INIT(&an->an_uapsd_overflowq);
	an->an_uapsd_overflowqdepth = 0;
	ATH_NODE_UAPSD_LOCK_INIT(an);

	DPRINTF(sc, ATH_DEBUG_NODE, "%s: an %p\n", __func__, an);
	return &an->an_node;
}

static void
ath_node_cleanup(struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;
	struct ath_node *an = ATH_NODE(ni);
	struct ath_buf *bf;
	
	/*
	 * U-APSD cleanup
	 */
	ATH_NODE_UAPSD_LOCK_IRQ(an);
	if (ni->ni_flags & IEEE80211_NODE_UAPSD_TRIG) {
		ni->ni_flags &= ~IEEE80211_NODE_UAPSD_TRIG;
		ic->ic_uapsdmaxtriggers--;
		ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
	}
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);
	while (an->an_uapsd_qdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_q);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
		bf->bf_desc->ds_link = 0;

		dev_kfree_skb_any(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
		ATH_TXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK_IRQ(sc);
		ieee80211_free_node(ni);

		an->an_uapsd_qdepth--;
	}

	while (an->an_uapsd_overflowqdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		bf->bf_desc->ds_link = 0;

		dev_kfree_skb_any(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;
		ATH_TXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK_IRQ(sc);
		ieee80211_free_node(ni);

		an->an_uapsd_overflowqdepth--;
	}

	ATH_NODE_UAPSD_LOCK_IRQ(an);
	sc->sc_node_cleanup(ni);
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);
}

static void
ath_node_free(struct ieee80211_node *ni)
{
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;

	ath_rate_node_cleanup(sc, ATH_NODE(ni));
	sc->sc_node_free(ni);
#ifdef ATH_SUPERG_XR
	ath_grppoll_period_update(sc);
#endif
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


#ifdef ATH_SUPERG_XR
/*
 * Stops the txqs and moves data between XR and Normal queues.
 * Also adjusts the rate info in the descriptors.
 */

static u_int8_t
ath_node_move_data(const struct ieee80211_node *ni)
{
#ifdef NOT_YET
	struct ath_txq *txq = NULL; 
	struct ieee80211com *ic = ni->ni_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_buf *bf, *prev, *bf_tmp, *bf_tmp1;
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb = NULL;
	struct ath_desc *ds;
	AR5K_STATUS status;
	int index;

	if (ni->ni_vap->iv_flags & IEEE80211_F_XR) {
		struct ath_txq tmp_q; 
		memset(&tmp_q, 0, sizeof(tmp_q));
		STAILQ_INIT(&tmp_q.axq_q);
		/*
		 * move data from Normal txqs to XR queue.
		 */
		printk("move data from NORMAL to XR\n");
		/*
		 * collect all the data towards the node
		 * in to the tmp_q.
		 */
		index = WME_AC_VO;
		while (index >= WME_AC_BE && txq != sc->sc_ac2q[index]) { 
			txq = sc->sc_ac2q[index]; 
			ATH_TXQ_LOCK(txq);
			ath_hal_stoptxdma(ah, txq->axq_qnum);
			bf = prev = STAILQ_FIRST(&txq->axq_q);
			/*
			 * skip all the buffers that are done
			 * until the first one that is in progress
			 */
			while (bf) {
#ifdef ATH_SUPERG_FF
				ds = &bf->bf_desc[bf->bf_numdesc - 1];
#else
				ds = bf->bf_desc;		/* NB: last descriptor */
#endif
				status = ath_hal_txprocdesc(ah, ds);
				if (status == AR5K_EINPROGRESS)
					break; 
				prev = bf;
				bf = STAILQ_NEXT(bf,bf_list);
			}
			/* 
			 * save the pointer to the last buf that's
			 * done
			 */
			if (prev == bf)
				bf_tmp = NULL;  
			else
				bf_tmp=prev;
			while (bf) {
				if (ni == bf->bf_node) {
					if (prev == bf) {
						ATH_TXQ_REMOVE_HEAD(txq, bf_list);
						STAILQ_INSERT_TAIL(&tmp_q.axq_q, bf, bf_list);
						bf = STAILQ_FIRST(&txq->axq_q);
						prev = bf;
					} else {
						STAILQ_REMOVE_AFTER(&(txq->axq_q), prev, bf_list);
						txq->axq_depth--;
						STAILQ_INSERT_TAIL(&tmp_q.axq_q, bf, bf_list);
						bf = STAILQ_NEXT(prev, bf_list);
						/*
						 * after deleting the node
						 * link the descriptors
						 */
#ifdef ATH_SUPERG_FF
						ds = &prev->bf_desc[prev->bf_numdesc - 1];
#else
						ds = prev->bf_desc;	/* NB: last descriptor */
#endif
#ifdef AH_NEED_DESC_SWAP
						ds->ds_link = cpu_to_le32(bf->bf_daddr);
#else
						ds->ds_link = bf->bf_daddr;
#endif
					}
				} else {
					prev = bf;
					bf = STAILQ_NEXT(bf, bf_list);
				}
			}
			/*
			 * if the last buf was deleted.
			 * set the pointer to the last descriptor.
			 */
			bf = STAILQ_FIRST(&txq->axq_q);
			if (bf) {
				if (prev) {
					bf = STAILQ_NEXT(prev, bf_list);
					if (!bf) { /* prev is the last one on the list */
#ifdef ATH_SUPERG_FF
						ds = &prev->bf_desc[prev->bf_numdesc - 1];
#else
						ds = prev->bf_desc;	/* NB: last descriptor */
#endif
						status = ath_hal_txprocdesc(ah, ds);
						if (status == AR5K_EINPROGRESS) 
							txq->axq_link = &ds->ds_link;
						else
							txq->axq_link = NULL;	
					}
				} 
			} else
				txq->axq_link = NULL;

			ATH_TXQ_UNLOCK(txq);
			/*
			 * restart the DMA from the first 
			 * buffer that was not DMA'd.
			 */
			if (bf_tmp)
				bf = STAILQ_NEXT(bf_tmp, bf_list);
			else
				bf = STAILQ_FIRST(&txq->axq_q);
			if (bf) {	
				ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
				ath_hal_txstart(ah, txq->axq_qnum);
			}
		}
		/* 
		 * queue them on to the XR txqueue. 
		 * can not directly put them on to the XR txq. since the
		 * skb data size may be greater than the XR fragmentation
		 * threshold size.
		 */
		bf  = STAILQ_FIRST(&tmp_q.axq_q);
		index = 0;
		while (bf) {
			skb = bf->bf_skb;
			bf->bf_skb = NULL;
			bf->bf_node = NULL;
			ATH_TXBUF_LOCK(sc);
			STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
			ATH_TXBUF_UNLOCK(sc);
			ath_hardstart(skb,sc->sc_dev);
			ATH_TXQ_REMOVE_HEAD(&tmp_q, bf_list);
			bf = STAILQ_FIRST(&tmp_q.axq_q);
			index++;
		}
		printk("moved %d buffers from NORMAL to XR\n", index);
	} else {
		struct ath_txq wme_tmp_qs[WME_AC_VO+1]; 
		struct ath_txq *wmeq = NULL, *prevq; 
		struct ieee80211_frame *wh;
		struct ath_desc *ds = NULL;
		int count = 0;

		/*
		 * move data from XR txq to Normal txqs.
		 */
		printk("move buffers from XR to NORMAL\n");
		memset(&wme_tmp_qs, 0, sizeof(wme_tmp_qs));
		for (index = 0; index <= WME_AC_VO; index++)
			STAILQ_INIT(&wme_tmp_qs[index].axq_q);
		txq = sc->sc_xrtxq; 
		ATH_TXQ_LOCK(txq);
		ath_hal_stoptxdma(ah, txq->axq_qnum);
		bf = prev = STAILQ_FIRST(&txq->axq_q);
		/*
		 * skip all the buffers that are done
		 * until the first one that is in progress
		 */
		while (bf) {
#ifdef ATH_SUPERG_FF
			ds = &bf->bf_desc[bf->bf_numdesc - 1];
#else
			ds = bf->bf_desc;		/* NB: last descriptor */
#endif
			status = ath_hal_txprocdesc(ah, ds);
			if (status == AR5K_EINPROGRESS)
				break;
			prev= bf;
			bf = STAILQ_NEXT(bf,bf_list);
		}
		/* 
		 * save the pointer to the last buf that's
		 * done
		 */
		if (prev == bf)
			bf_tmp1 = NULL;  
		else
			bf_tmp1 = prev;
		/*
		 * collect all the data in to four temp SW queues.
		 */
		while (bf) {
			if (ni == bf->bf_node) {
				if (prev == bf) {
					STAILQ_REMOVE_HEAD(&txq->axq_q,bf_list);
					bf_tmp=bf;
					bf = STAILQ_FIRST(&txq->axq_q);
					prev = bf;
				} else {
					STAILQ_REMOVE_AFTER(&(txq->axq_q),prev,bf_list);
					bf_tmp=bf;
					bf = STAILQ_NEXT(prev,bf_list);
				}
				count++;
				skb = bf_tmp->bf_skb;
				wh = (struct ieee80211_frame *) skb->data;
				if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
					/* XXX validate skb->priority, remove mask */
					wmeq = &wme_tmp_qs[skb->priority & 0x3];
				} else
					wmeq = &wme_tmp_qs[WME_AC_BE];
				STAILQ_INSERT_TAIL(&wmeq->axq_q, bf_tmp, bf_list);
				ds = bf_tmp->bf_desc;
				/* 
				 * link the descriptors
				 */
				if (wmeq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
					*wmeq->axq_link = cpu_to_le32(bf_tmp->bf_daddr);
#else
					*wmeq->axq_link = bf_tmp->bf_daddr;
#endif
					DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u](%p)=%p (%p)\n",
							__func__,
							wmeq->axq_qnum, wmeq->axq_link,
							(caddr_t)bf_tmp->bf_daddr, bf_tmp->bf_desc);
				}
				wmeq->axq_link = &ds->ds_link;
				/* 
				 * update the rate information  
				 */
			} else {
				prev = bf;
				bf = STAILQ_NEXT(bf, bf_list);
			}
		}
		/*
		 * reset the axq_link pointer to the last descriptor.
		 */
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf) {
			if (prev) {
				bf = STAILQ_NEXT(prev, bf_list);
				if (!bf) { /* prev is the last one on the list */
#ifdef ATH_SUPERG_FF
					ds = &prev->bf_desc[prev->bf_numdesc - 1];
#else
					ds = prev->bf_desc;	/* NB: last descriptor */
#endif
					status = ath_hal_txprocdesc(ah, ds);
					if (status == AR5K_EINPROGRESS) 
						txq->axq_link = &ds->ds_link;
					else
						txq->axq_link = NULL;
				} 
			} 
		} else {
			/*
			 * if the list is empty reset the pointer.
			 */
			txq->axq_link = NULL;
		}
		ATH_TXQ_UNLOCK(txq);
		/*
		 * restart the DMA from the first 
		 * buffer that was not DMA'd.
		 */
		if (bf_tmp1)
			bf = STAILQ_NEXT(bf_tmp1,bf_list);
		else
			bf = STAILQ_FIRST(&txq->axq_q);
		if (bf) {	
			ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
			ath_hal_txstart(ah, txq->axq_qnum);
		}

		/* 
		 * move (concant) the lists from the temp sw queues in to
		 * WME queues.
		 */
		index = WME_AC_VO;
		txq = NULL;
		while (index >= WME_AC_BE ) { 
			prevq = txq;
			txq = sc->sc_ac2q[index];
			if (txq != prevq) {
				ATH_TXQ_LOCK(txq);
				ath_hal_stoptxdma(ah, txq->axq_qnum);
			}
			
			wmeq = &wme_tmp_qs[index];
			bf = STAILQ_FIRST(&wmeq->axq_q);
			if (bf) {
				ATH_TXQ_MOVE_Q(wmeq,txq);
				if (txq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
					*(txq->axq_link) = cpu_to_le32(bf->bf_daddr);
#else
					*(txq->axq_link) = bf->bf_daddr;
#endif
				} 
			}
			if (index == WME_AC_BE || txq != prevq) {
				/* 
				 * find the first buffer to be DMA'd.
				 */
				bf = STAILQ_FIRST(&txq->axq_q);
				while (bf) {
#ifdef ATH_SUPERG_FF
					ds = &bf->bf_desc[bf->bf_numdesc - 1];
#else
					ds = bf->bf_desc;	/* NB: last descriptor */
#endif
					status = ath_hal_txprocdesc(ah, ds);
					if (status == AR5K_EINPROGRESS)
						break; 
					bf = STAILQ_NEXT(bf,bf_list);
				}
				if (bf) {
					ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
					ath_hal_txstart(ah, txq->axq_qnum);
				}
				ATH_TXQ_UNLOCK(txq);
			}
			index--;
		}
		printk("moved %d buffers from XR to NORMAL\n", count);
	}
#endif
	return 0;
}
#endif
#endif

static struct sk_buff *
ath_alloc_skb(u_int size, u_int align)
{
	struct sk_buff *skb;
	u_int off;

	skb = dev_alloc_skb(size + align - 1);
	if (skb != NULL) {
		off = ((unsigned long) skb->data) % align;
		if (off != 0)
			skb_reserve(skb, align - off);
	}
	return skb;
}

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;
	struct ath_desc *ds;

	skb = bf->bf_skb;
	if (skb == NULL) {
 		if (sc->sc_nmonvaps > 0) {
 			u_int off;
#if 0
			int extra = A_MAX(sizeof(struct ath_rx_radiotap_header), 
					  A_MAX(sizeof(wlan_ng_prism2_header), ATHDESC_HEADER_SIZE));
#else
			int extra = ATHDESC_HEADER_SIZE;
#endif
						
 			/*
 			 * Allocate buffer for monitor mode with space for the
			 * wlan-ng style physical layer header at the start.
 			 */
 			skb = dev_alloc_skb(sc->sc_rxbufsize + extra + sc->sc_cachelsz - 1);
 			if (skb == NULL) {
 				DPRINTF(sc, ATH_DEBUG_ANY,
					"%s: skbuff alloc of size %u failed\n",
					__func__,
					sc->sc_rxbufsize + extra + sc->sc_cachelsz - 1);
 				sc->sc_stats.ast_rx_nobuf++;
 				return -ENOMEM;
 			}
#if 0
 			/*
			 * Reserve space for the Prism header.
 			 */
 			skb_reserve(skb, sizeof(wlan_ng_prism2_header));
#endif
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
				DPRINTF(sc, ATH_DEBUG_ANY,
					"%s: skbuff alloc of size %u failed\n",
					__func__, sc->sc_rxbufsize);
				sc->sc_stats.ast_rx_nobuf++;
				return -ENOMEM;
			}
		}
		bf->bf_skb = skb;
		bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
			skb->data, sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
	}

	/*
	 * Setup descriptors.  For receive we always terminate
	 * the descriptor list with a self-linked entry so we'll
	 * not get overrun under high load (as can happen with a
	 * 5212 when ANI processing enables PHY error frames).
	 *
	 * To ensure the last descriptor is self-linked we create
	 * each descriptor as self-linked and add it to the end.  As
	 * each additional descriptor is added the previous self-linked
	 * entry is ``fixed'' naturally.  This should be safe even
	 * if DMA is happening.  When processing RX interrupts we
	 * never remove/process the last, self-linked, entry on the
	 * descriptor list.  This ensures the hardware always has
	 * someplace to write a new frame.
	 */
	ds = bf->bf_desc;
	ds->ds_link = bf->bf_daddr;		/* link to self */
	ds->ds_data = bf->bf_skbaddr;
//	ds->ds_vdata = (void *) skb->data;	/* virt addr of buffer */
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
 * Extend 15-bit time stamp from rx descriptor to
 * a full 64-bit TSF using the current h/w TSF.
 */
static __inline u_int64_t
ath_extend_tsf(struct ath_hal *ah, u_int32_t rstamp)
{
	u_int64_t tsf;

	tsf = ath_hal_gettsf64(ah);
	if ((tsf & 0x7fff) < rstamp)
		tsf -= 0x8000;
	return ((tsf &~ 0x7fff) | rstamp);
}

#if 0
/*
 * Add a prism2 header to a received frame and
 * dispatch it to capture tools like kismet.
 */
static void
ath_rx_capture(struct net_device *dev, struct ath_desc *ds, struct sk_buff *skb)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	u_int64_t tsf;

	/* Pass up tsf clock in mactime
	 * Rx descriptor has the low 15 bits of the tsf at
	 * the time the frame was received.  Use the current
	 * tsf to extend this to 64 bits.
	 */
	tsf = ath_extend_tsf(sc->sc_ah, ds->ds_rxstat.rs_tstamp);

	KASSERT(ic->ic_flags & IEEE80211_F_DATAPAD,
		("data padding not enabled?"));

	wh = (struct ieee80211_frame *) skb->data;
	if (IEEE80211_QOS_HAS_SEQ(wh)) {
		struct sk_buff *skb1 = skb_copy(skb, GFP_ATOMIC);
		/* Remove hw pad bytes */
		int headersize = ieee80211_hdrsize(wh);
		int padbytes = roundup(headersize, 4) - headersize;
		if (padbytes > 0) {
			memmove(skb1->data + padbytes, skb1->data, headersize);
			skb_pull(skb1, padbytes);
		}
		ieee80211_input_monitor(ic, skb1, ds, 0, tsf, sc);
		dev_kfree_skb(skb1);
	} else {
		ieee80211_input_monitor(ic, skb, ds, 0, tsf, sc);
	}
}


static void
ath_tx_capture(struct net_device *dev, struct ath_desc *ds, struct sk_buff *skb)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	int extra = A_MAX(sizeof(struct ath_tx_radiotap_header), 
			  A_MAX(sizeof(wlan_ng_prism2_header), ATHDESC_HEADER_SIZE));
	u_int64_t tsf;
	u_int32_t tstamp;
	
	/* Pass up tsf clock in mactime
	 * TX descriptor contains the transmit time in TU's,
	 * (bits 25-10 of the TSF).
	 */
	tsf = ath_hal_gettsf64(sc->sc_ah);
	tstamp = ds->ds_txstat.ts_tstamp << 10;
	
	if ((tsf & 0x3ffffff) < tstamp)
		tsf -= 0x4000000;
	tsf = ((tsf &~ 0x3ffffff) | tstamp);

	/*                                                                      
	 * release the owner of this skb since we're basically                  
	 * recycling it                                                         
	 */
	if (atomic_read(&skb->users) != 1) {
		struct sk_buff *skb2 = skb;
		skb = skb_copy(skb, GFP_ATOMIC);
		if (skb == NULL) {
			printk("%s:%d %s\n", __FILE__, __LINE__, __func__);
			dev_kfree_skb(skb2);
			return;
		}
		dev_kfree_skb(skb2);
	} else
		skb_orphan(skb);

	wh = (struct ieee80211_frame *) skb->data;
	if (IEEE80211_QOS_HAS_SEQ(wh)) {
		/* Unlike in rx_capture, we're freeing the skb at the end
		 * anyway, so we don't need to worry about using a copy */
		int headersize = ieee80211_hdrsize(wh);
		int padbytes = roundup(headersize, 4) - headersize;
		if (padbytes > 0) {
			memmove(skb->data + padbytes, skb->data, headersize);
			skb_pull(skb, padbytes);
		}
	}
	
	if (skb_headroom(skb) < extra &&
	    pskb_expand_head(skb, extra, 0, GFP_ATOMIC)) {
		printk("%s:%d %s\n", __FILE__, __LINE__, __func__);
		goto done;
	}
	ieee80211_input_monitor(ic, skb, ds, 1, tsf, sc);
 done:
	dev_kfree_skb(skb);
}
#endif

#if 0
/*
 * Intercept management frames to collect beacon rssi data
 * and to do ibss merges.
 */
static void
ath_recv_mgmt(struct ieee80211_node *ni, struct sk_buff *skb,
	int subtype, int rssi, u_int32_t rstamp)
{
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;
	struct ieee80211vap *vap = ni->ni_vap;

	/*
	 * Call up first so subsequent work can use information
	 * potentially stored in the node (e.g. for ibss merge).
	 */
	sc->sc_recv_mgmt(ni, skb, subtype, rssi, rstamp);
	switch (subtype) {
	case IEEE80211_FC0_SUBTYPE_BEACON:
		/* update rssi statistics for use by the HAL */
		ATH_RSSI_LPF(ATH_NODE(ni)->an_halstats.ns_avgbrssi, rssi);
		if (sc->sc_syncbeacon &&
		    ni == vap->iv_bss && vap->iv_state == IEEE80211_S_RUN) {
			/*
			 * Resync beacon timers using the tsf of the
			 * beacon frame we just received.
			 */
			ath_beacon_config(sc, vap);
		}
		/* fall thru... */
	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
		if (vap->iv_opmode == IEEE80211_M_IBSS &&
		    vap->iv_state == IEEE80211_S_RUN) {
			u_int64_t tsf = ath_extend_tsf(sc->sc_ah, rstamp);
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
			/* jal: added: don't merge if we have a desired
			   BSSID */
			if (!(vap->iv_flags & IEEE80211_F_DESBSSID) &&
				le64_to_cpu(ni->ni_tstamp.tsf) >= tsf) {
				DPRINTF(sc, ATH_DEBUG_STATE,
					"ibss merge, rstamp %u tsf %llu "
					"tstamp %llu\n", rstamp, (long long) tsf,
					(long long) ni->ni_tstamp.tsf);
				(void) ieee80211_ibss_merge(ni);
			}
		}
		break;
	}
}
#endif

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

/**
 * ath_rx_hw_decrypted - determine if this frame was decrypted by hardware
 *
 * Returns non-zero if the frame was decrypted by the device.
 */
static int
ath_rx_hw_decrypted(struct ath_softc *sc, struct ath_desc *ds,
		    struct sk_buff *skb, int header_len)
{
	u16 fc;
	struct ieee80211_hdr *hdr;
	int keyix;

	if (!(ds->ds_rxstat.rs_status & AR5K_RXERR_DECRYPT) &&
	    (ds->ds_rxstat.rs_keyix != AR5K_RXKEYIX_INVALID))
		return 1;

	hdr = (struct ieee80211_hdr *) skb->data;
	fc = le16_to_cpu(hdr->frame_control);

	/* Apparently when a default key is used to decrypt the packet
	   the hal does not set the index used to decrypt.  In such cases
	   get the index from the packet. */
	if (!(ds->ds_rxstat.rs_status & AR5K_RXERR_DECRYPT) &&
	    fc & IEEE80211_FCTL_PROTECTED &&
	    skb->len >= header_len + 4) {
		keyix = skb->data[header_len + 3] >> 6;

		if (isset(sc->sc_keymap, keyix)) {
			return 1;
		}
	}

	return 0;
}

static void
ath_rx_tasklet(TQUEUE_ARG data)
{
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc + \
		((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))
	struct ath_buf *bf;
	struct ath_softc *sc = (struct ath_softc *)data;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	struct sk_buff *skb;
	struct ieee80211_rx_status status;
#if 0
	struct ieee80211_node *ni;
	int type;
#endif
	int len;
	u_int phyerr;
#if 0
	/* Let the 802.11 layer know about the new noise floor */
	ic->ic_channoise = sc->sc_channoise;
#endif

	DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s\n", __func__);
	do {
		int header_len;
		int header_pad;

		status.flag = 0;

		bf = STAILQ_FIRST(&sc->sc_rxbuf);
		if (bf == NULL) {		/* XXX ??? can this happen */
			printk("%s: no buffer (%s)\n", sc->name, __func__);
			break;
		}

		/*
		 * Descriptors are now processed at in the first-level
		 * interrupt handler to support U-APSD trigger search.
		 * This must also be done even when U-APSD is not active to support
		 * other error handling that requires immediate attention.
		 * We check bf_status to find out if the bf's descriptors have 
		 * been processed by the HAL.
		 */
		if (!(bf->bf_status & ATH_BUFSTATUS_DONE))
			break;
		
		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at the end */
			break;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {		/* XXX ??? can this happen */
			printk("%s: no skbuff (%s)\n", sc->name, __func__);
			continue;
		}

#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RECV_DESC)
			ath_printrxbuf(bf, 1);
#endif

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
			if (sc->sc_opmode != AR5K_M_MONITOR) {
				sc->sc_stats.ast_rx_toobig++;
				goto rx_next;
			}
#endif
			/* fall thru for monitor mode handling... */
		} else if (ds->ds_rxstat.rs_status != 0) {
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_CRC)
				sc->sc_stats.ast_rx_crcerr++;
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_FIFO)
				sc->sc_stats.ast_rx_fifoerr++;
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_PHY) {
				sc->sc_stats.ast_rx_phyerr++;
				phyerr = ds->ds_rxstat.rs_phyerr & 0x1f;
				sc->sc_stats.ast_rx_phy[phyerr]++;
				goto rx_next;
			}
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_DECRYPT) {
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
				if (ds->ds_rxstat.rs_keyix == AR5K_RXKEYIX_INVALID)
					goto rx_accept;
				sc->sc_stats.ast_rx_badcrypt++;
			}
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_MIC) {
				status.flag |= RX_FLAG_MMIC_ERROR;
				sc->sc_stats.ast_rx_badmic++;
				goto rx_accept;
			}
			/*
			 * Reject error frames, we normally don't want
			 * to see them in monitor mode (in monitor mode
			 * allow through packets that have crypto problems).
			 */
			if ((ds->ds_rxstat.rs_status &~
				(AR5K_RXERR_DECRYPT|AR5K_RXERR_MIC)) ||
			    sc->sc_opmode != AR5K_M_MONITOR)
				goto rx_next;
		}
rx_accept:
		/*
		 * Sync and unmap the frame.  At this point we're
		 * committed to passing the sk_buff somewhere so
		 * clear buf_skb; this means a new sk_buff must be
		 * allocated when the rx descriptor is setup again
		 * to receive another frame.
		 */
		len = ds->ds_rxstat.rs_datalen;
		bus_dma_sync_single(sc->sc_bdev,
			bf->bf_skbaddr, len, BUS_DMA_FROMDEVICE);
		bus_unmap_single(sc->sc_bdev, bf->bf_skbaddr,
			sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
		bf->bf_skb = NULL;

		sc->sc_stats.ast_ant_rx[ds->ds_rxstat.rs_antenna]++;

		skb_put(skb, len);
		skb->protocol = __constant_htons(ETH_P_CONTROL);

		if (sc->sc_opmode == AR5K_M_MONITOR)
			status.mactime = ath_extend_tsf(ah, ds->ds_rxstat.rs_tstamp);
		else 
			status.mactime = ds->ds_rxstat.rs_tstamp;

		status.freq = sc->sc_curchan.freq;
		status.freq = sc->sc_ieee80211_channel;
		status.phymode = sc->sc_mode;
		status.ssi = ds->ds_rxstat.rs_rssi;
		status.antenna = ds->ds_rxstat.rs_antenna;
		status.rate = ds->ds_rxstat.rs_rate;

		/* The device will pad 802.11 header to a multiple of 4 bytes.
		   Remove the padding before passing the skb to the stack. */
		header_len = ieee80211_get_hdrlen_from_skb(skb);
		header_pad = (4 - (header_len & 0x3)) & 0x3;

		if (unlikely(header_pad)) {
			memmove(skb->data + header_pad, skb->data, header_len);
			skb_pull(skb, header_pad);
		}

		if (ath_rx_hw_decrypted(sc, ds, skb, header_len))
			status.flag |= RX_FLAG_DECRYPTED;

		__ieee80211_rx(sc->sc_hw, skb, &status);
		if (sc->sc_diversity) {
			/*
			 * When using hardware fast diversity, change the default rx
			 * antenna if rx diversity chooses the other antenna 3
			 * times in a row.
			 */
			if (sc->sc_defant != ds->ds_rxstat.rs_antenna) {
				if (++sc->sc_rxotherant >= 3)
					ath_setdefantenna(sc, ds->ds_rxstat.rs_antenna);
			} else
				sc->sc_rxotherant = 0;
		}
		if (sc->sc_softled) {
			/*
			 * Blink for any data frame.  Otherwise do a
			 * heartbeat-style blink when idle.  The latter
			 * is mainly for station mode where we depend on
			 * periodic beacon frames to trigger the poll event.
			 */
#if 0
			if (type == IEEE80211_FC0_TYPE_DATA) {
#endif
				sc->sc_rxrate = ds->ds_rxstat.rs_rate;
				ath_led_event(sc, ATH_LED_RX);
#if 0
			} else if (jiffies - sc->sc_ledevent >= sc->sc_ledidle)
				ath_led_event(sc, ATH_LED_POLL);
#endif
		}
rx_next:
		ATH_RXBUF_LOCK_IRQ(sc);
		STAILQ_REMOVE_HEAD(&sc->sc_rxbuf, bf_list);
		bf->bf_status &= ~ATH_BUFSTATUS_DONE;
		STAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
		ATH_RXBUF_UNLOCK_IRQ(sc);
	} while (ath_rxbuf_init(sc, bf) == 0);
	
	/* rx signal state monitoring -not yet implemented-*/
/*	ath_hal_rxmonitor(ah, &sc->sc_halstats, &sc->sc_curchan);
	if (ath_hal_radar_event(ah)) {
		sc->sc_rtasksched = 1;
		schedule_work(&sc->sc_radartask);
	}*/
#undef PA2DESC
}

#if 0
#ifdef ATH_SUPERG_XR

static void 
ath_grppoll_period_update(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	u_int16_t interval;
	u_int16_t xrsta;
	u_int16_t normalsta;
	u_int16_t allsta;

	xrsta = ic->ic_xr_sta_assoc;

	/*
	 * if no stations are in XR mode.
	 * use default poll interval.
	 */
	if (xrsta == 0) { 
		if (sc->sc_xrpollint != XR_DEFAULT_POLL_INTERVAL) {
			sc->sc_xrpollint = XR_DEFAULT_POLL_INTERVAL;
			ath_grppoll_txq_update(sc,XR_DEFAULT_POLL_INTERVAL);
		}
		return;
	}

	allsta = ic->ic_sta_assoc;
	/*
	 * if all the stations are in XR mode.
	 * use minimum poll interval.
	 */
	if (allsta == xrsta) { 
		if (sc->sc_xrpollint != XR_MIN_POLL_INTERVAL) {
			sc->sc_xrpollint = XR_MIN_POLL_INTERVAL;
			ath_grppoll_txq_update(sc,XR_MIN_POLL_INTERVAL);
		}
		return;
	}

	normalsta = allsta-xrsta;
	/*
	 * if stations are in both XR and normal mode. 
	 * use some fudge factor.
	 */
	interval = XR_DEFAULT_POLL_INTERVAL -
          ((XR_DEFAULT_POLL_INTERVAL - XR_MIN_POLL_INTERVAL) * xrsta)/(normalsta * XR_GRPPOLL_PERIOD_FACTOR);
	if (interval < XR_MIN_POLL_INTERVAL)
		interval = XR_MIN_POLL_INTERVAL;
	
	if (sc->sc_xrpollint != interval) {
		sc->sc_xrpollint = interval;
		ath_grppoll_txq_update(sc,interval);
	}

	/*
	 * XXX: what if stations go to sleep?
	 * ideally the interval should be adjusted dynamically based on
	 * xr and normal upstream traffic.
	 */
}

/*
 * update grppoll period.
 */
static void 
ath_grppoll_txq_update(struct ath_softc *sc, int period)
{
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;
	struct ath_txq *txq = &sc->sc_grpplq;

	if (sc->sc_grpplq.axq_qnum == -1)
		return; 

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = 0;
	qi.tqi_aifs = XR_AIFS;
	qi.tqi_cw_min = XR_CWMIN_CWMAX;
	qi.tqi_cw_max = XR_CWMIN_CWMAX;
	qi.tqi_comp_buffer = 0;
	qi.tqi_cbrPeriod = IEEE80211_TU_TO_MS(period) * 1000; /* usec */
	qi.tqi_cbrOverflowLimit = 2;
	ath_hal_settxqueueprops(ah, txq->axq_qnum,&qi);
	ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
}

/*
 * Setup grppoll  h/w transmit queue.
 */
static void 
ath_grppoll_txq_setup(struct ath_softc *sc, int qtype, int period)
{
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;
	int qnum;
	u_int compbufsz = 0;
	char *compbuf = NULL;
	dma_addr_t compbufp = 0;
	struct ath_txq *txq = &sc->sc_grpplq;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = 0;
	qi.tqi_aifs = XR_AIFS;
	qi.tqi_cw_min = XR_CWMIN_CWMAX;
	qi.tqi_cw_max = XR_CWMIN_CWMAX;
	qi.tqi_comp_buffer = 0;
	qi.tqi_cbrPeriod = IEEE80211_TU_TO_MS(period) * 1000; /* usec */
	qi.tqi_cbrOverflowLimit = 2;

	if (sc->sc_grpplq.axq_qnum == -1) {
		qnum = ath_hal_setuptxqueue(ah, qtype, &qi);
		if (qnum == -1)
			return ;
		if (qnum >= N(sc->sc_txq)) {
			printk("%s: HAL qnum %u out of range, max %u!\n",
				   sc->sc_dev->name, qnum, N(sc->sc_txq));
			ath_hal_releasetxqueue(ah, qnum);
			return;
		}

		txq->axq_qnum = qnum;
	}
	txq->axq_link = NULL;
	STAILQ_INIT(&txq->axq_q);
	ATH_TXQ_LOCK_INIT(txq);
	txq->axq_depth = 0;
	txq->axq_totalqueued = 0;
	txq->axq_intrcnt = 0;
	TAILQ_INIT(&txq->axq_stageq);
	txq->axq_compbuf = compbuf;
	txq->axq_compbufsz = compbufsz;
	txq->axq_compbufp = compbufp;
	ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
#undef N

}

/*
 * Setup group poll frames on the group poll queue.
 */
static void ath_grppoll_start(struct ieee80211vap *vap,int pollcount)
{
	int i, amode;
	int flags;
	struct sk_buff *skb = NULL;
	struct ath_buf *bf, *head = NULL;
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int8_t rate;
	int ctsrate = 0;
	int ctsduration = 0;
	const AR5K_RATE_TABLE *rt;
	u_int8_t cix, rtindex = 0;
	u_int type;
	struct ath_txq *txq = &sc->sc_grpplq;
	struct ath_desc *ds = NULL;
	int pktlen = 0, keyix = 0;
	int pollsperrate, pos;
	int rates[XR_NUM_RATES];
	u_int8_t ratestr[16], numpollstr[16];
	typedef struct rate_to_str_map {
		u_int8_t str[4];
		int rate_kbps;
	} RATE_TO_STR_MAP;

	static const RATE_TO_STR_MAP ratestrmap[] = {
		{"0.25",    250},
		{ ".25",    250},
		{"0.5",     500},
		{ ".5",     500},
		{  "1",    1000},
		{  "3",    3000},
		{  "6",    6000},
		{  "?",       0},
	};

#define MAX_GRPPOLL_RATE 5
#define	USE_SHPREAMBLE(_ic) \
	(((_ic)->ic_flags & (IEEE80211_F_SHPREAMBLE | IEEE80211_F_USEBARKER)) \
		== IEEE80211_F_SHPREAMBLE)

	if (sc->sc_xrgrppoll)
		return; 

	memset(&rates, 0, sizeof(rates));
	pos = 0;
	while (sscanf(&(sc->sc_grppoll_str[pos]), "%s %s", ratestr, numpollstr) == 2) {
		int rtx = 0;
		while (ratestrmap[rtx].rate_kbps != 0) {
			if (strcmp(ratestrmap[rtx].str, ratestr) == 0)
				break;
			rtx++;
		}
		sscanf(numpollstr, "%d", &(rates[rtx]));
		pos += strlen(ratestr) + strlen(numpollstr) + 2;
	}
	if (!sc->sc_grppolldma.dd_bufptr) {
		printk("grppoll_start: grppoll Buf allocation failed\n");
		return;
	}
	rt = sc->sc_currates;
	cix = rt->rates[sc->sc_protrix].controlRate;
	ctsrate = rt->rates[cix].rateCode;
	if (USE_SHPREAMBLE(ic))
			ctsrate |= rt->rates[cix].shortPreamble;
	rt = sc->sc_xr_rates;
	/*
	 * queue the group polls for each antenna mode. set the right keycache index for the
	 * broadcast packets. this will ensure that if the first poll
	 * does not elicit a single chirp from any XR station, hardware will
	 * not send the subsequent polls
	 */
	pollsperrate = 0;
	for (amode = AR5K_ANTENNA_FIXED_A; amode < AR5K_ANTENNA_MAX_MODE ; amode++) {
		for (i = 0; i < (pollcount + 1); i++) {

			flags = AR5K_TXDESC_NOACK;
			rate = rt->rates[rtindex].rateCode;
			/* 
			 * except for the last one every thing else is a CF poll.
			 * last one is  the CF End frame.
			 */

			if (i == pollcount) {
				skb = ieee80211_getcfframe(vap,IEEE80211_FC0_SUBTYPE_CF_END);
				rate = ctsrate;
				ctsduration = ath_hal_computetxtime(ah,
					sc->sc_currates, pktlen, sc->sc_protrix, FALSE);
			} else {
				skb = ieee80211_getcfframe(vap, IEEE80211_FC0_SUBTYPE_CFPOLL);
				pktlen = skb->len + IEEE80211_CRC_LEN;
				/*
				 * the very first group poll ctsduration  should be enough to allow
				 * an auth frame from station. This is to pass the wifi testing (as 
				 * some stations in testing do not honor CF_END and rely on CTS duration)
				 */
				if (i == 0 && amode == AR5K_ANTENNA_FIXED_A) {
					ctsduration = ath_hal_computetxtime(ah,	rt,
							pktlen,	rtindex,
							FALSE) /*cf-poll time */
						+ (XR_AIFS + (XR_CWMIN_CWMAX * XR_SLOT_DELAY))  
						+ ath_hal_computetxtime(ah, rt,
							2 * (sizeof(struct ieee80211_frame_min) + 6),
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							FALSE) /*auth packet time */
						+ ath_hal_computetxtime(ah, rt,
							IEEE80211_ACK_LEN,
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							FALSE); /*ack frame time */ 
				} else {
					ctsduration = ath_hal_computetxtime(ah, rt,
							pktlen, rtindex,
							FALSE) /*cf-poll time */
						+ (XR_AIFS + (XR_CWMIN_CWMAX * XR_SLOT_DELAY))  
						+ ath_hal_computetxtime(ah,rt,
							XR_FRAGMENTATION_THRESHOLD,
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							FALSE) /*data packet time */
						+ ath_hal_computetxtime(ah,rt,
							IEEE80211_ACK_LEN,
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							FALSE); /*ack frame time */ 
				}
				if ((vap->iv_flags & IEEE80211_F_PRIVACY) && keyix == 0) {
					struct ieee80211_key *k;
					k = ieee80211_crypto_encap(vap->iv_bss, skb);
					if (k)
						keyix = k->wk_keyix;
				}
			}
			ATH_TXBUF_LOCK_IRQ(sc);					
			bf = STAILQ_FIRST(&sc->sc_grppollbuf);
			if (bf != NULL)
				STAILQ_REMOVE_HEAD(&sc->sc_grppollbuf, bf_list);
			else {
				DPRINTF(sc, ATH_DEBUG_XMIT, "%s: No more TxBufs\n", __func__);
				ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
				return;
			}
			/* XXX use a counter and leave at least one for mgmt frames */
			if (STAILQ_EMPTY(&sc->sc_grppollbuf)) {				
				DPRINTF(sc, ATH_DEBUG_XMIT, "%s: No more TxBufs left\n", __func__);
				ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
				return;
			}					
			ATH_TXBUF_UNLOCK_IRQ(sc);
			bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
				skb->data, skb->len, BUS_DMA_TODEVICE);
			bf->bf_skb = skb;
			ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
			ds = bf->bf_desc;
			ds->ds_data = bf->bf_skbaddr;
			if (i == pollcount && amode == (AR5K_ANTENNA_MAX_MODE -1)) {
				type = AR5K_PKT_TYPE_NORMAL;
				flags |= (AR5K_TXDESC_CLRDMASK | AR5K_TXDESC_VEOL);
			} else {
				flags |= AR5K_TXDESC_CTSENA;
				type = AR5K_PKT_TYPE_GRP_POLL;
			}
			if (i == 0 && amode == AR5K_ANTENNA_FIXED_A ) {
				flags |= AR5K_TXDESC_CLRDMASK;  
				head = bf;
			}
			ath_hal_setuptxdesc(ah, ds
				, skb->len + IEEE80211_CRC_LEN	/* frame length */
				, sizeof(struct ieee80211_frame)	/* header length */
				, type			/* Atheros packet type */
				, ic->ic_txpowlimit	/* max txpower */
				, rate, 0		/* series 0 rate/tries */
				, keyix /* AR5K_TXKEYIX_INVALID */		/* use key index */
				, amode			/* antenna mode */
				, flags				
				, ctsrate		/* rts/cts rate */
				, ctsduration		/* rts/cts duration */
//				, 0			/* comp icv len */
//				, 0			/* comp iv len */
//				, ATH_COMP_PROC_NO_COMP_NO_CCS	/* comp scheme */
				);
			ath_hal_filltxdesc(ah, ds
				, roundup(skb->len, 4)	/* buffer length */
				, TRUE		/* first segment */
				, TRUE		/* last segment */
				, ds			/* first descriptor */
				);
			/* NB: The desc swap function becomes void, 
	 		* if descriptor swapping is not enabled
	 		*/
			ath_desc_swap(ds);
			if (txq->axq_link) {
#ifdef AH_NEED_DESC_SWAP
				*txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
				*txq->axq_link = bf->bf_daddr;
#endif
			}
			txq->axq_link = &ds->ds_link;
			pollsperrate++;
			if (pollsperrate > rates[rtindex]) {
				rtindex = (rtindex + 1) % MAX_GRPPOLL_RATE;
				pollsperrate = 0;
			}
		}
	}
	/* make it circular */
#ifdef AH_NEED_DESC_SWAP
	ds->ds_link = cpu_to_le32(head->bf_daddr);
#else
	ds->ds_link = head->bf_daddr;
#endif
	/* start the queue */
	ath_hal_puttxbuf(ah, txq->axq_qnum, head->bf_daddr);
	ath_hal_txstart(ah, txq->axq_qnum);
	sc->sc_xrgrppoll = 1;
#undef USE_SHPREAMBLE
}

static void ath_grppoll_stop(struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_txq *txq = &sc->sc_grpplq;
	struct ath_buf *bf;
	
	
	if (!sc->sc_xrgrppoll)
		return; 
	ath_hal_stoptxdma(ah, txq->axq_qnum);

	/* move the grppool bufs back to the grppollbuf */
	for (;;) {
		ATH_TXQ_LOCK(txq);
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK(txq);
			break;
		}
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK(txq);
		bus_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);
		dev_kfree_skb(bf->bf_skb);
		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		ATH_TXBUF_LOCK(sc);
		STAILQ_INSERT_TAIL(&sc->sc_grppollbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK(sc);
	}
	STAILQ_INIT(&txq->axq_q);
	ATH_TXQ_LOCK_INIT(txq);
	txq->axq_depth = 0;
	txq->axq_totalqueued = 0;
	txq->axq_intrcnt = 0;
	TAILQ_INIT(&txq->axq_stageq);
	sc->sc_xrgrppoll = 0;
}
#endif
#endif

/*
 * Setup a h/w transmit queue.
 */
static struct ath_txq *
ath_txq_setup(struct ath_softc *sc, int qtype, int subtype)
{
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;
	int qnum;
	u_int compbufsz = 0;
	char *compbuf = NULL;
	dma_addr_t compbufp = 0;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = subtype;
	qi.tqi_aifs = AR5K_TXQ_USEDEFAULT;
	qi.tqi_cw_min = AR5K_TXQ_USEDEFAULT;
	qi.tqi_cw_max = AR5K_TXQ_USEDEFAULT;
	qi.tqi_comp_buffer = 0;
#ifdef ATH_SUPERG_XR
	if (subtype == AR5K_XR_DATA) {
		qi.tqi_aifs  = XR_DATA_AIFS;
		qi.tqi_cw_min = XR_DATA_CWMIN;
		qi.tqi_cw_max = XR_DATA_CWMAX;
	}
#endif

#ifdef ATH_SUPERG_COMP
	/* allocate compression scratch buffer for data queues */
	if (((qtype == AR5K_TX_QUEUE_DATA)|| (qtype == AR5K_TX_QUEUE_UAPSD)) 
	    && ath_hal_compressionsupported(ah)) {
		compbufsz = roundup(AR5K_COMP_BUF_MAX_SIZE, 
			AR5K_COMP_BUF_ALIGN_SIZE) + AR5K_COMP_BUF_ALIGN_SIZE;
		compbuf = (char *)bus_alloc_consistent(sc->sc_bdev,
			compbufsz, &compbufp);
		if (compbuf == NULL)
			sc->sc_ic.ic_ath_cap &= ~IEEE80211_ATHC_COMP;	
		else
			qi.tqi_comp_buffer = (u_int32_t)compbufp;
	} 
#endif
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
	 *
	 * The UAPSD queue is an exception, since we take a desc-
	 * based intr on the EOSP frames.
	 */
	if (qtype == AR5K_TX_QUEUE_UAPSD)
		qi.tqi_flags = AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
	else
		qi.tqi_flags = AR5K_TXQ_FLAG_TXEOLINT_ENABLE | AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
	qnum = ath_hal_setuptxqueue(ah, qtype, &qi);
	if (qnum == -1) {
		/*
		 * NB: don't print a message, this happens
		 * normally on parts with too few tx queues
		 */
#ifdef ATH_SUPERG_COMP
		if (compbuf) {
			bus_free_consistent(sc->sc_bdev, compbufsz,
				compbuf, compbufp);
		}
#endif
		return NULL;
	}
	if (qnum >= N(sc->sc_txq)) {
		printk("%s: HAL qnum %u out of range, max %u!\n",
			sc->name, qnum, N(sc->sc_txq));
#ifdef ATH_SUPERG_COMP
		if (compbuf) {
			bus_free_consistent(sc->sc_bdev, compbufsz,
				compbuf, compbufp);
		}
#endif
		ath_hal_releasetxqueue(ah, qnum);
		return NULL;
	}
	if (!ATH_TXQ_SETUP(sc, qnum)) {
		struct ath_txq *txq = &sc->sc_txq[qnum];

		txq->axq_qnum = qnum;
		txq->axq_link = NULL;
		STAILQ_INIT(&txq->axq_q);
		ATH_TXQ_LOCK_INIT(txq);
		txq->axq_depth = 0;
		txq->axq_totalqueued = 0;
		txq->axq_intrcnt = 0;
		TAILQ_INIT(&txq->axq_stageq);
		txq->axq_compbuf = compbuf;
		txq->axq_compbufsz = compbufsz;
		txq->axq_compbufp = compbufp;
		sc->sc_txqsetup |= 1 << qnum;
	}
	return &sc->sc_txq[qnum];
#undef N
}

/*
 * Setup a hardware data transmit queue for the specified
 * access control.  The HAL may not support all requested
 * queues in which case it will return a reference to a
 * previously setup queue.  We record the mapping from ac's
 * to h/w queues for use by ath_tx_start and also track
 * the set of h/w queues being used to optimize work in the
 * transmit interrupt handler and related routines.
 */
static int
ath_tx_setup(struct ath_softc *sc, int ac, int haltype)
{
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
	struct ath_txq *txq;

	if (ac >= N(sc->sc_ac2q)) {
		printk("%s: AC %u out of range, max %u!\n",
		       sc->name, ac, (unsigned)N(sc->sc_ac2q));
		return 0;
	}
	txq = ath_txq_setup(sc, AR5K_TX_QUEUE_DATA, haltype);
	if (txq != NULL) {
		sc->sc_ac2q[ac] = txq;
		return 1;
	} else
		return 0;
#undef N
}

#if 0
/*
 * Update WME parameters for a transmit queue.
 */
static int
ath_txq_update(struct ath_softc *sc, struct ath_txq *txq, int ac)
{
#define	ATH_EXPONENT_TO_VALUE(v)	((1<<v)-1)
#define	ATH_TXOP_TO_US(v)		(v<<5)
	struct ieee80211com *ic = &sc->sc_ic;
	struct wmeParams *wmep = &ic->ic_wme.wme_chanParams.cap_wmeParams[ac];
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;

	ath_hal_gettxqueueprops(ah, txq->axq_qnum, &qi);
	qi.tqi_aifs = wmep->wmep_aifsn;
	qi.tqi_cw_min = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmin);
	qi.tqi_cw_max = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmax);
	qi.tqi_burstTime = ATH_TXOP_TO_US(wmep->wmep_txopLimit);

	if (!ath_hal_settxqueueprops(ah, txq->axq_qnum, &qi)) {
		printk("%s: unable to update hardware queue "
			"parameters for %s traffic!\n",
			sc->sc_dev->name, ieee80211_wme_acnames[ac]);
		return 0;
	} else {
		ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
		return 1;
	}
#undef ATH_TXOP_TO_US
#undef ATH_EXPONENT_TO_VALUE
}

/*
 * Callback from the 802.11 layer to update WME parameters.
 */
static int
ath_wme_update(struct ieee80211com *ic)
{
	struct ath_softc *sc = ic->ic_dev->priv;

	if (sc->sc_uapsdq)
		ath_txq_update(sc, sc->sc_uapsdq, WME_AC_VO);

	return !ath_txq_update(sc, sc->sc_ac2q[WME_AC_BE], WME_AC_BE) ||
	    !ath_txq_update(sc, sc->sc_ac2q[WME_AC_BK], WME_AC_BK) ||
	    !ath_txq_update(sc, sc->sc_ac2q[WME_AC_VI], WME_AC_VI) ||
	    !ath_txq_update(sc, sc->sc_ac2q[WME_AC_VO], WME_AC_VO) ? EIO : 0;
}

/*
 * Callback from 802.11 layer to flush a node's U-APSD queues
 */
static void	
ath_uapsd_flush(struct ieee80211_node *ni)
{
	struct ath_node *an = ATH_NODE(ni);
	struct ath_buf *bf;
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;
	struct ath_txq *txq;

	ATH_NODE_UAPSD_LOCK_IRQ(an);
	/*
	 * NB: could optimize for successive runs from the same AC
	 *     if we can assume that is the most frequent case.
	 */
	while (an->an_uapsd_qdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_q);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
		bf->bf_desc->ds_link = 0;
		txq = sc->sc_ac2q[bf->bf_skb->priority & 0x3];
		ath_tx_txqaddbuf(sc, ni, txq, bf, bf->bf_desc, bf->bf_skb->len);
		an->an_uapsd_qdepth--;
	}

	while (an->an_uapsd_overflowqdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		bf->bf_desc->ds_link = 0;
		txq = sc->sc_ac2q[bf->bf_skb->priority & 0x3];
		ath_tx_txqaddbuf(sc, ni, txq, bf, bf->bf_desc, bf->bf_skb->len);
		an->an_uapsd_overflowqdepth--;
	}
	if (IEEE80211_NODE_UAPSD_USETIM(ni))
		ni->ni_vap->iv_set_tim(ni, 0);
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);
}
#endif

/*
 * Reclaim resources for a setup queue.
 */
static void
ath_tx_cleanupq(struct ath_softc *sc, struct ath_txq *txq)
{

#ifdef ATH_SUPERG_COMP
	/* Release compression buffer */
	if (txq->axq_compbuf) {
		bus_free_consistent(sc->sc_bdev, txq->axq_compbufsz,
			txq->axq_compbuf, txq->axq_compbufp);
		txq->axq_compbuf = NULL;
	}
#endif
	ath_hal_releasetxqueue(sc->sc_ah, txq->axq_qnum);
	ATH_TXQ_LOCK_DESTROY(txq);
	sc->sc_txqsetup &= ~(1 << txq->axq_qnum);
}

/*
 * Reclaim all tx queue resources.
 */
static void
ath_tx_cleanup(struct ath_softc *sc)
{
	int i;

	ATH_TXBUF_LOCK_DESTROY(sc);
	for (i = 0; i < AR5K_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_cleanupq(sc, &sc->sc_txq[i]);
}

#ifdef ATH_SUPERG_COMP
static u_int32_t
ath_get_icvlen(struct ieee80211_key *k)
{
	const struct ieee80211_cipher *cip = k->wk_cipher;

	if (cip->ic_cipher == IEEE80211_CIPHER_AES_CCM ||
		cip->ic_cipher == IEEE80211_CIPHER_AES_OCB)
		return AES_ICV_FIELD_SIZE;

	return WEP_ICV_FIELD_SIZE;
}

static u_int32_t
ath_get_ivlen(struct ieee80211_key *k)
{
	const struct ieee80211_cipher *cip = k->wk_cipher;
	u_int32_t ivlen;

	ivlen = WEP_IV_FIELD_SIZE;

	if (cip->ic_cipher == IEEE80211_CIPHER_AES_CCM ||
		cip->ic_cipher == IEEE80211_CIPHER_AES_OCB)
		ivlen += EXT_IV_FIELD_SIZE;
	
	return ivlen;
}
#endif

#if 0
/*
 * Get transmit rate index using rate in Kbps
 */
static __inline int
ath_tx_findindex(const AR5K_RATE_TABLE *rt, int rate)
{
	int i;
	int ndx = 0;

	for (i = 0; i < rt->rate_count; i++) {
		if (rt->rates[i].rate_kbps == rate) {
			ndx = i;
			break;
		}
	}

	return ndx;
}

/*
 * Needs external locking!
 */
static void
ath_tx_uapsdqueue(struct ath_softc *sc, struct ath_node *an, struct ath_buf *bf)
{
	struct ath_buf *lastbuf;

	/* case the delivery queue just sent and can move overflow q over */
	if (an->an_uapsd_qdepth == 0 && an->an_uapsd_overflowqdepth != 0) {
		DPRINTF(sc, ATH_DEBUG_UAPSD,
			"%s: delivery Q empty, replacing with overflow Q\n",
			__func__);
		STAILQ_CONCAT(&an->an_uapsd_q, &an->an_uapsd_overflowq);
		an->an_uapsd_qdepth = an->an_uapsd_overflowqdepth;
		an->an_uapsd_overflowqdepth = 0;
	}

	/* most common case - room on delivery q */
	if (an->an_uapsd_qdepth < an->an_node.ni_uapsd_maxsp) {
		/* add to delivery q */
		if ((lastbuf = STAILQ_LAST(&an->an_uapsd_q, ath_buf, bf_list))) {
#ifdef AH_NEED_DESC_SWAP
			lastbuf->bf_desc->ds_link = cpu_to_le32(bf->bf_daddr);
#else
			lastbuf->bf_desc->ds_link = bf->bf_daddr;
#endif
		}
		STAILQ_INSERT_TAIL(&an->an_uapsd_q, bf, bf_list);
		an->an_uapsd_qdepth++;
		DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s: added AC %d frame to delivery Q, new depth = %d\n", 
				__func__, bf->bf_skb->priority, an->an_uapsd_qdepth);
		return;
	}
	
	/* check if need to make room on overflow queue */
	if (an->an_uapsd_overflowqdepth == an->an_node.ni_uapsd_maxsp) {
		/* 
		 *  pop oldest from delivery queue and cleanup
		 */ 
		lastbuf = STAILQ_FIRST(&an->an_uapsd_q);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
		dev_kfree_skb(lastbuf->bf_skb);
		lastbuf->bf_skb = NULL;
		ieee80211_free_node(lastbuf->bf_node);
		lastbuf->bf_node = NULL;
		ATH_TXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, lastbuf, bf_list);
		ATH_TXBUF_UNLOCK_IRQ(sc);
		
		/*
		 *  move oldest from overflow to delivery
		 */
		lastbuf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		an->an_uapsd_overflowqdepth--;
		STAILQ_INSERT_TAIL(&an->an_uapsd_q, lastbuf, bf_list);
		DPRINTF(sc, ATH_DEBUG_UAPSD,
			"%s: delivery and overflow Qs full, dropped oldest\n",
			__func__);
	}

	/* add to overflow q */
	if ((lastbuf = STAILQ_LAST(&an->an_uapsd_overflowq, ath_buf, bf_list))) {
#ifdef AH_NEED_DESC_SWAP
		lastbuf->bf_desc->ds_link = cpu_to_le32(bf->bf_daddr);
#else
		lastbuf->bf_desc->ds_link = bf->bf_daddr;
#endif
	}
	STAILQ_INSERT_TAIL(&an->an_uapsd_overflowq, bf, bf_list);
	an->an_uapsd_overflowqdepth++;
	DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: added AC %d to overflow Q, new depth = %d\n",
		__func__, bf->bf_skb->priority, an->an_uapsd_overflowqdepth);

	return;
}

static int
ath_tx_start(struct net_device *dev, struct ieee80211_node *ni, struct ath_buf *bf, struct sk_buff *skb, int nextfraglen)
{
#define	MIN(a,b)	((a) < (b) ? (a) : (b))
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_hal *ah = sc->sc_ah;
	int isprot, ismcast, keyix, hdrlen, pktlen, try0;
	u_int8_t rix, txrate, ctsrate;
	u_int32_t ivlen = 0, icvlen = 0;
	int comp = ATH_COMP_PROC_NO_COMP_NO_CCS;
	u_int8_t cix = 0xff;		/* NB: silence compiler */
	struct ath_desc *ds = NULL;
	struct ath_txq *txq = NULL;
	struct ieee80211_frame *wh;
	u_int subtype, flags, ctsduration;
	AR5K_PKT_TYPE atype;
	const AR5K_RATE_TABLE *rt;
	AR5K_BOOL shortPreamble;
	struct ath_node *an;
	struct ath_vap *avp = ATH_VAP(vap);
	int istxfrag;
	u_int8_t antenna;

	wh = (struct ieee80211_frame *) skb->data;
	isprot = wh->i_fc[1] & IEEE80211_FC1_PROT;
	ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
	hdrlen = ieee80211_anyhdrsize(wh);
	istxfrag = (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG) || 
		(((le16toh(*(__le16 *) &wh->i_seq[0]) >> 
		IEEE80211_SEQ_FRAG_SHIFT) & IEEE80211_SEQ_FRAG_MASK) > 0);

	pktlen = skb->len;
#ifdef ATH_SUPERG_FF
	{
		struct sk_buff *skbtmp = skb;
		while ((skbtmp = skbtmp->next))
			pktlen += skbtmp->len;
	}
#endif
	/*
	 * Packet length must not include any
	 * pad bytes; deduct them here.
	 */
	pktlen -= (hdrlen & 3);

	if (isprot) {
		const struct ieee80211_cipher *cip;
		struct ieee80211_key *k;

		/*
		 * Construct the 802.11 header+trailer for an encrypted
		 * frame. The only reason this can fail is because of an
		 * unknown or unsupported cipher/key type.
		 */

		/* FFXXX: change to handle linked skbs */
		k = ieee80211_crypto_encap(ni, skb);
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
		if ((k->wk_flags & IEEE80211_KEY_SWMIC) == 0) {
			if (!istxfrag)
				pktlen += cip->ic_miclen;
			else
				if (cip->ic_cipher != IEEE80211_CIPHER_TKIP)
					pktlen += cip->ic_miclen;
		}
		keyix = k->wk_keyix;

#ifdef ATH_SUPERG_COMP
		icvlen = ath_get_icvlen(k) / 4;
		ivlen = ath_get_ivlen(k) / 4;
#endif
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
#ifndef ATH_SUPERG_FF
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, pktlen, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %x\n",
		__func__, skb, skb->data, skb->len, bf->bf_skbaddr);
#else /* ATH_SUPERG_FF case */
	/* NB: ensure skb->len had been updated for each skb so we don't need pktlen */
	{
		struct sk_buff *skbtmp = skb;
		int i = 0;

		bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
			skb->data, skb->len, BUS_DMA_TODEVICE);
 		DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb%d %p [data %p len %u] skbaddr %llx\n",
			__func__, i, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));
		while ((skbtmp = skbtmp->next)) {
			bf->bf_skbaddrff[i++] = bus_map_single(sc->sc_bdev,
				skbtmp->data, skbtmp->len, BUS_DMA_TODEVICE);
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb%d %p [data %p len %u] skbaddr %llx\n",
				__func__, i, skbtmp, skbtmp->data, skbtmp->len,
				ito64(bf->bf_skbaddrff[i-1]));
		}
		bf->bf_numdesc = i + 1;
	}
#endif /* ATH_SUPERG_FF */
	bf->bf_skb = skb;
	bf->bf_node = ni;

	/* setup descriptors */
	ds = bf->bf_desc;
#ifdef ATH_SUPERG_XR
	if(vap->iv_flags & IEEE80211_F_XR ) 
		rt = sc->sc_xr_rates;
	else
		rt = sc->sc_currates;
#else
	rt = sc->sc_currates;
#endif

	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_mode));

	/*
	 * NB: the 802.11 layer marks whether or not we should
	 * use short preamble based on the current mode and
	 * negotiated parameters.
	 */
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)) {
		shortPreamble = TRUE;
		sc->sc_stats.ast_tx_shortpre++;
	} else
		shortPreamble = FALSE;

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
		rix = sc->sc_minrateix;
		txrate = rt->rates[rix].rateCode;
		if (shortPreamble)
			txrate |= rt->rates[rix].shortPreamble;
		try0 = ATH_TXMAXTRY;

		if (ni->ni_flags & IEEE80211_NODE_QOS) {
			/* NB: force all management frames to highest queue */
			txq = sc->sc_ac2q[WME_AC_VO];
		} else
			txq = sc->sc_ac2q[WME_AC_BE];
		break;
	case IEEE80211_FC0_TYPE_CTL:
		atype = AR5K_PKT_TYPE_PSPOLL;	/* stop setting of duration */
		rix = sc->sc_minrateix;
		txrate = rt->rates[rix].rateCode;
		if (shortPreamble)
			txrate |= rt->rates[rix].shortPreamble;
		try0 = ATH_TXMAXTRY;

		if (ni->ni_flags & IEEE80211_NODE_QOS) {
			/* NB: force all ctl frames to highest queue */
			txq = sc->sc_ac2q[WME_AC_VO];
		} else
			txq = sc->sc_ac2q[WME_AC_BE];
		break;
	case IEEE80211_FC0_TYPE_DATA:
		atype = AR5K_PKT_TYPE_NORMAL;		/* default */
		
		if (ismcast) {
			rix = ath_tx_findindex(rt, vap->iv_mcast_rate);
			txrate = rt->rates[rix].rateCode;
			if (shortPreamble)
				txrate |= rt->rates[rix].shortPreamble;
			/* 
			 * ATH_TXMAXTRY disables Multi-rate retries, which
			 * isn't applicable to mcast packets and overrides
			 * the desired transmission rate for mcast traffic.
			 */
			try0 = ATH_TXMAXTRY;
		} else {
			/*
			 * Data frames; consult the rate control module.
			 */
			ath_rate_findrate(sc, an, shortPreamble, skb->len,
				&rix, &try0, &txrate);

			/* Ratecontrol sometimes returns invalid rate index */
			if (rix != 0xff)
				an->an_prevdatarix = rix;
			else
				rix = an->an_prevdatarix;
		}

		if (M_FLAG_GET(skb, M_UAPSD)) {
			/* U-APSD frame, handle txq later */
			break;
		}

		/*
		 * Default all non-QoS traffic to the best-effort queue.
		 */
		if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
			/* XXX validate skb->priority, remove mask */
			txq = sc->sc_ac2q[skb->priority & 0x3];
			if (ic->ic_wme.wme_wmeChanParams.cap_wmeParams[skb->priority].wmep_noackPolicy) {
				flags |= AR5K_TXDESC_NOACK;
				sc->sc_stats.ast_tx_noack++;
			}
		} else
			txq = sc->sc_ac2q[WME_AC_BE];
		break;
	default:
		printk("%s: bogus frame type 0x%x (%s)\n", dev->name,
			wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK, __func__);
		/* XXX statistic */
		return -EIO;
	}

#ifdef ATH_SUPERG_XR 
	if (vap->iv_flags & IEEE80211_F_XR ) {
		txq = sc->sc_xrtxq;
		if (!txq)
			txq = sc->sc_ac2q[WME_AC_BK];
		flags |= AR5K_TXDESC_CTSENA;
		cix = rt->rates[sc->sc_protrix].controlRate;
	}
#endif
	/*
	 * When servicing one or more stations in power-save mode (or)
	 * if there is some mcast data waiting on mcast queue
	 * (to prevent out of order delivery of mcast,bcast packets)
	 * multicast frames must be buffered until after the beacon.
	 * We use the private mcast queue for that.
	 */
	if (ismcast && (vap->iv_ps_sta || avp->av_mcastq.axq_depth)) {
		txq = &avp->av_mcastq;
		/* XXX? more bit in 802.11 frame header */
	}

	/*
	 * Calculate miscellaneous flags.
	 */
	if (ismcast) {
		flags |= AR5K_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
		try0 = ATH_TXMAXTRY;    /* turn off multi-rate retry for multicast traffic */
	 } else if (pktlen > vap->iv_rtsthreshold) { 
#ifdef ATH_SUPERG_FF
		/* we could refine to only check that the frame of interest
		 * is a FF, but this seems inconsistent.
		 */
		if (!(vap->iv_ath_cap & ni->ni_ath_flags & IEEE80211_ATHC_FF)) {
#endif
			flags |= AR5K_TXDESC_RTSENA;	/* RTS based on frame length */
			cix = rt->rates[rix].controlRate;
			sc->sc_stats.ast_tx_rts++;
#ifdef ATH_SUPERG_FF
		}
#endif
	}

	/*
	 * If 802.11g protection is enabled, determine whether
	 * to use RTS/CTS or just CTS.  Note that this is only
	 * done for OFDM unicast frames.
	 */
	if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
	    rt->rates[rix].modulation == IEEE80211_T_OFDM &&
	    (flags & AR5K_TXDESC_NOACK) == 0) {
		/* XXX fragments must use CCK rates w/ protection */
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
			flags |= AR5K_TXDESC_RTSENA;
		else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
			flags |= AR5K_TXDESC_CTSENA;

		if (istxfrag)
			/*
			**  if Tx fragment, it would be desirable to 
			**  use highest CCK rate for RTS/CTS.
			**  However, stations farther away may detect it
			**  at a lower CCK rate. Therefore, use the 
			**  configured protect rate, which is 2 Mbps
			**  for 11G.
			*/
			cix = rt->rates[sc->sc_protrix].controlRate;
		else
			cix = rt->rates[sc->sc_protrix].controlRate;
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
		if (shortPreamble)
			dur = rt->rates[rix].spAckDuration;
		else
			dur = rt->rates[rix].lpAckDuration;

		if (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG) {
			dur += dur;  /* Add additional 'SIFS + ACK' */

			/*
			** Compute size of next fragment in order to compute
			** durations needed to update NAV.
			** The last fragment uses the ACK duration only.
			** Add time for next fragment.
			*/
			dur += ath_hal_computetxtime(ah, rt, nextfraglen, 
				rix, shortPreamble);
		}

		if (istxfrag) {
			/*
			**  Force hardware to use computed duration for next
			**  fragment by disabling multi-rate retry, which
			**  updates duration based on the multi-rate
			**  duration table.
			*/
			try0 = ATH_TXMAXTRY;
		}

		*(u_int16_t *)wh->i_dur = cpu_to_le16(dur);
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
		ctsrate = rt->rates[cix].rateCode;
		/*
		 * Compute the transmit duration based on the frame
		 * size and the size of an ACK frame.  We call into the
		 * HAL to do the computation since it depends on the
		 * characteristics of the actual PHY being used.
		 *
		 * NB: CTS is assumed the same size as an ACK so we can
		 *     use the precalculated ACK durations.
		 */
		if (shortPreamble) {
			ctsrate |= rt->rates[cix].shortPreamble;
			if (flags & AR5K_TXDESC_RTSENA)		/* SIFS + CTS */
				ctsduration += rt->rates[cix].spAckDuration;
			ctsduration += ath_hal_computetxtime(ah,
				rt, pktlen, rix, TRUE);
			if ((flags & AR5K_TXDESC_NOACK) == 0)	/* SIFS + ACK */
				ctsduration += rt->rates[rix].spAckDuration;
		} else {
			if (flags & AR5K_TXDESC_RTSENA)		/* SIFS + CTS */
				ctsduration += rt->rates[cix].lpAckDuration;
			ctsduration += ath_hal_computetxtime(ah,
				rt, pktlen, rix, FALSE);
			if ((flags & AR5K_TXDESC_NOACK) == 0)	/* SIFS + ACK */
				ctsduration += rt->rates[rix].lpAckDuration;
		}
		/*
		 * Must disable multi-rate retry when using RTS/CTS.
		 */
		try0 = ATH_TXMAXTRY;
	} else 
		ctsrate = 0;

	if (IFF_DUMPPKTS(sc, ATH_DEBUG_XMIT))
		/* FFXXX: need multi-skb version to dump entire FF */
		ieee80211_dump_pkt(ic, skb->data, skb->len,
			sc->sc_hwmap[txrate].ieeerate, -1);

	/*
	 * Determine if a tx interrupt should be generated for
	 * this descriptor.  We take a tx interrupt to reap
	 * descriptors when the h/w hits an EOL condition or
	 * when the descriptor is specifically marked to generate
	 * an interrupt.  We periodically mark descriptors in this
	 * way to ensure timely replenishing of the supply needed
	 * for sending frames.  Deferring interrupts reduces system
	 * load and potentially allows more concurrent work to be
	 * done, but if done too aggressively, it can cause senders
	 * to backup.
	 *
	 * NB: use >= to deal with sc_txintrperiod changing
	 *     dynamically through sysctl.
	 */
	if (!M_FLAG_GET(skb, M_UAPSD) &&
		++txq->axq_intrcnt >= sc->sc_txintrperiod) {
		flags |= AR5K_TXDESC_INTREQ;
		txq->axq_intrcnt = 0;
	}

#ifdef ATH_SUPERG_COMP
	if (ATH_NODE(ni)->an_decomp_index != INVALID_DECOMP_INDEX && 
	    !ismcast &&
	    ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) &&
	    ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) != IEEE80211_FC0_SUBTYPE_NODATA)) {
		if (pktlen > ATH_COMP_THRESHOLD)
			comp = ATH_COMP_PROC_COMP_OPTIMAL;
		else
			comp = ATH_COMP_PROC_NO_COMP_ADD_CCS;
	} 
#endif

	/*
	 * sc_txantenna == 0 means transmit diversity mode.
	 * sc_txantenna == 1 or sc_txantenna == 2 means the user has selected
	 * the first or second antenna port.
	 * If the user has set the txantenna, use it for multicast frames too.
	 */
	if (ismcast && !sc->sc_txantenna) {
		antenna = sc->sc_mcastantenna + 1;
		sc->sc_mcastantenna = (sc->sc_mcastantenna + 1) & 0x1;
	} else
		antenna = sc->sc_txantenna;

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	/* XXX check return value? */
	ath_hal_setuptxdesc(ah, ds
			    , pktlen		/* packet length */
			    , hdrlen		/* header length */
			    , atype		/* Atheros packet type */
			    , MIN(ni->ni_txpower, 60)/* txpower */
			    , txrate, try0	/* series 0 rate/tries */
			    , keyix		/* key cache index */
			    , antenna		/* antenna mode */
			    , flags		/* flags */
			    , ctsrate		/* rts/cts rate */
			    , ctsduration	/* rts/cts duration */
//			    , icvlen		/* comp icv len */
//			    , ivlen		/* comp iv len */
//			    , comp		/* comp scheme */
		);
	bf->bf_flags = flags;			/* record for post-processing */

	/*
	 * Setup the multi-rate retry state only when we're
	 * going to use it.  This assumes ath_hal_setuptxdesc
	 * initializes the descriptors (so we don't have to)
	 * when the hardware supports multi-rate retry and
	 * we don't use it.
	 */
	if (try0 != ATH_TXMAXTRY)
		ath_rate_setupxtxdesc(sc, an, ds, shortPreamble, skb->len, rix);

#ifndef ATH_SUPERG_FF
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;

	ath_hal_filltxdesc(ah, ds
			   , skb->len	/* segment length */
			   , TRUE	/* first segment */
			   , TRUE	/* last segment */
			   , ds		/* first descriptor */
		);

	/* NB: The desc swap function becomes void, 
	 * if descriptor swapping is not enabled
	 */
	ath_desc_swap(ds);

	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
	    __func__, M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum, ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
#else /* ATH_SUPERG_FF */
	{
		struct sk_buff *skbtmp = skb;
		struct ath_desc *ds0 = ds;
		int i;

		ds->ds_data = bf->bf_skbaddr;
		ds->ds_link = (skb->next == NULL) ? 0 : bf->bf_daddr + sizeof(*ds);

		ath_hal_filltxdesc(ah, ds
			, skbtmp->len		/* segment length */
			, TRUE		/* first segment */
			, skbtmp->next == NULL	/* last segment */
			, ds			/* first descriptor */
		);

		/* NB: The desc swap function becomes void, 
		 * if descriptor swapping is not enabled
		 */
		ath_desc_swap(ds);

		DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: (ds)%p (lk)%08x (d)%08x (c0)%08x (c1)%08x %08x %08x\n",
			__func__, M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum,
			ds, ds->ds_link, ds->ds_data, ds->ds_ctl0, ds->ds_ctl1,
			ds->ds_hw[0], ds->ds_hw[1]);
		for (i= 0, skbtmp = skbtmp->next; i < bf->bf_numdesc - 1; i++, skbtmp = skbtmp->next) {
			ds++;
			ds->ds_link = skbtmp->next == NULL ? 0 : bf->bf_daddr + sizeof(*ds) * (i + 2);
			ds->ds_data = bf->bf_skbaddrff[i];
			ath_hal_filltxdesc(ah, ds
				, skbtmp->len		/* segment length */
				, FALSE		/* first segment */
				, skbtmp->next == NULL	/* last segment */
				, ds0			/* first descriptor */
			);

			/* NB: The desc swap function becomes void, 
		 	 * if descriptor swapping is not enabled
		 	 */
			ath_desc_swap(ds);

			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
				__func__, M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum,
				ds->ds_link, ds->ds_data, ds->ds_ctl0,
				ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
		}
	}
#endif

	if (M_FLAG_GET(skb, M_UAPSD)) {
		/* must lock against interrupt-time processing (i.e., not just tasklet) */
		ATH_NODE_UAPSD_LOCK_IRQ(an);
		DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: Qing U-APSD data frame for node %s \n", 
			__func__, ether_sprintf(an->an_node.ni_macaddr));
		ath_tx_uapsdqueue(sc, an, bf);
		if (IEEE80211_NODE_UAPSD_USETIM(ni) && (an->an_uapsd_qdepth == 1))
			vap->iv_set_tim(ni, 1);
		ATH_NODE_UAPSD_UNLOCK_IRQ(an);

		return 0;
	}
	
	
	IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE, "%s: %p<%s> refcnt %d\n",
		__func__, vap->iv_bss, ether_sprintf(vap->iv_bss->ni_macaddr),
		ieee80211_node_refcnt(vap->iv_bss));


	ath_tx_txqaddbuf(sc, ni, txq, bf, ds, pktlen);
	return 0;
#undef MIN
}
#endif

/*
 * Process completed xmit descriptors from the specified queue.
 * Should only be called from tasklet context
 */
static void
ath_tx_processq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf = NULL;
	struct ath_desc *ds = NULL;
	struct ieee80211_tx_status txstatus;
	int header_len, header_pad;
	struct sk_buff *skb;
	AR5K_STATUS status;
#if 0
	int uapsdq = 0;
	unsigned long uapsdq_lockflags = 0;
#endif

	DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: tx queue %d (0x%x), link %p\n", __func__,
		txq->axq_qnum, ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum),
		txq->axq_link);

#if 0
	if (txq == sc->sc_uapsdq) {
		DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: reaping U-APSD txq\n", __func__);
		uapsdq = 1;
	}
#endif

	for (;;) {
#if 0
		if (uapsdq)
			ATH_TXQ_UAPSDQ_LOCK_IRQ(txq);
		else
#endif
			ATH_TXQ_LOCK(txq);
		txq->axq_intrcnt = 0; /* reset periodic desc intr count */
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
#if 0
			if (uapsdq)
				ATH_TXQ_UAPSDQ_UNLOCK_IRQ(txq);
			else
#endif
				ATH_TXQ_UNLOCK(txq);
			break;
		}

#ifdef ATH_SUPERG_FF
		ds = &bf->bf_desc[bf->bf_numdesc - 1];
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: frame's last desc: %p\n",
			__func__, ds);
#else
		ds = bf->bf_desc;		/* NB: last descriptor */
#endif
		status = ath_hal_txprocdesc(ah, ds);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_XMIT_DESC)
			ath_printtxbuf(bf, status == AR5K_OK);
#endif
		if (status == AR5K_EINPROGRESS) {
#if 0
			if (uapsdq)
				ATH_TXQ_UAPSDQ_UNLOCK_IRQ(txq);
			else
#endif
				ATH_TXQ_UNLOCK(txq);
			break;
		}

		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
#if 0
		if (uapsdq)
			ATH_TXQ_UAPSDQ_UNLOCK_IRQ(txq);
		else
#endif
			ATH_TXQ_UNLOCK(txq);

		memset(&txstatus, 0, sizeof(txstatus));
		skb = bf->bf_skb;

		txstatus.control = bf->control;

		if (ds->ds_txstat.ts_status == 0) {
			txstatus.flags |= IEEE80211_TX_STATUS_ACK;
			txstatus.ack_signal = ds->ds_txstat.ts_rssi;
		} else {

			if (ds->ds_txstat.ts_status & AR5K_TXERR_XRETRY) {
				txstatus.excessive_retries = 1;
			} else if (ds->ds_txstat.ts_status & AR5K_TXERR_FILT) {
				txstatus.flags |=
					IEEE80211_TX_STATUS_TX_FILTERED;
			}
		}

		txstatus.retry_count = ds->ds_txstat.ts_shortretry + 
				       ds->ds_txstat.ts_longretry;

		/* Remove possible alignment padding after 802.11 hdr to make
		 * frames look correct for upper level code. */
		header_len = ieee80211_get_hdrlen_from_skb(skb);
		header_pad = (4 - (header_len & 3)) & 3;
		if (unlikely(header_pad)) { 
			memmove(skb->data + header_pad, skb->data, header_len);
			skb_pull(skb, header_pad);
		}

		ieee80211_tx_status(sc->sc_hw, skb, &txstatus);
		bus_unmap_single(sc->sc_bdev, bf->bf_skbaddr, 
                                 bf->bf_skb->len, BUS_DMA_TODEVICE);
#if 0
		if (ni && uapsdq) {
			/* detect EOSP for this node */
			struct ieee80211_qosframe *qwh = (struct ieee80211_qosframe *)bf->bf_skb->data;
			an = ATH_NODE(ni);
			KASSERT(ni != NULL, ("Processing U-APSD txq for ath_buf with no node!\n"));
			if (qwh->i_qos[0] & IEEE80211_QOS_EOSP) {
				DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: EOSP detected for node (%s) on desc %p\n", 
					__func__, ether_sprintf(ni->ni_macaddr), ds);
				ATH_NODE_UAPSD_LOCK_IRQ(an);
				ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
				if (an->an_uapsd_qdepth == 0 && an->an_uapsd_overflowqdepth != 0) {
					STAILQ_CONCAT(&an->an_uapsd_q, &an->an_uapsd_overflowq);
					an->an_uapsd_qdepth = an->an_uapsd_overflowqdepth;
					an->an_uapsd_overflowqdepth = 0;
				}
				ATH_NODE_UAPSD_UNLOCK_IRQ(an);
			}
		}
#endif

#if 0 /* FIXME: See change 1768 */
		{
			struct ieee80211_frame *wh = (struct ieee80211_frame *)bf->bf_skb->data;
			if ((ds->ds_txstat.ts_seqnum << IEEE80211_SEQ_SEQ_SHIFT) & ~IEEE80211_SEQ_SEQ_MASK) {
				DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: h/w assigned sequence number is not sane (%d), ignoring it\n", __func__,
				        ds->ds_txstat.ts_seqnum);
			} else {
				DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: updating frame's sequence number from %d to %d\n", __func__, 
				        (le16toh(*(__le16 *)&wh->i_seq[0]) & IEEE80211_SEQ_SEQ_MASK) >> IEEE80211_SEQ_SEQ_SHIFT,
				        ds->ds_txstat.ts_seqnum);

				*(__le16 *)&wh->i_seq[0] = htole16(
					ds->ds_txstat.ts_seqnum << IEEE80211_SEQ_SEQ_SHIFT |
					(le16toh(*(__le16 *)&wh->i_seq[0]) & ~IEEE80211_SEQ_SEQ_MASK));
			}
		}
#endif
#ifdef ATH_SUPERG_FF
		{
			struct sk_buff *skbfree, *skb = bf->bf_skb;
			int i;

			skbfree = skb;
			skb = skb->next;
			DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: free skb %p\n",
				__func__, skbfree);
			ath_tx_capture(sc->sc_dev, ds, skbfree);
			for (i = 1; i < bf->bf_numdesc; i++) {
				bus_unmap_single(sc->sc_bdev, bf->bf_skbaddrff[i-1],
					bf->bf_skb->len, BUS_DMA_TODEVICE);
				skbfree = skb;
				skb = skb->next;
				DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: free skb %p\n",
					__func__, skbfree);
				ath_tx_capture(sc->sc_dev, ds, skbfree);
			}
		}
		bf->bf_numdesc = 0;
#else
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: free skb %p\n", __func__, bf->bf_skb);
#endif
		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		ATH_TXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		if (sc->sc_devstopped) {
			++sc->sc_reapcount;
			if (sc->sc_reapcount > ATH_TXBUF_FREE_THRESHOLD) {
				if (!sc->sc_dfswait)
					ieee80211_start_queues(sc->sc_hw);
				DPRINTF(sc, ATH_DEBUG_TX_PROC,
					"%s: tx tasklet restart the queue\n",
					__func__);
				sc->sc_reapcount = 0;
				sc->sc_devstopped = 0;
			} else
				ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, NULL);
		}
		ATH_TXBUF_UNLOCK_IRQ(sc);
	}
#ifdef ATH_SUPERG_FF
	/* flush ff staging queue if buffer low */
	if (txq->axq_depth <= sc->sc_fftxqmin - 1) {
		/* NB: consider only flushing a preset number based on age. */
		ath_ffstageq_flush(sc, txq, ath_ff_neverflushtestdone);
	}
#endif /* ATH_SUPERG_FF */
}

static __inline int
txqactive(struct ath_hal *ah, int qnum)
{
	u_int32_t txqs = 1 << qnum;
	ath_hal_gettxintrtxqs(ah, &txqs);
	return (txqs & (1 << qnum));
}

/*
 * Deferred processing of transmit interrupt; special-cased
 * for a single hardware transmit queue (e.g. 5210 and 5211).
 */
static void
ath_tx_tasklet_q0(TQUEUE_ARG data)
{
	struct ath_softc *sc = (struct ath_softc *)data;

	if (txqactive(sc->sc_ah, 0))
		ath_tx_processq(sc, &sc->sc_txq[0]);
	if (txqactive(sc->sc_ah, sc->sc_cabq->axq_qnum))
		ath_tx_processq(sc, sc->sc_cabq);

	ieee80211_wake_queue(sc->sc_hw, 0);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

#if 0
/*
 * Deferred processing of transmit interrupt; special-cased
 * for four hardware queues, 0-3 (e.g. 5212 w/ WME support).
 */
static void
ath_tx_tasklet_q0123(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	/*
	 * Process each active queue.
	 */
	if (txqactive(sc->sc_ah, 0))
		ath_tx_processq(sc, &sc->sc_txq[0]);
	if (txqactive(sc->sc_ah, 1))
		ath_tx_processq(sc, &sc->sc_txq[1]);
	if (txqactive(sc->sc_ah, 2))
		ath_tx_processq(sc, &sc->sc_txq[2]);
	if (txqactive(sc->sc_ah, 3))
		ath_tx_processq(sc, &sc->sc_txq[3]);
	if (txqactive(sc->sc_ah, sc->sc_cabq->axq_qnum))
		ath_tx_processq(sc, sc->sc_cabq);
#ifdef ATH_SUPERG_XR
	if (sc->sc_xrtxq && txqactive(sc->sc_ah, sc->sc_xrtxq->axq_qnum))
		ath_tx_processq(sc, sc->sc_xrtxq);
#endif
	if (sc->sc_uapsdq && txqactive(sc->sc_ah, sc->sc_uapsdq->axq_qnum))
		ath_tx_processq(sc, sc->sc_uapsdq);

	netif_wake_queue(dev);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

/*
 * Deferred processing of transmit interrupt.
 */
static void
ath_tx_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	int i;

	/*
	 * Process each active queue.
	 */
	for (i = 0; i < AR5K_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i) && txqactive(sc->sc_ah, i))
			ath_tx_processq(sc, &sc->sc_txq[i]);
#ifdef ATH_SUPERG_XR
	if (sc->sc_xrtxq && txqactive(sc->sc_ah, sc->sc_xrtxq->axq_qnum))
		ath_tx_processq(sc, sc->sc_xrtxq);
#endif

	netif_wake_queue(dev);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

static void
ath_tx_timeout(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_WATCHDOG, "%s: %sRUNNING %svalid\n",
		__func__, (dev->flags & IFF_RUNNING) ? "" : "!",
		sc->sc_invalid ? "in" : "");

	if ((dev->flags & IFF_RUNNING) && !sc->sc_invalid) {
		sc->sc_stats.ast_watchdog++;
		ath_reset(dev);	/* Avoid taking a semaphore in ath_init */
	}
}
#endif

/* 
 * Context: softIRQ and hwIRQ
 */
static void
ath_tx_draintxq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct sk_buff *skb;
#ifdef ATH_SUPERG_FF
	struct sk_buff *tskb;
#endif
	int i;

	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath_tx_tasklet
	 */
	for (;;) {
		ATH_TXQ_LOCK(txq);
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK(txq);
			break;
		}
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK(txq);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RESET)
			ath_printtxbuf(bf, ath_hal_txprocdesc(ah, bf->bf_desc) == AR5K_OK);
#endif /* AR_DEBUG */
		skb = bf->bf_skb->next;
		bus_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);
		dev_kfree_skb_any(bf->bf_skb);
		i = 0;
#ifdef ATH_SUPERG_FF
		while (skb) {
			struct sk_buff *tskb;
			tskb = skb->next;
			bus_unmap_single(sc->sc_bdev,
				 bf->bf_skbaddrff[i++], skb->len, BUS_DMA_TODEVICE);
			dev_kfree_skb_any(skb);
			skb = tskb;
		}
#endif /* ATH_SUPERG_FF */
#if 0
		if (bf->bf_node)
			ieee80211_free_node(bf->bf_node);
#endif

		bf->bf_skb = NULL;
		bf->bf_node = NULL;

		ATH_TXBUF_LOCK(sc);
		STAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK(sc);
	}
}

static void
ath_tx_stopdma(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;

	(void) ath_hal_stoptxdma(ah, txq->axq_qnum);
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: tx queue [%u] 0x%x, link %p\n",
		__func__, txq->axq_qnum,
		ath_hal_gettxbuf(ah, txq->axq_qnum), txq->axq_link);
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
		DPRINTF(sc, ATH_DEBUG_RESET, "%s: beacon queue 0x%x\n",
			__func__, ath_hal_gettxbuf(ah, sc->sc_bhalq));
		for (i = 0; i < AR5K_NUM_TX_QUEUES; i++)
			if (ATH_TXQ_SETUP(sc, i))
				ath_tx_stopdma(sc, &sc->sc_txq[i]);
	}
	ieee80211_start_queues(sc->sc_hw);		/* XXX move to callers */
	for (i = 0; i < AR5K_NUM_TX_QUEUES; i++)
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
	((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc + \
		((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))
	struct ath_hal *ah = sc->sc_ah;
	u_int64_t tsf;

	ath_hal_stoppcurecv(ah);	/* disable PCU */
	ath_hal_setrxfilter(ah, 0);	/* clear recv filter */
	ath_hal_stopdmarecv(ah);	/* disable DMA engine */
	mdelay(3);			/* 3 ms is long enough for 1 frame */
	tsf = ath_hal_gettsf64(ah);
#ifdef AR_DEBUG
	if (sc->sc_debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL)) {
		struct ath_buf *bf;

		printk("ath_stoprecv: rx queue 0x%x, link %p\n",
			ath_hal_getrxbuf(ah), sc->sc_rxlink);
		STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			struct ath_desc *ds = bf->bf_desc;
			AR5K_STATUS status = ath_hal_rxprocdesc(ah, ds,
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
	struct ath_buf *bf;

	/*
	 * Cisco's VPN software requires that drivers be able to
	 * receive encapsulated frames that are larger than the MTU.
	 * Since we can't be sure how large a frame we'll get, setup
	 * to handle the larges on possible.
	 */
#ifdef ATH_SUPERG_FF
	sc->sc_rxbufsize = roundup(ATH_FF_MAX_LEN, sc->sc_cachelsz);
#else
	sc->sc_rxbufsize = roundup(IEEE80211_MAX_LEN, sc->sc_cachelsz);
#endif
	DPRINTF(sc,ATH_DEBUG_RESET, "%s: cachelsz %u rxbufsize %u\n",
		__func__, sc->sc_cachelsz, sc->sc_rxbufsize);

	sc->sc_rxlink = NULL;
	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		ATH_RXBUF_RESET(bf);
		if (error < 0)
			return error;
	}

	sc->sc_rxbufcur = NULL;

	bf = STAILQ_FIRST(&sc->sc_rxbuf);
	ath_hal_putrxbuf(ah, bf->bf_daddr);
	ath_hal_rxena(ah);		/* enable recv descriptors */
	ath_mode_init(sc);		/* set filters, etc. */
	ath_hal_startpcurecv(ah);	/* re-enable PCU/DMA engine */
	return 0;
}

/*
 * Flush skb's allocate for receive.
 */
static void
ath_flushrecv(struct ath_softc *sc)
{
	struct ath_buf *bf;

	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
		if (bf->bf_skb != NULL) {
			bus_unmap_single(sc->sc_bdev,
				bf->bf_skbaddr, sc->sc_rxbufsize,
				BUS_DMA_FROMDEVICE);
			dev_kfree_skb(bf->bf_skb);
			bf->bf_skb = NULL;
		}
}


static u_int
hal_chan2mode(AR5K_CHANNEL *chan)
{
	if ((chan->channel_flags & CHANNEL_A) == CHANNEL_A)
		return AR5K_MODE_11A;
	else if ((chan->channel_flags & CHANNEL_B) == CHANNEL_B)
		return AR5K_MODE_11B;
	else if ((chan->channel_flags & CHANNEL_G) == CHANNEL_G)
		return AR5K_MODE_11G;
	else if ((chan->channel_flags & CHANNEL_108G) == CHANNEL_108G)
		return AR5K_MODE_108G;
	else if ((chan->channel_flags & CHANNEL_108A) == CHANNEL_108A)
		return AR5K_MODE_TURBO;

	printk(KERN_ERR "Unknown mode for channel flags %x\n",
		     chan->channel_flags);
	return AR5K_MODE_11B;
}

static struct {
	u_int	hal_mode;	/* hal phy mode */
	int	index;		/* index in sc_rates[] */
} ath_mode_map[] = {
	{ AR5K_MODE_11A,  0 },
	{ AR5K_MODE_11B,  1 },
	{ AR5K_MODE_11G,  2 },
	{ AR5K_MODE_TURBO, 3 },
	{ AR5K_MODE_108G, 4 },
};

static int
ath_mode_to_idx(u_int hal_mode)
{
	int i;

	BUG_ON(sizeof(ath_mode_map) / sizeof(ath_mode_map[0]) > AR5K_MAX_MODES);

	for (i = 0; i < sizeof(ath_mode_map) / sizeof(ath_mode_map[0]); i++) {
		if (ath_mode_map[i].hal_mode == hal_mode)
			return ath_mode_map[i].index;	
	}
	printk(KERN_ERR "Invalid mode.\n");
	return ath_mode_map[0].index;
}


/*
 * Update internal state after a channel change.
 */
static void
ath_chan_change(struct ath_softc *sc, AR5K_CHANNEL *chan)
{
	u_int mode;

	mode = hal_chan2mode(chan);

	ath_rate_setup(sc, mode);
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
#if 0
	if (ic->ic_curchanmaxpwr == 0)
		ic->ic_curchanmaxpwr = chan->ic_maxregpower;
#endif
}

/*
 * Set/change channels.  If the channel is really being changed,
 * it's done by resetting the chip.  To accomplish this we must
 * first cleanup any pending DMA, then restart stuff after a la
 * ath_init.
 */
int
ath_chan_set(struct ath_softc *sc, AR5K_CHANNEL hchan)
{
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %u (%u MHz) -> %u (%u MHz)\n",
		__func__, ath_hal_mhz2ieee(sc->sc_curchan.freq,
		sc->sc_curchan.channel_flags), sc->sc_curchan.freq,
		ath_hal_mhz2ieee(hchan.freq, hchan.channel_flags),
		hchan.freq);
#if 0
	/* check if it is turbo mode switch */
	if (hchan.freq == sc->sc_curchan.freq &&
	   (hchan.channel_flags & IEEE80211_CHAN_TURBO) != (sc->sc_curchan.channel_flags & IEEE80211_CHAN_TURBO)) 
		tswitch = 1;
#endif
	if (hchan.freq != sc->sc_curchan.freq ||
	    hchan.channel_flags != sc->sc_curchan.channel_flags) {
		AR5K_STATUS status;

		/*
		 * To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w.
		 */
		ath_hal_intrset(ah, 0);	/* disable interrupts */
		ath_draintxq(sc);	/* clear pending tx frames */
		ath_stoprecv(sc);	/* turn off frame recv */
		
#if 0		
		/* Set coverage class */
		if (sc->sc_scanning || !IEEE80211_IS_CHAN_A(chan))
			ath_hal_setcoverageclass(sc->sc_ah, 0, 0);
		else
			ath_hal_setcoverageclass(sc->sc_ah, ic->ic_coverageclass, 0);
#endif

		if (!ath_hal_reset(ah, sc->sc_opmode, &hchan, TRUE, &status)) {
			printk("%s: %s: unable to reset channel (%u Mhz) "
				"flags 0x%x '%s' (HAL status %u)\n",
				sc->name, __func__,
				hchan.freq,
			        hchan.channel_flags,
				ath_get_hal_status_desc(status), status);
			return -EIO;
		}

		if (sc->sc_softled)
			ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
		
		sc->sc_curchan = hchan;
		ath_update_txpow(sc);		/* update tx power state */

		/*
		 * Re-enable rx framework.
		 */
		if (ath_startrecv(sc) != 0) {
			printk("%s: %s: unable to restart recv logic\n",
				sc->name, __func__);
			return -EIO;
		}

		/*
		 * Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g.
		 */
		ath_chan_change(sc, &hchan);
#if 0
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			if (sc->sc_curchan.private_flags & CHANNEL_DFS) {
				if (!(sc->sc_curchan.private_flags & CHANNEL_DFS_CLEAR)) { 
					dev->watchdog_timeo = 120 * HZ;	/* set the timeout to normal */
					netif_stop_queue(dev);			
					if (sc->sc_dfswait)
						del_timer_sync(&sc->sc_dfswaittimer);
					DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: start dfs wait period\n",
						__func__, dev->name);
					sc->sc_dfswait = 1;
					sc->sc_dfswaittimer.function = ath_check_dfs_clear;
					sc->sc_dfswaittimer.expires = 
						jiffies + (ATH_DFS_WAIT_POLL_PERIOD * HZ);
					sc->sc_dfswaittimer.data = (unsigned long)sc;
					add_timer(&sc->sc_dfswaittimer);
				}
			} else
				if (sc->sc_dfswait == 1)
					mod_timer(&sc->sc_dfswaittimer, jiffies + 2);
		}
		/*
		 * re configure beacons when it is a turbo mode switch.
		 * HW seems to turn off beacons during turbo mode switch.
		 */
		if (sc->sc_beacons && tswitch) 
			ath_beacon_config(sc, NULL);	
#endif

		/*
		 * Re-enable interrupts.
		 */
		ath_hal_intrset(ah, sc->sc_imask);
	}
	return 0;
}

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
static void
ath_calibrate(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *)arg;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_BOOL isIQdone = FALSE;

	sc->sc_stats.ast_per_cal++;

	DPRINTF(sc, ATH_DEBUG_CALIBRATE, "%s: channel %u/%x\n",
		__func__, sc->sc_curchan.freq, sc->sc_curchan.channel_flags);

	if (ath_hal_getrfgain(ah) == AR5K_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		sc->sc_stats.ast_per_rfgain++;
		ath_reset(sc);
	}
	if (!ath_hal_calibrate(ah, &sc->sc_curchan)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: calibration of channel %u failed\n",
			__func__, sc->sc_curchan.freq);
		sc->sc_stats.ast_per_calfail++;
	}

	if (isIQdone == TRUE)
		ath_calinterval = ATH_LONG_CALINTERVAL;
	else
		ath_calinterval = ATH_SHORT_CALINTERVAL;

	sc->sc_cal_ch.expires = jiffies + (ath_calinterval * HZ);
	add_timer(&sc->sc_cal_ch);
}

#if 0
static void
ath_scan_start(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	/* XXX calibration timer? */

	sc->sc_scanning = 1;
	sc->sc_syncbeacon = 0;
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);
	ath_hal_setassocid(ah, dev->broadcast, 0);

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s aid 0\n",
		 __func__, rfilt, ether_sprintf(dev->broadcast));
}

static void
ath_scan_end(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	sc->sc_scanning = 0;
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);
	ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s aid 0x%x\n",
		 __func__, rfilt, ether_sprintf(sc->sc_curbssid),
		 sc->sc_curaid);
}

static void
ath_set_channel(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ieee80211_channel *chan;
	AR5K_CHANNEL hchan;

	chan = ic->ic_curchan;

	/*
	 * Convert to a HAL channel description with
	 * the flags constrained to reflect the current
	 * operating mode.
	 */
	hchan.freq = chan->ic_freq;
	hchan.channel_flags = ath_chan2flags(chan);
	KASSERT(hchan.freq != 0,
		("bogus channel %u/0x%x", hchan.freq, hchan.channel_flags));

	(void) ath_chan_set(sc, hchan);
	/*
	 * If we are returning to our bss channel then mark state
	 * so the next recv'd beacon's tsf will be used to sync the
	 * beacon timers.  Note that since we only hear beacons in
	 * sta/ibss mode this has no effect in other operating modes.
	 */
	if (!sc->sc_scanning && ic->ic_curchan == ic->ic_bsschan)
		sc->sc_syncbeacon = 1;
}

static void
ath_set_coverageclass(struct ieee80211com *ic) 
{
	struct ath_softc *sc = ic->ic_dev->priv;

	ath_hal_setcoverageclass(sc->sc_ah, ic->ic_coverageclass, 0);
	
	return;
}

static u_int
ath_mhz2ieee(struct ieee80211com *ic, u_int freq, u_int flags)
{
	struct ath_softc *sc = ic->ic_dev->priv;

 	return (ath_hal_mhz2ieee(freq, flags));
}


/*
 * Context: softIRQ and process context
 */
static int
ath_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct ath_vap *avp = ATH_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni, *wds_ni;
	int i, error, stamode;
	u_int32_t rfilt = 0;
	struct ieee80211vap *tmpvap;
	static const AR5K_LED_STATE leds[] = {
		AR5K_LED_INIT,	/* IEEE80211_S_INIT */
		AR5K_LED_SCAN,	/* IEEE80211_S_SCAN */
		AR5K_LED_AUTH,	/* IEEE80211_S_AUTH */
		AR5K_LED_ASSOC, 	/* IEEE80211_S_ASSOC */
		AR5K_LED_RUN, 	/* IEEE80211_S_RUN */
	};

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: %s -> %s\n", __func__, dev->name,
		ieee80211_state_name[vap->iv_state],
		ieee80211_state_name[nstate]);

	del_timer(&sc->sc_cal_ch);		/* periodic calibration timer */
	ath_hal_setledstate(ah, leds[nstate]);	/* set LED */
	netif_stop_queue(dev);			/* before we do anything else */

	if (nstate == IEEE80211_S_INIT) {
		/*
		 * if there is no VAP left in RUN state
		 * disable beacon interrupts.
		 */
		TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
			if (tmpvap != vap && tmpvap->iv_state == IEEE80211_S_RUN )
				break;
		}
		if (!tmpvap) {
			sc->sc_imask &= ~(AR5K_INT_SWBA | AR5K_INT_BMISS);
			/*
			 * Disable interrupts.
			 */
			ath_hal_intrset(ah, sc->sc_imask &~ AR5K_INT_GLOBAL);
			sc->sc_beacons = 0;
		}
		/*
		 * Notify the rate control algorithm.
		 */
		ath_rate_newstate(vap, nstate);
		goto done;
	}
	ni = vap->iv_bss;

	rfilt = ath_calcrxfilter(sc);
	stamode = (vap->iv_opmode == IEEE80211_M_STA ||
		   vap->iv_opmode == IEEE80211_M_IBSS ||
		   vap->iv_opmode == IEEE80211_M_AHDEMO);
	if (stamode && nstate == IEEE80211_S_RUN) {
		sc->sc_curaid = ni->ni_associd;
		IEEE80211_ADDR_COPY(sc->sc_curbssid, ni->ni_bssid);
	} else
		sc->sc_curaid = 0;

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s aid 0x%x\n",
		 __func__, rfilt, ether_sprintf(sc->sc_curbssid),
		 sc->sc_curaid);

	ath_hal_setrxfilter(ah, rfilt);
	if (stamode)
		ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);

	if ((vap->iv_opmode != IEEE80211_M_STA) &&
		 (vap->iv_flags & IEEE80211_F_PRIVACY)) {
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, ni->ni_bssid);
	}

	/*
	 * Notify the rate control algorithm so rates
	 * are setup should ath_beacon_alloc be called.
	 */
	ath_rate_newstate(vap, nstate);

	if (vap->iv_opmode == IEEE80211_M_MONITOR) {
		/* nothing to do */;
	} else if (nstate == IEEE80211_S_RUN) {
		DPRINTF(sc, ATH_DEBUG_STATE,
			"%s(RUN): ic_flags=0x%08x iv=%d bssid=%s "
			"capinfo=0x%04x chan=%d\n"
			 , __func__
			 , vap->iv_flags
			 , ni->ni_intval
			 , ether_sprintf(ni->ni_bssid)
			 , ni->ni_capinfo
			 , ieee80211_chan2ieee(ic, ni->ni_chan));

		switch (vap->iv_opmode) {
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
			ath_hal_stoptxdma(ah, sc->sc_bhalq);

        		/* Set default key index for static wep case */
			ni->ni_ath_defkeyindex = IEEE80211_INVAL_DEFKEY;
			if (((vap->iv_flags & IEEE80211_F_WPA) == 0) &&
			    (ni->ni_authmode != IEEE80211_AUTH_8021X) &&
			    (vap->iv_def_txkey != IEEE80211_KEYIX_NONE)) {
                       		ni->ni_ath_defkeyindex = vap->iv_def_txkey;
			}

			error = ath_beacon_alloc(sc, ni);
			if (error < 0)
				goto bad;
			/* 
			 * if the turbo flags have changed, then beacon and turbo
			 * need to be reconfigured.
			 */
			if ((sc->sc_dturbo && !(vap->iv_ath_cap & IEEE80211_ATHC_TURBOP)) || 
				(!sc->sc_dturbo && (vap->iv_ath_cap & IEEE80211_ATHC_TURBOP))) 
				sc->sc_beacons = 0;
			/* 
			 * if it is the first AP VAP moving to RUN state then beacon 
			 * needs to be reconfigured.
			 */
			TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
				if (tmpvap != vap && tmpvap->iv_state == IEEE80211_S_RUN &&
					tmpvap->iv_opmode == IEEE80211_M_HOSTAP)
					break;
			}
			if (!tmpvap) 
				sc->sc_beacons = 0;
			break;
		case IEEE80211_M_STA:
#ifdef ATH_SUPERG_COMP
			/* have we negotiated compression? */
			if (!(vap->iv_ath_cap & ni->ni_ath_flags & IEEE80211_NODE_COMP))
				ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;
#endif
			/*
			 * Allocate a key cache slot to the station.
			 */
			ath_setup_keycacheslot(sc, ni);
			/*
			 * Record negotiated dynamic turbo state for
			 * use by rate control modules.
			 */
			sc->sc_dturbo =
				(ni->ni_ath_flags & IEEE80211_ATHC_TURBOP) != 0;
			break;
		case IEEE80211_M_WDS:
			wds_ni = ieee80211_find_txnode(vap, vap->wds_mac);
			if (wds_ni) {
				/* XXX no rate negotiation; just dup */
				wds_ni->ni_rates = vap->iv_bss->ni_rates;
				/* Depending on the sequence of bringing up devices
				 * it's possible the rates of the root bss isn't
				 * filled yet. 
				 */
				if (vap->iv_ic->ic_newassoc != NULL && 
				    wds_ni->ni_rates.rs_nrates != 0) {
					/* Fill in the rates based on our own rates
					 * we rely on the rate selection mechanism
					 * to find out which rates actually work!
					 */
					vap->iv_ic->ic_newassoc(wds_ni, 1);
				}
			}
			break;
		default:
			break;
		}


		/*
		 * Configure the beacon and sleep timers.
		 */
		if (!sc->sc_beacons && vap->iv_opmode!=IEEE80211_M_WDS) {
			ath_beacon_config(sc, vap);
			sc->sc_beacons = 1;
		}

		/*
		 * Reset rssi stats; maybe not the best place...
		 */
		sc->sc_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
		sc->sc_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
		sc->sc_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
		/* 
		 * if it is a DFS channel and has not been checked for radar 
		 * do not let the 80211 state machine to go to RUN state.
		 *
		 */
		if (sc->sc_dfswait && vap->iv_opmode == IEEE80211_M_HOSTAP ) {
			/* push the VAP to RUN state once DFS is cleared */
			DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: VAP  -> DFS_WAIT\n",
				__func__, dev->name);
			avp->av_dfswait_run = 1; 
			return 0;
		}
	} else {
		if (sc->sc_dfswait &&
			vap->iv_opmode == IEEE80211_M_HOSTAP &&
			sc->sc_dfswaittimer.data == (unsigned long)vap) {
			del_timer_sync(&sc->sc_dfswaittimer);
			sc->sc_dfswait = 0;
			DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: VAP  out of DFS_WAIT\n",
				__func__, dev->name);
		}
		/*
		 *  XXXX
		 * if it is SCAN state, disable beacons. 
		 */
		if (nstate == IEEE80211_S_SCAN) {
			ath_hal_intrset(ah,sc->sc_imask &~ (AR5K_INT_SWBA | AR5K_INT_BMISS));
			sc->sc_imask &= ~(AR5K_INT_SWBA | AR5K_INT_BMISS);
			/* need to reconfigure the beacons when it moves to RUN */
			sc->sc_beacons = 0; 
		}
		avp->av_dfswait_run = 0; /* reset the dfs wait flag */ 
	}
done:
	/*
	 * Invoke the parent method to complete the work.
	 */
	error = avp->av_newstate(vap, nstate, arg);

	/*
	 * Finally, start any timers.
	 */
	if (nstate == IEEE80211_S_RUN) {
		/* start periodic recalibration timer */
		mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));
	}

#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR &&
		nstate == IEEE80211_S_RUN)
		ATH_SETUP_XR_VAP(sc,vap,rfilt);
	if (vap->iv_flags & IEEE80211_F_XR &&
		nstate == IEEE80211_S_INIT && sc->sc_xrgrppoll)
		ath_grppoll_stop(vap);
#endif
bad:
	netif_start_queue(dev);
	dev->watchdog_timeo = 5 * HZ;		/* set the timeout to normal */
	return error;
}

/*
 * periodically checks for the HAL to set
 * CHANNEL_DFS_CLEAR flag on current channel.
 * if the flag is set and a VAP is waiting for it, push 
 * transition the VAP to RUN state.
 *
 * Context: Timer (softIRQ)
 */
static void 
ath_check_dfs_clear(unsigned long data ) 
{
	struct ath_softc *sc = (struct ath_softc *)data; 
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = sc->sc_dev;
	struct ieee80211vap *vap ;
	AR5K_CHANNEL hchan;

	if(!sc->sc_dfswait) return;

	/* if still need to wait */
	ath_hal_radar_wait(sc->sc_ah, &hchan);

	if (hchan.private_flags & CHANNEL_INTERFERENCE)
		return; 

	if ((hchan.private_flags & CHANNEL_DFS_CLEAR) ||
	    (!(hchan.private_flags & CHANNEL_DFS))) { 
		sc->sc_curchan.private_flags |= CHANNEL_DFS_CLEAR;
		sc->sc_dfswait = 0;
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
			struct ath_vap *avp = ATH_VAP(vap);
			if (avp->av_dfswait_run) {
				/* re alloc beacons to update new channel info */
				int error;
				error = ath_beacon_alloc(sc, vap->iv_bss);
				if(error < 0) {
					/* XXX */
					return;
				}
				DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: VAP DFS_WAIT -> RUN\n",
					__func__, dev->name);
				avp->av_newstate(vap, IEEE80211_S_RUN, 0);
				/* start calibration timer */
				mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));
#ifdef ATH_SUPERG_XR
				if (vap->iv_flags & IEEE80211_F_XR ) {
					u_int32_t rfilt = 0;
					rfilt = ath_calcrxfilter(sc);
					ATH_SETUP_XR_VAP(sc, vap, rfilt);
				}
#endif
				avp->av_dfswait_run = 0;
			}
		}
		/* start the device */
		netif_start_queue(dev);
		dev->watchdog_timeo = 5 * HZ;	/* set the timeout to normal */
	} else {
		/* fire the timer again */
		sc->sc_dfswaittimer.expires = jiffies + (ATH_DFS_WAIT_POLL_PERIOD * HZ);
		sc->sc_dfswaittimer.data = (unsigned long)sc;
		add_timer(&sc->sc_dfswaittimer);
	}

}

#ifdef ATH_SUPERG_COMP
/* Enable/Disable de-compression mask for given node.
 * The routine is invoked after addition or deletion of the
 * key.
 */
static void
ath_comp_set(struct ieee80211vap *vap, struct ieee80211_node *ni, int en)
{
	ath_setup_comp(ni, en);
	return;
}

/* Set up decompression engine for this node. */
static void
ath_setup_comp(struct ieee80211_node *ni, int enable)
{
#define	IEEE80211_KEY_XR	(IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	struct ath_node *an = ATH_NODE(ni);
	u_int16_t keyindex;

	if (enable) {
		/* Have we negotiated compression? */
		if (!(ni->ni_ath_flags & IEEE80211_NODE_COMP))
			return;

		/* No valid key? */
		if (ni->ni_ucastkey.wk_keyix == IEEE80211_KEYIX_NONE)
			return;

		/* Setup decompression mask.
		 * For TKIP and split MIC case, recv. keyindex is at 32 offset
		 * from tx key.
		 */
		if ((ni->ni_wpa_ie != NULL) &&
		    (ni->ni_rsn.rsn_ucastcipher == IEEE80211_CIPHER_TKIP) &&
		    sc->sc_splitmic) {
			if ((ni->ni_ucastkey.wk_flags & IEEE80211_KEY_XR) 
							== IEEE80211_KEY_XR)
				keyindex = ni->ni_ucastkey.wk_keyix + 32;
			else
				keyindex = ni->ni_ucastkey.wk_keyix;
		} else
			keyindex = ni->ni_ucastkey.wk_keyix + ni->ni_rxkeyoff;

		ath_hal_setdecompmask(sc->sc_ah, keyindex, 1);
		an->an_decomp_index = keyindex;
	} else {
		if (an->an_decomp_index != INVALID_DECOMP_INDEX) {
			ath_hal_setdecompmask(sc->sc_ah, an->an_decomp_index, 0);
			an->an_decomp_index = INVALID_DECOMP_INDEX;
		}
	}

	return;
#undef IEEE80211_KEY_XR
}
#endif

/*
 * Allocate a key cache slot to the station so we can
 * setup a mapping from key index to node. The key cache
 * slot is needed for managing antenna state and for
 * compression when stations do not use crypto.  We do
 * it unilaterally here; if crypto is employed this slot
 * will be reassigned.
 */
static void
ath_setup_stationkey(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	u_int16_t keyix;

	keyix = ath_key_alloc(vap, &ni->ni_ucastkey);
	if (keyix == IEEE80211_KEYIX_NONE) {
		/*
		 * Key cache is full; we'll fall back to doing
		 * the more expensive lookup in software.  Note
		 * this also means no h/w compression.
		 */
		/* XXX msg+statistic */
		return;
	} else {
		ni->ni_ucastkey.wk_keyix = keyix;
		/* NB: this will create a pass-thru key entry */
		ath_keyset(sc, &ni->ni_ucastkey, ni->ni_macaddr, vap->iv_bss);

#ifdef ATH_SUPERG_COMP
		/* Enable de-compression logic */
		ath_setup_comp(ni, 1);
#endif
	}
	
	return;
}

/* Setup WEP key for the station if compression is negotiated.
 * When station and AP are using same default key index, use single key
 * cache entry for receive and transmit, else two key cache entries are
 * created. One for receive with MAC address of station and one for transmit
 * with NULL mac address. On receive key cache entry de-compression mask
 * is enabled.
 */
static void
ath_setup_stationwepkey(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_key *ni_key;
	struct ieee80211_key tmpkey;
	struct ieee80211_key *rcv_key, *xmit_key;
	int txkeyidx, rxkeyidx = IEEE80211_KEYIX_NONE, i;
	u_int8_t null_macaddr[IEEE80211_ADDR_LEN] = {0, 0, 0, 0, 0, 0};

	KASSERT(ni->ni_ath_defkeyindex < IEEE80211_WEP_NKID,
		("got invalid node key index 0x%x", ni->ni_ath_defkeyindex));
	KASSERT(vap->iv_def_txkey < IEEE80211_WEP_NKID,
		("got invalid vap def key index 0x%x", vap->iv_def_txkey));

	/* Allocate a key slot first */
	if (!ieee80211_crypto_newkey(vap, 
		IEEE80211_CIPHER_WEP, 
		IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV, 
		&ni->ni_ucastkey))
		goto error;

	txkeyidx = ni->ni_ucastkey.wk_keyix;
	xmit_key = &vap->iv_nw_keys[vap->iv_def_txkey];

	/* Do we need separate rx key? */
	if (ni->ni_ath_defkeyindex != vap->iv_def_txkey) {
		ni->ni_ucastkey.wk_keyix = IEEE80211_KEYIX_NONE;
		if (!ieee80211_crypto_newkey(vap, 
			IEEE80211_CIPHER_WEP, 
			IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV,
			&ni->ni_ucastkey)) {
			ni->ni_ucastkey.wk_keyix = txkeyidx;
			ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);
			goto error;
		}
		rxkeyidx = ni->ni_ucastkey.wk_keyix;
		ni->ni_ucastkey.wk_keyix = txkeyidx;

		rcv_key = &vap->iv_nw_keys[ni->ni_ath_defkeyindex];
	} else {
		rcv_key = xmit_key;
		rxkeyidx = txkeyidx;
	}

	/* Remember receive key offset */
	ni->ni_rxkeyoff = rxkeyidx - txkeyidx;

	/* Setup xmit key */
	ni_key = &ni->ni_ucastkey;
	if (rxkeyidx != txkeyidx)
		ni_key->wk_flags = IEEE80211_KEY_XMIT;
	else
		ni_key->wk_flags = IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV;

	ni_key->wk_keylen = xmit_key->wk_keylen;
	for (i = 0; i < IEEE80211_TID_SIZE; i++)
		ni_key->wk_keyrsc[i] = xmit_key->wk_keyrsc[i];
	ni_key->wk_keytsc = 0; 
	memset(ni_key->wk_key, 0, sizeof(ni_key->wk_key));
	memcpy(ni_key->wk_key, xmit_key->wk_key, xmit_key->wk_keylen);
	ieee80211_crypto_setkey(vap, &ni->ni_ucastkey, 
		(rxkeyidx == txkeyidx) ? ni->ni_macaddr:null_macaddr, ni);

	if (rxkeyidx != txkeyidx) {
		/* Setup recv key */
		ni_key = &tmpkey;
		ni_key->wk_keyix = rxkeyidx;
		ni_key->wk_flags = IEEE80211_KEY_RECV;
		ni_key->wk_keylen = rcv_key->wk_keylen;
		for(i = 0; i < IEEE80211_TID_SIZE; i++)
			ni_key->wk_keyrsc[i] = rcv_key->wk_keyrsc[i];
		ni_key->wk_keytsc = 0; 
		ni_key->wk_cipher = rcv_key->wk_cipher; 
		ni_key->wk_private = rcv_key->wk_private; 
		memset(ni_key->wk_key, 0, sizeof(ni_key->wk_key));
		memcpy(ni_key->wk_key, rcv_key->wk_key, rcv_key->wk_keylen);
		ieee80211_crypto_setkey(vap, &tmpkey, ni->ni_macaddr, ni);
	}

	return;

error:
	ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;
	return;
}

/* Create a keycache entry for given node in clearcase as well as static wep.
 * Handle compression state if required.
 * For non clearcase/static wep case, the key is plumbed by hostapd.
 */
static void
ath_setup_keycacheslot(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;

	if (ni->ni_ucastkey.wk_keyix != IEEE80211_KEYIX_NONE)
		ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);

	/* Only for clearcase and WEP case */
	if ((vap->iv_flags & IEEE80211_F_PRIVACY) == 0 ||
		(ni->ni_ath_defkeyindex != IEEE80211_INVAL_DEFKEY)) {

		if ((vap->iv_flags & IEEE80211_F_PRIVACY) == 0) {
			KASSERT(ni->ni_ucastkey.wk_keyix == IEEE80211_KEYIX_NONE,
		    		("new node with a ucast key already setup (keyix %u)",
		    		  ni->ni_ucastkey.wk_keyix));
			/* NB: 5210 has no passthru/clr key support */
			if (sc->sc_hasclrkey)
				ath_setup_stationkey(ni);
		} else
			ath_setup_stationwepkey(ni);
	}

	return;
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
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_softc *sc = ic->ic_dev->priv;

	ath_rate_newassoc(sc, ATH_NODE(ni), isnew);

	/* are we supporting compression? */
	if (!(vap->iv_ath_cap & ni->ni_ath_flags & IEEE80211_NODE_COMP))
		ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;

	/* disable compression for TKIP */
	if ((ni->ni_ath_flags & IEEE80211_NODE_COMP) &&
		(ni->ni_wpa_ie != NULL) &&
		(ni->ni_rsn.rsn_ucastcipher == IEEE80211_CIPHER_TKIP))
		ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;

	ath_setup_keycacheslot(sc, ni);
#ifdef ATH_SUPERG_XR
	if (1) {
		struct ath_node *an = ATH_NODE(ni);
		if (ic->ic_ath_cap & an->an_node.ni_ath_flags & IEEE80211_ATHC_XR)
			an->an_minffrate = ATH_MIN_FF_RATE;
		else
			an->an_minffrate = 0;
		ath_grppoll_period_update(sc);
	}
#endif
}
#endif

static int
ath_getchannels(struct ath_softc *sc, u_int cc,
	AR5K_BOOL outdoor, AR5K_BOOL xchanmode)
{
//	u_int8_t *regclassids = NULL;
//	u_int maxregclassids = 0;
//	u_int *nregclass = NULL;
	struct ath_hal *ah = sc->sc_ah;
	AR5K_CHANNEL *chans;
	u_int nchan;

	chans = kmalloc(IEEE80211_CHAN_MAX * sizeof(AR5K_CHANNEL), GFP_KERNEL);
	if (chans == NULL) {
		printk("%s: unable to allocate channel table\n", sc->name);
		return -ENOMEM;
	}
	if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, &nchan,
	    cc, AR5K_MODE_ALL, outdoor, xchanmode)) {
		u_int32_t rd;

		ath_hal_getregdomain(ah, &rd);
		printk("%s: unable to collect channel list from HAL; "
			"regdomain likely %u country code %u\n",
			sc->name, rd, cc);
		kfree(chans);
		return -EINVAL;
	}
	ath_d80211_add_channels(sc, MODE_IEEE80211A, chans, nchan, CHANNEL_A);
	ath_d80211_add_channels(sc, MODE_IEEE80211B, chans, nchan, CHANNEL_B);
	ath_d80211_add_channels(sc, MODE_IEEE80211G, chans, nchan, CHANNEL_G);
#if 0
	/* FIXME: hostapd does not support turbo modes. */
	ath_d80211_add_channels(dev, MODE_ATHEROS_TURBO, chans, nchan, CHANNEL_TURBO);
	ath_d80211_add_channels(dev, MODE_ATHEROS_TURBOG, chans, nchan, CHANNEL_108G);
#endif
	kfree(chans);
	return 0;
}

static void
ath_led_done(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *) arg;

	sc->sc_blinking = 0;
}

/*
 * Turn the LED off: flip the pin and then set a timer so no
 * update will happen for the specified duration.
 */
static void
ath_led_off(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *) arg;

	ath_hal_gpioset(sc->sc_ah, sc->sc_ledpin, !sc->sc_ledon);
	sc->sc_ledtimer.function = ath_led_done;
	sc->sc_ledtimer.expires = jiffies + sc->sc_ledoff;
	add_timer(&sc->sc_ledtimer);
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
	sc->sc_ledoff = off;
	sc->sc_ledtimer.function = ath_led_off;
	sc->sc_ledtimer.expires = jiffies + on;
	add_timer(&sc->sc_ledtimer);
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

#if 0
static void
set_node_txpower(void *arg, struct ieee80211_node *ni)
{
	int *value = (int *)arg;
	ni->ni_txpower = *value;
}
#endif

/* XXX: this function needs some locking to avoid being called twice/interrupted */
static void
ath_update_txpow(struct ath_softc *sc)
{
#if 0
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = NULL;
#endif
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t txpowlimit = 0;
	u_int32_t maxtxpowlimit = 9999;
#if 0
	u_int32_t clamped_txpow = 0;
#endif

	/*
	 * Find the maxtxpow of the card and regulatory constraints
	 */
	(void)ath_hal_getmaxtxpow(ah, &txpowlimit);
	ath_hal_settxpowlimit(ah, maxtxpowlimit);
	(void)ath_hal_getmaxtxpow(ah, &maxtxpowlimit);
#if 0
	ic->ic_txpowlimit = maxtxpowlimit;
#endif
	ath_hal_settxpowlimit(ah, txpowlimit);
 	
#if 0
 	/*
	 * Make sure the VAP's change is within limits, clamp it otherwise
 	 */
	if (ic->ic_newtxpowlimit > ic->ic_txpowlimit)
		clamped_txpow = ic->ic_txpowlimit;
	else
		clamped_txpow = ic->ic_newtxpowlimit;
	
	/*
	 * Search for the VAP that needs a txpow change, if any
	 */
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
#ifdef ATH_CAP_TPC
		if (ic->ic_newtxpowlimit == vap->iv_bss->ni_txpower) {
			vap->iv_bss->ni_txpower = clamped_txpow;
			ieee80211_iterate_nodes(&vap->iv_ic->ic_sta, set_node_txpower, &clamped_txpow);
		}
#else
		vap->iv_bss->ni_txpower = clamped_txpow;
		ieee80211_iterate_nodes(&vap->iv_ic->ic_sta, set_node_txpower, &clamped_txpow);
#endif
	}
 	
	ic->ic_newtxpowlimit = sc->sc_curtxpow = clamped_txpow;

#ifdef ATH_CAP_TPC
	if (ic->ic_newtxpowlimit >= ic->ic_txpowlimit)
		ath_hal_settxpowlimit(ah, ic->ic_newtxpowlimit);
#else
	if (ic->ic_newtxpowlimit != ic->ic_txpowlimit)
		ath_hal_settxpowlimit(ah, ic->ic_newtxpowlimit);
#endif
#endif
}
 

#if 0
#ifdef ATH_SUPERG_XR
static int
ath_xr_rate_setup(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const AR5K_RATE_TABLE *rt;
	struct ieee80211_rateset *rs;
	int i, maxrates;
	sc->sc_xr_rates = ath_hal_getratetable(ah, AR5K_MODE_XR);
	rt = sc->sc_xr_rates;
	if (rt == NULL)
		return 0;
	if (rt->rate_count > XR_NUM_SUP_RATES) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: rate table too small (%u > %u)\n",
			__func__, rt->rate_count, IEEE80211_RATE_MAXSIZE);
		maxrates = IEEE80211_RATE_MAXSIZE;
	} else
		maxrates = rt->rate_count;
	rs = &ic->ic_sup_xr_rates;
	for (i = 0; i < maxrates; i++)
		rs->rs_rates[i] = rt->rates[i].dot11Rate;
	rs->rs_nrates = maxrates;
	return 1;
}
#endif

/* Setup half/quarter rate table support */
static void
ath_setup_subrates(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const AR5K_RATE_TABLE *rt;
	struct ieee80211_rateset *rs;
	int i, maxrates;

	sc->sc_half_rates = ath_hal_getratetable(ah, AR5K_MODE_11A_HALF_RATE);
	rt = sc->sc_half_rates;
	if (rt != NULL) {
		if (rt->rate_count > IEEE80211_RATE_MAXSIZE) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: rate table too small (%u > %u)\n",
			       __func__, rt->rate_count, IEEE80211_RATE_MAXSIZE);
			maxrates = IEEE80211_RATE_MAXSIZE;
		} else
			maxrates = rt->rate_count;
		rs = &ic->ic_sup_half_rates;
		for (i = 0; i < maxrates; i++)
			rs->rs_rates[i] = rt->rates[i].dot11Rate;
		rs->rs_nrates = maxrates;
	}

	sc->sc_quarter_rates = ath_hal_getratetable(ah, AR5K_MODE_11A_QUARTER_RATE);
	rt = sc->sc_quarter_rates;
	if (rt != NULL) {
		if (rt->rate_count > IEEE80211_RATE_MAXSIZE) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: rate table too small (%u > %u)\n",
			       __func__, rt->rate_count, IEEE80211_RATE_MAXSIZE);
			maxrates = IEEE80211_RATE_MAXSIZE;
		} else
			maxrates = rt->rate_count;
		rs = &ic->ic_sup_quarter_rates;
		for (i = 0; i < maxrates; i++)
			rs->rs_rates[i] = rt->rates[i].dot11Rate;
		rs->rs_nrates = maxrates;
	}
}
#endif

static int
ath_rate_setup(struct ath_softc *sc, u_int mode)
{
	struct ath_hal *ah = sc->sc_ah;
	const AR5K_RATE_TABLE *rt;

	rt = ath_hal_getratetable(ah, mode);
	sc->sc_rates[ath_mode_to_idx(mode)] = rt;
	if (rt == NULL)
		return 0;
	ath_d80211_rate_setup(sc, mode, rt);
	return 1;
}

static void
ath_setcurmode(struct ath_softc *sc, u_int mode)
{
#define	N(a)	((int)(sizeof(a)/sizeof(a[0])))
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
	const AR5K_RATE_TABLE *rt;
	int i, j;

	memset(sc->sc_rixmap, 0xff, sizeof(sc->sc_rixmap));
	rt = sc->sc_rates[ath_mode_to_idx(mode)];
	KASSERT(rt != NULL, ("no h/w rate set for phy mode %u", mode));
#if 0
	/* Used for rate control. */
	for (i = 0; i < rt->rate_count; i++)
		sc->sc_rixmap[rt->rates[i].dot11Rate & IEEE80211_RATE_VAL] = i;
#endif
	memset(sc->sc_hwmap, 0, sizeof(sc->sc_hwmap));
	for (i = 0; i < 32; i++) {
		u_int8_t ix = rt->rate_code_to_index[i];
		if (ix == 0xff) {
			sc->sc_hwmap[i].ledon = msecs_to_jiffies(500);
			sc->sc_hwmap[i].ledoff = msecs_to_jiffies(130);
			continue;
		}
#if 0
		sc->sc_hwmap[i].ieeerate =
			rt->rates[ix].dot11Rate & IEEE80211_RATE_VAL;
		if (rt->rates[ix].shortPreamble ||
		    rt->rates[ix].modulation == IEEE80211_T_OFDM)
			sc->sc_hwmap[i].flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
#endif
		/* setup blink rate table to avoid per-packet lookup */
		for (j = 0; j < N(blinkrates) - 1; j++)
			if (blinkrates[j].rate == sc->sc_hwmap[i].ieeerate)
				break;
		/* NB: this uses the last entry if the rate isn't found */
		/* XXX beware of overflow */
		sc->sc_hwmap[i].ledon = msecs_to_jiffies(blinkrates[j].timeOn);
		sc->sc_hwmap[i].ledoff = msecs_to_jiffies(blinkrates[j].timeOff);
	}
	sc->sc_currates = rt;
	/*
	 * All protection frames are transmitted at 2Mb/s for
	 * 11g, otherwise at 1Mb/s.
	 * XXX select protection rate index from rate table.
	 */
	sc->sc_protrix = (mode == AR5K_MODE_11G ? 1 : 0);
	/* rate index used to send mgt frames */
	sc->sc_minrateix = 0;
#undef N
}

#ifdef ATH_SUPERG_FF
static u_int32_t
athff_approx_txtime(struct ath_softc *sc, struct ath_node *an, struct sk_buff *skb)
{
	u_int32_t txtime;
	u_int32_t framelen;

	/*
	 * Approximate the frame length to be transmitted. A swag to add
	 * the following maximal values to the skb payload:
	 *   - 32: 802.11 encap + CRC
	 *   - 24: encryption overhead (if wep bit)
	 *   - 4 + 6: fast-frame header and padding
	 *   - 16: 2 LLC FF tunnel headers
	 *   - 14: 1 802.3 FF tunnel header (skb already accounts for 2nd)
	 */
	framelen = skb->len + 32 + 4 + 6 + 16 + 14;
	if (sc->sc_ic.ic_flags & IEEE80211_F_PRIVACY)
		framelen += 24;
	if (an->an_tx_ffbuf[skb->priority])
		framelen += an->an_tx_ffbuf[skb->priority]->bf_skb->len;

	txtime = ath_hal_computetxtime(sc->sc_ah, sc->sc_currates, framelen,
		an->an_prevdatarix, FALSE);

	return txtime;
}
/*
 * Determine if a data frame may be aggregated via ff tunneling.
 *
 *  NB: allowing EAPOL frames to be aggregated with other unicast traffic.
 *      Do 802.1x EAPOL frames proceed in the clear? Then they couldn't
 *      be aggregated with other types of frames when encryption is on?
 *
 *  NB: assumes lock on an_tx_ffbuf effectively held by txq lock mechanism.
 */
static int
athff_can_aggregate(struct ath_softc *sc, struct ether_header *eh,
		    struct ath_node *an, struct sk_buff *skb, u_int16_t fragthreshold, int *flushq)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_txq *txq = sc->sc_ac2q[skb->priority];
	struct ath_buf *ffbuf = an->an_tx_ffbuf[skb->priority];
	u_int32_t txoplimit;

#define US_PER_4MS 4000
#define	MIN(a,b)	((a) < (b) ? (a) : (b))

	*flushq = FALSE;

	if (fragthreshold < 2346)
	    return FALSE;

	if ((!ffbuf) && (txq->axq_depth < sc->sc_fftxqmin))
		return FALSE;
	if (!(ic->ic_ath_cap & an->an_node.ni_ath_flags & IEEE80211_ATHC_FF))
		return FALSE;
	if (!(ic->ic_opmode == IEEE80211_M_STA ||
		  ic->ic_opmode == IEEE80211_M_HOSTAP))
		return FALSE;
	if ((ic->ic_opmode == IEEE80211_M_HOSTAP) &&
		  ETHER_IS_MULTICAST(eh->ether_dhost))
		return FALSE;

#ifdef ATH_SUPERG_XR
	if (sc->sc_currates->rates[an->an_prevdatarix].rate_kbps < an->an_minffrate)
		return FALSE;
#endif
	txoplimit = IEEE80211_TXOP_TO_US(
		ic->ic_wme.wme_chanParams.cap_wmeParams[skb->priority].wmep_txopLimit);

	/* if the 4 msec limit is set on the channel, take it into account */
	if (sc->sc_curchan.private_flags & CHANNEL_4MS_LIMIT)
		txoplimit = MIN(txoplimit, US_PER_4MS); 

	if (txoplimit != 0 && athff_approx_txtime(sc, an, skb) > txoplimit) {
		DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
			"%s: FF TxOp violation\n", __func__);
		if (ffbuf)
			*flushq = TRUE;
		return FALSE;
	}

	return TRUE;

#undef US_PER_4MS
#undef MIN
}
#endif

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

#if 0
/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ath_getstats(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct net_device_stats *stats = &sc->sc_devstats;

	/* update according to private statistics */
	stats->tx_errors = sc->sc_stats.ast_tx_xretries
			 + sc->sc_stats.ast_tx_fifoerr
			 + sc->sc_stats.ast_tx_filtered;
	stats->tx_dropped = sc->sc_stats.ast_tx_nobuf
			+ sc->sc_stats.ast_tx_encap
			+ sc->sc_stats.ast_tx_nonode
			+ sc->sc_stats.ast_tx_nobufmgt;
	stats->rx_errors = sc->sc_stats.ast_rx_fifoerr
			+ sc->sc_stats.ast_rx_badcrypt
			+ sc->sc_stats.ast_rx_badmic;
	stats->rx_dropped = sc->sc_stats.ast_rx_tooshort;
	stats->rx_crc_errors = sc->sc_stats.ast_rx_crcerr;

	return stats;
}

static int
ath_set_mac_address(struct net_device *dev, void *addr)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct sockaddr *mac = addr;
	int error;

	if (netif_running(dev)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: cannot set address; device running\n", __func__);
		return -EBUSY;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		__func__,
		mac->sa_data[0], mac->sa_data[1], mac->sa_data[2],
		mac->sa_data[3], mac->sa_data[4], mac->sa_data[5]);

	ATH_LOCK(sc);
	/* XXX not right for multiple VAPs */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, mac->sa_data);
	IEEE80211_ADDR_COPY(dev->dev_addr, mac->sa_data);
	ath_hal_setmac(ah, dev->dev_addr);
	error = ath_reset(dev);
	ATH_UNLOCK(sc);

	return error;
}

static int
ath_change_mtu(struct net_device *dev, int mtu)
{
	struct ath_softc *sc = dev->priv;
	int error;

	if (!(ATH_MIN_MTU < mtu && mtu <= ATH_MAX_MTU)) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid %d, min %u, max %u\n",
			__func__, mtu, ATH_MIN_MTU, ATH_MAX_MTU);
		return -EINVAL;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: %d\n", __func__, mtu);

	ATH_LOCK(sc);
	dev->mtu = mtu;
	/* NB: the rx buffers may need to be reallocated */
	tasklet_disable(&sc->sc_rxtq);
	error = ath_reset(dev);
	tasklet_enable(&sc->sc_rxtq);
	ATH_UNLOCK(sc);

	return error;
}

/*
 * Diagnostic interface to the HAL.  This is used by various
 * tools to do things like retrieve register contents for
 * debugging.  The mechanism is intentionally opaque so that
 * it can change frequently w/o concern for compatibility.
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
	} else
		error = -EINVAL;
bad:
	if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
		kfree(indata);
	if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
		kfree(outdata);
	return error;
}

static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int error;

	ATH_LOCK(sc);
	switch (cmd) {
	case SIOCGATHSTATS:
		sc->sc_stats.ast_tx_packets = sc->sc_devstats.tx_packets;
		sc->sc_stats.ast_rx_packets = sc->sc_devstats.rx_packets;
		sc->sc_stats.ast_rx_rssi = ieee80211_getrssi(ic);
		if (copy_to_user(ifr->ifr_data, &sc->sc_stats, sizeof (sc->sc_stats)))
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
	case SIOC80211IFCREATE:
		error = ieee80211_ioctl_create_vap(ic, ifr, dev); 
		break;
	default:
		error = -EINVAL;
		break;
	}
	ATH_UNLOCK(sc);
	return error;
}
#endif

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
	ATH_SLOTTIME		= 1,
	ATH_ACKTIMEOUT		= 2,
	ATH_CTSTIMEOUT		= 3,
	ATH_SOFTLED		= 4,
	ATH_LEDPIN		= 5,
	ATH_COUNTRYCODE		= 6,
	ATH_REGDOMAIN		= 7,
	ATH_DEBUG		= 8,
	ATH_TXANTENNA		= 9,
	ATH_RXANTENNA		= 10,
	ATH_DIVERSITY		= 11,
	ATH_TXINTRPERIOD 	= 12,
	ATH_FFTXQMIN		= 18,
	ATH_TKIPMIC		= 19,
	ATH_XR_POLL_PERIOD 	= 20,
	ATH_XR_POLL_COUNT 	= 21,
	ATH_ACKRATE             = 22,
};

static int
ATH_SYSCTL_DECL(ath_sysctl_halparam, ctl, write, filp, buffer, lenp, ppos)
{
	struct ath_softc *sc = ctl->extra1;
	struct ieee80211_hw *hw = sc->sc_hw;
	struct ath_hal *ah = sc->sc_ah;
	u_int val;
	int ret;

	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) {
		ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos);
		if (ret == 0) {
			switch (ctl->ctl_name) {
			case ATH_SLOTTIME:
				if (val > 0) {
					if (!ath_hal_setslottime(ah, val))
						ret = -EINVAL;
					else
						sc->sc_slottimeconf = val;
				} else {
					/* disable manual override */
					sc->sc_slottimeconf = 0;
					ath_setslottime(sc);
				}
				break;
			case ATH_ACKTIMEOUT:
				if (!ath_hal_setacktimeout(ah, val))
					ret = -EINVAL;
				break;
			case ATH_CTSTIMEOUT:
				if (!ath_hal_setctstimeout(ah, val))
					ret = -EINVAL;
				break;
			case ATH_SOFTLED:
				if (val != sc->sc_softled) {
					if (val)
						ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
					ath_hal_gpioset(ah, sc->sc_ledpin,!sc->sc_ledon);
					sc->sc_softled = val;
				}
				break;
			case ATH_LEDPIN:
				/* XXX validate? */
				sc->sc_ledpin = val;
				break;
			case ATH_DEBUG:
				sc->sc_debug = val;
				break;
			case ATH_TXANTENNA:
				/*
				 * antenna can be:
				 * 0 = transmit diversity
				 * 1 = antenna port 1
				 * 2 = antenna port 2
				 */
				if (val > 2)
					return -EINVAL;
				else
					sc->sc_txantenna = val;
				break;
			case ATH_RXANTENNA:
				/*
				 * antenna can be:
				 * 0 = receive diversity
				 * 1 = antenna port 1
				 * 2 = antenna port 2
				 */
				if (val > 2)
					return -EINVAL;
				else
					ath_setdefantenna(sc, val);
				break;
			case ATH_DIVERSITY:
				/*
				 * 0 = disallow use of diversity
				 * 1 = allow use of diversity
				 */
				if (val > 1)
					return -EINVAL;
				/* Don't enable diversity if XR is enabled */
				if (((!sc->sc_hasdiversity) || (sc->sc_xrtxq != NULL)) && val)
					return -EINVAL;
				sc->sc_diversity = val;
				ath_hal_setdiversity(ah, val);
				break;
			case ATH_TXINTRPERIOD:
				/* XXX: validate? */
				sc->sc_txintrperiod = val;
				break;
			case ATH_FFTXQMIN:
				/* XXX validate? */
				sc->sc_fftxqmin = val;
				break;
			case ATH_TKIPMIC: {
				if (!ath_hal_hastkipmic(ah))
					return -EINVAL;
				ath_hal_settkipmic(ah, val);
				if (val)
					hw->flags &= ~IEEE80211_HW_TKIP_INCLUDE_MMIC;
				else
					hw->flags |= IEEE80211_HW_TKIP_INCLUDE_MMIC;

				break;
			}
#ifdef ATH_SUPERG_XR
			case ATH_XR_POLL_PERIOD: 
				if (val > XR_MAX_POLL_INTERVAL)
					val = XR_MAX_POLL_INTERVAL;
				else if (val < XR_MIN_POLL_INTERVAL)
					val = XR_MIN_POLL_INTERVAL;
				sc->sc_xrpollint = val;
				break;

			case ATH_XR_POLL_COUNT: 
				if (val > XR_MAX_POLL_COUNT)
					val = XR_MAX_POLL_COUNT;
				else if (val < XR_MIN_POLL_COUNT)
					val = XR_MIN_POLL_COUNT;
				sc->sc_xrpollcount = val;
				break;
#endif
			case ATH_ACKRATE:
				sc->sc_ackrate = val;
				ath_set_ack_bitrate(sc, sc->sc_ackrate);
				break;
			default:
				return -EINVAL;
			}
		}
	} else {
		switch (ctl->ctl_name) {
		case ATH_SLOTTIME:
			val = ath_hal_getslottime(ah);
			break;
		case ATH_ACKTIMEOUT:
			val = ath_hal_getacktimeout(ah);
			break;
		case ATH_CTSTIMEOUT:
			val = ath_hal_getctstimeout(ah);
			break;
		case ATH_SOFTLED:
			val = sc->sc_softled;
			break;
		case ATH_LEDPIN:
			val = sc->sc_ledpin;
			break;
		case ATH_COUNTRYCODE:
			ath_hal_getcountrycode(ah, &val);
			break;
		case ATH_REGDOMAIN:
			ath_hal_getregdomain(ah, &val);
			break;
		case ATH_DEBUG:
			val = sc->sc_debug;
			break;
		case ATH_TXANTENNA:
			val = sc->sc_txantenna;
			break;
		case ATH_RXANTENNA:
			val = ath_hal_getdefantenna(ah);
			break;
		case ATH_DIVERSITY:
			val = sc->sc_diversity;
			break;
		case ATH_TXINTRPERIOD:
			val = sc->sc_txintrperiod;
			break;
		case ATH_FFTXQMIN:
			val = sc->sc_fftxqmin;
			break;
		case ATH_TKIPMIC:
			val = ath_hal_gettkipmic(ah);
			break;
#ifdef ATH_SUPERG_XR
		case ATH_XR_POLL_PERIOD: 
			val=sc->sc_xrpollint;
			break;
		case ATH_XR_POLL_COUNT: 
			val=sc->sc_xrpollcount;
			break;
#endif
		case ATH_ACKRATE:
			val = sc->sc_ackrate;
			break;
		default:
			return -EINVAL;
		}
		ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, lenp, ppos);
	}
	return ret;
}

static int mincalibrate = 1;			/* once a second */
static int maxint = 0x7fffffff;		/* 32-bit big */

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
	{ .ctl_name	= ATH_COUNTRYCODE,
	  .procname	= "countrycode",
	  .mode		= 0444,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_REGDOMAIN,
	  .procname	= "regdomain",
	  .mode		= 0444,
	  .proc_handler	= ath_sysctl_halparam
	},
#ifdef AR_DEBUG
	{ .ctl_name	= ATH_DEBUG,
	  .procname	= "debug",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
#endif
	{ .ctl_name	= ATH_TXANTENNA,
	  .procname	= "txantenna",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_RXANTENNA,
	  .procname	= "rxantenna",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_DIVERSITY,
	  .procname	= "diversity",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_TXINTRPERIOD,
	  .procname	= "txintrperiod",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_FFTXQMIN,
	  .procname	= "fftxqmin",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_TKIPMIC,
	  .procname	= "tkipmic",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
#ifdef ATH_SUPERG_XR
	{ .ctl_name	= ATH_XR_POLL_PERIOD,
	  .procname	= "xrpollperiod",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ .ctl_name	= ATH_XR_POLL_COUNT,
	  .procname	= "xrpollcount",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
#endif
	{ .ctl_name	= ATH_ACKRATE,
	  .procname	= "ackrate",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam
	},
	{ 0 }
};

static void
ath_dynamic_sysctl_register(struct ath_softc *sc)
{
	int i, space;
	char *dev_name = NULL;
	
	space = 5 * sizeof(struct ctl_table) + sizeof(ath_sysctl_template);
	sc->sc_sysctls = kmalloc(space, GFP_KERNEL);
	if (sc->sc_sysctls == NULL) {
		printk("%s: no memory for sysctl table!\n", __func__);
		return;
	}
	
	/* 
	 * We want to reserve space for the name of the device separate
	 * from the net_device structure, because when the name is changed
	 * it is changed in the net_device structure and the message given
	 * out.  Thus we won't know what the name it used to be if we rely
	 * on it.
	 */
	dev_name = kmalloc((strlen(sc->name) + 1) * sizeof(char), GFP_KERNEL);
	if (dev_name == NULL) {
		printk("%s: no memory for device name storage!\n", __func__);
		return;
	}
	strncpy(dev_name, sc->name, strlen(sc->name) + 1);

	/* setup the table */
	memset(sc->sc_sysctls, 0, space);
	sc->sc_sysctls[0].ctl_name = CTL_DEV;
	sc->sc_sysctls[0].procname = "dev";
	sc->sc_sysctls[0].mode = 0555;
	sc->sc_sysctls[0].child = &sc->sc_sysctls[2];
	/* [1] is NULL terminator */
	sc->sc_sysctls[2].ctl_name = CTL_AUTO;
	sc->sc_sysctls[2].procname = dev_name;
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
	sc->sc_sysctl_header = ATH_REGISTER_SYSCTL_TABLE(sc->sc_sysctls);
	if (!sc->sc_sysctl_header) {
		printk("%s: failed to register sysctls!\n", sc->name);
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}

	/* initialize values */
	sc->sc_debug = ath_debug;
	sc->sc_txantenna = 0;		/* default to auto-selection */
	sc->sc_txintrperiod = ATH_TXQ_INTR_PERIOD;
}

static void
ath_dynamic_sysctl_unregister(struct ath_softc *sc)
{
	if (sc->sc_sysctl_header) {
		unregister_sysctl_table(sc->sc_sysctl_header);
		sc->sc_sysctl_header = NULL;
	}
	if (sc->sc_sysctls[2].procname) {
		kfree(sc->sc_sysctls[2].procname);
		sc->sc_sysctls[2].procname = NULL;
	}
	if (sc->sc_sysctls) {
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}
}

/*
 * Announce various information on device/driver attach.
 */
static void
ath_announce(struct ath_softc *sc)
{
#define	AR5K_MODE_DUALBAND	(AR5K_MODE_11A|AR5K_MODE_11B)
	struct ath_hal *ah = sc->sc_ah;
	u_int modes, cc;

	printk("%s: mac %d.%d phy %d.%d", sc->name,
		ah->ah_mac_version, ah->ah_mac_revision,
		ah->ah_phy_revision >> 4, ah->ah_phy_revision & 0xf);
	/*
	 * Print radio revision(s).  We check the wireless modes
	 * to avoid falsely printing revs for inoperable parts.
	 * Dual-band radio revs are returned in the 5 GHz rev number.
	 */
	ath_hal_getcountrycode(ah, &cc);
	modes = ath_hal_getwirelessmodes(ah, cc);
	if ((modes & AR5K_MODE_DUALBAND) == AR5K_MODE_DUALBAND) {
		if (ah->ah_radio_5ghz_revision && ah->ah_radio_2ghz_revision)
			printk(" 5 GHz radio %d.%d 2 GHz radio %d.%d",
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
#if 0
	if (1/*bootverbose*/) {
		int i;
		for (i = 0; i <= WME_AC_VO; i++) {
			struct ath_txq *txq = sc->sc_ac2q[i];
			printk("%s: Use hw queue %u for %s traffic\n",
				dev->name, txq->axq_qnum,
				ieee80211_wme_acnames[i]);
		}
#endif
		printk("%s: Use hw queue %u for CAB traffic\n", sc->name,
			sc->sc_cabq->axq_qnum);
		printk("%s: Use hw queue %u for beacons\n", sc->name,
			sc->sc_bhalq);
#if 0
	}
#endif
#undef AR5K_MODE_DUALBAND
}

/*
 * Static (i.e. global) sysctls.  Note that the HAL sysctls
 * are located under ours by sharing the setting for DEV_ATH.
 */
enum {
	DEV_ATH		= 9,			/* XXX known by HAL */
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
	  .procname	= "calibrate",
	  .mode		= 0644,
	  .data		= &ath_calinterval,
	  .maxlen	= sizeof(ath_calinterval),
	  .extra1	= &mincalibrate,
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
		ath_sysctl_header = ATH_REGISTER_SYSCTL_TABLE(ath_root_table);
		initialized = 1;
	}
}

void
ath_sysctl_unregister(void)
{
	if (ath_sysctl_header)
		unregister_sysctl_table(ath_sysctl_header);
}

static const char* 
ath_get_hal_status_desc(AR5K_STATUS status)
{
	if (status > 0 && status < sizeof(hal_status_desc)/sizeof(char *))
		return hal_status_desc[status];
	else
		return "";
}
