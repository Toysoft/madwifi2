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
 * IEEE 802.11 generic handler
 *
 * This code is derived from NetBSD code; their copyright notice follows.
 */

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Atsushi Onoe.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include "rc4.h"
#define	arc4_ctxlen()			sizeof (struct rc4_state)
#define	arc4_setkey(_c,_k,_l)		rc4_init(_c,_k,_l)
#define	arc4_encrypt(_c,_d,_s,_l)	rc4_crypt(_c,_s,_d,_l)

#include "if_ieee80211.h"
#include "if_llc.h"
#include "if_ethersubr.h"

#ifdef NEW_MODULE_CODE
#define	__MOD_INC_USE_COUNT(_m)						\
	if (!try_module_get(_m)) {					\
		printk(KERN_WARNING "%s: try_module_get failed\n",	\
			ic->ic_dev.name);				\
		return (ENODEV);					\
	}
#define	__MOD_DEC_USE_COUNT(_m)		module_put(_m)
#endif

#define	IEEE80211_DEBUG
#ifdef IEEE80211_DEBUG
static	int ieee80211_debug = 0;
#define	DPRINTF(X)	if (ieee80211_debug) printk X
#define	DPRINTF2(X)	if (ieee80211_debug>1) printk X
#else
#define	DPRINTF(X)
#define	DPRINTF2(X)
#endif

static int ieee80211_send_prreq(struct ieee80211com *,
    struct ieee80211_node *, int, int);
static int ieee80211_send_prresp(struct ieee80211com *,
    struct ieee80211_node *, int, int);
static int ieee80211_send_auth(struct ieee80211com *,
    struct ieee80211_node *, int, int);
static int ieee80211_send_deauth(struct ieee80211com *,
    struct ieee80211_node *, int, int);
static int ieee80211_send_asreq(struct ieee80211com *,
    struct ieee80211_node *, int, int);
static int ieee80211_send_asresp(struct ieee80211com *,
    struct ieee80211_node *, int, int);
static int ieee80211_send_disassoc(struct ieee80211com *,
    struct ieee80211_node *, int, int);

static void ieee80211_recv_beacon(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);
static void ieee80211_recv_prreq(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);
static void ieee80211_recv_auth(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);
static void ieee80211_recv_asreq(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);
static void ieee80211_recv_asresp(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);
static void ieee80211_recv_disassoc(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);
static void ieee80211_recv_deauth(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t);

static int ieee80211_media_change(struct net_device *);
static void ieee80211_crc_init(void);
static u_int32_t ieee80211_crc_update(u_int32_t, u_int8_t *, int);
static struct net_device_stats *ieee80211_getstats(struct net_device *);
static void ieee80211_slowtimo(unsigned long);

extern	int ieee80211_cfgget(struct net_device *, u_long, caddr_t);
extern	int ieee80211_cfgset(struct net_device *, u_long, caddr_t);
#ifdef CONFIG_PROC_FS
static	void ieee80211_proc_init(struct ieee80211com *);
static	void ieee80211_proc_remove(struct ieee80211com *);
#endif /* CONFIG_PROC_FS */
#ifdef CONFIG_NET_WIRELESS
extern	struct iw_statistics *ieee80211_iw_getstats(struct net_device *);
extern	const struct iw_handler_def ieee80211_iw_handler_def;
#endif

static const char *ieee80211_mgt_subtype_name[] = {
	"assoc_req",	"assoc_resp",	"reassoc_req",	"reassoc_resp",
	"probe_req",	"probe_resp",	"reserved#6",	"reserved#7",
	"beacon",	"atim",		"disassoc",	"auth",
	"deauth",	"reserved#13",	"reserved#14",	"reserved#15"
};

int
ieee80211_ifattach(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ifmediareq imr;
	int i, rate, mword;

	__MOD_INC_USE_COUNT(THIS_MODULE);

	/*
	 * Register the netdevice early so the device
	 * name is filled in for any msgs.
	 *
	 * NB: The driver is expected to fill in anything
	 *     it cares about; we fillin only default handlers.
	 */
	if (dev->get_stats == NULL)
		dev->get_stats = ieee80211_getstats;
#ifdef CONFIG_NET_WIRELESS
	if (dev->get_wireless_stats == NULL)
		dev->get_wireless_stats = ieee80211_iw_getstats;
	if (dev->wireless_handlers == NULL)
		dev->wireless_handlers = 
			(struct iw_handler_def *) &ieee80211_iw_handler_def;
#endif /* CONFIG_NET_WIRELESS */
	if (register_netdev(&ic->ic_dev)) {
		printk(KERN_WARNING "%s: unable to register device\n",
			ic->ic_dev.name);
		return (EIO);
	}

	spin_lock_init(&ic->ic_lock);

	/*
	 * Emulate the BSD ifnet timer ourselves.
	 */
	init_timer(&ic->ic_slowtimo);
	ic->ic_slowtimo.data = (unsigned long) ic;
	ic->ic_slowtimo.function = ieee80211_slowtimo;
	ieee80211_slowtimo((unsigned long) ic);		/* prime timer */

	/*
	 * Setup crypto support.
	 */
	ieee80211_crc_init();
	get_random_bytes(&ic->ic_iv, sizeof(ic->ic_iv));

	/*
	 * Fill in 802.11 available channel set, mark
	 * all available channels as active, and pick
	 * a default channel if not already specified.
	 */
	memset(ic->ic_chan_avail, 0, sizeof(ic->ic_chan_avail));
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		struct ieee80211channel *c = &ic->ic_channels[i];
		if (c->ic_flags) {
			/*
			 * Verify driver passed us valid data.
			 */
			if (i != ieee80211_chan2ieee(ic, c)) {
				printk(KERN_WARNING "%s: bad channel ignored; "
					"freq %u flags %x number %u\n",
					dev->name, c->ic_freq, c->ic_flags, i);
				continue;
			}
			setbit(ic->ic_chan_avail, i);
		}
	}
	memcpy(ic->ic_chan_active, ic->ic_chan_avail,
	    sizeof(ic->ic_chan_active));
	if (ic->ic_ibss_chan == NULL) {
		/*
		 * Pick the first active channel for default.
		 * XXX probably not right.
		 */
		for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
			if (isset(ic->ic_chan_active, i)) {
				ic->ic_ibss_chan = &ic->ic_channels[i];
				break;
			}
		}
	}

	/*
	 * Fill in media characteristics.
	 */
#define	ADD(_ic, _s, _o) \
	ifmedia_add(&(_ic)->ic_media, \
		IFM_MAKEWORD(IFM_IEEE80211, (_s), (_o), 0), 0, NULL)
	ifmedia_init(&ic->ic_media, 0,
		ieee80211_media_change, ieee80211_media_status);
	ADD(ic, IFM_AUTO, 0);
	if (ic->ic_caps & IEEE80211_C_IBSS)
		ADD(ic, IFM_AUTO, IFM_IEEE80211_ADHOC);
	if (ic->ic_caps & IEEE80211_C_HOSTAP)
		ADD(ic, IFM_AUTO, IFM_IEEE80211_HOSTAP);
	if (ic->ic_caps & IEEE80211_C_AHDEMO)
		ADD(ic, IFM_AUTO, IFM_IEEE80211_ADHOC | IFM_FLAG0);
	for (i = 0; i < IEEE80211_RATE_SIZE; i++) {
		rate = ic->ic_sup_rates[i];
		mword = ieee80211_rate2media(rate, ic->ic_phytype);
		ADD(ic, mword, 0);
		if (ic->ic_caps & IEEE80211_C_IBSS)
			ADD(ic, mword, IFM_IEEE80211_ADHOC);
		if (ic->ic_caps & IEEE80211_C_HOSTAP)
			ADD(ic, mword, IFM_IEEE80211_HOSTAP);
		if (ic->ic_caps & IEEE80211_C_AHDEMO)
			ADD(ic, mword, IFM_IEEE80211_ADHOC | IFM_FLAG0);
	}
	ieee80211_media_status(dev, &imr);
	ifmedia_set(&ic->ic_media, imr.ifm_active);
#undef ADD

	ic->ic_rtsthreshold = 2347;		/* XXX not used yet */
	ic->ic_fragthreshold = 2346;		/* XXX not used yet */
	ic->ic_des_chan =			/* any channel is ok */
		(struct ieee80211channel *) IEEE80211_CHAN_ANY;
	ic->ic_fixed_rate = -1;			/* no fixed rate */
	if (ic->ic_lintval == 0)
		ic->ic_lintval = 100;		/* default sleep */
	TAILQ_INIT(&ic->ic_node);

	/* initialize management frame handlers */
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_PROBE_RESP
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_beacon;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_BEACON
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_beacon;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_PROBE_REQ
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_prreq;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_AUTH
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_auth;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_ASSOC_REQ
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_asreq;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_REASSOC_REQ
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_asreq;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_ASSOC_RESP
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_asresp;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_REASSOC_RESP
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_asresp;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_DEAUTH
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_deauth;
	ic->ic_recv_mgmt[IEEE80211_FC0_SUBTYPE_DISASSOC
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_recv_disassoc;

	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_PROBE_REQ
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_prreq;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_PROBE_RESP
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_prresp;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_AUTH
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_auth;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_DEAUTH
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_deauth;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_ASSOC_REQ
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_asreq;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_REASSOC_REQ
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_asreq;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_ASSOC_RESP
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_asresp;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_REASSOC_RESP
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_asresp;
	ic->ic_send_mgmt[IEEE80211_FC0_SUBTYPE_DISASSOC
	    >> IEEE80211_FC0_SUBTYPE_SHIFT] = ieee80211_send_disassoc;

#ifdef CONFIG_PROC_FS
	ieee80211_proc_init(ic);
#endif

	return (0);
}

void
ieee80211_ifdetach(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;

	IEEE80211_LOCK(ic);
	del_timer(&ic->ic_slowtimo);
	if (ic->ic_wep_ctx != NULL) {
		kfree(ic->ic_wep_ctx);
		ic->ic_wep_ctx = NULL;
	}
	ieee80211_free_allnodes(ic);
	ifmedia_removeall(&ic->ic_media);
	unregister_netdev(&ic->ic_dev);
#ifdef CONFIG_PROC_FS
	ieee80211_proc_remove(ic);
#endif
	IEEE80211_UNLOCK(ic);

	__MOD_DEC_USE_COUNT(THIS_MODULE);
}

/*
 * Once a second "slow timeout" a la the BSD ifnet timer.
 */
void
ieee80211_slowtimo(unsigned long arg)
{
	struct ieee80211com *ic = (struct ieee80211com *) arg;

	if (ic->ic_timer && --ic->ic_timer == 0)
		if (ic->ic_watchdog)
			(*ic->ic_watchdog)(&ic->ic_dev);

	ic->ic_slowtimo.expires = jiffies + HZ;		/* once a second */
	add_timer(&ic->ic_slowtimo);
}

/*
 * Locate the channel for the specified frequency.
 */
struct ieee80211channel *
ieee80211_chan_find(struct ieee80211com *ic, u_int freq)
{
	struct ieee80211channel *c;

	/* XXX use available channel bitmask??? */
	c = ic->ic_ibss_chan+1;
	for (; c <= &ic->ic_channels[IEEE80211_CHAN_MAX]; c++)
		if (c->ic_freq == freq)
			return c;
	for (c = &ic->ic_channels[0]; c < ic->ic_ibss_chan; c++)
		if (c->ic_freq == freq)
			return c;
	return NULL;
}

/*
 * Convert GHz frequency to IEEE channel number.
 */
u_int
ieee80211_ghz2ieee(u_int freq, u_int flags)
{
	if (flags & IEEE80211_CHAN_2GHZ) {	/* 2GHz band */
		if (freq == 2484)
			return 14;
		if (freq < 2484)
			return (freq - 2407) / 5;
		else
			return 15 + ((freq - 2512) / 20);
	} else if (IEEE80211_CHAN_5GHZ) {/* 5Ghz band */
		return (freq - 5000) / 5;
	} else {			/* either, guess */
		if (freq == 2484)
			return 14;
		if (freq < 2484)
			return (freq - 2407) / 5;
		if (freq < 5000)
			return 15 + ((freq - 2512) / 20);
		return (freq - 5000) / 5;
	}
}

/*
 * Convert channel to IEEE channel number.
 */
u_int
ieee80211_chan2ieee(struct ieee80211com *ic, struct ieee80211channel *c)
{
	if (ic->ic_channels <= c && c <= &ic->ic_channels[IEEE80211_CHAN_MAX])
		return c - ic->ic_channels;
	else {
		printk(KERN_ERR "wlan: invalid channel freq %u flags %x\n",
			c->ic_freq, c->ic_flags);
		return 0;		/* XXX */
	}
}

/*
 * Convert IEEE channel number to GHz frequency.
 */
u_int
ieee80211_ieee2ghz(u_int chan, u_int flags)
{
	if (flags & IEEE80211_CHAN_2GHZ) {	/* 2GHz band */
		if (chan == 14)
			return 2484;
		if (chan < 14)
			return 2407 + chan*5;
		else
			return 2512 + ((chan-15)*20);
	} else if (flags & IEEE80211_CHAN_5GHZ) {/* 5Ghz band */
		return 5000 + (chan*5);
	} else {			/* either, guess */
		if (chan == 14)
			return 2484;
		if (chan < 14)		/* 0-13 */
			return 2407 + chan*5;
		if (chan < 27)		/* 15-26 */
			return 2512 + ((chan-15)*20);
		return 5000 + (chan*5);
	}
}

static int
ieee80211_media_change(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ifmedia_entry *ime;
	int error;

	error = 0;
	ime = ic->ic_media.ifm_cur;
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
		ic->ic_flags &= ~IEEE80211_F_ROAMING;
	} else if (ime->ifm_media & IFM_IEEE80211_HOSTAP) {
		ic->ic_opmode = IEEE80211_M_HOSTAP;
		ic->ic_flags &= ~(IEEE80211_F_IBSSON | IEEE80211_F_ROAMING);
	} else {
		ic->ic_opmode = IEEE80211_M_STA;
		ic->ic_flags &= ~IEEE80211_F_IBSSON;
		ic->ic_flags |= IEEE80211_F_ROAMING;
	}
	return (*ic->ic_init)(dev);
}

void
ieee80211_media_status(struct net_device *dev, struct ifmediareq *imr)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni = NULL;

	imr->ifm_status = IFM_AVALID;
	imr->ifm_active = IFM_IEEE80211;
	if (ic->ic_state == IEEE80211_S_RUN)
		imr->ifm_status |= IFM_ACTIVE;
	imr->ifm_active |= IFM_AUTO;
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		ni = &ic->ic_bss;
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

void
ieee80211_input(struct net_device *dev, struct sk_buff *skb,
	int rssi, u_int32_t rstamp)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni = NULL;
	struct ieee80211_frame *wh;
	struct ether_header *eh;
	void (*rh)(struct ieee80211com *, struct sk_buff *, int, u_int);
	struct sk_buff *skb1;
	int len;
	u_int8_t dir, subtype;
	u_int8_t *bssid;
	u_int16_t rxseq;

	wh = (struct ieee80211_frame *) skb->data;
	if ((wh->i_fc[0] & IEEE80211_FC0_VERSION_MASK) !=
	    IEEE80211_FC0_VERSION_0) {
		if (netif_msg_debug(ic))
			printk("%s: discard packet with wrong version: %x\n",
			    dev->name, wh->i_fc[0]);
		goto err;
	}

	dir = wh->i_fc[1] & IEEE80211_FC1_DIR_MASK;

	if (ic->ic_state != IEEE80211_S_SCAN) {
		switch (ic->ic_opmode) {
		case IEEE80211_M_STA:
			ni = &ic->ic_bss;
			if (!IEEE80211_ADDR_EQ(wh->i_addr2, ni->ni_bssid)) {
				if (netif_msg_debug(ic))
					printk("%s: discard frame from bss %s\n",
					    dev->name,
					    ether_sprintf(wh->i_addr2));
				/* not interested in */
				goto out;
			}
			break;
		case IEEE80211_M_IBSS:
		case IEEE80211_M_AHDEMO:
		case IEEE80211_M_HOSTAP:
			if (dir == IEEE80211_FC1_DIR_NODS)
				bssid = wh->i_addr3;
			else
				bssid = wh->i_addr1;
			if (!IEEE80211_ADDR_EQ(bssid, ic->ic_bss.ni_bssid) &&
			    !IEEE80211_ADDR_EQ(bssid, dev->broadcast)) {
				/* not interested in */
				DPRINTF2(("ieee80211_input: other bss %s\n",
				    ether_sprintf(wh->i_addr3)));
				goto out;
			}
			ni = ieee80211_find_node(ic, wh->i_addr2);
			if (ni == NULL) {
				DPRINTF2(("ieee80211_input: unknown src %s\n",
				    ether_sprintf(wh->i_addr2)));
				ni = &ic->ic_bss;	/* XXX allocate? */
			}
			break;
		}
		ni->ni_rssi = rssi;
		ni->ni_rstamp = rstamp;
		rxseq = ni->ni_rxseq;
		ni->ni_rxseq =
		    le16_to_cpu(*(u_int16_t *)wh->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;
		/* TODO: fragment */
		if ((wh->i_fc[1] & IEEE80211_FC1_RETRY) &&
		    rxseq == ni->ni_rxseq) {
			/* duplicate, silently discarded */
			goto out;
		}
		ni->ni_inact = 0;
	}

	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_DATA:
		switch (ic->ic_opmode) {
		case IEEE80211_M_STA:
			if (dir != IEEE80211_FC1_DIR_FROMDS) {
				if (netif_msg_debug(ic))
					printk("%s:discard frame with invalid direction %x\n",
						dev->name, dir);
				goto out;
			}
#ifdef IFF_SIMPLEX
			if ((ifp->if_flags & IFF_SIMPLEX) &&
			    IEEE80211_IS_MULTICAST(wh->i_addr1) &&
			    IEEE80211_ADDR_EQ(wh->i_addr3, dev->dev_addr)) {
				/*
				 * In IEEE802.11 network, multicast packet
				 * sent from me is broadcasted from AP.
				 * It should be silently discarded for
				 * SIMPLEX interface.
				 */
				if (netif_msg_debug(ic))
					printk("%s: discard multicast echo\n",
						dev->name);
				goto out;
			}
#endif
			break;
		case IEEE80211_M_IBSS:
		case IEEE80211_M_AHDEMO:
			if (dir != IEEE80211_FC1_DIR_NODS)
				goto out;
			break;
		case IEEE80211_M_HOSTAP:
			if (dir != IEEE80211_FC1_DIR_TODS)
				goto out;
			/* check if source STA is associated */
			ni = ieee80211_find_node(ic, wh->i_addr2);
			if (ni == NULL) {
				DPRINTF(("ieee80211_input: "
				    "data from unknown src %s\n",
				    ether_sprintf(wh->i_addr2)));
				if ((ni = ieee80211_alloc_node(ic, wh->i_addr2,
				    1)) != NULL) {
					IEEE80211_SEND_MGMT(ic, ni,
					    IEEE80211_FC0_SUBTYPE_DEAUTH,
					    IEEE80211_REASON_NOT_AUTHED);
					ieee80211_free_node(ic, ni);
				}
				goto err;
			}
			if (ni->ni_associd == 0) {
				DPRINTF(("ieee80211_input: "
				    "data from unassoc src %s\n",
				    ether_sprintf(wh->i_addr2)));
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_DISASSOC,
				    IEEE80211_REASON_NOT_ASSOCED);
				goto err;
			}
			break;
		}
		if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
			if (ic->ic_flags & IEEE80211_F_WEPON) {
				skb = ieee80211_wep_crypt(dev, skb, 0);
				if (skb == NULL)
					goto err;
				wh = (struct ieee80211_frame *) skb->data;
			} else
				goto out;
		}
		/* copy to listener after decrypt */
		skb = ieee80211_decap(dev, skb);
		if (skb == NULL) {
			if (netif_msg_debug(ic))
				printk("%s: decapsulation failed\n", dev->name);
			goto err;
		}

		/* perform as a bridge within the AP */
		skb1 = NULL;
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			eh = (struct ether_header *) skb->data;
			if (ETHER_IS_MULTICAST(eh->ether_dhost)) {
				skb1 = skb_copy(skb, 0);
			} else {
				ni = ieee80211_find_node(ic, eh->ether_dhost);
				if (ni != NULL && ni->ni_associd != 0) {
					skb1 = skb;
					skb = NULL;
				}
			}
			if (skb1 != NULL) {
				len = skb1->len;
				skb1->dev = dev;
				skb1->protocol = __constant_htons(ETH_P_802_2);
				dev_queue_xmit(skb1);
			}
		}
		if (skb != NULL) {
			dev->last_rx = jiffies;
			skb->dev = dev;
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
		}
		return;

	case IEEE80211_FC0_TYPE_MGT:
		if (dir != IEEE80211_FC1_DIR_NODS)
			goto err;
		if (ic->ic_opmode == IEEE80211_M_AHDEMO)
			goto out;
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

		/* drop frames without interest */
		if (ic->ic_state == IEEE80211_S_SCAN) {
			if (subtype != IEEE80211_FC0_SUBTYPE_BEACON &&
			    subtype != IEEE80211_FC0_SUBTYPE_PROBE_RESP)
				goto out;
		} else {
			if (ic->ic_opmode != IEEE80211_M_IBSS &&
			    subtype == IEEE80211_FC0_SUBTYPE_BEACON)
				goto out;
		}

		if (netif_msg_debug(ic)) {
			/* avoid to print too many frames */
			int doprint = 0;

			switch (subtype) {
			case IEEE80211_FC0_SUBTYPE_BEACON:
				if (ic->ic_state == IEEE80211_S_SCAN)
					doprint = 1;
				break;
			case IEEE80211_FC0_SUBTYPE_PROBE_REQ:
				if (ic->ic_opmode == IEEE80211_M_IBSS)
					doprint = 1;
				break;
			default:
				doprint = 1;
				break;
			}
#ifdef IEEE80211_DEBUG
			doprint += ieee80211_debug;
#endif
			if (doprint)
				printk("%s: received %s from %s rssi %d\n",
				    dev->name,
				    ieee80211_mgt_subtype_name[subtype
				    >> IEEE80211_FC0_SUBTYPE_SHIFT],
				    ether_sprintf(wh->i_addr2), rssi);
		}
		rh = ic->ic_recv_mgmt[subtype >> IEEE80211_FC0_SUBTYPE_SHIFT];
		if (rh != NULL)
			(*rh)(ic, skb, rssi, rstamp);
		dev_kfree_skb(skb);
		return;

	case IEEE80211_FC0_TYPE_CTL:
	default:
		DPRINTF(("ieee80211_input: bad type %x\n", wh->i_fc[0]));
		/* should not come here */
		break;
	}
  err:
	ic->ic_stats.rx_errors++;		/* XXX */
  out:
	if (skb != NULL)
		dev_kfree_skb(skb);
}

int
ieee80211_mgmt_output(struct net_device *dev, struct ieee80211_node *ni,
    struct sk_buff *skb, int type)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_frame *wh;

	if (ni == NULL)
		ni = &ic->ic_bss;
	ni->ni_inact = 0;

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT | type;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(u_int16_t *)wh->i_dur = 0;
	*(u_int16_t *)wh->i_seq =
	    cpu_to_le16(ni->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
	ni->ni_txseq++;
	IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, dev->dev_addr);
	IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);

	if (netif_msg_debug(ic)) {
		/* avoid to print too many frames */
		if (ic->ic_opmode == IEEE80211_M_IBSS ||
#ifdef IEEE80211_DEBUG
		    ieee80211_debug > 1 ||
#endif
		    (type & IEEE80211_FC0_SUBTYPE_MASK) !=
		    IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			printk("%s: sending %s to %s on channel %u\n",
			    dev->name,
			    ieee80211_mgt_subtype_name[
			    (type & IEEE80211_FC0_SUBTYPE_MASK)
			    >> IEEE80211_FC0_SUBTYPE_SHIFT],
			    ether_sprintf(ni->ni_macaddr),
			    ieee80211_chan2ieee(ic, ni->ni_chan));
	}
	ic->ic_timer = 1;
	(void) (*ic->ic_mgtstart)(skb, dev);
	return 0;
}

struct sk_buff *
ieee80211_encap(struct net_device *dev, struct sk_buff *skb)
{
	struct ieee80211com *ic = (void *)dev;
	struct ether_header eh;
	struct ieee80211_frame *wh;
	struct llc *llc;
	struct ieee80211_node *ni;

	memcpy(&eh, skb->data, sizeof(struct ether_header));
	skb_pull(skb, sizeof(struct ether_header));

	if (!IEEE80211_IS_MULTICAST(eh.ether_dhost) &&
	    (ic->ic_opmode == IEEE80211_M_IBSS ||
	     ic->ic_opmode == IEEE80211_M_HOSTAP)) {
		ni = ieee80211_find_node(ic, eh.ether_dhost);
		if (ni == NULL)
			ni = &ic->ic_bss;	/*XXX*/
	} else
		ni = &ic->ic_bss;
	ni->ni_inact = 0;

	llc = (struct llc *) skb_push(skb, sizeof(struct llc));
	llc->llc_dsap = llc->llc_ssap = LLC_SNAP_LSAP;
	llc->llc_control = LLC_UI;
	llc->llc_snap.org_code[0] = 0;
	llc->llc_snap.org_code[1] = 0;
	llc->llc_snap.org_code[2] = 0;
	llc->llc_snap.ether_type = eh.ether_type;

	/*
	 * XXX If we're loaded as a module the system may not be
	 * configured to leave enough headroom for us to push the
	 * 802.11 frame.  In that case fallback on reallocating
	 * the frame with enough space.  Alternatively we can carry
	 * the frame separately and use s/g support in the hardware.
	 */
	if (skb_headroom(skb) < sizeof(struct ieee80211_frame) &&
	    pskb_expand_head(skb, sizeof(*wh), 0, GFP_ATOMIC)) {
		dev_kfree_skb(skb);
		return NULL;
	}
	wh = (struct ieee80211_frame *) skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA;
	*(u_int16_t *)wh->i_dur = 0;
	*(u_int16_t *)wh->i_seq =
	    cpu_to_le16(ni->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
	ni->ni_txseq++;
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		wh->i_fc[1] = IEEE80211_FC1_DIR_TODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_bssid);
		IEEE80211_ADDR_COPY(wh->i_addr2, eh.ether_shost);
		IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_dhost);
		break;
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
		wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
		IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
		IEEE80211_ADDR_COPY(wh->i_addr2, eh.ether_shost);
		IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);
		break;
	case IEEE80211_M_HOSTAP:
		wh->i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
		IEEE80211_ADDR_COPY(wh->i_addr1, eh.ether_dhost);
		IEEE80211_ADDR_COPY(wh->i_addr2, ni->ni_bssid);
		IEEE80211_ADDR_COPY(wh->i_addr3, eh.ether_shost);
		break;
	}
	return skb;
}

struct sk_buff *
ieee80211_decap(struct net_device *dev, struct sk_buff *skb)
{
	struct ether_header *eh;
	struct ieee80211_frame wh;
	struct llc *llc;

	memcpy(&wh, skb->data, sizeof(struct ieee80211_frame));
	llc = (struct llc *) skb_pull(skb, sizeof(struct ieee80211_frame));
	if (llc->llc_dsap == LLC_SNAP_LSAP && llc->llc_ssap == LLC_SNAP_LSAP &&
	    llc->llc_control == LLC_UI && llc->llc_snap.org_code[0] == 0 &&
	    llc->llc_snap.org_code[1] == 0 && llc->llc_snap.org_code[2] == 0) {
		skb_pull(skb, sizeof(struct llc));
		llc = NULL;
	}
	eh = (struct ether_header *) skb_push(skb, sizeof(struct ether_header));
	switch (wh.i_fc[1] & IEEE80211_FC1_DIR_MASK) {
	case IEEE80211_FC1_DIR_NODS:
		IEEE80211_ADDR_COPY(eh->ether_dhost, wh.i_addr1);
		IEEE80211_ADDR_COPY(eh->ether_shost, wh.i_addr2);
		break;
	case IEEE80211_FC1_DIR_TODS:
		IEEE80211_ADDR_COPY(eh->ether_dhost, wh.i_addr3);
		IEEE80211_ADDR_COPY(eh->ether_shost, wh.i_addr2);
		break;
	case IEEE80211_FC1_DIR_FROMDS:
		IEEE80211_ADDR_COPY(eh->ether_dhost, wh.i_addr1);
		IEEE80211_ADDR_COPY(eh->ether_shost, wh.i_addr3);
		break;
	case IEEE80211_FC1_DIR_DSTODS:
		/* not yet supported */
		DPRINTF(("ieee80211_decap: DS to DS\n"));
		dev_kfree_skb(skb);
		return NULL;
	}
	if (!ALIGNED_POINTER(skb->data + sizeof(*eh), u_int32_t)) {
		struct sk_buff *n;

		/* XXX does this always work? */
		n = skb_copy(skb, 0);
		dev_kfree_skb(skb);
		if (n == NULL)
			return NULL;
		skb = n;
		eh = (struct ether_header *) skb->data;
	}
	if (llc != NULL)
		eh->ether_type = htons(skb->len - sizeof(*eh));
	return skb;
}

void
ieee80211_print_essid(u_int8_t *essid, int len)
{
	int i;
	u_int8_t *p; 

	if (len > IEEE80211_NWID_LEN)
		len = IEEE80211_NWID_LEN;
	/* determine printable or not */
	for (i = 0, p = essid; i < len; i++, p++) {
		if (*p < ' ' || *p > 0x7e)
			break;
	}
	if (i == len) {
		printk("\"");
		for (i = 0, p = essid; i < len; i++, p++)
			printk("%c", *p);
		printk("\"");
	} else {
		printk("0x");
		for (i = 0, p = essid; i < len; i++, p++)
			printk("%02x", *p);
	}
}

void
ieee80211_dump_pkt(u_int8_t *buf, int len, int rate, int rssi)
{
	struct ieee80211_frame *wh;
	int i;

	wh = (struct ieee80211_frame *)buf;
	switch (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) {
	case IEEE80211_FC1_DIR_NODS:
		printk("NODS %s", ether_sprintf(wh->i_addr2));
		printk("->%s", ether_sprintf(wh->i_addr1));
		printk("(%s)", ether_sprintf(wh->i_addr3));
		break;
	case IEEE80211_FC1_DIR_TODS:
		printk("TODS %s", ether_sprintf(wh->i_addr2));
		printk("->%s", ether_sprintf(wh->i_addr3));
		printk("(%s)", ether_sprintf(wh->i_addr1));
		break;
	case IEEE80211_FC1_DIR_FROMDS:
		printk("FRDS %s", ether_sprintf(wh->i_addr3));
		printk("->%s", ether_sprintf(wh->i_addr1));
		printk("(%s)", ether_sprintf(wh->i_addr2));
		break;
	case IEEE80211_FC1_DIR_DSTODS:
		printk("DSDS %s", ether_sprintf((u_int8_t *)&wh[1]));
		printk("->%s", ether_sprintf(wh->i_addr3));
		printk("(%s", ether_sprintf(wh->i_addr2));
		printk("->%s)", ether_sprintf(wh->i_addr1));
		break;
	}
	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_DATA:
		printk(" data");
		break;
	case IEEE80211_FC0_TYPE_MGT:
		printk(" %s", ieee80211_mgt_subtype_name[
		    (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
		    >> IEEE80211_FC0_SUBTYPE_SHIFT]);
		break;
	default:
		printk(" type#%d", wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK);
		break;
	}
	if (wh->i_fc[1] & IEEE80211_FC1_WEP)
		printk(" WEP");
	if (rate >= 0)
		printk(" %dM", rate / 2);
	if (rssi >= 0)
		printk(" +%d", rssi);
	printk("\n");
	if (len > 0) {
		for (i = 0; i < len; i++) {
			if ((i & 1) == 0)
				printk(" ");
			printk("%02x", buf[i]);
		}
		printk("\n");
	}
}

void
ieee80211_watchdog(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;

	if (ic->ic_mgt_timer) {
		if (--ic->ic_mgt_timer == 0)
			ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
	}
	if (ic->ic_inact_timer) {
		if (--ic->ic_inact_timer == 0) {
			struct ieee80211_node *ni, *nextbs;

			for (ni = TAILQ_FIRST(&ic->ic_node); ni != NULL; ) {
				if (++ni->ni_inact <= IEEE80211_INACT_MAX) {
					ni = TAILQ_NEXT(ni, ni_list);
					continue;
				}
				if (netif_msg_debug(ic))
					printk("%s: station %s deauthenticate"
					    " (reason %d)\n",
					    dev->name,
					    ether_sprintf(ni->ni_macaddr),
					    IEEE80211_REASON_AUTH_EXPIRE);
				nextbs = TAILQ_NEXT(ni, ni_list);
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_DEAUTH,
				    IEEE80211_REASON_AUTH_EXPIRE);
				ieee80211_free_node(ic, ni);
				ni = nextbs;
			}
			if (!TAILQ_EMPTY(&ic->ic_node))
				ic->ic_inact_timer = IEEE80211_INACT_WAIT;
		}
	}
	if (ic->ic_mgt_timer != 0 || ic->ic_inact_timer != 0)
		ic->ic_timer = 1;
}

void
ieee80211_begin_scan(struct net_device *dev, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = (void *)dev;

	memcpy(ic->ic_chan_scan, ic->ic_chan_active,
			sizeof(ic->ic_chan_active));
	clrbit(ic->ic_chan_scan, ieee80211_chan2ieee(ic, ni->ni_chan));
	ic->ic_flags |= IEEE80211_F_ASCAN;
	IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_PROBE_REQ, 0);
}

void
ieee80211_next_scan(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211channel *chan;

	chan = ic->ic_bss.ni_chan;
	for (;;) {
		if (++chan > &ic->ic_channels[IEEE80211_CHAN_MAX])
			chan = &ic->ic_channels[0];
		if (isset(ic->ic_chan_scan, ieee80211_chan2ieee(ic, chan)))
			break;
		if (chan == ic->ic_bss.ni_chan) {
			ieee80211_end_scan(dev);
			return;
		}
	}
	clrbit(ic->ic_chan_scan, ieee80211_chan2ieee(ic, chan));
	DPRINTF(("ieee80211_next_scan: chan %d->%d\n",
	    ieee80211_chan2ieee(ic, ic->ic_bss.ni_chan),
	    ieee80211_chan2ieee(ic, chan)));
	ic->ic_bss.ni_chan = chan;
	ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
}

void
ieee80211_end_scan(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni, *nextbs, *selbs;
	void *p;
	u_int8_t rate;
	int i, fail;

	ni = TAILQ_FIRST(&ic->ic_node);
	if (ni == NULL) {
		DPRINTF(("ieee80211_end_scan: no scan candidate\n"));
  notfound:
		if (ic->ic_opmode == IEEE80211_M_IBSS &&
		    (ic->ic_flags & IEEE80211_F_IBSSON) &&
		    ic->ic_des_esslen != 0) {
			ni = &ic->ic_bss;
			if (netif_msg_debug(ic))
				printk("%s: creating ibss\n", dev->name);
			ic->ic_flags |= IEEE80211_F_SIBSS;
			ni->ni_nrate = 0;
			for (i = 0; i < IEEE80211_RATE_SIZE; i++) {
				if (ic->ic_sup_rates[i])
					ni->ni_rates[ni->ni_nrate++] =
					    ic->ic_sup_rates[i];
			}
			IEEE80211_ADDR_COPY(ni->ni_macaddr, dev->dev_addr);
			IEEE80211_ADDR_COPY(ni->ni_bssid, dev->dev_addr);
			ni->ni_bssid[0] |= 0x02;	/* local bit for IBSS */
			ni->ni_esslen = ic->ic_des_esslen;
			memcpy(ni->ni_essid, ic->ic_des_essid, ni->ni_esslen);
			ni->ni_rssi = 0;
			ni->ni_rstamp = 0;
			memset(ni->ni_tstamp, 0, sizeof(ni->ni_tstamp));
			ni->ni_intval = ic->ic_lintval;
			ni->ni_capinfo = IEEE80211_CAPINFO_IBSS;
			if (ic->ic_flags & IEEE80211_F_WEPON)
				ni->ni_capinfo |= IEEE80211_CAPINFO_PRIVACY;
			ni->ni_chan = ic->ic_ibss_chan;
			if (ic->ic_phytype == IEEE80211_T_FH) {
				ni->ni_fhdwell = 200;	/* XXX */
				ni->ni_fhindex = 1;
			}
			ieee80211_new_state(dev, IEEE80211_S_RUN, -1);
			return;
		}
		if (ic->ic_flags & IEEE80211_F_ASCAN) {
			if (netif_msg_debug(ic))
				printk("%s: entering passive scan mode\n",
					dev->name);
			ic->ic_flags &= ~IEEE80211_F_ASCAN;
		}
		/*
		 * Reset the list of channels to scan and start again.
		 */
		memcpy(ic->ic_chan_scan, ic->ic_chan_active,
			sizeof(ic->ic_chan_active));
		ieee80211_next_scan(dev);
		return;
	}
	selbs = NULL;
	if (netif_msg_debug(ic))
		printk("%s: \tmacaddr          bssid         chan  rssi rate flag  wep  essid\n", dev->name);
	for (; ni != NULL; ni = nextbs) {
		nextbs = TAILQ_NEXT(ni, ni_list);
		if (ni->ni_fails) {
			/*
			 * The configuration of the access points may change
			 * during my scan.  So delete the entry for the AP
			 * and retry to associate if there is another beacon.
			 */
			if (ni->ni_fails++ > 2)
				ieee80211_free_node(ic, ni);
			continue;
		}
		fail = 0;
		if (isclr(ic->ic_chan_active, ieee80211_chan2ieee(ic, ni->ni_chan)))
			fail |= 0x01;
		if (ic->ic_des_chan != (struct ieee80211channel *) IEEE80211_CHAN_ANY &&
		    ni->ni_chan != ic->ic_des_chan)
			fail |= 0x01;
		if (ic->ic_opmode == IEEE80211_M_IBSS) {
			if ((ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) == 0)
				fail |= 0x02;
		} else {
			if ((ni->ni_capinfo & IEEE80211_CAPINFO_ESS) == 0)
				fail |= 0x02;
		}
		if (ic->ic_flags & IEEE80211_F_WEPON) {
			if ((ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) == 0)
				fail |= 0x04;
		} else {
			if (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY)
				fail |= 0x04;
		}
		rate = ieee80211_fix_rate(ic, ni, IEEE80211_F_DONEGO);
		if (rate & IEEE80211_RATE_BASIC)
			fail |= 0x08;
		if (ic->ic_des_esslen != 0 &&
		    (ni->ni_esslen != ic->ic_des_esslen ||
		     memcmp(ni->ni_essid, ic->ic_des_essid,
		     ic->ic_des_esslen != 0)))
			fail |= 0x10;
		if ((ic->ic_flags & IEEE80211_F_DESBSSID) &&
		    !IEEE80211_ADDR_EQ(ic->ic_des_bssid, ni->ni_bssid))
			fail |= 0x20;
		if (netif_msg_debug(ic)) {
			printk(" %c %s", fail ? '-' : '+',
			    ether_sprintf(ni->ni_macaddr));
			printk(" %s%c", ether_sprintf(ni->ni_bssid),
			    fail & 0x20 ? '!' : ' ');
			printk(" %3d%c", ieee80211_chan2ieee(ic, ni->ni_chan),
				fail & 0x01 ? '!' : ' ');
			printk(" %+4d", ni->ni_rssi);
			printk(" %2dM%c", (rate & IEEE80211_RATE_VAL) / 2,
			    fail & 0x08 ? '!' : ' ');
			printk(" %4s%c",
			    (ni->ni_capinfo & IEEE80211_CAPINFO_ESS) ? "ess" :
			    (ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) ? "ibss" :
			    "????",
			    fail & 0x02 ? '!' : ' ');
			printk(" %3s%c ",
			    (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) ?
			    "wep" : "no",
			    fail & 0x04 ? '!' : ' ');
			ieee80211_print_essid(ni->ni_essid, ni->ni_esslen);
			printk("%s\n", fail & 0x10 ? "!" : "");
		}
		if (!fail) {
			if (selbs == NULL || ni->ni_rssi > selbs->ni_rssi)
				selbs = ni;
		}
	}
	if (selbs == NULL)
		goto notfound;
	p = ic->ic_bss.ni_private;
	ic->ic_bss = *selbs;
	ic->ic_bss.ni_private = p;
	if (p != NULL && ic->ic_node_privlen)
		memcpy(p, selbs->ni_private, ic->ic_node_privlen);
	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		ieee80211_fix_rate(ic, &ic->ic_bss, IEEE80211_F_DOFRATE |
		    IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
		if (ic->ic_bss.ni_nrate == 0) {
			selbs->ni_fails++;
			goto notfound;
		}
		ieee80211_new_state(dev, IEEE80211_S_RUN, -1);
	} else
		ieee80211_new_state(dev, IEEE80211_S_AUTH, -1);
}

struct ieee80211_node *
ieee80211_alloc_node(struct ieee80211com *ic, u_int8_t *macaddr, int copy)
{
	struct ieee80211_node *ni;
	int hash;

	ni = kmalloc(sizeof(struct ieee80211_node) + ic->ic_node_privlen,
		GFP_KERNEL);
	if (ni == NULL)
		return NULL;
	if (copy)
		memcpy(ni, &ic->ic_bss, sizeof(struct ieee80211_node));
	else
		memset(ni, 0, sizeof(struct ieee80211_node));
	IEEE80211_ADDR_COPY(ni->ni_macaddr, macaddr);
	if (ic->ic_node_privlen) {
		ni->ni_private = &ni[1];
		memset(ni->ni_private, 0, ic->ic_node_privlen);
	} else
		ni->ni_private = NULL;

	hash = IEEE80211_NODE_HASH(macaddr);
	IEEE80211_LOCK(ic);
	TAILQ_INSERT_TAIL(&ic->ic_node, ni, ni_list);
	LIST_INSERT_HEAD(&ic->ic_hash[hash], ni, ni_hash);
	IEEE80211_UNLOCK(ic);
	ic->ic_inact_timer = IEEE80211_INACT_WAIT;
	return ni;
}

struct ieee80211_node *
ieee80211_find_node(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;
	int hash;

	hash = IEEE80211_NODE_HASH(macaddr);
	IEEE80211_LOCK(ic);
	LIST_FOREACH(ni, &ic->ic_hash[hash], ni_hash) {
		if (IEEE80211_ADDR_EQ(ni->ni_macaddr, macaddr))
			break;
	}
	IEEE80211_UNLOCK(ic);
	return ni;
}

void
ieee80211_free_node(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	IEEE80211_LOCK(ic);
	if (ic->ic_node_free != NULL)
		(*ic->ic_node_free)(ic, ni);
	TAILQ_REMOVE(&ic->ic_node, ni, ni_list);
	LIST_REMOVE(ni, ni_hash);
	IEEE80211_UNLOCK(ic);
	kfree(ni);
	if (TAILQ_EMPTY(&ic->ic_node))
		ic->ic_inact_timer = 0;
}

void
ieee80211_free_allnodes(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;

	while ((ni = TAILQ_FIRST(&ic->ic_node)) != NULL)
		ieee80211_free_node(ic, ni);  
}

int
ieee80211_fix_rate(struct ieee80211com *ic, struct ieee80211_node *ni, int flags)
{
	int i, j, ignore, error;
	int okrate, badrate;
	u_int8_t r;

	error = 0;
	okrate = badrate = 0;
	for (i = 0; i < ni->ni_nrate; ) {
		ignore = 0;
		if (flags & IEEE80211_F_DOSORT) {
			for (j = i + 1; j < ni->ni_nrate; j++) {
				if ((ni->ni_rates[i] & IEEE80211_RATE_VAL) >
				    (ni->ni_rates[j] & IEEE80211_RATE_VAL)) {
					r = ni->ni_rates[i];
					ni->ni_rates[i] = ni->ni_rates[j];
					ni->ni_rates[j] = r;
				}
			}
		}
		r = ni->ni_rates[i] & IEEE80211_RATE_VAL;
		badrate = r;
		if (flags & IEEE80211_F_DOFRATE) {
			if (ic->ic_fixed_rate >= 0 &&
			    r != (ic->ic_sup_rates[ic->ic_fixed_rate] &
			    IEEE80211_RATE_VAL))
				ignore++;
		}
		if (flags & IEEE80211_F_DONEGO) {
			for (j = 0; j < IEEE80211_RATE_SIZE; j++) {
				if (r ==
				    (ic->ic_sup_rates[j] & IEEE80211_RATE_VAL))
					break;
			}
			if (j == IEEE80211_RATE_SIZE) {
				if (ni->ni_rates[i] & IEEE80211_RATE_BASIC)
					error++;
				ignore++;
			}
		}
		if (flags & IEEE80211_F_DODEL) {
			if (ignore) {
				ni->ni_nrate--;
				for (j = i; j < ni->ni_nrate; j++)
					ni->ni_rates[j] = ni->ni_rates[j + 1];
				ni->ni_rates[j] = 0;
				continue;
			}
		}
		if (!ignore)
			okrate = ni->ni_rates[i];
		i++;
	}
	if (okrate == 0 || error != 0)
		return badrate | IEEE80211_RATE_BASIC;
	return okrate & IEEE80211_RATE_VAL;
}

static u_int8_t *
ieee80211_add_rates(u_int8_t *frm, const u_int8_t rates[IEEE80211_RATE_SIZE])
{
	int i, j;

	*frm++ = IEEE80211_ELEMID_RATES;
	j = 0;
	for (i = 0; i < IEEE80211_RATE_SIZE; i++) {
		if (rates[i] != 0) {
			frm[1 + j] = rates[i];
			j++;
		}
	}
	*frm++ = j;
	return frm + j;
}

static u_int8_t *
ieee80211_add_ssid(u_int8_t *frm, const u_int8_t *ssid, u_int len)
{
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = len;
	memcpy(frm, ssid, len);
	return frm + len;
}

static int
ieee80211_send_prreq(struct ieee80211com *ic, struct ieee80211_node *ni,
    int type, int dummy)
{
	int ret, pktlen;
	struct sk_buff *skb;
	u_int8_t *frm;

	/*
	 * prreq frame format
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 */
	pktlen = 2 + ic->ic_des_esslen + 1 + IEEE80211_RATE_SIZE + 1;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL) {
		DPRINTF(("ieee80211_send_prreq: no space\n"));
		return ENOMEM;
	}
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = skb_put(skb, pktlen);
	frm = ieee80211_add_ssid(frm, ic->ic_des_essid, ic->ic_des_esslen);
	frm = ieee80211_add_rates(frm, ic->ic_sup_rates);
	skb_trim(skb, frm - skb->data);

	ret = ieee80211_mgmt_output(&ic->ic_dev, ni, skb, type);
	ic->ic_mgt_timer = IEEE80211_TRANS_WAIT;
	return ret;
}

static int
ieee80211_send_prresp(struct ieee80211com *ic, struct ieee80211_node *bs0,
    int type, int dummy)
{
	struct sk_buff *skb;
	u_int8_t *frm;
	struct ieee80211_node *ni = &ic->ic_bss;
	u_int16_t capinfo;
	int pktlen;

	/*
	 * probe response frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[tlv] parameter set (IBSS)
	 */
	pktlen = 8 + 2 + 2 + 2
	       + 2 + ni->ni_esslen
	       + 2 + ni->ni_nrate 
	       + 6;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = skb_put(skb, pktlen);

	memset(frm, 0, 8);	/* timestamp should be filled later */
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

	frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
	frm = ieee80211_add_rates(frm, ni->ni_rates);

	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		*frm++ = IEEE80211_ELEMID_IBSSPARMS;
		*frm++ = 2;
		*frm++ = 0; *frm++ = 0;		/* TODO: ATIM window */
	} else {	/* IEEE80211_M_HOSTAP */
		/* TODO: TIM */
		*frm++ = IEEE80211_ELEMID_TIM;
		*frm++ = 4;	/* length */
		*frm++ = 0;	/* DTIM count */
		*frm++ = 1;	/* DTIM period */
		*frm++ = 0;	/* bitmap control */
		*frm++ = 0;	/* Partial Virtual Bitmap (variable length) */
	}
	skb_trim(skb, frm - skb->data);

	return ieee80211_mgmt_output(&ic->ic_dev, bs0, skb, type);
}

static int
ieee80211_send_auth(struct ieee80211com *ic, struct ieee80211_node *ni,
    int type, int seq)
{
	struct sk_buff *skb;
	u_int16_t *frm;
	int ret, pktlen;

	pktlen = 3*sizeof(u_int16_t);
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	/* XXX alignment? */
	frm = (u_int16_t *) skb_put(skb, pktlen);
	/* TODO: shared key auth */
	frm[0] = cpu_to_le16(IEEE80211_AUTH_ALG_OPEN);
	frm[1] = cpu_to_le16(seq);
	frm[2] = 0;			/* status */
	ret = ieee80211_mgmt_output(&ic->ic_dev, ni, skb, type);
	if (ic->ic_opmode == IEEE80211_M_STA)
		ic->ic_mgt_timer = IEEE80211_TRANS_WAIT;
	return ret;
}

static int
ieee80211_send_deauth(struct ieee80211com *ic, struct ieee80211_node *ni,
    int type, int reason)
{
	struct net_device *dev = &ic->ic_dev;
	struct sk_buff *skb;
	u_int16_t *frm;
	int pktlen;

	if (netif_msg_debug(ic))
		printk("%s: station %s deauthenticate (reason %d)\n",
		    dev->name, ether_sprintf(ni->ni_macaddr), reason);
	pktlen = sizeof(u_int16_t);
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	/* XXX alignment? */
	frm = (u_int16_t *) skb_put(skb, pktlen);
	frm[0] = cpu_to_le16(reason);

	return ieee80211_mgmt_output(&ic->ic_dev, ni, skb, type);
}

static int
ieee80211_send_asreq(struct ieee80211com *ic, struct ieee80211_node *ni,
    int type, int dummy)
{
	struct sk_buff *skb;
	u_int8_t *frm;
	u_int16_t capinfo;
	int ret, pktlen;

	/*
	 * asreq frame format
	 *	[2] capability information
	 *	[2] listen interval
	 *	[6*] current AP address (reassoc only)
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 */
	pktlen = sizeof(capinfo)
	       + sizeof(u_int16_t)
	       + IEEE80211_ADDR_LEN
	       + 2 + ni->ni_esslen
	       + 1 + IEEE80211_RATE_SIZE;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = (u_int8_t *) skb_put(skb, pktlen);

	capinfo = 0;
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo |= IEEE80211_CAPINFO_IBSS;
	else		/* IEEE80211_M_STA */
		capinfo |= IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;

	*(u_int16_t *)frm = cpu_to_le16(ic->ic_lintval);
	frm += 2;

	if (type == IEEE80211_FC0_SUBTYPE_REASSOC_REQ) {
		IEEE80211_ADDR_COPY(frm, ic->ic_bss.ni_bssid);
		frm += IEEE80211_ADDR_LEN;
	}

	frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
	frm = ieee80211_add_rates(frm, ic->ic_sup_rates);
	skb_trim(skb, frm - skb->data);

	ret = ieee80211_mgmt_output(&ic->ic_dev, ni, skb, type);
	ic->ic_mgt_timer = IEEE80211_TRANS_WAIT;
	return ret;
}

static int
ieee80211_send_asresp(struct ieee80211com *ic, struct ieee80211_node *ni,
    int type, int status)
{
	struct sk_buff *skb;
	u_int8_t *frm;
	u_int16_t capinfo;
	int pktlen;

	/*
	 * asreq frame format
	 *	[2] capability information
	 *	[2] status
	 *	[2] association ID
	 *	[tlv] supported rates
	 */
	pktlen = sizeof(capinfo)
	       + sizeof(u_int16_t)
	       + sizeof(u_int16_t)
	       + 1 + IEEE80211_RATE_SIZE;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = (u_int8_t *) skb_put(skb, pktlen);

	capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;

	*(u_int16_t *)frm = cpu_to_le16(status);
	frm += 2;

	if (status == IEEE80211_STATUS_SUCCESS && ni != NULL)
		*(u_int16_t *)frm = cpu_to_le16(ni->ni_associd);
	else
		*(u_int16_t *)frm = cpu_to_le16(0);
	frm += 2;

	if (ni != NULL)
		frm = ieee80211_add_rates(frm, ni->ni_rates);
	else
		frm = ieee80211_add_rates(frm, ic->ic_bss.ni_rates);
	skb_trim(skb, frm - skb->data);

	return ieee80211_mgmt_output(&ic->ic_dev, ni, skb, type);
}

static int
ieee80211_send_disassoc(struct ieee80211com *ic, struct ieee80211_node *ni,
    int type, int reason)
{
	struct net_device *dev = &ic->ic_dev;
	struct sk_buff *skb;
	u_int16_t *frm;
	int pktlen;

	if (netif_msg_debug(ic))
		printk("%s: station %s disassociate (reason %d)\n",
		    dev->name, ether_sprintf(ni->ni_macaddr), reason);
	pktlen = sizeof(u_int16_t);
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	/* XXX alignment? */
	frm = (u_int16_t *) skb_put(skb, pktlen);
	frm[0] = cpu_to_le16(reason);

	return ieee80211_mgmt_output(&ic->ic_dev, ni, skb,
	    IEEE80211_FC0_SUBTYPE_DISASSOC);
}

static void
ieee80211_recv_beacon(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm, *tstamp, *bintval, *capinfo, *ssid, *rates;
	u_int8_t chan, fhindex;
	u_int16_t fhdwell;

	if (ic->ic_opmode != IEEE80211_M_IBSS &&
	    ic->ic_state != IEEE80211_S_SCAN) {
		/* XXX: may be useful for background scan */
		return;
	}

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	/*
	 * beacon frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[tlv] parameter set (FH/DS)
	 */
	tstamp  = frm;	frm += 8;
	bintval = frm;	frm += 2;
	capinfo = frm;	frm += 2;
	ssid = rates = NULL;
	chan = ieee80211_chan2ieee(ic, ic->ic_bss.ni_chan);
	fhdwell = 0;
	fhindex = 0;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_SSID:
			ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		case IEEE80211_ELEMID_FHPARMS:
			if (ic->ic_phytype == IEEE80211_T_FH) {
				fhdwell = (frm[3] << 8) | frm[2];
				chan = IEEE80211_FH_CHAN(frm[4], frm[5]);
				fhindex = frm[6];
			}
			break;
		case IEEE80211_ELEMID_DSPARMS:
			if (ic->ic_phytype == IEEE80211_T_DS)
				chan = frm[2];
			break;
		}
		frm += frm[1] + 2;
	}
	if (ssid == NULL || rates == NULL) {
		DPRINTF(("ieee80211_recv_beacon: ssid=%p, rates=%p, chan=%d\n",
		    ssid, rates, chan));
		return;
	}
	if (ssid[1] > IEEE80211_NWID_LEN) {
		DPRINTF(("ieee80211_recv_beacon: bad ssid len %d from %s\n",
		    ssid[1], ether_sprintf(wh->i_addr2)));
		return;
	}
	ni = ieee80211_find_node(ic, wh->i_addr2);
#ifdef IEEE80211_DEBUG
	if (ieee80211_debug &&
	    (ieee80211_debug > 1 || ni == NULL ||
	    ic->ic_state == IEEE80211_S_SCAN)) {
		printk("ieee80211_recv_prreq: %sbeacon on chan %u (bss chan %u) ",
		    (ni == NULL ? "new " : ""), chan,
		    ieee80211_chan2ieee(ic, ic->ic_bss.ni_chan));
		ieee80211_print_essid(ssid + 2, ssid[1]);
		printk(" from %s\n", ether_sprintf(wh->i_addr2));
	}
#endif
	if (ni == NULL) {
		if ((ni = ieee80211_alloc_node(ic, wh->i_addr2, 0)) == NULL)
			return;
		ni->ni_esslen = ssid[1];
		memset(ni->ni_essid, 0, sizeof(ni->ni_essid));
		memcpy(ni->ni_essid, ssid + 2, ssid[1]);
	} else {
		if (ssid[1] != 0) {
			/*
			 * Update ESSID at probe response to adopt hidden AP by
			 * Lucent/Cisco, which announces null ESSID in beacon.
			 */
			if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) ==
			    IEEE80211_FC0_SUBTYPE_PROBE_RESP) {
				ni->ni_esslen = ssid[1];
				memset(ni->ni_essid, 0, sizeof(ni->ni_essid));
				memcpy(ni->ni_essid, ssid + 2, ssid[1]);
			}
		}
	}
	IEEE80211_ADDR_COPY(ni->ni_bssid, wh->i_addr3);
	memset(ni->ni_rates, 0, IEEE80211_RATE_SIZE);
	ni->ni_nrate = rates[1];
	memcpy(ni->ni_rates, rates + 2, ni->ni_nrate);
	ieee80211_fix_rate(ic, ni, IEEE80211_F_DOSORT);
	ni->ni_rssi = rssi;
	ni->ni_rstamp = rstamp;
	memcpy(ni->ni_tstamp, tstamp, sizeof(ni->ni_tstamp));
	ni->ni_intval = le16_to_cpu(*(u_int16_t *)bintval);
	ni->ni_capinfo = le16_to_cpu(*(u_int16_t *)capinfo);
	/* XXX validate channel # */
	ni->ni_chan = &ic->ic_channels[chan];
	ni->ni_fhdwell = fhdwell;
	ni->ni_fhindex = fhindex;
	if (ic->ic_state == IEEE80211_S_SCAN &&
	    (ic->ic_flags & IEEE80211_F_ASCAN) == 0)
		ieee80211_end_scan(&ic->ic_dev);
}

static void
ieee80211_recv_prreq(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm, *ssid, *rates;
	u_int8_t rate;
	int allocbs;

	if (ic->ic_opmode == IEEE80211_M_STA)
		return;
	if (ic->ic_state != IEEE80211_S_RUN)
		return;

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	/*
	 * prreq frame format
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 */
	ssid = rates = NULL;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_SSID:
			ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		}
		frm += frm[1] + 2;
	}
	if (ssid == NULL || rates == NULL) {
		DPRINTF(("ieee80211_recv_prreq: ssid=%p, rates=%p\n",
		    ssid, rates));
		return;
	}
	if (ssid[1] != 0 &&
	    (ssid[1] != ic->ic_bss.ni_esslen ||
	    memcmp(ssid + 2, ic->ic_bss.ni_essid, ic->ic_bss.ni_esslen) != 0)) {
#ifdef IEEE80211_DEBUG
		if (ieee80211_debug) {
			printk("ieee80211_recv_prreq: ssid unmatch ");
			ieee80211_print_essid(ssid + 2, ssid[1]);
			printk(" from %s\n", ether_sprintf(wh->i_addr2));
		}
#endif
		return;
	}

	ni = ieee80211_find_node(ic, wh->i_addr2);
	if (ni == NULL) {
		if ((ni = ieee80211_alloc_node(ic, wh->i_addr2, 1)) == NULL)
			return;
		DPRINTF(("ieee80211_recv_prreq: new req from %s\n",
		    ether_sprintf(wh->i_addr2)));
		allocbs = 1;
	} else
		allocbs = 0;
	memset(ni->ni_rates, 0, IEEE80211_RATE_SIZE);
	ni->ni_nrate = rates[1];
	memcpy(ni->ni_rates, rates + 2, ni->ni_nrate);
	ni->ni_rssi = rssi;
	ni->ni_rstamp = rstamp;
	rate = ieee80211_fix_rate(ic, ni, IEEE80211_F_DOSORT |
	    IEEE80211_F_DOFRATE | IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
	if (rate & IEEE80211_RATE_BASIC) {
		DPRINTF(("ieee80211_recv_prreq: rate negotiation failed: %s\n",
		    ether_sprintf(wh->i_addr2)));
	} else {
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_PROBE_RESP,
		    0);
	}
	if (allocbs && (ic->ic_opmode == IEEE80211_M_HOSTAP))
		ieee80211_free_node(ic, ni);
}

static void
ieee80211_recv_auth(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm;
	u_int16_t algo, seq, status;
	int allocbs;

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	/*
	 * auth frame format
	 *	[2] algorithm
	 *	[2] sequence
	 *	[2] status
	 *	[tlv*] challenge
	 */
	if (frm + 6 > efrm) {
		DPRINTF(("ieee80211_recv_auth: too short from %s\n",
		    ether_sprintf(wh->i_addr2)));
		return;
	}
	algo   = le16_to_cpu(*(u_int16_t *)frm);
	seq    = le16_to_cpu(*(u_int16_t *)(frm + 2));
	status = le16_to_cpu(*(u_int16_t *)(frm + 4));
	if (algo != IEEE80211_AUTH_ALG_OPEN) {
		/* TODO: shared key auth */
		DPRINTF(("ieee80211_recv_auth: unsupported auth %d from %s\n",
		    algo, ether_sprintf(wh->i_addr2)));
		return;
	}
	switch (ic->ic_opmode) {
	case IEEE80211_M_IBSS:
		if (ic->ic_state != IEEE80211_S_RUN || seq != 1)
			return;
		ieee80211_new_state(&ic->ic_dev, IEEE80211_S_AUTH,
		    wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
		break;

	case IEEE80211_M_AHDEMO:
		/* should not come here */
		break;

	case IEEE80211_M_HOSTAP:
		if (ic->ic_state != IEEE80211_S_RUN || seq != 1)
			return;
		allocbs = 0;
		ni = ieee80211_find_node(ic, wh->i_addr2);
		if (ni == NULL) {
			ni = ieee80211_alloc_node(ic, wh->i_addr2, 0);
			if (ni == NULL)
				return;
			IEEE80211_ADDR_COPY(ni->ni_bssid, ic->ic_bss.ni_bssid);
			allocbs = 1;
		}
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_AUTH, 2);
		if (netif_msg_debug(ic))
			printk("%s: station %s %s authenticated\n",
			    dev->name,
			    (allocbs ? "newly" : "already"),
			    ether_sprintf(ni->ni_macaddr));
		break;

	case IEEE80211_M_STA:
		if (ic->ic_state != IEEE80211_S_AUTH || seq != 2)
			return;
		if (status != 0) {
			printk("%s: authentication failed (reason %d) for %s\n",
			    dev->name, status,
			    ether_sprintf(wh->i_addr3));
			ni = ieee80211_find_node(ic, wh->i_addr2);
			if (ni != NULL)
				ni->ni_fails++;
			return;
		}
		ieee80211_new_state(&ic->ic_dev, IEEE80211_S_ASSOC,
		    wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
		break;
	}
}

static void
ieee80211_recv_asreq(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = &ic->ic_bss;
	u_int8_t *frm, *efrm, *ssid, *rates;
	u_int16_t capinfo, bintval;
	int reassoc, resp, newassoc;

	if (ic->ic_opmode != IEEE80211_M_HOSTAP ||
	    (ic->ic_state != IEEE80211_S_RUN))
		return;

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	if ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) ==
	    IEEE80211_FC0_SUBTYPE_REASSOC_REQ) {
		reassoc = 1;
		resp = IEEE80211_FC0_SUBTYPE_REASSOC_RESP;
	} else {
		reassoc = 0;
		resp = IEEE80211_FC0_SUBTYPE_ASSOC_RESP;
	}
	/*
	 * asreq frame format
	 *	[2] capability information
	 *	[2] listen interval
	 *	[6*] current AP address (reassoc only)
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 */
	if (frm + (reassoc ? 10 : 4) > efrm) {
		DPRINTF(("ieee80211_recv_asreq: too short from %s\n",
		    ether_sprintf(wh->i_addr2)));
		return;
	}

	if (!IEEE80211_ADDR_EQ(wh->i_addr3, ic->ic_bss.ni_bssid)) {
		DPRINTF(("ieee80211_recv_asreq: ignore other bss from %s\n",
		    ether_sprintf(wh->i_addr2)));
		return;
	}
	capinfo = le16_to_cpu(*(u_int16_t *)frm);	frm += 2;
	bintval = le16_to_cpu(*(u_int16_t *)frm);	frm += 2;
	if (reassoc)
		frm += 6;	/* ignore current AP info */
	ssid = rates = NULL;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_SSID:
			ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		}
		frm += frm[1] + 2;
	}
	if (ssid == NULL || rates == NULL) {
		DPRINTF(("ieee80211_recv_asreq: ssid=%p, rates=%p\n",
		    ssid, rates));
		return;
	}
	if (ssid[1] > IEEE80211_NWID_LEN) {
		DPRINTF(("ieee80211_recv_asreq: bad ssid len %d from %s\n",
		    ssid[1], ether_sprintf(wh->i_addr2)));
		return;
	}
	if (ssid[1] != ic->ic_bss.ni_esslen ||
	    memcmp(ssid + 2, ic->ic_bss.ni_essid, ssid[1]) != 0) {
#ifdef IEEE80211_DEBUG
		if (ieee80211_debug) {
			printk("ieee80211_recv_asreq: ssid unmatch ");
			ieee80211_print_essid(ssid + 2, ssid[1]);
			printk(" from %s\n", ether_sprintf(wh->i_addr2));
		}
#endif
		return;
	}
	ni = ieee80211_find_node(ic, wh->i_addr2);
	if (ni == NULL) {
		DPRINTF(("ieee80211_recv_asreq: not authenticated for %s\n",
		    ether_sprintf(wh->i_addr2)));
		if ((ni = ieee80211_alloc_node(ic, wh->i_addr2, 1)) == NULL)
			return;
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_DEAUTH,
		    IEEE80211_REASON_ASSOC_NOT_AUTHED);
		ieee80211_free_node(ic, ni);
		return;
	}
	if ((capinfo & IEEE80211_CAPINFO_ESS) == 0 ||
	    (capinfo & IEEE80211_CAPINFO_PRIVACY) !=
	    ((ic->ic_flags & IEEE80211_F_WEPON) ?
	     IEEE80211_CAPINFO_PRIVACY : 0)) {
		DPRINTF(("ieee80211_recv_asreq: capability unmatch %x for %s\n",
		    capinfo, ether_sprintf(wh->i_addr2)));
		ni->ni_associd = 0;
		IEEE80211_SEND_MGMT(ic, ni, resp, IEEE80211_STATUS_CAPINFO);
		return;
	}
	memset(ni->ni_rates, 0, IEEE80211_RATE_SIZE);
	ni->ni_nrate = rates[1];
	memcpy(ni->ni_rates, rates + 2, ni->ni_nrate);
	ieee80211_fix_rate(ic, ni, IEEE80211_F_DOSORT | IEEE80211_F_DOFRATE |
	    IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
	if (ni->ni_nrate == 0) {
		DPRINTF(("ieee80211_recv_asreq: rate unmatch for %s\n",
		    ether_sprintf(wh->i_addr2)));
		ni->ni_associd = 0;
		IEEE80211_SEND_MGMT(ic, ni, resp, IEEE80211_STATUS_BASIC_RATE);
		return;
	}
	ni->ni_rssi = rssi;
	ni->ni_rstamp = rstamp;
	ni->ni_intval = bintval;
	ni->ni_capinfo = capinfo;
	ni->ni_chan = ic->ic_bss.ni_chan;
	ni->ni_fhdwell = ic->ic_bss.ni_fhdwell;
	ni->ni_fhindex = ic->ic_bss.ni_fhindex;
	if (ni->ni_associd == 0) {
		ni->ni_associd = 0xc000 | ic->ic_bss.ni_associd++;
		newassoc = 1;
	} else
		newassoc = 0;
	IEEE80211_SEND_MGMT(ic, ni, resp, IEEE80211_STATUS_SUCCESS);
	if (netif_msg_debug(ic))
		printk("%s: station %s %s associated\n",
		    dev->name,
		    (newassoc ? "newly" : "already"),
		    ether_sprintf(ni->ni_macaddr));
}

static void
ieee80211_recv_asresp(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = &ic->ic_bss;
	u_int8_t *frm, *efrm, *rates;
	int status;

	if (ic->ic_opmode != IEEE80211_M_STA ||
	    ic->ic_state != IEEE80211_S_ASSOC)
		return;

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	/*
	 * asresp frame format
	 *	[2] capability information
	 *	[2] status
	 *	[2] association ID
	 *	[tlv] supported rates
	 */
	if (frm + 6 > efrm) {
		DPRINTF(("ieee80211_recv_asresp: too short from %s\n",
		    ether_sprintf(wh->i_addr2)));
		return;
	}

	ni->ni_capinfo = le16_to_cpu(*(u_int16_t *)frm);
	frm += 2;

	status = le16_to_cpu(*(u_int16_t *)frm);
	frm += 2;
	if (status != 0) {
		printk("%s: association failed (reason %d) for %s\n",
		    dev->name, status, ether_sprintf(wh->i_addr3));
		ni = ieee80211_find_node(ic, wh->i_addr2);
		if (ni != NULL)
			ni->ni_fails++;
		return;
	}
	ni->ni_associd = le16_to_cpu(*(u_int16_t *)frm);
	frm += 2;
	rates = frm;

	memset(ni->ni_rates, 0, IEEE80211_RATE_SIZE);
	ni->ni_nrate = rates[1];
	memcpy(ni->ni_rates, rates + 2, ni->ni_nrate);
	ieee80211_fix_rate(ic, ni, IEEE80211_F_DOSORT | IEEE80211_F_DOFRATE |
	    IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
	if (ni->ni_nrate == 0)
		return;
	ieee80211_new_state(&ic->ic_dev, IEEE80211_S_RUN,
	    wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
}

static void
ieee80211_recv_disassoc(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm;
	u_int16_t reason;

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	/*
	 * disassoc frame format
	 *	[2] reason
	 */
	if (frm + 2 > efrm) {
		DPRINTF(("ieee80211_recv_disassoc: too short from %s\n",
		    ether_sprintf(wh->i_addr2)));
		return;
	}
	reason = le16_to_cpu(*(u_int16_t *)frm);
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		ieee80211_new_state(&ic->ic_dev, IEEE80211_S_ASSOC,
		    wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
		break;
	case IEEE80211_M_HOSTAP:
		if ((ni = ieee80211_find_node(ic, wh->i_addr2)) != NULL) {
			if (netif_msg_debug(ic))
				printk("%s: station %s disassociated"
				    " by peer (reason %d)\n",
				    dev->name,
				    ether_sprintf(ni->ni_macaddr), reason);
			ni->ni_associd = 0;
		}
		break;
	default:
		break;
	}
}

static void
ieee80211_recv_deauth(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm;
	u_int16_t reason;

	wh = (struct ieee80211_frame *) skb0->data;
	frm = (u_int8_t *)&wh[1];
	efrm = skb0->data + skb0->len;
	/*
	 * dauth frame format
	 *	[2] reason
	 */
	if (frm + 2 > efrm) {
		DPRINTF(("ieee80211_recv_deauth: too short from %s\n",
		    ether_sprintf(wh->i_addr2)));
		return;
	}
	reason = le16_to_cpu(*(u_int16_t *)frm);
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		ieee80211_new_state(&ic->ic_dev, IEEE80211_S_AUTH,
		    wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
		break;
	case IEEE80211_M_HOSTAP:
		if ((ni = ieee80211_find_node(ic, wh->i_addr2)) != NULL) {
			if (netif_msg_debug(ic))
				printk("%s: station %s deauthenticated"
				    " by peer (reason %d)\n",
				    dev->name,
				    ether_sprintf(ni->ni_macaddr), reason);
			ieee80211_free_node(ic, ni);
		}
		break;
	default:
		break;
	}
}

int
ieee80211_new_state(struct net_device *dev, enum ieee80211_state nstate, int mgt)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni = &ic->ic_bss;
	int i, error, ostate;
#ifdef IEEE80211_DEBUG
	static const char *stname[] = 
	    { "INIT", "SCAN", "AUTH", "ASSOC", "RUN" };
#endif

	ostate = ic->ic_state;
	DPRINTF(("ieee80211_new_state: %s -> %s\n",
	    stname[ostate], stname[nstate]));
	if (ic->ic_newstate) {
		error = (*ic->ic_newstate)(dev, nstate);
		if (error == EINPROGRESS)
			return 0;
		if (error != 0)
			return error;
	}

	/* state transition */
	ic->ic_state = nstate;
	switch (nstate) {
	case IEEE80211_S_INIT:
		switch (ostate) {
		case IEEE80211_S_INIT:
			break;
		case IEEE80211_S_RUN:
			switch (ic->ic_opmode) {
			case IEEE80211_M_STA:
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_DISASSOC,
				    IEEE80211_REASON_ASSOC_LEAVE);
				break;
			case IEEE80211_M_HOSTAP:
				TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
					if (ni->ni_associd == 0)
						continue;
					IEEE80211_SEND_MGMT(ic, ni,
					    IEEE80211_FC0_SUBTYPE_DISASSOC,
					    IEEE80211_REASON_ASSOC_LEAVE);
				}
				break;
			default:
				break;
			}
			/* FALLTHRU */
		case IEEE80211_S_ASSOC:
			switch (ic->ic_opmode) {
			case IEEE80211_M_STA:
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_DEAUTH,
				    IEEE80211_REASON_AUTH_LEAVE);
				break;
			case IEEE80211_M_HOSTAP:
				TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
					IEEE80211_SEND_MGMT(ic, ni,
					    IEEE80211_FC0_SUBTYPE_DEAUTH,
					    IEEE80211_REASON_AUTH_LEAVE);
				}
				break;
			default:
				break;
			}
			/* FALLTHRU */
		case IEEE80211_S_AUTH:
		case IEEE80211_S_SCAN:
			ic->ic_mgt_timer = 0;
			if (ic->ic_wep_ctx != NULL) {
				kfree(ic->ic_wep_ctx);
				ic->ic_wep_ctx = NULL;
			}
			ieee80211_free_allnodes(ic);
			break;
		}
		break;
	case IEEE80211_S_SCAN:
		ic->ic_flags &= ~IEEE80211_F_SIBSS;
		ni = &ic->ic_bss;
		/* initialize bss for probe request */
		IEEE80211_ADDR_COPY(ni->ni_macaddr, dev->broadcast);
		IEEE80211_ADDR_COPY(ni->ni_bssid, dev->broadcast);
		ni->ni_nrate = 0;
		memset(ni->ni_rates, 0, IEEE80211_RATE_SIZE);
		for (i = 0; i < IEEE80211_RATE_SIZE; i++) {
			if (ic->ic_sup_rates[i] != 0)
				ni->ni_rates[ni->ni_nrate++] =
				    ic->ic_sup_rates[i];
		}
		ni->ni_associd = 0;
		ni->ni_rstamp = 0;
		switch (ostate) {
		case IEEE80211_S_INIT:
			/* use lowest rate */
			ni->ni_txrate = 0;
			ieee80211_begin_scan(dev, ni);
			break;
		case IEEE80211_S_SCAN:
			/* scan next */
			if (ic->ic_flags & IEEE80211_F_ASCAN) {
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_PROBE_REQ, 0);
			}
			break;
		case IEEE80211_S_RUN:
			/* beacon miss */
			if (netif_msg_debug(ic)) {
				/* XXX bssid clobbered above */
				printk("%s: no recent beacons from %s;"
				    " rescanning\n",
				    dev->name,
				    ether_sprintf(ic->ic_bss.ni_bssid));
			}
			ieee80211_free_allnodes(ic);
			/* FALLTHRU */
		case IEEE80211_S_AUTH:
		case IEEE80211_S_ASSOC:
			/* timeout restart scan */
			ni = ieee80211_find_node(ic, ic->ic_bss.ni_macaddr);
			if (ni != NULL)
				ni->ni_fails++;
			ieee80211_begin_scan(dev, &ic->ic_bss);
			break;
		}
		break;
	case IEEE80211_S_AUTH:
		switch (ostate) {
		case IEEE80211_S_INIT:
			DPRINTF(("ieee80211_new_state: invalid transition\n"));
			break;
		case IEEE80211_S_SCAN:
			IEEE80211_SEND_MGMT(ic, ni,
			    IEEE80211_FC0_SUBTYPE_AUTH, 1);
			break;
		case IEEE80211_S_AUTH:
		case IEEE80211_S_ASSOC:
			switch (mgt) {
			case IEEE80211_FC0_SUBTYPE_AUTH:
				/* ??? */
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_AUTH, 2);
				break;
			case IEEE80211_FC0_SUBTYPE_DEAUTH:
				/* ignore and retry scan on timeout */
				break;
			}
			break;
		case IEEE80211_S_RUN:
			switch (mgt) {
			case IEEE80211_FC0_SUBTYPE_AUTH:
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_AUTH, 2);
				ic->ic_state = ostate;	/* stay RUN */
				break;
			case IEEE80211_FC0_SUBTYPE_DEAUTH:
				/* try to reauth */
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_AUTH, 1);
				break;
			}
			break;
		}
		break;
	case IEEE80211_S_ASSOC:
		switch (ostate) {
		case IEEE80211_S_INIT:
		case IEEE80211_S_SCAN:
		case IEEE80211_S_ASSOC:
			DPRINTF(("ieee80211_new_state: invalid transition\n"));
			break;
		case IEEE80211_S_AUTH:
			IEEE80211_SEND_MGMT(ic, ni,
			    IEEE80211_FC0_SUBTYPE_ASSOC_REQ, 0);
			break;
		case IEEE80211_S_RUN:
			IEEE80211_SEND_MGMT(ic, ni,
			    IEEE80211_FC0_SUBTYPE_ASSOC_REQ, 1);
			break;
		}
		break;
	case IEEE80211_S_RUN:
		switch (ostate) {
		case IEEE80211_S_INIT:
		case IEEE80211_S_AUTH:
		case IEEE80211_S_RUN:
			DPRINTF(("ieee80211_new_state: invalid transition\n"));
			break;
		case IEEE80211_S_SCAN:		/* adhoc mode */
		case IEEE80211_S_ASSOC:		/* infra mode */
			if (netif_msg_debug(ic)) {
				printk("%s: ", dev->name);
				if (ic->ic_opmode == IEEE80211_M_STA)
					printk("associated ");
				else
					printk("synchronized ");
				printk("with %s ssid ",
				    ether_sprintf(ic->ic_bss.ni_bssid));
				ieee80211_print_essid(ic->ic_bss.ni_essid,
				    ic->ic_bss.ni_esslen);
				printk(" channel %d\n",
					ieee80211_chan2ieee(ic, ic->ic_bss.ni_chan));
			}
			/* start with highest negotiated rate */
			ic->ic_bss.ni_txrate = ic->ic_bss.ni_nrate - 1;
			ic->ic_mgt_timer = 0;
			break;
		}
		break;
	}
	return 0;
}

struct sk_buff *
ieee80211_wep_crypt(struct net_device *dev, struct sk_buff *skb0, int txflag)
{
	struct ieee80211com *ic = (void *)dev;
	struct sk_buff *skb, *n, *n0;
	struct ieee80211_frame *wh;
	int i, left, len, moff, noff, kid;
	u_int32_t iv, crc;
	u_int8_t *ivp;
	void *ctx;
	u_int8_t keybuf[IEEE80211_WEP_IVLEN + IEEE80211_KEYBUF_SIZE];
	u_int8_t crcbuf[IEEE80211_WEP_CRCLEN];

	n0 = NULL;
	if ((ctx = ic->ic_wep_ctx) == NULL) {
		ctx = kmalloc(arc4_ctxlen(), GFP_KERNEL);
		if (ctx == NULL)
			goto fail;
		ic->ic_wep_ctx = ctx;
	}
	skb = skb0;
	left = skb->len;
	len = IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN + IEEE80211_WEP_CRCLEN;
	if (txflag) {
		n = dev_alloc_skb(skb->len + len);
	} else {
		n = dev_alloc_skb(skb->len - len);
		left -= len;
	}
	if (n == NULL)
		goto fail;
	n0 = n;
	memcpy(n->data, skb->data, sizeof(struct ieee80211_frame));
	wh = (struct ieee80211_frame *) n->data;
	left -= sizeof(struct ieee80211_frame);
	moff = sizeof(struct ieee80211_frame);
	noff = sizeof(struct ieee80211_frame);
	if (txflag) {
		kid = ic->ic_wep_txkey;
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
                iv = ic->ic_iv;
		/*
		 * Skip 'bad' IVs from Fluhrer/Mantin/Shamir:
		 * (B, 255, N) with 3 <= B < 8
		 */
		if (iv >= 0x03ff00 &&
		    (iv & 0xf8ff00) == 0x00ff00)
			iv += 0x000100;
		ic->ic_iv = iv + 1;
		/* put iv in little endian to prepare 802.11i */
		ivp = n->data + noff;
		for (i = 0; i < IEEE80211_WEP_IVLEN; i++) {
			ivp[i] = iv & 0xff;
			iv >>= 8;
		}
		ivp[IEEE80211_WEP_IVLEN] = kid << 6;	/* pad and keyid */
		noff += IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;
	} else {
		wh->i_fc[1] &= ~IEEE80211_FC1_WEP;
		ivp = skb->data + moff;
		kid = ivp[IEEE80211_WEP_IVLEN] >> 6;
		moff += IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;
	}
	memcpy(keybuf, ivp, IEEE80211_WEP_IVLEN);
	memcpy(keybuf + IEEE80211_WEP_IVLEN, ic->ic_nw_keys[kid].wk_key,
	    ic->ic_nw_keys[kid].wk_len);
	arc4_setkey(ctx, keybuf,
	    IEEE80211_WEP_IVLEN + ic->ic_nw_keys[kid].wk_len);

	/* encrypt with calculating CRC */
	crc = ~0;
	arc4_encrypt(ctx, n->data + noff, skb->data + moff, left);
	if (txflag) {
		crc = ieee80211_crc_update(crc, skb->data + moff, left);
		moff += left;
	} else {
		crc = ieee80211_crc_update(crc, n->data + noff, left);
		noff += left;
	}
	crc = ~crc;
	if (txflag) {
		*(u_int32_t *)crcbuf = cpu_to_le32(crc);
		arc4_encrypt(ctx, n->data + noff, crcbuf, sizeof(crcbuf));
	} else {
		arc4_encrypt(ctx, crcbuf, skb->data + moff, sizeof(crcbuf));
		if (crc != le32_to_cpu(*(u_int32_t *)crcbuf)) {
#ifdef IEEE80211_DEBUG
			if (ieee80211_debug) {
				printk("%s: decrypt CRC error\n", dev->name);
				if (ieee80211_debug > 1)
					ieee80211_dump_pkt(n0->data,
					    n0->len, -1, -1);
			}
#endif
			goto fail;
		}
	}
	dev_kfree_skb(skb0);
	return n0;

  fail:
	dev_kfree_skb(skb0);
	dev_kfree_skb(n0);
	return NULL;
}

/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ieee80211_getstats(struct net_device *dev)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	return &ic->ic_stats;
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/ctype.h>

static int
ieee80211_proc_debug_read(char *page, char **start, off_t off,
	int count, int *eof, void *data)
{
	if (off != 0) {
		*eof = 1;
		return 0;
	}
	return sprintf(page, "%d\n", ieee80211_debug);
}

static int
ieee80211_proc_debug_write(struct file *file, const char *buf,
	unsigned long count, void *data)
{
	int v;
	
	if (sscanf(buf, "%d", &v) == 1) {
		ieee80211_debug = v;
		return count;
	} else
		return -EINVAL;
}

static void
ieee80211_proc_init(struct ieee80211com *ic)
{
	struct proc_dir_entry *dp;
	const char *cp;

	for (cp = ic->ic_dev.name; *cp && !isdigit(*cp); cp++)
		;
	snprintf(ic->ic_procname, sizeof(ic->ic_procname), "wlan%s", cp);
	ic->ic_proc = proc_mkdir(ic->ic_procname, proc_net);
	if (ic->ic_proc == NULL) {
		printk(KERN_INFO "/proc/net/%s: failed to create\n",
			ic->ic_procname);
		return;
	}
	dp = create_proc_entry("debug", 0644, ic->ic_proc);
	if (dp) {
		dp->read_proc = ieee80211_proc_debug_read;
		dp->write_proc = ieee80211_proc_debug_write;
		dp->data = ic;
	}
}

static void
ieee80211_proc_remove(struct ieee80211com *ic)
{
	if (ic->ic_proc != NULL) {
		remove_proc_entry("debug", ic->ic_proc);
		remove_proc_entry(ic->ic_procname, proc_net);
		ic->ic_proc = NULL;
	}
}
#endif /* CONFIG_PROC_FS */

/*
 * CRC 32 -- routine from RFC 2083
 */

/* Table of CRCs of all 8-bit messages */
static u_int32_t ieee80211_crc_table[256];

/* Make the table for a fast CRC. */
static void
ieee80211_crc_init(void)
{
	u_int32_t c;
	int n, k;

	for (n = 0; n < 256; n++) {
		c = (u_int32_t)n;
		for (k = 0; k < 8; k++) {
			if (c & 1)
				c = 0xedb88320UL ^ (c >> 1);
			else
				c = c >> 1;
		}
		ieee80211_crc_table[n] = c;
	}
}

/*
 * Update a running CRC with the bytes buf[0..len-1]--the CRC
 * should be initialized to all 1's, and the transmitted value
 * is the 1's complement of the final running CRC
 */
static u_int32_t
ieee80211_crc_update(u_int32_t crc, u_int8_t *buf, int len)
{
	u_int8_t *endbuf;

	for (endbuf = buf + len; buf < endbuf; buf++)
		crc = ieee80211_crc_table[(crc ^ *buf) & 0xff] ^ (crc >> 8);
	return crc;
}

/*
 * convert IEEE80211 rate value to ifmedia subtype.
 * ieee80211 rate is in unit of 0.5Mbps.
 */
int
ieee80211_rate2media(int rate, enum ieee80211_phytype phytype)
{
	int mword;

	mword = 0;
	switch (phytype) {
	case IEEE80211_T_FH:
		switch (rate & IEEE80211_RATE_VAL) {
		case 0:
			mword = IFM_AUTO;
			break;
		case 2:
			mword = IFM_IEEE80211_FH1;
			break;
		case 4:
			mword = IFM_IEEE80211_FH2;
			break;
		default:
			mword = IFM_NONE;
			break;
		}
		break;

	case IEEE80211_T_DS:
		switch (rate & IEEE80211_RATE_VAL) {
		case 0:
			mword = IFM_AUTO;
			break;
		case 2:
			mword = IFM_IEEE80211_DS1;
			break;
		case 4:
			mword = IFM_IEEE80211_DS2;
			break;
		case 11:
			mword = IFM_IEEE80211_DS5;
			break;
		case 22:
			mword = IFM_IEEE80211_DS11;
			break;
		default:
			mword = IFM_NONE;
			break;
		}
		break;

	case IEEE80211_T_OFDM:
		switch (rate & IEEE80211_RATE_VAL) {
		case 0:
			mword = IFM_AUTO;
			break;
		case 12:
			mword = IFM_IEEE80211_ODFM6;
			break;
		case 18:
			mword = IFM_IEEE80211_ODFM9;
			break;
		case 24:
			mword = IFM_IEEE80211_ODFM12;
			break;
		case 36:
			mword = IFM_IEEE80211_ODFM18;
			break;
		case 48:
			mword = IFM_IEEE80211_ODFM24;
			break;
		case 72:
			mword = IFM_IEEE80211_ODFM36;
			break;
		case 108:
			mword = IFM_IEEE80211_ODFM54;
			break;
		case 144:
			mword = IFM_IEEE80211_ODFM72;
			break;
		default:
			mword = IFM_NONE;
			break;
		}
		break;

	default:
		mword = IFM_MANUAL;
		break;
	}
	return mword;
}

int
ieee80211_media2rate(int mword, enum ieee80211_phytype phytype)
{
	int rate;

	rate = 0;
	switch (phytype) {
	case IEEE80211_T_FH:
		switch (IFM_SUBTYPE(mword)) {
		case IFM_IEEE80211_FH1:
			rate = 2;
			break;
		case IFM_IEEE80211_FH2:
			rate = 4;
			break;
		}
		break;

	case IEEE80211_T_DS:
		switch (IFM_SUBTYPE(mword)) {
		case IFM_IEEE80211_DS1:
			rate = 2;
			break;
		case IFM_IEEE80211_DS2:
			rate = 4;
			break;
		case IFM_IEEE80211_DS5:
			rate = 11;
			break;
		case IFM_IEEE80211_DS11:
			rate = 22;
			break;
		}
		break;

	default:
		break;
	}
	return rate;
}

/*
 * Format an Ethernet MAC for printing.
 */
const char*
ether_sprintf(const u_int8_t *mac)
{
	static char etherbuf[18];
	snprintf(etherbuf, sizeof(etherbuf), "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return etherbuf;
}
EXPORT_SYMBOL(ether_sprintf);		/* XXX */

/*
 * Module glue.
 */
#include "version.h"
static	char *version = WLAN_VERSION " (Sam Leffler <sam@errno.com>)";
static	char *dev_info = "wlan";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("802.11 wireless LAN protocol support");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");		/* XXX really BSD only */
#endif

EXPORT_SYMBOL(ieee80211_chan_find);
EXPORT_SYMBOL(ieee80211_decap);
EXPORT_SYMBOL(ieee80211_dump_pkt);
EXPORT_SYMBOL(ieee80211_encap);
EXPORT_SYMBOL(ieee80211_end_scan);
EXPORT_SYMBOL(ieee80211_find_node);
EXPORT_SYMBOL(ieee80211_free_node);
EXPORT_SYMBOL(ieee80211_ghz2ieee);
EXPORT_SYMBOL(ieee80211_chan2ieee);
EXPORT_SYMBOL(ieee80211_ieee2ghz);
EXPORT_SYMBOL(ieee80211_ifattach);
EXPORT_SYMBOL(ieee80211_ifdetach);
EXPORT_SYMBOL(ieee80211_input);
EXPORT_SYMBOL(ieee80211_media2rate);
EXPORT_SYMBOL(ieee80211_new_state);
EXPORT_SYMBOL(ieee80211_next_scan);
EXPORT_SYMBOL(ieee80211_rate2media);
EXPORT_SYMBOL(ieee80211_watchdog);

static int __init
init_wlan(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);
	return 0;
}
module_init(init_wlan);

static void __exit
exit_wlan(void)
{
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_wlan);
