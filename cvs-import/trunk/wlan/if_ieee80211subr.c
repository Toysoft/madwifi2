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

#ifndef __MOD_INC_USE_COUNT
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
#define	DPRINTF(_ic, X)		if (netif_msg_debug(_ic)) printk X
#define	DPRINTF2(_ic, X)	if (netif_msg_debug(_ic)>1) printk X
#else
#define	DPRINTF(_ic, X)
#define	DPRINTF2(_ic, X)
#endif

#define	IEEE80211_RATE2MBS(r)	(((r) & IEEE80211_RATE_VAL) / 2)
#define	IEEE80211_IS_SUBTYPE(_fc, _type) \
	(((_fc) & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_##_type)

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
    struct sk_buff *, int, u_int32_t, u_int);
static void ieee80211_recv_prreq(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t, u_int);
static void ieee80211_recv_auth(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t, u_int);
static void ieee80211_recv_asreq(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t, u_int);
static void ieee80211_recv_asresp(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t, u_int);
static void ieee80211_recv_disassoc(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t, u_int);
static void ieee80211_recv_deauth(struct ieee80211com *,
    struct sk_buff *, int, u_int32_t, u_int);

static void ieee80211_set11gbasicrates(struct ieee80211_rateset *,
		enum ieee80211_phymode);
static void ieee80211_crc_init(void);
static u_int32_t ieee80211_crc_update(u_int32_t, u_int8_t *, int);
static struct net_device_stats *ieee80211_getstats(struct net_device *);
static void ieee80211_free_allnodes(struct ieee80211com *);
static void ieee80211_watchdog(unsigned long);
static void ieee80211_timeout_nodes(struct ieee80211com *);
static void ieee80211_reset_recvhist(struct ieee80211_node*);
static void ieee80211_add_recvhist(struct ieee80211_node*, u_int8_t rssi,
			u_int32_t rstamp, u_int8_t rantenna);
static int ieee80211_setup_rates(struct ieee80211com *, struct ieee80211_node *,
		u_int8_t *rates, u_int8_t *xrates, int flags);

extern	int ieee80211_cfgget(struct net_device *, u_long, caddr_t);
extern	int ieee80211_cfgset(struct net_device *, u_long, caddr_t);
#ifdef CONFIG_PROC_FS
static	void ieee80211_proc_init(struct ieee80211com *);
static	void ieee80211_proc_remove(struct ieee80211com *);
static void ieee80211_proc_add_sta(struct ieee80211com *ic, struct ieee80211_node *ni);
static void ieee80211_proc_del_sta(struct ieee80211com *ic, struct ieee80211_node *ni);
static int ieee80211_proc_read_sta(char *page, char **start, off_t off,
					int count, int *eof, void *data);
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
static const char *ieee80211_phymode_name[] = {
	"auto",		/* IEEE80211_MODE_AUTO */
	"11a",		/* IEEE80211_MODE_11A */
	"11b",		/* IEEE80211_MODE_11B */
	"11g",		/* IEEE80211_MODE_11G */
	"turbo",	/* IEEE80211_MODE_TURBO	*/
};

int
ieee80211_ifattach(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211channel *c;
	int i;

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

	init_timer(&ic->ic_slowtimo);
	ic->ic_slowtimo.data = (unsigned long) ic;
	ic->ic_slowtimo.function = ieee80211_watchdog;
	ieee80211_watchdog((unsigned long) ic);		/* prime timer */

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
	ic->ic_modecaps |= 1<<IEEE80211_MODE_AUTO;
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		c = &ic->ic_channels[i];
		if (c->ic_flags) {
			/*
			 * Verify driver passed us valid data.
			 */
			if (i != ieee80211_chan2ieee(ic, c)) {
				printk(KERN_WARNING "%s: bad channel ignored; "
					"freq %u flags %x number %u\n",
					dev->name, c->ic_freq, c->ic_flags, i);
				c->ic_flags = 0;	/* NB: remove */
				continue;
			}
			setbit(ic->ic_chan_avail, i);
			/*
			 * Identify mode capabilities.
			 */
			if (IEEE80211_IS_CHAN_A(c))
				ic->ic_modecaps |= 1<<IEEE80211_MODE_11A;
			if (IEEE80211_IS_CHAN_B(c))
				ic->ic_modecaps |= 1<<IEEE80211_MODE_11B;
			if (IEEE80211_IS_CHAN_PUREG(c))
				ic->ic_modecaps |= 1<<IEEE80211_MODE_11G;
			if (IEEE80211_IS_CHAN_T(c))
				ic->ic_modecaps |= 1<<IEEE80211_MODE_TURBO;
		}
	}
	/* validate ic->ic_curmode */
	if ((ic->ic_modecaps & (1<<ic->ic_curmode)) == 0)
		ic->ic_curmode = IEEE80211_MODE_AUTO;

	(void) ieee80211_setmode(ic, ic->ic_curmode);

#ifdef notdef
	ic->ic_rtsthreshold = IEEE80211_RTS_DEFAULT;
#else
	ic->ic_rtsthreshold = IEEE80211_RTS_MAX;
#endif
	ic->ic_fragthreshold = 2346;		/* XXX not used yet */
	ic->ic_des_chan = IEEE80211_CHAN_ANYC;	/* any channel is ok */
	ic->ic_fixed_rate = -1;			/* no fixed rate */
	if (ic->ic_lintval == 0)
		ic->ic_lintval = 100;		/* default sleep */
	ic->ic_bmisstimeout = 7*ic->ic_lintval;	/* default 7 beacons */

	rwlock_init(&ic->ic_nodelock);
	TAILQ_INIT(&ic->ic_node);
	for (i = 0; i < IEEE80211_NODE_HASHSIZE; i++)
		LIST_INIT(&ic->ic_hash[i]);

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

	__MOD_DEC_USE_COUNT(THIS_MODULE);
}

/*
 * Convert MHz frequency to IEEE channel number.
 */
u_int
ieee80211_mhz2ieee(u_int freq, u_int flags)
{
	if (flags & IEEE80211_CHAN_2GHZ) {	/* 2GHz band */
		if (freq == 2484)
			return 14;
		if (freq < 2484)
			return (freq - 2407) / 5;
		else
			return 15 + ((freq - 2512) / 20);
	} else if (flags & IEEE80211_CHAN_5GHZ) {	/* 5Ghz band */
		return (freq - 5000) / 5;
	} else {				/* either, guess */
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
	else if (c == IEEE80211_CHAN_ANYC)
		return IEEE80211_CHAN_ANY;
	else {
		printk(KERN_ERR "wlan: invalid channel freq %u flags %x\n",
			c->ic_freq, c->ic_flags);
		return 0;		/* XXX */
	}
}

/*
 * Convert IEEE channel number to MHz frequency.
 */
u_int
ieee80211_ieee2mhz(u_int chan, u_int flags)
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
	} else {				/* either, guess */
		if (chan == 14)
			return 2484;
		if (chan < 14)			/* 0-13 */
			return 2407 + chan*5;
		if (chan < 27)			/* 15-26 */
			return 2512 + ((chan-15)*20);
		return 5000 + (chan*5);
	}
}

/*
 * Setup the media data structures according to the channel and
 * rate tables.  This must be called by the driver after
 * ieee80211_attach and before most anything else.
 */
void
ieee80211_media_init(struct net_device *dev,
	ifm_change_cb_t media_change, ifm_stat_cb_t media_stat)
{
#define	ADD(_ic, _s, _o) \
	ifmedia_add(&(_ic)->ic_media, \
		IFM_MAKEWORD(IFM_IEEE80211, (_s), (_o), 0), 0, NULL)
	struct ieee80211com *ic = (void *)dev;
	struct ifmediareq imr;
	int i, j, mode, rate, maxrate, mword, mopt, r;
	struct ieee80211_rateset *rs;
	struct ieee80211_rateset allrates;

	/*
	 * Fill in media characteristics.
	 */
	ifmedia_init(&ic->ic_media, 0, media_change, media_stat);
	maxrate = 0;
	memset(&allrates, 0, sizeof(allrates));
	for (mode = IEEE80211_MODE_AUTO; mode < IEEE80211_MODE_MAX; mode++) {
		static const u_int mopts[] = { 
			IFM_AUTO,
			IFM_MAKEMODE(IFM_IEEE80211_11A),
			IFM_MAKEMODE(IFM_IEEE80211_11B),
			IFM_MAKEMODE(IFM_IEEE80211_11G),
			IFM_MAKEMODE(IFM_IEEE80211_11A) | IFM_IEEE80211_TURBO,
		};
		if ((ic->ic_modecaps & (1<<mode)) == 0)
			continue;
		mopt = mopts[mode];
		ADD(ic, IFM_AUTO, mopt);	/* e.g. 11a auto */
		if (ic->ic_caps & IEEE80211_C_IBSS)
			ADD(ic, IFM_AUTO, mopt | IFM_IEEE80211_ADHOC);
		if (ic->ic_caps & IEEE80211_C_HOSTAP)
			ADD(ic, IFM_AUTO, mopt | IFM_IEEE80211_HOSTAP);
		if (ic->ic_caps & IEEE80211_C_MONITOR)
			ADD(ic, IFM_AUTO, mopt | IFM_IEEE80211_MONITOR);
		if (ic->ic_caps & IEEE80211_C_AHDEMO)
			ADD(ic, IFM_AUTO, mopt | IFM_IEEE80211_ADHOC | IFM_FLAG0);
		if (mode == IEEE80211_MODE_AUTO)
			continue;
		printk("%s: %s rates: ", dev->name,
			ieee80211_phymode_name[mode]);
		rs = &ic->ic_sup_rates[mode];
		for (i = 0; i < rs->rs_nrates; i++) {
			rate = rs->rs_rates[i];
			mword = ieee80211_rate2media(ic, rate, mode);
			if (mword == 0)
				continue;
			printk("%s%d%sMbps", (i != 0 ? " " : ""),
			    (rate & IEEE80211_RATE_VAL) / 2,
			    ((rate & 0x1) != 0 ? ".5" : ""));
			ADD(ic, mword, mopt);
			if (ic->ic_caps & IEEE80211_C_IBSS)
				ADD(ic, mword, mopt | IFM_IEEE80211_ADHOC);
			if (ic->ic_caps & IEEE80211_C_HOSTAP)
				ADD(ic, mword, mopt | IFM_IEEE80211_HOSTAP);
			if (ic->ic_caps & IEEE80211_C_MONITOR)
				ADD(ic, mword, mopt | IFM_IEEE80211_MONITOR);
			if (ic->ic_caps & IEEE80211_C_AHDEMO)
				ADD(ic, mword, mopt | IFM_IEEE80211_ADHOC | IFM_FLAG0);
			/*
			 * Add rate to the collection of all rates.
			 */
			r = rate & IEEE80211_RATE_VAL;
			for (j = 0; j < allrates.rs_nrates; j++)
				if (allrates.rs_rates[j] == r)
					break;
			if (j == allrates.rs_nrates) {
				/* unique, add to the set */
				allrates.rs_rates[j] = r;
				allrates.rs_nrates++;
			}
			rate = (rate & IEEE80211_RATE_VAL) / 2;
			if (rate > maxrate)
				maxrate = rate;
		}
		printk("\n");
	}
	for (i = 0; i < allrates.rs_nrates; i++) {
		mword = ieee80211_rate2media(ic, allrates.rs_rates[i],
				IEEE80211_MODE_AUTO);
		if (mword == 0)
			continue;
		mword = IFM_SUBTYPE(mword);	/* remove media options */
		ADD(ic, mword, 0);
		if (ic->ic_caps & IEEE80211_C_IBSS)
			ADD(ic, mword, IFM_IEEE80211_ADHOC);
		if (ic->ic_caps & IEEE80211_C_HOSTAP)
			ADD(ic, mword, IFM_IEEE80211_HOSTAP);
		if (ic->ic_caps & IEEE80211_C_MONITOR)
			ADD(ic, mword, IFM_IEEE80211_MONITOR);
		if (ic->ic_caps & IEEE80211_C_AHDEMO)
			ADD(ic, mword, IFM_IEEE80211_ADHOC | IFM_FLAG0);
	}
	ieee80211_media_status(dev, &imr);
	ifmedia_set(&ic->ic_media, imr.ifm_active);

#undef ADD
}

static int
findrate(struct ieee80211com *ic, enum ieee80211_phymode mode, int rate)
{
#define	IEEERATE(_ic,_m,_i) \
	((_ic)->ic_sup_rates[_m].rs_rates[_i] & IEEE80211_RATE_VAL)
	int i, nrates = ic->ic_sup_rates[mode].rs_nrates;
	for (i = 0; i < nrates; i++)
		if (IEEERATE(ic, mode, i) == rate)
			return i;
	return -1;
#undef IEEERATE
}

/*
 * Handle a media change request.
 */
int
ieee80211_media_change(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ifmedia_entry *ime;
	enum ieee80211_opmode newopmode;
	enum ieee80211_phymode newphymode;
	int i, j, newrate, error = 0;

	ime = ic->ic_media.ifm_cur;
	/*
	 * First, identify the phy mode.
	 */
	switch (IFM_MODE(ime->ifm_media)) {
	case IFM_IEEE80211_11A:
		newphymode = IEEE80211_MODE_11A;
		break;
	case IFM_IEEE80211_11B:
		newphymode = IEEE80211_MODE_11B;
		break;
	case IFM_IEEE80211_11G:
		newphymode = IEEE80211_MODE_11G;
		break;
	case IFM_AUTO:
		newphymode = IEEE80211_MODE_AUTO;
		break;
	default:
		return EINVAL;
	}
	/*
	 * Turbo mode is an ``option''.  Eventually it
	 * needs to be applied to 11g too.
	 */
	if (ime->ifm_media & IFM_IEEE80211_TURBO) {
		if (newphymode != IEEE80211_MODE_11A)
			return EINVAL;
		newphymode = IEEE80211_MODE_TURBO;
	}
	/*
	 * Validate requested mode is available.
	 */
	if ((ic->ic_modecaps & (1<<newphymode)) == 0)
		return EINVAL;

	/*
	 * Next, the fixed/variable rate.
	 */
	i = -1;
	if (IFM_SUBTYPE(ime->ifm_media) != IFM_AUTO) {
		/*
		 * Convert media subtype to rate.
		 */
		newrate = ieee80211_media2rate(ime->ifm_media);
		if (newrate == 0)
			return EINVAL;
		/*
		 * Check the rate table for the specified/current phy.
		 */
		if (newphymode == IEEE80211_MODE_AUTO) {
			/*
			 * In autoselect mode search for the rate.
			 */
			for (j = IEEE80211_MODE_11A;
			     j < IEEE80211_MODE_MAX; j++) {
				if ((ic->ic_modecaps & (1<<j)) == 0)
					continue;
				i = findrate(ic, j, newrate);
				if (i != -1) {
					/* lock mode too */
					newphymode = j;
					break;
				}
			}
		} else {
			i = findrate(ic, newphymode, newrate);
		}
		if (i == -1)			/* mode/rate mismatch */
			return EINVAL;
	}
	/* NB: defer rate setting to later */

	/*
	 * Deduce new operating mode but don't install it just yet.
	 */
	if ((ime->ifm_media & (IFM_IEEE80211_ADHOC|IFM_FLAG0)) ==
	    (IFM_IEEE80211_ADHOC|IFM_FLAG0))
		newopmode = IEEE80211_M_AHDEMO;
	else if (ime->ifm_media & IFM_IEEE80211_HOSTAP)
		newopmode = IEEE80211_M_HOSTAP;
	else if (ime->ifm_media & IFM_IEEE80211_ADHOC)
		newopmode = IEEE80211_M_IBSS;
	else if (ime->ifm_media & IFM_IEEE80211_MONITOR)
		newopmode = IEEE80211_M_MONITOR;
	else
		newopmode = IEEE80211_M_STA;

	/*
	 * Autoselect doesn't make sense when operating as an AP.
	 * If no phy mode has been selected, pick one and lock it
	 * down so rate tables can be used in forming beacon frames
	 * and the like.
	 */
	if (newopmode == IEEE80211_M_HOSTAP &&
	    newphymode == IEEE80211_MODE_AUTO) {
		for (j = IEEE80211_MODE_11A; j < IEEE80211_MODE_MAX; j++)
			if (ic->ic_modecaps & (1<<j)) {
				newphymode = j;
				break;
			}
	}

	/*
	 * Handle phy mode change.
	 */
	if (ic->ic_curmode != newphymode) {		/* change phy mode */
		error = ieee80211_setmode(ic, newphymode);
		if (error != 0)
			return error;
		error = ENETRESET;
	}

	/*
	 * Committed to changes, install the rate setting.
	 */
	if (ic->ic_fixed_rate != i) {
		ic->ic_fixed_rate = i;			/* set fixed tx rate */
		error = ENETRESET;
	}

	/*
	 * Handle operating mode change.
	 */
	if (ic->ic_opmode != newopmode) {
		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
			/* forget channel assignment in transition from monitor mode
			 * since it is essentially random and will prevent us from associating
			 */
			ic->ic_des_chan = (struct ieee80211channel *) IEEE80211_CHAN_ANY;
		}
		ic->ic_opmode = newopmode;
		switch (newopmode) {
		case IEEE80211_M_AHDEMO:
		case IEEE80211_M_HOSTAP:
		case IEEE80211_M_STA:
		case IEEE80211_M_MONITOR:
			ic->ic_flags &= ~IEEE80211_F_IBSSON;
			break;

			break;
		case IEEE80211_M_IBSS:
			ic->ic_flags |= IEEE80211_F_IBSSON;
#ifdef notdef
			if (ic->ic_curmode == IEEE80211_MODE_11G)
				ieee80211_set11gbasicrates(
					&ic->ic_suprates[newphymode],
					IEEE80211_MODE_11B);
#endif
			break;
		}
		error = ENETRESET;
	}
	return error;
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
		/* calculate rate subtype */
		imr->ifm_active |= ieee80211_rate2media(ic,
			ni->ni_rates.rs_rates[ni->ni_txrate], ic->ic_curmode);
		break;
	case IEEE80211_M_IBSS:
		imr->ifm_active |= IFM_IEEE80211_ADHOC;
		break;
	case IEEE80211_M_AHDEMO:
		/* should not come here */
		break;
	case IEEE80211_M_HOSTAP:
		imr->ifm_active |= IFM_IEEE80211_HOSTAP;
		break;
	case IEEE80211_M_MONITOR:
		imr->ifm_active |= IFM_IEEE80211_MONITOR;
		break;
	}
	switch (ic->ic_curmode) {
	case IEEE80211_MODE_11A:
		imr->ifm_active |= IFM_MAKEMODE(IFM_IEEE80211_11A);
		break;
	case IEEE80211_MODE_11B:
		imr->ifm_active |= IFM_MAKEMODE(IFM_IEEE80211_11B);
		break;
	case IEEE80211_MODE_11G:
		imr->ifm_active |= IFM_MAKEMODE(IFM_IEEE80211_11G);
		break;
	case IEEE80211_MODE_TURBO:
		imr->ifm_active |= IFM_MAKEMODE(IFM_IEEE80211_11A)
				|  IFM_IEEE80211_TURBO;
		break;
	}
}

/*
 * This function reassemble fragments using the skb of the 1st fragment,
 * if large enough.  If not, a new skb is allocated to hold incoming fragments.
 *
 * Fragments are copied at the end of the previous fragment.  A different strategy 
 * could have been used, where a non-linear skb is allocated and fragments attached to
 * that skb.
 */
static struct sk_buff *
ieee80211_defrag(struct ieee80211com *ic, struct ieee80211_node *ni, struct sk_buff *skb)
{
	struct ieee80211_frame *wh = (struct ieee80211_frame *) skb->data;
	u_int16_t rxseq, last_rxseq;
	u_int8_t fragno, last_fragno;
	u_int8_t more_frag = wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG;

	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		/* Do not keep fragments of multicast frames */
		return skb;
	}
		
	rxseq =  le16_to_cpu(*(u_int16_t *)wh->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;
	fragno = le16_to_cpu(*(u_int16_t *)wh->i_seq) & IEEE80211_SEQ_FRAG_MASK;

	/* Quick way out, if there's nothing to defragment */
	if (!more_frag 
	    && fragno == 0
	    && ni->ni_rxfragskb == NULL) return skb;

	/* Use this lock to make sure ni->ni_rxfragskb is not freed by the timer process
	   while we use it */
	write_lock(&ic->ic_nodelock);

	/* Update the time stamp.  As a side effect, it also makes sure that the timer will
	   not change ni->ni_rxfragskb for at least 1 second, or in other words, for the remaining 
	   of this function */
	ni->ni_rxfragstamp = jiffies;

	write_unlock(&ic->ic_nodelock);

	/* Validate that fragment is in order and related to the previous ones */
	if (ni->ni_rxfragskb) {
		struct ieee80211_frame *lwh;

		lwh = (struct ieee80211_frame *) ni->ni_rxfragskb->data;
		last_rxseq =  le16_to_cpu(*(u_int16_t *)lwh->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;
		last_fragno = le16_to_cpu(*(u_int16_t *)lwh->i_seq) & IEEE80211_SEQ_FRAG_MASK;
		if (rxseq != last_rxseq
		    || fragno != last_fragno + 1
		    || (!IEEE80211_ADDR_EQ(wh->i_addr1, lwh->i_addr1))
		    || (!IEEE80211_ADDR_EQ(wh->i_addr2, lwh->i_addr2))
		    || (ni->ni_rxfragskb->end - ni->ni_rxfragskb->tail < skb->len)) {
			/* Unrelated fragment or no space for it, clear current fragments */
			dev_kfree_skb(ni->ni_rxfragskb);
			ni->ni_rxfragskb = NULL;
		}
	}

	/* If this is the first fragment */
 	if (ni->ni_rxfragskb == NULL && fragno == 0) {
		ni->ni_rxfragskb = skb;
		/* If more frags are coming */
		if (more_frag) {
			if (skb_is_nonlinear(skb)) {
				/* We need a continous buffer to assemble fragments */
				ni->ni_rxfragskb = skb_copy(skb, GFP_ATOMIC);
				dev_kfree_skb(skb);
			}
			/* Check that we have enough space to hold incoming fragments */
			else if (skb->end - skb->head < ic->ic_dev.mtu + sizeof(sizeof(struct ieee80211_frame))) {
				ni->ni_rxfragskb = skb_copy_expand(skb, 0, 
								   (ic->ic_dev.mtu + sizeof(sizeof(struct ieee80211_frame))) 
								   - (skb->end - skb->head),
								   GFP_ATOMIC);
				dev_kfree_skb(skb);
			}
		}
	}
	else {
		if (ni->ni_rxfragskb) {
			struct ieee80211_frame *lwh = (struct ieee80211_frame *) ni->ni_rxfragskb->data;

			/* We know we have enough space to copy, we've verified that before */
			/* Copy current fragment at end of previous one */
			memcpy(ni->ni_rxfragskb->tail,
			       skb->data + sizeof(struct ieee80211_frame),
			       skb->len - sizeof(struct ieee80211_frame)
				);
			/* Update tail and length */
			skb_put(ni->ni_rxfragskb, skb->len - sizeof(struct ieee80211_frame));
			/* Keep a copy of last sequence and fragno */
			*(u_int16_t *) lwh->i_seq = *(u_int16_t *) wh->i_seq;
			
		}
		/* we're done with the fragment */
		dev_kfree_skb(skb);
	}
		
	if (more_frag) {
		/* More to come */
		skb = NULL;
	}
	else {
		/* Last fragment received, we're done! */
		skb = ni->ni_rxfragskb;
		ni->ni_rxfragskb = NULL;
	}

	return skb;
}

void
ieee80211_input(struct net_device *dev, struct sk_buff *skb,
	int rssi, u_int32_t rstamp, u_int rantenna)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni = NULL;
	struct ieee80211_frame *wh;
	struct ether_header *eh;
	void (*rh)(struct ieee80211com *, struct sk_buff *, int, u_int32_t, u_int);
	struct sk_buff *skb1;
	int len;
	u_int8_t dir, subtype;
	u_int8_t *bssid;
	u_int16_t lastrxseq;
	u_int8_t lastfragno;

	wh = (struct ieee80211_frame *) skb->data;
	if ((wh->i_fc[0] & IEEE80211_FC0_VERSION_MASK) !=
	    IEEE80211_FC0_VERSION_0) {
		DPRINTF(ic, ("%s: discard packet with wrong version: %x\n",
			    dev->name, wh->i_fc[0]));
		goto err;
	}

	dir = wh->i_fc[1] & IEEE80211_FC1_DIR_MASK;

	if (ic->ic_state != IEEE80211_S_SCAN) {
		switch (ic->ic_opmode) {
		case IEEE80211_M_STA:
			ni = ieee80211_ref_node(&ic->ic_bss);
			if (!IEEE80211_ADDR_EQ(wh->i_addr2, ni->ni_bssid)) {
				DPRINTF(ic, ("%s: discard frame from bss %s\n",
					    dev->name,
					    ether_sprintf(wh->i_addr2)));
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
				DPRINTF2(ic, ("%s: other bss %s\n", __func__,
				    ether_sprintf(wh->i_addr3)));
				goto out;
			}
			ni = ieee80211_find_node(ic, wh->i_addr2);
			if (ni == NULL) {
				DPRINTF(ic, ("%s: warning, unknown src %s\n",
				    __func__, ether_sprintf(wh->i_addr2)));
				/*
				 * NB: Node allocation is handled in the
				 * management handling routines.  Just fake
				 * up a reference to the hosts's node to do
				 * the stuff below.
				 */
				ni = ieee80211_ref_node(&ic->ic_bss);
			}
			break;
		case IEEE80211_M_MONITOR:
			goto out;
		default:
			/* XXX catch bad values */
			break;
		}
		ieee80211_add_recvhist(ni, rssi, rstamp, rantenna);
		lastrxseq = ni->ni_rxseq;
		lastfragno = ni->ni_fragno;
		ni->ni_inact = 0;
		ni->ni_rxseq =
		    le16_to_cpu(*(u_int16_t *)wh->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;
		ni->ni_fragno = le16_to_cpu(*(u_int16_t *)wh->i_seq) & IEEE80211_SEQ_FRAG_MASK;
		if ((wh->i_fc[1] & IEEE80211_FC1_RETRY) &&
		    lastrxseq == ni->ni_rxseq &&
		    lastfragno == ni->ni_fragno) {
			/* duplicate, silently discarded */
			goto out;
		}
		if (ni == &ic->ic_bss) {
			ieee80211_unref_node(&ni);
			ni = NULL;
		}
	}

	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_DATA:
		switch (ic->ic_opmode) {
		case IEEE80211_M_STA:
			if (dir != IEEE80211_FC1_DIR_FROMDS) {
				DPRINTF(ic, ("%s:discard frame with invalid "
					"direction %x\n", dev->name, dir));
				goto out;
			}
			if ((dev->flags & IFF_MULTICAST) &&
			    IEEE80211_IS_MULTICAST(wh->i_addr1) &&
			    IEEE80211_ADDR_EQ(wh->i_addr3, dev->dev_addr)) {
				/*
				 * In IEEE802.11 network, multicast packet
				 * sent from me is broadcasted from AP.
				 * It should be silently discarded for
				 * SIMPLEX interface.
				 *
				 * NB: Linux has no IFF_ flag to indicate
				 *     if an interface is SIMPLEX or not;
				 *     so we always assume it to be true.
				 */
				DPRINTF(ic, ("%s: discard multicast echo\n",
						dev->name));
				goto out;
			}
			break;
		case IEEE80211_M_MONITOR:
			goto out;
		case IEEE80211_M_IBSS:
		case IEEE80211_M_AHDEMO:
			if (dir != IEEE80211_FC1_DIR_NODS)
				goto out;
			break;
		case IEEE80211_M_HOSTAP:
			if (dir != IEEE80211_FC1_DIR_TODS)
				goto out;
			/* ni has been set previously */
			if (ni == NULL) {
				DPRINTF(ic, ("%s: data from unknown src %s\n",
				    __func__, ether_sprintf(wh->i_addr2)));
				ni = ieee80211_dup_bss(ic, wh->i_addr2);
				if (ni != NULL) {
					IEEE80211_SEND_MGMT(ic, ni,
					    IEEE80211_FC0_SUBTYPE_DEAUTH,
					    IEEE80211_REASON_NOT_AUTHED);
					ieee80211_free_node(ic, ni);
					ni = NULL;
				}
				goto err;
			}
			if (ni->ni_associd == 0) {
				DPRINTF(ic, ("%s: data from unassoc src %s\n",
				    __func__, ether_sprintf(wh->i_addr2)));
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

		if (ni == NULL)
			ni = ieee80211_ref_node(&ic->ic_bss);

		if ((skb = ieee80211_defrag(ic, ni, skb)) == NULL) {
			/* Fragment dropped or frame not complete yet */
			goto out;
		}

		/* copy to listener after decrypt */
		skb = ieee80211_decap(dev, skb);
		if (skb == NULL) {
			DPRINTF(ic, ("%s: decapsulation failed\n", dev->name));
			goto err;
		}

		ieee80211_unref_node(&ni);

		/* perform as a bridge within the AP */
		skb1 = NULL;
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			eh = (struct ether_header *) skb->data;
			if (ETHER_IS_MULTICAST(eh->ether_dhost)) {
				skb1 = skb_copy(skb, GFP_ATOMIC);
			} else {
				ni = ieee80211_find_node(ic, eh->ether_dhost);
				if (ni != NULL) {
					if (ni->ni_associd != 0) {
						skb1 = skb;
						skb = NULL;
					}
					ieee80211_unref_node(&ni);
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

		/* drop uninteresting frames */
		if (ic->ic_state == IEEE80211_S_SCAN) {
			if (subtype != IEEE80211_FC0_SUBTYPE_BEACON &&
			    subtype != IEEE80211_FC0_SUBTYPE_PROBE_RESP)
				goto out;
		} else {
#if 0
			if (ic->ic_opmode != IEEE80211_M_IBSS &&
			    subtype == IEEE80211_FC0_SUBTYPE_BEACON)
				goto out;
#endif
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

		if (ni == NULL)
			ni = ieee80211_ref_node(&ic->ic_bss);

		if ((skb = ieee80211_defrag(ic, ni, skb)) == NULL) {
			/* Fragment dropped or frame not complete yet */
			goto out;
		}

		rh = ic->ic_recv_mgmt[subtype >> IEEE80211_FC0_SUBTYPE_SHIFT];
		if (rh != NULL)
			(*rh)(ic, skb, rssi, rstamp, rantenna);
		goto out;

	case IEEE80211_FC0_TYPE_CTL:
	default:
		DPRINTF(ic, ("%s: bad type %x\n", __func__, wh->i_fc[0]));
		/* should not come here */
		break;
	}
err:
	ic->ic_stats.rx_errors++;		/* XXX */
out:
	if (ni)
		ieee80211_unref_node(&ni);
	if (skb != NULL)
		dev_kfree_skb(skb);
}

int
ieee80211_mgmt_output(struct net_device *dev, struct ieee80211_node *ni,
    struct sk_buff *skb, int type)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_frame *wh;

	/* XXX this probably shouldn't be permitted */
	KASSERT(ni != NULL, ("%s: null node", __func__));
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
		    !IEEE80211_IS_SUBTYPE(type, PROBE_RESP))
			printk("%s: sending %s to %s on channel %u\n",
			    dev->name,
			    ieee80211_mgt_subtype_name[
			    (type & IEEE80211_FC0_SUBTYPE_MASK)
			    >> IEEE80211_FC0_SUBTYPE_SHIFT],
			    ether_sprintf(ni->ni_macaddr),
			    ieee80211_chan2ieee(ic, ni->ni_chan));
	}
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

	ni = ieee80211_find_node(ic, eh.ether_dhost);
	if (ni == NULL)			/* ic_opmode?? XXX*/
		ni = ieee80211_ref_node(&ic->ic_bss);
	ni->ni_inact = 0;

	llc = (struct llc *) skb_push(skb, sizeof(struct llc));
	llc->llc_dsap = llc->llc_ssap = LLC_SNAP_LSAP;
	llc->llc_control = LLC_UI;
	llc->llc_snap.org_code[0] = 0;
	llc->llc_snap.org_code[1] = 0;
	llc->llc_snap.org_code[2] = 0;
	llc->llc_snap.ether_type = eh.ether_type;

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
	case IEEE80211_M_MONITOR:
		printk("%s: invalid mode\n", __func__);
		break;
	}
	ieee80211_unref_node(&ni);
	return skb;
}

struct sk_buff *
ieee80211_decap(struct net_device *dev, struct sk_buff *skb)
{
	struct ether_header *eh;
	struct ieee80211_frame wh;
	struct llc *llc;
	u_short ether_type = 0;

	memcpy(&wh, skb->data, sizeof(struct ieee80211_frame));
	llc = (struct llc *) skb_pull(skb, sizeof(struct ieee80211_frame));
	if (llc->llc_dsap == LLC_SNAP_LSAP && llc->llc_ssap == LLC_SNAP_LSAP &&
	    llc->llc_control == LLC_UI && llc->llc_snap.org_code[0] == 0 &&
	    llc->llc_snap.org_code[1] == 0 && llc->llc_snap.org_code[2] == 0) {
		ether_type = llc->llc_un.type_snap.ether_type;
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
		DPRINTF((struct ieee80211com*) dev,
			("%s: DS to DS\n", __func__));
		dev_kfree_skb(skb);
		return NULL;
	}
	if (!ALIGNED_POINTER(skb->data + sizeof(*eh), u_int32_t)) {
		struct sk_buff *n;

		/* XXX does this always work? */
		n = skb_copy(skb, GFP_ATOMIC);
		dev_kfree_skb(skb);
		if (n == NULL)
			return NULL;
		skb = n;
		eh = (struct ether_header *) skb->data;
	}
	if (llc != NULL)
		eh->ether_type = htons(skb->len - sizeof(*eh));
	else
		eh->ether_type = ether_type;
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

/*
 * Once a second "slow timeout" processing.
 */
void
ieee80211_watchdog(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ieee80211com *ic = (void *)dev;

	if (ic->ic_mgt_timer && --ic->ic_mgt_timer == 0)
		ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
	if (ic->ic_inact_timer && --ic->ic_inact_timer == 0)
		ieee80211_timeout_nodes(ic);

	ic->ic_slowtimo.expires = jiffies + HZ;		/* once a second */
	add_timer(&ic->ic_slowtimo);
}

/*
 * Mark the basic rates for the 11g rate table based on the
 * operating mode.  For real 11g we mark all the 11b rates
 * and 6, 12, and 24 OFDM.  For 11b compatibility we mark only
 * 11b rates.  There's also a pseudo 11a-mode used to mark only
 * the basic OFDM rates.
 */
static void
ieee80211_set11gbasicrates(struct ieee80211_rateset *rs, enum ieee80211_phymode mode)
{
	static const struct ieee80211_rateset basic[] = {
	    { 3, { 12, 24, 48 } },		/* IEEE80211_MODE_11A */
	    { 4, { 2, 4, 11, 22 } },		/* IEEE80211_MODE_11B */
	    { 7, { 2, 4, 11, 22, 12, 24, 48 } },/* IEEE80211_MODE_11G */
	    { 0 },				/* IEEE80211_MODE_TURBO	*/
	};
	int i, j;

	for (i = 0; i < rs->rs_nrates; i++) {
		rs->rs_rates[i] &= IEEE80211_RATE_VAL;
		for (j = 0; j < basic[mode].rs_nrates; j++)
			if (basic[mode].rs_rates[j] == rs->rs_rates[i]) {
				rs->rs_rates[i] |= IEEE80211_RATE_BASIC;
				break;
			}
	}
}

/*
 * Set the current phy mode and recalculate the active channel
 * set based on the available channels for this mode.  Also
 * select a new default/current channel if the current one is
 * inappropriate for this mode.
 */
int
ieee80211_setmode(struct ieee80211com *ic, enum ieee80211_phymode mode)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	static const u_int chanflags[] = {
		0,			/* IEEE80211_MODE_AUTO */
		IEEE80211_CHAN_A,	/* IEEE80211_MODE_11A */
		IEEE80211_CHAN_B,	/* IEEE80211_MODE_11B */
		IEEE80211_CHAN_PUREG,	/* IEEE80211_MODE_11G */
		IEEE80211_CHAN_T,	/* IEEE80211_MODE_TURBO	*/
	};
	struct ieee80211channel *c;
	u_int modeflags;
	int i;

	/* validate new mode */
	if ((ic->ic_modecaps & (1<<mode)) == 0) {
		DPRINTF(ic, ("%s: mode %u not supported (caps 0x%x)\n",
			__func__, mode, ic->ic_modecaps));
		return EINVAL;
	}

	/*
	 * Verify at least one channel is present in the available
	 * channel list before committing to the new mode.
	 */
	KASSERT(mode < N(chanflags), ("Unexpected mode %u\n", mode));
	modeflags = chanflags[mode];
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		c = &ic->ic_channels[i];
		if (mode == IEEE80211_MODE_AUTO) {
			/* ignore turbo channels for autoselect */
			if ((c->ic_flags &~ IEEE80211_CHAN_TURBO) != 0)
				break;
		} else {
			if ((c->ic_flags & modeflags) == modeflags)
				break;
		}
	}
	if (i > IEEE80211_CHAN_MAX) {
		DPRINTF(ic, ("%s: no channels found for mode %u\n",
			__func__, mode));
		return EINVAL;
	}

	/*
	 * Calculate the active channel set.
	 */
	memset(ic->ic_chan_active, 0, sizeof(ic->ic_chan_active));
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++) {
		c = &ic->ic_channels[i];
		if (mode == IEEE80211_MODE_AUTO) {
			/* take anything but pure turbo channels */
			if ((c->ic_flags &~ IEEE80211_CHAN_TURBO) != 0)
				setbit(ic->ic_chan_active, i);
		} else {
			if ((c->ic_flags & modeflags) == modeflags)
				setbit(ic->ic_chan_active, i);
		}
	}
	/*
	 * If no current/default channel is setup or the current
	 * channel is wrong for the mode then pick the first
	 * available channel from the active list.  This is likely
	 * not the right one.
	 */
	if (ic->ic_ibss_chan == NULL ||
	    isclr(ic->ic_chan_active, ieee80211_chan2ieee(ic, ic->ic_ibss_chan))) {
		for (i = 0; i <= IEEE80211_CHAN_MAX; i++)
			if (isset(ic->ic_chan_active, i)) {
				ic->ic_ibss_chan = &ic->ic_channels[i];
				break;
			}
	}

	/*
	 * Set/reset state flags that influence beacon contents, etc.
	 *
	 * XXX what if we have stations already associated???
	 * XXX probably not right for autoselect?
	 */
	if (ic->ic_caps & IEEE80211_C_SHPREAMBLE)
		ic->ic_flags |= IEEE80211_F_SHPREAMBLE;
	if (mode == IEEE80211_MODE_11G) {
		if (ic->ic_caps & IEEE80211_C_SHSLOT)
			ic->ic_flags |= IEEE80211_F_SHSLOT;
		ieee80211_set11gbasicrates(&ic->ic_sup_rates[mode],
			IEEE80211_MODE_11G);
	} else {
		ic->ic_flags &= ~(IEEE80211_F_SHSLOT);
	}

	ic->ic_curmode = mode;
	return 0;
#undef N
}

/*
 * AP scanning support.
 */

/*
 * Initialize the active channel set based on the set
 * of available channels and the current PHY mode.
 */
void
ieee80211_reset_scan(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;

	memcpy(ic->ic_chan_scan, ic->ic_chan_active,
		sizeof(ic->ic_chan_active));
}

/*
 * Begin an active scan.
 */
void
ieee80211_begin_scan(struct net_device *dev, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = (void *)dev;

	DPRINTF(ic, ("%s: begin %s scan\n", dev->name,
			ic->ic_opmode != IEEE80211_M_HOSTAP ?
				"active" : "passive"));

	ieee80211_reset_scan(dev);
	/*
	 * Flush any previously seen AP's.  Note that this
	 * assumes we don't act as both an AP and a station,
	 * otherwise we'll potentially flush state of stations
	 * associated with us.
	 */
	ieee80211_free_allnodes(ic);

	clrbit(ic->ic_chan_scan, ieee80211_chan2ieee(ic, ni->ni_chan));
	if (ic->ic_opmode != IEEE80211_M_HOSTAP) {
		ic->ic_flags |= IEEE80211_F_ASCAN;
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_PROBE_REQ, 0);
	}
}

/*
 * Switch to the next channel marked for scanning.
 */
void
ieee80211_next_scan(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211channel *chan;

	chan = ic->ic_bss.ni_chan;
	for (;;) {
		if (++chan > &ic->ic_channels[IEEE80211_CHAN_MAX])
			chan = &ic->ic_channels[0];
		if (isset(ic->ic_chan_scan, ieee80211_chan2ieee(ic, chan))) {
			/*
			 * Honor channels marked passive-only
			 * during an active scan.
			 */
			if ((ic->ic_flags & IEEE80211_F_ASCAN) == 0 ||
			    (chan->ic_flags & IEEE80211_CHAN_PASSIVE) == 0)
				break;
		}
		if (chan == ic->ic_bss.ni_chan) {
			ieee80211_end_scan(dev);
			return;
		}
	}
	clrbit(ic->ic_chan_scan, ieee80211_chan2ieee(ic, chan));
	DPRINTF(ic, ("%s: chan %d->%d\n", __func__,
	    ieee80211_chan2ieee(ic, ic->ic_bss.ni_chan),
	    ieee80211_chan2ieee(ic, chan)));
	ic->ic_bss.ni_chan = chan;
	ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
}

static void
ieee80211_create_ibss(struct net_device* dev, struct ieee80211channel *chan)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni;

	ni = &ic->ic_bss;
	DPRINTF(ic, ("%s: creating ibss\n", dev->name));
	ic->ic_flags |= IEEE80211_F_SIBSS;
	ni->ni_chan = chan;
	ni->ni_rates = ic->ic_sup_rates[ieee80211_chan2mode(ic, ni->ni_chan)];
	IEEE80211_ADDR_COPY(ni->ni_macaddr, dev->dev_addr);
	IEEE80211_ADDR_COPY(ni->ni_bssid, dev->dev_addr);
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		ni->ni_bssid[0] |= 0x02;	/* local bit for IBSS */
	ni->ni_esslen = ic->ic_des_esslen;
	memcpy(ni->ni_essid, ic->ic_des_essid, ni->ni_esslen);
	ieee80211_reset_recvhist(ni);
	memset(ni->ni_tstamp, 0, sizeof(ni->ni_tstamp));
	ni->ni_intval = ic->ic_lintval;
	ni->ni_capinfo = IEEE80211_CAPINFO_IBSS;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		ni->ni_capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if (ic->ic_phytype == IEEE80211_T_FH) {
		ni->ni_fhdwell = 200;	/* XXX */
		ni->ni_fhindex = 1;
	}
	ieee80211_new_state(dev, IEEE80211_S_RUN, -1);
}

/*
 * Complete a scan of potential channels.
 */
void
ieee80211_end_scan(struct net_device *dev)
{
	struct ieee80211com *ic = (void *)dev;
	struct ieee80211_node *ni, *nextbs, *selbs;
	void *p;
	u_int8_t rate;
	int i, fail;

	ic->ic_flags &= ~IEEE80211_F_ASCAN;
	ni = TAILQ_FIRST(&ic->ic_node);

	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/* XXX off stack? */
		u_char occupied[roundup(IEEE80211_CHAN_MAX/NBBY, NBBY)];
		/*
		 * The passive scan to look for existing AP's completed,
		 * select a channel to camp on.  Identify the channels
		 * that already have one or more AP's and try to locate
		 * an unnoccupied one.  If that fails, pick a random
		 * channel from the active set.
		 */
		for (; ni != NULL; ni = nextbs) {
			ieee80211_ref_node(ni);
			nextbs = TAILQ_NEXT(ni, ni_list);
			setbit(occupied, ieee80211_chan2ieee(ic, ni->ni_chan));
			ieee80211_free_node(ic, ni);
		}
		for (i = 0; i < IEEE80211_CHAN_MAX; i++)
			if (isset(ic->ic_chan_active, i) && isclr(occupied, i))
				break;
		if (i == IEEE80211_CHAN_MAX) {
			get_random_bytes(&fail, sizeof(fail));
			fail &= 3;		/* random 0-3 */
			for (i = 0; i < IEEE80211_CHAN_MAX; i++)
				if (isset(ic->ic_chan_active, i) && fail-- == 0)
					break;
		}
		ieee80211_create_ibss(dev, &ic->ic_channels[i]);
		return;
	}
	if (ni == NULL) {
		DPRINTF(ic, ("%s: no scan candidate\n", __func__));
  notfound:
		if (ic->ic_opmode == IEEE80211_M_IBSS &&
		    (ic->ic_flags & IEEE80211_F_IBSSON) &&
		    ic->ic_des_esslen != 0) {
			ieee80211_create_ibss(dev, ic->ic_ibss_chan);
			return;
		}
		/*
		 * Reset the list of channels to scan and start again.
		 */
		ieee80211_reset_scan(dev);
		ieee80211_next_scan(dev);
		return;
	}
	selbs = NULL;
	DPRINTF(ic, ("%s: macaddr          bssid         chan  rssi rate ant flag  wep  essid\n", dev->name));
	for (; ni != NULL; ni = nextbs) {
		ieee80211_ref_node(ni);
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
		if (ni->ni_chan == NULL || ni->ni_chan->ic_flags == 0)
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
		     memcmp(ni->ni_essid, ic->ic_des_essid, ic->ic_des_esslen) != 0))
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
			printk(" %+4d",
			    ni->ni_recv_hist[ni->ni_hist_cur].hi_rssi);
			printk(" %2dM%c", IEEE80211_RATE2MBS(rate),
			    fail & 0x08 ? '!' : ' ');
			printk(" %3d",
			    ni->ni_recv_hist[ni->ni_hist_cur].hi_rantenna);
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
			if (selbs == NULL)
				selbs = ni;
			else if (ieee80211_get_rssi(ni) > ieee80211_get_rssi(selbs)) {
				ieee80211_unref_node(&selbs);
				selbs = ni;
			} else
				ieee80211_unref_node(&ni);
		} else {
			ieee80211_unref_node(&ni);
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
		if (ic->ic_bss.ni_rates.rs_nrates == 0) {
			selbs->ni_fails++;
			ieee80211_unref_node(&selbs);
			goto notfound;
		}
		ieee80211_unref_node(&selbs);
		ieee80211_new_state(dev, IEEE80211_S_RUN, -1);
	} else {
		ieee80211_unref_node(&selbs);
		ieee80211_new_state(dev, IEEE80211_S_AUTH, -1);
	}
}

void
ieee80211_setup_node(struct ieee80211com *ic,
	struct ieee80211_node *ni, u_int8_t *macaddr)
{
	int hash;

	IEEE80211_ADDR_COPY(ni->ni_macaddr, macaddr);
	if (ic->ic_node_privlen) {
		ni->ni_private = &ni[1];
		memset(ni->ni_private, 0, ic->ic_node_privlen);
	} else
		ni->ni_private = NULL;

	hash = IEEE80211_NODE_HASH(macaddr);
	write_lock_bh(&ic->ic_nodelock);
	atomic_set(&ni->ni_refcnt, 1);		/* mark referenced */
	TAILQ_INSERT_TAIL(&ic->ic_node, ni, ni_list);
	LIST_INSERT_HEAD(&ic->ic_hash[hash], ni, ni_hash);
	/* 
	 * Note we don't enable the inactive timer when acting
	 * as a station.  Nodes created in this mode represent
	 * AP's identified while scanning.  If we time them out
	 * then several things happen: we can't return the data
	 * to users to show the list of AP's we encountered, and
	 * more importantly, we'll incorrectly deauthenticate
	 * ourself because the inactivity timer will kick us off. 
	 */
	if (ic->ic_opmode != IEEE80211_M_STA)
		ic->ic_inact_timer = IEEE80211_INACT_WAIT;
	write_unlock_bh(&ic->ic_nodelock);
}

struct ieee80211_node *
ieee80211_alloc_node(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;

	ni = kmalloc(sizeof(struct ieee80211_node) + ic->ic_node_privlen,
		GFP_ATOMIC);
	if (ni != NULL) {
		memset(ni, 0, sizeof(struct ieee80211_node));
		ieee80211_setup_node(ic, ni, macaddr);
	}
	return ni;
}

struct ieee80211_node *
ieee80211_dup_bss(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;

	ni = kmalloc(sizeof(struct ieee80211_node) + ic->ic_node_privlen,
		GFP_ATOMIC);
	if (ni != NULL) {
		memcpy(ni, &ic->ic_bss, sizeof(struct ieee80211_node));
		ieee80211_setup_node(ic, ni, macaddr);
	}
	return ni;
}

/* 
 * Find a node state block given the mac address.  Note that
 * this returns the first node found with the mac address.
 */
struct ieee80211_node *
ieee80211_find_node(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;
	int hash;

	hash = IEEE80211_NODE_HASH(macaddr);
	read_lock_bh(&ic->ic_nodelock);
	LIST_FOREACH(ni, &ic->ic_hash[hash], ni_hash) {
		if (IEEE80211_ADDR_EQ(ni->ni_macaddr, macaddr)) {
			atomic_inc(&ni->ni_refcnt);	/* mark referenced */
			break;
		}
	}
	read_unlock_bh(&ic->ic_nodelock);
	return ni;
}

/*
 * Like find but search based on the channel too.
 */
struct ieee80211_node *
ieee80211_lookup_node(struct ieee80211com *ic,
	u_int8_t *macaddr, struct ieee80211channel *chan)
{
	struct ieee80211_node *ni;
	int hash;

	hash = IEEE80211_NODE_HASH(macaddr);
	read_lock_bh(&ic->ic_nodelock);
	LIST_FOREACH(ni, &ic->ic_hash[hash], ni_hash) {
		if (IEEE80211_ADDR_EQ(ni->ni_macaddr, macaddr) && ni->ni_chan == chan) {
			atomic_inc(&ni->ni_refcnt);	/* mark referenced */
			break;
		}
	}
	read_unlock_bh(&ic->ic_nodelock);
	return ni;
}

void
_ieee80211_free_node(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (atomic_read(&ni->ni_refcnt) != 0) {
		struct net_device *dev = &ic->ic_dev;
		printk("%s: freeing node %s with non-zero refcnt %u\n",
			dev->name, ether_sprintf(ni->ni_macaddr),
			atomic_read(&ni->ni_refcnt));
	}
#ifdef CONFIG_PROC_FS
	ieee80211_proc_del_sta(ic, ni);
#endif
	if (ic->ic_node_free != NULL)
		(*ic->ic_node_free)(ic, ni);
	TAILQ_REMOVE(&ic->ic_node, ni, ni_list);
	LIST_REMOVE(ni, ni_hash);
	if (TAILQ_EMPTY(&ic->ic_node))
		ic->ic_inact_timer = 0;

	if (ni->ni_rxfragskb) {
		kfree_skb(ni->ni_rxfragskb);
	}

	kfree(ni);
}

void
ieee80211_free_node(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (atomic_dec_and_test(&ni->ni_refcnt)) {
		write_lock_bh(&ic->ic_nodelock);
		_ieee80211_free_node(ic, ni);
		write_unlock_bh(&ic->ic_nodelock);
	}
}

void
ieee80211_free_allnodes(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;

	write_lock_bh(&ic->ic_nodelock);
	while ((ni = TAILQ_FIRST(&ic->ic_node)) != NULL)
		_ieee80211_free_node(ic, ni);  
	write_unlock_bh(&ic->ic_nodelock);
}

void
ieee80211_timeout_nodes(struct ieee80211com *ic)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_node *ni, *nextbs;

	write_lock(&ic->ic_nodelock);
	for (ni = TAILQ_FIRST(&ic->ic_node); ni != NULL;) {
		/* Free fragment skb if not needed anymore (last fragment older than 1s) */
		if (ni->ni_rxfragskb && (jiffies > ni->ni_rxfragstamp + HZ)) {
			kfree_skb(ni->ni_rxfragskb);
			ni->ni_rxfragskb = NULL;
		}
		if (++ni->ni_inact <= IEEE80211_INACT_MAX) {
			ni = TAILQ_NEXT(ni, ni_list);
			continue;
		}
		/* NB: don't honor reference count */
		DPRINTF(ic, ("%s: station %s timed out "
			    "due to inactivity (%u secs)\n",
			    dev->name,
			    ether_sprintf(ni->ni_macaddr),
			    ni->ni_inact));
		nextbs = TAILQ_NEXT(ni, ni_list);
		IEEE80211_SEND_MGMT(ic, ni,
		    IEEE80211_FC0_SUBTYPE_DEAUTH,
		    IEEE80211_REASON_AUTH_EXPIRE);
		_ieee80211_free_node(ic, ni);
		ni = nextbs;
	}
	if (!TAILQ_EMPTY(&ic->ic_node))
		ic->ic_inact_timer = IEEE80211_INACT_WAIT;
	write_unlock(&ic->ic_nodelock);
}

void
ieee80211_iterate_nodes(struct ieee80211com *ic, ieee80211_iter_func *f, void *arg)
{
	struct ieee80211_node *ni;

	read_lock_bh(&ic->ic_nodelock);
	TAILQ_FOREACH(ni, &ic->ic_node, ni_list)
		(*f)(arg, ni);
	read_unlock_bh(&ic->ic_nodelock);
}

static void
ieee80211_reset_recvhist(struct ieee80211_node* ni)
{
	int i;

	for (i = 0; i < IEEE80211_RECV_HIST_LEN; ++i) {
		ni->ni_recv_hist[i].hi_jiffies = IEEE80211_JIFFIES_NONE;
		ni->ni_recv_hist[i].hi_rssi = 0;
		ni->ni_recv_hist[i].hi_rstamp = 0;
		ni->ni_recv_hist[i].hi_rantenna = 0;
	}
	ni->ni_hist_cur = IEEE80211_RECV_HIST_LEN-1;
}

static void
ieee80211_add_recvhist(struct ieee80211_node* ni, u_int8_t rssi,
			u_int32_t rstamp, u_int8_t rantenna)
{
	if (++ni->ni_hist_cur >= IEEE80211_RECV_HIST_LEN)
		ni->ni_hist_cur = 0;
	ni->ni_recv_hist[ni->ni_hist_cur].hi_jiffies = jiffies;
	ni->ni_recv_hist[ni->ni_hist_cur].hi_rssi = rssi;
	ni->ni_recv_hist[ni->ni_hist_cur].hi_rstamp = rstamp;
	ni->ni_recv_hist[ni->ni_hist_cur].hi_rantenna = rantenna;
}

u_int8_t
ieee80211_get_rssi(struct ieee80211_node* ni)
{
	int i, ns, total;

	ns = 0;
	total = 0;
	for (i = 0; i < IEEE80211_RECV_HIST_LEN; ++i) {
		struct ieee80211_recv_hist* hi = &ni->ni_recv_hist[i];
		if (hi->hi_jiffies != IEEE80211_JIFFIES_NONE &&
		    jiffies - hi->hi_jiffies < 1*HZ) {
			total += hi->hi_rssi;
			ns++;
		}
	}
	return (ns ? total / ns :
		/* no recent samples, use last known value */
		ni->ni_recv_hist[ni->ni_hist_cur].hi_rssi);
}

/*
 * Install received rate set information in the node's state block.
 */
static int
ieee80211_setup_rates(struct ieee80211com *ic, struct ieee80211_node *ni,
	u_int8_t *rates, u_int8_t *xrates, int flags)
{
	struct ieee80211_rateset *rs = &ni->ni_rates;

	memset(rs, 0, sizeof(*rs));
	rs->rs_nrates = rates[1];
	memcpy(rs->rs_rates, rates + 2, rs->rs_nrates);
	if (xrates != NULL) {
		u_int8_t nxrates;
		/*
		 * Tack on 11g extended supported rate element.
		 */
		nxrates = xrates[1];
		if (rs->rs_nrates + nxrates > IEEE80211_RATE_MAXSIZE) {
			nxrates = IEEE80211_RATE_MAXSIZE - rs->rs_nrates;
			DPRINTF(ic, ("%s: extended rate set too large;"
				" only using %u of %u rates\n",
				__func__, nxrates, xrates[1]));
		}
		memcpy(rs->rs_rates + rs->rs_nrates, xrates+2, nxrates);
		rs->rs_nrates += nxrates;
	}
	return ieee80211_fix_rate(ic, ni, flags);
}

int
ieee80211_fix_rate(struct ieee80211com *ic, struct ieee80211_node *ni, int flags)
{
#define	RV(v)	((v) & IEEE80211_RATE_VAL)
	int i, j, ignore, error;
	int okrate, badrate;
	struct ieee80211_rateset *srs, *nrs;
	u_int8_t r;

	error = 0;
	okrate = badrate = 0;
	srs = &ic->ic_sup_rates[ieee80211_chan2mode(ic, ni->ni_chan)];
	nrs = &ni->ni_rates;
	for (i = 0; i < ni->ni_rates.rs_nrates; ) {
		ignore = 0;
		if (flags & IEEE80211_F_DOSORT) {
			/*
			 * Sort rates.
			 */
			for (j = i + 1; j < nrs->rs_nrates; j++) {
				if (RV(nrs->rs_rates[i]) > RV(nrs->rs_rates[j])) {
					r = nrs->rs_rates[i];
					nrs->rs_rates[i] = nrs->rs_rates[j];
					nrs->rs_rates[j] = r;
				}
			}
		}
		r = nrs->rs_rates[i] & IEEE80211_RATE_VAL;
		badrate = r;
		if (flags & IEEE80211_F_DOFRATE) {
			/*
			 * Apply fixed rate constraint.  Note that we do
			 * not apply the constraint to basic rates as
			 * otherwise we may not be able to associate if
			 * the rate set we submit to the AP is invalid
			 * (e.g. fix rate at 36Mb/s which is not a basic
			 * rate for 11a operation).
			 */
			if ((nrs->rs_rates[i] & IEEE80211_RATE_BASIC) == 0 &&
			    ic->ic_fixed_rate >= 0 &&
			    r != RV(srs->rs_rates[ic->ic_fixed_rate]))
				ignore++;
		}
		if (flags & IEEE80211_F_DONEGO) {
			/*
			 * Check against supported rates.
			 */
			for (j = 0; j < srs->rs_nrates; j++) {
				if (r == RV(srs->rs_rates[j]))
					break;
			}
			if (j == srs->rs_nrates) {
				if (nrs->rs_rates[i] & IEEE80211_RATE_BASIC)
					error++;
				ignore++;
			}
		}
		if (flags & IEEE80211_F_DODEL) {
			/*
			 * Delete unacceptable rates.
			 */
			if (ignore) {
				nrs->rs_nrates--;
				for (j = i; j < nrs->rs_nrates; j++)
					nrs->rs_rates[j] = nrs->rs_rates[j + 1];
				nrs->rs_rates[j] = 0;
				continue;
			}
		}
		if (!ignore)
			okrate = nrs->rs_rates[i];
		i++;
	}

	if (ic->ic_fixed_rate != -1) {
		/*
		 * make ni_txrate point to right index
		 */
		for (i = 0; i < ni->ni_rates.rs_nrates; i++ ) {
			if (RV(srs->rs_rates[ic->ic_fixed_rate]) == RV(nrs->rs_rates[i])) {
				ni->ni_txrate = i;
				break;
			}
			if ((i == ni->ni_rates.rs_nrates) && (ni->ni_rates.rs_nrates > 0))
				ni->ni_txrate =  ni->ni_rates.rs_nrates - 1;
		}
	} else {
		if (ni->ni_rates.rs_nrates > 0)
			ni->ni_txrate =  ni->ni_rates.rs_nrates - 1;
	}
	if (okrate == 0 || error != 0)
		return badrate | IEEE80211_RATE_BASIC;
	else
		return RV(okrate);
#undef RV
}

/*
 * Add a supported rates element id to a frame.
 */
u_int8_t *
ieee80211_add_rates(u_int8_t *frm, const struct ieee80211_rateset *rs)
{
	int nrates;

	*frm++ = IEEE80211_ELEMID_RATES;
	nrates = rs->rs_nrates;
	if (nrates > IEEE80211_RATE_SIZE)
		nrates = IEEE80211_RATE_SIZE;
	*frm++ = nrates;
	memcpy(frm, rs->rs_rates, nrates);
	return frm + nrates;
}

/*
 * Add an extended supported rates element id to a frame.
 */
u_int8_t *
ieee80211_add_xrates(u_int8_t *frm, const struct ieee80211_rateset *rs)
{
	/*
	 * Add an extended supported rates element if operating in 11g mode.
	 */
	if (rs->rs_nrates > IEEE80211_RATE_SIZE) {
		int nrates = rs->rs_nrates - IEEE80211_RATE_SIZE;
		*frm++ = IEEE80211_ELEMID_XRATES;
		*frm++ = nrates;
		memcpy(frm, rs->rs_rates + IEEE80211_RATE_SIZE, nrates);
		frm += nrates;
	}
	return frm;
}

/* 
 * Add an ssid elemet to a frame.
 */
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
	enum ieee80211_phymode mode;

	/*
	 * prreq frame format
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[tlv] extended supported rates
	 */
	pktlen = 2 + ic->ic_des_esslen
	       + 2 + IEEE80211_RATE_SIZE
	       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
	       ;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL) {
		DPRINTF(ic, ("ieee80211_send_prreq: no space\n"));
		return ENOMEM;
	}
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = skb_put(skb, pktlen);
	frm = ieee80211_add_ssid(frm, ic->ic_des_essid, ic->ic_des_esslen);
	mode = ieee80211_chan2mode(ic, ni->ni_chan);
	frm = ieee80211_add_rates(frm, &ic->ic_sup_rates[mode]);
	frm = ieee80211_add_xrates(frm, &ic->ic_sup_rates[mode]);
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
	 *	[tlv] extended supported rates
	 */
	pktlen = 8 + 2 + 2 + 2
	       + 2 + ni->ni_esslen
	       + 2 + IEEE80211_RATE_SIZE
	       + 3 + 6
	       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
	       ;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = skb_put(skb, pktlen);

	memset(frm, 0, 8);	/* NB: driver is required to fillin timestamp */
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
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;

	frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
	frm = ieee80211_add_rates(frm, &ni->ni_rates);

	*frm++ = IEEE80211_ELEMID_DSPARMS;
	*frm++ = 1;
	*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
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
	frm = ieee80211_add_xrates(frm, &ni->ni_rates);
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

	DPRINTF(ic, ("%s: station %s deauthenticate (reason %d)\n",
		    dev->name, ether_sprintf(ni->ni_macaddr), reason));
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
	u_int16_t capinfo = 0;
	int ret, pktlen;

	/*
	 * asreq frame format
	 *	[2] capability information
	 *	[2] listen interval
	 *	[6*] current AP address (reassoc only)
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[tlv] extended supported rates
	 */
	pktlen = sizeof(capinfo)
	       + sizeof(u_int16_t)
	       + IEEE80211_ADDR_LEN
	       + 2 + ni->ni_esslen
	       + 2 + IEEE80211_RATE_SIZE
	       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
	       ;
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
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;

	*(u_int16_t *)frm = cpu_to_le16(ic->ic_lintval);
	frm += 2;

	if (type == IEEE80211_FC0_SUBTYPE_REASSOC_REQ) {
		IEEE80211_ADDR_COPY(frm, ic->ic_bss.ni_bssid);
		frm += IEEE80211_ADDR_LEN;
	}

	frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
	frm = ieee80211_add_rates(frm, &ni->ni_rates);
	frm = ieee80211_add_xrates(frm, &ni->ni_rates);
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
	 *	[tlv] extended supported rates
	 */
	pktlen = sizeof(capinfo)
	       + sizeof(u_int16_t)
	       + sizeof(u_int16_t)
	       + 2 + IEEE80211_RATE_SIZE
	       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
	       ;
	skb = dev_alloc_skb(sizeof(struct ieee80211_frame) + pktlen);
	if (skb == NULL)
		return ENOMEM;
	skb_reserve(skb, sizeof(struct ieee80211_frame));

	frm = (u_int8_t *) skb_put(skb, pktlen);

	capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_WEPON)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;

	*(u_int16_t *)frm = cpu_to_le16(status);
	frm += 2;

	if (status == IEEE80211_STATUS_SUCCESS && ni != NULL)
		*(u_int16_t *)frm = cpu_to_le16(ni->ni_associd);
	else
		*(u_int16_t *)frm = cpu_to_le16(0);
	frm += 2;

	if (ni != NULL) {
		frm = ieee80211_add_rates(frm, &ni->ni_rates);
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
	} else {
		frm = ieee80211_add_rates(frm, &ic->ic_bss.ni_rates);
		frm = ieee80211_add_xrates(frm, &ic->ic_bss.ni_rates);
	}
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

	DPRINTF(ic, ("%s: station %s disassociate (reason %d)\n",
		    dev->name, ether_sprintf(ni->ni_macaddr), reason));
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

/* Verify the existence and length of __elem or get out. */
#define IEEE80211_VERIFY_ELEMENT(__elem, __maxlen, __wh) do {		\
	if ((__elem) == NULL) {						\
		DPRINTF(ic, ("%s: no " #__elem "\n", __func__));	\
		return;							\
	}								\
	if ((__elem)[1] > (__maxlen)) {					\
		DPRINTF(ic, ("%s: bad " #__elem " len %d from %s\n",	\
		    __func__, (__elem)[1],				\
		    ether_sprintf((__wh)->i_addr2)));			\
		return;							\
	}								\
} while (0)

/*
 * NB: This handles both beacon and probe response frames.
 */
static void
ieee80211_recv_beacon(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
{
#define	ISPROBE(_wh)	IEEE80211_IS_SUBTYPE((_wh)->i_fc[0], PROBE_RESP)
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm, *tstamp, *bintval, *capinfo, *ssid;
	u_int8_t *rates, *xrates, *country;
	u_int8_t chan, bchan, fhindex, erp;
	u_int16_t fhdwell;

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
	 *	[tlv] country information
	 *	[tlv] parameter set (FH/DS)
	 *	[tlv] erp information
	 *	[tlv] extended supported rates
	 */
	tstamp  = frm;	frm += 8;
	bintval = frm;	frm += 2;
	capinfo = frm;	frm += 2;
	ssid = rates = xrates = country = NULL;
	bchan = ieee80211_chan2ieee(ic, ic->ic_bss.ni_chan);
	chan = bchan;
	fhdwell = 0;
	fhindex = 0;
	erp = 0;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_SSID:
			ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		case IEEE80211_ELEMID_COUNTRY:
			country = frm;
			break;
		case IEEE80211_ELEMID_FHPARMS:
			if (ic->ic_phytype == IEEE80211_T_FH) {
				fhdwell = (frm[3] << 8) | frm[2];
				chan = IEEE80211_FH_CHAN(frm[4], frm[5]);
				fhindex = frm[6];
			}
			break;
		case IEEE80211_ELEMID_DSPARMS:
			/*
			 * XXX hack this since depending on phytype
			 * is problematic for multi-mode devices.
			 */
			if (ic->ic_phytype != IEEE80211_T_FH)
				chan = frm[2];
			break;
		case IEEE80211_ELEMID_TIM:
			break;
		case IEEE80211_ELEMID_XRATES:
			xrates = frm;
			break;
		case IEEE80211_ELEMID_ERP:
			if (frm[1] != 1) {
				DPRINTF(ic, ("%s: invalid ERP element; "
					"length %u, expecting 1\n",
					__func__, frm[1]));
				break;
			}
			erp = frm[2];
			break;
		default:
			DPRINTF(ic, ("%s: element id %u/len %u ignored\n",
				__func__, *frm, frm[1]));
			break;
		}
		frm += frm[1] + 2;
	}
	IEEE80211_VERIFY_ELEMENT(rates, IEEE80211_RATE_MAXSIZE, wh);
	IEEE80211_VERIFY_ELEMENT(ssid, IEEE80211_NWID_LEN, wh);
	if (chan > IEEE80211_CHAN_MAX || isclr(ic->ic_chan_active, chan)) {
		DPRINTF(ic, ("%s: ignore %s with invalid channel %u\n",
			__func__, ISPROBE(wh) ? "probe response" : "beacon",
			chan));
		return;
	}
	if (chan != bchan) {
		/*
		 * Frame was received on a channel different from the
		 * one indicated in the DS/FH params element id; silently
		 * discard it.
		 *
		 * NB: this can happen due to signal leakage.
		 */
		DPRINTF(ic, ("%s: ignore %s on channel %u marked for channel %u\n",
			__func__, ISPROBE(wh) ? "probe response" : "beacon",
			bchan, chan));
		/* XXX statistic */
		return;
	}

	/*
	 * Use mac and channel for lookup so we collect all
	 * potential AP's when scanning.  Otherwise we may
	 * see the same AP on multiple channels and will only
	 * record the last one.  We could filter APs here based
	 * on rssi, etc. but leave that to the end of the scan
	 * so we can keep the selection criteria in one spot.
	 * This may result in a bloat of the scanned AP list but
	 * it shouldn't be too much.
	 */
	ni = ieee80211_lookup_node(ic, wh->i_addr2, &ic->ic_channels[chan]);
#ifdef IEEE80211_DEBUG
	if (netif_msg_debug(ic) &&
	    (ni == NULL || ic->ic_state == IEEE80211_S_SCAN)) {
		printk("%s: %s%s on chan %u (bss chan %u) ",
		    __func__, (ni == NULL ? "new " : ""),
		    ISPROBE(wh) ? "probe response" : "beacon",
		    chan, bchan);
		ieee80211_print_essid(ssid + 2, ssid[1]);
		printk(" from %s\n", ether_sprintf(wh->i_addr2));
		printk("%s: caps 0x%x bintval %u erp 0x%x\n",
			__func__, le16_to_cpu(*(u_int16_t *)capinfo),
			le16_to_cpu(*(u_int16_t *)bintval), erp);
#if 0
		if (country)
			printk("%s: country info %*D\n",
				__func__, country[1], country+2, " ");
#endif
	}
#endif
	if (ni && ic->ic_opmode == IEEE80211_M_STA &&
	    ic->ic_state != IEEE80211_S_SCAN &&
	    IEEE80211_ADDR_EQ(wh->i_addr2, ic->ic_bss.ni_macaddr)) {
		/* from our ap but we are not scanning, update stats only */
		ieee80211_add_recvhist(ni, rssi, rstamp, rantenna);
		ieee80211_unref_node(&ni);
		return;
	}
	if (ic->ic_opmode != IEEE80211_M_IBSS &&
	    ic->ic_state != IEEE80211_S_SCAN) {
		if (ni != NULL)
			ieee80211_unref_node(&ni);
		return;
	}

	if (ni == NULL) {
		ni = ieee80211_alloc_node(ic, wh->i_addr2);
		if (ni == NULL)
			return;
		ni->ni_esslen = ssid[1];
		memset(ni->ni_essid, 0, sizeof(ni->ni_essid));
		memcpy(ni->ni_essid, ssid + 2, ssid[1]);
	} else if (ssid[1] != 0 && ISPROBE(wh)) {
		/*
		 * Update ESSID at probe response to adopt hidden AP by
		 * Lucent/Cisco, which announces null ESSID in beacon.
		 */
		ni->ni_esslen = ssid[1];
		memset(ni->ni_essid, 0, sizeof(ni->ni_essid));
		memcpy(ni->ni_essid, ssid + 2, ssid[1]);
	}
	IEEE80211_ADDR_COPY(ni->ni_bssid, wh->i_addr3);
	ieee80211_add_recvhist(ni, rssi, rstamp, rantenna);
	memcpy(ni->ni_tstamp, tstamp, sizeof(ni->ni_tstamp));
	ni->ni_intval = le16_to_cpu(*(u_int16_t *)bintval);
	ni->ni_capinfo = le16_to_cpu(*(u_int16_t *)capinfo);
	ni->ni_chan = &ic->ic_channels[chan];
	ni->ni_fhdwell = fhdwell;
	ni->ni_fhindex = fhindex;
	ni->ni_erp = erp;
	/* NB: must be after ni_chan is setup */
	ieee80211_setup_rates(ic, ni, rates, xrates, IEEE80211_F_DOSORT);
	ieee80211_unref_node(&ni);
#undef	ISPROBE
}

static void
ieee80211_recv_prreq(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
{
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm, *ssid, *rates, *xrates;
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
	 *	[tlv] extended supported rates
	 */
	ssid = rates = xrates = NULL;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_SSID:
			ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		case IEEE80211_ELEMID_XRATES:
			xrates = frm;
			break;
		}
		frm += frm[1] + 2;
	}
	IEEE80211_VERIFY_ELEMENT(rates, IEEE80211_RATE_MAXSIZE, wh);
	IEEE80211_VERIFY_ELEMENT(ssid, IEEE80211_NWID_LEN, wh);
	if (ssid[1] != 0 &&
	    (ssid[1] != ic->ic_bss.ni_esslen ||
	    memcmp(ssid + 2, ic->ic_bss.ni_essid, ic->ic_bss.ni_esslen) != 0)) {
		if (netif_msg_debug(ic)) {
			printk("%s: ssid unmatch ", __func__);
			ieee80211_print_essid(ssid + 2, ssid[1]);
			printk(" from %s\n", ether_sprintf(wh->i_addr2));
		}
		return;
	}

	ni = ieee80211_find_node(ic, wh->i_addr2);
	if (ni == NULL) {
		ni = ieee80211_dup_bss(ic, wh->i_addr2);
		if (ni == NULL)
			return;
		DPRINTF(ic, ("%s: new req from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		allocbs = 1;
	} else
		allocbs = 0;
	ieee80211_add_recvhist(ni, rssi, rstamp, rantenna);
	rate = ieee80211_setup_rates(ic, ni, rates, xrates,
			IEEE80211_F_DOSORT | IEEE80211_F_DOFRATE
			| IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
	if (rate & IEEE80211_RATE_BASIC) {
		DPRINTF(ic, ("%s: rate negotiation failed: %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
	} else {
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_PROBE_RESP,
		    0);
	}
	if (allocbs && ic->ic_opmode == IEEE80211_M_HOSTAP)
		ieee80211_free_node(ic, ni);
	else
		ieee80211_unref_node(&ni);
}

static void
ieee80211_recv_auth(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
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
		DPRINTF(ic, ("%s: too short from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		return;
	}
	algo   = le16_to_cpu(*(u_int16_t *)frm);
	seq    = le16_to_cpu(*(u_int16_t *)(frm + 2));
	status = le16_to_cpu(*(u_int16_t *)(frm + 4));
	if (algo != IEEE80211_AUTH_ALG_OPEN) {
		/* TODO: shared key auth */
		DPRINTF(ic, ("%s: unsupported auth %d from %s\n",
			__func__, algo, ether_sprintf(wh->i_addr2)));
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
			ni = ieee80211_alloc_node(ic, wh->i_addr2);
			if (ni == NULL)
				return;
			IEEE80211_ADDR_COPY(ni->ni_bssid, ic->ic_bss.ni_bssid);
			ieee80211_add_recvhist(ni, rssi, rstamp, rantenna);
			ni->ni_chan = ic->ic_bss.ni_chan;
			allocbs = 1;
		}
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_AUTH, 2);
		DPRINTF(ic, ("%s: station %s %s authenticated\n",
			    dev->name,
			    (allocbs ? "newly" : "already"),
			    ether_sprintf(ni->ni_macaddr)));
		ieee80211_unref_node(&ni);
		break;

	case IEEE80211_M_STA:
		if (ic->ic_state != IEEE80211_S_AUTH || seq != 2)
			return;
		if (status != 0) {
			printk("%s: authentication failed (reason %d) for %s\n",
			    dev->name, status,
			    ether_sprintf(wh->i_addr3));
			ni = ieee80211_find_node(ic, wh->i_addr2);
			if (ni != NULL) {
				ni->ni_fails++;
				ieee80211_unref_node(&ni);
			}
			return;
		}
		ieee80211_new_state(&ic->ic_dev, IEEE80211_S_ASSOC,
		    wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
		break;

	case IEEE80211_M_MONITOR:
		printk("%s: unexpected in monitor mode\n", __func__);
		break;
	}
}

static void
ieee80211_recv_asreq(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = &ic->ic_bss;
	u_int8_t *frm, *efrm, *ssid, *rates, *xrates;
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
	 *	[tlv] extended supported rates
	 */
	if (frm + (reassoc ? 10 : 4) > efrm) {
		DPRINTF(ic, ("%s: too short from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		return;
	}

	if (!IEEE80211_ADDR_EQ(wh->i_addr3, ic->ic_bss.ni_bssid)) {
		DPRINTF(ic, ("%s: ignore other bss from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		return;
	}
	capinfo = le16_to_cpu(*(u_int16_t *)frm);	frm += 2;
	bintval = le16_to_cpu(*(u_int16_t *)frm);	frm += 2;
	if (reassoc)
		frm += 6;	/* ignore current AP info */
	ssid = rates = xrates = NULL;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_SSID:
			ssid = frm;
			break;
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		case IEEE80211_ELEMID_XRATES:
			xrates = frm;
			break;
		}
		frm += frm[1] + 2;
	}
	IEEE80211_VERIFY_ELEMENT(rates, IEEE80211_RATE_MAXSIZE, wh);
	IEEE80211_VERIFY_ELEMENT(ssid, IEEE80211_NWID_LEN, wh);
	if (ssid[1] != ic->ic_bss.ni_esslen ||
	    memcmp(ssid + 2, ic->ic_bss.ni_essid, ssid[1]) != 0) {
		if (netif_msg_debug(ic)) {
			printk("%s: ssid unmatch ", __func__);
			ieee80211_print_essid(ssid + 2, ssid[1]);
			printk(" from %s\n", ether_sprintf(wh->i_addr2));
		}
		return;
	}
	ni = ieee80211_find_node(ic, wh->i_addr2);
	if (ni == NULL) {
		DPRINTF(ic, ("%s: not authenticated for %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		ni = ieee80211_dup_bss(ic, wh->i_addr2);
		if (ni == NULL)
			return;
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_DEAUTH,
		    IEEE80211_REASON_ASSOC_NOT_AUTHED);
		ieee80211_free_node(ic, ni);
		return;
	}
	/* XXX per-node cipher suite */
	/* XXX some stations use the privacy bit for handling APs
	       that suport both encrypted and unencrypted traffic */
	if ((capinfo & IEEE80211_CAPINFO_ESS) == 0 ||
	    (capinfo & IEEE80211_CAPINFO_PRIVACY) !=
	    ((ic->ic_flags & IEEE80211_F_WEPON) ?
	     IEEE80211_CAPINFO_PRIVACY : 0)) {
		DPRINTF(ic, ("%s: capability unmatch %x for %s\n",
			__func__, capinfo, ether_sprintf(wh->i_addr2)));
		ni->ni_associd = 0;
		IEEE80211_SEND_MGMT(ic, ni, resp, IEEE80211_STATUS_CAPINFO);
		ieee80211_unref_node(&ni);
		return;
	}
	ieee80211_setup_rates(ic, ni, rates, xrates,
			IEEE80211_F_DOSORT | IEEE80211_F_DOFRATE |
			IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
	if (ni->ni_rates.rs_nrates == 0) {
		DPRINTF(ic, ("%s: rate unmatch for %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		ni->ni_associd = 0;
		IEEE80211_SEND_MGMT(ic, ni, resp, IEEE80211_STATUS_BASIC_RATE);
		ieee80211_unref_node(&ni);
		return;
	}
	ieee80211_add_recvhist(ni, rssi, rstamp, rantenna);
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
	/* XXX for 11g must turn off short slot time if long
	       slot time sta associates */
	IEEE80211_SEND_MGMT(ic, ni, resp, IEEE80211_STATUS_SUCCESS);
	DPRINTF(ic, ("%s: station %s %s associated\n",
		    dev->name,
		    (newassoc ? "newly" : "already"),
		    ether_sprintf(ni->ni_macaddr)));
#ifdef CONFIG_PROC_FS
	ieee80211_proc_add_sta(ic, ni);
#endif
	ieee80211_unref_node(&ni);
}

static void
ieee80211_recv_asresp(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
{
	struct net_device *dev = &ic->ic_dev;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	u_int8_t *frm, *efrm, *rates, *xrates;
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
	 *	[tlv] extended supported rates
	 */
	if (frm + 6 > efrm) {
		DPRINTF(ic, ("%s: too short from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
		return;
	}

	ni = &ic->ic_bss;
	ni->ni_capinfo = le16_to_cpu(*(u_int16_t *)frm);
	frm += 2;

	status = le16_to_cpu(*(u_int16_t *)frm);
	frm += 2;
	if (status != 0) {
		printk("%s: association failed (reason %d) for %s\n",
		    dev->name, status, ether_sprintf(wh->i_addr3));
		ni = ieee80211_find_node(ic, wh->i_addr2);
		if (ni != NULL) {
			ni->ni_fails++;
			ieee80211_unref_node(&ni);
		}
		return;
	}
	ni->ni_associd = le16_to_cpu(*(u_int16_t *)frm);
	frm += 2;

	rates = xrates = NULL;
	while (frm < efrm) {
		switch (*frm) {
		case IEEE80211_ELEMID_RATES:
			rates = frm;
			break;
		case IEEE80211_ELEMID_XRATES:
			xrates = frm;
			break;
		}
		frm += frm[1] + 2;
	}

	IEEE80211_VERIFY_ELEMENT(rates, IEEE80211_RATE_MAXSIZE, wh);
	ieee80211_setup_rates(ic, ni, rates, xrates,
			IEEE80211_F_DOSORT | IEEE80211_F_DOFRATE |
			IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
	if (ni->ni_rates.rs_nrates != 0)
		ieee80211_new_state(&ic->ic_dev, IEEE80211_S_RUN,
			wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK);
}

static void
ieee80211_recv_disassoc(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
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
		DPRINTF(ic, ("%s: too short from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
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
			DPRINTF(ic, ("%s: station %s disassociated"
				    " by peer (reason %d)\n",
				    dev->name,
				    ether_sprintf(ni->ni_macaddr), reason));
			ni->ni_associd = 0;
#ifdef CONFIG_PROC_FS
			ieee80211_proc_del_sta(ic, ni);
#endif
			ieee80211_unref_node(&ni);
		}
		break;
	default:
		break;
	}
}

static void
ieee80211_recv_deauth(struct ieee80211com *ic, struct sk_buff *skb0, int rssi,
    u_int32_t rstamp, u_int rantenna)
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
		DPRINTF(ic, ("%s: too short from %s\n",
			__func__, ether_sprintf(wh->i_addr2)));
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
			DPRINTF(ic, ("%s: station %s deauthenticated"
				    " by peer (reason %d)\n",
				    dev->name,
				    ether_sprintf(ni->ni_macaddr), reason));
#ifdef CONFIG_PROC_FS
			ieee80211_proc_del_sta(ic, ni);
#endif
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
	struct ieee80211_node *ni;
	int error, ostate;
#ifdef IEEE80211_DEBUG
	static const char *stname[] = 
	    { "INIT", "SCAN", "AUTH", "ASSOC", "RUN" };
#endif

	ostate = ic->ic_state;
	DPRINTF(ic, ("%s: %s -> %s\n",
		__func__, stname[ostate], stname[nstate]));
	if (ic->ic_newstate) {
		error = (*ic->ic_newstate)(dev, nstate);
		if (error == EINPROGRESS)
			return 0;
		if (error != 0)
			return error;
	}

	/* state transition */
	ic->ic_state = nstate;
	ni = &ic->ic_bss;			/* NB: no reference held */
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
				read_lock_bh(&ic->ic_nodelock);
				TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
					if (ni->ni_associd == 0)
						continue;
					IEEE80211_SEND_MGMT(ic, ni,
					    IEEE80211_FC0_SUBTYPE_DISASSOC,
					    IEEE80211_REASON_ASSOC_LEAVE);
				}
				read_unlock_bh(&ic->ic_nodelock);
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
				read_lock_bh(&ic->ic_nodelock);
				TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
					IEEE80211_SEND_MGMT(ic, ni,
					    IEEE80211_FC0_SUBTYPE_DEAUTH,
					    IEEE80211_REASON_AUTH_LEAVE);
				}
				read_unlock_bh(&ic->ic_nodelock);
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
		/* initialize bss for probe request */
		IEEE80211_ADDR_COPY(ni->ni_macaddr, dev->broadcast);
		IEEE80211_ADDR_COPY(ni->ni_bssid, dev->broadcast);
		ni->ni_rates = ic->ic_sup_rates[
			ieee80211_chan2mode(ic, ni->ni_chan)];
		ni->ni_associd = 0;
		ieee80211_reset_recvhist(ni);
		switch (ostate) {
		case IEEE80211_S_INIT:
			if ((ic->ic_opmode == IEEE80211_M_HOSTAP ||
			    ic->ic_opmode == IEEE80211_M_IBSS) &&
			    ic->ic_des_chan != IEEE80211_CHAN_ANYC) {
				/*
				 * AP operation and we already have a channel;
				 * bypass the scan and startup immediately.
				 * Same applies to ad-hoc mode.
				 */
				ieee80211_create_ibss(dev, ic->ic_des_chan);
			} else {
				ieee80211_begin_scan(dev, ni);
			}
			break;
		case IEEE80211_S_SCAN:
			/* scan next */
			if (ic->ic_flags & IEEE80211_F_ASCAN) {
				IEEE80211_SEND_MGMT(ic, ni,
				    IEEE80211_FC0_SUBTYPE_PROBE_REQ, 0);
			} else
				dev->trans_start = jiffies;
			break;
		case IEEE80211_S_RUN:
			/* beacon miss */
			/* XXX bssid clobbered above */
			DPRINTF(ic, ("%s: no recent beacons from %s;"
				    " rescanning\n",
				    dev->name,
				    ether_sprintf(ic->ic_bss.ni_bssid)));
			ieee80211_free_allnodes(ic);
			/* FALLTHRU */
		case IEEE80211_S_AUTH:
		case IEEE80211_S_ASSOC:
			/* timeout restart scan */
			ni = ieee80211_find_node(ic, ic->ic_bss.ni_macaddr);
			if (ni != NULL) {
				ni->ni_fails++;
				ieee80211_unref_node(&ni);
			}
			ieee80211_begin_scan(dev, &ic->ic_bss);
			break;
		}
		break;
	case IEEE80211_S_AUTH:
		switch (ostate) {
		case IEEE80211_S_INIT:
			DPRINTF(ic, ("%s: invalid transition\n", __func__));
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
			DPRINTF(ic, ("%s: invalid transition\n", __func__));
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
			if (ic->ic_opmode != IEEE80211_M_MONITOR)
				DPRINTF(ic, ("%s: invalid transition\n", __func__));
			break;
		case IEEE80211_S_SCAN:		/* adhoc/hostap mode */
		case IEEE80211_S_ASSOC:		/* infra mode */
			KASSERT(ni->ni_txrate < ni->ni_rates.rs_nrates,
				("%s: bogus xmit rate %u setup\n", __func__,
					ni->ni_txrate));
			if (netif_msg_debug(ic)) {
				printk("%s: ", dev->name);
				if (ic->ic_opmode == IEEE80211_M_STA)
					printk("associated ");
				else
					printk("synchronized ");
				printk("with %s ssid ",
				    ether_sprintf(ni->ni_bssid));
				ieee80211_print_essid(ni->ni_essid,
				    ni->ni_esslen);
				printk(" channel %d start %uMb\n",
					ieee80211_chan2ieee(ic, ni->ni_chan),
					IEEE80211_RATE2MBS(ni->ni_rates.rs_rates[ni->ni_txrate]));
			}
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
		ctx = kmalloc(arc4_ctxlen(), GFP_ATOMIC);
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
			if (netif_msg_debug(ic)) {
				printk("%s: decrypt CRC error\n", dev->name);
				if (ieee80211_debug > 1)
					ieee80211_dump_pkt(n0->data,
					    n0->len, -1, -1);
			}
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

static void
ieee80211_proc_add_sta(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (!ni->ni_proc) {
		ni->ni_proc = create_proc_entry(ether_sprintf(ni->ni_macaddr),
			0444, ic->ic_proc);
		if (ni->ni_proc) {
			ni->ni_proc->read_proc = ieee80211_proc_read_sta;
			ni->ni_proc->data = ni;
		}
	}
}

static void
ieee80211_proc_del_sta(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (ni->ni_proc) {
		remove_proc_entry(ether_sprintf(ni->ni_macaddr), ic->ic_proc);
		ni->ni_proc = 0;
	}
}

static int
ieee80211_proc_read_sta(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	char *p = page;
	struct ieee80211_node *ni = data;

	if (off != 0) {
		*eof = 1;
		return 0;
	}

	p += sprintf(p, "rssi: %d\n", ieee80211_get_rssi(ni));
	p += sprintf(p, "capinfo: %x\n", ni->ni_capinfo);
	p += sprintf(p, "freq: %d\n", ni->ni_chan->ic_freq);
	p += sprintf(p, "flags: %x\n", ni->ni_chan->ic_flags);
	p += sprintf(p, "txseq: %d\n", ni->ni_txseq);
	p += sprintf(p, "rxseq: %d\n", ni->ni_rxseq);
	p += sprintf(p, "fails: %d\n", ni->ni_fails);
	p += sprintf(p, "inact: %d\n", ni->ni_inact);
	if ((ni->ni_txrate >= 0) && (ni->ni_txrate < IEEE80211_RATE_MAXSIZE))
		p += sprintf(p, "txrate: %d\n", ni->ni_rates.rs_rates[ni->ni_txrate]);
	else
		p += sprintf(p, "txrate!: %d\n", ni->ni_txrate);

	return (p - page);
}

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
	struct ieee80211com *ic = data;
	int v;
	
	if (sscanf(buf, "%d", &v) == 1) {
		if (v)
			ic->msg_enable |= NETIF_MSG_DEBUG;
		else
			ic->msg_enable &= ~NETIF_MSG_DEBUG;
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
 * Return the phy mode for with the specified channel so the
 * caller can select a rate set.  This is problematic and the
 * work here assumes how things work elsewhere in this code.
 */
enum ieee80211_phymode
ieee80211_chan2mode(struct ieee80211com *ic, struct ieee80211channel *chan)
{
	/*
	 * NB: this assumes the channel would not be supplied to us
	 *     unless it was already compatible with the current mode.
	 */
	if (ic->ic_curmode != IEEE80211_MODE_AUTO)
		return ic->ic_curmode;
	/*
	 * In autoselect mode; deduce a mode based on the channel
	 * characteristics.  We assume that turbo-only channels
	 * are not considered when the channel set is constructed.
	 */
	if (IEEE80211_IS_CHAN_5GHZ(chan))
		return IEEE80211_MODE_11A;
	else if (chan->ic_flags & (IEEE80211_CHAN_OFDM|IEEE80211_CHAN_DYN))
		return IEEE80211_MODE_11G;
	else
		return IEEE80211_MODE_11B;
}

/*
 * convert IEEE80211 rate value to ifmedia subtype.
 * ieee80211 rate is in unit of 0.5Mbps.
 */
int
ieee80211_rate2media(struct ieee80211com *ic, int rate, enum ieee80211_phymode mode)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	static const struct {
		u_int	m;	/* rate + mode */
		u_int	r;	/* if_media rate */
	} rates[] = {
		{   2 | IFM_MAKEMODE(IFM_IEEE80211_11B), IFM_IEEE80211_DS1 },
		{   4 | IFM_MAKEMODE(IFM_IEEE80211_11B), IFM_IEEE80211_DS2 },
		{  11 | IFM_MAKEMODE(IFM_IEEE80211_11B), IFM_IEEE80211_DS5 },
		{  22 | IFM_MAKEMODE(IFM_IEEE80211_11B), IFM_IEEE80211_DS11 },
		{  44 | IFM_MAKEMODE(IFM_IEEE80211_11B), IFM_IEEE80211_DS22 },
		{  12 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM6 },
		{  18 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM9 },
		{  24 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM12 },
		{  36 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM18 },
		{  48 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM24 },
		{  72 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM36 },
		{  96 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM48 },
		{ 108 | IFM_MAKEMODE(IFM_IEEE80211_11A), IFM_IEEE80211_OFDM54 },
		{   2 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_DS1 },
		{   4 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_DS2 },
		{  11 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_DS5 },
		{  22 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_DS11 },
		{  12 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM6 },
		{  18 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM9 },
		{  24 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM12 },
		{  36 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM18 },
		{  48 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM24 },
		{  72 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM36 },
		{  96 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM48 },
		{ 108 | IFM_MAKEMODE(IFM_IEEE80211_11G), IFM_IEEE80211_OFDM54 },
		/* NB: OFDM72 doesn't realy exist so we don't handle it */
	};
	u_int mask, i;

	mask = rate & IEEE80211_RATE_VAL;
	switch (mode) {
	case IEEE80211_MODE_11A:
	case IEEE80211_MODE_TURBO:
		mask |= IFM_MAKEMODE(IFM_IEEE80211_11A);
		break;
	case IEEE80211_MODE_11B:
		mask |= IFM_MAKEMODE(IFM_IEEE80211_11B);
		break;
	case IEEE80211_MODE_AUTO:
		if (ic->ic_phytype == IEEE80211_T_FH) {
			/* must handle these specially */
			switch (mask) {
			case 2:		return IFM_IEEE80211_FH1;
			case 4:		return IFM_IEEE80211_FH2;
			}
			return IFM_AUTO;
		}
		/* NB: hack, 11g matches both 11b+11a rates */
		/* fall thru... */
	case IEEE80211_MODE_11G:
		mask |= IFM_MAKEMODE(IFM_IEEE80211_11G);
		break;
	}
	for (i = 0; i < N(rates); i++)
		if (rates[i].m == mask)
			return rates[i].r;
	return IFM_AUTO;
#undef N
}

int
ieee80211_media2rate(int mword)
{
#define	N(a)	(sizeof(a) / sizeof(a[0]))
	static const int ieeerates[] = {
		-1,		/* IFM_AUTO */
		0,		/* IFM_MANUAL */
		0,		/* IFM_NONE */
		2,		/* IFM_IEEE80211_FH1 */
		4,		/* IFM_IEEE80211_FH2 */
		2,		/* IFM_IEEE80211_DS1 */
		4,		/* IFM_IEEE80211_DS2 */
		11,		/* IFM_IEEE80211_DS5 */
		22,		/* IFM_IEEE80211_DS11 */
		44,		/* IFM_IEEE80211_DS22 */
		12,		/* IFM_IEEE80211_OFDM6 */
		18,		/* IFM_IEEE80211_OFDM9 */
		24,		/* IFM_IEEE80211_OFDM12 */
		36,		/* IFM_IEEE80211_OFDM18 */
		48,		/* IFM_IEEE80211_OFDM24 */
		72,		/* IFM_IEEE80211_OFDM36 */
		96,		/* IFM_IEEE80211_OFDM48 */
		108,		/* IFM_IEEE80211_OFDM54 */
		144,		/* IFM_IEEE80211_OFDM72 */
	};
	return IFM_SUBTYPE(mword) < N(ieeerates) ?
		ieeerates[IFM_SUBTYPE(mword)] : 0;
#undef N
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
#include "release.h"
#include "version.h"
static	char *version = WLAN_VERSION " " RELEASE_TYPE;
static	char *dev_info = "wlan";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("802.11 wireless LAN protocol support");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");		/* XXX really BSD only */
#endif

EXPORT_SYMBOL(ieee80211_add_rates);
EXPORT_SYMBOL(ieee80211_add_xrates);
EXPORT_SYMBOL(ieee80211_chan2ieee);
EXPORT_SYMBOL(ieee80211_chan2mode);
EXPORT_SYMBOL(ieee80211_decap);
EXPORT_SYMBOL(ieee80211_dump_pkt);
EXPORT_SYMBOL(ieee80211_encap);
EXPORT_SYMBOL(ieee80211_end_scan);
EXPORT_SYMBOL(ieee80211_find_node);
EXPORT_SYMBOL(ieee80211_ieee2mhz);
EXPORT_SYMBOL(ieee80211_ifattach);
EXPORT_SYMBOL(ieee80211_ifdetach);
EXPORT_SYMBOL(ieee80211_input);
EXPORT_SYMBOL(ieee80211_iterate_nodes);
EXPORT_SYMBOL(ieee80211_media_init);
EXPORT_SYMBOL(ieee80211_media_change);
EXPORT_SYMBOL(ieee80211_media_status);
EXPORT_SYMBOL(ieee80211_media2rate);
EXPORT_SYMBOL(ieee80211_mhz2ieee);
EXPORT_SYMBOL(ieee80211_new_state);
EXPORT_SYMBOL(ieee80211_next_scan);
EXPORT_SYMBOL(ieee80211_rate2media);
EXPORT_SYMBOL(ieee80211_setmode);

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
