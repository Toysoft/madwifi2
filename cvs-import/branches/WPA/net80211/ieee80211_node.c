/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
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
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_node.c,v 1.13 2003/11/09 23:36:46 sam Exp $");
__KERNEL_RCSID(0, "$NetBSD: ieee80211_node.c,v 1.8 2003/11/02 01:29:05 dyoung Exp $");

/*
 * IEEE 802.11 node handling support.
 */
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

static struct ieee80211_node *ieee80211_node_alloc(struct ieee80211com *);
static void ieee80211_node_free(struct ieee80211com *, struct ieee80211_node *);
static void ieee80211_node_copy(struct ieee80211com *,
		struct ieee80211_node *, const struct ieee80211_node *);
static u_int8_t ieee80211_node_getrssi(struct ieee80211com *,
		struct ieee80211_node *);

static void ieee80211_setup_node(struct ieee80211com *ic,
		struct ieee80211_node *ni, u_int8_t *macaddr);
static void _ieee80211_free_node(struct ieee80211com *,
		struct ieee80211_node *);

MALLOC_DEFINE(M_80211_NODE, "node", "802.11 node state");

void
ieee80211_node_attach(struct ieee80211com *ic)
{

	/* XXX need unit */
	IEEE80211_NODE_LOCK_INIT(ic, ic->ic_ifp->if_xname);
	TAILQ_INIT(&ic->ic_node);
	ic->ic_node_alloc = ieee80211_node_alloc;
	ic->ic_node_free = ieee80211_node_free;
	ic->ic_node_copy = ieee80211_node_copy;
	ic->ic_node_getrssi = ieee80211_node_getrssi;
	ic->ic_scangen = 1;

	if (ic->ic_max_aid == 0)
		ic->ic_max_aid = IEEE80211_AID_DEF;
	else if (ic->ic_max_aid > IEEE80211_AID_MAX)
		ic->ic_max_aid = IEEE80211_AID_MAX;
	MALLOC(ic->ic_aid_bitmap, u_int32_t *,
		howmany(ic->ic_max_aid, 32) * sizeof(u_int32_t),
		M_DEVBUF, M_NOWAIT | M_ZERO);
	if (ic->ic_aid_bitmap == NULL) {
		/* XXX no way to recover */
		printf("%s: no memory for AID bitmap!\n", __func__);
		ic->ic_max_aid = 0;
	}
}

void
ieee80211_node_lateattach(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;

	/* NB: allocator is responsible for initializing the structure */
	ni = (*ic->ic_node_alloc)(ic);
	KASSERT(ni != NULL, ("unable to setup inital BSS node"));
	/*
	 * Setup "global settings" in the bss node so that
	 * each new station automatically inherits them.
	 */
	ni->ni_chan = IEEE80211_CHAN_ANYC;
	ni->ni_authmode = IEEE80211_AUTH_OPEN;
	ni->ni_txpower = IEEE80211_TXPOWER_MAX;
	ic->ic_bss = ni;
}

void
ieee80211_node_detach(struct ieee80211com *ic)
{

	if (ic->ic_bss != NULL)
		(*ic->ic_node_free)(ic, ic->ic_bss);
	ieee80211_free_allnodes(ic);
	IEEE80211_NODE_LOCK_DESTROY(ic);
	if (ic->ic_aid_bitmap != NULL)
		FREE(ic->ic_aid_bitmap, M_DEVBUF);
}

/* 
 * Port authorize/unauthorize interfaces for use by an authenticator.
 */

void
ieee80211_node_authorize(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	ni->ni_flags |= IEEE80211_NODE_AUTH;
}
EXPORT_SYMBOL(ieee80211_node_authorize);

void
ieee80211_node_unauthorize(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	ni->ni_flags &= ~IEEE80211_NODE_AUTH;
}
EXPORT_SYMBOL(ieee80211_node_unauthorize);

/*
 * Construct and a unicast key for the node and allocate
 * a key index from the driver if needed.  This is typically
 * called by the 802.1x authenticator as part of arranging
 * key state after a station has been authorized.
 */
int
ieee80211_node_newkey(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	/*
	 * Construct a key.  We create a maximally-sized key;
	 * the caller can shrink as desired.
	 *
	 * XXX just use random data for now; probably want something better
	 * XXX cipher suites
	 */
	ni->ni_ucastkey.wk_len = sizeof(ni->ni_ucastkey.wk_key);
	get_random_bytes(ni->ni_ucastkey.wk_key, ni->ni_ucastkey.wk_len);
	/*
	 * Ask the driver for a key index if we don't have one.
	 */
	if (ni->ni_ucastkeyix == IEEE80211_KEYIX_NONE) {
		ni->ni_ucastkeyix = (*ic->ic_key_alloc)(ic);
		if (ni->ni_ucastkeyix == IEEE80211_KEYIX_NONE) {
			 /*
			 * Driver has no room or support for this algorithm;
			 * fallback to doing crypto in the host.
			 */
			/* XXX */
			return 0;
		}
	}
	return 1;
}
EXPORT_SYMBOL(ieee80211_node_newkey);

/*
 * Remove the unicast key for the specified node.
 */
int
ieee80211_node_delkey(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (ni->ni_ucastkeyix != IEEE80211_KEYIX_NONE) {
		if (!(*ic->ic_key_delete)(ic, ni->ni_ucastkeyix))
			return 0;
		ni->ni_ucastkeyix = IEEE80211_KEYIX_NONE;
	}
	return 1;
}
EXPORT_SYMBOL(ieee80211_node_delkey);

/*
 * Install the unicast key as ready for use.
 */
int
ieee80211_node_setkey(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (ni->ni_ucastkeyix == IEEE80211_KEYIX_NONE) {
		/* XXX nothing allocated */
		return 0;
	}
	return (*ic->ic_key_set)(ic, ni->ni_ucastkeyix,
		&ni->ni_ucastkey, ni->ni_macaddr);
}
EXPORT_SYMBOL(ieee80211_node_setkey);

/*
 * AP scanning support.
 */

/*
 * Initialize the active channel set based on the set
 * of available channels and the current PHY mode.
 */
static void
ieee80211_reset_scan(struct ieee80211com *ic)
{

	memcpy(ic->ic_chan_scan, ic->ic_chan_active,
		sizeof(ic->ic_chan_active));
	/* NB: hack, setup so next_scan starts with the first channel */
	if (ic->ic_bss->ni_chan == IEEE80211_CHAN_ANYC)
		ic->ic_bss->ni_chan = &ic->ic_channels[IEEE80211_CHAN_MAX];
}

/*
 * Begin an active scan.
 */
void
ieee80211_begin_scan(struct ieee80211com *ic)
{

	/*
	 * In all but hostap mode scanning starts off in
	 * an active mode before switching to passive.
	 */
	if (ic->ic_opmode != IEEE80211_M_HOSTAP) {
		ic->ic_flags |= IEEE80211_F_ASCAN;
		ic->ic_stats.is_scan_active++;
	} else
		ic->ic_stats.is_scan_passive++;
	IEEE80211_DPRINTF(ic, IEEE80211_MSG_SCAN, ("begin %s scan\n",
		(ic->ic_flags & IEEE80211_F_ASCAN) ?  "active" : "passive"));
	/*
	 * Clear scan state and flush any previously seen
	 * AP's.  Note that the latter assumes we don't act
	 * as both an AP and a station, otherwise we'll
	 * potentially flush state of stations associated
	 * with us.
	 */
	ieee80211_reset_scan(ic);
	ieee80211_free_allnodes(ic);

	/* Scan the next channel. */
	ieee80211_next_scan(ic);
}

/*
 * Switch to the next channel marked for scanning.
 */
void
ieee80211_next_scan(struct ieee80211com *ic)
{
	struct ieee80211_channel *chan;

	chan = ic->ic_bss->ni_chan;
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
		if (chan == ic->ic_bss->ni_chan) {
			ieee80211_end_scan(ic);
			return;
		}
	}
	clrbit(ic->ic_chan_scan, ieee80211_chan2ieee(ic, chan));
	IEEE80211_DPRINTF(ic, IEEE80211_MSG_SCAN,
	    ("%s: chan %d->%d\n", __func__,
	    ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan),
	    ieee80211_chan2ieee(ic, chan)));
	ic->ic_bss->ni_chan = chan;
	ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
}
EXPORT_SYMBOL(ieee80211_next_scan);

void
ieee80211_create_ibss(struct ieee80211com* ic, struct ieee80211_channel *chan)
{
	struct ieee80211_node *ni;

	ni = ic->ic_bss;
	IEEE80211_DPRINTF(ic, IEEE80211_MSG_SCAN, ("creating ibss\n"));
	ic->ic_flags |= IEEE80211_F_SIBSS;
	ni->ni_chan = chan;
	ni->ni_rates = ic->ic_sup_rates[ieee80211_chan2mode(ic, ni->ni_chan)];
	IEEE80211_ADDR_COPY(ni->ni_macaddr, ic->ic_myaddr);
	IEEE80211_ADDR_COPY(ni->ni_bssid, ic->ic_myaddr);
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		ni->ni_bssid[0] |= 0x02;	/* local bit for IBSS */
	ni->ni_esslen = ic->ic_des_esslen;
	memcpy(ni->ni_essid, ic->ic_des_essid, ni->ni_esslen);
	ni->ni_rssi = 0;
	ni->ni_rstamp = 0;
	memset(ni->ni_tstamp, 0, sizeof(ni->ni_tstamp));
	ni->ni_intval = ic->ic_lintval;
	ni->ni_capinfo = IEEE80211_CAPINFO_IBSS;
	if (ic->ic_flags & IEEE80211_F_PRIVACY)
		ni->ni_capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if (ic->ic_phytype == IEEE80211_T_FH) {
		ni->ni_fhdwell = 200;	/* XXX */
		ni->ni_fhindex = 1;
	}
	ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
}

static int
ieee80211_match_bss(struct ieee80211com *ic, struct ieee80211_node *ni)
{
        u_int8_t rate;
        int fail;

	fail = 0;
	if (isclr(ic->ic_chan_active, ieee80211_chan2ieee(ic, ni->ni_chan)))
		fail |= 0x01;
	if (ic->ic_des_chan != IEEE80211_CHAN_ANYC &&
	    ni->ni_chan != ic->ic_des_chan)
		fail |= 0x01;
	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		if ((ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) == 0)
			fail |= 0x02;
	} else {
		if ((ni->ni_capinfo & IEEE80211_CAPINFO_ESS) == 0)
			fail |= 0x02;
	}
	if (ic->ic_flags & IEEE80211_F_PRIVACY) {
		if ((ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) == 0)
			fail |= 0x04;
	} else {
		/* XXX does this mean privacy is supported or required? */
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
#ifdef IEEE80211_DEBUG
	if (ieee80211_msg_debug(ic)) {
		printf(" %c %s", fail ? '-' : '+',
		    ether_sprintf(ni->ni_macaddr));
		printf(" %s%c", ether_sprintf(ni->ni_bssid),
		    fail & 0x20 ? '!' : ' ');
		printf(" %3d%c", ieee80211_chan2ieee(ic, ni->ni_chan),
			fail & 0x01 ? '!' : ' ');
		printf(" %+4d", ni->ni_rssi);
		printf(" %2dM%c", (rate & IEEE80211_RATE_VAL) / 2,
		    fail & 0x08 ? '!' : ' ');
		printf(" %4s%c",
		    (ni->ni_capinfo & IEEE80211_CAPINFO_ESS) ? "ess" :
		    (ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) ? "ibss" :
		    "????",
		    fail & 0x02 ? '!' : ' ');
		printf(" %3s%c ",
		    (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) ?
		    "wep" : "no",
		    fail & 0x04 ? '!' : ' ');
		ieee80211_print_essid(ni->ni_essid, ni->ni_esslen);
		printf("%s\n", fail & 0x10 ? "!" : "");
	}
#endif
	return fail;
}

/*
 * Complete a scan of potential channels.
 */
void
ieee80211_end_scan(struct ieee80211com *ic)
{
	struct ieee80211_node *ni, *nextbs, *selbs;
	int i, fail;

	ic->ic_flags &= ~IEEE80211_F_ASCAN;
	ni = TAILQ_FIRST(&ic->ic_node);

	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/* XXX off stack? */
		u_char occupied[roundup(IEEE80211_CHAN_MAX, NBBY)];
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
		ieee80211_create_ibss(ic, &ic->ic_channels[i]);
		return;
	}
	if (ni == NULL) {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_SCAN,
			("%s: no scan candidate\n", __func__));
  notfound:
		if (ic->ic_opmode == IEEE80211_M_IBSS &&
		    (ic->ic_flags & IEEE80211_F_IBSSON) &&
		    ic->ic_des_esslen != 0) {
			ieee80211_create_ibss(ic, ic->ic_ibss_chan);
			return;
		}
		/*
		 * Reset the list of channels to scan and start again.
		 */
		ieee80211_reset_scan(ic);
		ieee80211_next_scan(ic);
		return;
	}
	selbs = NULL;
	IEEE80211_DPRINTF(ic, IEEE80211_MSG_DEBUG,
		("\tmacaddr          bssid         chan  rssi rate flag  wep  essid\n"));
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
		if (ieee80211_match_bss(ic, ni) == 0) {
			if (selbs == NULL)
				selbs = ni;
			else if (ni->ni_rssi > selbs->ni_rssi) {
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
	(*ic->ic_node_copy)(ic, ic->ic_bss, selbs);
	/*
	 * Set the erp state (mostly the slot time) to deal with
	 * the auto-select case; this should be redundant if the
	 * mode is locked.
	 */ 
	ieee80211_reset_erp(ic, ieee80211_chan2mode(ic, ic->ic_bss->ni_chan));
	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		ieee80211_fix_rate(ic, ic->ic_bss, IEEE80211_F_DOFRATE |
		    IEEE80211_F_DONEGO | IEEE80211_F_DODEL);
		if (ic->ic_bss->ni_rates.rs_nrates == 0) {
			selbs->ni_fails++;
			ieee80211_unref_node(&selbs);
			goto notfound;
		}
		ieee80211_unref_node(&selbs);
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	} else {
		ieee80211_unref_node(&selbs);
		ieee80211_new_state(ic, IEEE80211_S_AUTH, -1);
	}
}

static struct ieee80211_node *
ieee80211_node_alloc(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;
	MALLOC(ni, struct ieee80211_node *, sizeof(struct ieee80211_node),
		M_80211_NODE, M_NOWAIT | M_ZERO);
	return ni;
}

static void
ieee80211_node_free(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	if (ni->ni_challenge != NULL) {
		FREE(ni->ni_challenge, M_DEVBUF);
		ni->ni_challenge = NULL;
	}
	FREE(ni, M_80211_NODE);
}

static void
ieee80211_node_copy(struct ieee80211com *ic,
	struct ieee80211_node *dst, const struct ieee80211_node *src)
{
	*dst = *src;
	dst->ni_challenge = NULL;
}

static u_int8_t
ieee80211_node_getrssi(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	return ni->ni_rssi;
}

static void
ieee80211_setup_node(struct ieee80211com *ic,
	struct ieee80211_node *ni, u_int8_t *macaddr)
{
	int hash;

	IEEE80211_NODE_LOCK_ASSERT(ic);

	IEEE80211_DPRINTF(ic, IEEE80211_MSG_NODE,
		("%s %s\n", __func__, ether_sprintf(macaddr)));
	IEEE80211_ADDR_COPY(ni->ni_macaddr, macaddr);
	hash = IEEE80211_NODE_HASH(macaddr);
	skb_queue_head_init(&ni->ni_savedq);
	ieee80211_node_initref(ni);		/* mark referenced */
	ni->ni_ucastkeyix = IEEE80211_KEYIX_NONE;
	ni->ni_inact = IEEE80211_INACT_INIT;

	IEEE80211_NODE_LOCK_BH(ic);
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
	IEEE80211_NODE_UNLOCK_BH(ic);
}

struct ieee80211_node *
ieee80211_alloc_node(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni = (*ic->ic_node_alloc)(ic);
	if (ni != NULL)
		ieee80211_setup_node(ic, ni, macaddr);
	else
		ic->ic_stats.is_rx_nodealloc++;
	return ni;
}

struct ieee80211_node *
ieee80211_dup_bss(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni = (*ic->ic_node_alloc)(ic);
	if (ni != NULL) {
		ieee80211_setup_node(ic, ni, macaddr);
		/*
		 * Inherit from ic_bss.
		 */
		ni->ni_authmode = ic->ic_bss->ni_authmode;
		ni->ni_txpower = ic->ic_bss->ni_txpower;
		ni->ni_vlan = ic->ic_bss->ni_vlan;	/* XXX?? */
		IEEE80211_ADDR_COPY(ni->ni_bssid, ic->ic_bss->ni_bssid);
		ni->ni_chan = ic->ic_bss->ni_chan;
	} else
		ic->ic_stats.is_rx_nodealloc++;
	return ni;
}

struct ieee80211_node *
_ieee80211_find_node(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;
	int hash;

	IEEE80211_NODE_LOCK_ASSERT(ic);

	hash = IEEE80211_NODE_HASH(macaddr);
	LIST_FOREACH(ni, &ic->ic_hash[hash], ni_hash) {
		if (IEEE80211_ADDR_EQ(ni->ni_macaddr, macaddr)) {
			ieee80211_node_incref(ni); /* mark referenced */
			return ni;
		}
	}
	return NULL;
}

struct ieee80211_node *
ieee80211_find_node(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;

	IEEE80211_NODE_LOCK(ic);
	ni = _ieee80211_find_node(ic, macaddr);
	IEEE80211_NODE_UNLOCK(ic);
	return ni;
}
EXPORT_SYMBOL(ieee80211_find_node);

/*
 * Return a reference to the appropriate node for sending
 * a data frame.  This handles node discovery in adhoc networks.
 */
struct ieee80211_node *
ieee80211_find_txnode(struct ieee80211com *ic, u_int8_t *macaddr)
{
	struct ieee80211_node *ni;

	/*
	 * The destination address should be in the node table
	 * unless we are operating in station mode or this is a
	 * multicast/broadcast frame.
	 */
	if (ic->ic_opmode == IEEE80211_M_STA || IEEE80211_IS_MULTICAST(macaddr))
		return ic->ic_bss;

	/* XXX can't hold lock across dup_bss 'cuz of recursive locking */
	IEEE80211_NODE_LOCK(ic);
	ni = _ieee80211_find_node(ic, macaddr);
	IEEE80211_NODE_UNLOCK(ic);
	if (ni == NULL &&
	    (ic->ic_opmode == IEEE80211_M_IBSS ||
	     ic->ic_opmode == IEEE80211_M_AHDEMO)) {
		/*
		 * Fake up a node; this handles node
		 * discovery in adhoc mode.
		 */
		ni = ieee80211_dup_bss(ic, macaddr);
		/* XXX not sure how 802.1x works w/ IBSS */
		ieee80211_node_authorize(ic, ni);
	}
	return ni;
}

/*
 * Like find but search based on the channel too.
 */
struct ieee80211_node *
ieee80211_lookup_node(struct ieee80211com *ic,
	u_int8_t *macaddr, struct ieee80211_channel *chan)
{
	struct ieee80211_node *ni;
	int hash;

	hash = IEEE80211_NODE_HASH(macaddr);
	IEEE80211_NODE_LOCK(ic);
	LIST_FOREACH(ni, &ic->ic_hash[hash], ni_hash) {
		if (IEEE80211_ADDR_EQ(ni->ni_macaddr, macaddr) &&
		    ni->ni_chan == chan) {
			ieee80211_node_incref(ni);/* mark referenced */
			break;
		}
	}
	IEEE80211_NODE_UNLOCK(ic);
	return ni;
}

static void
_ieee80211_free_node(struct ieee80211com *ic, struct ieee80211_node *ni)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	int i;

	KASSERT(ni != ic->ic_bss, ("freeing bss node"));

	IEEE80211_DPRINTF(ic, IEEE80211_MSG_NODE,
		("%s %s\n", __func__, ether_sprintf(ni->ni_macaddr)));
	IEEE80211_AID_CLR(ni->ni_associd, ic->ic_aid_bitmap);
	TAILQ_REMOVE(&ic->ic_node, ni, ni_list);
	if (TAILQ_EMPTY(&ic->ic_node))
		ic->ic_inact_timer = 0;
	LIST_REMOVE(ni, ni_hash);
	if (_IF_QLEN(&ni->ni_savedq) != 0) {
		/*
		 * Drain power save queue.
		 */
		IF_DRAIN(&ni->ni_savedq);
		if (ic->ic_set_tim)
			ic->ic_set_tim(ic, ni->ni_associd, 0);
	}
	for (i = 0; i < N(ni->ni_rxfrag); i++)
		if (ni->ni_rxfrag[i] != NULL)
			kfree_skb(ni->ni_rxfrag[i]);
	if (ni->ni_ucastkeyix != IEEE80211_KEYIX_NONE)
		(*ic->ic_key_delete)(ic, ni->ni_ucastkeyix);
	(*ic->ic_node_free)(ic, ni);
#undef N
}

void
ieee80211_free_node(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	KASSERT(ni != ic->ic_bss, ("freeing ic_bss"));

	IEEE80211_DPRINTF(ic, IEEE80211_MSG_NODE,
		("%s %s refcnt %d\n", __func__,
		 ether_sprintf(ni->ni_macaddr), ieee80211_node_refcnt(ni)));
	if (ieee80211_node_dectestref(ni)) {
		IEEE80211_NODE_LOCK_BH(ic);
		_ieee80211_free_node(ic, ni);
		IEEE80211_NODE_UNLOCK_BH(ic);
	}
}
EXPORT_SYMBOL(ieee80211_free_node);

void
ieee80211_free_allnodes(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;

	IEEE80211_DPRINTF(ic, IEEE80211_MSG_NODE, ("free all nodes\n"));
	IEEE80211_NODE_LOCK_BH(ic);
	while ((ni = TAILQ_FIRST(&ic->ic_node)) != NULL) {
		if (ni->ni_associd != 0) {
			if (ic->ic_ec != NULL)
				(*ic->ic_node_leave)(ic, ni);
			IEEE80211_AID_CLR(ni->ni_associd, ic->ic_aid_bitmap);
		}
		_ieee80211_free_node(ic, ni);
	}
	ieee80211_reset_erp(ic, ic->ic_curmode);
	IEEE80211_NODE_UNLOCK_BH(ic);
}

/*
 * Timeout inactive nodes.  Note that we cannot hold the node
 * lock while sending a frame as this would lead to a LOR.
 * Instead we use a generation number to mark nodes that we've
 * scanned and drop the lock and restart a scan if we have to
 * time out a node.  Since we are single-threaded by virtue of
 * controlling the inactivity timer we can be sure this will
 * process each node only once.
 */
void
ieee80211_timeout_nodes(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;
	u_int gen = ic->ic_scangen++;		/* NB: ok 'cuz single-threaded*/

restart:
	IEEE80211_NODE_LOCK(ic);
	TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
		if (ni->ni_scangen == gen)	/* previously handled */
			continue;
		ni->ni_scangen = gen;
		/*
		 * Free fragment skb if not needed anymore
		 * (last fragment older than 1s).
		 * XXX doesn't belong here
		 */
		if (ni->ni_rxfrag[0] && jiffies > ni->ni_rxfragstamp + HZ) {
			kfree_skb(ni->ni_rxfrag[0]);
			ni->ni_rxfrag[0] = NULL;
		}
		if (--ni->ni_inact <= 0) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_NODE,
			    ("station %s timed out due to inactivity\n",
			    ether_sprintf(ni->ni_macaddr)));
			/*
			 * Send a deauthenticate frame.
			 *
			 * Drop the node lock before sending the
			 * deauthentication frame in case the driver takes     
			 * a lock, as this will result in a LOR between the     
			 * node lock and the driver lock.
			 */
			IEEE80211_NODE_UNLOCK(ic);
			IEEE80211_SEND_MGMT(ic, ni,
			    IEEE80211_FC0_SUBTYPE_DEAUTH,
			    IEEE80211_REASON_AUTH_EXPIRE);
			ieee80211_node_leave(ic, ni);
			ic->ic_stats.is_node_timeout++;
			goto restart;
		}
	}
	if (!TAILQ_EMPTY(&ic->ic_node))
		ic->ic_inact_timer = IEEE80211_INACT_WAIT;
	IEEE80211_NODE_UNLOCK(ic);
}

void
ieee80211_iterate_nodes(struct ieee80211com *ic, ieee80211_iter_func *f, void *arg)
{
	struct ieee80211_node *ni;

	IEEE80211_NODE_LOCK(ic);
	TAILQ_FOREACH(ni, &ic->ic_node, ni_list)
		(*f)(arg, ni);
	IEEE80211_NODE_UNLOCK(ic);
}
EXPORT_SYMBOL(ieee80211_iterate_nodes);

void
ieee80211_dump_node(struct ieee80211_node *ni)
{
	printf("0x%p: mac %s refcnt %d\n", ni,
		ether_sprintf(ni->ni_macaddr), ieee80211_node_refcnt(ni));
	printf("\tscangen %u fragno %u authmode %u flags 0x%x\n",
		ni->ni_scangen, ni->ni_fragno, ni->ni_authmode, ni->ni_flags);
	printf("\tassocid 0x%x txpower %u vlan %u\n",
		ni->ni_associd, ni->ni_txpower, ni->ni_vlan);
	printf("\ttxseq %u rxseq %u rxfragstamp %u\n",
		ni->ni_txseq, ni->ni_rxseq, ni->ni_rxfragstamp);
	printf("\trstamp %u rssi %u intval %u capinfo 0x%x\n",
		ni->ni_rstamp, ni->ni_rssi, ni->ni_intval, ni->ni_capinfo);
	printf("\tbssid %s essid \"%.*s\" channel %u:0x%x\n",
		ether_sprintf(ni->ni_bssid),
		ni->ni_esslen, ni->ni_essid,
		ni->ni_chan->ic_freq, ni->ni_chan->ic_flags);
	printf("\tfails %u inact %u txrate %u\n",
		ni->ni_fails, ni->ni_inact, ni->ni_txrate);
}

void
ieee80211_dump_nodes(struct ieee80211com *ic)
{
	struct ieee80211_node *ni;

	IEEE80211_NODE_LOCK(ic);
	TAILQ_FOREACH(ni, &ic->ic_node, ni_list)
		ieee80211_dump_node(ni);
	IEEE80211_NODE_UNLOCK(ic);
}

/*
 * Handle bookkeeping for station deauthentication/disassociation
 * when operating as an ap.
 */
void
ieee80211_node_leave(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	KASSERT(ic->ic_opmode == IEEE80211_M_HOSTAP,
		("not in ap mode, mode %u", ic->ic_opmode));
	/*
	 * If node wasn't previously associated all
	 * we need to do is reclaim the reference.
	 */
	if (ni->ni_associd == 0)
		goto done;
	/*
	 * Tell the authenticator the station is leaving.
	 * Note that we must do this before yanking the
	 * association id as the authenticator uses the
	 * associd to locate it's state block.
	 */
	if (ic->ic_ec != NULL)
		(*ic->ic_node_leave)(ic, ni);
	IEEE80211_AID_CLR(ni->ni_associd, ic->ic_aid_bitmap);
	ni->ni_associd = 0;

	/*
	 * If a long slot station do the slot time bookkeeping.
	 */
	if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME) == 0) {
		KASSERT(ic->ic_longslotsta > 0,
		    ("bogus long slot station count %d", ic->ic_longslotsta));
		ic->ic_longslotsta--;
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
			("long slot time station %s leaves, count now %d\n",
			ether_sprintf(ni->ni_macaddr), ic->ic_longslotsta));
		if (ic->ic_longslotsta == 0 &&
		    ic->ic_curmode == IEEE80211_MODE_11G) {
			/*
			 * Re-enable use of short slot time if supported
			 * and not operating in IBSS mode (per spec).
			 */
			if ((ic->ic_caps & IEEE80211_C_SHSLOT) &&
			    ic->ic_opmode != IEEE80211_M_IBSS) {
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
					("re-enable use of short slot time\n"));
				ic->ic_flags |= IEEE80211_F_SHSLOT;
			}
		}
	}
	/*
	 * If a non-ERP station do the protection-related bookkeeping.
	 */
	if ((ni->ni_flags & IEEE80211_NODE_ERP) == 0) {
		KASSERT(ic->ic_nonerpsta > 0,
		    ("bogus non-ERP station count %d", ic->ic_nonerpsta));
		ic->ic_nonerpsta--;
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
			("non-ERP station %s leaves, count now %d\n",
			ether_sprintf(ni->ni_macaddr), ic->ic_nonerpsta));
		if (ic->ic_nonerpsta == 0) {
			IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
				("disable use of protection\n"));
			ic->ic_flags &= ~IEEE80211_F_USEPROT;
			/* XXX verify mode? */
			if (ic->ic_caps & IEEE80211_C_SHPREAMBLE) {
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
					("re-enable use of short preamble\n"));
				ic->ic_flags |= IEEE80211_F_SHPREAMBLE;
				ic->ic_flags &= ~IEEE80211_F_USEBARKER;
			}
		}
	}
done:
	ieee80211_free_node(ic, ni);
}
