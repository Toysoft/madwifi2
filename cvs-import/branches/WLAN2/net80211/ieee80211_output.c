/*-
 * Copyright (c) 2001 Atsushi Onoe
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

#if 0
#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_output.c,v 1.6 2003/09/14 22:34:24 sam Exp $");

#include "opt_inet.h"

#include <sys/param.h>
#include <sys/systm.h> 
#include <sys/mbuf.h>   
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/endian.h>
#include <sys/errno.h>
#include <sys/bus.h>
#include <sys/proc.h>
#include <sys/sysctl.h>

#include <machine/atomic.h>
 
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_llc.h>

#include <net80211/ieee80211_var.h>

#include <net/bpf.h>

#ifdef INET
#include <netinet/in.h> 
#include <netinet/if_ether.h>
#endif

#else
/*********** linux stuff ************/
#include <net80211/ieee80211_var.h>
/*********** linux stuff ************/
#endif
/*
 * Send a management frame to the specified node.  The node pointer
 * must have a reference as the pointer will be passed to the driver
 * and potentially held for a long time.  If the frame is successfully
 * dispatched to the driver, then it is responsible for freeing the
 * reference (and potentially free'ing up any associated storage).
 */
static int
ieee80211_mgmt_output(struct ieee80211com *ic, struct ieee80211_node *ni,
		      struct sk_buff *skb, int type)
{
	struct ieee80211_frame *wh;

	KASSERT(ni != NULL, ("null node"));
	ni->ni_inact = 0;

#if 0
	/*
	 * Yech, hack alert!  We want to pass the node down to the
	 * driver's start routine.  If we don't do so then the start
	 * routine must immediately look it up again and that can
	 * cause a lock order reversal if, for example, this frame
	 * is being sent because the station is being timedout and
	 * the frame being sent is a DEAUTH message.  We could stick
	 * this in an m_tag and tack that on to the mbuf.  However
	 * that's rather expensive to do for every frame so instead
	 * we stuff it in the rcvif field since outbound frames do
	 * not (presently) use this.
	 */
	M_PREPEND(m, sizeof(struct ieee80211_frame), M_DONTWAIT);
	if (m == NULL)
		return ENOMEM;
	KASSERT(m->m_pkthdr.rcvif == NULL, ("rcvif not null"));
	m->m_pkthdr.rcvif = (void *)ni;

	wh = mtod(m, struct ieee80211_frame *);
#else
	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
#endif
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT | type;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(u_int16_t *)wh->i_dur = 0;
	*(u_int16_t *)wh->i_seq =
	    htole16(ni->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
	ni->ni_txseq++;
	IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, ieee80211_get_address (ic));
	IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);

#ifdef IEEE80211_DEBUG
	if (ieee80211_debug & IEEE80211_DEBUG_IFF &&
	    ieee80211_debug & IEEE80211_DEBUG_LEVEL2) {
		/* avoid to print too many frames */
		if (ic->ic_opmode == IEEE80211_M_IBSS ||
		    (type & IEEE80211_FC0_SUBTYPE_MASK) !=
		    IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			if_printf(ic, "sending %s to %s on channel %u\n",
			    ieee80211_mgt_subtype_name[
			    (type & IEEE80211_FC0_SUBTYPE_MASK)
			    >> IEEE80211_FC0_SUBTYPE_SHIFT],
			    ether_sprintf(ni->ni_macaddr),
			    ieee80211_chan2ieee(ic, ni->ni_chan));
	}
#endif

#if 0
	/* IF_ENQUEUE(&ic->ic_mgtq, m);
	   ifp->if_timer = 1;
	   (*ifp->if_start)(ifp); */
#else
	return (*ic->ic_mgtstart) (ic, skb);
#endif
}

/*
 * Encapsulate an outbound data frame.  The mbuf chain is updated and
 * a reference to the destination node is returned.  If an error is
 * encountered NULL is returned and the node reference will also be NULL.
 * 
 * NB: The caller is responsible for free'ing a returned node reference.
 *     The convention is ic_bss is not reference counted; the caller must
 *     maintain that.
 */
struct sk_buff *
ieee80211_encap(struct ieee80211com *ic, struct sk_buff *skb, struct ieee80211_node **pni)
{
	struct ether_header eh;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni = NULL;
	struct llc *llc;

	memcpy(&eh, skb->data, sizeof(struct ether_header));
	skb_pull(skb, sizeof(struct ether_header));

	if (ic->ic_opmode != IEEE80211_M_STA) {
		ni = ieee80211_find_node(ic, eh.ether_dhost);
		if (ni == NULL) {
			/*
			 * When not in station mode the
			 * destination address should always be
			 * in the node table unless this is a
			 * multicast/broadcast frame.
			 */
			if (!IEEE80211_IS_MULTICAST(eh.ether_dhost)) {
				/* ic->ic_stats.st_tx_nonode++; XXX statistic */
				goto bad;
			}
			ni = ic->ic_bss;
		}
	} else
		ni = ic->ic_bss;
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
		goto bad;
	}
	wh = (struct ieee80211_frame *) skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA;
	*(u_int16_t *)wh->i_dur = 0;
	*(u_int16_t *)wh->i_seq =
	    htole16(ni->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
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
		goto bad;
	}
	*pni = ni;
	return skb;
bad:
	if (skb != NULL)
		dev_kfree_skb (skb);
	if (ni && ni != ic->ic_bss)
		ieee80211_free_node(ic, ni);
	*pni = NULL;
	return NULL;
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

/**
 * This function is always called with the same value for its first two parameters. 
 * We should remove them probably.
 */
static struct sk_buff *
ieee80211_getmbuf(u_int pktlen)
{
	struct sk_buff *skb;
	void *ptr;
	skb = dev_alloc_skb (pktlen + sizeof(struct ieee80211_frame));
	if (skb == NULL)
		return NULL;
	skb_reserve(skb, sizeof(struct ieee80211_frame));
	ptr = skb_put(skb, pktlen);
	KASSERT (ptr == skb->data, ("")); /* I _think_ */
	return skb;
}

/*
 * Send a management frame.  The node is for the destination (or ic_bss
 * when in station mode).  Nodes other than ic_bss have their reference
 * count bumped to reflect our use for an indeterminant time.
 */
int
ieee80211_send_mgmt(struct ieee80211com *ic, struct ieee80211_node *ni,
	int type, int arg)
{
#define	senderr(_x)	do { ret = _x; goto bad; } while (0)
	struct sk_buff *skb;
	u_int8_t *frm;
	u_int16_t *frm16;
	enum ieee80211_phymode mode;
	u_int16_t capinfo;
	int ret, timer;

	KASSERT(ni != NULL, ("null node"));

	/*
	 * Hold a reference on the node so it doesn't go away until after
	 * the xmit is complete all the way in the driver.  On error we
	 * will remove our reference.
	 */
	if (ni != ic->ic_bss)
		ieee80211_ref_node(ni);
	timer = 0;
	switch (type) {
	case IEEE80211_FC0_SUBTYPE_PROBE_REQ:
		/*
		 * prreq frame format
		 *	[tlv] ssid
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 */
		skb = ieee80211_getmbuf(
			 2 + ic->ic_des_esslen
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM);
		frm = skb->data;
		frm = ieee80211_add_ssid(frm, ic->ic_des_essid, ic->ic_des_esslen);
		mode = ieee80211_chan2mode(ic, ni->ni_chan);
		frm = ieee80211_add_rates(frm, &ic->ic_sup_rates[mode]);
		frm = ieee80211_add_xrates(frm, &ic->ic_sup_rates[mode]);
		skb_trim(skb, frm - skb->data);

		timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
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
		skb = ieee80211_getmbuf(
			 8 + 2 + 2 + 2
		       + 2 + ni->ni_esslen
		       + 2 + IEEE80211_RATE_SIZE
		       + 6
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM);
		frm = skb->data;

		memset(frm, 0, 8);	/* timestamp should be filled later */
		frm += 8;
		/* XXX alignment ?*/
		*(u_int16_t *)frm = htole16(ic->ic_bss->ni_intval);
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
		/* XXX alignment ?*/
		*(u_int16_t *)frm = htole16(capinfo);
		frm += 2;

		frm = ieee80211_add_ssid(frm, ic->ic_bss->ni_essid,
				ic->ic_bss->ni_esslen);
		frm = ieee80211_add_rates(frm, &ic->ic_bss->ni_rates);

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
		frm = ieee80211_add_xrates(frm, &ic->ic_bss->ni_rates);
		skb_trim(skb, frm - skb->data);
		break;

	case IEEE80211_FC0_SUBTYPE_AUTH:
		skb = ieee80211_getmbuf (3*sizeof (u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM);
		/* XXX alignment?  */
		frm16 = (u_int16_t *) skb->data;
		/* TODO: shared key auth */
		frm16[0] = htole16(IEEE80211_AUTH_ALG_OPEN);
		frm16[1] = htole16(arg);	/* sequence number */
		frm16[2] = 0;		/* status */
		if (ic->ic_opmode == IEEE80211_M_STA)
			timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_DEAUTH:
		IEEE80211_DPRINTF_IFF (("station %s deauthenticate (reason %d)\n",
			    ether_sprintf(ni->ni_macaddr), arg));
		skb = ieee80211_getmbuf (sizeof (u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM);
		/* XXX alignment? */
		frm16 = (u_int16_t *) skb->data;
		frm16[0] = htole16(arg); /* reason */
		break;

	case IEEE80211_FC0_SUBTYPE_ASSOC_REQ:
	case IEEE80211_FC0_SUBTYPE_REASSOC_REQ:
		/*
		 * asreq frame format
		 *	[2] capability information
		 *	[2] listen interval
		 *	[6*] current AP address (reassoc only)
		 *	[tlv] ssid
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 */
		skb = ieee80211_getmbuf(
			 sizeof(capinfo)
		       + sizeof(u_int16_t)
		       + IEEE80211_ADDR_LEN
		       + 2 + ni->ni_esslen
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM);
		frm = skb->data;

		capinfo = 0;
		if (ic->ic_opmode == IEEE80211_M_IBSS)
			capinfo |= IEEE80211_CAPINFO_IBSS;
		else		/* IEEE80211_M_STA */
			capinfo |= IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_WEPON)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		/*
		 * NB: Some 11a AP's reject the request when
		 *     short premable is set.
		 */
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if (ic->ic_flags & IEEE80211_F_SHSLOT)
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		/* XXX alignment ?*/
		*(u_int16_t *)frm = htole16(capinfo);
		frm += 2;

		/* XXX alignment ?*/
		*(u_int16_t *)frm = htole16(ic->ic_lintval);
		frm += 2;

		if (type == IEEE80211_FC0_SUBTYPE_REASSOC_REQ) {
			IEEE80211_ADDR_COPY(frm, ic->ic_bss->ni_bssid);
			frm += IEEE80211_ADDR_LEN;
		}

		frm = ieee80211_add_ssid(frm, ni->ni_essid, ni->ni_esslen);
		frm = ieee80211_add_rates(frm, &ni->ni_rates);
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		skb_trim(skb, frm - skb->data);

		timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_ASSOC_RESP:
	case IEEE80211_FC0_SUBTYPE_REASSOC_RESP:
		/*
		 * asreq frame format
		 *	[2] capability information
		 *	[2] status
		 *	[2] association ID
		 *	[tlv] supported rates
		 *	[tlv] extended supported rates
		 */
		skb = ieee80211_getmbuf(
			 sizeof(capinfo)
		       + sizeof(u_int16_t)
		       + sizeof(u_int16_t)
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM);
		frm = skb->data;

		capinfo = IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_WEPON)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		/* XXX alignment ?*/
		*(u_int16_t *)frm = htole16(capinfo);
		frm += 2;

		/* XXX alignment ?*/
		*(u_int16_t *)frm = htole16(arg);	/* status */
		frm += 2;

		if (arg == IEEE80211_STATUS_SUCCESS)
			*(u_int16_t *)frm = htole16(ni->ni_associd); /* XXX alignment ?*/
		frm += 2;

		frm = ieee80211_add_rates(frm, &ni->ni_rates);
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		skb_trim(skb, frm - skb->data);
		break;

	case IEEE80211_FC0_SUBTYPE_DISASSOC:
		IEEE80211_DPRINTF_IFF (("station %s disassociate (reason %d)\n",
			    ether_sprintf(ni->ni_macaddr), arg));
		skb = ieee80211_getmbuf(sizeof (u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM);
		/* XXX alignment ?*/
		frm16 = (u_int16_t *)skb->data;
		frm16[0] = htole16(arg);	/* reason */
		break;

	default:
		IEEE80211_DPRINTF(("%s: invalid mgmt frame type %u\n",
			__func__, type));
		senderr(EINVAL);
		/* NOTREACHED */
	}

	ret = ieee80211_mgmt_output(ic, ni, skb, type);
	if (ret == 0) {
		if (timer)
			ic->ic_mgt_timer = timer;
	} else {
bad:
		if (ni != ic->ic_bss)		/* remove ref we added */
			ieee80211_free_node(ic, ni);
	}
	return ret;
#undef senderr
}
