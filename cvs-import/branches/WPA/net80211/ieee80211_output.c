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

__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_output.c,v 1.9 2003/10/17 23:15:30 sam Exp $");
__KERNEL_RCSID(0, "$NetBSD: ieee80211_output.c,v 1.9 2003/11/02 00:17:27 dyoung Exp $");

/*
 * IEEE 802.11 output handling.
 */
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/if_vlan.h>

#include "if_llc.h"
#include "if_ethersubr.h"
#include "if_media.h"

#include <net80211/ieee80211_var.h>

/*
 * Send a management frame to the specified node.  The node pointer
 * must have a reference as the pointer will be passed to the driver
 * and potentially held for a long time.  If the frame is successfully
 * dispatched to the driver, then it is responsible for freeing the
 * reference (and potentially free'ing up any associated storage).
 */
static void
ieee80211_mgmt_output(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct sk_buff *skb, int type)
{
	struct ieee80211_frame *wh;
	struct ieee80211_cb *cb = (struct ieee80211_cb *)skb->cb;

	KASSERT(ni != NULL, ("null node"));
	ni->ni_inact = IEEE80211_INACT_AUTH;

	/*
	 * Yech, hack alert!  We want to pass the node down to the
	 * driver's start routine.  If we don't do so then the start
	 * routine must immediately look it up again and that can
	 * cause a lock order reversal if, for example, this frame
	 * is being sent because the station is being timedout and
	 * the frame being sent is a DEAUTH message. 
	 */
	cb->ni = ni;

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT | type;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(u_int16_t *)&wh->i_dur[0] = 0;
	*(u_int16_t *)&wh->i_seq[0] =
	    htole16(ni->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
	ni->ni_txseq++;
	IEEE80211_ADDR_COPY(wh->i_addr1, ni->ni_macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, ic->ic_myaddr);
	IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);

	if ((cb->flags & M_LINK0) != 0 && ni->ni_challenge != NULL) {
		cb->flags &= ~M_LINK0;
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_AUTH,
			("%s: encrypting frame for %s\n",
			__func__, ether_sprintf(wh->i_addr1)));
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
	}

	/* avoid to print too many frames */
	if (ic->ic_opmode == IEEE80211_M_IBSS ||
	    (type & IEEE80211_FC0_SUBTYPE_MASK) !=
	    IEEE80211_FC0_SUBTYPE_PROBE_RESP)
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_DEBUG,
		    ("sending %s to %s on channel %u\n",
		    ieee80211_mgt_subtype_name[
		    (type & IEEE80211_FC0_SUBTYPE_MASK)
		    >> IEEE80211_FC0_SUBTYPE_SHIFT],
		    ether_sprintf(ni->ni_macaddr),
		    ieee80211_chan2ieee(ic, ni->ni_chan)));

	(void) (*ic->ic_mgtstart)(ic, skb);
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
ieee80211_encap(struct ieee80211com *ic, struct sk_buff *skb,
	struct ieee80211_node **pni)
{
	struct ether_header eh;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	struct llc *llc;

	memcpy(&eh, skb->data, sizeof(struct ether_header));
	skb_pull(skb, sizeof(struct ether_header));

	ni = ieee80211_find_txnode(ic, eh.ether_dhost);
	if (ni == NULL) {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_OUTPUT,
			("%s: no node for dst %s, discard frame\n",
			__func__, ether_sprintf(eh.ether_dhost)));
		ic->ic_stats.is_tx_nonode++; 
		goto bad;
	}

	/* 
	 * If node has a vlan tag then all traffic
	 * to it must have a matching tag.
	 */
	if (ni->ni_vlan != 0) {
		if (ic->ic_vlgrp == NULL || !vlan_tx_tag_present(skb)) {
			ni->ni_stats.ns_tx_novlantag++;
			goto bad;
		}
		if (vlan_tx_tag_get(skb) != ni->ni_vlan) {
			ni->ni_stats.ns_tx_vlanmismatch++;
			goto bad;
		}
	}

	llc = (struct llc *) skb_push(skb, sizeof(struct llc));
	llc->llc_dsap = llc->llc_ssap = LLC_SNAP_LSAP;
	llc->llc_control = LLC_UI;
	llc->llc_snap.org_code[0] = 0;
	llc->llc_snap.org_code[1] = 0;
	llc->llc_snap.org_code[2] = 0;
	llc->llc_snap.ether_type = eh.ether_type;

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));;
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA;
	*(u_int16_t *)&wh->i_dur[0] = 0;
	*(u_int16_t *)&wh->i_seq[0] =
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
	if (eh.ether_type != __constant_htons(ETHERTYPE_PAE)) {
		/*
		 * Reset the inactivity timer only for non-PAE traffic
		 * to avoid a problem where the station leaves w/o
		 * notice while we're requesting Identity.  In this
		 * situation the 802.1x state machine will continue
		 * to retransmit the requests because it assumes the
		 * station will be timed out for inactivity, but our
		 * retransmits will reset the inactivity timer.
		 */ 
		ni->ni_inact = IEEE80211_INACT_RUN;
		/* NB: PAE frames have their own encryption policy */
		if (ic->ic_flags & IEEE80211_F_WEPON)
			wh->i_fc[1] |= IEEE80211_FC1_WEP;
	}
	*pni = ni;
	return skb;
bad:
	dev_kfree_skb(skb);
	if (ni && ni != ic->ic_bss)
		ieee80211_free_node(ic, ni);
	*pni = NULL;
	return NULL;
}
EXPORT_SYMBOL(ieee80211_encap);

/*
 * Add a supported rates element id to a frame.
 */
static u_int8_t *
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
static u_int8_t *
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

/*
 * Add an erp element to a frame.
 */
static u_int8_t *
ieee80211_add_erp(u_int8_t *frm, struct ieee80211com *ic)
{
	u_int8_t erp;

	*frm++ = IEEE80211_ELEMID_ERP;
	*frm++ = 1;
	erp = 0;
	if (ic->ic_nonerpsta != 0)
		erp |= IEEE80211_ERP_NON_ERP_PRESENT;
	if (ic->ic_flags & IEEE80211_F_USEPROT)
		erp |= IEEE80211_ERP_USE_PROTECTION;
	if (ic->ic_flags & IEEE80211_F_USEBARKER)
		erp |= IEEE80211_ERP_LONG_PREAMBLE;
	*frm++ = erp;
	return frm;
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
#define	senderr(_x, _v)	do { ic->ic_stats._v++; ret = _x; goto bad; } while (0)
	struct sk_buff *skb;
	u_int8_t *frm;
	enum ieee80211_phymode mode;
	u_int16_t capinfo;
	int has_challenge, is_shared_key, ret, timer, status;

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
		skb = ieee80211_getmgtframe(&frm,
			 2 + ic->ic_des_esslen
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

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
		 *	[tlv] parameter set (FH/DS)
		 *	[tlv] parameter set (IBSS)
		 *	[tlv] extended rate phy (ERP)
		 *	[tlv] extended supported rates
		 */
		skb = ieee80211_getmgtframe(&frm,
			 8 + 2 + 2 + 2
		       + 2 + ni->ni_esslen
		       + 2 + IEEE80211_RATE_SIZE
		       + (ic->ic_phytype == IEEE80211_T_FH ? 7 : 3)
		       + 6
		       + (ic->ic_curmode == IEEE80211_MODE_11G ? 3 : 0)
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		memset(frm, 0, 8);	/* timestamp should be filled later */
		frm += 8;
		*(u_int16_t *)frm = htole16(ic->ic_bss->ni_intval);
		frm += 2;
		if (ic->ic_opmode == IEEE80211_M_IBSS)
			capinfo = IEEE80211_CAPINFO_IBSS;
		else
			capinfo = IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_PRIVACY)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if (ic->ic_flags & IEEE80211_F_SHSLOT)
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		*(u_int16_t *)frm = htole16(capinfo);
		frm += 2;

		frm = ieee80211_add_ssid(frm, ic->ic_bss->ni_essid,
				ic->ic_bss->ni_esslen);
		frm = ieee80211_add_rates(frm, &ic->ic_bss->ni_rates);

		if (ic->ic_phytype == IEEE80211_T_FH) {
                        *frm++ = IEEE80211_ELEMID_FHPARMS;
                        *frm++ = 5;
                        *frm++ = ni->ni_fhdwell & 0x00ff;
                        *frm++ = (ni->ni_fhdwell >> 8) & 0x00ff;
                        *frm++ = IEEE80211_FH_CHANSET(
			    ieee80211_chan2ieee(ic, ni->ni_chan));
                        *frm++ = IEEE80211_FH_CHANPAT(
			    ieee80211_chan2ieee(ic, ni->ni_chan));
                        *frm++ = ni->ni_fhindex;
		} else {
			*frm++ = IEEE80211_ELEMID_DSPARMS;
			*frm++ = 1;
			*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
		}

		if (ic->ic_opmode == IEEE80211_M_IBSS) {
			*frm++ = IEEE80211_ELEMID_IBSSPARMS;
			*frm++ = 2;
			*frm++ = 0; *frm++ = 0;		/* TODO: ATIM window */
		}
		if (ic->ic_curmode == IEEE80211_MODE_11G)
			frm = ieee80211_add_erp(frm, ic);
		frm = ieee80211_add_xrates(frm, &ic->ic_bss->ni_rates);
		skb_trim(skb, frm - skb->data);
		break;

	case IEEE80211_FC0_SUBTYPE_AUTH:
		status = arg >> 16;
		arg &= 0xffff;
		has_challenge = ((arg == IEEE80211_AUTH_SHARED_CHALLENGE ||
		    arg == IEEE80211_AUTH_SHARED_RESPONSE) &&
		    ni->ni_challenge != NULL);

		/*
		 * Deduce whether we're doing open authentication or
		 * shared key authentication.  We do the latter if
		 * we're in the middle of a shared key authentication
		 * handshake or if we're initiating an authentication
		 * request and configured to use shared key.
		 */
		is_shared_key = has_challenge ||
		     arg >= IEEE80211_AUTH_SHARED_RESPONSE ||
		     (arg == IEEE80211_AUTH_SHARED_REQUEST &&
		      ic->ic_bss->ni_authmode == IEEE80211_AUTH_SHARED);

		skb = ieee80211_getmgtframe(&frm,
			  3 * sizeof(u_int16_t)
			+ (has_challenge && status == IEEE80211_STATUS_SUCCESS ?
				sizeof(u_int16_t)+IEEE80211_CHALLENGE_LEN : 0));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		((u_int16_t *)frm)[0] =
		    (is_shared_key) ? htole16(IEEE80211_AUTH_ALG_SHARED)
		                    : htole16(IEEE80211_AUTH_ALG_OPEN);
		((u_int16_t *)frm)[1] = htole16(arg);	/* sequence number */
		((u_int16_t *)frm)[2] = htole16(status);/* status */

		if (has_challenge && status == IEEE80211_STATUS_SUCCESS) {
			((u_int16_t *)frm)[3] =
			    htole16((IEEE80211_CHALLENGE_LEN << 8) |
			    IEEE80211_ELEMID_CHALLENGE);
			memcpy(&((u_int16_t *)frm)[4], ni->ni_challenge,
			    IEEE80211_CHALLENGE_LEN);
			if (arg == IEEE80211_AUTH_SHARED_RESPONSE) {
				struct ieee80211_cb *cb =
					(struct ieee80211_cb *)skb->cb;
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_AUTH,
				    ("%s: request encrypt frame\n", __func__));
				cb->flags |= M_LINK0; /* WEP-encrypt, please */
			}
		}
		/*
		 * When 802.1x is not in use mark the port
		 * authorized at this point so traffic can flow.
		 */
		if (ic->ic_opmode == IEEE80211_M_HOSTAP &&
		    status == IEEE80211_STATUS_SUCCESS &&
		    ni->ni_authmode != IEEE80211_AUTH_8021X)
			ieee80211_node_authorize(ic, ni);
		if (ic->ic_opmode == IEEE80211_M_STA)
			timer = IEEE80211_TRANS_WAIT;
		break;

	case IEEE80211_FC0_SUBTYPE_DEAUTH:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_AUTH,
			("station %s deauthenticate (reason %d)\n",
			ether_sprintf(ni->ni_macaddr), arg));
		skb = ieee80211_getmgtframe(&frm, sizeof(u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);
		*(u_int16_t *)frm = htole16(arg);	/* reason */
		ieee80211_node_unauthorize(ic, ni);	/* port closed */
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
		skb = ieee80211_getmgtframe(&frm,
			 sizeof(capinfo)
		       + sizeof(u_int16_t)
		       + IEEE80211_ADDR_LEN
		       + 2 + ni->ni_esslen
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		capinfo = 0;
		if (ic->ic_opmode == IEEE80211_M_IBSS)
			capinfo |= IEEE80211_CAPINFO_IBSS;
		else		/* IEEE80211_M_STA */
			capinfo |= IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_PRIVACY)
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
		*(u_int16_t *)frm = htole16(capinfo);
		frm += 2;

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
		skb = ieee80211_getmgtframe(&frm,
			 sizeof(capinfo)
		       + sizeof(u_int16_t)
		       + sizeof(u_int16_t)
		       + 2 + IEEE80211_RATE_SIZE
		       + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);

		capinfo = IEEE80211_CAPINFO_ESS;
		if (ic->ic_flags & IEEE80211_F_PRIVACY)
			capinfo |= IEEE80211_CAPINFO_PRIVACY;
		if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
		    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
			capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
		if (ic->ic_flags & IEEE80211_F_SHSLOT)
			capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
		*(u_int16_t *)frm = htole16(capinfo);
		frm += 2;

		*(u_int16_t *)frm = htole16(arg);	/* status */
		frm += 2;

		if (arg == IEEE80211_STATUS_SUCCESS)
			*(u_int16_t *)frm = htole16(ni->ni_associd);
		frm += 2;

		frm = ieee80211_add_rates(frm, &ni->ni_rates);
		frm = ieee80211_add_xrates(frm, &ni->ni_rates);
		skb_trim(skb, frm - skb->data);
		break;

	case IEEE80211_FC0_SUBTYPE_DISASSOC:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ASSOC,
			("station %s disassociate (reason %d)\n",
			ether_sprintf(ni->ni_macaddr), arg));
		skb = ieee80211_getmgtframe(&frm, sizeof(u_int16_t));
		if (skb == NULL)
			senderr(ENOMEM, is_tx_nobuf);
		*(u_int16_t *)frm = htole16(arg);	/* reason */
		break;

	default:
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ANY,
			("%s: invalid mgmt frame type %u\n", __func__, type));
		senderr(EINVAL, is_tx_unknownmgt);
		/* NOTREACHED */
	}

	ieee80211_mgmt_output(ic, ni, skb, type);
	if (timer)
		ic->ic_mgt_timer = timer;
	return 0;
bad:
	if (ni != ic->ic_bss)		/* remove ref we added */
		ieee80211_free_node(ic, ni);
	return ret;
#undef senderr
}

/*
 * Allocate a beacon frame and fillin the appropriate bits.
 */
struct sk_buff *
ieee80211_beacon_alloc(struct ieee80211com *ic, struct ieee80211_node *ni,
	struct ieee80211_beacon_offsets *bo)
{
	struct net_device *dev = ic->ic_dev;
	struct ieee80211_frame *wh;
	struct sk_buff *skb;
	int pktlen;
	u_int8_t *frm, *efrm;
	u_int16_t capinfo;
	struct ieee80211_rateset *rs;

	/*
	 * beacon frame format
	 *	[8] time stamp
	 *	[2] beacon interval
	 *	[2] cabability information
	 *	[tlv] ssid
	 *	[tlv] supported rates
	 *	[3] parameter set (DS)
	 *	[tlv] parameter set (IBSS/TIM)
	 *	[tlv] extended rate phy (ERP)
	 *	[tlv] extended supported rates
	 * XXX WME, WPA, etc.
	 * XXX Vendor-specific OIDs (e.g. Atheros)
	 */
	rs = &ni->ni_rates;
	pktlen =   8
		 + sizeof(u_int16_t)
		 + sizeof(u_int16_t)
		 + 2 + ni->ni_esslen
		 + 2 + rs->rs_nrates
		 + 6;
	if (ic->ic_curmode != IEEE80211_MODE_FH)
		 pktlen += 3;		/* DS parameter set */
	if (ic->ic_curmode == IEEE80211_MODE_11G)
		 pktlen += 3;		/* ERP information element */
	if (rs->rs_nrates > IEEE80211_RATE_SIZE)
		pktlen += 2;		/* extended rate set */
	/* XXX may be better to just allocate a max-sized buffer */
	skb = ieee80211_getmgtframe(&frm, pktlen);
	if (skb == NULL) {
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ANY,
			("%s: cannot get buf; size %u\n", __func__, pktlen));
		ic->ic_stats.is_tx_nobuf++;
		return NULL;
	}

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_BEACON;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	*(u_int16_t *)wh->i_dur = 0;
	memcpy(wh->i_addr1, dev->broadcast, IEEE80211_ADDR_LEN);
	memcpy(wh->i_addr2, ic->ic_myaddr, IEEE80211_ADDR_LEN);
	memcpy(wh->i_addr3, ni->ni_bssid, IEEE80211_ADDR_LEN);
	*(u_int16_t *)wh->i_seq = 0;

	memset(frm, 0, 8);	/* XXX timestamp is set by hardware */
	frm += 8;
	*(u_int16_t *)frm = cpu_to_le16(ni->ni_intval);
	frm += 2;
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_PRIVACY)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	bo->bo_caps = (u_int16_t *)frm;
	*(u_int16_t *)frm = cpu_to_le16(capinfo);
	frm += 2;
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = ni->ni_esslen;
	memcpy(frm, ni->ni_essid, ni->ni_esslen);
	frm += ni->ni_esslen;
	frm = ieee80211_add_rates(frm, rs);
	if (ic->ic_curmode != IEEE80211_MODE_FH) {
		*frm++ = IEEE80211_ELEMID_DSPARMS;
		*frm++ = 1;
		*frm++ = ieee80211_chan2ieee(ic, ni->ni_chan);
	}
	bo->bo_tim = frm;
	if (ic->ic_opmode == IEEE80211_M_IBSS) {
		*frm++ = IEEE80211_ELEMID_IBSSPARMS;
		*frm++ = 2;
		*frm++ = 0; *frm++ = 0;		/* TODO: ATIM window */
		bo->bo_tim_len = 4;
	} else {
		/* TODO: TIM */
		*frm++ = IEEE80211_ELEMID_TIM;
		*frm++ = 4;	/* length */
		*frm++ = 0;	/* DTIM count */ 
		*frm++ = 1;	/* DTIM period */
		*frm++ = 0;	/* bitmap control */
		*frm++ = 0;	/* Partial Virtual Bitmap (variable length) */
		bo->bo_tim_len = 6;
	}
	if (ic->ic_curmode == IEEE80211_MODE_11G)
		frm = ieee80211_add_erp(frm, ic);
	bo->bo_xrates = frm;
	efrm = ieee80211_add_xrates(frm, rs);
	bo->bo_xrates_len = efrm - frm;
	skb_trim(skb, efrm - skb->data);
	return skb;
}
EXPORT_SYMBOL(ieee80211_beacon_alloc);

/*
 * Update the dynamic parts of a beacon frame based on the current state.
 */
int
ieee80211_beacon_update(struct ieee80211com *ic, struct ieee80211_node *ni,
	struct ieee80211_beacon_offsets *bo, struct sk_buff **skb0)
{
	u_int16_t capinfo;

	/* XXX lock out changes */
	/* XXX only update as needed */
	/* XXX faster to recalculate entirely or just changes? */
	if (ic->ic_opmode == IEEE80211_M_IBSS)
		capinfo = IEEE80211_CAPINFO_IBSS;
	else
		capinfo = IEEE80211_CAPINFO_ESS;
	if (ic->ic_flags & IEEE80211_F_PRIVACY)
		capinfo |= IEEE80211_CAPINFO_PRIVACY;
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
	*bo->bo_caps = htole16(capinfo);

	if (ic->ic_flags & IEEE80211_F_TIMUPDATE) {
		/* 
		 * ATIM/DTIM needs updating.  If it fits in the
		 * current space allocated then just copy in the
		 * new bits.  Otherwise we need to move any extended
		 * rate set the follows and, possibly, allocate a
		 * new buffer if the this current one isn't large
		 * enough.  XXX It may be better to just allocate
		 * a max-sized buffer so we don't re-allocate.
		 */
		/* XXX fillin */
		ic->ic_flags &= ~IEEE80211_F_TIMUPDATE;
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_beacon_update);

/*
 * Save an outbound packet for a node in power-save sleep state.
 * The new packet is placed on the node's saved queue, and the TIM
 * is changed, if necessary.
 */
void
ieee80211_pwrsave(struct ieee80211com *ic, struct ieee80211_node *ni, 
		  struct sk_buff *skb)
{
	if (_IF_QLEN(&ni->ni_savedq) == 0)
		(*ic->ic_set_tim)(ic, ni->ni_associd, 1);
	if (_IF_QLEN(&ni->ni_savedq) >= IEEE80211_PS_MAX_QUEUE) {
		ni->ni_pwrsavedrops++;	/* XXX atomic_inc */
		dev_kfree_skb(skb);
		IEEE80211_DPRINTF(ic, IEEE80211_MSG_ANY,
			("station %s pwr save q overflow of size %d drops %d\n",
			ether_sprintf(ni->ni_macaddr), 
		        IEEE80211_PS_MAX_QUEUE, ni->ni_pwrsavedrops));
	} else {
		struct ieee80211_cb *cb = (struct ieee80211_cb *)skb->cb;
		/*
		 * Similar to ieee80211_mgmt_output, save the node in
		 * the packet for use by the driver start routine.
		 */
		/* XXX do we know we have a reference? */
		cb->ni = ni;
		IF_ENQUEUE(&ni->ni_savedq, skb);
	}
}
