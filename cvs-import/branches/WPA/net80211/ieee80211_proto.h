/*	$NetBSD: ieee80211_proto.h,v 1.3 2003/10/13 04:23:56 dyoung Exp $	*/
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
 *
 * $FreeBSD: src/sys/net80211/ieee80211_proto.h,v 1.4 2003/08/19 22:17:03 sam Exp $
 */
#ifndef _NET80211_IEEE80211_PROTO_H_
#define _NET80211_IEEE80211_PROTO_H_

/*
 * 802.11 protocol implementation definitions.
 */

enum ieee80211_state {
	IEEE80211_S_INIT	= 0,	/* default state */
	IEEE80211_S_SCAN	= 1,	/* scanning */
	IEEE80211_S_AUTH	= 2,	/* try to authenticate */
	IEEE80211_S_ASSOC	= 3,	/* try to assoc */
	IEEE80211_S_RUN		= 4,	/* associated */
};
#define	IEEE80211_S_MAX		(IEEE80211_S_RUN+1)

#define	IEEE80211_SEND_MGMT(_ic,_ni,_type,_arg) \
	((*(_ic)->ic_send_mgmt)(_ic, _ni, _type, _arg))

/*
 * Transmitted frames have the following information
 * held in the sk_buff control buffer.  This is used to
 * communicate various inter-procedural state that needs
 * to be associated with the frame for the duration of
 * it's existence.
 */
struct ieee80211_cb {
	struct ieee80211_node	*ni;
	u_int8_t		flags;
};

extern	const char *ieee80211_mgt_subtype_name[];

extern	void ieee80211_proto_attach(struct ieee80211com *);
extern	void ieee80211_proto_detach(struct ieee80211com *);

struct ieee80211_node;
extern	void ieee80211_input(struct ieee80211com *, struct sk_buff *,
		struct ieee80211_node *, int, u_int32_t);
extern	void ieee80211_recv_mgmt(struct ieee80211com *, struct sk_buff *,
		struct ieee80211_node *, int, int, u_int32_t);
extern	int ieee80211_send_mgmt(struct ieee80211com *, struct ieee80211_node *,
		int, int);
extern	void ieee80211_pwrsave(struct ieee80211com *, struct ieee80211_node *, 
		struct sk_buff *);
extern	struct sk_buff *ieee80211_encap(struct ieee80211com *, struct sk_buff *,
		struct ieee80211_node **);

/* flags for ieee80211_fix_rate() */
#define	IEEE80211_F_DOSORT	0x00000001	/* sort rate list */
#define	IEEE80211_F_DOFRATE	0x00000002	/* use fixed rate */
#define	IEEE80211_F_DONEGO	0x00000004	/* calc negotiated rate */
#define	IEEE80211_F_DODEL	0x00000008	/* delete ignore rate */
extern	int ieee80211_fix_rate(struct ieee80211com *,
		struct ieee80211_node *, int);

extern	int ieee80211_iserp_rateset(struct ieee80211com *,
		struct ieee80211_rateset *);
#define	ieee80211_new_state(_ic, _nstate, _arg) \
	(((_ic)->ic_newstate)((_ic), (_nstate), (_arg)))
extern	void ieee80211_print_essid(u_int8_t *, int);
extern	void ieee80211_dump_pkt(u_int8_t *, int, int, int);

extern	const char *ieee80211_state_name[IEEE80211_S_MAX];

/*
 * Beacon frames constructed by ieee80211_beacon_alloc
 * have the following structure filled in so drivers
 * can update the frame later w/ minimal overhead.
 */
struct ieee80211_beacon_offsets {
	u_int16_t	*bo_caps;	/* capabilities */
	u_int8_t	*bo_tim;	/* start of atim/dtim */
	u_int8_t	*bo_xrates;	/* start of extended rates */
	u_int16_t	bo_tim_len;	/* atim/dtim length in bytes */
	u_int16_t	bo_xrates_len;	/* xrates length in bytes */
};
extern	struct sk_buff *ieee80211_beacon_alloc(struct ieee80211com *,
		struct ieee80211_node *, struct ieee80211_beacon_offsets *);
extern	int ieee80211_beacon_update(struct ieee80211com *,
		struct ieee80211_node *, struct ieee80211_beacon_offsets *,
		struct sk_buff **);
#endif /* _NET80211_IEEE80211_PROTO_H_ */
