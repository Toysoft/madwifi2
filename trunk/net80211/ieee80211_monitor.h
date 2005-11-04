/*-
 * Copyright (c) 2005 John Bicket
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
 */
#ifndef _NET80211_IEEE80211_MONITOR_H_
#define _NET80211_IEEE80211_MONITOR_H_


#ifndef ARPHRD_IEEE80211_RADIOTAP
#define ARPHRD_IEEE80211_RADIOTAP 803 /* IEEE 802.11 + radiotap header */
#endif /* ARPHRD_IEEE80211_RADIOTAP */

struct ieee80211_phy_params {
	u_int8_t rate0;
	u_int8_t rate1;
	u_int8_t rate2;
	u_int8_t rate3;

	u_int8_t try0;
	u_int8_t try1;
	u_int8_t try2;
	u_int8_t try3;

	u_int8_t power;
	u_int32_t flags;
};

/*
 * Dispatch an skb to monitor-mode vap's.  The skb is assumed
 * to have space at the front to push a wlan_ng_prims2_header.
 */
void	ieee80211_input_monitor(struct ieee80211com *, struct sk_buff *,
		u_int32_t mactime, u_int32_t rssi, u_int32_t signal,
		u_int32_t rate);


void    ieee80211_monitor_encap(struct ieee80211com *ic, struct sk_buff *skb);


#endif /* _NET80211_IEEE80211_MONITOR_H_ */
