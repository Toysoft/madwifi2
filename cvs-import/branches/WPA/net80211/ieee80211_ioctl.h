/*	$NetBSD: ieee80211_ioctl.h,v 1.5 2003/10/13 04:16:59 dyoung Exp $	*/
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
 * $FreeBSD: src/sys/net80211/ieee80211_ioctl.h,v 1.4 2003/10/17 23:15:30 sam Exp $
 */
#ifndef _NET80211_IEEE80211_IOCTL_H_
#define _NET80211_IEEE80211_IOCTL_H_

/*
 * IEEE 802.11 ioctls.
 */

/*
 * Per/node (station) statistics available when operating as an AP.
 */
struct ieee80211_nodestats {
	u_int32_t	ns_rx_data;		/* rx data frames */
	u_int32_t	ns_rx_mgmt;		/* rx management frames */
	u_int32_t	ns_rx_ctrl;		/* rx control frames */
	u_int32_t	ns_rx_ucast;		/* rx unicast frames */
	u_int32_t	ns_rx_mcast;		/* rx multi/broadcast frames */
	u_int64_t	ns_rx_bytes;		/* rx data count (bytes) */

	u_int32_t	ns_rx_dup;		/* rx discard 'cuz dup */
	u_int32_t	ns_rx_noprivacy;	/* rx w/ wep but privacy off */
	u_int32_t	ns_rx_wepfail;		/* rx wep processing failed */
	u_int32_t	ns_rx_decap;		/* rx decapsulation failed */
	u_int32_t	ns_rx_disassoc;		/* rx disassociation */
	u_int32_t	ns_rx_deauth;		/* rx deauthentication */
	u_int32_t	ns_rx_decryptcrc;	/* rx decrypt failed on crc */
	u_int32_t	ns_rx_unauth;		/* rx on unauthorized port */

	u_int32_t	ns_tx_data;		/* tx data frames */
	u_int32_t	ns_tx_mgmt;		/* tx management frames */
	u_int32_t	ns_tx_ucast;		/* tx unicast frames */
	u_int32_t	ns_tx_mcast;		/* tx multi/broadcast frames */
	u_int64_t	ns_tx_bytes;		/* tx data count (bytes) */

	u_int32_t	ns_tx_novlantag;	/* tx discard 'cuz no tag */
	u_int32_t	ns_tx_vlanmismatch;	/* tx discard 'cuz bad tag */

	/* MIB-related state */
	u_int32_t	ns_mib_assoc;		/* [re]associations */
	u_int32_t	ns_mib_assoc_fail;	/* [re]association failures */
	u_int32_t	ns_mib_auth;		/* [re]authentications */
	u_int32_t	ns_mib_auth_fail;	/* [re]authentication failures*/
	u_int32_t	ns_mib_deauth;		/* deauthentications */
	u_int32_t	ns_mib_deauth_code;	/* last deauth reason */
	u_int32_t	ns_mib_disassoc;	/* disassociations */
	u_int32_t	ns_mib_disassoc_code;	/* last disassociation reason */
};

/*
 * Summary statistics.
 */
struct ieee80211_stats {
	u_int32_t	is_rx_badversion;	/* rx frame with bad version */
	u_int32_t	is_rx_tooshort;		/* rx frame too short */
	u_int32_t	is_rx_wrongbss;		/* rx from wrong bssid */
	u_int32_t	is_rx_dup;		/* rx discard 'cuz dup */
	u_int32_t	is_rx_wrongdir;		/* rx w/ wrong direction */
	u_int32_t	is_rx_mcastecho;	/* rx discard 'cuz mcast echo */
	u_int32_t	is_rx_notassoc;		/* rx discard 'cuz sta !assoc */
	u_int32_t	is_rx_noprivacy;	/* rx w/ wep but privacy off */
	u_int32_t	is_rx_unencrypted;	/* rx w/o wep and privacy on */
	u_int32_t	is_rx_wepfail;		/* rx wep processing failed */
	u_int32_t	is_rx_decap;		/* rx decapsulation failed */
	u_int32_t	is_rx_mgtdiscard;	/* rx discard mgt frames */
	u_int32_t	is_rx_ctl;		/* rx discard ctrl frames */
	u_int32_t	is_rx_beacon;		/* rx beacon frames */
	u_int32_t	is_rx_rstoobig;		/* rx rate set truncated */
	u_int32_t	is_rx_elem_missing;	/* rx required element missing*/
	u_int32_t	is_rx_elem_toobig;	/* rx element too big */
	u_int32_t	is_rx_elem_toosmall;	/* rx element too small */
	u_int32_t	is_rx_elem_unknown;	/* rx element unknown */
	u_int32_t	is_rx_badchan;		/* rx frame w/ invalid chan */
	u_int32_t	is_rx_chanmismatch;	/* rx frame chan mismatch */
	u_int32_t	is_rx_nodealloc;	/* rx frame dropped */
	u_int32_t	is_rx_ssidmismatch;	/* rx frame ssid mismatch  */
	u_int32_t	is_rx_auth_unsupported;	/* rx w/ unsupported auth alg */
	u_int32_t	is_rx_auth_fail;	/* rx sta auth failure */
	u_int32_t	is_rx_assoc_bss;	/* rx assoc from wrong bssid */
	u_int32_t	is_rx_assoc_notauth;	/* rx assoc w/o auth */
	u_int32_t	is_rx_assoc_capmismatch;/* rx assoc w/ cap mismatch */
	u_int32_t	is_rx_assoc_norate;	/* rx assoc w/ no rate match */
	u_int32_t	is_rx_assoc_badwpaie;	/* rx assoc w/ bad WPA IE */
	u_int32_t	is_rx_deauth;		/* rx deauthentication */
	u_int32_t	is_rx_disassoc;		/* rx disassociation */
	u_int32_t	is_rx_badsubtype;	/* rx frame w/ unknown subtype*/
	u_int32_t	is_rx_nobuf;		/* rx failed for lack of buf */
	u_int32_t	is_rx_decryptcrc;	/* rx decrypt failed on crc */
	u_int32_t	is_rx_ahdemo_mgt;	/* rx discard ahdemo mgt frame*/
	u_int32_t	is_rx_bad_auth;		/* rx bad auth request */
	u_int32_t	is_rx_unauth;		/* rx on unauthorized port */
	u_int32_t	is_rx_badkeyid;		/* rx w/ incorrect keyid */
	u_int32_t	is_rx_ccmpreplay;	/* rx seq# violation (CCMP) */
	u_int32_t	is_rx_ccmpformat;	/* rx format bad (CCMP) */
	u_int32_t	is_rx_ccmpmic;		/* rx MIC check failed (CCMP) */
	u_int32_t	is_rx_tkipreplay;	/* rx seq# violation (TKIP) */
	u_int32_t	is_rx_tkipformat;	/* rx format bad (TKIP) */
	u_int32_t	is_rx_tkipmic;		/* rx MIC check failed (TKIP) */
	u_int32_t	is_rx_tkipicv;		/* rx ICV check failed (TKIP) */
	u_int32_t	is_rx_badcipher;	/* rx failed 'cuz key type */
	u_int32_t	is_rx_nocipherctx;	/* rx failed 'cuz key !setup */
	u_int32_t	is_tx_nobuf;		/* tx failed for lack of buf */
	u_int32_t	is_tx_nonode;		/* tx failed for no node */
	u_int32_t	is_tx_unknownmgt;	/* tx of unknown mgt frame */
	u_int32_t	is_tx_badcipher;	/* tx failed 'cuz key type */
	u_int32_t	is_tx_nocipherctx;	/* tx failed 'cuz key !setup */
	u_int32_t	is_tx_wepfail;		/* tx wep processing failed */
	u_int32_t	is_scan_active;		/* active scans started */
	u_int32_t	is_scan_passive;	/* passive scans started */
	u_int32_t	is_node_timeout;	/* nodes timed out inactivity */
	u_int32_t	is_crypto_nomem;	/* no memory for crypto ctx */
};

/*
 * Max size of optional information elements.  We artificially
 * constrain this; it's limited only by the max frame size (and
 * the max parameter size of the wireless extensions).
 */
#define	IEEE80211_MAX_OPT_IE	256

/*
 * MLME state manipulation request.  IEEE80211_MLME_ASSOC
 * only makes sense when operating as a station.  The other
 * requests can be used when operating as a station or an
 * ap (to effect a station).
 */
struct ieee80211req_mlme {
	u_int8_t	im_op;		/* operation to perform */
#define	IEEE80211_MLME_ASSOC		1	/* associate station */
#define	IEEE80211_MLME_DISASSOC		2	/* disassociate station */
#define	IEEE80211_MLME_DEAUTH		3	/* deauthenticate station */
	u_int16_t	im_reason;	/* 802.11 reason code */
	u_int8_t	im_macaddr[IEEE80211_ADDR_LEN];
};

#ifdef __FreeBSD__
/*
 * FreeBSD-style ioctls.
 */
/* the first member must be matched with struct ifreq */
struct ieee80211req {
	char		i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	u_int16_t	i_type;			/* req type */
	int16_t		i_val;			/* Index or simple value */
	int16_t		i_len;			/* Index or simple value */
	void		*i_data;		/* Extra data */
};
#define	SIOCS80211		 _IOW('i', 234, struct ieee80211req)
#define	SIOCG80211		_IOWR('i', 235, struct ieee80211req)

#define IEEE80211_IOC_SSID		1
#define IEEE80211_IOC_NUMSSIDS		2
#define IEEE80211_IOC_WEP		3
#define 	IEEE80211_WEP_NOSUP	-1
#define 	IEEE80211_WEP_OFF	0
#define 	IEEE80211_WEP_ON	1
#define 	IEEE80211_WEP_MIXED	2
#define IEEE80211_IOC_WEPKEY		4
#define IEEE80211_IOC_NUMWEPKEYS	5
#define IEEE80211_IOC_WEPTXKEY		6
#define IEEE80211_IOC_AUTHMODE		7
#define IEEE80211_IOC_STATIONNAME	8
#define IEEE80211_IOC_CHANNEL		9
#define IEEE80211_IOC_POWERSAVE		10
#define 	IEEE80211_POWERSAVE_NOSUP	-1
#define 	IEEE80211_POWERSAVE_OFF		0
#define 	IEEE80211_POWERSAVE_CAM		1
#define 	IEEE80211_POWERSAVE_PSP		2
#define 	IEEE80211_POWERSAVE_PSP_CAM	3
#define 	IEEE80211_POWERSAVE_ON		IEEE80211_POWERSAVE_CAM
#define IEEE80211_IOC_POWERSAVESLEEP	11
#define	IEEE80211_IOC_RTSTHRESHOLD	12
#define IEEE80211_IOC_PROTMODE		13
#define 	IEEE80211_PROTMODE_OFF		0
#define 	IEEE80211_PROTMODE_CTS		1
#define 	IEEE80211_PROTMODE_RTSCTS	2
#define	IEEE80211_IOC_TXPOWER		14
#define	IEEE80211_IOC_BSSID		15
#define	IEEE80211_IOC_ROAMING		16
#define	IEEE80211_IOC_PRIVACY		17
#define	IEEE80211_IOC_DROP_UNENCRYPTED	18
#define	IEEE80211_IOC_MLME		21
#define	IEEE80211_IOC_OPTIE		22
#define	IEEE80211_IOC_SCAN_REQ		23
#define	IEEE80211_IOC_SCAN_RESULTS	24
#define	IEEE80211_IOC_COUNTERMEASURES	25
#define	IEEE80211_IOC_WPA		26

#ifndef IEEE80211_CHAN_ANY
#define	IEEE80211_CHAN_ANY	0xffff		/* token for ``any channel'' */
#endif

struct ieee80211req_scan_req {
	u_int8_t	isq_ssid_len;			/* SSID length */
	u_int8_t	isq_ssid[IEEE80211_NWID_LEN];
};

struct ieee80211req_scan_result {
	u_int16_t	isr_len;			/* length (mult of 4) */
	u_int16_t	isr_freq;			/* MHz */
	u_int16_t	isr_flags;			/* channel flags */
	u_int8_t	isr_noise;
	u_int8_t	isr_rssi;
	u_int8_t	isr_intval;			/* beacon interval */
	u_int8_t	isr_capinfo;			/* capabilities */
	u_int8_t	isr_bssid[IEEE80211_ADDR_LEN];
	u_int8_t	isr_nrates;
	u_int8_t	isr_rates[15];			/* XXX */
	u_int8_t	isr_ssid_len;			/* SSID length */
	u_int8_t	isr_ie_len;			/* IE length */
	/* variable length SSID followed by IE data */
};

#define	SIOCG80211STATS		_IOWR('i', 236, struct ifreq)
#endif /* __FreeBSD__ */

#ifdef __linux__
/*
 * Wireless Extensions API, private ioctl interfaces.
 *
 * NB: Even-numbered ioctl numbers have set semantics and are privileged!
 *     (regardless of the incorrect comment in wireless.h!)
 */
#define	IEEE80211_IOCTL_SETPARAM	(SIOCIWFIRSTPRIV+0)
#define	IEEE80211_IOCTL_GETPARAM	(SIOCIWFIRSTPRIV+1)
#define	IEEE80211_IOCTL_SETMLME		(SIOCIWFIRSTPRIV+6)
#define	IEEE80211_IOCTL_SETOPTIE	(SIOCIWFIRSTPRIV+8)
#define	IEEE80211_IOCTL_GETOPTIE	(SIOCIWFIRSTPRIV+9)

enum {
	IEEE80211_PARAM_TURBO		= 1,	/* turbo mode */
	IEEE80211_PARAM_MODE		= 2,	/* phy mode (11a, 11b, etc.) */
	IEEE80211_PARAM_AUTHMODE	= 3,	/* authentication mode */
	IEEE80211_PARAM_PROTMODE	= 4,	/* 802.11g protection */
	IEEE80211_PARAM_MCASTCIPHER	= 5,	/* multicast/default cipher */
	IEEE80211_PARAM_MCASTKEYLEN	= 6,	/* multicast key length */
	IEEE80211_PARAM_UCASTCIPHERS	= 7,	/* unicast cipher suites */
	IEEE80211_PARAM_UCASTCIPHER	= 8,	/* unicast cipher */
	IEEE80211_PARAM_UCASTKEYLEN	= 9,	/* unicast key length */
	IEEE80211_PARAM_WPA		= 10,	/* WPA mode (0,1,2) */
	IEEE80211_PARAM_ROAMING		= 12,	/* roaming mode */
	IEEE80211_PARAM_PRIVACY		= 13,	/* privacy invoked */
	IEEE80211_PARAM_COUNTERMEASURES	= 14,	/* WPA/TKIP countermeasures */
	IEEE80211_PARAM_DROPUNENCRYPTED	= 15,	/* discard unencrypted frames */
	IEEE80211_PARAM_DRIVER_CAPS	= 16,	/* driver capabilities */
};

#define	SIOCG80211STATS		(SIOCDEVPRIVATE+2)
#endif /* __linux__ */

#endif /* _NET80211_IEEE80211_IOCTL_H_ */
