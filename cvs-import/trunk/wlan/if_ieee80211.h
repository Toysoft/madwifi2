/*	$NetBSD: if_ieee80211.h,v 1.23 2002/10/15 08:51:50 onoe Exp $	*/
/*	$FreeBSD: src/sys/net/if_ieee80211.h,v 1.8 2003/01/15 20:01:50 sam Exp $ */
/*	$Id$	*/

/*-
 * Copyright (c) 2000, 2001 The NetBSD Foundation, Inc.
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

#ifndef _NET_IF_IEEE80211_H_
#define _NET_IF_IEEE80211_H_

#include <sys/queue.h>
#include "if_media.h"
#ifdef CONFIG_NET_WIRELESS
#include <linux/wireless.h>
#endif

#define	IEEE80211_ADDR_LEN			6

/*
 * generic definitions for IEEE 802.11 frames
 */
struct ieee80211_frame {
	u_int8_t	i_fc[2];
	u_int8_t	i_dur[2];
	u_int8_t	i_addr1[IEEE80211_ADDR_LEN];
	u_int8_t	i_addr2[IEEE80211_ADDR_LEN];
	u_int8_t	i_addr3[IEEE80211_ADDR_LEN];
	u_int8_t	i_seq[2];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __attribute__((__packed__));

struct ieee80211_frame_addr4 {
	u_int8_t	i_fc[2];
	u_int8_t	i_dur[2];
	u_int8_t	i_addr1[IEEE80211_ADDR_LEN];
	u_int8_t	i_addr2[IEEE80211_ADDR_LEN];
	u_int8_t	i_addr3[IEEE80211_ADDR_LEN];
	u_int8_t	i_seq[2];
	u_int8_t	i_addr4[IEEE80211_ADDR_LEN];
} __attribute__((__packed__));

#define	IEEE80211_FC0_VERSION_MASK		0x03
#define	IEEE80211_FC0_VERSION_SHIFT		0
#define	IEEE80211_FC0_VERSION_0			0x00
#define	IEEE80211_FC0_TYPE_MASK			0x0c
#define	IEEE80211_FC0_TYPE_SHIFT		2
#define	IEEE80211_FC0_TYPE_MGT			0x00
#define	IEEE80211_FC0_TYPE_CTL			0x04
#define	IEEE80211_FC0_TYPE_DATA			0x08

#define	IEEE80211_FC0_SUBTYPE_MASK		0xf0
#define	IEEE80211_FC0_SUBTYPE_SHIFT		4
/* for TYPE_MGT */
#define	IEEE80211_FC0_SUBTYPE_ASSOC_REQ		0x00
#define	IEEE80211_FC0_SUBTYPE_ASSOC_RESP	0x10
#define	IEEE80211_FC0_SUBTYPE_REASSOC_REQ	0x20
#define	IEEE80211_FC0_SUBTYPE_REASSOC_RESP	0x30
#define	IEEE80211_FC0_SUBTYPE_PROBE_REQ		0x40
#define	IEEE80211_FC0_SUBTYPE_PROBE_RESP	0x50
#define	IEEE80211_FC0_SUBTYPE_BEACON		0x80
#define	IEEE80211_FC0_SUBTYPE_ATIM		0x90
#define	IEEE80211_FC0_SUBTYPE_DISASSOC		0xa0
#define	IEEE80211_FC0_SUBTYPE_AUTH		0xb0
#define	IEEE80211_FC0_SUBTYPE_DEAUTH		0xc0
/* for TYPE_CTL */
#define	IEEE80211_FC0_SUBTYPE_PS_POLL		0xa0
#define	IEEE80211_FC0_SUBTYPE_RTS		0xb0
#define	IEEE80211_FC0_SUBTYPE_CTS		0xc0
#define	IEEE80211_FC0_SUBTYPE_ACK		0xd0
#define	IEEE80211_FC0_SUBTYPE_CF_END		0xe0
#define	IEEE80211_FC0_SUBTYPE_CF_END_ACK	0xf0
/* for TYPE_DATA (bit combination) */
#define	IEEE80211_FC0_SUBTYPE_DATA		0x00
#define	IEEE80211_FC0_SUBTYPE_CF_ACK		0x10
#define	IEEE80211_FC0_SUBTYPE_CF_POLL		0x20
#define	IEEE80211_FC0_SUBTYPE_CF_ACPL		0x30
#define	IEEE80211_FC0_SUBTYPE_NODATA		0x40
#define	IEEE80211_FC0_SUBTYPE_CFACK		0x50
#define	IEEE80211_FC0_SUBTYPE_CFPOLL		0x60
#define	IEEE80211_FC0_SUBTYPE_CF_ACK_CF_ACK	0x70

#define	IEEE80211_FC1_DIR_MASK			0x03
#define	IEEE80211_FC1_DIR_NODS			0x00	/* STA->STA */
#define	IEEE80211_FC1_DIR_TODS			0x01	/* STA->AP  */
#define	IEEE80211_FC1_DIR_FROMDS		0x02	/* AP ->STA */
#define	IEEE80211_FC1_DIR_DSTODS		0x03	/* AP ->AP  */

#define	IEEE80211_FC1_MORE_FRAG			0x04
#define	IEEE80211_FC1_RETRY			0x08
#define	IEEE80211_FC1_PWR_MGT			0x10
#define	IEEE80211_FC1_MORE_DATA			0x20
#define	IEEE80211_FC1_WEP			0x40
#define	IEEE80211_FC1_ORDER			0x80

#define	IEEE80211_SEQ_FRAG_MASK			0x000f
#define	IEEE80211_SEQ_FRAG_SHIFT		0
#define	IEEE80211_SEQ_SEQ_MASK			0xfff0
#define	IEEE80211_SEQ_SEQ_SHIFT			4

#define	IEEE80211_NWID_LEN			32

/*
 * BEACON management packets
 *
 *	octet timestamp[8]
 *	octet beacon interval[2]
 *	octet capability information[2]
 *	information element
 *		octet elemid
 *		octet length
 *		octet information[length]
 */

typedef uint8_t *ieee80211_mgt_beacon_t;

#define	IEEE80211_BEACON_INTERVAL(beacon) \
	((beacon)[8] | ((beacon)[9] << 8))
#define	IEEE80211_BEACON_CAPABILITY(beacon) \
	((beacon)[10] | ((beacon)[11] << 8))

#define	IEEE80211_CAPINFO_ESS			0x0001
#define	IEEE80211_CAPINFO_IBSS			0x0002
#define	IEEE80211_CAPINFO_CF_POLLABLE		0x0004
#define	IEEE80211_CAPINFO_CF_POLLREQ		0x0008
#define	IEEE80211_CAPINFO_PRIVACY		0x0010
#define	IEEE80211_CAPINFO_SHORT_PREAMBLE	0x0020
#define	IEEE80211_CAPINFO_PBCC			0x0040
#define	IEEE80211_CAPINFO_CHNL_AGILITY		0x0080

#define	IEEE80211_RATE_BASIC			0x80
#define	IEEE80211_RATE_VAL			0x7f

/*
 * Management information elements
 */

struct ieee80211_information {
	char	ssid[IEEE80211_NWID_LEN+1];
	struct rates {
		u_int8_t	*p;
	} rates;
	struct fh {
		u_int16_t	dwell;
		u_int8_t	set;
		u_int8_t	pattern;
		u_int8_t	index;
	} fh;
	struct ds {
		u_int8_t	channel;
	} ds;
	struct cf {
		u_int8_t	count;
		u_int8_t	period;
		u_int8_t	maxdur[2];
		u_int8_t	dur[2];
	} cf;
	struct tim {
		u_int8_t	count;
		u_int8_t	period;
		u_int8_t	bitctl;
		/* u_int8_t	pvt[251]; The driver needs to use this. */
	} tim;
	struct ibss {
		u_int16_t	atim;
	} ibss;
	struct challenge {
		u_int8_t	*p;
		u_int8_t	len;
	} challenge;
};

#define	IEEE80211_ELEMID_SSID			0
#define	IEEE80211_ELEMID_RATES			1
#define	IEEE80211_ELEMID_FHPARMS		2
#define	IEEE80211_ELEMID_DSPARMS		3
#define	IEEE80211_ELEMID_CFPARMS		4
#define	IEEE80211_ELEMID_TIM			5
#define	IEEE80211_ELEMID_IBSSPARMS		6
#define	IEEE80211_ELEMID_CHALLENGE		16

/*
 * AUTH management packets
 *
 *	octet algo[2]
 *	octet seq[2]
 *	octet status[2]
 *	octet chal.id
 *	octet chal.length
 *	octet chal.text[253]
 */

typedef u_int8_t *ieee80211_mgt_auth_t;

#define	IEEE80211_AUTH_ALGORITHM(auth) \
	((auth)[0] | ((auth)[1] << 8))
#define	IEEE80211_AUTH_TRANSACTION(auth) \
	((auth)[2] | ((auth)[3] << 8))
#define	IEEE80211_AUTH_STATUS(auth) \
	((auth)[4] | ((auth)[5] << 8))

#define	IEEE80211_AUTH_ALG_OPEN			0x0000
#define	IEEE80211_AUTH_ALG_SHARED		0x0001

#define	IEEE80211_AUTH_OPEN_REQUEST		1
#define	IEEE80211_AUTH_OPEN_RESPONSE		2

#define	IEEE80211_AUTH_SHARED_REQUEST		1
#define	IEEE80211_AUTH_SHARED_CHALLENGE		2
#define	IEEE80211_AUTH_SHARED_RESPONSE		3
#define	IEEE80211_AUTH_SHARED_PASS		4

/*
 * Reason codes
 *
 * Unlisted codes are reserved
 */

#define	IEEE80211_REASON_UNSPECIFIED		1
#define	IEEE80211_REASON_AUTH_EXPIRE		2
#define	IEEE80211_REASON_AUTH_LEAVE		3
#define	IEEE80211_REASON_ASSOC_EXPIRE		4
#define	IEEE80211_REASON_ASSOC_TOOMANY		5
#define	IEEE80211_REASON_NOT_AUTHED		6  
#define	IEEE80211_REASON_NOT_ASSOCED		7
#define	IEEE80211_REASON_ASSOC_LEAVE		8
#define	IEEE80211_REASON_ASSOC_NOT_AUTHED	9

#define	IEEE80211_STATUS_SUCCESS		0
#define	IEEE80211_STATUS_UNSPECIFIED		1
#define	IEEE80211_STATUS_CAPINFO		10
#define	IEEE80211_STATUS_NOT_ASSOCED		11
#define	IEEE80211_STATUS_OTHER			12
#define	IEEE80211_STATUS_ALG			13
#define	IEEE80211_STATUS_SEQUENCE		14
#define	IEEE80211_STATUS_CHALLENGE		15
#define	IEEE80211_STATUS_TIMEOUT		16
#define	IEEE80211_STATUS_TOOMANY		17
#define	IEEE80211_STATUS_BASIC_RATE		18
#define	IEEE80211_STATUS_SP_REQUIRED		19
#define	IEEE80211_STATUS_PBCC_REQUIRED		20
#define	IEEE80211_STATUS_CA_REQUIRED		21
#define	IEEE80211_STATUS_TOO_MANY_STATIONS	22
#define	IEEE80211_STATUS_RATES			23

#define	IEEE80211_WEP_KEYLEN			5	/* 40bit */
#define	IEEE80211_WEP_IVLEN			3	/* 24bit */
#define	IEEE80211_WEP_KIDLEN			1	/* 1 octet */
#define	IEEE80211_WEP_CRCLEN			4	/* CRC-32 */
#define	IEEE80211_WEP_NKID			4	/* number of key ids */

#define	IEEE80211_CRC_LEN			4

#define	IEEE80211_MTU				1500
#define	IEEE80211_MAX_LEN			(2300 + IEEE80211_CRC_LEN + \
    (IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN + IEEE80211_WEP_CRCLEN))

/*
 * ioctls
 */

#ifndef SIOCSIFGENERIC
#define	SIOCSIFGENERIC	 _IOW('i', 57, struct ifreq)	/* generic IF set op */
#endif
#ifndef SIOCGIFGENERIC
#define	SIOCGIFGENERIC	_IOWR('i', 58, struct ifreq)	/* generic IF get op */
#endif

/* nwid is pointed at by ifr.ifr_data */
struct ieee80211_nwid {
	u_int8_t	i_len;
	u_int8_t	i_nwid[IEEE80211_NWID_LEN];
};

#define	SIOCS80211NWID		_IOWR('i', 230, struct ifreq)
#define	SIOCG80211NWID		_IOWR('i', 231, struct ifreq)

/* the first member must be matched with struct ifreq */
struct ieee80211_nwkey {
	char		i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	int		i_wepon;		/* wep enabled flag */
	int		i_defkid;		/* default encrypt key id */
	struct {
		int		i_keylen;
		u_int8_t	*i_keydat;
	}		i_key[IEEE80211_WEP_NKID];
};
#define	SIOCS80211NWKEY		 _IOW('i', 232, struct ieee80211_nwkey)
#define	SIOCG80211NWKEY		_IOWR('i', 233, struct ieee80211_nwkey)
/* i_wepon */
#define	IEEE80211_NWKEY_OPEN	0		/* No privacy */
#define	IEEE80211_NWKEY_WEP	1		/* WEP enabled */
#define	IEEE80211_NWKEY_EAP	2		/* EAP enabled */
#define	IEEE80211_NWKEY_PERSIST	0x100		/* designate persist keyset */

/* power management parameters */
struct ieee80211_power {
	char		i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	int		i_enabled;		/* 1 == on, 0 == off */
	int		i_maxsleep;		/* max sleep in ms */
};
#ifdef __NetBSD__
#define	SIOCS80211POWER		 _IOW('i', 234, struct ieee80211_power)
#define	SIOCG80211POWER		_IOWR('i', 235, struct ieee80211_power)
#else
#define	SIOCS80211POWER		 _IOW('i', 242, struct ieee80211_power)
#define	SIOCG80211POWER		_IOWR('i', 243, struct ieee80211_power)
#endif

struct ieee80211_auth {
	char		i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	int		i_authtype;
};

#define	IEEE80211_AUTH_NONE	0
#define	IEEE80211_AUTH_OPEN	1
#define	IEEE80211_AUTH_SHARED	2

#define	SIOCS80211AUTH		 _IOW('i', 236, struct ieee80211_auth)
#define	SIOCG80211AUTH		_IOWR('i', 237, struct ieee80211_auth)

struct ieee80211_channel {
	char		i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	u_int16_t	i_channel;
};

#define	IEEE80211_CHAN_ANY	0xffff

#define	SIOCS80211CHANNEL	 _IOW('i', 238, struct ieee80211_channel)
#define	SIOCG80211CHANNEL	_IOWR('i', 239, struct ieee80211_channel)

struct ieee80211_bssid {
	char		i_name[IFNAMSIZ];	/* if_name, e.g. "wi0" */
	u_int8_t	i_bssid[IEEE80211_ADDR_LEN];
};

#define	SIOCS80211BSSID		 _IOW('i', 240, struct ieee80211_bssid)
#define	SIOCG80211BSSID		_IOWR('i', 241, struct ieee80211_bssid)

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
#ifdef __FreeBSD__
#define	SIOCS80211		 _IOW('i', 234, struct ieee80211req)
#define	SIOCG80211		_IOWR('i', 235, struct ieee80211req)
#else
#define	SIOCS80211		 _IOW('i', 242, struct ieee80211req)
#define	SIOCG80211		_IOWR('i', 243, struct ieee80211req)
#endif

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

#ifdef __KERNEL__

#define	IEEE80211_PSCAN_WAIT 	5		/* passive scan wait */
#define	IEEE80211_TRANS_WAIT 	5		/* transition wait */
#define	IEEE80211_INACT_WAIT	5		/* inactivity timer interval */
#define	IEEE80211_INACT_MAX	(300/IEEE80211_INACT_WAIT)

/*
 * Structure for IEEE 802.11 drivers.
 */

#define	IEEE80211_CHAN_MAX	64
#define	IEEE80211_RATE_SIZE	12
#define	IEEE80211_KEYBUF_SIZE	16
#define	IEEE80211_NODE_HASHSIZE	32
/* simple hash is enough for variation of macaddr */
#define	IEEE80211_NODE_HASH(addr)	\
	(((u_int8_t *)(addr))[IEEE80211_ADDR_LEN - 1] % IEEE80211_NODE_HASHSIZE)

enum ieee80211_phytype {
	IEEE80211_T_DS,			/* direct sequence spread spectrum */
	IEEE80211_T_FH,			/* frequency hopping */
	IEEE80211_T_OFDM,		/* frequency division multiplexing */
	IEEE80211_T_HRDS		/* high rate DS */
};

enum ieee80211_opmode {
	IEEE80211_M_STA = 1,		/* infrastructure station */
	IEEE80211_M_IBSS = 0,		/* IBSS (adhoc) station */
	IEEE80211_M_AHDEMO = 3,		/* Old lucent compatible adhoc demo */
	IEEE80211_M_HOSTAP = 6		/* Software Access Point */
};

enum ieee80211_state {
	IEEE80211_S_INIT,		/* default state */
	IEEE80211_S_SCAN,		/* scanning */
	IEEE80211_S_AUTH,		/* try to authenticate */
	IEEE80211_S_ASSOC,		/* try to assoc */
	IEEE80211_S_RUN			/* associated */
};

/*
 * Channels are specified by frequency and attributes.
 */
/* XXX should be ieee80211_channel but NetBSD took that */
struct ieee80211channel {
	u_int16_t	ic_freq;	/* setting in Mhz */
	u_int16_t	ic_flags;	/* see below */
};

#define	IEEE80211_CHAN_PASSIVE	0x0001	/* Only passive scan allowed on channel */
/* bits 4-7 are for private use by drivers */
#define	IEEE80211_CHAN_PRIV3	0x0010
#define	IEEE80211_CHAN_PRIV2	0x0020
#define	IEEE80211_CHAN_PRIV1	0x0040
#define	IEEE80211_CHAN_PRIV0	0x0080
/* channel attributes */
#define	IEEE80211_CHAN_TURBO	0x0100	/* Turbo channel */
#define	IEEE80211_CHAN_CCK	0x0200	/* CCK channel */
#define	IEEE80211_CHAN_OFDM	0x0400	/* OFDM channel */
#define	IEEE80211_CHAN_2GHZ	0x0800	/* 2 GHz spectrum channel. */
#define	IEEE80211_CHAN_5GHZ	0x1000	/* 5 GHz spectrum channel */
/* bits 13-15 are reserved for future use and should not be used */

/*
 * Useful combinations of channel characteristics.
 */
#define	IEEE80211_CHAN_A \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM)
#define	IEEE80211_CHAN_B \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_CCK)
#define	IEEE80211_CHAN_PUREG \
	(IEEE80211_CHAN_2GHZ | IEEE80211_CHAN_OFDM)
#define	IEEE80211_CHAN_T \
	(IEEE80211_CHAN_5GHZ | IEEE80211_CHAN_OFDM | IEEE80211_CHAN_TURBO)

/*
 * Node specific information.
 */
struct ieee80211_node {
	TAILQ_ENTRY(ieee80211_node)	ni_list;
	LIST_ENTRY(ieee80211_node)	ni_hash;
	atomic_t		ni_refcnt;

	/* hardware */
	u_int8_t		ni_rssi;
	u_int32_t		ni_rstamp;

	/* header */
	u_int8_t		ni_macaddr[IEEE80211_ADDR_LEN];
	u_int8_t		ni_bssid[IEEE80211_ADDR_LEN];

	/* beacon, probe response */
	u_int8_t		ni_tstamp[8];
	u_int16_t		ni_intval;
	u_int16_t		ni_capinfo;
	u_int8_t		ni_esslen;
	u_int8_t		ni_essid[IEEE80211_NWID_LEN];
	int			ni_nrate;
	u_int8_t		ni_rates[IEEE80211_RATE_SIZE];
	struct ieee80211channel	*ni_chan;
	u_int16_t		ni_fhdwell;	/* FH only */
	u_int8_t		ni_fhindex;	/* FH only */

	/* DTIM and contention free period (CFP) */
	u_int8_t		ni_dtimperiod;
	u_int8_t		ni_cfpperiod;	/* # of DTIMs between CFPs */
	u_int16_t		ni_cfpmaxduration;/* max CFP duration in TU */
	u_int16_t		ni_cfpduremain;	/* remaining duration */
	u_int16_t		ni_timoffset;

	/* others */
	u_int16_t		ni_associd;	/* assoc response */
	u_int16_t		ni_txseq;	/* seq to be transmitted */
	u_int16_t		ni_rxseq;	/* seq previous received */
	int			ni_fails;	/* failure count to associate */
	int			ni_inact;	/* inactivity mark count */
	int			ni_txrate;	/* index to ni_rates[] */
	void			*ni_private;	/* driver private */
};

static inline struct ieee80211_node *
ieee80211_ref_node(struct ieee80211_node *ni)
{
	atomic_inc(&ni->ni_refcnt);
	return ni;
}

static inline void
ieee80211_unref_node(struct ieee80211_node **ni)
{
	atomic_dec(&(*ni)->ni_refcnt);
	*ni = NULL;			/* guard against use */
}

/* ni_chan encoding for FH phy */
#define	IEEE80211_FH_CHANMOD	80
#define	IEEE80211_FH_CHAN(set,pat)	(((set)-1)*IEEE80211_FH_CHANMOD+(pat))
#define	IEEE80211_FH_CHANSET(chan)	((chan)/IEEE80211_FH_CHANMOD+1)
#define	IEEE80211_FH_CHANPAT(chan)	((chan)%IEEE80211_FH_CHANMOD)

struct ieee80211_wepkey {
	int			wk_len;
	u_int8_t		wk_key[IEEE80211_KEYBUF_SIZE];
};

struct proc_dir_entry;

struct ieee80211com {
	struct net_device	ic_dev;		/* NB: this must be first */
	struct timer_list	ic_slowtimo;	/* mgmt/inactivity timer */
	int			ic_mgt_timer;	/* mgmt timeout */
	int			ic_inact_timer;	/* inactivity timer wait */
	int			(*ic_mgtstart)(struct sk_buff *,
					struct net_device *);
	int			(*ic_init)(struct net_device *);
	void			(*ic_recv_mgmt[16])(struct ieee80211com *,
				    struct sk_buff *, int, u_int32_t);
	struct net_device_stats	ic_stats;	/* interface statistics */
	u_int32_t		msg_enable;	/* interface message flags */
	int			(*ic_send_mgmt[16])(struct ieee80211com *,
				    struct ieee80211_node *, int, int);
	int			(*ic_newstate)(void *, enum ieee80211_state);
	int			(*ic_chancheck)(void *, u_char *);
	u_int8_t		ic_sup_rates[IEEE80211_RATE_SIZE];
	struct ieee80211channel ic_channels[IEEE80211_CHAN_MAX+1];
	u_char			ic_chan_avail[roundup(IEEE80211_CHAN_MAX,NBBY)];
	u_char			ic_chan_active[roundup(IEEE80211_CHAN_MAX, NBBY)];
	u_char			ic_chan_scan[roundup(IEEE80211_CHAN_MAX,NBBY)];
	u_int32_t		ic_flags;	/* state flags */
	u_int32_t		ic_caps;	/* capabilities */
	enum ieee80211_phytype	ic_phytype;
	enum ieee80211_opmode	ic_opmode;	/* operation mode */
	enum ieee80211_state	ic_state;	/* 802.11 state */
	struct ifmedia		ic_media;	/* interface media config */
	struct ieee80211_node	ic_bss;		/* information for this node */
	int			ic_node_privlen;/* size for ni_private */
	void			(*ic_node_free)(struct ieee80211com *,
				    struct ieee80211_node *);	/* callback */
	struct ieee80211channel	*ic_ibss_chan;
	int			ic_fixed_rate;	/* index to ic_sup_rates[] */
	u_int16_t		ic_rtsthreshold;
	u_int16_t		ic_fragthreshold;
	rwlock_t		ic_nodelock;	/* on node table */
	TAILQ_HEAD(, ieee80211_node) ic_node;	/* information of all nodes */
	LIST_HEAD(, ieee80211_node) ic_hash[IEEE80211_NODE_HASHSIZE];
	u_int16_t		ic_lintval;	/* listen interval */
	u_int16_t		ic_holdover;	/* PM hold over duration */
	u_int16_t		ic_txmin;	/* min tx retry count */
	u_int16_t		ic_txmax;	/* max tx retry count */
	u_int16_t		ic_txlifetime;	/* tx lifetime */
	u_int16_t		ic_txpower;	/* tx power setting (dbM) */
	int			ic_nicknamelen;
	u_int8_t		ic_nickname[IEEE80211_NWID_LEN];
	int			ic_des_esslen;
	u_int8_t		ic_des_essid[IEEE80211_NWID_LEN];
	struct ieee80211channel	*ic_des_chan;	/* desired channel */
	u_int8_t		ic_des_bssid[IEEE80211_ADDR_LEN];
	struct ieee80211_wepkey	ic_nw_keys[IEEE80211_WEP_NKID];
	int			ic_wep_txkey;	/* default tx key index */
	void			*ic_wep_ctx;	/* wep crypt context */
	u_int32_t		ic_iv;		/* initial vector for wep */
#ifdef CONFIG_PROC_FS
	char			ic_procname[12];/* e.g. wlan%d */
	struct proc_dir_entry	*ic_proc;	/* /proc/net/wlan%d */
#endif
#ifdef CONFIG_NET_WIRELESS
	struct iw_statistics	ic_iwstats;	/* wireless statistics block */
#endif
};

#define	IEEE80211_SEND_MGMT(ic,ni,type,arg)	do {			      \
	if ((ic)->ic_send_mgmt[(type)>>IEEE80211_FC0_SUBTYPE_SHIFT] != NULL)  \
		(*(ic)->ic_send_mgmt[(type)>>IEEE80211_FC0_SUBTYPE_SHIFT])    \
		    (ic,ni,type,arg);					      \
} while (0)

#define	IEEE80211_ADDR_EQ(a1,a2)	(memcmp(a1,a2,IEEE80211_ADDR_LEN) == 0)
#define	IEEE80211_ADDR_COPY(dst,src)	memcpy(dst,src,IEEE80211_ADDR_LEN)

#define	IEEE80211_IS_MULTICAST(a)	ETHER_IS_MULTICAST(a)

/* ic_flags */
#define	IEEE80211_F_ASCAN	0x00000001	/* STATUS: active scan */
#define	IEEE80211_F_SIBSS	0x00000002	/* STATUS: start IBSS */
#define	IEEE80211_F_WEPON	0x00000100	/* CONF: WEP enabled */
#define	IEEE80211_F_IBSSON	0x00000200	/* CONF: IBSS creation enable */
#define	IEEE80211_F_PMGTON	0x00000400	/* CONF: Power mgmt enable */
#define	IEEE80211_F_DESBSSID	0x00000800	/* CONF: des_bssid is set */
#define	IEEE80211_F_SCANAP	0x00001000	/* CONF: Scanning AP */
#define	IEEE80211_F_ROAMING	0x00002000	/* CONF: roaming enabled */
#define	IEEE80211_F_SWRETRY	0x00004000	/* CONF: sw tx retry enabled */
#define	IEEE80211_F_TXPMGT	0x00018000	/* STATUS: tx power */
#define IEEE80211_F_TXPOW_OFF	0x00000000	/* TX Power: radio disabled */
#define IEEE80211_F_TXPOW_FIXED	0x00008000	/* TX Power: fixed rate */
#define IEEE80211_F_TXPOW_AUTO	0x00010000	/* TX Power: undefined */

/* ic_capabilities */
#define	IEEE80211_C_WEP		0x00000001	/* CAPABILITY: WEP available */
#define	IEEE80211_C_IBSS	0x00000002	/* CAPABILITY: IBSS available */
#define	IEEE80211_C_PMGT	0x00000004	/* CAPABILITY: Power mgmt */
#define	IEEE80211_C_HOSTAP	0x00000008	/* CAPABILITY: HOSTAP avail */
#define	IEEE80211_C_AHDEMO	0x00000010	/* CAPABILITY: Old Adhoc Demo */
#define	IEEE80211_C_SWRETRY	0x00000020	/* CAPABILITY: sw tx retry */
#define	IEEE80211_C_TXPMGT	0x00000040	/* CAPABILITY: tx power mgmt */

/* flags for ieee80211_fix_rate() */
#define	IEEE80211_F_DOSORT	0x00000001	/* sort rate list */
#define	IEEE80211_F_DOFRATE	0x00000002	/* use fixed rate */
#define	IEEE80211_F_DONEGO	0x00000004	/* calc negotiated rate */
#define	IEEE80211_F_DODEL	0x00000008	/* delete ignore rate */

/* private extensions to netdevice.h's netif_msg* mechanism */
#define	NETIF_MSG_DEBUG		0x80000000	/* IFF_DEBUG equivalent */
#define	NETIF_MSG_LINK2		0x40000000	/* IFF_LINK2 equivalant */
#define	netif_msg_debug(p)	((p)->msg_enable & NETIF_MSG_DEBUG)
#define	netif_msg_dumppkts(p) \
	(((p)->msg_enable & (NETIF_MSG_DEBUG|NETIF_MSG_LINK2)) == \
		(NETIF_MSG_DEBUG|NETIF_MSG_LINK2))

int	ieee80211_ifattach(struct net_device *);
void	ieee80211_ifdetach(struct net_device *);
void	ieee80211_input(struct net_device *, struct sk_buff *, int, u_int32_t);
int	ieee80211_mgmt_output(struct net_device *, struct ieee80211_node *,
		struct sk_buff *, int);
struct sk_buff *ieee80211_encap(struct net_device *, struct sk_buff *);
struct sk_buff *ieee80211_decap(struct net_device *, struct sk_buff *);
void	ieee80211_media_status(struct net_device *, struct ifmediareq *);
void	ieee80211_print_essid(u_int8_t *, int);
void	ieee80211_dump_pkt(u_int8_t *, int, int, int);
void	ieee80211_next_scan(struct net_device *);
void	ieee80211_end_scan(struct net_device *);
struct ieee80211_node *ieee80211_alloc_node(struct ieee80211com *, u_int8_t *);
struct ieee80211_node *ieee80211_dup_bss(struct ieee80211com *, u_int8_t *);
struct ieee80211_node *ieee80211_find_node(struct ieee80211com *, u_int8_t *);
void	ieee80211_free_node(struct ieee80211com *, struct ieee80211_node *);
typedef void ieee80211_iter_func(void *, struct ieee80211_node *);
void	ieee80211_iterate_nodes(struct ieee80211com *ic,
		ieee80211_iter_func *, void *);
int	ieee80211_fix_rate(struct ieee80211com *, struct ieee80211_node *, int);
int	ieee80211_new_state(struct net_device *, enum ieee80211_state, int);
struct sk_buff *ieee80211_wep_crypt(struct net_device *, struct sk_buff *, int);
int	ieee80211_rate2media(int, enum ieee80211_phytype);
int	ieee80211_media2rate(int, enum ieee80211_phytype);
u_int	ieee80211_ghz2ieee(u_int, u_int);
u_int	ieee80211_chan2ieee(struct ieee80211com *, struct ieee80211channel *);
u_int	ieee80211_ieee2ghz(u_int, u_int);
struct ieee80211channel *ieee80211_chan_find(struct ieee80211com *, u_int);

extern	const char *ether_sprintf(const u_int8_t *);		/* XXX */

#endif /* __KERNEL__ */

#endif /* _NET_IF_IEEE80211_H_ */
