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
 * Wireless extensions support for 802.11 common code.
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/utsname.h>
#include <linux/if_arp.h>		/* XXX for ARPHRD_ETHER */

#include <net/iw_handler.h>

#include <asm/uaccess.h>

#include "if_ieee80211.h"

#ifdef CONFIG_NET_WIRELESS

struct iw_statistics *
ieee80211_iw_getstats(struct net_device *dev)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	ic->ic_iwstats.qual.qual = 0;
	ic->ic_iwstats.qual.level = ic->ic_bss.ni_rssi;
	ic->ic_iwstats.qual.noise = 0;
	ic->ic_iwstats.qual.updated = 0;

	ic->ic_iwstats.status = ic->ic_state;
	ic->ic_iwstats.discard.nwid = 0;
	ic->ic_iwstats.discard.code = 0;
	ic->ic_iwstats.discard.fragment = 0;
	ic->ic_iwstats.discard.retries = 0;
	ic->ic_iwstats.discard.misc = 0;

	ic->ic_iwstats.miss.beacon = 0;

	return &ic->ic_iwstats;
}

int
ieee80211_ioctl_giwname(struct net_device *dev,
		   struct iw_request_info *info,
		   char *name, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	/* XXX should use media status but IFM_AUTO case gets tricky */
	switch (ic->ic_curmode) {
	case IEEE80211_MODE_11A:
		strncpy(name, "IEEE 802.11-OFDM", IFNAMSIZ);
		break;
	case IEEE80211_MODE_11B:
		strncpy(name, "IEEE 802.11-DS", IFNAMSIZ);
		break;
	case IEEE80211_MODE_11G:
		strncpy(name, "IEEE 802.11-OFDM/DS", IFNAMSIZ);
		break;
	case IEEE80211_MODE_TURBO:
		strncpy(name, "IEEE 802.11-TURBO", IFNAMSIZ);
		break;
	default:
		strncpy(name, "IEEE 802.11", IFNAMSIZ);
		break;
	}
	return 0;
}

int
ieee80211_ioctl_siwencode(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_point *erq, char *keybuf)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	int kid, error;

	if (erq->flags & IW_ENCODE_DISABLED) {
		ic->ic_flags &= ~IEEE80211_F_WEPON;
		error = ENETRESET;
		goto done;
	}
	if ((ic->ic_caps & IEEE80211_C_WEP) == 0) {
		error = EOPNOTSUPP;
		goto done;
	}

	kid = erq->flags & IW_ENCODE_INDEX;
	if (kid >= IEEE80211_WEP_NKID) {
		error = EINVAL;
		goto done;
	}
	if (erq->length > 0) {
		if (erq->length > IEEE80211_KEYBUF_SIZE) {
			error = EINVAL;
			goto done;
		}
		memcpy(ic->ic_nw_keys[kid].wk_key, keybuf, erq->length);
		memset(ic->ic_nw_keys[kid].wk_key + erq->length, 0,
			IEEE80211_KEYBUF_SIZE - erq->length);
		ic->ic_nw_keys[kid].wk_len = erq->length;
	}
	ic->ic_wep_txkey = kid;
	ic->ic_flags |= IEEE80211_F_WEPON;
	error = ENETRESET;
done:
	if (error == ENETRESET)
		error = (*ic->ic_init)(dev);	/* reset mode */
	return -error;
}

int
ieee80211_ioctl_giwencode(struct net_device *dev,
			  struct iw_request_info *info,
			  struct iw_point *erq, char *key)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if ((ic->ic_caps & IEEE80211_C_WEP) == 0) {
		erq->length = 0;
		erq->flags = IW_ENCODE_DISABLED;
	} else {
		int kid = erq->flags & IW_ENCODE_INDEX;
		if (kid >= IEEE80211_WEP_NKID)
			return -EINVAL;
		erq->flags = kid + 1;
		if (erq->length > ic->ic_nw_keys[kid].wk_len)
			erq->length = ic->ic_nw_keys[kid].wk_len;
		memcpy(key, ic->ic_nw_keys[kid].wk_key, erq->length);
		erq->flags |= IW_ENCODE_ENABLED;	/* XXX */
		erq->flags |= IW_ENCODE_OPEN;		/* XXX */
	}
	return 0;
}

#ifndef ifr_media
#define	ifr_media	ifr_ifru.ifru_ivalue
#endif

int
ieee80211_ioctl_siwrate(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ifmediareq imr;
	struct ifreq ifr;
	int rate;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(dev, &imr);

	if (rrq->fixed) {
		/* XXX fudge checking rates */
		rate = ieee80211_rate2media(ic, 2 * (rrq->value / 1000000),
				ic->ic_curmode);
	} else
		rate = IFM_AUTO;
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_media = (imr.ifm_active & ~IFM_TMASK) | IFM_SUBTYPE(rate);

	return -ifmedia_ioctl(dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
}

int
ieee80211_ioctl_giwrate(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ifmediareq imr;
	int rate;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(dev, &imr);

	rrq->fixed = IFM_SUBTYPE(ic->ic_media.ifm_media) != IFM_AUTO;
	/* media status will have the current xmit rate if available */
	rate = ieee80211_media2rate(imr.ifm_active);
	if (rate == -1)		/* IFM_AUTO */
		rate = 0;
	rrq->value = 1000000 * (rate / 2);

	return 0;
}

int
ieee80211_ioctl_siwsens(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *sens, char *extra)
{
	return 0;
}

int
ieee80211_ioctl_giwsens(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *sens, char *extra)
{
	sens->value = 0;
	sens->fixed = 1;

	return 0;
}

int
ieee80211_ioctl_siwrts(struct net_device *dev,
		       struct iw_request_info *info,
		       struct iw_param *rts, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	u16 val;

	if (rts->disabled)
		val = IEEE80211_RTS_MAX;
	else if (IEEE80211_RTS_MIN < rts->value || rts->value > IEEE80211_RTS_MAX)
		return -EINVAL;
	else
		val = rts->value;
	if (val != ic->ic_rtsthreshold) {
		ic->ic_rtsthreshold = val;
		return -(*ic->ic_init)(dev);
	} else {
		return 0;
	}
}

int
ieee80211_ioctl_giwrts(struct net_device *dev,
		       struct iw_request_info *info,
		       struct iw_param *rts, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	rts->value = ic->ic_rtsthreshold;
	rts->disabled = (rts->value == IEEE80211_RTS_MAX);
	rts->fixed = 1;

	return 0;
}


int
ieee80211_ioctl_siwfrag(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *rts, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	u16 val;

	if (rts->disabled)
		val = __constant_cpu_to_le16(2346);
	else if (rts->value < 256 || rts->value > 2346)
		return -EINVAL;
	else
		val = __cpu_to_le16(rts->value & ~0x1); /* even numbers only */
	if (val != ic->ic_fragthreshold) {
		ic->ic_fragthreshold = val;
		return -(*ic->ic_init)(dev);
	}
	return 0;
}

int
ieee80211_ioctl_giwfrag(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *rts, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	rts->value = ic->ic_fragthreshold;
	rts->disabled = (rts->value == 2346);
	rts->fixed = 1;

	return 0;
}

int
ieee80211_ioctl_siwap(struct net_device *dev,
		      struct iw_request_info *info,
		      struct sockaddr *ap_addr, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	static const u_int8_t zero_bssid[IEEE80211_ADDR_LEN];

	/* NB: should only be set when in STA mode */
	if (ic->ic_opmode == IEEE80211_M_STA) {
		IEEE80211_ADDR_COPY(ic->ic_des_bssid, &ap_addr->sa_data);
		/* looks like a zero address disables */
		if (IEEE80211_ADDR_EQ(ic->ic_des_bssid, zero_bssid))
			ic->ic_flags &= ~IEEE80211_F_DESBSSID;
		else
			ic->ic_flags |= IEEE80211_F_DESBSSID;
		return -(*ic->ic_init)(dev);
	} else
		return -EINVAL;
}

int
ieee80211_ioctl_giwap(struct net_device *dev,
		      struct iw_request_info *info,
		      struct sockaddr *ap_addr, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (ic->ic_flags & IEEE80211_F_DESBSSID)
		IEEE80211_ADDR_COPY(&ap_addr->sa_data, ic->ic_des_bssid);
	else
		IEEE80211_ADDR_COPY(&ap_addr->sa_data, ic->ic_bss.ni_bssid);
	ap_addr->sa_family = ARPHRD_ETHER;
	return 0;
}

int
ieee80211_ioctl_siwnickn(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *data, char *nickname)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (data->length > IEEE80211_NWID_LEN)
		return -EINVAL;

	memset(ic->ic_nickname, 0, IEEE80211_NWID_LEN);
	memcpy(ic->ic_nickname, nickname, data->length);
	ic->ic_nicknamelen = data->length;

	return 0;
}

int
ieee80211_ioctl_giwnickn(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *data, char *nickname)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (data->length > ic->ic_nicknamelen + 1)
		data->length = ic->ic_nicknamelen + 1;
	if (data->length > 0) {
		memcpy(nickname, ic->ic_nickname, data->length-1);
		nickname[data->length-1] = '\0';
	}
	return 0;
}

int
ieee80211_ioctl_siwfreq(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_freq *freq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ieee80211channel *c;
	int i;
	
	if (freq->e > 1)
		return -EINVAL;
	if (freq->e == 1)
		i = ieee80211_mhz2ieee(freq->m / 100000, 0);
	else
		i = freq->m;
	if (i > IEEE80211_CHAN_MAX || isclr(ic->ic_chan_active, i))
		return -EINVAL;
	c = &ic->ic_channels[i];
	if (c == ic->ic_ibss_chan)	/* no change, just return */
		return 0;
	ic->ic_ibss_chan = ic->ic_des_chan = c;
	return -(*ic->ic_init)(dev);
}

int
ieee80211_ioctl_giwfreq(struct net_device *dev,
				struct iw_request_info *info,
				struct iw_freq *freq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (!ic->ic_ibss_chan)
		return -EINVAL;

	freq->m = ic->ic_ibss_chan->ic_freq * 100000;
	freq->e = 1;

	return 0;
}

int
ieee80211_ioctl_siwessid(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *data, char *ssid)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (data->flags == 0) {		/* ANY */
		memset(ic->ic_des_essid, 0, IEEE80211_NWID_LEN);
		ic->ic_des_esslen = 0;
	} else {
		int i, len;

		len = IW_ESSID_MAX_SIZE;
		if (len > IEEE80211_NWID_LEN)
			len = IEEE80211_NWID_LEN;
		memcpy(ic->ic_des_essid, ssid, len);
		/* XXX: scan for \0 to calculate the length */
		for (i = 0; i < len && ic->ic_des_essid[i] != '\0'; i++)
			;
		ic->ic_des_esslen = i;
	}
	return -(*ic->ic_init)(dev);
}

int
ieee80211_ioctl_giwessid(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *data, char *essid)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	data->flags = 1;		/* active */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		if (data->length > ic->ic_des_esslen)
			data->length = ic->ic_des_esslen;
		memcpy(essid, ic->ic_des_essid, data->length);
	} else {
		if (data->length > ic->ic_bss.ni_esslen)
			data->length = ic->ic_bss.ni_esslen;
		memcpy(essid, ic->ic_bss.ni_essid, data->length);
	}
	return 0;
}

int
ieee80211_ioctl_giwrange(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_point *data, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ieee80211_node *ni = &ic->ic_bss;
	struct iw_range *range = (struct iw_range *) extra;
	struct ieee80211_rateset *rs;
	int i, r;

	data->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(struct iw_range));

	/* TODO: could fill num_txpower and txpower array with
	 * something; however, there are 128 different values.. */

	range->txpower_capa = IW_TXPOW_DBM;

	if (ic->ic_opmode == IEEE80211_M_STA || ic->ic_opmode == IEEE80211_M_IBSS) {
		range->min_pmp = 1 * 1024;
		range->max_pmp = 65535 * 1024;
		range->min_pmt = 1 * 1024;
		range->max_pmt = 1000 * 1024;
		range->pmp_flags = IW_POWER_PERIOD;
		range->pmt_flags = IW_POWER_TIMEOUT;
		range->pm_capa = IW_POWER_PERIOD | IW_POWER_TIMEOUT |
			IW_POWER_UNICAST_R | IW_POWER_ALL_R;
	}

	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 13;

	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT;
	range->min_retry = 0;
	range->max_retry = 255;

	range->num_channels = IEEE80211_CHAN_MAX;	/* XXX */

	range->num_frequency = 0;
	for (i = 0; i <= IEEE80211_CHAN_MAX; i++)
		if (isset(ic->ic_chan_active, i)) {
			range->freq[range->num_frequency].i = i;
			range->freq[range->num_frequency].m =
				ic->ic_channels[i].ic_freq * 100000;
			range->freq[range->num_frequency].e = 1;
			if (++range->num_frequency == IW_MAX_FREQUENCIES)
				break;
		}

	range->max_qual.qual = 92; /* 0 .. 92 */
	range->max_qual.level = 154; /* 27 .. 154 */
	range->max_qual.noise = 154; /* 27 .. 154 */
	range->sensitivity = 3;

	range->max_encoding_tokens = IEEE80211_WEP_NKID;
	range->num_encoding_sizes = 2;
	range->encoding_size[0] = 5;
	range->encoding_size[1] = 13;

	/* XXX this only works for station mode */
	rs = &ni->ni_rates;
	range->num_bitrates = rs->rs_nrates;
	if (range->num_bitrates > IW_MAX_BITRATES)
		range->num_bitrates = IW_MAX_BITRATES;
	for (i = 0; i < range->num_bitrates; i++) {
		r = rs->rs_rates[i] & IEEE80211_RATE_VAL;
		range->bitrate[i] = (r / 2) * 1000000;
	}

	/* estimated maximum TCP throughput values (bps) */
	range->throughput = 5500000;

	range->min_rts = 0;
	range->max_rts = 2347;
	range->min_frag = 256;
	range->max_frag = 2346;

	return 0;
}

int
ieee80211_ioctl_siwmode(struct net_device *dev,
			struct iw_request_info *info,
			__u32 *mode, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ifmediareq imr;
	struct ifreq ifr;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(dev, &imr);

	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_media = imr.ifm_active &~ IFM_OMASK;
	switch (*mode) {
	case IW_MODE_INFRA:
		/* NB: this is the default */
		break;
	case IW_MODE_ADHOC:
		ifr.ifr_media |= IFM_IEEE80211_IBSS;
		break;
	case IW_MODE_MASTER:
		ifr.ifr_media |= IFM_IEEE80211_HOSTAP;
		break;
	case IW_MODE_MONITOR:
		ifr.ifr_media |= IFM_IEEE80211_MONITOR;
		break;
	default:
		return -EINVAL;
	}
	return -ifmedia_ioctl(dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
}

int
ieee80211_ioctl_giwmode(struct net_device *dev,
			struct iw_request_info *info,
			__u32 *mode, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ifmediareq imr;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(dev, &imr);

	if (imr.ifm_active & IFM_IEEE80211_HOSTAP)
		*mode = IW_MODE_MASTER;
	else if (imr.ifm_active & IFM_IEEE80211_MONITOR)
		*mode = IW_MODE_MONITOR;
	else if (imr.ifm_active & IFM_IEEE80211_IBSS)
		*mode = IW_MODE_ADHOC;
	else
		*mode = IW_MODE_INFRA;
	return 0;
}

int
ieee80211_ioctl_siwpower(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_param *wrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (wrq->disabled) {
		if (ic->ic_flags & IEEE80211_F_PMGTON) {
			ic->ic_flags &= ~IEEE80211_F_PMGTON;
			goto done;
		}
		return 0;
	}

	if ((ic->ic_caps & IEEE80211_C_PMGT) == 0)
		return -EOPNOTSUPP;
	switch (wrq->flags & IW_POWER_MODE) {
	case IW_POWER_UNICAST_R:
	case IW_POWER_ALL_R:
	case IW_POWER_ON:
		ic->ic_flags |= IEEE80211_F_PMGTON;
		break;
	default:
		return -EINVAL;
	}
	if (wrq->flags & IW_POWER_TIMEOUT) {
		ic->ic_holdover = wrq->value / 1024;
		ic->ic_flags |= IEEE80211_F_PMGTON;
	}
	if (wrq->flags & IW_POWER_PERIOD) {
		ic->ic_lintval = wrq->value / 1024;
		ic->ic_flags |= IEEE80211_F_PMGTON;
	}
done:
	return -(*ic->ic_init)(dev);
}

int
ieee80211_ioctl_giwpower(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	rrq->disabled = (ic->ic_flags & IEEE80211_F_PMGTON) == 0;
	if (!rrq->disabled) {
		switch (rrq->flags & IW_POWER_TYPE) {
		case IW_POWER_TIMEOUT:
			rrq->flags = IW_POWER_TIMEOUT;
			rrq->value = ic->ic_holdover * 1024;
			break;
		case IW_POWER_PERIOD:
			rrq->flags = IW_POWER_PERIOD;
			rrq->value = ic->ic_lintval * 1024;
			break;
		}
		rrq->flags |= IW_POWER_ALL_R;
	}
	return 0;
}

int
ieee80211_ioctl_siwretry(struct net_device *dev,
				 struct iw_request_info *info,
				 struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (rrq->disabled) {
		if (ic->ic_flags & IEEE80211_F_SWRETRY) {
			ic->ic_flags &= ~IEEE80211_F_SWRETRY;
			goto done;
		}
		return 0;
	}

	if ((ic->ic_caps & IEEE80211_C_SWRETRY) == 0)
		return -EOPNOTSUPP;
	if (rrq->flags == IW_RETRY_LIMIT) {
		if (rrq->value >= 0) {
			ic->ic_txmin = rrq->value;
			ic->ic_txmax = rrq->value;	/* XXX */
			ic->ic_txlifetime = 0;		/* XXX */
			ic->ic_flags |= IEEE80211_F_SWRETRY;
		} else {
			ic->ic_flags &= ~IEEE80211_F_SWRETRY;
		}
		return 0;
	}
done:
	return -(*ic->ic_init)(dev);
}

int
ieee80211_ioctl_giwretry(struct net_device *dev,
				 struct iw_request_info *info,
				 struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	rrq->disabled = (ic->ic_flags & IEEE80211_F_SWRETRY) == 0;
	if (!rrq->disabled) {
		switch (rrq->flags & IW_RETRY_TYPE) {
		case IW_RETRY_LIFETIME:
			rrq->flags = IW_RETRY_LIFETIME;
			rrq->value = ic->ic_txlifetime * 1024;
			break;
		case IW_RETRY_LIMIT:
			rrq->flags = IW_RETRY_LIMIT;
			switch (rrq->flags & IW_RETRY_MODIFIER) {
			case IW_RETRY_MIN:
				rrq->flags |= IW_RETRY_MAX;
				rrq->value = ic->ic_txmin;
				break;
			case IW_RETRY_MAX:
				rrq->flags |= IW_RETRY_MAX;
				rrq->value = ic->ic_txmax;
				break;
			}
			break;
		}
	}
	return 0;
}

int
ieee80211_ioctl_siwtxpow(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	int curtxpmgt;

	curtxpmgt = ic->ic_flags & IEEE80211_F_TXPMGT;
	if (rrq->disabled) {
		if (curtxpmgt != IEEE80211_F_TXPOW_OFF) {
			if ((ic->ic_caps & IEEE80211_C_TXPMGT) == 0)
				return -EOPNOTSUPP;
			ic->ic_flags &= ~IEEE80211_F_TXPMGT;
			ic->ic_flags |= IEEE80211_F_TXPOW_OFF;
			goto done;
		}
		return 0;
	}

	if (rrq->fixed) {
		if ((ic->ic_caps & IEEE80211_C_TXPMGT) == 0)
			return -EOPNOTSUPP;
		if (rrq->flags != IW_TXPOW_DBM)
			return -EOPNOTSUPP;
		ic->ic_txpower = rrq->value;
		if (curtxpmgt != IEEE80211_F_TXPOW_FIXED) {
			ic->ic_flags &= ~IEEE80211_F_TXPMGT;
			ic->ic_flags |= IEEE80211_F_TXPOW_FIXED;
		}
	} else {
		if (curtxpmgt == IEEE80211_F_TXPOW_AUTO)
			return 0;
		/* NB: auto is always supported */
		ic->ic_flags &= ~IEEE80211_F_TXPMGT;
		ic->ic_flags |= IEEE80211_F_TXPOW_AUTO;
	}
done:
	return -(*ic->ic_init)(dev);
}

int
ieee80211_ioctl_giwtxpow(struct net_device *dev,
			 struct iw_request_info *info,
			 struct iw_param *rrq, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	u_int32_t curtxmgt = ic->ic_flags & IEEE80211_F_TXPMGT;

	rrq->disabled = (curtxmgt == IEEE80211_F_TXPOW_OFF);
	rrq->fixed = (curtxmgt == IEEE80211_F_TXPOW_FIXED);
	rrq->flags = IW_TXPOW_DBM;
	rrq->value = ic->ic_txpower;
	return 0;
}

int
ieee80211_ioctl_iwaplist(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ieee80211_node *ni;
	struct sockaddr addr[IW_MAX_AP];
	struct iw_quality qual[IW_MAX_AP];
	int i;

	i = 0;
	/* XXX lock node list */
	TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
		addr[i].sa_family = ARPHRD_ETHER;
		if (ic->ic_opmode == IEEE80211_M_HOSTAP)
			IEEE80211_ADDR_COPY(addr[i].sa_data, ni->ni_macaddr);
		else
			IEEE80211_ADDR_COPY(addr[i].sa_data, ni->ni_bssid);
		qual[i].qual = ni->ni_rssi;
		qual[i].level = 0;
		qual[i].noise = 0;
		qual[i].updated = jiffies;		/* XXX */
		if (++i >= IW_MAX_AP)
			break;
	}
	data->length = i;
	memcpy(extra, &addr, i*sizeof(addr[0]));
	data->flags = 1;		/* signal quality present (sort of) */
	memcpy(extra + i*sizeof(addr[0]), &qual, i*sizeof(qual[i]));

	return 0;

}

#ifdef SIOCGIWSCAN
int
ieee80211_ioctl_siwscan(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;

	if (ic->ic_opmode != IEEE80211_M_HOSTAP) {
		/* just return existing station tables */
		data->length = 0;
	} else {
#ifdef notyet
		/* XXX honor scan request flags */
		HAL_CHANNEL *c = sc->sc_cur_chan;
		ni->ni_chan = ath_hal_ghz2ieee(c->channel, c->channelFlags);
		ieee80211_new_state(dev, IEEE80211_S_SCAN, -1);
#endif
	}
	return 0;
}

int
ieee80211_ioctl_giwscan(struct net_device *dev,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ieee80211_node *ni;
	char *current_ev = extra;
	char *end_buf = extra + IW_SCAN_MAX_DATA;
	char *current_val;
	struct iw_event iwe;
	int j;

	if (ic->ic_state == IEEE80211_S_SCAN &&
	    (ic->ic_flags & (IEEE80211_F_SCANAP | IEEE80211_F_ASCAN))) {
		/*
		 * Still scanning, indicate the caller should try again.
		 */
		return -EAGAIN;
	}

	/*
	 * Translate data to WE format.
	 */
	TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
		if (current_ev >= end_buf)
			break;
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			IEEE80211_ADDR_COPY(iwe.u.ap_addr.sa_data,
				ni->ni_macaddr);
			current_ev = iwe_stream_add_event(current_ev,
				end_buf, &iwe, IW_EV_ADDR_LEN);

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = SIOCGIWMODE;
			iwe.u.mode = IW_MODE_INFRA;
			current_ev = iwe_stream_add_event(current_ev,
				end_buf, &iwe, IW_EV_UINT_LEN);

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = SIOCGIWESSID;
			iwe.u.data.length = ic->ic_des_esslen;
			iwe.u.data.flags = 1;
			current_ev = iwe_stream_add_point(current_ev,
				end_buf, &iwe, ic->ic_des_essid);
		} else {
			IEEE80211_ADDR_COPY(iwe.u.ap_addr.sa_data,
				ni->ni_bssid);
			current_ev = iwe_stream_add_event(current_ev,
				end_buf, &iwe, IW_EV_ADDR_LEN);

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = SIOCGIWMODE;
			iwe.u.mode = IW_MODE_MASTER;
			current_ev = iwe_stream_add_event(current_ev,
				end_buf, &iwe, IW_EV_UINT_LEN);

			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = SIOCGIWENCODE;
			iwe.u.data.length = ni->ni_esslen;
			if (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY)
				iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
			else
				iwe.u.data.flags = IW_ENCODE_DISABLED;
			current_ev = iwe_stream_add_point(current_ev,
				end_buf, &iwe, ni->ni_essid);
		}
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.level = ni->ni_rssi;
		iwe.u.qual.updated = jiffies;	/* XXX */
		current_ev = iwe_stream_add_event(current_ev,
			end_buf, &iwe, IW_EV_QUAL_LEN);

		if (ni->ni_capinfo & (IEEE80211_CAPINFO_ESS|IEEE80211_CAPINFO_IBSS)) {
			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = SIOCGIWMODE;
			iwe.u.mode = ni->ni_capinfo & IEEE80211_CAPINFO_ESS ?
				IW_MODE_MASTER : IW_MODE_ADHOC;
			current_ev = iwe_stream_add_event(current_ev,
					end_buf, &iwe, IW_EV_UINT_LEN);
		}

		/* return essid if present */
		if (ni->ni_essid[0]) {	
			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = SIOCGIWESSID;
			iwe.u.data.length = strlen(ni->ni_essid);
			iwe.u.data.flags = 1;
			current_ev = iwe_stream_add_point(current_ev,
					end_buf, &iwe, ni->ni_essid);
		}

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWFREQ;
		iwe.u.freq.m = ni->ni_chan->ic_freq * 100000;
		iwe.u.freq.e = 1;
		current_ev = iwe_stream_add_event(current_ev,
				end_buf, &iwe, IW_EV_FREQ_LEN);
#ifdef notdef
		ap->interval = ni->ni_intval;
#endif
		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWRATE;
		current_val = current_ev + IW_EV_LCP_LEN;
		for (j = 0; j < ni->ni_rates.rs_nrates; j++) {
			if (ni->ni_rates.rs_rates[j]) {
				iwe.u.bitrate.value = ((ni->ni_rates.rs_rates[j] &
				    IEEE80211_RATE_VAL) / 2) * 1000000;
				current_val = iwe_stream_add_value(current_ev,
					current_val, end_buf, &iwe,
					IW_EV_PARAM_LEN);
			}
		}
		if ((current_val - current_ev) > IW_EV_LCP_LEN)
			current_ev = current_val;
	}
	data->length = current_ev - extra;
	return 0;
}
#endif /* SIOCGIWSCAN */

static int
ieee80211_ioctl_setparam(struct net_device *dev, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	int *i = (int *) extra;
	int param = i[0];		/* parameter id is 1st */
	int value = i[1];		/* NB: all values are TYPE_INT */
	struct ifmediareq imr;
	struct ifreq ifr;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(dev, &imr);

	memset(&ifr, 0, sizeof(ifr));

	switch (param) {
	case IEEE80211_PARAM_TURBO:
		if (value)
			imr.ifm_active |= IFM_IEEE80211_TURBO;
		else
			imr.ifm_active &= ~IFM_IEEE80211_TURBO;
		ifr.ifr_media = imr.ifm_active;
		break;
	case IEEE80211_PARAM_MODE:
		ifr.ifr_media = (imr.ifm_active &~ IFM_MMASK)
			      | IFM_MAKEMODE(value);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return -ifmedia_ioctl(dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);

	return 0;
}

static int
ieee80211_ioctl_getparam(struct net_device *dev, struct iw_request_info *info,
		   	void *w, char *extra)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	struct ifmediareq imr;
	int *param = (int *) extra;

	(*ic->ic_media.ifm_status)(dev, &imr);
	switch (param[0]) {
	case IEEE80211_PARAM_TURBO:
		param[0] = (imr.ifm_active & IFM_IEEE80211_TURBO) != 0;
		break;
	case IEEE80211_PARAM_MODE:
		param[0] = IFM_MODE(imr.ifm_active);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/* Structures to export the Wireless Handlers */
static const iw_handler ieee80211_handlers[] = {
	(iw_handler) NULL,				/* SIOCSIWCOMMIT */
	(iw_handler) ieee80211_ioctl_giwname,		/* SIOCGIWNAME */
	(iw_handler) NULL,				/* SIOCSIWNWID */
	(iw_handler) NULL,				/* SIOCGIWNWID */
	(iw_handler) ieee80211_ioctl_siwfreq,		/* SIOCSIWFREQ */
	(iw_handler) ieee80211_ioctl_giwfreq,		/* SIOCGIWFREQ */
	(iw_handler) ieee80211_ioctl_siwmode,		/* SIOCSIWMODE */
	(iw_handler) ieee80211_ioctl_giwmode,		/* SIOCGIWMODE */
	(iw_handler) ieee80211_ioctl_siwsens,		/* SIOCSIWSENS */
	(iw_handler) ieee80211_ioctl_giwsens,		/* SIOCGIWSENS */
	(iw_handler) NULL /* not used */,		/* SIOCSIWRANGE */
	(iw_handler) ieee80211_ioctl_giwrange,		/* SIOCGIWRANGE */
	(iw_handler) NULL /* not used */,		/* SIOCSIWPRIV */
	(iw_handler) NULL /* kernel code */,		/* SIOCGIWPRIV */
	(iw_handler) NULL /* not used */,		/* SIOCSIWSTATS */
	(iw_handler) NULL /* kernel code */,		/* SIOCGIWSTATS */
	(iw_handler) NULL,				/* SIOCSIWSPY */
	(iw_handler) NULL,				/* SIOCGIWSPY */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) ieee80211_ioctl_siwap,		/* SIOCSIWAP */
	(iw_handler) ieee80211_ioctl_giwap,		/* SIOCGIWAP */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) ieee80211_ioctl_iwaplist,		/* SIOCGIWAPLIST */
#ifdef SIOCGIWSCAN
	(iw_handler) ieee80211_ioctl_siwscan,		/* SIOCSIWSCAN */
	(iw_handler) ieee80211_ioctl_giwscan,		/* SIOCGIWSCAN */
#else
	(iw_handler) NULL,				/* SIOCSIWSCAN */
	(iw_handler) NULL,				/* SIOCGIWSCAN */
#endif /* SIOCGIWSCAN */
	(iw_handler) ieee80211_ioctl_siwessid,		/* SIOCSIWESSID */
	(iw_handler) ieee80211_ioctl_giwessid,		/* SIOCGIWESSID */
	(iw_handler) ieee80211_ioctl_siwnickn,		/* SIOCSIWNICKN */
	(iw_handler) ieee80211_ioctl_giwnickn,		/* SIOCGIWNICKN */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) NULL,				/* -- hole -- */
	(iw_handler) ieee80211_ioctl_siwrate,		/* SIOCSIWRATE */
	(iw_handler) ieee80211_ioctl_giwrate,		/* SIOCGIWRATE */
	(iw_handler) ieee80211_ioctl_siwrts,		/* SIOCSIWRTS */
	(iw_handler) ieee80211_ioctl_giwrts,		/* SIOCGIWRTS */
	(iw_handler) ieee80211_ioctl_siwfrag,		/* SIOCSIWFRAG */
	(iw_handler) ieee80211_ioctl_giwfrag,		/* SIOCGIWFRAG */
	(iw_handler) ieee80211_ioctl_siwtxpow,		/* SIOCSIWTXPOW */
	(iw_handler) ieee80211_ioctl_giwtxpow,		/* SIOCGIWTXPOW */
	(iw_handler) ieee80211_ioctl_siwretry,		/* SIOCSIWRETRY */
	(iw_handler) ieee80211_ioctl_giwretry,		/* SIOCGIWRETRY */
	(iw_handler) ieee80211_ioctl_siwencode,		/* SIOCSIWENCODE */
	(iw_handler) ieee80211_ioctl_giwencode,		/* SIOCGIWENCODE */
	(iw_handler) ieee80211_ioctl_siwpower,		/* SIOCSIWPOWER */
	(iw_handler) ieee80211_ioctl_giwpower,		/* SIOCGIWPOWER */
};
static const iw_handler ieee80211_priv_handlers[] = {
	(iw_handler) ieee80211_ioctl_setparam,		/* SIOCWFIRSTPRIV+0 */
	(iw_handler) ieee80211_ioctl_getparam,		/* SIOCWFIRSTPRIV+1 */
};
static const struct iw_priv_args ieee80211_priv_args[] = {
	{ IEEE80211_IOCTL_SETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "setparam" },
#if WIRELESS_EXT >= 12
	/*
	 * These depends on sub-ioctl support which added in version 12.
	 */
	{ IEEE80211_IOCTL_GETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "getparam" },
	/* sub-ioctl handlers */
	{ IEEE80211_IOCTL_SETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "" },
	{ IEEE80211_IOCTL_GETPARAM,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "" },
	/* sub-ioctl definitions */
	{ IEEE80211_PARAM_TURBO,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "turbo" },
	{ IEEE80211_PARAM_TURBO,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_turbo" },
	{ IEEE80211_PARAM_MODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mode" },
	{ IEEE80211_PARAM_MODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mode" },
#endif /* WIRELESS_EXT >= 12 */
};

const struct iw_handler_def ieee80211_iw_handler_def = {
#define	N(a)	(sizeof (a) / sizeof (a[0]))
	.standard		= (iw_handler *) ieee80211_handlers,
	.num_standard		= N(ieee80211_handlers),
	.private		= (iw_handler *) ieee80211_priv_handlers,
	.num_private		= N(ieee80211_priv_handlers),
	.private_args		= (struct iw_priv_args *) ieee80211_priv_args,
	.num_private_args	= N(ieee80211_priv_args),
#undef N
};
#endif /* CONFIG_NET_WIRELESS */
