/*-
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
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
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * Wireless extensions support for 802.11 common code.
 */
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/netdevice.h>

#include <linux/utsname.h>
#include <linux/if_arp.h>		/* XXX for ARPHRD_ETHER */

#include <net/iw_handler.h>

#include <asm/uaccess.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

#ifdef CONFIG_NET_WIRELESS
#include <linux/wireless.h>

#define	IS_UP(_dev) \
	(((_dev)->flags & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))

/*
 * Units are in db above the noise floor. That means the
 * rssi values reported in the tx/rx descriptors in the
 * driver are the SNR expressed in db.
 *
 * If you assume that the noise floor is -95, which is an
 * excellent assumption 99.5 % of the time, then you can
 * derive the absolute signal level (i.e. -95 + rssi). 
 * There are some other slight factors to take into account
 * depending on whether the rssi measurement is from 11b,
 * 11g, or 11a.   These differences are at most 2db and
 * can be documented.
 *
 * NB: various calculations are based on the orinoco/wavelan
 *     drivers for compatibility
 */
static void
set_quality(struct iw_quality *iq, u_int rssi)
{
	iq->qual = rssi;
	/* NB: max is 94 because noise is hardcoded to 161 */
	if (iq->qual > 94)
		iq->qual = 94;

	iq->noise = 161;		/* -95dBm */
	iq->level = iq->noise + iq->qual;
	iq->updated = 7;
}

void
ieee80211_iw_getstats(struct ieee80211com *ic, struct iw_statistics *is)
{
#define	NZ(x)	((x) ? (x) : 1)

	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		/* use stats from associated ap */
		if (ic->ic_bss)
			set_quality(&is->qual,
				(*ic->ic_node_getrssi)(ic, ic->ic_bss));
		else
			set_quality(&is->qual, 0);
		break;
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
	case IEEE80211_M_HOSTAP: {
		struct ieee80211_node* ni;
		u_int32_t rssi_samples = 0;
		u_int32_t rssi_total = 0;

		/* average stats from all nodes */
		TAILQ_FOREACH(ni, &ic->ic_node, ni_list) {
			rssi_samples++;
			rssi_total += (*ic->ic_node_getrssi)(ic, ni);
		}
		set_quality(&is->qual, rssi_total / NZ(rssi_samples));
		break;
	}
	case IEEE80211_M_MONITOR:
	default:
		/* no stats */
		set_quality(&is->qual, 0);
		break;
	}
	is->status = ic->ic_state;
	is->discard.nwid = ic->ic_stats.is_rx_wrongbss
			 + ic->ic_stats.is_rx_ssidmismatch;
	is->discard.code = ic->ic_stats.is_rx_wepfail
			 + ic->ic_stats.is_rx_decryptcrc;
	is->discard.fragment = 0;
	is->discard.retries = 0;
	is->discard.misc = 0;

	is->miss.beacon = 0;
#undef NZ
}
EXPORT_SYMBOL(ieee80211_iw_getstats);

int
ieee80211_ioctl_giwname(struct ieee80211com *ic,
		   struct iw_request_info *info,
		   char *name, char *extra)
{

	/* XXX should use media status but IFM_AUTO case gets tricky */
	switch (ic->ic_curmode) {
	case IEEE80211_MODE_11A:
		strncpy(name, "IEEE 802.11a", IFNAMSIZ);
		break;
	case IEEE80211_MODE_11B:
		strncpy(name, "IEEE 802.11b", IFNAMSIZ);
		break;
	case IEEE80211_MODE_11G:
		strncpy(name, "IEEE 802.11g", IFNAMSIZ);
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
EXPORT_SYMBOL(ieee80211_ioctl_giwname);

/*
 * Get a key index from a request.  If nothing is
 * specified in the request we use the current xmit
 * key index.  Otherwise we just convert the index
 * to be base zero.
 */
static int
getiwkeyix(struct ieee80211com *ic, const struct iw_point* erq, int *kix)
{
	int kid;

	kid = erq->flags & IW_ENCODE_INDEX;
	if (kid < 1 || kid > IEEE80211_WEP_NKID) {
		kid = ic->ic_wep_txkey;
		if (kid == IEEE80211_KEYIX_NONE)
			kid = 0;
	} else
		--kid;
	if (0 <= kid && kid < IEEE80211_WEP_NKID) {
		*kix = kid;
		return 0;
	} else
		return EINVAL;
}

int
ieee80211_ioctl_siwencode(struct ieee80211com *ic,
			  struct iw_request_info *info,
			  struct iw_point *erq, char *keybuf)
{
	int kid, error, wepchange;

	if ((erq->flags & IW_ENCODE_DISABLED) == 0) {
		/*
		 * Enable crypto.  The device must support
		 * it and any specified key must be valid.
		 */
		if ((ic->ic_caps & IEEE80211_C_WEP) == 0)
			return -EOPNOTSUPP;
		error = getiwkeyix(ic, erq, &kid);
		if (error)
			return -error;
		if (erq->length > IEEE80211_KEYBUF_SIZE)
			return -EINVAL;
		/* XXX no way to install 0-length key */
		if (erq->length > 0) {
			memcpy(ic->ic_nw_keys[kid].wk_key, keybuf, erq->length);
			memset(ic->ic_nw_keys[kid].wk_key + erq->length, 0,
				IEEE80211_KEYBUF_SIZE - erq->length);
			ic->ic_nw_keys[kid].wk_len = erq->length;
		} else {
			/* verify the key to be installed is non-zero length */
			if (ic->ic_nw_keys[kid].wk_len == 0)
				return -EINVAL;
		}
		ic->ic_wep_txkey = kid;
		wepchange = (ic->ic_flags & IEEE80211_F_WEPON) == 0;
		ic->ic_flags |= IEEE80211_F_WEPON | IEEE80211_F_PRIVACY;
	} else {
		if ((ic->ic_flags & IEEE80211_F_WEPON) == 0)
			return 0;
		ic->ic_flags &= ~IEEE80211_F_WEPON;
		/* turn off privacy if no crypto algorithms in use */
		if ((ic->ic_flags & IEEE80211_F_CRYPTON) == 0)
			ic->ic_flags &= ~IEEE80211_F_PRIVACY;
		wepchange = 1;
		error = 0;
	}
	KASSERT(error == 0, ("something wrong, error %d", error));
	if (IS_UP(ic->ic_dev)) {
		/*
		 * Device is up and running; we must kick it to
		 * effect the change.  If we're enabling/disabling
		 * crypto use then we must re-initialize the device
		 * so the 802.11 state machine is reset.  Otherwise
		 * we only need to reset the hardware state.  By
		 * differentiating the two we allow key changes to
		 * be done w/o changing the 802.11 state machine
		 * which is important for applications like 802.1x.
		 */
		if (wepchange)
			error = (*ic->ic_init)(ic->ic_dev);
		else
			error = (*ic->ic_reset)(ic->ic_dev);
	}
	return -error;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwencode);

int
ieee80211_ioctl_giwencode(struct ieee80211com *ic,
			  struct iw_request_info *info,
			  struct iw_point *erq, char *key)
{
	int error, kid;

	if ((ic->ic_flags & IEEE80211_F_WEPON) == 0) {
		erq->length = 0;
		erq->flags = IW_ENCODE_DISABLED;
		error = 0;
	} else if ((error = getiwkeyix(ic, erq, &kid)) == 0) {
		erq->flags = kid + 1;			/* NB: base 1 */
		if (erq->length > ic->ic_nw_keys[kid].wk_len)
			erq->length = ic->ic_nw_keys[kid].wk_len;
		memcpy(key, ic->ic_nw_keys[kid].wk_key, erq->length);
		erq->flags |= IW_ENCODE_ENABLED;	/* XXX */
		erq->flags |= IW_ENCODE_OPEN;		/* XXX */
	}
	return -error;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwencode);

#ifndef ifr_media
#define	ifr_media	ifr_ifru.ifru_ivalue
#endif

int
ieee80211_ioctl_siwrate(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ifreq ifr;
	int rate;

	if (!ic->ic_media.ifm_cur)
		return -EINVAL;
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~ IFM_TMASK;
	if (rrq->fixed) {
		/* XXX fudge checking rates */
		rate = ieee80211_rate2media(ic, 2 * (rrq->value / 1000000),
				ic->ic_curmode);
	} else
		rate = IFM_AUTO;
	ifr.ifr_media |= IFM_SUBTYPE(rate);

	return -ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
}
EXPORT_SYMBOL(ieee80211_ioctl_siwrate);

int
ieee80211_ioctl_giwrate(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{
	struct ifmediareq imr;
	int rate;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);

	rrq->fixed = IFM_SUBTYPE(ic->ic_media.ifm_media) != IFM_AUTO;
	/* media status will have the current xmit rate if available */
	rate = ieee80211_media2rate(imr.ifm_active);
	if (rate == -1)		/* IFM_AUTO */
		rate = 0;
	rrq->value = 1000000 * (rate / 2);

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwrate);

int
ieee80211_ioctl_siwsens(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *sens, char *extra)
{
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwsens);

int
ieee80211_ioctl_giwsens(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *sens, char *extra)
{
	sens->value = 0;
	sens->fixed = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwsens);

int
ieee80211_ioctl_siwrts(struct ieee80211com *ic,
		       struct iw_request_info *info,
		       struct iw_param *rts, char *extra)
{
	u16 val;

	if (rts->disabled)
		val = IEEE80211_RTS_MAX;
	else if (IEEE80211_RTS_MIN <= rts->value &&
	    rts->value <= IEEE80211_RTS_MAX)
		val = rts->value;
	else
		return -EINVAL;
	if (val != ic->ic_rtsthreshold) {
		ic->ic_rtsthreshold = val;
		if (IS_UP(ic->ic_dev))
			return -(*ic->ic_reset)(ic->ic_dev);
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwrts);

int
ieee80211_ioctl_giwrts(struct ieee80211com *ic,
		       struct iw_request_info *info,
		       struct iw_param *rts, char *extra)
{

	rts->value = ic->ic_rtsthreshold;
	rts->disabled = (rts->value == IEEE80211_RTS_MAX);
	rts->fixed = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwrts);

int
ieee80211_ioctl_siwfrag(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rts, char *extra)
{
	u16 val;

	if (rts->disabled)
		val = __constant_cpu_to_le16(2346);
	else if (rts->value < 256 || rts->value > 2346)
		return -EINVAL;
	else
		val = __cpu_to_le16(rts->value & ~0x1); /* even numbers only */
	if (val != ic->ic_fragthreshold) {
		ic->ic_fragthreshold = val;
		if (IS_UP(ic->ic_dev))
			return -(*ic->ic_reset)(ic->ic_dev);
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwfrag);

int
ieee80211_ioctl_giwfrag(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rts, char *extra)
{

	rts->value = ic->ic_fragthreshold;
	rts->disabled = (rts->value == 2346);
	rts->fixed = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwfrag);

int
ieee80211_ioctl_siwap(struct ieee80211com *ic,
		      struct iw_request_info *info,
		      struct sockaddr *ap_addr, char *extra)
{
	static const u_int8_t zero_bssid[IEEE80211_ADDR_LEN];

	/* NB: should only be set when in STA mode */
	if (ic->ic_opmode != IEEE80211_M_STA)
		return -EINVAL;
	IEEE80211_ADDR_COPY(ic->ic_des_bssid, &ap_addr->sa_data);
	/* looks like a zero address disables */
	if (IEEE80211_ADDR_EQ(ic->ic_des_bssid, zero_bssid))
		ic->ic_flags &= ~IEEE80211_F_DESBSSID;
	else
		ic->ic_flags |= IEEE80211_F_DESBSSID;
	return IS_UP(ic->ic_dev) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwap);

int
ieee80211_ioctl_giwap(struct ieee80211com *ic,
		      struct iw_request_info *info,
		      struct sockaddr *ap_addr, char *extra)
{

	if (ic->ic_flags & IEEE80211_F_DESBSSID)
		IEEE80211_ADDR_COPY(&ap_addr->sa_data, ic->ic_des_bssid);
	else
		IEEE80211_ADDR_COPY(&ap_addr->sa_data, ic->ic_bss->ni_bssid);
	ap_addr->sa_family = ARPHRD_ETHER;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwap);

int
ieee80211_ioctl_siwnickn(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *nickname)
{

	if (data->length > IEEE80211_NWID_LEN)
		return -EINVAL;

	memset(ic->ic_nickname, 0, IEEE80211_NWID_LEN);
	memcpy(ic->ic_nickname, nickname, data->length);
	ic->ic_nicknamelen = data->length;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwnickn);

int
ieee80211_ioctl_giwnickn(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *nickname)
{

	if (data->length > ic->ic_nicknamelen + 1)
		data->length = ic->ic_nicknamelen + 1;
	if (data->length > 0) {
		memcpy(nickname, ic->ic_nickname, data->length-1);
		nickname[data->length-1] = '\0';
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwnickn);

static struct ieee80211_channel *
getcurchan(struct ieee80211com *ic)
{
	switch (ic->ic_state) {
	case IEEE80211_S_INIT:
	case IEEE80211_S_SCAN:
		if (ic->ic_opmode == IEEE80211_M_STA)
			return ic->ic_des_chan;
		break;
	default:
		break;
	}
	return ic->ic_ibss_chan;
}

int
ieee80211_ioctl_siwfreq(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_freq *freq, char *extra)
{
	struct ieee80211_channel *c;
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
	if (c == getcurchan(ic)) {	/* no change, just return */
		ic->ic_des_chan = c;	/* XXX */
		return 0;
	}
	ic->ic_des_chan = c;
	if (c != IEEE80211_CHAN_ANYC)
		ic->ic_ibss_chan = c;
	if (ic->ic_opmode == IEEE80211_M_MONITOR)
		return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
	else
		return IS_UP(ic->ic_dev) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwfreq);

int
ieee80211_ioctl_giwfreq(struct ieee80211com *ic,
				struct iw_request_info *info,
				struct iw_freq *freq, char *extra)
{

	if (!ic->ic_ibss_chan)
		return -EINVAL;

	freq->m = ic->ic_ibss_chan->ic_freq * 100000;
	freq->e = 1;

	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwfreq);

int
ieee80211_ioctl_siwessid(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *ssid)
{

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
	return IS_UP(ic->ic_dev) ? -(*ic->ic_init)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwessid);

int
ieee80211_ioctl_giwessid(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *essid)
{

	data->flags = 1;		/* active */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		if (data->length > ic->ic_des_esslen)
			data->length = ic->ic_des_esslen;
		memcpy(essid, ic->ic_des_essid, data->length);
	} else {
		if (strlen(ic->ic_des_essid) == 0) {
			if (data->length > ic->ic_bss->ni_esslen)
				data->length = ic->ic_bss->ni_esslen;
			memcpy(essid, ic->ic_bss->ni_essid, data->length);
		} else {
			if (data->length > ic->ic_des_esslen)
				data->length = ic->ic_des_esslen;
			memcpy(essid, ic->ic_des_essid, data->length);
		}
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwessid);

int
ieee80211_ioctl_giwrange(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_point *data, char *extra)
{
	struct ieee80211_node *ni = ic->ic_bss;
	struct iw_range *range = (struct iw_range *) extra;
	struct ieee80211_rateset *rs;
	int i, r;

	data->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(struct iw_range));

	/* TODO: could fill num_txpower and txpower array with
	 * something; however, there are 128 different values.. */

	range->txpower_capa = IW_TXPOW_DBM;

	if (ic->ic_opmode == IEEE80211_M_STA ||
	    ic->ic_opmode == IEEE80211_M_IBSS) {
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

	/* Max quality is max field value minus noise floor */
	range->max_qual.qual  = 0xff - 161;

	/*
	 * In order to use dBm measurements, 'level' must be lower
	 * than any possible measurement (see iw_print_stats() in
	 * wireless tools).  It's unclear how this is meant to be
	 * done, but setting zero in these values forces dBm and
	 * the actual numbers are not used.
	 */
	range->max_qual.level = 0;
	range->max_qual.noise = 0;

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
EXPORT_SYMBOL(ieee80211_ioctl_giwrange);

int
ieee80211_ioctl_siwmode(struct ieee80211com *ic,
			struct iw_request_info *info,
			__u32 *mode, char *extra)
{
	struct ifreq ifr;

	if (!ic->ic_media.ifm_cur)
		return -EINVAL;
	memset(&ifr, 0, sizeof(ifr));
	/* NB: remove any fixed-rate at the same time */
	ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~
		(IFM_OMASK | IFM_TMASK);
	switch (*mode) {
	case IW_MODE_INFRA:
		/* NB: this is the default */
		ic->ic_des_chan = IEEE80211_CHAN_ANYC;
		break;
	case IW_MODE_ADHOC:
		ifr.ifr_media |= IFM_IEEE80211_ADHOC;
		break;
	case IW_MODE_MASTER:
		ifr.ifr_media |= IFM_IEEE80211_HOSTAP;
		break;
#if WIRELESS_EXT >= 15
	case IW_MODE_MONITOR:
		ifr.ifr_media |= IFM_IEEE80211_MONITOR;
		break;
#endif
	default:
		return -EINVAL;
	}
	return -ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
}
EXPORT_SYMBOL(ieee80211_ioctl_siwmode);

int
ieee80211_ioctl_giwmode(struct ieee80211com *ic,
			struct iw_request_info *info,
			__u32 *mode, char *extra)
{
	struct ifmediareq imr;

	memset(&imr, 0, sizeof(imr));
	(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);

	if (imr.ifm_active & IFM_IEEE80211_HOSTAP)
		*mode = IW_MODE_MASTER;
#if WIRELESS_EXT >= 15
	else if (imr.ifm_active & IFM_IEEE80211_MONITOR)
		*mode = IW_MODE_MONITOR;
#endif
	else if (imr.ifm_active & IFM_IEEE80211_ADHOC)
		*mode = IW_MODE_ADHOC;
	else
		*mode = IW_MODE_INFRA;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwmode);

int
ieee80211_ioctl_siwpower(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_param *wrq, char *extra)
{

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
	return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwpower);

int
ieee80211_ioctl_giwpower(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_param *rrq, char *extra)
{

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
EXPORT_SYMBOL(ieee80211_ioctl_giwpower);

int
ieee80211_ioctl_siwretry(struct ieee80211com *ic,
				 struct iw_request_info *info,
				 struct iw_param *rrq, char *extra)
{

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
	return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwretry);

int
ieee80211_ioctl_giwretry(struct ieee80211com *ic,
				 struct iw_request_info *info,
				 struct iw_param *rrq, char *extra)
{

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
EXPORT_SYMBOL(ieee80211_ioctl_giwretry);

int
ieee80211_ioctl_siwtxpow(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_param *rrq, char *extra)
{
	int fixed, disabled;

	fixed = (ic->ic_flags & IEEE80211_F_TXPOW_FIXED);
	disabled = (fixed && ic->ic_bss->ni_txpower == 0);
	if (rrq->disabled) {
		if (!disabled) {
			if ((ic->ic_caps & IEEE80211_C_TXPMGT) == 0)
				return -EOPNOTSUPP;
			ic->ic_flags |= IEEE80211_F_TXPOW_FIXED;
			ic->ic_bss->ni_txpower = 0;
			goto done;
		}
		return 0;
	}

	if (rrq->fixed) {
		if ((ic->ic_caps & IEEE80211_C_TXPMGT) == 0)
			return -EOPNOTSUPP;
		if (rrq->flags != IW_TXPOW_DBM)
			return -EOPNOTSUPP;
		ic->ic_bss->ni_txpower = 2*rrq->value;
		ic->ic_flags |= IEEE80211_F_TXPOW_FIXED;
	} else {
		if (!fixed)		/* no change */
			return 0;
		ic->ic_flags &= ~IEEE80211_F_TXPOW_FIXED;
	}
done:
	return IS_UP(ic->ic_dev) ? -(*ic->ic_reset)(ic->ic_dev) : 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwtxpow);

int
ieee80211_ioctl_giwtxpow(struct ieee80211com *ic,
			 struct iw_request_info *info,
			 struct iw_param *rrq, char *extra)
{

	rrq->value = ic->ic_bss->ni_txpower/2;
	rrq->fixed = (ic->ic_flags & IEEE80211_F_TXPOW_FIXED) != 0;
	rrq->disabled = (rrq->fixed && rrq->value == 0);
	rrq->flags = IW_TXPOW_DBM;
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_giwtxpow);

int
ieee80211_ioctl_iwaplist(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
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
		set_quality(&qual[i], (*ic->ic_node_getrssi)(ic, ni));
		if (++i >= IW_MAX_AP)
			break;
	}
	data->length = i;
	memcpy(extra, &addr, i*sizeof(addr[0]));
	data->flags = 1;		/* signal quality present (sort of) */
	memcpy(extra + i*sizeof(addr[0]), &qual, i*sizeof(qual[i]));

	return 0;

}
EXPORT_SYMBOL(ieee80211_ioctl_iwaplist);

#ifdef SIOCGIWSCAN
int
ieee80211_ioctl_siwscan(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{

	if (ic->ic_opmode != IEEE80211_M_HOSTAP) {
		/* just return existing station tables */
		data->length = 0;
	} else {
#ifdef notyet
		/* XXX honor scan request flags */
		HAL_CHANNEL *c = sc->sc_cur_chan;
		ni->ni_chan = ath_hal_ghz2ieee(c->channel, c->channelFlags);
		ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
#endif
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_siwscan);

int
ieee80211_ioctl_giwscan(struct ieee80211com *ic,
			struct iw_request_info *info,
			struct iw_point *data, char *extra)
{
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
		set_quality(&iwe.u.qual, (*ic->ic_node_getrssi)(ic, ni));
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
EXPORT_SYMBOL(ieee80211_ioctl_giwscan);
#endif /* SIOCGIWSCAN */

int
ieee80211_ioctl_setparam(struct ieee80211com *ic, struct iw_request_info *info,
		   	 void *w, char *extra)
{
	int *i = (int *) extra;
	int param = i[0];		/* parameter id is 1st */
	int value = i[1];		/* NB: all values are TYPE_INT */
	struct ifreq ifr;
	int retv = EOPNOTSUPP;

	switch (param) {
	case IEEE80211_PARAM_TURBO:
		if (!ic->ic_media.ifm_cur)
			return -EINVAL;
		memset(&ifr, 0, sizeof(ifr));
		ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media;
		if (value)
			ifr.ifr_media |= IFM_IEEE80211_TURBO;
		else
			ifr.ifr_media &= ~IFM_IEEE80211_TURBO;
		retv = ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
		break;
	case IEEE80211_PARAM_MODE:
		if (!ic->ic_media.ifm_cur)
			return -EINVAL;
		memset(&ifr, 0, sizeof(ifr));
		ifr.ifr_media = ic->ic_media.ifm_cur->ifm_media &~ IFM_MMASK;
		ifr.ifr_media |= IFM_MAKEMODE(value);
		retv = ifmedia_ioctl(ic->ic_dev, &ifr, &ic->ic_media, SIOCSIFMEDIA);
		break;
	case IEEE80211_PARAM_AUTHMODE:
		if (value > IEEE80211_AUTH_AUTO)
			return -EINVAL;
		/* shared key authentication requires WEP support */
		if (value == IEEE80211_AUTH_SHARED &&
		    (ic->ic_caps & IEEE80211_C_WEP) == 0)
			return -EINVAL;
		/* 802.1x authentication requires crypto support */
		if (value == IEEE80211_AUTH_8021X &&
		    (ic->ic_caps & IEEE80211_C_CRYPTO) == 0)
			return -EINVAL;
		/* NB: authenticator attach/detach happens on state change */
		if (value == IEEE80211_AUTH_8021X) {
			ic->ic_flags |= IEEE80211_F_PRIVACY;
		} else {
			/* turn off privacy if no crypto algorithms in use */
			if ((ic->ic_flags & IEEE80211_F_CRYPTON) == 0)
				ic->ic_flags &= ~IEEE80211_F_PRIVACY;
		}
		ic->ic_bss->ni_authmode = value;
		retv = ENETRESET;
		break;
	case IEEE80211_PARAM_PROTMODE:
		if (value > IEEE80211_PROT_RTSCTS)
			return -EINVAL;
		ic->ic_protmode = value;
		/* NB: if not operating in 11g this can wait */
		retv = (ic->ic_curmode == IEEE80211_MODE_11G ? ENETRESET : 0);
		break;
	}
	if (retv == ENETRESET)
		retv = IS_UP(ic->ic_dev) ? (*ic->ic_init)(ic->ic_dev) : 0;
	return -retv;
}
EXPORT_SYMBOL(ieee80211_ioctl_setparam);

int
ieee80211_ioctl_getparam(struct ieee80211com *ic, struct iw_request_info *info,
			void *w, char *extra)
{
	struct ifmediareq imr;
	int *param = (int *) extra;

	switch (param[0]) {
	case IEEE80211_PARAM_TURBO:
		(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);
		param[0] = (imr.ifm_active & IFM_IEEE80211_TURBO) != 0;
		break;
	case IEEE80211_PARAM_MODE:
		(*ic->ic_media.ifm_status)(ic->ic_dev, &imr);
		switch (IFM_MODE(imr.ifm_active)) {
		case IFM_IEEE80211_11A:
			param[0] = 1;
			break;
		case IFM_IEEE80211_11B:
			param[0] = 2;
			break;
		case IFM_IEEE80211_11G:
			param[0] = 3;
			break;
		case IFM_IEEE80211_FH:
			param[0] = 4;
			break;
		case IFM_AUTO:
			param[0] = 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IEEE80211_PARAM_AUTHMODE:
		param[0] = ic->ic_bss->ni_authmode;
		break;
	case IEEE80211_PARAM_PROTMODE:
		param[0] = ic->ic_protmode;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}
EXPORT_SYMBOL(ieee80211_ioctl_getparam);

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
	{ IEEE80211_PARAM_AUTHMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "authmode" },
	{ IEEE80211_PARAM_AUTHMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_authmode" },
	{ IEEE80211_PARAM_PROTMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "protmode" },
	{ IEEE80211_PARAM_PROTMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_protmode" },
#endif /* WIRELESS_EXT >= 12 */
};

void
ieee80211_ioctl_iwsetup(struct iw_handler_def *def)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	def->private_args = (struct iw_priv_args *) ieee80211_priv_args;
	def->num_private_args = N(ieee80211_priv_args);
#undef N
}
EXPORT_SYMBOL(ieee80211_ioctl_iwsetup);
#endif /* CONFIG_NET_WIRELESS */
