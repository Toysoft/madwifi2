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

__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_crypto.c,v 1.3 2003/10/17 23:15:30 sam Exp $");
__KERNEL_RCSID(0, "$NetBSD: ieee80211_crypto.c,v 1.4 2003/09/23 16:03:46 dyoung Exp $");

/*
 * IEEE 802.11 crypto support (when done in the host).
 */
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

#include "rc4.h"
#define	arc4_ctxlen()			sizeof (struct rc4_state)
#define	arc4_setkey(_c,_k,_l)		rc4_init(_c,_k,_l)
#define	arc4_encrypt(_c,_d,_s,_l)	rc4_crypt(_c,_s,_d,_l)

static	void ieee80211_crc_init(void);
static	u_int32_t ieee80211_crc_update(u_int32_t crc, u_int8_t *buf, int len);

void
ieee80211_crypto_attach(struct ieee80211com *ic)
{

	/*
	 * Setup crypto support.
	 */
	ieee80211_crc_init();
	get_random_bytes(&ic->ic_iv, sizeof(ic->ic_iv));
	ic->ic_wep_txkey = IEEE80211_KEYIX_NONE;
}

void
ieee80211_crypto_detach(struct ieee80211com *ic)
{

	if (ic->ic_wep_ctx != NULL) {
		FREE(ic->ic_wep_ctx, M_DEVBUF);
		ic->ic_wep_ctx = NULL;
	}
}

struct sk_buff *
ieee80211_wep_crypt(struct ieee80211com *ic, struct ieee80211_node *ni,
	struct sk_buff *skb0, int txflag)
{
	struct sk_buff *skb, *n, *n0;
	struct ieee80211_frame *wh;
	struct ieee80211_wepkey *key;
	int i, left, len, moff, noff, ismcast, kid;
	u_int32_t iv, crc;
	u_int8_t *ivp;
	void *ctx;
	u_int8_t keybuf[IEEE80211_WEP_IVLEN + IEEE80211_KEYBUF_SIZE];
	u_int8_t crcbuf[IEEE80211_WEP_CRCLEN];

	n0 = NULL;
	if ((ctx = ic->ic_wep_ctx) == NULL) {
		MALLOC(ctx, void *, arc4_ctxlen(), M_DEVBUF, M_NOWAIT);
		if (ctx == NULL) {
			ic->ic_stats.is_crypto_nomem++;
			goto fail;
		}
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
	if (n == NULL) {
		if (txflag)
			ic->ic_stats.is_tx_nobuf++;
		else
			ic->ic_stats.is_rx_nobuf++;
		goto fail;
	}
	n0 = n;
	memcpy(n->data, skb->data, sizeof(struct ieee80211_frame));
	wh = (struct ieee80211_frame *) n->data;
	left -= sizeof(struct ieee80211_frame);
	moff = sizeof(struct ieee80211_frame);
	noff = sizeof(struct ieee80211_frame);
	if (txflag) {
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
                iv = ic->ic_iv;
		/*
		 * Skip 'bad' IVs from Fluhrer/Mantin/Shamir:
		 * (B, 255, N) with 3 <= B < 8
		 */
		if (iv >= 0x03ff00 && (iv & 0xf8ff00) == 0x00ff00)
			iv += 0x000100;
		ic->ic_iv = iv + 1;
		/* put iv in little endian to prepare 802.11i */
		ivp = n->data + noff;
		for (i = 0; i < IEEE80211_WEP_IVLEN; i++) {
			ivp[i] = iv & 0xff;
			iv >>= 8;
		}
		ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
		if (ismcast || ni->ni_ucastkeyix == IEEE80211_KEYIX_NONE) {
			kid = ic->ic_wep_txkey;
			if (kid == IEEE80211_KEYIX_NONE) {
				IEEE80211_DPRINTF(ic, IEEE80211_MSG_CRYPTO,
					("no xmit key setup\n"));
				/* XXX statistic */
				goto fail;
			}
			ivp[IEEE80211_WEP_IVLEN] = kid << 6;/* pad and keyid */
			key = &ic->ic_nw_keys[kid];
		} else {
			ivp[IEEE80211_WEP_IVLEN] = 0;	/* pad and keyid */
			key= &ni->ni_ucastkey;
		}
		noff += IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;
	} else {
		wh->i_fc[1] &= ~IEEE80211_FC1_WEP;
		ivp = skb->data + moff;
		kid = ivp[IEEE80211_WEP_IVLEN] >> 6;
		/* check for station key; fallback to shared key */
		if (kid == 0 && ni->ni_ucastkeyix != IEEE80211_KEYIX_NONE)
			key = &ni->ni_ucastkey;
		else
			key = &ic->ic_nw_keys[kid];
		moff += IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;
	}
	memcpy(keybuf, ivp, IEEE80211_WEP_IVLEN);
	memcpy(keybuf + IEEE80211_WEP_IVLEN, key->wk_key, key->wk_len);
	arc4_setkey(ctx, keybuf, IEEE80211_WEP_IVLEN + key->wk_len);

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
			if (ieee80211_msg_debug(ic)) {
				if_printf(ic->ic_dev, "decrypt CRC error\n");
				if (ieee80211_msg_dumppkts(ic))
					ieee80211_dump_pkt(n0->data,
					    n0->len, -1, -1);
			}
			ic->ic_stats.is_rx_decryptcrc++;
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
