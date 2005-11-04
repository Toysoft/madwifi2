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
 */

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * IEEE 802.11 monitor mode 
 */
#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>

#include <net/iw_handler.h>
#include <linux/wireless.h>
#include <linux/if_arp.h>		/* XXX for ARPHRD_* */

#include <asm/uaccess.h>

#include "if_media.h"
#include "if_ethersubr.h"

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_monitor.h>
enum {
	DIDmsg_lnxind_wlansniffrm		= 0x00000044,
	DIDmsg_lnxind_wlansniffrm_hosttime	= 0x00010044,
	DIDmsg_lnxind_wlansniffrm_mactime	= 0x00020044,
	DIDmsg_lnxind_wlansniffrm_channel	= 0x00030044,
	DIDmsg_lnxind_wlansniffrm_rssi		= 0x00040044,
	DIDmsg_lnxind_wlansniffrm_sq		= 0x00050044,
	DIDmsg_lnxind_wlansniffrm_signal	= 0x00060044,
	DIDmsg_lnxind_wlansniffrm_noise		= 0x00070044,
	DIDmsg_lnxind_wlansniffrm_rate		= 0x00080044,
	DIDmsg_lnxind_wlansniffrm_istx		= 0x00090044,
	DIDmsg_lnxind_wlansniffrm_frmlen	= 0x000A0044
};
enum {
	P80211ENUM_msgitem_status_no_value	= 0x00
};
enum {
	P80211ENUM_truth_false			= 0x00
};


void
ieee80211_monitor_encap(struct ieee80211com *ic, struct sk_buff *skb) 
{
	struct ieee80211_cb *cb = (struct ieee80211_cb *) skb->cb;
	struct ieee80211_phy_params *ph = (struct ieee80211_phy_params *) (skb->cb + sizeof(struct ieee80211_cb));
	cb = (struct ieee80211_cb *) skb->cb;
	cb->flags = M_RAW;
	cb->ni = NULL;
	cb->next = NULL;
	memset(ph, 0, sizeof(struct ieee80211_phy_params));

	/* defaults */
	ph->rate0 = 2;
	ph->try0 = 11;
	ph->power = 60;

	switch (skb->dev->type) {
	case ARPHRD_IEEE80211: break;
	case ARPHRD_IEEE80211_PRISM: {
		wlan_ng_prism2_header *wh = NULL;
		wh = (wlan_ng_prism2_header *) skb->data;
		/* does it look like there is a prism header here? */
		if (skb->len > sizeof (wlan_ng_prism2_header) &&
                    wh->msgcode == DIDmsg_lnxind_wlansniffrm &&
		    wh->rate.did == DIDmsg_lnxind_wlansniffrm_rate) {
                        ph->rate0 = wh->rate.data;
                        skb_pull(skb, sizeof(wlan_ng_prism2_header));
		}
                break;
	}
		
	default: break;
	}

	if (!ph->rate0) {
		ph->rate0 = 0;
		ph->try0 = 11;
	}
}
EXPORT_SYMBOL(ieee80211_monitor_encap);

void
ieee80211_input_monitor(struct ieee80211com *ic, struct sk_buff *skb,
	u_int32_t mactime, u_int32_t rssi, u_int32_t signal, u_int32_t rate)
{
	static const wlan_ng_prism2_header template = {
		.msgcode	= DIDmsg_lnxind_wlansniffrm,
		.msglen		= sizeof(wlan_ng_prism2_header),

		.hosttime	= { .did = DIDmsg_lnxind_wlansniffrm_hosttime,
				    .len = 4},

		.mactime	= { .did = DIDmsg_lnxind_wlansniffrm_mactime,
				    .len = 4},

		.istx		= { .did = DIDmsg_lnxind_wlansniffrm_istx,
				    .len = 4,
				    .data = P80211ENUM_truth_false},

		.frmlen		= { .did = DIDmsg_lnxind_wlansniffrm_frmlen,
				    .len = 4},

		.channel	= { .did = DIDmsg_lnxind_wlansniffrm_channel,
				    .len = 4},

		.rssi		= { .did = DIDmsg_lnxind_wlansniffrm_rssi,
				    .status = P80211ENUM_msgitem_status_no_value,
				    .len = 4},

		.signal		= { .did = DIDmsg_lnxind_wlansniffrm_signal,
				    .len = 4},

		.rate		= { .did = DIDmsg_lnxind_wlansniffrm_rate,
				    .len = 4},
	};
	struct ieee80211vap *vap, *next;
	/* XXX locking */
	for (vap = TAILQ_FIRST(&ic->ic_vaps); vap != NULL; vap = next) {
		struct sk_buff *skb1;
		struct net_device *dev = vap->iv_dev;
		wlan_ng_prism2_header *ph;
		next = TAILQ_NEXT(vap, iv_next);
		if (vap->iv_opmode != IEEE80211_M_MONITOR ||
		    vap->iv_state != IEEE80211_S_RUN)
			continue;
		
		skb1 = skb_copy(skb, GFP_ATOMIC);
		if (skb1 == NULL) {
			/* XXX stat+msg */
			continue;
		}
		if (skb_headroom(skb1) < sizeof(wlan_ng_prism2_header)) {
			dev_kfree_skb(skb1);
			return;
		}
		
		ph = (wlan_ng_prism2_header *)
			skb_push(skb1, sizeof(wlan_ng_prism2_header));
		*ph = template;
		
		ph->hosttime.data = jiffies;
		ph->mactime.data = mactime;
		ph->frmlen.data = skb->len - sizeof(wlan_ng_prism2_header);
		/* XXX no way to pass channel flag state */
		ph->channel.data = ieee80211_chan2ieee(ic, ic->ic_curchan);
		ph->rssi.data = rssi;
		ph->signal.data = signal;
		ph->rate.data = rate;



		strncpy(ph->devname, dev->name, sizeof(ph->devname));

		skb1->dev = dev; /* NB: deliver to wlanX */
		skb1->mac.raw = skb1->data;
		skb1->ip_summed = CHECKSUM_NONE;
		skb1->pkt_type = PACKET_OTHERHOST;
		skb1->protocol = __constant_htons(0x0019); /* ETH_P_80211_RAW */

		netif_rx(skb1);

		vap->iv_devstats.rx_packets++;
		vap->iv_devstats.rx_bytes += skb1->len;
	}
}
EXPORT_SYMBOL(ieee80211_input_monitor);

