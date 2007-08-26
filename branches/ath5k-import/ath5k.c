/*
 * Copyright (c) 2002-2007 Sam Leffler, Errno Consulting
 * Copyright (c) 2006 Devicescape Software, Inc. All Rights Reserved
 * Copyright (c) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
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
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/pci.h>

#include "if_ath.h"
#include "ath5k.h"

static struct ieee80211_ops ath_ops = {
	.tx = ath5k_tx,
	.reset = ath5k_reset,
	.open = ath5k_open, 
	.stop = ath5k_stop,
	.add_interface = ath5k_add_interface,
	.remove_interface = ath5k_remove_interface,
	.config = ath5k_config,
	.config_interface = ath5k_config_interface,
	.get_tsf = ath5k_get_tsf, /* 64 bit */
	.reset_tsf = ath5k_reset_tsf,
};

static int ath5k_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
	struct ieee80211_tx_control *control)
{
        struct ath_softc *sc = hw->priv;
        struct ath_buf *bf = NULL;
	unsigned long flags;
        STAILQ_HEAD(tmp_bf_head, ath_buf) bf_head;
        STAILQ_INIT(&bf_head);
	spin_lock_irqsave(&(sc)->sc_txbuflock, flags);
	bf = STAILQ_FIRST(&sc->sc_txbuf);
	if (bf != NULL) {
		STAILQ_REMOVE_HEAD(&sc->sc_txbuf, bf_list);
		STAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
	}
	/* XXX use a counter and leave at least one for mgmt frames */
	if (STAILQ_EMPTY(&sc->sc_txbuf)) {
		printk(KERN_INFO "%s: stop queue\n", __func__);
		sc->sc_stats.ast_tx_qstop++;
		ieee80211_stop_queues(sc->sc_hw);
		sc->sc_devstopped = 1;
		tasklet_schedule(&sc->sc_txtq, NULL);
	}
	spin_unlock_irqrestore(&(sc)->sc_txbuflock, flags);
	if (bf == NULL) { /* NB: should not happen */
		printk(KERN_ERROR "%s: discard, no xmit buf\n", __func__);
		sc->sc_stats.ast_tx_nobuf++;
		return 1;
	}
        /* FIXME: we are only using a single hardware queue. */
        ath_tx_startraw(sc, bf, skb, control, sc->sc_ac2q[WME_AC_BK]);
        return 0;
}

static int
ath5k_reset(struct ieee80211_hw *hw)
{
        struct ath_softc *sc = hw->priv;
        return ath_reset(sc);
}

static int
ath5k_open(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	int r;
	r = ath_init(sc);
	if (r == 0)
		sc->sc_dev_open = 1;
	return r;
}

static int
ath5k_stop(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	sc->sc_dev_open = 0;
	return ath_stop(sc); /* Rename atheros ath_stop to ath_stop_hw */
}

static int
ath5k_add_interface(struct ieee80211_hw *hw,
	struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	int error = 0;
	down(&(sc)->sc_lock);
	switch (conf->type) {
	case IEEE80211_IF_TYPE_STA:
		sc->sc_opmode = AR5K_M_STA;
		break;
	case IEEE80211_IF_TYPE_IBSS:
		sc->sc_opmode = AR5K_M_IBSS;
		break;
	case IEEE80211_IF_TYPE_MNTR:
		sc->sc_opmode = AR5K_M_MONITOR;
		break;
	case IEEE80211_IF_TYPE_AP: /* TODO */
        default:
                error = -EINVAL;
                goto done;
	}
	if (ath5k_calc_bssid_mask(hw))
		error = ath_reset(sc);
done:
	up(&(sc)->sc_lock);
	return error;
}


static void
ath5k_remove_interface(struct ieee80211_hw *hw,
	struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	int i;
	down(&(sc)->sc_lock);
	if (sc->sc_num_bss == 0)
		sc->sc_beacons = 0;
	if (ath5k_calc_bssid_mask(hw))
		ath_reset(sc);
	up(&(sc)->sc_lock);
}

static int
ath5k_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	AR5K_CHANNEL hchan;
	int ret;
	sc->sc_ieee80211_channel = conf->channel;
	sc->sc_mode = conf->phymode;
	sc->sc_beacon_interval = (conf->beacon_int * 1000) >> 10;
	if (sc->sc_shortslottime !=
		!!(conf->flags & IEEE80211_CONF_SHORT_SLOT_TIME)) {
		sc->sc_shortslottime =
			!!(conf->flags & IEEE80211_CONF_SHORT_SLOT_TIME);
		sc->sc_updateslot = UPDATE;
	}
	if (!sc->sc_dev_open || !conf->radio_enabled)
		return 0;
	hchan.freq = conf->freq;
	hchan.channel_flags = conf->channel_val;
	ret = ath_chan_set(sc, hchan);
	if (ret) 
		return ret;
	return 0;
}


static int
ath5k_config_interface(struct ieee80211_hw *hw, int if_id,
	struct ieee80211_if_conf *conf)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;
	if (conf->bssid)
		ath_hal_setassocid(ah, conf->bssid, 0 /* FIXME: aid */);
	return ath_reset(sc);
}

static u64
ath5k_get_tsf(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;
	return ath_hal_gettsf64(ah);
}

static void
ath5k_reset_tsf(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;
	ath_hal_resettsf(ah);
}

/**
 * ath5k_calc_bssid_mask - Calculate the required BSSID mask.
 * 
 * Note: Caller must hold down(&(sc)->sc_lock);
 *
 * Returns 1 if the bssidmask changed otherwise returns 0.
 */
static int
ath5k_calc_bssid_mask(struct ieee80211_hw *hw)
{
	struct ath_softc *sc = hw->priv;
	int i, j;
	struct net_device *dev;
	unsigned char mask[ETH_ALEN];
	memset(mask, 0xff, ETH_ALEN);
	for (i = 0; i < sc->sc_num_bss; i++) {
		dev = dev_get_by_index(sc->sc_bss[i].ab_if_id);
		for (j = 0; j < ETH_ALEN; j++) {
			mask[j] &= ~(hw->wiphy->perm_addr[j] ^ 
				dev->dev_addr[j]);
		}
		dev_put(dev);
	}
	if (memcmp(sc->sc_bssidmask, mask, ETH_ALEN)) {
		memcpy(sc->sc_bssidmask, mask, ETH_ALEN);
		return 1;
	}
	return 0;
}

/**
 * ath5k_add_channels - Setup channel array for a given hardware mode.
 * @sc: device in question
 * @hw_mode: ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 * @hal_chans: pointer to an array of channels from the hal
 * @hal_nchan: number of total channels in @hal_chans
 * @hal_flags: hal channel flags which identify which channels pertain to the
 *             hardware mode in question (the array @hal_chans includes all
 *             channels supported by the device).
 *
 * Returns 0 on success or < 0 on error.
 */
int
ath5k_add_channels(struct ath_softc *sc, int hw_mode,
	       		AR5K_CHANNEL *hal_chans, int hal_nchan, int hal_flags)
{
	struct ieee80211_hw_mode *mode;
	int error = 0;
	int i;
	/* Find the next mode index we can use to initialize a new mode */
	for (i = 0; i < sc->sc_num_modes ; i++) {
		if (sc->sc_hw_modes[i].mode == hw_mode)
			break;
	}
	/* sc_num_modes starts at 0, and we increase it at the bottom here
	 * as we add new modes */
	if (i == sc->sc_num_modes) {
		if (sc->sc_num_modes == ATH_MAX_HW_MODES) { 
			printk(KERN_ERROR, "%s: no free mode elements\n", 
				__func__);
			return -1;
		}
		mode = &sc->sc_hw_modes[sc->sc_num_modes];
	} else {
		printk(KERN_ERROR, "%s: mode %d already initialized\n", 
			__func__, hw_mode);
		return -1;
	}
	for (i = 0; i < hal_nchan; i++) {
		AR5K_CHANNEL *c = &hal_chans[i];
		if ((c->channel_flags & CHANNEL_ALL) == hal_flags) {
			struct ieee80211_channel *channel;
			if (mode->num_channels == ATH_MAX_CHANNELS) {
				printk(KERN_ERR "channel list truncated\n");
				error = -E2BIG;
				goto done;
			}
			channel = &mode->channels[mode->num_channels];
			channel->chan = ath_hal_mhz2ieee(c->freq, c->channel_flags);
			channel->freq = c->freq;
			channel->val = hal_flags;
			/* ? = c->private_flags; FIXME */
			/* ? = c->minTxPower; FIXME */
			/* channel->flag = ? FIXME */	
//			channel->power_level = c->maxRegTxPower; /* ??? FIXME */
//			channel->antenna_max = c->maxTxPower; /* ??? FIXME */
			mode->num_channels++;
		}
	}
done:
	if (mode->num_channels != 0) {
		DPRINTF(sc, ATH_DEBUG_MAC80211, "%s: hal_chan %x hal_flags %x\n", __func__,
			hal_nchan, hal_flags);
		mode->mode = hw_mode;
		sc->sc_num_modes++;
	}
	return error;
}


/**
 * ath5k_rate_setup - Setup a rate array for a given hardware mode.
 * @sc: device in question
 * @hal_mode: hal hardware mode (AR5K_MODE_11A, AR5K_MODE_11B, ...)
 * @rt: hal rate table for the mode in question
 * 
 * Returns 0 on success or < 0 on error.
 *
 * XXX: This happens on every channel change? locking? 
 */
int
ath5k_rate_setup(struct ath_softc *sc, u_int hal_mode,
		      const struct ar5k_rate_table *rt)
{
	struct ieee80211_hw_mode *mode;
	int hw_mode;
	struct ieee80211_rate *rates;
	int i;
	hw_mode = ath5k_mode_to_mac80211_mode(hal_mode);
	for (i = 0; i < sc->sc_num_modes ; i++) {
		if (sc->sc_hw_modes[i].mode == hw_mode)
			break;
	}
	if (i == sc->sc_num_modes) {
		printk(KERN_ERR "cannot find mode element.\n");
		return -1;
	}
	mode = &sc->sc_hw_modes[i];
	mode->num_rates = 0;
	rates = mode->rates;
	for (i = 0; i < rt->rate_count; i++) {
		if (mode->num_rates == ATH_MAX_RATES) {
			printk(KERN_ERR "rate list truncated\n");
			return -1;
		}
		rates[i].rate = rt->rates[i].rate_kbps / 100;
		rates[i].val = rt->rates[i].rate_code;
		rates[i].flags = rt->rates[i].modulation;
		/* FIXME rates[i].min_rssi_ack = ?; */
		/* FIXME rates[i].min_rssi_ack_delta = ?; */
		mode->num_rates++;
	}
	return 0;
}

/**
 * ath5k_alloc - Allocate and initialize hardware structure
 * @priv_size: size of private data
 *
 * Returns a pointer to the newly allocated struct ath_softc on success.
 * Returns NULL on error.
 */
struct ath_softc *
ath5k_alloc(size_t priv_size)
{
	struct ieee80211_hw *hw;
	struct ieee80211_hw_mode *modes;
	struct ath_softc *sc;
	int i;

	hw = ieee80211_alloc_hw(priv_size, &ath5k__ops);

	if (!hw) {
		printk("ath_d80211: Failed to allocate hw\n");
		return NULL;
	}

	sc = hw->priv;
	sc->sc_hw = hw;

	DPRINTF(sc, ATH_DEBUG_MAC80211, "%s\n", __func__);

	hw->flags = IEEE80211_HW_HOST_GEN_BEACON |
		 IEEE80211_HW_RX_INCLUDES_FCS |
		 IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING |
		 IEEE80211_HW_WEP_INCLUDE_IV |
		 IEEE80211_HW_DATA_NULLFUNC_ACK;
	hw->extra_tx_headroom = 2;
	hw->channel_change_time = 5000;
	hw->max_rssi = 127; /* FIXME: get a real value for this. */
	hw->queues = 1;
	sc->sc_num_modes = 0;

	modes = &sc->sc_hw_modes[0];

	for (i = 0; i < ATH_MAX_HW_MODES; i++) {
		modes[i].num_channels = 0;
		modes[i].channels = &sc->sc_channels[i * ATH_MAX_CHANNELS];
		modes[i].num_rates = 0;
		modes[i].rates = &sc->sc_ieee80211_rates[i * ATH_MAX_RATES];
	}

	sc->sc_opmode = AR5K_M_STA;

	spin_lock_init(&sc->sc_bss_lock);

	return sc;
}


/**
 * ath_d80211_free - Free memory allocated by ath_d80211_alloc()
 * @sc: A pointer returned by ath_d80211_alloc().
 */
void ath5k_free(struct ath_softc *sc)
{
	ieee80211_free_hw(sc->sc_hw);
}


int
ath5k_attach(struct ath_softc *sc)
{
	struct ieee80211_hw *hw = sc->sc_hw;
	struct pci_dev *pdev = (struct pci_dev *)sc->sc_bdev;
	unsigned int i;
	int r = 0;

	DPRINTF(sc, ATH_DEBUG_MAC80211, "%s\n", __func__);

	for (i = 0; i < sc->sc_num_modes; i++) {
		ieee80211_register_hwmode(hw, &sc->sc_hw_modes[i]);
		printk(KERN_DEBUG "register hw_mode %d\n",sc->sc_hw_modes[i].mode);
	}

	SET_IEEE80211_DEV(hw, &pdev->dev);

	r = ieee80211_register_hw(hw);
	if (rv)
		printk(KERN_ERR "%s: device registration failed.\n", sc->name);

	return r;
}

void
ath5k_detach(struct ath_softc *sc)
{
	kfree(sc->sc_bss);
}
