/*-
 * Copyright (c) 2006 Devicescape Software, Inc. All Rights Reserved.
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
 */

#include "opt_ah.h"

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>

#include "if_athvar.h"
#include "if_ath_d80211.h"
#include "if_ath.h"

#include <net/d80211.h>


static struct {
	u_int	hal_mode;	/* hal phy mode */
	int	d80211_mode;	/* d80211 phy mode */
} ath_mode_map[] = {
	{ HAL_MODE_11A,  MODE_IEEE80211A	},
	{ HAL_MODE_11B,  MODE_IEEE80211B	},
	{ HAL_MODE_11G,  MODE_IEEE80211G	},
	{ HAL_MODE_108A, MODE_ATHEROS_TURBO	},
	{ HAL_MODE_108G, MODE_ATHEROS_TURBOG	},
};


/**
 * ath_hal_mode_to_d80211_mode - Convert a hal mode to an IEEE80211 mode.
 * @hal_mode: hal hardware mode (HAL_MODE_11A, HAL_MODE_11B, ...)
 *
 * Returns ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 */
static int
ath_hal_mode_to_d80211_mode(u_int hal_mode)
{
	int i;

	for (i = 0; i < sizeof(ath_mode_map) / sizeof(ath_mode_map[0]); i++) {
		if (ath_mode_map[i].hal_mode == hal_mode)
			return ath_mode_map[i].d80211_mode;
	}
	printk(KERN_ERR "Invalid mode.\n");
	return ath_mode_map[0].d80211_mode;
}


/**
 * ath_d80211_add_channels - Setup channel array for a given hardware mode.
 * @dev: device in question
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
ath_d80211_add_channels(struct net_device *dev, int hw_mode,
	       		HAL_CHANNEL *hal_chans, int hal_nchan, int hal_flags)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_hw_modes *mode;
	int i;

	for (i = 0; i < sc->hw_conf.num_modes ; i++) {
		if (sc->hw_modes[i].mode == hw_mode)
			break;
	}

	if (i == sc->hw_conf.num_modes) {
		printk(KERN_ERR "cannot find mode element.\n");
		return -1;
	}

	mode = &sc->hw_modes[i];
	mode->num_channels = 0;

	for (i = 0; i < hal_nchan; i++) {
		HAL_CHANNEL *c = &hal_chans[i];

		if ((c->channelFlags & CHANNEL_ALL) == hal_flags) {
			struct ieee80211_channel *channel;
		
			if (mode->num_channels == ATH_MAX_CHANNELS) {
				printk(KERN_ERR "channel list truncated\n");
				return -1;
			}

			channel = &mode->channels[mode->num_channels];

			channel->chan = ath_hal_mhz2ieee(ah, c->channel, 
							 c->channelFlags);
			channel->freq = c->channel;
			channel->val = hal_flags;
			/* ? = c->privFlags; FIXME */
			/* ? = c->minTxPower; FIXME */
			/* channel->flag = ? FIXME */	
			channel->power_level = c->maxRegTxPower; /* ??? FIXME */
			channel->antenna_max = c->maxTxPower; /* ??? FIXME */

			mode->num_channels++;
		}
	}

	return 0;
}	


/**
 * ath_d80211_rate_setup - Setup a rate array for a given hardware mode.
 * @dev: device in question
 * @hal_mode: hal hardware mode (HAL_MODE_11A, HAL_MODE_11B, ...)
 * @rt: hal rate table for the mode in question
 * 
 * Returns 0 on success or < 0 on error.
 *
 * XXX: This happens on every channel change? locking? 
 */
int
ath_d80211_rate_setup(struct net_device *dev, u_int hal_mode,
		      const HAL_RATE_TABLE *rt)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	struct ieee80211_hw_modes *mode;
	int hw_mode;
	struct ieee80211_rate *rates;
	int i;

	hw_mode = ath_hal_mode_to_d80211_mode(hal_mode);

	for (i = 0; i < sc->hw_conf.num_modes ; i++) {
		if (sc->hw_modes[i].mode == hw_mode)
			break;
	}

	if (i == sc->hw_conf.num_modes) {
		printk(KERN_ERR "cannot find mode element.\n");
		return -1;
	}

	mode = &sc->hw_modes[i];
	mode->num_rates = 0;
	rates = mode->rates;

	for (i = 0; i < rt->rateCount; i++) {

		if (mode->num_rates == ATH_MAX_RATES) {
			printk(KERN_ERR "rate list truncated\n");
			return -1;
		}

		rates[i].rate = rt->info[i].rateKbps / 100;
		rates[i].val = rt->info[i].rateCode;

		switch (rt->info[i].phy) {
			case IEEE80211_T_OFDM:
				rates[i].flags |= IEEE80211_RATE_OFDM;
				break;
			case IEEE80211_T_CCK:
				rates[i].flags |= IEEE80211_RATE_CCK;
				break;
			case IEEE80211_T_TURBO:
				rates[i].flags |= IEEE80211_RATE_TURBO;
				break;
			default:
				printk("unknown phy type %d\n",
				       rt->info[i].phy);
		}

		if (rt->info[i].shortPreamble) {
			rates[i].flags |= IEEE80211_RATE_PREAMBLE2;
		}
		/* FIXME rates[i].val2 = ?; */
		/* FIXME rates[i].min_rssi_ack = ?; */
		/* FIXME rates[i].min_rssi_ack_delta = ?; */
		mode->num_rates++;
	}

	return 0;
}


static int
ath_d80211_reset(struct net_device *dev)
{
	return ath_reset(dev);
}


static int
ath_d80211_open(struct net_device *dev)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	int rv;

	rv = ath_init(dev);

	if (rv == 0)
		sc->sc_dev_open = 1;

	return rv;
}


static int
ath_d80211_stop(struct net_device *dev)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);

	sc->sc_dev_open = 0;

	return ath_stop(dev);
}


/**
 * ath_d80211_calc_bssid_mask - Calculate the required BSSID mask.
 * @mdev: master device in question
 * 
 * Returns 1 if the bssidmask changed otherwise returns 0.
 */
static int
ath_d80211_calc_bssid_mask(struct net_device *mdev)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(mdev);
	int i, j;
	struct net_device *dev;
	unsigned char mask[ETH_ALEN];

	memset(mask, 0xff, ETH_ALEN);

	spin_lock_bh(&sc->sc_bss_lock);

	for (i = 0; i < sc->sc_bss_count; i++) {
		dev = dev_get_by_index(sc->sc_bss_if_ids[i]);

		for (j = 0; j < ETH_ALEN; j++) {
			mask[j] &= ~(mdev->dev_addr[j] ^ dev->dev_addr[j]);
		}

		dev_put(dev);
	}

	spin_unlock_bh(&sc->sc_bss_lock);

	if (memcmp(sc->sc_bssidmask, mask, ETH_ALEN)) {
		memcpy(sc->sc_bssidmask, mask, ETH_ALEN);
		return 1;
	}

	return 0;
}


static int
ath_d80211_add_interface(struct net_device *dev,
			 struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	int reset;

	switch (conf->type) {
	case IEEE80211_IF_TYPE_STA:
		sc->sc_opmode = HAL_M_STA;
		break;
	case IEEE80211_IF_TYPE_IBSS:
		sc->sc_opmode = HAL_M_IBSS;
		break;
	case IEEE80211_IF_TYPE_MNTR:
		sc->sc_opmode = HAL_M_MONITOR;
		break;
	case IEEE80211_IF_TYPE_AP:

		spin_lock_bh(&sc->sc_bss_lock);

		if (sc->sc_num_bss_if_ids < sc->sc_bss_count + 1) {
			int *ids;

			ids = kmalloc((sc->sc_num_bss_if_ids + 1) *
				      sizeof(ids[0]), GFP_KERNEL);

			if (!ids) {
				spin_unlock_bh(&sc->sc_bss_lock);
				return -ENOMEM;
			}

			sc->sc_num_bss_if_ids++;
			memcpy(ids, sc->sc_bss_if_ids,
			       sc->sc_bss_count * sizeof(ids[0]));
		
			kfree(sc->sc_bss_if_ids);
			sc->sc_bss_if_ids = ids;
		}

		sc->sc_bss_if_ids[sc->sc_bss_count] = conf->if_id;
		sc->sc_bss_count++;
		
		spin_unlock_bh(&sc->sc_bss_lock);

		sc->sc_opmode = HAL_M_HOSTAP;

		break;
	default:
		return -EINVAL;
		break;
	}

	reset = ath_d80211_calc_bssid_mask(dev);

	if (reset)
		return ath_reset(dev);

	return 0;
}


static void
ath_d80211_remove_interface(struct net_device *dev,
			    struct ieee80211_if_init_conf *conf)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	int i;

	switch (conf->type) {
	case IEEE80211_IF_TYPE_AP:

		spin_lock_bh(&sc->sc_bss_lock);

		for (i = 0; i < sc->sc_bss_count; i++) {
			if (sc->sc_bss_if_ids[i] == conf->if_id)
				break;
		}

		if (i == sc->sc_bss_count) {
			printk(KERN_ERR "%s: remove cannot find if_id %d\n",
			       dev->name, conf->if_id);
			spin_unlock_bh(&sc->sc_bss_lock);
			return;
		}

		memmove(&sc->sc_bss_if_ids[i], &sc->sc_bss_if_ids[i+1],
			(sc->sc_bss_count - i - 1) * sizeof(sc->sc_bss_if_ids[0]));

		sc->sc_bss_count--;

		spin_unlock_bh(&sc->sc_bss_lock);

		break;
	}

	if (ath_d80211_calc_bssid_mask(dev))
		ath_reset(dev);
}


static int
ath_d80211_config(struct net_device *dev, struct ieee80211_conf *conf)
{
	/* FIXME: more to configure */
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	HAL_CHANNEL hchan;
	int ret;

	sc->sc_ieee80211_channel = conf->channel;
	sc->sc_mode = conf->phymode;

	if (!sc->sc_dev_open || !conf->radio_enabled)
		return 0;

	hchan.channel = conf->freq;
	hchan.channelFlags = conf->channel_val;

	if ((ret = ath_chan_set(sc, hchan)))
		return ret;

	return 0;
}


static int
ath_d80211_config_interface(struct net_device *dev, int if_id,
			    struct ieee80211_if_conf *conf)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	struct ath_hal *ah = sc->sc_ah;

	ath_hal_setassocid(ah, conf->bssid, 0 /* FIXME: aid */);
	return ath_reset(dev);
}


static int
ath_d80211_set_key(struct net_device *dev, set_key_cmd cmd, u8 * addr,
		   struct ieee80211_key_conf *key, int aid)
{
	/* FIXME */
	return 0;
}


static int
ath_d80211_passive_scan(struct net_device *dev, int state,
			struct ieee80211_scan_conf *conf)
{
	/* FIXME */
	return 0;
}


static int
ath_d80211_get_stats(struct net_device *dev,
		     struct ieee80211_low_level_stats *stats)
{
	/* FIXME */
	return 0;
}


static int
ath_d80211_test_mode(struct net_device *dev, int mode)
{
	/* FIXME */
	return 0;
}


static int
ath_d80211_test_param(struct net_device *dev, int param, int value)
{
	/* FIXME */
	return 0;
}


static void
ath_d80211_sta_table_notification(struct net_device *dev, int num_sta)
{
	/* FIXME */
}


static int
ath_d80211_conf_tx(struct net_device *dev, int queue,
		   const struct ieee80211_tx_queue_params *params)
{
	/* FIXME */
	return 0;
}


static int
ath_d80211_get_tx_stats(struct net_device *dev,
			struct ieee80211_tx_queue_stats *stats)
{
	/* FIXME */
	return 0;
}


static u64
ath_d80211_get_tsf(struct net_device *dev)
{
	struct ath_softc *sc = ATH_GET_SOFTC(dev);
	struct ath_hal *ah = sc->sc_ah;

	return ath_hal_gettsf64(ah);
}


static void
ath_d80211_reset_tsf(struct net_device *dev)
{
	struct ath_softc *sc = ATH_GET_SOFTC(dev);
	struct ath_hal *ah = sc->sc_ah;

	ath_hal_resettsf(ah);
}


static int
ath_d80211_beacon_update(struct net_device *dev, struct sk_buff *skb,
			 struct ieee80211_tx_control *control)
{
	/* FIXME */
	return 0;
}


static int
ath_d80211_tx_last_beacon(struct net_device *dev)
{
	/* FIXME */
	return 0;
}


static struct ieee80211_hw ath_d80211_hw = {
	.version = IEEE80211_VERSION,
	.name = "atheros",
	.host_gen_beacon = 1,
	.rx_includes_fcs = 1,
	.host_broadcast_ps_buffering = 1,
	.wep_include_iv = 1,
	.data_nullfunc_ack = 1,
	.channel_change_time = 5000,
	.tx = ath_d80211_tx,
	.reset = ath_d80211_reset,
	.open = ath_d80211_open,
	.stop = ath_d80211_stop,
	.add_interface = ath_d80211_add_interface,
	.remove_interface = ath_d80211_remove_interface,
	.config = ath_d80211_config,
	.config_interface = ath_d80211_config_interface,
	.set_key = ath_d80211_set_key,
	.passive_scan = ath_d80211_passive_scan,
	.get_stats = ath_d80211_get_stats,
	.test_mode = ath_d80211_test_mode,
	.test_param = ath_d80211_test_param,
	.sta_table_notification = ath_d80211_sta_table_notification,
	.conf_tx = ath_d80211_conf_tx,
	.get_tx_stats = ath_d80211_get_tx_stats,
	.queues = 1,
	.get_tsf = ath_d80211_get_tsf,
	.reset_tsf = ath_d80211_reset_tsf,
	.beacon_update = ath_d80211_beacon_update,
	.tx_last_beacon = ath_d80211_tx_last_beacon,
};

void
ath_d80211_init_softc(struct ath_softc *sc)
{
	struct ieee80211_hw *hw = &sc->hw_conf;
	int i;

	memcpy(hw, &ath_d80211_hw, sizeof(ath_d80211_hw));

	hw->modes = &sc->hw_modes[0];

	for (i = 0; i < ATH_MAX_HW_MODES; i++) {
		hw->modes[i].num_channels = 0;
		hw->modes[i].channels = &sc->channels[i * ATH_MAX_CHANNELS];
		hw->modes[i].num_rates = 0;
		hw->modes[i].rates = &sc->rates[i * ATH_MAX_RATES];
		hw->num_modes++;
	}

	BUG_ON(ATH_MAX_HW_MODES < 5);

	hw->modes[0].mode = MODE_IEEE80211A;
	hw->modes[1].mode = MODE_IEEE80211B;
	hw->modes[2].mode = MODE_ATHEROS_TURBO;
	hw->modes[3].mode = MODE_IEEE80211G;
	hw->modes[4].mode = MODE_ATHEROS_TURBOG;

	sc->sc_opmode = HAL_M_STA;

	spin_lock_init(&sc->sc_bss_lock);
}

int
ath_d80211_attach(struct net_device *dev)
{
	struct ath_softc *sc = ieee80211_dev_hw_data(dev);
	struct ieee80211_hw *hw = &sc->hw_conf;
	int rv = 0;

	rv = ieee80211_register_hw(dev, hw);
	if (rv) {
		printk(KERN_ERR "%s: device registration failed.\n", dev->name);
	}
	return rv;
}


