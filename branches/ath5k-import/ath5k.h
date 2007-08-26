/*
 * Copyright (C) 2006 Devicescape Software, Inc. All Rights Reserved.
 * Copyright (C) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _IF_ATH5K_H_
#define _IF_ATH5K_H_

#include <linux/netdevice.h>
#include <net/mac80211.h>
#include "ath5k_hw.h"

static struct {
	u_int   ath_mode; /* hal phy mode */
	int     mac80211_mode; /* mac80211 phy mode */
} ath5k_mode_map[] = {
	{ AR5K_MODE_11A,  MODE_IEEE80211A       },
	{ AR5K_MODE_11B,  MODE_IEEE80211B       },
	{ AR5K_MODE_11G,  MODE_IEEE80211G       },
	{ AR5K_MODE_TURBO, MODE_ATHEROS_TURBO   },
	{ AR5K_MODE_108G, MODE_ATHEROS_TURBOG   },
};

struct ath_softc *ath5k_alloc(size_t priv_size);
void ath5k_free(struct ath_softc *sc);
int ath5k_attach(struct ath_softc *sc);
void ath5k_detach(struct ath_softc *sc);
int ath5k_add_channels(struct ath_softc *sc, int hw_mode,
	AR5K_CHANNEL *hal_chans, int hal_nchan,
	int hal_flags);
int ath5k_rate_setup(struct ath_softc *sc, u_int hal_mode,
	const struct ar5k_rate_table *rt);

/**
 * ath5k_mode_to_mac80211_mode - Convert an atheros mode to an IEEE80211 mode.
 * @ath_mode: atheros hardware mode (AR5K_MODE_11A, AR5K_MODE_11B, ...)
 *
 * Returns ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 */
static int
ath5k_mode_to_mac80211_mode(u_int ath_mode)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(ath5k_mode_map); i++) {
                if (ath5k_mode_map[i].ath_mode == ath_mode)
                        return ath5k_mode_map[i].mac80211_mode;
        }
        printk(KERN_ERR "Invalid ath5k mode.\n");
        return ath5k_mode_map[0].mac80211_mode;
}

/**
 * mac80211_mode_to_ath_mode - Convert an IEEE80211 mode to atheros mode.
 * @mac80211_mode: ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 *
 * Returns atheros hardware mode (AR5K_MODE_11A, AR5K_MODE_11B, ...)
 */
static int
mac80211_mode_to_ath_mode(u_int mac80211_mode)
{
        int i;
        for (i = 0; i < sizeof(ath5k_mode_map) / sizeof(ath5k_mode_map[0]); i++) {
                if (ath5k_mode_map[i].mac80211_mode == mac80211_mode)
                        return ath5k_mode_map[i].ath_mode;
        }
        printk(KERN_ERR "Invalid mac80211 mode.\n");
        return ath5k_mode_map[0].ath_mode;
}

#endif /* _IF_ATH5K_H*/
