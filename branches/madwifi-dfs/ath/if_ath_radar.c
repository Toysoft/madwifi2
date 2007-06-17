/*
 * This software is distributed under the terms of the
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
 * $Id: if_ath_radar.c 2464 2007-06-15 22:51:56Z mtaylor $
 */
#include "opt_ah.h"

#define	AR_DEBUG
#include "if_ath_debug.h"

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <linux/time.h>
#include <asm/uaccess.h>

#include "if_ethersubr.h"		/* for ETHER_IS_MULTICAST */
#include "if_media.h"
#include "if_llc.h"

#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_monitor.h>
#include <net80211/ieee80211_rate.h>

#ifdef USE_HEADERLEN_RESV
#include <net80211/if_llc.h>
#endif

#define	AR_DEBUG

#include "net80211/if_athproto.h"
#include "if_athvar.h"

#include "ah_desc.h"

#include "ah_devid.h"			/* XXX to identify chipset */

#ifdef ATH_PCI		/* PCI BUS */
#include "if_ath_pci.h"
#endif			/* PCI BUS */
#ifdef ATH_AHB		/* AHB BUS */
#include "if_ath_ahb.h"
#endif			/* AHB BUS */

#include "ah.h"
#include "if_ath_hal.h"

#ifdef ATH_TX99_DIAG
#include "ath_tx99.h"
#endif

#include "ah_os.h"
#include "if_ath_radar.h"

/* Returns true if radar detection is enabled. */
int ath_radar_is_enabled(struct ath_softc *sc) {
	struct ath_hal *ah = sc->sc_ah;
	if (ar_device(sc->devid) >= 5211)
		return (OS_REG_READ(ah, AR5K_AR5212_PHY_ERR_FIL) & 
				AR5K_AR5212_PHY_ERR_FIL_RADAR) && 
			(sc->sc_imask & HAL_INT_RXPHY) && 
			(ath_hal_intrget(ah) & HAL_INT_RXPHY);
	else
		return (sc->sc_imask & HAL_INT_RXPHY) && 
			(ath_hal_intrget(ah) & HAL_INT_RXPHY);
	return 0;
}

/* Read the radar pulse detection parameters. */
void ath_radar_get_params(struct ath_softc *sc, RADAR_PARAM* rp) {
	u_int32_t radar = ath_reg_read(sc, AR5K_PHY_RADAR);
	rp->rp_fir_filter_output_power_threshold = 
		(radar & AR5K_PHY_RADAR_FIRPWROUTTHR) 
		>> AR5K_PHY_RADAR_FIRPWROUTTHR_S;
	rp->rp_inband_threshold = 
		(radar & AR5K_PHY_RADAR_INBANDTHR) 
		>> AR5K_PHY_RADAR_INBANDTHR_S;
	rp->rp_radar_rssi_threshold = 
		(radar & AR5K_PHY_RADAR_PULSERSSITHR) 
		>> AR5K_PHY_RADAR_PULSERSSITHR_S;
	rp->rp_pulse_rssi_threshold = 
		(radar & AR5K_PHY_RADAR_RADARRSSITHR) 
		>> AR5K_PHY_RADAR_RADARRSSITHR_S;
	rp->rp_pulse_rssi_threshold = 
		(radar & AR5K_PHY_RADAR_PULSEHEIGHTTHR) 
		>> AR5K_PHY_RADAR_PULSEHEIGHTTHR_S;
}

/* Update the radar pulse detection parameters. 
 * If rp is NULL, defaults are used for all fields.
 * If any member of rp is set to RADAR_PARAM_USE_DEFAULT, the default
 * is used for that field. */
void ath_radar_set_params(struct ath_softc *sc, RADAR_PARAM* rp) {

#define BUILD_PHY_RADAR_FIELD(_MASK,_SHIFT,_FIELD) \
	((NULL == rp || (rp->_FIELD == RADAR_PARAM_USE_DEFAULT)) ? \
		((AR5K_PHY_RADAR_ENABLED_AR5213 & (_MASK))) : \
		((rp->_FIELD << (_SHIFT)) & (_MASK)))

	ath_reg_write(sc, AR5K_PHY_RADAR, 
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_FIRPWROUTTHR,
			AR5K_PHY_RADAR_FIRPWROUTTHR_S, 
			rp_fir_filter_output_power_threshold) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_INBANDTHR,
			AR5K_PHY_RADAR_INBANDTHR_S,
			rp_inband_threshold) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_PULSERSSITHR,
			AR5K_PHY_RADAR_PULSERSSITHR_S,
			rp_radar_rssi_threshold) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_RADARRSSITHR,
			AR5K_PHY_RADAR_RADARRSSITHR_S,
			rp_pulse_rssi_threshold) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_PULSEHEIGHTTHR,
                        AR5K_PHY_RADAR_PULSEHEIGHTTHR_S,
			rp_pulse_height_threshold)
		);
#undef BUILD_PHY_RADAR_FIELD
}
/* This is called on channel change to enable radar detection for 5211+ chips.  
 * NOTE: AR5210 doesn't have radar pulse detection support. */
int ath_radar_update(struct ath_softc *sc) {

	struct ath_hal *ah = sc->sc_ah;
	struct net_device *dev = sc->sc_dev;
	struct ieee80211com *ic = &sc->sc_ic;
	int required = 0;

	/* Do not attmpet to change radar state when bg scanning is the cause */
	if (ic->ic_flags & IEEE80211_F_SCAN)
		return 1;

	/* Update the DFS flags (as a sanity check) */
	if (ath_radar_correct_dfs_flags(sc, &sc->sc_curchan))
		DPRINTF(sc, ATH_DEBUG_DOTH, 
			"%s: %s: channel required corrections to private flags.\n", 
			DEV_NAME(dev), __func__);
	required = ath_radar_is_dfs_required(sc, &sc->sc_curchan);
	/* configure radar pulse detector register using default values, but do
	 * not toggle the enable bit.  XXX: allow tweaking?? */
	ath_radar_set_params(sc, NULL);
	if (ar_device(sc->devid) >= 5211) {
		HAL_INT old_ier = ath_hal_intrget(ah);
		HAL_INT new_ier = old_ier;
		unsigned int old_radar  = OS_REG_READ(ah, AR5K_PHY_RADAR);
		unsigned int old_filter = OS_REG_READ(ah, AR5K_AR5212_PHY_ERR_FIL);
		unsigned int old_rxfilt = ath_hal_getrxfilter(ah);
		unsigned int old_mask   = sc->sc_imask;
		unsigned int new_radar  = old_radar;
		unsigned int new_filter = old_filter;
		unsigned int new_mask   = old_mask;
		unsigned int new_rxfilt = old_rxfilt;

		ath_hal_intrset(ah, old_ier & ~HAL_INT_GLOBAL);
		if (required) {
			new_radar    |= AR5K_PHY_RADAR_ENABLE;
			new_filter   |= AR5K_AR5212_PHY_ERR_FIL_RADAR;
			new_rxfilt   |= (HAL_RX_FILTER_PHYERR | HAL_RX_FILTER_PHYRADAR);
			new_mask     |= HAL_INT_RXPHY;
			new_ier      |= HAL_INT_RXPHY;
		} else {
			new_radar    &= ~AR5K_PHY_RADAR_ENABLE;
			new_filter   &= ~AR5K_AR5212_PHY_ERR_FIL_RADAR;
			new_rxfilt   &= ~HAL_RX_FILTER_PHYRADAR;
			new_mask     &= ~HAL_INT_RXPHY;
			new_ier      &= ~HAL_INT_RXPHY;
		}

		if (old_filter != new_filter)
			OS_REG_WRITE(ah, AR5K_AR5212_PHY_ERR_FIL, new_filter);
		if (old_radar != new_radar)
			OS_REG_WRITE(ah, AR5K_PHY_RADAR,   new_radar);
		if (old_rxfilt != new_rxfilt)
			ath_hal_setrxfilter(ah, new_rxfilt);

		sc->sc_imask = new_mask;
		if (IFF_DUMPPKTS(sc, ATH_DEBUG_DOTH) && ((old_radar != new_radar) || 
				(old_filter != new_filter) || (old_rxfilt != new_rxfilt) || 
				(old_mask != new_mask) || (old_ier != new_ier)))
			DPRINTF(sc, ATH_DEBUG_DOTH, "%s: %s: Radar detection %s.\n", DEV_NAME(dev), __func__, required ? "enabled" : "disabled");
		ath_hal_intrset(ah, new_ier);
	}

	return (required == ath_radar_is_enabled(sc));
}

/* Update channel's DFS flags based upon whether DFS is reqired.  Return
 * true if the value was repaired. */
int ath_radar_correct_dfs_flags(struct ath_softc *sc, HAL_CHANNEL *hchan) {
	u_int32_t old_channelFlags  = hchan->channelFlags;
	u_int32_t old_privFlags     = hchan->privFlags;
	if (ath_radar_is_dfs_required(sc, hchan)) {
		hchan->channelFlags |= CHANNEL_PASSIVE;
		hchan->privFlags    |= CHANNEL_DFS;
	}
	else {
		hchan->channelFlags &= ~CHANNEL_PASSIVE;
		hchan->privFlags    &= ~CHANNEL_DFS;
	}
	return (old_privFlags != hchan->privFlags) || 
		(old_channelFlags != hchan->channelFlags);
}

/* Returns true if DFS is required for the regulatory domain, country and 
 * combination in use. 
 * XXX: Need to add regulatory rules in here.  This is too conservative!
 */
int ath_radar_is_dfs_required(struct ath_softc *sc, HAL_CHANNEL *hchan)
{
	/* For FCC: 5.25 to 5.35GHz (channel 52 to 60) and for Europe added 
	 * 5.47 to 5.725GHz (channel 100 to 140). 
	 * Being conservative, go with the entire band from 5225-5725 MHz. */
	return ((hchan->channel >= 5225) && (hchan->channel <= 5725)) ? 1 : 0;
}

