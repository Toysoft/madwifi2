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
#include <linux/param.h>

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

#define sizetab(t) (sizeof(t)/sizeof(t[0]))

struct radar_pattern_specification {
 	/* The name of the rule/specification (i.e. what did we detect) */
 	const char* 	name;
 	/* Interval MIN = 1000000 / FREQ - 2% (a.k.a. Pulse/Burst Repetition Interval) */
 	u_int32_t	repetition_interval_min; 	
 	/* Interval MAX = 1000000 / FREQ + 2% (a.k.a. Pulse/Burst Repetition Interval) */
 	u_int32_t	repetition_interval_max; 	
 	/* Do we adjust the min/max interval values dynamically based upon running mean interval? */
 	HAL_BOOL 	dynamic_interval_adjustment;
 	/* Fuzz factor dynamic matching, as unsigned integer percentage of variation (i.e. 2 for +/- 2% timing) */
 	u_int32_t 	dynamic_interval_adjustment_fuzz_pct;
 	/* Match MIN (Minimum Pulse/Burst events required) */
 	u_int32_t	required_matches;
 	/* Match MIN duration (Minimum Pulse/Burst events required including missed) */
 	u_int32_t	duration_in_intervals_min;
 	/* Match MAX duration (Maximum Pulse/Burst events required including missed) */
 	u_int32_t	duration_in_intervals_max;
 	/* Maximum consecutive missing pulses */
 	u_int32_t       maximum_consecutive_missing;
 	/* Maximum missing pulses */
 	u_int32_t       maximum_missing;
};

/* We will use 1% after initial pulse for FCC */
#define FUZZF 10
/* We will use 2% after initial pulse for ETSI */
#define FUZZE 20

/*
PRF are ordered from high to low, because lower frequencies will match higher ones
and the debug output is more reassuring the closer we get to actual PRF.
*/
static struct radar_pattern_specification radar_patterns[] = {
/*        name                   min intvl     max intvl     dynamic   fuzz   min     interval  max    max    
                                 pulse 0       pulse 0       intvl     1/10%  matches range     consec total  
			                                                              min,max   miss   miss   
*/                               
	{ "ETSI [ 200]",         4900,         5100,         AH_TRUE,  FUZZE, 3,      4, 10,    4,     8     },
	{ "ETSI [ 300]",         3267,         3399,         AH_TRUE,  FUZZE, 3,      4, 10,    4,     6     },
	{ "ETSI [ 500]",         1960,         2040,         AH_TRUE,  FUZZE, 4,      4, 10,    4,     8     },
	{ "ETSI [ 750]",         1307,         1359,         AH_TRUE,  FUZZE, 4,      4, 15,    4,     13    },
	{ "ETSI [ 800]",         1225,         1275,         AH_TRUE,  FUZZE, 4,      4, 10,    4,     8     },
	{ "ETSI [1000]",         980,          1020,         AH_TRUE,  FUZZE, 4,      4, 10,    4,     8     },
	{ "ETSI [1200]",         817,          849,          AH_TRUE,  FUZZE, 4,      4, 15,    4,     13    },
        { "ETSI [1500]",         653,          679,          AH_TRUE,  FUZZE, 4,      4, 15,    4,     6     },
	{ "ETSI [1600]",         613,          637,          AH_TRUE,  FUZZE, 4,      4, 15,    4,     7     },
	{ "ETSI [2000]",         490,          510,          AH_TRUE,  FUZZE, 4,      4, 20,    4,     10    },
	{ "ETSI [2300]",         426,          442,          AH_TRUE,  FUZZE, 4,      4, 25,    4,     20    },
	{ "ETSI [3000]",         327,          339,          AH_TRUE,  FUZZE, 3,      4, 25,    5,     20    },
	{ "ETSI [3500]",         280,          290,          AH_TRUE,  FUZZE, 4,      4, 25,    2,     20    },
	{ "ETSI [4000]",         245,          255,          AH_TRUE,  FUZZE, 4,      4, 25,    5,     20    },
        { "FCC [1,1399-1714]",   1399,         1714,         AH_TRUE,  FUZZF, 5,      10,18,    4,     6     },
        { "FCC [2,147-235]",     147,          235,          AH_TRUE,  FUZZF, 8,      10,29,    6,     12    },
        { "FCC [3-4,196-273]",   196,          273,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
        { "FCC [3-4,275-352]",   275,          352,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
        { "FCC [3-4,354-431]",   354,          431,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
        { "FCC [3-4,433-510]",   433,          510,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
        { "FCC [3-4,235-313]",   235,          313,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
        { "FCC [3-4,314-392]",   314,          392,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
        { "FCC [3-4,393-471]",   393,          471,          AH_TRUE,  FUZZF, 8,      8, 18,    2,     16    },
};
#undef MN5
#undef MX5

static inline u_int32_t interval_to_frequency(u_int32_t pri);

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
	rp->rp_fir_filter_output_power_thr = 
		(radar & AR5K_PHY_RADAR_FIRPWROUTTHR) 
		>> AR5K_PHY_RADAR_FIRPWROUTTHR_S;
	rp->rp_radar_rssi_thr = 
		(radar & AR5K_PHY_RADAR_PULSERSSITHR) 
		>> AR5K_PHY_RADAR_PULSERSSITHR_S;
	rp->rp_pulse_height_thr = 
		(radar & AR5K_PHY_RADAR_PULSEHEIGHTTHR) 
		>> AR5K_PHY_RADAR_PULSEHEIGHTTHR_S;
	rp->rp_pulse_rssi_thr = 
		(radar & AR5K_PHY_RADAR_RADARRSSITHR) 
		>> AR5K_PHY_RADAR_RADARRSSITHR_S;
	rp->rp_inband_thr = 
		(radar & AR5K_PHY_RADAR_INBANDTHR) 
		>> AR5K_PHY_RADAR_INBANDTHR_S;
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
			rp_fir_filter_output_power_thr) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_RADARRSSITHR,
			AR5K_PHY_RADAR_RADARRSSITHR_S,
			rp_pulse_rssi_thr) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_PULSEHEIGHTTHR,
			AR5K_PHY_RADAR_PULSEHEIGHTTHR_S,
			rp_pulse_height_thr) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_PULSERSSITHR,
			AR5K_PHY_RADAR_PULSERSSITHR_S,
			rp_radar_rssi_thr) |
		BUILD_PHY_RADAR_FIELD(
			AR5K_PHY_RADAR_INBANDTHR,
			AR5K_PHY_RADAR_INBANDTHR_S,
			rp_inband_thr)
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

	/* Do not attempt to change radar state when bg scanning is
	   the cause */
	if (ic->ic_flags & IEEE80211_F_SCAN)
		return 1;

	/* Update the DfFS flags (as a sanity check) */
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
		if (DFLAG_ISSET(sc, ATH_DEBUG_DOTH) && ((old_radar != new_radar) || 
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
 * XXX: Need to add regulatory rules in here.  This is too conservative! */
int ath_radar_is_dfs_required(struct ath_softc *sc, HAL_CHANNEL *hchan)
{
	/* For FCC: 5250 to 5350MHz (channel 52 to 60) and for Europe added 
	 * 5470 to 5725 MHz (channel 100 to 140). 
	 * Being conservative, go with the entire band from 5250-5725 MHz. */
	return ((hchan->channel >= 5250) && (hchan->channel <= 5725)) ? 1 : 0;
}

static struct ath_radar_pulse * pulse_head(struct ath_softc *sc)
{
	return list_entry(sc->sc_radar_pulse_head.next,
			struct ath_radar_pulse, list);
}

static struct ath_radar_pulse * pulse_tail(struct ath_softc *sc)
{
	return list_entry(sc->sc_radar_pulse_head.prev,
			struct ath_radar_pulse, list);
}

static struct ath_radar_pulse * pulse_prev(struct ath_radar_pulse *pulse)
{
	return list_entry(pulse->list.prev,
			struct ath_radar_pulse, list);
}

#if 0
static struct ath_radar_pulse * pulse_next(struct ath_radar_pulse *pulse)
{
	return list_entry(pulse->list.next,
			struct ath_radar_pulse, list);
}
#endif

/*
 * This accounts for hardware inconsistencies when detecting/returning pulses.  4% was not good enough for FCC bin 5 matching.
*/
static HAL_BOOL radar_pulse_analyze_one_pulse(
	struct ath_softc *sc, 
	struct ath_radar_pulse * last_pulse,
	u_int32_t* index,
	u_int32_t* pri,
	u_int32_t* matching_pulses,
	u_int32_t* missed_pulses,
	u_int32_t* noise_pulses
	)
{
	struct net_device *dev = sc->sc_dev;
	int i;
	int best_index = -1;
	unsigned int best_matched   = 0;
	unsigned int best_noise  = 0;
	unsigned int best_missed = 0;
	unsigned int best_pri    = 0;
	unsigned int best_code = 0;

	u_int64_t t0_min, t0_max, t1, t_min, t_max;
	u_int32_t noise = 0, matched = 0, missed = 0, partial_miss = 0;
	struct ath_radar_pulse * pulse;
	u_int32_t pulse_count_minimum = 0;

	u_int32_t new_period       = 0;
	u_int64_t last_seen_tsf    = 0;
	u_int32_t last_seen_period = 0;
	u_int32_t sum_periods      = 0;
	u_int32_t mean_period 	   = 0;
	u_int32_t adjusted_repetition_interval_max = 0;
	u_int32_t adjusted_repetition_interval_min = 0;

	if(index) 
		*index = 0;
	if(pri) 
		*pri = 0;
	if(noise_pulses)
		*noise_pulses = 0;
	if(matching_pulses)
		*matching_pulses = 0;
	if(missed_pulses)
		*missed_pulses = 0;

	/* we need at least sc_radar_pulse_minimum_to_match pulses or 2 pulses */
	pulse_count_minimum = sc->sc_radar_pulse_minimum_to_match;
	if ((sc->sc_radar_pulse_nr < pulse_count_minimum) || (sc->sc_radar_pulse_nr < 2))
		return 0;

	/* Search algorithm:
	 *
	 *  - since we have a limited and known number of radar patterns, we
	 *  loop on all possible radar pulse period
	 *
	 *  - we start the search from the last timestamp (t1), going
	 *  backward in time, up to the point for the first possible radar
	 *  pulse, ie t0_min - PERIOD * BURST_MAX
	 *
	 *  - on this timescale, we matched the number of hit/missed using T -
	 *  PERIOD*n taking into account the 2% error margin (using
	 *  repetition_interval_min, repetition_interval_max)
	 *
	 *  At the end, we have a number of pulse hit for each PRF
	 *
         * TSF will roll over after just over 584,542 years of operation without restart.
	 * This exceeds all known Atheros MTBF so, forget about TSF roll over.
	 */

	/* t1 is the timestamp of the last radar pulse */
	t1 = (u_int64_t)last_pulse->rp_tsf;

	/* loop through all patterns */
	for (i = 0; i < sizetab(radar_patterns); i++) {

		/* underflow */
		if ((radar_patterns[i].repetition_interval_min * (radar_patterns[i].duration_in_intervals_min-1)) >= t1) {
			DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
				"%s: %s skipped (last pulse isn't old enough to match this pattern).  %10llu >= %10llu.\n",
				DEV_NAME(dev),
				radar_patterns[i].name, (u_int64_t)(radar_patterns[i].repetition_interval_min * radar_patterns[i].duration_in_intervals_min), (u_int64_t)t1);
			continue;
		}

		/* this min formula is to check for underflow.  It's the minimum needed duration to gather
		 * specified number of matches, assuming minimum match interval. */
		t0_min  = (radar_patterns[i].repetition_interval_min * radar_patterns[i].duration_in_intervals_min) < t1 ? 
			t1 - (radar_patterns[i].repetition_interval_min * radar_patterns[i].duration_in_intervals_min) :
			0;
		/* this max formula is to stop when we exceed maximum time period for the pattern.  It's the 
		   oldest possible TSF that could match. */
		t0_max  = (radar_patterns[i].repetition_interval_max * radar_patterns[i].duration_in_intervals_max) < t1 ? 
			t1 - (radar_patterns[i].repetition_interval_max * radar_patterns[i].duration_in_intervals_max) :
			0;

		/* we directly start with the timestamp before t1 */
		pulse = pulse_prev(last_pulse);

		/* initial values for t_min, t_max */
		t_min = radar_patterns[i].repetition_interval_max < t1 ?
			t1 - radar_patterns[i].repetition_interval_max :
			0;
		t_max = radar_patterns[i].repetition_interval_min < t1 ?
			t1 - radar_patterns[i].repetition_interval_min :
			0;

		last_seen_tsf = t1;
		last_seen_period = 0;
		sum_periods = 0;
		matched = 0;
		noise = 0;
		missed  = 0;
		partial_miss = 0;
		mean_period = 0;
		adjusted_repetition_interval_max = radar_patterns[i].repetition_interval_max;
		adjusted_repetition_interval_min = radar_patterns[i].repetition_interval_min;
		/* DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
				"%s: %s t1=%10lld t0_min=%10lld t_min=%10lld t_max=%10lld\n",
				DEV_NAME(dev),
				radar_patterns[i].name, t1, t0_min, t_min, t_max); */
		for (;;) {
			adjusted_repetition_interval_max = ((radar_patterns[i].dynamic_interval_adjustment == AH_TRUE) && mean_period && (mean_period * (1000+radar_patterns[i].dynamic_interval_adjustment_fuzz_pct) / 1000) < radar_patterns[i].repetition_interval_max) ? 
				(mean_period * (1000+radar_patterns[i].dynamic_interval_adjustment_fuzz_pct) / 1000) : 
				radar_patterns[i].repetition_interval_max;
			adjusted_repetition_interval_min = ((radar_patterns[i].dynamic_interval_adjustment == AH_TRUE) && mean_period && (mean_period *  (1000-radar_patterns[i].dynamic_interval_adjustment_fuzz_pct) / 1000) > radar_patterns[i].repetition_interval_min) ? 
				(mean_period * (1000-radar_patterns[i].dynamic_interval_adjustment_fuzz_pct) / 1000) : 
				radar_patterns[i].repetition_interval_min;
			/* DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: rp_tsf=%10llu t_min = %10lld, t_max = %10lld, "
					"matched=%d missed=%d partial_miss=%d\n",
					DEV_NAME(dev),
					pulse->rp_tsf, t_min, t_max, matched, missed, partial_miss); */
			/* check if we are at the end of the list */
			if ((&pulse->list == &sc->sc_radar_pulse_head) || (!pulse->rp_allocated)) {
				break;
			}

			/* Do not go too far... this is an optimization to not keep checking after we hit maximum time span 
			 * for the pattern. */
			if (pulse->rp_tsf < t0_max) {
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %s matching stopped (pulse->rp_tsf < t0_max).  t1=%10llu t0_max=%10llu t_min=%10llu "
					"t_max=%10llu matched=%u missed=%u\n",
					DEV_NAME(dev),
					radar_patterns[i].name, t1, t0_max, t_min, t_max, matched, missed);
				break;
			}
			/* if we missed more than specified number of pulses, we stop searching */
			if (partial_miss > radar_patterns[i].maximum_consecutive_missing) {
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
						"%s: %s matching stopped (too many consecutive pulses missing). %d>%d  matched=%u. missed=%u.\n",
						DEV_NAME(dev),
						radar_patterns[i].name, partial_miss, radar_patterns[i].maximum_consecutive_missing, matched, missed);
				break;
			}

		        new_period = (u_int64_t)(last_seen_tsf && last_seen_tsf > pulse->rp_tsf) ? last_seen_tsf - pulse->rp_tsf : 0;
			if ( pulse->rp_tsf > t_max ) {
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %-17s [%2d] %5s [**:**] tsf: %10llu [range: %5llu-%5llu].  width: %3d. period: %4llu. last_period: %4llu. mean_period: %4llu. last_tsf: %10llu.\n", 
				       DEV_NAME(dev),
				       radar_patterns[i].name,
					pulse->rp_index,
					"noise",
				       (u_int64_t)pulse->rp_tsf,
				       (u_int64_t)t_min, 
				       (u_int64_t)t_max,
				       (u_int8_t)pulse->rp_width,
				       (u_int64_t)new_period,
				       (u_int64_t)last_seen_period,
				       (u_int64_t)mean_period,
				       (u_int64_t)last_seen_tsf
				       );
				/* this event is noise, ignore it */
				pulse = pulse_prev(pulse);
				noise++;
			} else if (pulse->rp_tsf >= t_min) {
				/* we found a match */
				matched++;
				new_period = partial_miss ? new_period/(partial_miss+1) : new_period;
				missed  += partial_miss;
				partial_miss = 0;
				sum_periods += new_period;
				mean_period = matched ? (sum_periods/matched) : 0;
				if(mean_period && 
				   (AH_TRUE == radar_patterns[i].dynamic_interval_adjustment) &&
				   (mean_period > radar_patterns[i].repetition_interval_max || 
				    mean_period < radar_patterns[i].repetition_interval_min)) {
					DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%s: %s mean period deviated from original range [period: %4u, range: %4u-%4u]\n",
							DEV_NAME(dev),
							radar_patterns[i].name,
							mean_period,
							radar_patterns[i].repetition_interval_min,
							radar_patterns[i].repetition_interval_max);
					break;
				}

				/* Remember we are scanning backwards... */
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %-17s [%2d] %5s [%2d:%-2d] tsf: %10llu [range: %5llu-%5llu].  width: %3d. period: %4llu. last_period: %4llu. mean_period: %4llu. last_tsf: %10llu.\n", 
				       DEV_NAME(dev),
				       radar_patterns[i].name,
					pulse->rp_index,
					"match",
					(matched + missed + partial_miss) ? (matched + missed + partial_miss) - 1 : 0,
					(matched + missed + partial_miss),
				       (u_int64_t)pulse->rp_tsf,
				       (u_int64_t)t_min, 
				       (u_int64_t)t_max,
				       (u_int8_t)pulse->rp_width,
				       (u_int64_t)new_period,
				       (u_int64_t)last_seen_period,
				       (u_int64_t)mean_period,
				       (u_int64_t)last_seen_tsf
				       );
				/* record tsf and period */
				last_seen_period  = new_period;
				last_seen_tsf     = pulse->rp_tsf;

				/* advance to next pulse */
				pulse = pulse_prev(pulse);

				/* update bounds */
				t_min = adjusted_repetition_interval_max < t_min ? 
					t_min - adjusted_repetition_interval_max :
					0;
				t_max = adjusted_repetition_interval_min < t_max ?
					t_max - adjusted_repetition_interval_min :
					0;
			} else {
				partial_miss++;
				/* if we missed more than specified number of pulses, we stop searching */
				if ((missed+partial_miss) > radar_patterns[i].maximum_missing) {
					DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%s: %s matching stopped (too many total pulses missing). %d>%d  matched=%u. missed=%u.\n",
							DEV_NAME(dev),
							radar_patterns[i].name, missed, radar_patterns[i].maximum_missing, matched, missed);
					break;
				}
				/* Default mean period to approximate center of range */
				/* Remember we are scanning backwards... */
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %-17s [**] %5s [%2d:%-2d] tsf:  (missing) [range: %5llu-%5llu].  width: ***. period: ****. last_period: %4llu. mean_period: %4llu. last_tsf: %10llu.\n", 
				        DEV_NAME(dev),
				        radar_patterns[i].name,
					"missed",
					(matched + missed + partial_miss) ? (matched + missed + partial_miss) - 1 : 0,
					(matched + missed + partial_miss),
					(u_int64_t)t_min, 
					(u_int64_t)t_max,
					(u_int64_t)last_seen_period,
					(u_int64_t)mean_period,
					(u_int64_t)last_seen_tsf);

				/* update bounds */
				t_min = adjusted_repetition_interval_max < t_min ? 
					t_min - adjusted_repetition_interval_max :
					0;
				t_max = adjusted_repetition_interval_min < t_max ?
					t_max - adjusted_repetition_interval_min :
					0;
			}
		}

		/* print counters for this PRF */
		if (matched > 1) {
			const char* match_status = NULL;
			/* we add one to the matched since we counted only the time
			 * differences */
			matched++;

			match_status =
				(matched+missed) < radar_patterns[i].duration_in_intervals_min ? 
					"TOO-SHORT" :
				(matched > radar_patterns[i].duration_in_intervals_max) ?
					"TOO-LONG" :
				(matched < radar_patterns[i].required_matches) ?
					"TOO-LOSSY" :
				"MATCH";

			DPRINTF(sc, ATH_DEBUG_DOTHFILT,
				"%s: [%02d] %13s: %-17s"
				"[match=%2u {%2u..%2u},missed=%2u/%2u,dur=%2d {%2u..%2u},noise=%2u/%2u]\n",
				DEV_NAME(dev),
				last_pulse->rp_index,
				match_status,
				radar_patterns[i].name,
				matched, radar_patterns[i].required_matches, radar_patterns[i].duration_in_intervals_max, 
				missed, radar_patterns[i].maximum_missing, 
				matched+missed,
				radar_patterns[i].duration_in_intervals_min, 
				radar_patterns[i].duration_in_intervals_max, 
				noise, matched+noise);

			/* check if PRF counters match a known radar, if we are
			   confident enought */
			if ((matched+missed) >= radar_patterns[i].duration_in_intervals_min &&
			    matched >= radar_patterns[i].required_matches) {
				/*
				Scoring: 
					matches <= max 
					closer matches 
					duration <= max, 
					closer duration
                                */
				int better = (-1 == best_index) ? 1 : 0;
				/* best hits is over maximum and new one is not */
				if(!better && 
				    (matched    	 		<= radar_patterns[i].duration_in_intervals_max) && 
				    (best_matched 			> radar_patterns[best_index].duration_in_intervals_max) ) {
					better = 2;
				}
				/* both hits are under maximum and new one is higher */
				else if(!better &&
					(matched     			<= radar_patterns[i].duration_in_intervals_max) && 
					(best_matched 			<= radar_patterns[best_index].duration_in_intervals_max) && 
					(matched     			> best_matched)) {
					better = 3;
				}
				/* both hits are over maximum and new one is lower */
				else if(!better && 
					(matched     			> radar_patterns[i].duration_in_intervals_max) && 
					(best_matched 			> radar_patterns[best_index].duration_in_intervals_max) && 
					(matched     			< best_matched)) {
					better = 4;
				}
				/* best duration is over maximum and new one is not */
				else if(!better && 
					(matched == best_matched) && 
					((matched + missed) 		<= radar_patterns[i].duration_in_intervals_max) && 
					((best_matched + best_missed) 	>  radar_patterns[best_index].duration_in_intervals_max)) {
					better = 5;
				}
				/* both durations are under maximum and new one is LONGER */
				else if(!better && 
					(matched == best_matched) && 
					((matched + missed) 			<= radar_patterns[i].duration_in_intervals_max) && 
					((best_matched + best_missed) 		<= radar_patterns[best_index].duration_in_intervals_max) && 
					(((matched + missed) * mean_period) 	> ((best_matched + best_missed) * best_pri))) {
					better = 6;
				}
				/* both durations are over maximum and new one is SHORTER */
				else if(!better && 
					(matched == best_matched) && 
					((matched + missed) 			> radar_patterns[i].duration_in_intervals_max) && 
					((best_matched + best_missed) 		> radar_patterns[best_index].duration_in_intervals_max) && 
					(((matched + missed) * mean_period) 	< ((best_matched + best_missed) * best_pri))) {
					better = 7;
				}
				/* both durations are under maximum and new match has shorter max duration in intervals  */
				else if(!better && 
					(matched == best_matched) && 
					((matched + missed) 			<= radar_patterns[i].duration_in_intervals_max) && 
					((best_matched + best_missed) 		<= radar_patterns[best_index].duration_in_intervals_max) && 
					(((matched + missed) * mean_period) 	== ((best_matched + best_missed) * best_pri)) &&
					(radar_patterns[i].duration_in_intervals_max < radar_patterns[best_index].duration_in_intervals_max)) {
					better = 8;
				}
				/* both durations are under maximum and new match has closer to original center frequency */
/*				else if(!better && 
					(matched == best_matched) && 
					((matched + missed) 			<= radar_patterns[i].duration_in_intervals_max) && 
					((best_matched + best_missed) 		<= radar_patterns[best_index].duration_in_intervals_max) && 
					labs(mean_period - (radar_patterns[i].repetition_interval_min 		+ ((radar_patterns[i].repetition_interval_max - radar_patterns[i].repetition_interval_min) / 2))) <
					labs(best_pri    - (radar_patterns[best_index].repetition_interval_min 	+ ((radar_patterns[best_index].repetition_interval_max - radar_patterns[best_index].repetition_interval_min) / 2)))) {
					better = 9;
				}
*/
				if (better) {
					best_matched 	= matched;
					best_missed 	= missed;
					best_index 	= i;
					best_pri 	= mean_period;
					best_noise	= noise;
					best_code       = better;
				}
				/* If nobody is looking, short-circuilt on first match. */
//				if (!(DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILTNOSC) && !DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILT)))
//					break;
			}
			else {
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %s matching failed (insufficient score) matched: %d missed: %d duration_in_intervals_min: %d\n",
					DEV_NAME(dev),
					radar_patterns[i].name,
					matched,missed,radar_patterns[i].duration_in_intervals_min);
			}
		}
	}
	if (-1 != best_index) {
		DPRINTF(sc, ATH_DEBUG_DOTHFILT,
			"%s: [%02d] %10s: %-17s"
			"[match=%2u {%2u..%2u},missed=%2u/%2u,dur=%2d {%2u..%2u},noise=%2u/%2u,code=%2d] RI=%-9u RF=%-4u\n",
			DEV_NAME(dev),
			last_pulse->rp_index,
			"BEST MATCH",
			radar_patterns[best_index].name,
			best_matched, radar_patterns[best_index].required_matches, radar_patterns[best_index].duration_in_intervals_max, 
			best_missed, radar_patterns[best_index].maximum_missing, 
			best_matched + best_missed,
			radar_patterns[best_index].duration_in_intervals_min, 
			radar_patterns[best_index].duration_in_intervals_max, 
			best_noise,
			best_matched + best_noise,
			best_code,
			best_pri,
			interval_to_frequency(best_pri));

		if(index) 
			*index 		 = best_index;
		if(pri) 
			*pri 		 = best_pri;
		if(matching_pulses)
			*matching_pulses = best_matched;
		if(noise_pulses)
			*noise_pulses    = best_noise;
		if(missed_pulses)
			*missed_pulses   = best_missed;
	}

    return -1 != best_index ? AH_TRUE : AH_FALSE;
}

static inline u_int32_t interval_to_frequency(u_int32_t interval)
{
	/* Calculate BRI from PRI */
	u_int32_t frequency = interval ? (1000000 / interval) : 0;
	/* Round to nearest multiple of 50 */
//	return frequency + ((frequency % 10) >= 5 ? 10 : 0) - (frequency % 10);
	return frequency + ((frequency % 50) >= 25 ? 50 : 0) - (frequency % 50);
}

static HAL_BOOL radar_pulse_analyze(struct ath_softc *sc)
{
        HAL_BOOL radar = 0;
	struct ath_radar_pulse * pulse;
	u_int32_t best_index = 0;
	u_int32_t best_pri = 0;
	u_int32_t best_matched = 0;
	u_int32_t best_missed = 0;
	u_int32_t best_noise = 0;
	u_int32_t pass = 0;

	/* start the analysis by the last pulse since it might speed up
	 * things and then move backward for all non-analyzed pulses.
	 * For debugging ONLY - we continue to run this scan after radar is detected,
	 * processing all pulses... even when they come in after an iteration of all 
	 * pulses that were present when this function was invoked.  This can happen
	 * at some radar waveforms where we will match the first few pulses and then the
	 * rest of the burst will come in, but never be analyzed.
	 * Under normal conditions, skip all that and just detect the darn radar.
	 * This doesn't actually work as the tasklet must finish for the rx tasklet to run and fill the queue again,
	 * but it should work for simulation/debugging.
	 */
	while(pulse_tail(sc)->rp_allocated && !pulse_tail(sc)->rp_analyzed && 
	      (AH_FALSE == radar || (DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILT) && ++pass <= 3)))
	{
		list_for_each_entry_reverse(pulse, &sc->sc_radar_pulse_head, list) {
			if (!pulse->rp_allocated) {
				break;
			}
	
			if (pulse->rp_analyzed) {
				break;
			}
	
			/* Skip pulse analysis after we have confirmed radar presence unless we 
			 * are debugging and have disabled short-circuit logic.  In this case,
			 * we'll go through ALL the signatures and find the best match.
			 */
			if(AH_FALSE == radar || DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILTNOSC)) {
				u_int32_t index = 0, pri = 0, hit = 0, missed = 0, noise = 0;
				if(AH_TRUE == radar_pulse_analyze_one_pulse(sc, pulse, &index, &pri, &hit, &missed, &noise)) {
					if ((AH_FALSE == radar) || (hit > best_matched) || (hit == best_matched && missed < best_missed)) {
						best_matched   = hit;
						best_missed = missed;
						best_index  = index;
						best_pri    = pri;
						best_noise  = noise;
						radar       = AH_TRUE;
					}
				}
			}
			pulse->rp_analyzed = 1;
		}
	}
	if (AH_TRUE == radar) {
		if (DFLAG_ISSET(sc, ATH_DEBUG_DOTHPULSES)) {
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"%s: ========================================\n",
					DEV_NAME(sc->sc_dev));	
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"%s: ==BEGIN RADAR SAMPLE====================\n",
					DEV_NAME(sc->sc_dev));	
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"%s: ========================================\n",
					DEV_NAME(sc->sc_dev));	
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES, 
				"%s: Sample contains data matching %s "
				"[match=%2u {%2u..%2u},missed=%2u/%2u,dur=%2d {%2u..%2u},noise=%2u/%2u] RI=%-9u RF=%-4u\n",
				DEV_NAME(sc->sc_dev),
				radar_patterns[best_index].name, 
				best_matched, radar_patterns[best_index].required_matches, radar_patterns[best_index].duration_in_intervals_max, 
				best_missed, radar_patterns[best_index].maximum_missing, 
				best_matched + best_missed,
				radar_patterns[best_index].duration_in_intervals_min, 
				radar_patterns[best_index].duration_in_intervals_max, 
				best_noise,
				best_noise + best_matched,
				best_pri,
				interval_to_frequency(best_pri));
			ath_radar_pulse_print(sc, 0 /* analyzed pulses only */);
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"%s: ========================================\n",
					DEV_NAME(sc->sc_dev));	
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"%s: ==END RADAR SAMPLE======================\n",
					DEV_NAME(sc->sc_dev));	
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"%s: ========================================\n",
					DEV_NAME(sc->sc_dev));	
		}
		ath_radar_detected(sc, radar_patterns[best_index].name);
	}
	return radar;
}

/* initialize ath_softc members so sensible values */
static void ath_radar_pulse_safety_belt(struct ath_softc *sc)
{
	sc->sc_radar_pulse_mem     = NULL;
	INIT_LIST_HEAD(&sc->sc_radar_pulse_head);
	sc->sc_radar_pulse_nr      = 0;
	sc->sc_radar_pulse_analyze = NULL;
}

static void
ath_radar_pulse_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	if (sc->sc_radar_pulse_analyze != NULL) {
		sc->sc_radar_pulse_analyze(sc);
	}
}

void ath_radar_pulse_init(struct ath_softc *sc)
{
	struct net_device *dev = sc->sc_dev;
	int i;

	ath_radar_pulse_safety_belt(sc);

	sc->sc_radar_pulse_mem = (struct ath_radar_pulse *) kmalloc(
			sizeof(struct ath_radar_pulse) * ATH_RADAR_PULSE_NR, GFP_KERNEL);
	if (sc->sc_radar_pulse_mem == NULL)
		return ;

	/* initialize the content of the array */
	memset(sc->sc_radar_pulse_mem, 0,
			sizeof(struct ath_radar_pulse) * ATH_RADAR_PULSE_NR);

	/* initialize the circular list */
	INIT_LIST_HEAD(&sc->sc_radar_pulse_head);
	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		sc->sc_radar_pulse_mem[i].rp_index = i;
		list_add_tail(&sc->sc_radar_pulse_mem[i].list,
				&sc->sc_radar_pulse_head);
	}

	sc->sc_radar_pulse_nr = 0;
	sc->sc_radar_pulse_analyze = radar_pulse_analyze;

	/* compute sc_radar_pulse_minimum_to_match */
	sc->sc_radar_pulse_minimum_to_match = radar_patterns[0].duration_in_intervals_min;
	for (i = 1; i < sizetab(radar_patterns); i++) {
		if (sc->sc_radar_pulse_minimum_to_match > radar_patterns[i].duration_in_intervals_min)
			sc->sc_radar_pulse_minimum_to_match = radar_patterns[i].duration_in_intervals_min;
	}

	ATH_INIT_TQUEUE(&sc->sc_radartq, ath_radar_pulse_tasklet, dev);
}

void ath_radar_pulse_done(struct ath_softc *sc)
{
    /* free what we allocated in ath_radar_pulse_init() */
    kfree (sc->sc_radar_pulse_mem);

    ath_radar_pulse_safety_belt(sc);
}

void ath_radar_pulse_record(struct ath_softc *sc,
                            u_int64_t tsf, u_int8_t rssi, u_int8_t width,
			    HAL_BOOL is_simulated)
{
	struct net_device *dev = sc->sc_dev;
	struct ath_radar_pulse * pulse;

	DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
		"%s: ath_radar_pulse_record: tsf=%10llu rssi=%3u width=%3u\n",
		DEV_NAME(dev),
		tsf, rssi, width);

	/* check if the new radar pulse is after the last one recorded, or
	 * else, we flush the history */
	pulse = pulse_tail(sc);
	if (tsf < pulse->rp_tsf) {
		if (is_simulated == AH_TRUE && 0 == tsf) {
			DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %s: ath_radar_pulse_flush: simulated tsf reset.  tsf =%10llu, rptsf =%10llu\n",
					DEV_NAME(dev),
					__func__,
					tsf, pulse->rp_tsf);
			ath_radar_pulse_flush(sc);
		}
		else if ((pulse->rp_tsf - tsf) > (1<<15)) {
			DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%s: %s: ath_radar_pulse_flush: tsf reset.  (rp_tsf - tsf > 0x8000) tsf =%10llu, rptsf =%10llu\n",
					DEV_NAME(dev),
					__func__,
					tsf, pulse->rp_tsf);
			ath_radar_pulse_flush(sc);
		}
		else {
			DPRINTF(sc, ATH_DEBUG_DOTHFILT,
					"%s: %s: tsf jitter/bug detected: tsf =%10llu, rptsf =%10llu, rp_tsf - tsf = %10llu\n",
					DEV_NAME(dev),
					__func__,
					tsf, pulse->rp_tsf, pulse->rp_tsf - tsf);
		}
	}

	/* remove the head of the list */
	pulse = pulse_head(sc);
	list_del(&pulse->list);

	pulse->rp_tsf       = tsf;
	pulse->rp_rssi      = rssi;
	pulse->rp_width     = width;
	pulse->rp_allocated = 1;
	pulse->rp_analyzed  = 0;

	/* add at the tail of the list */
	list_add_tail(&pulse->list, &sc->sc_radar_pulse_head);
	if (ATH_RADAR_PULSE_NR > sc->sc_radar_pulse_nr)
		sc->sc_radar_pulse_nr++;
}

void ath_radar_pulse_print_mem(struct ath_softc *sc, int analyzed_pulses_only)
{
	struct net_device *dev = sc->sc_dev;
	struct ath_radar_pulse * pulse;
	u_int64_t oldest_tsf = ~0;
	int i;
	printk("%s: pulse dump of %spulses using sc_radar_pulse_mem containing  %d allocated pulses.\n",
		DEV_NAME(dev),
		analyzed_pulses_only ? "analyzed " : "",
		sc->sc_radar_pulse_nr);

	/* Find oldest TSF value so we can print relative times */
	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		pulse = &sc->sc_radar_pulse_mem[i];
		if (pulse->rp_allocated && pulse->rp_tsf < oldest_tsf)
			oldest_tsf = pulse->rp_tsf;
	}

	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		pulse = &sc->sc_radar_pulse_mem[i];
		if (!pulse->rp_allocated)
			break;
		if ( (!analyzed_pulses_only) || pulse->rp_analyzed) 
			printk("%s: pulse [%3d, %p] : relative_tsf=%10llu tsf=%10llu rssi=%3u width=%3u "
				"allocated=%d analyzed=%d next=%p prev=%p\n",
				DEV_NAME(dev),
				pulse->rp_index, 
				pulse,
				pulse->rp_tsf - oldest_tsf,
				pulse->rp_tsf,
				pulse->rp_rssi,
				pulse->rp_width,
				pulse->rp_allocated,
				pulse->rp_analyzed,
				pulse->list.next,
				pulse->list.prev);
	}
}

void ath_radar_pulse_print(struct ath_softc *sc, int analyzed_pulses_only)
{
	struct net_device *dev = sc->sc_dev;
	struct ath_radar_pulse * pulse;
	u_int64_t oldest_tsf = ~0;

	printk("%s: pulse dump of %spulses from ring buffer containing %d pulses.\n",
		DEV_NAME(dev),
		analyzed_pulses_only ? "analyzed " : "", 
		sc->sc_radar_pulse_nr);

	/* Find oldest TSF value so we can print relative times */
	oldest_tsf = ~0;
	list_for_each_entry_reverse(pulse, &sc->sc_radar_pulse_head, list) {
		if (pulse->rp_allocated && pulse->rp_tsf < oldest_tsf)
			oldest_tsf = pulse->rp_tsf;
	}

	list_for_each_entry_reverse(pulse, &sc->sc_radar_pulse_head, list) {
		if (!pulse->rp_allocated)
			continue;
		if ( (!analyzed_pulses_only) || pulse->rp_analyzed) 
			printk("%s: pulse [%3d, %p] : relative_tsf=%10llu tsf=%10llu rssi=%3u width=%3u "
				"allocated=%d analyzed=%d next=%p prev=%p\n",
				DEV_NAME(dev),
				pulse->rp_index, 
				pulse,
				pulse->rp_tsf - oldest_tsf,
				pulse->rp_tsf,
				pulse->rp_rssi,
				pulse->rp_width,
				pulse->rp_allocated,
				pulse->rp_analyzed,
				pulse->list.next,
				pulse->list.prev);
	}
}

void ath_radar_pulse_flush(struct ath_softc *sc)
{
	struct ath_radar_pulse * pulse;

	list_for_each_entry_reverse(pulse, &sc->sc_radar_pulse_head, list) {
		pulse->rp_allocated = 0;
	}
	sc->sc_radar_pulse_nr = 0;
}

