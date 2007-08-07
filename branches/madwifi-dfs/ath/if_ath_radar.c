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

#define sizetab(t) (sizeof(t)/sizeof(t[0]))

struct etsi_pulse_prf {
  int width; /* value as specified in the ETSI standards, ie us */
  int width_min; /* value as reported by Atheros hardware */
  int width_max; /* value as reported by Atheros hardware */
	int prf;
	int period_min; /* 1000000 / PRF - 2% */
	int period_max; /* 1000000 / PRF + 2% */
	int burst_min;
};

static struct etsi_pulse_prf etsi_possible_prf[] = {
  /* ETSI radar #1 */
  {  1,  0,  0, 750, 1307, 1359, 15 },
  /* ETSI radar #2 */
  {  1,  0,  0, 200, 4900, 5100, 10 },
  {  1,  0,  0,  300, 3267, 3399, 10 },
  {  1,  0,  0,  500, 1960, 2040, 10 },
  {  1,  0,  0,  800, 1225, 1275, 10 },
  {  1,  0,  0, 1000,  980, 1020, 10 },
  {  2,  4,  5,  200, 4900, 5100, 10 },
  {  2,  4,  5,  300, 3267, 3399, 10 },
  {  2,  4,  5,  500, 1960, 2040, 10 },
  {  2,  4,  5,  800, 1225, 1275, 10 },
  {  2,  4,  5, 1000,  980, 1020, 10 },
  {  5,  7,  9,  200, 4900, 5100, 10 },
  {  5,  7,  9,  300, 3267, 3399, 10 },
  {  5,  7,  9,  500, 1960, 2040, 10 },
  {  5,  7,  9,  800, 1225, 1275, 10 },
  {  5,  7,  9, 1000,  980, 1020, 10 },
  /* ETSI radar #3 */
  { 10, 14, 16,  200, 4900, 5100, 15 },
  { 10, 14, 16,  300, 3267, 3399, 15 },
  { 10, 14, 16,  500, 1960, 2040, 15 },
  { 10, 14, 16,  800, 1225, 1275, 15 },
  { 10, 14, 16, 1000,  980, 1020, 15 },
  { 15, 21, 23,  200, 4900, 5100, 15 },
  { 15, 21, 23,  300, 3267, 3399, 15 },
  { 15, 21, 23,  500, 1960, 2040, 15 },
  { 15, 21, 23,  800, 1225, 1275, 15 },
  { 15, 21, 23, 1000,  980, 1020, 15 },
  /* ETSI radar #4 */
  {  1,  0,  0, 1200,  817,  849, 15 },
  {  1,  0,  0, 1500,  653,  679, 15 },
  {  1,  0,  0, 1600,  613,  637, 15 },
  {  2,  4,  5, 1200,  817,  849, 15 },
  {  2,  4,  5, 1500,  653,  679, 15 },
  {  2,  4,  5, 1600,  613,  637, 15 },
  {  5,  7,  9, 1200,  817,  849, 15 },
  {  5,  7,  9, 1500,  653,  679, 15 },
  {  5,  7,  9, 1600,  613,  637, 15 },
  { 10, 14, 16, 1200,  817,  849, 15 },
  { 10, 14, 16, 1500,  653,  679, 15 },
  { 10, 14, 16, 1600,  613,  637, 15 },
  { 15, 21, 23, 1200,  817,  849, 15 },
  { 15, 21, 23, 1500,  653,  679, 15 },
  { 15, 21, 23, 1600,  613,  637, 15 },
  /* ETSI radar #5 */
  {  1,  0,  0, 2300,  426,  442, 25 },
  {  1,  0,  0, 3000,  327,  339, 25 },
  {  1,  0,  0, 3500,  280,  290, 25 },
  {  1,  0,  0, 4000,  245,  255, 25 },
  {  2,  4,  5, 2300,  426,  442, 25 },
  {  2,  4,  5, 3000,  327,  339, 25 },
  {  2,  4,  5, 3500,  280,  290, 25 },
  {  2,  4,  5, 4000,  245,  255, 25 },
  {  5,  7,  9, 2300,  426,  442, 25 },
  {  5,  7,  9, 3000,  327,  339, 25 },
  {  5,  7,  9, 3500,  280,  290, 25 },
  {  5,  7,  9, 4000,  245,  255, 25 },
  { 10, 14, 16, 2300,  426,  442, 25 },
  { 10, 14, 16, 3000,  327,  339, 25 },
  { 10, 14, 16, 3500,  280,  290, 25 },
  { 10, 14, 16, 4000,  245,  255, 25 },
  { 15, 21, 23, 2300,  426,  442, 25 },
  { 15, 21, 23, 3000,  327,  339, 25 },
  { 15, 21, 23, 3500,  280,  290, 25 },
  { 15, 21, 23, 4000,  245,  255, 25 },
  /* ETSI radar #6 */
  { 20, 27, 29, 2000,  490,  510, 20 },
  { 20, 27, 29, 3000,  327,  339, 20 },
  { 20, 27, 29, 4000,  245,  255, 20 },
  { 30, 39, 41, 2000,  490,  510, 20 },
  { 30, 39, 41, 3000,  327,  339, 20 },
  { 30, 39, 41, 4000,  245,  255, 20 }
};

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

	/* reset the radar pulse circular array */
	ath_radar_pulse_flush(sc);

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

#define ETSI_MAX_MISS 2

/*
  burst : expected number of pulses
*/

static int etsi_radar_match(int count, int miss, int burst)
{
	/* ideal case */
	if (count == burst && miss == 0)
		return 1;

	if ((burst - count) < (burst/2)) {
	  return 1;
	}

	if ((count >= burst / 3)
	    && (miss <= (2*burst)/3)) {
	  return 1;
	}

	/* X start/end pulse missing */
	if ((count+miss+ETSI_MAX_MISS >= burst)
	  && (miss <= ETSI_MAX_MISS)) {
	  return 1;
	}

	/* mid pulse missing */
	if (count == (burst - 1) && miss == 1)
		return 1;

	/* start + end pulse missing */
	if (count == (burst - 2) && miss == 0)
		return 1;

	/* start/end + mid pulse missing */
	if (count == (burst - 2) && miss == 1)
		return 1;

	return 0;
}

void etsi_radar_pulse_analyze_one_pulse(struct ath_softc *sc,
		struct ath_radar_pulse * last_pulse)
{
	struct net_device *dev = sc->sc_dev;
	int i;

	/* We use int64_t instead of u_int64_t since:
	 *  - we compute negative values
	 *  - u_int64_t wrapped around every 507,356 years only ! */

	int64_t t0, t1, t_min, t_max;
	int count, miss, partial_miss;
	struct ath_radar_pulse * pulse;

	DPRINTF(sc, ATH_DEBUG_DOTH,
			"%s: ath_radar_pulse_etsi\n", DEV_NAME(dev));

	/* we need at least (burst_min - 2) pulses and 2 pulses */
	if ((sc->sc_radar_pulse_nr < sc->sc_radar_pulse_burst_min -2)
			|| (sc->sc_radar_pulse_nr < 2)) {
		return;
	}

	/* Search algorithm:
	 *
	 *  - since we have a limited and known number of radar patterns, we
	 *  loop on all possible radar pulse period
	 *
	 *  - we start the search from the last timestamp (t0), going
	 *  backward in time, up to the point for the first possible radar
	 *  pulse, ie t0 - PERIOD * BURST_MAX
	 *
	 *  - on this timescale, we count the number of hit/miss using T -
	 *  PERIOD*n taking into account the 2% error margin (using
	 *  period_min, period_max)
	 *
	 *  At the end, we have a number of pulse hit for each PRF
	 *
	 * Improvements: the PRF can be found using the most common 
	 * interval found */

	/* t1 is the timestamp of the last radar pulse */
	t1 = (int64_t)last_pulse->rp_tsf;

	for (i = 0; i < sizetab(etsi_possible_prf); i++) {
		/* this min formula is to check for underflow */
		t0 = t1 - etsi_possible_prf[i].period_max *
			etsi_possible_prf[i].burst_min;

		/* we directly start with the timestamp before t1 */
		pulse = pulse_prev(last_pulse);

		/* initial values for t_min, t_max */
		t_min = t1 - etsi_possible_prf[i].period_max;
		t_max = t1 - etsi_possible_prf[i].period_min;

		count = 0;
		miss  = 0;
		partial_miss = 0;
#if 0
		DPRINTF(sc, ATH_DEBUG_DOTH,
				"%s: PRF=%4d t1=%10lld t0=%10lld t_min=%10lld "
				"t_max=%10lld\n",
				DEV_NAME(dev),
				etsi_possible_prf[i].prf, t1, t0, t_min, t_max);
#endif
		for (;;) {
#if 0
			DPRINTF(sc, ATH_DEBUG_DOTH,
					"%s: rp_tsf=%10llu t_min = %10lld, t_max = %10lld, "
					"count=%d miss=%d partial_miss=%d\n",
					DEV_NAME(dev),
					pulse->rp_tsf, t_min, t_max, count, miss, partial_miss);
#endif
			/* check if we are at the end of the list */
			if ((&pulse->list == &sc->sc_radar_pulse_head)
					|| (!pulse->rp_allocated)) {
				break;
			}

			/* if we are below t0, stop the loop */
			if ((int64_t)pulse->rp_tsf < t0) {
				break;
			}

			/* check pulse width */
			if (pulse->rp_width < etsi_possible_prf[i].width_min
			    || pulse->rp_width > etsi_possible_prf[i].width_max) {
			  /* ignores this pulse */
#if 1
			  pulse = pulse_prev(pulse);
			  continue ;
#endif
			}

			/* if we miss more than 2 pulses, we stop searching */
			if (partial_miss >= etsi_possible_prf[i].burst_min/2) {
				break;
			}

			if ((int64_t)pulse->rp_tsf > t_max) {
				/* this event is noise, ignores it */
				pulse = pulse_prev(pulse);
			} else if ((int64_t)pulse->rp_tsf >= t_min) {
				/* we found a match */
				count++;
				miss  += partial_miss;

				pulse = pulse_prev(pulse);
				t_min = t_min - etsi_possible_prf[i].period_max;
				t_max = t_max - etsi_possible_prf[i].period_min;
				partial_miss = 0;
			} else {
				t_min = t_min - etsi_possible_prf[i].period_max;
				t_max = t_max - etsi_possible_prf[i].period_min;
				partial_miss++;
			}
		}

		/* print counters for this PRF */
		if (count != 0) {
			/* we add one to the count since we counted only the time
			 * differences */
			count++;

			DPRINTF(sc, ATH_DEBUG_DOTH,
					"%s: PRF [%4d] : %3d pulses %3d miss\n",
					DEV_NAME(dev),
					etsi_possible_prf[i].prf,
					count, miss);

			/* check if PRF counters match a known radar, if we are
			   confident enought */

			if (etsi_radar_match(count, miss,
						etsi_possible_prf[i].burst_min)) {
				DPRINTF(sc, ATH_DEBUG_DOTH,
						"%s: RADAR detected at PRF %4d\n",
						DEV_NAME(dev),
						etsi_possible_prf[i].prf);
				ath_radar_detected(sc, "ETSI RADAR detected");
			}
		}
	}
}

static void etsi_radar_pulse_analyze(struct ath_softc *sc)
{
	struct ath_radar_pulse * pulse;

	/* start the analysis by the last pulse since it might speed up
	   things and then move backward for all non-analyzed pulses */
	list_for_each_entry_reverse(pulse, &sc->sc_radar_pulse_head, list) {
		if (!pulse->rp_allocated) {
			break;
		}

		if (pulse->rp_analyzed) {
			break;
		}

		etsi_radar_pulse_analyze_one_pulse(sc, pulse);
		pulse->rp_analyzed = 1;
	}
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
	sc->sc_radar_pulse_analyze = etsi_radar_pulse_analyze;

	/* compute sc_radar_pulse_burst_min */
	sc->sc_radar_pulse_burst_min = etsi_possible_prf[0].burst_min;
	for (i = 1; i < sizetab(etsi_possible_prf); i++) {
		if (sc->sc_radar_pulse_burst_min > etsi_possible_prf[i].burst_min)
			sc->sc_radar_pulse_burst_min = etsi_possible_prf[i].burst_min;
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
                            u_int64_t tsf, u_int8_t rssi, u_int8_t width)
{
	struct net_device *dev = sc->sc_dev;
	struct ath_radar_pulse * pulse;

	DPRINTF(sc, ATH_DEBUG_DOTH,
			"%s: ath_radar_pulse_record: tsf=%10llu rssi=%3u width=%3u\n",
			DEV_NAME(dev),
			tsf, rssi, width);

	/* check if the new radar pulse is after the last one recorded, or
	 * else, we flush the history */
	pulse = pulse_tail(sc);
	if (tsf < pulse->rp_tsf) {
		ath_radar_pulse_flush(sc);
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
	sc->sc_radar_pulse_nr++;
}

void ath_radar_pulse_print(struct ath_softc *sc)
{
	struct net_device *dev = sc->sc_dev;
	struct ath_radar_pulse * pulse;
	int i;

	DPRINTF(sc, ATH_DEBUG_DOTH,
			"%s: pulse number : %d\n",
			DEV_NAME(dev), sc->sc_radar_pulse_nr);

	DPRINTF(sc, ATH_DEBUG_DOTH,
			"%s: pulse dump using sc_radar_pulse_mem\n",
			DEV_NAME(dev));

	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		pulse = &sc->sc_radar_pulse_mem[i];
		if (!pulse->rp_allocated)
			break;

		DPRINTF(sc, ATH_DEBUG_DOTH,
				"%s: pulse [%3d, %p] : tsf=%10llu rssi=%3u width=%3u "
				"allocated=%d next=%p prev=%p\n",
				DEV_NAME(dev),
				pulse->rp_index, pulse,
				pulse->rp_tsf,
				pulse->rp_rssi,
				pulse->rp_width,
				pulse->rp_allocated,
				pulse->list.next,
				pulse->list.prev);
	}

	DPRINTF(sc, ATH_DEBUG_DOTH,
			"%s: pulse dump using sc_radar_pulse_head\n",
			DEV_NAME(dev));
	list_for_each_entry(pulse, &sc->sc_radar_pulse_head, list) {
		if (!pulse->rp_allocated)
			break;

		DPRINTF(sc, ATH_DEBUG_DOTH,
				"%s: pulse [%3d, %p] : tsf=%10llu rssi=%3u width=%3u "
				"allocated=%d next=%p prev=%p\n",
				DEV_NAME(dev),
				pulse->rp_index, pulse,
				pulse->rp_tsf,
				pulse->rp_rssi,
				pulse->rp_width,
				pulse->rp_allocated,
				pulse->list.next,
				pulse->list.prev);
	}
}

void ath_radar_pulse_flush(struct ath_softc *sc)
{
	struct ath_radar_pulse * pulse;

	list_for_each_entry(pulse, &sc->sc_radar_pulse_head, list) {
		pulse->rp_allocated = 0;
	}
	sc->sc_radar_pulse_nr = 0;
}

