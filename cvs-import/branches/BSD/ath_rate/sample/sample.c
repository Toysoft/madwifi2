/*-
 * Copyright (c) 2005 John Bicket
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
 */


/*
 * John Bicket's SampleRate control algorithm.
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>

#include <asm/uaccess.h>

#include <net80211/if_media.h>
#include <net80211/ieee80211_var.h>

#include "if_athvar.h"
#include "ah_desc.h"

#include "sample.h"

#define	SAMPLE_DEBUG
#ifdef SAMPLE_DEBUG
enum {
	ATH_DEBUG_RATE		= 0x00000010	/* rate control */
};
#define	DPRINTF(sc, _fmt, ...) do {				\
	if (sc->sc_debug & ATH_DEBUG_RATE)			\
		printf(_fmt, __VA_ARGS__);			\
} while (0)
#else
#define	DPRINTF(sc, _fmt, ...)
#endif

/*
 * This file is an implementation of the SampleRate algorithm
 * in "Bit-rate Selection in Wireless Networks"
 * (http://www.pdos.lcs.mit.edu/papers/jbicket-ms.ps)
 *
 * SampleRate chooses the bit-rate it predicts will provide the most
 * throughput based on estimates of the expected per-packet
 * transmission time for each bit-rate.  SampleRate periodically sends
 * packets at bit-rates other than the current one to estimate when
 * another bit-rate will provide better performance. SampleRate
 * switches to another bit-rate when its estimated per-packet
 * transmission time becomes smaller than the current bit-rate's.
 * SampleRate reduces the number of bit-rates it must sample by
 * eliminating those that could not perform better than the one
 * currently being used.  SampleRate also stops probing at a bit-rate
 * if it experiences several successive losses.
 *
 * The difference between the algorithm in the thesis and the one in this
 * file is that the one in this file uses a ewma instead of a window.
 *
 */


static char *version = "1.1";
static char *dev_info = "ath_rate_sample";


#define STALE_FAILURE_TIMEOUT_MS 10000

static	int ath_smoothing_rate = 95;	/* ewma percentage (out of 100) */
static	int ath_sample_rate = 10;	/* use x% of transmission time 
					 * sending at a different bit-rate */

static void ath_rate_ctl_reset(struct ath_softc *, struct ieee80211_node *);




static __inline int size_to_bin(int size) 
{
	int x = 0;
	for (x = 0; x < NUM_PACKET_SIZE_BINS; x++) {
		if (size <= packet_size_bins[x]) {
			return x;
		}
	}
	return NUM_PACKET_SIZE_BINS-1;
}
static __inline int bin_to_size(int index) {
	return packet_size_bins[index];
}

static __inline int rate_to_ndx(struct sample_node *sn, int rate) {
	int x = 0;
	for (x = 0; x < sn->num_rates; x++) {
		if (sn->rates[x].rate == rate) {
			return x;
		}      
	}
	return -1;
}


void
ath_rate_node_init(struct ath_softc *sc, struct ath_node *an){
	/* NB: assumed to be zero'd by caller */
	ath_rate_ctl_reset(sc,&an->an_node);
}
EXPORT_SYMBOL(ath_rate_node_init);

void
ath_rate_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{

}
EXPORT_SYMBOL(ath_rate_node_cleanup);

void
ath_rate_node_copy(struct ath_softc *sc,
		   struct ath_node *dst, const struct ath_node *src)
{
	struct sample_node *odst = ATH_NODE_SAMPLE(dst);
	const struct sample_node *osrc = (const struct sample_node *)&src[1];
	memcpy(odst, osrc, sizeof(struct sample_node));
}
EXPORT_SYMBOL(ath_rate_node_copy);


/*
 * returns the ndx with the lowest average_tx_time,
 * or -1 if all the average_tx_times are 0.
 */
static __inline int best_rate_ndx(struct sample_node *sn, int size_bin, 
				  int require_acked_before)
{
	int x = 0;
        int best_rate_ndx = 0;
        int best_rate_tt = 0;
        for (x = 0; x < sn->num_rates; x++) {
		int tt = sn->stats[size_bin][x].average_tx_time;
		if (tt <= 0 || (require_acked_before && 
				!sn->stats[size_bin][x].packets_acked)) {
			continue;
		}
		if (!best_rate_tt || best_rate_tt > tt) {
			best_rate_tt = tt;
			best_rate_ndx = x;
		}
        }
        return (best_rate_tt) ? best_rate_ndx : -1;
}

/*
 * pick a ndx s.t. the perfect_tx_time
 * is less than the best bit-rate's average_tx_time
 * and the ndx has not had four successive failures.
 */
static __inline int pick_sample_ndx(struct sample_node *sn, int size_bin) 
{
	int x = 0;
	int current_ndx = 0;
	unsigned current_tt = 0;
	
	current_ndx = sn->current_rate[size_bin];
	if (current_ndx < 0) {
		/* no successes yet, send at the lowest bit-rate */
		return 0;
	}
	
	current_tt = sn->stats[size_bin][current_ndx].average_tx_time;
	
	for (x = 0; x < sn->num_rates; x++) {
		int ndx = (sn->last_sample_ndx[size_bin] + 1 + x) % sn->num_rates;
		/* 
		 * clear any stale stuff out.
		 */
		if (jiffies - sn->stats[size_bin][ndx].last_tx > ((HZ * STALE_FAILURE_TIMEOUT_MS)/1000)) {
			sn->stats[size_bin][ndx].average_tx_time = sn->stats[size_bin][ndx].perfect_tx_time;
			sn->stats[size_bin][ndx].successive_failures = 0;
			sn->stats[size_bin][ndx].tries = 0;
			sn->stats[size_bin][ndx].total_packets = 0;
			sn->stats[size_bin][ndx].packets_acked = 0;
		}

		if (ndx != current_ndx &&
		    sn->stats[size_bin][ndx].perfect_tx_time < current_tt &&
		    sn->stats[size_bin][ndx].successive_failures < 4) {
			sn->last_sample_ndx[size_bin] = ndx;
			return ndx;
		}
	}
	return current_ndx;
}


void
ath_rate_findrate(struct ath_softc *sc, struct ath_node *an,
		  int shortPreamble, size_t frameLen,
		  u_int8_t *rix, int *try0, u_int8_t *txrate)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	struct sample_softc *ssc = ATH_SOFTC_SAMPLE(sc);
	struct ieee80211com *ic = &sc->sc_ic;
	int ndx, size_bin, mrr, best_ndx;
	unsigned average_tx_time;

	mrr = sc->sc_mrretry && !(ic->ic_flags & IEEE80211_F_USEPROT) && 
		!(frameLen > ic->ic_rtsthreshold);
	size_bin = size_to_bin(frameLen);
	best_ndx = best_rate_ndx(sn, size_bin, !mrr);

	if (best_ndx >= 0) {
		average_tx_time = sn->stats[size_bin][best_ndx].average_tx_time;
	} else {
		average_tx_time = 0;
	}
	
	if (sn->static_rate_ndx != -1) {
		ndx = sn->static_rate_ndx;
		*try0 = ATH_TXMAXTRY;
	} else {
		*try0 = mrr ? 2 : ATH_TXMAXTRY;
		
		if (!sn->packets_sent[size_bin]) {
			/* no packets sent */
			if (best_ndx == -1) {
				ndx = sn->num_rates - 1;
				if (sc->sc_curmode != IEEE80211_MODE_11B) {
					for (; ndx >= 0 && sn->rates[ndx].rate > 72; ndx--)
						;
						
				}
			} else {
				ndx = best_ndx;
			}
		} else if (best_ndx == -1) {
			/* no packet has succeeded yet */
			if (mrr) {
				/* 
				 * no packet has succeeded, try the
				 * highest bitrate that hasn't failed 
				 */  
				for (ndx = sn->num_rates-1; ndx >= 0; ndx--) {
					if (sn->stats[size_bin][ndx].successive_failures == 0) {
						break;
					}
				}
			} else {
				ndx = 0;
			}
		} else if (sn->sample_tt[size_bin] < (sn->packets_since_sample[size_bin]*ssc->ath_sample_rate/100) * average_tx_time &&
			   sn->packets_since_sample[size_bin] > 15) {
			/*
			 * we want to limit the time measuring the performance
			 * of other bit-rates to ath_sample_rate% of the
			 * total transmission time.
			 */
			ndx = pick_sample_ndx(sn, size_bin);
			if (ndx != sn->current_rate[size_bin]) {
#if 0
				DPRINTF(sc, "%s: %s size %d last sample tt %d sampling %d packets since %d\n",
					dev_info,
					ether_sprintf(an->an_node.ni_macaddr),
					packet_size_bins[size_bin],
					sn->sample_tt[size_bin], 
					sn->rates[ndx].rate,
					sn->packets_since_sample[size_bin]);
#endif
				sn->current_sample_ndx[size_bin] = ndx;
			} else {
				sn->current_sample_ndx[size_bin] = -1;
			}
			sn->packets_since_sample[size_bin] = 0;

		} else {
			sn->packets_since_sample[size_bin]++;
			/*
			 * don't switch bit-rates every packet.  only
			 * switch during the first few packets we send
			 * or after 100 packets, or if the current
			 * bit-rate begins to perform twice as bad as
			 * another one.
			 */
			if (sn->packets_sent[size_bin] < 20 ||
			    jiffies - ((HZ*2000)/1000) > sn->jiffies_since_switch[size_bin] ||
			    average_tx_time * 2 < sn->stats[size_bin][sn->current_rate[size_bin]].average_tx_time ) {
				if (sn->packets_sent[size_bin] > 20) {
					DPRINTF(sc, "%s: %s size %d switch rate %d (%d/%d) -> %d (%d/%d) after %d packets mmr %d\n",
						dev_info,
						ether_sprintf(an->an_node.ni_macaddr),
						packet_size_bins[size_bin],
						sn->rates[sn->current_rate[size_bin]].rate,
						sn->stats[size_bin][sn->current_rate[size_bin]].average_tx_time,
						sn->stats[size_bin][sn->current_rate[size_bin]].perfect_tx_time,
						sn->rates[best_ndx].rate,
						sn->stats[size_bin][best_ndx].average_tx_time,
						sn->stats[size_bin][best_ndx].perfect_tx_time,
						sn->packets_since_switch[size_bin],
						mrr);
				}
				sn->packets_since_switch[size_bin] = 0;
				sn->current_rate[size_bin] = best_ndx;
				sn->jiffies_since_switch[size_bin] = jiffies;
			}
			ndx = sn->current_rate[size_bin];
			sn->packets_since_switch[size_bin]++;
			if (size_bin == 0) {
	    			/* 
	    			 * set the visible txrate for this node
			         * to the rate of small packets
			         */
				an->an_node.ni_txrate = ndx;
			}
		}
	}
	if (ndx < 0) {
		ndx = 0;
	}

	*rix = sn->rates[ndx].rix;
	if (shortPreamble) {
		*txrate = sn->rates[ndx].shortPreambleRateCode;
	} else {
		*txrate = sn->rates[ndx].rateCode;
	}
	sn->packets_sent[size_bin]++;
}
EXPORT_SYMBOL(ath_rate_findrate);

void
ath_rate_setupxtxdesc(struct ath_softc *sc, struct ath_node *an,
		      struct ath_desc *ds, int shortPreamble, u_int8_t rix)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	int rateCode = -1;
	int frame_size = 0;
	int size_bin = 0;
	int best_ndx = 0;
	int ndx = -1;

	size_bin = size_to_bin(frame_size);
	best_ndx = best_rate_ndx(sn, size_bin, 0);
	frame_size = ds->ds_ctl0 & 0x0fff; /* low-order 12 bits of ds_ctl0 */
	

	if (best_ndx == -1 || !sn->stats[size_bin][best_ndx].packets_acked) {
		/* 
		 * no packet has succeeded, so also try at the
		 * lowest bitate.
		 */
		ndx = 0;
	} else {
		/*
		 * we're trying a different bit-rate, and it could be lossy, 
		 * so if it fails try at the best bit-rate.
		 */
		ndx = best_ndx;
	}
	if (ndx != -1) {
		if (shortPreamble) {
			rateCode = sn->rates[ndx].shortPreambleRateCode;
		} else {
			rateCode = sn->rates[ndx].rateCode;
		}
		ath_hal_setupxtxdesc(sc->sc_ah, ds
				     , rateCode, 3	        /* series 1 */
				     , sn->rates[0].rateCode, 3	/* series 2 */
				     , 0, 0	                /* series 3 */
				     );
	}
	
}
EXPORT_SYMBOL(ath_rate_setupxtxdesc);



void update_stats(struct ath_softc *sc, struct ath_node *an, 
		  int frame_size,
		  int ndx0, int tries0,
		  int ndx1, int tries1,
		  int ndx2, int tries2,
		  int ndx3, int tries3,
		  int short_tries, int tries, int status)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	struct sample_softc *ssc = ATH_SOFTC_SAMPLE(sc);
	int tt = 0;
	int tries_so_far = 0;
	int size_bin = 0;
	int size = 0;
	int rate = 0;

	size_bin = size_to_bin(frame_size);
	size = bin_to_size(size_bin);
	rate = sn->rates[ndx0].rate;

	tt += calc_usecs_unicast_packet(sc, size, sn->rates[ndx0].rix, 
					short_tries-1, 
					MIN(tries0, tries) - 1);
	tries_so_far += tries0;
	if (tries1 && tries0 < tries) {
		tt += calc_usecs_unicast_packet(sc, size, sn->rates[ndx1].rix, 
						short_tries-1, 
						MIN(tries1 + tries_so_far, tries) - tries_so_far - 1);
	}
	tries_so_far += tries1;

	if (tries2 && tries0 + tries1 < tries) {
		tt += calc_usecs_unicast_packet(sc, size, sn->rates[ndx2].rix, 
					       short_tries-1, 
						MIN(tries2 + tries_so_far, tries) - tries_so_far - 1);
	}

	tries_so_far += tries2;

	if (tries3 && tries0 + tries1 + tries2 < tries) {
		tt += calc_usecs_unicast_packet(sc, size, sn->rates[ndx3].rix, 
						short_tries-1, 
						MIN(tries3 + tries_so_far, tries) - tries_so_far - 1);
	}
	
	if (0 && (short_tries + tries > 3 || status)) {
		DPRINTF(sc, "%s: %s size %d rate %d ndx %d tries (%d/%d) tries0 %d tt %d avg_tt %d perfect_tt %d status %d\n", 
			dev_info, ether_sprintf(an->an_node.ni_macaddr),
			size, 
			rate, ndx0, short_tries, tries, tries0, tt, 
			sn->stats[size_bin][ndx0].average_tx_time,
			sn->stats[size_bin][ndx0].perfect_tx_time,
			status);
	}
	
	if (sn->stats[size_bin][ndx0].total_packets < (100 / (100 - ssc->ath_smoothing_rate))) {
		/* just average the first few packets */
		int avg_tx = sn->stats[size_bin][ndx0].average_tx_time;
		int packets = sn->stats[size_bin][ndx0].total_packets;
		sn->stats[size_bin][ndx0].average_tx_time = (tt+(avg_tx*packets))/(packets+1);
	} else {
		/* use a ewma */
		sn->stats[size_bin][ndx0].average_tx_time = 
			((sn->stats[size_bin][ndx0].average_tx_time * ssc->ath_smoothing_rate) + 
			 (tt * (100 - ath_smoothing_rate))) / 100;
	}
	
	if (status) {
		/* 
		 * this packet failed - count this as a failure
		 * for larger packets also, since we assume
		 * if a small packet fails at a lower bit-rate 
		 * then a larger one will also.
		 */
		int y;
		for (y = size_bin; y < NUM_PACKET_SIZE_BINS; y++) {
			sn->stats[y][ndx0].successive_failures++;
			sn->stats[y][ndx0].last_tx = jiffies;
		}
	} else {
		sn->stats[size_bin][ndx0].packets_acked++;
		sn->stats[size_bin][ndx0].successive_failures = 0;
	}
	sn->stats[size_bin][ndx0].tries += tries;
	sn->stats[size_bin][ndx0].last_tx = jiffies;
	sn->stats[size_bin][ndx0].total_packets++;


	if (ndx0 == sn->current_sample_ndx[size_bin]) {
		DPRINTF(sc, "%s: %s size %d sample rate %d tries (%d/%d) tt %d avg_tt (%d/%d) status %d\n", 
			dev_info, ether_sprintf(an->an_node.ni_macaddr), 
			size, rate, short_tries, tries, tt, 
			sn->stats[size_bin][ndx0].average_tx_time,
			sn->stats[size_bin][ndx0].perfect_tx_time,
			status);
		sn->sample_tt[size_bin] = tt;
		sn->current_sample_ndx[size_bin] = -1;
	}

}
void
ath_rate_tx_complete(struct ath_softc *sc,
		     struct ath_node *an, const struct ath_desc *ds)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	const struct ar5212_desc *ads = (const struct ar5212_desc *)&ds->ds_ctl0;
	int final_rate = 0;
	int short_tries = 0;
	int long_tries = 0;
	int ndx = -1;
	int frame_size = 0;

	final_rate = sc->sc_hwmap[ds->ds_txstat.ts_rate &~ HAL_TXSTAT_ALTRATE].ieeerate;
	short_tries = ds->ds_txstat.ts_shortretry + 1;
	long_tries = ds->ds_txstat.ts_longretry + 1;
	frame_size = ds->ds_ctl0 & 0x0fff; /* low-order 12 bits of ds_ctl0 */

	if (frame_size == 0) {
		frame_size = 1500;
	}
	if (sn->num_rates <= 0) {
		DPRINTF(sc, "%s: %s %s no rates yet\n", dev_info, 
			ether_sprintf(an->an_node.ni_macaddr), __func__);
		return;
	}

	if (sc->sc_mrretry && ds->ds_txstat.ts_status) {
		/* this packet failed */
		DPRINTF(sc, "%s: %s size %d rate/try %d/%d %d/%d %d/%d %d/%d status %s retries (%d/%d)\n", 
			dev_info,
			ether_sprintf(an->an_node.ni_macaddr),
			bin_to_size(size_to_bin(frame_size)),
			sc->sc_hwmap[ads->xmit_rate0].ieeerate, ads->xmit_tries0,
			sc->sc_hwmap[ads->xmit_rate1].ieeerate, ads->xmit_tries1,
			sc->sc_hwmap[ads->xmit_rate2].ieeerate, ads->xmit_tries2,
			sc->sc_hwmap[ads->xmit_rate3].ieeerate, ads->xmit_tries3,
			ds->ds_txstat.ts_status ? "FAIL" : "OK",
			short_tries, 
			long_tries);
	}


	if (!(ds->ds_txstat.ts_rate & HAL_TXSTAT_ALTRATE)) {
		/* only one rate was used */
		ndx = rate_to_ndx(sn, final_rate);
		if (ndx >= 0 && ndx < sn->num_rates) {
			update_stats(sc, an, frame_size, 
				     ndx, long_tries,
				     0, 0,
				     0, 0,
				     0, 0,
				     short_tries, long_tries, ds->ds_txstat.ts_status);
		}
	} else {
		int rate0, tries0, ndx0;
		int rate1, tries1, ndx1;
		int rate2, tries2, ndx2;
		int rate3, tries3, ndx3;
		int finalTSIdx = ads->final_ts_index;

		/*
		 * Process intermediate rates that failed.
		 */

		rate0 = sc->sc_hwmap[ads->xmit_rate0].ieeerate;
		tries0 = ads->xmit_tries0;
		ndx0 = rate_to_ndx(sn, rate0);
		
		rate1 = sc->sc_hwmap[ads->xmit_rate1].ieeerate;
		tries1 = ads->xmit_tries1;
		ndx1 = rate_to_ndx(sn, rate1);
		
		rate2 = sc->sc_hwmap[ads->xmit_rate2].ieeerate;
		tries2 = ads->xmit_tries2;
		ndx2 = rate_to_ndx(sn, rate2);
		
		rate3 = sc->sc_hwmap[ads->xmit_rate3].ieeerate;
		tries3 = ads->xmit_tries3;
		ndx3 = rate_to_ndx(sn, rate3);
		
#if 0
		DPRINTF(sc, "%s: %s size %d finaltsidx %d tries %d status %d rate/try %d/%d %d/%d %d/%d %d/%d\n", 
			dev_info, ether_sprintf(an->an_node.ni_macaddr),
			bin_to_size(size_to_bin(frame_size)),
			finalTSIdx,
			long_tries, 
			ds->ds_txstat.ts_status,
			rate0, tries0,
			rate1, tries1,
			rate2, tries2,
			rate3, tries3);
#endif

		if (tries0) {
			update_stats(sc, an, frame_size, 
				     ndx0, tries0, 
				     ndx1, tries1, 
				     ndx2, tries2, 
				     ndx3, tries3, 
				     short_tries, ds->ds_txstat.ts_longretry + 1, 
				     ds->ds_txstat.ts_status);
		}
		
		if (tries1 && finalTSIdx > 0) {
			update_stats(sc, an, frame_size, 
				     ndx1, tries1, 
				     ndx2, tries2, 
				     ndx3, tries3, 
				     0, 0, 
				     short_tries, ds->ds_txstat.ts_longretry + 1 - tries0, 
				     ds->ds_txstat.ts_status);
		}

		if (tries2 && finalTSIdx > 1) {
			update_stats(sc, an, frame_size, 
				     ndx2, tries2, 
				     ndx3, tries3, 
				     0, 0,
				     0, 0,
				     short_tries, ds->ds_txstat.ts_longretry + 1 - tries0 - tries1, 
				     ds->ds_txstat.ts_status);
		}

		if (tries3 && finalTSIdx > 2) {
			update_stats(sc, an, frame_size, 
				     ndx3, tries3, 
				     0, 0,
				     0, 0,
				     0, 0,
				     short_tries, ds->ds_txstat.ts_longretry + 1 - tries0 - tries1 - tries2, 
				     ds->ds_txstat.ts_status);
		}
	}
}
EXPORT_SYMBOL(ath_rate_tx_complete);

void
ath_rate_newassoc(struct ath_softc *sc, struct ath_node *an, int isnew)
{
	DPRINTF(sc, "%s: %s %s\n", dev_info,
		ether_sprintf(an->an_node.ni_macaddr), __func__);
	if (isnew)
		ath_rate_ctl_reset(sc, &an->an_node);
}
EXPORT_SYMBOL(ath_rate_newassoc);



/*
 * Initialize the tables for a node.
 */
static void
ath_rate_ctl_reset(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_node *an = ATH_NODE(ni);
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	const HAL_RATE_TABLE *rt = sc->sc_currates;
	const struct ieee80211_rateset *rs;
	int r, x, y;
	int srate;

	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));
        sn->static_rate_ndx = -1;
        DPRINTF(sc, "%s: %s size 1600 rate/tt", dev_info, ether_sprintf(ni->ni_macaddr));

	sn->num_rates = ni->ni_rates.rs_nrates;
        for (x = 0; x < ni->ni_rates.rs_nrates; x++) {
		sn->rates[x].rate = ni->ni_rates.rs_rates[x] & IEEE80211_RATE_VAL;
		sn->rates[x].rix = sc->sc_rixmap[sn->rates[x].rate];
		sn->rates[x].rateCode = rt->info[sn->rates[x].rix].rateCode;
		sn->rates[x].shortPreambleRateCode = 
			rt->info[sn->rates[x].rix].rateCode | 
			rt->info[sn->rates[x].rix].shortPreamble;

		DPRINTF(sc, " %d/%d", sn->rates[x].rate,
			calc_usecs_unicast_packet(sc, 1600, sn->rates[x].rix, 
						  0,0));

	}
	DPRINTF(sc, "%s\n", "");
	
	/* set the visible bit-rate to the lowest one available */
	ni->ni_txrate = 0;
	an->an_tx_mgtrate = rt->info[0].rateCode;
	an->an_tx_mgtratesp = an->an_tx_mgtrate | rt->info[0].shortPreamble;
	sn->num_rates = ni->ni_rates.rs_nrates;

	if (sn->num_rates <= 0) {
		/* there are no rates yet we're done */
		return;
	}

	if (ic->ic_fixed_rate != -1) {
		srate = sn->num_rates - 1;

		/*
		 * A fixed rate is to be used; ic_fixed_rate is an
		 * index into the supported rate set.  Convert this
		 * to the index into the negotiated rate set for
		 * the node.  We know the rate is there because the
		 * rate set is checked when the station associates.
		 */
		rs = &ic->ic_sup_rates[ic->ic_curmode];
		r = rs->rs_rates[ic->ic_fixed_rate] & IEEE80211_RATE_VAL;
		/* NB: the rate set is assumed sorted */
		for (; srate >= 0 && sn->rates[srate].rate != r; srate--)
			;

		KASSERT(srate >= 0,
			("fixed rate %d not in rate set", ic->ic_fixed_rate));

		sn->static_rate_ndx = srate;
	}

	for (y = 0; y < NUM_PACKET_SIZE_BINS; y++) {
		int size = bin_to_size(y);
		sn->packets_sent[y] = 0;
		sn->current_sample_ndx[y] = -1;
		sn->last_sample_ndx[y] = 0;
		
		for (x = 0; x < ni->ni_rates.rs_nrates; x++) {
			sn->stats[y][x].successive_failures = 0;
			sn->stats[y][x].tries = 0;
			sn->stats[y][x].total_packets = 0;
			sn->stats[y][x].packets_acked = 0;
			sn->stats[y][x].last_tx = 0;
			
			sn->stats[y][x].perfect_tx_time = 
				calc_usecs_unicast_packet(sc, size, 
							  sn->rates[x].rix,
							  0, 0);
			sn->stats[y][x].average_tx_time = sn->stats[y][x].perfect_tx_time;
		}
	}
}

static void
ath_rate_cb(void *arg, struct ieee80211_node *ni)
{
        ath_rate_ctl_reset((struct ath_softc *) arg, ni);
}

/*
 * Reset the rate control state for each 802.11 state transition.
 */
void
ath_rate_newstate(struct ath_softc *sc, enum ieee80211_state newstate)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	if (ic->ic_opmode == IEEE80211_M_STA) {
		/*
		 * Reset local xmit state; this is really only
		 * meaningful when operating in station mode.
		 */
		ni = ic->ic_bss;
		ath_rate_ctl_reset(sc, ni);
        } else {
		/*
		 * When operating as a station the node table holds
		 * the AP's that were discovered during scanning.
		 * For any other operating mode we want to reset the
		 * tx rate state of each node.
		 */
                ieee80211_iterate_nodes(&ic->ic_sta, ath_rate_cb, sc);
		ath_rate_ctl_reset(sc, ic->ic_bss);
	}
}
EXPORT_SYMBOL(ath_rate_newstate);

struct ath_ratectrl *
ath_rate_attach(struct ath_softc *sc)
{
	struct sample_softc *osc;
	DPRINTF(sc, "%s: %s\n", dev_info, __func__);
	
	osc = kmalloc(sizeof(struct sample_softc), GFP_ATOMIC);
	if (osc == NULL)
		return NULL;
	osc->arc.arc_space = sizeof(struct sample_node);

	osc->ath_smoothing_rate = ath_smoothing_rate;
	osc->ath_sample_rate = ath_sample_rate;

	return &osc->arc;
}
EXPORT_SYMBOL(ath_rate_attach);

void
ath_rate_detach(struct ath_ratectrl *arc)
{
	struct sample_softc *osc = (struct sample_softc *) arc;
	kfree(osc);
}
EXPORT_SYMBOL(ath_rate_detach);

static int min_smoothing_rate = 0;		
static int max_smoothing_rate = 99;		

static int min_sample_rate = 2;
static int max_sample_rate = 100;

#ifdef CONFIG_SYSCTL
void
ath_rate_dynamic_sysctl_register(struct ath_softc *sc)
{
}
EXPORT_SYMBOL(ath_rate_dynamic_sysctl_register);
#endif /* CONFIG_SYSCTL */

#define	CTL_AUTO	-2	/* cannot be CTL_ANY or CTL_NONE */

/*
 * Static (i.e. global) sysctls.
 */
enum {
	DEV_ATH		= 9			/* XXX known by many */
};


static ctl_table ath_rate_static_sysctls[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "smoothing_rate",
	  .mode		= 0644,
	  .data		= &ath_smoothing_rate,
	  .maxlen	= sizeof(ath_smoothing_rate),
	  .extra1	= &min_smoothing_rate,
	  .extra2	= &max_smoothing_rate,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "sample_rate",
	  .mode		= 0644,
	  .data		= &ath_sample_rate,
	  .maxlen	= sizeof(ath_sample_rate),
	  .extra1	= &min_sample_rate,
	  .extra2	= &max_sample_rate,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ 0 }
};
static ctl_table ath_rate_table[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "rate",
	  .mode		= 0555,
	  .child	= ath_rate_static_sysctls
	}, { 0 }
};
static ctl_table ath_ath_table[] = {
	{ .ctl_name	= DEV_ATH,
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_rate_table
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{ .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_sysctl_header;

MODULE_AUTHOR("John Bicket");
MODULE_DESCRIPTION("SampleRate bit-rate selection algorithm for Atheros devices");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

static int __init
init_ath_rate_sample(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);

#ifdef CONFIG_SYSCTL
	ath_sysctl_header = register_sysctl_table(ath_root_table, 1);
#endif
	return (0);
}
module_init(init_ath_rate_sample);

static void __exit
exit_ath_rate_sample(void)
{
#ifdef CONFIG_SYSCTL
	if (ath_sysctl_header != NULL)
		unregister_sysctl_table(ath_sysctl_header);
#endif

	printk(KERN_INFO "%s: unloaded\n", dev_info);
}
module_exit(exit_ath_rate_sample);
