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
#define	DPRINTF(sc, _fmt, ...) do {		\
	if (sc->sc_debug & 0x10)		\
		printk(_fmt, __VA_ARGS__);	\
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

static	int ath_smoothing_rate = 90;	/* ewma percentage (out of 100) */
static	int ath_sample_rate = 10;	/* send a different bit-rate 1/X packets */

static void	ath_rate_ctl_start(struct ath_softc *, struct ieee80211_node *);

void
ath_rate_node_init(struct ath_softc *sc, struct ath_node *an){
	DPRINTF(sc, "%s:\n", __func__);
	/* NB: assumed to be zero'd by caller */
}
EXPORT_SYMBOL(ath_rate_node_init);

void
ath_rate_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{
	DPRINTF(sc, "%s:\n", __func__);
}
EXPORT_SYMBOL(ath_rate_node_cleanup);

void
ath_rate_node_copy(struct ath_softc *sc,
		   struct ath_node *dst, const struct ath_node *src)
{
	DPRINTF(sc, "%s:\n", __func__);
	struct sample_node *odst = ATH_NODE_SAMPLE(dst);
	const struct sample_node *osrc = (const struct sample_node *)&src[1];

	memcpy(odst, osrc, sizeof(struct sample_node));
}
EXPORT_SYMBOL(ath_rate_node_copy);


/*
 * returns the ndx with the lowest average_tx_time,
 * or -1 if all the average_tx_times are 0.
 */
inline int best_rate_ndx(struct sample_node *sn)
{
	int x = 0;
        int best_rate_ndx = 0;
        int best_rate_tt = 0;
        for (x = 0; x < sn->num_rates; x++) {
		int tt = sn->rates[x].average_tx_time;
		if (tt > 0) {
			if (!best_rate_tt || best_rate_tt > tt) {
				best_rate_tt = tt;
				best_rate_ndx = x;
			}
		}
        }

        return (best_rate_tt) ? best_rate_ndx : -1;
}

/*
 * pick a ndx s.t. the perfect_tx_time
 * is less than the best bit-rate's average_tx_time
 * and the ndx has not had four successive failures.
 */
inline int pick_sample_ndx(struct sample_node *sn) 
{
	int x = 0;
	int best_ndx = best_rate_ndx(sn);
	int best_tt = 0;
	int num_eligible = 0;
	
	if (best_ndx < 0) {
		/* no successes yet, send at the lowest bit-rate */
		return 0;
	}
	
	best_tt = sn->rates[best_ndx].average_tx_time;
	sn->sample_num++;
	
	/*
	 * first, find the number of bit-rates we could potentially
	 * sample. we assume this list doesn't change a lot, so
	 * we will just cycle through them.
	 */
	for (x = 0; x < sn->num_rates; x++) {
		if (x != best_ndx && 
		    sn->rates[x].perfect_tx_time < best_tt &&
		    sn->rates[x].successive_failures < 4) {
			num_eligible++;
		}
	}
	
	if (num_eligible > 0) {
		int pick = sn->sample_num % num_eligible;
		for (x = 0; x < sn->num_rates; x++) {
			if (x != best_ndx && 
			    sn->rates[x].perfect_tx_time < best_tt &&
			    sn->rates[x].successive_failures < 4) {
				if (pick == 0) {
					return x;
				}
				pick--;
			}
		}
	}
	return best_ndx;
}


void
ath_rate_findrate(struct ath_softc *sc, struct ath_node *an,
		  HAL_BOOL shortPreamble, size_t frameLen,
		  u_int8_t *rix, int *try0, u_int8_t *txrate)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	
	if (sn->static_rate_ndx != -1) {
		*try0 = 4;
		*rix = sn->rates[sn->static_rate_ndx].rix;
		*txrate = sn->rates[sn->static_rate_ndx].rateCode;
		return;
	}
	int ndx = 0;
	*try0 = 4;
	int best_ndx = best_rate_ndx(sn);
	if (!sn->packets_sent || sn->packets_sent % ath_sample_rate > 0) {
		/*
		 * for most packets, send the packet at the bit-rate with 
		 * the lowest estimated transmission time.
		 */
		if (best_ndx != -1) {
			ndx = best_ndx;
		} else {
			/* 
			 * no packet has succeeded, try the highest bitrate
			 * that hasn't failed.
			 */  
			*try0 = 2;
			for (ndx = sn->num_rates-1; ndx >= 0; ndx--) {
				if (sn->rates[ndx].successive_failures == 0) {
					break;
				}
			}
		}
	} else {
		/* send the packet at a different bit-rate */
		ndx = pick_sample_ndx(sn);
		if (best_ndx != ndx) {
			*try0 = 2;
		}
	}
	
	
	*rix = sn->rates[ndx].rix;
	if (shortPreamble) {
		*txrate = sn->rates[ndx].shortPreambleRateCode;
	} else {
		
		*txrate = sn->rates[ndx].rateCode;
	}
	
	
	sn->packets_sent++;
	DPRINTF(sc, "packets %d rate %d ndx %d rateCode %d try0 %d average %d\n",
		sn->packets_sent, 
		sn->rates[ndx].rate, ndx, sn->rates[ndx].rateCode, 
		*try0, sn->rates[*rix].average_tx_time);
	
}
EXPORT_SYMBOL(ath_rate_findrate);

void
ath_rate_setupxtxdesc(struct ath_softc *sc, struct ath_node *an,
		      struct ath_desc *ds, HAL_BOOL shortPreamble, u_int8_t rix)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	int best_ndx = best_rate_ndx(sn);
	int rateCode = 0;
	if (best_ndx == -1) {
		/* 
		 * no packet has succeeded, so also try twice at the lowest bitate.
		 */
		rateCode = sn->rates[0].shortPreambleRateCode;
	} else if (sn->rates[best_ndx].rix != rix) {
		/*
		 * we're trying a different bit-rate, and it could be lossy, 
		 * so if it fails try at the best bit-rate.
		 */
		rateCode = sn->rates[best_ndx].shortPreambleRateCode;
	}
	ath_hal_setupxtxdesc(sc->sc_ah, ds
			     , rateCode, 2	/* series 1 */
			     , 0, 0	        /* series 2 */
			     , 0, 0	        /* series 3 */
			     );
	
}
EXPORT_SYMBOL(ath_rate_setupxtxdesc);

void
ath_rate_tx_complete(struct ath_softc *sc,
		     struct ath_node *an, const struct ath_desc *ds)
{
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	int rate = sc->sc_hwmap[ds->ds_txstat.ts_rate & IEEE80211_RATE_VAL].ieeerate;
	int used_alt_rate = ds->ds_txstat.ts_rate & HAL_TXSTAT_ALTRATE;
	int retries = ds->ds_txstat.ts_longretry;
	int tt = calc_usecs_wifi_packet(1500, rate, MIN(retries, 4));
	int rix = -1;
	int x = 0;
	
	if (!sn->num_rates) {
		DPRINTF(sc, "%s: no rates yet\n", __func__);
		return;
	}
	for (x = 0; x < sn->num_rates; x++) {
		if (sn->rates[x].rate == rate) {
			rix = x;
			break;
		}      
	}
	
	if (rix < 0 || rix > sn->num_rates) {
		/* maybe a management packet */
		return;
	}
	
	DPRINTF(sc, "%s: rate %d rix %d retries %d tt %d success %d ts_rate %d\n", 
		__func__, rate, rix, retries, tt, used_alt_rate, ds->ds_txstat.ts_rate);
	
	if (!sn->rates[rix].average_tx_time) {
		sn->rates[rix].average_tx_time = tt;
	} else {
		sn->rates[rix].average_tx_time = 
			((sn->rates[rix].average_tx_time * ath_smoothing_rate) + 
			 (tt * (100 - ath_smoothing_rate))) / 100;
	}
	
	if (!used_alt_rate) {
		sn->rates[rix].packets_acked++;
		sn->rates[rix].successive_failures = 0;
	} else {
		sn->rates[rix].successive_failures++;
	}
	sn->rates[rix].tries += (1+retries);
	
}
EXPORT_SYMBOL(ath_rate_tx_complete);

void
ath_rate_newassoc(struct ath_softc *sc, struct ath_node *an, int isnew)
{
	DPRINTF(sc, "%s:\n", __func__);
	if (isnew)
		ath_rate_ctl_start(sc, &an->an_node);
}
EXPORT_SYMBOL(ath_rate_newassoc);



/*
 * Initialize the tables for a node.
 */
static void
ath_rate_ctl_start(struct ath_softc *sc, struct ieee80211_node *ni)
{
#define	RATE(_ix)	(ni->ni_rates.rs_rates[(_ix)] & IEEE80211_RATE_VAL)
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_node *an = ATH_NODE(ni);
	struct sample_node *sn = ATH_NODE_SAMPLE(an);
	const HAL_RATE_TABLE *rt = sc->sc_currates;
	int srate;
        int x = 0;
	
        DPRINTF(sc, "%s:\n", __func__);
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));
	KASSERT(ni->ni_rates.rs_nrates > 0, ("no rates"));
        sn->static_rate_ndx = -1;
	if (ic->ic_fixed_rate != -1) {
		/*
		 * A fixed rate is to be used; ic_fixed_rate is an
		 * index into the supported rate set.  Convert this
		 * to the index into the negotiated rate set for
		 * the node.  We know the rate is there because the
		 * rate set is checked when the station associates.
		 */
		const struct ieee80211_rateset *rs =
			&ic->ic_sup_rates[ic->ic_curmode];
		int r = rs->rs_rates[ic->ic_fixed_rate] & IEEE80211_RATE_VAL;
		/* NB: the rate set is assumed sorted */
		srate = ni->ni_rates.rs_nrates - 1;
		for (; srate >= 0 && RATE(srate) != r; srate--)
			;
		KASSERT(srate >= 0,
			("fixed rate %d not in rate set", ic->ic_fixed_rate));
                sn->static_rate_ndx = srate;
                        
	}
	sn->num_rates = ni->ni_rates.rs_nrates;

        for (x = 0; x < ni->ni_rates.rs_nrates; x++) {
          sn->rates[x].rate = ni->ni_rates.rs_rates[x] & IEEE80211_RATE_VAL;
          sn->rates[x].rix = sc->sc_rixmap[sn->rates[x].rate];
          sn->rates[x].rateCode = rt->info[sn->rates[x].rix].rateCode;
          sn->rates[x].shortPreambleRateCode = 
		  rt->info[sn->rates[x].rix].rateCode | 
		  rt->info[sn->rates[x].rix].shortPreamble;
          sn->rates[x].perfect_tx_time = calc_usecs_wifi_packet(1500, 
								sn->rates[x].rate, 
								0);

          DPRINTF(sc, "%s: %d rate %d rix %d rateCode %d perfect_tx_time %d \n", __func__, 
                  x, sn->rates[x].rate, 
                  sn->rates[x].rix, sn->rates[x].rateCode,
                  sn->rates[x].perfect_tx_time);

        }
#undef RATE

	/* XXX management/control frames always go at the lowest speed */
	an->an_tx_mgtrate = rt->info[0].rateCode;
	an->an_tx_mgtratesp = an->an_tx_mgtrate | rt->info[0].shortPreamble;
}
/*
 * Reset the rate control state for each 802.11 state transition.
 */
void
ath_rate_newstate(struct ath_softc *sc, enum ieee80211_state state)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	if (ic->ic_opmode == IEEE80211_M_STA) {
		/*
		 * Reset local xmit state; this is really only
		 * meaningful when operating in station mode.
		 */
		ni = ic->ic_bss;
		if (state == IEEE80211_S_RUN) {
			ath_rate_ctl_start(sc, ni);
                }
        }
}
EXPORT_SYMBOL(ath_rate_newstate);

struct ath_ratectrl *
ath_rate_attach(struct ath_softc *sc)
{
	DPRINTF(sc, "%s:\n", __func__);
	struct sample_softc *osc;
	
	osc = kmalloc(sizeof(struct sample_softc), GFP_ATOMIC);
	if (osc == NULL)
		return NULL;
	osc->arc.arc_space = sizeof(struct sample_node);
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

static	int min_smoothing_rate = 0;		
static	int max_smoothing_rate = 100;		

static int min_sample_rate = 2;
static int max_sample_rate = 100;

#define	CTL_AUTO	-2	/* cannot be CTL_ANY or CTL_NONE */

/*
 * Static (i.e. global) sysctls.
 */
enum {
	DEV_ATH		= 9,			/* XXX known by many */
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

static char *version = ".9";
static char *dev_info = "ath_rate_sample";

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
