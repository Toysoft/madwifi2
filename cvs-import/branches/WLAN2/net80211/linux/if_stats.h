#ifndef IF_STATS_H
#define IF_STATS_H

#include <linux/config.h>

#ifdef CONFIG_NET_WIRELESS
#include <linux/wireless.h>
#endif


struct ieee80211_stats {
	struct net_device_stats ic_stats;
#ifdef CONFIG_PROC_FS
	char			ic_procname[12];/* e.g. wlan%d */
	struct proc_dir_entry  *ic_proc;	/* /proc/net/wlan%d */
#endif
#ifdef CONFIG_NET_WIRELESS
	struct iw_statistics	ic_iwstats;	/* wireless statistics block */
	u_int8_t ic_nickname[IEEE80211_NWID_LEN];
	int ic_nicknamelen;
#endif
};

struct ieee80211com;

/* API to be called by the Linux driver. */
void ieee80211_stats_attach (struct ieee80211com *com);

void ieee80211_stats_detach (struct ieee80211com *com);


/* API to be called by the 802.11 code */
static __inline void
ieee80211_stats_report_rx_error (struct ieee80211_stats *stats)
{
	stats->ic_stats.rx_errors++;
}


static __inline void
ieee80211_stats_report_tx_error (struct ieee80211_stats *stats)
{
	stats->ic_stats.tx_errors++;
}

static __inline void
ieee80211_stats_report_tx (struct ieee80211_stats *stats)
{
	stats->ic_stats.tx_packets++;
}

static __inline void
ieee80211_stats_report_rx (struct ieee80211_stats *stats)
{
	stats->ic_stats.rx_packets++;
}

static __inline void
ieee80211_stats_report_rx_bytes (struct ieee80211_stats *stats, unsigned long bytes)
{
	stats->ic_stats.rx_bytes += bytes;
}

static __inline void
ieee80211_stats_report_tx_bytes (struct ieee80211_stats *stats, unsigned long bytes)
{
	stats->ic_stats.tx_bytes += bytes;
}

static __inline void
ieee80211_stats_report_multicast (struct ieee80211_stats *stats)
{
	stats->ic_stats.multicast ++;
}

#endif /* IF_STATS_H */
