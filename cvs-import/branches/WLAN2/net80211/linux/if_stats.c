

#include "ath-linux.h"
#include "ieee80211_var.h"
#include "if_stats.h"
#include "if_proc.h"
#include "if_wireless.h"

/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ieee80211_getstats(struct net_device *dev)
{
	struct ieee80211com *ic = (struct ieee80211com *) dev;
	return &ic->ic_stats.ic_stats;
}


void ieee80211_stats_attach (struct ieee80211com *com)
{
	struct net_device *dev = (struct net_device *)com;

	/*
	 * Register the netdevice early so the device
	 * name is filled in for any msgs.
	 *
	 * NB: The driver is expected to fill in anything
	 *     it cares about; we fillin only default handlers.
	 */
	if (dev->get_stats == NULL)
		dev->get_stats = ieee80211_getstats;
#ifdef CONFIG_NET_WIRELESS
	if (dev->get_wireless_stats == NULL)
		dev->get_wireless_stats = ieee80211_iw_getstats;
	if (dev->wireless_handlers == NULL)
		dev->wireless_handlers = (struct iw_handler_def *) &ieee80211_iw_handler_def;
#endif /* CONFIG_NET_WIRELESS */
}

void ieee80211_stats_detach (struct ieee80211com *com)
{
}
