#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>


/*
 Upon "in-service" radar detection, this code handles the frequency hopping
 by finding a new channel where we haven't detected any radar or have not
 tried to use for 30 minutes
*/

/* 30 minutes */
#define RADAR_CHANNEL_REUSE_LIMIT (HZ * 60 * 30) 

static u_int32_t channelList[IEEE80211_CHAN_MAX];

void
radar_init(void)
{
    memset (channelList, 0, sizeof (channelList));
}

struct ieee80211_channel *
radar_handle_interference(struct ieee80211com *ic)
{
    struct ieee80211_channel *c;

    int index = 0;
    int chan;

    /* Mark current channel as having radar on it */

    chan = ic->ic_ibss_chan - ic->ic_channels;

    printk ("%s: Marking channel %d as radar disturbed.\n", __func__, chan);

    if (ic->ic_ibss_chan != NULL)
	channelList[chan] = jiffies;

    /* Find next appropriate channel */ 

    while (index < IEEE80211_CHAN_MAX) {
	if (!isclr(ic->ic_chan_active, index) && (channelList[index] == 0 || (jiffies - channelList[index]) > RADAR_CHANNEL_REUSE_LIMIT)) {
	    printk ("%s: Hoping to channel %d (%u).\n", __func__, index, channelList[index]);
	    break;
	}

	index ++;
    }

    if (index == IEEE80211_CHAN_MAX) {
	/* No channels are availiable and we are in deep shit */
	return NULL;
    }

    c = &ic->ic_channels[index];

    return c;
}
