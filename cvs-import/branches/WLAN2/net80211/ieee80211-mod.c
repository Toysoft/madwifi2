#include <net80211/ieee80211_var.h>

#include "linux/if_proc.h"

#include <linux/module.h>

struct sk_buff;
extern struct sk_buff *
ieee80211_encap(struct ieee80211com *ic, struct sk_buff *skb, struct ieee80211_node **pni);

const char*
ether_sprintf(const u_int8_t *mac)
{
        static char etherbuf[18];
        snprintf(etherbuf, sizeof(etherbuf), "%02x:%02x:%02x:%02x:%02x:%02x",
		 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return etherbuf;
}
EXPORT_SYMBOL(ether_sprintf);		/* XXX */

/*
 * Module glue.
 */
#include "release.h"
#include "version.h"
static	char *version = WLAN_VERSION " " RELEASE_TYPE " (Sam Leffler <sam@errno.com>)";
static	char *dev_info = "wlan";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("802.11 wireless LAN protocol support");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

static int __init
init_wlan(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);
	return 0;
}
module_init(init_wlan);

static void __exit
exit_wlan(void)
{
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_wlan);

#ifndef __MOD_INC_USE_COUNT
#define __MOD_INC_USE_COUNT(_m)                     \
    if (!try_module_get(_m)) {                  \
        printk(KERN_WARNING "%s: try_module_get failed\n",  \
            ieee80211_get_device_name (ic));               \
        return (ENODEV);                    \
    }
#define __MOD_DEC_USE_COUNT(_m)     module_put(_m)
#endif

int ieee80211_linuxmoduleref (struct ieee80211com *ic)
{
	__MOD_INC_USE_COUNT(THIS_MODULE);
	return 0;
}

void ieee80211_linuxmoduleunref (struct ieee80211com *ic)
{
	__MOD_DEC_USE_COUNT(THIS_MODULE);
}


EXPORT_SYMBOL(ieee80211_add_rates);
EXPORT_SYMBOL(ieee80211_add_xrates);
EXPORT_SYMBOL(ieee80211_chan2ieee);
EXPORT_SYMBOL(ieee80211_chan2mode);
/*EXPORT_SYMBOL(ieee80211_decap);*/
EXPORT_SYMBOL(ieee80211_dump_pkt);
EXPORT_SYMBOL(ieee80211_encap);
EXPORT_SYMBOL(ieee80211_end_scan);
EXPORT_SYMBOL(ieee80211_find_node);
EXPORT_SYMBOL(ieee80211_ieee2mhz);
EXPORT_SYMBOL(ieee80211_ifattach);
EXPORT_SYMBOL(ieee80211_ifdetach);
EXPORT_SYMBOL(ieee80211_input);
EXPORT_SYMBOL(ieee80211_iterate_nodes);
EXPORT_SYMBOL(ieee80211_media_init);
EXPORT_SYMBOL(ieee80211_media_change);
EXPORT_SYMBOL(ieee80211_media_status);
EXPORT_SYMBOL(ieee80211_media2rate);
EXPORT_SYMBOL(ieee80211_mhz2ieee);
/*EXPORT_SYMBOL(ieee80211_new_state);*/
EXPORT_SYMBOL(ieee80211_next_scan);
EXPORT_SYMBOL(ieee80211_rate2media);
EXPORT_SYMBOL(ieee80211_setmode);
EXPORT_SYMBOL(ieee80211_stats_attach);
EXPORT_SYMBOL(ieee80211_stats_detach);
EXPORT_SYMBOL(ieee80211_proc_init);
EXPORT_SYMBOL(ieee80211_proc_remove);
EXPORT_SYMBOL(ieee80211_free_node);
EXPORT_SYMBOL(ieee80211_watchdog);
EXPORT_SYMBOL(ieee80211_linuxmoduleref);
EXPORT_SYMBOL(ieee80211_linuxmoduleunref);


#ifdef IEEE80211_DEBUG
MODULE_PARM(ieee80211_debug, "i");
MODULE_PARM_DESC(ieee80211_debug, "ORed debugging flags:\n - 0x1: DPRINTF debugging\n - 0x2: DPRINTF2 debugging\n - 0x4: DPRINTF3 debugging\n - 0x8: DPRINTF_MSG debugging\n");
#endif
