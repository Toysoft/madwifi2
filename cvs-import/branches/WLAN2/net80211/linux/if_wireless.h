#ifndef IF_WIRELESS_H
#define IF_WIRELESS_H

struct iw_statistics * ieee80211_iw_getstats(struct net_device *dev);

extern const struct iw_handler_def ieee80211_iw_handler_def;

#endif /* IF_WIRELESS_H */
