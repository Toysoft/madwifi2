#ifndef IF_PROC_H
#define IF_PROC_H

struct ieee80211com;

void ieee80211_proc_init(struct ieee80211_stats *stats, char const *devname);

void ieee80211_proc_remove(struct ieee80211_stats *stats);

#endif /* IF_PROC_H */
