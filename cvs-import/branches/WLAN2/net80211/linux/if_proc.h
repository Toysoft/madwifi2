#ifndef IF_PROC_H
#define IF_PROC_H

struct ieee80211com;

void ieee80211_proc_init(struct ieee80211com *ic, char const *devname);

void ieee80211_proc_remove(struct ieee80211com *ic);

#endif /* IF_PROC_H */
