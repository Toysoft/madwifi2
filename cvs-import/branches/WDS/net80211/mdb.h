#ifndef __MDB_H__
#define __MDB_H__

#define MDB_TRUE 1 
#define MDB_FALSE 0

extern int mdb_init (struct ieee80211com *ic);
extern void mdb_local_add (struct ieee80211com *ic, u_int8_t *macaddr);
extern int mdb_local_get (struct ieee80211com *ic, u_int8_t *macaddr);
extern void mdb_remote_add (struct ieee80211com *ic, u_int8_t *macaddr, u_int8_t *nimacaddr);
extern struct ieee80211_node *mdb_remote_get (struct ieee80211com *ic, u_int8_t *macaddr);

#endif
