/*
  MAC Database for wireless bridging
  Contributed by Jonas Tärnström, Repeatit AB 2004 (jonas.tarnstrom@repeatit.se)

  TODO:
  [ ] The only "major" flaw is that the ieee80211_node:s are refered to by a weak reference
       and this will result in two hash look-ups, maybe not that importanta after all

  [X] Expire hash entries over time, see MDB_MAX_AGE value (jiffies)

  [ ] Make MDB compile as module

  [ ] Hash operations must be protected by lock

  2004-09-23: Initial release
*/

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>
#include "mdb.h"

#include <../ath/if_athvar.h>

#define MDB_MAX_AGE (HZ * 3600)

//#define __MDB_DEBUG__


#define _LOCALHASH ic->mdbLocalHash
#define _REMOTEHASH ic->mdbRemoteHash


static unsigned int 
_hash_key(u_int8_t *macaddr)
{
    unsigned int retVal;
    memcpy((void *) &retVal, macaddr + 2, 4);
    return retVal;
}

/*
  Adds an item to a hash, if nimacaddr is NULL, it's ignored
  NOTE: Does not check if item exists before adding
*/
static void
_hash_add(MDBMACITEM **hash, u_int8_t *macaddr, u_int8_t *nimacaddr)
{
    MDBMACITEM *pitem;
    unsigned int hashvalue;
    unsigned int hashindex;

    /* Do some hashing */
    hashvalue = _hash_key(macaddr);
    hashindex = hashvalue % MDB_HASH_SIZE;

    pitem = hash[hashindex];

    hash[hashindex] = (MDBMACITEM *) kmalloc(sizeof(MDBMACITEM), GFP_ATOMIC);
    hash[hashindex]->key = hashvalue;
    hash[hashindex]->time = jiffies;
    memcpy(hash[hashindex]->mac, macaddr, 6);
    if (nimacaddr)
	memcpy(hash[hashindex]->nimac, nimacaddr, 6);

    hash[hashindex]->pnext = pitem;
    return;
}

/*
  Finds an item in the hash and returns a pointer to it, NULL on failure
*/
static MDBMACITEM *
_hash_get (MDBMACITEM **hash, u_int8_t *macaddr)
{
    MDBMACITEM *pitem, *pprev;
    unsigned int hashvalue;
    unsigned int hashindex;

    /* Do some hashing */
    hashvalue = _hash_key(macaddr);
    hashindex = hashvalue % MDB_HASH_SIZE;

    pitem = hash[hashindex];

    while (pitem) {
	if (pitem->key == hashvalue && IEEE80211_ADDR_EQ(pitem->mac, macaddr))
	    break;

	pprev = pitem;
	pitem = (MDBMACITEM *) pitem->pnext;
    }

    return pitem;
}

/*
  Unlinks/Removes an item from the hash.
*/
static MDBMACITEM
*_hash_del(MDBMACITEM **hash, MDBMACITEM *pitem, MDBMACITEM *pprev, int hashindex)
{
    /* Unlink */
    if (pprev)
	pprev->pnext = pitem->pnext;

    if (pitem == hash[hashindex])
	hash[hashindex] = pitem->pnext;
	
    kfree(pitem);
    return NULL;
}

/*
  Unlinks/Removes expired items (use limit 0 to delete all)
*/

static void
_hash_expire (MDBMACITEM **hash, unsigned int agelimit)
{
    int index;
    MDBMACITEM *pprev = NULL;
    MDBMACITEM *pitem = NULL;
    MDBMACITEM *pnext = NULL;

    for(index = 0; index < MDB_HASH_SIZE; index ++) {
	pitem = (MDBMACITEM *) hash[index];

	while(pitem) {
	    pnext = (MDBMACITEM *) pitem->pnext;

	    if ((jiffies - pitem->time) >= agelimit) {
#ifdef __MDB_DEBUG__
		if (agelimit > 0)
		    printk("%s: Expiering MAC %s due to max age\n", __func__, ether_sprintf(pitem->mac));
		else
		    printk("%s: Deleting MAC %s\n", __func__, ether_sprintf(pitem->mac));
#endif
		_hash_del(hash, pitem, pprev, index);
		pprev = NULL;
	    } else {
		pprev = pitem;
    	    }

	    pitem = pnext;
	}
    }
}

/*
  Adds a MAC as being local (Being behind our side of the bridge)
  Updates if already found
*/
void
mdb_local_add(struct ieee80211com *ic, u_int8_t *macaddr)
{
    MDBMACITEM *pitem;

    if (IEEE80211_IS_MULTICAST(macaddr)) {
#ifdef __MDB_DEBUG__
	printk("%s: Rejecting multicast MAC\n", __func__);
#endif
	return;
    }

    /* Check if item exists */
    pitem = _hash_get(_LOCALHASH, macaddr);

    /* If it does, update expire time */
    if (pitem) {
#ifdef __MDB_DEBUG__
	printk("%s: MAC %s UPDATE\n", __func__, ether_sprintf(macaddr));
#endif
	pitem->time = jiffies;

	return;
    }

  /* Add new item */
#ifdef __MDB_DEBUG__
    printk("%s: MAC %s ADD\n", __func__, ether_sprintf(macaddr));
#endif

    _hash_add (_LOCALHASH, macaddr, NULL);
    return;
}

/*
  Returns MDB_TRUE (1) if item is found, MDB_FALSE (0) on failure
*/
int
mdb_local_get(struct ieee80211com *ic, u_int8_t *macaddr)
{
    MDBMACITEM *pitem;

    /* Check if item exists */
    pitem = _hash_get(_LOCALHASH, macaddr);

    if (pitem) {
	if ((jiffies - pitem->time) < MDB_MAX_AGE) {
	    return MDB_TRUE;
	} else {
#ifdef __MDB_DEBUG__
	    printk("%s: MAC %s HAS EXPIRED\n", __func__, ether_sprintf(macaddr));
#endif
	    return MDB_FALSE;
	}
    }

    return MDB_FALSE;
}

/*
  Adds a MAC as being remote (Being behind some other wireless bridge node)
  Updates if already found
*/

void
mdb_remote_add(struct ieee80211com *ic, u_int8_t *macaddr, u_int8_t *nimacaddr)
{
    MDBMACITEM *pitem;

    if (IEEE80211_IS_MULTICAST(macaddr)) {
#ifdef __MDB_DEBUG__
	printk("%s: Rejecting multicast MAC\n", __func__);
#endif
	return;
    }

    /* Check if item exists */
    pitem = _hash_get(_REMOTEHASH, macaddr);

    /* If it does, update expire time */
    if (pitem) {
#ifdef __MDB_DEBUG__
	printk("%s: MAC %s ->", __func__, ether_sprintf(macaddr));
	printk(" %s UPDATE\n", ether_sprintf(nimacaddr));
#endif

	pitem->time = jiffies;

	if (!IEEE80211_ADDR_EQ (pitem->nimac, nimacaddr)) {
	    /* This means a MAC behind a bridge has moved physically, let's update (RARE!) */
#ifdef __MDB_DEBUG__
	    printk("%s: MAC has changed origin node to %s, updating\n", __func__, ether_sprintf(nimacaddr));
#endif
	    memcpy(pitem->nimac, nimacaddr, 6);
	}

	return;
    }

#ifdef __MDB_DEBUG__
    printk("%s: MAC %s ->", __func__, ether_sprintf(macaddr));
    printk(" %s ADD\n", ether_sprintf(nimacaddr));
#endif

    /* Add new item */
    _hash_add(_REMOTEHASH, macaddr, nimacaddr);

    return;
}

/*
  Returns a pointer to a ieee80211_node structure of the provided MAC.
  Uses ieee80211_get_txnode as fallback and will finally return NULL if failure.
*/
struct ieee80211_node *
mdb_remote_get(struct ieee80211com *ic, u_int8_t *macaddr)
{
    MDBMACITEM *pitem;

    /* Check if item exists */
    pitem = _hash_get(_REMOTEHASH, macaddr);

    if (pitem) {
	if ((jiffies - pitem->time) < MDB_MAX_AGE) {
	    return ieee80211_find_txnode (ic, pitem->nimac);
	} else {
#ifdef __MDB_DEBUG__
	    printk("%s: MAC %s HAS EXPIRED\n", __func__, ether_sprintf(macaddr));
#endif
	    return ieee80211_find_txnode(ic, macaddr);
	}
    }

    return ieee80211_find_txnode(ic, macaddr);
}


/*
  Initializes data structures, should be performed once per device
*/
int
mdb_init(struct ieee80211com *ic)
{
    memset((void *) _REMOTEHASH, 0, sizeof(_REMOTEHASH));
    memset((void *) _LOCALHASH, 0, sizeof(_LOCALHASH));
    return 0;
}


/*
  Export this function
*/
int
mdb_exp_get_node_mac(struct net_device *dev, unsigned char *mac, unsigned char *nodemac)
{
    struct ath_softc *sc = dev->priv;
    struct ieee80211com *ic = &sc->sc_ic;
    struct ieee80211_node *pnode;

    /* Get pointer to node */
    pnode = mdb_remote_get(ic, mac);

    if (pnode) {
	/* Copy MAC */
	memcpy(nodemac, pnode->ni_macaddr, IEEE80211_ADDR_LEN);

	/* Release node */
	ieee80211_node_decref(pnode);
    }

    return 0;
}
EXPORT_SYMBOL(mdb_exp_get_node_mac);

/*
  Export this function
*/
int
mdb_exp_get_associd(struct net_device *dev, unsigned char *nodemac)
{
    struct ath_softc *sc = dev->priv;
    struct ieee80211com *ic = &sc->sc_ic;
    struct ieee80211_node *pnode;
    int retval;

    /* Get pointer to node */
    pnode = mdb_remote_get(ic, nodemac);

    if (pnode) {
	/* Copy MAC */
	retval = (int) pnode->ni_associd;

	/* Release node */
	ieee80211_node_decref(pnode);
	return retval;
    }

    return -1;
}
EXPORT_SYMBOL(mdb_exp_get_associd);
