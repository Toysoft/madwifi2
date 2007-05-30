/*
 * Wrapper macros/functions for the binary HAL to comply with local coding convention.
 * 
 * Provides function-style calling convention using either macros or wrapper functions for function pointers in the HAL.
 * 
 * The typical convention is foo(hal,p1,p2,p3,...) turns into hal->foo(p1,p2,p3,...) where foo 
 * is a function pointer and a member of the struct ath_hal.
 *
 * The macros are faster, but the functions have the advantage that they show up unobfuscated in backtraces thus giving you a clue as to where in 
 * your code you went wrong in your use of the HAL.
 *
 */
#include "opt_ah.h"

#ifdef AH_HALOPS_FUNC

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <asm/uaccess.h>

#include "ath_hal_api.h"

/*
 * HAL definitions to comply with local coding convention.
 */
HAL_BOOL ath_hal_reset(struct ath_hal* ah, HAL_OPMODE opmode, HAL_CHANNEL* chan,  HAL_BOOL outdoor, HAL_STATUS* pstatus) {
	return ((*ah->ah_reset)(ah, (opmode), (chan), (outdoor), (pstatus)));
}
const HAL_RATE_TABLE* ath_hal_getratetable(struct ath_hal* ah, u_int mode) {
	return ((*ah->ah_getRateTable)(ah, (mode)));
}
void ath_hal_getmac(struct ath_hal* ah, u_int8_t* mac) {
	(*ah->ah_getMacAddress)(ah, (mac));
}
HAL_BOOL ath_hal_setmac(struct ath_hal* ah, const u_int8_t* mac) {
	return ((*ah->ah_setMacAddress)(ah, (mac)));
}
void ath_hal_getbssidmask(struct ath_hal * ah, u_int8_t * mask) {
	(*ah->ah_getBssIdMask)(ah, (mask));
}
HAL_BOOL ath_hal_setbssidmask(struct ath_hal * ah, const u_int8_t* mask) {
	return ((*ah->ah_setBssIdMask)(ah, (mask)));
}
HAL_INT	ath_hal_intrset(struct ath_hal* ah, HAL_INT mask) {
	return ((*ah->ah_setInterrupts)(ah, (mask)));
}
HAL_INT	ath_hal_intrget(struct ath_hal* ah) {
	return ((*ah->ah_getInterrupts)(ah));
}
HAL_BOOL ath_hal_intrpend(struct ath_hal* ah) {
	return ((*ah->ah_isInterruptPending)(ah));
}
HAL_BOOL ath_hal_getisr(struct ath_hal* ah, HAL_INT* pmask) {
	return ((*ah->ah_getPendingInterrupts)(ah, (pmask)));
}
HAL_BOOL ath_hal_updatetxtriglevel(struct ath_hal* ah, HAL_BOOL inc) {
	return ((*ah->ah_updateTxTrigLevel)(ah, (inc)));
}
HAL_BOOL ath_hal_setpower(struct ath_hal* ah, HAL_POWER_MODE mode) {
	return ((*ah->ah_setPowerMode)(ah, (mode), AH_TRUE));
}
u_int32_t ath_hal_keycachesize(struct ath_hal* ah) {
	return ((*ah->ah_getKeyCacheSize)(ah));
}
HAL_BOOL ath_hal_keyreset(struct ath_hal* ah, u_int16_t ix) {
	return ((*ah->ah_resetKeyCacheEntry)(ah, (ix)));
}
HAL_BOOL ath_hal_keyset(struct ath_hal* ah, u_int16_t ix, const HAL_KEYVAL * pk, const u_int8_t * mac) {
	return ((*ah->ah_setKeyCacheEntry)(ah, (ix), (pk), (mac), AH_FALSE));
}
HAL_BOOL ath_hal_keyisvalid(struct ath_hal *ah, u_int16_t ix) {
	return (((*ah->ah_isKeyCacheEntryValid)(ah, (ix))));
}
HAL_BOOL ath_hal_keysetmac(struct ath_hal* ah, u_int16_t ix, const u_int8_t *mac) {
	return ((*ah->ah_setKeyCacheEntryMac)(ah, (ix), (mac)));
}
u_int32_t ath_hal_getrxfilter(struct ath_hal* ah) {
	return ((*ah->ah_getRxFilter)(ah));
}
void ath_hal_setrxfilter(struct ath_hal* ah, u_int32_t filter) {
	(*ah->ah_setRxFilter)(ah, (filter));
}
void ath_hal_setmcastfilter(struct ath_hal* ah, u_int32_t mfilt0, u_int32_t mfilt1) {
	((*ah->ah_setMulticastFilter)(ah, (mfilt0), (mfilt1)));
}
void ath_hal_putrxbuf(struct ath_hal* ah, u_int32_t bufaddr) {
	((*ah->ah_setRxDP)(ah, (bufaddr)));
}
u_int32_t ath_hal_gettsf32(struct ath_hal* ah) {
	return ((*ah->ah_getTsf32)(ah));
}
u_int64_t ath_hal_gettsf64(struct ath_hal* ah) {
	return ((*ah->ah_getTsf64)(ah));
}
void ath_hal_resettsf(struct ath_hal* ah) {
	((*ah->ah_resetTsf)(ah));
}
void ath_hal_rxena(struct ath_hal* ah) {
	((*ah->ah_enableReceive)(ah));
}

u_int32_t ath_hal_numtxpending(struct ath_hal *ah, u_int q) {
	return ((*ah->ah_numTxPending)(ah, (q)));
}

HAL_BOOL ath_hal_puttxbuf(struct ath_hal* ah, u_int q, u_int32_t bufaddr) {
	return ((*ah->ah_setTxDP)(ah, (q), (bufaddr)));
}

u_int32_t ath_hal_gettxbuf(struct ath_hal* ah, u_int q) {
	return ((*ah->ah_getTxDP)(ah, (q)));
}

u_int32_t ath_hal_getrxbuf(struct ath_hal* ah) {
	return ((*ah->ah_getRxDP)(ah));
}

HAL_BOOL ath_hal_txstart(struct ath_hal* ah, u_int q) {
	return ((*ah->ah_startTxDma)(ah, (q)));
}

HAL_BOOL ath_hal_calibrate(struct ath_hal* ah, HAL_CHANNEL * chan, HAL_BOOL * isIQdone) {
	return ((*ah->ah_perCalibration)(ah, (chan), (isIQdone)));
}

void ath_hal_setledstate(struct ath_hal* ah, HAL_LED_STATE state) {
	return ((*ah->ah_setLedState)(ah, (state)));
}
void ath_hal_beaconinit(struct ath_hal * ah, u_int32_t nextb, u_int32_t bperiod) {
	((*ah->ah_beaconInit)(ah, (nextb), (bperiod)));
}
void ath_hal_beaconreset(struct ath_hal* ah) {
	((*ah->ah_resetStationBeaconTimers)(ah));
}
void ath_hal_beacontimers(struct ath_hal* ah, const HAL_BEACON_STATE *bs) {
	((*ah->ah_setStationBeaconTimers)(ah, (bs)));
}

void ath_hal_setassocid(struct ath_hal* ah, const u_int8_t *bss, u_int16_t associd) {
	((*ah->ah_writeAssocid)(ah, (bss), (associd)));
}

HAL_BOOL ath_hal_phydisable(struct ath_hal * ah) {
	return ((*ah->ah_phyDisable)(ah));
}

void ath_hal_setopmode(struct ath_hal * ah) {
	((*ah->ah_setPCUConfig)(ah));
}

HAL_BOOL ath_hal_stoptxdma(struct ath_hal* ah, u_int q) {
	return ((*ah->ah_stopTxDma)(ah, q));
}

void ath_hal_stoppcurecv(struct ath_hal* ah) {
	((*ah->ah_stopPcuReceive)(ah));
}

void ath_hal_startpcurecv(struct ath_hal* ah) {
	((*ah->ah_startPcuReceive)(ah));
}

HAL_BOOL ath_hal_stopdmarecv(struct ath_hal* ah) {
	return ((*ah->ah_stopDmaReceive)(ah));
}

HAL_BOOL ath_hal_getdiagstate(struct ath_hal *ah, int id, const void *indata, u_int32_t insize, void** outdata, u_int32_t* outsize) {
	return ((*ah->ah_getDiagState)(ah, (id), (indata), (insize), (outdata), (outsize)));
}

HAL_BOOL ath_hal_gettxqueueprops(struct ath_hal * ah, int q, HAL_TXQ_INFO *qi) {
	return ((*ah->ah_getTxQueueProps)(ah, (q), (qi)));
}

HAL_BOOL ath_hal_settxqueueprops(struct ath_hal *ah, int q, const HAL_TXQ_INFO *qi) {
	return ((*ah->ah_setTxQueueProps)(ah, (q), (qi)));
}
int	ath_hal_setuptxqueue(struct ath_hal * ah, HAL_TX_QUEUE type, const HAL_TXQ_INFO * qi) {
	return (*ah->ah_setupTxQueue)(ah, type, qi);
}

HAL_BOOL ath_hal_resettxqueue(struct ath_hal *ah, u_int q) {
	return ((*ah->ah_resetTxQueue)(ah, (q)));
}
HAL_BOOL ath_hal_releasetxqueue(struct ath_hal *ah, u_int q) {
	return ((*ah->ah_releaseTxQueue)(ah, (q)));
}
HAL_RFGAIN ath_hal_getrfgain(struct ath_hal* ah) {
	return ((*ah->ah_getRfGain)(ah));
}
u_int ath_hal_getdefantenna(struct ath_hal* ah) {
	return ((*ah->ah_getDefAntenna)(ah));
}
void ath_hal_setdefantenna(struct ath_hal* ah, u_int ant) {
	(*ah->ah_setDefAntenna)(ah, (ant));
}

void ath_hal_rxmonitor(struct ath_hal* ah, const HAL_NODE_STATS *stats, HAL_CHANNEL *chan) {
	(*ah->ah_rxMonitor)(ah, stats, chan);
}
void ath_hal_mibevent(struct ath_hal * ah, const HAL_NODE_STATS *stats) {
	((*ah->ah_procMibEvent)(ah, (stats)));
}
HAL_BOOL ath_hal_setslottime(struct ath_hal* ah, u_int us) {
	return ((*ah->ah_setSlotTime)(ah, (us)));
}
u_int ath_hal_getslottime(struct ath_hal* ah) {
	return ((*ah->ah_getSlotTime)(ah));
}

HAL_BOOL ath_hal_setacktimeout(struct ath_hal* ah, u_int us) {
	return ((*ah->ah_setAckTimeout)(ah, (us)));
}

u_int ath_hal_getacktimeout(struct ath_hal* ah) {
	return ((*ah->ah_getAckTimeout)(ah));
}

HAL_BOOL ath_hal_setctstimeout(struct ath_hal* ah, u_int us) {
	return ((*ah->ah_setCTSTimeout)(ah, (us)));
}

u_int ath_hal_getctstimeout(struct ath_hal* ah) {
	return ((*ah->ah_getCTSTimeout)(ah));
}

HAL_BOOL ath_hal_setdecompmask(struct ath_hal* ah, u_int16_t keyid, int b) {
	return ((*ah->ah_setDecompMask)(ah, (keyid), (b)));
}

HAL_STATUS ath_hal_getcapability(struct ath_hal * ah, HAL_CAPABILITY_TYPE type, u_int32_t capability, u_int32_t *result) {
	return ((*ah->ah_getCapability)(ah, (type), (capability), (result)));
}

HAL_BOOL ath_hal_setcapability(struct ath_hal * ah, HAL_CAPABILITY_TYPE type, u_int32_t capability, u_int32_t setting, HAL_STATUS *status) {
	return ((*ah->ah_setCapability)(ah, (type), (capability), (setting), (status)));
}

HAL_BOOL ath_hal_ciphersupported(struct ath_hal * ah, u_int32_t cipher) {
	return (ath_hal_getcapability(ah, HAL_CAP_CIPHER, cipher, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_fastframesupported(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_burstsupported(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_BURST, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_xrsupported(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_XR, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_compressionsupported(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_turboagsupported(struct ath_hal * ah, int countrycode) {
	return (ath_hal_getwirelessmodes(ah, countrycode) & (HAL_MODE_108G|HAL_MODE_TURBO));
}

HAL_BOOL ath_hal_halfrate_chansupported(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_CHAN_HALFRATE, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_quarterrate_chansupported(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_CHAN_QUARTERRATE, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_getregdomain(struct ath_hal * ah, u_int32_t* destination) {
	return ath_hal_getcapability(ah, HAL_CAP_REG_DMN, 0, destination);
}

HAL_BOOL ath_hal_getcountrycode(struct ath_hal * ah, u_int32_t* destination) {
	(*(destination) = ah->ah_countryCode);
	return AH_TRUE;
}

HAL_BOOL ath_hal_hastkipsplit(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TKIP_SPLIT, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_gettkipsplit(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TKIP_SPLIT, 1, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_settkipsplit(struct ath_hal * ah, int v) {
	return ath_hal_setcapability(ah, HAL_CAP_TKIP_SPLIT, 1, v, NULL);
}

HAL_BOOL ath_hal_wmetkipmic(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_WME_TKIPMIC, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hwphycounters(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_PHYCOUNTERS, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hasdiversity(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_getdiversity(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 1, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_setdiversity(struct ath_hal * ah, int v) {
	return ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 1, v, NULL);
}

HAL_BOOL ath_hal_getnumtxqueues(struct ath_hal * ah, u_int32_t* destination) {
	return (ath_hal_getcapability(ah, HAL_CAP_NUM_TXQUEUES, 0, destination) == HAL_OK);
}

HAL_BOOL ath_hal_hasveol(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_VEOL, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hastxpowlimit(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_settxpowlimit(struct ath_hal * ah, u_int32_t power) {
	return ((*ah->ah_setTxPowerLimit)(ah, power));
}

HAL_BOOL ath_hal_gettxpowlimit(struct ath_hal * ah, u_int32_t* destination) {
	return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 1, destination) == HAL_OK);
}

HAL_BOOL ath_hal_getmaxtxpow(struct ath_hal * ah, u_int32_t* destination) {
	return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 2, destination) == HAL_OK);
}

HAL_BOOL ath_hal_gettpscale(struct ath_hal * ah, u_int32_t* destination) {
	return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 3, destination) == HAL_OK);
}

HAL_BOOL ath_hal_settpscale(struct ath_hal * ah, u_int32_t v) {
	return ath_hal_setcapability(ah, HAL_CAP_TXPOW, 3, v, NULL);
}

HAL_BOOL ath_hal_hastpc(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TPC, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_gettpc(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TPC, 1, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_settpc(struct ath_hal * ah, u_int32_t v) {
	return ath_hal_setcapability(ah, HAL_CAP_TPC, 1, v, NULL);
}

HAL_BOOL ath_hal_hasbursting(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_BURST, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hascompression(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hasfastframes(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hasbssidmask(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_BSSIDMASK, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hasmcastkeysearch(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_MCAST_KEYSRCH, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_getmcastkeysearch(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_MCAST_KEYSRCH, 1, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_hastkipmic(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TKIP_MIC, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_gettkipmic(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TKIP_MIC, 1, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_settkipmic(struct ath_hal * ah, u_int32_t v) {
	return ath_hal_setcapability(ah, HAL_CAP_TKIP_MIC, 1, v, NULL);
}

HAL_BOOL ath_hal_hastsfadjust(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TSF_ADJUST, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_gettsfadjust(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_TSF_ADJUST, 1, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_settsfadjust(struct ath_hal * ah, u_int32_t v) {
	return ath_hal_setcapability(ah, HAL_CAP_TSF_ADJUST, 1, v, NULL);
}

HAL_BOOL ath_hal_setrfsilent(struct ath_hal * ah, u_int32_t v) {
	    return ath_hal_setcapability(ah, HAL_CAP_RFSILENT, 1, v, NULL);
}

HAL_BOOL ath_hal_hasrfsilent(struct ath_hal * ah) {
	return (ath_hal_getcapability(ah, HAL_CAP_RFSILENT, 0, NULL) == HAL_OK);
}

HAL_BOOL ath_hal_setuprxdesc(struct ath_hal *ah, struct ath_desc *ds, u_int32_t size, u_int flags) {
	return ((*ah->ah_setupRxDesc)(ah, (ds), (size), flags));
}

HAL_BOOL ath_hal_rxprocdesc(struct ath_hal *ah, struct ath_desc *ds, u_int32_t dspa, struct ath_desc *dsnext, u_int64_t tsf, struct ath_rx_status *rs) {
	return ((*ah->ah_procRxDesc)(ah, (ds), (dspa), (dsnext), (tsf), (rs)));
}

HAL_BOOL ath_hal_setuptxdesc(struct ath_hal *ah, struct ath_desc *ds, u_int plen, u_int hlen, HAL_PKT_TYPE atype, u_int txpow, u_int txr0, u_int txtr0, u_int keyix, u_int ant, u_int flags, u_int rtsrate, u_int rtsdura, u_int compicvlen, u_int compivlen, u_int comp) {
	return ((*ah->ah_setupTxDesc)(ah, (ds), (plen), (hlen), (atype), (txpow), (txr0), (txtr0), (keyix), (ant), (flags), (rtsrate), (rtsdura), (compicvlen), (compivlen), (comp)));
}
HAL_BOOL ath_hal_setupxtxdesc(struct ath_hal * ah, struct ath_desc* ds, u_int txr1, u_int txtr1, u_int txr2, u_int txtr2, u_int txr3, u_int txtr3) {
	return ((*ah->ah_setupXTxDesc)(ah, (ds), (txr1), (txtr1), (txr2), (txtr2), (txr3), (txtr3)));
}
HAL_BOOL ath_hal_filltxdesc(struct ath_hal * ah, struct ath_desc * ds, u_int l, HAL_BOOL first, HAL_BOOL last, const struct ath_desc * ds0) {
	return ((*ah->ah_fillTxDesc)(ah, (ds), (l), (first), (last), (ds0)));
}
HAL_BOOL ath_hal_txprocdesc(struct ath_hal * ah, struct ath_desc *ds, struct ath_tx_status *ts) {
	return ((*ah->ah_procTxDesc)(ah, (ds), (ts)));
}
void ath_hal_gettxintrtxqs(struct ath_hal *ah, u_int32_t *txqs) {
	((*ah->ah_getTxIntrQueue)(ah, (txqs)));
}
void ath_hal_txreqintrdesc(struct ath_hal * ah, struct ath_desc *ds) {
	((*ah->ah_reqTxIntrDesc)(ah, (ds)));
}
HAL_BOOL ath_hal_gpioCfgOutput(struct ath_hal * ah, u_int32_t gpio) {
	return ((*ah->ah_gpioCfgOutput)(ah, (gpio)));
}
HAL_BOOL ath_hal_gpioset(struct ath_hal * ah, u_int32_t gpio, u_int32_t val) {
	return ((*ah->ah_gpioSet)(ah, (gpio), val));
}
void ath_hal_setcoverageclass(struct ath_hal * ah, u_int8_t coverageclass, int now) {
	((*ah->ah_setCoverageClass)(ah, (coverageclass), (now)));
}
HAL_BOOL ath_hal_radar_wait(struct ath_hal * ah, HAL_CHANNEL *chan) {
	return ((*ah->ah_radarWait)(ah, (chan)));
}
int16_t ath_hal_get_channel_noise(struct ath_hal * ah, HAL_CHANNEL *chan) {
	return ((*ah->ah_getChanNoise)(ah, (chan)));
}

HAL_BOOL ath_hal_set_regulatory_domain(struct ath_hal* ah, u_int16_t domain, HAL_STATUS* pstatus) {
	return ((*ah->ah_setRegulatoryDomain)(ah, domain, pstatus));

}

HAL_POWER_MODE ath_hal_getpower(struct ath_hal* ah) {
	return ((*ah->ah_getPowerMode)(ah));
}

HAL_BOOL ath_hal_set_multicast_filter_index(struct ath_hal* ah, u_int32_t index) {
	return ((*ah->ah_setMulticastFilterIndex)(ah,index));
}

HAL_BOOL ath_hal_clear_multicast_filter_index(struct ath_hal* ah, u_int32_t index) {
	return ((*ah->ah_clrMulticastFilterIndex)(ah,index));
}

HAL_BOOL ath_hal_gpioGet(struct ath_hal *ah, u_int32_t gpio) {
	return ((*ah->ah_gpioGet)(ah,gpio));
}
HAL_BOOL ath_hal_gpioCfgInput(struct ath_hal *ah, u_int32_t gpio) {
	return ((*ah->ah_gpioCfgInput)(ah,gpio));
}
void ath_hal_gpioSetIntr(struct ath_hal *ah, u_int p1, u_int32_t p2) {
	((*ah->ah_gpioSetIntr)(ah,p1,p2));
}

HAL_BOOL ath_hal_detect_card_present(struct ath_hal* ah) {
	return ((*ah->ah_detectCardPresent)(ah));
}

void ath_hal_update_mib_counters(struct ath_hal* ah, HAL_MIB_STATS* stats) {
	((*ah->ah_updateMibCounters)(ah, stats));
}
HAL_ANT_SETTING	ath_hal_get_antenna_switch(struct ath_hal* ah) {
	 return ((*ah->ah_getAntennaSwitch)(ah));
}

HAL_BOOL ath_hal_set_antenna_switch(struct ath_hal* ah, HAL_ANT_SETTING ant) {
	 return ((*ah->ah_setAntennaSwitch)(ah, ant));
}

void ath_hal_set_beacon_timers(struct ath_hal* ah, const HAL_BEACON_TIMERS *timers) {
	 ((*ah->ah_setBeaconTimers)(ah, timers));
}
EXPORT_SYMBOL(ath_hal_reset);
EXPORT_SYMBOL(ath_hal_getratetable);
EXPORT_SYMBOL(ath_hal_getmac);
EXPORT_SYMBOL(ath_hal_setmac);
EXPORT_SYMBOL(ath_hal_getbssidmask);
EXPORT_SYMBOL(ath_hal_setbssidmask);
EXPORT_SYMBOL(ath_hal_intrset);
EXPORT_SYMBOL(ath_hal_intrget);
EXPORT_SYMBOL(ath_hal_intrpend);
EXPORT_SYMBOL(ath_hal_getisr);
EXPORT_SYMBOL(ath_hal_updatetxtriglevel);
EXPORT_SYMBOL(ath_hal_setpower);
EXPORT_SYMBOL(ath_hal_keycachesize);
EXPORT_SYMBOL(ath_hal_keyreset);
EXPORT_SYMBOL(ath_hal_keyset);
EXPORT_SYMBOL(ath_hal_keyisvalid);
EXPORT_SYMBOL(ath_hal_keysetmac);
EXPORT_SYMBOL(ath_hal_getrxfilter);
EXPORT_SYMBOL(ath_hal_setrxfilter);
EXPORT_SYMBOL(ath_hal_setmcastfilter);
EXPORT_SYMBOL(ath_hal_putrxbuf);
EXPORT_SYMBOL(ath_hal_gettsf32);
EXPORT_SYMBOL(ath_hal_gettsf64);
EXPORT_SYMBOL(ath_hal_resettsf);
EXPORT_SYMBOL(ath_hal_rxena);
EXPORT_SYMBOL(ath_hal_numtxpending);
EXPORT_SYMBOL(ath_hal_puttxbuf);
EXPORT_SYMBOL(ath_hal_gettxbuf);
EXPORT_SYMBOL(ath_hal_getrxbuf);
EXPORT_SYMBOL(ath_hal_txstart);
EXPORT_SYMBOL(ath_hal_calibrate);
EXPORT_SYMBOL(ath_hal_setledstate);
EXPORT_SYMBOL(ath_hal_beaconinit);
EXPORT_SYMBOL(ath_hal_beaconreset);
EXPORT_SYMBOL(ath_hal_beacontimers);
EXPORT_SYMBOL(ath_hal_setassocid);
EXPORT_SYMBOL(ath_hal_phydisable);
EXPORT_SYMBOL(ath_hal_setopmode);
EXPORT_SYMBOL(ath_hal_stoptxdma);
EXPORT_SYMBOL(ath_hal_stoppcurecv);
EXPORT_SYMBOL(ath_hal_startpcurecv);
EXPORT_SYMBOL(ath_hal_stopdmarecv);
EXPORT_SYMBOL(ath_hal_getdiagstate);
EXPORT_SYMBOL(ath_hal_gettxqueueprops);
EXPORT_SYMBOL(ath_hal_settxqueueprops);
EXPORT_SYMBOL(ath_hal_setuptxqueue);
EXPORT_SYMBOL(ath_hal_resettxqueue);
EXPORT_SYMBOL(ath_hal_releasetxqueue);
EXPORT_SYMBOL(ath_hal_getrfgain);
EXPORT_SYMBOL(ath_hal_getdefantenna);
EXPORT_SYMBOL(ath_hal_setdefantenna);
EXPORT_SYMBOL(ath_hal_rxmonitor);
EXPORT_SYMBOL(ath_hal_mibevent);
EXPORT_SYMBOL(ath_hal_setslottime);
EXPORT_SYMBOL(ath_hal_getslottime);
EXPORT_SYMBOL(ath_hal_setacktimeout);
EXPORT_SYMBOL(ath_hal_getacktimeout);
EXPORT_SYMBOL(ath_hal_setctstimeout);
EXPORT_SYMBOL(ath_hal_getctstimeout);
EXPORT_SYMBOL(ath_hal_setdecompmask);
EXPORT_SYMBOL(ath_hal_getcapability);
EXPORT_SYMBOL(ath_hal_setcapability);
EXPORT_SYMBOL(ath_hal_ciphersupported);
EXPORT_SYMBOL(ath_hal_fastframesupported);
EXPORT_SYMBOL(ath_hal_burstsupported);
EXPORT_SYMBOL(ath_hal_xrsupported);
EXPORT_SYMBOL(ath_hal_compressionsupported);
EXPORT_SYMBOL(ath_hal_turboagsupported);
EXPORT_SYMBOL(ath_hal_halfrate_chansupported);
EXPORT_SYMBOL(ath_hal_quarterrate_chansupported);
EXPORT_SYMBOL(ath_hal_getregdomain);
EXPORT_SYMBOL(ath_hal_getcountrycode);
EXPORT_SYMBOL(ath_hal_hastkipsplit);
EXPORT_SYMBOL(ath_hal_gettkipsplit);
EXPORT_SYMBOL(ath_hal_settkipsplit);
EXPORT_SYMBOL(ath_hal_wmetkipmic);
EXPORT_SYMBOL(ath_hal_hwphycounters);
EXPORT_SYMBOL(ath_hal_hasdiversity);
EXPORT_SYMBOL(ath_hal_getdiversity);
EXPORT_SYMBOL(ath_hal_setdiversity);
EXPORT_SYMBOL(ath_hal_getnumtxqueues);
EXPORT_SYMBOL(ath_hal_hasveol);
EXPORT_SYMBOL(ath_hal_hastxpowlimit);
EXPORT_SYMBOL(ath_hal_settxpowlimit);
EXPORT_SYMBOL(ath_hal_gettxpowlimit);
EXPORT_SYMBOL(ath_hal_getmaxtxpow);
EXPORT_SYMBOL(ath_hal_gettpscale);
EXPORT_SYMBOL(ath_hal_settpscale);
EXPORT_SYMBOL(ath_hal_hastpc);
EXPORT_SYMBOL(ath_hal_gettpc);
EXPORT_SYMBOL(ath_hal_settpc);
EXPORT_SYMBOL(ath_hal_hasbursting);
EXPORT_SYMBOL(ath_hal_hascompression);
EXPORT_SYMBOL(ath_hal_hasfastframes);
EXPORT_SYMBOL(ath_hal_hasbssidmask);
EXPORT_SYMBOL(ath_hal_hasmcastkeysearch);
EXPORT_SYMBOL(ath_hal_getmcastkeysearch);
EXPORT_SYMBOL(ath_hal_hastkipmic);
EXPORT_SYMBOL(ath_hal_gettkipmic);
EXPORT_SYMBOL(ath_hal_settkipmic);
EXPORT_SYMBOL(ath_hal_hastsfadjust);
EXPORT_SYMBOL(ath_hal_gettsfadjust);
EXPORT_SYMBOL(ath_hal_settsfadjust);
EXPORT_SYMBOL(ath_hal_setrfsilent);
EXPORT_SYMBOL(ath_hal_hasrfsilent);
EXPORT_SYMBOL(ath_hal_setuprxdesc);
EXPORT_SYMBOL(ath_hal_rxprocdesc);
EXPORT_SYMBOL(ath_hal_setuptxdesc);
EXPORT_SYMBOL(ath_hal_setupxtxdesc);
EXPORT_SYMBOL(ath_hal_filltxdesc);
EXPORT_SYMBOL(ath_hal_txprocdesc);
EXPORT_SYMBOL(ath_hal_gettxintrtxqs);
EXPORT_SYMBOL(ath_hal_txreqintrdesc);
EXPORT_SYMBOL(ath_hal_gpioCfgOutput);
EXPORT_SYMBOL(ath_hal_gpioset);
EXPORT_SYMBOL(ath_hal_setcoverageclass);
EXPORT_SYMBOL(ath_hal_radar_wait);
EXPORT_SYMBOL(ath_hal_get_channel_noise);
EXPORT_SYMBOL(ath_hal_set_regulatory_domain);
EXPORT_SYMBOL(ath_hal_getpower);
EXPORT_SYMBOL(ath_hal_set_multicast_filter_index);
EXPORT_SYMBOL(ath_hal_clear_multicast_filter_index);
EXPORT_SYMBOL(ath_hal_gpioGet);
EXPORT_SYMBOL(ath_hal_gpioCfgInput);
EXPORT_SYMBOL(ath_hal_gpioSetIntr);
EXPORT_SYMBOL(ath_hal_detect_card_present);
EXPORT_SYMBOL(ath_hal_update_mib_counters);
EXPORT_SYMBOL(ath_hal_get_antenna_switch);
EXPORT_SYMBOL(ath_hal_set_antenna_switch);
EXPORT_SYMBOL(ath_hal_set_beacon_timers);

#endif /* #ifndef AH_HALOPS_FUNC */
