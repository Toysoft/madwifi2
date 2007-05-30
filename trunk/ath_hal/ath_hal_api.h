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
 *s
 */
#ifndef _IF_ATH_HAL_H
#define _IF_ATH_HAL_H

#include "ah.h"

#ifndef AH_HALOPS_FUNC
/*
 * HAL definitions to comply with local coding convention.
 */
#define	ath_hal_reset(_ah, _opmode, _chan, _outdoor, _pstatus) \
	((*(_ah)->ah_reset)((_ah), (_opmode), (_chan), (_outdoor), (_pstatus)))
#define	ath_hal_getratetable(_ah, _mode) \
	((*(_ah)->ah_getRateTable)((_ah), (_mode)))
#define	ath_hal_getmac(_ah, _mac) \
	((*(_ah)->ah_getMacAddress)((_ah), (_mac)))
#define	ath_hal_setmac(_ah, _mac) \
	((*(_ah)->ah_setMacAddress)((_ah), (_mac)))
#define	ath_hal_getbssidmask(_ah, _mask) \
	((*(_ah)->ah_getBssIdMask)((_ah), (_mask)))
#define	ath_hal_setbssidmask(_ah, _mask) \
	((*(_ah)->ah_setBssIdMask)((_ah), (_mask)))
#define	ath_hal_intrset(_ah, _mask) \
	((*(_ah)->ah_setInterrupts)((_ah), (_mask)))
#define	ath_hal_intrget(_ah) \
	((*(_ah)->ah_getInterrupts)((_ah)))
#define	ath_hal_intrpend(_ah) \
	((*(_ah)->ah_isInterruptPending)((_ah)))
#define	ath_hal_getisr(_ah, _pmask) \
	((*(_ah)->ah_getPendingInterrupts)((_ah), (_pmask)))
#define	ath_hal_updatetxtriglevel(_ah, _inc) \
	((*(_ah)->ah_updateTxTrigLevel)((_ah), (_inc)))
#define	ath_hal_setpower(_ah, _mode) \
	((*(_ah)->ah_setPowerMode)((_ah), (_mode), AH_TRUE))
#define	ath_hal_keycachesize(_ah) \
	((*(_ah)->ah_getKeyCacheSize)((_ah)))
#define	ath_hal_keyreset(_ah, _ix) \
	((*(_ah)->ah_resetKeyCacheEntry)((_ah), (_ix)))
#define	ath_hal_keyset(_ah, _ix, _pk, _mac) \
	((*(_ah)->ah_setKeyCacheEntry)((_ah), (_ix), (_pk), (_mac), AH_FALSE))
#define	ath_hal_keyisvalid(_ah, _ix) \
	(((*(_ah)->ah_isKeyCacheEntryValid)((_ah), (_ix))))
#define	ath_hal_keysetmac(_ah, _ix, _mac) \
	((*(_ah)->ah_setKeyCacheEntryMac)((_ah), (_ix), (_mac)))
#define	ath_hal_getrxfilter(_ah) \
	((*(_ah)->ah_getRxFilter)((_ah)))
#define	ath_hal_setrxfilter(_ah, _filter) \
	((*(_ah)->ah_setRxFilter)((_ah), (_filter)))
#define	ath_hal_setmcastfilter(_ah, _mfilt0, _mfilt1) \
	((*(_ah)->ah_setMulticastFilter)((_ah), (_mfilt0), (_mfilt1)))
#define	ath_hal_waitforbeacon(_ah, _bf) \
	((*(_ah)->ah_waitForBeaconDone)((_ah), (_bf)->bf_daddr))
#define	ath_hal_putrxbuf(_ah, _bufaddr) \
	((*(_ah)->ah_setRxDP)((_ah), (_bufaddr)))
#define	ath_hal_gettsf32(_ah) \
	((*(_ah)->ah_getTsf32)((_ah)))
#define	ath_hal_gettsf64(_ah) \
	((*(_ah)->ah_getTsf64)((_ah)))
#define	ath_hal_resettsf(_ah) \
	((*(_ah)->ah_resetTsf)((_ah)))
#define	ath_hal_rxena(_ah) \
	((*(_ah)->ah_enableReceive)((_ah)))
#define	ath_hal_numtxpending(_ah, _q) \
	((*(_ah)->ah_numTxPending)((_ah), (_q)))
#define	ath_hal_puttxbuf(_ah, _q, _bufaddr) \
	((*(_ah)->ah_setTxDP)((_ah), (_q), (_bufaddr)))
#define	ath_hal_gettxbuf(_ah, _q) \
	((*(_ah)->ah_getTxDP)((_ah), (_q)))
#define	ath_hal_getrxbuf(_ah) \
	((*(_ah)->ah_getRxDP)((_ah)))
#define	ath_hal_txstart(_ah, _q) \
	((*(_ah)->ah_startTxDma)((_ah), (_q)))
#define	ath_hal_setchannel(_ah, _chan) \
	((*(_ah)->ah_setChannel)((_ah), (_chan)))
#define	ath_hal_calibrate(_ah, _chan, _isIQdone) \
	((*(_ah)->ah_perCalibration)((_ah), (_chan), (_isIQdone)))
#define	ath_hal_setledstate(_ah, _state) \
	((*(_ah)->ah_setLedState)((_ah), (_state)))
#define	ath_hal_beaconinit(_ah, _nextb, _bperiod) \
	((*(_ah)->ah_beaconInit)((_ah), (_nextb), (_bperiod)))
#define	ath_hal_beaconreset(_ah) \
	((*(_ah)->ah_resetStationBeaconTimers)((_ah)))
#define	ath_hal_beacontimers(_ah, _bs) \
	((*(_ah)->ah_setStationBeaconTimers)((_ah), (_bs)))
#define	ath_hal_setassocid(_ah, _bss, _associd) \
	((*(_ah)->ah_writeAssocid)((_ah), (_bss), (_associd)))
#define	ath_hal_phydisable(_ah) \
	((*(_ah)->ah_phyDisable)((_ah)))
#define	ath_hal_setopmode(_ah) \
	((*(_ah)->ah_setPCUConfig)((_ah)))
#define	ath_hal_stoptxdma(_ah, _qnum) \
	((*(_ah)->ah_stopTxDma)((_ah), (_qnum)))
#define	ath_hal_stoppcurecv(_ah) \
	((*(_ah)->ah_stopPcuReceive)((_ah)))
#define	ath_hal_startpcurecv(_ah) \
	((*(_ah)->ah_startPcuReceive)((_ah)))
#define	ath_hal_stopdmarecv(_ah) \
	((*(_ah)->ah_stopDmaReceive)((_ah)))
#define	ath_hal_getdiagstate(_ah, _id, _indata, _insize, _outdata, _outsize) \
	((*(_ah)->ah_getDiagState)((_ah), (_id), \
		(_indata), (_insize), (_outdata), (_outsize)))
#define	ath_hal_gettxqueueprops(_ah, _q, _qi) \
	((*(_ah)->ah_getTxQueueProps)((_ah), (_q), (_qi)))
#define	ath_hal_settxqueueprops(_ah, _q, _qi) \
	((*(_ah)->ah_setTxQueueProps)((_ah), (_q), (_qi)))
#define	ath_hal_setuptxqueue(_ah, _type, _irq) \
	((*(_ah)->ah_setupTxQueue)((_ah), (_type), (_irq)))
#define	ath_hal_resettxqueue(_ah, _q) \
	((*(_ah)->ah_resetTxQueue)((_ah), (_q)))
#define	ath_hal_releasetxqueue(_ah, _q) \
	((*(_ah)->ah_releaseTxQueue)((_ah), (_q)))
#define	ath_hal_getrfgain(_ah) \
	((*(_ah)->ah_getRfGain)((_ah)))
#define	ath_hal_getdefantenna(_ah) \
	((*(_ah)->ah_getDefAntenna)((_ah)))
#define	ath_hal_setdefantenna(_ah, _ant) \
	((*(_ah)->ah_setDefAntenna)((_ah), (_ant)))
#define	ath_hal_rxmonitor(_ah, _arg, _chan) \
	((*(_ah)->ah_rxMonitor)((_ah), (_arg), (_chan)))
#define	ath_hal_mibevent(_ah, _stats) \
	((*(_ah)->ah_procMibEvent)((_ah), (_stats)))
#define	ath_hal_setslottime(_ah, _us) \
	((*(_ah)->ah_setSlotTime)((_ah), (_us)))
#define	ath_hal_getslottime(_ah) \
	((*(_ah)->ah_getSlotTime)((_ah)))
#define	ath_hal_setacktimeout(_ah, _us) \
	((*(_ah)->ah_setAckTimeout)((_ah), (_us)))
#define	ath_hal_getacktimeout(_ah) \
	((*(_ah)->ah_getAckTimeout)((_ah)))
#define	ath_hal_setctstimeout(_ah, _us) \
	((*(_ah)->ah_setCTSTimeout)((_ah), (_us)))
#define	ath_hal_getctstimeout(_ah) \
	((*(_ah)->ah_getCTSTimeout)((_ah)))
#define ath_hal_setdecompmask(_ah, _keyid, _b) \
	((*(_ah)->ah_setDecompMask)((_ah), (_keyid), (_b)))
#define	ath_hal_enablePhyDiag(_ah) \
	((*(_ah)->ah_enablePhyErrDiag)((_ah)))
#define	ath_hal_disablePhyDiag(_ah) \
	((*(_ah)->ah_disablePhyErrDiag)((_ah)))
#define	ath_hal_getcapability(_ah, _cap, _param, _result) \
	((*(_ah)->ah_getCapability)((_ah), (_cap), (_param), (_result)))
#define	ath_hal_setcapability(_ah, _cap, _param, _v, _status) \
	((*(_ah)->ah_setCapability)((_ah), (_cap), (_param), (_v), (_status)))
#define	ath_hal_ciphersupported(_ah, _cipher) \
	(ath_hal_getcapability(_ah, HAL_CAP_CIPHER, _cipher, NULL) == HAL_OK)
#define	ath_hal_fastframesupported(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK)
#define ath_hal_burstsupported(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_BURST, 0, NULL) == HAL_OK)
#define ath_hal_xrsupported(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_XR, 0, NULL) == HAL_OK)
#define ath_hal_compressionsupported(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK)
#define ath_hal_turboagsupported(_ah, _countrycode) \
	(ath_hal_getwirelessmodes(_ah, _countrycode) & (HAL_MODE_108G|HAL_MODE_TURBO))
#define ath_hal_halfrate_chansupported(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_CHAN_HALFRATE, 0, NULL) == HAL_OK)
#define ath_hal_quarterrate_chansupported(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_CHAN_QUARTERRATE, 0, NULL) == HAL_OK)
#define	ath_hal_getregdomain(_ah, _prd) \
	ath_hal_getcapability(_ah, HAL_CAP_REG_DMN, 0, (_prd))
#define	ath_hal_getcountrycode(_ah, _pcc) \
	(*(_pcc) = (_ah)->ah_countryCode)
#define ath_hal_hastkipsplit(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TKIP_SPLIT, 0, NULL) == HAL_OK)
#define ath_hal_gettkipsplit(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TKIP_SPLIT, 1, NULL) == HAL_OK)
#define ath_hal_settkipsplit(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_TKIP_SPLIT, 1, _v, NULL)
#define	ath_hal_wmetkipmic(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_WME_TKIPMIC, 0, NULL) == HAL_OK)
#define	ath_hal_hwphycounters(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_PHYCOUNTERS, 0, NULL) == HAL_OK)
#define	ath_hal_hasdiversity(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_DIVERSITY, 0, NULL) == HAL_OK)
#define	ath_hal_getdiversity(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_DIVERSITY, 1, NULL) == HAL_OK)
#define	ath_hal_setdiversity(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_DIVERSITY, 1, _v, NULL)
#define	ath_hal_getnumtxqueues(_ah, _pv) \
	(ath_hal_getcapability(_ah, HAL_CAP_NUM_TXQUEUES, 0, _pv) == HAL_OK)
#define	ath_hal_hasveol(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_VEOL, 0, NULL) == HAL_OK)
#define	ath_hal_hastxpowlimit(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TXPOW, 0, NULL) == HAL_OK)
#define	ath_hal_settxpowlimit(_ah, _pow) \
	((*(_ah)->ah_setTxPowerLimit)((_ah), (_pow)))
#define	ath_hal_gettxpowlimit(_ah, _ppow) \
	(ath_hal_getcapability(_ah, HAL_CAP_TXPOW, 1, _ppow) == HAL_OK)
#define	ath_hal_getmaxtxpow(_ah, _ppow) \
	(ath_hal_getcapability(_ah, HAL_CAP_TXPOW, 2, _ppow) == HAL_OK)
#define	ath_hal_gettpscale(_ah, _scale) \
	(ath_hal_getcapability(_ah, HAL_CAP_TXPOW, 3, _scale) == HAL_OK)
#define	ath_hal_settpscale(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_TXPOW, 3, _v, NULL)
#define	ath_hal_hastpc(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TPC, 0, NULL) == HAL_OK)
#define	ath_hal_gettpc(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TPC, 1, NULL) == HAL_OK)
#define	ath_hal_settpc(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_TPC, 1, _v, NULL)
#define	ath_hal_hasbursting(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_BURST, 0, NULL) == HAL_OK)
#define ath_hal_hascompression(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK)
#define	ath_hal_hasfastframes(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK)
#define	ath_hal_hasbssidmask(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_BSSIDMASK, 0, NULL) == HAL_OK)
#define	ath_hal_hasmcastkeysearch(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_MCAST_KEYSRCH, 0, NULL) == HAL_OK)
#define	ath_hal_getmcastkeysearch(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_MCAST_KEYSRCH, 1, NULL) == HAL_OK)
#define	ath_hal_hastkipmic(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TKIP_MIC, 0, NULL) == HAL_OK)
#define	ath_hal_gettkipmic(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TKIP_MIC, 1, NULL) == HAL_OK)
#define	ath_hal_settkipmic(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_TKIP_MIC, 1, _v, NULL)
#define	ath_hal_hastsfadjust(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TSF_ADJUST, 0, NULL) == HAL_OK)
#define	ath_hal_gettsfadjust(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_TSF_ADJUST, 1, NULL) == HAL_OK)
#define	ath_hal_settsfadjust(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_TSF_ADJUST, 1, _v, NULL)
#define ath_hal_setrfsilent(_ah, _v) \
	ath_hal_setcapability(_ah, HAL_CAP_RFSILENT, 1, _v, NULL)
#define ath_hal_hasrfsilent(_ah) \
	(ath_hal_getcapability(_ah, HAL_CAP_RFSILENT, 0, NULL) == HAL_OK)

#define	ath_hal_setuprxdesc(_ah, _ds, _size, _intreq) \
	((*(_ah)->ah_setupRxDesc)((_ah), (_ds), (_size), (_intreq)))
#define	ath_hal_rxprocdesc(_ah, _ds, _dspa, _dsnext, _tsf, _rs) \
	((*(_ah)->ah_procRxDesc)((_ah), (_ds), (_dspa), (_dsnext), (_tsf), (_rs)))
#define	ath_hal_updateCTSForBursting(_ah, _ds, _prevds, _prevdsWithCTS, _gatingds,    \
                                     _txOpLimit, _ctsDuration)			      \
	((*(_ah)->ah_updateCTSForBursting)((_ah), (_ds), (_prevds), (_prevdsWithCTS), \
	                                   (_gatingds), (_txOpLimit), (_ctsDuration)))
#define	ath_hal_setuptxdesc(_ah, _ds, _plen, _hlen, _atype, _txpow, \
		_txr0, _txtr0, _keyix, _ant, _flags, \
		_rtsrate, _rtsdura, \
		_compicvlen, _compivlen, _comp) \
	((*(_ah)->ah_setupTxDesc)((_ah), (_ds), (_plen), (_hlen), (_atype), \
		(_txpow), (_txr0), (_txtr0), (_keyix), (_ant), \
		(_flags), (_rtsrate), (_rtsdura), \
		(_compicvlen), (_compivlen), (_comp)))
#define	ath_hal_setupxtxdesc(_ah, _ds, \
		_txr1, _txtr1, _txr2, _txtr2, _txr3, _txtr3) \
	((*(_ah)->ah_setupXTxDesc)((_ah), (_ds), \
		(_txr1), (_txtr1), (_txr2), (_txtr2), (_txr3), (_txtr3)))
#define	ath_hal_filltxdesc(_ah, _ds, _l, _first, _last, _ds0) \
	((*(_ah)->ah_fillTxDesc)((_ah), (_ds), (_l), (_first), (_last), (_ds0)))
#define	ath_hal_txprocdesc(_ah, _ds, _ts) \
	((*(_ah)->ah_procTxDesc)((_ah), (_ds), (_ts)))
#define ath_hal_gettxintrtxqs(_ah, _txqs) \
	((*(_ah)->ah_getTxIntrQueue)((_ah), (_txqs)))
#define	ath_hal_txreqintrdesc(_ah, _ds) \
	((*(_ah)->ah_reqTxIntrDesc)((_ah), (_ds)))

#define ath_hal_gpioCfgOutput(_ah, _gpio) \
	((*(_ah)->ah_gpioCfgOutput)((_ah), (_gpio)))
#define ath_hal_gpioset(_ah, _gpio, _b) \
	((*(_ah)->ah_gpioSet)((_ah), (_gpio), (_b)))
#define	ath_hal_setcoverageclass(_ah, _coverageclass, _now) \
	((*(_ah)->ah_setCoverageClass)((_ah), (_coverageclass), (_now)))
#define ath_hal_radar_wait(_ah, _chan) \
	((*(_ah)->ah_radarWait)((_ah), (_chan)))
#define ath_hal_get_channel_noise(_ah, _chan) \
	((*(_ah)->ah_getChanNoise)((_ah), (_chan)))

#else // if AH_HALOPS_FUNC

HAL_BOOL ath_hal_reset(struct ath_hal* ah, HAL_OPMODE opmode, HAL_CHANNEL* chan,  HAL_BOOL outdoor, HAL_STATUS* pstatus);
const HAL_RATE_TABLE* ath_hal_getratetable(struct ath_hal* ah, u_int mode);
void ath_hal_getmac(struct ath_hal* ah, u_int8_t* mac);
HAL_BOOL ath_hal_setmac(struct ath_hal* ah, const u_int8_t* mac);
void ath_hal_getbssidmask(struct ath_hal * ah, u_int8_t * mask);
HAL_BOOL ath_hal_setbssidmask(struct ath_hal * ah, const u_int8_t* mask);
HAL_INT	ath_hal_intrset(struct ath_hal* ah, HAL_INT mask);
HAL_INT	ath_hal_intrget(struct ath_hal* ah);
HAL_BOOL ath_hal_intrpend(struct ath_hal* ah);
HAL_BOOL ath_hal_getisr(struct ath_hal* ah, HAL_INT* pmask);
HAL_BOOL ath_hal_updatetxtriglevel(struct ath_hal* ah, HAL_BOOL inc);
HAL_BOOL ath_hal_setpower(struct ath_hal* ah, HAL_POWER_MODE mode);
u_int32_t ath_hal_keycachesize(struct ath_hal* ah);
HAL_BOOL ath_hal_keyreset(struct ath_hal* ah, u_int16_t ix);
HAL_BOOL ath_hal_keyset(struct ath_hal* ah, u_int16_t ix, const HAL_KEYVAL * pk, const u_int8_t * mac);
HAL_BOOL ath_hal_keyisvalid(struct ath_hal *ah, u_int16_t ix);
HAL_BOOL ath_hal_keysetmac(struct ath_hal* ah, u_int16_t ix, const u_int8_t *mac);
u_int32_t ath_hal_getrxfilter(struct ath_hal* ah);
void ath_hal_setrxfilter(struct ath_hal* ah, u_int32_t filter);
void ath_hal_setmcastfilter(struct ath_hal* ah, u_int32_t mfilt0, u_int32_t mfilt1);
void ath_hal_putrxbuf(struct ath_hal* ah, u_int32_t bufaddr);
u_int32_t ath_hal_gettsf32(struct ath_hal* ah);
u_int64_t ath_hal_gettsf64(struct ath_hal* ah);
void ath_hal_resettsf(struct ath_hal* ah);
void ath_hal_rxena(struct ath_hal* ah);
u_int32_t ath_hal_numtxpending(struct ath_hal *hal, u_int q);
HAL_BOOL ath_hal_puttxbuf(struct ath_hal* ah, u_int q, u_int32_t bufaddr);
u_int32_t ath_hal_gettxbuf(struct ath_hal* ah, u_int q);
u_int32_t ath_hal_getrxbuf(struct ath_hal* ah);
HAL_BOOL ath_hal_txstart(struct ath_hal* ah, u_int q);
HAL_BOOL ath_hal_calibrate(struct ath_hal* ah, HAL_CHANNEL * chan, HAL_BOOL * isIQdone);
void ath_hal_setledstate(struct ath_hal* ah, HAL_LED_STATE state);
void ath_hal_beaconinit(struct ath_hal * ah, u_int32_t nextb, u_int32_t bperiod);
void ath_hal_beaconreset(struct ath_hal* ah);
void ath_hal_beacontimers(struct ath_hal* ah, const HAL_BEACON_STATE *bs);
void ath_hal_setassocid(struct ath_hal* ah, const u_int8_t *bss, u_int16_t associd);
HAL_BOOL ath_hal_phydisable(struct ath_hal * ah);
void ath_hal_setopmode(struct ath_hal * ah);
HAL_BOOL ath_hal_stoptxdma(struct ath_hal* ah, u_int q);
void ath_hal_stoppcurecv(struct ath_hal* ah);
void ath_hal_startpcurecv(struct ath_hal* ah);
HAL_BOOL ath_hal_stopdmarecv(struct ath_hal* ah);
HAL_BOOL ath_hal_getdiagstate(struct ath_hal *ah, int id, const void *indata, u_int32_t insize, void** outdata, u_int32_t* outsize);
HAL_BOOL ath_hal_gettxqueueprops(struct ath_hal * ah, int q, HAL_TXQ_INFO *qi);
HAL_BOOL ath_hal_settxqueueprops(struct ath_hal *ah, int q, const HAL_TXQ_INFO *qi);
int	ath_hal_setuptxqueue(struct ath_hal * ah, HAL_TX_QUEUE type, const HAL_TXQ_INFO * qi);
HAL_BOOL ath_hal_resettxqueue(struct ath_hal *ah, u_int q);
HAL_BOOL ath_hal_releasetxqueue(struct ath_hal *ah, u_int q);
HAL_RFGAIN ath_hal_getrfgain(struct ath_hal* ah);
u_int ath_hal_getdefantenna(struct ath_hal* ah);
void ath_hal_setdefantenna(struct ath_hal* ah, u_int ant);
void ath_hal_rxmonitor(struct ath_hal* ah, const HAL_NODE_STATS *stats, HAL_CHANNEL *chan);
void ath_hal_mibevent(struct ath_hal * ah, const HAL_NODE_STATS *stats);
HAL_BOOL ath_hal_setslottime(struct ath_hal* ah, u_int us);
u_int ath_hal_getslottime(struct ath_hal* ah);
HAL_BOOL ath_hal_setacktimeout(struct ath_hal* ah, u_int us);
u_int ath_hal_getacktimeout(struct ath_hal* ah);
HAL_BOOL ath_hal_setctstimeout(struct ath_hal* ah, u_int us);
u_int ath_hal_getctstimeout(struct ath_hal* ah);
HAL_BOOL ath_hal_setdecompmask(struct ath_hal* ah, u_int16_t keyid, int b);
HAL_STATUS ath_hal_getcapability(struct ath_hal * ah, HAL_CAPABILITY_TYPE type, u_int32_t capability, u_int32_t *result);
HAL_BOOL ath_hal_setcapability(struct ath_hal * ah, HAL_CAPABILITY_TYPE type, u_int32_t capability, u_int32_t setting, HAL_STATUS *status);
HAL_BOOL ath_hal_ciphersupported(struct ath_hal * ah, u_int32_t cipher);
HAL_BOOL ath_hal_fastframesupported(struct ath_hal * ah);
HAL_BOOL ath_hal_burstsupported(struct ath_hal * ah);
HAL_BOOL ath_hal_xrsupported(struct ath_hal * ah);
HAL_BOOL ath_hal_compressionsupported(struct ath_hal * ah);
HAL_BOOL ath_hal_turboagsupported(struct ath_hal * ah, int countrycode);
HAL_BOOL ath_hal_halfrate_chansupported(struct ath_hal * ah);
HAL_BOOL ath_hal_quarterrate_chansupported(struct ath_hal * ah);
HAL_BOOL ath_hal_getregdomain(struct ath_hal * ah, u_int32_t* destination);
HAL_BOOL ath_hal_getcountrycode(struct ath_hal * ah, u_int32_t* destination);
HAL_BOOL ath_hal_hastkipsplit(struct ath_hal * ah);
HAL_BOOL ath_hal_gettkipsplit(struct ath_hal * ah);
HAL_BOOL ath_hal_settkipsplit(struct ath_hal * ah, int v);
HAL_BOOL ath_hal_wmetkipmic(struct ath_hal * ah);
HAL_BOOL ath_hal_hwphycounters(struct ath_hal * ah);
HAL_BOOL ath_hal_hasdiversity(struct ath_hal * ah);
HAL_BOOL ath_hal_getdiversity(struct ath_hal * ah);
HAL_BOOL ath_hal_setdiversity(struct ath_hal * ah, int v);
HAL_BOOL ath_hal_getnumtxqueues(struct ath_hal * ah, u_int32_t* destination);
HAL_BOOL ath_hal_hasveol(struct ath_hal * ah);
HAL_BOOL ath_hal_hastxpowlimit(struct ath_hal * ah);
HAL_BOOL ath_hal_settxpowlimit(struct ath_hal * ah, u_int32_t power);
HAL_BOOL ath_hal_gettxpowlimit(struct ath_hal * ah, u_int32_t* destination);
HAL_BOOL ath_hal_getmaxtxpow(struct ath_hal * ah, u_int32_t* destination);
HAL_BOOL ath_hal_gettpscale(struct ath_hal * ah, u_int32_t* destination);
HAL_BOOL ath_hal_settpscale(struct ath_hal * ah, u_int32_t v);
HAL_BOOL ath_hal_hastpc(struct ath_hal * ah);
HAL_BOOL ath_hal_gettpc(struct ath_hal * ah);
HAL_BOOL ath_hal_settpc(struct ath_hal * ah, u_int32_t v);
HAL_BOOL ath_hal_hasbursting(struct ath_hal * ah);
HAL_BOOL ath_hal_hascompression(struct ath_hal * ah);
HAL_BOOL ath_hal_hasfastframes(struct ath_hal * ah);
HAL_BOOL ath_hal_hasbssidmask(struct ath_hal * ah);
HAL_BOOL ath_hal_hasmcastkeysearch(struct ath_hal * ah);
HAL_BOOL ath_hal_getmcastkeysearch(struct ath_hal * ah);
HAL_BOOL ath_hal_hastkipmic(struct ath_hal * ah);
HAL_BOOL ath_hal_gettkipmic(struct ath_hal * ah);
HAL_BOOL ath_hal_settkipmic(struct ath_hal * ah, u_int32_t v);
HAL_BOOL ath_hal_hastsfadjust(struct ath_hal * ah);
HAL_BOOL ath_hal_gettsfadjust(struct ath_hal * ah);
HAL_BOOL ath_hal_settsfadjust(struct ath_hal * ah, u_int32_t v);
HAL_BOOL ath_hal_setrfsilent(struct ath_hal * ah, u_int32_t v);
HAL_BOOL ath_hal_hasrfsilent(struct ath_hal * ah);
HAL_BOOL ath_hal_setuprxdesc(struct ath_hal *ah, struct ath_desc *ds, u_int32_t size, u_int flags);
HAL_BOOL ath_hal_rxprocdesc(struct ath_hal *ah, struct ath_desc *ds, u_int32_t dspa, struct ath_desc *dsnext, u_int64_t tsf, struct ath_rx_status *rs);
HAL_BOOL ath_hal_setuptxdesc(struct ath_hal *ah, struct ath_desc *ds, u_int plen, u_int hlen, HAL_PKT_TYPE atype, u_int txpow, u_int txr0, u_int txtr0, u_int keyix, u_int ant, u_int flags, u_int rtsrate, u_int rtsdura, u_int compicvlen, u_int compivlen, u_int comp);
HAL_BOOL ath_hal_setupxtxdesc(struct ath_hal * ah, struct ath_desc* ds, u_int txr1, u_int txtr1, u_int txr2, u_int txtr2, u_int txr3, u_int txtr3);
HAL_BOOL ath_hal_filltxdesc(struct ath_hal * ah, struct ath_desc * ds, u_int l, HAL_BOOL first, HAL_BOOL last, const struct ath_desc * ds0);
HAL_BOOL ath_hal_txprocdesc(struct ath_hal * ah, struct ath_desc *ds, struct ath_tx_status *ts);
void ath_hal_gettxintrtxqs(struct ath_hal *ah, u_int32_t *txqs);
void ath_hal_txreqintrdesc(struct ath_hal * ah, struct ath_desc *ds);
HAL_BOOL ath_hal_gpioCfgOutput(struct ath_hal * ah, u_int32_t gpio);
HAL_BOOL ath_hal_gpioset(struct ath_hal * ah, u_int32_t gpio, u_int32_t val);
void ath_hal_setcoverageclass(struct ath_hal * ah, u_int8_t coverageclass, int now);
HAL_BOOL ath_hal_radar_wait(struct ath_hal * ah, HAL_CHANNEL *chan);
int16_t ath_hal_get_channel_noise(struct ath_hal * ah, HAL_CHANNEL *chan);
HAL_BOOL ath_hal_set_regulatory_domain(struct ath_hal* ah, u_int16_t domain, HAL_STATUS* pstatus);
HAL_POWER_MODE ath_hal_getpower(struct ath_hal* ah);
HAL_BOOL ath_hal_set_multicast_filter_index(struct ath_hal* ah, u_int32_t index);
HAL_BOOL ath_hal_clear_multicast_filter_index(struct ath_hal* ah, u_int32_t index);
HAL_BOOL ath_hal_gpioGet(struct ath_hal *ah, u_int32_t gpio);
HAL_BOOL ath_hal_gpioCfgInput(struct ath_hal *ah, u_int32_t gpio);
void ath_hal_gpioSetIntr(struct ath_hal *ah, u_int p1, u_int32_t p2);
HAL_BOOL ath_hal_detect_card_present(struct ath_hal* ah);
void ath_hal_update_mib_counters(struct ath_hal* ah, HAL_MIB_STATS* stats);
HAL_ANT_SETTING	ath_hal_get_antenna_switch(struct ath_hal* ah);
HAL_BOOL ath_hal_set_antenna_switch(struct ath_hal* ah, HAL_ANT_SETTING ant);
void ath_hal_set_beacon_timers(struct ath_hal* ah, const HAL_BEACON_TIMERS *timers);

#endif /* #ifndef AH_HALOPS_FUNC */

#endif /* _IF_ATH_HAL_H */

