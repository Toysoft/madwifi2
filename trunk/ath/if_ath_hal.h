/* Wrapper macros/functions for the binary HAL to comply with local coding
 * convention.  Provides function-style calling convention using either macros
 * or wrapper functions for function pointers in the HAL.
 *
 * The typical convention is ath_hal_foo(ah,p1,p2,p3,...) turns into
 * ah->ah_foo(p1,p2,p3,...) where ah_foo is a function pointer and a member
 * of the struct ath_hal (usually named ah). */


#ifndef _IF_ATH_HAL_H
#define _IF_ATH_HAL_H

#define GET_ATH_SOFTC(_ah) 	((struct ath_softc*)(_ah->ah_sc))
#define ATH_HAL_LOCK_INIT(_sc) 	spin_lock_init(&(_sc)->sc_hal_lock)
#define ATH_HAL_LOCK_DESTROY(_sc)
#define ATH_HAL_LOCK_IRQ(_sc) 	do { \
   unsigned long __sc_halLockflags; \
   spin_lock_irqsave(&(_sc)->sc_hal_lock, __sc_halLockflags);
#define ATH_HAL_UNLOCK_IRQ(_sc) \
   spin_unlock_irqrestore(&(_sc)->sc_hal_lock, __sc_halLockflags); \
   } while(0)
#define ATH_HAL_UNLOCK_IRQ_EARLY(_sc) \
   spin_unlock_irqrestore(&(_sc)->sc_hal_lock, __sc_halLockflags);

#ifdef ATH_HALOPS_TRACEABLE
#define __hal_wrapper
#ifdef TRACEABLE_IMPL
#define IMPLEMENTATION(_CODEBLOCK) _CODEBLOCK
#else /* #ifdef TRACEABLE_IMPL */
#define IMPLEMENTATION(_CODEBLOCK)
#endif /* #ifdef TRACEABLE_IMPL */
#else /* #ifdef ATH_HALOPS_TRACEABLE */
#define __hal_wrapper static inline
#define IMPLEMENTATION(_CODEBLOCK) _CODEBLOCK
#endif /* #ifdef ATH_HALOPS_TRACEABLE */
__hal_wrapper void ath_hal_getmac(struct ath_hal* ah, u_int8_t* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_getMacAddress(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_POWER_MODE ath_hal_getPowerMode(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_POWER_MODE ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getPowerMode(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_getdiagstate(struct ath_hal* ah, int request, const void* args, u_int32_t argsize, void* *result, u_int32_t* resultsize)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getDiagState(ah, request, args, argsize, *result, resultsize);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_beaconreset(struct ath_hal* ah)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_resetStationBeaconTimers(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_setcoverageclass(struct ath_hal* ah, u_int8_t a1, int a2)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setCoverageClass(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper u_int64_t ath_hal_gettsf64(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int64_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getTsf64(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_rxena(struct ath_hal* ah)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_enableReceive(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_ANT_SETTING ath_hal_getantennaswitch(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_ANT_SETTING ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getAntennaSwitch(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_gpioset(struct ath_hal* ah, u_int32_t gpio, u_int32_t val)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_gpioSet(ah, gpio, val);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_gpioCfgOutput(struct ath_hal* ah, u_int32_t gpio)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_gpioCfgOutput(ah, gpio);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_clearmcastfilter(struct ath_hal* ah, u_int32_t index)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_clrMulticastFilterIndex(ah, index);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_txreqintrdesc(struct ath_hal* ah, struct ath_desc* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_reqTxIntrDesc(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_rxmonitor(struct ath_hal* ah, const HAL_NODE_STATS* a1, HAL_CHANNEL* a2)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_rxMonitor(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_puttxbuf(struct ath_hal* ah, u_int a1, u_int32_t txdp)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setTxDP(ah, a1, txdp);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_keyset(struct ath_hal* ah, u_int16_t a1, const HAL_KEYVAL* a2, const u_int8_t* a3, int a4)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setKeyCacheEntry(ah, a1, a2, a3, a4);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_setopmode(struct ath_hal* ah)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setPCUConfig(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_RFGAIN ath_hal_getrfgain(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_RFGAIN ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getRfGain(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_setmcastfilter(struct ath_hal* ah, u_int32_t filter0, u_int32_t filter1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setMulticastFilter(ah, filter0, filter1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper u_int ath_hal_getacktimeout(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getAckTimeout(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_beacontimers(struct ath_hal* ah, const HAL_BEACON_STATE* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setStationBeaconTimers(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_detectcardpresent(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_detectCardPresent(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int ath_hal_getslottime(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getSlotTime(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_beaconinit(struct ath_hal* ah, u_int32_t nexttbtt, u_int32_t intval)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_beaconInit(ah, nexttbtt, intval);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_gpiosetintr(struct ath_hal* ah, u_int a1, u_int32_t a2)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_gpioSetIntr(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_releasetxqueue(struct ath_hal* ah, u_int q)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_releaseTxQueue(ah, q);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_keysetmac(struct ath_hal* ah, u_int16_t a1, const u_int8_t* a2)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setKeyCacheEntryMac(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_STATUS ath_hal_txprocdesc(struct ath_hal* ah, struct ath_desc* a1, struct ath_tx_status* a2)
	IMPLEMENTATION({
		HAL_STATUS ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_procTxDesc(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_INT ath_hal_intrget(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_INT ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getInterrupts(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setacktimeout(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setAckTimeout(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setbssidmask(struct ath_hal* ah, const u_int8_t* a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setBssIdMask(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setackctsrate(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setAckCTSRate(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_getrxfilter(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getRxFilter(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper int16_t ath_hal_get_channel_noise(struct ath_hal* ah, HAL_CHANNEL* a1)
	IMPLEMENTATION({
		int16_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getChanNoise(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_keyreset(struct ath_hal* ah, u_int16_t a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_resetKeyCacheEntry(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setantennaswitch(struct ath_hal* ah, HAL_ANT_SETTING a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setAntennaSwitch(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_settxqueueprops(struct ath_hal* ah, int q, const HAL_TXQ_INFO* qInfo)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setTxQueueProps(ah, q, qInfo);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_putrxbuf(struct ath_hal* ah, u_int32_t rxdp)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setRxDP(ah, rxdp);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_setdecompmask(struct ath_hal* ah, u_int16_t a1, int a2)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setDecompMask(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_gettxqueueprops(struct ath_hal* ah, int q, HAL_TXQ_INFO* qInfo)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getTxQueueProps(ah, q, qInfo);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_filltxdesc(struct ath_hal* ah, struct ath_desc* a1, u_int segLen, HAL_BOOL firstSeg, HAL_BOOL lastSeg, const struct ath_desc* a5)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_fillTxDesc(ah, a1, segLen, firstSeg, lastSeg, a5);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_numtxpending(struct ath_hal* ah, u_int q)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_numTxPending(ah, q);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_startpcurecv(struct ath_hal* ah)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_startPcuReceive(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_setdefantenna(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setDefAntenna(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_setpower(struct ath_hal* ah, HAL_POWER_MODE mode, int setChip)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setPowerMode(ah, mode, setChip);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_STATUS ath_hal_rxprocdesc(struct ath_hal* ah, struct ath_desc* a1, u_int32_t phyAddr, struct ath_desc* next, u_int64_t tsf, struct ath_rx_status* a5)
	IMPLEMENTATION({
		HAL_STATUS ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_procRxDesc(ah, a1, phyAddr, next, tsf, a5);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int ath_hal_getackctsrate(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getAckCTSRate(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_keycachesize(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getKeyCacheSize(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setupxtxdesc(struct ath_hal* ah, struct ath_desc* a1, u_int txRate1, u_int txTries1, u_int txRate2, u_int txTries2, u_int txRate3, u_int txTries3)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setupXTxDesc(ah, a1, txRate1, txTries1, txRate2, txTries2, txRate3, txTries3);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_INT ath_hal_intrset(struct ath_hal* ah, HAL_INT a1)
	IMPLEMENTATION({
		HAL_INT ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setInterrupts(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int ath_hal_getctstimeout(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getCTSTimeout(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_updatemibcounters(struct ath_hal* ah, HAL_MIB_STATS* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_updateMibCounters(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_calibrate(struct ath_hal* ah, HAL_CHANNEL* a1, HAL_BOOL* a2)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_perCalibration(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_getrxbuf(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getRxDP(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_settxpowlimit(struct ath_hal* ah, u_int32_t a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setTxPowerLimit(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_getisr(struct ath_hal* ah, HAL_INT* a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getPendingInterrupts(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_updatetxtriglevel(struct ath_hal* ah, HAL_BOOL incTrigLevel)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_updateTxTrigLevel(ah, incTrigLevel);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_resettxqueue(struct ath_hal* ah, u_int q)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_resetTxQueue(ah, q);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setmac(struct ath_hal* ah, const u_int8_t* a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setMacAddress(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setctstimeout(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setCTSTimeout(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper const HAL_RATE_TABLE * ath_hal_getratetable(struct ath_hal* ah, u_int mode)
	IMPLEMENTATION({
		const HAL_RATE_TABLE * ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getRateTable(ah, mode);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_gettsf32(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getTsf32(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_mibevent(struct ath_hal* ah, const HAL_NODE_STATS* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_procMibEvent(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_setbeacontimers(struct ath_hal* ah, const HAL_BEACON_TIMERS* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setBeaconTimers(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_STATUS ath_hal_getcapability(struct ath_hal* ah, HAL_CAPABILITY_TYPE a1, u_int32_t capability, u_int32_t* result)
	IMPLEMENTATION({
		HAL_STATUS ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getCapability(ah, a1, capability, result);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_radar_wait(struct ath_hal* ah, HAL_CHANNEL* a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_radarWait(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setmcastfilterindex(struct ath_hal* ah, u_int32_t index)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setMulticastFilterIndex(ah, index);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_getbssidmask(struct ath_hal* ah, u_int8_t* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_getBssIdMask(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_intrpend(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_isInterruptPending(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_txstart(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_startTxDma(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_gettxintrtxqs(struct ath_hal* ah, u_int32_t* a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_getTxIntrQueue(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_setslottime(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setSlotTime(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_setledstate(struct ath_hal* ah, HAL_LED_STATE a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setLedState(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_setassocid(struct ath_hal* ah, const u_int8_t* bssid, u_int16_t assocId)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_writeAssocid(ah, bssid, assocId);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper void ath_hal_resettsf(struct ath_hal* ah)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_resetTsf(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_setuprxdesc(struct ath_hal* ah, struct ath_desc* a1, u_int32_t size, u_int flags)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setupRxDesc(ah, a1, size, flags);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_setrxfilter(struct ath_hal* ah, u_int32_t a1)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_setRxFilter(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_keyisvalid(struct ath_hal* ah, u_int16_t a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_isKeyCacheEntryValid(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper void ath_hal_stoppcurecv(struct ath_hal* ah)
	IMPLEMENTATION({
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ah->ah_stopPcuReceive(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	})
__hal_wrapper HAL_BOOL ath_hal_stoptxdma(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_stopTxDma(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setcapability(struct ath_hal* ah, HAL_CAPABILITY_TYPE a1, u_int32_t capability, u_int32_t setting, HAL_STATUS* a4)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setCapability(ah, a1, capability, setting, a4);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_stopdmarecv(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_stopDmaReceive(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_gettxbuf(struct ath_hal* ah, u_int a1)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getTxDP(ah, a1);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper int ath_hal_setuptxqueue(struct ath_hal* ah, HAL_TX_QUEUE a1, const HAL_TXQ_INFO* qInfo)
	IMPLEMENTATION({
		int ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setupTxQueue(ah, a1, qInfo);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int ath_hal_getdefantenna(struct ath_hal* ah)
	IMPLEMENTATION({
		u_int ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_getDefAntenna(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_phydisable(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_phyDisable(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setregulatorydomain(struct ath_hal* ah, u_int16_t a1, HAL_STATUS* a2)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setRegulatoryDomain(ah, a1, a2);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_setuptxdesc(struct ath_hal* ah, struct ath_desc* a1, u_int pktLen, u_int hdrLen, HAL_PKT_TYPE type, u_int txPower, u_int txRate0, u_int txTries0, u_int keyIx, u_int antMode, u_int flags, u_int rtsctsRate, u_int rtsctsDuration, u_int compicvLen, u_int compivLen, u_int comp)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_setupTxDesc(ah, a1, pktLen, hdrLen, type, txPower, txRate0, txTries0, keyIx, antMode, flags, rtsctsRate, rtsctsDuration, compicvLen, compivLen, comp);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_gpiCfgInput(struct ath_hal* ah, u_int32_t gpio)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_gpioCfgInput(ah, gpio);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper u_int32_t ath_hal_gpioget(struct ath_hal* ah, u_int32_t gpio)
	IMPLEMENTATION({
		u_int32_t ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_gpioGet(ah, gpio);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_disable(struct ath_hal* ah)
	IMPLEMENTATION({
		HAL_BOOL ret;
		ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
		ret = ah->ah_disable(ah);
		ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
		return ret;
	})
/* Begin custom wrappers.  Edit scripts/if_ath_hal_settings.pl!!! */
__hal_wrapper void ath_reg_write(struct ath_softc *sc, u_int reg, u_int32_t val)
IMPLEMENTATION({
   	ATH_HAL_LOCK_IRQ(sc);
   	OS_REG_WRITE(sc->sc_ah, reg, val);
   	ATH_HAL_UNLOCK_IRQ(sc);
        })

__hal_wrapper u_int32_t ath_reg_read(struct ath_softc *sc, u_int reg)
IMPLEMENTATION({
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(sc);
	ret = OS_REG_READ(sc->sc_ah, reg);
	ATH_HAL_UNLOCK_IRQ(sc);
	return ret;
        })
__hal_wrapper HAL_BOOL ath_hal_reset(struct ath_hal* ah, HAL_OPMODE opMode, HAL_CHANNEL* chan, HAL_BOOL bChannelChange, HAL_STATUS* status)
	IMPLEMENTATION({
	HAL_BOOL  ret;
	ATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));
	ret = ah->ah_reset(ah, opMode, chan, bChannelChange, status);
	ATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));
	return ret;
	})
__hal_wrapper HAL_BOOL ath_hal_burstsupported(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_BURST, 0, NULL) == HAL_OK); 
	})
__hal_wrapper HAL_BOOL ath_hal_ciphersupported(struct ath_hal * ah, u_int32_t cipher)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_CIPHER, cipher, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_compressionsupported(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_fastframesupported(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_getcountrycode(struct ath_hal * ah, u_int32_t* destination)
	IMPLEMENTATION({ 
		return ((*(destination) = ah->ah_countryCode), AH_TRUE);
	})
__hal_wrapper HAL_BOOL ath_hal_getdiversity(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 1, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_getmaxtxpow(struct ath_hal * ah, u_int32_t* destination)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 2, destination) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_getmcastkeysearch(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_MCAST_KEYSRCH, 1, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_getnumtxqueues(struct ath_hal * ah, u_int32_t* destination)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_NUM_TXQUEUES, 0, destination) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_getregdomain(struct ath_hal * ah, u_int32_t* destination)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_REG_DMN, 0, destination));
	})
__hal_wrapper HAL_BOOL ath_hal_gettkipmic(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TKIP_MIC, 1, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_gettkipsplit(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TKIP_SPLIT, 1, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_gettpc(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TPC, 1, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_gettpscale(struct ath_hal * ah, u_int32_t* destination)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 3, destination) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_gettsfadjust(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TSF_ADJUST, 1, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_gettxpowlimit(struct ath_hal * ah, u_int32_t* destination)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 1, destination) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_halfrate_chansupported(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_CHAN_HALFRATE, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasbssidmask(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_BSSIDMASK, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasbursting(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_BURST, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hascompression(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasdiversity(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasfastframes(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasmcastkeysearch(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_MCAST_KEYSRCH, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasrfsilent(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_RFSILENT, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hastkipmic(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TKIP_MIC, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hastkipsplit(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TKIP_SPLIT, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hastpc(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TPC, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hastsfadjust(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TSF_ADJUST, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hastxpowlimit(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_TXPOW, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hasveol(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_VEOL, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_hwphycounters(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_PHYCOUNTERS, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_quarterrate_chansupported(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_CHAN_QUARTERRATE, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_setdiversity(struct ath_hal * ah, int v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 1, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_setrfsilent(struct ath_hal * ah, u_int32_t v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_RFSILENT, 1, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_settkipmic(struct ath_hal * ah, u_int32_t v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_TKIP_MIC, 1, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_settkipsplit(struct ath_hal * ah, int v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_TKIP_SPLIT, 1, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_settpc(struct ath_hal * ah, u_int32_t v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_TPC, 1, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_settpscale(struct ath_hal * ah, u_int32_t v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_TXPOW, 3, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_settsfadjust(struct ath_hal * ah, u_int32_t v)
	IMPLEMENTATION({ 
		return (ath_hal_setcapability(ah, HAL_CAP_TSF_ADJUST, 1, v, NULL));
	})
__hal_wrapper HAL_BOOL ath_hal_turboagsupported(struct ath_hal * ah, int countrycode)
	IMPLEMENTATION({ 
		return (ath_hal_getwirelessmodes(ah, countrycode) & (HAL_MODE_108G|HAL_MODE_TURBO));
	})
__hal_wrapper HAL_BOOL ath_hal_wmetkipmic(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return (ath_hal_getcapability(ah, HAL_CAP_WME_TKIPMIC, 0, NULL) == HAL_OK);
	})
__hal_wrapper HAL_BOOL ath_hal_xrsupported(struct ath_hal * ah)
	IMPLEMENTATION({ 
		return ath_hal_getcapability(ah, HAL_CAP_XR, 0, NULL) == HAL_OK;;
	})

#endif /* _IF_ATH_HAL_H */
