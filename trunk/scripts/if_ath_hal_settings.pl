#!/usr/bin/perl
#
#
# This file contains settings used to generate the HAL wrapper API for MadWifi from the
# binary HAL api header.  You can edit these settings to customize the behavior of the
# code generator.
#

#
# This section contains the output file paths
#
$path_to_hal  = 'hal';
$path_to_ath  = 'ath';
$hal_h        = 'ah.h';
$if_ath_hal_c = 'if_ath_hal.c';
$if_ath_hal_h = 'if_ath_hal.h';

#
# This section defines the name translation from the binary HAL's function
# pointers to our API names.
#
%hal_function_name_to_madwifi_name = (
	"ah_beaconInit"               => "ath_hal_beaconinit",
	"ah_disablePhyErrDiag"        => "ath_hal_disablePhyDiag",
	"ah_enablePhyErrDiag"         => "ath_hal_enablePhyDiag",
	"ah_enableReceive"            => "ath_hal_rxena",
	"ah_fillTxDesc"               => "ath_hal_filltxdesc",
	"ah_getAckCTSRate"            => "ath_hal_getackctsrate",
	"ah_setAckCTSRate"            => "ath_hal_setackctsrate",
	"ah_updateMibCounters"        => "ath_hal_updatemibcounters",
	"ah_getAntennaSwitch"         => "ath_hal_getantennaswitch",
	"ah_setAntennaSwitch"         => "ath_hal_setantennaswitch",
	"ah_getAckTimeout"            => "ath_hal_getacktimeout",
	"ah_getBssIdMask"             => "ath_hal_getbssidmask",
	"ah_getCapability"            => "ath_hal_getcapability",
	"ah_getChanNoise"             => "ath_hal_get_channel_noise",
	"ah_getCTSTimeout"            => "ath_hal_getctstimeout",
	"ah_getDefAntenna"            => "ath_hal_getdefantenna",
	"ah_getDiagState"             => "ath_hal_getdiagstate",
	"ah_getInterrupts"            => "ath_hal_intrget",
	"ah_getKeyCacheSize"          => "ath_hal_keycachesize",
	"ah_getMacAddress"            => "ath_hal_getmac",
	"ah_getPendingInterrupts"     => "ath_hal_getisr",
	"ah_getPowerMode"             => "ath_hal_getPowerMode",
	"ah_getRateTable"             => "ath_hal_getratetable",
	"ah_getRfGain"                => "ath_hal_getrfgain",
	"ah_getRxDP"                  => "ath_hal_getrxbuf",
	"ah_getRxFilter"              => "ath_hal_getrxfilter",
	"ah_getSlotTime"              => "ath_hal_getslottime",
	"ah_getTsf32"                 => "ath_hal_gettsf32",
	"ah_getTsf64"                 => "ath_hal_gettsf64",
	"ah_getTxDP"                  => "ath_hal_gettxbuf",
	"ah_getTxIntrQueue"           => "ath_hal_gettxintrtxqs",
	"ah_getTxQueueProps"          => "ath_hal_gettxqueueprops",
	"ah_gpioCfgOutput"            => "ath_hal_gpioCfgOutput",
	"ah_gpioSet"                  => "ath_hal_gpioset",
	"ah_gpioGet"                  => "ath_hal_gpioget",
	"ah_gpioSetIntr"              => "ath_hal_gpiosetintr",
	"ah_gpioCfgInput"             => "ath_hal_gpiCfgInput",
	"ah_isInterruptPending"       => "ath_hal_intrpend",
	"ah_isKeyCacheEntryValid"     => "ath_hal_keyisvalid",
	"ah_numTxPending"             => "ath_hal_numtxpending",
	"ah_perCalibration"           => "ath_hal_calibrate",
	"ah_phyDisable"               => "ath_hal_phydisable",
	"ah_disable"                  => "ath_hal_disable",
	"ah_procMibEvent"             => "ath_hal_mibevent",
	"ah_procRxDesc"               => "ath_hal_rxprocdesc",
	"ah_procTxDesc"               => "ath_hal_txprocdesc",
	"ah_radarWait"                => "ath_hal_radar_wait",
	"ah_releaseTxQueue"           => "ath_hal_releasetxqueue",
	"ah_reqTxIntrDesc"            => "ath_hal_txreqintrdesc",
	"ah_reset"                    => "ath_hal_reset",
	"ah_resetKeyCacheEntry"       => "ath_hal_keyreset",
	"ah_resetStationBeaconTimers" => "ath_hal_beaconreset",
	"ah_resetTsf"                 => "ath_hal_resettsf",
	"ah_resetTxQueue"             => "ath_hal_resettxqueue",
	"ah_rxMonitor"                => "ath_hal_rxmonitor",
	"ah_setAckTimeout"            => "ath_hal_setacktimeout",
	"ah_setBssIdMask"             => "ath_hal_setbssidmask",
	"ah_setCapability"            => "ath_hal_setcapability",
	"ah_setChannel"               => "ath_hal_setchannel",
	"ah_setCoverageClass"         => "ath_hal_setcoverageclass",
	"ah_setCTSTimeout"            => "ath_hal_setctstimeout",
	"ah_setDecompMask"            => "ath_hal_setdecompmask",
	"ah_setDefAntenna"            => "ath_hal_setdefantenna",
	"ah_setInterrupts"            => "ath_hal_intrset",
	"ah_setKeyCacheEntry"         => "ath_hal_keyset",
	"ah_setKeyCacheEntryMac"      => "ath_hal_keysetmac",
	"ah_setLedState"              => "ath_hal_setledstate",
	"ah_setMacAddress"            => "ath_hal_setmac",
	"ah_setMulticastFilter"       => "ath_hal_setmcastfilter",
	"ah_setMulticastFilterIndex"  => "ath_hal_setmcastfilterindex",
	"ah_setPCUConfig"             => "ath_hal_setopmode",
	"ah_setPowerMode"             => "ath_hal_setpower",
	"ah_setRxDP"                  => "ath_hal_putrxbuf",
	"ah_setRxFilter"              => "ath_hal_setrxfilter",
	"ah_setRegulatoryDomain"      => "ath_hal_setregulatorydomain",
	"ah_setSlotTime"              => "ath_hal_setslottime",
	"ah_setStationBeaconTimers"   => "ath_hal_beacontimers",
	"ah_setTxDP"                  => "ath_hal_puttxbuf",
	"ah_setTxQueueProps"          => "ath_hal_settxqueueprops",
	"ah_setTxPowerLimit"          => "ath_hal_settxpowlimit",
	"ah_setBeaconTimers"          => "ath_hal_setbeacontimers",
	"ah_setupRxDesc"              => "ath_hal_setuprxdesc",
	"ah_setupTxDesc"              => "ath_hal_setuptxdesc",
	"ah_setupTxQueue"             => "ath_hal_setuptxqueue",
	"ah_setupXTxDesc"             => "ath_hal_setupxtxdesc",
	"ah_startPcuReceive"          => "ath_hal_startpcurecv",
	"ah_startTxDma"               => "ath_hal_txstart",
	"ah_stopDmaReceive"           => "ath_hal_stopdmarecv",
	"ah_stopPcuReceive"           => "ath_hal_stoppcurecv",
	"ah_stopTxDma"                => "ath_hal_stoptxdma",
	"ah_updateCTSForBursting"     => "ath_hal_updateCTSForBursting",
	"ah_updateTxTrigLevel"        => "ath_hal_updatetxtriglevel",
	"ah_waitForBeaconDone"        => "ath_hal_waitforbeacon",
	"ah_writeAssocid"             => "ath_hal_setassocid",
	"ah_clrMulticastFilterIndex"  => "ath_hal_clearmcastfilter",
	"ah_detectCardPresent"        => "ath_hal_detectcardpresent" );
#
# List any functions that should NOT be generated here (such as those that conflict with
# other functions, perhaps.
#
@hal_functions_not_to_wrap = ( "ah_detach" );
#
# Boilerplate text
#
$header_comment = <<EOF
/* Wrapper macros/functions for the binary HAL to comply with local coding
 * convention.  Provides function-style calling convention using either macros
 * or wrapper functions for function pointers in the HAL.
 *
 * The typical convention is ath_hal_foo(ah,p1,p2,p3,...) turns into
 * ah->ah_foo(p1,p2,p3,...) where ah_foo is a function pointer and a member
 * of the struct ath_hal (usually named ah). */
EOF
;
# This section defines additional HAL APIs that we we want to expose.  It
# is structured the way that it is so that we can parse the declaration portion
# and easily turn it into function headers, inlines in headers,
# implementations in .c files, etc..
%custom_wrappers = (
	"HAL_BOOL ath_hal_burstsupported(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_BURST, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_ciphersupported(struct ath_hal * ah, u_int32_t cipher)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_CIPHER, cipher, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_compressionsupported(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_fastframesupported(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_getcountrycode(struct ath_hal * ah, u_int32_t* destination)" =>
		"((*(destination) = ah->ah_countryCode), AH_TRUE)",
	"HAL_BOOL ath_hal_getdiversity(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 1, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_getmaxtxpow(struct ath_hal * ah, u_int32_t* destination)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TXPOW, 2, destination) == HAL_OK)",
	"HAL_BOOL ath_hal_getmcastkeysearch(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_MCAST_KEYSRCH, 1, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_getnumtxqueues(struct ath_hal * ah, u_int32_t* destination)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_NUM_TXQUEUES, 0, destination) == HAL_OK)",
	"HAL_BOOL ath_hal_getregdomain(struct ath_hal * ah, u_int32_t* destination)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_REG_DMN, 0, destination))",
	"HAL_BOOL ath_hal_gettkipmic(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TKIP_MIC, 1, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_gettkipsplit(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TKIP_SPLIT, 1, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_gettpc(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TPC, 1, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_gettpscale(struct ath_hal * ah, u_int32_t* destination)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TXPOW, 3, destination) == HAL_OK)",
	"HAL_BOOL ath_hal_gettsfadjust(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TSF_ADJUST, 1, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_gettxpowlimit(struct ath_hal * ah, u_int32_t* destination)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TXPOW, 1, destination) == HAL_OK)",
	"HAL_BOOL ath_hal_halfrate_chansupported(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_CHAN_HALFRATE, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasbssidmask(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_BSSIDMASK, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasbursting(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_BURST, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hascompression(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_COMPRESSION, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasdiversity(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasfastframes(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_FASTFRAME, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasmcastkeysearch(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_MCAST_KEYSRCH, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasrfsilent(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_RFSILENT, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hastkipmic(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TKIP_MIC, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hastkipsplit(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TKIP_SPLIT, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hastpc(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TPC, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hastsfadjust(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TSF_ADJUST, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hastxpowlimit(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_TXPOW, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hasveol(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_VEOL, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_hwphycounters(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_PHYCOUNTERS, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_quarterrate_chansupported(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_CHAN_QUARTERRATE, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_setdiversity(struct ath_hal * ah, int v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 1, v, NULL))",
	"HAL_BOOL ath_hal_setrfsilent(struct ath_hal * ah, u_int32_t v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_RFSILENT, 1, v, NULL))",
	"HAL_BOOL ath_hal_settkipmic(struct ath_hal * ah, u_int32_t v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_TKIP_MIC, 1, v, NULL))",
	"HAL_BOOL ath_hal_settkipsplit(struct ath_hal * ah, int v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_TKIP_SPLIT, 1, v, NULL))",
	"HAL_BOOL ath_hal_settpc(struct ath_hal * ah, u_int32_t v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_TPC, 1, v, NULL))",
	"HAL_BOOL ath_hal_settpscale(struct ath_hal * ah, u_int32_t v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_TXPOW, 3, v, NULL))",
	"HAL_BOOL ath_hal_settsfadjust(struct ath_hal * ah, u_int32_t v)" =>
		"(ath_hal_setcapability(ah, HAL_CAP_TSF_ADJUST, 1, v, NULL))",
	"HAL_BOOL ath_hal_turboagsupported(struct ath_hal * ah, int countrycode)" =>
		"(ath_hal_getwirelessmodes(ah, countrycode) & (HAL_MODE_108G|HAL_MODE_TURBO))",
	"HAL_BOOL ath_hal_wmetkipmic(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_WME_TKIPMIC, 0, NULL) == HAL_OK)",
	"HAL_BOOL ath_hal_xrsupported(struct ath_hal * ah)" =>
		"(ath_hal_getcapability(ah, HAL_CAP_XR, 0, NULL) == HAL_OK)"
	);


#
# This text is generated verbatim at the top of the .h file,
# before generated content is added.
#
$header_for_h = <<EOF
$header_comment

#ifndef _IF_ATH_HAL_H
#define _IF_ATH_HAL_H

#define GET_ATH_SOFTC(_ah) 	((struct ath_softc*)(_ah->ah_sc))
#define ATH_HAL_LOCK_INIT(_sc) 	spin_lock_init(&(_sc)->sc_hal_lock)
#define ATH_HAL_LOCK_DESTROY(_sc)
#define ATH_HAL_LOCK_IRQ(_sc) 	do { \\
   unsigned long __sc_halLockflags; \\
   spin_lock_irqsave(&(_sc)->sc_hal_lock, __sc_halLockflags);
#define ATH_HAL_UNLOCK_IRQ(_sc) \\
   spin_unlock_irqrestore(&(_sc)->sc_hal_lock, __sc_halLockflags); \\
   } while(0)
#define ATH_HAL_UNLOCK_IRQ_EARLY(_sc) \\
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
EOF
;

#
# This text is generated verbatim at the bottom of the .h file
#
$footer_for_h = <<EOF

#endif /* _IF_ATH_HAL_H */
EOF
;
#
# This text is generated verbatim at the top of the .c file
#
$header_for_c = <<EOF
$header_comment
#include "opt_ah.h"

#ifdef ATH_HALOPS_TRACEABLE

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
EOF
;
#
# This text is generated verbatim at the bottom of the .c file
#
$footer_for_c = <<EOF
#endif /* #ifdef ATH_HALOPS_TRACEABLE */
EOF
;

