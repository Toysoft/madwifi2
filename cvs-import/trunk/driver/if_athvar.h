/*-
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 * $Id$
 */

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_ATHVAR_H
#define _DEV_ATH_ATHVAR_H

#include "if_ieee80211.h"
#include "ah.h"
#include "if_athioctl.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,41)
#include <linux/workqueue.h>
#define tq_struct work_struct
#define INIT_TQUEUE INIT_WORK
#define queue_task(a,b) schedule_work(a)
#define mark_bh(a)
#elif !defined(IRQ_NONE)
typedef void irqreturn_t;
#define	IRQ_NONE
#define	IRQ_HANDLED
#endif

#define	ATH_TIMEOUT		1000

/*
 * Maximum acceptable MTU
 * MAXFRAMEBODY - WEP - QOS - RSN/WPA:
 * 2312 - 8 - 2 - 12 = 2290
 */
#define ATH_MAX_MTU     2290
#define ATH_MIN_MTU     32  

#define	ATH_TPC_MAX	63		/* 6 bits */

#define	ATH_RXBUF	40		/* number of RX buffers */
#define	ATH_TXBUF	200		/* number of TX buffers */
#define	ATH_TXDESC	1		/* number of descriptors per buffer */
#define	ATH_TXMAXTRY	11		/* max number of transmit attempts */

/* driver-specific node state */
struct ath_node {
	struct ieee80211_node an_node;	/* base class */
	u_int		an_tx_ok;	/* tx ok pkt */
	u_int		an_tx_err;	/* tx !ok pkt */
	u_int		an_tx_retr;	/* tx retry count */
	int		an_tx_upper;	/* tx upper rate req cnt */
	u_int		an_tx_antenna;	/* antenna for last good frame */
	u_int8_t	an_tx_rix0;	/* series 0 rate index */
	u_int8_t	an_tx_try0;	/* series 0 try count */
	u_int8_t	an_tx_mgtrate;	/* h/w rate for management/ctl frames */
	u_int8_t	an_tx_mgtratesp;/* short preamble h/w rate for " " */
	u_int8_t	an_tx_rate0;	/* series 0 h/w rate */
	u_int8_t	an_tx_rate1;	/* series 1 h/w rate */
	u_int8_t	an_tx_rate2;	/* series 2 h/w rate */
	u_int8_t	an_tx_rate3;	/* series 3 h/w rate */
	u_int8_t	an_tx_rate0sp;	/* series 0 short preamble h/w rate */
	u_int8_t	an_tx_rate1sp;	/* series 1 short preamble h/w rate */
	u_int8_t	an_tx_rate2sp;	/* series 2 short preamble h/w rate */
	u_int8_t	an_tx_rate3sp;	/* series 3 short preamble h/w rate */
};
#define	ATH_NODE(_n)	((struct ath_node *)(_n))

struct ath_buf {
	TAILQ_ENTRY(ath_buf)	bf_list;
	struct ath_desc		*bf_desc;	/* virtual addr of desc */
	dma_addr_t		bf_daddr;	/* physical addr of desc */
	struct sk_buff		*bf_skb;	/* skbuff for buf */
	dma_addr_t		bf_skbaddr;	/* physical addr of skb data */
	struct ieee80211_node	*bf_node;	/* pointer to the node */
};

struct ath_hal;
struct ath_desc;
struct proc_dir_entry;

struct ath_softc {
	struct ieee80211com	sc_ic;		/* IEEE 802.11 common */
	struct ath_hal		*sc_ah;		/* Atheros HAL */
	unsigned int		sc_invalid : 1,	/* being detached */
				sc_mrretry : 1;	/* multi-rate retry support */
						/* rate tables */
	const HAL_RATE_TABLE	*sc_rates[IEEE80211_MODE_MAX];
	const HAL_RATE_TABLE	*sc_currates;	/* current rate table */
	enum ieee80211_phymode	sc_curmode;	/* current phy mode */
	u_int8_t		sc_rixmap[256];	/* IEEE to h/w rate table ix */
	u_int8_t		sc_hwmap[32];	/* h/w rate ix to IEEE table */
	HAL_INT			sc_imask;	/* interrupt mask copy */

        void                    *sc_bdev;       /* bus device (will be cast by 
                                                 * bus specific code) 
                                                 */
	struct ath_desc		*sc_desc;	/* TX/RX descriptors */
	size_t			sc_desc_len;	/* size of TX/RX descriptors */
	u_int16_t		sc_cachelsz;	/* cache line size */
	dma_addr_t		sc_desc_daddr;	/* DMA (physical) address */

	struct tq_struct	sc_fataltq;	/* fatal error intr tasklet */

	int			sc_rxbufsize;	/* rx size based on mtu */
	TAILQ_HEAD(, ath_buf)	sc_rxbuf;	/* receive buffer */
	u_int32_t		*sc_rxlink;	/* link ptr in last RX desc */
	struct tq_struct	sc_rxtq;	/* rx intr tasklet */
	struct tq_struct	sc_rxorntq;	/* rxorn intr tasklet */

        int sc_AC2qNum[4];                      /* HAL q nums for each AC value */
	u_int32_t		*sc_txlink[HAL_NUM_TX_QUEUES];	/* link ptr in last TX desc */
	TAILQ_HEAD(, ath_buf)	sc_txbuf;	/* tx buffer queue */
	spinlock_t		sc_txbuflock;	/* txbuf lock */
        TAILQ_HEAD(, ath_buf)	sc_txq[HAL_NUM_TX_QUEUES];		/* transmit queue */
        spinlock_t		sc_txqlock[HAL_NUM_TX_QUEUES];	/* lock on txq and txlink */
	struct tq_struct	sc_txtq;	/* tx intr tasklet */

	u_int			sc_bhalq;	/* HAL q for outgoing beacons */
	struct ath_buf		*sc_bcbuf;	/* beacon buffer */
	struct ath_buf		*sc_bufptr;	/* allocated buffer ptr */
	struct tq_struct	sc_bmisstq;	/* bmiss intr tasklet */

	struct timer_list	sc_rate_ctl;	/* tx rate control timer */
	struct timer_list	sc_cal_ch;	/* calibration timer */
	struct timer_list	sc_scan_ch;	/* AP scan timer */
	struct ath_stats	sc_stats;	/* interface statistics */
};

#ifdef AR_DEBUG
extern	int ath_debug;
#define	DPRINTF(X)	if (ath_debug) printk X
#define	DPRINTF2(X)	if (ath_debug > 1) printk X
#else
#define	DPRINTF(X)
#define	DPRINTF2(X)
#endif

int	ath_attach(u_int16_t, struct net_device *);
int	ath_detach(struct net_device *);
void	ath_resume(struct net_device *);
void	ath_suspend(struct net_device *);
void	ath_shutdown(struct net_device *);
irqreturn_t ath_intr(int irq, void *dev_id, struct pt_regs *regs);
#ifdef CONFIG_SYSCTL
void	ath_sysctl_register(void);
void	ath_sysctl_unregister(void);
#endif /* CONFIG_SYSCTL */

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
#define	ath_hal_setpower(_ah, _mode, _sleepduration) \
	((*(_ah)->ah_setPowerMode)((_ah), (_mode), AH_TRUE, (_sleepduration)))
#define	ath_hal_keyreset(_ah, _ix) \
	((*(_ah)->ah_resetKeyCacheEntry)((_ah), (_ix)))
#define	ath_hal_keyset(_ah, _ix, _pk) \
	((*(_ah)->ah_setKeyCacheEntry)((_ah), (_ix), (_pk), NULL, AH_FALSE))
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
#define	ath_hal_calibrate(_ah, _chan) \
	((*(_ah)->ah_perCalibration)((_ah), (_chan)))
#define	ath_hal_setledstate(_ah, _state) \
	((*(_ah)->ah_setLedState)((_ah), (_state)))
#define	ath_hal_beaconinit(_ah, _nextb, _bperiod) \
	((*(_ah)->ah_beaconInit)((_ah), (_nextb), (_bperiod)))
#define	ath_hal_beaconreset(_ah) \
	((*(_ah)->ah_resetStationBeaconTimers)((_ah)))
#define	ath_hal_beacontimers(_ah, _bs, _tsf, _dc, _cc) \
	((*(_ah)->ah_setStationBeaconTimers)((_ah), (_bs), (_tsf), \
		(_dc), (_cc)))
#define	ath_hal_setassocid(_ah, _bss, _associd) \
	((*(_ah)->ah_writeAssocid)((_ah), (_bss), (_associd), 0))
#define	ath_hal_getcapability(_ah, _cap, _param, _result) \
	((*(_ah)->ah_getCapability)((_ah), (_cap), (_param), (_result)))
#define	ath_hal_getregdomain(_ah, _prd) \
	ath_hal_getcapability(_ah, HAL_CAP_REG_DMN, 0, (_prd))
#define	ath_hal_getcountrycode(_ah, _pcc) \
	(*(_pcc) = (_ah)->ah_countryCode)

#ifdef SOFTLED
#define ath_hal_gpioCfgOutput(_ah, _gpio) \
        ((*(_ah)->ah_gpioCfgOutput)((_ah), (_gpio)))
#define ath_hal_gpioCfgInput(_ah, _gpio) \
        ((*(_ah)->ah_gpioCfgInput)((_ah), (_gpio)))
#define ath_hal_gpioGet(_ah, _gpio) \
        ((*(_ah)->ah_gpioGet)((_ah), (_gpio)))
#define ath_hal_gpioSet(_ah, _gpio, _b) \
        ((*(_ah)->ah_gpioSet)((_ah), (_gpio), (_b)))
#define ath_hal_gpioSetIntr(_ah, _gpioSel, _b) \
        ((*(_ah)->ah_gpioSetIntr)((_ah), (_sel), (_b)))
#endif

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
#define	ath_hal_getregdomain(_ah, _prd) \
	ath_hal_getcapability(_ah, HAL_CAP_REG_DMN, 0, (_prd))
#define	ath_hal_getcountrycode(_ah, _pcc) \
	(*(_pcc) = (_ah)->ah_countryCode)

#define	ath_hal_setuptxqueue(_ah, _type, _qinfo) \
	((*(_ah)->ah_setupTxQueue)((_ah), (_type), (_qinfo)))
#define	ath_hal_resettxqueue(_ah, _q) \
	((*(_ah)->ah_resetTxQueue)((_ah), (_q)))
#define	ath_hal_releasetxqueue(_ah, _q) \
	((*(_ah)->ah_releaseTxQueue)((_ah), (_q)))
#define	ath_hal_hasveol(_ah) \
	((*(_ah)->ah_hasVEOL)((_ah)))
#define	ath_hal_getrfgain(_ah) \
	((*(_ah)->ah_getRfGain)((_ah)))
#define	ath_hal_rxmonitor(_ah) \
	((*(_ah)->ah_rxMonitor)((_ah)))

#define	ath_hal_setuprxdesc(_ah, _ds, _size, _intreq) \
	((*(_ah)->ah_setupRxDesc)((_ah), (_ds), (_size), (_intreq)))
#define	ath_hal_rxprocdesc(_ah, _ds, _dspa, _dsnext) \
	((*(_ah)->ah_procRxDesc)((_ah), (_ds), (_dspa), (_dsnext)))
#define	ath_hal_setuptxdesc(_ah, _ds, _plen, _hlen, _atype, _txpow, \
		_txr0, _txtr0, _keyix, _ant, _flags, \
		_rtsrate, _rtsdura) \
	((*(_ah)->ah_setupTxDesc)((_ah), (_ds), (_plen), (_hlen), (_atype), \
		(_txpow), (_txr0), (_txtr0), (_keyix), (_ant), \
		(_flags), (_rtsrate), (_rtsdura)))
#define	ath_hal_setupxtxdesc(_ah, _ds, \
		_txr1, _txtr1, _txr2, _txtr2, _txr3, _txtr3) \
	((*(_ah)->ah_setupXTxDesc)((_ah), (_ds), \
		(_txr1), (_txtr1), (_txr2), (_txtr2), (_txr3), (_txtr3)))
#define	ath_hal_filltxdesc(_ah, _ds, _l, _first, _last) \
	((*(_ah)->ah_fillTxDesc)((_ah), (_ds), (_l), (_first), (_last)))
#define	ath_hal_txprocdesc(_ah, _ds) \
	((*(_ah)->ah_procTxDesc)((_ah), (_ds)))

#endif /* _DEV_ATH_ATHVAR_H */
