/*
 * Copyright (c) 2002, 2003 Sam Leffler.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Sam Leffler ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL John Hay BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_ATHVAR_H
#define _DEV_ATH_ATHVAR_H

#include "if_ieee80211.h"
#include "ah.h"

#define	ATH_TIMEOUT		1000

#define	ATH_RXBUF	40		/* number of RX buffers */
#define	ATH_TXBUF	40		/* number of TX buffers */
#define	ATH_TXDESC	1		/* number of descriptors per buffer */

/* statistics for node */
struct ath_nodestat {
	u_int		st_tx_ok;	/* tx ok pkt */
	u_int		st_tx_err;	/* tx ok pkt */
	u_int		st_tx_retr;	/* tx retry count */
	int		st_tx_upper;	/* tx upper rate req cnt */
};

struct ath_stats {
	u_int32_t	ast_watchdog;	/* device reset by watchdog */
	u_int32_t	ast_hardware;	/* fatal hardware error interrupt */
	u_int32_t	ast_bmiss;	/* beacon miss interrupt */
	u_int32_t	ast_rxorn;	/* rxorn interrupts */
	u_int32_t	ast_rxeol;	/* rxeol interrupts */
	u_int32_t	ast_tx_mgmt;	/* management frames transmitted */
	u_int32_t	ast_tx_discard;	/* frames discarded prior to assoc */
	u_int32_t	ast_tx_invalid;	/* frames discarded 'cuz device gone */
	u_int32_t	ast_tx_qstop;	/* tx queue stopped 'cuz full */
	u_int32_t	ast_tx_encap;	/* tx encapsulation failed */
	u_int32_t	ast_tx_nonode;	/* tx failed 'cuz no node */
	u_int32_t	ast_tx_nobuf;	/* tx failed 'cuz no tx buffer (data) */
	u_int32_t	ast_tx_nobufmgt;/* tx failed 'cuz no tx buffer (mgmt)*/
	u_int32_t	ast_tx_descerr;	/* tx failure reported in desc*/
	u_int32_t	ast_rx_orn;	/* rx failed 'cuz of desc overrun */
	u_int32_t	ast_rx_tooshort;/* rx failed 'cuz frame too short */
	u_int32_t	ast_rx_crcerr;	/* rx failed 'cuz of bad CRC */
	u_int32_t	ast_rx_fifoerr;	/* rx failed 'cuz of FIFO overrun */
	u_int32_t	ast_rx_badcrypt;/* rx failed 'cuz decryption */
	u_int32_t	ast_rx_phyerr;	/* rx failed 'cuz of PHY err */
	u_int32_t	ast_rx_phy_tim;	/* rx PHY: timing error */
	u_int32_t	ast_rx_phy_par;	/* rx PHY: parity error */
	u_int32_t	ast_rx_phy_rate;/* rx PHY: illegal rate */
	u_int32_t	ast_rx_phy_len;	/* rx PHY: illegal length */
	u_int32_t	ast_rx_phy_qam;	/* rx PHY: 64 QAM rate */
	u_int32_t	ast_rx_phy_srv;	/* rx PHY: service bit error */
	u_int32_t	ast_rx_phy_tor;	/* rx PHY: transmit override receive */
	u_int32_t	ast_rx_nobuf;	/* rx setup failed 'cuz no skbuff */
	u_int32_t	ast_beacon_nobuf;/* no skbuff available for beacon */
};

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
	struct pci_dev		*sc_pdev;	/* associated pci device */
	volatile int		sc_invalid;	/* being detached */
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

	u_int32_t		*sc_txlink;	/* link ptr in last TX desc */
	TAILQ_HEAD(, ath_buf)	sc_txbuf;	/* tx buffer queue */
	spinlock_t		sc_txbuflock;	/* txbuf lock */
	TAILQ_HEAD(, ath_buf)	sc_txq;		/* transmit queue */
	spinlock_t		sc_txqlock;	/* lock on txq and txlink */
	struct tq_struct	sc_txtq;	/* tx intr tasklet */

	struct ath_buf		*sc_bcbuf;	/* beacon buffer */
	struct ath_buf		*sc_bufptr;	/* allocated buffer ptr */
	struct tq_struct	sc_swbatq;	/* swba intr tasklet */
	struct tq_struct	sc_bmisstq;	/* bmiss intr tasklet */

	struct timer_list	sc_cal_ch;	/* calibration timer */
	struct timer_list	sc_scan_ch;	/* AP scan timer */
	struct ath_nodestat	sc_bss_stat;	/* statistics for infra mode */
	struct ath_stats	sc_stats;	/* interface statistics */
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry	*sc_proc;	/* /proc/net/drivers/ath%d */
#endif
};

#define	ATH_BITVAL(val, name)	(((val) & name) >> name##_S)

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
void	ath_intr(int irq, void *dev_id, struct pt_regs *regs);

/*
 * HAL definitions to comply with local coding convention.
 */
#define	ath_hal_reset(_ah, _opmode, _chan, _outdoor, _pstatus) \
	((*(_ah)->ah_reset)((_ah), (_opmode), (_chan), (_outdoor), (_pstatus)))
#define	ath_hal_getmac(_ah, _mac) \
	((*(_ah)->ah_getMacAddress)((_ah), (_mac)))
#define	ath_hal_detach(_ah) \
	((*(_ah)->ah_detach)((_ah)))
#define	ath_hal_disable(_ah) \
	((*(_ah)->ah_disable)((_ah)))
#define	ath_hal_intrset(_ah, _mask) \
	((*(_ah)->ah_setInterrupts)((_ah), (_mask)))
#define	ath_hal_intrget(_ah) \
	((*(_ah)->ah_getInterrupts)((_ah)))
#define	ath_hal_intrpend(_ah) \
	((*(_ah)->ah_isInterruptPending)((_ah)))
#define	ath_hal_getisr(_ah, _pmask) \
	((*(_ah)->ah_getPendingInterrupts)((_ah), (_pmask)))
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
#define	ath_hal_setrxfilter(_ah, _filter) \
	((*(_ah)->ah_setRxFilter)((_ah), (_filter)))
#define	ath_hal_setmcastfilter(_ah, _mfilt0, _mfilt1) \
	((*(_ah)->ah_setMulticastFilter)((_ah), (_mfilt0), (_mfilt1)))
#define	ath_hal_waitforbeacon(_ah, _bf) \
	((*(_ah)->ah_waitForBeaconDone)((_ah), (_bf)->bf_daddr))
#define	ath_hal_qbeacon(_ah, _mode, _bf) \
	((*(_ah)->ah_queueBeaconAndCab)((_ah), (_mode), (_bf)->bf_daddr, 0))
#define	ath_hal_putrxbuf(_ah, _bufaddr) \
	((*(_ah)->ah_setRxDP)((_ah), (_bufaddr)))
#define	ath_hal_gettsf(_ah) \
	((*(_ah)->ah_getTsf)((_ah)))
#define	ath_hal_rxena(_ah) \
	((*(_ah)->ah_enableReceive)((_ah)))
#define	ath_hal_puttxbuf(_ah, _bufaddr) \
	((*(_ah)->ah_setTxDP)((_ah), 0, (_bufaddr)))
#define	ath_hal_gettxbuf(_ah) \
	((*(_ah)->ah_getTxDP)((_ah), 0))
#define	ath_hal_getrxbuf(_ah) \
	((*(_ah)->ah_getRxDP)((_ah)))
#define	ath_hal_txstart(_ah) \
	((*(_ah)->ah_startTxDma)((_ah), HAL_TX_QUEUE_DATA))
#define	ath_hal_setchannel(_ah, _chan) \
	((*(_ah)->ah_setChannel)((_ah), (_chan)))
#define	ath_hal_calibrate(_ah, _chan) \
	((*(_ah)->ah_perCalibration)((_ah), (_chan)))
#define	ath_hal_setledstate(_ah, _state) \
	((*(_ah)->ah_setLedState)((_ah), (_state)))
#define	ath_hal_beaconinit(_ah, _opmode, _nextb, _bperiod, _bf) \
	((*(_ah)->ah_beaconInit)((_ah), (_opmode), (_nextb), (_bperiod), \
		(_bf) ? (_bf)->bf_daddr : 0))
#define	ath_hal_beaconreset(_ah) \
	((*(_ah)->ah_resetStationBeaconTimers)((_ah)))
#define	ath_hal_setassocid(_ah, _bss, _associd) \
	((*(_ah)->ah_writeAssocid)((_ah), (_bss), (_associd), 0))
#define	ath_hal_setopmode(_ah, _opmode) \
	((*(_ah)->ah_setPCUConfig)((_ah), (_opmode)))
#define	ath_hal_stoptxdma(_ah, _qnum) \
	((*(_ah)->ah_stopTxDma)((_ah), (_qnum)))
#define	ath_hal_stoppcurecv(_ah) \
	((*(_ah)->ah_stopPcuReceive)((_ah)))
#define	ath_hal_startpcurecv(_ah) \
	((*(_ah)->ah_startPcuReceive)((_ah)))
#define	ath_hal_stopdmarecv(_ah) \
	((*(_ah)->ah_stopDmaReceive)((_ah)))
#define	ath_hal_dumpstate(_ah) \
	((*(_ah)->ah_dumpState)((_ah)))

#define	ath_hal_settxdeschdrlen(_ah, _ds, _l) \
	((*(_ah)->ah_setTxDescHdrLen)((_ah), (_ds), (_l)))
#define	ath_hal_settxdescpkttype(_ah, _ds, _t) \
	((*(_ah)->ah_setTxDescPktType)((_ah), (_ds), (_t)))
#define	ath_hal_settxdesckey(_ah, _ds, _kix) \
	((*(_ah)->ah_setTxDescEncryptKeyIndex)((_ah), (_ds), (_kix)))

#endif /* _DEV_ATH_ATHVAR_H */
