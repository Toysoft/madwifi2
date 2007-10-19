/*
 * Copyright (c) 2004-2007 Reyk Floeter <reyk@openbsd.org>
 * Copyright (c) 2006-2007 Nick Kossifidis <mickflemm@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _ATH5K_H
#define _ATH5K_H

/* Set this to 1 to disable regulatory domain restrictions for channel tests.
 * WARNING: This is for debuging only and has side effects (eg. scan takes too
 * long and results timeouts). It's also illegal to tune to some of the
 * supported frequencies in some countries, so use this at your own risk,
 * you've been warned. */
#define CHAN_DEBUG	0

/* Uncomment this for debuging (warning that it results in TOO much output) */
/*#define AR5K_DEBUG	1 */

#include <linux/io.h>
#include <linux/types.h>
#include <net/mac80211.h>

#include "hw.h"
#include "regdom.h"

/* PCI IDs */
#define PCI_DEVICE_ID_ATHEROS_AR5210 		0x0007 /* AR5210 */
#define PCI_DEVICE_ID_ATHEROS_AR5311 		0x0011 /* AR5311 */
#define PCI_DEVICE_ID_ATHEROS_AR5211 		0x0012 /* AR5211 */
#define PCI_DEVICE_ID_ATHEROS_AR5212 		0x0013 /* AR5212 */
#define PCI_DEVICE_ID_3COM_3CRDAG675 		0x0013 /* 3CRDAG675 (Atheros AR5212) */
#define PCI_DEVICE_ID_3COM_2_3CRPAG175 		0x0013 /* 3CRPAG175 (Atheros AR5212) */
#define PCI_DEVICE_ID_ATHEROS_AR5210_AP 	0x0207 /* AR5210 (Early) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_IBM	0x1014 /* AR5212 (IBM MiniPCI) */
#define PCI_DEVICE_ID_ATHEROS_AR5210_DEFAULT 	0x1107 /* AR5210 (no eeprom) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_DEFAULT 	0x1113 /* AR5212 (no eeprom) */
#define PCI_DEVICE_ID_ATHEROS_AR5211_DEFAULT 	0x1112 /* AR5211 (no eeprom) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_FPGA 	0xf013 /* AR5212 (emulation board) */
#define PCI_DEVICE_ID_ATHEROS_AR5211_LEGACY 	0xff12 /* AR5211 (emulation board) */
#define PCI_DEVICE_ID_ATHEROS_AR5211_FPGA11B 	0xf11b /* AR5211 (emulation board) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_REV2 	0x0052 /* AR5312 WMAC (AP31) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_REV7 	0x0057 /* AR5312 WMAC (AP30-040) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_REV8 	0x0058 /* AR5312 WMAC (AP43-030) */
#define PCI_DEVICE_ID_ATHEROS_AR5212_0014 	0x0014 /* AR5212 compatible */
#define PCI_DEVICE_ID_ATHEROS_AR5212_0015 	0x0015 /* AR5212 compatible */
#define PCI_DEVICE_ID_ATHEROS_AR5212_0016 	0x0016 /* AR5212 compatible */
#define PCI_DEVICE_ID_ATHEROS_AR5212_0017 	0x0017 /* AR5212 compatible */
#define PCI_DEVICE_ID_ATHEROS_AR5212_0018 	0x0018 /* AR5212 compatible */
#define PCI_DEVICE_ID_ATHEROS_AR5212_0019 	0x0019 /* AR5212 compatible */
#define PCI_DEVICE_ID_ATHEROS_AR2413 		0x001a /* AR2413 (Griffin-lite) */
#define PCI_DEVICE_ID_ATHEROS_AR5413 		0x001b /* AR5413 (Eagle) */
#define PCI_DEVICE_ID_ATHEROS_AR5424 		0x001c /* AR5424 (Condor PCI-E) */

/****************************\
  GENERIC DRIVER DEFINITIONS
\****************************/

#define AR5K_PRINTF(fmt, ...)   printk("%s: " fmt, __func__, ##__VA_ARGS__)
#define AR5K_PRINT(fmt)         printk("%s: " fmt, __func__)
#ifdef AR5K_DEBUG
#define AR5K_TRACE              printk(KERN_DEBUG "%s:%d\n", __func__, __LINE__)
#else
#define AR5K_TRACE
#endif

/*
 * Some tuneable values (these should be changeable by the user)
 */
#define AR5K_TUNE_DMA_BEACON_RESP		2
#define AR5K_TUNE_SW_BEACON_RESP		10
#define AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF	0
#define AR5K_TUNE_RADAR_ALERT			false
#define AR5K_TUNE_MIN_TX_FIFO_THRES		1
#define AR5K_TUNE_MAX_TX_FIFO_THRES		((IEEE80211_MAX_LEN / 64) + 1)
#define AR5K_TUNE_RSSI_THRES			1792
#define AR5K_TUNE_REGISTER_TIMEOUT		20000
#define AR5K_TUNE_REGISTER_DWELL_TIME		20000
#define AR5K_TUNE_BEACON_INTERVAL		100
#define AR5K_TUNE_AIFS				2
#define AR5K_TUNE_AIFS_11B			2
#define AR5K_TUNE_AIFS_XR			0
#define AR5K_TUNE_CWMIN				15
#define AR5K_TUNE_CWMIN_11B			31
#define AR5K_TUNE_CWMIN_XR			3
#define AR5K_TUNE_CWMAX				1023
#define AR5K_TUNE_CWMAX_11B			1023
#define AR5K_TUNE_CWMAX_XR			7
#define AR5K_TUNE_NOISE_FLOOR			-72
#define AR5K_TUNE_MAX_TXPOWER			60
#define AR5K_TUNE_DEFAULT_TXPOWER		30
#define AR5K_TUNE_TPC_TXPOWER			true
#define AR5K_TUNE_ANT_DIVERSITY			true
#define AR5K_TUNE_HWTXTRIES			4

/* token to use for aifs, cwmin, cwmax in MadWiFi */
#define	AR5K_TXQ_USEDEFAULT	((u32) -1)

/* GENERIC CHIPSET DEFINITIONS */

/* MAC Chips */
enum ath5k_version {
	AR5K_AR5210	= 0,
	AR5K_AR5211	= 1,
	AR5K_AR5212	= 2,
};

/* PHY Chips */
enum ath5k_radio {
	AR5K_RF5110	= 0,
	AR5K_RF5111	= 1,
	AR5K_RF5112	= 2,
};

/*
 * Common silicon revision/version values
 */
#define AR5K_SREV_UNKNOWN	0xffff

#define AR5K_SREV_VER_AR5210	0x00
#define AR5K_SREV_VER_AR5311	0x10
#define AR5K_SREV_VER_AR5311A	0x20
#define AR5K_SREV_VER_AR5311B	0x30
#define AR5K_SREV_VER_AR5211	0x40
#define AR5K_SREV_VER_AR5212	0x50
#define AR5K_SREV_VER_AR5213	0x55
#define AR5K_SREV_VER_UNSUPP	0x60

#define AR5K_SREV_RAD_5110	0x00
#define AR5K_SREV_RAD_5111	0x10
#define AR5K_SREV_RAD_5111A	0x15
#define AR5K_SREV_RAD_2111	0x20
#define AR5K_SREV_RAD_5112	0x30
#define AR5K_SREV_RAD_5112A	0x35
#define AR5K_SREV_RAD_2112	0x40
#define AR5K_SREV_RAD_2112A	0x45
#define AR5K_SREV_RAD_UNSUPP	0x50

/* IEEE defs */

#define IEEE80211_MAX_LEN       2500

/* TODO add support to mac80211 for vendor-specific rates and modes */

/*
 * Some of this information is based on Documentation from:
 *
 * http://madwifi.org/wiki/ChipsetFeatures/SuperAG
 *
 * Modulation for Atheros' eXtended Range - range enhancing extension that is
 * supposed to double the distance an Atheros client device can keep a
 * connection with an Atheros access point. This is achieved by increasing
 * the receiver sensitivity up to, -105dBm, which is about 20dB above what
 * the 802.11 specifications demand. In addition, new (proprietary) data rates
 * are introduced: 3, 2, 1, 0.5 and 0.25 MBit/s.
 *
 * Please note that can you either use XR or TURBO but you cannot use both,
 * they are exclusive.
 *
 */
#define MODULATION_XR 		0x00000200
/*
 * Modulation for Atheros' Turbo G and Turbo A, its supposed to provide a
 * throughput transmission speed up to 40Mbit/s-60Mbit/s at a 108Mbit/s
 * signaling rate achieved through the bonding of two 54Mbit/s 802.11g
 * channels. To use this feature your Access Point must also suport it.
 * There is also a distinction between "static" and "dynamic" turbo modes:
 *
 * - Static: is the dumb version: devices set to this mode stick to it until
 *     the mode is turned off.
 * - Dynamic: is the intelligent version, the network decides itself if it
 *     is ok to use turbo. As soon as traffic is detected on adjacent channels
 *     (which would get used in turbo mode), or when a non-turbo station joins
 *     the network, turbo mode won't be used until the situation changes again.
 *     Dynamic mode is achieved by Atheros' Adaptive Radio (AR) feature which
 *     monitors the used radio band in order to decide whether turbo mode may
 *     be used or not.
 *
 * This article claims Super G sticks to bonding of channels 5 and 6 for
 * USA:
 *
 * http://www.pcworld.com/article/id,113428-page,1/article.html
 *
 * The channel bonding seems to be driver specific though. In addition to
 * deciding what channels will be used, these "Turbo" modes are accomplished
 * by also enabling the following features:
 *
 * - Bursting: allows multiple frames to be sent at once, rather than pausing
 *     after each frame. Bursting is a standards-compliant feature that can be
 *     used with any Access Point.
 * - Fast frames: increases the amount of information that can be sent per
 *     frame, also resulting in a reduction of transmission overhead. It is a
 *     proprietary feature that needs to be supported by the Access Point.
 * - Compression: data frames are compressed in real time using a Lempel Ziv
 *     algorithm. This is done transparently. Once this feature is enabled,
 *     compression and decompression takes place inside the chipset, without
 *     putting additional load on the host CPU.
 */
#define MODULATION_TURBO	0x00000080

enum ath5k_vendor_mode {
	MODE_ATHEROS_TURBO = NUM_IEEE80211_MODES+1,
	MODE_ATHEROS_TURBOG
};

/* Number of supported mac80211 enum ieee80211_phymode modes by this driver */
#define NUM_DRIVER_MODES	3

/* adding this flag to rate_code enables short preamble, see ar5212_reg.h */
#define AR5K_SET_SHORT_PREAMBLE 0x04

#define HAS_SHPREAMBLE(_ix) (rt->rates[_ix].modulation == IEEE80211_RATE_CCK_2)
#define SHPREAMBLE_FLAG(_ix) (HAS_SHPREAMBLE(_ix) ? AR5K_SET_SHORT_PREAMBLE : 0)

/****************\
  TX DEFINITIONS
\****************/

/*
 * Tx Descriptor
 */
struct ath_tx_status {
	u16	ts_seqnum;
	u16	ts_tstamp;
	u8	ts_status;
	u8	ts_rate;
	s8	ts_rssi;
	u8	ts_shortretry;
	u8	ts_longretry;
	u8	ts_virtcol;
	u8	ts_antenna;
};

#define AR5K_TXSTAT_ALTRATE	0x80
#define AR5K_TXERR_XRETRY	0x01
#define AR5K_TXERR_FILT		0x02
#define AR5K_TXERR_FIFO		0x04

/*
 * Queue types used to classify tx queues.
 */
enum ath5k_tx_queue {
	AR5K_TX_QUEUE_INACTIVE = 0, /* q is unused -- see ath5k_hw_release_tx_queue */
	AR5K_TX_QUEUE_DATA,	  /*A normal data queue*/
	AR5K_TX_QUEUE_XR_DATA,	  /*An XR-data queue*/
	AR5K_TX_QUEUE_BEACON,	  /*The beacon queue*/
	AR5K_TX_QUEUE_CAB,	  /*The ater-beacon queue*/
	AR5K_TX_QUEUE_UAPSD,	  /*Unscheduled Automatic Power Save Delivery queue*/
};

#define	AR5K_NUM_TX_QUEUES		10
#define	AR5K_NUM_TX_QUEUES_NOQCU	2

/*
 * Queue syb-types to classify normal data queues.
 * These are the 4 Access Categories as defined in
 * WME spec. 0 is the lowest priority and 4 is the
 * highest. Normal data that hasn't been classified
 * goes to the Best Effort AC.
 */
enum ath5k_tx_queue_subtype {
	AR5K_WME_AC_BK = 0,	/*Background traffic*/
	AR5K_WME_AC_BE, 	/*Best-effort (normal) traffic)*/
	AR5K_WME_AC_VI, 	/*Video traffic*/
	AR5K_WME_AC_VO, 	/*Voice traffic*/
};

/*
 * Queue ID numbers as returned by the HAL, each number
 * represents a hw queue. If hw does not support hw queues
 * (eg 5210) all data goes in one queue. These match
 * d80211 definitions (net80211/MadWiFi don't use them).
 */
enum ath5k_tx_queue_id {
	AR5K_TX_QUEUE_ID_NOQCU_DATA	= 0,
	AR5K_TX_QUEUE_ID_NOQCU_BEACON	= 1,
	AR5K_TX_QUEUE_ID_DATA_MIN	= 0, /*IEEE80211_TX_QUEUE_DATA0*/
	AR5K_TX_QUEUE_ID_DATA_MAX	= 4, /*IEEE80211_TX_QUEUE_DATA4*/
	AR5K_TX_QUEUE_ID_DATA_SVP	= 5, /*IEEE80211_TX_QUEUE_SVP - Spectralink Voice Protocol*/
	AR5K_TX_QUEUE_ID_CAB		= 6, /*IEEE80211_TX_QUEUE_AFTER_BEACON*/
	AR5K_TX_QUEUE_ID_BEACON		= 7, /*IEEE80211_TX_QUEUE_BEACON*/
	AR5K_TX_QUEUE_ID_UAPSD		= 8,
	AR5K_TX_QUEUE_ID_XR_DATA	= 9,
};


/*
 * Flags to set hw queue's parameters...
 */
#define AR5K_TXQ_FLAG_TXINT_ENABLE		0x0001	/* Enable TXOK and TXERR interrupts -not used- */
#define AR5K_TXQ_FLAG_TXDESCINT_ENABLE		0x0002	/* Enable TXDESC interrupt -not implemented- */
#define AR5K_TXQ_FLAG_BACKOFF_DISABLE		0x0004	/* Disable random post-backoff */
#define AR5K_TXQ_FLAG_COMPRESSION_ENABLE	0x0008	/* Enable hw compression -not implemented-*/
#define AR5K_TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE	0x0010	/* Enable ready time expiry policy (?)*/
#define AR5K_TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE	0x0020	/* Enable backoff while bursting */
#define AR5K_TXQ_FLAG_POST_FR_BKOFF_DIS		0x0040	/* Disable backoff while bursting */
#define AR5K_TXQ_FLAG_TXEOLINT_ENABLE		0x0080	/* Enable TXEOL interrupt -not implemented-*/

/*
 * A struct to hold tx queue's parameters
 */
struct ath5k_txq_info {
	enum ath5k_tx_queue tqi_type;
	enum ath5k_tx_queue_subtype tqi_subtype;
	u16	tqi_flags;	/* Tx queue flags (see above) */
	u32	tqi_aifs;	/* Arbitrated Interframe Space */
	s32	tqi_cw_min;	/* Minimum Contention Window */
	s32	tqi_cw_max;	/* Maximum Contention Window */
	u32	tqi_cbr_period; /* Constant bit rate period */
	u32	tqi_cbr_overflow_limit;
	u32	tqi_burst_time;
	u32	tqi_ready_time; /* Not used */
	u32	tqi_comp_buffer;/* Compression Buffer's phys addr */
};

/*
 * Transmit packet types.
 * These are not fully used inside OpenHAL yet
 */
enum ath5k_pkt_type {
	AR5K_PKT_TYPE_NORMAL		= 0,
	AR5K_PKT_TYPE_ATIM		= 1,
	AR5K_PKT_TYPE_PSPOLL		= 2,
	AR5K_PKT_TYPE_BEACON		= 3,
	AR5K_PKT_TYPE_PROBE_RESP	= 4,
	AR5K_PKT_TYPE_PIFS		= 5,
};

/*
 * TX power and TPC settings
 */
#define AR5K_TXPOWER_OFDM(_r, _v)	(			\
	((0 & 1) << ((_v) + 6)) |				\
	(((hal->ah_txpower.txp_rates[(_r)]) & 0x3f) << (_v))	\
)

#define AR5K_TXPOWER_CCK(_r, _v)	(			\
	(hal->ah_txpower.txp_rates[(_r)] & 0x3f) << (_v)	\
)

/*
 * Used to compute TX times
 */
#define AR5K_CCK_SIFS_TIME		10
#define AR5K_CCK_PREAMBLE_BITS		144
#define AR5K_CCK_PLCP_BITS		48

#define AR5K_OFDM_SIFS_TIME		16
#define AR5K_OFDM_PREAMBLE_TIME		20
#define AR5K_OFDM_PLCP_BITS		22
#define AR5K_OFDM_SYMBOL_TIME		4

#define AR5K_TURBO_SIFS_TIME		8
#define AR5K_TURBO_PREAMBLE_TIME	14
#define AR5K_TURBO_PLCP_BITS		22
#define AR5K_TURBO_SYMBOL_TIME		4

#define AR5K_XR_SIFS_TIME		16
#define AR5K_XR_PLCP_BITS		22
#define AR5K_XR_SYMBOL_TIME		4

/* CCK */
#define AR5K_CCK_NUM_BITS(_frmlen) (_frmlen << 3)

#define AR5K_CCK_PHY_TIME(_sp) (_sp ?					\
	((AR5K_CCK_PREAMBLE_BITS + AR5K_CCK_PLCP_BITS) >> 1) :		\
	(AR5K_CCK_PREAMBLE_BITS + AR5K_CCK_PLCP_BITS))

#define AR5K_CCK_TX_TIME(_kbps, _frmlen, _sp)				\
	(AR5K_CCK_PHY_TIME(_sp) +					\
	((AR5K_CCK_NUM_BITS(_frmlen) * 1000) / _kbps) +			\
	AR5K_CCK_SIFS_TIME)

/* OFDM */
#define AR5K_OFDM_NUM_BITS(_frmlen) (AR5K_OFDM_PLCP_BITS + (_frmlen << 3))

#define AR5K_OFDM_NUM_BITS_PER_SYM(_kbps) ((_kbps *			\
	AR5K_OFDM_SYMBOL_TIME) / 1000)

#define AR5K_OFDM_NUM_BITS(_frmlen) (AR5K_OFDM_PLCP_BITS + (_frmlen << 3))

#define AR5K_OFDM_NUM_SYMBOLS(_kbps, _frmlen)				\
	DIV_ROUND_UP(AR5K_OFDM_NUM_BITS(_frmlen),			\
			AR5K_OFDM_NUM_BITS_PER_SYM(_kbps))

#define AR5K_OFDM_TX_TIME(_kbps, _frmlen)				\
	(AR5K_OFDM_PREAMBLE_TIME + AR5K_OFDM_SIFS_TIME +		\
	(AR5K_OFDM_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_OFDM_SYMBOL_TIME))

/* TURBO */
#define AR5K_TURBO_NUM_BITS(_frmlen) (AR5K_TURBO_PLCP_BITS + (_frmlen << 3))

#define AR5K_TURBO_NUM_BITS_PER_SYM(_kbps) (((_kbps << 1) *		\
	AR5K_TURBO_SYMBOL_TIME) / 1000)

#define AR5K_TURBO_NUM_BITS(_frmlen) (AR5K_TURBO_PLCP_BITS + (_frmlen << 3))

#define AR5K_TURBO_NUM_SYMBOLS(_kbps, _frmlen)				\
	DIV_ROUND_UP(AR5K_TURBO_NUM_BITS(_frmlen),			\
			AR5K_TURBO_NUM_BITS_PER_SYM(_kbps))

#define AR5K_TURBO_TX_TIME(_kbps, _frmlen)				\
	(AR5K_TURBO_PREAMBLE_TIME + AR5K_TURBO_SIFS_TIME +		\
	(AR5K_TURBO_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_TURBO_SYMBOL_TIME))

/* eXtendent Range (?)*/
#define AR5K_XR_PREAMBLE_TIME(_kbps) (((_kbps) < 1000) ? 173 : 76)

#define AR5K_XR_NUM_BITS_PER_SYM(_kbps) ((_kbps *			\
	AR5K_XR_SYMBOL_TIME) / 1000)

#define AR5K_XR_NUM_BITS(_frmlen) (AR5K_XR_PLCP_BITS + (_frmlen << 3))

#define AR5K_XR_NUM_SYMBOLS(_kbps, _frmlen)				\
	DIV_ROUND_UP(AR5K_XR_NUM_BITS(_frmlen), AR5K_XR_NUM_BITS_PER_SYM(_kbps))

#define AR5K_XR_TX_TIME(_kbps, _frmlen)					\
	(AR5K_XR_PREAMBLE_TIME(_kbps) + AR5K_XR_SIFS_TIME +		\
	(AR5K_XR_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_XR_SYMBOL_TIME))

/*
 * DMA size definitions (2^n+2)
 */
enum ath5k_dmasize {
	AR5K_DMASIZE_4B	= 0,
	AR5K_DMASIZE_8B,
	AR5K_DMASIZE_16B,
	AR5K_DMASIZE_32B,
	AR5K_DMASIZE_64B,
	AR5K_DMASIZE_128B,
	AR5K_DMASIZE_256B,
	AR5K_DMASIZE_512B
};


/****************\
  RX DEFINITIONS
\****************/

/*
 * Rx Descriptor
 */
struct ath_rx_status {
	u16	rs_datalen;
	u16	rs_tstamp;
	u8	rs_status;
	u8	rs_phyerr;
	s8	rs_rssi;
	u8	rs_keyix;
	u8	rs_rate;
	u8	rs_antenna;
	u8	rs_more;
};

#define AR5K_RXERR_CRC		0x01
#define AR5K_RXERR_PHY		0x02
#define AR5K_RXERR_FIFO		0x04
#define AR5K_RXERR_DECRYPT	0x08
#define AR5K_RXERR_MIC		0x10
#define AR5K_RXKEYIX_INVALID	((u8) - 1)
#define AR5K_TXKEYIX_INVALID	((u32) - 1)

struct ath5k_mib_stats {
	u32	ackrcv_bad;
	u32	rts_bad;
	u32	rts_good;
	u32	fcs_bad;
	u32	beacons;
};




/**************************\
 BEACON TIMERS DEFINITIONS
\**************************/

#define AR5K_BEACON_PERIOD	0x0000ffff
#define AR5K_BEACON_ENA		0x00800000 /*enable beacon xmit*/
#define AR5K_BEACON_RESET_TSF	0x01000000 /*force a TSF reset*/

/*
 * Per-station beacon timer state.
 */
struct ath5k_beacon_state {
	u32	bs_next_beacon;
	u32	bs_next_dtim;
	u32	bs_interval;		/*in TU's -see net80211/ieee80211_var.h-
						can also include the above flags*/
	u8	bs_dtim_period;
	u8	bs_cfp_period;
	u16	bs_cfp_max_duration;	/*if non-zero hw is setup to coexist with
						a Point Coordination Function capable AP*/
	u16	bs_cfp_du_remain;
	u16	bs_tim_offset;
	u16	bs_sleep_duration;
	u16	bs_bmiss_threshold;
	u32  	bs_cfp_next;
};




/********************\
  COMMON DEFINITIONS
\********************/

/*
 * Atheros descriptor
 */
struct ath_desc {
	u32	ds_link;
	u32	ds_data;
	u32	ds_ctl0;
	u32	ds_ctl1;
	u32	ds_hw[4];

	union {
		struct ath_rx_status rx;
		struct ath_tx_status tx;
	} ds_us;

#define ds_rxstat ds_us.rx
#define ds_txstat ds_us.tx

} __packed;

#define AR5K_RXDESC_INTREQ	0x0020

#define AR5K_TXDESC_CLRDMASK	0x0001
#define AR5K_TXDESC_NOACK	0x0002	/*[5211+]*/
#define AR5K_TXDESC_RTSENA	0x0004
#define AR5K_TXDESC_CTSENA	0x0008
#define AR5K_TXDESC_INTREQ	0x0010
#define AR5K_TXDESC_VEOL	0x0020	/*[5211+]*/

#define AR5K_SLOT_TIME_9	396
#define AR5K_SLOT_TIME_20	880
#define AR5K_SLOT_TIME_MAX	0xffff

/* channel_flags */
#define	CHANNEL_CW_INT	0x0008	/* Contention Window interference detected */
#define	CHANNEL_TURBO	0x0010	/* Turbo Channel */
#define	CHANNEL_CCK	0x0020	/* CCK channel */
#define	CHANNEL_OFDM	0x0040	/* OFDM channel */
#define	CHANNEL_2GHZ	0x0080	/* 2GHz channel. */
#define	CHANNEL_5GHZ	0x0100	/* 5GHz channel */
#define	CHANNEL_PASSIVE	0x0200	/* Only passive scan allowed */
#define	CHANNEL_DYN	0x0400	/* Dynamic CCK-OFDM channel (for g operation) */
#define	CHANNEL_XR	0x0800	/* XR channel */

#define	CHANNEL_A	(CHANNEL_5GHZ|CHANNEL_OFDM)
#define	CHANNEL_B	(CHANNEL_2GHZ|CHANNEL_CCK)
#define	CHANNEL_G	(CHANNEL_2GHZ|CHANNEL_OFDM)
#define	CHANNEL_T	(CHANNEL_5GHZ|CHANNEL_OFDM|CHANNEL_TURBO)
#define	CHANNEL_TG	(CHANNEL_2GHZ|CHANNEL_OFDM|CHANNEL_TURBO)
#define	CHANNEL_108A	CHANNEL_T
#define	CHANNEL_108G	CHANNEL_TG
#define	CHANNEL_X	(CHANNEL_5GHZ|CHANNEL_OFDM|CHANNEL_XR)

#define	CHANNEL_ALL 	(CHANNEL_OFDM|CHANNEL_CCK|CHANNEL_2GHZ|CHANNEL_5GHZ| \
		CHANNEL_TURBO)

#define	CHANNEL_ALL_NOTURBO 	(CHANNEL_ALL & ~CHANNEL_TURBO)
#define CHANNEL_MODES		CHANNEL_ALL

/*
 * Used internaly in OpenHAL (ar5211.c/ar5212.c
 * for reset_tx_queue). Also see struct struct ieee80211_channel.
 */
#define IS_CHAN_XR(_c)	((_c.val & CHANNEL_XR) != 0)
#define IS_CHAN_B(_c)	((_c.val & CHANNEL_B) != 0)

/*
 * The following structure will be used to map 2GHz channels to
 * 5GHz Atheros channels.
 */
struct ath5k_athchan_2ghz {
	u32	a2_flags;
	u16	a2_athchan;
};

/*
 * Rate definitions
 */

#define AR5K_MAX_RATES	32 /*max number of rates on the rate table*/

struct ath5k_rate {
	u8	valid;		/* Valid for rate control */
	u32	modulation;
	u16	rate_kbps;
	u8	rate_code;	/* Rate mapping for h/w descriptors */
	u8	dot11_rate;
	u8	control_rate;
	u16	lp_ack_duration;/* long preamble ACK duration */
	u16	sp_ack_duration;/* short preamble ACK duration*/
};

struct ath5k_rate_table {
	u16	rate_count;
	u8	rate_code_to_index[AR5K_MAX_RATES];	/* Back-mapping */
	struct ath5k_rate rates[AR5K_MAX_RATES];
};

/*
 * Rate tables...
 */
#define AR5K_RATES_11A { 8, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 6, 4, 2, 0,	\
	7, 5, 3, 1, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255 }, {		\
	{ 1, IEEE80211_RATE_OFDM, 6000, 11, 140, 0 },		\
	{ 1, IEEE80211_RATE_OFDM, 9000, 15, 18, 0 },		\
	{ 1, IEEE80211_RATE_OFDM, 12000, 10, 152, 2 },		\
	{ 1, IEEE80211_RATE_OFDM, 18000, 14, 36, 2 },		\
	{ 1, IEEE80211_RATE_OFDM, 24000, 9, 176, 4 },		\
	{ 1, IEEE80211_RATE_OFDM, 36000, 13, 72, 4 },		\
	{ 1, IEEE80211_RATE_OFDM, 48000, 8, 96, 4 },		\
	{ 1, IEEE80211_RATE_OFDM, 54000, 12, 108, 4 } }		\
}

#define AR5K_RATES_11B { 4, {						\
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,	\
	3, 2, 1, 0, 255, 255, 255, 255 }, {				\
	{ 1, IEEE80211_RATE_CCK, 1000, 27, 130, 0 },	\
	{ 1, IEEE80211_RATE_CCK_2, 2000, 26, 132, 1 },	\
	{ 1, IEEE80211_RATE_CCK_2, 5500, 25, 139, 1 },	\
	{ 1, IEEE80211_RATE_CCK_2, 11000, 24, 150, 1 } }	\
}

#define AR5K_RATES_11G { 12, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 10, 8, 6, 4,	\
	11, 9, 7, 5, 255, 255, 255, 255, 255, 255, 255, 255,	\
	3, 2, 1, 0, 255, 255, 255, 255 }, {			\
	{ 1, IEEE80211_RATE_CCK, 1000, 27, 2, 0 },		\
	{ 1, IEEE80211_RATE_CCK_2, 2000, 26, 4, 1 },		\
	{ 1, IEEE80211_RATE_CCK_2, 5500, 25, 11, 1 },		\
	{ 1, IEEE80211_RATE_CCK_2, 11000, 24, 22, 1 },	\
	{ 0, IEEE80211_RATE_OFDM, 6000, 11, 12, 4 },	\
	{ 0, IEEE80211_RATE_OFDM, 9000, 15, 18, 4 },	\
	{ 1, IEEE80211_RATE_OFDM, 12000, 10, 24, 6 },	\
	{ 1, IEEE80211_RATE_OFDM, 18000, 14, 36, 6 },	\
	{ 1, IEEE80211_RATE_OFDM, 24000, 9, 48, 8 },	\
	{ 1, IEEE80211_RATE_OFDM, 36000, 13, 72, 8 },	\
	{ 1, IEEE80211_RATE_OFDM, 48000, 8, 96, 8 },	\
	{ 1, IEEE80211_RATE_OFDM, 54000, 12, 108, 8 } }	\
}

#define AR5K_RATES_TURBO { 8, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 6, 4, 2, 0,	\
	7, 5, 3, 1, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255 }, {		\
	{ 1, MODULATION_TURBO, 6000, 11, 140, 0 },	\
	{ 1, MODULATION_TURBO, 9000, 15, 18, 0 },	\
	{ 1, MODULATION_TURBO, 12000, 10, 152, 2 },	\
	{ 1, MODULATION_TURBO, 18000, 14, 36, 2 },	\
	{ 1, MODULATION_TURBO, 24000, 9, 176, 4 },	\
	{ 1, MODULATION_TURBO, 36000, 13, 72, 4 },	\
	{ 1, MODULATION_TURBO, 48000, 8, 96, 4 },	\
	{ 1, MODULATION_TURBO, 54000, 12, 108, 4 } }	\
}

#define AR5K_RATES_XR { 12, {					\
	255, 3, 1, 255, 255, 255, 2, 0, 10, 8, 6, 4,		\
	11, 9, 7, 5, 255, 255, 255, 255, 255, 255, 255, 255,	\
	255, 255, 255, 255, 255, 255, 255, 255 }, {		\
	{ 1, MODULATION_XR, 500, 7, 129, 0 },		\
	{ 1, MODULATION_XR, 1000, 2, 139, 1 },		\
	{ 1, MODULATION_XR, 2000, 6, 150, 2 },		\
	{ 1, MODULATION_XR, 3000, 1, 150, 3 },		\
	{ 1, IEEE80211_RATE_OFDM, 6000, 11, 140, 4 },	\
	{ 1, IEEE80211_RATE_OFDM, 9000, 15, 18, 4 },	\
	{ 1, IEEE80211_RATE_OFDM, 12000, 10, 152, 6 },	\
	{ 1, IEEE80211_RATE_OFDM, 18000, 14, 36, 6 },	\
	{ 1, IEEE80211_RATE_OFDM, 24000, 9, 176, 8 },	\
	{ 1, IEEE80211_RATE_OFDM, 36000, 13, 72, 8 },	\
	{ 1, IEEE80211_RATE_OFDM, 48000, 8, 96, 8 },	\
	{ 1, IEEE80211_RATE_OFDM, 54000, 12, 108, 8 } }	\
}

/*
 * Crypto definitions
 */

#define AR5K_KEYCACHE_SIZE	8

/***********************\
 HW RELATED DEFINITIONS
\***********************/

/*
 * Misc definitions
 */
#define	AR5K_RSSI_EP_MULTIPLIER	(1<<7)

#define AR5K_ASSERT_ENTRY(_e, _s) do {		\
	if (_e >= _s)				\
		return (false);			\
} while (0)


enum ath5k_ant_setting {
	AR5K_ANT_VARIABLE	= 0,	/* variable by programming */
	AR5K_ANT_FIXED_A	= 1,	/* fixed to 11a frequencies */
	AR5K_ANT_FIXED_B	= 2,	/* fixed to 11b frequencies */
	AR5K_ANT_MAX		= 3,
};

/*
 * HAL interrupt abstraction
 */

/**
 * enum ath5k_int - Hardware interrupt masks helpers
 *
 * @AR5K_INT_RX: mask to identify received frame interrupts, of type
 * 	AR5K_ISR_RXOK or AR5K_ISR_RXERR
 * @AR5K_INT_RXDESC: Request RX descriptor/Read RX descriptor (?)
 * @AR5K_INT_RXNOFRM: No frame received (?)
 * @AR5K_INT_RXEOL: received End Of List for VEOL (Virtual End Of List). The
 * 	Queue Control Unit (QCU) signals an EOL interrupt only if a descriptor's
 * 	LinkPtr is NULL. For more details, refer to:
 * 	http://www.freepatentsonline.com/20030225739.html
 * @AR5K_INT_RXORN: indicates a hardware reset is required on certain hardware.
 * 	Note that Rx overrun is not always fatal, on some chips we can continue
 * 	operation without reseting the card, that's why int_fatal is not
 * 	common for all chips.
 * @AR5K_INT_TX: mask to identify received frame interrupts, of type
 * 	AR5K_ISR_TXOK or AR5K_ISR_TXERR
 * @AR5K_INT_TXDESC: Request TX descriptor/Read TX status descriptor (?)
 * @AR5K_INT_TXURN: received when we should increase the TX trigger threshold
 * 	We currently do increments on interrupt by
 * 	(AR5K_TUNE_MAX_TX_FIFO_THRES - current_trigger_level) / 2
 * @AR5K_INT_MIB: Indicates the Management Information Base counters should be
 * 	checked. We should do this with ath5k_hw_update_mib_counters() but
 * 	it seems we should also then do some noise immunity work.
 * @AR5K_INT_RXPHY: RX PHY Error
 * @AR5K_INT_RXKCM: ??
 * @AR5K_INT_SWBA: SoftWare Beacon Alert - indicates its time to send a
 * 	beacon that must be handled in software. The alternative is if you
 * 	have VEOL support, in that case you let the hardware deal with things.
 * @AR5K_INT_BMISS: If in STA mode this indicates we have stopped seeing
 * 	beacons from the AP have associated with, we should probably try to
 * 	reassociate. When in IBSS mode this might mean we have not received
 * 	any beacons from any local stations. Note that every station in an
 * 	IBSS schedules to send beacons at the Target Beacon Transmission Time
 * 	(TBTT) with a random backoff.
 * @AR5K_INT_BNR: Beacon Not Ready interrupt - ??
 * @AR5K_INT_GPIO: GPIO interrupt is used for RF Kill, disabled for now
 * 	until properly handled
 * @AR5K_INT_FATAL: Fatal errors were encountered, typically caused by DMA
 * 	errors. These types of errors we can enable seem to be of type
 * 	AR5K_SIMR2_MCABT, AR5K_SIMR2_SSERR and AR5K_SIMR2_DPERR.
 * @AR5K_INT_GLOBAL: Seems to be used to clear and set the IER
 * @AR5K_INT_NOCARD: signals the card has been removed
 * @AR5K_INT_COMMON: common interrupts shared amogst MACs with the same
 * 	bit value
 *
 * These are mapped to take advantage of some common bits
 * between the MACs, to be able to set intr properties
 * easier. Some of them are not used yet inside hw.c. Most map
 * to the respective hw interrupt value as they are common amogst different
 * MACs.
 */
enum ath5k_int {
	AR5K_INT_RX	= 0x00000001, /* Not common */
	AR5K_INT_RXDESC	= 0x00000002,
	AR5K_INT_RXNOFRM = 0x00000008,
	AR5K_INT_RXEOL	= 0x00000010,
	AR5K_INT_RXORN	= 0x00000020,
	AR5K_INT_TX	= 0x00000040, /* Not common */
	AR5K_INT_TXDESC	= 0x00000080,
	AR5K_INT_TXURN	= 0x00000800,
	AR5K_INT_MIB	= 0x00001000,
	AR5K_INT_RXPHY	= 0x00004000,
	AR5K_INT_RXKCM	= 0x00008000,
	AR5K_INT_SWBA	= 0x00010000,
	AR5K_INT_BMISS	= 0x00040000,
	AR5K_INT_BNR	= 0x00100000, /* Not common */
	AR5K_INT_GPIO	= 0x01000000,
	AR5K_INT_FATAL	= 0x40000000, /* Not common */
	AR5K_INT_GLOBAL	= 0x80000000,

	AR5K_INT_COMMON  = AR5K_INT_RXNOFRM
			| AR5K_INT_RXDESC
			| AR5K_INT_RXEOL
			| AR5K_INT_RXORN
			| AR5K_INT_TXURN
			| AR5K_INT_TXDESC
			| AR5K_INT_MIB
			| AR5K_INT_RXPHY
			| AR5K_INT_RXKCM
			| AR5K_INT_SWBA
			| AR5K_INT_BMISS
			| AR5K_INT_GPIO,
	AR5K_INT_NOCARD	= 0xffffffff
};

/*
 * Power management
 */
enum ath5k_power_mode {
	AR5K_PM_UNDEFINED = 0,
	AR5K_PM_AUTO,
	AR5K_PM_AWAKE,
	AR5K_PM_FULL_SLEEP,
	AR5K_PM_NETWORK_SLEEP,
};

/*
 * These match net80211 definitions (not used in
 * d80211).
 */
#define AR5K_LED_INIT	0 /*IEEE80211_S_INIT*/
#define AR5K_LED_SCAN	1 /*IEEE80211_S_SCAN*/
#define AR5K_LED_AUTH	2 /*IEEE80211_S_AUTH*/
#define AR5K_LED_ASSOC	3 /*IEEE80211_S_ASSOC*/
#define AR5K_LED_RUN	4 /*IEEE80211_S_RUN*/

/* GPIO-controlled software LED */
#define AR5K_SOFTLED_PIN	0
#define AR5K_SOFTLED_ON		0
#define AR5K_SOFTLED_OFF	1

/*
 * Chipset capabilities -see ath_hal_getcapability-
 * get_capability function is not yet fully implemented
 * in OpenHAL so most of these don't work yet...
 */
enum ath5k_capability_type {
	AR5K_CAP_REG_DMN		= 0,	/* Used to get current reg. domain id */
	AR5K_CAP_TKIP_MIC		= 2,	/* Can handle TKIP MIC in hardware */
	AR5K_CAP_TKIP_SPLIT		= 3,	/* TKIP uses split keys */
	AR5K_CAP_PHYCOUNTERS		= 4,	/* PHY error counters */
	AR5K_CAP_DIVERSITY		= 5,	/* Supports fast diversity */
	AR5K_CAP_NUM_TXQUEUES		= 6,	/* Used to get max number of hw txqueues */
	AR5K_CAP_VEOL			= 7,	/* Supports virtual EOL */
	AR5K_CAP_COMPRESSION		= 8,	/* Supports compression */
	AR5K_CAP_BURST			= 9,	/* Supports packet bursting */
	AR5K_CAP_FASTFRAME		= 10,	/* Supports fast frames */
	AR5K_CAP_TXPOW			= 11,	/* Used to get global tx power limit */
	AR5K_CAP_TPC			= 12,	/* Can do per-packet tx power control (needed for 802.11a) */
	AR5K_CAP_BSSIDMASK		= 13,	/* Supports bssid mask */
	AR5K_CAP_MCAST_KEYSRCH		= 14,	/* Supports multicast key search */
	AR5K_CAP_TSF_ADJUST		= 15,	/* Supports beacon tsf adjust */
	AR5K_CAP_XR			= 16,	/* Supports XR mode */
	AR5K_CAP_WME_TKIPMIC 		= 17,	/* Supports TKIP MIC when using WMM */
	AR5K_CAP_CHAN_HALFRATE 		= 18,	/* Supports half rate channels */
	AR5K_CAP_CHAN_QUARTERRATE 	= 19,	/* Supports quarter rate channels */
	AR5K_CAP_RFSILENT		= 20,	/* Supports RFsilent */
};

struct ath5k_capabilities {
	/*
	 * Supported PHY modes
	 * (ie. CHANNEL_A, CHANNEL_B, ...)
	 */
	DECLARE_BITMAP(cap_mode, NUM_DRIVER_MODES);

	/*
	 * Frequency range (without regulation restrictions)
	 */
	struct {
		u16	range_2ghz_min;
		u16	range_2ghz_max;
		u16	range_5ghz_min;
		u16	range_5ghz_max;
	} cap_range;

	/*
	 * Active regulation domain settings
	 */
	struct {
		enum ath5k_regdom reg_current;
		enum ath5k_regdom reg_hw;
	} cap_regdomain;

	/*
	 * Values stored in the EEPROM (some of them...)
	 */
	struct ath5k_eeprom_info	cap_eeprom;

	/*
	 * Queue information
	 */
	struct {
		u8	q_tx_num;
	} cap_queues;
};


/***************************************\
  HARDWARE ABSTRACTION LAYER STRUCTURE
\***************************************/

/*
 * Misc defines
 */

#define AR5K_MAX_GPIO		10
#define AR5K_MAX_RF_BANKS	8

struct ath_hw {
	u32			ah_magic;

	void			*ah_sc;
	void __iomem		*ah_sh;

	enum ath5k_int		ah_imr;

	enum ieee80211_if_types	ah_op_mode;
	enum ath5k_power_mode	ah_power_mode;
	struct ieee80211_channel ah_current_channel;
	/* Current BSSID we are trying to assoc to / creating, this
	 * comes from ieee80211_if_conf. This is passed by mac80211 on
	 * config_interface() */
	u8			bssid[ETH_ALEN];
	bool			ah_turbo;
	bool			ah_calibration;
	bool			ah_running;
	bool			ah_single_chip;
	enum ath5k_rfgain	ah_rf_gain;

	u32			ah_mac_srev;
	u16			ah_mac_version;
	u16			ah_mac_revision;
	u16			ah_phy_revision;
	u16			ah_radio_5ghz_revision;
	u16			ah_radio_2ghz_revision;

	enum ath5k_version	ah_version;
	enum ath5k_radio	ah_radio;
	u32			ah_phy;

	bool			ah_5ghz;
	bool			ah_2ghz;

#define ah_regdomain		ah_capabilities.cap_regdomain.reg_current
#define ah_regdomain_hw		ah_capabilities.cap_regdomain.reg_hw
#define ah_modes		ah_capabilities.cap_mode
#define ah_ee_version		ah_capabilities.cap_eeprom.ee_version

	u32			ah_atim_window;
	u32			ah_aifs;
	u32			ah_cw_min;
	u32			ah_cw_max;
	bool			ah_software_retry;
	u32			ah_limit_tx_retries;

	u32			ah_antenna[AR5K_EEPROM_N_MODES][AR5K_ANT_MAX];
	bool			ah_ant_diversity;

	u8			ah_sta_id[ETH_ALEN];
	u8			ah_bssid[ETH_ALEN];

	u32			ah_gpio[AR5K_MAX_GPIO];
	int			ah_gpio_npins;

	struct ath5k_capabilities ah_capabilities;

	struct ath5k_txq_info	ah_txq[AR5K_NUM_TX_QUEUES];
	u32			ah_txq_interrupts;

	u32			*ah_rf_banks;
	size_t			ah_rf_banks_size;
	struct ath5k_gain	ah_gain;
	u32			ah_offset[AR5K_MAX_RF_BANKS];

	struct {
		u16		txp_pcdac[AR5K_EEPROM_POWER_TABLE_SIZE];
		u16		txp_rates[AR5K_MAX_RATES];
		s16		txp_min;
		s16		txp_max;
		bool		txp_tpc;
		s16		txp_ofdm;
	} ah_txpower;

	struct {
		bool		r_enabled;
		int		r_last_alert;
		struct ieee80211_channel r_last_channel;
	} ah_radar;

	/*
	 * Function pointers
	 */
	int (*ah_setup_tx_desc)(struct ath_hw *, struct ath_desc *,
		unsigned int, unsigned int, enum ath5k_pkt_type, unsigned int,
		unsigned int, unsigned int, unsigned int, unsigned int,
		unsigned int, unsigned int, unsigned int);
	bool (*ah_setup_xtx_desc)(struct ath_hw *, struct ath_desc *,
		unsigned int, unsigned int, unsigned int, unsigned int,
		unsigned int, unsigned int);
	int (*ah_fill_tx_desc)(struct ath_hw *, struct ath_desc *,
		unsigned int, bool, bool);
	int (*ah_proc_tx_desc)(struct ath_hw *, struct ath_desc *);
	int (*ah_proc_rx_desc)(struct ath_hw *, struct ath_desc *);
};

/*
 * Prototypes
 */

/* General Functions */
extern int ath5k_hw_register_timeout(struct ath_hw *hal, u32 reg, u32 flag, u32 val, bool is_set);
/* Attach/Detach Functions */
extern struct ath_hw *ath5k_hw_attach(u16 device, u8 mac_version, void *sc, void __iomem *sh);
extern const struct ath5k_rate_table *ath5k_hw_get_rate_table(struct ath_hw *hal, unsigned int mode);
extern void ath5k_hw_detach(struct ath_hw *hal);
/* Reset Functions */
extern int ath5k_hw_reset(struct ath_hw *hal, enum ieee80211_if_types op_mode, struct ieee80211_channel *channel, bool change_channel);
/* Power management functions */
extern int ath5k_hw_set_power(struct ath_hw *hal, enum ath5k_power_mode mode, bool set_chip, u16 sleep_duration);
/* DMA Related Functions */
extern void ath5k_hw_start_rx(struct ath_hw *hal);
extern int ath5k_hw_stop_rx_dma(struct ath_hw *hal);
extern u32 ath5k_hw_get_rx_buf(struct ath_hw *hal);
extern void ath5k_hw_put_rx_buf(struct ath_hw *hal, u32 phys_addr);
extern int ath5k_hw_tx_start(struct ath_hw *hal, unsigned int queue);
extern int ath5k_hw_stop_tx_dma(struct ath_hw *hal, unsigned int queue);
extern u32 ath5k_hw_get_tx_buf(struct ath_hw *hal, unsigned int queue);
extern int ath5k_hw_put_tx_buf(struct ath_hw *hal, unsigned int queue, u32 phys_addr);
extern int ath5k_hw_update_tx_triglevel(struct ath_hw *hal, bool increase);
/* Interrupt handling */
extern bool ath5k_hw_is_intr_pending(struct ath_hw *hal);
extern int ath5k_hw_get_isr(struct ath_hw *hal, enum ath5k_int *interrupt_mask);
extern enum ath5k_int ath5k_hw_set_intr(struct ath_hw *hal, enum ath5k_int new_mask);
/* EEPROM access functions */
extern int ath5k_hw_set_regdomain(struct ath_hw *hal, u16 regdomain);
/* Protocol Control Unit Functions */
extern int ath5k_hw_set_opmode(struct ath_hw *hal);
/* BSSID Functions */
extern void ath5k_hw_get_lladdr(struct ath_hw *hal, u8 *mac);
extern int ath5k_hw_set_lladdr(struct ath_hw *hal, const u8 *mac);
extern void ath5k_hw_set_associd(struct ath_hw *hal, const u8 *bssid, u16 assoc_id);
extern int ath5k_hw_set_bssid_mask(struct ath_hw *hal, const u8 *mask);
/* Receive start/stop functions */
extern void ath5k_hw_start_rx_pcu(struct ath_hw *hal);
extern void ath5k_hw_stop_pcu_recv(struct ath_hw *hal);
/* RX Filter functions */
extern void ath5k_hw_set_mcast_filter(struct ath_hw *hal, u32 filter0, u32 filter1);
extern int ath5k_hw_set_mcast_filterindex(struct ath_hw *hal, u32 index);
extern int ath5k_hw_clear_mcast_filter_idx(struct ath_hw *hal, u32 index);
extern u32 ath5k_hw_get_rx_filter(struct ath_hw *ah);
extern void ath5k_hw_set_rx_filter(struct ath_hw *ah, u32 filter);
/* Beacon related functions */
extern u32 ath5k_hw_get_tsf32(struct ath_hw *hal);
extern u64 ath5k_hw_get_tsf64(struct ath_hw *hal);
extern void ath5k_hw_reset_tsf(struct ath_hw *hal);
extern void ath5k_hw_init_beacon(struct ath_hw *hal, u32 next_beacon, u32 interval);
extern int ath5k_hw_set_beacon_timers(struct ath_hw *hal, const struct ath5k_beacon_state *state);
extern void ath5k_hw_reset_beacon(struct ath_hw *hal);
extern int ath5k_hw_wait_for_beacon(struct ath_hw *hal, unsigned long phys_addr);
extern void ath5k_hw_update_mib_counters(struct ath_hw *hal, struct ath5k_mib_stats *statistics);
/* ACK/CTS Timeouts */
extern int ath5k_hw_set_ack_timeout(struct ath_hw *hal, unsigned int timeout);
extern unsigned int ath5k_hw_get_ack_timeout(struct ath_hw *hal);
extern int ath5k_hw_set_cts_timeout(struct ath_hw *hal, unsigned int timeout);
extern unsigned int ath5k_hw_get_cts_timeout(struct ath_hw *hal);
/* Key table (WEP) functions */
extern int ath5k_hw_reset_key(struct ath_hw *hal, u16 entry);
extern int ath5k_hw_is_key_valid(struct ath_hw *hal, u16 entry);
extern int ath5k_hw_set_key(struct ath_hw *hal, u16 entry, const struct ieee80211_key_conf *key, const u8 *mac);
extern int ath5k_hw_set_key_lladdr(struct ath_hw *hal, u16 entry, const u8 *mac);
/* Queue Control Unit, DFS Control Unit Functions */
extern int ath5k_hw_setup_tx_queue(struct ath_hw *hal, enum ath5k_tx_queue queue_type, struct ath5k_txq_info *queue_info);
extern int ath5k_hw_setup_tx_queueprops(struct ath_hw *hal, int queue, const struct ath5k_txq_info *queue_info);
extern int ath5k_hw_get_tx_queueprops(struct ath_hw *hal, int queue, struct ath5k_txq_info *queue_info);
extern void ath5k_hw_release_tx_queue(struct ath_hw *hal, unsigned int queue);
extern int ath5k_hw_reset_tx_queue(struct ath_hw *hal, unsigned int queue);
extern u32 ath5k_hw_num_tx_pending(struct ath_hw *hal, unsigned int queue);
extern int ath5k_hw_set_slot_time(struct ath_hw *hal, unsigned int slot_time);
extern unsigned int ath5k_hw_get_slot_time(struct ath_hw *hal);
/* Hardware Descriptor Functions */
extern int ath5k_hw_setup_rx_desc(struct ath_hw *hal, struct ath_desc *desc, u32 size, unsigned int flags);
/* GPIO Functions */
extern void ath5k_hw_set_ledstate(struct ath_hw *hal, unsigned int state);
extern int ath5k_hw_set_gpio_output(struct ath_hw *hal, u32 gpio);
extern int ath5k_hw_set_gpio_input(struct ath_hw *hal, u32 gpio);
extern u32 ath5k_hw_get_gpio(struct ath_hw *hal, u32 gpio);
extern int ath5k_hw_set_gpio(struct ath_hw *hal, u32 gpio, u32 val);
extern void ath5k_hw_set_gpio_intr(struct ath_hw *hal, unsigned int gpio, u32 interrupt_level);
/* Regulatory Domain/Channels Setup */
extern u16 ath5k_get_regdomain(struct ath_hw *hal);
/* Misc functions */
extern void ath5k_hw_dump_state(struct ath_hw *hal);
extern int ath5k_hw_get_capability(struct ath_hw *hal, enum ath5k_capability_type cap_type, u32 capability, u32 *result);


/* Initial register settings functions */
extern int ath5k_hw_write_initvals(struct ath_hw *hal, u8 mode, bool change_channel);
/* Initialize RF */
extern int ath5k_hw_rfregs(struct ath_hw *hal, struct ieee80211_channel *channel, unsigned int mode);
extern int ath5k_hw_rfgain(struct ath_hw *hal, unsigned int freq);
extern enum ath5k_rfgain ath5k_hw_get_rf_gain(struct ath_hw *hal);
extern int ath5k_hw_set_rfgain_opt(struct ath_hw *hal);


/* PHY/RF channel functions */
extern bool ath5k_channel_ok(struct ath_hw *hal, u16 freq, unsigned int flags);
extern int ath5k_hw_channel(struct ath_hw *hal, struct ieee80211_channel *channel);
/* PHY calibration */
extern int ath5k_hw_phy_calibrate(struct ath_hw *hal, struct ieee80211_channel *channel);
extern int ath5k_hw_phy_disable(struct ath_hw *hal);
/* Misc PHY functions */
extern u16 ath5k_hw_radio_revision(struct ath_hw *hal, unsigned int chan);
extern void ath5k_hw_set_def_antenna(struct ath_hw *hal, unsigned int ant);
extern unsigned int ath5k_hw_get_def_antenna(struct ath_hw *hal);
/* TX power setup */
extern int ath5k_hw_txpower(struct ath_hw *hal, struct ieee80211_channel *channel, unsigned int txpower);
extern int ath5k_hw_set_txpower_limit(struct ath_hw *hal, unsigned int power);


static inline u32 ath5k_hw_reg_read(struct ath_hw *hal, u16 reg)
{
	return ioread32(hal->ah_sh + reg);
}

static inline void ath5k_hw_reg_write(struct ath_hw *hal, u32 val, u16 reg)
{
	iowrite32(val, hal->ah_sh + reg);
}

#endif
