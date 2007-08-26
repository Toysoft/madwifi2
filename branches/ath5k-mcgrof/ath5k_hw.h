/*
 * Copyright (C) 2002-2007 Sam Leffler, Errno Consulting
 * Copyright (C) 2004-2007 Reyk Floeter <reyk@openbsd.org>
 * Copyright (C) 2006-2007 Nick Kossifidis <mickflemm@gmail.com>
 * Copyright (C) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
 *
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
 */

#ifndef _ATH5K_HW_H
#define _ATH5K_HW_H

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/cache.h>
#include <linux/if_arp.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h> /* For radar functions */
#include <linux/wireless.h>
#include <linux/interrupt.h>

#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <net/mac80211.h>

#include "ath5k_reg.h"
/* XXX: Remove this, use regdomain which supports all channels and 
 * powers for now, then use cfg80211/nl80211 reg domain support */
#include "ieee80211_regdomain.h"
#include "if_athrate.h"

/* Atheros Device IDs */
#define PCI_ATH(dev_id)	\
	PCI_VDEVICE(ATHEROS, PCI_PRODUCT_ATHEROS_AR##dev_id)
#define ATH5K_PRODUCT(dev_id, mac_ver) \
	PCI_VENDOR_ID_ATHEROS,  PCI_PRODUCT_ATHEROS_AR##dev_id, mac_ver

/* These have 5210 mac */
#define PCI_PRODUCT_ATHEROS_AR5210 		0x0007 /* AR5210 */
#define PCI_PRODUCT_ATHEROS_AR5210_AP 		0x0207 /* AR5210 (Early) */
#define PCI_PRODUCT_ATHEROS_AR5210_DEFAULT 	0x1107 /* AR5210 (no eeprom) */

/* These have 5211 mac */
#define PCI_PRODUCT_ATHEROS_AR5211 		0x0012 /* AR5211 */
#define PCI_PRODUCT_ATHEROS_AR5212_DEFAULT 	0x1113 /* AR5212 (no eeprom) */
#define PCI_PRODUCT_ATHEROS_AR5311 		0x0011 /* AR5311 */
#define PCI_PRODUCT_ATHEROS_AR5211_FPGA11B 	0xf11b /* AR5211 (emulation board) */
#define PCI_PRODUCT_ATHEROS_AR5211_LEGACY 	0xff12 /* AR5211 (emulation board) */

/* These have 5212 mac */
#define PCI_PRODUCT_ATHEROS_AR5212 		0x0013 /* AR5212 */
#define PCI_PRODUCT_ATHEROS_AR5211_DEFAULT 	0x1112 /* AR5211 (no eeprom) */
#define PCI_PRODUCT_ATHEROS_AR5212_FPGA 	0xf013 /* AR5212 (emulation board) */
#define PCI_PRODUCT_ATHEROS_AR5212_IBM 		0x1014 /* AR5212 (IBM MiniPCI) */
#define PCI_PRODUCT_3COM_3CRDAG675 		0x0013 /* 3CRDAG675 (Atheros AR5212) */
#define PCI_PRODUCT_3COM2_3CRPAG175 		0x0013 /* 3CRPAG175 (Atheros AR5212) */
#define PCI_PRODUCT_ATHEROS_AR5212_REV2 	0x0052 /* AR5312 WMAC (AP31) */
#define PCI_PRODUCT_ATHEROS_AR5212_REV7 	0x0057 /* AR5312 WMAC (AP30-040) */
#define PCI_PRODUCT_ATHEROS_AR5212_REV8 	0x0058 /* AR5312 WMAC (AP43-030) */
#define PCI_PRODUCT_ATHEROS_AR5212_0014 	0x0014 /* AR5212 compatible */
#define PCI_PRODUCT_ATHEROS_AR5212_0015 	0x0015 /* AR5212 compatible */
#define PCI_PRODUCT_ATHEROS_AR5212_0016 	0x0016 /* AR5212 compatible */
#define PCI_PRODUCT_ATHEROS_AR5212_0017 	0x0017 /* AR5212 compatible */
#define PCI_PRODUCT_ATHEROS_AR5212_0018 	0x0018 /* AR5212 compatible */
#define PCI_PRODUCT_ATHEROS_AR5212_0019 	0x0019 /* AR5212 compatible */
/* XXX: These are the WMACs. They are Atheros' System on Chip (SoC).
 * They are not connected via PCI and require the use of 
 * linux/platform_device.h driver infrastructure. See MadWifi's 
 * if_ath_ahb.c implementation for details */
#define PCI_PRODUCT_ATHEROS_AR2413 		0x001a /* AR2413 (Griffin-lite) */
#define PCI_PRODUCT_ATHEROS_AR5413 		0x001b /* AR5413 (Eagle) */
#define PCI_PRODUCT_ATHEROS_AR5424 		0x001c /* AR5424 (Condor PCI-E) */

#define IEEE80211_MAX_LEN       2500
#ifndef IEEE80211_RATE_TURBO /* may be removed from mac80211 soon */
#define IEEE80211_RATE_TURBO    0x00000080
#define MODULATION_XR           0x00000200
#endif

/* Adding this flag to rate_code enables short preamble */
#define AR5K_SET_SHORT_PREAMBLE 0x04
#define HAS_SHPREAMBLE(_ix) (rt->rates[_ix].modulation == IEEE80211_RATE_CCK_2)
#define SHPREAMBLE_FLAG(_ix) HAS_SHPREAMBLE(_ix)?AR5K_SET_SHORT_PREAMBLE:0

typedef enum {
        AR5K_M_STA      = IEEE80211_IF_TYPE_STA,
        AR5K_M_IBSS     = IEEE80211_IF_TYPE_IBSS,
        AR5K_M_HOSTAP   = IEEE80211_IF_TYPE_AP,
        AR5K_M_MONITOR  = IEEE80211_IF_TYPE_MNTR,
} ar5k_optmode;

#define AR5K_PRINTF(fmt, ...)   printk("%s: " fmt, __func__, ##__VA_ARGS__)
#define AR5K_PRINT(fmt)         printk("%s: " fmt, __func__)
#ifdef AR5K_DEBUG
#define AR5K_TRACE              printk("%s:%d\n", __func__, __LINE__)
#else
#define AR5K_TRACE
#endif

/* Set this to 1 to disable regulatory domain restrictions for channel tests.
 * WARNING: This is for debuging only and has side effects (eg. scan takes too long
 * and results timeouts). It's also illegal to tune to some of the supported frequencies
 * in some countries, so use this at your own risk, you 've been warned. */
#define CHAN_DEBUG    0
/* Uncomment this for debuging (warning that results TOO much output) */
//#define AR5K_DEBUG    1

/* XXX: remove this */
#ifndef TRUE
#define	TRUE	1
#endif
#ifndef FALSE
#define	FALSE	0
#endif
typedef u_int8_t AR5K_BOOL;

/*
 * Error codes reported from HAL to the driver
 */
typedef enum {
	AR5K_OK		= 0,	/* Everything went O.K.*/
	AR5K_ENOMEM	= 1,	/* Unable to allocate memory for ath_hal*/
	AR5K_EIO	= 2,	/* Hardware I/O Error*/
	AR5K_EELOCKED	= 3,	/* Unable to access EEPROM*/
	AR5K_EEBADSUM	= 4,	/* Invalid EEPROM checksum*/
	AR5K_EEREAD	= 5,	/* Unable to get device caps from EEPROM */
	AR5K_EEBADMAC	= 6,	/* Unable to read MAC address from EEPROM */
	AR5K_EINVAL	= 7,	/* Invalid parameter to function */
	AR5K_ENOTSUPP	= 8,	/* Hardware revision not supported */
	AR5K_EINPROGRESS= 9,	/* Unexpected error ocured during process */
} AR5K_STATUS;

/*
 * Some tuneable values (these should be changeable by the user)
 */
#define AR5K_TUNE_DMA_BEACON_RESP		2
#define AR5K_TUNE_SW_BEACON_RESP		10
#define AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF	0
#define AR5K_TUNE_RADAR_ALERT			FALSE
#define AR5K_TUNE_MIN_TX_FIFO_THRES		1
#define AR5K_TUNE_MAX_TX_FIFO_THRES		((IEEE80211_MAX_LEN/ 64) + 1)
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
#define AR5K_TUNE_TPC_TXPOWER			TRUE
#define AR5K_TUNE_ANT_DIVERSITY			TRUE
#define AR5K_TUNE_HWTXTRIES			4

/* token to use for aifs, cwmin, cwmax in MadWiFi */
#define	AR5K_TXQ_USEDEFAULT	((u_int32_t) -1)

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
enum ath5k_srev_type {
	AR5K_VERSION_VER,
	AR5K_VERSION_REV,
	AR5K_VERSION_RAD,
	AR5K_VERSION_DEV
};

struct ath5k_srev_name {
	const char		*sr_name;
	enum ath5k_srev_type	sr_type;
	u_int			sr_val;
};

#define AR5K_SREV_NAME	{						\
	{ "5210",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5210 },	\
	{ "5311",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311 },	\
	{ "5311a",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311A },\
	{ "5311b",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311B },\
	{ "5211",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5211 },	\
	{ "5212",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5212 },	\
	{ "5213",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213 },	\
	{ "xxxx",	AR5K_VERSION_VER,	AR5K_SREV_UNKNOWN },	\
	{ "5110",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5110 },	\
	{ "5111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111 },	\
	{ "2111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2111 },	\
	{ "5112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112 },	\
	{ "5112a",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112A },	\
	{ "2112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112 },	\
	{ "2112a",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112A },	\
	{ "xxxx",	AR5K_VERSION_RAD,	AR5K_SREV_UNKNOWN },	\
	{ "2413",	AR5K_VERSION_DEV,	PCI_PRODUCT_ATHEROS_AR2413 },	\
 	{ "5413",	AR5K_VERSION_DEV,	PCI_PRODUCT_ATHEROS_AR5413 },	\
 	{ "5424",	AR5K_VERSION_DEV,	PCI_PRODUCT_ATHEROS_AR5424 },	\
 	{ "xxxx",	AR5K_VERSION_DEV,	AR5K_SREV_UNKNOWN }	\
}

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


/* -- TX Definitions -- */

/* Tx Descriptor */
struct ath_tx_status {
	u_int16_t	ts_seqnum;
	u_int16_t	ts_tstamp;
	u_int8_t	ts_status;
	u_int8_t	ts_rate;
	int8_t		ts_rssi;
	u_int8_t	ts_shortretry;
	u_int8_t	ts_longretry;
	u_int8_t	ts_virtcol;
	u_int8_t	ts_antenna;
};

#define AR5K_TXSTAT_ALTRATE	0x80
#define AR5K_TXERR_XRETRY	0x01
#define AR5K_TXERR_FILT		0x02
#define AR5K_TXERR_FIFO		0x04

/*
 * Queue types used to classify tx queues.
 */
typedef enum {
	AR5K_TX_QUEUE_INACTIVE = 0,/*This queue is not used -see ath_hal_releasetxqueue*/
	AR5K_TX_QUEUE_DATA,	  /*A normal data queue*/
	AR5K_TX_QUEUE_XR_DATA,	  /*An XR-data queue*/
	AR5K_TX_QUEUE_BEACON,	  /*The beacon queue*/
	AR5K_TX_QUEUE_CAB,	  /*The ater-beacon queue*/
	AR5K_TX_QUEUE_UAPSD,	  /*Unscheduled Automatic Power Save Delivery queue*/
} AR5K_TX_QUEUE;

#define	AR5K_NUM_TX_QUEUES		10
#define	AR5K_NUM_TX_QUEUES_NOQCU	2

/*
 * Queue syb-types to classify normal data queues.
 * These are the 4 Access Categories as defined in
 * WME spec. 0 is the lowest priority and 4 is the
 * highest. Normal data that hasn't been classified
 * goes to the Best Effort AC.
 */
typedef enum {
	AR5K_WME_AC_BK = 0,	/*Background traffic*/
	AR5K_WME_AC_BE, 	/*Best-effort (normal) traffic)*/
	AR5K_WME_AC_VI, 	/*Video traffic*/
	AR5K_WME_AC_VO, 	/*Voice traffic*/
} AR5K_TX_QUEUE_SUBTYPE;

/*
 * Queue ID numbers as returned by the HAL, each number
 * represents a hw queue. If hw does not support hw queues
 * (eg 5210) all data goes in one queue. These match
 * d80211 definitions (net80211/MadWiFi don't use them).
 */
typedef enum {
	AR5K_TX_QUEUE_ID_NOQCU_DATA	= 0,
	AR5K_TX_QUEUE_ID_NOQCU_BEACON	= 1,
	AR5K_TX_QUEUE_ID_DATA_MIN	= 0, /*IEEE80211_TX_QUEUE_DATA0*/
	AR5K_TX_QUEUE_ID_DATA_MAX	= 4, /*IEEE80211_TX_QUEUE_DATA4*/
	AR5K_TX_QUEUE_ID_DATA_SVP	= 5, /*IEEE80211_TX_QUEUE_SVP - Spectralink Voice Protocol*/
	AR5K_TX_QUEUE_ID_CAB		= 6, /*IEEE80211_TX_QUEUE_AFTER_BEACON*/
	AR5K_TX_QUEUE_ID_BEACON		= 7, /*IEEE80211_TX_QUEUE_BEACON*/
	AR5K_TX_QUEUE_ID_UAPSD		= 8,
	AR5K_TX_QUEUE_ID_XR_DATA	= 9,
} AR5K_TX_QUEUE_ID;


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
typedef struct {
	AR5K_TX_QUEUE			tqi_type;	/* See AR5K_TX_QUEUE */
	AR5K_TX_QUEUE_SUBTYPE		tqi_subtype;	/* See AR5K_TX_QUEUE_SUBTYPE */
	u_int16_t			tqi_flags;	/* Tx queue flags (see above) */
	u_int32_t			tqi_aifs;	/* Arbitrated Interframe Space */
	int32_t				tqi_cw_min;	/* Minimum Contention Window */
	int32_t				tqi_cw_max;	/* Maximum Contention Window */
	u_int32_t			tqi_cbr_period; /* Constant bit rate period */
	u_int32_t			tqi_cbr_overflow_limit;
	u_int32_t			tqi_burst_time;
	u_int32_t			tqi_ready_time; /* Not used */
	u_int32_t			tqi_comp_buffer;/* Compression Buffer's phys addr */
} AR5K_TXQ_INFO;

/*
 * Transmit packet types.
 * These are not fully used inside OpenHAL yet
 */
typedef enum {
	AR5K_PKT_TYPE_NORMAL		= 0,
	AR5K_PKT_TYPE_ATIM		= 1,
	AR5K_PKT_TYPE_PSPOLL		= 2,
	AR5K_PKT_TYPE_BEACON		= 3,
	AR5K_PKT_TYPE_PROBE_RESP	= 4,
	AR5K_PKT_TYPE_PIFS		= 5,
} AR5K_PKT_TYPE;

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
	AR5K_CCK_PHY_TIME(_sp) +					\
	((AR5K_CCK_NUM_BITS(_frmlen) * 1000) / _kbps) +		\
	AR5K_CCK_SIFS_TIME

/* OFDM */
#define AR5K_OFDM_NUM_BITS(_frmlen) (AR5K_OFDM_PLCP_BITS + (_frmlen << 3))

#define AR5K_OFDM_NUM_BITS_PER_SYM(_kbps) ((_kbps *			\
	AR5K_OFDM_SYMBOL_TIME) / 1000)

#define AR5K_OFDM_NUM_BITS(_frmlen) (AR5K_OFDM_PLCP_BITS + (_frmlen << 3))

#define AR5K_OFDM_NUM_SYMBOLS(_kbps, _frmlen)				\
	howmany(AR5K_OFDM_NUM_BITS(_frmlen), AR5K_OFDM_NUM_BITS_PER_SYM(_kbps))

#define AR5K_OFDM_TX_TIME(_kbps, _frmlen)				\
	AR5K_OFDM_PREAMBLE_TIME + AR5K_OFDM_SIFS_TIME +			\
	(AR5K_OFDM_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_OFDM_SYMBOL_TIME)

/* TURBO */
#define AR5K_TURBO_NUM_BITS(_frmlen) (AR5K_TURBO_PLCP_BITS + (_frmlen << 3))

#define AR5K_TURBO_NUM_BITS_PER_SYM(_kbps) (((_kbps << 1) *		\
	AR5K_TURBO_SYMBOL_TIME) / 1000)

#define AR5K_TURBO_NUM_BITS(_frmlen) (AR5K_TURBO_PLCP_BITS + (_frmlen << 3))

#define AR5K_TURBO_NUM_SYMBOLS(_kbps, _frmlen)				\
	howmany(AR5K_TURBO_NUM_BITS(_frmlen),				\
	AR5K_TURBO_NUM_BITS_PER_SYM(_kbps))

#define AR5K_TURBO_TX_TIME(_kbps, _frmlen)				\
	AR5K_TURBO_PREAMBLE_TIME + AR5K_TURBO_SIFS_TIME +		\
	(AR5K_TURBO_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_TURBO_SYMBOL_TIME)

/* eXtendent Range (?)*/
#define AR5K_XR_PREAMBLE_TIME(_kbps) (((_kbps) < 1000) ? 173 : 76)

#define AR5K_XR_NUM_BITS_PER_SYM(_kbps) ((_kbps *			\
	AR5K_XR_SYMBOL_TIME) / 1000)

#define AR5K_XR_NUM_BITS(_frmlen) (AR5K_XR_PLCP_BITS + (_frmlen << 3))

#define AR5K_XR_NUM_SYMBOLS(_kbps, _frmlen)				\
	howmany(AR5K_XR_NUM_BITS(_frmlen), AR5K_XR_NUM_BITS_PER_SYM(_kbps))

#define AR5K_XR_TX_TIME(_kbps, _frmlen)				\
	AR5K_XR_PREAMBLE_TIME(_kbps) + AR5K_XR_SIFS_TIME +		\
	(AR5K_XR_NUM_SYMBOLS(_kbps, _frmlen) * AR5K_XR_SYMBOL_TIME)

/*
 * DMA size definitions (2^n+2)
 */
typedef enum {
	AR5K_DMASIZE_4B	= 0,
	AR5K_DMASIZE_8B,
	AR5K_DMASIZE_16B,
	AR5K_DMASIZE_32B,
	AR5K_DMASIZE_64B,
	AR5K_DMASIZE_128B,
	AR5K_DMASIZE_256B,
	AR5K_DMASIZE_512B
} ath5k_dmasize_t;



/****************\
  RX DEFINITIONS
\****************/

/*
 * Rx Descriptor
 */
struct ath_rx_status {
	u_int16_t	rs_datalen;
	u_int16_t	rs_tstamp;
	u_int8_t	rs_status;
	u_int8_t	rs_phyerr;
	int8_t		rs_rssi;
	u_int8_t	rs_keyix;
	u_int8_t	rs_rate;
	u_int8_t	rs_antenna;
	u_int8_t	rs_more;
};

#define AR5K_RXERR_CRC		0x01
#define AR5K_RXERR_PHY		0x02
#define AR5K_RXERR_FIFO		0x04
#define AR5K_RXERR_DECRYPT	0x08
#define AR5K_RXERR_MIC		0x10
#define AR5K_RXKEYIX_INVALID	((u_int8_t) - 1)
#define AR5K_TXKEYIX_INVALID	((u_int32_t) - 1)

#define	AR5K_WEP_NKID		4 /* number of key ids */

/*
 * RX filters
 * Most of them are not yet used inside OpenHAL
 */
#define	AR5K_RX_FILTER_UCAST 		0x00000001	/* Don't filter unicast frames */
#define	AR5K_RX_FILTER_MCAST 		0x00000002	/* Don't filter multicast frames */
#define	AR5K_RX_FILTER_BCAST 		0x00000004	/* Don't filter broadcast frames */
#define	AR5K_RX_FILTER_CONTROL 		0x00000008	/* Don't filter control frames */
#define	AR5K_RX_FILTER_BEACON 		0x00000010	/* Don't filter beacon frames */
#define	AR5K_RX_FILTER_PROM 		0x00000020	/* Set promiscuous mode */
#define	AR5K_RX_FILTER_XRPOLL 		0x00000040	/* Don't filter XR poll frame */
#define	AR5K_RX_FILTER_PROBEREQ 	0x00000080	/* Don't filter probe requests */
#define	AR5K_RX_FILTER_PHYERROR		0x00000100	/* Don't filter phy errors */
#define	AR5K_RX_FILTER_PHYRADAR 	0x00000200	/* Don't filter phy radar errors*/

typedef struct {
	u_int32_t	ackrcv_bad;
	u_int32_t	rts_bad;
	u_int32_t	rts_good;
	u_int32_t	fcs_bad;
	u_int32_t	beacons;
} AR5K_MIB_STATS;


/* -- Beacon timer definitions -- */

#define AR5K_BEACON_PERIOD	0x0000ffff
#define AR5K_BEACON_ENA		0x00800000 /*enable beacon xmit*/
#define AR5K_BEACON_RESET_TSF	0x01000000 /*force a TSF reset*/

/*
 * Per-station beacon timer state.
 */
typedef struct {
	u_int32_t	bs_next_beacon;
	u_int32_t	bs_next_dtim;
	u_int32_t	bs_interval;		/*in TU's -see net80211/ieee80211_var.h-
						can also include the above flags*/
	u_int8_t	bs_dtim_period;
	u_int8_t	bs_cfp_period;
	u_int16_t	bs_cfp_max_duration;	/*if non-zero hw is setup to coexist with
						a Point Coordination Function capable AP*/
	u_int16_t	bs_cfp_du_remain;
	u_int16_t	bs_tim_offset;
	u_int16_t	bs_sleep_duration;
	u_int16_t	bs_bmiss_threshold;
	u_int32_t  	bs_cfp_next;
} AR5K_BEACON_STATE;




/* -- Common definitions -- */

/*
 * Atheros descriptor
 */
struct ath_desc {
	u_int32_t	ds_link;
	u_int32_t	ds_data;
	u_int32_t	ds_ctl0;
	u_int32_t	ds_ctl1;
	u_int32_t	ds_hw[4];

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

/*
 * 802.11 operating modes...
 */
#define AR5K_MODE_11A	0x01
#define AR5K_MODE_11B	0x02
#define AR5K_MODE_11G	0x04
#define AR5K_MODE_TURBO	0x08
#define AR5K_MODE_108G	0x10
#define AR5K_MODE_XR	0x20
#define AR5K_MODE_ALL	(AR5K_MODE_11A   |	\
			 AR5K_MODE_11B   |	\
			 AR5K_MODE_11G   |	\
			 AR5K_MODE_TURBO |	\
			 AR5K_MODE_108G	 |	\
			 AR5K_MODE_XR)

/*
 * Channel definitions
 */
typedef struct {
	u_int16_t	freq;		/* setting in Mhz */
	u_int16_t	channel_flags;
	u_int8_t	private_flags;	/* not used in OpenHAL yet*/
} AR5K_CHANNEL;

#define AR5K_SLOT_TIME_9	396
#define AR5K_SLOT_TIME_20	880
#define AR5K_SLOT_TIME_MAX	0xffff

/* channel_flags */
/* XXX mcgrof: I think we can move this to use mac80211 definitions as
 * this is non-hardware specific. Its just used in ath5k_hw_nic_wakeup()
 * to setup the mode. Regulatory domains uses this as well but we are
 * going to be implementing our own regulatory domain module. 
 *
 * XXX mcgrof: move to IEEE80211_RATE_OFDM, IEEE80211_RATE_CCK, 
 * IEEE80211_RATE_TURBO and friends from mac80211.h
 *
 * */
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

#define	CHANNEL_ALL 	(CHANNEL_OFDM|CHANNEL_CCK| CHANNEL_2GHZ |\
			 CHANNEL_5GHZ | CHANNEL_TURBO)

#define	CHANNEL_ALL_NOTURBO 	(CHANNEL_ALL &~ CHANNEL_TURBO)
#define CHANNEL_MODES	CHANNEL_ALL

/*
 * Used internaly in OpenHAL (ar5211.c/ar5212.c
 * for reset_tx_queue). Also see struct AR5K_CHANNEL.
 */
#define IS_CHAN_XR(_c) \
        ((_c.channel_flags & CHANNEL_XR) != 0)

#define IS_CHAN_B(_c) \
        ((_c.channel_flags & CHANNEL_B) != 0)

typedef enum {
	AR5K_CHIP_5GHZ = CHANNEL_5GHZ,
	AR5K_CHIP_2GHZ = CHANNEL_2GHZ,
} AR5K_CHIP;

/*
 * The following structure will be used to map 2GHz channels to
 * 5GHz Atheros channels.
 */
struct ath5k_athchan_2ghz {
	u_int32_t	a2_flags;
	u_int16_t	a2_athchan;
};

/*
 * Rate definitions
 */

#define AR5K_MAX_RATES	32 /*max number of rates on the rate table*/

typedef struct {
	u_int8_t	valid;		/* Valid for rate control */
	u_int32_t	modulation;
	u_int16_t	rate_kbps;		
	u_int8_t	rate_code;	/* Rate mapping for h/w descriptors */
	u_int8_t	dot11_rate;
	u_int8_t	control_rate;
	u_int16_t	lp_ack_duration;/* long preamble ACK duration */
	u_int16_t	sp_ack_duration;/* short preamble ACK duration*/
} AR5K_RATE;

struct {
	u_int16_t	rate_count;				
	u_int8_t	rate_code_to_index[AR5K_MAX_RATES];	/* Back-mapping */
	AR5K_RATE	rates[AR5K_MAX_RATES];
} ar5k_rate_table;

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
	{ 1, IEEE80211_RATE_CCK, 2000, 26, 132, 1 },	\
	{ 1, IEEE80211_RATE_CCK, 5500, 25, 139, 1 },	\
	{ 1, IEEE80211_RATE_CCK, 11000, 24, 150, 1 } }	\
}

#define AR5K_RATES_11G { 12, {					\
	255, 255, 255, 255, 255, 255, 255, 255, 10, 8, 6, 4,	\
	11, 9, 7, 5, 255, 255, 255, 255, 255, 255, 255, 255,	\
	3, 2, 1, 0, 255, 255, 255, 255 }, {			\
	{ 1, IEEE80211_RATE_CCK, 1000, 27, 2, 0 },		\
	{ 1, IEEE80211_RATE_CCK, 2000, 26, 4, 1 },		\
	{ 1, IEEE80211_RATE_CCK, 5500, 25, 11, 1 },		\
	{ 1, IEEE80211_RATE_CCK, 11000, 24, 22, 1 },	\
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
	{ 1, IEEE80211_RATE_TURBO, 6000, 11, 140, 0 },	\
	{ 1, IEEE80211_RATE_TURBO, 9000, 15, 18, 0 },	\
	{ 1, IEEE80211_RATE_TURBO, 12000, 10, 152, 2 },	\
	{ 1, IEEE80211_RATE_TURBO, 18000, 14, 36, 2 },	\
	{ 1, IEEE80211_RATE_TURBO, 24000, 9, 176, 4 },	\
	{ 1, IEEE80211_RATE_TURBO, 36000, 13, 72, 4 },	\
	{ 1, IEEE80211_RATE_TURBO, 48000, 8, 96, 4 },	\
	{ 1, IEEE80211_RATE_TURBO, 54000, 12, 108, 4 } }	\
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

/* key types */
typedef enum {
	AR5K_CIPHER_WEP		= 0,
	AR5K_CIPHER_AES_OCB	= 1,
	AR5K_CIPHER_AES_CCM	= 2,
	AR5K_CIPHER_CKIP	= 3,
	AR5K_CIPHER_TKIP	= 4,
	AR5K_CIPHER_CLR		= 5,	/* no encryption */
	AR5K_CIPHER_MIC		= 127	/* used for Message 
					   Integrity Code */
} AR5K_CIPHER;

#define AR5K_KEYVAL_LENGTH_40	5
#define AR5K_KEYVAL_LENGTH_104	13
#define AR5K_KEYVAL_LENGTH_128	16
#define AR5K_KEYVAL_LENGTH_MAX	AR5K_KEYVAL_LENGTH_128

typedef struct {
	int		wk_len;		/* key's length */
	u_int8_t	wk_key[AR5K_KEYVAL_LENGTH_MAX];
	u_int8_t	wk_type;	/* see above */
	u_int8_t	wk_mic[8];	/* TKIP MIC key */
} AR5K_KEYVAL;



/* -- Hardware related definitions -- */

/*
 * Misc definitions
 */
#define	AR5K_RSSI_EP_MULTIPLIER	(1<<7)

#define AR5K_ASSERT_ENTRY(_e, _s) do {		\
	if (_e >= _s)				\
		return (FALSE);			\
} while (0)

typedef struct {
	u_int32_t	ns_avgbrssi;	/* average beacon rssi */
	u_int32_t	ns_avgrssi;	/* average data rssi */
	u_int32_t	ns_avgtxrssi;	/* average tx rssi */
} AR5K_NODE_STATS;

typedef enum {
	AR5K_ANT_VARIABLE	= 0,	/* variable by programming */
	AR5K_ANT_FIXED_A	= 1,	/* fixed to 11a frequencies */
	AR5K_ANT_FIXED_B	= 2,	/* fixed to 11b frequencies */
	AR5K_ANT_MAX		= 3,
} AR5K_ANT_SETTING;

/*
 * Power management
 */
enum {
	AR5K_PM_UNDEFINED = 0,
	AR5K_PM_AUTO,
	AR5K_PM_AWAKE,
	AR5K_PM_FULL_SLEEP,
	AR5K_PM_NETWORK_SLEEP,
} ar5k_power_mode;


/*
 * LED states
 */
typedef int AR5K_LED_STATE;

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
 * Chipset capabilities -- see ath5k_hw_get_capability.
 * ath5k_hw_get_capability function is not yet fully implemented
 * so most of these don't work yet...
 */
typedef enum {
	AR5K_CAP_REG_DMN		= 0,	/* Used to get current reg. domain id */
	AR5K_CAP_CIPHER			= 1,	/* Can handle encryption */
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
} AR5K_CAPABILITY_TYPE;

typedef struct {
	/*
	 * Supported PHY modes
	 * (ie. CHANNEL_A, CHANNEL_B, ...)
	 */
	u_int16_t	cap_mode;

	/*
	 * Frequency range (without regulation restrictions)
	 */
	struct {
		u_int16_t	range_2ghz_min;
		u_int16_t	range_2ghz_max;
		u_int16_t	range_5ghz_min;
		u_int16_t	range_5ghz_max;
	} cap_range;

	/*
	 * Active regulation domain settings
	 */
	struct {
		ieee80211_regdomain_t	reg_current;
		ieee80211_regdomain_t	reg_hw;
	} cap_regdomain;

	/*
	 * Values stored in the EEPROM (some of them...)
	 */
	struct ath5k_eeprom_info	cap_eeprom;

	/*
	 * Queue information
	 */
	struct {
		u_int8_t	q_tx_num;
	} cap_queues;
} ath5k_capabilities_t;

#define ath5k_hw_get_cap_phycounters(ah) \
	ath5k_hw_get_capability(ah, AR5K_CAP_PHYCOUNTERS, 0, NULL) == AR5K_OK



/* -- HAL defintions -- */

/*
 * XXX: remove all regulatory stuff and just use defaults which lets hardware do anything for now. 
 * Later on extend the kernel's new cfg80211/nl80211 regulatory agent.
 *
 * Regulation stuff
 */
typedef enum ieee80211_countrycode AR5K_CTRY_CODE;

/* Default regulation domain if stored value EEPROM value is invalid */
#define AR5K_TUNE_REGDOMAIN	DMN_FCC2_FCCA	/* Canada */
#define AR5K_TUNE_CTRY		CTRY_DEFAULT

/*
 * Misc defines
 */

typedef AR5K_BOOL (ath5k_rfgain_t)
	(struct ath_hal *, AR5K_CHANNEL *, u_int);

#define AR5K_MAX_GPIO		10
#define AR5K_MAX_RF_BANKS	8

struct ath_hal {
	u_int32_t		ah_magic;
	u_int16_t		ah_device;

	__iomem void*		ah_iobase;
	AR5K_CTRY_CODE		ah_country_code;

	AR5K_INT		ah_imr;

	enum ar5k_opmode	ah_op_mode;
	enum ar5k_power_mode	ah_power_mode;
	AR5K_CHANNEL		ah_current_channel;
	AR5K_BOOL		ah_turbo;
	AR5K_BOOL		ah_calibration;
	AR5K_BOOL		ah_running;
	AR5K_BOOL		ah_single_chip;
	AR5K_RFGAIN		ah_rf_gain;

	struct ar5k_rate_table	ah_rt_11a;
	struct ar5k_rate_table	ah_rt_11b;
	struct ar5k_rate_table	ah_rt_11g;
	struct ar5k_rate_table	ah_rt_turbo;
	struct ar5k_rate_table	ah_rt_xr;

	u_int32_t		ah_mac_srev;
	u_int16_t		ah_mac_version;
	u_int16_t		ah_mac_revision;
	u_int16_t		ah_phy_revision;
	u_int16_t		ah_radio_5ghz_revision;
	u_int16_t		ah_radio_2ghz_revision;

	enum ath5k_version	ah_version;
	enum ath5k_radio	ah_radio;
	u_int32_t		ah_phy;

	AR5K_BOOL		ah_5ghz;
	AR5K_BOOL		ah_2ghz;

#define ah_ee_version		ah_capabilities.cap_eeprom.ee_version

	u_int32_t		ah_atim_window;
	u_int32_t		ah_aifs;
	u_int32_t		ah_cw_min;
	u_int32_t		ah_cw_max;
	AR5K_BOOL		ah_software_retry;
	u_int32_t		ah_limit_tx_retries;

	u_int32_t		ah_antenna[AR5K_EEPROM_N_MODES][AR5K_ANT_MAX];
	AR5K_BOOL		ah_ant_diversity;

	u_int8_t		ah_sta_id[ETH_ALEN];
	u_int8_t		ah_bssid[ETH_ALEN];

	u_int32_t		ah_gpio[AR5K_MAX_GPIO];
	int			ah_gpio_npins;

	ath5k_capabilities_t	ah_capabilities;

	AR5K_TXQ_INFO		ah_txq[AR5K_NUM_TX_QUEUES];
	u_int32_t		ah_txq_interrupts;

	u_int32_t		*ah_rf_banks;
	size_t			ah_rf_banks_size;
	struct ath5k_gain	ah_gain;
	u_int32_t		ah_offset[AR5K_MAX_RF_BANKS];

	struct {
		u_int16_t	txp_pcdac[AR5K_EEPROM_POWER_TABLE_SIZE];
		u_int16_t	txp_rates[AR5K_MAX_RATES];
		int16_t		txp_min, txp_max;
		AR5K_BOOL	txp_tpc;
		int16_t		txp_ofdm;
	} ah_txpower;

	struct {
		AR5K_BOOL	r_enabled;
		int		r_last_alert;
		AR5K_CHANNEL	r_last_channel;
	} ah_radar;
};

/*
 * Defintions for the Atheros Wireless LAN controller driver.
 */
/*
 * Macro to expand scalars to 64-bit objects
 */
#define	ito64(x) (sizeof(x)==8) ? (((unsigned long long int)(x)) & (0xff)) : \
		 (sizeof(x)==16) ? (((unsigned long long int)(x)) & 0xffff) : \
		 ((sizeof(x)==32) ? (((unsigned long long int)(x)) & 0xffffffff): (unsigned long long int)(x))

#define	ATH_TIMEOUT		1000

/*
 * Maximum acceptable MTU
 * MAXFRAMEBODY - WEP - QOS - RSN/WPA:
 * 2312 - 8 - 2 - 12 = 2290
 */
#define ATH_MAX_MTU     2290
#define ATH_MIN_MTU     32  

#define	ATH_RXBUF	40		/* number of RX buffers */
#define	ATH_TXBUF	200		/* number of TX buffers */
#define	ATH_TXDESC	1		/* number of descriptors per buffer */
#define ATH_BCBUF       1               /* number of beacon buffers */
#define	ATH_TXMAXTRY	11		/* max number of transmit attempts */
#define	ATH_TXINTR_PERIOD 5		/* max number of batched tx descriptors */

#define ATH_BEACON_AIFS_DEFAULT  1      /* default aifs for ap beacon q */
#define ATH_BEACON_CWMIN_DEFAULT 0      /* default cwmin for ap beacon q */
#define ATH_BEACON_CWMAX_DEFAULT 0      /* default cwmax for ap beacon q */

/*
 * The key cache is used for h/w cipher state and also for
 * tracking station state such as the current tx antenna.
 * We also setup a mapping table between key cache slot indices
 * and station state to short-circuit node lookups on rx.
 * Different parts have different size key caches.  We handle
 * up to ATH_KEYMAX entries (could dynamically allocate state).
 */
#define ATH_KEYMAX      128             /* max key cache size we handle */
#define ATH_KEYBYTES    (ATH_KEYMAX/NBBY)       /* storage space in bytes */

/* driver-specific node state */
struct ath_node {
	struct ieee80211_node an_node;	/* base class */
	u_int8_t	an_tx_mgtrate;	/* h/w rate for management/ctl frames */
	u_int8_t	an_tx_mgtratesp;/* short preamble h/w rate for " " */
	u_int32_t	an_avgrssi;	/* average rssi over all rx frames */
	AR5K_NODE_STATS	an_halstats;	/* rssi statistics used by hal */
	/* variable-length rate control state follows */
};
#define	ATH_NODE(ni)	((struct ath_node *)(ni))
#define	ATH_NODE_CONST(ni)	((const struct ath_node *)(ni))

#define ATH_RSSI_LPF_LEN	10
#define ATH_RSSI_DUMMY_MARKER	0x127
#define ATH_EP_MUL(x, mul)	((x) * (mul))
#define ATH_RSSI_IN(x)		(ATH_EP_MUL((x), AR5K_RSSI_EP_MULTIPLIER))
#define ATH_LPF_RSSI(x, y, len) \
    ((x != ATH_RSSI_DUMMY_MARKER) ? (((x) * ((len) - 1) + (y)) / (len)) : (y))
#define ATH_RSSI_LPF(x, y) do {						\
    if ((y) >= -20)							\
    	x = ATH_LPF_RSSI((x), ATH_RSSI_IN((y)), ATH_RSSI_LPF_LEN);	\
} while (0)

struct ath_buf {
	STAILQ_ENTRY(ath_buf)	bf_list;
	//int			bf_nseg;
	int			bf_flags;	/* tx descriptor flags */
	struct ath_desc		*bf_desc;	/* virtual addr of desc */
	dma_addr_t		bf_daddr;	/* physical addr of desc */
	struct sk_buff		*bf_skb;	/* skbuff for buf */
	dma_addr_t		bf_skbaddr;	/* physical addr of skb data */
	struct ieee80211_node	*bf_node;	/* pointer to the node */
};
typedef STAILQ_HEAD(, ath_buf) ath_bufhead;

/*
 * Data transmit queue state.  One of these exists for each
 * hardware transmit queue.  Packets sent to us from above
 * are assigned to queues based on their priority.  Not all
 * devices support a complete set of hardware transmit queues.
 * For those devices the array sc_ac2q will map multiple
 * priorities to fewer hardware queues (typically all to one
 * hardware queue).
 */
struct ath_txq {
	u_int			axq_qnum;	/* hardware q number */
	u_int			axq_depth;	/* queue depth (stat only) */
	u_int			axq_intrcnt;	/* interrupt count */
	u_int32_t		*axq_link;	/* link ptr in last TX desc */
	STAILQ_HEAD(, ath_buf)	axq_q;		/* transmit queue */
	spinlock_t		axq_lock;	/* lock on q and link */
	/*
	 * State for patching up CTS when bursting.
	 */
	/* virtual address of last buffer */
	struct	ath_buf		*axq_linkbuf;
	/* first desc of last descriptor that contains CTS */
	struct	ath_desc	*axq_lastdsWithCTS;
	/* final desc of the gating desc that determines whether lastdsWithCTS 
	 * has been DMA'ed or not */
	struct	ath_desc	*axq_gatingds;
};

#define KASSERT(exp, msg) do {	\
	if (unlikely(!(exp))) {	\
		printk msg;	\
		BUG();		\
	}			\
} while(0)

#define	ATH_TXQ_LOCK_ASSERT(_tq) \
	KASSERT(spin_is_locked(&(_tq)->axq_lock), ("txq not locked!"))

#define ATH_TXQ_INSERT_TAIL(_tq, _elm, _field) do { \
	STAILQ_INSERT_TAIL(&(_tq)->axq_q, (_elm), _field); \
	(_tq)->axq_depth++; \
} while (0)
#define ATH_TXQ_REMOVE_HEAD(_tq, _field) do { \
	STAILQ_REMOVE_HEAD(&(_tq)->axq_q, _field); \
	(_tq)->axq_depth--; \
} while (0)

struct ath_softc {
	struct net_device	sc_dev;		/* NB: must be first */
	void __iomem		*sc_iobase;	/* address of the device */
	struct net_device	sc_rawdev;	/* live monitor device */
	struct semaphore	sc_lock;	/* dev-level lock */
	struct net_device_stats	sc_devstats;	/* device statistics */
	struct ath_stats	sc_stats;	/* private statistics */
	int devid;
	struct ieee80211com	sc_ic;		/* IEEE 802.11 common */
	int			sc_regdomain;
	int			sc_countrycode;
	int			sc_debug;
	void			(*sc_recv_mgmt)(struct ieee80211com *,
					struct sk_buff *,
					struct ieee80211_node *,
					int, int, u_int32_t);
	int			(*sc_newstate)(struct ieee80211com *,
					enum ieee80211_state, int);
	void 			(*sc_node_free)(struct ieee80211_node *);
	void			*sc_bdev;	/* associated bus device */
	struct ath_desc		*sc_desc;	/* TX/RX descriptors */
	size_t			sc_desc_len;	/* size of TX/RX descriptors */
	u_int16_t		sc_cachelsz;	/* cache line size */
	dma_addr_t		sc_desc_daddr;	/* DMA (physical) address */
	struct ath_hal		*sc_ah;		/* Atheros HAL */
	struct ath_ratectrl	*sc_rc;		/* tx rate control support */
	void			(*sc_setdefantenna)(struct ath_softc *, u_int);
	unsigned int		sc_invalid : 1,	/* disable hardware accesses */
				sc_softled : 1,	/* enable LED gpio status */
				sc_splitmic: 1,	/* split TKIP MIC keys */
				sc_diversity : 1,/* enable rx diversity */
				sc_lockslottime : 1,/* lock slot time value */
				sc_hasveol : 1,	/* tx VEOL support */
				sc_ledstate: 1,	/* LED on/off state */
				sc_blinking: 1,	/* LED blink operation active */
				sc_endblink: 1,	/* finish LED blink operation */
				sc_mcastkey: 1, /* mcast key cache search */
				sc_hasclrkey:1, /* CLR key supported */
				sc_rawdev_enabled : 1;  /* enable sc_rawdev */
#define ATH_MAX_HW_MODES	5
#define ATH_MAX_CHANNELS	64
#define ATH_MAX_RATES		16 
	/* rate tables */
	const struct ar5k_rate_table	*sc_rates[ATH_MAX_HW_MODES];
	const struct ar5k_rate_table	*sc_currates;	/* current rate table */
	enum ieee80211_phymode	sc_curmode;	/* current phy mode */
	u_int16_t		sc_curtxpow;	/* current tx power limit */
	AR5K_CHANNEL		sc_curchan;	/* current h/w channel */
	u_int8_t		sc_rixmap[256];	/* IEEE to h/w rate table ix */
	struct {
		u_int8_t	ieeerate;	/* IEEE rate */
		u_int8_t	rxflags;	/* radiotap rx flags */
		u_int8_t	txflags;	/* radiotap tx flags */
		u_int16_t	ledon;		/* softled on time */
		u_int16_t	ledoff;		/* softled off time */
	} sc_hwmap[32];				/* h/w rate ix mappings */
	u_int8_t		sc_protrix;	/* protection rate index */
	u_int			sc_txantenna;	/* tx antenna (fixed or auto) */
	enum ar5k_int		sc_imask;	/* interrupt mask copy */
	u_int8_t                sc_keymap[ATH_KEYBYTES];/* key use bit map */
	struct ieee80211_node   *sc_keyixmap[ATH_KEYMAX];/* key ix->node map */

	u_int			sc_ledpin;	/* GPIO pin for driving LED */
	u_int			sc_ledon;	/* pin setting for LED on */
	u_int			sc_ledidle;	/* idle polling interval */
	int			sc_ledevent;	/* time of last LED event */
	u_int8_t		sc_rxrate;	/* current rx rate for LED */
	u_int8_t		sc_txrate;	/* current tx rate for LED */
	u_int16_t		sc_ledoff;	/* off time for current blink */
	struct timer_list	sc_ledtimer;	/* led off timer */
	u_int32_t               sc_rxfilter;

	union {
		struct ath_tx_radiotap_header th;
		u_int8_t	pad[64];
	} u_tx_rt;
	int			sc_tx_th_len;
	union {
		struct ath_rx_radiotap_header th;
		u_int8_t	pad[64];
	} u_rx_rt;
	int			sc_rx_th_len;

	int			sc_rxbufsize;	/* rx size based on mtu */
	ath_bufhead		sc_rxbuf;	/* receive buffer */
	u_int32_t		*sc_rxlink;	/* link ptr in last RX desc */
	struct tasklet_struct	sc_rxtq;	/* rx intr tasklet */
	/* reset interrupts tasklet, called on during AR5K_INT_FATAL and 
 	 * AR5K_INT_RXORN interrupts */
	struct tasklet_struct	sc_resettq;
	u_int8_t		sc_defant;	/* current default antenna */
	u_int8_t		sc_rxotherant;	/* rx's on non-default antenna*/

	ath_bufhead		sc_txbuf;	/* transmit buffer */
	spinlock_t		sc_txbuflock;	/* txbuf lock */
	int			sc_tx_timer;	/* transmit timeout */
	u_int			sc_txqsetup;	/* hardware queues setup */
	u_int			sc_txintrperiod;/* tx interrupt batching */
	struct ath_txq		sc_txq[AR5K_NUM_TX_QUEUES];
	struct ath_txq		*sc_ac2q[5];	/* WME AC -> hardware q map */ 
	struct tasklet_struct	sc_txtq;	/* tx intr tasklet */

	ath_bufhead		sc_bbuf;	/* beacon buffers */
	u_int			sc_bhalq;	/* HAL q for outgoing beacons */
	u_int			sc_bmisscount;	/* missed beacon transmits */
	u_int32_t		sc_ant_tx[8];	/* recent tx frames/antenna */
	struct ath_txq		*sc_cabq;	/* tx q for cab frames */
	struct ath_buf		*sc_bufptr;	/* allocated buffer ptr */
	struct ieee80211_beacon_offsets sc_boff;/* dynamic update state */
	struct tasklet_struct	sc_bmisstq;	/* bmiss intr tasklet */
	enum {
		OK,				/* no change needed */
		UPDATE,				/* update pending */
		COMMIT				/* beacon sent, commit change */
	} sc_updateslot;			/* slot time update fsm */

	struct timer_list	sc_cal_ch;	/* calibration timer */
	struct timer_list	sc_scan_ch;	/* AP scan timer */
	struct iw_statistics	sc_iwstats;	/* wireless statistics block */
	struct ieee80211_hw *sc_hw;
	struct ieee80211_hw_mode sc_hw_modes[ATH_MAX_HW_MODES];
	struct ieee80211_rate sc_ieee80211_rates[ATH_MAX_HW_MODES *
		ATH_MAX_RATES];
	int sc_num_modes;	/* number of modes in sc_hw_modes */
	int sc_beacon_interval;	/* beacon interval in units of TU */
	enum ar5k_opmode	sc_opmode; /* current hal operating mode */
};

#define	ATH_TXQ_SETUP(sc, i)	((sc)->sc_txqsetup & (1<<i))

int	ath_attach(u_int16_t, struct net_device *);
int	ath_detach(struct net_device *);
#ifdef CONFIG_PM
void	ath_resume(struct net_device *);
void	ath_suspend(struct net_device *);
#endif
irqreturn_t ath_intr(int, void *); 

/*
 * Prototypes
 */

const char*		ath5k_hw_mac_name(u_int16_t, u_int16_t);
struct ath_hal*		ath5k_hw_init(u_int16_t device,  __iomem void, 
	AR5K_STATUS *);
u_int16_t		ath_hal_computetxtime(struct ath_hal *, 
	const struct ar5k_rate_table *, u_int32_t, u_int16_t, AR5K_BOOL);
ath_hal_mhz2ieee(u_int, u_int);
u_int			ath_hal_ieee2mhz(u_int, u_int);
const char*		ath5k_printver(enum ath5k_srev_type, u_int32_t);
void			ath5k_radar_alert(struct ath_hal *);
ieee80211_regdomain_t	ath5k_regdomain_to_ieee(u_int16_t);
u_int16_t		ath5k_regdomain_from_ieee(ieee80211_regdomain_t);
u_int16_t		ath5k_get_regdomain(struct ath_hal *);
u_int32_t		ath5k_bitswap(u_int32_t, u_int);
inline u_int		ath5k_clocktoh(u_int, AR5K_BOOL);
inline u_int		ath5k_htoclock(u_int, AR5K_BOOL);
void			ath5k_rt_copy(struct ar5k_rate_table *, const struct ar5k_rate_table *);
AR5K_BOOL		ath5k_register_timeout(struct ath_hal *, u_int32_t, u_int32_t,
	u_int32_t, AR5K_BOOL);
int			ath5k_eeprom_init(struct ath_hal *);
int			ath5k_eeprom_read_mac(struct ath_hal *, u_int8_t *);
AR5K_BOOL		ath5k_eeprom_regulation_domain(struct ath_hal *, AR5K_BOOL,
	ieee80211_regdomain_t *);
int			ath5k_eeprom_read_ants(struct ath_hal *, u_int32_t *, u_int);
int			ath5k_eeprom_read_modes(struct ath_hal *, u_int32_t *, u_int);
u_int16_t		ath5k_eeprom_bin2freq(struct ath_hal *, u_int16_t, u_int);
	AR5K_BOOL		ath5k_hw_channel(struct ath_hal *, AR5K_CHANNEL *);
AR5K_BOOL		ath5k_hw_rf5110_channel(struct ath_hal *, AR5K_CHANNEL *);
u_int32_t		ath5k_hw_rf5110_chan2athchan(AR5K_CHANNEL *);
AR5K_BOOL		ath5k_hw_rf5111_channel(struct ath_hal *, AR5K_CHANNEL *);
AR5K_BOOL		ath5k_hw_rf5111_chan2athchan(u_int, struct ath5k_athchan_2ghz *);
AR5K_BOOL		ath5k_hw_rf5112_channel(struct ath_hal *, AR5K_CHANNEL *);
AR5K_BOOL		ath5k_check_channel(struct ath_hal *, u_int16_t, u_int flags);

AR5K_BOOL		ath5k_hw_phy_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel);
AR5K_BOOL		ath5k_hw_rf5110_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel);
AR5K_BOOL		ath5k_hw_rf511x_calibrate(struct ath_hal *hal, AR5K_CHANNEL *channel);

AR5K_BOOL		ath5k_hw_rfregs(struct ath_hal *, AR5K_CHANNEL *, u_int);
AR5K_BOOL		ath5k_hw_rf5111_rfregs(struct ath_hal *, AR5K_CHANNEL *, u_int);
AR5K_BOOL		ath5k_hw_rf5112_rfregs(struct ath_hal *, AR5K_CHANNEL *, u_int);
void	 		ath5k_hw_ar5211_rfregs(struct ath_hal *, AR5K_CHANNEL *, u_int, u_int);
u_int			ath5k_hw_rfregs_op(u_int32_t *, u_int32_t, u_int32_t, u_int32_t,
	u_int32_t, u_int32_t, AR5K_BOOL);
u_int32_t		ath5k_hw_rfregs_gainf_corr(struct ath_hal *);
AR5K_BOOL		ath5k_hw_rfregs_gain_readback(struct ath_hal *);
int32_t			ath5k_hw_rfregs_gain_adjust(struct ath_hal *);
AR5K_BOOL		ath5k_hw_rfgain(struct ath_hal *, u_int, u_int);
void			ath5k_txpower_table(struct ath_hal *, AR5K_CHANNEL *, int16_t);

/*added*/
extern	u_int  ath_hal_getwirelessmodes(struct ath_hal*, AR5K_CTRY_CODE);
void ath_hal_detach(struct ath_hal *ah);

/*
 * Ioctl-related defintions for the Atheros Wireless LAN controller driver.
 */

struct ath_stats {
	u_int32_t	ast_watchdog;	/* device reset by watchdog */
	u_int32_t	ast_hardware;	/* fatal hardware error interrupts */
	u_int32_t	ast_bmiss;	/* beacon miss interrupts */
	u_int32_t	ast_bstuck;	/* beacon stuck interrupts */
	u_int32_t	ast_rxorn;	/* rx overrun interrupts */
	u_int32_t	ast_rxeol;	/* rx eol interrupts */
	u_int32_t	ast_txurn;	/* tx underrun interrupts */
	u_int32_t	ast_mib;	/* mib interrupts */
	u_int32_t	ast_intrcoal;	/* interrupts coalesced */
	u_int32_t	ast_tx_packets;	/* packet sent on the interface */
	u_int32_t	ast_tx_mgmt;	/* management frames transmitted */
	u_int32_t	ast_tx_discard;	/* frames discarded prior to assoc */
	u_int32_t	ast_tx_invalid;	/* frames discarded 'cuz device gone */
	u_int32_t	ast_tx_qstop;	/* output stopped 'cuz no buffer */
	u_int32_t	ast_tx_encap;	/* tx encapsulation failed */
	u_int32_t	ast_tx_nonode;	/* tx failed 'cuz no node */
	u_int32_t	ast_tx_nobuf;	/* tx failed 'cuz no tx buffer (data) */
	u_int32_t	ast_tx_nobufmgt;/* tx failed 'cuz no tx buffer (mgmt)*/
	u_int32_t	ast_tx_linear;	/* tx linearized to cluster */
	u_int32_t	ast_tx_nodata;	/* tx discarded empty frame */
	u_int32_t	ast_tx_busdma;	/* tx failed for dma resrcs */
	u_int32_t	ast_tx_xretries;/* tx failed 'cuz too many retries */
	u_int32_t	ast_tx_fifoerr;	/* tx failed 'cuz FIFO underrun */
	u_int32_t	ast_tx_filtered;/* tx failed 'cuz xmit filtered */
	u_int32_t	ast_tx_shortretry;/* tx on-chip retries (short) */
	u_int32_t	ast_tx_longretry;/* tx on-chip retries (long) */
	u_int32_t	ast_tx_badrate;	/* tx failed 'cuz bogus xmit rate */
	u_int32_t	ast_tx_noack;	/* tx frames with no ack marked */
	u_int32_t	ast_tx_rts;	/* tx frames with rts enabled */
	u_int32_t	ast_tx_cts;	/* tx frames with cts enabled */
	u_int32_t	ast_tx_shortpre;/* tx frames with short preamble */
	u_int32_t	ast_tx_altrate;	/* tx frames with alternate rate */
	u_int32_t	ast_tx_protect;	/* tx frames with protection */
	u_int32_t       ast_tx_ctsburst;/* tx frames with cts and bursting */
	u_int32_t       ast_tx_ctsext;  /* tx frames with cts extension */
	u_int32_t	ast_rx_nobuf;	/* rx setup failed 'cuz no skb */
	u_int32_t	ast_rx_busdma;	/* rx setup failed for dma resrcs */
	u_int32_t	ast_rx_orn;	/* rx failed 'cuz of desc overrun */
	u_int32_t	ast_rx_crcerr;	/* rx failed 'cuz of bad CRC */
	u_int32_t	ast_rx_fifoerr;	/* rx failed 'cuz of FIFO overrun */
	u_int32_t	ast_rx_badcrypt;/* rx failed 'cuz decryption */
	u_int32_t	ast_rx_badmic;	/* rx failed 'cuz MIC failure */
	u_int32_t	ast_rx_phyerr;	/* rx failed 'cuz of PHY err */
	u_int32_t	ast_rx_phy[32];	/* rx PHY error per-code counts */
	u_int32_t	ast_rx_tooshort;/* rx discarded 'cuz frame too short */
	u_int32_t	ast_rx_toobig;	/* rx discarded 'cuz frame too large */
	u_int32_t	ast_rx_packets;	/* packet recv on the interface */
	u_int32_t	ast_rx_mgt;	/* management frames received */
	u_int32_t	ast_rx_ctl;	/* rx discarded 'cuz ctl frame */
	int8_t		ast_tx_rssi;	/* tx rssi of last ack */
	int8_t		ast_rx_rssi;	/* rx rssi from histogram */
	u_int32_t	ast_be_xmit;	/* beacons transmitted */
	u_int32_t	ast_be_nobuf;	/* beacon setup failed 'cuz no skb */
	u_int32_t	ast_per_cal;	/* periodic calibration calls */
	u_int32_t	ast_per_calfail;/* periodic calibration failed */
	u_int32_t	ast_per_rfgain;	/* periodic calibration rfgain reset */
	u_int32_t	ast_rate_calls;	/* rate control checks */
	u_int32_t	ast_rate_raise;	/* rate control raised xmit rate */
	u_int32_t	ast_rate_drop;	/* rate control dropped xmit rate */
	u_int32_t	ast_ant_defswitch;/* rx/default antenna switches */
	u_int32_t	ast_ant_txswitch;/* tx antenna switches */
	u_int32_t	ast_ant_rx[8];	/* rx frames with antenna */
	u_int32_t	ast_ant_tx[8];	/* tx frames with antenna */
};

struct ath_diag {
	char	ad_name[IFNAMSIZ];	/* if name, e.g. "ath0" */
	u_int16_t ad_id;
#define	ATH_DIAG_DYN	0x8000		/* allocate buffer in caller */
#define	ATH_DIAG_IN	0x4000		/* copy in parameters */
#define	ATH_DIAG_OUT	0x0000		/* copy out results (always) */
#define	ATH_DIAG_ID	0x0fff
	u_int16_t ad_in_size;		/* pack to fit, yech */
	void __user *ad_in_data;
	void __user *ad_out_data;
	u_int	ad_out_size;

};

/*
 * Radio capture format.
 */
#define ATH_RX_RADIOTAP_PRESENT (		\
	(1 << IEEE80211_RADIOTAP_FLAGS)		| \
	(1 << IEEE80211_RADIOTAP_RATE)		| \
	(1 << IEEE80211_RADIOTAP_CHANNEL)	| \
	(1 << IEEE80211_RADIOTAP_ANTENNA)	| \
	(1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL)	| \
	0)

struct ath_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
	u_int8_t	wr_flags;		/* XXX for padding */
	u_int8_t	wr_rate;
	__le16		wr_chan_freq;
	__le16		wr_chan_flags;
	u_int8_t	wr_antenna;
	u_int8_t	wr_antsignal;
};

#define ATH_TX_RADIOTAP_PRESENT (		\
	(1 << IEEE80211_RADIOTAP_FLAGS)		| \
	(1 << IEEE80211_RADIOTAP_RATE)		| \
	(1 << IEEE80211_RADIOTAP_DBM_TX_POWER)	| \
	(1 << IEEE80211_RADIOTAP_ANTENNA)	| \
	(1 << IEEE80211_RADIOTAP_TX_FLAGS)	| \
	(1 << IEEE80211_RADIOTAP_RTS_RETRIES)	| \
	(1 << IEEE80211_RADIOTAP_DATA_RETRIES)	| \
	0)

struct ath_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	u_int8_t	wt_flags;		/* XXX for padding */
	u_int8_t	wt_rate;
	u_int8_t	wt_txpower;
	u_int8_t	wt_antenna;
	__le16		wt_tx_flags;
	u_int8_t        wt_rts_retries;
	u_int8_t        wt_data_retries;

};

#define	SIOCGATHSTATS	(SIOCDEVPRIVATE+0)
#define	SIOCGATHDIAG	(SIOCDEVPRIVATE+1)

#define AR5K_REG_SM(_val, _flags)		\
        (((_val) << _flags##_S) & (_flags))
#define AR5K_REG_MS(_val, _flags)		\
        (((_val) & (_flags)) >> _flags##_S)

/* Read from a device register */
static inline u32 ath5k_hw_reg_read(struct ath_hal *hw, u16 reg)
{
	return readl(hw->ah_iomem + reg);
}

/* Write to a device register */
static inline void ath5k_hw_reg_write(struct ath_hal *hw, u16 reg, u32 val)
{
	writel(val, hw->ah_iomem + reg);
}

/* Enable some new additional bits on reg */
inline void ath5k_reg_enable_bits(struct ah_hal, *hw, u16 reg, u32 flags)
{
	u32 bits = ath5k_hw_reg_read(hw, reg) |  flags;
	ath5k_hw_reg_write(hw, reg, bits);
}

/* Check if there is an interrupt waiting to be processed.
 * Return 1 if there is an interrupt for us, or 0 if there is none or if
 * the device has been removed. */
static inline int ath5k_hw_irq_pending(struct ath_hal *hw)
{
	if (ath5k_hw_reg_read(hw, ATH_HW_IRQ_PENDING) == ATH_HW_IRQ_PENDING_TRUE)
		return 1;
	else
		return 0;
}

#define ath5k_hw_tx_desc_setup(hal, x...)\
	do { \
		(hal->ah_version == AR5K_AR5212) ?  \
		ath5k_hw_tx_desc_setup_4word( hal, ##x ) : \
		ath5k_hw_tx_desc_setup_2word( hal, ##x ) \
		\
	} while (0) 

#define ath5k_hw_tx_desc_fill(hal, x...)\
	do { \
		(hal->ah_version == AR5K_AR5212) ?  \
		ath5k_hw_tx_desc_fill_4word( hal, ##x ) : \
		ath5k_hw_tx_desc_fill_2word( hal, ##x ) \
		\
	} while (0) 

#define ath5k_hw_tx_status_proc(hal, x...)\
	do { \
		(hal->ah_version == AR5K_AR5212) ?  \
		ath5k_hw_status_proc_4word( hal, ##x ) : \
		ath5k_hw_status_proc_2word( hal, ##x ) \
		\
	} while (0) 

#define ath5k_hw_rx_desc_proc(hal, x...)\
	do { \
		(hal->ah_version == AR5K_AR5212) ?  \
		ath5k_hw_rx_desc_proc_new( hal, ##x ) : \
		ath5k_hw_rx_desc_proc_old( hal, ##x ) \
		\
	} while (0) 


/*
 * Gain settings
 */

typedef enum {
	AR5K_RFGAIN_INACTIVE = 0,
	AR5K_RFGAIN_READ_REQUESTED,
	AR5K_RFGAIN_NEED_CHANGE,
} AR5K_RFGAIN;

#define AR5K_GAIN_CRN_FIX_BITS_5111		4
#define AR5K_GAIN_CRN_FIX_BITS_5112		7
#define AR5K_GAIN_CRN_MAX_FIX_BITS		AR5K_GAIN_CRN_FIX_BITS_5112
#define AR5K_GAIN_DYN_ADJUST_HI_MARGIN		15
#define AR5K_GAIN_DYN_ADJUST_LO_MARGIN		20
#define AR5K_GAIN_CCK_PROBE_CORR		5
#define AR5K_GAIN_CCK_OFDM_GAIN_DELTA		15
#define AR5K_GAIN_STEP_COUNT			10
#define AR5K_GAIN_PARAM_TX_CLIP			0
#define AR5K_GAIN_PARAM_PD_90			1
#define AR5K_GAIN_PARAM_PD_84			2
#define AR5K_GAIN_PARAM_GAIN_SEL		3
#define AR5K_GAIN_PARAM_MIX_ORN			0
#define AR5K_GAIN_PARAM_PD_138			1
#define AR5K_GAIN_PARAM_PD_137			2
#define AR5K_GAIN_PARAM_PD_136			3
#define AR5K_GAIN_PARAM_PD_132			4
#define AR5K_GAIN_PARAM_PD_131			5
#define AR5K_GAIN_PARAM_PD_130			6
#define AR5K_GAIN_CHECK_ADJUST(_g) 		\
	((_g)->g_current <= (_g)->g_low || (_g)->g_current >= (_g)->g_high)

struct ath5k_gain_opt_step {
	int16_t				gos_param[AR5K_GAIN_CRN_MAX_FIX_BITS];
	int32_t				gos_gain;
};

struct ath5k_gain_opt {
	u_int32_t			go_default;
	u_int32_t			go_steps_count;
	const struct ath5k_gain_opt_step	go_step[AR5K_GAIN_STEP_COUNT];
};

struct ath5k_gain {
	u_int32_t			g_step_idx;
	u_int32_t			g_current;
	u_int32_t			g_target;
	u_int32_t			g_low;
	u_int32_t			g_high;
	u_int32_t			g_f_corr;
	u_int32_t			g_active;
	const struct ath5k_gain_opt_step	*g_step;
};

/*
 * Gain optimization tables...
 */
#define AR5K_RF5111_GAIN_OPT	{		\
	4,					\
	9,					\
	{					\
		{ { 4, 1, 1, 1 }, 6 },		\
		{ { 4, 0, 1, 1 }, 4 },		\
		{ { 3, 1, 1, 1 }, 3 },		\
		{ { 4, 0, 0, 1 }, 1 },		\
		{ { 4, 1, 1, 0 }, 0 },		\
		{ { 4, 0, 1, 0 }, -2 },		\
		{ { 3, 1, 1, 0 }, -3 },		\
		{ { 4, 0, 0, 0 }, -4 },		\
		{ { 2, 1, 1, 0 }, -6 }		\
	}					\
}

#define AR5K_RF5112_GAIN_OPT	{			\
	1,						\
	8,						\
	{						\
		{ { 3, 0, 0, 0, 0, 0, 0 }, 6 },		\
		{ { 2, 0, 0, 0, 0, 0, 0 }, 0 },		\
		{ { 1, 0, 0, 0, 0, 0, 0 }, -3 },	\
		{ { 0, 0, 0, 0, 0, 0, 0 }, -6 },	\
		{ { 0, 1, 1, 0, 0, 0, 0 }, -8 },	\
		{ { 0, 1, 1, 0, 1, 1, 0 }, -10 },	\
		{ { 0, 1, 0, 1, 1, 1, 0 }, -13 },	\
		{ { 0, 1, 0, 1, 1, 0, 1 }, -16 },	\
	}						\
}

/* 
 * HW SPECIFIC STRUCTS
 */

/* Some EEPROM defines */
#define AR5K_EEPROM_EEP_SCALE		100
#define AR5K_EEPROM_EEP_DELTA		10
#define AR5K_EEPROM_N_MODES		3
#define AR5K_EEPROM_N_5GHZ_CHAN		10
#define AR5K_EEPROM_N_2GHZ_CHAN		3
#define AR5K_EEPROM_MAX_CHAN		10
#define AR5K_EEPROM_N_PCDAC		11
#define AR5K_EEPROM_N_TEST_FREQ		8
#define AR5K_EEPROM_N_EDGES		8
#define AR5K_EEPROM_N_INTERCEPTS	11
#define AR5K_EEPROM_FREQ_M(_v)		AR5K_EEPROM_OFF(_v, 0x7f, 0xff)
#define AR5K_EEPROM_PCDAC_M		0x3f
#define AR5K_EEPROM_PCDAC_START		1
#define AR5K_EEPROM_PCDAC_STOP		63
#define AR5K_EEPROM_PCDAC_STEP		1
#define AR5K_EEPROM_NON_EDGE_M		0x40
#define AR5K_EEPROM_CHANNEL_POWER	8
#define AR5K_EEPROM_N_OBDB		4
#define AR5K_EEPROM_OBDB_DIS		0xffff
#define AR5K_EEPROM_CHANNEL_DIS		0xff
#define AR5K_EEPROM_SCALE_OC_DELTA(_x)	(((_x) * 2) / 10)
#define AR5K_EEPROM_N_CTLS(_v)		AR5K_EEPROM_OFF(_v, 16, 32)
#define AR5K_EEPROM_MAX_CTLS		32
#define AR5K_EEPROM_N_XPD_PER_CHANNEL	4
#define AR5K_EEPROM_N_XPD0_POINTS	4
#define AR5K_EEPROM_N_XPD3_POINTS	3
#define AR5K_EEPROM_N_INTERCEPT_10_2GHZ	35
#define AR5K_EEPROM_N_INTERCEPT_10_5GHZ	55
#define AR5K_EEPROM_POWER_M		0x3f
#define AR5K_EEPROM_POWER_MIN		0
#define AR5K_EEPROM_POWER_MAX		3150
#define AR5K_EEPROM_POWER_STEP		50
#define AR5K_EEPROM_POWER_TABLE_SIZE	64
#define AR5K_EEPROM_N_POWER_LOC_11B	4
#define AR5K_EEPROM_N_POWER_LOC_11G	6
#define AR5K_EEPROM_I_GAIN		10
#define AR5K_EEPROM_CCK_OFDM_DELTA	15
#define AR5K_EEPROM_N_IQ_CAL		2

/* Struct to hold EEPROM calibration data */
struct ath5k_eeprom_info {
	u_int16_t	ee_magic;
	u_int16_t	ee_protect;
	u_int16_t	ee_regdomain;
	u_int16_t	ee_version;
	u_int16_t	ee_header;
	u_int16_t	ee_ant_gain;
	u_int16_t	ee_misc0;
	u_int16_t	ee_misc1;
	u_int16_t	ee_cck_ofdm_gain_delta;
	u_int16_t	ee_cck_ofdm_power_delta;
	u_int16_t	ee_scaled_cck_delta;
	u_int16_t	ee_tx_clip;
	u_int16_t	ee_pwd_84;
	u_int16_t	ee_pwd_90;
	u_int16_t	ee_gain_select;

	u_int16_t	ee_i_cal[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_q_cal[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_fixed_bias[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_turbo_max_power[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_xr_power[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_switch_settling[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_ant_tx_rx[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_ant_control[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_PCDAC];
	u_int16_t	ee_ob[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_OBDB];
	u_int16_t	ee_db[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_OBDB];
	u_int16_t	ee_tx_end2xlna_enable[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_tx_end2xpa_disable[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_tx_frm2xpa_enable[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_thr_62[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_xlna_gain[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_xpd[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_x_gain[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_i_gain[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_margin_tx_rx[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_false_detect[AR5K_EEPROM_N_MODES];
	u_int16_t	ee_cal_pier[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_2GHZ_CHAN];
	u_int16_t	ee_channel[AR5K_EEPROM_N_MODES][AR5K_EEPROM_MAX_CHAN];

	u_int16_t	ee_ctls;
	u_int16_t	ee_ctl[AR5K_EEPROM_MAX_CTLS];

	int16_t		ee_noise_floor_thr[AR5K_EEPROM_N_MODES];
	int8_t		ee_adc_desired_size[AR5K_EEPROM_N_MODES];
	int8_t		ee_pga_desired_size[AR5K_EEPROM_N_MODES];
};

/*
 * Internal HW RX/TX descriptor structures
 * (rX: reserved fields possibily used by future versions of the ar5k chipset)
 */

/*
 * Common rx control descriptor
 */
struct ath5k_rx_desc {

	/* RX control word 0 */
	u_int32_t	rx_control_0;

#define AR5K_DESC_RX_CTL0			0x00000000

	/* RX control word 1 */
	u_int32_t	rx_control_1;

#define AR5K_DESC_RX_CTL1_BUF_LEN		0x00000fff
#define AR5K_DESC_RX_CTL1_INTREQ		0x00002000

} __packed;

/*
 * 5210/5211 rx status descriptor
 */
struct ath5k_hw_old_rx_status {

	/*`RX status word 0`*/
	u_int32_t	rx_status_0;

#define AR5K_OLD_RX_DESC_STATUS0_DATA_LEN		0x00000fff
#define AR5K_OLD_RX_DESC_STATUS0_MORE			0x00001000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE		0x00078000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE_S		15
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL		0x07f80000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL_S	19
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA	0x38000000
#define AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA_S	27

	/* RX status word 1 */
	u_int32_t	rx_status_1;

#define AR5K_OLD_RX_DESC_STATUS1_DONE			0x00000001
#define AR5K_OLD_RX_DESC_STATUS1_FRAME_RECEIVE_OK	0x00000002
#define AR5K_OLD_RX_DESC_STATUS1_CRC_ERROR		0x00000004
#define AR5K_OLD_RX_DESC_STATUS1_FIFO_OVERRUN		0x00000008
#define AR5K_OLD_RX_DESC_STATUS1_DECRYPT_CRC_ERROR	0x00000010
#define AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR		0x000000e0
#define AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR_S		5
#define AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_VALID	0x00000100
#define AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX		0x00007e00
#define AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_S		9
#define AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP	0x0fff8000
#define AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP_S	15
#define AR5K_OLD_RX_DESC_STATUS1_KEY_CACHE_MISS		0x10000000

} __packed;

/*
 * 5212 rx status descriptor
 */
struct ath5k_hw_new_rx_status {

	/* RX status word 0 */
	u_int32_t	rx_status_0;

#define AR5K_NEW_RX_DESC_STATUS0_DATA_LEN		0x00000fff
#define AR5K_NEW_RX_DESC_STATUS0_MORE			0x00001000
#define AR5K_NEW_RX_DESC_STATUS0_DECOMP_CRC_ERROR	0x00002000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE		0x000f8000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE_S		15
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL		0x0ff00000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL_S	20
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA	0xf0000000
#define AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA_S	28

	/* RX status word 1 */
	u_int32_t	rx_status_1;

#define AR5K_NEW_RX_DESC_STATUS1_DONE			0x00000001
#define AR5K_NEW_RX_DESC_STATUS1_FRAME_RECEIVE_OK	0x00000002
#define AR5K_NEW_RX_DESC_STATUS1_CRC_ERROR		0x00000004
#define AR5K_NEW_RX_DESC_STATUS1_DECRYPT_CRC_ERROR	0x00000008
#define AR5K_NEW_RX_DESC_STATUS1_PHY_ERROR		0x00000010
#define AR5K_NEW_RX_DESC_STATUS1_MIC_ERROR		0x00000020
#define AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_VALID	0x00000100
#define AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX		0x0000fe00
#define AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_S		9
#define AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP	0x7fff0000
#define AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP_S	16
#define AR5K_NEW_RX_DESC_STATUS1_KEY_CACHE_MISS		0x80000000
} __packed;

/*
 * 5212 rx error descriptor
 */
struct ath5k_hw_rx_error {

	/* RX error word 0 */
	u_int32_t	rx_error_0;

#define AR5K_RX_DESC_ERROR0			0x00000000

	/* RX error word 1 */
	u_int32_t	rx_error_1;

#define AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE	0x0000ff00
#define AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE_S	8

} __packed;

#define AR5K_DESC_RX_PHY_ERROR_NONE		0x00
#define AR5K_DESC_RX_PHY_ERROR_TIMING		0x20
#define AR5K_DESC_RX_PHY_ERROR_PARITY		0x40
#define AR5K_DESC_RX_PHY_ERROR_RATE		0x60
#define AR5K_DESC_RX_PHY_ERROR_LENGTH		0x80
#define AR5K_DESC_RX_PHY_ERROR_64QAM		0xa0
#define AR5K_DESC_RX_PHY_ERROR_SERVICE		0xc0
#define AR5K_DESC_RX_PHY_ERROR_TRANSMITOVR	0xe0

/*
 * 5210/5211 2-word tx control descriptor
 */
struct ath5k_hw_2w_tx_desc {

	/* TX control word 0 */
	u_int32_t	tx_control_0;

#define AR5K_2W_TX_DESC_CTL0_FRAME_LEN		0x00000fff
#define AR5K_2W_TX_DESC_CTL0_HEADER_LEN		0x0003f000 /*[5210 ?]*/
#define AR5K_2W_TX_DESC_CTL0_HEADER_LEN_S	12
#define AR5K_2W_TX_DESC_CTL0_XMIT_RATE		0x003c0000
#define AR5K_2W_TX_DESC_CTL0_XMIT_RATE_S	18
#define AR5K_2W_TX_DESC_CTL0_RTSENA		0x00400000
#define AR5K_2W_TX_DESC_CTL0_CLRDMASK		0x01000000
#define AR5K_2W_TX_DESC_CTL0_LONG_PACKET	0x00800000 /*[5210]*/
#define AR5K_2W_TX_DESC_CTL0_VEOL		0x00800000 /*[5211]*/
#define AR5K_2W_TX_DESC_CTL0_FRAME_TYPE		0x1c000000 /*[5210]*/
#define AR5K_2W_TX_DESC_CTL0_FRAME_TYPE_S	26
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5210	0x02000000
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5211	0x1e000000
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT	(hal->ah_version == AR5K_AR5210 ? \
						AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5210 : \
						AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_5211)
#define AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT_S	25
#define AR5K_2W_TX_DESC_CTL0_INTREQ		0x20000000
#define AR5K_2W_TX_DESC_CTL0_ENCRYPT_KEY_VALID	0x40000000

	/* TX control word 1 */
	u_int32_t	tx_control_1;

#define AR5K_2W_TX_DESC_CTL1_BUF_LEN		0x00000fff
#define AR5K_2W_TX_DESC_CTL1_MORE		0x00001000
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5210	0x0007e000
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5211	0x000fe000
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX	(hal->ah_version == AR5K_AR5210 ? \
						AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5210 : \
						AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_5211)
#define AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_S	13
#define AR5K_2W_TX_DESC_CTL1_FRAME_TYPE		0x00700000 /*[5211]*/
#define AR5K_2W_TX_DESC_CTL1_FRAME_TYPE_S	20
#define AR5K_2W_TX_DESC_CTL1_NOACK		0x00800000 /*[5211]*/
#define AR5K_2W_TX_DESC_CTL1_RTS_DURATION	0xfff80000 /*[5210 ?]*/

} __packed;

#define AR5K_AR5210_TX_DESC_FRAME_TYPE_NORMAL   0x00
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_ATIM     0x04
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_PSPOLL   0x08
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_NO_DELAY 0x0c
#define AR5K_AR5210_TX_DESC_FRAME_TYPE_PIFS     0x10

/*
 * 5212 4-word tx control descriptor
 */
struct ath5k_hw_4w_tx_desc {

	/* TX control word 0 */
	u_int32_t	tx_control_0;

#define AR5K_4W_TX_DESC_CTL0_FRAME_LEN		0x00000fff
#define AR5K_4W_TX_DESC_CTL0_XMIT_POWER		0x003f0000
#define AR5K_4W_TX_DESC_CTL0_XMIT_POWER_S	16
#define AR5K_4W_TX_DESC_CTL0_RTSENA		0x00400000
#define AR5K_4W_TX_DESC_CTL0_VEOL		0x00800000
#define AR5K_4W_TX_DESC_CTL0_CLRDMASK		0x01000000
#define AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT	0x1e000000
#define AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT_S	25
#define AR5K_4W_TX_DESC_CTL0_INTREQ		0x20000000
#define AR5K_4W_TX_DESC_CTL0_ENCRYPT_KEY_VALID	0x40000000
#define AR5K_4W_TX_DESC_CTL0_CTSENA		0x80000000

	/* TX control word 1 */
	u_int32_t	tx_control_1;

#define AR5K_4W_TX_DESC_CTL1_BUF_LEN		0x00000fff
#define AR5K_4W_TX_DESC_CTL1_MORE		0x00001000
#define AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX	0x000fe000
#define AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX_S	13
#define AR5K_4W_TX_DESC_CTL1_FRAME_TYPE		0x00f00000
#define AR5K_4W_TX_DESC_CTL1_FRAME_TYPE_S	20
#define AR5K_4W_TX_DESC_CTL1_NOACK		0x01000000
#define AR5K_4W_TX_DESC_CTL1_COMP_PROC		0x06000000
#define AR5K_4W_TX_DESC_CTL1_COMP_PROC_S	25
#define AR5K_4W_TX_DESC_CTL1_COMP_IV_LEN	0x18000000
#define AR5K_4W_TX_DESC_CTL1_COMP_IV_LEN_S	27
#define AR5K_4W_TX_DESC_CTL1_COMP_ICV_LEN	0x60000000
#define AR5K_4W_TX_DESC_CTL1_COMP_ICV_LEN_S	29

	/* TX control word 2 */
	u_int32_t	tx_control_2;

#define AR5K_4W_TX_DESC_CTL2_RTS_DURATION		0x00007fff
#define AR5K_4W_TX_DESC_CTL2_DURATION_UPDATE_ENABLE	0x00008000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0		0x000f0000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0_S		16
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1		0x00f00000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1_S		20
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2		0x0f000000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2_S		24
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3		0xf0000000
#define AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3_S		28

	/* TX control word 3 */
	u_int32_t	tx_control_3;

#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE0		0x0000001f
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE1		0x000003e0
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE1_S	5
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE2		0x00007c00
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE2_S	10
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE3		0x000f8000
#define AR5K_4W_TX_DESC_CTL3_XMIT_RATE3_S	15
#define AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE	0x01f00000
#define AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE_S	20

} __packed;


/*
 * Common tx status descriptor
 */
struct ath5k_hw_tx_status {

	/* TX status word 0 */
	u_int32_t	tx_status_0;

#define AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK	0x00000001
#define AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES	0x00000002
#define AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN	0x00000004
#define AR5K_DESC_TX_STATUS0_FILTERED		0x00000008
/*???
#define AR5K_DESC_TX_STATUS0_RTS_FAIL_COUNT	0x000000f0
#define AR5K_DESC_TX_STATUS0_RTS_FAIL_COUNT_S	4
*/
#define AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT	0x000000f0
#define AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT_S	4
/*???
#define AR5K_DESC_TX_STATUS0_DATA_FAIL_COUNT	0x00000f00
#define AR5K_DESC_TX_STATUS0_DATA_FAIL_COUNT_S	8
*/
#define AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT	0x00000f00
#define AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT_S	8
#define AR5K_DESC_TX_STATUS0_VIRT_COLL_COUNT	0x0000f000
#define AR5K_DESC_TX_STATUS0_VIRT_COLL_COUNT_S	12
#define AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP	0xffff0000
#define AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP_S	16

	/* TX status word 1 */
	u_int32_t	tx_status_1;

#define AR5K_DESC_TX_STATUS1_DONE		0x00000001
#define AR5K_DESC_TX_STATUS1_SEQ_NUM		0x00001ffe
#define AR5K_DESC_TX_STATUS1_SEQ_NUM_S		1
#define AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH	0x001fe000
#define AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH_S	13
#define AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX	0x00600000
#define AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX_S	21
#define AR5K_DESC_TX_STATUS1_COMP_SUCCESS	0x00800000
#define AR5K_DESC_TX_STATUS1_XMIT_ANTENNA	0x01000000

} __packed;



/*
 * AR5K REGISTER ACCESS
 */

/*Swap RX/TX Descriptor for big endian archs*/
#if defined(__BIG_ENDIAN)
#define AR5K_INIT_CFG	(		\
	AR5K_CFG_SWTD | AR5K_CFG_SWRD	\
)
#else
#define AR5K_INIT_CFG	0x00000000
#endif

#define AR5K_REG_READ(_reg)		ath5k_hw_reg_read(hal, _reg)
#define AR5K_REG_WRITE(_reg, _val)	ath5k_hw_reg_write(hal, _reg, _val)

#define AR5K_REG_SM(_val, _flags)					\
	(((_val) << _flags##_S) & (_flags))

#define AR5K_REG_MS(_val, _flags)					\
	(((_val) & (_flags)) >> _flags##_S)

/* Some registers can hold multiple values of interest. For this
 * reason when we want to write to these registers we must first
 * retrieve the values which we do not want to clear (lets call this 
 * old_data) and then set the register with this and our new_value: 
 * ( old_data | new_value) */
#define AR5K_REG_WRITE_BITS(_reg, _flags, _val)				\
	AR5K_REG_WRITE(_reg, (AR5K_REG_READ(_reg) &~ (_flags)) |	\
	    (((_val) << _flags##_S) & (_flags)))

#define AR5K_REG_MASKED_BITS(_reg, _flags, _mask)			\
	AR5K_REG_WRITE(_reg, (AR5K_REG_READ(_reg) & (_mask)) | (_flags))

#define AR5K_REG_ENABLE_BITS(_reg, _flags)				\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) | (_flags))

#define AR5K_REG_DISABLE_BITS(_reg, _flags)				\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) &~ (_flags))

#define AR5K_PHY_WRITE(_reg, _val)					\
	AR5K_REG_WRITE(hal->ah_phy + ((_reg) << 2), _val)

#define AR5K_PHY_READ(_reg)						\
	AR5K_REG_READ(hal->ah_phy + ((_reg) << 2))

#define AR5K_REG_WAIT(_i)						\
	if (_i % 64)							\
		udelay(1);

#define AR5K_EEPROM_READ(_o, _v)	{				\
	if ((ret = hal->ah_eeprom_read(hal, (_o),			\
		 &(_v))) != 0)						\
		return (ret);						\
}

#define AR5K_EEPROM_READ_HDR(_o, _v)					\
	AR5K_EEPROM_READ(_o, hal->ah_capabilities.cap_eeprom._v);	\

/* Read status of selected queue */
#define AR5K_REG_READ_Q(_reg, _queue)					\
	(AR5K_REG_READ(_reg) & (1 << _queue))				\

#define AR5K_REG_WRITE_Q(_reg, _queue)					\
	AR5K_REG_WRITE(_reg, (1 << _queue))

#define AR5K_Q_ENABLE_BITS(_reg, _queue) do {				\
	_reg |= 1 << _queue;						\
} while (0)

#define AR5K_Q_DISABLE_BITS(_reg, _queue) do {				\
	_reg &= ~(1 << _queue);						\
} while (0)

/*
 * Unaligned little endian access
 */
#define AR5K_LE_READ_2	ath5k_hw_read_unaligned_16
#define AR5K_LE_READ_4	ath5k_hw_read_unaligned_32
#define AR5K_LE_WRITE_2	ath5k_hw_write_unaligned_16
#define AR5K_LE_WRITE_4	ath5k_hw_write_unaligned_32

#define AR5K_LOW_ID(_a)(				\
(_a)[0] | (_a)[1] << 8 | (_a)[2] << 16 | (_a)[3] << 24	\
)

#define AR5K_HIGH_ID(_a)	((_a)[4] | (_a)[5] << 8)

/*
 * Initial register values
 */

/*
 * Common initial register values
 */
#define AR5K_INIT_MODE				CHANNEL_B

#define AR5K_INIT_TX_LATENCY			502
#define AR5K_INIT_USEC				39
#define AR5K_INIT_USEC_TURBO			79
#define AR5K_INIT_USEC_32			31
#define AR5K_INIT_CARR_SENSE_EN			1
#define AR5K_INIT_PROG_IFS			920
#define AR5K_INIT_PROG_IFS_TURBO		960
#define AR5K_INIT_EIFS				3440
#define AR5K_INIT_EIFS_TURBO			6880
#define AR5K_INIT_SLOT_TIME			396
#define AR5K_INIT_SLOT_TIME_TURBO		480
#define AR5K_INIT_ACK_CTS_TIMEOUT		1024
#define AR5K_INIT_ACK_CTS_TIMEOUT_TURBO		0x08000800
#define AR5K_INIT_SIFS				560
#define AR5K_INIT_SIFS_TURBO			480
#define AR5K_INIT_SH_RETRY			10
#define AR5K_INIT_LG_RETRY			AR5K_INIT_SH_RETRY
#define AR5K_INIT_SSH_RETRY			32
#define AR5K_INIT_SLG_RETRY			AR5K_INIT_SSH_RETRY
#define AR5K_INIT_TX_RETRY			10
#define AR5K_INIT_TOPS				8
#define AR5K_INIT_RXNOFRM			8
#define AR5K_INIT_RPGTO				0
#define AR5K_INIT_TXNOFRM			0
#define AR5K_INIT_BEACON_PERIOD			65535
#define AR5K_INIT_TIM_OFFSET			0
#define AR5K_INIT_BEACON_EN			0
#define AR5K_INIT_RESET_TSF			0

#define AR5K_INIT_TRANSMIT_LATENCY		(			\
	(AR5K_INIT_TX_LATENCY << 14) | (AR5K_INIT_USEC_32 << 7) |	\
	(AR5K_INIT_USEC)						\
)
#define AR5K_INIT_TRANSMIT_LATENCY_TURBO	(			\
	(AR5K_INIT_TX_LATENCY << 14) | (AR5K_INIT_USEC_32 << 7) |	\
	(AR5K_INIT_USEC_TURBO)						\
)
#define AR5K_INIT_PROTO_TIME_CNTRL		(			\
	(AR5K_INIT_CARR_SENSE_EN << 26) | (AR5K_INIT_EIFS << 12) |	\
	(AR5K_INIT_PROG_IFS)						\
)
#define AR5K_INIT_PROTO_TIME_CNTRL_TURBO	(			\
	(AR5K_INIT_CARR_SENSE_EN << 26) | (AR5K_INIT_EIFS_TURBO << 12) |\
	(AR5K_INIT_PROG_IFS_TURBO)					\
)
#define AR5K_INIT_BEACON_CONTROL		(			\
	(AR5K_INIT_RESET_TSF << 24) | (AR5K_INIT_BEACON_EN << 23) |	\
	(AR5K_INIT_TIM_OFFSET << 16) | (AR5K_INIT_BEACON_PERIOD)	\
)

/*
 * Non-common initial register values which have to be loaded into the
 * card at boot time and after each reset.
 */

/* Register dumps are done per operation mode */
#define AR5K_INI_VAL_11A		0
#define AR5K_INI_VAL_11A_TURBO		1
#define AR5K_INI_VAL_11B		2
#define AR5K_INI_VAL_11G		3
#define AR5K_INI_VAL_11G_TURBO		4
#define AR5K_INI_VAL_XR			0
#define AR5K_INI_VAL_MAX		5

#define AR5K_INI_PHY_5111		0
#define AR5K_INI_PHY_5112		1
#define AR5K_INI_PHY_511X		1

#define AR5K_RF5111_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS
#define AR5K_RF5112_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS

/* Struct to hold initial RF register values */
struct ath5k_ini_rf {
	u_int8_t	rf_bank;	/* check out ath5kreg.h */
	u_int16_t	rf_register;	/* register address */
	u_int32_t	rf_value[5];	/* register value for
					   different modes (see avove) */
};

/* RF5111 mode-specific init registers */
#define AR5K_RF5111_INI_RF	{						\
	{ 0, 0x989c,								\
	/*     mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00380000, 0x00380000, 0x00380000, 0x00380000, 0x00380000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 0, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x000000c0, 0x00000080, 0x00000080 } },	\
	{ 0, 0x989c,								\
	    { 0x000400f9, 0x000400f9, 0x000400ff, 0x000400fd, 0x000400fd } },	\
	{ 0, 0x98d4,								\
	    { 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 } },	\
	{ 1, 0x98d4,								\
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },	\
	{ 2, 0x98d4,								\
	    { 0x00000010, 0x00000014, 0x00000010, 0x00000010, 0x00000014 } },	\
	{ 3, 0x98d8,								\
	    { 0x00601068, 0x00601068, 0x00601068, 0x00601068, 0x00601068 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x10000000, 0x10000000, 0x10000000, 0x10000000, 0x10000000 } },	\
	{ 6, 0x989c,								\
	    { 0x04000000, 0x04000000, 0x04000000, 0x04000000, 0x04000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x0a000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x003800c0, 0x00380080, 0x023800c0, 0x003800c0, 0x003800c0 } },	\
	{ 6, 0x989c,								\
	    { 0x00020006, 0x00020006, 0x00000006, 0x00020006, 0x00020006 } },	\
	{ 6, 0x989c,								\
	    { 0x00000089, 0x00000089, 0x00000089, 0x00000089, 0x00000089 } },	\
	{ 6, 0x989c,								\
	    { 0x000000a0, 0x000000a0, 0x000000a0, 0x000000a0, 0x000000a0 } },	\
	{ 6, 0x989c,								\
	    { 0x00040007, 0x00040007, 0x00040007, 0x00040007, 0x00040007 } },	\
	{ 6, 0x98d4,								\
	    { 0x0000001a, 0x0000001a, 0x0000001a, 0x0000001a, 0x0000001a } },	\
	{ 7, 0x989c,								\
	    { 0x00000040, 0x00000048, 0x00000040, 0x00000040, 0x00000040 } },	\
	{ 7, 0x989c,								\
	    { 0x00000010, 0x00000010, 0x00000010, 0x00000010, 0x00000010 } },	\
	{ 7, 0x989c,								\
	    { 0x00000008, 0x00000008, 0x00000008, 0x00000008, 0x00000008 } },	\
	{ 7, 0x989c,								\
	    { 0x0000004f, 0x0000004f, 0x0000004f, 0x0000004f, 0x0000004f } },	\
	{ 7, 0x989c,								\
	    { 0x000000f1, 0x000000f1, 0x00000061, 0x000000f1, 0x000000f1 } },	\
	{ 7, 0x989c,								\
	    { 0x0000904f, 0x0000904f, 0x0000904c, 0x0000904f, 0x0000904f } },	\
	{ 7, 0x989c,								\
	    { 0x0000125a, 0x0000125a, 0x0000129a, 0x0000125a, 0x0000125a } },	\
	{ 7, 0x98cc,								\
	    { 0x0000000e, 0x0000000e, 0x0000000f, 0x0000000e, 0x0000000e } },	\
}

/* RF5112 mode-specific init registers */
#define AR5K_RF5112_INI_RF	{						\
	{ 1, 0x98d4,								\
	/*     mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },	\
	{ 2, 0x98d0,								\
	    { 0x03060408, 0x03070408, 0x03060408, 0x03060408, 0x03070408 } },	\
	{ 3, 0x98dc,								\
	    { 0x00a0c0c0, 0x00a0c0c0, 0x00e0c0c0, 0x00e0c0c0, 0x00e0c0c0 } },	\
	{ 6, 0x989c,								\
	    { 0x00a00000, 0x00a00000, 0x00a00000, 0x00a00000, 0x00a00000 } },	\
	{ 6, 0x989c,								\
	    { 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00660000, 0x00660000, 0x00660000, 0x00660000, 0x00660000 } },	\
	{ 6, 0x989c,								\
	    { 0x00db0000, 0x00db0000, 0x00db0000, 0x00db0000, 0x00db0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00f10000, 0x00f10000, 0x00f10000, 0x00f10000, 0x00f10000 } },	\
	{ 6, 0x989c,								\
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },	\
	{ 6, 0x989c,								\
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },	\
	{ 6, 0x989c,								\
	    { 0x00730000, 0x00730000, 0x00730000, 0x00730000, 0x00730000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },	\
	{ 6, 0x989c,								\
	    { 0x008b0000, 0x008b0000, 0x008b0000, 0x008b0000, 0x008b0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00600000, 0x00600000, 0x00600000, 0x00600000, 0x00600000 } },	\
	{ 6, 0x989c,								\
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00840000, 0x00840000, 0x00840000, 0x00840000, 0x00840000 } },	\
	{ 6, 0x989c,								\
	    { 0x00640000, 0x00640000, 0x00640000, 0x00640000, 0x00640000 } },	\
	{ 6, 0x989c,								\
	    { 0x00200000, 0x00200000, 0x00200000, 0x00200000, 0x00200000 } },	\
	{ 6, 0x989c,								\
	    { 0x00240000, 0x00240000, 0x00240000, 0x00240000, 0x00240000 } },	\
	{ 6, 0x989c,								\
	    { 0x00250000, 0x00250000, 0x00250000, 0x00250000, 0x00250000 } },	\
	{ 6, 0x989c,								\
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },	\
	{ 6, 0x989c,								\
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },	\
	{ 6, 0x989c,								\
	    { 0x00510000, 0x00510000, 0x00510000, 0x00510000, 0x00510000 } },	\
	{ 6, 0x989c,								\
	    { 0x1c040000, 0x1c040000, 0x1c040000, 0x1c040000, 0x1c040000 } },	\
	{ 6, 0x989c,								\
	    { 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00a10000, 0x00a10000, 0x00a10000, 0x00a10000, 0x00a10000 } },	\
	{ 6, 0x989c,								\
	    { 0x00400000, 0x00400000, 0x00400000, 0x00400000, 0x00400000 } },	\
	{ 6, 0x989c,								\
	    { 0x03090000, 0x03090000, 0x03090000, 0x03090000, 0x03090000 } },	\
	{ 6, 0x989c,								\
	    { 0x06000000, 0x06000000, 0x06000000, 0x06000000, 0x06000000 } },	\
	{ 6, 0x989c,								\
	    { 0x000000b0, 0x000000b0, 0x000000a8, 0x000000a8, 0x000000a8 } },	\
	{ 6, 0x989c,								\
	    { 0x0000002e, 0x0000002e, 0x0000002e, 0x0000002e, 0x0000002e } },	\
	{ 6, 0x989c,								\
	    { 0x006c4a41, 0x006c4a41, 0x006c4af1, 0x006c4a61, 0x006c4a61 } },	\
	{ 6, 0x989c,								\
	    { 0x0050892a, 0x0050892a, 0x0050892b, 0x0050892b, 0x0050892b } },	\
	{ 6, 0x989c,								\
	    { 0x00842400, 0x00842400, 0x00842400, 0x00842400, 0x00842400 } },	\
	{ 6, 0x989c,								\
	    { 0x00c69200, 0x00c69200, 0x00c69200, 0x00c69200, 0x00c69200 } },	\
	{ 6, 0x98d0,								\
	    { 0x0002000c, 0x0002000c, 0x0002000c, 0x0002000c, 0x0002000c } },	\
	{ 7, 0x989c,								\
	    { 0x00000094, 0x00000094, 0x00000094, 0x00000094, 0x00000094 } },	\
	{ 7, 0x989c,								\
	    { 0x00000091, 0x00000091, 0x00000091, 0x00000091, 0x00000091 } },	\
	{ 7, 0x989c,								\
	    { 0x0000000a, 0x0000000a, 0x00000012, 0x00000012, 0x00000012 } },	\
	{ 7, 0x989c,								\
	    { 0x00000080, 0x00000080, 0x00000080, 0x00000080, 0x00000080 } },	\
	{ 7, 0x989c,								\
	    { 0x000000c1, 0x000000c1, 0x000000c1, 0x000000c1, 0x000000c1 } },	\
	{ 7, 0x989c,								\
	    { 0x00000060, 0x00000060, 0x00000060, 0x00000060, 0x00000060 } },	\
	{ 7, 0x989c,								\
	    { 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0 } },	\
	{ 7, 0x989c,								\
	    { 0x00000022, 0x00000022, 0x00000022, 0x00000022, 0x00000022 } },	\
	{ 7, 0x989c,								\
	    { 0x00000092, 0x00000092, 0x00000092, 0x00000092, 0x00000092 } },	\
	{ 7, 0x989c,								\
	    { 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4 } },	\
	{ 7, 0x989c,								\
	    { 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc } },	\
	{ 7, 0x989c,								\
	    { 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c } },	\
	{ 7, 0x98c4,								\
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },	\
	}

/* RF5112A mode-specific init registers */ 	 
#define AR5K_RF5112A_INI_RF     {						\
	{ 1, 0x98d4,								\
	/*     mode a/XR  mode aTurbo   mode b     mode g     mode gTurbo */	\
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },	\
	{ 2, 0x98d0,								\
	    { 0x03060408, 0x03070408, 0x03060408, 0x03060408, 0x03070408 } },	\
	{ 3, 0x98dc,								\
	    { 0x00a0c0c0, 0x00a0c0c0, 0x00e0c0c0, 0x00e0c0c0, 0x00e0c0c0 } },	\
	{ 6, 0x989c,								\
	    { 0x0f000000, 0x0f000000, 0x0f000000, 0x0f000000, 0x0f000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000 } },	\
	{ 6, 0x989c,								\
	    { 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00010000, 0x00010000, 0x00010000, 0x00010000, 0x00010000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00180000, 0x00180000, 0x00180000, 0x00180000, 0x00180000 } },	\
	{ 6, 0x989c,								\
	    { 0x00600000, 0x00600000, 0x006e0000, 0x006e0000, 0x006e0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00c70000, 0x00c70000, 0x00c70000, 0x00c70000, 0x00c70000 } },	\
	{ 6, 0x989c,								\
	    { 0x004b0000, 0x004b0000, 0x004b0000, 0x004b0000, 0x004b0000 } },	\
	{ 6, 0x989c,								\
	    { 0x04480000, 0x04480000, 0x04480000, 0x04480000, 0x04480000 } },	\
	{ 6, 0x989c,								\
	    { 0x00220000, 0x00220000, 0x00220000, 0x00220000, 0x00220000 } },	\
	{ 6, 0x989c,								\
	    { 0x00e40000, 0x00e40000, 0x00e40000, 0x00e40000, 0x00e40000 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x00fc0000, 0x00fc0000, 0x00fc0000, 0x00fc0000, 0x00fc0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },	\
	{ 6, 0x989c,								\
	    { 0x043f0000, 0x043f0000, 0x043f0000, 0x043f0000, 0x043f0000 } },	\
	{ 6, 0x989c,								\
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00190000, 0x00190000, 0x00190000, 0x00190000, 0x00190000 } },	\
	{ 6, 0x989c,								\
	    { 0x00240000, 0x00240000, 0x00240000, 0x00240000, 0x00240000 } },	\
	{ 6, 0x989c,								\
	    { 0x00b40000, 0x00b40000, 0x00b40000, 0x00b40000, 0x00b40000 } },	\
	{ 6, 0x989c,								\
	    { 0x00990000, 0x00990000, 0x00990000, 0x00990000, 0x00990000 } },	\
	{ 6, 0x989c,								\
	    { 0x00500000, 0x00500000, 0x00500000, 0x00500000, 0x00500000 } },	\
	{ 6, 0x989c,								\
	    { 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000 } },	\
	{ 6, 0x989c,								\
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },	\
	{ 6, 0x989c,								\
	    { 0xc0320000, 0xc0320000, 0xc0320000, 0xc0320000, 0xc0320000 } },	\
	{ 6, 0x989c,								\
	    { 0x01740000, 0x01740000, 0x01740000, 0x01740000, 0x01740000 } },	\
	{ 6, 0x989c,								\
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },	\
	{ 6, 0x989c,								\
	    { 0x86280000, 0x86280000, 0x86280000, 0x86280000, 0x86280000 } },	\
	{ 6, 0x989c,								\
	    { 0x31840000, 0x31840000, 0x31840000, 0x31840000, 0x31840000 } },	\
	{ 6, 0x989c,								\
	    { 0x00020080, 0x00020080, 0x00020080, 0x00020080, 0x00020080 } },	\
	{ 6, 0x989c,								\
	    { 0x00080009, 0x00080009, 0x00080009, 0x00080009, 0x00080009 } },	\
	{ 6, 0x989c,								\
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },	\
	{ 6, 0x989c,								\
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },	\
	{ 6, 0x989c,								\
	    { 0x000000b2, 0x000000b2, 0x000000b2, 0x000000b2, 0x000000b2 } },	\
	{ 6, 0x989c,								\
	    { 0x00b02084, 0x00b02084, 0x00b02084, 0x00b02084, 0x00b02084 } },	\
	{ 6, 0x989c,								\
	    { 0x004125a4, 0x004125a4, 0x004125a4, 0x004125a4, 0x004125a4 } },	\
	{ 6, 0x989c,								\
	    { 0x00119220, 0x00119220, 0x00119220, 0x00119220, 0x00119220 } },	\
	{ 6, 0x989c,								\
	    { 0x001a4800, 0x001a4800, 0x001a4800, 0x001a4800, 0x001a4800 } },	\
	{ 6, 0x98d8,								\
	    { 0x000b0230, 0x000b0230, 0x000b0230, 0x000b0230, 0x000b0230 } },	\
	{ 7, 0x989c,								\
	    { 0x00000094, 0x00000094, 0x00000094, 0x00000094, 0x00000094 } },	\
	{ 7, 0x989c,								\
	    { 0x00000091, 0x00000091, 0x00000091, 0x00000091, 0x00000091 } },	\
	{ 7, 0x989c,								\
	    { 0x00000012, 0x00000012, 0x00000012, 0x00000012, 0x00000012 } },	\
	{ 7, 0x989c,								\
	    { 0x00000080, 0x00000080, 0x00000080, 0x00000080, 0x00000080 } },	\
	{ 7, 0x989c,								\
	    { 0x000000d9, 0x000000d9, 0x000000d9, 0x000000d9, 0x000000d9 } },	\
	{ 7, 0x989c,								\
	    { 0x00000060, 0x00000060, 0x00000060, 0x00000060, 0x00000060 } },	\
	{ 7, 0x989c,								\
	    { 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0 } },	\
	{ 7, 0x989c,								\
	    { 0x000000a2, 0x000000a2, 0x000000a2, 0x000000a2, 0x000000a2 } },	\
	{ 7, 0x989c,								\
	    { 0x00000052, 0x00000052, 0x00000052, 0x00000052, 0x00000052 } },	\
	{ 7, 0x989c,								\
	    { 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4 } },	\
	{ 7, 0x989c,								\
	    { 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc } },	\
	{ 7, 0x989c,								\
	    { 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c } },	\
	{ 7, 0x98c4,								\
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },	\
}

/*
 * Mode-specific RF Gain registers
 */
struct ath5k_ini_rfgain {
	u_int16_t	rfg_register;		/* RF Gain register address */
	u_int32_t	rfg_value[2][2];	/* [phy (see above)][freq (below)] */

#define AR5K_INI_RFGAIN_5GHZ	0
#define AR5K_INI_RFGAIN_2GHZ	1
};

#define AR5K_INI_RFGAIN	{							\
	{ 0x9a00, {								\
		/* 5111 5Ghz  5111 2Ghz	       5112 5Ghz   5112 2Ghz */		\
		{ 0x000001a9, 0x00000000 }, { 0x00000007, 0x00000007 } } },	\
	{ 0x9a04, {								\
		{ 0x000001e9, 0x00000040 }, { 0x00000047, 0x00000047 } } },	\
	{ 0x9a08, {								\
		{ 0x00000029, 0x00000080 }, { 0x00000087, 0x00000087 } } },	\
	{ 0x9a0c, {								\
		{ 0x00000069, 0x00000150 }, { 0x000001a0, 0x000001a0 } } },	\
	{ 0x9a10, {								\
		{ 0x00000199, 0x00000190 }, { 0x000001e0, 0x000001e0 } } },	\
	{ 0x9a14, {								\
		{ 0x000001d9, 0x000001d0 }, { 0x00000020, 0x00000020 } } },	\
	{ 0x9a18, {								\
		{ 0x00000019, 0x00000010 }, { 0x00000060, 0x00000060 } } },	\
	{ 0x9a1c, {								\
		{ 0x00000059, 0x00000044 }, { 0x000001a1, 0x000001a1 } } },	\
	{ 0x9a20, {								\
		{ 0x00000099, 0x00000084 }, { 0x000001e1, 0x000001e1 } } },	\
	{ 0x9a24, {								\
		{ 0x000001a5, 0x00000148 }, { 0x00000021, 0x00000021 } } },	\
	{ 0x9a28, {								\
		{ 0x000001e5, 0x00000188 }, { 0x00000061, 0x00000061 } } },	\
	{ 0x9a2c, {								\
		{ 0x00000025, 0x000001c8 }, { 0x00000162, 0x00000162 } } },	\
	{ 0x9a30, {								\
		{ 0x000001c8, 0x00000014 }, { 0x000001a2, 0x000001a2 } } },	\
	{ 0x9a34, {								\
		{ 0x00000008, 0x00000042 }, { 0x000001e2, 0x000001e2 } } },	\
	{ 0x9a38, {								\
		{ 0x00000048, 0x00000082 }, { 0x00000022, 0x00000022 } } },	\
	{ 0x9a3c, {								\
		{ 0x00000088, 0x00000178 }, { 0x00000062, 0x00000062 } } },	\
	{ 0x9a40, {								\
		{ 0x00000198, 0x000001b8 }, { 0x00000163, 0x00000163 } } },	\
	{ 0x9a44, {								\
		{ 0x000001d8, 0x000001f8 }, { 0x000001a3, 0x000001a3 } } },	\
	{ 0x9a48, {								\
		{ 0x00000018, 0x00000012 }, { 0x000001e3, 0x000001e3 } } },	\
	{ 0x9a4c, {								\
		{ 0x00000058, 0x00000052 }, { 0x00000023, 0x00000023 } } },	\
	{ 0x9a50, {								\
		{ 0x00000098, 0x00000092 }, { 0x00000063, 0x00000063 } } },	\
	{ 0x9a54, {								\
		{ 0x000001a4, 0x0000017c }, { 0x00000184, 0x00000184 } } },	\
	{ 0x9a58, {								\
		{ 0x000001e4, 0x000001bc }, { 0x000001c4, 0x000001c4 } } },	\
	{ 0x9a5c, {								\
		{ 0x00000024, 0x000001fc }, { 0x00000004, 0x00000004 } } },	\
	{ 0x9a60, {								\
		{ 0x00000064, 0x0000000a }, { 0x000001ea, 0x0000000b } } },	\
	{ 0x9a64, {								\
		{ 0x000000a4, 0x0000004a }, { 0x0000002a, 0x0000004b } } },	\
	{ 0x9a68, {								\
		{ 0x000000e4, 0x0000008a }, { 0x0000006a, 0x0000008b } } },	\
	{ 0x9a6c, {								\
		{ 0x0000010a, 0x0000015a }, { 0x000000aa, 0x000001ac } } },	\
	{ 0x9a70, {								\
		{ 0x0000014a, 0x0000019a }, { 0x000001ab, 0x000001ec } } },	\
	{ 0x9a74, {								\
		{ 0x0000018a, 0x000001da }, { 0x000001eb, 0x0000002c } } },	\
	{ 0x9a78, {								\
		{ 0x000001ca, 0x0000000e }, { 0x0000002b, 0x00000012 } } },	\
	{ 0x9a7c, {								\
		{ 0x0000000a, 0x0000004e }, { 0x0000006b, 0x00000052 } } },	\
	{ 0x9a80, {								\
		{ 0x0000004a, 0x0000008e }, { 0x000000ab, 0x00000092 } } },	\
	{ 0x9a84, {								\
		{ 0x0000008a, 0x0000015e }, { 0x000001ac, 0x00000193 } } },	\
	{ 0x9a88, {								\
		{ 0x000001ba, 0x0000019e }, { 0x000001ec, 0x000001d3 } } },	\
	{ 0x9a8c, {								\
		{ 0x000001fa, 0x000001de }, { 0x0000002c, 0x00000013 } } },	\
	{ 0x9a90, {								\
		{ 0x0000003a, 0x00000009 }, { 0x0000003a, 0x00000053 } } },	\
	{ 0x9a94, {								\
		{ 0x0000007a, 0x00000049 }, { 0x0000007a, 0x00000093 } } },	\
	{ 0x9a98, {								\
		{ 0x00000186, 0x00000089 }, { 0x000000ba, 0x00000194 } } },	\
	{ 0x9a9c, {								\
		{ 0x000001c6, 0x00000179 }, { 0x000001bb, 0x000001d4 } } },	\
	{ 0x9aa0, {								\
		{ 0x00000006, 0x000001b9 }, { 0x000001fb, 0x00000014 } } },	\
	{ 0x9aa4, {								\
		{ 0x00000046, 0x000001f9 }, { 0x0000003b, 0x0000003a } } },	\
	{ 0x9aa8, {								\
		{ 0x00000086, 0x00000039 }, { 0x0000007b, 0x0000007a } } },	\
	{ 0x9aac, {								\
		{ 0x000000c6, 0x00000079 }, { 0x000000bb, 0x000000ba } } },	\
	{ 0x9ab0, {								\
		{ 0x000000c6, 0x000000b9 }, { 0x000001bc, 0x000001bb } } },	\
	{ 0x9ab4, {								\
		{ 0x000000c6, 0x000001bd }, { 0x000001fc, 0x000001fb } } },	\
	{ 0x9ab8, {								\
		{ 0x000000c6, 0x000001fd }, { 0x0000003c, 0x0000003b } } },	\
	{ 0x9abc, {								\
		{ 0x000000c6, 0x0000003d }, { 0x0000007c, 0x0000007b } } },	\
	{ 0x9ac0, {								\
		{ 0x000000c6, 0x0000007d }, { 0x000000bc, 0x000000bb } } },	\
	{ 0x9ac4, {								\
		{ 0x000000c6, 0x000000bd }, { 0x000000fc, 0x000001bc } } },	\
	{ 0x9ac8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000001fc } } },	\
	{ 0x9acc, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x0000003c } } },	\
	{ 0x9ad0, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x0000007c } } },	\
	{ 0x9ad4, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000bc } } },	\
	{ 0x9ad8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9adc, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9ae0, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9ae4, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9ae8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9aec, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9af0, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9af4, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9af8, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
	{ 0x9afc, {								\
		{ 0x000000c6, 0x000000fd }, { 0x000000fc, 0x000000fc } } },	\
}


/*
 * Mode-independet initial register writes
 */

struct ath5k_ini {
	u_int16_t	ini_register;
	u_int32_t	ini_value;

	enum {
		AR5K_INI_WRITE = 0,	/* Default */
		AR5K_INI_READ = 1,	/* Cleared on read */
	} ini_mode;
};

/* Initial register settings for AR5210 */
#define AR5K_AR5210_INI {		\
	/* PCU and MAC registers */	\
	{ AR5K_NOQCU_TXDP0,	0 },	\
	{ AR5K_NOQCU_TXDP1,	0 },	\
	{ AR5K_RXDP,		0 },	\
	{ AR5K_CR,		0 },	\
	{ AR5K_ISR,		0, AR5K_INI_READ },	\
	{ AR5K_IMR,		0 },	\
	{ AR5K_IER,		AR5K_IER_DISABLE },	\
	{ AR5K_BSR,		0, AR5K_INI_READ },	\
	{ AR5K_TXCFG,		AR5K_DMASIZE_128B },	\
	{ AR5K_RXCFG,		AR5K_DMASIZE_128B },	\
	{ AR5K_CFG,		AR5K_INIT_CFG },	\
	{ AR5K_TOPS,		AR5K_INIT_TOPS },	\
	{ AR5K_RXNOFRM,		AR5K_INIT_RXNOFRM },	\
	{ AR5K_RPGTO,		AR5K_INIT_RPGTO },	\
	{ AR5K_TXNOFRM,		AR5K_INIT_TXNOFRM },	\
	{ AR5K_SFR,		0 },	\
	{ AR5K_MIBC,		0 },	\
	{ AR5K_MISC,		0 },	\
	{ AR5K_RX_FILTER_5210,	0 },	\
	{ AR5K_MCAST_FILTER0_5210, 0 },	\
	{ AR5K_MCAST_FILTER1_5210, 0 },	\
	{ AR5K_TX_MASK0,	0 },	\
	{ AR5K_TX_MASK1,	0 },	\
	{ AR5K_CLR_TMASK,	0 },	\
	{ AR5K_TRIG_LVL,	AR5K_TUNE_MIN_TX_FIFO_THRES },	\
	{ AR5K_DIAG_SW_5210,	0 },	\
	{ AR5K_RSSI_THR,	AR5K_TUNE_RSSI_THRES },	\
	{ AR5K_TSF_L32_5210,	0 },	\
	{ AR5K_TIMER0_5210,	0 },	\
	{ AR5K_TIMER1_5210,	0xffffffff },	\
	{ AR5K_TIMER2_5210,	0xffffffff },	\
	{ AR5K_TIMER3_5210,	1 },	\
	{ AR5K_CFP_DUR_5210,	0 },	\
	{ AR5K_CFP_PERIOD_5210,	0 },	\
	/* PHY registers */		\
	{ AR5K_PHY(0),	0x00000047 },	\
	{ AR5K_PHY_AGC,	0x00000000 },	\
	{ AR5K_PHY(3),	0x09848ea6 },	\
	{ AR5K_PHY(4),	0x3d32e000 },	\
	{ AR5K_PHY(5),	0x0000076b },	\
	{ AR5K_PHY_ACT,	AR5K_PHY_ACT_DISABLE },	\
	{ AR5K_PHY(8),	0x02020200 },	\
	{ AR5K_PHY(9),	0x00000e0e },	\
	{ AR5K_PHY(10),	0x0a020201 },	\
	{ AR5K_PHY(11),	0x00036ffc },	\
	{ AR5K_PHY(12),	0x00000000 },	\
	{ AR5K_PHY(13),	0x00000e0e },	\
	{ AR5K_PHY(14),	0x00000007 },	\
	{ AR5K_PHY(15),	0x00020100 },	\
	{ AR5K_PHY(16),	0x89630000 },	\
	{ AR5K_PHY(17),	0x1372169c },	\
	{ AR5K_PHY(18),	0x0018b633 },	\
	{ AR5K_PHY(19),	0x1284613c },	\
	{ AR5K_PHY(20),	0x0de8b8e0 },	\
	{ AR5K_PHY(21),	0x00074859 },	\
	{ AR5K_PHY(22),	0x7e80beba },	\
	{ AR5K_PHY(23),	0x313a665e },	\
	{ AR5K_PHY_AGCCTL, 0x00001d08 },\
	{ AR5K_PHY(25),	0x0001ce00 },	\
	{ AR5K_PHY(26),	0x409a4190 },	\
	{ AR5K_PHY(28),	0x0000000f },	\
	{ AR5K_PHY(29),	0x00000080 },	\
	{ AR5K_PHY(30),	0x00000004 },	\
	{ AR5K_PHY(31),	0x00000018 }, 	/* 0x987c */	\
	{ AR5K_PHY(64),	0x00000000 }, 	/* 0x9900 */	\
	{ AR5K_PHY(65),	0x00000000 },	\
	{ AR5K_PHY(66),	0x00000000 },	\
	{ AR5K_PHY(67),	0x00800000 },	\
	{ AR5K_PHY(68),	0x00000003 },	\
	/* BB gain table (64bytes) */	\
	{ AR5K_BB_GAIN(0), 0x00000000 },	\
	{ AR5K_BB_GAIN(1), 0x00000020 },	\
	{ AR5K_BB_GAIN(2), 0x00000010 },	\
	{ AR5K_BB_GAIN(3), 0x00000030 },	\
	{ AR5K_BB_GAIN(4), 0x00000008 },	\
	{ AR5K_BB_GAIN(5), 0x00000028 },	\
	{ AR5K_BB_GAIN(6), 0x00000028 },	\
	{ AR5K_BB_GAIN(7), 0x00000004 },	\
	{ AR5K_BB_GAIN(8), 0x00000024 },	\
	{ AR5K_BB_GAIN(9), 0x00000014 },	\
	{ AR5K_BB_GAIN(10), 0x00000034 },	\
	{ AR5K_BB_GAIN(11), 0x0000000c },	\
	{ AR5K_BB_GAIN(12), 0x0000002c },	\
	{ AR5K_BB_GAIN(13), 0x00000002 },	\
	{ AR5K_BB_GAIN(14), 0x00000022 },	\
	{ AR5K_BB_GAIN(15), 0x00000012 },	\
	{ AR5K_BB_GAIN(16), 0x00000032 },	\
	{ AR5K_BB_GAIN(17), 0x0000000a },	\
	{ AR5K_BB_GAIN(18), 0x0000002a },	\
	{ AR5K_BB_GAIN(19), 0x00000001 },	\
	{ AR5K_BB_GAIN(20), 0x00000021 },	\
	{ AR5K_BB_GAIN(21), 0x00000011 },	\
	{ AR5K_BB_GAIN(22), 0x00000031 },	\
	{ AR5K_BB_GAIN(23), 0x00000009 },	\
	{ AR5K_BB_GAIN(24), 0x00000029 },	\
	{ AR5K_BB_GAIN(25), 0x00000005 },	\
	{ AR5K_BB_GAIN(26), 0x00000025 },	\
	{ AR5K_BB_GAIN(27), 0x00000015 },	\
	{ AR5K_BB_GAIN(28), 0x00000035 },	\
	{ AR5K_BB_GAIN(29), 0x0000000d },	\
	{ AR5K_BB_GAIN(30), 0x0000002d },	\
	{ AR5K_BB_GAIN(31), 0x00000003 },	\
	{ AR5K_BB_GAIN(32), 0x00000023 },	\
	{ AR5K_BB_GAIN(33), 0x00000013 },	\
	{ AR5K_BB_GAIN(34), 0x00000033 },	\
	{ AR5K_BB_GAIN(35), 0x0000000b },	\
	{ AR5K_BB_GAIN(36), 0x0000002b },	\
	{ AR5K_BB_GAIN(37), 0x00000007 },	\
	{ AR5K_BB_GAIN(38), 0x00000027 },	\
	{ AR5K_BB_GAIN(39), 0x00000017 },	\
	{ AR5K_BB_GAIN(40), 0x00000037 },	\
	{ AR5K_BB_GAIN(41), 0x0000000f },	\
	{ AR5K_BB_GAIN(42), 0x0000002f },	\
	{ AR5K_BB_GAIN(43), 0x0000002f },	\
	{ AR5K_BB_GAIN(44), 0x0000002f },	\
	{ AR5K_BB_GAIN(45), 0x0000002f },	\
	{ AR5K_BB_GAIN(46), 0x0000002f },	\
	{ AR5K_BB_GAIN(47), 0x0000002f },	\
	{ AR5K_BB_GAIN(48), 0x0000002f },	\
	{ AR5K_BB_GAIN(49), 0x0000002f },	\
	{ AR5K_BB_GAIN(50), 0x0000002f },	\
	{ AR5K_BB_GAIN(51), 0x0000002f },	\
	{ AR5K_BB_GAIN(52), 0x0000002f },	\
	{ AR5K_BB_GAIN(53), 0x0000002f },	\
	{ AR5K_BB_GAIN(54), 0x0000002f },	\
	{ AR5K_BB_GAIN(55), 0x0000002f },	\
	{ AR5K_BB_GAIN(56), 0x0000002f },	\
	{ AR5K_BB_GAIN(57), 0x0000002f },	\
	{ AR5K_BB_GAIN(58), 0x0000002f },	\
	{ AR5K_BB_GAIN(59), 0x0000002f },	\
	{ AR5K_BB_GAIN(60), 0x0000002f },	\
	{ AR5K_BB_GAIN(61), 0x0000002f },	\
	{ AR5K_BB_GAIN(62), 0x0000002f },	\
	{ AR5K_BB_GAIN(63), 0x0000002f },	\
	/* RF gain table (64btes) */		\
	{ AR5K_RF_GAIN(0), 0x0000001d },	\
	{ AR5K_RF_GAIN(1), 0x0000005d },	\
	{ AR5K_RF_GAIN(2), 0x0000009d },	\
	{ AR5K_RF_GAIN(3), 0x000000dd },	\
	{ AR5K_RF_GAIN(4), 0x0000011d },	\
	{ AR5K_RF_GAIN(5), 0x00000021 },	\
	{ AR5K_RF_GAIN(6), 0x00000061 },	\
	{ AR5K_RF_GAIN(7), 0x000000a1 },	\
	{ AR5K_RF_GAIN(8), 0x000000e1 },	\
	{ AR5K_RF_GAIN(9), 0x00000031 },	\
	{ AR5K_RF_GAIN(10), 0x00000071 },	\
	{ AR5K_RF_GAIN(11), 0x000000b1 },	\
	{ AR5K_RF_GAIN(12), 0x0000001c },	\
	{ AR5K_RF_GAIN(13), 0x0000005c },	\
	{ AR5K_RF_GAIN(14), 0x00000029 },	\
	{ AR5K_RF_GAIN(15), 0x00000069 },	\
	{ AR5K_RF_GAIN(16), 0x000000a9 },	\
	{ AR5K_RF_GAIN(17), 0x00000020 },	\
	{ AR5K_RF_GAIN(18), 0x00000019 },	\
	{ AR5K_RF_GAIN(19), 0x00000059 },	\
	{ AR5K_RF_GAIN(20), 0x00000099 },	\
	{ AR5K_RF_GAIN(21), 0x00000030 },	\
	{ AR5K_RF_GAIN(22), 0x00000005 },	\
	{ AR5K_RF_GAIN(23), 0x00000025 },	\
	{ AR5K_RF_GAIN(24), 0x00000065 },	\
	{ AR5K_RF_GAIN(25), 0x000000a5 },	\
	{ AR5K_RF_GAIN(26), 0x00000028 },	\
	{ AR5K_RF_GAIN(27), 0x00000068 },	\
	{ AR5K_RF_GAIN(28), 0x0000001f },	\
	{ AR5K_RF_GAIN(29), 0x0000001e },	\
	{ AR5K_RF_GAIN(30), 0x00000018 },	\
	{ AR5K_RF_GAIN(31), 0x00000058 },	\
	{ AR5K_RF_GAIN(32), 0x00000098 },	\
	{ AR5K_RF_GAIN(33), 0x00000003 },	\
	{ AR5K_RF_GAIN(34), 0x00000004 },	\
	{ AR5K_RF_GAIN(35), 0x00000044 },	\
	{ AR5K_RF_GAIN(36), 0x00000084 },	\
	{ AR5K_RF_GAIN(37), 0x00000013 },	\
	{ AR5K_RF_GAIN(38), 0x00000012 },	\
	{ AR5K_RF_GAIN(39), 0x00000052 },	\
	{ AR5K_RF_GAIN(40), 0x00000092 },	\
	{ AR5K_RF_GAIN(41), 0x000000d2 },	\
	{ AR5K_RF_GAIN(42), 0x0000002b },	\
	{ AR5K_RF_GAIN(43), 0x0000002a },	\
	{ AR5K_RF_GAIN(44), 0x0000006a },	\
	{ AR5K_RF_GAIN(45), 0x000000aa },	\
	{ AR5K_RF_GAIN(46), 0x0000001b },	\
	{ AR5K_RF_GAIN(47), 0x0000001a },	\
	{ AR5K_RF_GAIN(48), 0x0000005a },	\
	{ AR5K_RF_GAIN(49), 0x0000009a },	\
	{ AR5K_RF_GAIN(50), 0x000000da },	\
	{ AR5K_RF_GAIN(51), 0x00000006 },	\
	{ AR5K_RF_GAIN(52), 0x00000006 },	\
	{ AR5K_RF_GAIN(53), 0x00000006 },	\
	{ AR5K_RF_GAIN(54), 0x00000006 },	\
	{ AR5K_RF_GAIN(55), 0x00000006 },	\
	{ AR5K_RF_GAIN(56), 0x00000006 },	\
	{ AR5K_RF_GAIN(57), 0x00000006 },	\
	{ AR5K_RF_GAIN(58), 0x00000006 },	\
	{ AR5K_RF_GAIN(59), 0x00000006 },	\
	{ AR5K_RF_GAIN(60), 0x00000006 },	\
	{ AR5K_RF_GAIN(61), 0x00000006 },	\
	{ AR5K_RF_GAIN(62), 0x00000006 },	\
	{ AR5K_RF_GAIN(63), 0x00000006 },	\
	/* PHY activation */			\
	{ AR5K_PHY(53), 0x00000020 },		\
	{ AR5K_PHY(51), 0x00000004 },		\
	{ AR5K_PHY(50), 0x00060106 },		\
	{ AR5K_PHY(39), 0x0000006d },		\
	{ AR5K_PHY(48), 0x00000000 },		\
	{ AR5K_PHY(52), 0x00000014 },		\
	{ AR5K_PHY_ACT, AR5K_PHY_ACT_ENABLE },	\
}


/* Initial register settings for AR5211 */
#define AR5K_AR5211_INI {			\
	{ AR5K_RXDP,		0x00000000 },	\
	{ AR5K_RTSD0,		0x84849c9c },	\
	{ AR5K_RTSD1,		0x7c7c7c7c },	\
	{ AR5K_RXCFG,		0x00000005 },	\
	{ AR5K_MIBC,		0x00000000 },	\
	{ AR5K_TOPS,		0x00000008 },	\
	{ AR5K_RXNOFRM,		0x00000008 },	\
	{ AR5K_TXNOFRM,		0x00000010 },	\
	{ AR5K_RPGTO,		0x00000000 },	\
	{ AR5K_RFCNT,		0x0000001f },	\
	{ AR5K_QUEUE_TXDP(0),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(1),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(2),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(3),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(4),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(5),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(6),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(7),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(8),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(9),	0x00000000 },	\
	{ AR5K_DCU_FP,		0x00000000 },	\
	{ AR5K_STA_ID1,		0x00000000 },	\
	{ AR5K_BSS_ID0,		0x00000000 },	\
	{ AR5K_BSS_ID1,		0x00000000 },	\
	{ AR5K_RSSI_THR,	0x00000000 },	\
	{ AR5K_CFP_PERIOD_5211,	0x00000000 },	\
	{ AR5K_TIMER0_5211,	0x00000030 },	\
	{ AR5K_TIMER1_5211,	0x0007ffff },	\
	{ AR5K_TIMER2_5211,	0x01ffffff },	\
	{ AR5K_TIMER3_5211,	0x00000031 },	\
	{ AR5K_CFP_DUR_5211,	0x00000000 },	\
	{ AR5K_RX_FILTER_5211,	0x00000000 },	\
	{ AR5K_MCAST_FILTER0_5211, 0x00000000 },\
	{ AR5K_MCAST_FILTER1_5211, 0x00000002 },\
	{ AR5K_DIAG_SW_5211,	0x00000000 },	\
	{ AR5K_ADDAC_TEST,	0x00000000 },	\
	{ AR5K_DEFAULT_ANTENNA,	0x00000000 },	\
        /* PHY registers */			\
	{ AR5K_PHY_AGC,	0x00000000 },		\
	{ AR5K_PHY(3),	0x2d849093 },		\
	{ AR5K_PHY(4),	0x7d32e000 },		\
	{ AR5K_PHY(5),	0x00000f6b },		\
	{ AR5K_PHY_ACT,	0x00000000 },		\
	{ AR5K_PHY(11),	0x00026ffe },		\
	{ AR5K_PHY(12),	0x00000000 },		\
	{ AR5K_PHY(15),	0x00020100 },		\
	{ AR5K_PHY(16),	0x206a017a },		\
	{ AR5K_PHY(19),	0x1284613c },		\
	{ AR5K_PHY(21),	0x00000859 },		\
	{ AR5K_PHY(26),	0x409a4190 },	/* 0x9868 */	\
	{ AR5K_PHY(27),	0x050cb081 },		\
	{ AR5K_PHY(28),	0x0000000f },		\
	{ AR5K_PHY(29),	0x00000080 },		\
	{ AR5K_PHY(30),	0x0000000c },		\
	{ AR5K_PHY(64),	0x00000000 },		\
	{ AR5K_PHY(65),	0x00000000 },		\
	{ AR5K_PHY(66),	0x00000000 },		\
	{ AR5K_PHY(67),	0x00800000 },		\
	{ AR5K_PHY(68),	0x00000001 },		\
	{ AR5K_PHY(71),	0x0000092a },		\
	{ AR5K_PHY_IQ,	0x00000000 },		\
	{ AR5K_PHY(73),	0x00058a05 },		\
	{ AR5K_PHY(74),	0x00000001 },		\
	{ AR5K_PHY(75),	0x00000000 },		\
	{ AR5K_PHY_PAPD_PROBE, 0x00000000 },	\
	{ AR5K_PHY(77),	0x00000000 },	/* 0x9934 */	\
	{ AR5K_PHY(78),	0x00000000 },	/* 0x9938 */	\
	{ AR5K_PHY(79),	0x0000003f },	/* 0x993c */	\
	{ AR5K_PHY(80),	0x00000004 },		\
	{ AR5K_PHY(82),	0x00000000 },		\
	{ AR5K_PHY(83),	0x00000000 },		\
	{ AR5K_PHY(84),	0x00000000 },		\
	{ AR5K_PHY_RADAR, 0x5d50f14c },		\
	{ AR5K_PHY(86),	0x00000018 },		\
	{ AR5K_PHY(87),	0x004b6a8e },		\
	/* Power table (32bytes) */			\
	{ AR5K_PHY_PCDAC_TXPOWER(1), 0x06ff05ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(2), 0x07ff07ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(3), 0x08ff08ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(4), 0x09ff09ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(5), 0x0aff0aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(6), 0x0bff0bff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(7), 0x0cff0cff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(8), 0x0dff0dff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(9), 0x0fff0eff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(10), 0x12ff12ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(11), 0x14ff13ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(12), 0x16ff15ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(13), 0x19ff17ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(14), 0x1bff1aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(15), 0x1eff1dff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(16), 0x23ff20ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(17), 0x27ff25ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(18), 0x2cff29ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(19), 0x31ff2fff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(20), 0x37ff34ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(21), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(22), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(23), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(24), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(25), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(26), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(27), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(28), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(29), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(30), 0x3aff3aff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(31), 0x3aff3aff },	\
	{ AR5K_PHY_CCKTXCTL, 0x00000000 },	\
	{ AR5K_PHY(642), 0x503e4646 },		\
	{ AR5K_PHY_GAIN_2GHZ, 0x6480416c },	\
	{ AR5K_PHY(644), 0x0199a003 },		\
	{ AR5K_PHY(645), 0x044cd610 },		\
	{ AR5K_PHY(646), 0x13800040 },		\
	{ AR5K_PHY(647), 0x1be00060 },		\
	{ AR5K_PHY(648), 0x0c53800a },		\
	{ AR5K_PHY(649), 0x0014df3b },		\
	{ AR5K_PHY(650), 0x000001b5 },		\
	{ AR5K_PHY(651), 0x00000020 },		\
}

/* Initial register settings for AR5212 */
#define AR5K_AR5212_INI {			\
	{ AR5K_RXDP,		0x00000000 },	\
	{ AR5K_RXCFG,		0x00000005 },	\
	{ AR5K_MIBC,		0x00000000 },	\
	{ AR5K_TOPS,		0x00000008 },	\
	{ AR5K_RXNOFRM,		0x00000008 },	\
	{ AR5K_TXNOFRM,		0x00000010 },	\
	{ AR5K_RPGTO,		0x00000000 },	\
	{ AR5K_RFCNT,		0x0000001f },	\
	{ AR5K_QUEUE_TXDP(0),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(1),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(2),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(3),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(4),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(5),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(6),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(7),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(8),	0x00000000 },	\
	{ AR5K_QUEUE_TXDP(9),	0x00000000 },	\
	{ AR5K_DCU_FP,		0x00000000 },	\
	{ AR5K_DCU_TXP,		0x00000000 },	\
	{ AR5K_DCU_TX_FILTER,	0x00000000 },	\
	/* Unknown table */			\
	{ 0x1078, 0x00000000 },			\
	{ 0x10b8, 0x00000000 },			\
	{ 0x10f8, 0x00000000 },			\
	{ 0x1138, 0x00000000 },			\
	{ 0x1178, 0x00000000 },			\
	{ 0x11b8, 0x00000000 },			\
	{ 0x11f8, 0x00000000 },			\
	{ 0x1238, 0x00000000 },			\
	{ 0x1278, 0x00000000 },			\
	{ 0x12b8, 0x00000000 },			\
	{ 0x12f8, 0x00000000 },			\
	{ 0x1338, 0x00000000 },			\
	{ 0x1378, 0x00000000 },			\
	{ 0x13b8, 0x00000000 },			\
	{ 0x13f8, 0x00000000 },			\
	{ 0x1438, 0x00000000 },			\
	{ 0x1478, 0x00000000 },			\
	{ 0x14b8, 0x00000000 },			\
	{ 0x14f8, 0x00000000 },			\
	{ 0x1538, 0x00000000 },			\
	{ 0x1578, 0x00000000 },			\
	{ 0x15b8, 0x00000000 },			\
	{ 0x15f8, 0x00000000 },			\
	{ 0x1638, 0x00000000 },			\
	{ 0x1678, 0x00000000 },			\
	{ 0x16b8, 0x00000000 },			\
	{ 0x16f8, 0x00000000 },			\
	{ 0x1738, 0x00000000 },			\
	{ 0x1778, 0x00000000 },			\
	{ 0x17b8, 0x00000000 },			\
	{ 0x17f8, 0x00000000 },			\
	{ 0x103c, 0x00000000 },			\
	{ 0x107c, 0x00000000 },			\
	{ 0x10bc, 0x00000000 },			\
	{ 0x10fc, 0x00000000 },			\
	{ 0x113c, 0x00000000 },			\
	{ 0x117c, 0x00000000 },			\
	{ 0x11bc, 0x00000000 },			\
	{ 0x11fc, 0x00000000 },			\
	{ 0x123c, 0x00000000 },			\
	{ 0x127c, 0x00000000 },			\
	{ 0x12bc, 0x00000000 },			\
	{ 0x12fc, 0x00000000 },			\
	{ 0x133c, 0x00000000 },			\
	{ 0x137c, 0x00000000 },			\
	{ 0x13bc, 0x00000000 },			\
	{ 0x13fc, 0x00000000 },			\
	{ 0x143c, 0x00000000 },			\
	{ 0x147c, 0x00000000 },			\
	{ AR5K_STA_ID1,		0x00000000 },	\
	{ AR5K_BSS_ID0,		0x00000000 },	\
	{ AR5K_BSS_ID1,		0x00000000 },	\
	{ AR5K_RSSI_THR,	0x00000000 },	\
	{ AR5K_BEACON_5211,	0x00000000 },	\
	{ AR5K_CFP_PERIOD_5211,	0x00000000 },	\
	{ AR5K_TIMER0_5211,	0x00000030 },	\
	{ AR5K_TIMER1_5211,	0x0007ffff },	\
	{ AR5K_TIMER2_5211,	0x01ffffff },	\
	{ AR5K_TIMER3_5211,	0x00000031 },	\
	{ AR5K_CFP_DUR_5211,	0x00000000 },	\
	{ AR5K_RX_FILTER_5211,	0x00000000 },	\
	{ AR5K_DIAG_SW_5211,	0x00000000 },	\
	{ AR5K_ADDAC_TEST,	0x00000000 },	\
	{ AR5K_DEFAULT_ANTENNA,	0x00000000 },	\
	{ 0x805c, 0xffffc7ff },			\
	{ 0x8080, 0x00000000 },			\
	{ AR5K_NAV_5211,	0x00000000 },	\
	{ AR5K_RTS_OK_5211,	0x00000000 },	\
	{ AR5K_RTS_FAIL_5211,	0x00000000 },	\
	{ AR5K_ACK_FAIL_5211,	0x00000000 },	\
	{ AR5K_FCS_FAIL_5211,	0x00000000 },	\
	{ AR5K_BEACON_CNT_5211,	0x00000000 },	\
	{ AR5K_XRMODE,		0x2a82301a },	\
	{ AR5K_XRDELAY,		0x05dc01e0 },	\
	{ AR5K_XRTIMEOUT,	0x1f402710 },	\
	{ AR5K_XRCHIRP,		0x01f40000 },	\
	{ AR5K_XRSTOMP,		0x00001e1c },	\
	{ AR5K_SLEEP0,		0x0002aaaa },	\
	{ AR5K_SLEEP1,		0x02005555 },	\
	{ AR5K_SLEEP2,		0x00000000 },	\
	{ AR5K_BSS_IDM0,	0xffffffff },	\
	{ AR5K_BSS_IDM1,	0x0000ffff },	\
	{ AR5K_TXPC,		0x00000000 },	\
	{ AR5K_PROFCNT_TX,	0x00000000 },	\
	{ AR5K_PROFCNT_RX,	0x00000000 },	\
	{ AR5K_PROFCNT_RXCLR,	0x00000000 },	\
	{ AR5K_PROFCNT_CYCLE,	0x00000000 },	\
	{ 0x80fc, 0x00000088 },			\
	{ AR5K_RATE_DUR(0),	0x00000000 },	\
	{ AR5K_RATE_DUR(1),	0x0000008c },	\
	{ AR5K_RATE_DUR(2),	0x000000e4 },	\
	{ AR5K_RATE_DUR(3),	0x000002d5 },	\
	{ AR5K_RATE_DUR(4),	0x00000000 },	\
	{ AR5K_RATE_DUR(5),	0x00000000 },	\
	{ AR5K_RATE_DUR(6),	0x000000a0 },	\
	{ AR5K_RATE_DUR(7),	0x000001c9 },	\
	{ AR5K_RATE_DUR(8),	0x0000002c },	\
	{ AR5K_RATE_DUR(9),	0x0000002c },	\
	{ AR5K_RATE_DUR(10),	0x00000030 },	\
	{ AR5K_RATE_DUR(11),	0x0000003c },	\
	{ AR5K_RATE_DUR(12),	0x0000002c },	\
	{ AR5K_RATE_DUR(13),	0x0000002c },	\
	{ AR5K_RATE_DUR(14),	0x00000030 },	\
	{ AR5K_RATE_DUR(15),	0x0000003c },	\
	{ AR5K_RATE_DUR(16),	0x00000000 },	\
	{ AR5K_RATE_DUR(17),	0x00000000 },	\
	{ AR5K_RATE_DUR(18),	0x00000000 },	\
	{ AR5K_RATE_DUR(19),	0x00000000 },	\
	{ AR5K_RATE_DUR(20),	0x00000000 },	\
	{ AR5K_RATE_DUR(21),	0x00000000 },	\
	{ AR5K_RATE_DUR(22),	0x00000000 },	\
	{ AR5K_RATE_DUR(23),	0x00000000 },	\
	{ AR5K_RATE_DUR(24),	0x000000d5 },	\
	{ AR5K_RATE_DUR(25),	0x000000df },	\
	{ AR5K_RATE_DUR(26),	0x00000102 },	\
	{ AR5K_RATE_DUR(27),	0x0000013a },	\
	{ AR5K_RATE_DUR(28),	0x00000075 },	\
	{ AR5K_RATE_DUR(29),	0x0000007f },	\
	{ AR5K_RATE_DUR(30),	0x000000a2 },	\
	{ AR5K_RATE_DUR(31),	0x00000000 },	\
	{ 0x8100, 0x00010002},			\
	{ AR5K_TSF_PARM,	0x00000001 },	\
	{ 0x8108, 0x000000c0 },			\
	{ AR5K_PHY_ERR_FIL,	0x00000000 },	\
	{ 0x8110, 0x00000168 },			\
	{ 0x8114, 0x00000000 },			\
	/* Some kind of table			\
	 * also notice ...03<-02<-01<-00) */	\
	{ 0x87c0, 0x03020100 },			\
	{ 0x87c4, 0x07060504 },			\
	{ 0x87c8, 0x0b0a0908 },			\
	{ 0x87cc, 0x0f0e0d0c },			\
	{ 0x87d0, 0x13121110 },			\
	{ 0x87d4, 0x17161514 },			\
	{ 0x87d8, 0x1b1a1918 },			\
	{ 0x87dc, 0x1f1e1d1c },			\
	{ 0x87e0, 0x03020100 },			\
	{ 0x87e4, 0x07060504 },			\
	{ 0x87e8, 0x0b0a0908 },			\
	{ 0x87ec, 0x0f0e0d0c },			\
	{ 0x87f0, 0x13121110 },			\
	{ 0x87f4, 0x17161514 },			\
	{ 0x87f8, 0x1b1a1918 },			\
	{ 0x87fc, 0x1f1e1d1c },			\
	/* PHY registers */			\
	{ AR5K_PHY_AGC,	0x00000000 },		\
	{ AR5K_PHY(3),	0xad848e19 },		\
	{ AR5K_PHY(4),	0x7d28e000 },		\
	{ AR5K_PHY_TIMING_3, 0x9c0a9f6b },	\
	{ AR5K_PHY_ACT,	0x00000000 },		\
	{ AR5K_PHY(11),	0x00022ffe },		\
	{ AR5K_PHY(15),	0x00020100 },		\
	{ AR5K_PHY(16),	0x206a017a },		\
	{ AR5K_PHY(19),	0x1284613c },		\
	{ AR5K_PHY(21),	0x00000859 },		\
	{ AR5K_PHY(64),	0x00000000 },		\
	{ AR5K_PHY(65),	0x00000000 },		\
	{ AR5K_PHY(66),	0x00000000 },		\
	{ AR5K_PHY(67),	0x00800000 },		\
	{ AR5K_PHY(68),	0x00000001 },		\
	{ AR5K_PHY(71),	0x0000092a },		\
	{ AR5K_PHY_IQ,	0x05100000 },		\
	{ AR5K_PHY(74), 0x00000001 },		\
	{ AR5K_PHY(75), 0x00000004 },		\
	{ AR5K_PHY_TXPOWER_RATE1, 0x1e1f2022 },	\
	{ AR5K_PHY_TXPOWER_RATE2, 0x0a0b0c0d },	\
	{ AR5K_PHY_TXPOWER_RATE_MAX, 0x0000003f },\
	{ AR5K_PHY(80), 0x00000004 },		\
	{ AR5K_PHY(82), 0x9280b212 },		\
	{ AR5K_PHY_RADAR, 0x5d50e188 },		\
	{ AR5K_PHY(86),	0x000000ff },		\
	{ AR5K_PHY(87),	0x004b6a8e },		\
	{ AR5K_PHY(90),	0x000003ce },		\
	{ AR5K_PHY(92),	0x192fb515 },		\
	{ AR5K_PHY(93),	0x00000000 },		\
	{ AR5K_PHY(94),	0x00000001 },		\
	{ AR5K_PHY(95),	0x00000000 },		\
	/* Power table (32bytes) */		\
	{ AR5K_PHY_PCDAC_TXPOWER(1), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(2), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(3), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(4), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(5), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(6), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(7), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(8), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(9), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(10), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(11), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(12), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(13), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(14), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(15), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(16), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(17), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(18), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(19), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(20), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(21), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(22), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(23), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(24), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(25), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(26), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(27), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(28), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(29), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(30), 0x10ff10ff },	\
	{ AR5K_PHY_PCDAC_TXPOWER(31),0x10ff10ff },	\
	{ AR5K_PHY(644), 0x0080a333 },		\
	{ AR5K_PHY(645), 0x00206c10 },		\
	{ AR5K_PHY(646), 0x009c4060 },		\
	{ AR5K_PHY(647), 0x1483800a },		\
	{ AR5K_PHY(648), 0x01831061 },		\
	{ AR5K_PHY(649), 0x00000400 },		\
	{ AR5K_PHY(650), 0x000001b5 },		\
	{ AR5K_PHY(651), 0x00000000 },		\
	{ AR5K_PHY_TXPOWER_RATE3, 0x20202020 },	\
	{ AR5K_PHY_TXPOWER_RATE2, 0x20202020 },	\
	{ AR5K_PHY(655), 0x13c889af },		\
	{ AR5K_PHY(656), 0x38490a20 },		\
	{ AR5K_PHY(657), 0x00007bb6 },		\
	{ AR5K_PHY(658), 0x0fff3ffc },		\
	{ AR5K_PHY_CCKTXCTL, 0x00000000 },	\
}

/* RF5111 Initial BB Gain settings */
#define AR5K_RF5111_BBGAIN_INI {		\
	{ AR5K_BB_GAIN(0), 0x00000000 },	\
	{ AR5K_BB_GAIN(1), 0x00000020 },	\
	{ AR5K_BB_GAIN(2), 0x00000010 },	\
	{ AR5K_BB_GAIN(3), 0x00000030 },	\
	{ AR5K_BB_GAIN(4), 0x00000008 },	\
	{ AR5K_BB_GAIN(5), 0x00000028 },	\
	{ AR5K_BB_GAIN(6), 0x00000004 },	\
	{ AR5K_BB_GAIN(7), 0x00000024 },	\
	{ AR5K_BB_GAIN(8), 0x00000014 },	\
	{ AR5K_BB_GAIN(9), 0x00000034 },	\
	{ AR5K_BB_GAIN(10), 0x0000000c },	\
	{ AR5K_BB_GAIN(11), 0x0000002c },	\
	{ AR5K_BB_GAIN(12), 0x00000002 },	\
	{ AR5K_BB_GAIN(13), 0x00000022 },	\
	{ AR5K_BB_GAIN(14), 0x00000012 },	\
	{ AR5K_BB_GAIN(15), 0x00000032 },	\
	{ AR5K_BB_GAIN(16), 0x0000000a },	\
	{ AR5K_BB_GAIN(17), 0x0000002a },	\
	{ AR5K_BB_GAIN(18), 0x00000006 },	\
	{ AR5K_BB_GAIN(19), 0x00000026 },	\
	{ AR5K_BB_GAIN(20), 0x00000016 },	\
	{ AR5K_BB_GAIN(21), 0x00000036 },	\
	{ AR5K_BB_GAIN(22), 0x0000000e },	\
	{ AR5K_BB_GAIN(23), 0x0000002e },	\
	{ AR5K_BB_GAIN(24), 0x00000001 },	\
	{ AR5K_BB_GAIN(25), 0x00000021 },	\
	{ AR5K_BB_GAIN(26), 0x00000011 },	\
	{ AR5K_BB_GAIN(27), 0x00000031 },	\
	{ AR5K_BB_GAIN(28), 0x00000009 },	\
	{ AR5K_BB_GAIN(29), 0x00000029 },	\
	{ AR5K_BB_GAIN(30), 0x00000005 },	\
	{ AR5K_BB_GAIN(31), 0x00000025 },	\
	{ AR5K_BB_GAIN(32), 0x00000015 },	\
	{ AR5K_BB_GAIN(33), 0x00000035 },	\
	{ AR5K_BB_GAIN(34), 0x0000000d },	\
	{ AR5K_BB_GAIN(35), 0x0000002d },	\
	{ AR5K_BB_GAIN(36), 0x00000003 },	\
	{ AR5K_BB_GAIN(37), 0x00000023 },	\
	{ AR5K_BB_GAIN(38), 0x00000013 },	\
	{ AR5K_BB_GAIN(39), 0x00000033 },	\
	{ AR5K_BB_GAIN(40), 0x0000000b },	\
	{ AR5K_BB_GAIN(41), 0x0000002b },	\
	{ AR5K_BB_GAIN(42), 0x0000002b },	\
	{ AR5K_BB_GAIN(43), 0x0000002b },	\
	{ AR5K_BB_GAIN(44), 0x0000002b },	\
	{ AR5K_BB_GAIN(45), 0x0000002b },	\
	{ AR5K_BB_GAIN(46), 0x0000002b },	\
	{ AR5K_BB_GAIN(47), 0x0000002b },	\
	{ AR5K_BB_GAIN(48), 0x0000002b },	\
	{ AR5K_BB_GAIN(49), 0x0000002b },	\
	{ AR5K_BB_GAIN(50), 0x0000002b },	\
	{ AR5K_BB_GAIN(51), 0x0000002b },	\
	{ AR5K_BB_GAIN(52), 0x0000002b },	\
	{ AR5K_BB_GAIN(53), 0x0000002b },	\
	{ AR5K_BB_GAIN(54), 0x0000002b },	\
	{ AR5K_BB_GAIN(55), 0x0000002b },	\
	{ AR5K_BB_GAIN(56), 0x0000002b },	\
	{ AR5K_BB_GAIN(57), 0x0000002b },	\
	{ AR5K_BB_GAIN(58), 0x0000002b },	\
	{ AR5K_BB_GAIN(59), 0x0000002b },	\
	{ AR5K_BB_GAIN(60), 0x0000002b },	\
	{ AR5K_BB_GAIN(61), 0x0000002b },	\
	{ AR5K_BB_GAIN(62), 0x00000002 },	\
	{ AR5K_BB_GAIN(63), 0x00000016 },	\
}

/* RF 5112 Initial BB Gain settings */
#define AR5K_RF5112_BBGAIN_INI {		\
 	{ AR5K_BB_GAIN(0), 0x00000000 },	\
	{ AR5K_BB_GAIN(1), 0x00000001 },	\
	{ AR5K_BB_GAIN(2), 0x00000002 },	\
	{ AR5K_BB_GAIN(3), 0x00000003 },	\
	{ AR5K_BB_GAIN(4), 0x00000004 },	\
	{ AR5K_BB_GAIN(5), 0x00000005 },	\
	{ AR5K_BB_GAIN(6), 0x00000008 },	\
	{ AR5K_BB_GAIN(7), 0x00000009 },	\
	{ AR5K_BB_GAIN(8), 0x0000000a },	\
	{ AR5K_BB_GAIN(9), 0x0000000b },	\
	{ AR5K_BB_GAIN(10), 0x0000000c },	\
	{ AR5K_BB_GAIN(11), 0x0000000d },	\
	{ AR5K_BB_GAIN(12), 0x00000010 },	\
	{ AR5K_BB_GAIN(13), 0x00000011 },	\
	{ AR5K_BB_GAIN(14), 0x00000012 },	\
	{ AR5K_BB_GAIN(15), 0x00000013 },	\
	{ AR5K_BB_GAIN(16), 0x00000014 },	\
	{ AR5K_BB_GAIN(17), 0x00000015 },	\
	{ AR5K_BB_GAIN(18), 0x00000018 },	\
	{ AR5K_BB_GAIN(19), 0x00000019 },	\
	{ AR5K_BB_GAIN(20), 0x0000001a },	\
	{ AR5K_BB_GAIN(21), 0x0000001b },	\
	{ AR5K_BB_GAIN(22), 0x0000001c },	\
	{ AR5K_BB_GAIN(23), 0x0000001d },	\
	{ AR5K_BB_GAIN(24), 0x00000020 },	\
	{ AR5K_BB_GAIN(25), 0x00000021 },	\
	{ AR5K_BB_GAIN(26), 0x00000022 },	\
	{ AR5K_BB_GAIN(27), 0x00000023 },	\
	{ AR5K_BB_GAIN(28), 0x00000024 },	\
	{ AR5K_BB_GAIN(29), 0x00000025 },	\
	{ AR5K_BB_GAIN(30), 0x00000028 },	\
	{ AR5K_BB_GAIN(31), 0x00000029 },	\
	{ AR5K_BB_GAIN(32), 0x0000002a },	\
	{ AR5K_BB_GAIN(33), 0x0000002b },	\
	{ AR5K_BB_GAIN(34), 0x0000002c },	\
	{ AR5K_BB_GAIN(35), 0x0000002d },	\
	{ AR5K_BB_GAIN(36), 0x00000030 },	\
	{ AR5K_BB_GAIN(37), 0x00000031 },	\
	{ AR5K_BB_GAIN(38), 0x00000032 },	\
	{ AR5K_BB_GAIN(39), 0x00000033 },	\
	{ AR5K_BB_GAIN(40), 0x00000034 },	\
	{ AR5K_BB_GAIN(41), 0x00000035 },	\
	{ AR5K_BB_GAIN(42), 0x00000035 },	\
	{ AR5K_BB_GAIN(43), 0x00000035 },	\
	{ AR5K_BB_GAIN(44), 0x00000035 },	\
	{ AR5K_BB_GAIN(45), 0x00000035 },	\
	{ AR5K_BB_GAIN(46), 0x00000035 },	\
	{ AR5K_BB_GAIN(47), 0x00000035 },	\
	{ AR5K_BB_GAIN(48), 0x00000035 },	\
	{ AR5K_BB_GAIN(49), 0x00000035 },	\
	{ AR5K_BB_GAIN(50), 0x00000035 },	\
	{ AR5K_BB_GAIN(51), 0x00000035 },	\
	{ AR5K_BB_GAIN(52), 0x00000035 },	\
	{ AR5K_BB_GAIN(53), 0x00000035 },	\
	{ AR5K_BB_GAIN(54), 0x00000035 },	\
	{ AR5K_BB_GAIN(55), 0x00000035 },	\
	{ AR5K_BB_GAIN(56), 0x00000035 },	\
	{ AR5K_BB_GAIN(57), 0x00000035 },	\
	{ AR5K_BB_GAIN(58), 0x00000035 },	\
	{ AR5K_BB_GAIN(59), 0x00000035 },	\
	{ AR5K_BB_GAIN(60), 0x00000035 },	\
	{ AR5K_BB_GAIN(61), 0x00000035 },	\
	{ AR5K_BB_GAIN(62), 0x00000010 },	\
	{ AR5K_BB_GAIN(63), 0x0000001a },	\
}

struct ath5k_ar5210_ini_mode{
	u_int16_t mode_register;
	u_int32_t mode_base, mode_turbo;
};

#define AR5K_AR5210_INI_MODE(_aifs) {				\
	{ AR5K_SLOT_TIME,					\
	    AR5K_INIT_SLOT_TIME,				\
	    AR5K_INIT_SLOT_TIME_TURBO },			\
	{ AR5K_SLOT_TIME,					\
	    AR5K_INIT_ACK_CTS_TIMEOUT,				\
	    AR5K_INIT_ACK_CTS_TIMEOUT_TURBO },			\
	{ AR5K_USEC_5210,					\
	    AR5K_INIT_TRANSMIT_LATENCY,				\
	    AR5K_INIT_TRANSMIT_LATENCY_TURBO},			\
	{ AR5K_IFS0,						\
	    ((AR5K_INIT_SIFS + (_aifs) * AR5K_INIT_SLOT_TIME)	\
	    << AR5K_IFS0_DIFS_S) | AR5K_INIT_SIFS,		\
	    ((AR5K_INIT_SIFS_TURBO + (_aifs) * AR5K_INIT_SLOT_TIME_TURBO) \
	    << AR5K_IFS0_DIFS_S) | AR5K_INIT_SIFS_TURBO },	\
	{ AR5K_IFS1,						\
	    AR5K_INIT_PROTO_TIME_CNTRL,				\
	    AR5K_INIT_PROTO_TIME_CNTRL_TURBO },			\
	{ AR5K_PHY(17),						\
	    (AR5K_REG_READ(AR5K_PHY(17)) & ~0x7F) | 0x1C,	\
	    (AR5K_REG_READ(AR5K_PHY(17)) & ~0x7F) | 0x38 },	\
	{ AR5K_PHY_FRAME_CTL_5210,				\
	    AR5K_PHY_FRAME_CTL_SERVICE_ERR |			\
	    AR5K_PHY_FRAME_CTL_TXURN_ERR |			\
	    AR5K_PHY_FRAME_CTL_ILLLEN_ERR |			\
	    AR5K_PHY_FRAME_CTL_ILLRATE_ERR |			\
	    AR5K_PHY_FRAME_CTL_PARITY_ERR |			\
	    AR5K_PHY_FRAME_CTL_TIMING_ERR | 0x1020,		\
	    AR5K_PHY_FRAME_CTL_SERVICE_ERR |			\
	    AR5K_PHY_FRAME_CTL_TXURN_ERR |			\
	    AR5K_PHY_FRAME_CTL_ILLLEN_ERR |			\
	    AR5K_PHY_FRAME_CTL_ILLRATE_ERR |			\
	    AR5K_PHY_FRAME_CTL_PARITY_ERR |			\
	/*PHY_TURBO is PHY_FRAME_CTL on 5210*/			\
	    AR5K_PHY_TURBO_MODE |				\
	    AR5K_PHY_TURBO_SHORT |				\
	    AR5K_PHY_FRAME_CTL_TIMING_ERR | 0x2020 },		\
}

struct ath5k_ar5211_ini_mode {
	u_int16_t	mode_register;
	u_int32_t	mode_value[4];
};

#define AR5K_AR5211_INI_MODE {						\
	{ 0x0030, { 0x00000017, 0x00000017, 0x00000017, 0x00000017 } },	\
	{ 0x1040, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1044, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1048, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x104c, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1050, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1054, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1058, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x105c, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1060, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1064, { 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f } },	\
	{ 0x1070, { 0x00000168, 0x000001e0, 0x000001b8, 0x00000168 } },	\
	{ 0x1030, { 0x00000230, 0x000001e0, 0x000000b0, 0x00000230 } },	\
	{ 0x10b0, { 0x00000d98, 0x00001180, 0x00001f48, 0x00000d98 } },	\
	{ 0x10f0, { 0x0000a0e0, 0x00014068, 0x00005880, 0x0000a0e0 } },	\
	{ 0x8014, { 0x04000400, 0x08000800, 0x20003000, 0x04000400 } },	\
	{ 0x801c, { 0x0e8d8fa7, 0x0e8d8fcf, 0x01608f95, 0x0e8d8fa7 } },	\
	{ 0x9804, { 0x00000000, 0x00000003, 0x00000000, 0x00000000 } },	\
	{ 0x9820, { 0x02020200, 0x02020200, 0x02010200, 0x02020200 } },	\
	{ 0x9824, { 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e } },	\
	{ 0x9828, { 0x0a020001, 0x0a020001, 0x05010000, 0x0a020001 } },	\
	{ 0x9834, { 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },	\
	{ 0x9838, { 0x00000007, 0x00000007, 0x0000000b, 0x0000000b } },	\
	{ 0x9844, { 0x1372169c, 0x137216a5, 0x137216a8, 0x1372169c } },	\
	{ 0x9848, { 0x0018ba67, 0x0018ba67, 0x0018ba69, 0x0018ba69 } },	\
	{ 0x9850, { 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0 } },	\
	{ 0x9858, { 0x7e800d2e, 0x7e800d2e, 0x7ec00d2e, 0x7e800d2e } },	\
	{ 0x985c, { 0x31375d5e, 0x31375d5e, 0x313a5d5e, 0x31375d5e } },	\
	{ 0x9860, { 0x0000bd10, 0x0000bd10, 0x0000bd38, 0x0000bd10 } },	\
	{ 0x9864, { 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },	\
	{ 0x9914, { 0x00002710, 0x00002710, 0x0000157c, 0x00002710 } },	\
	{ 0x9918, { 0x00000190, 0x00000190, 0x00000084, 0x00000190 } },	\
	{ 0x9944, { 0x6fe01020, 0x6fe01020, 0x6fe00920, 0x6fe01020 } },	\
	{ 0xa180, { 0x05ff14ff, 0x05ff14ff, 0x05ff14ff, 0x05ff19ff } },	\
	{ 0x98d4, { 0x00000010, 0x00000014, 0x00000010, 0x00000010 } },	\
}

struct ath5k_ar5212_ini_mode {
	u_int16_t	mode_register;
	u_int8_t	mode_flags;
	u_int32_t	mode_value[2][5];
};

#define AR5K_INI_FLAG_511X	0x00
#define AR5K_INI_FLAG_5111	0x01
#define AR5K_INI_FLAG_5112	0x02
#define AR5K_INI_FLAG_BOTH	(AR5K_INI_FLAG_5111 | AR5K_INI_FLAG_5112)

#define AR5K_AR5212_INI_MODE {							\
	{ 0x0030, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00008107, 0x00008107, 0x00008107, 0x00008107, 0x00008107 }	\
	} },									\
	{ 0x1040, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1044, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1048, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x104c, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1050, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1054, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1058, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x105c, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1060, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1064, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1030, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000230, 0x000001e0, 0x000000b0, 0x00000160, 0x000001e0 }	\
	} },									\
	{ 0x1070, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000168, 0x000001e0, 0x000001b8, 0x0000018c, 0x000001e0 }	\
	} },									\
	{ 0x10b0, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000e60, 0x00001180, 0x00001f1c, 0x00003e38, 0x00001180 }	\
	} },									\
	{ 0x10f0, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x0000a0e0, 0x00014068, 0x00005880, 0x0000b0e0, 0x00014068 }	\
	} },									\
	{ 0x8014, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x03e803e8, 0x06e006e0, 0x04200420, 0x08400840, 0x06e006e0 }	\
	} },									\
	{ 0x9804, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000003 }	\
	} },									\
	{ 0x9820, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 }	\
	} },									\
	{ 0x9834, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e }	\
	} },									\
	{ 0x9838, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b }	\
	} },									\
	{ 0x9844, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x1372161c, 0x13721c25, 0x13721728, 0x137216a2, 0x13721c25 }	\
	} },									\
	{ 0x9850, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0 }	\
	} },									\
	{ 0x9858, AR5K_INI_FLAG_511X, {						\
 		{ 0, },								\
		{ 0x7e800d2e, 0x7e800d2e, 0x7ee84d2e, 0x7ee84d2e, 0x7e800d2e }	\
	} },									\
	{ 0x9860, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00009d10, 0x00009d10, 0x00009d18, 0x00009d10, 0x00009d10 }	\
	} },									\
	{ 0x9864, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 }	\
	} },									\
	{ 0x9868, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190 }	\
	} },									\
	{ 0x9918, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x000001b8, 0x000001b8, 0x00000084, 0x00000108, 0x000001b8 }	\
	} },									\
	{ 0x9924, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05 }	\
	} },									\
	{ 0xa180, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x10ff14ff, 0x10ff14ff, 0x10ff10ff, 0x10ff19ff, 0x10ff19ff }	\
	} },									\
	{ 0xa230, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000108, 0x00000000 }	\
	} },									\
	{ 0x801c, AR5K_INI_FLAG_BOTH, {						\
		{ 0x128d8fa7, 0x09880fcf, 0x04e00f95, 0x128d8fab, 0x09880fcf }, \
		{ 0x128d93a7, 0x098813cf, 0x04e01395, 0x128d93ab, 0x098813cf }	\
	} },									\
	{ 0x9824, AR5K_INI_FLAG_BOTH, {						\
		{ 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e, 0x00000e0e },	\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e }	\
	} },									\
	{ 0x9828, AR5K_INI_FLAG_BOTH, {						\
		{ 0x0a020001, 0x0a020001, 0x05010100, 0x0a020001, 0x0a020001 },	\
		{ 0x0a020001, 0x0a020001, 0x05020100, 0x0a020001, 0x0a020001 }	\
	} },									\
	{ 0x9848, AR5K_INI_FLAG_BOTH, {						\
		{ 0x0018da5a, 0x0018da5a, 0x0018ca69, 0x0018ca69, 0x0018ca69 },	\
		{ 0x0018da6d, 0x0018da6d, 0x0018ca75, 0x0018ca75, 0x0018ca75 }	\
	} },									\
	{ 0x985c, AR5K_INI_FLAG_BOTH, {						\
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137615e },	\
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e }	\
	} },									\
	{ 0x986c, AR5K_INI_FLAG_BOTH, {						\
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb080, 0x050cb080 },	\
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081 }	\
	} },									\
	{ 0x9914, AR5K_INI_FLAG_BOTH, {						\
		{ 0x00002710, 0x00002710, 0x0000157c, 0x00002af8, 0x00002710 },	\
		{ 0x000007d0, 0x000007d0, 0x0000044c, 0x00000898, 0x000007d0 }	\
	} },									\
	{ 0x9944, AR5K_INI_FLAG_BOTH, {						\
		{ 0xffb81020, 0xffb81020, 0xffb80d20, 0xffb81020, 0xffb81020 },	\
		{ 0xffb81020, 0xffb81020, 0xffb80d10, 0xffb81010, 0xffb81010 }	\
	} },									\
	{ 0xa204, AR5K_INI_FLAG_5112, {						\
		{ 0, },								\
		{ 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 }	\
	} },									\
	{ 0xa208, AR5K_INI_FLAG_BOTH, {						\
		{ 0xd03e6788, 0xd03e6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 },	\
		{ 0xd6be6788, 0xd6be6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 }	\
	} },									\
	{ 0xa20c, AR5K_INI_FLAG_5112, {						\
		{ 0, },								\
		{ 0x642c0140, 0x642c0140, 0x6442c160, 0x6442c160, 0x6442c160 }	\
	} },									\
}

struct ath5k_ar5211_ini_rf {
	u_int16_t	rf_register;
	u_int32_t	rf_value[2];
};

#define AR5K_AR5211_INI_RF	{					\
/* Static -> moved on ar5211_ini */					\
	{ 0x0000a204, { 0x00000000, 0x00000000 } },			\
	{ 0x0000a208, { 0x503e4646, 0x503e4646 } },			\
	{ 0x0000a20c, { 0x6480416c, 0x6480416c } },			\
	{ 0x0000a210, { 0x0199a003, 0x0199a003 } },			\
	{ 0x0000a214, { 0x044cd610, 0x044cd610 } },			\
	{ 0x0000a218, { 0x13800040, 0x13800040 } },			\
	{ 0x0000a21c, { 0x1be00060, 0x1be00060 } },			\
	{ 0x0000a220, { 0x0c53800a, 0x0c53800a } },			\
	{ 0x0000a224, { 0x0014df3b, 0x0014df3b } },			\
	{ 0x0000a228, { 0x000001b5, 0x000001b5 } },			\
	{ 0x0000a22c, { 0x00000020, 0x00000020 } },			\
/* Bank 6 ? */								\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00380000, 0x00380000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x000400f9, 0x000400f9 } },			\
	{ 0x000098d4, { 0x00000000, 0x00000004 } },			\
/* Bank 7 ? */								\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x10000000, 0x10000000 } },			\
	{ 0x0000989c, { 0x04000000, 0x04000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x00000000 } },			\
	{ 0x0000989c, { 0x00000000, 0x0a000000 } },			\
	{ 0x0000989c, { 0x00380080, 0x02380080 } },			\
	{ 0x0000989c, { 0x00020006, 0x00000006 } },			\
	{ 0x0000989c, { 0x00000092, 0x00000092 } },			\
	{ 0x0000989c, { 0x000000a0, 0x000000a0 } },			\
	{ 0x0000989c, { 0x00040007, 0x00040007 } },			\
	{ 0x000098d4, { 0x0000001a, 0x0000001a } },			\
	{ 0x0000989c, { 0x00000048, 0x00000048 } },			\
	{ 0x0000989c, { 0x00000010, 0x00000010 } },			\
	{ 0x0000989c, { 0x00000008, 0x00000008 } },			\
	{ 0x0000989c, { 0x0000000f, 0x0000000f } },			\
	{ 0x0000989c, { 0x000000f2, 0x00000062 } },			\
	{ 0x0000989c, { 0x0000904f, 0x0000904c } },			\
	{ 0x0000989c, { 0x0000125a, 0x0000129a } },			\
	{ 0x000098cc, { 0x0000000e, 0x0000000f } },			\
}


enum ieee80211_countrycode {
	CTRY_DEFAULT            = 0,   /* Default domain (NA) */
	CTRY_ALBANIA            = 8,   /* Albania */
	CTRY_ALGERIA            = 12,  /* Algeria */
	CTRY_ARGENTINA          = 32,  /* Argentina */
	CTRY_ARMENIA            = 51,  /* Armenia */
	CTRY_AUSTRALIA          = 36,  /* Australia */
	CTRY_AUSTRIA            = 40,  /* Austria */
	CTRY_AZERBAIJAN         = 31,  /* Azerbaijan */
	CTRY_BAHRAIN            = 48,  /* Bahrain */
	CTRY_BELARUS            = 112, /* Belarus */
	CTRY_BELGIUM            = 56,  /* Belgium */
	CTRY_BELIZE             = 84,  /* Belize */
	CTRY_BOLIVIA            = 68,  /* Bolivia */
	CTRY_BRAZIL             = 76,  /* Brazil */
	CTRY_BRUNEI_DARUSSALAM  = 96,  /* Brunei Darussalam */
	CTRY_BULGARIA           = 100, /* Bulgaria */
	CTRY_CANADA             = 124, /* Canada */
	CTRY_CHILE              = 152, /* Chile */
	CTRY_CHINA              = 156, /* People's Republic of China */
	CTRY_COLOMBIA           = 170, /* Colombia */
	CTRY_COSTA_RICA         = 188, /* Costa Rica */
	CTRY_CROATIA            = 191, /* Croatia */
	CTRY_CYPRUS             = 196, /* Cyprus */
	CTRY_CZECH              = 203, /* Czech Republic */
	CTRY_DENMARK            = 208, /* Denmark */
	CTRY_DOMINICAN_REPUBLIC = 214, /* Dominican Republic */
	CTRY_ECUADOR            = 218, /* Ecuador */
	CTRY_EGYPT              = 818, /* Egypt */
	CTRY_EL_SALVADOR        = 222, /* El Salvador */
	CTRY_ESTONIA            = 233, /* Estonia */
	CTRY_FAROE_ISLANDS      = 234, /* Faroe Islands */
	CTRY_FINLAND            = 246, /* Finland */
	CTRY_FRANCE             = 250, /* France */
	CTRY_FRANCE2            = 255, /* France2 */
	CTRY_GEORGIA            = 268, /* Georgia */
	CTRY_GERMANY            = 276, /* Germany */
	CTRY_GREECE             = 300, /* Greece */
	CTRY_GUATEMALA          = 320, /* Guatemala */
	CTRY_HONDURAS           = 340, /* Honduras */
	CTRY_HONG_KONG          = 344, /* Hong Kong S.A.R., P.R.C. */
	CTRY_HUNGARY            = 348, /* Hungary */
	CTRY_ICELAND            = 352, /* Iceland */
	CTRY_INDIA              = 356, /* India */
	CTRY_INDONESIA          = 360, /* Indonesia */
	CTRY_IRAN               = 364, /* Iran */
	CTRY_IRAQ               = 368, /* Iraq */
	CTRY_IRELAND            = 372, /* Ireland */
	CTRY_ISRAEL             = 376, /* Israel */
	CTRY_ITALY              = 380, /* Italy */
	CTRY_JAMAICA            = 388, /* Jamaica */
	CTRY_JAPAN              = 392, /* Japan */
	CTRY_JAPAN1             = 393, /* Japan (JP1) */
	CTRY_JAPAN2             = 394, /* Japan (JP0) */
	CTRY_JAPAN3             = 395, /* Japan (JP1-1) */
	CTRY_JAPAN4             = 396, /* Japan (JE1) */
	CTRY_JAPAN5             = 397, /* Japan (JE2) */
	CTRY_JORDAN             = 400, /* Jordan */
	CTRY_KAZAKHSTAN         = 398, /* Kazakhstan */
	CTRY_KENYA              = 404, /* Kenya */
	CTRY_KOREA_NORTH        = 408, /* North Korea */
	CTRY_KOREA_ROC          = 410, /* South Korea */
	CTRY_KOREA_ROC2         = 411, /* South Korea */
	CTRY_KUWAIT             = 414, /* Kuwait */
	CTRY_LATVIA             = 428, /* Latvia */
	CTRY_LEBANON            = 422, /* Lebanon */
	CTRY_LIBYA              = 434, /* Libya */
	CTRY_LIECHTENSTEIN      = 438, /* Liechtenstein */
	CTRY_LITHUANIA          = 440, /* Lithuania */
	CTRY_LUXEMBOURG         = 442, /* Luxembourg */
	CTRY_MACAU              = 446, /* Macau */
	CTRY_MACEDONIA          = 807, /* Republic of Macedonia */
	CTRY_MALAYSIA           = 458, /* Malaysia */
	CTRY_MEXICO             = 484, /* Mexico */
	CTRY_MONACO             = 492, /* Principality of Monaco */
	CTRY_MOROCCO            = 504, /* Morocco */
	CTRY_NETHERLANDS        = 528, /* Netherlands */
	CTRY_NEW_ZEALAND        = 554, /* New Zealand */
	CTRY_NICARAGUA          = 558, /* Nicaragua */
	CTRY_NORWAY             = 578, /* Norway */
	CTRY_OMAN               = 512, /* Oman */
	CTRY_PAKISTAN           = 586, /* Islamic Republic of Pakistan */
	CTRY_PANAMA             = 591, /* Panama */
	CTRY_PARAGUAY           = 600, /* Paraguay */
	CTRY_PERU               = 604, /* Peru */
	CTRY_PHILIPPINES        = 608, /* Republic of the Philippines */
	CTRY_POLAND             = 616, /* Poland */
	CTRY_PORTUGAL           = 620, /* Portugal */
	CTRY_PUERTO_RICO        = 630, /* Puerto Rico */
	CTRY_QATAR              = 634, /* Qatar */
	CTRY_ROMANIA            = 642, /* Romania */
	CTRY_RUSSIA             = 643, /* Russia */
	CTRY_SAUDI_ARABIA       = 682, /* Saudi Arabia */
	CTRY_SINGAPORE          = 702, /* Singapore */
	CTRY_SLOVAKIA           = 703, /* Slovak Republic */
	CTRY_SLOVENIA           = 705, /* Slovenia */
	CTRY_SOUTH_AFRICA       = 710, /* South Africa */
	CTRY_SPAIN              = 724, /* Spain */
	CTRY_SRI_LANKA          = 728, /* Sri Lanka */
	CTRY_SWEDEN             = 752, /* Sweden */
	CTRY_SWITZERLAND        = 756, /* Switzerland */
	CTRY_SYRIA              = 760, /* Syria */
	CTRY_TAIWAN             = 158, /* Taiwan */
	CTRY_THAILAND           = 764, /* Thailand */
	CTRY_TRINIDAD_Y_TOBAGO  = 780, /* Trinidad y Tobago */
	CTRY_TUNISIA            = 788, /* Tunisia */
	CTRY_TURKEY             = 792, /* Turkey */
	CTRY_UAE                = 784, /* U.A.E. */
	CTRY_UKRAINE            = 804, /* Ukraine */
	CTRY_UNITED_KINGDOM     = 826, /* United Kingdom */
	CTRY_UNITED_STATES      = 840, /* United States */
	CTRY_URUGUAY            = 858, /* Uruguay */
	CTRY_UZBEKISTAN         = 860, /* Uzbekistan */
	CTRY_VENEZUELA          = 862, /* Venezuela */
	CTRY_VIET_NAM           = 704, /* Viet Nam */
	CTRY_YEMEN              = 887, /* Yemen */
	CTRY_ZIMBABWE           = 716, /* Zimbabwe */
};

struct ar5k_country_code_alpha2 {
	u_int16_t	cn_code;
	const char	*cn_name;
};

/* Note: Handle CTRY_DEFAULT internally as it has no ISO3166-1 alpha2
 * map. */
#define AR5K_COUNTRIES { \
	{ CTRY_DEFAULT,            "00"}, \
	{ CTRY_UAE,                "ae"}, \
	{ CTRY_ALBANIA,            "al"}, \
	{ CTRY_ARMENIA,            "am"}, \
	{ CTRY_ARGENTINA,          "ar"}, \
	{ CTRY_AUSTRIA,            "at"}, \
	{ CTRY_AUSTRALIA,          "au"}, \
	{ CTRY_AZERBAIJAN,         "az"}, \
	{ CTRY_BELGIUM,            "be"}, \
	{ CTRY_BULGARIA,           "bg"}, \
	{ CTRY_BAHRAIN,            "bh"}, \
	{ CTRY_BRUNEI_DARUSSALAM,  "bn"}, \
	{ CTRY_BOLIVIA,            "bo"}, \
	{ CTRY_BRAZIL,             "br"}, \
	{ CTRY_BELARUS,            "by"}, \
	{ CTRY_BELIZE,             "bz"}, \
	{ CTRY_CANADA,             "ca"}, \
	{ CTRY_SWITZERLAND,        "ch"}, \
	{ CTRY_CHILE,              "cl"}, \
	{ CTRY_CHINA,              "cn"}, \
	{ CTRY_COLOMBIA,           "co"}, \
	{ CTRY_COSTA_RICA,         "cr"}, \
	{ CTRY_CYPRUS,             "cy"}, \
	{ CTRY_CZECH,              "cz"}, \
	{ CTRY_GERMANY,            "de"}, \
	{ CTRY_DENMARK,            "dk"}, \
	{ CTRY_DOMINICAN_REPUBLIC, "do"}, \
	{ CTRY_ALGERIA,            "dz"}, \
	{ CTRY_ECUADOR,            "ec"}, \
	{ CTRY_ESTONIA,            "ee"}, \
	{ CTRY_EGYPT,              "eg"}, \
	{ CTRY_SPAIN,              "es"}, \
	{ CTRY_FRANCE2,            "f2"}, \
	{ CTRY_FINLAND,            "fi"}, \
	{ CTRY_FAEROE_ISLANDS,     "fo"}, \
	{ CTRY_FRANCE,             "fr"}, \
	{ CTRY_GEORGIA,            "ge"}, \
	{ CTRY_GREECE,             "gr"}, \
	{ CTRY_GUATEMALA,          "gt"}, \
	{ CTRY_HONG_KONG,          "hk"}, \
	{ CTRY_HONDURAS,           "hn"}, \
	{ CTRY_CROATIA,            "hr"}, \
	{ CTRY_HUNGARY,            "hu"}, \
	{ CTRY_INDONESIA,          "id"}, \
	{ CTRY_IRELAND,            "ie"}, \
	{ CTRY_ISRAEL,             "il"}, \
	{ CTRY_INDIA,              "in"}, \
	{ CTRY_IRAQ,               "iq"}, \
	{ CTRY_IRAN,               "ir"}, \
	{ CTRY_ICELAND,            "is"}, \
	{ CTRY_ITALY,              "it"}, \
	{ CTRY_JAPAN1,             "j1"}, \
	{ CTRY_JAPAN2,             "j2"}, \
	{ CTRY_JAPAN3,             "j3"}, \
	{ CTRY_JAPAN4,             "j4"}, \
	{ CTRY_JAPAN5,             "j5"}, \
	{ CTRY_JAMAICA,            "jm"}, \
	{ CTRY_JORDAN,             "jo"}, \
	{ CTRY_JAPAN,              "jp"}, \
	{ CTRY_KOREA_ROC2,         "k2"}, \
	{ CTRY_KENYA,              "ke"}, \
	{ CTRY_KOREA_NORTH,        "kp"}, \
	{ CTRY_KOREA_ROC,          "kr"}, \
	{ CTRY_KUWAIT,             "kw"}, \
	{ CTRY_KAZAKHSTAN,         "kz"}, \
	{ CTRY_LEBANON,            "lb"}, \
	{ CTRY_LIECHTENSTEIN,      "li"}, \
	{ CTRY_SRI_LANKA,          "lk"}, \
	{ CTRY_LITHUANIA,          "lt"}, \
	{ CTRY_LUXEMBOURG,         "lu"}, \
	{ CTRY_LATVIA,             "lv"}, \
	{ CTRY_LIBYA,              "ly"}, \
	{ CTRY_MOROCCO,            "ma"}, \
	{ CTRY_MONACO,             "mc"}, \
	{ CTRY_MACEDONIA,          "mk"}, \
	{ CTRY_MACAU,              "mo"}, \
	{ CTRY_MEXICO,             "mx"}, \
	{ CTRY_MALAYSIA,           "my"}, \
	{ CTRY_NICARAGUA,          "ni"}, \
	{ CTRY_NETHERLANDS,        "nl"}, \
	{ CTRY_NORWAY,             "no"}, \
	{ CTRY_NEW_ZEALAND,        "nz"}, \
	{ CTRY_OMAN,               "om"}, \
	{ CTRY_PANAMA,             "pa"}, \
	{ CTRY_PERU,               "pe"}, \
	{ CTRY_PHILIPPINES,        "ph"}, \
	{ CTRY_PAKISTAN,           "pk"}, \
	{ CTRY_POLAND,             "pl"}, \
	{ CTRY_PUERTO_RICO,        "pr"}, \
	{ CTRY_PORTUGAL,           "pt"}, \
	{ CTRY_PARAGUAY,           "py"}, \
	{ CTRY_QATAR,              "qa"}, \
	{ CTRY_ROMANIA,            "ro"}, \
	{ CTRY_RUSSIA,             "ru"}, \
	{ CTRY_SAUDI_ARABIA,       "sa"}, \
	{ CTRY_SWEDEN,             "se"}, \
	{ CTRY_SINGAPORE,          "sg"}, \
	{ CTRY_SLOVENIA,           "si"}, \
	{ CTRY_SLOVAKIA,           "sk"}, \
	{ CTRY_EL_SALVADOR,        "sv"}, \
	{ CTRY_SYRIA,              "sy"}, \
	{ CTRY_THAILAND,           "th"}, \
	{ CTRY_TUNISIA,            "tn"}, \
	{ CTRY_TURKEY,             "tr"}, \
	{ CTRY_TRINIDAD_Y_TOBAGO,  "tt"}, \
	{ CTRY_TAIWAN,             "tw"}, \
	{ CTRY_UKRAINE,            "ua"}, \
	{ CTRY_UNITED_KINGDOM,     "uk"}, \
	{ CTRY_UNITED_STATES,      "us"}, \
	{ CTRY_URUGUAY,            "uy"}, \
	{ CTRY_UZBEKISTAN,         "uz"}, \
	{ CTRY_VENEZUELA,          "ve"}, \
	{ CTRY_VIET_NAM,           "vn"}, \
	{ CTRY_YEMEN,              "ye"}, \
	{ CTRY_SOUTH_AFRICA,       "za"}, \
	{ CTRY_ZIMBABWE,           "zw"}, \
}

#endif /* _ATH5K_HW_H */
