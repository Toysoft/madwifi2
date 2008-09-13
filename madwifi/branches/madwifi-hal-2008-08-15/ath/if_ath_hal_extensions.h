/*-
 * Copyright (c) 2007 Michael Taylor
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
	without modification.
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
 * $Id: foo mtaylor $
 */

/* This file provides some wrapper functions that invoke functions in 
 * if_ath_hal.h.  Since all the functions in the generated file, if_ath_hal.h
 * have locks to protect them... no further locking is required in these 
 * additional helper functions.  Mostly these just provide a series of nicknames
 * for specific sets of arguments to HAL functions that are commonly needed. */

#ifndef _IF_ATH_HAL_EXTENSIONS_H_
#define _IF_ATH_HAL_EXTENSIONS_H_

#include <linux/types.h>

/*
 * Transmit configuration register
 */
#define AR5K_TXCFG		0x0030			/* Register Address */
#define AR5K_TXCFG_SDMAMR	0x00000007	/* DMA size */
#define AR5K_TXCFG_SDMAMR_S	0

/*
 * Receive configuration register
 */
#define AR5K_RXCFG		0x0034			/* Register Address */
#define AR5K_RXCFG_SDMAMW	0x00000007	/* DMA size */
#define AR5K_RXCFG_SDMAMW_S	0

/*
 * Second station id register (MAC address in upper 16 bits)
 */
#define AR5K_STA_ID1			0x8004			/* Register Address */
#define AR5K_STA_ID1_AP			0x00010000	/* Set AP mode */
#define AR5K_STA_ID1_ADHOC		0x00020000	/* Set Ad-Hoc mode */
#define AR5K_STA_ID1_PWR_SV		0x00040000	/* Power save reporting (?) */
#define AR5K_STA_ID1_NO_KEYSRCH		0x00080000	/* No key search */
#define AR5K_STA_ID1_NO_PSPOLL		0x00100000	/* No power save polling [5210] */
#define AR5K_STA_ID1_PCF_5211		0x00100000	/* Enable PCF on [5211+] */
#define AR5K_STA_ID1_PCF_5210		0x00200000	/* Enable PCF on [5210] */
#define	AR5K_STA_ID1_PCF		(ah->ah_version == AR5K_AR5210 ? \
					AR5K_STA_ID1_PCF_5210 : AR5K_STA_ID1_PCF_5211)
#define AR5K_STA_ID1_DEFAULT_ANTENNA	0x00200000	/* Use default antenna */
#define AR5K_STA_ID1_DESC_ANTENNA	0x00400000	/* Update antenna from descriptor */
#define AR5K_STA_ID1_RTS_DEF_ANTENNA	0x00800000	/* Use default antenna for RTS (?) */
#define AR5K_STA_ID1_ACKCTS_6MB		0x01000000	/* Use 6Mbit/s for ACK/CTS (?) */
#define AR5K_STA_ID1_BASE_RATE_11B	0x02000000	/* Use 11b base rate (for ACK/CTS ?) [5211+] */


enum ath5k_srev_type {
	AR5K_VERSION_VER,
	AR5K_VERSION_RAD,
};

struct ath5k_srev_name {
	const char		*sr_name;
	enum ath5k_srev_type	sr_type;
	u_int			sr_val;
};

#define AR5K_SREV_UNKNOWN	0xffff

#define AR5K_SREV_VER_AR5210	0x00
#define AR5K_SREV_VER_AR5311	0x10
#define AR5K_SREV_VER_AR5311A	0x20
#define AR5K_SREV_VER_AR5311B	0x30
#define AR5K_SREV_VER_AR5211	0x40
#define AR5K_SREV_VER_AR5212	0x50
#define AR5K_SREV_VER_AR5213	0x55
#define AR5K_SREV_VER_AR5213A	0x59
#define AR5K_SREV_VER_AR2413	0x78
#define AR5K_SREV_VER_AR2414	0x79
#define AR5K_SREV_VER_AR2424	0xa0 /* PCI-E */
#define AR5K_SREV_VER_AR5424	0xa3 /* PCI-E */
#define AR5K_SREV_VER_AR5413	0xa4
#define AR5K_SREV_VER_AR5414	0xa5
#define AR5K_SREV_VER_AR5416	0xc0 /* PCI-E */
#define AR5K_SREV_VER_AR5418	0xca /* PCI-E */
#define AR5K_SREV_VER_AR2425	0xe2 /* PCI-E */

#define AR5K_SREV_RAD_5110	0x00
#define AR5K_SREV_RAD_5111	0x10
#define AR5K_SREV_RAD_5111A	0x15
#define AR5K_SREV_RAD_2111	0x20
#define AR5K_SREV_RAD_5112	0x30
#define AR5K_SREV_RAD_5112A	0x35
#define AR5K_SREV_RAD_2112	0x40
#define AR5K_SREV_RAD_2112A	0x45
#define AR5K_SREV_RAD_SC0	0x56	/* Found on 2413/2414 */
#define AR5K_SREV_RAD_SC1	0x63	/* Found on 5413/5414 */
#define AR5K_SREV_RAD_SC2	0xa2	/* Found on 2424-5/5424 */
#define AR5K_SREV_RAD_5133	0xc0	/* MIMO found on 5418 */

#define ATH_SREV_FROM_AH(_ah)	((_ah)->ah_macVersion << 4 | (_ah)->ah_macRev)

/*
 * DMA size definitions (2^(n+2))
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

int ath_set_ack_bitrate(struct ath_softc *sc, int);
int ar_device(int devid);
const char * ath5k_chip_name(enum ath5k_srev_type type, u_int16_t val);

static inline unsigned long field_width(unsigned long mask, unsigned long shift)
{
	unsigned long r = 0;
	unsigned long x = mask >> shift;
	if ( 0 == mask )  return  0;
#if  BITS_PER_LONG >= 64
	if ( x & (~0UL<<32) )  { x >>= 32;  r += 32; }
#endif
	if ( x & 0xffff0000 )  { x >>= 16;  r += 16; }
	if ( x & 0x0000ff00 )  { x >>=  8;  r +=  8; }
	if ( x & 0x000000f0 )  { x >>=  4;  r +=  4; }
	if ( x & 0x0000000c )  { x >>=  2;  r +=  2; }
	if ( x & 0x00000002 )  {            r +=  1; }
	return r+1;
}

static inline u_int32_t get_field(struct ath_hal *ah, u_int32_t reg, u_int32_t mask, u_int32_t shift, int is_signed) {
	unsigned long x = ((OS_REG_READ(ah, reg) & mask) >> shift);
	if (is_signed) {
		unsigned long c =(-1) << (field_width(mask, shift)-1);
		return (x + c) ^ c;
	}
	return x;
}

static inline void set_field(struct ath_hal *ah, u_int32_t reg, u_int32_t mask, u_int32_t shift, u_int32_t value) {
	OS_REG_WRITE(ah, reg, 
			  (OS_REG_READ(ah, reg) & ~mask) | 
			  ((value << shift) & mask));
}

static inline u_int32_t field_eq(struct ath_hal *ah, u_int32_t reg, 
				 u_int32_t mask, u_int32_t shift, 
				 u_int32_t value, int is_signed) {
	return  (get_field(ah, reg, mask, shift, is_signed) & (mask >> shift)) == 
		(value & (mask >> shift));
}

#define GET_FIELD(ah, __reg, __mask, __signed) \
	get_field(ah, __reg, __mask, __mask ## _S, __signed)
#define SET_FIELD(ah, __reg, __mask, __value) \
	set_field(ah, __reg, __mask, __mask ## _S, __value);
#define FIELD_EQ(ah, __reg, __mask, __value, __signed) \
	field_eq(ah, __reg, __mask, __mask ## _S, __value, __signed)

static inline void ath_hal_set_dmasize_pcie(struct ath_hal *ah) {
	SET_FIELD(ah, AR5K_TXCFG, AR5K_TXCFG_SDMAMR, AR5K_DMASIZE_128B);
	SET_FIELD(ah, AR5K_RXCFG, AR5K_RXCFG_SDMAMW, AR5K_DMASIZE_128B);
}

#endif /* _IF_ATH_HAL_EXTENSIONS_H_ */
