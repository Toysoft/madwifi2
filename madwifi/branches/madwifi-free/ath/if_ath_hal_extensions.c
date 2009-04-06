/*
 * Copyright (c) 2008 Bruno Randolf <br1@einfach.org>
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
 * $Id: br1 $
 */

#include "ath_hal/ah_devid.h"
#include "if_media.h"
#include <net80211/ieee80211_var.h>
#include "if_athvar.h"
#include "if_ath_hal.h"
#include "if_ath_hal_macros.h"
#include "if_ath_hal_wrappers.h"
#include "if_ath_hal_extensions.h"


/* Known SREVs */
static struct ath5k_srev_name srev_names[] = {
	{ "5210",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5210 },
	{ "5311",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311 },
	{ "5311A",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311A },
	{ "5311B",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311B },
	{ "5211",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5211 },
	{ "5212",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5212 },
	{ "5213",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213 },
	{ "5213A",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213A },
	{ "2413",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2413 },
	{ "2414",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2414 },
	{ "2424",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2424 },
	{ "5424",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5424 },
	{ "5413",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5413 },
	{ "5414",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5414 },
	{ "5416",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5416 },
	{ "5418",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5418 },
	{ "2425",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2425 },
	{ "xxxxx",	AR5K_VERSION_VER,	AR5K_SREV_UNKNOWN },
	{ "5110",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5110 },
	{ "5111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111 },
	{ "2111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2111 },
	{ "5112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112 },
	{ "5112A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112A },
	{ "2112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112 },
	{ "2112A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112A },
	{ "SChip",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_SC0 },
	{ "SChip",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_SC1 },
	{ "SChip",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_SC2 },
	{ "5133",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5133 },
	{ "xxxxx",	AR5K_VERSION_RAD,	AR5K_SREV_UNKNOWN },
};


int
ar_device(int devid)
{
	switch (devid) {
	case AR5210_DEFAULT:
	case AR5210_PROD:
	case AR5210_AP:
		return 5210;
	case AR5211_DEFAULT:
	case AR5311_DEVID:
	case AR5211_LEGACY:
	case AR5211_FPGA11B:
		return 5211;
	case AR5212_DEFAULT:
	case AR5212_DEVID:
	case AR5212_FPGA:
	case AR5212_DEVID_IBM:
	case AR5212_AR5312_REV2:
	case AR5212_AR5312_REV7:
	case AR5212_AR2313_REV8:
	case AR5212_AR2315_REV6:
	case AR5212_AR2315_REV7:
	case AR5212_AR2317_REV1:
	case AR5212_DEVID_0014:
	case AR5212_DEVID_0015:
	case AR5212_DEVID_0016:
	case AR5212_DEVID_0017:
	case AR5212_DEVID_0018:
	case AR5212_DEVID_0019:
	case AR5212_AR2413:
	case AR5212_AR5413:
	case AR5212_AR5424:
	case AR5212_DEVID_FF19:
		return 5212;
	case AR5213_SREV_1_0:
	case AR5213_SREV_REG:
	case AR_SUBVENDOR_ID_NOG:
	case AR_SUBVENDOR_ID_NEW_A:
		return 5213;
	default:
		return 0; /* unknown */
	}
}


int
ath_set_ack_bitrate(struct ath_softc *sc, int high)
{
	if (ar_device(sc->devid) == 5212 || ar_device(sc->devid) == 5213) {
		/* set ack to be sent at low bit-rate */
		u_int32_t v = AR5K_STA_ID1_BASE_RATE_11B | AR5K_STA_ID1_ACKCTS_6MB;
		if (high)
			ath_reg_write(sc, AR5K_STA_ID1,
					ath_reg_read(sc, AR5K_STA_ID1) & ~v);
		else
			ath_reg_write(sc, AR5K_STA_ID1,
					ath_reg_read(sc, AR5K_STA_ID1) | v);
		return 0;
	}
	return 1;
}


const char *
ath5k_chip_name(enum ath5k_srev_type type, u_int16_t val)
{
	const char *name = "xxxxx";
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(srev_names); i++) {
		if (srev_names[i].sr_type != type)
			continue;
		if ((val & 0xff) < srev_names[i + 1].sr_val) {
			name = srev_names[i].sr_name;
			break;
		}
	}

	return name;
}
