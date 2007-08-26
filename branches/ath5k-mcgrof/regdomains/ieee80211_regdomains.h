#ifndef _IEEE80211_REGDOMAIN_H
#define _IEEE80211_REGDOMAIN_H
/*
 *  Copyright (C) 2006-2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <net/mac80211.h>
#include <linux/kernel.h>
#include "iso3166-1.h"

#define REGDOMAINS_VERSION "1.0"
#define CONFIG_REGDOMAINS_DBG 1
#define REGNAMSIZ		22
#define REGSBNAMSIZ		30

/* XXX: <net/ieee80211.h> has two band defs bellow */
#ifndef IEEE80211_24GHZ_BAND
#define IEEE80211_24GHZ_BAND     (1<<0)
#define IEEE80211_52GHZ_BAND     (1<<1)
#endif

#define REG_NUM_SUBBANDS	7

/**
 * struct ieee80211_subband_restrictions - defines a regulatory domain
 * 	subband restrictions list
 *
 * @band: which band we this subband belongs to, could be either IEEE80211_24GHZ_BAND
 * 	or IEEE80211_5GHZ_BAND
 * @name: name for this subband.
 * @min_freq: minimum frequency for this subband, in MHz. This represents the 
 * 	center of frequency of a channel.
 * @max_freq: maximum frequency for this subband, in MHz. This represents the 
 * 	center of frequency of a channel.
 * @modulation_cap: modes this subband is capable of handling. This is defined by mac80211's
 *	MODE_IEEE80211A, MODE_IEEE80211B, MODE_IEEE80211G, MODE_ATHEROS_TURBO,
 *	and MODE_ATHEROS_TURBOG.
 * @environment_cap: defines indoor/outdoor capability for subband as defined by
 * 	the 802.11d Country Information element string. This can be 
 * 	Indoor REG_CAP_INDOOR ('I'), REG_CAP_OUTDOOR ('O'), or both 
 * 	REG_CAP_INOUT (' ').
 * @max_antenna_gain: defines the maximum antenna gain set by regulatory 
 * 	agencies for this subband. This is expressed in dBi, which is 
 * 	dB relative to isotropic, used to measure antenna gain compared to an
 * 	isotropic antenna on which radiation is distributed uniformly on the
 * 	surface of a sphere. This can be used to define mac80211's
 * 	conf.antenna_max and channel.antenna_max for example.
 * @max_ir_ptmp: maximum allowed Intentional Radiated power on the transmitter
 * 	for Point to Multi Points, given in dBm. This applies to stations, 
 * 	Access Points and PtMP bridges.
 * @max_ir_ptmp: maximum allowed Intentional Radiated power on the transmitter
 *	for Point to Point links such as PtP Bridges, given in dBm.
 * @max_eirp_ptmp: maximum allowed Equivalent Isotropically Radiated Power on 
 *	the transmitter for Point to Point to Multi Points links, given in dBm
 * @max_eirp_ptp: maximum allowed Equivalent Isotropically Radiated Power on 
 *	the transmitter for Point to Point links such as PtP Bridges, given in
 *	dBm.
 * @channel_list: linked list of all channels in this subband. These are of
 * 	type @ieee80211_regdomain_channel
 *
 * This structure contains the regualtory power restrictions on this subband 
 * and has a linked list of all possible channels. Wireless drivers can compute
 * their maximum allowed TX power as follows:
 *
 * Max TX Power = Max EIRP - passive gain of antenna
 * 
 * Keep in mind that the passive gain of the standard dipole antenna is approx
 * 2.2 dBi. You can use this value for most wireless setups. Please note that 
 * some regulatory domains also have rules to which change the max IR if you 
 * change the antenna, such as the 1:1 and 3:1 rule for the FCC. For more 
 * information please refer to the Documentation/ieee80211_regdomains.txt
 * documentation.
 */
struct ieee80211_subband_restrictions {
	u8 band;
	char name[REGSBNAMSIZ];
	u16 min_freq;
	u16 max_freq;
	u32 modulation_cap;
	u8 max_antenna_gain;
	u8 max_ir_ptmp;
	u8 max_ir_ptp;
#define REG_DIPOLE_ANTENNA_GAIN 2
	u8 max_eirp_ptmp;
	u8 max_eirp_ptp;
#define REG_CAP_INDOOR 	'I'
#define REG_CAP_OUTDOOR	'O'
#define REG_CAP_INOUT	' '
	char environment_cap;
	struct list_head channel_list;
};

/**
 * struct ieee80211_regdomain - defines a regulatory domain
 *
 * @regdomain_id: ID of this regulatory domain. Some come from
 * 	http://standards.ieee.org/getieee802/download/802.11b-1999_Cor1-2001.pdf
 * @regdomain_name: name of this regulatory domain.
 * @list: node, part of band_restrictions_list
 *
 * This structure defines a regulatory domain, which consists of channel and
 * power restrictions. Some regulatory domains come from 
 * 802.11b-1999_Cor1-2001, the rest are based on Reyk Floeter's ar5k. If 
 * there is need to add more values here, please add one that is either 
 * defined in a standard or that many hardware devices have adopted. Also 
 * note that multiple countries can map to the same @regdomain_id
 */
struct ieee80211_regdomain {
	u32 regdomain_id;
	char regdomain_name[REGNAMSIZ];
	struct ieee80211_subband_restrictions subbands[REG_NUM_SUBBANDS];
};

/**
 * struct ieee80211_regdomain_channel - linked list node wrapper for an 
 * 	mac80211 ieee80211_channel
 *
 * @list: node, part of @ieee80211_subband_restrictions's channel_list
 * @channel: defines a mac80211 ieee80211_channel which is part of a subband
 * 	along with its defined regulatory restrictions.
 *
 * This structure defines a channel, part of a subband along with its
 * regulatory restrictions. The restrictions are set in ieee80211_channel's
 * flag element. We iterate over these settings when creating each 
 * @ieee80211_subband_restrictions structure for a regulatory domain and 
 * define the subband's mode by this. We do not keep these settings
 * when the channel is set on the channel_list linked list.
 */
struct ieee80211_regdomain_channel {
	struct list_head list;
	struct ieee80211_channel channel;
};

/**
 * struct ieee80211_iso3166_reg_map - map of an ISO3166-1 country to a regdomain
 *
 * @alpha2: the ISO-3166-1 alpha2 country code string. Example: 'CR' for Costa Rica.
 *
 * @regdomain_id: a regualtory domain ID this country maps to.
 *
 * This structure holds the mapping of the country to a specific regulatory
 * domain. Keep in mind more than one country can map to the same regulatory
 * domain. The ISO-3166-1 alpha2 country code also happens to be used in the 
 * 802.11d Country Information Element on the string for the country. It 
 * should be noted, however, that in that the size of this string, is 
 * three octects while our string is only 2. The third octet is used to 
 * indicate Indoor/outdoor capabilities which we set in 
 * @ieee80211_subband_restrictions environment_cap.
 */
struct ieee80211_iso3166_reg_map {
	char alpha2[ISOCOUNTRYSIZ2];
	u32 regdomain_id; /* stack-aware value */
	/* XXX: shall we just use an array? */
	struct list_head list; /* node, part of iso3166_reg_map_list */
};

/**
 * regdomain_mhz2ieee - convert a frequency to an IEEE-80211 channel number
 * @freq: center of frequency in MHz. We support a range from 
 * 2412 - 2732 MHz (Channel 1 - 26) in the 2GHz band and 
 * 5005 - 6100 MHz (Channel 1 - 220) in the 5GHz band.
 *
 * @regdomain_id: a regualtory domain ID this country maps to.
 *
 * Given a frequency in MHz returns the respective IEEE-80211 channel
 * number. You are expected to provide the center of freqency in MHz.
 */
u16 regdomain_mhz2ieee(u16);

static inline void setup_regdomain(struct ieee80211_regdomain *, u8 ,char *);
static inline void setup_subband_restrictions(
		struct ieee80211_subband_restrictions *, char *,
		u16, u16, u32, u8, u8, u8, u8, u8, char);
		

static void free_regdomain(struct ieee80211_regdomain *);

void print_regdomain(struct ieee80211_regdomain *);
void print_iso3166_reg_map(void);

int get_ieee80211_regname(u32, char *);
static int load_user_regdomains(void);
static int load_regdomain_defaults(void);
static int update_ieee80211_regdomains(void);

static int add_iso3166_reg_map(char *, u32);
inline int iso3166_to_reg_exists(char *);
int iso3166_to_reg(char *, u32);

#endif /* _IEEE80211_REGDOMAIN_H_ */ 
