/*
 * Copyright (c) 2004, 2005 Reyk Floeter <reyk@openbsd.org>
 * Copyright (c) 2007 Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>
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
#include <linux/module.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include "ieee80211_regdomains.h"
#include "regulatory_map.h"

#define DRV_NAME	"ieee80211_regdomains"
#define DRV_DESCRIPTION	"IEEE-802.11 Regulatory Domain wireless driver"
#define DRV_VERSION	REGDOMAINS_VERSION

MODULE_AUTHOR("Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_VERSION(DRV_VERSION);

LIST_HEAD(iso3166_reg_map_list);
EXPORT_SYMBOL(iso3166_reg_map_list);

rwlock_t iso3166_reg_rwlock	= RW_LOCK_UNLOCKED;
EXPORT_SYMBOL(iso3166_reg_rwlock);

/* Changing this will change the default regulatory domain
 * for countries we don't have a map for */
#define DEFAULT_REG_REGDOMAIN	DMN_NULL_WORLD

/* 
 * Loads regulatory maps, used to build regualtory domains. We later
 * can provide support to update maps via sysfs and let that be triggered
 * via nl80211
 * */
struct reg_band		reg_band_map[] =	REG_MAP;
struct reg_channel	reg_2ghz_channels[] =	REG_CHANNELS_2GHZ;
struct reg_channel	reg_5ghz_channels[] =	REG_CHANNELS_5GHZ;
struct reg_domainname	reg_domainname_map[] =	REG_NAMES;
struct reg_country	reg_ctry_map[] =	REG_COUNTRY_NAMES;
struct reg_subband	reg_subbands_map[] =	REG_SUBBANDS;
struct reg_pw		reg_pw_2ghz_map[] =	REG_PW_24_MAP;
struct reg_pw		reg_pw_t_map[] =	REG_PW_T_MAP;
struct reg_pw		reg_pw_u1_map[] =	REG_PW_U1_MAP;
struct reg_pw		reg_pw_u2_map[] =	REG_PW_U2_MAP;
struct reg_pw		reg_pw_e_map[] =	REG_PW_E_MAP;
struct reg_pw		reg_pw_u3_map[] =	REG_PW_U3_MAP;
struct reg_pw		reg_pw_aad_map[] =	REG_PW_AAD_MAP;
struct reg_pw_subbands	reg_pw_map[] =		REG_PW_MAP;

static int ieee80211_regdomain_compare_cn(const void *a, const void *b)
{
	return(strcmp(((const struct reg_country*)a)->cn_name, 
		   ((const struct reg_country*)b)->cn_name));
}

static u32 ieee80211_regdomain2flag(u32 regdomain, u16 mhz)
{
	int i;
	
	for(i = 0; i < ARRAY_SIZE(reg_band_map); i++) {
		if(reg_band_map[i].rm_domain == regdomain) {
			if(mhz >= 2000 && mhz <= 3000)
				return((u32)reg_band_map[i].rm_domain_2ghz);
			if(mhz >= REG_5GHZ_MIN && 
			    mhz <= REG_5GHZ_MAX)
				return((u32)reg_band_map[i].rm_domain_5ghz);
		}
	}

	return((u32)DMN_DEBUG);
}

static int ieee80211_regdomain2name(u32 regdomain, char *name)
{
	int i;
	/* Linear search over the table */
	for(i = 0; i < ARRAY_SIZE(reg_domainname_map); i++)
		if(reg_domainname_map[i].rn_domain == regdomain) {
			memcpy(name, reg_domainname_map[i].rn_name, REGNAMSIZ);
			return 0;
		}
	return -EINVAL;
}

static const void *
bsearch(const void *key, const void *base0, size_t nmemb, size_t size,
    int (*compar)(const void *, const void *))
{
	const char *base = base0;
	int lim, cmp;
	const void *p;
	
	for (lim = nmemb; lim != 0; lim >>= 1) {
		p = base + (lim >> 1) * size;
		cmp = (*compar)(key, p);
		if (cmp == 0)
			return ((const void *)p);
		if (cmp > 0) {  /* key > p: move right */
			base = (const char *)p + size;
			lim--;
		} /* else move left */
	}
	return NULL;
}

int freq_to_subband(u16 freq, u32 sb_id)
{
	int i;
	struct reg_subband *sb;
	for (i=0; i < ARRAY_SIZE(reg_subbands_map); i++) {
		sb = &reg_subbands_map[i];
		if (freq <= sb->sb_max_freq && freq >= sb->sb_min_freq) {
			sb_id = sb->sb_id;
			return 0;
		}
	}
	return -EINVAL;
}

static void update_reg_subband_map()
{
	reg_subbands_map[DMN_SB_ISM24].sb_reg_pw = reg_pw_2ghz_map;
	reg_subbands_map[DMN_SB_ISM24].sb_reg_pw_size = ARRAY_SIZE(reg_pw_2ghz_map);

	reg_subbands_map[DMN_SB_TELEC].sb_reg_pw = reg_pw_t_map;
	reg_subbands_map[DMN_SB_TELEC].sb_reg_pw_size = ARRAY_SIZE(reg_pw_t_map);

	reg_subbands_map[DMN_SB_UNII_1].sb_reg_pw = reg_pw_u1_map;
	reg_subbands_map[DMN_SB_UNII_1].sb_reg_pw_size = ARRAY_SIZE(reg_pw_u1_map);

	reg_subbands_map[DMN_SB_UNII_2].sb_reg_pw = reg_pw_u2_map;
	reg_subbands_map[DMN_SB_UNII_2].sb_reg_pw_size = ARRAY_SIZE(reg_pw_u2_map);

	reg_subbands_map[DMN_SB_ETSI].sb_reg_pw = reg_pw_e_map;
	reg_subbands_map[DMN_SB_ETSI].sb_reg_pw_size = ARRAY_SIZE(reg_pw_e_map);

	reg_subbands_map[DMN_SB_UNII_3].sb_reg_pw = reg_pw_u3_map;
	reg_subbands_map[DMN_SB_UNII_3].sb_reg_pw_size = ARRAY_SIZE(reg_pw_u3_map);

	reg_subbands_map[DMN_SB_AAD].sb_reg_pw = reg_pw_aad_map;
	reg_subbands_map[DMN_SB_AAD].sb_reg_pw_size = ARRAY_SIZE(reg_pw_aad_map);
}

static int get_subband_pw_id(u32 regdomain, u32 sb_id, u32 pw_id)
{
	int i;
	struct reg_pw_subbands *pw_sb;
	for (i=0; i <= ARRAY_SIZE(reg_pw_map); i++) {
		pw_sb = &reg_pw_map[i];
		if (pw_sb->domain_id == regdomain) {
			switch(sb_id) {
				case DMN_SB_ISM24:
					pw_id = pw_sb->p_domain_2ghz;
					break;
				case DMN_SB_TELEC:
					pw_id = pw_sb->p_domain_telec;
					break;
				case DMN_SB_UNII_1:
					pw_id = pw_sb->p_domain_unii1;
					break;
				case DMN_SB_UNII_2:
					pw_id = pw_sb->p_domain_unii2;
					break;
				case DMN_SB_ETSI:
					pw_id = pw_sb->p_domain_etsi;
					break;
				case DMN_SB_UNII_3:
					pw_id = pw_sb->p_domain_unii3;
					break;
				case DMN_SB_AAD:
					pw_id = pw_sb->p_domain_aad;
					break;
				default:	
					return -EINVAL;
			}
			return 0;
		}
	}
	return -EINVAL;
}


u16 regdomain_mhz2ieee(u16 freq)
{
	if (freq >= REG_2GHZ_MIN &&  /* Channel 1 */
		freq <= REG_2GHZ_MAX) { /* Channel 26 */
		if (freq == 2484) /* Japan */
			return 14;
		if ((freq >= 2412) && (freq < 2484))
			return (freq - 2407) / 5;
		if (freq < 2512) /* No new channel until 2512 */
			return 0;
		return ((freq - 2512)/20) + 15; /* 15-26 */
	} else if (freq >= REG_5GHZ_MIN && /* Channel 1 */
		freq <= REG_5GHZ_MAX) { /* Channel 220 */
		return (freq - 5000) / 5;
	} else
	return 0;
}
EXPORT_SYMBOL(regdomain_mhz2ieee);

static inline u8 freq_band_id(u16 freq)
{
	if (freq >= REG_2GHZ_MIN &&  /* Channel 1 */
		freq <= REG_2GHZ_MAX) /* Channel 26 */
		return IEEE80211_24GHZ_BAND;
	else if (freq >= REG_5GHZ_MIN && /* Channel 1 */
		freq <= REG_5GHZ_MAX) /* Channel 220 */
		return IEEE80211_52GHZ_BAND;
	else {
		BUG();
		return -EINVAL;
	}
	
}

static inline int is_japan_regdomain_id(u32 regdomain_id)
{
	switch(regdomain_id) {
	case DMN_MKK1_MKKB:
	case DMN_MKK1_FCCA:
	case DMN_MKK1_MKKA:
	case DMN_MKK2_MKKA:
	case DMN_MKK1_MKKA1:
	case DMN_MKK1_MKKA2:
		return 1;
	default:
		return 0;
	}
}

static inline void setup_regdomain(struct ieee80211_regdomain *r,
		u8 regdomain_id, char *regdomain_name)
{
	strcpy(r->regdomain_name, regdomain_name);
	r->regdomain_id = regdomain_id;
}

static inline void setup_subband_restrictions(
		struct ieee80211_subband_restrictions *sb, char *name,
		u16 min_freq, u16 max_freq, u32 modulation_cap, u8 max_antenna_gain,
		u8 max_ir_ptmp, u8 max_ir_ptp,
		u8 max_eirp_ptmp, u8 max_eirp_ptp,
		char environment_cap)
{
	memcpy(sb->name, name, REGSBNAMSIZ);
	sb->band = freq_band_id(sb->min_freq);
	sb->min_freq = min_freq;
	sb->max_freq = max_freq;
	sb->modulation_cap = modulation_cap;
	sb->max_antenna_gain = max_antenna_gain;
	sb->max_ir_ptmp = max_ir_ptmp;
	sb->max_ir_ptp = max_ir_ptp;
	sb->max_eirp_ptmp = max_eirp_ptmp;
	sb->max_eirp_ptp = max_eirp_ptp;
	sb->environment_cap = environment_cap;
}

static inline void setup_reg_channel(u32 regdomain_id,
		struct ieee80211_regdomain_channel *reg_chan,
		u8 band, struct ieee80211_subband_restrictions *sb,
		u16 mhz)
{
	struct ieee80211_channel *chan = &reg_chan->channel;
	chan->freq = mhz;
	/* We set chan->val with what is common, drivers can overwrite 
	 * that if they so wish to */
	chan->chan = chan->val = regdomain_mhz2ieee(mhz);

	chan->flag = IEEE80211_CHAN_W_SCAN | 
		IEEE80211_CHAN_W_ACTIVE_SCAN |
		IEEE80211_CHAN_W_IBSS;

	/* For now assume standard dipole antenna */
	chan->power_level = sb->max_eirp_ptmp - REG_DIPOLE_ANTENNA_GAIN;

	if (chan->freq > REG_5GHZ_MIN) {
		if (is_japan_regdomain_id(regdomain_id) &&
			(chan->freq == 5170 || chan->freq == 5190 ||
			chan->freq == 5210 || chan->freq == 5230)) {
			/*
			* New regulatory rules in Japan have backwards
			* compatibility with old channels in 5.15-5.25
			* GHz band, but the station is not allowed to
			* use active scan on these old channels.
			*/
			chan->flag &= ~IEEE80211_CHAN_W_ACTIVE_SCAN;
		}
		if (is_japan_regdomain_id(regdomain_id) &&
			(chan->freq == 5260 || chan->freq == 5280 ||
			chan->freq == 5300 || chan->freq == 5320)) {
			/*
			 * IBSS is not allowed on 5.25-5.35 GHz band
			 * due to radar detection requirements.
			 */
			chan->flag &= ~IEEE80211_CHAN_W_IBSS;
		}
	}
}

static int alloc_reg_subbands(struct ieee80211_regdomain **reg,
		struct ieee80211_subband_restrictions *subbands[], int num_subs)
{
	int s_num, i;
	struct ieee80211_subband_restrictions *subband;

	/* Alloc regdomain */
	*reg = kmalloc(sizeof(struct ieee80211_regdomain), GFP_KERNEL);
	if (*reg == NULL)
		goto exit;
	memset(*reg, 0, sizeof(struct ieee80211_regdomain));

	/* Alloc subband restrictions */
	for(s_num = 0; s_num < num_subs; s_num++) {
		subband=kmalloc(sizeof(struct ieee80211_subband_restrictions), 
				GFP_KERNEL);
		if (subband == NULL)
			goto free_subbands;
		memset(subband, 0, 
				sizeof(struct ieee80211_subband_restrictions));
		subbands[s_num] = subband;
	}

	return 0;

free_subbands:
	for(i=0; i < s_num; i++)
		kfree(subbands[i]);
	kfree(*reg);
exit:
	return -ENOMEM;
}

static void free_subband_channel_list(struct list_head *channel_list)
{	
	struct ieee80211_regdomain_channel *reg_chan, *reg_chan_tmp;
	list_for_each_entry_safe(reg_chan, reg_chan_tmp, channel_list, list) {
		list_del(&reg_chan->list);
		kfree(reg_chan);
	}
}

static void free_regdomain(struct ieee80211_regdomain *reg)
{
	struct ieee80211_subband_restrictions *sb;
	int y;
	for (y=0; y<=REG_NUM_SUBBANDS; y++) {
		sb = &reg->subbands[y];
		free_subband_channel_list(&sb->channel_list);
		kfree(sb);
	}
	kfree(reg);
}

static void free_iso3166_reg_map_list(void)
{
	struct ieee80211_iso3166_reg_map *map, *tmp;
	list_for_each_entry_safe(map, tmp, &iso3166_reg_map_list, list) {
		list_del(&map->list);
		kfree(map);
	}
}

static inline int check_reg_addition(int r, char *reg_name)
{
	printk("%s: regulatory domain %18s - ", DRV_NAME, reg_name);
	switch(r) {
		case -ENOMEM:
			printk("Unable to allocate memory\n");
			break;
		case -EINVAL:
			printk("reg_name is invalid\n");
			break;
		case 0:
			printk("created\n");
			break;
		default:
			printk("Unexpected error detected\n");
			r = -EINVAL;
			break;
	}
	return r;
}

void print_regdomain(struct ieee80211_regdomain *reg)
{
	struct ieee80211_subband_restrictions *sb;
	struct ieee80211_regdomain_channel *reg_chan;
	int y;

	if (reg == NULL) {
		printk("Invalid regulatory domain encountered\n");
		return;
	}

	printk("Regulatory Domain:\t%s\tRegulatory Domain ID:\t0x%02x\n",
			reg->regdomain_name,
			reg->regdomain_id);

	
	for (y=0; y<= REG_NUM_SUBBANDS; y++) {
		sb = &reg->subbands[y];
		if (sb->modulation_cap & MODE_IEEE80211A) {
			printk("\tIEEE 802.11a");
			if (sb->modulation_cap & MODE_ATHEROS_TURBO)
				printk("(Turbo allowed)");
		}
		else if (sb->modulation_cap & MODE_IEEE80211G) {
			printk("\tIEEE 802.11bg");
			if (sb->modulation_cap & MODE_ATHEROS_TURBOG)
				printk("(Turbo allowed)");
		}
		else if (sb->modulation_cap & MODE_IEEE80211B)
				printk("\tIEEE 802.11b");
		else
			BUG();

		printk("\n\tmax_ir_ptmp:\t%d dBm\tmax_ir_ptp:\t%d dBm\n", 
			sb->max_ir_ptmp, sb->max_ir_ptp);
		printk("\n\tmax_eirp_ptmp:\t%d dBm\tmax_eirp_ptp:\t%d dBm\n", 
			sb->max_eirp_ptmp, sb->max_eirp_ptp);
		printk("\n\tmax_antenna_gain:\t%d dBi"
			"\n\tEnvironment capability:\t", sb->max_antenna_gain);
		if(sb->environment_cap == REG_CAP_INDOOR)
			printk("Indoor\n");
		else if(sb->environment_cap == REG_CAP_OUTDOOR)
			printk("Outdoor\n");
		else if(sb->environment_cap == REG_CAP_INOUT)
			printk("Indoor & Outdoor\n");
		else
			BUG();
		printk("\t\tChannel\tFreq\t\n");

		list_for_each_entry(reg_chan, &sb->channel_list, list){
			struct ieee80211_channel *chan = &reg_chan->channel;
			printk("\t\t%d\t%d\t\n", chan->chan, chan->freq);
		}
	}
	printk("\n");
}
EXPORT_SYMBOL(print_regdomain);

#ifdef CONFIG_REGDOMAINS_DBG
void print_iso3166_reg_map(void)
{
	struct ieee80211_iso3166_reg_map *map;
	char regdomain_name[REGNAMSIZ];
	printk("ISO3166 <--> regulatory domain map:\n");
	printk("\tCTRY\t-->\tRegdomain\n");
	read_lock(&iso3166_reg_rwlock);
	list_for_each_entry(map, &iso3166_reg_map_list, list) {
		if (ieee80211_regdomain2name(map->regdomain_id, regdomain_name))
			printk("\t%s\t-->\t0x%02x (reg_id not registered in "
					"regulatory db)\n", 
					map->alpha2, map->regdomain_id);
		else
			printk("\t%s\t-->\t%s\n", map->alpha2, regdomain_name);
	}
	read_unlock(&iso3166_reg_rwlock);
}
#else

void print_iso3166_reg_map(void)
{
	return;
}
#endif /* CONFIG_REGDOMAINS_DBG */
EXPORT_SYMBOL(print_iso3166_reg_map);

static inline void setup_iso3166_reg_map(struct ieee80211_iso3166_reg_map *map,
		char *alpha2, u32 regdomain_id)
{
	strcpy(map->alpha2, alpha2);
	map->regdomain_id = regdomain_id;
}

inline int iso3166_to_reg_exists(char *alpha2)
{
	struct ieee80211_iso3166_reg_map *map;
	read_lock(&iso3166_reg_rwlock);
	list_for_each_entry(map, &iso3166_reg_map_list, list)
		if(strncmp(map->alpha2, alpha2, ISOCOUNTRYSIZ2-1)==0) {
			read_unlock(&iso3166_reg_rwlock);
			return 1;
		}
	read_unlock(&iso3166_reg_rwlock);
	return 0;
}
EXPORT_SYMBOL(iso3166_to_reg_exists);

int iso3166_to_reg(char *alpha2, u32 regdomain_id)
{
	struct ieee80211_iso3166_reg_map *map;
	read_lock(&iso3166_reg_rwlock);
	list_for_each_entry(map, &iso3166_reg_map_list, list)
		if(strcmp(map->alpha2, alpha2)==0) {
			regdomain_id = map->regdomain_id;
			read_unlock(&iso3166_reg_rwlock);
			return 0;
		}
	read_unlock(&iso3166_reg_rwlock);
	return -EINVAL;
}
EXPORT_SYMBOL(iso3166_to_reg);

static int add_iso3166_reg_map(char *alpha2, u32 regdomain_id)
{
	int r = 0;
	struct ieee80211_iso3166_reg_map *map;
	write_lock(&iso3166_reg_rwlock);
	/* If not valid or, iso map does not exist or if already already present */
	if(!iso3166_1_exists(alpha2) || iso3166_to_reg_exists(alpha2)){
		r = -EPERM;
		goto unlock_and_exit;
	}
	map = kmalloc(sizeof(struct ieee80211_iso3166_reg_map), GFP_KERNEL);
	if(map == NULL) {
		r = -ENOMEM;
		goto unlock_and_exit;
	}
	setup_iso3166_reg_map(map, alpha2, regdomain_id);
	list_add_tail(&map->list, &iso3166_reg_map_list);
unlock_and_exit:
	write_unlock(&iso3166_reg_rwlock);
	return r;
}

static int __load_iso3166_reg_map_list(void)
{
	int i, r;
	struct reg_country *r_country;
	INIT_LIST_HEAD(&iso3166_reg_map_list);
	for (i = 0; i < ARRAY_SIZE(reg_ctry_map); i++) {
		r_country = &reg_ctry_map[i];
		r = add_iso3166_reg_map(r_country->cn_name, 
			r_country->cn_domain);
		BUG_ON(r);
	}
	return r;
}

/* Called by users of this module, cfg80211 at the moment */
int ieee80211_regdomain_init(u32 regdomain_id, struct ieee80211_regdomain **reg)
{
	struct ieee80211_subband_restrictions *subbands[REG_NUM_SUBBANDS];
	struct reg_channel *r_chan;
	char reg_name[REGNAMSIZ];
	u32 reg_2ghz_id;
	u32 reg_5ghz_id;
	u8 subbands_set = 0;
	u32 sb_mode_cap[REG_NUM_SUBBANDS];
	int i;
	int r = -ENOMEM;

	reg_2ghz_id = ieee80211_regdomain2flag(regdomain_id, REG_2GHZ_MIN);
	reg_5ghz_id = ieee80211_regdomain2flag(regdomain_id, REG_5GHZ_MIN);
	r = ieee80211_regdomain2name(regdomain_id, reg_name);
	BUG_ON(r);

	/* We're going to allocate memory for the new regdomain now,
	 * and then we're going to initialize it */

	/* Allocate bands, subbands */
	r = alloc_reg_subbands(reg, subbands, 
		ARRAY_SIZE(reg_subbands_map));
	if (r)
		goto exit;

	/* Iterate over our subbands and set respective power restrictions */

	/* First update subband power maps */
	update_reg_subband_map();
	/* Now lets set each subband's config */
	for (i=0; i<=REG_NUM_SUBBANDS; i++) {
		struct reg_subband *reg_sb = &reg_subbands_map[i];
		struct reg_pw *sb_reg_pw_map = reg_sb->sb_reg_pw;
		struct ieee80211_subband_restrictions *sb = subbands[i];
		u32 pw_sb_id;
		int pw_idx;

		/* Initialize all lists */
		INIT_LIST_HEAD(&subbands[i]->channel_list);

		/* Iterate through this subbands possible power maps, and setup
		 * the subband as corresponds to this regdomain */
		r = get_subband_pw_id(regdomain_id, i, pw_sb_id);
		BUG_ON(r);
		for (pw_idx = 0; pw_idx < reg_sb->sb_reg_pw_size; i++) {
			struct reg_pw *sb_reg_pw = sb_reg_pw_map + 
				(sizeof(struct reg_pw) * pw_idx);
			if (sb_reg_pw->p_domain != pw_sb_id)
				continue;
			/* We found a match */
			setup_subband_restrictions(sb, reg_sb->sb_name,
				reg_sb->sb_min_freq, reg_sb->sb_max_freq,
				sb_mode_cap[i], sb_reg_pw->max_antenna_gain,
				sb_reg_pw->max_ir_ptmp, sb_reg_pw->max_ir_ptp,
				sb_reg_pw->max_eirp_ptmp, 
				sb_reg_pw->max_eirp_ptp,
				sb_reg_pw->environment_cap);
			break; /* Start working on next subband */
		}
	}
	
	/* Now, iterate over the regdomain's channel list, and for each
 	 * subband, add channels to it */

	/* Iterate first over 2GHz band */
	for (i = 0; i < ARRAY_SIZE(reg_2ghz_channels); i++) {
		struct ieee80211_regdomain_channel *sb_chan; /* For subband */
		u32 sb_id;
		struct reg_channel *r_channel = &reg_2ghz_channels[i];
		if (r_channel->rc_domain != reg_2ghz_id || 
			reg_2ghz_id == DMN_NULL)
			continue;
		/* We know now channel belongs in this regdomain, build
		 * band for 2GHz and the ISM subband. Then start adding channels
		 * until we're done */

		if (r_channel->rc_freq < REG_2GHZ_MIN ||
			r_channel->rc_freq > REG_2GHZ_MAX)
			continue;

		if (subbands_set == 0) 
			subbands_set |= 1 << DMN_SB_ISM24;
	
		sb_mode_cap[DMN_SB_ISM24] |= r_channel->rc_modulation_cap;

		/* Add to subband linked list */
		sb_chan = kmalloc(sizeof(struct ieee80211_regdomain_channel), 
					GFP_KERNEL);
		if (sb_chan == NULL) {
			r = -ENOMEM;
			goto free_sb_channels;
		}
		memset(sb_chan, 0, 
			sizeof(struct ieee80211_regdomain_channel));
		setup_reg_channel(regdomain_id, sb_chan, 
				IEEE80211_24GHZ_BAND, &subbands[DMN_SB_ISM24],
				r_channel->rc_freq);
		list_add_tail(&sb_chan->list, &subbands[DMN_SB_ISM24]->channel_list);
	}

	/* Iterate over 5GHz band */
	for (i = 0; i < ARRAY_SIZE(reg_5ghz_channels); i++) {
		u32 sb_id;
		struct ieee80211_regdomain_channel *sb_chan; /* For subband */
		struct reg_subband *reg_sb;
		r_channel = reg_5ghz_channels[i];

		if (r_channel->rm_domain_5ghz != reg_5ghz_id || 
			reg_5ghz_id == DMN_NULL)
			continue;

		BUG_ON(freq_to_subband(r_channel->rc_freq, sb_id));
		BUG_ON(reg_subbands_map[sb_id] == NULL);

		reg_sb = reg_subbands_map[sb_id];
		subbands_set |= 1 << sb_id;

		/* Get Power band array, implement routine */
		sb_mode_cap[sb_id] |= r_channel->rc_modulation_cap;

		/* Add to subband linked list */
		reg_chan = kmalloc(sizeof(struct ieee80211_regdomain_channel), 
					GFP_KERNEL);
		if(reg_chan == NULL) {
			r = -ENOMEM;
			goto free_sb_channels;
		}
		memset(reg_chan, 0, 
			sizeof(struct ieee80211_regdomain_channel));
		setup_reg_channel(regdomain_id, reg_chan, 
				IEEE80211_52GHZ_BAND, &subbands[sb_id], 
				r_channel->rc_freq);
		list_add_tail(&sb_chan->list, &subbands[sb_id]->channel_list);
	}

	/* Setup regdomain, bands and subbands */
	setup_regdomain(reg, regdomain_id, reg_name);


free_sb_channels:
	for (i=0; i<=REG_NUM_SUBBANDS; i++) {
		if (subbands_set & (1 << i))
			free_subband_channel_list(
				&subbands[i]->channel_list);
		list_del(&subbands[i]->channel_list);
	}

exit:
	check_reg_addition(r, reg_name);
	return r;
}

static int regdomains_init(void)
{
	int r;
	printk("%s: %s v%s loaded\n", DRV_NAME, DRV_DESCRIPTION, DRV_VERSION);

	/* Load iso->reg map */
	r = __load_iso3166_reg_map_list();
	if(r)
		goto free_iso_reg_map_list;
	print_iso3166_reg_map();

	return r;
free_iso_reg_map_list:
	free_iso3166_reg_map_list();
	return r;
}

static void regdomains_exit(void)
{
	/* Iso->reg map */
	write_lock(&iso3166_reg_rwlock);
	free_iso3166_reg_map_list();
	list_del(&iso3166_reg_map_list);
	write_unlock(&iso3166_reg_rwlock);
	printk("%s: unloaded\n", DRV_NAME);
}

module_init(regdomains_init);
module_exit(regdomains_exit);
