#ifndef _ISO3166_1_H
#define _ISO3166_1_H
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

/* 
 * ISO 3166-1, as part of the ISO 3166 standard, provides codes for the names 
 * of countries and dependent areas. It was first published in 1974 by 
 * the International Organization for Standardization (ISO) and defines three 
 * different codes for each area:
 *
 *     * ISO 3166-1 alpha-2, a two-letter system with many applications, 
 *       most notably the Internet top-level domains (ccTLD) for countries.
 *     * ISO 3166-1 alpha-3, a three-letter system.
 *     * ISO 3166-1 numeric, a three-digit numerical system, which is 
 *     identical to that defined by the United Nations Statistical Division.
 *
 * Although this would usually be only used in userspace IEEE-802.11d
 * has made use of ISO-3166-1 alpha 2. This mapping was added
 * to enhance stack support for IEEE-802.11d and 802.11 Regulatory 
 * Domain control.
 *
 */

#include <linux/list.h>

#define ISO3166_1_VERSION	"2006-09-18"
#define ISOCOUNTRYSIZ2		3
#define ISOCOUNTRYSIZ3		4
#define ISOCOUNTRYSIZ		51
#define NUM_ISO_COUNTRIES	243

/**
 * struct iso3166_1 - ISO3166-1 country map
 *
 * @numeric: ISO3166-1 numeric of country
 * @alpha2: ISO3166-1 alpha2 of country
 * @alpha3: ISO3166-1 alpha3 of country
 * @country: ISO3166-1 country name
 *
 * This structure is used to create the map of ISO3166-1 countries with their
 * respective numeriv value, alpha2, alpha3 and official country name.
 */
struct iso3166_1 {
	u16 numeric;
	char alpha2[ISOCOUNTRYSIZ2];
	char alpha3[ISOCOUNTRYSIZ3];
	char country[ISOCOUNTRYSIZ];
};

int get_iso3166_1_numeric(char *, u16);
int get_iso3166_1_alpha2(u16, char *);
/* These three expect an alpha2 as an index for search */
int get_iso3166_1_alpha3(char *, char *);
int get_iso3166_1_country(char *, char *);
int iso3166_1_exists(char *);

#endif /* _ISO3166_1_H */
