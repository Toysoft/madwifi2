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

#include <linux/module.h>
#include "iso3166-1.h"

#define DRV_NAME        "iso3166_1"
#define DRV_DESCRIPTION "ISO 3166-1 support"
#define DRV_VERSION     ISO3166_1_VERSION

#define CONFIG_ISO3166_1_DEBUG 1

MODULE_AUTHOR("Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_VERSION(DRV_VERSION);

#define ISO_COUNTRIES { \
	 { 4, "AF", "AFG", "Afghanistan" }, \
	 { 8, "AL", "ALB", "Albania" }, \
	 { 10, "AQ", "ATA", "Antarctica" }, \
	 { 12, "DZ", "DZA", "Algeria" }, \
	 { 16, "AS", "ASM", "American Samoa" }, \
	 { 20, "AD", "AND", "Andorra" }, \
	 { 24, "AO", "AGO", "Angola" }, \
	 { 28, "AG", "ATG", "Antigua and Barbuda" }, \
	 { 31, "AZ", "AZE", "Azerbaijan" }, \
	 { 32, "AR", "ARG", "Argentina" }, \
	 { 36, "AU", "AUS", "Australia" }, \
	 { 40, "AT", "AUT", "Austria" }, \
	 { 44, "BS", "BHS", "Bahamas" }, \
	 { 48, "BH", "BHR", "Bahrain" }, \
	 { 50, "BD", "BGD", "Bangladesh" }, \
	 { 51, "AM", "ARM", "Armenia" }, \
	 { 52, "BB", "BRB", "Barbados" }, \
	 { 56, "BE", "BEL", "Belgium" }, \
	 { 60, "BM", "BMU", "Bermuda" }, \
	 { 64, "BT", "BTN", "Bhutan" }, \
	 { 68, "BO", "BOL", "Bolivia" }, \
	 { 70, "BA", "BIH", "Bosnia and Herzegovina" }, \
	 { 72, "BW", "BWA", "Botswana" }, \
	 { 74, "BV", "BVT", "Bouvet Island" }, \
	 { 76, "BR", "BRA", "Brazil" }, \
	 { 84, "BZ", "BLZ", "Belize" }, \
	 { 86, "IO", "IOT", "British Indian Ocean Territory" }, \
	 { 90, "SB", "SLB", "Solomon Islands" }, \
	 { 92, "VG", "VGB", "Virgin Islands, British" }, \
	 { 96, "BN", "BRN", "Brunei Darussalam" }, \
	 { 100, "BG", "BGR", "Bulgaria" }, \
	 { 104, "MM", "MMR", "Myanmar" }, \
	 { 108, "BI", "BDI", "Burundi" }, \
	 { 112, "BY", "BLR", "Belarus" }, \
	 { 116, "KH", "KHM", "Cambodia" }, \
	 { 120, "CM", "CMR", "Cameroon" }, \
	 { 124, "CA", "CAN", "Canada" }, \
	 { 132, "CV", "CPV", "Cape Verde" }, \
	 { 136, "KY", "CYM", "Cayman Islands" }, \
	 { 140, "CF", "CAF", "Central African Republic" }, \
	 { 144, "LK", "LKA", "Sri Lanka" }, \
	 { 148, "TD", "TCD", "Chad" }, \
	 { 152, "CL", "CHL", "Chile" }, \
	 { 156, "CN", "CHN", "China" }, \
	 { 158, "TW", "TWN", "Taiwan, Province of China" }, \
	 { 162, "CX", "CXR", "Christmas Island" }, \
	 { 166, "CC", "CCK", "Cocos (Keeling Islands" }, \
	 { 170, "CO", "COL", "Colombia" }, \
	 { 174, "KM", "COM", "Comoros" }, \
	 { 175, "YT", "MYT", "Mayotte" }, \
	 { 178, "CG", "COG", "Congo" }, \
	 { 180, "CD", "COD", "Congo, the Democratic Republic of the" }, \
	 { 184, "CK", "COK", "Cook Islands" }, \
	 { 188, "CR", "CRI", "Costa Rica" }, \
	 { 191, "HR", "HRV", "Croatia" }, \
	 { 192, "CU", "CUB", "Cuba" }, \
	 { 196, "CY", "CYP", "Cyprus" }, \
	 { 203, "CZ", "CZE", "Czech Republic" }, \
	 { 204, "BJ", "BEN", "Benin" }, \
	 { 208, "DK", "DNK", "Denmark" }, \
	 { 212, "DM", "DMA", "Dominica" }, \
	 { 214, "DO", "DOM", "Dominican Republic" }, \
	 { 218, "EC", "ECU", "Ecuador" }, \
	 { 222, "SV", "SLV", "El Salvador" }, \
	 { 226, "GQ", "GNQ", "Equatorial Guinea" }, \
	 { 231, "ET", "ETH", "Ethiopia" }, \
	 { 232, "ER", "ERI", "Eritrea" }, \
	 { 233, "EE", "EST", "Estonia" }, \
	 { 234, "FO", "FRO", "Faroe Islands" }, \
	 { 238, "FK", "FLK", "Falkland Islands (Malvinas" }, \
	 { 239, "GS", "SGS", "South Georgia and the South Sandwich Islands" }, \
	 { 242, "FJ", "FJI", "Fiji" }, \
	 { 246, "FI", "FIN", "Finland" }, \
	 { 248, "AX", "ALA", "Åland Islands" }, \
	 { 250, "FR", "FRA", "France" }, \
	 { 254, "GF", "GUF", "French Guiana" }, \
	 { 258, "PF", "PYF", "French Polynesia" }, \
	 { 260, "TF", "ATF", "French Southern Territories" }, \
	 { 262, "DJ", "DJI", "Djibouti" }, \
	 { 266, "GA", "GAB", "Gabon" }, \
	 { 268, "GE", "GEO", "Georgia" }, \
	 { 270, "GM", "GMB", "Gambia" }, \
	 { 275, "PS", "PSE", "Palestinian Territory, Occupied" }, \
	 { 276, "DE", "DEU", "Germany" }, \
	 { 288, "GH", "GHA", "Ghana" }, \
	 { 292, "GI", "GIB", "Gibraltar" }, \
	 { 296, "KI", "KIR", "Kiribati" }, \
	 { 300, "GR", "GRC", "Greece" }, \
	 { 304, "GL", "GRL", "Greenland" }, \
	 { 308, "GD", "GRD", "Grenada" }, \
	 { 312, "GP", "GLP", "Guadeloupe" }, \
	 { 316, "GU", "GUM", "Guam" }, \
	 { 320, "GT", "GTM", "Guatemala" }, \
	 { 324, "GN", "GIN", "Guinea" }, \
	 { 328, "GY", "GUY", "Guyana" }, \
	 { 332, "HT", "HTI", "Haiti" }, \
	 { 334, "HM", "HMD", "Heard Island and McDonald Islands" }, \
	 { 336, "VA", "VAT", "Holy See (Vatican City State" }, \
	 { 340, "HN", "HND", "Honduras" }, \
	 { 344, "HK", "HKG", "Hong Kong" }, \
	 { 348, "HU", "HUN", "Hungary" }, \
	 { 352, "IS", "ISL", "Iceland" }, \
	 { 356, "IN", "IND", "India" }, \
	 { 360, "ID", "IDN", "Indonesia" }, \
	 { 364, "IR", "IRN", "Iran, Islamic Republic of" }, \
	 { 368, "IQ", "IRQ", "Iraq" }, \
	 { 372, "IE", "IRL", "Ireland" }, \
	 { 376, "IL", "ISR", "Israel" }, \
	 { 380, "IT", "ITA", "Italy" }, \
	 { 384, "CI", "CIV", "Côte d'Ivoire" }, \
	 { 388, "JM", "JAM", "Jamaica" }, \
	 { 392, "JP", "JPN", "Japan" }, \
	 { 398, "KZ", "KAZ", "Kazakhstan" }, \
	 { 400, "JO", "JOR", "Jordan" }, \
	 { 404, "KE", "KEN", "Kenya" }, \
	 { 408, "KP", "PRK", "Korea, Democratic People's Republic of" }, \
	 { 410, "KR", "KOR", "Korea, Republic of" }, \
	 { 414, "KW", "KWT", "Kuwait" }, \
	 { 417, "KG", "KGZ", "Kyrgyzstan" }, \
	 { 418, "LA", "LAO", "Lao People's Democratic Republic" }, \
	 { 422, "LB", "LBN", "Lebanon" }, \
	 { 426, "LS", "LSO", "Lesotho" }, \
	 { 428, "LV", "LVA", "Latvia" }, \
	 { 430, "LR", "LBR", "Liberia" }, \
	 { 434, "LY", "LBY", "Libyan Arab Jamahiriya" }, \
	 { 438, "LI", "LIE", "Liechtenstein" }, \
	 { 440, "LT", "LTU", "Lithuania" }, \
	 { 442, "LU", "LUX", "Luxembourg" }, \
	 { 446, "MO", "MAC", "Macao" }, \
	 { 450, "MG", "MDG", "Madagascar" }, \
	 { 454, "MW", "MWI", "Malawi" }, \
	 { 458, "MY", "MYS", "Malaysia" }, \
	 { 462, "MV", "MDV", "Maldives" }, \
	 { 466, "ML", "MLI", "Mali" }, \
	 { 470, "MT", "MLT", "Malta" }, \
	 { 474, "MQ", "MTQ", "Martinique" }, \
	 { 478, "MR", "MRT", "Mauritania" }, \
	 { 480, "MU", "MUS", "Mauritius" }, \
	 { 484, "MX", "MEX", "Mexico" }, \
	 { 492, "MC", "MCO", "Monaco" }, \
	 { 496, "MN", "MNG", "Mongolia" }, \
	 { 498, "MD", "MDA", "Moldova, Republic of" }, \
	 { 500, "MS", "MSR", "Montserrat" }, \
	 { 504, "MA", "MAR", "Morocco" }, \
	 { 508, "MZ", "MOZ", "Mozambique" }, \
	 { 512, "OM", "OMN", "Oman" }, \
	 { 516, "NA", "NAM", "Namibia" }, \
	 { 520, "NR", "NRU", "Nauru" }, \
	 { 524, "NP", "NPL", "Nepal" }, \
	 { 528, "NL", "NLD", "Netherlands" }, \
	 { 530, "AN", "ANT", "Netherlands Antilles" }, \
	 { 533, "AW", "ABW", "Aruba" }, \
	 { 540, "NC", "NCL", "New Caledonia" }, \
	 { 548, "VU", "VUT", "Vanuatu" }, \
	 { 554, "NZ", "NZL", "New Zealand" }, \
	 { 558, "NI", "NIC", "Nicaragua" }, \
	 { 562, "NE", "NER", "Niger" }, \
	 { 566, "NG", "NGA", "Nigeria" }, \
	 { 570, "NU", "NIU", "Niue" }, \
	 { 574, "NF", "NFK", "Norfolk Island" }, \
	 { 578, "NO", "NOR", "Norway" }, \
	 { 580, "MP", "MNP", "Northern Mariana Islands" }, \
	 { 581, "UM", "UMI", "United States Minor Outlying Islands" }, \
	 { 583, "FM", "FSM", "Micronesia, Federated States of" }, \
	 { 584, "MH", "MHL", "Marshall Islands" }, \
	 { 585, "PW", "PLW", "Palau" }, \
	 { 586, "PK", "PAK", "Pakistan" }, \
	 { 591, "PA", "PAN", "Panama" }, \
	 { 598, "PG", "PNG", "Papua New Guinea" }, \
	 { 600, "PY", "PRY", "Paraguay" }, \
	 { 604, "PE", "PER", "Peru" }, \
	 { 608, "PH", "PHL", "Philippines" }, \
	 { 612, "PN", "PCN", "Pitcairn" }, \
	 { 616, "PL", "POL", "Poland" }, \
	 { 620, "PT", "PRT", "Portugal" }, \
	 { 624, "GW", "GNB", "Guinea-Bissau" }, \
	 { 626, "TL", "TLS", "Timor-Leste" }, \
	 { 630, "PR", "PRI", "Puerto Rico" }, \
	 { 634, "QA", "QAT", "Qatar" }, \
	 { 638, "RE", "REU", "Réunion" }, \
	 { 642, "RO", "ROU", "Romania" }, \
	 { 643, "RU", "RUS", "Russian Federation" }, \
	 { 646, "RW", "RWA", "Rwanda" }, \
	 { 654, "SH", "SHN", "Saint Helena" }, \
	 { 659, "KN", "KNA", "Saint Kitts and Nevis" }, \
	 { 660, "AI", "AIA", "Anguilla" }, \
	 { 662, "LC", "LCA", "Saint Lucia" }, \
	 { 666, "PM", "SPM", "Saint Pierre and Miquelon" }, \
	 { 670, "VC", "VCT", "Saint Vincent and the Grenadines" }, \
	 { 674, "SM", "SMR", "San Marino" }, \
	 { 678, "ST", "STP", "Sao Tome and Principe" }, \
	 { 682, "SA", "SAU", "Saudi Arabia" }, \
	 { 686, "SN", "SEN", "Senegal" }, \
	 { 690, "SC", "SYC", "Seychelles" }, \
	 { 694, "SL", "SLE", "Sierra Leone" }, \
	 { 702, "SG", "SGP", "Singapore" }, \
	 { 703, "SK", "SVK", "Slovakia" }, \
	 { 704, "VN", "VNM", "Viet Nam" }, \
	 { 705, "SI", "SVN", "Slovenia" }, \
	 { 706, "SO", "SOM", "Somalia" }, \
	 { 710, "ZA", "ZAF", "South Africa" }, \
	 { 716, "ZW", "ZWE", "Zimbabwe" }, \
	 { 724, "ES", "ESP", "Spain" }, \
	 { 732, "EH", "ESH", "Western Sahara" }, \
	 { 736, "SD", "SDN", "Sudan" }, \
	 { 740, "SR", "SUR", "Suriname" }, \
	 { 744, "SJ", "SJM", "Svalbard and Jan Mayen" }, \
	 { 748, "SZ", "SWZ", "Swaziland" }, \
	 { 752, "SE", "SWE", "Sweden" }, \
	 { 756, "CH", "CHE", "Switzerland" }, \
	 { 760, "SY", "SYR", "Syrian Arab Republic" }, \
	 { 762, "TJ", "TJK", "Tajikistan" }, \
	 { 764, "TH", "THA", "Thailand" }, \
	 { 768, "TG", "TGO", "Togo" }, \
	 { 772, "TK", "TKL", "Tokelau" }, \
	 { 776, "TO", "TON", "Tonga" }, \
	 { 780, "TT", "TTO", "Trinidad and Tobago" }, \
	 { 784, "AE", "ARE", "United Arab Emirates" }, \
	 { 788, "TN", "TUN", "Tunisia" }, \
	 { 792, "TR", "TUR", "Turkey" }, \
	 { 795, "TM", "TKM", "Turkmenistan" }, \
	 { 796, "TC", "TCA", "Turks and Caicos Islands" }, \
	 { 798, "TV", "TUV", "Tuvalu" }, \
	 { 800, "UG", "UGA", "Uganda" }, \
	 { 804, "UA", "UKR", "Ukraine" }, \
	 { 807, "MK", "MKD", "Macedonia, the former Yugoslav Republic of" }, \
	 { 818, "EG", "EGY", "Egypt" }, \
	 { 826, "GB", "GBR", "United Kingdom" }, \
	 { 831, "GG", "GGY", "Guernsey" }, \
	 { 832, "JE", "JEY", "Jersey" }, \
	 { 833, "IM", "IMN", "Isle of Man" }, \
	 { 834, "TZ", "TZA", "Tanzania, United Republic of" }, \
	 { 840, "US", "USA", "United States" }, \
	 { 850, "VI", "VIR", "Virgin Islands, U.S." }, \
	 { 854, "BF", "BFA", "Burkina Faso" }, \
	 { 858, "UY", "URY", "Uruguay" }, \
	 { 860, "UZ", "UZB", "Uzbekistan" }, \
	 { 862, "VE", "VEN", "Venezuela" }, \
	 { 876, "WF", "WLF", "Wallis and Futuna" }, \
	 { 882, "WS", "WSM", "Samoa" }, \
	 { 887, "YE", "YEM", "Yemen" }, \
	 { 891, "CS", "SCG", "Serbia and Montenegro" }, \
	 { 894, "ZM", "ZMB", "Zambia" } \
}

struct iso3166_1 iso3166_1_map[] = ISO_COUNTRIES;
EXPORT_SYMBOL(iso3166_1_map);

#ifdef CONFIG_ISO3166_1_DEBUG
static void iso3166_1_show_all(void)
{
	int i;
	struct iso3166_1 *iso;
	char alpha2[ISOCOUNTRYSIZ2];
	char alpha3[ISOCOUNTRYSIZ3];
	char country[ISOCOUNTRYSIZ];

	for(i = 0; i < NUM_ISO_COUNTRIES; i++) {
		iso = &iso3166_1_map[i];
		BUG_ON(get_iso3166_1_alpha2(iso->numeric, alpha2));
		BUG_ON(get_iso3166_1_alpha3(iso->alpha2, alpha3));
		BUG_ON(get_iso3166_1_country(iso->alpha2, country));
		printk("%d\t%s\t%s\t%s\n", iso->numeric, alpha2, 
			alpha3, country);
	}
}
#else
#define iso3166_1_show_all()	do { /* nothing */ } while (0)
#endif /* CONFIG_ISO3166_1_DEBUG */

static int iso3166_1_init(void)
{
	printk("%s: %s loaded, last updated %s\n", 
			DRV_NAME, DRV_DESCRIPTION, DRV_VERSION);
	iso3166_1_show_all();
	return 0;
}

static void iso3166_1_exit(void)
{
	printk("%s: unloaded\n", DRV_NAME);
	return;
}

int get_iso3166_1_numeric(char *alpha2, u16 numeric)
{
	int i;
	struct iso3166_1 *iso;
	for(i = 0; i < NUM_ISO_COUNTRIES; i++){
		iso = &iso3166_1_map[i];
		if(strncmp(iso->alpha2, alpha2, ISOCOUNTRYSIZ2) == 0) {
			numeric = iso->numeric;
			return 0;
		}
	}
	return -ENODATA;
}
EXPORT_SYMBOL(get_iso3166_1_numeric);

int get_iso3166_1_alpha2(u16 numeric, char *req)
{
	int i;
	struct iso3166_1 *iso;
	for(i = 0; i < NUM_ISO_COUNTRIES; i++) {
		iso = &iso3166_1_map[i];
		if (iso->numeric == numeric) {
			memcpy(req, iso->alpha2, ISOCOUNTRYSIZ2-1);
			req[ISOCOUNTRYSIZ2-1] = '\0';
			return 0;
		}
	}
	return -ENODATA;
}
EXPORT_SYMBOL(get_iso3166_1_alpha2);

int get_iso3166_1_alpha3(char *alpha2, char *req)
{
	int i;
	struct iso3166_1 *iso;
	for(i = 0; i < NUM_ISO_COUNTRIES; i++) {
		iso = &iso3166_1_map[i];
		if(strncmp(iso->alpha2, alpha2, ISOCOUNTRYSIZ2) == 0) {
			memcpy(req, iso->alpha3, ISOCOUNTRYSIZ3-1);
			req[ISOCOUNTRYSIZ3-1] = '\0';
			return 0;
		}
	}
	return -ENODATA;
}
EXPORT_SYMBOL(get_iso3166_1_alpha3);


int get_iso3166_1_country(char *alpha2, char *req)
{
	int i;
	struct iso3166_1 *iso;
	for(i = 0; i < NUM_ISO_COUNTRIES; i++) {
		iso = &iso3166_1_map[i];
		if(strncmp(iso->alpha2, alpha2, ISOCOUNTRYSIZ2) == 0) {
			memcpy(req, iso->country, ISOCOUNTRYSIZ-1);
			req[ISOCOUNTRYSIZ-1] = '\0';
			return 0;
		}
	}
	return -ENODATA;
}

int iso3166_1_exists(char *alpha2)
{
	u16 numeric = 0;
	return get_iso3166_1_numeric(alpha2, numeric);
}
EXPORT_SYMBOL(iso3166_1_exists);

module_init(iso3166_1_init);
module_exit(iso3166_1_exit);
