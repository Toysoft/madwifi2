#ifndef _REGULATORY_COMMON_H_

#define REG_2GHZ_MIN	2412	/* 2GHz channel 1 */
#define REG_2GHZ_MAX	2732	/* 2GHz channel 26 */
#define REG_5GHZ_MIN	5005	/* 5GHz channel 1 */
#define REG_5GHZ_MAX	6100	/* 5GHz channel 220 */

#define DMN_NULL_WORLD	0x03 /* World default */
#define DMN_NULL_ETSIB	0x07
#define DMN_NULL_ETSIC	0x08
#define DMN_FCC1_FCCA	0x10
#define DMN_FCC1_WORLD	0x11
#define DMN_FCC2_FCCA	0x20
#define DMN_FCC2_WORLD	0x21
#define DMN_FCC2_ETSIC	0x22
#define DMN_FCC3_FCCA	0x3A
#define DMN_ETSI1_WORLD	0x37
#define DMN_ETSI3_ETSIA	0x32
#define DMN_ETSI2_WORLD	0x35
#define DMN_ETSI3_WORLD	0x36
#define DMN_ETSI4_WORLD	0x30
#define DMN_ETSI4_ETSIC	0x38
#define DMN_ETSI5_WORLD	0x39
#define DMN_ETSI6_WORLD	0x34
#define DMN_MKK1_MKKA	0x40
#define DMN_MKK1_MKKB	0x41
#define DMN_APL4_WORLD	0x42
#define DMN_MKK2_MKKA	0x43
#define DMN_APL2_WORLD	0x45
#define DMN_APL2_APLC	0x46
#define DMN_APL3_WORLD	0x47
#define DMN_MKK1_FCCA	0x48
#define DMN_APL2_APLD	0x49
#define DMN_MKK1_MKKA1	0x4A
#define DMN_MKK1_MKKA2	0x4B
#define DMN_APL1_WORLD	0x52
#define DMN_APL1_FCCA	0x53
#define DMN_APL1_APLA	0x54
#define DMN_APL1_ETSIC	0x55
#define DMN_APL2_ETSIC	0x56
#define DMN_APL5_WORLD	0x58
#define DMN_DEBUG_DEBUG	0xFF

/* 2.4GHz and 5GHz common subbands */
#define	DMN_APL1	(1<<0)
#define DMN_APL2	(1<<1)
#define DMN_APL3	(1<<2)
#define DMN_APL4	(1<<3)
#define DMN_APL5	(1<<4)
#define DMN_ETSI1	(1<<5)
#define DMN_ETSI2	(1<<6)
#define DMN_ETSI3	(1<<7)
#define DMN_ETSI4	(1<<8)
#define DMN_ETSI5	(1<<9)
#define DMN_ETSI6	(1<<10)
#define DMN_ETSIA	(1<<11)
#define DMN_ETSIB	(1<<12)
#define DMN_ETSIC	(1<<13)
#define DMN_FCC1	(1<<14)
#define DMN_FCC2	(1<<15)
#define DMN_FCC3	(1<<16)
#define DMN_FCCA	(1<<17)
#define DMN_APLD	(1<<18)
#define DMN_MKK1	(1<<19)
#define DMN_MKK2	(1<<20)
#define DMN_MKKA	(1<<21)
#define DMN_NULL	(1<<22)
#define DMN_WORLD	(1<<23)
#define DMN_DEBUG	(1<<31) /* Use on any map for debugging */

/* Default max antenna gain, in dBi */
#define REG_MAX_ANTENNA	6

/* Subbands definitions. We break the IEEE-802.11 spectrum into 7 
 * logical subbands */
#define DMN_SB_ISM24	0
#define DMN_SB_TELEC	1
#define DMN_SB_UNII_1	2
#define DMN_SB_UNII_2	3
#define DMN_SB_ETSI	4
#define DMN_SB_UNII_3	5
#define DMN_SB_AAD	6

#define REG_ISM24_MIN		2412	/* 2GHz channel 1 */
#define REG_ISM24_MAX		2732	/* 2GHz channel 26 */
#define REG_TELEC_MIN 		5005 	/* 5GHz channel 1 */
#define REG_TELEC_MAX 		5145 	/* 5GHz channel 29 */
#define REG_UNII_1_MIN 		5150 	/* 5GHz channel 30 */
#define REG_UNII_1_MAX 		5245	/* 5GHz channel 49 */
#define REG_UNII_2_MIN 		5250	/* 5GHz channel 50 */
#define REG_UNII_2_MAX 		5350	/* 5GHz channel 70 */
#define REG_ETSI_MIN		5355	/* 5GHz channel 71 */
#define REG_ETSI_MAX		5720	/* 5GHz channel 144 */
#define REG_UNII_3_MIN 		5725	/* 5GHz channel 145 */
#define REG_UNII_3_MAX 		5825	/* 5GHz channel 165 */
#define REG_AAD_MIN		5830	/* 5GHz channel 166 */
#define REG_AAD_MAX 		6100	/* 5GHz channel 220 */

/* Just FYI for now, technically DSRC is Dedicated Short Range Communications, 
 * for vehicles in North America (USA, Canada, and Mexico) */
#define REG_DSRC_MIN		5850
#define REG_DSRC_MAX		5925

/* 2.4GHz power subband IDs, note the entire world enables the 
 * 2.4GHz subband */
#define	DMN_P24_FCC	(2<<1)
#define	DMN_P24_ETSI	(2<<2)
#define DMN_P24_MKK	(2<<3)
#define	DMN_P24_WORLD	(2<<4)

/* Now for all 5GHz power subbands IDs, note that WORLD
 * regdomain has no 5GHz subbands */

/* TELEC subband */
#define	DMN_PT_MKK	(2<<0)
/* UNII-1 Used for indoor -- regulations require use of an integrated antenna */
#define	DMN_PU1_FCC	(2<<1)
#define	DMN_PU1_ETSI	(2<<2)
#define	DMN_PU1_MKK	(2<<3)
#define	DMN_PU1_APL	(2<<4)
/* UNII-2 Used for indoor -- regulations allow for a user-installable antenna */
#define	DMN_PU2_FCC	(2<<5)
#define	DMN_PU2_ETSI	(2<<6)
#define	DMN_PU2_APL	(2<<7)
/* ETSI band */
#define	DMN_PE_FCC	(2<<8)
#define	DMN_PE_ETSI	(2<<9)
/* UNII-3 Used for both outdoor and indoor use -- subject to 
 * Dynamic Frequency Selection (DFS, or radar avoidance */
#define	DMN_PU3_FCC	(2<<10)
#define	DMN_PU3_APL	(2<<11)
/* Asia/Africa/DSRC */
/* XXX: fill these power restrictions out, for now only
 * debug makes use of it */
//#define	DMN_PA_FCC	(2<<12)
//#define	DMN_PA_MKK	(2<<13)

#endif /*_REGULATORY_COMMON_H_ */
