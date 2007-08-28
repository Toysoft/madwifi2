/*
 * Initial register settings functions
 *
 * Copyright (c) 2007 The MadWiFi Team <www.madwifi.org>
 * Copyright (c) 2004, 2005 Reyk Floeter <reyk@vantronix.net>
 *
 * This file is released under GPLv2
 */

#include "ath5k.h"
#include "ath5k_reg.h"

/*
 * MAC/PHY REGISTERS
 */


/*
 * Mode-independent initial register writes
 */

struct ath5k_ini {
	u16	ini_register;
	u32	ini_value;

	enum {
		AR5K_INI_WRITE = 0,	/* Default */
		AR5K_INI_READ = 1,	/* Cleared on read */
	} ini_mode;
};

/*
 * Mode specific initial register values
 */

struct ath5k_ini_mode {
	u16	mode_register;
	u32	mode_value[5];
};

#ifdef CONFIG_ATHEROS_AR5K_AR5210
/* Initial register settings for AR5210 */
static const struct ath5k_ini ar5210_ini[] = {
	/* PCU and MAC registers */
	{ AR5K_NOQCU_TXDP0,	0 },
	{ AR5K_NOQCU_TXDP1,	0 },
	{ AR5K_RXDP,		0 },
	{ AR5K_CR,		0 },
	{ AR5K_ISR,		0, AR5K_INI_READ },
	{ AR5K_IMR,		0 },
	{ AR5K_IER,		AR5K_IER_DISABLE },
	{ AR5K_BSR,		0, AR5K_INI_READ },
	{ AR5K_TXCFG,		AR5K_DMASIZE_128B },
	{ AR5K_RXCFG,		AR5K_DMASIZE_128B },
	{ AR5K_CFG,		AR5K_INIT_CFG },
	{ AR5K_TOPS,		AR5K_INIT_TOPS },
	{ AR5K_RXNOFRM,		AR5K_INIT_RXNOFRM },
	{ AR5K_RPGTO,		AR5K_INIT_RPGTO },
	{ AR5K_TXNOFRM,		AR5K_INIT_TXNOFRM },
	{ AR5K_SFR,		0 },
	{ AR5K_MIBC,		0 },
	{ AR5K_MISC,		0 },
	{ AR5K_RX_FILTER_5210,	0 },
	{ AR5K_MCAST_FILTER0_5210, 0 },
	{ AR5K_MCAST_FILTER1_5210, 0 },
	{ AR5K_TX_MASK0,	0 },
	{ AR5K_TX_MASK1,	0 },
	{ AR5K_CLR_TMASK,	0 },
	{ AR5K_TRIG_LVL,	AR5K_TUNE_MIN_TX_FIFO_THRES },
	{ AR5K_DIAG_SW_5210,	0 },
	{ AR5K_RSSI_THR,	AR5K_TUNE_RSSI_THRES },
	{ AR5K_TSF_L32_5210,	0 },
	{ AR5K_TIMER0_5210,	0 },
	{ AR5K_TIMER1_5210,	0xffffffff },
	{ AR5K_TIMER2_5210,	0xffffffff },
	{ AR5K_TIMER3_5210,	1 },
	{ AR5K_CFP_DUR_5210,	0 },
	{ AR5K_CFP_PERIOD_5210,	0 },
	/* PHY registers */
	{ AR5K_PHY(0),	0x00000047 },
	{ AR5K_PHY_AGC,	0x00000000 },
	{ AR5K_PHY(3),	0x09848ea6 },
	{ AR5K_PHY(4),	0x3d32e000 },
	{ AR5K_PHY(5),	0x0000076b },
	{ AR5K_PHY_ACT,	AR5K_PHY_ACT_DISABLE },
	{ AR5K_PHY(8),	0x02020200 },
	{ AR5K_PHY(9),	0x00000e0e },
	{ AR5K_PHY(10),	0x0a020201 },
	{ AR5K_PHY(11),	0x00036ffc },
	{ AR5K_PHY(12),	0x00000000 },
	{ AR5K_PHY(13),	0x00000e0e },
	{ AR5K_PHY(14),	0x00000007 },
	{ AR5K_PHY(15),	0x00020100 },
	{ AR5K_PHY(16),	0x89630000 },
	{ AR5K_PHY(17),	0x1372169c },
	{ AR5K_PHY(18),	0x0018b633 },
	{ AR5K_PHY(19),	0x1284613c },
	{ AR5K_PHY(20),	0x0de8b8e0 },
	{ AR5K_PHY(21),	0x00074859 },
	{ AR5K_PHY(22),	0x7e80beba },
	{ AR5K_PHY(23),	0x313a665e },
	{ AR5K_PHY_AGCCTL, 0x00001d08 },
	{ AR5K_PHY(25),	0x0001ce00 },
	{ AR5K_PHY(26),	0x409a4190 },
	{ AR5K_PHY(28),	0x0000000f },
	{ AR5K_PHY(29),	0x00000080 },
	{ AR5K_PHY(30),	0x00000004 },
	{ AR5K_PHY(31),	0x00000018 }, 	/* 0x987c */
	{ AR5K_PHY(64),	0x00000000 }, 	/* 0x9900 */
	{ AR5K_PHY(65),	0x00000000 },
	{ AR5K_PHY(66),	0x00000000 },
	{ AR5K_PHY(67),	0x00800000 },
	{ AR5K_PHY(68),	0x00000003 },
	/* BB gain table (64bytes) */
	{ AR5K_BB_GAIN(0), 0x00000000 },
	{ AR5K_BB_GAIN(1), 0x00000020 },
	{ AR5K_BB_GAIN(2), 0x00000010 },
	{ AR5K_BB_GAIN(3), 0x00000030 },
	{ AR5K_BB_GAIN(4), 0x00000008 },
	{ AR5K_BB_GAIN(5), 0x00000028 },
	{ AR5K_BB_GAIN(6), 0x00000028 },
	{ AR5K_BB_GAIN(7), 0x00000004 },
	{ AR5K_BB_GAIN(8), 0x00000024 },
	{ AR5K_BB_GAIN(9), 0x00000014 },
	{ AR5K_BB_GAIN(10), 0x00000034 },
	{ AR5K_BB_GAIN(11), 0x0000000c },
	{ AR5K_BB_GAIN(12), 0x0000002c },
	{ AR5K_BB_GAIN(13), 0x00000002 },
	{ AR5K_BB_GAIN(14), 0x00000022 },
	{ AR5K_BB_GAIN(15), 0x00000012 },
	{ AR5K_BB_GAIN(16), 0x00000032 },
	{ AR5K_BB_GAIN(17), 0x0000000a },
	{ AR5K_BB_GAIN(18), 0x0000002a },
	{ AR5K_BB_GAIN(19), 0x00000001 },
	{ AR5K_BB_GAIN(20), 0x00000021 },
	{ AR5K_BB_GAIN(21), 0x00000011 },
	{ AR5K_BB_GAIN(22), 0x00000031 },
	{ AR5K_BB_GAIN(23), 0x00000009 },
	{ AR5K_BB_GAIN(24), 0x00000029 },
	{ AR5K_BB_GAIN(25), 0x00000005 },
	{ AR5K_BB_GAIN(26), 0x00000025 },
	{ AR5K_BB_GAIN(27), 0x00000015 },
	{ AR5K_BB_GAIN(28), 0x00000035 },
	{ AR5K_BB_GAIN(29), 0x0000000d },
	{ AR5K_BB_GAIN(30), 0x0000002d },
	{ AR5K_BB_GAIN(31), 0x00000003 },
	{ AR5K_BB_GAIN(32), 0x00000023 },
	{ AR5K_BB_GAIN(33), 0x00000013 },
	{ AR5K_BB_GAIN(34), 0x00000033 },
	{ AR5K_BB_GAIN(35), 0x0000000b },
	{ AR5K_BB_GAIN(36), 0x0000002b },
	{ AR5K_BB_GAIN(37), 0x00000007 },
	{ AR5K_BB_GAIN(38), 0x00000027 },
	{ AR5K_BB_GAIN(39), 0x00000017 },
	{ AR5K_BB_GAIN(40), 0x00000037 },
	{ AR5K_BB_GAIN(41), 0x0000000f },
	{ AR5K_BB_GAIN(42), 0x0000002f },
	{ AR5K_BB_GAIN(43), 0x0000002f },
	{ AR5K_BB_GAIN(44), 0x0000002f },
	{ AR5K_BB_GAIN(45), 0x0000002f },
	{ AR5K_BB_GAIN(46), 0x0000002f },
	{ AR5K_BB_GAIN(47), 0x0000002f },
	{ AR5K_BB_GAIN(48), 0x0000002f },
	{ AR5K_BB_GAIN(49), 0x0000002f },
	{ AR5K_BB_GAIN(50), 0x0000002f },
	{ AR5K_BB_GAIN(51), 0x0000002f },
	{ AR5K_BB_GAIN(52), 0x0000002f },
	{ AR5K_BB_GAIN(53), 0x0000002f },
	{ AR5K_BB_GAIN(54), 0x0000002f },
	{ AR5K_BB_GAIN(55), 0x0000002f },
	{ AR5K_BB_GAIN(56), 0x0000002f },
	{ AR5K_BB_GAIN(57), 0x0000002f },
	{ AR5K_BB_GAIN(58), 0x0000002f },
	{ AR5K_BB_GAIN(59), 0x0000002f },
	{ AR5K_BB_GAIN(60), 0x0000002f },
	{ AR5K_BB_GAIN(61), 0x0000002f },
	{ AR5K_BB_GAIN(62), 0x0000002f },
	{ AR5K_BB_GAIN(63), 0x0000002f },
	/* 5110 RF gain table (64btes) */
	{ AR5K_RF_GAIN(0), 0x0000001d },
	{ AR5K_RF_GAIN(1), 0x0000005d },
	{ AR5K_RF_GAIN(2), 0x0000009d },
	{ AR5K_RF_GAIN(3), 0x000000dd },
	{ AR5K_RF_GAIN(4), 0x0000011d },
	{ AR5K_RF_GAIN(5), 0x00000021 },
	{ AR5K_RF_GAIN(6), 0x00000061 },
	{ AR5K_RF_GAIN(7), 0x000000a1 },
	{ AR5K_RF_GAIN(8), 0x000000e1 },
	{ AR5K_RF_GAIN(9), 0x00000031 },
	{ AR5K_RF_GAIN(10), 0x00000071 },
	{ AR5K_RF_GAIN(11), 0x000000b1 },
	{ AR5K_RF_GAIN(12), 0x0000001c },
	{ AR5K_RF_GAIN(13), 0x0000005c },
	{ AR5K_RF_GAIN(14), 0x00000029 },
	{ AR5K_RF_GAIN(15), 0x00000069 },
	{ AR5K_RF_GAIN(16), 0x000000a9 },
	{ AR5K_RF_GAIN(17), 0x00000020 },
	{ AR5K_RF_GAIN(18), 0x00000019 },
	{ AR5K_RF_GAIN(19), 0x00000059 },
	{ AR5K_RF_GAIN(20), 0x00000099 },
	{ AR5K_RF_GAIN(21), 0x00000030 },
	{ AR5K_RF_GAIN(22), 0x00000005 },
	{ AR5K_RF_GAIN(23), 0x00000025 },
	{ AR5K_RF_GAIN(24), 0x00000065 },
	{ AR5K_RF_GAIN(25), 0x000000a5 },
	{ AR5K_RF_GAIN(26), 0x00000028 },
	{ AR5K_RF_GAIN(27), 0x00000068 },
	{ AR5K_RF_GAIN(28), 0x0000001f },
	{ AR5K_RF_GAIN(29), 0x0000001e },
	{ AR5K_RF_GAIN(30), 0x00000018 },
	{ AR5K_RF_GAIN(31), 0x00000058 },
	{ AR5K_RF_GAIN(32), 0x00000098 },
	{ AR5K_RF_GAIN(33), 0x00000003 },
	{ AR5K_RF_GAIN(34), 0x00000004 },
	{ AR5K_RF_GAIN(35), 0x00000044 },
	{ AR5K_RF_GAIN(36), 0x00000084 },
	{ AR5K_RF_GAIN(37), 0x00000013 },
	{ AR5K_RF_GAIN(38), 0x00000012 },
	{ AR5K_RF_GAIN(39), 0x00000052 },
	{ AR5K_RF_GAIN(40), 0x00000092 },
	{ AR5K_RF_GAIN(41), 0x000000d2 },
	{ AR5K_RF_GAIN(42), 0x0000002b },
	{ AR5K_RF_GAIN(43), 0x0000002a },
	{ AR5K_RF_GAIN(44), 0x0000006a },
	{ AR5K_RF_GAIN(45), 0x000000aa },
	{ AR5K_RF_GAIN(46), 0x0000001b },
	{ AR5K_RF_GAIN(47), 0x0000001a },
	{ AR5K_RF_GAIN(48), 0x0000005a },
	{ AR5K_RF_GAIN(49), 0x0000009a },
	{ AR5K_RF_GAIN(50), 0x000000da },
	{ AR5K_RF_GAIN(51), 0x00000006 },
	{ AR5K_RF_GAIN(52), 0x00000006 },
	{ AR5K_RF_GAIN(53), 0x00000006 },
	{ AR5K_RF_GAIN(54), 0x00000006 },
	{ AR5K_RF_GAIN(55), 0x00000006 },
	{ AR5K_RF_GAIN(56), 0x00000006 },
	{ AR5K_RF_GAIN(57), 0x00000006 },
	{ AR5K_RF_GAIN(58), 0x00000006 },
	{ AR5K_RF_GAIN(59), 0x00000006 },
	{ AR5K_RF_GAIN(60), 0x00000006 },
	{ AR5K_RF_GAIN(61), 0x00000006 },
	{ AR5K_RF_GAIN(62), 0x00000006 },
	{ AR5K_RF_GAIN(63), 0x00000006 },
	/* PHY activation */
	{ AR5K_PHY(53), 0x00000020 },
	{ AR5K_PHY(51), 0x00000004 },
	{ AR5K_PHY(50), 0x00060106 },
	{ AR5K_PHY(39), 0x0000006d },
	{ AR5K_PHY(48), 0x00000000 },
	{ AR5K_PHY(52), 0x00000014 },
	{ AR5K_PHY_ACT, AR5K_PHY_ACT_ENABLE },
};
#endif

#ifdef CONFIG_ATHEROS_AR5K_AR5211
/* Initial register settings for AR5211 */
static const struct ath5k_ini ar5211_ini[] = {
	{ AR5K_RXDP,		0x00000000 },
	{ AR5K_RTSD0,		0x84849c9c },
	{ AR5K_RTSD1,		0x7c7c7c7c },
	{ AR5K_RXCFG,		0x00000005 },
	{ AR5K_MIBC,		0x00000000 },
	{ AR5K_TOPS,		0x00000008 },
	{ AR5K_RXNOFRM,		0x00000008 },
	{ AR5K_TXNOFRM,		0x00000010 },
	{ AR5K_RPGTO,		0x00000000 },
	{ AR5K_RFCNT,		0x0000001f },
	{ AR5K_QUEUE_TXDP(0),	0x00000000 },
	{ AR5K_QUEUE_TXDP(1),	0x00000000 },
	{ AR5K_QUEUE_TXDP(2),	0x00000000 },
	{ AR5K_QUEUE_TXDP(3),	0x00000000 },
	{ AR5K_QUEUE_TXDP(4),	0x00000000 },
	{ AR5K_QUEUE_TXDP(5),	0x00000000 },
	{ AR5K_QUEUE_TXDP(6),	0x00000000 },
	{ AR5K_QUEUE_TXDP(7),	0x00000000 },
	{ AR5K_QUEUE_TXDP(8),	0x00000000 },
	{ AR5K_QUEUE_TXDP(9),	0x00000000 },
	{ AR5K_DCU_FP,		0x00000000 },
	{ AR5K_STA_ID1,		0x00000000 },
	{ AR5K_BSS_ID0,		0x00000000 },
	{ AR5K_BSS_ID1,		0x00000000 },
	{ AR5K_RSSI_THR,	0x00000000 },
	{ AR5K_CFP_PERIOD_5211,	0x00000000 },
	{ AR5K_TIMER0_5211,	0x00000030 },
	{ AR5K_TIMER1_5211,	0x0007ffff },
	{ AR5K_TIMER2_5211,	0x01ffffff },
	{ AR5K_TIMER3_5211,	0x00000031 },
	{ AR5K_CFP_DUR_5211,	0x00000000 },
	{ AR5K_RX_FILTER_5211,	0x00000000 },
	{ AR5K_MCAST_FILTER0_5211, 0x00000000 },
	{ AR5K_MCAST_FILTER1_5211, 0x00000002 },
	{ AR5K_DIAG_SW_5211,	0x00000000 },
	{ AR5K_ADDAC_TEST,	0x00000000 },
	{ AR5K_DEFAULT_ANTENNA,	0x00000000 },
	/* PHY registers */
	{ AR5K_PHY_AGC,	0x00000000 },
	{ AR5K_PHY(3),	0x2d849093 },
	{ AR5K_PHY(4),	0x7d32e000 },
	{ AR5K_PHY(5),	0x00000f6b },
	{ AR5K_PHY_ACT,	0x00000000 },
	{ AR5K_PHY(11),	0x00026ffe },
	{ AR5K_PHY(12),	0x00000000 },
	{ AR5K_PHY(15),	0x00020100 },
	{ AR5K_PHY(16),	0x206a017a },
	{ AR5K_PHY(19),	0x1284613c },
	{ AR5K_PHY(21),	0x00000859 },
	{ AR5K_PHY(26),	0x409a4190 },	/* 0x9868 */
	{ AR5K_PHY(27),	0x050cb081 },
	{ AR5K_PHY(28),	0x0000000f },
	{ AR5K_PHY(29),	0x00000080 },
	{ AR5K_PHY(30),	0x0000000c },
	{ AR5K_PHY(64),	0x00000000 },
	{ AR5K_PHY(65),	0x00000000 },
	{ AR5K_PHY(66),	0x00000000 },
	{ AR5K_PHY(67),	0x00800000 },
	{ AR5K_PHY(68),	0x00000001 },
	{ AR5K_PHY(71),	0x0000092a },
	{ AR5K_PHY_IQ,	0x00000000 },
	{ AR5K_PHY(73),	0x00058a05 },
	{ AR5K_PHY(74),	0x00000001 },
	{ AR5K_PHY(75),	0x00000000 },
	{ AR5K_PHY_PAPD_PROBE, 0x00000000 },
	{ AR5K_PHY(77),	0x00000000 },	/* 0x9934 */
	{ AR5K_PHY(78),	0x00000000 },	/* 0x9938 */
	{ AR5K_PHY(79),	0x0000003f },	/* 0x993c */
	{ AR5K_PHY(80),	0x00000004 },
	{ AR5K_PHY(82),	0x00000000 },
	{ AR5K_PHY(83),	0x00000000 },
	{ AR5K_PHY(84),	0x00000000 },
	{ AR5K_PHY_RADAR, 0x5d50f14c },
	{ AR5K_PHY(86),	0x00000018 },
	{ AR5K_PHY(87),	0x004b6a8e },
	/* Power table (32bytes) */
	{ AR5K_PHY_PCDAC_TXPOWER(1), 0x06ff05ff },
	{ AR5K_PHY_PCDAC_TXPOWER(2), 0x07ff07ff },
	{ AR5K_PHY_PCDAC_TXPOWER(3), 0x08ff08ff },
	{ AR5K_PHY_PCDAC_TXPOWER(4), 0x09ff09ff },
	{ AR5K_PHY_PCDAC_TXPOWER(5), 0x0aff0aff },
	{ AR5K_PHY_PCDAC_TXPOWER(6), 0x0bff0bff },
	{ AR5K_PHY_PCDAC_TXPOWER(7), 0x0cff0cff },
	{ AR5K_PHY_PCDAC_TXPOWER(8), 0x0dff0dff },
	{ AR5K_PHY_PCDAC_TXPOWER(9), 0x0fff0eff },
	{ AR5K_PHY_PCDAC_TXPOWER(10), 0x12ff12ff },
	{ AR5K_PHY_PCDAC_TXPOWER(11), 0x14ff13ff },
	{ AR5K_PHY_PCDAC_TXPOWER(12), 0x16ff15ff },
	{ AR5K_PHY_PCDAC_TXPOWER(13), 0x19ff17ff },
	{ AR5K_PHY_PCDAC_TXPOWER(14), 0x1bff1aff },
	{ AR5K_PHY_PCDAC_TXPOWER(15), 0x1eff1dff },
	{ AR5K_PHY_PCDAC_TXPOWER(16), 0x23ff20ff },
	{ AR5K_PHY_PCDAC_TXPOWER(17), 0x27ff25ff },
	{ AR5K_PHY_PCDAC_TXPOWER(18), 0x2cff29ff },
	{ AR5K_PHY_PCDAC_TXPOWER(19), 0x31ff2fff },
	{ AR5K_PHY_PCDAC_TXPOWER(20), 0x37ff34ff },
	{ AR5K_PHY_PCDAC_TXPOWER(21), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(22), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(23), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(24), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(25), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(26), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(27), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(28), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(29), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(30), 0x3aff3aff },
	{ AR5K_PHY_PCDAC_TXPOWER(31), 0x3aff3aff },
	{ AR5K_PHY_CCKTXCTL, 0x00000000 },
	{ AR5K_PHY(642), 0x503e4646 },
	{ AR5K_PHY_GAIN_2GHZ, 0x6480416c },
	{ AR5K_PHY(644), 0x0199a003 },
	{ AR5K_PHY(645), 0x044cd610 },
	{ AR5K_PHY(646), 0x13800040 },
	{ AR5K_PHY(647), 0x1be00060 },
	{ AR5K_PHY(648), 0x0c53800a },
	{ AR5K_PHY(649), 0x0014df3b },
	{ AR5K_PHY(650), 0x000001b5 },
	{ AR5K_PHY(651), 0x00000020 },
};

/* Initial mode-specific settings for AR5211
 * XXX: how about gTurbo ? RF5111 supports it, how about AR5211 ?
 */
static const struct ath5k_ini_mode ar5211_ini_mode[] = {
	{ AR5K_TXCFG,
	/*	  a/XR	      aTurbo	  b	      g(OFDM?)	  gTurbo (N/A) */
 		{ 0x00000017, 0x00000017, 0x00000017, 0x00000017, 0x00000017 } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(0),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(1),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(2),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(3),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(4),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(5),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(6),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(7),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(8),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(9),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_DCU_GBL_IFS_SLOT,
		{ 0x00000168, 0x000001e0, 0x000001b8, 0x00000168, 0x00000168 } },
	{ AR5K_DCU_GBL_IFS_SIFS,
		{ 0x00000230, 0x000001e0, 0x000000b0, 0x00000230, 0x00000230 } },
	{ AR5K_DCU_GBL_IFS_EIFS,
		{ 0x00000d98, 0x00001180, 0x00001f48, 0x00000d98, 0x00000d98 } },
	{ AR5K_DCU_GBL_IFS_MISC,
		{ 0x0000a0e0, 0x00014068, 0x00005880, 0x0000a0e0, 0x0000a0e0 } },
	{ AR5K_TIME_OUT,
		{ 0x04000400, 0x08000800, 0x20003000, 0x04000400, 0x04000400 } },
	{ AR5K_USEC_5211,
		{ 0x0e8d8fa7, 0x0e8d8fcf, 0x01608f95, 0x0e8d8fa7, 0x0e8d8fa7 } },
	{ AR5K_PHY_TURBO,
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0x9820,
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 } },
	{ 0x9824,
		{ 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e, 0x00000e0e } },
	{ 0x9828,
		{ 0x0a020001, 0x0a020001, 0x05010000, 0x0a020001, 0x0a020001 } },
	{ 0x9834,
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },
	{ 0x9838,
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b } },
	{ 0x9844,
		{ 0x1372169c, 0x137216a5, 0x137216a8, 0x1372169c, 0x1372169c } },
	{ 0x9848,
		{ 0x0018ba67, 0x0018ba67, 0x0018ba69, 0x0018ba69, 0x0018ba69 } },
	{ 0x9850,
		{ 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0, 0x0c28b4e0 } },
	{ AR5K_PHY_SIG,
		{ 0x7e800d2e, 0x7e800d2e, 0x7ec00d2e, 0x7e800d2e, 0x7e800d2e } },
	{ AR5K_PHY_AGCCOARSE,
		{ 0x31375d5e, 0x31375d5e, 0x313a5d5e, 0x31375d5e, 0x31375d5e } },
	{ AR5K_PHY_AGCCTL,
		{ 0x0000bd10, 0x0000bd10, 0x0000bd38, 0x0000bd10, 0x0000bd10 } },
	{ AR5K_PHY_NF,
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },
	{ AR5K_PHY_RX_DELAY,
		{ 0x00002710, 0x00002710, 0x0000157c, 0x00002710, 0x00002710 } },
	{ 0x9918,
		{ 0x00000190, 0x00000190, 0x00000084, 0x00000190, 0x00000190 } },
	{ AR5K_PHY_FRAME_CTL_5211,
		{ 0x6fe01020, 0x6fe01020, 0x6fe00920, 0x6fe01020, 0x6fe01020 } },
	{ AR5K_PHY_PCDAC_TXPOWER(0),
		{ 0x05ff14ff, 0x05ff14ff, 0x05ff14ff, 0x05ff19ff, 0x05ff19ff } },
	{ AR5K_RF_BUFFER_CONTROL_4,
		{ 0x00000010, 0x00000014, 0x00000010, 0x00000010, 0x00000010 } },
};
#endif

#ifdef CONFIG_ATHEROS_AR5K_AR5212
/* Initial register settings for AR5212 */
static const struct ath5k_ini ar5212_ini[] = {
	{ AR5K_RXDP,		0x00000000 },
	{ AR5K_RXCFG,		0x00000005 },
	{ AR5K_MIBC,		0x00000000 },
	{ AR5K_TOPS,		0x00000008 },
	{ AR5K_RXNOFRM,		0x00000008 },
	{ AR5K_TXNOFRM,		0x00000010 },
	{ AR5K_RPGTO,		0x00000000 },
	{ AR5K_RFCNT,		0x0000001f },
	{ AR5K_QUEUE_TXDP(0),	0x00000000 },
	{ AR5K_QUEUE_TXDP(1),	0x00000000 },
	{ AR5K_QUEUE_TXDP(2),	0x00000000 },
	{ AR5K_QUEUE_TXDP(3),	0x00000000 },
	{ AR5K_QUEUE_TXDP(4),	0x00000000 },
	{ AR5K_QUEUE_TXDP(5),	0x00000000 },
	{ AR5K_QUEUE_TXDP(6),	0x00000000 },
	{ AR5K_QUEUE_TXDP(7),	0x00000000 },
	{ AR5K_QUEUE_TXDP(8),	0x00000000 },
	{ AR5K_QUEUE_TXDP(9),	0x00000000 },
	{ AR5K_DCU_FP,		0x00000000 },
	{ AR5K_DCU_TXP,		0x00000000 },
	{ AR5K_DCU_TX_FILTER,	0x00000000 },
	/* Unknown table */
	{ 0x1078, 0x00000000 },
	{ 0x10b8, 0x00000000 },
	{ 0x10f8, 0x00000000 },
	{ 0x1138, 0x00000000 },
	{ 0x1178, 0x00000000 },
	{ 0x11b8, 0x00000000 },
	{ 0x11f8, 0x00000000 },
	{ 0x1238, 0x00000000 },
	{ 0x1278, 0x00000000 },
	{ 0x12b8, 0x00000000 },
	{ 0x12f8, 0x00000000 },
	{ 0x1338, 0x00000000 },
	{ 0x1378, 0x00000000 },
	{ 0x13b8, 0x00000000 },
	{ 0x13f8, 0x00000000 },
	{ 0x1438, 0x00000000 },
	{ 0x1478, 0x00000000 },
	{ 0x14b8, 0x00000000 },
	{ 0x14f8, 0x00000000 },
	{ 0x1538, 0x00000000 },
	{ 0x1578, 0x00000000 },
	{ 0x15b8, 0x00000000 },
	{ 0x15f8, 0x00000000 },
	{ 0x1638, 0x00000000 },
	{ 0x1678, 0x00000000 },
	{ 0x16b8, 0x00000000 },
	{ 0x16f8, 0x00000000 },
	{ 0x1738, 0x00000000 },
	{ 0x1778, 0x00000000 },
	{ 0x17b8, 0x00000000 },
	{ 0x17f8, 0x00000000 },
	{ 0x103c, 0x00000000 },
	{ 0x107c, 0x00000000 },
	{ 0x10bc, 0x00000000 },
	{ 0x10fc, 0x00000000 },
	{ 0x113c, 0x00000000 },
	{ 0x117c, 0x00000000 },
	{ 0x11bc, 0x00000000 },
	{ 0x11fc, 0x00000000 },
	{ 0x123c, 0x00000000 },
	{ 0x127c, 0x00000000 },
	{ 0x12bc, 0x00000000 },
	{ 0x12fc, 0x00000000 },
	{ 0x133c, 0x00000000 },
	{ 0x137c, 0x00000000 },
	{ 0x13bc, 0x00000000 },
	{ 0x13fc, 0x00000000 },
	{ 0x143c, 0x00000000 },
	{ 0x147c, 0x00000000 },
	{ AR5K_STA_ID1,		0x00000000 },
	{ AR5K_BSS_ID0,		0x00000000 },
	{ AR5K_BSS_ID1,		0x00000000 },
	{ AR5K_RSSI_THR,	0x00000000 },
	{ AR5K_BEACON_5211,	0x00000000 },
	{ AR5K_CFP_PERIOD_5211,	0x00000000 },
	{ AR5K_TIMER0_5211,	0x00000030 },
	{ AR5K_TIMER1_5211,	0x0007ffff },
	{ AR5K_TIMER2_5211,	0x01ffffff },
	{ AR5K_TIMER3_5211,	0x00000031 },
	{ AR5K_CFP_DUR_5211,	0x00000000 },
	{ AR5K_RX_FILTER_5211,	0x00000000 },
	{ AR5K_DIAG_SW_5211,	0x00000000 },
	{ AR5K_ADDAC_TEST,	0x00000000 },
	{ AR5K_DEFAULT_ANTENNA,	0x00000000 },
	{ 0x805c, 0xffffc7ff },
	{ 0x8080, 0x00000000 },
	{ AR5K_NAV_5211,	0x00000000 },
	{ AR5K_RTS_OK_5211,	0x00000000 },
	{ AR5K_RTS_FAIL_5211,	0x00000000 },
	{ AR5K_ACK_FAIL_5211,	0x00000000 },
	{ AR5K_FCS_FAIL_5211,	0x00000000 },
	{ AR5K_BEACON_CNT_5211,	0x00000000 },
	{ AR5K_XRMODE,		0x2a82301a },
	{ AR5K_XRDELAY,		0x05dc01e0 },
	{ AR5K_XRTIMEOUT,	0x1f402710 },
	{ AR5K_XRCHIRP,		0x01f40000 },
	{ AR5K_XRSTOMP,		0x00001e1c },
	{ AR5K_SLEEP0,		0x0002aaaa },
	{ AR5K_SLEEP1,		0x02005555 },
	{ AR5K_SLEEP2,		0x00000000 },
	{ AR5K_BSS_IDM0,	0xffffffff },
	{ AR5K_BSS_IDM1,	0x0000ffff },
	{ AR5K_TXPC,		0x00000000 },
	{ AR5K_PROFCNT_TX,	0x00000000 },
	{ AR5K_PROFCNT_RX,	0x00000000 },
	{ AR5K_PROFCNT_RXCLR,	0x00000000 },
	{ AR5K_PROFCNT_CYCLE,	0x00000000 },
	{ 0x80fc, 0x00000088 },
	{ AR5K_RATE_DUR(0),	0x00000000 },
	{ AR5K_RATE_DUR(1),	0x0000008c },
	{ AR5K_RATE_DUR(2),	0x000000e4 },
	{ AR5K_RATE_DUR(3),	0x000002d5 },
	{ AR5K_RATE_DUR(4),	0x00000000 },
	{ AR5K_RATE_DUR(5),	0x00000000 },
	{ AR5K_RATE_DUR(6),	0x000000a0 },
	{ AR5K_RATE_DUR(7),	0x000001c9 },
	{ AR5K_RATE_DUR(8),	0x0000002c },
	{ AR5K_RATE_DUR(9),	0x0000002c },
	{ AR5K_RATE_DUR(10),	0x00000030 },
	{ AR5K_RATE_DUR(11),	0x0000003c },
	{ AR5K_RATE_DUR(12),	0x0000002c },
	{ AR5K_RATE_DUR(13),	0x0000002c },
	{ AR5K_RATE_DUR(14),	0x00000030 },
	{ AR5K_RATE_DUR(15),	0x0000003c },
	{ AR5K_RATE_DUR(16),	0x00000000 },
	{ AR5K_RATE_DUR(17),	0x00000000 },
	{ AR5K_RATE_DUR(18),	0x00000000 },
	{ AR5K_RATE_DUR(19),	0x00000000 },
	{ AR5K_RATE_DUR(20),	0x00000000 },
	{ AR5K_RATE_DUR(21),	0x00000000 },
	{ AR5K_RATE_DUR(22),	0x00000000 },
	{ AR5K_RATE_DUR(23),	0x00000000 },
	{ AR5K_RATE_DUR(24),	0x000000d5 },
	{ AR5K_RATE_DUR(25),	0x000000df },
	{ AR5K_RATE_DUR(26),	0x00000102 },
	{ AR5K_RATE_DUR(27),	0x0000013a },
	{ AR5K_RATE_DUR(28),	0x00000075 },
	{ AR5K_RATE_DUR(29),	0x0000007f },
	{ AR5K_RATE_DUR(30),	0x000000a2 },
	{ AR5K_RATE_DUR(31),	0x00000000 },
	{ 0x8100, 0x00010002},
	{ AR5K_TSF_PARM,	0x00000001 },
	{ 0x8108, 0x000000c0 },
	{ AR5K_PHY_ERR_FIL,	0x00000000 },
	{ 0x8110, 0x00000168 },
	{ 0x8114, 0x00000000 },
	/* Some kind of table
	 * also notice ...03<-02<-01<-00) */
	{ 0x87c0, 0x03020100 },
	{ 0x87c4, 0x07060504 },
	{ 0x87c8, 0x0b0a0908 },
	{ 0x87cc, 0x0f0e0d0c },
	{ 0x87d0, 0x13121110 },
	{ 0x87d4, 0x17161514 },
	{ 0x87d8, 0x1b1a1918 },
	{ 0x87dc, 0x1f1e1d1c },
	/* loop ? */
	{ 0x87e0, 0x03020100 },
	{ 0x87e4, 0x07060504 },
	{ 0x87e8, 0x0b0a0908 },
	{ 0x87ec, 0x0f0e0d0c },
	{ 0x87f0, 0x13121110 },
	{ 0x87f4, 0x17161514 },
	{ 0x87f8, 0x1b1a1918 },
	{ 0x87fc, 0x1f1e1d1c },
	/* PHY registers */
	{ AR5K_PHY_AGC,	0x00000000 },
	{ AR5K_PHY(3),	0xad848e19 },
	{ AR5K_PHY(4),	0x7d28e000 },
	{ AR5K_PHY_TIMING_3, 0x9c0a9f6b },
	{ AR5K_PHY_ACT,	0x00000000 },
	{ AR5K_PHY(11),	0x00022ffe },
	{ AR5K_PHY(15),	0x00020100 },
	{ AR5K_PHY(16),	0x206a017a },
	{ AR5K_PHY(19),	0x1284613c },
	{ AR5K_PHY(21),	0x00000859 },
	{ AR5K_PHY(64),	0x00000000 },
	{ AR5K_PHY(65),	0x00000000 },
	{ AR5K_PHY(66),	0x00000000 },
	{ AR5K_PHY(67),	0x00800000 },
	{ AR5K_PHY(68),	0x00000001 },
	{ AR5K_PHY(71),	0x0000092a },
	{ AR5K_PHY_IQ,	0x05100000 },
	{ AR5K_PHY(74), 0x00000001 },
	{ AR5K_PHY(75), 0x00000004 },
	{ AR5K_PHY_TXPOWER_RATE1, 0x1e1f2022 },
	{ AR5K_PHY_TXPOWER_RATE2, 0x0a0b0c0d },
	{ AR5K_PHY_TXPOWER_RATE_MAX, 0x0000003f },
	{ AR5K_PHY(80), 0x00000004 },
	{ AR5K_PHY(82), 0x9280b212 },
	{ AR5K_PHY_RADAR, 0x5d50e188 },
	{ AR5K_PHY(86),	0x000000ff },
	{ AR5K_PHY(87),	0x004b6a8e },
	{ AR5K_PHY(90),	0x000003ce },
	{ AR5K_PHY(92),	0x192fb515 },
	{ AR5K_PHY(93),	0x00000000 },
	{ AR5K_PHY(94),	0x00000001 },
	{ AR5K_PHY(95),	0x00000000 },
	/* Power table (32bytes) */
	{ AR5K_PHY_PCDAC_TXPOWER(1), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(2), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(3), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(4), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(5), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(6), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(7), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(8), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(9), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(10), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(11), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(12), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(13), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(14), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(15), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(16), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(17), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(18), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(19), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(20), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(21), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(22), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(23), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(24), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(25), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(26), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(27), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(28), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(29), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(30), 0x10ff10ff },
	{ AR5K_PHY_PCDAC_TXPOWER(31), 0x10ff10ff },
	{ AR5K_PHY(644), 0x0080a333 },
	{ AR5K_PHY(645), 0x00206c10 },
	{ AR5K_PHY(646), 0x009c4060 },
	{ AR5K_PHY(647), 0x1483800a },
	{ AR5K_PHY(648), 0x01831061 },
	{ AR5K_PHY(649), 0x00000400 },
	{ AR5K_PHY(650), 0x000001b5 },
	{ AR5K_PHY(651), 0x00000000 },
	{ AR5K_PHY_TXPOWER_RATE3, 0x20202020 },
	{ AR5K_PHY_TXPOWER_RATE2, 0x20202020 },
	{ AR5K_PHY(655), 0x13c889af },
	{ AR5K_PHY(656), 0x38490a20 },
	{ AR5K_PHY(657), 0x00007bb6 },
	{ AR5K_PHY(658), 0x0fff3ffc },
	{ AR5K_PHY_CCKTXCTL, 0x00000000 },
};

/* Initial mode-specific settings for AR5212 */
static const struct ath5k_ini_mode ar5212_ini_mode[] = {
	{ AR5K_TXCFG,
	/*	  a/XR	      aTurbo	  b	      g (DYN)	  gTurbo */
		{ 0x00008107, 0x00008107, 0x00008107, 0x00008107, 0x00008107 } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(0),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(1),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(2),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(3),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(4),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(5),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(6),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(7),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(8),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_QUEUE_DFS_LOCAL_IFS(9),
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f } },
	{ AR5K_DCU_GBL_IFS_SIFS,
		{ 0x00000230, 0x000001e0, 0x000000b0, 0x00000160, 0x000001e0 } },
	{ AR5K_DCU_GBL_IFS_SLOT,
		{ 0x00000168, 0x000001e0, 0x000001b8, 0x0000018c, 0x000001e0 } },
	{ AR5K_DCU_GBL_IFS_EIFS,
		{ 0x00000e60, 0x00001180, 0x00001f1c, 0x00003e38, 0x00001180 } },
	{ AR5K_DCU_GBL_IFS_MISC,
		{ 0x0000a0e0, 0x00014068, 0x00005880, 0x0000b0e0, 0x00014068 } },
	{ AR5K_TIME_OUT,
		{ 0x03e803e8, 0x06e006e0, 0x04200420, 0x08400840, 0x06e006e0 } },
};

/* Initial mode-specific settings for AR5212 + RF5111 */
static const struct ath5k_ini_mode ar5212_rf5111_ini_mode[] = {
	{ AR5K_USEC_5211,
	/*	  a/XR	      aTurbo	  b	      g		  gTurbo */
		{ 0x128d8fa7, 0x09880fcf, 0x04e00f95, 0x128d8fab, 0x09880fcf } },
	{ AR5K_PHY_TURBO,
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000003 } },
	{ 0x9820,
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 } },
	{ 0x9824,
		{ 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e, 0x00000e0e } },
	{ 0x9828,
		{ 0x0a020001, 0x0a020001, 0x05010100, 0x0a020001, 0x0a020001 } },
	{ 0x9834,
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },
	{ 0x9838,
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b } },
	{ 0x9844,
		{ 0x1372161c, 0x13721c25, 0x13721728, 0x137216a2, 0x13721c25 } },
	{ 0x9848,
		{ 0x0018da5a, 0x0018da5a, 0x0018ca69, 0x0018ca69, 0x0018ca69 } },
	{ 0x9850,
		{ 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0 } },
	{ AR5K_PHY_SIG,
		{ 0x7e800d2e, 0x7e800d2e, 0x7ee84d2e, 0x7ee84d2e, 0x7e800d2e } },
	{ AR5K_PHY_AGCCOARSE,
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137615e } },
	{ AR5K_PHY_AGCCTL,
		{ 0x00009d10, 0x00009d10, 0x00009d18, 0x00009d10, 0x00009d10 } },
	{ AR5K_PHY_NF,
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },
	{ AR5K_PHY_ADCSAT,
		{ 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190 } },
	{ 0x986c,
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb080, 0x050cb080 } },
	{ AR5K_PHY_RX_DELAY,
		{ 0x00002710, 0x00002710, 0x0000157c, 0x00002af8, 0x00002710 } },
	{ 0x9918,
		{ 0x000001b8, 0x000001b8, 0x00000084, 0x00000108, 0x000001b8 } },
	{ 0x9924,
		{ 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05 } },
	{ AR5K_PHY_FRAME_CTL_5211,
		{ 0xffb81020, 0xffb81020, 0xffb80d20, 0xffb81020, 0xffb81020 } },
	{ AR5K_PHY_PCDAC_TXPOWER(0),
		{ 0x10ff14ff, 0x10ff14ff, 0x10ff10ff, 0x10ff19ff, 0x10ff19ff } },
	{ 0xa230,
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000108, 0x00000000 } },
	{ 0xa208,
		{ 0xd03e6788, 0xd03e6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 } },
};

/* Initial mode-specific settings for AR5212 + RF5112 */
static const struct ath5k_ini_mode ar5212_rf5112_ini_mode[] = {
	{ AR5K_USEC_5211,
	/*	  a/XR	      aTurbo	  b	      g		  gTurbo */
		{ 0x128d93a7, 0x098813cf, 0x04e01395, 0x128d93ab, 0x098813cf } },
	{ AR5K_PHY_TURBO,
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000003 } },
	{ 0x9820,
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 } },
	{ 0x9824,
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },
	{ 0x9828,
		{ 0x0a020001, 0x0a020001, 0x05020100, 0x0a020001, 0x0a020001 } },
	{ 0x9834,
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e } },
	{ 0x9838,
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b } },
	{ 0x9844,
		{ 0x1372161c, 0x13721c25, 0x13721728, 0x137216a2, 0x13721c25 } },
	{ 0x9848,
		{ 0x0018da6d, 0x0018da6d, 0x0018ca75, 0x0018ca75, 0x0018ca75 } },
	{ 0x9850,
		{ 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0 } },
	{ AR5K_PHY_SIG,
		{ 0x7e800d2e, 0x7e800d2e, 0x7ee84d2e, 0x7ee84d2e, 0x7e800d2e } },
	{ AR5K_PHY_AGCCOARSE,
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e } },
	{ AR5K_PHY_AGCCTL,
		{ 0x00009d10, 0x00009d10, 0x00009d18, 0x00009d10, 0x00009d10 } },
	{ AR5K_PHY_NF,
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 } },
	{ AR5K_PHY_ADCSAT,
		{ 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190 } },
	{ 0x986c,
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081 } },
	{ AR5K_PHY_RX_DELAY,
		{ 0x000007d0, 0x000007d0, 0x0000044c, 0x00000898, 0x000007d0 } },
	{ 0x9918,
		{ 0x000001b8, 0x000001b8, 0x00000084, 0x00000108, 0x000001b8 } },
	{ 0x9924,
		{ 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05 } },
	{ AR5K_PHY_FRAME_CTL_5211,
		{ 0xffb81020, 0xffb81020, 0xffb80d10, 0xffb81010, 0xffb81010 } },
	{ AR5K_PHY_PCDAC_TXPOWER(0),
		{ 0x10ff14ff, 0x10ff14ff, 0x10ff10ff, 0x10ff19ff, 0x10ff19ff } },
	{ 0xa230,
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000108, 0x00000000 } },
	{ AR5K_PHY_CCKTXCTL,
		{ 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 } },
	{ 0xa208,
		{ 0xd6be6788, 0xd6be6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 } },
	{ AR5K_PHY_GAIN_2GHZ,
		{ 0x642c0140, 0x642c0140, 0x6442c160, 0x6442c160, 0x6442c160 } },
};
#endif

/*
 * Initial BaseBand Gain settings for RF5111/5112 (only AR5210 comes with
 * RF5110 so initial BB Gain settings are included in AR5K_AR5210_INI)
 */

#if defined(CONFIG_ATHEROS_AR5K_AR5211) || defined (CONFIG_ATHEROS_AR5K_AR5212)
/* RF5111 Initial BaseBand Gain settings */
static const struct ath5k_ini rf5111_ini_bbgain[] = {
	{ AR5K_BB_GAIN(0), 0x00000000 },
	{ AR5K_BB_GAIN(1), 0x00000020 },
	{ AR5K_BB_GAIN(2), 0x00000010 },
	{ AR5K_BB_GAIN(3), 0x00000030 },
	{ AR5K_BB_GAIN(4), 0x00000008 },
	{ AR5K_BB_GAIN(5), 0x00000028 },
	{ AR5K_BB_GAIN(6), 0x00000004 },
	{ AR5K_BB_GAIN(7), 0x00000024 },
	{ AR5K_BB_GAIN(8), 0x00000014 },
	{ AR5K_BB_GAIN(9), 0x00000034 },
	{ AR5K_BB_GAIN(10), 0x0000000c },
	{ AR5K_BB_GAIN(11), 0x0000002c },
	{ AR5K_BB_GAIN(12), 0x00000002 },
	{ AR5K_BB_GAIN(13), 0x00000022 },
	{ AR5K_BB_GAIN(14), 0x00000012 },
	{ AR5K_BB_GAIN(15), 0x00000032 },
	{ AR5K_BB_GAIN(16), 0x0000000a },
	{ AR5K_BB_GAIN(17), 0x0000002a },
	{ AR5K_BB_GAIN(18), 0x00000006 },
	{ AR5K_BB_GAIN(19), 0x00000026 },
	{ AR5K_BB_GAIN(20), 0x00000016 },
	{ AR5K_BB_GAIN(21), 0x00000036 },
	{ AR5K_BB_GAIN(22), 0x0000000e },
	{ AR5K_BB_GAIN(23), 0x0000002e },
	{ AR5K_BB_GAIN(24), 0x00000001 },
	{ AR5K_BB_GAIN(25), 0x00000021 },
	{ AR5K_BB_GAIN(26), 0x00000011 },
	{ AR5K_BB_GAIN(27), 0x00000031 },
	{ AR5K_BB_GAIN(28), 0x00000009 },
	{ AR5K_BB_GAIN(29), 0x00000029 },
	{ AR5K_BB_GAIN(30), 0x00000005 },
	{ AR5K_BB_GAIN(31), 0x00000025 },
	{ AR5K_BB_GAIN(32), 0x00000015 },
	{ AR5K_BB_GAIN(33), 0x00000035 },
	{ AR5K_BB_GAIN(34), 0x0000000d },
	{ AR5K_BB_GAIN(35), 0x0000002d },
	{ AR5K_BB_GAIN(36), 0x00000003 },
	{ AR5K_BB_GAIN(37), 0x00000023 },
	{ AR5K_BB_GAIN(38), 0x00000013 },
	{ AR5K_BB_GAIN(39), 0x00000033 },
	{ AR5K_BB_GAIN(40), 0x0000000b },
	{ AR5K_BB_GAIN(41), 0x0000002b },
	{ AR5K_BB_GAIN(42), 0x0000002b },
	{ AR5K_BB_GAIN(43), 0x0000002b },
	{ AR5K_BB_GAIN(44), 0x0000002b },
	{ AR5K_BB_GAIN(45), 0x0000002b },
	{ AR5K_BB_GAIN(46), 0x0000002b },
	{ AR5K_BB_GAIN(47), 0x0000002b },
	{ AR5K_BB_GAIN(48), 0x0000002b },
	{ AR5K_BB_GAIN(49), 0x0000002b },
	{ AR5K_BB_GAIN(50), 0x0000002b },
	{ AR5K_BB_GAIN(51), 0x0000002b },
	{ AR5K_BB_GAIN(52), 0x0000002b },
	{ AR5K_BB_GAIN(53), 0x0000002b },
	{ AR5K_BB_GAIN(54), 0x0000002b },
	{ AR5K_BB_GAIN(55), 0x0000002b },
	{ AR5K_BB_GAIN(56), 0x0000002b },
	{ AR5K_BB_GAIN(57), 0x0000002b },
	{ AR5K_BB_GAIN(58), 0x0000002b },
	{ AR5K_BB_GAIN(59), 0x0000002b },
	{ AR5K_BB_GAIN(60), 0x0000002b },
	{ AR5K_BB_GAIN(61), 0x0000002b },
	{ AR5K_BB_GAIN(62), 0x00000002 },
	{ AR5K_BB_GAIN(63), 0x00000016 },
};
#endif

#ifdef CONFIG_ATHEROS_AR5K_AR5212
/* RF 5112 Initial BaseBand Gain settings */
static const struct ath5k_ini rf5112_ini_bbgain[] = {
	{ AR5K_BB_GAIN(0), 0x00000000 },
	{ AR5K_BB_GAIN(1), 0x00000001 },
	{ AR5K_BB_GAIN(2), 0x00000002 },
	{ AR5K_BB_GAIN(3), 0x00000003 },
	{ AR5K_BB_GAIN(4), 0x00000004 },
	{ AR5K_BB_GAIN(5), 0x00000005 },
	{ AR5K_BB_GAIN(6), 0x00000008 },
	{ AR5K_BB_GAIN(7), 0x00000009 },
	{ AR5K_BB_GAIN(8), 0x0000000a },
	{ AR5K_BB_GAIN(9), 0x0000000b },
	{ AR5K_BB_GAIN(10), 0x0000000c },
	{ AR5K_BB_GAIN(11), 0x0000000d },
	{ AR5K_BB_GAIN(12), 0x00000010 },
	{ AR5K_BB_GAIN(13), 0x00000011 },
	{ AR5K_BB_GAIN(14), 0x00000012 },
	{ AR5K_BB_GAIN(15), 0x00000013 },
	{ AR5K_BB_GAIN(16), 0x00000014 },
	{ AR5K_BB_GAIN(17), 0x00000015 },
	{ AR5K_BB_GAIN(18), 0x00000018 },
	{ AR5K_BB_GAIN(19), 0x00000019 },
	{ AR5K_BB_GAIN(20), 0x0000001a },
	{ AR5K_BB_GAIN(21), 0x0000001b },
	{ AR5K_BB_GAIN(22), 0x0000001c },
	{ AR5K_BB_GAIN(23), 0x0000001d },
	{ AR5K_BB_GAIN(24), 0x00000020 },
	{ AR5K_BB_GAIN(25), 0x00000021 },
	{ AR5K_BB_GAIN(26), 0x00000022 },
	{ AR5K_BB_GAIN(27), 0x00000023 },
	{ AR5K_BB_GAIN(28), 0x00000024 },
	{ AR5K_BB_GAIN(29), 0x00000025 },
	{ AR5K_BB_GAIN(30), 0x00000028 },
	{ AR5K_BB_GAIN(31), 0x00000029 },
	{ AR5K_BB_GAIN(32), 0x0000002a },
	{ AR5K_BB_GAIN(33), 0x0000002b },
	{ AR5K_BB_GAIN(34), 0x0000002c },
	{ AR5K_BB_GAIN(35), 0x0000002d },
	{ AR5K_BB_GAIN(36), 0x00000030 },
	{ AR5K_BB_GAIN(37), 0x00000031 },
	{ AR5K_BB_GAIN(38), 0x00000032 },
	{ AR5K_BB_GAIN(39), 0x00000033 },
	{ AR5K_BB_GAIN(40), 0x00000034 },
	{ AR5K_BB_GAIN(41), 0x00000035 },
	{ AR5K_BB_GAIN(42), 0x00000035 },
	{ AR5K_BB_GAIN(43), 0x00000035 },
	{ AR5K_BB_GAIN(44), 0x00000035 },
	{ AR5K_BB_GAIN(45), 0x00000035 },
	{ AR5K_BB_GAIN(46), 0x00000035 },
	{ AR5K_BB_GAIN(47), 0x00000035 },
	{ AR5K_BB_GAIN(48), 0x00000035 },
	{ AR5K_BB_GAIN(49), 0x00000035 },
	{ AR5K_BB_GAIN(50), 0x00000035 },
	{ AR5K_BB_GAIN(51), 0x00000035 },
	{ AR5K_BB_GAIN(52), 0x00000035 },
	{ AR5K_BB_GAIN(53), 0x00000035 },
	{ AR5K_BB_GAIN(54), 0x00000035 },
	{ AR5K_BB_GAIN(55), 0x00000035 },
	{ AR5K_BB_GAIN(56), 0x00000035 },
	{ AR5K_BB_GAIN(57), 0x00000035 },
	{ AR5K_BB_GAIN(58), 0x00000035 },
	{ AR5K_BB_GAIN(59), 0x00000035 },
	{ AR5K_BB_GAIN(60), 0x00000035 },
	{ AR5K_BB_GAIN(61), 0x00000035 },
	{ AR5K_BB_GAIN(62), 0x00000010 },
	{ AR5K_BB_GAIN(63), 0x0000001a },
};
#endif

#if defined(CONFIG_ATHEROS_AR5K_AR5210) || defined(CONFIG_ATHEROS_AR5K_AR5211) \
	|| defined(CONFIG_ATHEROS_AR5K_AR5212)
/*
 * Write initial register dump
 */
static void ath5k_hw_ini_registers(struct ath_hw *hal, unsigned int size,
		const struct ath5k_ini *ini_regs, bool change_channel)
{
	unsigned int i;

	/* Write initial registers */
	for (i = 0; i < size; i++) {
		/* On channel change there is
		 * no need to mess with PCU */
		if (change_channel &&
				ini_regs[i].ini_register >= AR5K_PCU_MIN &&
				ini_regs[i].ini_register <= AR5K_PCU_MAX)
			continue;

		switch (ini_regs[i].ini_mode) {
		case AR5K_INI_READ:
			/* Cleared on read */
			ath5k_hw_reg_read(hal, ini_regs[i].ini_register);
			break;
		case AR5K_INI_WRITE:
		default:
			AR5K_REG_WAIT(i);
			ath5k_hw_reg_write(hal, ini_regs[i].ini_value,
					ini_regs[i].ini_register);
		}
	}
}
#endif

#if defined(CONFIG_ATHEROS_AR5K_AR5211) || defined(CONFIG_ATHEROS_AR5K_AR5212)
/*
 * Write initial mode-specific register dump
 */
static void ath5k_hw_ini_mode_registers(struct ath_hw *hal,
		unsigned int size, const struct ath5k_ini_mode *ini_mode,
		u8 mode)
{
	unsigned int i;

	for (i = 0; i < size; i++) {
		AR5K_REG_WAIT(i);
		ath5k_hw_reg_write(hal, ini_mode[i].mode_value[mode],
			(u32)ini_mode[i].mode_register);
	}

}
#endif

int ath5k_hw_write_initvals(struct ath_hw *hal, u8 mode, bool change_channel)
{
	/*
	 * Write initial mode-specific settings
	 */
	/*For 5212*/
#ifdef CONFIG_ATHEROS_AR5K_AR5212
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_ini_mode_registers(hal, ARRAY_SIZE(ar5212_ini_mode),
				ar5212_ini_mode, mode);
		if (hal->ah_radio == AR5K_RF5111)
			ath5k_hw_ini_mode_registers(hal,
					ARRAY_SIZE(ar5212_rf5111_ini_mode),
					ar5212_rf5111_ini_mode, mode);
		else if (hal->ah_radio == AR5K_RF5112)
			ath5k_hw_ini_mode_registers(hal,
					ARRAY_SIZE(ar5212_rf5112_ini_mode),
					ar5212_rf5112_ini_mode, mode);
	}
#endif
#ifdef CONFIG_ATHEROS_AR5K_AR5211
	/*For 5211*/
	if (hal->ah_version == AR5K_AR5211)
		ath5k_hw_ini_mode_registers(hal, ARRAY_SIZE(ar5211_ini_mode),
				ar5211_ini_mode, mode);
#endif
	/* For 5210 mode settings check out ath5k_hw_reset_tx_queue */

	/*
	 * Write initial settings common for all modes
	 */
#ifdef CONFIG_ATHEROS_AR5K_AR5212
	if (hal->ah_version == AR5K_AR5212) {
		ath5k_hw_ini_registers(hal, ARRAY_SIZE(ar5212_ini),
				ar5212_ini, change_channel);
		if (hal->ah_radio == AR5K_RF5112) {
			ath5k_hw_reg_write(hal, AR5K_PHY_PAPD_PROBE_INI_5112,
					AR5K_PHY_PAPD_PROBE);
			ath5k_hw_ini_registers(hal,
					ARRAY_SIZE(rf5112_ini_bbgain),
					rf5112_ini_bbgain, change_channel);
		} else if (hal->ah_radio == AR5K_RF5111) {
			ath5k_hw_reg_write(hal, AR5K_PHY_GAIN_2GHZ_INI_5111,
					AR5K_PHY_GAIN_2GHZ);
			ath5k_hw_reg_write(hal, AR5K_PHY_PAPD_PROBE_INI_5111,
					AR5K_PHY_PAPD_PROBE);
			ath5k_hw_ini_registers(hal,
					ARRAY_SIZE(rf5111_ini_bbgain),
					rf5111_ini_bbgain, change_channel);
		}
	}
#endif
#ifdef CONFIG_ATHEROS_AR5K_AR5211
	if (hal->ah_version == AR5K_AR5211) {
		ath5k_hw_ini_registers(hal, ARRAY_SIZE(ar5211_ini),
				ar5211_ini, change_channel);
		/* AR5211 only comes with 5111 */
		ath5k_hw_ini_registers(hal, ARRAY_SIZE(rf5111_ini_bbgain),
				rf5111_ini_bbgain, change_channel);
	}
#endif
#ifdef CONFIG_ATHEROS_AR5K_AR5210
	if (hal->ah_version == AR5K_AR5210) {
		ath5k_hw_ini_registers(hal, ARRAY_SIZE(ar5210_ini),
				ar5210_ini, change_channel);
	}
#endif

	return 0;
}
