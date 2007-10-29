/*
 * PHY functions
 *
 * Copyright (c) 2004, 2005, 2006, 2007 Reyk Floeter <reyk@openbsd.org>
 * Copyright (c) 2006, 2007 Nick Kossifidis <mickflemm@gmail.com>
 * Copyright (c) 2007 Jiri Slaby <jirislaby@gmail.com>
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
 */

#include <linux/delay.h>

#include "ath5k.h"
#include "reg.h"

/* Struct to hold initial RF register values (RF Banks) */
struct ath5k_ini_rf {
	u8	rf_bank;	/* check out ath5k_reg.h */
	u16	rf_register;	/* register address */
	u32	rf_value[5];	/* register value for different modes (above) */
};

/*
 * Mode-specific RF Gain table (64bytes) for RF5111/5112
 * (RF5110 only comes with AR5210 and only supports a/turbo a mode so initial
 * RF Gain values are included in AR5K_AR5210_INI)
 */
struct ath5k_ini_rfgain {
	u16	rfg_register;	/* RF Gain register address */
	u32	rfg_value[2];	/* [freq (see below)] */
};

struct ath5k_gain_opt {
	u32			go_default;
	u32			go_steps_count;
	const struct ath5k_gain_opt_step	go_step[AR5K_GAIN_STEP_COUNT];
};

/* RF5111 mode-specific init registers */
static const struct ath5k_ini_rf rfregs_5111[] = {
	{ 0, 0x989c,
	/*    mode a/XR   mode aTurbo mode b      mode g      mode gTurbo */
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00380000, 0x00380000, 0x00380000, 0x00380000, 0x00380000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 0, 0x989c,
	    { 0x00000000, 0x00000000, 0x000000c0, 0x00000080, 0x00000080 } },
	{ 0, 0x989c,
	    { 0x000400f9, 0x000400f9, 0x000400ff, 0x000400fd, 0x000400fd } },
	{ 0, 0x98d4,
	    { 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 } },
	{ 1, 0x98d4,
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },
	{ 2, 0x98d4,
	    { 0x00000010, 0x00000014, 0x00000010, 0x00000010, 0x00000014 } },
	{ 3, 0x98d8,
	    { 0x00601068, 0x00601068, 0x00601068, 0x00601068, 0x00601068 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x10000000, 0x10000000, 0x10000000, 0x10000000, 0x10000000 } },
	{ 6, 0x989c,
	    { 0x04000000, 0x04000000, 0x04000000, 0x04000000, 0x04000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x0a000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x003800c0, 0x00380080, 0x023800c0, 0x003800c0, 0x003800c0 } },
	{ 6, 0x989c,
	    { 0x00020006, 0x00020006, 0x00000006, 0x00020006, 0x00020006 } },
	{ 6, 0x989c,
	    { 0x00000089, 0x00000089, 0x00000089, 0x00000089, 0x00000089 } },
	{ 6, 0x989c,
	    { 0x000000a0, 0x000000a0, 0x000000a0, 0x000000a0, 0x000000a0 } },
	{ 6, 0x989c,
	    { 0x00040007, 0x00040007, 0x00040007, 0x00040007, 0x00040007 } },
	{ 6, 0x98d4,
	    { 0x0000001a, 0x0000001a, 0x0000001a, 0x0000001a, 0x0000001a } },
	{ 7, 0x989c,
	    { 0x00000040, 0x00000048, 0x00000040, 0x00000040, 0x00000040 } },
	{ 7, 0x989c,
	    { 0x00000010, 0x00000010, 0x00000010, 0x00000010, 0x00000010 } },
	{ 7, 0x989c,
	    { 0x00000008, 0x00000008, 0x00000008, 0x00000008, 0x00000008 } },
	{ 7, 0x989c,
	    { 0x0000004f, 0x0000004f, 0x0000004f, 0x0000004f, 0x0000004f } },
	{ 7, 0x989c,
	    { 0x000000f1, 0x000000f1, 0x00000061, 0x000000f1, 0x000000f1 } },
	{ 7, 0x989c,
	    { 0x0000904f, 0x0000904f, 0x0000904c, 0x0000904f, 0x0000904f } },
	{ 7, 0x989c,
	    { 0x0000125a, 0x0000125a, 0x0000129a, 0x0000125a, 0x0000125a } },
	{ 7, 0x98cc,
	    { 0x0000000e, 0x0000000e, 0x0000000f, 0x0000000e, 0x0000000e } },
};

/* Initial RF Gain settings for RF5111 */
static const struct ath5k_ini_rfgain rfgain_5111[] = {
	/*			      5Ghz	2Ghz	*/
	{ AR5K_RF_GAIN(0),	{ 0x000001a9, 0x00000000 } },
	{ AR5K_RF_GAIN(1),	{ 0x000001e9, 0x00000040 } },
	{ AR5K_RF_GAIN(2),	{ 0x00000029, 0x00000080 } },
	{ AR5K_RF_GAIN(3),	{ 0x00000069, 0x00000150 } },
	{ AR5K_RF_GAIN(4),	{ 0x00000199, 0x00000190 } },
	{ AR5K_RF_GAIN(5),	{ 0x000001d9, 0x000001d0 } },
	{ AR5K_RF_GAIN(6),	{ 0x00000019, 0x00000010 } },
	{ AR5K_RF_GAIN(7),	{ 0x00000059, 0x00000044 } },
	{ AR5K_RF_GAIN(8),	{ 0x00000099, 0x00000084 } },
	{ AR5K_RF_GAIN(9),	{ 0x000001a5, 0x00000148 } },
	{ AR5K_RF_GAIN(10),	{ 0x000001e5, 0x00000188 } },
	{ AR5K_RF_GAIN(11),	{ 0x00000025, 0x000001c8 } },
	{ AR5K_RF_GAIN(12),	{ 0x000001c8, 0x00000014 } },
	{ AR5K_RF_GAIN(13),	{ 0x00000008, 0x00000042 } },
	{ AR5K_RF_GAIN(14),	{ 0x00000048, 0x00000082 } },
	{ AR5K_RF_GAIN(15),	{ 0x00000088, 0x00000178 } },
	{ AR5K_RF_GAIN(16),	{ 0x00000198, 0x000001b8 } },
	{ AR5K_RF_GAIN(17),	{ 0x000001d8, 0x000001f8 } },
	{ AR5K_RF_GAIN(18),	{ 0x00000018, 0x00000012 } },
	{ AR5K_RF_GAIN(19),	{ 0x00000058, 0x00000052 } },
	{ AR5K_RF_GAIN(20),	{ 0x00000098, 0x00000092 } },
	{ AR5K_RF_GAIN(21),	{ 0x000001a4, 0x0000017c } },
	{ AR5K_RF_GAIN(22),	{ 0x000001e4, 0x000001bc } },
	{ AR5K_RF_GAIN(23),	{ 0x00000024, 0x000001fc } },
	{ AR5K_RF_GAIN(24),	{ 0x00000064, 0x0000000a } },
	{ AR5K_RF_GAIN(25),	{ 0x000000a4, 0x0000004a } },
	{ AR5K_RF_GAIN(26),	{ 0x000000e4, 0x0000008a } },
	{ AR5K_RF_GAIN(27),	{ 0x0000010a, 0x0000015a } },
	{ AR5K_RF_GAIN(28),	{ 0x0000014a, 0x0000019a } },
	{ AR5K_RF_GAIN(29),	{ 0x0000018a, 0x000001da } },
	{ AR5K_RF_GAIN(30),	{ 0x000001ca, 0x0000000e } },
	{ AR5K_RF_GAIN(31),	{ 0x0000000a, 0x0000004e } },
	{ AR5K_RF_GAIN(32),	{ 0x0000004a, 0x0000008e } },
	{ AR5K_RF_GAIN(33),	{ 0x0000008a, 0x0000015e } },
	{ AR5K_RF_GAIN(34),	{ 0x000001ba, 0x0000019e } },
	{ AR5K_RF_GAIN(35),	{ 0x000001fa, 0x000001de } },
	{ AR5K_RF_GAIN(36),	{ 0x0000003a, 0x00000009 } },
	{ AR5K_RF_GAIN(37),	{ 0x0000007a, 0x00000049 } },
	{ AR5K_RF_GAIN(38),	{ 0x00000186, 0x00000089 } },
	{ AR5K_RF_GAIN(39),	{ 0x000001c6, 0x00000179 } },
	{ AR5K_RF_GAIN(40),	{ 0x00000006, 0x000001b9 } },
	{ AR5K_RF_GAIN(41),	{ 0x00000046, 0x000001f9 } },
	{ AR5K_RF_GAIN(42),	{ 0x00000086, 0x00000039 } },
	{ AR5K_RF_GAIN(43),	{ 0x000000c6, 0x00000079 } },
	{ AR5K_RF_GAIN(44),	{ 0x000000c6, 0x000000b9 } },
	{ AR5K_RF_GAIN(45),	{ 0x000000c6, 0x000001bd } },
	{ AR5K_RF_GAIN(46),	{ 0x000000c6, 0x000001fd } },
	{ AR5K_RF_GAIN(47),	{ 0x000000c6, 0x0000003d } },
	{ AR5K_RF_GAIN(48),	{ 0x000000c6, 0x0000007d } },
	{ AR5K_RF_GAIN(49),	{ 0x000000c6, 0x000000bd } },
	{ AR5K_RF_GAIN(50),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(51),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(52),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(53),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(54),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(55),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(56),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(57),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(58),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(59),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(60),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(61),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(62),	{ 0x000000c6, 0x000000fd } },
	{ AR5K_RF_GAIN(63),	{ 0x000000c6, 0x000000fd } },
};

static const struct ath5k_gain_opt rfgain_opt_5111 = {
	4,
	9,
	{
		{ { 4, 1, 1, 1 }, 6 },
		{ { 4, 0, 1, 1 }, 4 },
		{ { 3, 1, 1, 1 }, 3 },
		{ { 4, 0, 0, 1 }, 1 },
		{ { 4, 1, 1, 0 }, 0 },
		{ { 4, 0, 1, 0 }, -2 },
		{ { 3, 1, 1, 0 }, -3 },
		{ { 4, 0, 0, 0 }, -4 },
		{ { 2, 1, 1, 0 }, -6 }
	}
};

/* RF5112 mode-specific init registers */
static const struct ath5k_ini_rf rfregs_5112[] = {
	{ 1, 0x98d4,
	/*    mode a/XR   mode aTurbo mode b      mode g      mode gTurbo */
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },
	{ 2, 0x98d0,
	    { 0x03060408, 0x03070408, 0x03060408, 0x03060408, 0x03070408 } },
	{ 3, 0x98dc,
	    { 0x00a0c0c0, 0x00a0c0c0, 0x00e0c0c0, 0x00e0c0c0, 0x00e0c0c0 } },
	{ 6, 0x989c,
	    { 0x00a00000, 0x00a00000, 0x00a00000, 0x00a00000, 0x00a00000 } },
	{ 6, 0x989c,
	    { 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00660000, 0x00660000, 0x00660000, 0x00660000, 0x00660000 } },
	{ 6, 0x989c,
	    { 0x00db0000, 0x00db0000, 0x00db0000, 0x00db0000, 0x00db0000 } },
	{ 6, 0x989c,
	    { 0x00f10000, 0x00f10000, 0x00f10000, 0x00f10000, 0x00f10000 } },
	{ 6, 0x989c,
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },
	{ 6, 0x989c,
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },
	{ 6, 0x989c,
	    { 0x00730000, 0x00730000, 0x00730000, 0x00730000, 0x00730000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },
	{ 6, 0x989c,
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },
	{ 6, 0x989c,
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },
	{ 6, 0x989c,
	    { 0x008b0000, 0x008b0000, 0x008b0000, 0x008b0000, 0x008b0000 } },
	{ 6, 0x989c,
	    { 0x00600000, 0x00600000, 0x00600000, 0x00600000, 0x00600000 } },
	{ 6, 0x989c,
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },
	{ 6, 0x989c,
	    { 0x00840000, 0x00840000, 0x00840000, 0x00840000, 0x00840000 } },
	{ 6, 0x989c,
	    { 0x00640000, 0x00640000, 0x00640000, 0x00640000, 0x00640000 } },
	{ 6, 0x989c,
	    { 0x00200000, 0x00200000, 0x00200000, 0x00200000, 0x00200000 } },
	{ 6, 0x989c,
	    { 0x00240000, 0x00240000, 0x00240000, 0x00240000, 0x00240000 } },
	{ 6, 0x989c,
	    { 0x00250000, 0x00250000, 0x00250000, 0x00250000, 0x00250000 } },
	{ 6, 0x989c,
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },
	{ 6, 0x989c,
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },
	{ 6, 0x989c,
	    { 0x00510000, 0x00510000, 0x00510000, 0x00510000, 0x00510000 } },
	{ 6, 0x989c,
	    { 0x1c040000, 0x1c040000, 0x1c040000, 0x1c040000, 0x1c040000 } },
	{ 6, 0x989c,
	    { 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000, 0x000a0000 } },
	{ 6, 0x989c,
	    { 0x00a10000, 0x00a10000, 0x00a10000, 0x00a10000, 0x00a10000 } },
	{ 6, 0x989c,
	    { 0x00400000, 0x00400000, 0x00400000, 0x00400000, 0x00400000 } },
	{ 6, 0x989c,
	    { 0x03090000, 0x03090000, 0x03090000, 0x03090000, 0x03090000 } },
	{ 6, 0x989c,
	    { 0x06000000, 0x06000000, 0x06000000, 0x06000000, 0x06000000 } },
	{ 6, 0x989c,
	    { 0x000000b0, 0x000000b0, 0x000000a8, 0x000000a8, 0x000000a8 } },
	{ 6, 0x989c,
	    { 0x0000002e, 0x0000002e, 0x0000002e, 0x0000002e, 0x0000002e } },
	{ 6, 0x989c,
	    { 0x006c4a41, 0x006c4a41, 0x006c4af1, 0x006c4a61, 0x006c4a61 } },
	{ 6, 0x989c,
	    { 0x0050892a, 0x0050892a, 0x0050892b, 0x0050892b, 0x0050892b } },
	{ 6, 0x989c,
	    { 0x00842400, 0x00842400, 0x00842400, 0x00842400, 0x00842400 } },
	{ 6, 0x989c,
	    { 0x00c69200, 0x00c69200, 0x00c69200, 0x00c69200, 0x00c69200 } },
	{ 6, 0x98d0,
	    { 0x0002000c, 0x0002000c, 0x0002000c, 0x0002000c, 0x0002000c } },
	{ 7, 0x989c,
	    { 0x00000094, 0x00000094, 0x00000094, 0x00000094, 0x00000094 } },
	{ 7, 0x989c,
	    { 0x00000091, 0x00000091, 0x00000091, 0x00000091, 0x00000091 } },
	{ 7, 0x989c,
	    { 0x0000000a, 0x0000000a, 0x00000012, 0x00000012, 0x00000012 } },
	{ 7, 0x989c,
	    { 0x00000080, 0x00000080, 0x00000080, 0x00000080, 0x00000080 } },
	{ 7, 0x989c,
	    { 0x000000c1, 0x000000c1, 0x000000c1, 0x000000c1, 0x000000c1 } },
	{ 7, 0x989c,
	    { 0x00000060, 0x00000060, 0x00000060, 0x00000060, 0x00000060 } },
	{ 7, 0x989c,
	    { 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0 } },
	{ 7, 0x989c,
	    { 0x00000022, 0x00000022, 0x00000022, 0x00000022, 0x00000022 } },
	{ 7, 0x989c,
	    { 0x00000092, 0x00000092, 0x00000092, 0x00000092, 0x00000092 } },
	{ 7, 0x989c,
	    { 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4 } },
	{ 7, 0x989c,
	    { 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc } },
	{ 7, 0x989c,
	    { 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c } },
	{ 7, 0x98c4,
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },
};

/* RF5112A mode-specific init registers */
static const struct ath5k_ini_rf rfregs_5112a[] = {
	{ 1, 0x98d4,
	/*    mode a/XR   mode aTurbo mode b      mode g      mode gTurbo */
	    { 0x00000020, 0x00000020, 0x00000020, 0x00000020, 0x00000020 } },
	{ 2, 0x98d0,
	    { 0x03060408, 0x03070408, 0x03060408, 0x03060408, 0x03070408 } },
	{ 3, 0x98dc,
	    { 0x00a0c0c0, 0x00a0c0c0, 0x00e0c0c0, 0x00e0c0c0, 0x00e0c0c0 } },
	{ 6, 0x989c,
	    { 0x0f000000, 0x0f000000, 0x0f000000, 0x0f000000, 0x0f000000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000 } },
	{ 6, 0x989c,
	    { 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000 } },
	{ 6, 0x989c,
	    { 0x00010000, 0x00010000, 0x00010000, 0x00010000, 0x00010000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00180000, 0x00180000, 0x00180000, 0x00180000, 0x00180000 } },
	{ 6, 0x989c,
	    { 0x00600000, 0x00600000, 0x006e0000, 0x006e0000, 0x006e0000 } },
	{ 6, 0x989c,
	    { 0x00c70000, 0x00c70000, 0x00c70000, 0x00c70000, 0x00c70000 } },
	{ 6, 0x989c,
	    { 0x004b0000, 0x004b0000, 0x004b0000, 0x004b0000, 0x004b0000 } },
	{ 6, 0x989c,
	    { 0x04480000, 0x04480000, 0x04480000, 0x04480000, 0x04480000 } },
	{ 6, 0x989c,
	    { 0x00220000, 0x00220000, 0x00220000, 0x00220000, 0x00220000 } },
	{ 6, 0x989c,
	    { 0x00e40000, 0x00e40000, 0x00e40000, 0x00e40000, 0x00e40000 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x00fc0000, 0x00fc0000, 0x00fc0000, 0x00fc0000, 0x00fc0000 } },
	{ 6, 0x989c,
	    { 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000, 0x00ff0000 } },
	{ 6, 0x989c,
	    { 0x043f0000, 0x043f0000, 0x043f0000, 0x043f0000, 0x043f0000 } },
	{ 6, 0x989c,
	    { 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000, 0x000c0000 } },
	{ 6, 0x989c,
	    { 0x00190000, 0x00190000, 0x00190000, 0x00190000, 0x00190000 } },
	{ 6, 0x989c,
	    { 0x00240000, 0x00240000, 0x00240000, 0x00240000, 0x00240000 } },
	{ 6, 0x989c,
	    { 0x00b40000, 0x00b40000, 0x00b40000, 0x00b40000, 0x00b40000 } },
	{ 6, 0x989c,
	    { 0x00990000, 0x00990000, 0x00990000, 0x00990000, 0x00990000 } },
	{ 6, 0x989c,
	    { 0x00500000, 0x00500000, 0x00500000, 0x00500000, 0x00500000 } },
	{ 6, 0x989c,
	    { 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000, 0x002a0000 } },
	{ 6, 0x989c,
	    { 0x00120000, 0x00120000, 0x00120000, 0x00120000, 0x00120000 } },
	{ 6, 0x989c,
	    { 0xc0320000, 0xc0320000, 0xc0320000, 0xc0320000, 0xc0320000 } },
	{ 6, 0x989c,
	    { 0x01740000, 0x01740000, 0x01740000, 0x01740000, 0x01740000 } },
	{ 6, 0x989c,
	    { 0x00110000, 0x00110000, 0x00110000, 0x00110000, 0x00110000 } },
	{ 6, 0x989c,
	    { 0x86280000, 0x86280000, 0x86280000, 0x86280000, 0x86280000 } },
	{ 6, 0x989c,
	    { 0x31840000, 0x31840000, 0x31840000, 0x31840000, 0x31840000 } },
	{ 6, 0x989c,
	    { 0x00020080, 0x00020080, 0x00020080, 0x00020080, 0x00020080 } },
	{ 6, 0x989c,
	    { 0x00080009, 0x00080009, 0x00080009, 0x00080009, 0x00080009 } },
	{ 6, 0x989c,
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },
	{ 6, 0x989c,
	    { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
	{ 6, 0x989c,
	    { 0x000000b2, 0x000000b2, 0x000000b2, 0x000000b2, 0x000000b2 } },
	{ 6, 0x989c,
	    { 0x00b02084, 0x00b02084, 0x00b02084, 0x00b02084, 0x00b02084 } },
	{ 6, 0x989c,
	    { 0x004125a4, 0x004125a4, 0x004125a4, 0x004125a4, 0x004125a4 } },
	{ 6, 0x989c,
	    { 0x00119220, 0x00119220, 0x00119220, 0x00119220, 0x00119220 } },
	{ 6, 0x989c,
	    { 0x001a4800, 0x001a4800, 0x001a4800, 0x001a4800, 0x001a4800 } },
	{ 6, 0x98d8,
	    { 0x000b0230, 0x000b0230, 0x000b0230, 0x000b0230, 0x000b0230 } },
	{ 7, 0x989c,
	    { 0x00000094, 0x00000094, 0x00000094, 0x00000094, 0x00000094 } },
	{ 7, 0x989c,
	    { 0x00000091, 0x00000091, 0x00000091, 0x00000091, 0x00000091 } },
	{ 7, 0x989c,
	    { 0x00000012, 0x00000012, 0x00000012, 0x00000012, 0x00000012 } },
	{ 7, 0x989c,
	    { 0x00000080, 0x00000080, 0x00000080, 0x00000080, 0x00000080 } },
	{ 7, 0x989c,
	    { 0x000000d9, 0x000000d9, 0x000000d9, 0x000000d9, 0x000000d9 } },
	{ 7, 0x989c,
	    { 0x00000060, 0x00000060, 0x00000060, 0x00000060, 0x00000060 } },
	{ 7, 0x989c,
	    { 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0, 0x000000f0 } },
	{ 7, 0x989c,
	    { 0x000000a2, 0x000000a2, 0x000000a2, 0x000000a2, 0x000000a2 } },
	{ 7, 0x989c,
	    { 0x00000052, 0x00000052, 0x00000052, 0x00000052, 0x00000052 } },
	{ 7, 0x989c,
	    { 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4, 0x000000d4 } },
	{ 7, 0x989c,
	    { 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc, 0x000014cc } },
	{ 7, 0x989c,
	    { 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c, 0x0000048c } },
	{ 7, 0x98c4,
	    { 0x00000003, 0x00000003, 0x00000003, 0x00000003, 0x00000003 } },
};


/* Initial RF Gain settings for RF5112 */
static const struct ath5k_ini_rfgain rfgain_5112[] = {
	/*			      5Ghz	2Ghz	*/
	{ AR5K_RF_GAIN(0),	{ 0x00000007, 0x00000007 } },
	{ AR5K_RF_GAIN(1),	{ 0x00000047, 0x00000047 } },
	{ AR5K_RF_GAIN(2),	{ 0x00000087, 0x00000087 } },
	{ AR5K_RF_GAIN(3),	{ 0x000001a0, 0x000001a0 } },
	{ AR5K_RF_GAIN(4),	{ 0x000001e0, 0x000001e0 } },
	{ AR5K_RF_GAIN(5),	{ 0x00000020, 0x00000020 } },
	{ AR5K_RF_GAIN(6),	{ 0x00000060, 0x00000060 } },
	{ AR5K_RF_GAIN(7),	{ 0x000001a1, 0x000001a1 } },
	{ AR5K_RF_GAIN(8),	{ 0x000001e1, 0x000001e1 } },
	{ AR5K_RF_GAIN(9),	{ 0x00000021, 0x00000021 } },
	{ AR5K_RF_GAIN(10),	{ 0x00000061, 0x00000061 } },
	{ AR5K_RF_GAIN(11),	{ 0x00000162, 0x00000162 } },
	{ AR5K_RF_GAIN(12),	{ 0x000001a2, 0x000001a2 } },
	{ AR5K_RF_GAIN(13),	{ 0x000001e2, 0x000001e2 } },
	{ AR5K_RF_GAIN(14),	{ 0x00000022, 0x00000022 } },
	{ AR5K_RF_GAIN(15),	{ 0x00000062, 0x00000062 } },
	{ AR5K_RF_GAIN(16),	{ 0x00000163, 0x00000163 } },
	{ AR5K_RF_GAIN(17),	{ 0x000001a3, 0x000001a3 } },
	{ AR5K_RF_GAIN(18),	{ 0x000001e3, 0x000001e3 } },
	{ AR5K_RF_GAIN(19),	{ 0x00000023, 0x00000023 } },
	{ AR5K_RF_GAIN(20),	{ 0x00000063, 0x00000063 } },
	{ AR5K_RF_GAIN(21),	{ 0x00000184, 0x00000184 } },
	{ AR5K_RF_GAIN(22),	{ 0x000001c4, 0x000001c4 } },
	{ AR5K_RF_GAIN(23),	{ 0x00000004, 0x00000004 } },
	{ AR5K_RF_GAIN(24),	{ 0x000001ea, 0x0000000b } },
	{ AR5K_RF_GAIN(25),	{ 0x0000002a, 0x0000004b } },
	{ AR5K_RF_GAIN(26),	{ 0x0000006a, 0x0000008b } },
	{ AR5K_RF_GAIN(27),	{ 0x000000aa, 0x000001ac } },
	{ AR5K_RF_GAIN(28),	{ 0x000001ab, 0x000001ec } },
	{ AR5K_RF_GAIN(29),	{ 0x000001eb, 0x0000002c } },
	{ AR5K_RF_GAIN(30),	{ 0x0000002b, 0x00000012 } },
	{ AR5K_RF_GAIN(31),	{ 0x0000006b, 0x00000052 } },
	{ AR5K_RF_GAIN(32),	{ 0x000000ab, 0x00000092 } },
	{ AR5K_RF_GAIN(33),	{ 0x000001ac, 0x00000193 } },
	{ AR5K_RF_GAIN(34),	{ 0x000001ec, 0x000001d3 } },
	{ AR5K_RF_GAIN(35),	{ 0x0000002c, 0x00000013 } },
	{ AR5K_RF_GAIN(36),	{ 0x0000003a, 0x00000053 } },
	{ AR5K_RF_GAIN(37),	{ 0x0000007a, 0x00000093 } },
	{ AR5K_RF_GAIN(38),	{ 0x000000ba, 0x00000194 } },
	{ AR5K_RF_GAIN(39),	{ 0x000001bb, 0x000001d4 } },
	{ AR5K_RF_GAIN(40),	{ 0x000001fb, 0x00000014 } },
	{ AR5K_RF_GAIN(41),	{ 0x0000003b, 0x0000003a } },
	{ AR5K_RF_GAIN(42),	{ 0x0000007b, 0x0000007a } },
	{ AR5K_RF_GAIN(43),	{ 0x000000bb, 0x000000ba } },
	{ AR5K_RF_GAIN(44),	{ 0x000001bc, 0x000001bb } },
	{ AR5K_RF_GAIN(45),	{ 0x000001fc, 0x000001fb } },
	{ AR5K_RF_GAIN(46),	{ 0x0000003c, 0x0000003b } },
	{ AR5K_RF_GAIN(47),	{ 0x0000007c, 0x0000007b } },
	{ AR5K_RF_GAIN(48),	{ 0x000000bc, 0x000000bb } },
	{ AR5K_RF_GAIN(49),	{ 0x000000fc, 0x000001bc } },
	{ AR5K_RF_GAIN(50),	{ 0x000000fc, 0x000001fc } },
	{ AR5K_RF_GAIN(51),	{ 0x000000fc, 0x0000003c } },
	{ AR5K_RF_GAIN(52),	{ 0x000000fc, 0x0000007c } },
	{ AR5K_RF_GAIN(53),	{ 0x000000fc, 0x000000bc } },
	{ AR5K_RF_GAIN(54),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(55),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(56),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(57),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(58),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(59),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(60),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(61),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(62),	{ 0x000000fc, 0x000000fc } },
	{ AR5K_RF_GAIN(63),	{ 0x000000fc, 0x000000fc } },
};

static const struct ath5k_gain_opt rfgain_opt_5112 = {
	1,
	8,
	{
		{ { 3, 0, 0, 0, 0, 0, 0 }, 6 },
		{ { 2, 0, 0, 0, 0, 0, 0 }, 0 },
		{ { 1, 0, 0, 0, 0, 0, 0 }, -3 },
		{ { 0, 0, 0, 0, 0, 0, 0 }, -6 },
		{ { 0, 1, 1, 0, 0, 0, 0 }, -8 },
		{ { 0, 1, 1, 0, 1, 1, 0 }, -10 },
		{ { 0, 1, 0, 1, 1, 1, 0 }, -13 },
		{ { 0, 1, 0, 1, 1, 0, 1 }, -16 },
	}
};

/*
 * Used to modify RF Banks before writing them to AR5K_RF_BUFFER
 */
static unsigned int ath5k_hw_rfregs_op(u32 *rf, u32 offset, u32 reg, u32 bits,
		u32 first, u32 col, bool set)
{
	u32 mask, entry, last, data, shift, position;
	s32 left;
	int i;

	data = 0;

	if (rf == NULL)
		/* should not happen */
		return 0;

	if (!(col <= 3 && bits <= 32 && first + bits <= 319)) {
		AR5K_PRINTF("invalid values at offset %u\n", offset);
		return 0;
	}

	entry = ((first - 1) / 8) + offset;
	position = (first - 1) % 8;

	if (set == true)
		data = ath5k_hw_bitswap(reg, bits);

	for (i = shift = 0, left = bits; left > 0; position = 0, entry++, i++) {
		last = (position + left > 8) ? 8 : position + left;
		mask = (((1 << last) - 1) ^ ((1 << position) - 1)) << (col * 8);

		if (set == true) {
			rf[entry] &= ~mask;
			rf[entry] |= ((data << position) << (col * 8)) & mask;
			data >>= (8 - position);
		} else {
			data = (((rf[entry] & mask) >> (col * 8)) >> position)
				<< shift;
			shift += last - position;
		}

		left -= 8 - position;
	}

	data = set == true ? 1 : ath5k_hw_bitswap(data, bits);

	return data;
}

static u32 ath5k_hw_rfregs_gainf_corr(struct ath5k_hw *ah)
{
	u32 mix, step;
	u32 *rf;

	if (ah->ah_rf_banks == NULL)
		return 0;

	rf = ah->ah_rf_banks;
	ah->ah_gain.g_f_corr = 0;

	if (ath5k_hw_rfregs_op(rf, ah->ah_offset[7], 0, 1, 36, 0, false) != 1)
		return 0;

	step = ath5k_hw_rfregs_op(rf, ah->ah_offset[7], 0, 4, 32, 0, false);
	mix = ah->ah_gain.g_step->gos_param[0];

	switch (mix) {
	case 3:
		ah->ah_gain.g_f_corr = step * 2;
		break;
	case 2:
		ah->ah_gain.g_f_corr = (step - 5) * 2;
		break;
	case 1:
		ah->ah_gain.g_f_corr = step;
		break;
	default:
		ah->ah_gain.g_f_corr = 0;
		break;
	}

	return ah->ah_gain.g_f_corr;
}

static bool ath5k_hw_rfregs_gain_readback(struct ath5k_hw *ah)
{
	u32 step, mix, level[4];
	u32 *rf;

	if (ah->ah_rf_banks == NULL)
		return false;

	rf = ah->ah_rf_banks;

	if (ah->ah_radio == AR5K_RF5111) {
		step = ath5k_hw_rfregs_op(rf, ah->ah_offset[7], 0, 6, 37, 0,
				false);
		level[0] = 0;
		level[1] = (step == 0x3f) ? 0x32 : step + 4;
		level[2] = (step != 0x3f) ? 0x40 : level[0];
		level[3] = level[2] + 0x32;

		ah->ah_gain.g_high = level[3] -
			(step == 0x3f ? AR5K_GAIN_DYN_ADJUST_HI_MARGIN : -5);
		ah->ah_gain.g_low = level[0] +
			(step == 0x3f ? AR5K_GAIN_DYN_ADJUST_LO_MARGIN : 0);
	} else {
		mix = ath5k_hw_rfregs_op(rf, ah->ah_offset[7], 0, 1, 36, 0,
				false);
		level[0] = level[2] = 0;

		if (mix == 1) {
			level[1] = level[3] = 83;
		} else {
			level[1] = level[3] = 107;
			ah->ah_gain.g_high = 55;
		}
	}

	return (ah->ah_gain.g_current >= level[0] &&
			ah->ah_gain.g_current <= level[1]) ||
		(ah->ah_gain.g_current >= level[2] &&
			ah->ah_gain.g_current <= level[3]);
}

static s32 ath5k_hw_rfregs_gain_adjust(struct ath5k_hw *ah)
{
	const struct ath5k_gain_opt *go;
	int ret = 0;

	switch (ah->ah_radio) {
	case AR5K_RF5111:
		go = &rfgain_opt_5111;
		break;
	case AR5K_RF5112:
		go = &rfgain_opt_5112;
		break;
	default:
		return 0;
	}

	ah->ah_gain.g_step = &go->go_step[ah->ah_gain.g_step_idx];

	if (ah->ah_gain.g_current >= ah->ah_gain.g_high) {
		if (ah->ah_gain.g_step_idx == 0)
			return -1;
		for (ah->ah_gain.g_target = ah->ah_gain.g_current;
				ah->ah_gain.g_target >=  ah->ah_gain.g_high &&
				ah->ah_gain.g_step_idx > 0;
				ah->ah_gain.g_step =
					&go->go_step[ah->ah_gain.g_step_idx])
			ah->ah_gain.g_target -= 2 *
			    (go->go_step[--(ah->ah_gain.g_step_idx)].gos_gain -
			    ah->ah_gain.g_step->gos_gain);

		ret = 1;
		goto done;
	}

	if (ah->ah_gain.g_current <= ah->ah_gain.g_low) {
		if (ah->ah_gain.g_step_idx == (go->go_steps_count - 1))
			return -2;
		for (ah->ah_gain.g_target = ah->ah_gain.g_current;
				ah->ah_gain.g_target <= ah->ah_gain.g_low &&
				ah->ah_gain.g_step_idx < go->go_steps_count-1;
				ah->ah_gain.g_step =
					&go->go_step[ah->ah_gain.g_step_idx])
			ah->ah_gain.g_target -= 2 *
			    (go->go_step[++ah->ah_gain.g_step_idx].gos_gain -
			    ah->ah_gain.g_step->gos_gain);

		ret = 2;
		goto done;
	}

done:
#ifdef AR5K_DEBUG
	AR5K_PRINTF("ret %d, gain step %u, current gain %u, target gain %u\n",
		ret, ah->ah_gain.g_step_idx, ah->ah_gain.g_current,
		ah->ah_gain.g_target);
#endif

	return ret;
}

/*
 * Read EEPROM Calibration data, modify RF Banks and Initialize RF5111
 */
static int ath5k_hw_rf5111_rfregs(struct ath5k_hw *ah,
		struct ieee80211_channel *channel, unsigned int mode)
{
	struct ath5k_eeprom_info *ee = &ah->ah_capabilities.cap_eeprom;
	u32 *rf;
	const unsigned int rf_size = ARRAY_SIZE(rfregs_5111);
	unsigned int i;
	int obdb = -1, bank = -1;
	u32 ee_mode;

	AR5K_ASSERT_ENTRY(mode, AR5K_INI_VAL_MAX);

	rf = ah->ah_rf_banks;

	/* Copy values to modify them */
	for (i = 0; i < rf_size; i++) {
		if (rfregs_5111[i].rf_bank >= AR5K_RF5111_INI_RF_MAX_BANKS) {
			AR5K_PRINT("invalid bank\n");
			return -EINVAL;
		}

		if (bank != rfregs_5111[i].rf_bank) {
			bank = rfregs_5111[i].rf_bank;
			ah->ah_offset[bank] = i;
		}

		rf[i] = rfregs_5111[i].rf_value[mode];
	}

	/* Modify bank 0 */
	if (channel->val & CHANNEL_2GHZ) {
		if (channel->val & CHANNEL_B)
			ee_mode = AR5K_EEPROM_MODE_11B;
		else
			ee_mode = AR5K_EEPROM_MODE_11G;
		obdb = 0;

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[0],
				ee->ee_ob[ee_mode][obdb], 3, 119, 0, true))
			return -EINVAL;

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[0],
				ee->ee_ob[ee_mode][obdb], 3, 122, 0, true))
			return -EINVAL;

		obdb = 1;
	/* Modify bank 6 */
	} else {
		/* For 11a, Turbo and XR */
		ee_mode = AR5K_EEPROM_MODE_11A;
		obdb =	 channel->freq >= 5725 ? 3 :
			(channel->freq >= 5500 ? 2 :
			(channel->freq >= 5260 ? 1 :
			 (channel->freq > 4000 ? 0 : -1)));

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
				ee->ee_pwd_84, 1, 51, 3, true))
			return -EINVAL;

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
				ee->ee_pwd_90, 1, 45, 3, true))
			return -EINVAL;
	}

	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
			!ee->ee_xpd[ee_mode], 1, 95, 0, true))
		return -EINVAL;

	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
			ee->ee_x_gain[ee_mode], 4, 96, 0, true))
		return -EINVAL;

	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6], obdb >= 0 ?
			ee->ee_ob[ee_mode][obdb] : 0, 3, 104, 0, true))
		return -EINVAL;

	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6], obdb >= 0 ?
			ee->ee_db[ee_mode][obdb] : 0, 3, 107, 0, true))
		return -EINVAL;

	/* Modify bank 7 */
	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[7],
			ee->ee_i_gain[ee_mode], 6, 29, 0, true))
		return -EINVAL;

	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[7],
			ee->ee_xpd[ee_mode], 1, 4, 0, true))
		return -EINVAL;

	/* Write RF values */
	for (i = 0; i < rf_size; i++) {
		AR5K_REG_WAIT(i);
		ath5k_hw_reg_write(ah, rf[i], rfregs_5111[i].rf_register);
	}

	return 0;
}

/*
 * Read EEPROM Calibration data, modify RF Banks and Initialize RF5112
 */
static int ath5k_hw_rf5112_rfregs(struct ath5k_hw *ah,
		struct ieee80211_channel *channel, unsigned int mode)
{
	const struct ath5k_ini_rf *rf_ini;
	struct ath5k_eeprom_info *ee = &ah->ah_capabilities.cap_eeprom;
	u32 *rf;
	unsigned int rf_size, i;
	int obdb = -1, bank = -1;
	u32 ee_mode;

	AR5K_ASSERT_ENTRY(mode, AR5K_INI_VAL_MAX);

	rf = ah->ah_rf_banks;

	if (ah->ah_radio_5ghz_revision >= AR5K_SREV_RAD_5112A) {
		rf_ini = rfregs_5112a;
		rf_size = ARRAY_SIZE(rfregs_5112a);
	} else {
		rf_ini = rfregs_5112;
		rf_size = ARRAY_SIZE(rfregs_5112);
	}

	/* Copy values to modify them */
	for (i = 0; i < rf_size; i++) {
		if (rf_ini[i].rf_bank >= AR5K_RF5112_INI_RF_MAX_BANKS) {
			AR5K_PRINT("invalid bank\n");
			return -EINVAL;
		}

		if (bank != rf_ini[i].rf_bank) {
			bank = rf_ini[i].rf_bank;
			ah->ah_offset[bank] = i;
		}

		rf[i] = rf_ini[i].rf_value[mode];
	}

	/* Modify bank 6 */
	if (channel->val & CHANNEL_2GHZ) {
		if (channel->val & CHANNEL_B)
			ee_mode = AR5K_EEPROM_MODE_11B;
		else
			ee_mode = AR5K_EEPROM_MODE_11G;
		obdb = 0;

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
				ee->ee_ob[ee_mode][obdb], 3, 287, 0, true))
			return -EINVAL;

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
				ee->ee_ob[ee_mode][obdb], 3, 290, 0, true))
			return -EINVAL;
	} else {
		/* For 11a, Turbo and XR */
		ee_mode = AR5K_EEPROM_MODE_11A;
		obdb = channel->freq >= 5725 ? 3 :
		    (channel->freq >= 5500 ? 2 :
			(channel->freq >= 5260 ? 1 :
			    (channel->freq > 4000 ? 0 : -1)));

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
				ee->ee_ob[ee_mode][obdb], 3, 279, 0, true))
			return -EINVAL;

		if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
				ee->ee_ob[ee_mode][obdb], 3, 282, 0, true))
			return -EINVAL;
	}

#ifdef notyet
	ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
	    ee->ee_x_gain[ee_mode], 2, 270, 0, true);
	ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
	    ee->ee_x_gain[ee_mode], 2, 257, 0, true);
#endif

	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[6],
			ee->ee_xpd[ee_mode], 1, 302, 0, true))
		return -EINVAL;

	/* Modify bank 7 */
	if (!ath5k_hw_rfregs_op(rf, ah->ah_offset[7],
			ee->ee_i_gain[ee_mode], 6, 14, 0, true))
		return -EINVAL;

	/* Write RF values */
	for (i = 0; i < rf_size; i++)
		ath5k_hw_reg_write(ah, rf[i], rf_ini[i].rf_register);

	return 0;
}

/*
 * Initialize RF
 */
int ath5k_hw_rfregs(struct ath5k_hw *ah, struct ieee80211_channel *channel,
		unsigned int mode)
{
	int (*func)(struct ath5k_hw *, struct ieee80211_channel *, unsigned int);
	int ret;

	switch (ah->ah_radio) {
	case AR5K_RF5111:
		ah->ah_rf_banks_size = sizeof(rfregs_5111);
		func = ath5k_hw_rf5111_rfregs;
		break;
	case AR5K_RF5112:
		if (ah->ah_radio_5ghz_revision >= AR5K_SREV_RAD_5112A)
			ah->ah_rf_banks_size = sizeof(rfregs_5112a);
		else
			ah->ah_rf_banks_size = sizeof(rfregs_5112);
		func = ath5k_hw_rf5112_rfregs;
		break;
	default:
		return -EINVAL;
	}

	if (ah->ah_rf_banks == NULL) {
		/* XXX do extra checks? */
		ah->ah_rf_banks = kmalloc(ah->ah_rf_banks_size, GFP_KERNEL);
		if (ah->ah_rf_banks == NULL) {
			AR5K_PRINT("out of memory\n");
			return -ENOMEM;
		}
	}

	ret = func(ah, channel, mode);
	if (!ret)
		ah->ah_rf_gain = AR5K_RFGAIN_INACTIVE;

	return ret;
}

int ath5k_hw_rfgain(struct ath5k_hw *ah, unsigned int freq)
{
	const struct ath5k_ini_rfgain *ath5k_rfg;
	unsigned int i, size;

	switch (ah->ah_radio) {
	case AR5K_RF5111:
		ath5k_rfg = rfgain_5111;
		size = ARRAY_SIZE(rfgain_5111);
		break;
	case AR5K_RF5112:
		ath5k_rfg = rfgain_5112;
		size = ARRAY_SIZE(rfgain_5112);
		break;
	default:
		return -EINVAL;
	}

	switch (freq) {
	case AR5K_INI_RFGAIN_2GHZ:
	case AR5K_INI_RFGAIN_5GHZ:
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < size; i++) {
		AR5K_REG_WAIT(i);
		ath5k_hw_reg_write(ah, ath5k_rfg[i].rfg_value[freq],
			(u32)ath5k_rfg[i].rfg_register);
	}

	return 0;
}

enum ath5k_rfgain ath5k_hw_get_rf_gain(struct ath5k_hw *ah)
{
	u32 data, type;

	AR5K_TRACE;

	if (ah->ah_rf_banks == NULL || !ah->ah_gain.g_active ||
			ah->ah_version <= AR5K_AR5211)
		return AR5K_RFGAIN_INACTIVE;

	if (ah->ah_rf_gain != AR5K_RFGAIN_READ_REQUESTED)
		goto done;

	data = ath5k_hw_reg_read(ah, AR5K_PHY_PAPD_PROBE);

	if (!(data & AR5K_PHY_PAPD_PROBE_TX_NEXT)) {
		ah->ah_gain.g_current = data >> AR5K_PHY_PAPD_PROBE_GAINF_S;
		type = AR5K_REG_MS(data, AR5K_PHY_PAPD_PROBE_TYPE);

		if (type == AR5K_PHY_PAPD_PROBE_TYPE_CCK)
			ah->ah_gain.g_current += AR5K_GAIN_CCK_PROBE_CORR;

		if (ah->ah_radio == AR5K_RF5112) {
			ath5k_hw_rfregs_gainf_corr(ah);
			ah->ah_gain.g_current =
				ah->ah_gain.g_current>=ah->ah_gain.g_f_corr ?
				(ah->ah_gain.g_current-ah->ah_gain.g_f_corr) :
				0;
		}

		if (ath5k_hw_rfregs_gain_readback(ah) &&
				AR5K_GAIN_CHECK_ADJUST(&ah->ah_gain) &&
				ath5k_hw_rfregs_gain_adjust(ah))
			ah->ah_rf_gain = AR5K_RFGAIN_NEED_CHANGE;
	}

done:
	return ah->ah_rf_gain;
}

int ath5k_hw_set_rfgain_opt(struct ath5k_hw *ah)
{
	/* Initialize the gain optimization values */
	switch (ah->ah_radio) {
	case AR5K_RF5111:
		ah->ah_gain.g_step_idx = rfgain_opt_5111.go_default;
		ah->ah_gain.g_step =
		    &rfgain_opt_5111.go_step[ah->ah_gain.g_step_idx];
		ah->ah_gain.g_low = 20;
		ah->ah_gain.g_high = 35;
		ah->ah_gain.g_active = 1;
		break;
	case AR5K_RF5112:
		ah->ah_gain.g_step_idx = rfgain_opt_5112.go_default;
		ah->ah_gain.g_step =
		    &rfgain_opt_5112.go_step[ah->ah_gain.g_step_idx];
		ah->ah_gain.g_low = 20;
		ah->ah_gain.g_high = 85;
		ah->ah_gain.g_active = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**************************\
  PHY/RF channel functions
\**************************/

/*
 * Check if a channel is supported
 */
bool ath5k_channel_ok(struct ath5k_hw *ah, u16 freq, unsigned int flags)
{
	/* Check if the channel is in our supported range */
	if (flags & CHANNEL_2GHZ) {
		if ((freq >= ah->ah_capabilities.cap_range.range_2ghz_min) &&
		    (freq <= ah->ah_capabilities.cap_range.range_2ghz_max))
			return true;
	} else if (flags & CHANNEL_5GHZ)
		if ((freq >= ah->ah_capabilities.cap_range.range_5ghz_min) &&
		    (freq <= ah->ah_capabilities.cap_range.range_5ghz_max))
			return true;

	return false;
}

/*
 * Convertion needed for RF5110
 */
static u32 ath5k_hw_rf5110_chan2athchan(struct ieee80211_channel *channel)
{
	u32 athchan;

	/*
	 * Convert IEEE channel/MHz to an internal channel value used
	 * by the AR5210 chipset. This has not been verified with
	 * newer chipsets like the AR5212A who have a completely
	 * different RF/PHY part.
	 */
	athchan = (ath5k_hw_bitswap((channel->chan - 24) / 2, 5) << 1) |
		(1 << 6) | 0x1;

	return athchan;
}

/*
 * Set channel on RF5110
 */
static int ath5k_hw_rf5110_channel(struct ath5k_hw *ah,
		struct ieee80211_channel *channel)
{
	u32 data;

	/*
	 * Set the channel and wait
	 */
	data = ath5k_hw_rf5110_chan2athchan(channel);
	ath5k_hw_reg_write(ah, data, AR5K_RF_BUFFER);
	ath5k_hw_reg_write(ah, 0, AR5K_RF_BUFFER_CONTROL_0);
	mdelay(1);

	return 0;
}

/*
 * Convertion needed for 5111
 */
static int ath5k_hw_rf5111_chan2athchan(unsigned int ieee,
		struct ath5k_athchan_2ghz *athchan)
{
	int channel;

	/* Cast this value to catch negative channel numbers (>= -19) */
	channel = (int)ieee;

	/*
	 * Map 2GHz IEEE channel to 5GHz Atheros channel
	 */
	if (channel <= 13) {
		athchan->a2_athchan = 115 + channel;
		athchan->a2_flags = 0x46;
	} else if (channel == 14) {
		athchan->a2_athchan = 124;
		athchan->a2_flags = 0x44;
	} else if (channel >= 15 && channel <= 26) {
		athchan->a2_athchan = ((channel - 14) * 4) + 132;
		athchan->a2_flags = 0x46;
	} else
		return -EINVAL;

	return 0;
}

/*
 * Set channel on 5111
 */
static int ath5k_hw_rf5111_channel(struct ath5k_hw *ah,
		struct ieee80211_channel *channel)
{
	struct ath5k_athchan_2ghz ath5k_channel_2ghz;
	unsigned int ath5k_channel = channel->chan;
	u32 data0, data1, clock;
	int ret;

	/*
	 * Set the channel on the RF5111 radio
	 */
	data0 = data1 = 0;

	if (channel->val & CHANNEL_2GHZ) {
		/* Map 2GHz channel to 5GHz Atheros channel ID */
		ret = ath5k_hw_rf5111_chan2athchan(channel->chan,
				&ath5k_channel_2ghz);
		if (ret)
			return ret;

		ath5k_channel = ath5k_channel_2ghz.a2_athchan;
		data0 = ((ath5k_hw_bitswap(ath5k_channel_2ghz.a2_flags, 8) & 0xff)
		    << 5) | (1 << 4);
	}

	if (ath5k_channel < 145 || !(ath5k_channel & 1)) {
		clock = 1;
		data1 = ((ath5k_hw_bitswap(ath5k_channel - 24, 8) & 0xff) << 2) |
			(clock << 1) | (1 << 10) | 1;
	} else {
		clock = 0;
		data1 = ((ath5k_hw_bitswap((ath5k_channel - 24) / 2, 8) & 0xff)
			<< 2) | (clock << 1) | (1 << 10) | 1;
	}

	ath5k_hw_reg_write(ah, (data1 & 0xff) | ((data0 & 0xff) << 8),
			AR5K_RF_BUFFER);
	ath5k_hw_reg_write(ah, ((data1 >> 8) & 0xff) | (data0 & 0xff00),
			AR5K_RF_BUFFER_CONTROL_3);

	return 0;
}

/*
 * Set channel on 5112
 */
static int ath5k_hw_rf5112_channel(struct ath5k_hw *ah,
		struct ieee80211_channel *channel)
{
	u32 data, data0, data1, data2;
	u16 c;

	data = data0 = data1 = data2 = 0;
	c = channel->freq;

	/*
	 * Set the channel on the RF5112 or newer
	 */
	if (c < 4800) {
		if (!((c - 2224) % 5)) {
			data0 = ((2 * (c - 704)) - 3040) / 10;
			data1 = 1;
		} else if (!((c - 2192) % 5)) {
			data0 = ((2 * (c - 672)) - 3040) / 10;
			data1 = 0;
		} else
			return -EINVAL;

		data0 = ath5k_hw_bitswap((data0 << 2) & 0xff, 8);
	} else {
		if (!(c % 20) && c >= 5120) {
			data0 = ath5k_hw_bitswap(((c - 4800) / 20 << 2), 8);
			data2 = ath5k_hw_bitswap(3, 2);
		} else if (!(c % 10)) {
			data0 = ath5k_hw_bitswap(((c - 4800) / 10 << 1), 8);
			data2 = ath5k_hw_bitswap(2, 2);
		} else if (!(c % 5)) {
			data0 = ath5k_hw_bitswap((c - 4800) / 5, 8);
			data2 = ath5k_hw_bitswap(1, 2);
		} else
			return -EINVAL;
	}

	data = (data0 << 4) | (data1 << 1) | (data2 << 2) | 0x1001;

	ath5k_hw_reg_write(ah, data & 0xff, AR5K_RF_BUFFER);
	ath5k_hw_reg_write(ah, (data >> 8) & 0x7f, AR5K_RF_BUFFER_CONTROL_5);

	return 0;
}

/*
 * Set a channel on the radio chip
 */
int ath5k_hw_channel(struct ath5k_hw *ah, struct ieee80211_channel *channel)
{
	int ret;

	/*
	 * Check bounds supported by the PHY
	 * (don't care about regulation restrictions at this point)
	 */
	if ((channel->freq < ah->ah_capabilities.cap_range.range_2ghz_min ||
	    channel->freq > ah->ah_capabilities.cap_range.range_2ghz_max) &&
	    (channel->freq < ah->ah_capabilities.cap_range.range_5ghz_min ||
	    channel->freq > ah->ah_capabilities.cap_range.range_5ghz_max)) {
		AR5K_PRINTF("channel out of supported range (%u MHz)\n",
			channel->freq);
		return -EINVAL;
	}

	/*
	 * Set the channel and wait
	 */
	switch (ah->ah_radio) {
	case AR5K_RF5110:
		ret = ath5k_hw_rf5110_channel(ah, channel);
		break;
	case AR5K_RF5111:
		ret = ath5k_hw_rf5111_channel(ah, channel);
		break;
	default:
		ret = ath5k_hw_rf5112_channel(ah, channel);
		break;
	}

	if (ret)
		return ret;

	ah->ah_current_channel.freq = channel->freq;
	ah->ah_current_channel.val = channel->val;
	ah->ah_turbo = channel->val == CHANNEL_T ? true : false;

	return 0;
}

/*****************\
  PHY calibration
\*****************/

/*
 * Perform a PHY calibration on RF5110
 * -Fix BPSK/QAM Constellation (I/Q correction)
 * -Calculate Noise Floor
 */
static int ath5k_hw_rf5110_calibrate(struct ath5k_hw *ah,
		struct ieee80211_channel *channel)
{
	u32 phy_sig, phy_agc, phy_sat, beacon;
	s32 noise_floor;
	unsigned int i;
	int ret;

	/*
	 * Disable beacons and RX/TX queues, wait
	 */
	AR5K_REG_ENABLE_BITS(ah, AR5K_DIAG_SW_5210,
		AR5K_DIAG_SW_DIS_TX | AR5K_DIAG_SW_DIS_RX_5210);
	beacon = ath5k_hw_reg_read(ah, AR5K_BEACON_5210);
	ath5k_hw_reg_write(ah, beacon & ~AR5K_BEACON_ENABLE, AR5K_BEACON_5210);

	udelay(2300);

	/*
	 * Set the channel (with AGC turned off)
	 */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGC, AR5K_PHY_AGC_DISABLE);
	udelay(10);
	ret = ath5k_hw_channel(ah, channel);

	/*
	 * Activate PHY and wait
	 */
	ath5k_hw_reg_write(ah, AR5K_PHY_ACT_ENABLE, AR5K_PHY_ACT);
	mdelay(1);

	AR5K_REG_DISABLE_BITS(ah, AR5K_PHY_AGC, AR5K_PHY_AGC_DISABLE);

	if (ret)
		return ret;

	/*
	 * Calibrate the radio chip
	 */

	/* Remember normal state */
	phy_sig = ath5k_hw_reg_read(ah, AR5K_PHY_SIG);
	phy_agc = ath5k_hw_reg_read(ah, AR5K_PHY_AGCCOARSE);
	phy_sat = ath5k_hw_reg_read(ah, AR5K_PHY_ADCSAT);

	/* Update radio registers */
	ath5k_hw_reg_write(ah, (phy_sig & ~(AR5K_PHY_SIG_FIRPWR)) |
		AR5K_REG_SM(-1, AR5K_PHY_SIG_FIRPWR), AR5K_PHY_SIG);

	ath5k_hw_reg_write(ah, (phy_agc & ~(AR5K_PHY_AGCCOARSE_HI |
			AR5K_PHY_AGCCOARSE_LO)) |
		AR5K_REG_SM(-1, AR5K_PHY_AGCCOARSE_HI) |
		AR5K_REG_SM(-127, AR5K_PHY_AGCCOARSE_LO), AR5K_PHY_AGCCOARSE);

	ath5k_hw_reg_write(ah, (phy_sat & ~(AR5K_PHY_ADCSAT_ICNT |
			AR5K_PHY_ADCSAT_THR)) |
		AR5K_REG_SM(2, AR5K_PHY_ADCSAT_ICNT) |
		AR5K_REG_SM(12, AR5K_PHY_ADCSAT_THR), AR5K_PHY_ADCSAT);

	udelay(20);

	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGC, AR5K_PHY_AGC_DISABLE);
	udelay(10);
	ath5k_hw_reg_write(ah, AR5K_PHY_RFSTG_DISABLE, AR5K_PHY_RFSTG);
	AR5K_REG_DISABLE_BITS(ah, AR5K_PHY_AGC, AR5K_PHY_AGC_DISABLE);

	mdelay(1);

	/*
	 * Enable calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGCCTL, AR5K_PHY_AGCCTL_CAL);

	ret = ath5k_hw_register_timeout(ah, AR5K_PHY_AGCCTL,
			AR5K_PHY_AGCCTL_CAL, 0, false);

	/* Reset to normal state */
	ath5k_hw_reg_write(ah, phy_sig, AR5K_PHY_SIG);
	ath5k_hw_reg_write(ah, phy_agc, AR5K_PHY_AGCCOARSE);
	ath5k_hw_reg_write(ah, phy_sat, AR5K_PHY_ADCSAT);

	if (ret) {
		AR5K_PRINTF("calibration timeout (%uMHz)\n", channel->freq);
		return ret;
	}

	/*
	 * Enable noise floor calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGCCTL, AR5K_PHY_AGCCTL_NF);

	ret = ath5k_hw_register_timeout(ah, AR5K_PHY_AGCCTL,
			AR5K_PHY_AGCCTL_NF, 0, false);
	if (ret) {
		AR5K_PRINTF("noise floor calibration timeout (%uMHz)\n",
				channel->freq);
		return ret;
	}

	/* Wait until the noise floor is calibrated */
	for (i = 20; i > 0; i--) {
		mdelay(1);
		noise_floor = ath5k_hw_reg_read(ah, AR5K_PHY_NF);

		if (AR5K_PHY_NF_RVAL(noise_floor) & AR5K_PHY_NF_ACTIVE)
			noise_floor = AR5K_PHY_NF_AVAL(noise_floor);

		if (noise_floor <= AR5K_TUNE_NOISE_FLOOR)
			break;
	}

	if (noise_floor > AR5K_TUNE_NOISE_FLOOR) {
		AR5K_PRINTF("noise floor calibration failed (%uMHz)\n",
			channel->freq);
		return -EIO;
	}

	/*
	 * Re-enable RX/TX and beacons
	 */
	AR5K_REG_DISABLE_BITS(ah, AR5K_DIAG_SW_5210,
		AR5K_DIAG_SW_DIS_TX | AR5K_DIAG_SW_DIS_RX_5210);
	ath5k_hw_reg_write(ah, beacon, AR5K_BEACON_5210);

	return 0;
}

/*
 * Perform a PHY calibration on RF5111/5112
 */
static int ath5k_hw_rf511x_calibrate(struct ath5k_hw *ah,
		struct ieee80211_channel *channel)
{
	u32 i_pwr, q_pwr;
	s32 iq_corr, i_coff, i_coffd, q_coff, q_coffd;
	AR5K_TRACE;

	if (ah->ah_calibration == false ||
			ath5k_hw_reg_read(ah, AR5K_PHY_IQ) & AR5K_PHY_IQ_RUN)
		goto done;

	ah->ah_calibration = false;

	iq_corr = ath5k_hw_reg_read(ah, AR5K_PHY_IQRES_CAL_CORR);
	i_pwr = ath5k_hw_reg_read(ah, AR5K_PHY_IQRES_CAL_PWR_I);
	q_pwr = ath5k_hw_reg_read(ah, AR5K_PHY_IQRES_CAL_PWR_Q);
	i_coffd = ((i_pwr >> 1) + (q_pwr >> 1)) >> 7;
	q_coffd = q_pwr >> 6;

	if (i_coffd == 0 || q_coffd == 0)
		goto done;

	i_coff = ((-iq_corr) / i_coffd) & 0x3f;
	q_coff = (((s32)i_pwr / q_coffd) - 64) & 0x1f;

	/* Commit new IQ value */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_IQ, AR5K_PHY_IQ_CORR_ENABLE |
		((u32)q_coff) | ((u32)i_coff << AR5K_PHY_IQ_CORR_Q_I_COFF_S));

done:
	/* Start noise floor calibration */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGCCTL, AR5K_PHY_AGCCTL_NF);

	/* Request RF gain */
	if (channel->val & CHANNEL_5GHZ) {
		ath5k_hw_reg_write(ah, AR5K_REG_SM(ah->ah_txpower.txp_max,
			AR5K_PHY_PAPD_PROBE_TXPOWER) |
			AR5K_PHY_PAPD_PROBE_TX_NEXT, AR5K_PHY_PAPD_PROBE);
		ah->ah_rf_gain = AR5K_RFGAIN_READ_REQUESTED;
	}

	return 0;
}

/*
 * Perform a PHY calibration
 */
int ath5k_hw_phy_calibrate(struct ath5k_hw *ah,
		struct ieee80211_channel *channel)
{
	int ret;

	if (ah->ah_radio == AR5K_RF5110)
		ret = ath5k_hw_rf5110_calibrate(ah, channel);
	else
		ret = ath5k_hw_rf511x_calibrate(ah, channel);

	return ret;
}

int ath5k_hw_phy_disable(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	ath5k_hw_reg_write(ah, AR5K_PHY_ACT_DISABLE, AR5K_PHY_ACT);

	return 0;
}

/********************\
  Misc PHY functions
\********************/

/*
 * Get the PHY Chip revision
 */
u16 ath5k_hw_radio_revision(struct ath5k_hw *ah, unsigned int chan)
{
	unsigned int i;
	u32 srev;
	u16 ret;

	AR5K_TRACE;

	/*
	 * Set the radio chip access register
	 */
	switch (chan) {
	case CHANNEL_2GHZ:
		ath5k_hw_reg_write(ah, AR5K_PHY_SHIFT_2GHZ, AR5K_PHY(0));
		break;
	case CHANNEL_5GHZ:
		ath5k_hw_reg_write(ah, AR5K_PHY_SHIFT_5GHZ, AR5K_PHY(0));
		break;
	default:
		return 0;
	}

	mdelay(2);

	/* ...wait until PHY is ready and read the selected radio revision */
	ath5k_hw_reg_write(ah, 0x00001c16, AR5K_PHY(0x34));

	for (i = 0; i < 8; i++)
		ath5k_hw_reg_write(ah, 0x00010000, AR5K_PHY(0x20));

	if (ah->ah_version == AR5K_AR5210) {
		srev = ath5k_hw_reg_read(ah, AR5K_PHY(256) >> 28) & 0xf;
		ret = (u16)ath5k_hw_bitswap(srev, 4) + 1;
	} else {
		srev = (ath5k_hw_reg_read(ah, AR5K_PHY(0x100)) >> 24) & 0xff;
		ret = (u16)ath5k_hw_bitswap(((srev & 0xf0) >> 4) |
				((srev & 0x0f) << 4), 8);
	}

	/* Reset to the 5GHz mode */
	ath5k_hw_reg_write(ah, AR5K_PHY_SHIFT_5GHZ, AR5K_PHY(0));

	return ret;
}

void /*TODO:Boundary check*/
ath5k_hw_set_def_antenna(struct ath5k_hw *ah, unsigned int ant)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	if (ah->ah_version != AR5K_AR5210)
		ath5k_hw_reg_write(ah, ant, AR5K_DEFAULT_ANTENNA);
}

unsigned int ath5k_hw_get_def_antenna(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	/*Just a try M.F.*/
	if (ah->ah_version != AR5K_AR5210)
		return ath5k_hw_reg_read(ah, AR5K_DEFAULT_ANTENNA);

	return false; /*XXX: What do we return for 5210 ?*/
}

/*
 * TX power setup
 */

/*
 * Initialize the tx power table (not fully implemented)
 */
static void ath5k_txpower_table(struct ath5k_hw *ah,
		struct ieee80211_channel *channel, s16 max_power)
{
	unsigned int i, min, max, n;
	u16 txpower, *rates;

	rates = ah->ah_txpower.txp_rates;

	txpower = AR5K_TUNE_DEFAULT_TXPOWER * 2;
	if (max_power > txpower)
		txpower = max_power > AR5K_TUNE_MAX_TXPOWER ?
		    AR5K_TUNE_MAX_TXPOWER : max_power;

	for (i = 0; i < AR5K_MAX_RATES; i++)
		rates[i] = txpower;

	/* XXX setup target powers by rate */

	ah->ah_txpower.txp_min = rates[7];
	ah->ah_txpower.txp_max = rates[0];
	ah->ah_txpower.txp_ofdm = rates[0];

	/* Calculate the power table */
	n = ARRAY_SIZE(ah->ah_txpower.txp_pcdac);
	min = AR5K_EEPROM_PCDAC_START;
	max = AR5K_EEPROM_PCDAC_STOP;
	for (i = 0; i < n; i += AR5K_EEPROM_PCDAC_STEP)
		ah->ah_txpower.txp_pcdac[i] =
#ifdef notyet
		min + ((i * (max - min)) / n);
#else
		min;
#endif
}

/*
 * Set transmition power
 */
int /*O.K. - txpower_table is unimplemented so this doesn't work*/
ath5k_hw_txpower(struct ath5k_hw *ah, struct ieee80211_channel *channel,
		unsigned int txpower)
{
	bool tpc = ah->ah_txpower.txp_tpc;
	unsigned int i;

	AR5K_TRACE;
	if (txpower > AR5K_TUNE_MAX_TXPOWER) {
		AR5K_PRINTF("invalid tx power: %u\n", txpower);
		return -EINVAL;
	}

	/* Reset TX power values */
	memset(&ah->ah_txpower, 0, sizeof(ah->ah_txpower));
	ah->ah_txpower.txp_tpc = tpc;

	/* Initialize TX power table */
	ath5k_txpower_table(ah, channel, txpower);

	/*
	 * Write TX power values
	 */
	for (i = 0; i < (AR5K_EEPROM_POWER_TABLE_SIZE / 2); i++) {
		ath5k_hw_reg_write(ah,
			((((ah->ah_txpower.txp_pcdac[(i << 1) + 1] << 8) | 0xff) & 0xffff) << 16) |
			(((ah->ah_txpower.txp_pcdac[(i << 1)    ] << 8) | 0xff) & 0xffff),
			AR5K_PHY_PCDAC_TXPOWER(i));
	}

	ath5k_hw_reg_write(ah, AR5K_TXPOWER_OFDM(3, 24) |
		AR5K_TXPOWER_OFDM(2, 16) | AR5K_TXPOWER_OFDM(1, 8) |
		AR5K_TXPOWER_OFDM(0, 0), AR5K_PHY_TXPOWER_RATE1);

	ath5k_hw_reg_write(ah, AR5K_TXPOWER_OFDM(7, 24) |
		AR5K_TXPOWER_OFDM(6, 16) | AR5K_TXPOWER_OFDM(5, 8) |
		AR5K_TXPOWER_OFDM(4, 0), AR5K_PHY_TXPOWER_RATE2);

	ath5k_hw_reg_write(ah, AR5K_TXPOWER_CCK(10, 24) |
		AR5K_TXPOWER_CCK(9, 16) | AR5K_TXPOWER_CCK(15, 8) |
		AR5K_TXPOWER_CCK(8, 0), AR5K_PHY_TXPOWER_RATE3);

	ath5k_hw_reg_write(ah, AR5K_TXPOWER_CCK(14, 24) |
		AR5K_TXPOWER_CCK(13, 16) | AR5K_TXPOWER_CCK(12, 8) |
		AR5K_TXPOWER_CCK(11, 0), AR5K_PHY_TXPOWER_RATE4);

	if (ah->ah_txpower.txp_tpc == true)
		ath5k_hw_reg_write(ah, AR5K_PHY_TXPOWER_RATE_MAX_TPC_ENABLE |
			AR5K_TUNE_MAX_TXPOWER, AR5K_PHY_TXPOWER_RATE_MAX);
	else
		ath5k_hw_reg_write(ah, AR5K_PHY_TXPOWER_RATE_MAX |
			AR5K_TUNE_MAX_TXPOWER, AR5K_PHY_TXPOWER_RATE_MAX);

	return 0;
}

int ath5k_hw_set_txpower_limit(struct ath5k_hw *ah, unsigned int power)
{
	/*Just a try M.F.*/
	struct ieee80211_channel *channel = &ah->ah_current_channel;

	AR5K_TRACE;
#ifdef AR5K_DEBUG
	AR5K_PRINTF("changing txpower to %d\n", power);
#endif
	return ath5k_hw_txpower(ah, channel, power);
}
