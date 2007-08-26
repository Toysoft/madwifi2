/*
 * Copyright (c) 2007 The MadWiFi Team <www.madwifi.org>
 * Copyright (c) 2004-2007 Reyk Floeter <reyk@openbsd.org>
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $Id$
 */

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
	u_int16_t	ee_magic;		/* Magic Number */
	u_int16_t	ee_protect;		/* Protection bits (ath5kreg.h) */
	u_int16_t	ee_regdomain;		/* Regulatory Domain */
	u_int16_t	ee_version;		/* EEPROM Revision */
	u_int16_t	ee_header;		/* EEPROM Header (ath5kreg.h,get_capabilities) */
	u_int16_t	ee_ant_gain;		/* Antenna Gain (ath5kreg.h) */
	u_int16_t	ee_misc0;
	u_int16_t	ee_misc1;
	u_int16_t	ee_cck_ofdm_gain_delta;	/* CCK to OFDM gain delta */
	u_int16_t	ee_cck_ofdm_power_delta;/* CCK to OFDM power delta */
	u_int16_t	ee_scaled_cck_delta;

	/* Used for tx thermal adjustment (eeprom_init, rfregs) */
	u_int16_t	ee_tx_clip;
	u_int16_t	ee_pwd_84;
	u_int16_t	ee_pwd_90;
	u_int16_t	ee_gain_select;

	/* RF Calibration settings (reset, rfregs) */
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
	u_int16_t	ee_false_detect[AR5K_EEPROM_N_MODES];	/* Unused */
	u_int16_t	ee_cal_pier[AR5K_EEPROM_N_MODES][AR5K_EEPROM_N_2GHZ_CHAN]; /* Unused */
	u_int16_t	ee_channel[AR5K_EEPROM_N_MODES][AR5K_EEPROM_MAX_CHAN];	/* Empty ! */

	/* Conformance test limits (Unused) */
	u_int16_t	ee_ctls;
	u_int16_t	ee_ctl[AR5K_EEPROM_MAX_CTLS];

	/* Noise Floor Calibration settings */
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

#define AR5K_REG_READ(_reg)	ath5k_hw_reg_read(hal, _reg)

#define AR5K_REG_WRITE(_reg, _val)	ath5k_hw_reg_write(hal, _val, _reg)

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
	if ((ret = ath5k_hw_eeprom_read(hal, (_o),			\
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
 * INITIAL REGISTER VALUES
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


/*
 * RF REGISTERS
 */

/* Register dumps are done per operation mode */
#define AR5K_INI_RFGAIN_5GHZ	0
#define AR5K_INI_RFGAIN_2GHZ	1

#define AR5K_INI_VAL_11A		0
#define AR5K_INI_VAL_11A_TURBO		1
#define AR5K_INI_VAL_11B		2
#define AR5K_INI_VAL_11G		3
#define AR5K_INI_VAL_11G_TURBO		4
#define AR5K_INI_VAL_XR			0
#define AR5K_INI_VAL_MAX		5

#define AR5K_RF5111_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS
#define AR5K_RF5112_INI_RF_MAX_BANKS	AR5K_MAX_RF_BANKS

static inline u_int32_t
ath5k_hw_bitswap(u_int32_t val, u_int bits)
{
	u_int32_t retval = 0, bit, i;

	for (i = 0; i < bits; i++) {
		bit = (val >> i) & 1;
		retval = (retval << 1) | bit;
	}

	return (retval);
}


