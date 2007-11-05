 /*
 * Copyright (c) 2004-2007 Reyk Floeter <reyk@openbsd.org>
 * Copyright (c) 2006-2007 Nick Kossifidis <mickflemm@gmail.com>
 * Copyright (c) 2007 Matthew W. S. Bell  <mentor@madwifi.org>
 * Copyright (c) 2007 Luis Rodriguez <mcgrof@winlab.rutgers.edu>
 * Copyright (c) 2007 Pavel Roskin <proski@gnu.org>
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

/*
 * HW related functions for Atheros Wireless LAN devices.
 */

#include <linux/pci.h>
#include <linux/delay.h>

#include "reg.h"
#include "base.h"

/*Rate tables*/
static const struct ath5k_rate_table ath5k_rt_11a = AR5K_RATES_11A;
static const struct ath5k_rate_table ath5k_rt_11b = AR5K_RATES_11B;
static const struct ath5k_rate_table ath5k_rt_11g = AR5K_RATES_11G;
static const struct ath5k_rate_table ath5k_rt_turbo = AR5K_RATES_TURBO;
static const struct ath5k_rate_table ath5k_rt_xr = AR5K_RATES_XR;

/*Prototypes*/
static int ath5k_hw_nic_reset(struct ath5k_hw *, u32);
static int ath5k_hw_nic_wakeup(struct ath5k_hw *, int, bool);
static int ath5k_hw_setup_4word_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, unsigned int, enum ath5k_pkt_type, unsigned int,
	unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
	unsigned int, unsigned int);
static bool ath5k_hw_setup_xr_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
	unsigned int);
static int ath5k_hw_proc_4word_tx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_setup_2word_tx_desc(struct ath5k_hw *, struct ath5k_desc *,
	unsigned int, unsigned int, enum ath5k_pkt_type, unsigned int,
	unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
	unsigned int, unsigned int);
static int ath5k_hw_proc_2word_tx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_proc_new_rx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_proc_old_rx_status(struct ath5k_hw *, struct ath5k_desc *);
static int ath5k_hw_get_capabilities(struct ath5k_hw *);

static int ath5k_eeprom_init(struct ath5k_hw *);
static int ath5k_eeprom_read_mac(struct ath5k_hw *, u8 *);

static int ath5k_hw_enable_pspoll(struct ath5k_hw *, u8 *, u16);
static int ath5k_hw_disable_pspoll(struct ath5k_hw *);

/*
 * Enable to overwrite the country code (use "00" for debug)
 */
#if 0
#define COUNTRYCODE "00"
#endif

/*******************\
  General Functions
\*******************/

/*
 * Functions used internaly
 */

static inline unsigned int ath5k_hw_htoclock(unsigned int usec, bool turbo)
{
	return turbo == true ? (usec * 80) : (usec * 40);
}

static inline unsigned int ath5k_hw_clocktoh(unsigned int clock, bool turbo)
{
	return turbo == true ? (clock / 80) : (clock / 40);
}

/*
 * Check if a register write has been completed
 */
int ath5k_hw_register_timeout(struct ath5k_hw *ah, u32 reg, u32 flag, u32 val,
		bool is_set)
{
	int i;
	u32 data;

	for (i = AR5K_TUNE_REGISTER_TIMEOUT; i > 0; i--) {
		data = ath5k_hw_reg_read(ah, reg);
		if ((is_set == true) && (data & flag))
			break;
		else if ((data & flag) == val)
			break;
		udelay(15);
	}

	return (i <= 0) ? -EAGAIN : 0;
}


/***************************************\
	Attach/Detach Functions
\***************************************/

/*
 * Check if the device is supported and initialize the needed structs
 */
struct ath5k_hw *ath5k_hw_attach(struct ath5k_softc *sc, u8 mac_version)
{
	struct ath5k_hw *ah;
	u8 mac[ETH_ALEN];
	int ret;
	u32 srev;

	/*If we passed the test malloc a ath5k_hw struct*/
	ah = kzalloc(sizeof(struct ath5k_hw), GFP_KERNEL);
	if (ah == NULL) {
		ret = -ENOMEM;
		AR5K_PRINT("out of memory\n");
		goto err;
	}

	ah->ah_sc = sc;
	ah->ah_iobase = sc->iobase;

	/*
	 * HW information
	 */

	/* Get reg domain from eeprom */
	ath5k_get_regdomain(ah);

	ah->ah_op_mode = IEEE80211_IF_TYPE_STA;
	ah->ah_radar.r_enabled = AR5K_TUNE_RADAR_ALERT;
	ah->ah_turbo = false;
	ah->ah_txpower.txp_tpc = AR5K_TUNE_TPC_TXPOWER;
	ah->ah_imr = 0;
	ah->ah_atim_window = 0;
	ah->ah_aifs = AR5K_TUNE_AIFS;
	ah->ah_cw_min = AR5K_TUNE_CWMIN;
	ah->ah_limit_tx_retries = AR5K_INIT_TX_RETRY;
	ah->ah_software_retry = false;
	ah->ah_ant_diversity = AR5K_TUNE_ANT_DIVERSITY;

	/*
	 * Set the mac revision based on the pci id
	 */
	ah->ah_version = mac_version;

	/*Fill the ath5k_hw struct with the needed functions*/
	if (ah->ah_version == AR5K_AR5212)
		ah->ah_magic = AR5K_EEPROM_MAGIC_5212;
	else if (ah->ah_version == AR5K_AR5211)
		ah->ah_magic = AR5K_EEPROM_MAGIC_5211;

	if (ah->ah_version == AR5K_AR5212) {
		ah->ah_setup_tx_desc = ath5k_hw_setup_4word_tx_desc;
		ah->ah_setup_xtx_desc = ath5k_hw_setup_xr_tx_desc;
		ah->ah_proc_tx_desc = ath5k_hw_proc_4word_tx_status;
	} else {
		ah->ah_setup_tx_desc = ath5k_hw_setup_2word_tx_desc;
		ah->ah_setup_xtx_desc = ath5k_hw_setup_xr_tx_desc;
		ah->ah_proc_tx_desc = ath5k_hw_proc_2word_tx_status;
	}

	if (ah->ah_version == AR5K_AR5212)
		ah->ah_proc_rx_desc = ath5k_hw_proc_new_rx_status;
	else if (ah->ah_version <= AR5K_AR5211)
		ah->ah_proc_rx_desc = ath5k_hw_proc_old_rx_status;

	/* Bring device out of sleep and reset it's units */
	ret = ath5k_hw_nic_wakeup(ah, AR5K_INIT_MODE, true);
	if (ret)
		goto err_free;

	/* Get MAC, PHY and RADIO revisions */
	srev = ath5k_hw_reg_read(ah, AR5K_SREV);
	ah->ah_mac_srev = srev;
	ah->ah_mac_version = AR5K_REG_MS(srev, AR5K_SREV_VER);
	ah->ah_mac_revision = AR5K_REG_MS(srev, AR5K_SREV_REV);
	ah->ah_phy_revision = ath5k_hw_reg_read(ah, AR5K_PHY_CHIP_ID) &
			0xffffffff;
	ah->ah_radio_5ghz_revision = ath5k_hw_radio_revision(ah,
			CHANNEL_5GHZ);

	if (ah->ah_version == AR5K_AR5210)
		ah->ah_radio_2ghz_revision = 0;
	else
		ah->ah_radio_2ghz_revision = ath5k_hw_radio_revision(ah,
				CHANNEL_2GHZ);

	/* Return on unsuported chips (unsupported eeprom etc) */
	if(srev >= AR5K_SREV_VER_AR5416){
		printk(KERN_ERR "ath5k: Device not yet supported.\n");
		ret = -ENODEV;
		goto err_free;
	}

	/* Warn for partially supported chips (unsupported phy etc) */
	if(srev >= AR5K_SREV_VER_AR2424){
		printk(KERN_DEBUG "ath5k: Device partially supported.\n");
	}

	/* Identify single chip solutions */
	if((srev <= AR5K_SREV_VER_AR5414) &&
	(srev >= AR5K_SREV_VER_AR2424)) {
		ah->ah_single_chip = true;
	} else {
		ah->ah_single_chip = false;
	}

	/* Single chip radio */
	if (ah->ah_radio_2ghz_revision == ah->ah_radio_5ghz_revision)
		ah->ah_radio_2ghz_revision = 0;

	/* Identify the radio chip*/
	if (ah->ah_version == AR5K_AR5210)
		ah->ah_radio = AR5K_RF5110;
	else
		ah->ah_radio = ah->ah_radio_5ghz_revision <
			AR5K_SREV_RAD_5112 ? AR5K_RF5111 : AR5K_RF5112;

	ah->ah_phy = AR5K_PHY(0);

#ifdef AR5K_DEBUG
	ath5k_hw_dump_state(ah);
#endif

	/*
	 * Get card capabilities, values, ...
	 */

	ret = ath5k_eeprom_init(ah);
	if (ret) {
		AR5K_PRINT("unable to init EEPROM\n");
		goto err_free;
	}

	/* Get misc capabilities */
	ret = ath5k_hw_get_capabilities(ah);
	if (ret) {
		AR5K_PRINTF("unable to get device capabilities: 0x%04x\n",
			sc->pdev->device);
		goto err_free;
	}

	/* Get MAC address */
	ret = ath5k_eeprom_read_mac(ah, mac);
	if (ret) {
		AR5K_PRINTF("unable to read address from EEPROM: 0x%04x\n",
			sc->pdev->device);
		goto err_free;
	}

	ath5k_hw_set_lladdr(ah, mac);
	/* Set BSSID to bcast address: ff:ff:ff:ff:ff:ff for now */
	memset(ah->bssid, 0xff, ETH_ALEN);
	ath5k_hw_set_associd(ah, ah->bssid, 0);
	ath5k_hw_set_opmode(ah);

	ath5k_hw_set_rfgain_opt(ah);

	return ah;
err_free:
	kfree(ah);
err:
	return ERR_PTR(ret);
}

/*
 * Bring up MAC + PHY Chips
 */
static int ath5k_hw_nic_wakeup(struct ath5k_hw *ah, int flags, bool initial)
{
	u32 turbo, mode, clock;
	int ret;

	turbo = 0;
	mode = 0;
	clock = 0;

	AR5K_TRACE;

	if (ah->ah_version != AR5K_AR5210) {
		/*
		 * Get channel mode flags
		 */

		if (ah->ah_radio >= AR5K_RF5112) {
			mode = AR5K_PHY_MODE_RAD_RF5112;
			clock = AR5K_PHY_PLL_RF5112;
		} else {
			mode = AR5K_PHY_MODE_RAD_RF5111;	/*Zero*/
			clock = AR5K_PHY_PLL_RF5111;		/*Zero*/
		}

		if (flags & CHANNEL_2GHZ) {
			mode |= AR5K_PHY_MODE_FREQ_2GHZ;
			clock |= AR5K_PHY_PLL_44MHZ;

			if (flags & CHANNEL_CCK) {
				mode |= AR5K_PHY_MODE_MOD_CCK;
			} else if (flags & CHANNEL_OFDM) {
				/* XXX Dynamic OFDM/CCK is not supported by the
				 * AR5211 so we set MOD_OFDM for plain g (no
				 * CCK headers) operation. We need to test
				 * this, 5211 might support ofdm-only g after
				 * all, there are also initial register values
				 * in the code for g mode (see ath5k_hw.h). */
				if (ah->ah_version == AR5K_AR5211)
					mode |= AR5K_PHY_MODE_MOD_OFDM;
				else
					mode |= AR5K_PHY_MODE_MOD_DYN;
			} else {
				AR5K_PRINT("invalid radio modulation mode\n");
				return -EINVAL;
			}
		} else if (flags & CHANNEL_5GHZ) {
			mode |= AR5K_PHY_MODE_FREQ_5GHZ;
			clock |= AR5K_PHY_PLL_40MHZ;

			if (flags & CHANNEL_OFDM)
				mode |= AR5K_PHY_MODE_MOD_OFDM;
			else {
				AR5K_PRINT("invalid radio modulation mode\n");
				return -EINVAL;
			}
		} else {
			AR5K_PRINT("invalid radio frequency mode\n");
			return -EINVAL;
		}

		if (flags & CHANNEL_TURBO)
			turbo = AR5K_PHY_TURBO_MODE | AR5K_PHY_TURBO_SHORT;
	} else { /* Reset and wakeup the device */
		if (initial == true) {
			/* ...reset hardware */
			if (ath5k_hw_nic_reset(ah, AR5K_RESET_CTL_PCI)) {
				AR5K_PRINT("failed to reset the PCI chipset\n");
				return -EIO;
			}

			mdelay(1);
		}

		/* ...wakeup */
		ret = ath5k_hw_set_power(ah, AR5K_PM_AWAKE, true, 0);
		if (ret) {
			AR5K_PRINT("failed to resume the MAC Chip\n");
			return ret;
		}

		/* ...enable Atheros turbo mode if requested */
		if (flags & CHANNEL_TURBO)
			ath5k_hw_reg_write(ah, AR5K_PHY_TURBO_MODE,
					AR5K_PHY_TURBO);

		/* ...reset chipset */
		if (ath5k_hw_nic_reset(ah, AR5K_RESET_CTL_CHIP)) {
			AR5K_PRINT("failed to reset the AR5210 chipset\n");
			return -EIO;
		}

		mdelay(1);
	}

	/* ...reset chipset and PCI device */
	if (ah->ah_single_chip == false && ath5k_hw_nic_reset(ah,
				AR5K_RESET_CTL_CHIP | AR5K_RESET_CTL_PCI)) {
		AR5K_PRINT("failed to reset the MAC Chip + PCI\n");
		return -EIO;
	}

	if (ah->ah_version == AR5K_AR5210)
		udelay(2300);

	/* ...wakeup */
	ret = ath5k_hw_set_power(ah, AR5K_PM_AWAKE, true, 0);
	if (ret) {
		AR5K_PRINT("failed to resume the MAC Chip\n");
		return ret;
	}

	/* ...final warm reset */
	if (ath5k_hw_nic_reset(ah, 0)) {
		AR5K_PRINT("failed to warm reset the MAC Chip\n");
		return -EIO;
	}

	if (ah->ah_version != AR5K_AR5210) {
		/* ...set the PHY operating mode */
		ath5k_hw_reg_write(ah, clock, AR5K_PHY_PLL);
		udelay(300);

		ath5k_hw_reg_write(ah, mode, AR5K_PHY_MODE);
		ath5k_hw_reg_write(ah, turbo, AR5K_PHY_TURBO);
	}

	return 0;
}

/*
 * Get the rate table for a specific operation mode
 */
const struct ath5k_rate_table *ath5k_hw_get_rate_table(struct ath5k_hw *ah,
		unsigned int mode)
{
	AR5K_TRACE;

	if (!test_bit(mode, ah->ah_capabilities.cap_mode))
		return NULL;

	/* Get rate tables */
	switch (mode) {
	case MODE_IEEE80211A:
		return &ath5k_rt_11a;
	case MODE_ATHEROS_TURBO:
		return &ath5k_rt_turbo;
	case MODE_IEEE80211B:
		return &ath5k_rt_11b;
	case MODE_IEEE80211G:
		return &ath5k_rt_11g;
	case MODE_ATHEROS_TURBOG:
		return &ath5k_rt_xr;
	}

	return NULL;
}

/*
 * Free the ath5k_hw struct
 */
void ath5k_hw_detach(struct ath5k_hw *ah)
{
	AR5K_TRACE;

	if (ah->ah_rf_banks != NULL)
		kfree(ah->ah_rf_banks);

	/* assume interrupts are down */
	kfree(ah);
}

/*
 * Reset function and helpers
 */

/**
 * ath5k_hw_write_ofdm_timings - set OFDM timings on AR5212
 *
 * @ah: the &struct ath5k_hw
 * @channel: the currently set channel upon reset
 *
 * Write the OFDM timings for the AR5212 upon reset. This is a helper for
 * ath5k_hw_reset(). This seems to tune the PLL a specified frequency
 * depending on the bandwidth of the channel.
 *
 */
static inline int ath5k_hw_write_ofdm_timings(struct ath5k_hw *ah,
	struct ieee80211_channel *channel)
{
	/* Get exponent and mantissa and set it */
	u32 coef_scaled, coef_exp, coef_man,
		ds_coef_exp, ds_coef_man, clock;

	if (!(ah->ah_version == AR5K_AR5212) ||
		!(channel->val & CHANNEL_OFDM))
		BUG();

	/* Seems there are two PLLs, one for baseband sampling and one
	 * for tuning. Tuning basebands are 40 MHz or 80MHz when in
	 * turbo. */
	clock = channel->val & CHANNEL_TURBO ? 80 : 40;
	coef_scaled = ((5 * (clock << 24)) / 2) /
	channel->freq;

	for (coef_exp = 31; coef_exp > 0; coef_exp--)
		if ((coef_scaled >> coef_exp) & 0x1)
			break;

	if (!coef_exp)
		return -EINVAL;

	coef_exp = 14 - (coef_exp - 24);
	coef_man = coef_scaled +
		(1 << (24 - coef_exp - 1));
	ds_coef_man = coef_man >> (24 - coef_exp);
	ds_coef_exp = coef_exp - 16;

	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_TIMING_3,
		AR5K_PHY_TIMING_3_DSC_MAN, ds_coef_man);
	AR5K_REG_WRITE_BITS(ah, AR5K_PHY_TIMING_3,
		AR5K_PHY_TIMING_3_DSC_EXP, ds_coef_exp);

	return 0;
}

/**
 * ath5k_hw_write_rate_duration - set rate duration during hw resets
 *
 * @ah: the &struct ath5k_hw
 * @driver_mode: one of enum ieee80211_phymode or our one of our own
 *     vendor modes
 *
 * Write the rate duration table for the current mode upon hw reset. This
 * is a helper for ath5k_hw_reset(). It seems all this is doing is setting
 * an ACK timeout for the hardware for the current mode for each rate. The
 * rates which are capable of short preamble (802.11b rates 2Mbps, 5.5Mbps,
 * and 11Mbps) have another register for the short preamble ACK timeout
 * calculation.
 *
 */
static inline void ath5k_hw_write_rate_duration(struct ath5k_hw *ah,
       unsigned int driver_mode)
{
	struct ath5k_softc *sc = ah->ah_sc;
	const struct ath5k_rate_table *rt;
	unsigned int i;

	/* Get rate table for the current operating mode */
	rt = ath5k_hw_get_rate_table(ah,
		driver_mode);

	/* Write rate duration table */
	for (i = 0; i < rt->rate_count; i++) {
		const struct ath5k_rate *rate, *control_rate;
		u32 reg;
		u16 tx_time;

		rate = &rt->rates[i];
		control_rate = &rt->rates[rate->control_rate];

		/* Set ACK timeout */
		reg = AR5K_RATE_DUR(rate->rate_code);

		/* An ACK frame consists of 10 bytes. If you add the FCS,
		 * which ieee80211_generic_frame_duration() adds,
		 * its 14 bytes. Note we use the control rate and not the
		 * actual rate for this rate. See mac80211 tx.c
		 * ieee80211_duration() for a brief description of
		 * what rate we should choose to TX ACKs. */
		tx_time = ieee80211_generic_frame_duration(sc->hw,
			sc->iface_id, 10, control_rate->rate_kbps/100);

		ath5k_hw_reg_write(ah, tx_time, reg);

		if (!HAS_SHPREAMBLE(i))
			continue;

		/*
		 * We're not distinguishing short preamble here,
		 * This is true, all we'll get is a longer value here
		 * which is not necessarilly bad. We could use
		 * export ieee80211_frame_duration() but that needs to be
		 * fixed first to be properly used by mac802111 drivers:
		 *
		 *  - remove erp stuff and let the routine figure ofdm
		 *    erp rates
		 *  - remove passing argument ieee80211_local as
		 *    drivers don't have access to it
		 *  - move drivers using ieee80211_generic_frame_duration()
		 *    to this
		 */
		ath5k_hw_reg_write(ah, tx_time,
			reg + (AR5K_SET_SHORT_PREAMBLE << 2));
	}
}

/*
 * Main reset function
 */
int ath5k_hw_reset(struct ath5k_hw *ah, enum ieee80211_if_types op_mode,
	struct ieee80211_channel *channel, bool change_channel)
{
	struct ath5k_eeprom_info *ee = &ah->ah_capabilities.cap_eeprom;
	u32 data, s_seq, s_ant, s_led[3];
	s32 noise_floor;
	unsigned int i, mode, freq, ee_mode, ant[2], driver_mode = -1;
	int ret;

	AR5K_TRACE;

	s_seq = 0;
	s_ant = 1;
	ee_mode = 0;
	freq = 0;
	mode = 0;

	/*
	 * Save some registers before a reset
	 */
	/*DCU/Antenna selection not available on 5210*/
	if (ah->ah_version != AR5K_AR5210) {
		if (change_channel == true) {
			/* Seq number for queue 0 -do this for all queues ? */
			s_seq = ath5k_hw_reg_read(ah,
					AR5K_QUEUE_DFS_SEQNUM(0));
			/*Default antenna*/
			s_ant = ath5k_hw_reg_read(ah, AR5K_DEFAULT_ANTENNA);
		}
	}

	/*GPIOs*/
	s_led[0] = ath5k_hw_reg_read(ah, AR5K_PCICFG) & AR5K_PCICFG_LEDSTATE;
	s_led[1] = ath5k_hw_reg_read(ah, AR5K_GPIOCR);
	s_led[2] = ath5k_hw_reg_read(ah, AR5K_GPIODO);

	if (change_channel == true && ah->ah_rf_banks != NULL)
		ath5k_hw_get_rf_gain(ah);


	/*Wakeup the device*/
	ret = ath5k_hw_nic_wakeup(ah, channel->val, false);
	if (ret)
		return ret;

	/*
	 * Initialize operating mode
	 */
	ah->ah_op_mode = op_mode;

	/*
	 * 5111/5112 Settings
	 * 5210 only comes with RF5110
	 */
	if (ah->ah_version != AR5K_AR5210) {
		if (ah->ah_radio != AR5K_RF5111 &&
				ah->ah_radio != AR5K_RF5112) {
			AR5K_PRINTF("invalid phy radio: %u\n", ah->ah_radio);
			return -EINVAL;
		}

		switch (channel->val & CHANNEL_MODES) {
		case CHANNEL_A:
			mode = AR5K_INI_VAL_11A;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			driver_mode = MODE_IEEE80211A;
			break;
		case CHANNEL_G:
			mode = AR5K_INI_VAL_11G;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11G;
			driver_mode = MODE_IEEE80211G;
			break;
		case CHANNEL_B:
			mode = AR5K_INI_VAL_11B;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11B;
			driver_mode = MODE_IEEE80211B;
			break;
		case CHANNEL_T:
			mode = AR5K_INI_VAL_11A_TURBO;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			driver_mode = MODE_ATHEROS_TURBO;
			break;
		/*Is this ok on 5211 too ?*/
		case CHANNEL_TG:
			mode = AR5K_INI_VAL_11G_TURBO;
			freq = AR5K_INI_RFGAIN_2GHZ;
			ee_mode = AR5K_EEPROM_MODE_11G;
			driver_mode = MODE_ATHEROS_TURBOG;
			break;
		case CHANNEL_XR:
			if (ah->ah_version == AR5K_AR5211) {
				AR5K_PRINTF("XR mode not available on 5211");
				return -EINVAL;
			}
			mode = AR5K_INI_VAL_XR;
			freq = AR5K_INI_RFGAIN_5GHZ;
			ee_mode = AR5K_EEPROM_MODE_11A;
			driver_mode = MODE_IEEE80211A;
			break;
		default:
			AR5K_PRINTF("invalid channel: %d\n", channel->freq);
			return -EINVAL;
		}

		/* PHY access enable */
		ath5k_hw_reg_write(ah, AR5K_PHY_SHIFT_5GHZ, AR5K_PHY(0));

	}

	ath5k_hw_write_initvals(ah, mode, change_channel);

	/*
	 * 5211/5212 Specific
	 */
	if (ah->ah_version != AR5K_AR5210) {
		/*
		 * Write initial RF gain settings
		 * This should work for both 5111/5112
		 */
		ret = ath5k_hw_rfgain(ah, freq);
		if (ret)
			return ret;

		mdelay(1);

		if (ah->ah_version == AR5K_AR5212)
			ath5k_hw_write_rate_duration(ah, driver_mode);

		/* Fix for first revision of the RF5112 RF chipset */
		if (ah->ah_radio >= AR5K_RF5112 &&
				ah->ah_radio_5ghz_revision <
				AR5K_SREV_RAD_5112A) {
			ath5k_hw_reg_write(ah, AR5K_PHY_CCKTXCTL_WORLD,
					AR5K_PHY_CCKTXCTL);
			if (channel->val & CHANNEL_A)
				data = 0xffb81020;
			else
				data = 0xffb80d20;
			ath5k_hw_reg_write(ah, data, AR5K_PHY_FRAME_CTL);
		}

		/*
		 * Set TX power (XXX use txpower from net80211)
		 */
		ret = ath5k_hw_txpower(ah, channel, AR5K_TUNE_DEFAULT_TXPOWER);
		if (ret)
			return ret;

		/*
		 * Write RF registers
		 * TODO:Does this work on 5211 (5111) ?
		 */
		ret = ath5k_hw_rfregs(ah, channel, mode);
		if (ret)
			return ret;

		/*
		 * Configure additional registers
		 */

		/* Write OFDM timings on 5212*/
		if (ah->ah_version == AR5K_AR5212 &&
			channel->val & CHANNEL_OFDM) {
			ret = ath5k_hw_write_ofdm_timings(ah, channel);
			if (ret)
				return ret;
		}

		/*Enable/disable 802.11b mode on 5111
		(enable 2111 frequency converter + CCK)*/
		if (ah->ah_radio == AR5K_RF5111) {
			if (driver_mode == MODE_IEEE80211B)
				AR5K_REG_ENABLE_BITS(ah, AR5K_TXCFG,
				    AR5K_TXCFG_B_MODE);
			else
				AR5K_REG_DISABLE_BITS(ah, AR5K_TXCFG,
				    AR5K_TXCFG_B_MODE);
		}

		/* Set antenna mode */
		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x44),
			ah->ah_antenna[ee_mode][0], 0xfffffc06);

		if (freq == AR5K_INI_RFGAIN_2GHZ)
			ant[0] = ant[1] = AR5K_ANT_FIXED_B;
		else
			ant[0] = ant[1] = AR5K_ANT_FIXED_A;

		ath5k_hw_reg_write(ah, ah->ah_antenna[ee_mode][ant[0]],
			AR5K_PHY_ANT_SWITCH_TABLE_0);
		ath5k_hw_reg_write(ah, ah->ah_antenna[ee_mode][ant[1]],
			AR5K_PHY_ANT_SWITCH_TABLE_1);

		/* Commit values from EEPROM */
		if (ah->ah_radio == AR5K_RF5111)
			AR5K_REG_WRITE_BITS(ah, AR5K_PHY_FRAME_CTL,
			    AR5K_PHY_FRAME_CTL_TX_CLIP, ee->ee_tx_clip);

		ath5k_hw_reg_write(ah,
			AR5K_PHY_NF_SVAL(ee->ee_noise_floor_thr[ee_mode]),
			AR5K_PHY(0x5a));

		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x11),
			(ee->ee_switch_settling[ee_mode] << 7) & 0x3f80,
			0xffffc07f);
		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x12),
			(ee->ee_ant_tx_rx[ee_mode] << 12) & 0x3f000,
			0xfffc0fff);
		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x14),
			(ee->ee_adc_desired_size[ee_mode] & 0x00ff) |
			((ee->ee_pga_desired_size[ee_mode] << 8) & 0xff00),
			0xffff0000);

		ath5k_hw_reg_write(ah,
			(ee->ee_tx_end2xpa_disable[ee_mode] << 24) |
			(ee->ee_tx_end2xpa_disable[ee_mode] << 16) |
			(ee->ee_tx_frm2xpa_enable[ee_mode] << 8) |
			(ee->ee_tx_frm2xpa_enable[ee_mode]), AR5K_PHY(0x0d));

		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x0a),
			ee->ee_tx_end2xlna_enable[ee_mode] << 8, 0xffff00ff);
		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x19),
			(ee->ee_thr_62[ee_mode] << 12) & 0x7f000, 0xfff80fff);
		AR5K_REG_MASKED_BITS(ah, AR5K_PHY(0x49), 4, 0xffffff01);

		AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_IQ,
		    AR5K_PHY_IQ_CORR_ENABLE |
		    (ee->ee_i_cal[ee_mode] << AR5K_PHY_IQ_CORR_Q_I_COFF_S) |
		    ee->ee_q_cal[ee_mode]);

		if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_1)
			AR5K_REG_WRITE_BITS(ah, AR5K_PHY_GAIN_2GHZ,
				AR5K_PHY_GAIN_2GHZ_MARGIN_TXRX,
				ee->ee_margin_tx_rx[ee_mode]);

	} else {
		mdelay(1);
		/* Disable phy and wait */
		ath5k_hw_reg_write(ah, AR5K_PHY_ACT_DISABLE, AR5K_PHY_ACT);
		mdelay(1);
	}

	/*
	 * Restore saved values
	 */
	/*DCU/Antenna selection not available on 5210*/
	if (ah->ah_version != AR5K_AR5210) {
		ath5k_hw_reg_write(ah, s_seq, AR5K_QUEUE_DFS_SEQNUM(0));
		ath5k_hw_reg_write(ah, s_ant, AR5K_DEFAULT_ANTENNA);
	}
	AR5K_REG_ENABLE_BITS(ah, AR5K_PCICFG, s_led[0]);
	ath5k_hw_reg_write(ah, s_led[1], AR5K_GPIOCR);
	ath5k_hw_reg_write(ah, s_led[2], AR5K_GPIODO);

	/*
	 * Misc
	 */
	/* XXX: add ah->aid once mac80211 gives this to us */
	ath5k_hw_set_associd(ah, ah->bssid, 0);

	ath5k_hw_set_opmode(ah);
	/*PISR/SISR Not available on 5210*/
	if (ah->ah_version != AR5K_AR5210) {
		ath5k_hw_reg_write(ah, 0xffffffff, AR5K_PISR);
		/* If we later allow tuning for this, store into sc structure */
		data = AR5K_TUNE_RSSI_THRES |
			AR5K_TUNE_BMISS_THRES << AR5K_RSSI_THR_BMISS_S;
		ath5k_hw_reg_write(ah, data, AR5K_RSSI_THR);
	}

	/*
	 * Set Rx/Tx DMA Configuration
	 *(passing dma size not available on 5210)
	 */
	if (ah->ah_version != AR5K_AR5210) {
		AR5K_REG_WRITE_BITS(ah, AR5K_TXCFG, AR5K_TXCFG_SDMAMR,
				AR5K_DMASIZE_512B | AR5K_TXCFG_DMASIZE);
		AR5K_REG_WRITE_BITS(ah, AR5K_RXCFG, AR5K_RXCFG_SDMAMW,
				AR5K_DMASIZE_512B);
	}

	/*
	 * Set channel and calibrate the PHY
	 */
	ret = ath5k_hw_channel(ah, channel);
	if (ret)
		return ret;

	/*
	 * Enable the PHY and wait until completion
	 */
	ath5k_hw_reg_write(ah, AR5K_PHY_ACT_ENABLE, AR5K_PHY_ACT);

	/*
	 * 5111/5112 Specific
	 */
	if (ah->ah_version != AR5K_AR5210) {
		data = ath5k_hw_reg_read(ah, AR5K_PHY_RX_DELAY) &
			AR5K_PHY_RX_DELAY_M;
		data = (channel->val & CHANNEL_CCK) ?
			((data << 2) / 22) : (data / 10);

		udelay(100 + data);
	} else {
		mdelay(1);
	}

	/*
	 * Enable calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_CAL);

	if (ath5k_hw_register_timeout(ah, AR5K_PHY_AGCCTL,
			AR5K_PHY_AGCCTL_CAL, 0, false)) {
		AR5K_PRINTF("calibration timeout (%uMHz)\n", channel->freq);
		return -EAGAIN;
	}

	/*
	 * Enable noise floor calibration and wait until completion
	 */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_AGCCTL,
				AR5K_PHY_AGCCTL_NF);

	if (ath5k_hw_register_timeout(ah, AR5K_PHY_AGCCTL,
			AR5K_PHY_AGCCTL_NF, 0, false)) {
		AR5K_PRINTF("noise floor calibration timeout (%uMHz)\n",
				channel->freq);
		return -EAGAIN;
	}

	/* Wait until the noise floor is calibrated and read the value */
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

	ah->ah_calibration = false;

	/* A and G modes can use QAM modulation which requires enabling
	 * I and Q calibration. Don't bother in B mode. */
	if (!(driver_mode == MODE_IEEE80211B)) {
		ah->ah_calibration = true;
		AR5K_REG_WRITE_BITS(ah, AR5K_PHY_IQ,
				AR5K_PHY_IQ_CAL_NUM_LOG_MAX, 15);
		AR5K_REG_ENABLE_BITS(ah, AR5K_PHY_IQ,
				AR5K_PHY_IQ_RUN);
	}

	/*
	 * Reset queues and start beacon timers at the end of the reset routine
	 */
	for (i = 0; i < ah->ah_capabilities.cap_queues.q_tx_num; i++) {
		/*No QCU on 5210*/
		if (ah->ah_version != AR5K_AR5210)
			AR5K_REG_WRITE_Q(ah, AR5K_QUEUE_QCUMASK(i), i);

		ret = ath5k_hw_reset_tx_queue(ah, i);
		if (ret) {
			AR5K_PRINTF("failed to reset TX queue #%d\n", i);
			return ret;
		}
	}

	/* Pre-enable interrupts on 5211/5212*/
	if (ah->ah_version != AR5K_AR5210)
		ath5k_hw_set_intr(ah, AR5K_INT_RX | AR5K_INT_TX |
				AR5K_INT_FATAL);

	/*
	 * Set RF kill flags if supported by the device (read from the EEPROM)
	 * Disable gpio_intr for now since it results system hang.
	 * TODO: Handle this in ath5k_intr
	 */
#if 0
	if (AR5K_EEPROM_HDR_RFKILL(ah->ah_capabilities.cap_eeprom.ee_header)) {
		ath5k_hw_set_gpio_input(ah, 0);
		ah->ah_gpio[0] = ath5k_hw_get_gpio(ah, 0);
		if (ah->ah_gpio[0] == 0)
			ath5k_hw_set_gpio_intr(ah, 0, 1);
		else
			ath5k_hw_set_gpio_intr(ah, 0, 0);
	}
#endif

	/*
	 * Set the 32MHz reference clock on 5212 phy clock sleep register
	 */
	if (ah->ah_version == AR5K_AR5212) {
		ath5k_hw_reg_write(ah, AR5K_PHY_SCR_32MHZ, AR5K_PHY_SCR);
		ath5k_hw_reg_write(ah, AR5K_PHY_SLMT_32MHZ, AR5K_PHY_SLMT);
		ath5k_hw_reg_write(ah, AR5K_PHY_SCAL_32MHZ, AR5K_PHY_SCAL);
		ath5k_hw_reg_write(ah, AR5K_PHY_SCLOCK_32MHZ, AR5K_PHY_SCLOCK);
		ath5k_hw_reg_write(ah, AR5K_PHY_SDELAY_32MHZ, AR5K_PHY_SDELAY);
		ath5k_hw_reg_write(ah, ah->ah_radio == AR5K_RF5111 ?
			AR5K_PHY_SPENDING_RF5111 : AR5K_PHY_SPENDING_RF5112,
			AR5K_PHY_SPENDING);
	}

	/*
	 * Disable beacons and reset the register
	 */
	AR5K_REG_DISABLE_BITS(ah, AR5K_BEACON, AR5K_BEACON_ENABLE |
			AR5K_BEACON_RESET_TSF);

	return 0;
}

/*
 * Reset chipset
 */
static int ath5k_hw_nic_reset(struct ath5k_hw *ah, u32 val)
{
	int ret;
	u32 mask = val ? val : ~0U;

	AR5K_TRACE;

	/* Read-and-clear RX Descriptor Pointer*/
	ath5k_hw_reg_read(ah, AR5K_RXDP);

	/*
	 * Reset the device and wait until success
	 */
	ath5k_hw_reg_write(ah, val, AR5K_RESET_CTL);

	/* Wait at least 128 PCI clocks */
	udelay(15);

	if (ah->ah_version == AR5K_AR5210) {
		val &= AR5K_RESET_CTL_CHIP;
		mask &= AR5K_RESET_CTL_CHIP;
	} else {
		val &= AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_BASEBAND;
		mask &= AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_BASEBAND;
	}

	ret = ath5k_hw_register_timeout(ah, AR5K_RESET_CTL, mask, val, false);

	/*
	 * Reset configuration register (for hw byte-swap). Note that this
	 * is only set for big endian. We do the necessary magic in
	 * AR5K_INIT_CFG.
	 */
	if ((val & AR5K_RESET_CTL_PCU) == 0)
		ath5k_hw_reg_write(ah, AR5K_INIT_CFG, AR5K_CFG);

	return ret;
}

/*
 * Power management functions
 */

/*
 * Sleep control
 */
int ath5k_hw_set_power(struct ath5k_hw *ah, enum ath5k_power_mode mode,
		bool set_chip, u16 sleep_duration)
{
	unsigned int i;
	u32 staid;

	AR5K_TRACE;
	staid = ath5k_hw_reg_read(ah, AR5K_STA_ID1);

	switch (mode) {
	case AR5K_PM_AUTO:
		staid &= ~AR5K_STA_ID1_DEFAULT_ANTENNA;
		/* fallthrough */
	case AR5K_PM_NETWORK_SLEEP:
		if (set_chip == true)
			ath5k_hw_reg_write(ah,
				AR5K_SLEEP_CTL_SLE | sleep_duration,
				AR5K_SLEEP_CTL);

		staid |= AR5K_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_FULL_SLEEP:
		if (set_chip == true)
			ath5k_hw_reg_write(ah, AR5K_SLEEP_CTL_SLE_SLP,
				AR5K_SLEEP_CTL);

		staid |= AR5K_STA_ID1_PWR_SV;
		break;

	case AR5K_PM_AWAKE:
		if (set_chip == false)
			goto commit;

		ath5k_hw_reg_write(ah, AR5K_SLEEP_CTL_SLE_WAKE,
				AR5K_SLEEP_CTL);

		for (i = 5000; i > 0; i--) {
			/* Check if the chip did wake up */
			if ((ath5k_hw_reg_read(ah, AR5K_PCICFG) &
					AR5K_PCICFG_SPWR_DN) == 0)
				break;

			/* Wait a bit and retry */
			udelay(200);
			ath5k_hw_reg_write(ah, AR5K_SLEEP_CTL_SLE_WAKE,
				AR5K_SLEEP_CTL);
		}

		/* Fail if the chip didn't wake up */
		if (i <= 0)
			return -EIO;

		staid &= ~AR5K_STA_ID1_PWR_SV;
		break;

	default:
		return -EINVAL;
	}

commit:
	ah->ah_power_mode = mode;
	ath5k_hw_reg_write(ah, staid, AR5K_STA_ID1);

	return 0;
}

/***********************\
  DMA Related Functions
\***********************/

/*
 * Receive functions
 */

/*
 * Start DMA receive
 */
void ath5k_hw_start_rx(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	ath5k_hw_reg_write(ah, AR5K_CR_RXE, AR5K_CR);
}

/*
 * Stop DMA receive
 */
int ath5k_hw_stop_rx_dma(struct ath5k_hw *ah)
{
	unsigned int i;

	AR5K_TRACE;
	ath5k_hw_reg_write(ah, AR5K_CR_RXD, AR5K_CR);

	/*
	 * It may take some time to disable the DMA receive unit
	 */
	for (i = 2000; i > 0 &&
			(ath5k_hw_reg_read(ah, AR5K_CR) & AR5K_CR_RXE) != 0;
			i--)
		udelay(10);

	return i ? 0 : -EBUSY;
}

/*
 * Get the address of the RX Descriptor
 */
u32 ath5k_hw_get_rx_buf(struct ath5k_hw *ah)
{
	return ath5k_hw_reg_read(ah, AR5K_RXDP);
}

/*
 * Set the address of the RX Descriptor
 */
void ath5k_hw_put_rx_buf(struct ath5k_hw *ah, u32 phys_addr)
{
	AR5K_TRACE;

	/*TODO:Shouldn't we check if RX is enabled first ?*/
	ath5k_hw_reg_write(ah, phys_addr, AR5K_RXDP);
}

/*
 * Transmit functions
 */

/*
 * Start DMA transmit for a specific queue
 * (see also QCU/DCU functions)
 */
int ath5k_hw_tx_start(struct ath5k_hw *ah, unsigned int queue)
{
	u32 tx_queue;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (ah->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	if (ah->ah_version == AR5K_AR5210) {
		tx_queue = ath5k_hw_reg_read(ah, AR5K_CR);

		/*
		 * Set the queue by type on 5210
		 */
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_queue |= AR5K_CR_TXE0 & ~AR5K_CR_TXD0;
			break;
		case AR5K_TX_QUEUE_BEACON:
			tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(ah, AR5K_BCR_TQ1V | AR5K_BCR_BDMAE,
					AR5K_BSR);
			break;
		case AR5K_TX_QUEUE_CAB:
			tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(ah, AR5K_BCR_TQ1FV | AR5K_BCR_TQ1V |
				AR5K_BCR_BDMAE, AR5K_BSR);
			break;
		default:
			return -EINVAL;
		}
		/* Start queue */
		ath5k_hw_reg_write(ah, tx_queue, AR5K_CR);
	} else {
		/* Return if queue is disabled */
		if (AR5K_REG_READ_Q(ah, AR5K_QCU_TXD, queue))
			return -EIO;

		/* Start queue */
		AR5K_REG_WRITE_Q(ah, AR5K_QCU_TXE, queue);
	}

	return 0;
}

/*
 * Stop DMA transmit for a specific queue
 * (see also QCU/DCU functions)
 */
int ath5k_hw_stop_tx_dma(struct ath5k_hw *ah, unsigned int queue)
{
	unsigned int i = 100;
	u32 tx_queue, pending;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (ah->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	if (ah->ah_version == AR5K_AR5210) {
		tx_queue = ath5k_hw_reg_read(ah, AR5K_CR);

		/*
		 * Set by queue type
		 */
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_queue |= AR5K_CR_TXD0 & ~AR5K_CR_TXE0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			/* XXX Fix me... */
			tx_queue |= AR5K_CR_TXD1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(ah, 0, AR5K_BSR);
			break;
		default:
			return -EINVAL;
		}

		/* Stop queue */
		ath5k_hw_reg_write(ah, tx_queue, AR5K_CR);
	} else {
		/*
		 * Schedule TX disable and wait until queue is empty
		 */
		AR5K_REG_WRITE_Q(ah, AR5K_QCU_TXD, queue);

		/*Check for pending frames*/
		do {
			pending = ath5k_hw_reg_read(ah,
				AR5K_QUEUE_STATUS(queue)) &
				AR5K_QCU_STS_FRMPENDCNT;
			udelay(100);
		} while (--i && pending);

		/* Clear register */
		ath5k_hw_reg_write(ah, 0, AR5K_QCU_TXD);
	}

	/* TODO: Check for success else return error */
	return 0;
}

/*
 * Get the address of the TX Descriptor for a specific queue
 * (see also QCU/DCU functions)
 */
u32 ath5k_hw_get_tx_buf(struct ath5k_hw *ah, unsigned int queue)
{
	u16 tx_reg;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Get the transmit queue descriptor pointer from the selected queue
	 */
	/*5210 doesn't have QCU*/
	if (ah->ah_version == AR5K_AR5210) {
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_reg = AR5K_NOQCU_TXDP0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			tx_reg = AR5K_NOQCU_TXDP1;
			break;
		default:
			return 0xffffffff;
		}
	} else {
		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	return ath5k_hw_reg_read(ah, tx_reg);
}

/*
 * Set the address of the TX Descriptor for a specific queue
 * (see also QCU/DCU functions)
 */
int ath5k_hw_put_tx_buf(struct ath5k_hw *ah, unsigned int queue, u32 phys_addr)
{
	u16 tx_reg;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Set the transmit queue descriptor pointer register by type
	 * on 5210
	 */
	if (ah->ah_version == AR5K_AR5210) {
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_reg = AR5K_NOQCU_TXDP0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			tx_reg = AR5K_NOQCU_TXDP1;
			break;
		default:
			return -EINVAL;
		}
	} else {
		/*
		 * Set the transmit queue descriptor pointer for
		 * the selected queue on QCU for 5211+
		 * (this won't work if the queue is still active)
		 */
		if (AR5K_REG_READ_Q(ah, AR5K_QCU_TXE, queue))
			return -EIO;

		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	/* Set descriptor pointer */
	ath5k_hw_reg_write(ah, phys_addr, tx_reg);

	return 0;
}

/*
 * Update tx trigger level
 */
int ath5k_hw_update_tx_triglevel(struct ath5k_hw *ah, bool increase)
{
	u32 trigger_level, imr;
	int ret = -EIO;

	AR5K_TRACE;

	/*
	 * Disable interrupts by setting the mask
	 */
	imr = ath5k_hw_set_intr(ah, ah->ah_imr & ~AR5K_INT_GLOBAL);

	/*TODO: Boundary check on trigger_level*/
	trigger_level = AR5K_REG_MS(ath5k_hw_reg_read(ah, AR5K_TXCFG),
			AR5K_TXCFG_TXFULL);

	if (increase == false) {
		if (--trigger_level < AR5K_TUNE_MIN_TX_FIFO_THRES)
			goto done;
	} else
		trigger_level +=
			((AR5K_TUNE_MAX_TX_FIFO_THRES - trigger_level) / 2);

	/*
	 * Update trigger level on success
	 */
	if (ah->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(ah, trigger_level, AR5K_TRIG_LVL);
	else
		AR5K_REG_WRITE_BITS(ah, AR5K_TXCFG,
				AR5K_TXCFG_TXFULL, trigger_level);

	ret = 0;

done:
	/*
	 * Restore interrupt mask
	 */
	ath5k_hw_set_intr(ah, imr);

	return ret;
}

/*
 * Interrupt handling
 */

/*
 * Check if we have pending interrupts
 */
bool ath5k_hw_is_intr_pending(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	return ath5k_hw_reg_read(ah, AR5K_INTPEND);
}

/*
 * Get interrupt mask (ISR)
 */
int ath5k_hw_get_isr(struct ath5k_hw *ah, enum ath5k_int *interrupt_mask)
{
	u32 data;

	AR5K_TRACE;

	/*
	 * Read interrupt status from the Interrupt Status register
	 * on 5210
	 */
	if (ah->ah_version == AR5K_AR5210) {
		data = ath5k_hw_reg_read(ah, AR5K_ISR);
		if (unlikely(data == AR5K_INT_NOCARD)) {
			*interrupt_mask = data;
			return -ENODEV;
		}
	} else {
		/*
		 * Read interrupt status from the Read-And-Clear shadow register
		 * Note: PISR/SISR Not available on 5210
		 */
		data = ath5k_hw_reg_read(ah, AR5K_RAC_PISR);
	}

	/*
	 * Get abstract interrupt mask (driver-compatible)
	 */
	*interrupt_mask = (data & AR5K_INT_COMMON) & ah->ah_imr;

	if (unlikely(data == AR5K_INT_NOCARD))
		return -ENODEV;

	if (data & (AR5K_ISR_RXOK | AR5K_ISR_RXERR))
		*interrupt_mask |= AR5K_INT_RX;

	if (data & (AR5K_ISR_TXOK | AR5K_ISR_TXERR))
		*interrupt_mask |= AR5K_INT_TX;

	if (ah->ah_version != AR5K_AR5210) {
		/*HIU = Host Interface Unit (PCI etc)*/
		if (unlikely(data & (AR5K_ISR_HIUERR)))
			*interrupt_mask |= AR5K_INT_FATAL;

		/*Beacon Not Ready*/
		if (unlikely(data & (AR5K_ISR_BNR)))
			*interrupt_mask |= AR5K_INT_BNR;
	}

	/*
	 * XXX: BMISS interrupts may occur after association.
	 * I found this on 5210 code but it needs testing. If this is
	 * true we should disable them before assoc and re-enable them
	 * after a successfull assoc + some jiffies.
	 */
#if 0
	interrupt_mask &= ~AR5K_INT_BMISS;
#endif

	/*
	 * In case we didn't handle anything,
	 * print the register value.
	 */
	if (unlikely(*interrupt_mask == 0 && net_ratelimit()))
		AR5K_PRINTF("0x%08x\n", data);

	return 0;
}

/*
 * Set interrupt mask
 */
enum ath5k_int ath5k_hw_set_intr(struct ath5k_hw *ah, enum ath5k_int new_mask)
{
	enum ath5k_int old_mask, int_mask;

	/*
	 * Disable card interrupts to prevent any race conditions
	 * (they will be re-enabled afterwards).
	 */
	ath5k_hw_reg_write(ah, AR5K_IER_DISABLE, AR5K_IER);

	old_mask = ah->ah_imr;

	/*
	 * Add additional, chipset-dependent interrupt mask flags
	 * and write them to the IMR (interrupt mask register).
	 */
	int_mask = new_mask & AR5K_INT_COMMON;

	if (new_mask & AR5K_INT_RX)
		int_mask |= AR5K_IMR_RXOK | AR5K_IMR_RXERR | AR5K_IMR_RXORN |
			AR5K_IMR_RXDESC;

	if (new_mask & AR5K_INT_TX)
		int_mask |= AR5K_IMR_TXOK | AR5K_IMR_TXERR | AR5K_IMR_TXDESC |
			AR5K_IMR_TXURN;

	if (ah->ah_version != AR5K_AR5210) {
		if (new_mask & AR5K_INT_FATAL) {
			int_mask |= AR5K_IMR_HIUERR;
			AR5K_REG_ENABLE_BITS(ah, AR5K_SIMR2, AR5K_SIMR2_MCABT |
					AR5K_SIMR2_SSERR | AR5K_SIMR2_DPERR);
		}
	}

	ath5k_hw_reg_write(ah, int_mask, AR5K_PIMR);

	/* Store new interrupt mask */
	ah->ah_imr = new_mask;

	/* ..re-enable interrupts */
	ath5k_hw_reg_write(ah, AR5K_IER_ENABLE, AR5K_IER);

	return old_mask;
}


/*************************\
  EEPROM access functions
\*************************/

/*
 * Read from eeprom
 */
static int ath5k_hw_eeprom_read(struct ath5k_hw *ah, u32 offset, u16 *data)
{
	u32 status, timeout;

	AR5K_TRACE;
	/*
	 * Initialize EEPROM access
	 */
	if (ah->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(ah, AR5K_PCICFG, AR5K_PCICFG_EEAE);
		(void)ath5k_hw_reg_read(ah, AR5K_EEPROM_BASE + (4 * offset));
	} else {
		ath5k_hw_reg_write(ah, offset, AR5K_EEPROM_BASE);
		AR5K_REG_ENABLE_BITS(ah, AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_READ);
	}

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = ath5k_hw_reg_read(ah, AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_RDDONE) {
			if (status & AR5K_EEPROM_STAT_RDERR)
				return -EIO;
			*data = (u16)(ath5k_hw_reg_read(ah, AR5K_EEPROM_DATA) &
					0xffff);
			return 0;
		}
		udelay(15);
	}

	return -ETIMEDOUT;
}

/*
 * Write to eeprom - currently disabled, use at your own risk
 */
static int ath5k_hw_eeprom_write(struct ath5k_hw *ah, u32 offset, u16 data)
{
#if 0
	u32 status, timeout;

	AR5K_TRACE;

	/*
	 * Initialize eeprom access
	 */

	if (ah->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(ah, AR5K_PCICFG, AR5K_PCICFG_EEAE);
	} else {
		AR5K_REG_ENABLE_BITS(ah, AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_RESET);
	}

	/*
	 * Write data to data register
	 */

	if (ah->ah_version == AR5K_AR5210) {
		ath5k_hw_reg_write(ah, data, AR5K_EEPROM_BASE + (4 * offset));
	} else {
		ath5k_hw_reg_write(ah, offset, AR5K_EEPROM_BASE);
		ath5k_hw_reg_write(ah, data, AR5K_EEPROM_DATA);
		AR5K_REG_ENABLE_BITS(ah, AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_WRITE);
	}

	/*
	 * Check status
	 */

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = ath5k_hw_reg_read(ah, AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_WRDONE) {
			if (status & AR5K_EEPROM_STAT_WRERR)
				return EIO;
			return 0;
		}
		udelay(15);
	}
#endif
	AR5K_PRINTF("EEPROM Write is disabled!");
	return -EIO;
}

/*
 * Translate binary channel representation in EEPROM to frequency
 */
static u16 ath5k_eeprom_bin2freq(struct ath5k_hw *ah, u16 bin, unsigned int mode)
{
	u16 val;

	if (bin == AR5K_EEPROM_CHANNEL_DIS)
		return bin;

	if (mode == AR5K_EEPROM_MODE_11A) {
		if (ah->ah_ee_version > AR5K_EEPROM_VERSION_3_2)
			val = (5 * bin) + 4800;
		else
			val = bin > 62 ? (10 * 62) + (5 * (bin - 62)) + 5100 :
				(bin * 10) + 5100;
	} else {
		if (ah->ah_ee_version > AR5K_EEPROM_VERSION_3_2)
			val = bin + 2300;
		else
			val = bin + 2400;
	}

	return val;
}

/*
 * Read antenna infos from eeprom
 */
static int ath5k_eeprom_read_ants(struct ath5k_hw *ah, u32 *offset,
		unsigned int mode)
{
	struct ath5k_eeprom_info *ee = &ah->ah_capabilities.cap_eeprom;
	u32 o = *offset;
	u16 val;
	int ret, i = 0;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_switch_settling[mode]	= (val >> 8) & 0x7f;
	ee->ee_ant_tx_rx[mode]		= (val >> 2) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 4) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 12) & 0xf;
	ee->ee_ant_control[mode][i++]	= (val >> 6) & 0x3f;
	ee->ee_ant_control[mode][i++]	= val & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	= (val >> 10) & 0x3f;
	ee->ee_ant_control[mode][i++]	= (val >> 4) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 2) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 14) & 0x3;
	ee->ee_ant_control[mode][i++]	= (val >> 8) & 0x3f;
	ee->ee_ant_control[mode][i++]	= (val >> 2) & 0x3f;
	ee->ee_ant_control[mode][i]	= (val << 4) & 0x3f;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_ant_control[mode][i++]	|= (val >> 12) & 0xf;
	ee->ee_ant_control[mode][i++]	= (val >> 6) & 0x3f;
	ee->ee_ant_control[mode][i++]	= val & 0x3f;

	/* Get antenna modes */
	ah->ah_antenna[mode][0] =
	    (ee->ee_ant_control[mode][0] << 4) | 0x1;
	ah->ah_antenna[mode][AR5K_ANT_FIXED_A] =
	     ee->ee_ant_control[mode][1] 	|
	    (ee->ee_ant_control[mode][2] << 6) 	|
	    (ee->ee_ant_control[mode][3] << 12) |
	    (ee->ee_ant_control[mode][4] << 18) |
	    (ee->ee_ant_control[mode][5] << 24);
	ah->ah_antenna[mode][AR5K_ANT_FIXED_B] =
	     ee->ee_ant_control[mode][6] 	|
	    (ee->ee_ant_control[mode][7] << 6) 	|
	    (ee->ee_ant_control[mode][8] << 12) |
	    (ee->ee_ant_control[mode][9] << 18) |
	    (ee->ee_ant_control[mode][10] << 24);

	/* return new offset */
	*offset = o;

	return 0;
}

/*
 * Read supported modes from eeprom
 */
static int ath5k_eeprom_read_modes(struct ath5k_hw *ah, u32 *offset,
		unsigned int mode)
{
	struct ath5k_eeprom_info *ee = &ah->ah_capabilities.cap_eeprom;
	u32 o = *offset;
	u16 val;
	int ret;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_tx_end2xlna_enable[mode]	= (val >> 8) & 0xff;
	ee->ee_thr_62[mode]		= val & 0xff;

	if (ah->ah_ee_version <= AR5K_EEPROM_VERSION_3_2)
		ee->ee_thr_62[mode] = mode == AR5K_EEPROM_MODE_11A ? 15 : 28;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_tx_end2xpa_disable[mode]	= (val >> 8) & 0xff;
	ee->ee_tx_frm2xpa_enable[mode]	= val & 0xff;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_pga_desired_size[mode]	= (val >> 8) & 0xff;

	if ((val & 0xff) & 0x80)
		ee->ee_noise_floor_thr[mode] = -((((val & 0xff) ^ 0xff)) + 1);
	else
		ee->ee_noise_floor_thr[mode] = val & 0xff;

	if (ah->ah_ee_version <= AR5K_EEPROM_VERSION_3_2)
		ee->ee_noise_floor_thr[mode] =
		    mode == AR5K_EEPROM_MODE_11A ? -54 : -1;

	AR5K_EEPROM_READ(o++, val);
	ee->ee_xlna_gain[mode]		= (val >> 5) & 0xff;
	ee->ee_x_gain[mode]		= (val >> 1) & 0xf;
	ee->ee_xpd[mode]		= val & 0x1;

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_0)
		ee->ee_fixed_bias[mode] = (val >> 13) & 0x1;

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_3_3) {
		AR5K_EEPROM_READ(o++, val);
		ee->ee_false_detect[mode] = (val >> 6) & 0x7f;

		if (mode == AR5K_EEPROM_MODE_11A)
			ee->ee_xr_power[mode] = val & 0x3f;
		else {
			ee->ee_ob[mode][0] = val & 0x7;
			ee->ee_db[mode][0] = (val >> 3) & 0x7;
		}
	}

	if (ah->ah_ee_version < AR5K_EEPROM_VERSION_3_4) {
		ee->ee_i_gain[mode] = AR5K_EEPROM_I_GAIN;
		ee->ee_cck_ofdm_power_delta = AR5K_EEPROM_CCK_OFDM_DELTA;
	} else {
		ee->ee_i_gain[mode] = (val >> 13) & 0x7;

		AR5K_EEPROM_READ(o++, val);
		ee->ee_i_gain[mode] |= (val << 3) & 0x38;

		if (mode == AR5K_EEPROM_MODE_11G)
			ee->ee_cck_ofdm_power_delta = (val >> 3) & 0xff;
	}

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_0 &&
			mode == AR5K_EEPROM_MODE_11A) {
		ee->ee_i_cal[mode] = (val >> 8) & 0x3f;
		ee->ee_q_cal[mode] = (val >> 3) & 0x1f;
	}

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_6 &&
	    mode == AR5K_EEPROM_MODE_11G)
		ee->ee_scaled_cck_delta = (val >> 11) & 0x1f;

	/* return new offset */
	*offset = o;

	return 0;
}

/*
 * Initialize eeprom & capabilities structs
 */
static int ath5k_eeprom_init(struct ath5k_hw *ah)
{
	struct ath5k_eeprom_info *ee = &ah->ah_capabilities.cap_eeprom;
	unsigned int mode, i;
	int ret;
	u32 offset;
	u16 val;

	/* Initial TX thermal adjustment values */
	ee->ee_tx_clip = 4;
	ee->ee_pwd_84 = ee->ee_pwd_90 = 1;
	ee->ee_gain_select = 1;

	/*
	 * Read values from EEPROM and store them in the capability structure
	 */
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MAGIC, ee_magic);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_PROTECT, ee_protect);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_REG_DOMAIN, ee_regdomain);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_VERSION, ee_version);
	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_HDR, ee_header);

	/* Return if we have an old EEPROM */
	if (ah->ah_ee_version < AR5K_EEPROM_VERSION_3_0)
		return 0;

#ifdef notyet
	/*
	 * Validate the checksum of the EEPROM date. There are some
	 * devices with invalid EEPROMs.
	 */
	for (cksum = 0, offset = 0; offset < AR5K_EEPROM_INFO_MAX; offset++) {
		AR5K_EEPROM_READ(AR5K_EEPROM_INFO(offset), val);
		cksum ^= val;
	}
	if (cksum != AR5K_EEPROM_INFO_CKSUM) {
		AR5K_PRINTF("Invalid EEPROM checksum 0x%04x\n", cksum);
		return -EIO;
	}
#endif

	AR5K_EEPROM_READ_HDR(AR5K_EEPROM_ANT_GAIN(ah->ah_ee_version),
	    ee_ant_gain);

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MISC0, ee_misc0);
		AR5K_EEPROM_READ_HDR(AR5K_EEPROM_MISC1, ee_misc1);
	}

	if (ah->ah_ee_version < AR5K_EEPROM_VERSION_3_3) {
		AR5K_EEPROM_READ(AR5K_EEPROM_OBDB0_2GHZ, val);
		ee->ee_ob[AR5K_EEPROM_MODE_11B][0] = val & 0x7;
		ee->ee_db[AR5K_EEPROM_MODE_11B][0] = (val >> 3) & 0x7;

		AR5K_EEPROM_READ(AR5K_EEPROM_OBDB1_2GHZ, val);
		ee->ee_ob[AR5K_EEPROM_MODE_11G][0] = val & 0x7;
		ee->ee_db[AR5K_EEPROM_MODE_11G][0] = (val >> 3) & 0x7;
	}

	/*
	 * Get conformance test limit values
	 */
	offset = AR5K_EEPROM_CTL(ah->ah_ee_version);
	ee->ee_ctls = AR5K_EEPROM_N_CTLS(ah->ah_ee_version);

	for (i = 0; i < ee->ee_ctls; i++) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_ctl[i] = (val >> 8) & 0xff;
		ee->ee_ctl[i + 1] = val & 0xff;
	}

	/*
	 * Get values for 802.11a (5GHz)
	 */
	mode = AR5K_EEPROM_MODE_11A;

	ee->ee_turbo_max_power[mode] =
			AR5K_EEPROM_HDR_T_5GHZ_DBM(ee->ee_header);

	offset = AR5K_EEPROM_MODES_11A(ah->ah_ee_version);

	ret = ath5k_eeprom_read_ants(ah, &offset, mode);
	if (ret)
		return ret;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (s8)((val >> 8) & 0xff);
	ee->ee_ob[mode][3]		= (val >> 5) & 0x7;
	ee->ee_db[mode][3]		= (val >> 2) & 0x7;
	ee->ee_ob[mode][2]		= (val << 1) & 0x7;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_ob[mode][2]		|= (val >> 15) & 0x1;
	ee->ee_db[mode][2]		= (val >> 12) & 0x7;
	ee->ee_ob[mode][1]		= (val >> 9) & 0x7;
	ee->ee_db[mode][1]		= (val >> 6) & 0x7;
	ee->ee_ob[mode][0]		= (val >> 3) & 0x7;
	ee->ee_db[mode][0]		= val & 0x7;

	ret = ath5k_eeprom_read_modes(ah, &offset, mode);
	if (ret)
		return ret;

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_1) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_margin_tx_rx[mode] = val & 0x3f;
	}

	/*
	 * Get values for 802.11b (2.4GHz)
	 */
	mode = AR5K_EEPROM_MODE_11B;
	offset = AR5K_EEPROM_MODES_11B(ah->ah_ee_version);

	ret = ath5k_eeprom_read_ants(ah, &offset, mode);
	if (ret)
		return ret;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (s8)((val >> 8) & 0xff);
	ee->ee_ob[mode][1]		= (val >> 4) & 0x7;
	ee->ee_db[mode][1]		= val & 0x7;

	ret = ath5k_eeprom_read_modes(ah, &offset, mode);
	if (ret)
		return ret;

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][0] =
			ath5k_eeprom_bin2freq(ah, val & 0xff, mode);
		ee->ee_cal_pier[mode][1] =
			ath5k_eeprom_bin2freq(ah, (val >> 8) & 0xff, mode);

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][2] =
			ath5k_eeprom_bin2freq(ah, val & 0xff, mode);
	}

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_1)
		ee->ee_margin_tx_rx[mode] = (val >> 8) & 0x3f;

	/*
	 * Get values for 802.11g (2.4GHz)
	 */
	mode = AR5K_EEPROM_MODE_11G;
	offset = AR5K_EEPROM_MODES_11G(ah->ah_ee_version);

	ret = ath5k_eeprom_read_ants(ah, &offset, mode);
	if (ret)
		return ret;

	AR5K_EEPROM_READ(offset++, val);
	ee->ee_adc_desired_size[mode]	= (s8)((val >> 8) & 0xff);
	ee->ee_ob[mode][1]		= (val >> 4) & 0x7;
	ee->ee_db[mode][1]		= val & 0x7;

	ret = ath5k_eeprom_read_modes(ah, &offset, mode);
	if (ret)
		return ret;

	if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_0) {
		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][0] =
			ath5k_eeprom_bin2freq(ah, val & 0xff, mode);
		ee->ee_cal_pier[mode][1] =
			ath5k_eeprom_bin2freq(ah, (val >> 8) & 0xff, mode);

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_turbo_max_power[mode] = val & 0x7f;
		ee->ee_xr_power[mode] = (val >> 7) & 0x3f;

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_cal_pier[mode][2] =
			ath5k_eeprom_bin2freq(ah, val & 0xff, mode);

		if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_1)
			ee->ee_margin_tx_rx[mode] = (val >> 8) & 0x3f;

		AR5K_EEPROM_READ(offset++, val);
		ee->ee_i_cal[mode] = (val >> 8) & 0x3f;
		ee->ee_q_cal[mode] = (val >> 3) & 0x1f;

		if (ah->ah_ee_version >= AR5K_EEPROM_VERSION_4_2) {
			AR5K_EEPROM_READ(offset++, val);
			ee->ee_cck_ofdm_gain_delta = val & 0xff;
		}
	}

	/*
	 * Read 5GHz EEPROM channels
	 */

	return 0;
}

/*
 * Read the MAC address from eeprom
 */
static int ath5k_eeprom_read_mac(struct ath5k_hw *ah, u8 *mac)
{
	u8 mac_d[ETH_ALEN];
	u32 total, offset;
	u16 data;
	int octet, ret;

	memset(mac, 0, ETH_ALEN);
	memset(mac_d, 0, ETH_ALEN);

	ret = ath5k_hw_eeprom_read(ah, 0x20, &data);
	if (ret)
		return ret;

	for (offset = 0x1f, octet = 0, total = 0; offset >= 0x1d; offset--) {
		ret = ath5k_hw_eeprom_read(ah, offset, &data);
		if (ret)
			return ret;

		total += data;
		mac_d[octet + 1] = data & 0xff;
		mac_d[octet] = data >> 8;
		octet += 2;
	}

	memcpy(mac, mac_d, ETH_ALEN);

	if (!total || total == 3 * 0xffff)
		return -EINVAL;

	return 0;
}

/*
 * Read/Write regulatory domain
 */
static bool ath5k_eeprom_regulation_domain(struct ath5k_hw *ah, bool write,
	enum ath5k_regdom *regdomain)
{
	u16 ee_regdomain;

	/* Read current value */
	if (write != true) {
		ee_regdomain = ah->ah_capabilities.cap_eeprom.ee_regdomain;
		*regdomain = ath5k_regdom_to_ieee(ee_regdomain);
		return true;
	}

	ee_regdomain = ath5k_regdom_from_ieee(*regdomain);

	/* Try to write a new value */
	if (ah->ah_capabilities.cap_eeprom.ee_protect &
			AR5K_EEPROM_PROTECT_WR_128_191)
		return false;
	if (ath5k_hw_eeprom_write(ah, AR5K_EEPROM_REG_DOMAIN, ee_regdomain)!=0)
		return false;

	ah->ah_capabilities.cap_eeprom.ee_regdomain = ee_regdomain;

	return true;
}

/*
 * Use the above to write a new regulatory domain
 */
int ath5k_hw_set_regdomain(struct ath5k_hw *ah, u16 regdomain)
{
	enum ath5k_regdom ieee_regdomain;

	ieee_regdomain = ath5k_regdom_to_ieee(regdomain);

	if (ath5k_eeprom_regulation_domain(ah, true, &ieee_regdomain) == true)
		return 0;

	return -EIO;
}

/*
 * Fill the capabilities struct
 */
static int ath5k_hw_get_capabilities(struct ath5k_hw *ah)
{
	u16 ee_header;

	AR5K_TRACE;
	/* Capabilities stored in the EEPROM */
	ee_header = ah->ah_capabilities.cap_eeprom.ee_header;

	if (ah->ah_version == AR5K_AR5210) {
		/*
		 * Set radio capabilities
		 * (The AR5110 only supports the middle 5GHz band)
		 */
		ah->ah_capabilities.cap_range.range_5ghz_min = 5120;
		ah->ah_capabilities.cap_range.range_5ghz_max = 5430;
		ah->ah_capabilities.cap_range.range_2ghz_min = 0;
		ah->ah_capabilities.cap_range.range_2ghz_max = 0;

		/* Set supported modes */
		__set_bit(MODE_IEEE80211A, ah->ah_capabilities.cap_mode);
		__set_bit(MODE_ATHEROS_TURBO, ah->ah_capabilities.cap_mode);
	} else {
		/*
		 * XXX The tranceiver supports frequencies from 4920 to 6100GHz
		 * XXX and from 2312 to 2732GHz. There are problems with the
		 * XXX current ieee80211 implementation because the IEEE
		 * XXX channel mapping does not support negative channel
		 * XXX numbers (2312MHz is channel -19). Of course, this
		 * XXX doesn't matter because these channels are out of range
		 * XXX but some regulation domains like MKK (Japan) will
		 * XXX support frequencies somewhere around 4.8GHz.
		 */

		/*
		 * Set radio capabilities
		 */

		if (AR5K_EEPROM_HDR_11A(ee_header)) {
			ah->ah_capabilities.cap_range.range_5ghz_min = 5005; /* 4920 */
			ah->ah_capabilities.cap_range.range_5ghz_max = 6100;

			/* Set supported modes */
			__set_bit(MODE_IEEE80211A,
					ah->ah_capabilities.cap_mode);
			__set_bit(MODE_ATHEROS_TURBO,
					ah->ah_capabilities.cap_mode);
			if (ah->ah_version == AR5K_AR5212)
				__set_bit(MODE_ATHEROS_TURBOG,
						ah->ah_capabilities.cap_mode);
		}

		/* Enable  802.11b if a 2GHz capable radio (2111/5112) is
		 * connected */
		if (AR5K_EEPROM_HDR_11B(ee_header) ||
				AR5K_EEPROM_HDR_11G(ee_header)) {
			ah->ah_capabilities.cap_range.range_2ghz_min = 2412; /* 2312 */
			ah->ah_capabilities.cap_range.range_2ghz_max = 2732;

			if (AR5K_EEPROM_HDR_11B(ee_header))
				__set_bit(MODE_IEEE80211B,
						ah->ah_capabilities.cap_mode);

			if (AR5K_EEPROM_HDR_11G(ee_header))
				__set_bit(MODE_IEEE80211G,
						ah->ah_capabilities.cap_mode);
		}
	}

	/* GPIO */
	ah->ah_gpio_npins = AR5K_NUM_GPIO;

	/* Set number of supported TX queues */
	if (ah->ah_version == AR5K_AR5210)
		ah->ah_capabilities.cap_queues.q_tx_num =
			AR5K_NUM_TX_QUEUES_NOQCU;
	else
		ah->ah_capabilities.cap_queues.q_tx_num = AR5K_NUM_TX_QUEUES;

	return 0;
}

/*********************************\
  Protocol Control Unit Functions
\*********************************/

/*
 * Set Operation mode
 */
int ath5k_hw_set_opmode(struct ath5k_hw *ah)
{
	u32 pcu_reg, beacon_reg, low_id, high_id;

	pcu_reg = 0;
	beacon_reg = 0;

	AR5K_TRACE;

	switch (ah->ah_op_mode) {
	case IEEE80211_IF_TYPE_IBSS:
		pcu_reg |= AR5K_STA_ID1_ADHOC | AR5K_STA_ID1_DESC_ANTENNA |
			(ah->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_NO_PSPOLL : 0);
		beacon_reg |= AR5K_BCR_ADHOC;
		break;

	case IEEE80211_IF_TYPE_AP:
		pcu_reg |= AR5K_STA_ID1_AP | AR5K_STA_ID1_RTS_DEF_ANTENNA |
			(ah->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_NO_PSPOLL : 0);
		beacon_reg |= AR5K_BCR_AP;
		break;

	case IEEE80211_IF_TYPE_STA:
		pcu_reg |= AR5K_STA_ID1_DEFAULT_ANTENNA |
			(ah->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_PWR_SV : 0);
	case IEEE80211_IF_TYPE_MNTR:
		pcu_reg |= AR5K_STA_ID1_DEFAULT_ANTENNA |
			(ah->ah_version == AR5K_AR5210 ?
				AR5K_STA_ID1_NO_PSPOLL : 0);
		break;

	default:
		return -EINVAL;
	}

	/*
	 * Set PCU registers
	 */
	low_id = AR5K_LOW_ID(ah->ah_sta_id);
	high_id = AR5K_HIGH_ID(ah->ah_sta_id);
	ath5k_hw_reg_write(ah, low_id, AR5K_STA_ID0);
	ath5k_hw_reg_write(ah, pcu_reg | high_id, AR5K_STA_ID1);

	/*
	 * Set Beacon Control Register on 5210
	 */
	if (ah->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(ah, beacon_reg, AR5K_BCR);

	return 0;
}

/*
 * BSSID Functions
 */

/*
 * Get station id
 */
void ath5k_hw_get_lladdr(struct ath5k_hw *ah, u8 *mac)
{
	AR5K_TRACE;
	memcpy(mac, ah->ah_sta_id, ETH_ALEN);
}

/*
 * Set station id
 */
int ath5k_hw_set_lladdr(struct ath5k_hw *ah, const u8 *mac)
{
	u32 low_id, high_id;

	AR5K_TRACE;
	/* Set new station ID */
	memcpy(ah->ah_sta_id, mac, ETH_ALEN);

	low_id = AR5K_LOW_ID(mac);
	high_id = AR5K_HIGH_ID(mac);

	ath5k_hw_reg_write(ah, low_id, AR5K_STA_ID0);
	ath5k_hw_reg_write(ah, high_id, AR5K_STA_ID1);

	return 0;
}

/*
 * Set BSSID
 */
void ath5k_hw_set_associd(struct ath5k_hw *ah, const u8 *bssid, u16 assoc_id)
{
	u32 low_id, high_id;
	u16 tim_offset = 0;

	/*
	 * Set simple BSSID mask on 5212
	 */
	if (ah->ah_version == AR5K_AR5212) {
		ath5k_hw_reg_write(ah, 0xfffffff, AR5K_BSS_IDM0);
		ath5k_hw_reg_write(ah, 0xfffffff, AR5K_BSS_IDM1);
	}

	/*
	 * Set BSSID which triggers the "SME Join" operation
	 */
	low_id = AR5K_LOW_ID(bssid);
	high_id = AR5K_HIGH_ID(bssid);
	ath5k_hw_reg_write(ah, low_id, AR5K_BSS_ID0);
	ath5k_hw_reg_write(ah, high_id | ((assoc_id & 0x3fff) <<
				AR5K_BSS_ID1_AID_S), AR5K_BSS_ID1);
	memcpy(&ah->ah_bssid, bssid, ETH_ALEN);

	if (assoc_id == 0) {
		ath5k_hw_disable_pspoll(ah);
		return;
	}

	AR5K_REG_WRITE_BITS(ah, AR5K_BEACON, AR5K_BEACON_TIM,
			tim_offset ? tim_offset + 4 : 0);

	ath5k_hw_enable_pspoll(ah, NULL, 0);
}
/**
 * ath5k_hw_set_bssid_mask - set common bits we should listen to
 *
 * The bssid_mask is a utility used by AR5212 hardware to inform the hardware
 * which bits of the interface's MAC address should be looked at when trying
 * to decide which packets to ACK. In station mode every bit matters. In AP
 * mode with a single BSS every bit matters as well. In AP mode with
 * multiple BSSes not every bit matters.
 *
 * @ah: the &struct ath5k_hw
 * @mask: the bssid_mask, a u8 array of size ETH_ALEN
 *
 * Note that this is a simple filter and *does* not filter out all
 * relevant frames. Some non-relevant frames will get through, probability
 * jocks are welcomed to compute.
 *
 * When handling multiple BSSes (or VAPs) you can get the BSSID mask by
 * computing the set of:
 *
 *     ~ ( MAC XOR BSSID )
 *
 * When you do this you are essentially computing the common bits. Later it
 * is assumed the harware will "and" (&) the BSSID mask with the MAC address
 * to obtain the relevant bits which should match on the destination frame.
 *
 * Simple example: on your card you have have two BSSes you have created with
 * BSSID-01 and BSSID-02. Lets assume BSSID-01 will not use the MAC address.
 * There is another BSSID-03 but you are not part of it. For simplicity's sake,
 * assuming only 4 bits for a mac address and for BSSIDs you can then have:
 *
 *                  \
 * MAC:                0001 |
 * BSSID-01:   0100 | --> Belongs to us
 * BSSID-02:   1001 |
 *                  /
 * -------------------
 * BSSID-03:   0110  | --> External
 * -------------------
 *
 * Our bssid_mask would then be:
 *
 *             On loop iteration for BSSID-01:
 *             ~(0001 ^ 0100)  -> ~(0101)
 *                             ->   1010
 *             bssid_mask      =    1010
 *
 *             On loop iteration for BSSID-02:
 *             bssid_mask &= ~(0001   ^   1001)
 *             bssid_mask =   (1010)  & ~(0001 ^ 1001)
 *             bssid_mask =   (1010)  & ~(1001)
 *             bssid_mask =   (1010)  &  (0110)
 *             bssid_mask =   0010
 *
 * A bssid_mask of 0010 means "only pay attention to the second least
 * significant bit". This is because its the only bit common
 * amongst the MAC and all BSSIDs we support. To findout what the real
 * common bit is we can simply "&" the bssid_mask now with any BSSID we have
 * or our MAC address (we assume the hardware uses the MAC address).
 *
 * Now, suppose there's an incoming frame for BSSID-03:
 *
 * IFRAME-01:  0110
 *
 * An easy eye-inspeciton of this already should tell you that this frame
 * will not pass our check. This is beacuse the bssid_mask tells the
 * hardware to only look at the second least significant bit and the
 * common bit amongst the MAC and BSSIDs is 0, this frame has the 2nd LSB
 * as 1, which does not match 0.
 *
 * So with IFRAME-01 we *assume* the hardware will do:
 *
 *     allow = (IFRAME-01 & bssid_mask) == (bssid_mask & MAC) ? 1 : 0;
 *  --> allow = (0110 & 0010) == (0010 & 0001) ? 1 : 0;
 *  --> allow = (0010) == 0000 ? 1 : 0;
 *  --> allow = 0
 *
 *  Lets now test a frame that should work:
 *
 * IFRAME-02:  0001 (we should allow)
 *
 *     allow = (0001 & 1010) == 1010
 *
 *     allow = (IFRAME-02 & bssid_mask) == (bssid_mask & MAC) ? 1 : 0;
 *  --> allow = (0001 & 0010) ==  (0010 & 0001) ? 1 :0;
 *  --> allow = (0010) == (0010)
 *  --> allow = 1
 *
 * Other examples:
 *
 * IFRAME-03:  0100 --> allowed
 * IFRAME-04:  1001 --> allowed
 * IFRAME-05:  1101 --> allowed but its not for us!!!
 *
 */
int ath5k_hw_set_bssid_mask(struct ath5k_hw *ah, const u8 *mask)
{
	u32 low_id, high_id;
	AR5K_TRACE;

	if (ah->ah_version == AR5K_AR5212) {
		low_id = AR5K_LOW_ID(mask);
		high_id = AR5K_HIGH_ID(mask);

		ath5k_hw_reg_write(ah, low_id, AR5K_BSS_IDM0);
		ath5k_hw_reg_write(ah, high_id, AR5K_BSS_IDM1);

		return 0;
	}

	return -EIO;
}

/*
 * Receive start/stop functions
 */

/*
 * Start receive on PCU
 */
void ath5k_hw_start_rx_pcu(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	AR5K_REG_DISABLE_BITS(ah, AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_RX);
}

/*
 * Stop receive on PCU
 */
void ath5k_hw_stop_pcu_recv(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	AR5K_REG_ENABLE_BITS(ah, AR5K_DIAG_SW, AR5K_DIAG_SW_DIS_RX);
}

/*
 * RX Filter functions
 */

/*
 * Set multicast filter
 */
void ath5k_hw_set_mcast_filter(struct ath5k_hw *ah, u32 filter0, u32 filter1)
{
	AR5K_TRACE;
	/* Set the multicat filter */
	ath5k_hw_reg_write(ah, filter0, AR5K_MCAST_FILTER0);
	ath5k_hw_reg_write(ah, filter1, AR5K_MCAST_FILTER1);
}

/*
 * Set multicast filter by index
 */
int ath5k_hw_set_mcast_filterindex(struct ath5k_hw *ah, u32 index)
{

	AR5K_TRACE;
	if (index >= 64)
		return -EINVAL;
	else if (index >= 32)
		AR5K_REG_ENABLE_BITS(ah, AR5K_MCAST_FILTER1,
				(1 << (index - 32)));
	else
		AR5K_REG_ENABLE_BITS(ah, AR5K_MCAST_FILTER0, (1 << index));

	return 0;
}

/*
 * Clear Multicast filter by index
 */
int ath5k_hw_clear_mcast_filter_idx(struct ath5k_hw *ah, u32 index)
{

	AR5K_TRACE;
	if (index >= 64)
		return -EINVAL;
	else if (index >= 32)
		AR5K_REG_DISABLE_BITS(ah, AR5K_MCAST_FILTER1,
				(1 << (index - 32)));
	else
		AR5K_REG_DISABLE_BITS(ah, AR5K_MCAST_FILTER0, (1 << index));

	return 0;
}

/*
 * Get current rx filter
 */
u32 ath5k_hw_get_rx_filter(struct ath5k_hw *ah)
{
	u32 data, filter = 0;

	AR5K_TRACE;
	filter = ath5k_hw_reg_read(ah, AR5K_RX_FILTER);

	/*Radar detection for 5212*/
	if (ah->ah_version == AR5K_AR5212) {
		data = ath5k_hw_reg_read(ah, AR5K_PHY_ERR_FIL);

		if (data & AR5K_PHY_ERR_FIL_RADAR)
			filter |= AR5K_RX_FILTER_RADARERR;
		if (data & (AR5K_PHY_ERR_FIL_OFDM | AR5K_PHY_ERR_FIL_CCK))
			filter |= AR5K_RX_FILTER_PHYERR;
	}

	return filter;
}

/*
 * Set rx filter
 */
void ath5k_hw_set_rx_filter(struct ath5k_hw *ah, u32 filter)
{
	u32 data = 0;

	AR5K_TRACE;

	/* Set PHY error filter register on 5212*/
	if (ah->ah_version == AR5K_AR5212) {
		if (filter & AR5K_RX_FILTER_RADARERR)
			data |= AR5K_PHY_ERR_FIL_RADAR;
		if (filter & AR5K_RX_FILTER_PHYERR)
			data |= AR5K_PHY_ERR_FIL_OFDM | AR5K_PHY_ERR_FIL_CCK;
	}

	/*
	 * The AR5210 uses promiscous mode to detect radar activity
	 */
	if (ah->ah_version == AR5K_AR5210 &&
			(filter & AR5K_RX_FILTER_RADARERR)) {
		filter &= ~AR5K_RX_FILTER_RADARERR;
		filter |= AR5K_RX_FILTER_PROM;
	}

	/*Zero length DMA*/
	if (data)
		AR5K_REG_ENABLE_BITS(ah, AR5K_RXCFG, AR5K_RXCFG_ZLFDMA);
	else
		AR5K_REG_DISABLE_BITS(ah, AR5K_RXCFG, AR5K_RXCFG_ZLFDMA);

	/*Write RX Filter register*/
	ath5k_hw_reg_write(ah, filter & 0xff, AR5K_RX_FILTER);

	/*Write PHY error filter register on 5212*/
	if (ah->ah_version == AR5K_AR5212)
		ath5k_hw_reg_write(ah, data, AR5K_PHY_ERR_FIL);

}

/*
 * Beacon related functions
 */

/*
 * Get a 32bit TSF
 */
u32 ath5k_hw_get_tsf32(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	return ath5k_hw_reg_read(ah, AR5K_TSF_L32);
}

/*
 * Get the full 64bit TSF
 */
u64 ath5k_hw_get_tsf64(struct ath5k_hw *ah)
{
	u64 tsf = ath5k_hw_reg_read(ah, AR5K_TSF_U32);
	AR5K_TRACE;

	return ath5k_hw_reg_read(ah, AR5K_TSF_L32) | (tsf << 32);
}

/*
 * Force a TSF reset
 */
void ath5k_hw_reset_tsf(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	AR5K_REG_ENABLE_BITS(ah, AR5K_BEACON, AR5K_BEACON_RESET_TSF);
}

/*
 * Initialize beacon timers
 */
void ath5k_hw_init_beacon(struct ath5k_hw *ah, u32 next_beacon, u32 interval)
{
	u32 timer1, timer2, timer3;

	AR5K_TRACE;
	/*
	 * Set the additional timers by mode
	 */
	switch (ah->ah_op_mode) {
	case IEEE80211_IF_TYPE_STA:
		if (ah->ah_version == AR5K_AR5210) {
			timer1 = 0xffffffff;
			timer2 = 0xffffffff;
		} else {
			timer1 = 0x0000ffff;
			timer2 = 0x0007ffff;
		}
		break;

	default:
		timer1 = (next_beacon - AR5K_TUNE_DMA_BEACON_RESP) <<
		    0x00000003;
		timer2 = (next_beacon - AR5K_TUNE_SW_BEACON_RESP) <<
		    0x00000003;
	}

	timer3 = next_beacon + (ah->ah_atim_window ? ah->ah_atim_window : 1);

	/*
	 * Set the beacon register and enable all timers.
	 * (next beacon, DMA beacon, software beacon, ATIM window time)
	 */
	ath5k_hw_reg_write(ah, next_beacon, AR5K_TIMER0);
	ath5k_hw_reg_write(ah, timer1, AR5K_TIMER1);
	ath5k_hw_reg_write(ah, timer2, AR5K_TIMER2);
	ath5k_hw_reg_write(ah, timer3, AR5K_TIMER3);

	ath5k_hw_reg_write(ah, interval & (AR5K_BEACON_PERIOD |
			AR5K_BEACON_RESET_TSF | AR5K_BEACON_ENABLE),
		AR5K_BEACON);
}

/*
 * Set beacon timers
 */
int ath5k_hw_set_beacon_timers(struct ath5k_hw *ah,
		const struct ath5k_beacon_state *state)
{
	u32 cfp_period, next_cfp, dtim, interval, next_beacon;

	/*
	 * TODO: should be changed through *state
	 * review struct ath5k_beacon_state struct
	 *
	 * XXX: These are used for cfp period bellow, are they
	 * ok ? Is it O.K. for tsf here to be 0 or should we use
	 * get_tsf ?
	 */
	u32 dtim_count = 0; /* XXX */
	u32 cfp_count = 0; /* XXX */
	u32 tsf = 0; /* XXX */

	AR5K_TRACE;
	/* Return on an invalid beacon state */
	if (state->bs_interval < 1)
		return -EINVAL;

	interval = state->bs_interval;
	dtim = state->bs_dtim_period;

	/*
	 * PCF support?
	 */
	if (state->bs_cfp_period > 0) {
		/*
		 * Enable PCF mode and set the CFP
		 * (Contention Free Period) and timer registers
		 */
		cfp_period = state->bs_cfp_period * state->bs_dtim_period *
			state->bs_interval;
		next_cfp = (cfp_count * state->bs_dtim_period + dtim_count) *
			state->bs_interval;

		AR5K_REG_ENABLE_BITS(ah, AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA |
				AR5K_STA_ID1_PCF);
		ath5k_hw_reg_write(ah, cfp_period, AR5K_CFP_PERIOD);
		ath5k_hw_reg_write(ah, state->bs_cfp_max_duration,
				AR5K_CFP_DUR);
		ath5k_hw_reg_write(ah, (tsf + (next_cfp == 0 ? cfp_period :
						next_cfp)) << 3, AR5K_TIMER2);
	} else {
		/* Disable PCF mode */
		AR5K_REG_DISABLE_BITS(ah, AR5K_STA_ID1,
				AR5K_STA_ID1_DEFAULT_ANTENNA |
				AR5K_STA_ID1_PCF);
	}

	/*
	 * Enable the beacon timer register
	 */
	ath5k_hw_reg_write(ah, state->bs_next_beacon, AR5K_TIMER0);

	/*
	 * Start the beacon timers
	 */
	ath5k_hw_reg_write(ah, (ath5k_hw_reg_read(ah, AR5K_BEACON) &~
		(AR5K_BEACON_PERIOD | AR5K_BEACON_TIM)) |
		AR5K_REG_SM(state->bs_tim_offset ? state->bs_tim_offset + 4 : 0,
		AR5K_BEACON_TIM) | AR5K_REG_SM(state->bs_interval,
		AR5K_BEACON_PERIOD), AR5K_BEACON);

	/*
	 * Write new beacon miss threshold, if it appears to be valid
	 * XXX: Figure out right values for min <= bs_bmiss_threshold <= max
	 * and return if its not in range. We can test this by reading value and
	 * setting value to a largest value and seeing which values register.
	 */

	AR5K_REG_WRITE_BITS(ah, AR5K_RSSI_THR, AR5K_RSSI_THR_BMISS,
			state->bs_bmiss_threshold);

	/*
	 * Set sleep control register
	 * XXX: Didn't find this in 5210 code but since this register
	 * exists also in ar5k's 5210 headers i leave it as common code.
	 */
	AR5K_REG_WRITE_BITS(ah, AR5K_SLEEP_CTL, AR5K_SLEEP_CTL_SLDUR,
			(state->bs_sleep_duration - 3) << 3);

	/*
	 * Set enhanced sleep registers on 5212
	 */
	if (ah->ah_version == AR5K_AR5212) {
		if (state->bs_sleep_duration > state->bs_interval &&
				roundup(state->bs_sleep_duration, interval) ==
				state->bs_sleep_duration)
			interval = state->bs_sleep_duration;

		if (state->bs_sleep_duration > dtim && (dtim == 0 ||
				roundup(state->bs_sleep_duration, dtim) ==
				state->bs_sleep_duration))
			dtim = state->bs_sleep_duration;

		if (interval > dtim)
			return -EINVAL;

		next_beacon = interval == dtim ? state->bs_next_dtim :
			state->bs_next_beacon;

		ath5k_hw_reg_write(ah,
			AR5K_REG_SM((state->bs_next_dtim - 3) << 3,
			AR5K_SLEEP0_NEXT_DTIM) |
			AR5K_REG_SM(10, AR5K_SLEEP0_CABTO) |
			AR5K_SLEEP0_ENH_SLEEP_EN |
			AR5K_SLEEP0_ASSUME_DTIM, AR5K_SLEEP0);

		ath5k_hw_reg_write(ah, AR5K_REG_SM((next_beacon - 3) << 3,
			AR5K_SLEEP1_NEXT_TIM) |
			AR5K_REG_SM(10, AR5K_SLEEP1_BEACON_TO), AR5K_SLEEP1);

		ath5k_hw_reg_write(ah,
			AR5K_REG_SM(interval, AR5K_SLEEP2_TIM_PER) |
			AR5K_REG_SM(dtim, AR5K_SLEEP2_DTIM_PER), AR5K_SLEEP2);
	}

	return 0;
}

/*
 * Reset beacon timers
 */
void ath5k_hw_reset_beacon(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	/*
	 * Disable beacon timer
	 */
	ath5k_hw_reg_write(ah, 0, AR5K_TIMER0);

	/*
	 * Disable some beacon register values
	 */
	AR5K_REG_DISABLE_BITS(ah, AR5K_STA_ID1,
			AR5K_STA_ID1_DEFAULT_ANTENNA | AR5K_STA_ID1_PCF);
	ath5k_hw_reg_write(ah, AR5K_BEACON_PERIOD, AR5K_BEACON);
}

/*
 * Wait for beacon queue to finish
 * TODO: This function's name is misleading, rename
 */
int ath5k_hw_wait_for_beacon(struct ath5k_hw *ah, unsigned long phys_addr)
{
	unsigned int i;
	int ret;

	AR5K_TRACE;

	/* 5210 doesn't have QCU*/
	if (ah->ah_version == AR5K_AR5210) {
		/*
		 * Wait for beaconn queue to finish by checking
		 * Control Register and Beacon Status Register.
		 */
		for (i = AR5K_TUNE_BEACON_INTERVAL / 2; i > 0; i--) {
			if (!(ath5k_hw_reg_read(ah, AR5K_BSR) & AR5K_BSR_TXQ1F)
					||
			    !(ath5k_hw_reg_read(ah, AR5K_CR) & AR5K_BSR_TXQ1F))
				break;
			udelay(10);
		}

		/* Timeout... */
		if (i <= 0) {
			/*
			 * Re-schedule the beacon queue
			 */
			ath5k_hw_reg_write(ah, phys_addr, AR5K_NOQCU_TXDP1);
			ath5k_hw_reg_write(ah, AR5K_BCR_TQ1V | AR5K_BCR_BDMAE,
					AR5K_BCR);

			return -EIO;
		}
		ret = 0;
	} else {
	/*5211/5212*/
		ret = ath5k_hw_register_timeout(ah,
			AR5K_QUEUE_STATUS(AR5K_TX_QUEUE_ID_BEACON),
			AR5K_QCU_STS_FRMPENDCNT, 0, false);

		if (AR5K_REG_READ_Q(ah, AR5K_QCU_TXE, AR5K_TX_QUEUE_ID_BEACON))
			return -EIO;
	}

	return ret;
}

/*
 * Update mib counters (statistics)
 */
void ath5k_hw_update_mib_counters(struct ath5k_hw *ah,
		struct ath5k_mib_stats *statistics)
{
	AR5K_TRACE;
	/* Read-And-Clear */
	statistics->ackrcv_bad += ath5k_hw_reg_read(ah, AR5K_ACK_FAIL);
	statistics->rts_bad += ath5k_hw_reg_read(ah, AR5K_RTS_FAIL);
	statistics->rts_good += ath5k_hw_reg_read(ah, AR5K_RTS_OK);
	statistics->fcs_bad += ath5k_hw_reg_read(ah, AR5K_FCS_FAIL);
	statistics->beacons += ath5k_hw_reg_read(ah, AR5K_BEACON_CNT);

	/* Reset profile count registers on 5212*/
	if (ah->ah_version == AR5K_AR5212) {
		ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_TX);
		ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_RX);
		ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_RXCLR);
		ath5k_hw_reg_write(ah, 0, AR5K_PROFCNT_CYCLE);
	}
}

/*
 * ACK/CTS Timeouts
 */

/*
 * Set ACK timeout on PCU
 */
int ath5k_hw_set_ack_timeout(struct ath5k_hw *ah, unsigned int timeout)
{
	AR5K_TRACE;
	if (ath5k_hw_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_TIME_OUT_ACK),
			ah->ah_turbo) <= timeout)
		return -EINVAL;

	AR5K_REG_WRITE_BITS(ah, AR5K_TIME_OUT, AR5K_TIME_OUT_ACK,
		ath5k_hw_htoclock(timeout, ah->ah_turbo));

	return 0;
}

/*
 * Read the ACK timeout from PCU
 */
unsigned int ath5k_hw_get_ack_timeout(struct ath5k_hw *ah)
{
	AR5K_TRACE;

	return ath5k_hw_clocktoh(AR5K_REG_MS(ath5k_hw_reg_read(ah,
			AR5K_TIME_OUT), AR5K_TIME_OUT_ACK), ah->ah_turbo);
}

/*
 * Set CTS timeout on PCU
 */
int ath5k_hw_set_cts_timeout(struct ath5k_hw *ah, unsigned int timeout)
{
	AR5K_TRACE;
	if (ath5k_hw_clocktoh(AR5K_REG_MS(0xffffffff, AR5K_TIME_OUT_CTS),
			ah->ah_turbo) <= timeout)
		return -EINVAL;

	AR5K_REG_WRITE_BITS(ah, AR5K_TIME_OUT, AR5K_TIME_OUT_CTS,
			ath5k_hw_htoclock(timeout, ah->ah_turbo));

	return 0;
}

/*
 * Read CTS timeout from PCU
 */
unsigned int ath5k_hw_get_cts_timeout(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	return ath5k_hw_clocktoh(AR5K_REG_MS(ath5k_hw_reg_read(ah,
			AR5K_TIME_OUT), AR5K_TIME_OUT_CTS), ah->ah_turbo);
}

/*
 * Key table (WEP) functions
 */

int ath5k_hw_reset_key(struct ath5k_hw *ah, u16 entry)
{
	unsigned int i;

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	for (i = 0; i < AR5K_KEYCACHE_SIZE; i++)
		ath5k_hw_reg_write(ah, 0, AR5K_KEYTABLE_OFF(entry, i));

	/* Set NULL encryption on non-5210*/
	if (ah->ah_version != AR5K_AR5210)
		ath5k_hw_reg_write(ah, AR5K_KEYTABLE_TYPE_NULL,
				AR5K_KEYTABLE_TYPE(entry));

	return 0;
}

int ath5k_hw_is_key_valid(struct ath5k_hw *ah, u16 entry)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	/* Check the validation flag at the end of the entry */
	return ath5k_hw_reg_read(ah, AR5K_KEYTABLE_MAC1(entry)) &
		AR5K_KEYTABLE_VALID;
}

int ath5k_hw_set_key(struct ath5k_hw *ah, u16 entry,
		const struct ieee80211_key_conf *key, const u8 *mac)
{
	unsigned int i;
	__le32 key_v[5] = {};
	u32 keytype;

	AR5K_TRACE;

	/* key->keylen comes in from mac80211 in bytes */

	if (key->keylen > AR5K_KEYTABLE_SIZE / 8)
		return -EOPNOTSUPP;

	switch (key->keylen) {
	/* WEP 40-bit   = 40-bit  entered key + 24 bit IV = 64-bit */
	case 40 / 8:
		memcpy(&key_v[0], key->key, 5);
		keytype = AR5K_KEYTABLE_TYPE_40;
		break;

	/* WEP 104-bit  = 104-bit entered key + 24-bit IV = 128-bit */
	case 104 / 8:
		memcpy(&key_v[0], &key->key[0], 6);
		memcpy(&key_v[2], &key->key[6], 6);
		memcpy(&key_v[4], &key->key[12], 1);
		keytype = AR5K_KEYTABLE_TYPE_104;
		break;
	/* WEP 128-bit  = 128-bit entered key + 24 bit IV = 152-bit */
	case 128 / 8:
		memcpy(&key_v[0], &key->key[0], 6);
		memcpy(&key_v[2], &key->key[6], 6);
		memcpy(&key_v[4], &key->key[12], 4);
		keytype = AR5K_KEYTABLE_TYPE_128;
		break;

	default:
		return -EINVAL; /* shouldn't happen */
	}

	for (i = 0; i < ARRAY_SIZE(key_v); i++)
		ath5k_hw_reg_write(ah, le32_to_cpu(key_v[i]),
				AR5K_KEYTABLE_OFF(entry, i));

	ath5k_hw_reg_write(ah, keytype, AR5K_KEYTABLE_TYPE(entry));

	return ath5k_hw_set_key_lladdr(ah, entry, mac);
}

int ath5k_hw_set_key_lladdr(struct ath5k_hw *ah, u16 entry, const u8 *mac)
{
	u32 low_id, high_id;

	AR5K_TRACE;
	 /* Invalid entry (key table overflow) */
	AR5K_ASSERT_ENTRY(entry, AR5K_KEYTABLE_SIZE);

	/* MAC may be NULL if it's a broadcast key. In this case no need to
	 * to compute AR5K_LOW_ID and AR5K_HIGH_ID as we already know it. */
	if (unlikely(mac == NULL)) {
		low_id = 0xffffffff;
		high_id = 0xffff | AR5K_KEYTABLE_VALID;
	} else {
		low_id = AR5K_LOW_ID(mac);
		high_id = AR5K_HIGH_ID(mac) | AR5K_KEYTABLE_VALID;
	}

	ath5k_hw_reg_write(ah, low_id, AR5K_KEYTABLE_MAC0(entry));
	ath5k_hw_reg_write(ah, high_id, AR5K_KEYTABLE_MAC1(entry));

	return 0;
}


/********************************************\
Queue Control Unit, DFS Control Unit Functions
\********************************************/

/*
 * Initialize a transmit queue
 */
int ath5k_hw_setup_tx_queue(struct ath5k_hw *ah, enum ath5k_tx_queue queue_type,
		struct ath5k_txq_info *queue_info)
{
	unsigned int queue;
	int ret;

	AR5K_TRACE;

	/*
	 * Get queue by type
	 */
	/*5210 only has 2 queues*/
	if (ah->ah_version == AR5K_AR5210) {
		switch (queue_type) {
		case AR5K_TX_QUEUE_DATA:
			queue = AR5K_TX_QUEUE_ID_NOQCU_DATA;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			queue = AR5K_TX_QUEUE_ID_NOQCU_BEACON;
			break;
		default:
			return -EINVAL;
		}
	} else {
		switch (queue_type) {
		case AR5K_TX_QUEUE_DATA:
			for (queue = AR5K_TX_QUEUE_ID_DATA_MIN;
				ah->ah_txq[queue].tqi_type !=
				AR5K_TX_QUEUE_INACTIVE; queue++) {

				if (queue > AR5K_TX_QUEUE_ID_DATA_MAX)
					return -EINVAL;
			}
			break;
		case AR5K_TX_QUEUE_UAPSD:
			queue = AR5K_TX_QUEUE_ID_UAPSD;
			break;
		case AR5K_TX_QUEUE_BEACON:
			queue = AR5K_TX_QUEUE_ID_BEACON;
			break;
		case AR5K_TX_QUEUE_CAB:
			queue = AR5K_TX_QUEUE_ID_CAB;
			break;
		case AR5K_TX_QUEUE_XR_DATA:
			if (ah->ah_version != AR5K_AR5212)
				AR5K_PRINTF("XR data queues only supported in "
						"5212!\n");
			queue = AR5K_TX_QUEUE_ID_XR_DATA;
			break;
		default:
			return -EINVAL;
		}
	}

	/*
	 * Setup internal queue structure
	 */
	memset(&ah->ah_txq[queue], 0, sizeof(struct ath5k_txq_info));
	ah->ah_txq[queue].tqi_type = queue_type;

	if (queue_info != NULL) {
		queue_info->tqi_type = queue_type;
		ret = ath5k_hw_setup_tx_queueprops(ah, queue, queue_info);
		if (ret)
			return ret;
	}
	/*
	 * We use ah_txq_interrupts to hold a temp value for
	 * the Secondary interrupt mask registers on 5211+
	 * check out ath5k_hw_reset_tx_queue
	 */
	AR5K_Q_ENABLE_BITS(ah->ah_txq_interrupts, queue);

	return queue;
}

/*
 * Setup a transmit queue
 */
int ath5k_hw_setup_tx_queueprops(struct ath5k_hw *ah, int queue,
				const struct ath5k_txq_info *queue_info)
{
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	if (ah->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	memcpy(&ah->ah_txq[queue], queue_info, sizeof(struct ath5k_txq_info));

	/*XXX: Is this supported on 5210 ?*/
	if ((queue_info->tqi_type == AR5K_TX_QUEUE_DATA &&
			((queue_info->tqi_subtype == AR5K_WME_AC_VI) ||
			(queue_info->tqi_subtype == AR5K_WME_AC_VO))) ||
			queue_info->tqi_type == AR5K_TX_QUEUE_UAPSD)
		ah->ah_txq[queue].tqi_flags |= AR5K_TXQ_FLAG_POST_FR_BKOFF_DIS;

	return 0;
}

/*
 * Get properties for a specific transmit queue
 */
int ath5k_hw_get_tx_queueprops(struct ath5k_hw *ah, int queue,
		struct ath5k_txq_info *queue_info)
{
	AR5K_TRACE;
	memcpy(queue_info, &ah->ah_txq[queue], sizeof(struct ath5k_txq_info));
	return 0;
}

/*
 * Set a transmit queue inactive
 */
void ath5k_hw_release_tx_queue(struct ath5k_hw *ah, unsigned int queue)
{
	AR5K_TRACE;
	if (WARN_ON(queue >= ah->ah_capabilities.cap_queues.q_tx_num))
		return;

	/* This queue will be skipped in further operations */
	ah->ah_txq[queue].tqi_type = AR5K_TX_QUEUE_INACTIVE;
	/*For SIMR setup*/
	AR5K_Q_DISABLE_BITS(ah->ah_txq_interrupts, queue);
}

/*
 * Set DFS params for a transmit queue
 */
int ath5k_hw_reset_tx_queue(struct ath5k_hw *ah, unsigned int queue)
{
	u32 cw_min, cw_max, retry_lg, retry_sh;
	struct ath5k_txq_info *tq = &ah->ah_txq[queue];

	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	tq = &ah->ah_txq[queue];

	if (tq->tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return 0;

	if (ah->ah_version == AR5K_AR5210) {
		/* Only handle data queues, others will be ignored */
		if (tq->tqi_type != AR5K_TX_QUEUE_DATA)
			return -EINVAL;

		/* Set Slot time */
		ath5k_hw_reg_write(ah, ah->ah_turbo == true ?
			AR5K_INIT_SLOT_TIME_TURBO : AR5K_INIT_SLOT_TIME,
			AR5K_SLOT_TIME);
		/* Set ACK_CTS timeout */
		ath5k_hw_reg_write(ah, ah->ah_turbo == true ?
			AR5K_INIT_ACK_CTS_TIMEOUT_TURBO :
			AR5K_INIT_ACK_CTS_TIMEOUT, AR5K_SLOT_TIME);
		/* Set Transmit Latency */
		ath5k_hw_reg_write(ah, ah->ah_turbo == true ?
			AR5K_INIT_TRANSMIT_LATENCY_TURBO :
			AR5K_INIT_TRANSMIT_LATENCY, AR5K_USEC_5210);
		/* Set IFS0 */
		if (ah->ah_turbo == true)
			 ath5k_hw_reg_write(ah, ((AR5K_INIT_SIFS_TURBO +
				(ah->ah_aifs + tq->tqi_aifs) *
				AR5K_INIT_SLOT_TIME_TURBO) <<
				AR5K_IFS0_DIFS_S) | AR5K_INIT_SIFS_TURBO,
				AR5K_IFS0);
		else
			ath5k_hw_reg_write(ah, ((AR5K_INIT_SIFS +
				(ah->ah_aifs + tq->tqi_aifs) *
				AR5K_INIT_SLOT_TIME) << AR5K_IFS0_DIFS_S) |
				AR5K_INIT_SIFS, AR5K_IFS0);

		/* Set IFS1 */
		ath5k_hw_reg_write(ah, ah->ah_turbo == true ?
			AR5K_INIT_PROTO_TIME_CNTRL_TURBO :
			AR5K_INIT_PROTO_TIME_CNTRL, AR5K_IFS1);
		/* Set PHY register 0x9844 (??) */
		ath5k_hw_reg_write(ah, ah->ah_turbo == true ?
			(ath5k_hw_reg_read(ah, AR5K_PHY(17)) & ~0x7F) | 0x38 :
			(ath5k_hw_reg_read(ah, AR5K_PHY(17)) & ~0x7F) | 0x1C,
			AR5K_PHY(17));
		/* Set Frame Control Register */
		ath5k_hw_reg_write(ah, ah->ah_turbo == true ?
			(AR5K_PHY_FRAME_CTL_INI | AR5K_PHY_TURBO_MODE |
			AR5K_PHY_TURBO_SHORT | 0x2020) :
			(AR5K_PHY_FRAME_CTL_INI | 0x1020),
			AR5K_PHY_FRAME_CTL_5210);
	}

	/*
	 * Calculate cwmin/max by channel mode
	 */
	cw_min = ah->ah_cw_min = AR5K_TUNE_CWMIN;
	cw_max = ah->ah_cw_max = AR5K_TUNE_CWMAX;
	ah->ah_aifs = AR5K_TUNE_AIFS;
	/*XR is only supported on 5212*/
	if (IS_CHAN_XR(ah->ah_current_channel) &&
			ah->ah_version == AR5K_AR5212) {
		cw_min = ah->ah_cw_min = AR5K_TUNE_CWMIN_XR;
		cw_max = ah->ah_cw_max = AR5K_TUNE_CWMAX_XR;
		ah->ah_aifs = AR5K_TUNE_AIFS_XR;
	/*B mode is not supported on 5210*/
	} else if (IS_CHAN_B(ah->ah_current_channel) &&
			ah->ah_version != AR5K_AR5210) {
		cw_min = ah->ah_cw_min = AR5K_TUNE_CWMIN_11B;
		cw_max = ah->ah_cw_max = AR5K_TUNE_CWMAX_11B;
		ah->ah_aifs = AR5K_TUNE_AIFS_11B;
	}

	cw_min = 1;
	while (cw_min < ah->ah_cw_min)
		cw_min = (cw_min << 1) | 1;

	cw_min = tq->tqi_cw_min < 0 ? (cw_min >> (-tq->tqi_cw_min)) :
		((cw_min << tq->tqi_cw_min) + (1 << tq->tqi_cw_min) - 1);
	cw_max = tq->tqi_cw_max < 0 ? (cw_max >> (-tq->tqi_cw_max)) :
		((cw_max << tq->tqi_cw_max) + (1 << tq->tqi_cw_max) - 1);

	/*
	 * Calculate and set retry limits
	 */
	if (ah->ah_software_retry == true) {
		/* XXX Need to test this */
		retry_lg = ah->ah_limit_tx_retries;
		retry_sh = retry_lg = retry_lg > AR5K_DCU_RETRY_LMT_SH_RETRY ?
			AR5K_DCU_RETRY_LMT_SH_RETRY : retry_lg;
	} else {
		retry_lg = AR5K_INIT_LG_RETRY;
		retry_sh = AR5K_INIT_SH_RETRY;
	}

	/*No QCU/DCU [5210]*/
	if (ah->ah_version == AR5K_AR5210) {
		ath5k_hw_reg_write(ah,
			(cw_min << AR5K_NODCU_RETRY_LMT_CW_MIN_S)
			| AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
				AR5K_NODCU_RETRY_LMT_SLG_RETRY)
			| AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
				AR5K_NODCU_RETRY_LMT_SSH_RETRY)
			| AR5K_REG_SM(retry_lg, AR5K_NODCU_RETRY_LMT_LG_RETRY)
			| AR5K_REG_SM(retry_sh, AR5K_NODCU_RETRY_LMT_SH_RETRY),
			AR5K_NODCU_RETRY_LMT);
	} else {
		/*QCU/DCU [5211+]*/
		ath5k_hw_reg_write(ah,
			AR5K_REG_SM(AR5K_INIT_SLG_RETRY,
				AR5K_DCU_RETRY_LMT_SLG_RETRY) |
			AR5K_REG_SM(AR5K_INIT_SSH_RETRY,
				AR5K_DCU_RETRY_LMT_SSH_RETRY) |
			AR5K_REG_SM(retry_lg, AR5K_DCU_RETRY_LMT_LG_RETRY) |
			AR5K_REG_SM(retry_sh, AR5K_DCU_RETRY_LMT_SH_RETRY),
			AR5K_QUEUE_DFS_RETRY_LIMIT(queue));

	/*===Rest is also for QCU/DCU only [5211+]===*/

		/*
		 * Set initial content window (cw_min/cw_max)
		 * and arbitrated interframe space (aifs)...
		 */
		ath5k_hw_reg_write(ah,
			AR5K_REG_SM(cw_min, AR5K_DCU_LCL_IFS_CW_MIN) |
			AR5K_REG_SM(cw_max, AR5K_DCU_LCL_IFS_CW_MAX) |
			AR5K_REG_SM(ah->ah_aifs + tq->tqi_aifs,
				AR5K_DCU_LCL_IFS_AIFS),
			AR5K_QUEUE_DFS_LOCAL_IFS(queue));

		/*
		 * Set misc registers
		 */
		ath5k_hw_reg_write(ah, AR5K_QCU_MISC_DCU_EARLY,
			AR5K_QUEUE_MISC(queue));

		if (tq->tqi_cbr_period) {
			ath5k_hw_reg_write(ah, AR5K_REG_SM(tq->tqi_cbr_period,
				AR5K_QCU_CBRCFG_INTVAL) |
				AR5K_REG_SM(tq->tqi_cbr_overflow_limit,
				AR5K_QCU_CBRCFG_ORN_THRES),
				AR5K_QUEUE_CBRCFG(queue));
			AR5K_REG_ENABLE_BITS(ah, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_CBR);
			if (tq->tqi_cbr_overflow_limit)
				AR5K_REG_ENABLE_BITS(ah,
					AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_CBR_THRES_ENABLE);
		}

		if (tq->tqi_ready_time)
			ath5k_hw_reg_write(ah, AR5K_REG_SM(tq->tqi_ready_time,
				AR5K_QCU_RDYTIMECFG_INTVAL) |
				AR5K_QCU_RDYTIMECFG_ENABLE,
				AR5K_QUEUE_RDYTIMECFG(queue));

		if (tq->tqi_burst_time) {
			ath5k_hw_reg_write(ah, AR5K_REG_SM(tq->tqi_burst_time,
				AR5K_DCU_CHAN_TIME_DUR) |
				AR5K_DCU_CHAN_TIME_ENABLE,
				AR5K_QUEUE_DFS_CHANNEL_TIME(queue));

			if (tq->tqi_flags & AR5K_TXQ_FLAG_RDYTIME_EXP_POLICY_ENABLE)
				AR5K_REG_ENABLE_BITS(ah,
					AR5K_QUEUE_MISC(queue),
					AR5K_QCU_MISC_TXE);
		}

		if (tq->tqi_flags & AR5K_TXQ_FLAG_BACKOFF_DISABLE)
			ath5k_hw_reg_write(ah, AR5K_DCU_MISC_POST_FR_BKOFF_DIS,
				AR5K_QUEUE_DFS_MISC(queue));

		if (tq->tqi_flags & AR5K_TXQ_FLAG_FRAG_BURST_BACKOFF_ENABLE)
			ath5k_hw_reg_write(ah, AR5K_DCU_MISC_BACKOFF_FRAG,
				AR5K_QUEUE_DFS_MISC(queue));

		/*
		 * Set registers by queue type
		 */
		switch (tq->tqi_type) {
		case AR5K_TX_QUEUE_BEACON:
			AR5K_REG_ENABLE_BITS(ah, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_DBA_GT |
				AR5K_QCU_MISC_CBREXP_BCN |
				AR5K_QCU_MISC_BCN_ENABLE);

			AR5K_REG_ENABLE_BITS(ah, AR5K_QUEUE_DFS_MISC(queue),
				(AR5K_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
				AR5K_DCU_MISC_ARBLOCK_CTL_S) |
				AR5K_DCU_MISC_POST_FR_BKOFF_DIS |
				AR5K_DCU_MISC_BCN_ENABLE);

			ath5k_hw_reg_write(ah, ((AR5K_TUNE_BEACON_INTERVAL -
				(AR5K_TUNE_SW_BEACON_RESP -
				AR5K_TUNE_DMA_BEACON_RESP) -
				AR5K_TUNE_ADDITIONAL_SWBA_BACKOFF) * 1024) |
				AR5K_QCU_RDYTIMECFG_ENABLE,
				AR5K_QUEUE_RDYTIMECFG(queue));
			break;

		case AR5K_TX_QUEUE_CAB:
			AR5K_REG_ENABLE_BITS(ah, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_FRSHED_DBA_GT |
				AR5K_QCU_MISC_CBREXP |
				AR5K_QCU_MISC_CBREXP_BCN);

			AR5K_REG_ENABLE_BITS(ah, AR5K_QUEUE_DFS_MISC(queue),
				(AR5K_DCU_MISC_ARBLOCK_CTL_GLOBAL <<
				AR5K_DCU_MISC_ARBLOCK_CTL_S));
			break;

		case AR5K_TX_QUEUE_UAPSD:
			AR5K_REG_ENABLE_BITS(ah, AR5K_QUEUE_MISC(queue),
				AR5K_QCU_MISC_CBREXP);
			break;

		case AR5K_TX_QUEUE_DATA:
		default:
			break;
		}

		/*
		 * Enable tx queue in the secondary interrupt mask registers
		 */
		ath5k_hw_reg_write(ah, AR5K_REG_SM(ah->ah_txq_interrupts,
			AR5K_SIMR0_QCU_TXOK) |
			AR5K_REG_SM(ah->ah_txq_interrupts,
			AR5K_SIMR0_QCU_TXDESC), AR5K_SIMR0);
		ath5k_hw_reg_write(ah, AR5K_REG_SM(ah->ah_txq_interrupts,
			AR5K_SIMR1_QCU_TXERR), AR5K_SIMR1);
		ath5k_hw_reg_write(ah, AR5K_REG_SM(ah->ah_txq_interrupts,
			AR5K_SIMR2_QCU_TXURN), AR5K_SIMR2);
	}

	return 0;
}

/*
 * Get number of pending frames
 * for a specific queue [5211+]
 */
u32 ath5k_hw_num_tx_pending(struct ath5k_hw *ah, unsigned int queue) {
	AR5K_TRACE;
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (ah->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return false;

	/* XXX: How about AR5K_CFG_TXCNT ? */
	if (ah->ah_version == AR5K_AR5210)
		return false;

	return AR5K_QUEUE_STATUS(queue) & AR5K_QCU_STS_FRMPENDCNT;
}

/*
 * Set slot time
 */
int ath5k_hw_set_slot_time(struct ath5k_hw *ah, unsigned int slot_time)
{
	AR5K_TRACE;
	if (slot_time < AR5K_SLOT_TIME_9 || slot_time > AR5K_SLOT_TIME_MAX)
		return -EINVAL;

	if (ah->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(ah, ath5k_hw_htoclock(slot_time,
				ah->ah_turbo), AR5K_SLOT_TIME);
	else
		ath5k_hw_reg_write(ah, slot_time, AR5K_DCU_GBL_IFS_SLOT);

	return 0;
}

/*
 * Get slot time
 */
unsigned int ath5k_hw_get_slot_time(struct ath5k_hw *ah)
{
	AR5K_TRACE;
	if (ah->ah_version == AR5K_AR5210)
		return ath5k_hw_clocktoh(ath5k_hw_reg_read(ah,
				AR5K_SLOT_TIME) & 0xffff, ah->ah_turbo);
	else
		return ath5k_hw_reg_read(ah, AR5K_DCU_GBL_IFS_SLOT) & 0xffff;
}


/******************************\
 Hardware Descriptor Functions
\******************************/

/*
 * TX Descriptor
 */

/*
 * Initialize the 2-word tx descriptor on 5210/5211
 */
static int
ath5k_hw_setup_2word_tx_desc(struct ath5k_hw *ah, struct ath5k_desc *desc,
	unsigned int pkt_len, unsigned int hdr_len, enum ath5k_pkt_type type,
	unsigned int tx_power, unsigned int tx_rate0, unsigned int tx_tries0,
	unsigned int key_index, unsigned int antenna_mode, unsigned int flags,
	unsigned int rtscts_rate, unsigned int rtscts_duration)
{
	u32 frame_type;
	struct ath5k_hw_2w_tx_desc *tx_desc;
	unsigned int buff_len;

	tx_desc = (struct ath5k_hw_2w_tx_desc *)&desc->ds_ctl0;

	/*
	 * Validate input
	 */
	if (tx_tries0 == 0)
		return -EINVAL;

	/* Clear status descriptor */
	memset(desc->ds_hw, 0, sizeof(struct ath5k_hw_tx_status));

	/* Initialize control descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;

	/* Setup control descriptor */

	/* Verify and set frame length */
	if (pkt_len & ~AR5K_2W_TX_DESC_CTL0_FRAME_LEN)
		return -EINVAL;

	tx_desc->tx_control_0 = pkt_len & AR5K_2W_TX_DESC_CTL0_FRAME_LEN;

	/* Verify and set buffer length */
	buff_len = pkt_len - FCS_LEN;

	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	if(type == AR5K_PKT_TYPE_BEACON)
		buff_len = roundup(buff_len, 4);

	if (buff_len & ~AR5K_2W_TX_DESC_CTL1_BUF_LEN)
		return -EINVAL;

	tx_desc->tx_control_1 = buff_len & AR5K_2W_TX_DESC_CTL1_BUF_LEN;

	/*
	 * Verify and set header length
	 * XXX: I only found that on 5210 code, does it work on 5211 ?
	 */
	if (ah->ah_version == AR5K_AR5210) {
		if (hdr_len & ~AR5K_2W_TX_DESC_CTL0_HEADER_LEN)
			return -EINVAL;
		tx_desc->tx_control_0 |=
			AR5K_REG_SM(hdr_len, AR5K_2W_TX_DESC_CTL0_HEADER_LEN);
	}

	/*Diferences between 5210-5211*/
	if (ah->ah_version == AR5K_AR5210) {
		switch (type) {
		case AR5K_PKT_TYPE_BEACON:
		case AR5K_PKT_TYPE_PROBE_RESP:
			frame_type = AR5K_AR5210_TX_DESC_FRAME_TYPE_NO_DELAY;
		case AR5K_PKT_TYPE_PIFS:
			frame_type = AR5K_AR5210_TX_DESC_FRAME_TYPE_PIFS;
		default:
			frame_type = type /*<< 2 ?*/;
		}

		tx_desc->tx_control_0 |=
			AR5K_REG_SM(frame_type, AR5K_2W_TX_DESC_CTL0_FRAME_TYPE) |
			AR5K_REG_SM(tx_rate0, AR5K_2W_TX_DESC_CTL0_XMIT_RATE);
	} else {
		tx_desc->tx_control_0 |=
			AR5K_REG_SM(tx_rate0, AR5K_2W_TX_DESC_CTL0_XMIT_RATE) |
			AR5K_REG_SM(antenna_mode, AR5K_2W_TX_DESC_CTL0_ANT_MODE_XMIT);
		tx_desc->tx_control_1 |=
			AR5K_REG_SM(type, AR5K_2W_TX_DESC_CTL1_FRAME_TYPE);
	}
#define _TX_FLAGS(_c, _flag)						\
	if (flags & AR5K_TXDESC_##_flag)				\
		tx_desc->tx_control_##_c |=				\
			AR5K_2W_TX_DESC_CTL##_c##_##_flag

	_TX_FLAGS(0, CLRDMASK);
	_TX_FLAGS(0, VEOL);
	_TX_FLAGS(0, INTREQ);
	_TX_FLAGS(0, RTSENA);
	_TX_FLAGS(1, NOACK);

#undef _TX_FLAGS

	/*
	 * WEP crap
	 */
	if (key_index != AR5K_TXKEYIX_INVALID) {
		tx_desc->tx_control_0 |=
			AR5K_2W_TX_DESC_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |=
			AR5K_REG_SM(key_index,
			AR5K_2W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX);
	}

	/*
	 * RTS/CTS Duration [5210 ?]
	 */
	if ((ah->ah_version == AR5K_AR5210) &&
			(flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)))
		tx_desc->tx_control_1 |= rtscts_duration &
				AR5K_2W_TX_DESC_CTL1_RTS_DURATION;

	return 0;
}

/*
 * Initialize the 4-word tx descriptor on 5212
 */
static int ath5k_hw_setup_4word_tx_desc(struct ath5k_hw *ah,
	struct ath5k_desc *desc, unsigned int pkt_len, unsigned int hdr_len,
	enum ath5k_pkt_type type, unsigned int tx_power, unsigned int tx_rate0,
	unsigned int tx_tries0, unsigned int key_index,
	unsigned int antenna_mode, unsigned int flags, unsigned int rtscts_rate,
	unsigned int rtscts_duration)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;
	struct ath5k_hw_tx_status *tx_status;
	unsigned int buff_len;

	AR5K_TRACE;
	tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status *)&desc->ds_hw[2];

	/*
	 * Validate input
	 */
	if (tx_tries0 == 0)
		return -EINVAL;

	/* Clear status descriptor */
	memset(tx_status, 0, sizeof(struct ath5k_hw_tx_status));

	/* Initialize control descriptor */
	tx_desc->tx_control_0 = 0;
	tx_desc->tx_control_1 = 0;
	tx_desc->tx_control_2 = 0;
	tx_desc->tx_control_3 = 0;

	/* Setup control descriptor */

	/* Verify and set frame length */
	if (pkt_len & ~AR5K_4W_TX_DESC_CTL0_FRAME_LEN)
		return -EINVAL;

	tx_desc->tx_control_0 = pkt_len & AR5K_4W_TX_DESC_CTL0_FRAME_LEN;

	/* Verify and set buffer length */
	buff_len = pkt_len - FCS_LEN;

	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	if(type == AR5K_PKT_TYPE_BEACON)
		buff_len = roundup(buff_len, 4);

	if (buff_len & ~AR5K_4W_TX_DESC_CTL1_BUF_LEN)
		return -EINVAL;

	tx_desc->tx_control_1 = buff_len & AR5K_4W_TX_DESC_CTL1_BUF_LEN;

	tx_desc->tx_control_0 |=
		AR5K_REG_SM(tx_power, AR5K_4W_TX_DESC_CTL0_XMIT_POWER) |
		AR5K_REG_SM(antenna_mode, AR5K_4W_TX_DESC_CTL0_ANT_MODE_XMIT);
	tx_desc->tx_control_1 |= AR5K_REG_SM(type,
					AR5K_4W_TX_DESC_CTL1_FRAME_TYPE);
	tx_desc->tx_control_2 = AR5K_REG_SM(tx_tries0 + AR5K_TUNE_HWTXTRIES,
					AR5K_4W_TX_DESC_CTL2_XMIT_TRIES0);
	tx_desc->tx_control_3 = tx_rate0 & AR5K_4W_TX_DESC_CTL3_XMIT_RATE0;

#define _TX_FLAGS(_c, _flag)			\
	if (flags & AR5K_TXDESC_##_flag)	\
		tx_desc->tx_control_##_c |=	\
			AR5K_4W_TX_DESC_CTL##_c##_##_flag

	_TX_FLAGS(0, CLRDMASK);
	_TX_FLAGS(0, VEOL);
	_TX_FLAGS(0, INTREQ);
	_TX_FLAGS(0, RTSENA);
	_TX_FLAGS(0, CTSENA);
	_TX_FLAGS(1, NOACK);

#undef _TX_FLAGS

	/*
	 * WEP crap
	 */
	if (key_index != AR5K_TXKEYIX_INVALID) {
		tx_desc->tx_control_0 |= AR5K_4W_TX_DESC_CTL0_ENCRYPT_KEY_VALID;
		tx_desc->tx_control_1 |= AR5K_REG_SM(key_index,
				AR5K_4W_TX_DESC_CTL1_ENCRYPT_KEY_INDEX);
	}

	/*
	 * RTS/CTS
	 */
	if (flags & (AR5K_TXDESC_RTSENA | AR5K_TXDESC_CTSENA)) {
		if ((flags & AR5K_TXDESC_RTSENA) &&
				(flags & AR5K_TXDESC_CTSENA))
			return -EINVAL;
		tx_desc->tx_control_2 |= rtscts_duration &
				AR5K_4W_TX_DESC_CTL2_RTS_DURATION;
		tx_desc->tx_control_3 |= AR5K_REG_SM(rtscts_rate,
				AR5K_4W_TX_DESC_CTL3_RTS_CTS_RATE);
	}

	return 0;
}

/*
 * Initialize a 4-word multirate tx descriptor on 5212
 */
static bool
ath5k_hw_setup_xr_tx_desc(struct ath5k_hw *ah, struct ath5k_desc *desc,
	unsigned int tx_rate1, u_int tx_tries1, u_int tx_rate2, u_int tx_tries2,
	unsigned int tx_rate3, u_int tx_tries3)
{
	struct ath5k_hw_4w_tx_desc *tx_desc;

	if (ah->ah_version == AR5K_AR5212) {
		tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;

#define _XTX_TRIES(_n)							\
	if (tx_tries##_n) {						\
		tx_desc->tx_control_2 |=				\
		    AR5K_REG_SM(tx_tries##_n,				\
		    AR5K_4W_TX_DESC_CTL2_XMIT_TRIES##_n);		\
		tx_desc->tx_control_3 |=				\
		    AR5K_REG_SM(tx_rate##_n,				\
		    AR5K_4W_TX_DESC_CTL3_XMIT_RATE##_n);		\
	}

		_XTX_TRIES(1);
		_XTX_TRIES(2);
		_XTX_TRIES(3);

#undef _XTX_TRIES

		return true;
	}

	return false;
}

/*
 * Proccess the tx status descriptor on 5210/5211
 */
static int ath5k_hw_proc_2word_tx_status(struct ath5k_hw *ah,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_tx_status *tx_status;
	struct ath5k_hw_2w_tx_desc *tx_desc;

	tx_desc = (struct ath5k_hw_2w_tx_desc *)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status *)&desc->ds_hw[0];

	/* No frame has been send or error */
	if (unlikely((tx_status->tx_status_1 & AR5K_DESC_TX_STATUS1_DONE) == 0))
		return -EINPROGRESS;

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp = AR5K_REG_MS(tx_status->tx_status_0,
		AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry = AR5K_REG_MS(tx_status->tx_status_0,
		AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT);
	desc->ds_us.tx.ts_longretry = AR5K_REG_MS(tx_status->tx_status_0,
		AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT);
	/*TODO: desc->ds_us.tx.ts_virtcol + test*/
	desc->ds_us.tx.ts_seqnum = AR5K_REG_MS(tx_status->tx_status_1,
		AR5K_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi = AR5K_REG_MS(tx_status->tx_status_1,
		AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = 1;
	desc->ds_us.tx.ts_status = 0;
	desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_0,
		AR5K_2W_TX_DESC_CTL0_XMIT_RATE);

	if ((tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK) == 0){
		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return 0;
}

/*
 * Proccess a tx descriptor on 5212
 */
static int ath5k_hw_proc_4word_tx_status(struct ath5k_hw *ah,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_tx_status *tx_status;
	struct ath5k_hw_4w_tx_desc *tx_desc;

	AR5K_TRACE;
	tx_desc = (struct ath5k_hw_4w_tx_desc *)&desc->ds_ctl0;
	tx_status = (struct ath5k_hw_tx_status *)&desc->ds_hw[2];

	/* No frame has been send or error */
	if (unlikely((tx_status->tx_status_1 & AR5K_DESC_TX_STATUS1_DONE) == 0))
		return -EINPROGRESS;

	/*
	 * Get descriptor status
	 */
	desc->ds_us.tx.ts_tstamp = AR5K_REG_MS(tx_status->tx_status_0,
		AR5K_DESC_TX_STATUS0_SEND_TIMESTAMP);
	desc->ds_us.tx.ts_shortretry = AR5K_REG_MS(tx_status->tx_status_0,
		AR5K_DESC_TX_STATUS0_SHORT_RETRY_COUNT);
	desc->ds_us.tx.ts_longretry = AR5K_REG_MS(tx_status->tx_status_0,
		AR5K_DESC_TX_STATUS0_LONG_RETRY_COUNT);
	desc->ds_us.tx.ts_seqnum = AR5K_REG_MS(tx_status->tx_status_1,
		AR5K_DESC_TX_STATUS1_SEQ_NUM);
	desc->ds_us.tx.ts_rssi = AR5K_REG_MS(tx_status->tx_status_1,
		AR5K_DESC_TX_STATUS1_ACK_SIG_STRENGTH);
	desc->ds_us.tx.ts_antenna = (tx_status->tx_status_1 &
		AR5K_DESC_TX_STATUS1_XMIT_ANTENNA) ? 2 : 1;
	desc->ds_us.tx.ts_status = 0;

	switch (AR5K_REG_MS(tx_status->tx_status_1,
			AR5K_DESC_TX_STATUS1_FINAL_TS_INDEX)) {
	case 0:
		desc->ds_us.tx.ts_rate = tx_desc->tx_control_3 &
			AR5K_4W_TX_DESC_CTL3_XMIT_RATE0;
		break;
	case 1:
		desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_3,
			AR5K_4W_TX_DESC_CTL3_XMIT_RATE1);
		desc->ds_us.tx.ts_longretry +=AR5K_REG_MS(tx_desc->tx_control_2,
			AR5K_4W_TX_DESC_CTL2_XMIT_TRIES1);
		break;
	case 2:
		desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_3,
			AR5K_4W_TX_DESC_CTL3_XMIT_RATE2);
		desc->ds_us.tx.ts_longretry +=AR5K_REG_MS(tx_desc->tx_control_2,
			AR5K_4W_TX_DESC_CTL2_XMIT_TRIES2);
		break;
	case 3:
		desc->ds_us.tx.ts_rate = AR5K_REG_MS(tx_desc->tx_control_3,
			AR5K_4W_TX_DESC_CTL3_XMIT_RATE3);
		desc->ds_us.tx.ts_longretry +=AR5K_REG_MS(tx_desc->tx_control_2,
			AR5K_4W_TX_DESC_CTL2_XMIT_TRIES3);
		break;
	}

	if ((tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK) == 0){
		if (tx_status->tx_status_0 &
				AR5K_DESC_TX_STATUS0_EXCESSIVE_RETRIES)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_XRETRY;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FIFO_UNDERRUN)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FIFO;

		if (tx_status->tx_status_0 & AR5K_DESC_TX_STATUS0_FILTERED)
			desc->ds_us.tx.ts_status |= AR5K_TXERR_FILT;
	}

	return 0;
}

/*
 * RX Descriptor
 */

/*
 * Initialize an rx descriptor
 */
int ath5k_hw_setup_rx_desc(struct ath5k_hw *ah, struct ath5k_desc *desc,
			u32 size, unsigned int flags)
{
	struct ath5k_rx_desc *rx_desc;

	AR5K_TRACE;
	rx_desc = (struct ath5k_rx_desc *)&desc->ds_ctl0;

	/*
	 *Clear ds_hw
	 * If we don't clean the status descriptor,
	 * while scanning we get too many results,
	 * most of them virtual, after some secs
	 * of scanning system hangs. M.F.
	*/
	memset(desc->ds_hw, 0, sizeof(desc->ds_hw));

	/*Initialize rx descriptor*/
	rx_desc->rx_control_0 = 0;
	rx_desc->rx_control_1 = 0;

	/* Setup descriptor */
	rx_desc->rx_control_1 = size & AR5K_DESC_RX_CTL1_BUF_LEN;
	if (unlikely(rx_desc->rx_control_1 != size))
		return -EINVAL;

	if (flags & AR5K_RXDESC_INTREQ)
		rx_desc->rx_control_1 |= AR5K_DESC_RX_CTL1_INTREQ;

	return 0;
}

/*
 * Proccess the rx status descriptor on 5210/5211
 */
static int ath5k_hw_proc_old_rx_status(struct ath5k_hw *ah,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_old_rx_status *rx_status;

	rx_status = (struct ath5k_hw_old_rx_status *)&desc->ds_hw[0];

	/* No frame received / not ready */
	if (unlikely((rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_DONE)
				== 0))
		return -EINPROGRESS;

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
		AR5K_OLD_RX_DESC_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi = AR5K_REG_MS(rx_status->rx_status_0,
		AR5K_OLD_RX_DESC_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate = AR5K_REG_MS(rx_status->rx_status_0,
		AR5K_OLD_RX_DESC_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
		AR5K_OLD_RX_DESC_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
		AR5K_OLD_RX_DESC_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp = AR5K_REG_MS(rx_status->rx_status_1,
		AR5K_OLD_RX_DESC_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX_VALID)
		desc->ds_us.rx.rs_keyix = AR5K_REG_MS(rx_status->rx_status_1,
			AR5K_OLD_RX_DESC_STATUS1_KEY_INDEX);
	else
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_FRAME_RECEIVE_OK)
			== 0) {
		if (rx_status->rx_status_1 & AR5K_OLD_RX_DESC_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_FIFO_OVERRUN)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_FIFO;

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
				AR5K_REG_MS(rx_status->rx_status_1,
					AR5K_OLD_RX_DESC_STATUS1_PHY_ERROR);
		}

		if (rx_status->rx_status_1 &
				AR5K_OLD_RX_DESC_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;
	}

	return 0;
}

/*
 * Proccess the rx status descriptor on 5212
 */
static int ath5k_hw_proc_new_rx_status(struct ath5k_hw *ah,
		struct ath5k_desc *desc)
{
	struct ath5k_hw_new_rx_status *rx_status;
	struct ath5k_hw_rx_error *rx_err;

	AR5K_TRACE;
	rx_status = (struct ath5k_hw_new_rx_status *)&desc->ds_hw[0];

	/* Overlay on error */
	rx_err = (struct ath5k_hw_rx_error *)&desc->ds_hw[0];

	/* No frame received / not ready */
	if (unlikely((rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_DONE)
				== 0))
		return -EINPROGRESS;

	/*
	 * Frame receive status
	 */
	desc->ds_us.rx.rs_datalen = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_DATA_LEN;
	desc->ds_us.rx.rs_rssi = AR5K_REG_MS(rx_status->rx_status_0,
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_SIGNAL);
	desc->ds_us.rx.rs_rate = AR5K_REG_MS(rx_status->rx_status_0,
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_RATE);
	desc->ds_us.rx.rs_antenna = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_RECEIVE_ANTENNA;
	desc->ds_us.rx.rs_more = rx_status->rx_status_0 &
		AR5K_NEW_RX_DESC_STATUS0_MORE;
	desc->ds_us.rx.rs_tstamp = AR5K_REG_MS(rx_status->rx_status_1,
		AR5K_NEW_RX_DESC_STATUS1_RECEIVE_TIMESTAMP);
	desc->ds_us.rx.rs_status = 0;

	/*
	 * Key table status
	 */
	if (rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX_VALID)
		desc->ds_us.rx.rs_keyix = AR5K_REG_MS(rx_status->rx_status_1,
				AR5K_NEW_RX_DESC_STATUS1_KEY_INDEX);
	else
		desc->ds_us.rx.rs_keyix = AR5K_RXKEYIX_INVALID;

	/*
	 * Receive/descriptor errors
	 */
	if ((rx_status->rx_status_1 &
			AR5K_NEW_RX_DESC_STATUS1_FRAME_RECEIVE_OK) == 0) {
		if (rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_CRC;

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_PHY_ERROR) {
			desc->ds_us.rx.rs_status |= AR5K_RXERR_PHY;
			desc->ds_us.rx.rs_phyerr =
				AR5K_REG_MS(rx_err->rx_error_1,
					AR5K_RX_DESC_ERROR1_PHY_ERROR_CODE);
		}

		if (rx_status->rx_status_1 &
				AR5K_NEW_RX_DESC_STATUS1_DECRYPT_CRC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_DECRYPT;

		if (rx_status->rx_status_1 & AR5K_NEW_RX_DESC_STATUS1_MIC_ERROR)
			desc->ds_us.rx.rs_status |= AR5K_RXERR_MIC;
	}

	return 0;
}


/****************\
  GPIO Functions
\****************/

/*
 * Set led state
 */
void ath5k_hw_set_ledstate(struct ath5k_hw *ah, unsigned int state)
{
	u32 led;
	/*5210 has different led mode handling*/
	u32 led_5210;

	AR5K_TRACE;

	/*Reset led status*/
	if (ah->ah_version != AR5K_AR5210)
		AR5K_REG_DISABLE_BITS(ah, AR5K_PCICFG,
			AR5K_PCICFG_LEDMODE |  AR5K_PCICFG_LED);
	else
		AR5K_REG_DISABLE_BITS(ah, AR5K_PCICFG, AR5K_PCICFG_LED);

	/*
	 * Some blinking values, define at your wish
	 */
	switch (state) {
	case AR5K_LED_SCAN:
	case AR5K_LED_AUTH:
		led = AR5K_PCICFG_LEDMODE_PROP | AR5K_PCICFG_LED_PEND;
		led_5210 = AR5K_PCICFG_LED_PEND | AR5K_PCICFG_LED_BCTL;
		break;

	case AR5K_LED_INIT:
		led = AR5K_PCICFG_LEDMODE_PROP | AR5K_PCICFG_LED_NONE;
		led_5210 = AR5K_PCICFG_LED_PEND;
		break;

	case AR5K_LED_ASSOC:
	case AR5K_LED_RUN:
		led = AR5K_PCICFG_LEDMODE_PROP | AR5K_PCICFG_LED_ASSOC;
		led_5210 = AR5K_PCICFG_LED_ASSOC;
		break;

	default:
		led = AR5K_PCICFG_LEDMODE_PROM | AR5K_PCICFG_LED_NONE;
		led_5210 = AR5K_PCICFG_LED_PEND;
		break;
	}

	/*Write new status to the register*/
	if (ah->ah_version != AR5K_AR5210)
		AR5K_REG_ENABLE_BITS(ah, AR5K_PCICFG, led);
	else
		AR5K_REG_ENABLE_BITS(ah, AR5K_PCICFG, led_5210);
}

/*
 * Set GPIO outputs
 */
int ath5k_hw_set_gpio_output(struct ath5k_hw *ah, u32 gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return -EINVAL;

	ath5k_hw_reg_write(ah, (ath5k_hw_reg_read(ah, AR5K_GPIOCR) &~
		AR5K_GPIOCR_OUT(gpio)) | AR5K_GPIOCR_OUT(gpio), AR5K_GPIOCR);

	return 0;
}

/*
 * Set GPIO inputs
 */
int ath5k_hw_set_gpio_input(struct ath5k_hw *ah, u32 gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return -EINVAL;

	ath5k_hw_reg_write(ah, (ath5k_hw_reg_read(ah, AR5K_GPIOCR) &~
		AR5K_GPIOCR_OUT(gpio)) | AR5K_GPIOCR_IN(gpio), AR5K_GPIOCR);

	return 0;
}

/*
 * Get GPIO state
 */
u32 ath5k_hw_get_gpio(struct ath5k_hw *ah, u32 gpio)
{
	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return 0xffffffff;

	/* GPIO input magic */
	return ((ath5k_hw_reg_read(ah, AR5K_GPIODI) & AR5K_GPIODI_M) >> gpio) &
		0x1;
}

/*
 * Set GPIO state
 */
int ath5k_hw_set_gpio(struct ath5k_hw *ah, u32 gpio, u32 val)
{
	u32 data;
	AR5K_TRACE;

	if (gpio > AR5K_NUM_GPIO)
		return -EINVAL;

	/* GPIO output magic */
	data = ath5k_hw_reg_read(ah, AR5K_GPIODO);

	data &= ~(1 << gpio);
	data |= (val & 1) << gpio;

	ath5k_hw_reg_write(ah, data, AR5K_GPIODO);

	return 0;
}

/*
 * Initialize the GPIO interrupt (RFKill switch)
 */
void ath5k_hw_set_gpio_intr(struct ath5k_hw *ah, unsigned int gpio,
		u32 interrupt_level)
{
	u32 data;

	AR5K_TRACE;
	if (gpio > AR5K_NUM_GPIO)
		return;

	/*
	 * Set the GPIO interrupt
	 */
	data = (ath5k_hw_reg_read(ah, AR5K_GPIOCR) &
		~(AR5K_GPIOCR_INT_SEL(gpio) | AR5K_GPIOCR_INT_SELH |
		AR5K_GPIOCR_INT_ENA | AR5K_GPIOCR_OUT(gpio))) |
		(AR5K_GPIOCR_INT_SEL(gpio) | AR5K_GPIOCR_INT_ENA);

	ath5k_hw_reg_write(ah, interrupt_level ? data :
		(data | AR5K_GPIOCR_INT_SELH), AR5K_GPIOCR);

	ah->ah_imr |= AR5K_IMR_GPIO;

	/* Enable GPIO interrupts */
	AR5K_REG_ENABLE_BITS(ah, AR5K_PIMR, AR5K_IMR_GPIO);
}


/*********************************\
 Regulatory Domain/Channels Setup
\*********************************/

u16 ath5k_get_regdomain(struct ath5k_hw *ah)
{
	u16 regdomain;
	enum ath5k_regdom ieee_regdomain;
#ifdef COUNTRYCODE
	u16 code;
#endif

	ath5k_eeprom_regulation_domain(ah, false, &ieee_regdomain);
	ah->ah_capabilities.cap_regdomain.reg_hw = ieee_regdomain;

#ifdef COUNTRYCODE
	/*
	 * Get the regulation domain by country code. This will ignore
	 * the settings found in the EEPROM.
	 */
	code = ieee80211_name2countrycode(COUNTRYCODE);
	ieee_regdomain = ieee80211_countrycode2regdomain(code);
#endif

	regdomain = ath5k_regdom_from_ieee(ieee_regdomain);
	ah->ah_capabilities.cap_regdomain.reg_current = regdomain;

	return regdomain;
}



/****************\
  Misc functions
\****************/

void /*O.K.*/
ath5k_hw_dump_state(struct ath5k_hw *ah)
{
#ifdef AR5K_DEBUG
#define AR5K_PRINT_REGISTER(_x)						\
	AR5K_PRINTF("(%s: %08x) ", #_x, ath5k_hw_reg_read(ah, AR5K_##_x));

	AR5K_PRINT("MAC registers:\n");
	AR5K_PRINT_REGISTER(CR);
	AR5K_PRINT_REGISTER(CFG);
	AR5K_PRINT_REGISTER(IER);
	AR5K_PRINT_REGISTER(TXCFG);
	AR5K_PRINT_REGISTER(RXCFG);
	AR5K_PRINT_REGISTER(MIBC);
	AR5K_PRINT_REGISTER(TOPS);
	AR5K_PRINT_REGISTER(RXNOFRM);
	AR5K_PRINT_REGISTER(RPGTO);
	AR5K_PRINT_REGISTER(RFCNT);
	AR5K_PRINT_REGISTER(MISC);
	AR5K_PRINT_REGISTER(PISR);
	AR5K_PRINT_REGISTER(SISR0);
	AR5K_PRINT_REGISTER(SISR1);
	AR5K_PRINT_REGISTER(SISR3);
	AR5K_PRINT_REGISTER(SISR4);
	AR5K_PRINT_REGISTER(DCM_ADDR);
	AR5K_PRINT_REGISTER(DCM_DATA);
	AR5K_PRINT_REGISTER(DCCFG);
	AR5K_PRINT_REGISTER(CCFG);
	AR5K_PRINT_REGISTER(CCFG_CUP);
	AR5K_PRINT_REGISTER(CPC0);
	AR5K_PRINT_REGISTER(CPC1);
	AR5K_PRINT_REGISTER(CPC2);
	AR5K_PRINT_REGISTER(CPCORN);
	AR5K_PRINT_REGISTER(QCU_TXE);
	AR5K_PRINT_REGISTER(QCU_TXD);
	AR5K_PRINT_REGISTER(DCU_GBL_IFS_SIFS);
	AR5K_PRINT_REGISTER(DCU_GBL_IFS_SLOT);
	AR5K_PRINT_REGISTER(DCU_FP);
	AR5K_PRINT_REGISTER(DCU_TXP);
	AR5K_PRINT_REGISTER(DCU_TX_FILTER);
	AR5K_PRINT_REGISTER(INTPEND);
	AR5K_PRINT_REGISTER(PCICFG);
	AR5K_PRINT_REGISTER(GPIOCR);
	AR5K_PRINT_REGISTER(GPIODO);
	AR5K_PRINT_REGISTER(SREV);
	AR5K_PRINT_REGISTER(EEPROM_BASE);
	AR5K_PRINT_REGISTER(EEPROM_DATA);
	AR5K_PRINT_REGISTER(EEPROM_CMD);
	AR5K_PRINT_REGISTER(EEPROM_CFG);
	AR5K_PRINT_REGISTER(PCU_MIN);
	AR5K_PRINT_REGISTER(STA_ID0);
	AR5K_PRINT_REGISTER(STA_ID1);
	AR5K_PRINT_REGISTER(BSS_ID0);
	AR5K_PRINT_REGISTER(SLOT_TIME);
	AR5K_PRINT_REGISTER(TIME_OUT);
	AR5K_PRINT_REGISTER(RSSI_THR);
	AR5K_PRINT_REGISTER(BEACON);
	AR5K_PRINT_REGISTER(CFP_PERIOD);
	AR5K_PRINT_REGISTER(TIMER0);
	AR5K_PRINT_REGISTER(TIMER2);
	AR5K_PRINT_REGISTER(TIMER3);
	AR5K_PRINT_REGISTER(CFP_DUR);
	AR5K_PRINT_REGISTER(MCAST_FILTER0);
	AR5K_PRINT_REGISTER(MCAST_FILTER1);
	AR5K_PRINT_REGISTER(DIAG_SW);
	AR5K_PRINT_REGISTER(TSF_U32);
	AR5K_PRINT_REGISTER(ADDAC_TEST);
	AR5K_PRINT_REGISTER(DEFAULT_ANTENNA);
	AR5K_PRINT_REGISTER(LAST_TSTP);
	AR5K_PRINT_REGISTER(NAV);
	AR5K_PRINT_REGISTER(RTS_OK);
	AR5K_PRINT_REGISTER(ACK_FAIL);
	AR5K_PRINT_REGISTER(FCS_FAIL);
	AR5K_PRINT_REGISTER(BEACON_CNT);
	AR5K_PRINT_REGISTER(TSF_PARM);
	AR5K_PRINT("\n");

	AR5K_PRINT("PHY registers:\n");
	AR5K_PRINT_REGISTER(PHY_TURBO);
	AR5K_PRINT_REGISTER(PHY_AGC);
	AR5K_PRINT_REGISTER(PHY_TIMING_3);
	AR5K_PRINT_REGISTER(PHY_CHIP_ID);
	AR5K_PRINT_REGISTER(PHY_AGCCTL);
	AR5K_PRINT_REGISTER(PHY_NF);
	AR5K_PRINT_REGISTER(PHY_SCR);
	AR5K_PRINT_REGISTER(PHY_SLMT);
	AR5K_PRINT_REGISTER(PHY_SCAL);
	AR5K_PRINT_REGISTER(PHY_RX_DELAY);
	AR5K_PRINT_REGISTER(PHY_IQ);
	AR5K_PRINT_REGISTER(PHY_PAPD_PROBE);
	AR5K_PRINT_REGISTER(PHY_TXPOWER_RATE1);
	AR5K_PRINT_REGISTER(PHY_TXPOWER_RATE2);
	AR5K_PRINT_REGISTER(PHY_RADAR);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_0);
	AR5K_PRINT_REGISTER(PHY_ANT_SWITCH_TABLE_1);
	AR5K_PRINT("\n");
#endif
}

int ath5k_hw_get_capability(struct ath5k_hw *ah,
		enum ath5k_capability_type cap_type,
		u32 capability, u32 *result)
{
	AR5K_TRACE;

	switch (cap_type) {
	case AR5K_CAP_NUM_TXQUEUES:
		if (result) {
			if (ah->ah_version == AR5K_AR5210)
				*result = AR5K_NUM_TX_QUEUES_NOQCU;
			else
				*result = AR5K_NUM_TX_QUEUES;
			goto yes;
		}
	case AR5K_CAP_VEOL:
		goto yes;
	case AR5K_CAP_COMPRESSION:
		if (ah->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	case AR5K_CAP_BURST:
		goto yes;
	case AR5K_CAP_TPC:
		goto yes;
	case AR5K_CAP_BSSIDMASK:
		if (ah->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	case AR5K_CAP_XR:
		if (ah->ah_version == AR5K_AR5212)
			goto yes;
		else
			goto no;
	default:
		goto no;
	}

no:
	return -EINVAL;
yes:
	return 0;
}

static int ath5k_hw_enable_pspoll(struct ath5k_hw *ah, u8 *bssid,
		u16 assoc_id)
{
	AR5K_TRACE;

	if (ah->ah_version == AR5K_AR5210) {
		AR5K_REG_DISABLE_BITS(ah, AR5K_STA_ID1,
			AR5K_STA_ID1_NO_PSPOLL | AR5K_STA_ID1_DEFAULT_ANTENNA);
		return 0;
	}

	return -EIO;
}

static int ath5k_hw_disable_pspoll(struct ath5k_hw *ah)
{
	AR5K_TRACE;

	if (ah->ah_version == AR5K_AR5210) {
		AR5K_REG_ENABLE_BITS(ah, AR5K_STA_ID1,
			AR5K_STA_ID1_NO_PSPOLL | AR5K_STA_ID1_DEFAULT_ANTENNA);
		return 0;
	}

	return -EIO;
}
