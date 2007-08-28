/*
 * Copyright (c) 2007 The MadWiFi Team <www.madwifi.org>
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

#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/etherdevice.h>
#include <linux/pci.h>

#include "ath5k.h"


/******************\
* HELPER FUNCTIONS *
\******************/

/*
 * Functions to translate local structs to mac80211 structs
 * we should get rid of as much of those as we can.
 */

static struct ieee80211_channel*
ath5k_ar5k_to_mac80211_channel(AR5K_CHANNEL *ar5k_channel)
{
	struct ieee80211_channel *mac80211_channel;

	mac80211_channel = kmalloc(sizeof(struct ieee80211_channel), GFP_KERNEL);
	memset(mac80211_channel, 0, sizeof(struct ieee80211_channel));

	mac80211_channel->chan = 
		ath5k_mhz2ieee(ar5k_channel->freq,
				ar5k_channel->channel_flags);
	mac80211_channel->freq = ar5k_channel->freq;
	mac80211_channel->val = ar5k_channel->channel_flags;
	/* TODO: Use new regdomain code for these */
	/* mick: I did a little search on mac80211 code and it
	 * seems these are not used yet, we set them 0 for now. */
	mac80211_channel->flag = 0; /* Set by the regdomain code */
	mac80211_channel->power_level = 0; /* regulatory limit in dBm */
	mac80211_channel->antenna_max = 0; /* ??? */

	return mac80211_channel;
}

static AR5K_CHANNEL*
ath5k_mac80211_to_ar5k_channel(struct ieee80211_channel *mac80211_channel)
{
	AR5K_CHANNEL *ar5k_channel;
	
	ar5k_channel = kmalloc(sizeof(AR5K_CHANNEL), GFP_KERNEL);
	memset(ar5k_channel, 0, sizeof(AR5K_CHANNEL));

	ar5k_channel->freq = mac80211_channel->freq;
	/* XXX: Is this ok ? */
	if (!mac80211_channel->val) {
		ar5k_channel->channel_flags = mac80211_channel->val;
	} else {
	/* Try to set the flags based on freq */
		printk(KERN_ALERT "No channel flags passed FIX it !\n");
		ar5k_channel->channel_flags = 0;
		if ((ar5k_channel->freq >= 2412) && /* chan 1 */
				(ar5k_channel->freq < 2512)) {	/* chan 26 */
			ar5k_channel->channel_flags |= 
				CHANNEL_2GHZ | CHANNEL_CCK | CHANNEL_OFDM;
		} else if ((ar5k_channel->freq >= 5150) &&
				(ar5k_channel->freq <= 5825)) {
			ar5k_channel->channel_flags |= 
				CHANNEL_2GHZ | CHANNEL_OFDM;
		}
	}

	return ar5k_channel;
}

static struct ieee80211_tx_queue_params*
ath5k_ar5k_to_mac80211_queue_params(AR5K_TXQ_INFO *ar5k_queueparams)
{
	struct ieee80211_tx_queue_params *mac80211_queue_params;

	mac80211_queue_params = kmalloc(sizeof(struct ieee80211_tx_queue_params),
					GFP_KERNEL);
	memset(mac80211_queue_params, 0, sizeof(struct ieee80211_tx_queue_params));

	mac80211_queue_params->aifs = ar5k_queueparams->tqi_aifs;
	mac80211_queue_params->cw_min = ar5k_queueparams->tqi_cw_min;
	mac80211_queue_params->cw_max = ar5k_queueparams->tqi_cw_max;
	mac80211_queue_params->burst_time = ar5k_queueparams->tqi_burst_time;

	return mac80211_queue_params;
}

/* From dadwifi */
static struct {
	u_int   ar5k_mode;	/* ar5k phy mode */
	int     mac80211_mode;	/* mac80211 hw mode */
} ar5k_mode_map[] = {
	{ AR5K_MODE_11A,	MODE_IEEE80211A     },
	{ AR5K_MODE_11B,	MODE_IEEE80211B     },
	{ AR5K_MODE_11G,	MODE_IEEE80211G     }, /* Dynamic b/g */
	{ AR5K_MODE_TURBO,	MODE_ATHEROS_TURBO  },
	{ AR5K_MODE_108G,	MODE_ATHEROS_TURBOG },
};

/**
 * ath5k_mode_to_mac80211_mode - Map ar5k phy opmode to a mac80211 hw mode.
 * @ath_mode: ar5k hardware mode (AR5K_MODE_*)
 *
 * Maps the given ar5k phy operation mode to it's mac80211 equivalent hw
 * operation mode (MODE_IEEE80211*). Returns -EINVAL on failure.
 */
static int
ath5k_ar5k_to_mac80211_mode(u_int ar5k_mode)
{
	int i;

	printk("ar5k_to_mac80211_mode called\n");
	for (i = 0; i < ARRAY_SIZE(ar5k_mode_map); i++) {
		if (ar5k_mode_map[i].ar5k_mode == ar5k_mode) {
			printk("ar5k_mode = %d, mac80211_mode = %d\n", 
					ar5k_mode, ar5k_mode_map[i].mac80211_mode);
			return ar5k_mode_map[i].mac80211_mode;
		}
	}
	printk(KERN_ERR "Invalid ar5k phy operation mode.\n");
	return -EINVAL ;
}

/**
 * mac80211_mode_to_ath_mode - Map mac80211 hw opmode to a ar5k phy opmode.
 * @mac80211_mode: ieee80211 hardware mode (MODE_IEEE80211A, MODE_IEEE80211B ...)
 *
 * Maps the given mac80211 hw operation mode to it's equivalent ar5k phy 
 * operation mode (MODE_IEEE80211*). Returns -1 on failure.
 */
	static int
ath5k_mac80211_to_ar5k_mode(u_int mac80211_mode)
{
	int i;
	for (i = 0; i < sizeof(ar5k_mode_map) / sizeof(ar5k_mode_map[0]); i++) {
		if (ar5k_mode_map[i].mac80211_mode == mac80211_mode)
			return ar5k_mode_map[i].ar5k_mode;
	}
	printk(KERN_ERR "Invalid mac80211 hw mode.\n");
	return -EINVAL ;
}

int
ath5k_calc_bssid_mask(struct ieee80211_hw *hw)
{
	struct ath5k_softc *sc = hw->priv;
	int i, j;
	struct net_device *dev;
	unsigned char mask[ETH_ALEN];
	memset(mask, 0xff, ETH_ALEN);
	for (i = 0; i < sc->sc_num_bss; i++) {
		dev = dev_get_by_index(sc->sc_bss[i].ab_if_id);
		for (j = 0; j < ETH_ALEN; j++) {
			mask[j] &= 
				~(hw->wiphy->perm_addr[j] ^ dev->dev_addr[j]);
		}
		dev_put(dev);
	}
	if (memcmp(sc->sc_bssidmask, mask, ETH_ALEN)) {
		memcpy(sc->sc_bssidmask, mask, ETH_ALEN);
		return 1;
	}
	return 0;
}

/* /dadwifi */

/*
 * Convert MHz frequency to IEEE channel number.
 */
u_int
ath5k_mhz2ieee(u_int freq, u_int flags)
{
	if (flags & CHANNEL_2GHZ) {	/* 2GHz band */
		if (freq == 2484)	/* Japan */
			return 14;
		/* don't number non-IEEE channels unless we do channel tests */
		if ((freq >= 2412) && (freq < 2484))
			return (freq - 2407) / 5;
		if (CHAN_DEBUG == 1) /* 15-26 */
			return ((freq - 2512)/20) + 15;
		return 0;
	} else if (flags & CHANNEL_5GHZ)	{	/* 5Ghz band */
		/* don't number non-IEEE channels unless we do channel tests */
		if (((freq >= 5150) && (freq <= 5825))|| CHAN_DEBUG == 1)
			return (freq - 5000) / 5;
		return 0;
	} else
		/* something is fishy, don't do anything */
		return 0;
}

/*
 * Convert IEEE channel number to MHz frequency.
 */
static u_int
ath5k_ieee2mhz(u_int chan, u_int flags)
{
	if (flags & CHANNEL_2GHZ) {	/* 2GHz band */
		if (chan == 14)
			return 2484;
		if (chan < 14)
			return 2407 + chan * 5;
		else
			return 2512 + ((chan - 15) * 20);
	} else if (flags & CHANNEL_5GHZ) /* 5Ghz band */
		return 5000 + (chan * 5);
	else {					/* either, guess */
		if (chan == 14)
			return 2484;
		if (chan < 14)			/* 0-13 */
			return 2407 + chan * 5;
		if (chan < 27)			/* 15-26 */
			return 2512 + ((chan - 15) * 20);
		return 5000 + (chan * 5);
	}
}



/***********************\
* DRIVER INITIALIZATION *
\***********************/

/*
 * Module info
 */

MODULE_AUTHOR("The MadWiFi Team <www.madwifi.org>");
MODULE_DESCRIPTION("Support for Atheros 802.11 wireless LAN cards.");
#ifdef MODULE_VERSION
MODULE_VERSION("alpha");
#endif
MODULE_SUPPORTED_DEVICE("Atheros 5xxx WLAN cards (PCI)");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

/*
 * Static driver structs
 */

/* Known PCI ids */
static struct pci_device_id ar5k_pci_ids[] __devinitdata = {
	{ 0x168c, 0x0207, PCI_ANY_ID, PCI_ANY_ID },	/* 5210 early */
	{ 0x168c, 0x0007, PCI_ANY_ID, PCI_ANY_ID },	/* 5210 */
	{ 0x168c, 0x0011, PCI_ANY_ID, PCI_ANY_ID },	/* 5311 */
	{ 0x168c, 0x0012, PCI_ANY_ID, PCI_ANY_ID },	/* 5211 */
	{ 0x168c, 0x0013, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 */
	{ 0xa727, 0x0013, PCI_ANY_ID, PCI_ANY_ID },	/* 3com 5212 */
	{ 0x10b7, 0x0013, PCI_ANY_ID, PCI_ANY_ID },	/* 3com 3CRDAG675 5212 */
	{ 0x168c, 0x1014, PCI_ANY_ID, PCI_ANY_ID },	/* IBM minipci 5212 */
	{ 0x168c, 0x0014, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 combatible */
	{ 0x168c, 0x0015, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 combatible */
	{ 0x168c, 0x0016, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 combatible */
	{ 0x168c, 0x0017, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 combatible */
	{ 0x168c, 0x0018, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 combatible */
	{ 0x168c, 0x0019, PCI_ANY_ID, PCI_ANY_ID },	/* 5212 combatible */
	{ 0x168c, 0x001a, PCI_ANY_ID, PCI_ANY_ID },	/* 2413 Griffin-lite */
	{ 0x168c, 0x001b, PCI_ANY_ID, PCI_ANY_ID },	/* 5413 Eagle */
	{ 0x168c, 0x001c, PCI_ANY_ID, PCI_ANY_ID },	/* 5424 Condor (PCI-E)*/
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ar5k_pci_ids);

/* PCI stack related functions */
static struct pci_driver ath5k_pci_ops = {
	.name		= "ath5k_pci",
	.id_table	= ar5k_pci_ids,
	.probe		= ath5k_pci_probe,
	.remove		= ath5k_pci_remove,
#ifdef CONFIG_PM
	.suspend	= ath5k_pci_suspend,
	.resume		= ath5k_pci_resume,
#endif
};

/* mac80211 stack related functions */
static struct ieee80211_ops ath5k_ops = {
	.tx = ath5k_tx,
	.reset = ath5k_reset,
	.open = ath5k_open, 
	.stop = ath5k_stop,
	.add_interface = ath5k_add_interface,
	.remove_interface = ath5k_remove_interface,
	.config = ath5k_config,
	.config_interface = ath5k_config_interface,
	.get_tsf = ath5k_get_tsf, /* 64 bit */
	.reset_tsf = ath5k_reset_tsf,
};

/* 
 * PCI Initialization
 */
static int __init
ath5k_pci_init(void)
{
	printk(KERN_INFO "ath5k_pci loading (trying at least)\n");

	if (pci_register_driver(&ath5k_pci_ops) < 0) {
		printk(KERN_INFO "ath5k_pci: No devices found, driver not installed.\n");
		return (-ENODEV);
	}
	return (0);
}
module_init(ath5k_pci_init);

static void __exit
ath5k_pci_exit(void)
{
	pci_unregister_driver(&ath5k_pci_ops);
	printk(KERN_INFO "ath5k_pci: Driver unloaded fine\n");
}
module_exit(ath5k_pci_exit);

int
ath5k_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	unsigned long phymem;
	void __iomem *mem;
	struct ieee80211_hw *hw ;
	struct ath5k_softc *sc;
	u_int32_t val;
	u_int8_t csz;
	int error = 0;

	if (pci_enable_device(pdev))
		return -EIO;

	/* XXX 32-bit addressing only */
	if (pci_set_dma_mask(pdev, 0xffffffff)) {
		printk(KERN_ERR "ath5k_pci: 32-bit DMA not available\n");
		goto bad;
	}

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &csz);
	if (csz == 0) {
		/*
		 * Linux 2.4.18 (at least) writes the cache line size
		 * register as a 16-bit wide register which is wrong.
		 * We must have this setup properly for rx buffer
		 * DMA to work so force a reasonable value here if it
		 * comes up zero.
		 */
		csz = L1_CACHE_BYTES / sizeof(u_int32_t);
		pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, csz);
	}
	/*
	 * The default setting of latency timer yields poor results,
	 * set it to the value used by other systems.  It may be worth
	 * tweaking this setting more.
	 */
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0xa8);

	/* Enable bus mastering */
	pci_set_master(pdev);

	/*
	 * Disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 *
	 * Code taken from ipw2100 driver - jg
	 */
	pci_read_config_dword(pdev, 0x40, &val);
	if ((val & 0x0000ff00) != 0)
		pci_write_config_dword(pdev, 0x40, val & 0xffff00ff);

	/* Reserve PCI memory region */
	phymem = pci_resource_start(pdev, 0);
	if (!request_mem_region(phymem, pci_resource_len(pdev, 0), "ath")) {
		printk(KERN_ERR "ath5k_pci: cannot reserve PCI memory region\n");
		goto bad;
	}

	/* Get the virtual address in kernel space */
	mem = ioremap(phymem, pci_resource_len(pdev, 0));
	if (!mem) {
		printk(KERN_ERR "ath5k_pci: cannot remap PCI memory region\n") ;
		goto bad1;
	}


	/* 
	 * Allocate hw (mac80211 main struct)
	 * and hw->priv (driver private data)
	 */
	hw = ieee80211_alloc_hw(sizeof(struct ath5k_softc), &ath5k_ops);

	if (!hw) {
		printk("ath5k: Failed to allocate ieee80211_hw\n");
		goto bad2;
	}
	sc = hw->priv;

	/* Initialize driver private data */

	/*
	 * In case we support ahb bus in the future we
	 * have a void pointer to the (abstract) device struct
	 */
	sc->sc_bdev = (void *) pdev;

	/* convert cache line size to bytes */
	sc->sc_cachelsz = csz << 2;

	/* So we can unmap it on detach */
	sc->sc_iobase = mem;

	/* Finish private driver data initialization */
	error = ath5k_init(hw, id->device);
	if (error) {
		printk(KERN_ERR "ath5k: Unable to initialize private driver data !\n");
		goto bad3;
	}

	/* Set hw as private driver data */
	pci_set_drvdata(pdev, hw);

	/* Don't enable interrupt handling yet,
	 * wait for rx/tx setup */
	sc->sc_intr_pause = 1;

	/*TODO: Interrupt handler setup */

	/* Start interrupt processing */
	sc->sc_intr_pause = 0;

	return 0;

bad3:
	ieee80211_free_hw(hw);
bad2:
	iounmap(mem);
bad1:
	release_mem_region(phymem, pci_resource_len(pdev, 0));
bad:
	pci_disable_device(pdev);
	printk(KERN_INFO "Detach ok\n");
	return -ENODEV;
}

void
ath5k_pci_remove(struct pci_dev *pdev)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	struct ath5k_softc *sc = hw->priv;

	/* Stop driver activity */
	ath5k_detach(hw);

	/* Free allocated pci resources */
	if (pdev->irq)
		free_irq(pdev->irq, hw);
	iounmap(sc->sc_iobase);
	release_mem_region(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
	pci_disable_device(pdev);

	/* Detach driver from mac80211 stack */
	ieee80211_free_hw(hw);
}

#ifdef CONFIG_PM
int
ath5k_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);

	if (ath5k_tx_stop(hw))
		printk(KERN_ERR "ath5k_pci: Could not stop tx!\n");

	if (ath5k_rx_stop(hw))
		printk(KERN_ERR "ath5k_pci: Could not stop rx!\n");

	/*
	 * TODO: Stop interrupts on hw, put chip on sleep, disable phy
	 */

	pci_save_state(pdev);
	pci_disable_device(pdev);
	return pci_set_power_state(pdev, PCI_D3hot);
}

int
ath5k_pci_resume(struct pci_dev *pdev)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	u32 val;
	int err;

	err = pci_set_power_state(pdev, PCI_D0);
	if (err)
		return err;

	pci_restore_state(pdev);

	err = pci_enable_device(pdev);
	if (err)
		return err;

	pci_set_master(pdev);
	/*
	 * Suspend/Resume resets the PCI configuration space, so we have to
	 * re-disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state
	 *
	 * Code taken from ipw2100 driver - jg
	 */
	pci_read_config_dword(pdev, 0x40, &val);
	if ((val & 0x0000ff00) != 0)
		pci_write_config_dword(pdev, 0x40, val & 0xffff00ff);

	/* TODO:
	 * Re enable interrupts on hw
	 * Pause interrupts on sw
	 * Chip reset
	 */

	if (ath5k_rx_start(hw))
		printk(KERN_ERR "ath5k_pci: Could not start rx!\n");

	if (ath5k_tx_start(hw))
		printk(KERN_ERR "ath5k_pci: Could not start tx!\n");

	/* TODO: Resume interrupts on sw */

	return 0;
}
#endif /* CONFIG_PM */

/*
 * Private driver data
 * initialization
 */

/*
 * HW Functions error codes
 */
static const char *ath5k_hw_status_msg[] = {
	"Everything went O.K.",
	"Unable to allocate memory for ath_hal",
	"Hardware I/O Error",
	"Unable to access EEPROM",
	"Invalid EEPROM checksum",
	"Unable to get device caps from EEPROM",
	"Unable to read MAC address from EEPROM",
	"Invalid parameter to function",
	"Hardware revision not supported",
	"Unexpected error ocured during process"
};

static const char* 
ath5k_get_hw_status_message(AR5K_STATUS status)
{
	if ((status > 0) && (status < ARRAY_SIZE(ath5k_hw_status_msg)))
		return ath5k_hw_status_msg[status];
	else
		return "";
}

int
ath5k_init(struct ieee80211_hw *hw, u_int16_t devid)
{
	struct ath_hal *ah;
	AR5K_STATUS status;
	struct ath5k_softc *sc = hw->priv;
	struct pci_dev *pdev = (struct pci_dev *)sc->sc_bdev;
	int error, i = 0;

	/*
	 * Initialize hw
	 */
	ah = ath5k_hw_init(devid, sc, 0,
			(__force AR5K_BUS_HANDLE) sc->sc_iobase, &status);

	if (!ah) {
		printk(KERN_ERR "ath5k: unable to attach hardware: '%s'\n",
			ath5k_get_hw_status_message(status));
		error = -ENXIO;
		goto bad;
	}
	sc->sc_ah = ah;

	/*
	 * Check if the device has hardware counters for PHY
	 * errors.  If so we need to enable the MIB interrupt
	 * so we can act on stat triggers.
	 */
	/* TODO: Implement this in openhal */
	/*
	if (ath_hal_hwphycounters(ah))
		sc->sc_needmib = 1;
	*/

	/*
	 * Get the hardware key cache size.
	 */
	sc->sc_keymax = ath5k_hw_get_keycache_size(ah);

	/*
	 * Reset the key cache since some parts do not
	 * reset the contents on initial power up.
	 */
	for (i = 0; i < sc->sc_keymax; i++)
		ath5k_hw_reset_key(ah, i);

	/* Initialize PHY calibration timer */
#if 0
	init_timer(&sc->sc_cal_ch);
	sc->sc_cal_ch.function = ath5k_hw_phy_calibrate;
	sc->sc_cal_ch.data = (unsigned long) sc;
#endif

	if (sc->sc_softled) {
		ath5k_hw_set_gpio_output(ah, sc->sc_ledpin);
		ath5k_hw_set_gpio(ah, sc->sc_ledpin, !sc->sc_ledon);
	}

	/*
	 * Query the HAL about antenna support
	 * TODO: Implement diversity enable/disable in openhal
	 */
	sc->sc_defant = ath5k_hw_get_def_antenna(ah);


	/* get mac address from hardware */
	ath5k_hw_get_lladdr(ah, hw->wiphy->perm_addr);
	if (sc->sc_hasbmask) {
		memset(sc->sc_bssidmask, 0xff, ETH_ALEN);
		ath5k_hw_set_bssid_mask(ah, sc->sc_bssidmask);
	}

	/* Misc stack parameters TODO: REVIEW !!!*/
	hw->flags =	IEEE80211_HW_HOST_GEN_BEACON |
			IEEE80211_HW_RX_INCLUDES_FCS |
			IEEE80211_HW_HOST_BROADCAST_PS_BUFFERING |
			IEEE80211_HW_WEP_INCLUDE_IV |
			IEEE80211_HW_DATA_NULLFUNC_ACK;

	hw->extra_tx_headroom = 2;
	hw->channel_change_time = 5000;
	hw->max_rssi = 127; /* FIXME: get a real value for this (sync with rssi threshold on openhal). */
	hw->queues = 1;	/* FIXME use WME */

	sc->sc_num_modes = 0;

	/* Setup supported hw modes */

	if (sc->sc_ah->ah_capabilities.cap_mode & AR5K_MODE_11A)
		ath5k_mode_setup(hw, AR5K_MODE_11A);
	if (sc->sc_ah->ah_capabilities.cap_mode & AR5K_MODE_11B)
		ath5k_mode_setup(hw, AR5K_MODE_11B);
	if (sc->sc_ah->ah_capabilities.cap_mode & AR5K_MODE_11G)
		ath5k_mode_setup(hw, AR5K_MODE_11G);
	/* XXX: Turbo ? */

	/* Register device on 80211 stack */
	SET_IEEE80211_DEV(hw, &pdev->dev);
	if (ieee80211_register_hw(hw)) {
		printk(KERN_ERR "ath5k: Could not register device on 80211 stack !\n");
		error = -EINVAL;
	}

	/* Set the first mode/channel on the channel list */
	if ((sc->sc_num_modes > 0) && (sc->sc_hw_modes[0].num_channels > 0)) {
		hw->conf.freq = sc->sc_hw_modes[0].channels[0].freq;
		hw->conf.channel_val = sc->sc_hw_modes[0].channels[0].val;
	}

	/* Set up rx */
	if (ath5k_rx_setup(hw))
		printk(KERN_ERR "ath5k: Could not set up rx !\n");

	/* Set up tx */
	if (ath5k_rx_setup(hw))
		printk(KERN_ERR "ath5k: Could not set up rx !\n");

	/* Start interrupt processing */
	sc->sc_intr_pause = 0;

	/* Start rx */
	if (ath5k_rx_start(hw))
		printk(KERN_ERR "ath5k: Could not start rx !\n");

	return 0;

bad:
	if (ah)
		ath5k_hw_detach(ah);
	sc->sc_intr_pause = 1;

	return error;
}

int
ath5k_detach(struct ieee80211_hw *hw)
{
	struct ath5k_softc *sc;
	struct ath_hal *ah = sc->sc_ah;
#if 0
	u_int32_t tmp;
#endif

	sc = hw->priv;
	ath5k_hw_set_power(sc->sc_ah, AR5K_PM_AWAKE, TRUE, 0);

	/* Stop interrupt handling */
	sc->sc_intr_pause = 1;

	/*
	 * NB: the order of these is important:
	 * o call the 802.11 layer before detaching the HAL to
	 *   ensure callbacks into the driver to delete global
	 *   key cache entries can be handled
	 * o reclaim the tx queue data structures after calling
	 *   the 802.11 layer as we'll get called back to reclaim
	 *   node state and potentially want to use them
	 * o to cleanup the tx queues the HAL is called, so detach
	 *   it last
	 * Other than that, it's straightforward...
	 */
	ieee80211_unregister_hw(hw);	

#if 0
	/* Temporarily disable to avoid null pointer */
	ath5k_hw_set_intr(ah, 0);	/* disable further intr's */
	ath5k_hw_get_isr(ah, &tmp);	/* clear ISR */
#endif


	if (ath5k_rx_stop(hw))
		printk(KERN_ERR "ath5k: Could not stop rx !\n");

	if (ath5k_tx_stop(hw))
		printk(KERN_ERR "ath5k: Could not stop rx !\n");

	/* XXX: This function needs work */
	ath5k_hw_detach(ah);

	ieee80211_free_hw(hw);
	kfree(sc->sc_bss);
	return 0;
}


/*
 * Setup operating phymodes
 */
int
ath5k_mode_setup(struct ieee80211_hw *hw, u_int ar5k_mode)
{
	struct ath5k_softc *sc;
	struct ieee80211_hw_mode *mode;
	struct ieee80211_rate *mac80211_rates;
	const AR5K_RATE_TABLE *ar5k_rt;
	int i;
	u_int max_chans;

	sc = hw->priv;

	mode = kmalloc(sizeof(struct ieee80211_hw_mode), GFP_KERNEL);
	memset(mode, 0, sizeof(struct ieee80211_hw_mode));

	ar5k_rt = ath5k_hw_get_rate_table(sc->sc_ah, ar5k_mode);

	mac80211_rates = kmalloc((sizeof(struct ieee80211_rate) * 
				ar5k_rt->rate_count), GFP_KERNEL);
	memset(mode, 0, (sizeof(struct ieee80211_rate) * ar5k_rt->rate_count));

	if (!ar5k_rt)
		return -EINVAL;

	mode = &sc->sc_hw_modes[sc->sc_num_modes];
	mode->mode = ath5k_ar5k_to_mac80211_mode(ar5k_mode);

	/* Setup channels */

	/* 
	 * We set the maximum channels
	 * supported by hw, regulatory
	 * domain restrictions are handled
	 * in init_channels_for_mode.
	 */
	switch(ar5k_mode) {
	case AR5K_MODE_11A:
		max_chans = 221;
		break;
	case AR5K_MODE_11B:
	case AR5K_MODE_11G:
		max_chans = 26;
		break;
	default:
		printk(KERN_ERR "ath5k: Unsupported ar5k_mode passed on mode_setup!\n");
		return -EINVAL;
	}

	if (ath5k_init_channels_for_mode(sc->sc_ah, mode, max_chans)) {
		printk(KERN_ERR "ath5k: Could not set up channels for mode: %d !\n",
			mode->mode);
		return -EINVAL;
	}

	/* Setup rates */
	mode->num_rates = 0;
	mode->rates = mac80211_rates;
	for (i = 0; i < ar5k_rt->rate_count; i++) {
		mac80211_rates[i].rate = ar5k_rt->rates[i].rate_kbps / 100;
		mac80211_rates[i].val = ar5k_rt->rates[i].rate_code;
		mac80211_rates[i].flags = ar5k_rt->rates[i].modulation;
		mac80211_rates[i].val2 = 0 ;

		/* XXX: not used in mac80211 */
#if 0
		mac80211_rate->min_rssi_ack;
		mac80211_rate->min_rssi_ack_delta;
#endif

		mode->num_rates++;

	}

	/* Register mode */
	if (ieee80211_register_hwmode(hw, mode)) {
		printk(KERN_ERR "Could not register phymode to stack !\n");
		return -EINVAL;
	} else {
		printk(KERN_DEBUG "register hw_mode %d\n", mode->mode);
		sc->sc_num_modes++;
	}

	return 0;
}

/* XXX: What about sc->channels array ??? is it needed ??? */
int
ath5k_init_channels_for_mode(struct ath_hal *hal,
		struct ieee80211_hw_mode *mode, u_int max_chans)
{
	struct ieee80211_channel *channels;
	u_int flags;
	short freq;
	int i;

	/* Allocate and initialize channel array */
	channels = kmalloc((sizeof(struct ieee80211_channel) * max_chans),
			GFP_KERNEL);

	if (!channels)
		return -ENOMEM;

	memset(channels, 0, (sizeof(struct ieee80211_channel) * max_chans));

	switch(mode->mode) {
	case MODE_IEEE80211B:
		flags = CHANNEL_B;
		break;
	case MODE_IEEE80211G:
		flags = CHANNEL_G;
		break;
	case MODE_IEEE80211A:
		flags = CHANNEL_A;
		break;
	default:
		printk(KERN_ERR "ath5k: Invalid mode passed for channel setup !\n");
		return -EINVAL;
	}

	for (i = 1; i < max_chans; i++) {
		/* TODO: Channel Restrictions based on reg domain !!*/
		freq = ath5k_ieee2mhz(i, flags);
		if (!ath5k_hw_check_channel(hal, freq, flags)) {
			printk(KERN_DEBUG "ath5k: Channel out of hw limits !\n");
			i--;
			break;
		} 
		channels[i].chan = i;
		channels[i].freq = freq;
		channels[i].val = flags;
	}

	mode->num_channels = i;
	mode->channels = channels;

	return 0;
}

/*
 * RX setup
 */
int
ath5k_rx_setup(struct ieee80211_hw *hw)
{
	return 0;
}

/*
 * RX start
 */
int
ath5k_rx_start(struct ieee80211_hw *hw)
{
	return 0;
}

/*
 * RX stop
 */
int
ath5k_rx_stop(struct ieee80211_hw *hw)
{
	return 0;
}

/*
 * Tx Queue setup
 */
static int
ath5k_tx_queue_setup(struct ieee80211_hw *hw, 
		u_int16_t flags, u_int8_t qnum)
{
	return 0;
}

/*
 * Tx Queue start
 */
static int
ath5k_tx_queue_start(struct ieee80211_hw *hw, u_int8_t qnum) {

	return 0;
}

/*
 * Tx Queue stop
 */
static int
ath5k_tx_queue_stop(struct ieee80211_hw *hw, u_int8_t qnum) {

	return 0;
}

/*
 * Tx Setup
 */
int
ath5k_tx_setup(struct ieee80211_hw *hw) {

	return 0;
}

/*
 * Tx start
 */
int
ath5k_tx_start(struct ieee80211_hw *hw) {

	return 0;
}

/*
 * Tx stop
 */
int
ath5k_tx_stop(struct ieee80211_hw *hw) {

	return 0;
}

/*
 * Main reset function
 * action:
 * 0 -> normal reset
 * 1 -> initial reset
 * 2 -> just change channel
 */
int
ath5k_state_reset(struct ieee80211_hw *hw, u_int8_t action)
{
	return 0;
}

/********************\
* Interrupt handling *
\********************/

/* TODO */

/********************\
* mac80211 functions *
\********************/

int ath5k_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
		struct ieee80211_tx_control *control)
{
	/* TODO:
	 * -Get infos from control and set them on sc (review sc)
	 * -Check if the specified q is active
	 * -Set rate
	 * -Set tx power
	 * -Set antenna
	 * -Handle encryption stuff
	 * -Setup descriptor for this frame/queue
	 * (review sc, we have to check status descriptor on interrupt handling)
	 * -Fill descriptor
	 * -Pass descriptor on hw (put_tx_buf)
	 * -Sync dma stuff
	 * -Start tx dma on hw for this q (tx_start)
	 */
	return 0;
}

int
ath5k_reset(struct ieee80211_hw *hw)
{
	int error;
	error = ath5k_state_reset(hw, 0);
	return error;
}

int
ath5k_open(struct ieee80211_hw *hw)
{
	int error;
	error = ath5k_state_reset(hw, 1);
	return error;
}

	int
ath5k_stop(struct ieee80211_hw *hw)
{
	/* TODO:
	 * -Stop queues on stack
	 * -Disable interrupts
	 * -Wait untill we have no pending frames
	 * -Call rx_stop
	 * -Call tx_stop
	 * -Check if everything went ok
	 * -Stop calibration timer
	 * -Put device on sleep
	 */
	return 0;
}

	int
ath5k_add_interface(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct ath5k_softc *sc = hw->priv;
	int error = 0;
	down(&(sc)->sc_lock);
	/* TODO: Use mac80211 definitions
	 * inside openhal */
	switch (conf->type) {
		case IEEE80211_IF_TYPE_STA:
			sc->sc_opmode = AR5K_M_STA;
			break;
		case IEEE80211_IF_TYPE_IBSS:
			sc->sc_opmode = AR5K_M_IBSS;
			break;
		case IEEE80211_IF_TYPE_MNTR:
			sc->sc_opmode = AR5K_M_MONITOR;
			break;
		case IEEE80211_IF_TYPE_AP: /* Notyet */
		default:
			error = -EINVAL;
			goto done;
	}
	if (ath5k_calc_bssid_mask(hw))
		error = ath5k_state_reset(hw, 0);
done:
	up(&(sc)->sc_lock);
	return error;
}

	void
ath5k_remove_interface(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct ath5k_softc *sc = hw->priv;

	down(&(sc)->sc_lock);
	if (sc->sc_num_bss == 0)
		sc->sc_beacons = 0;
	if (ath5k_calc_bssid_mask(hw))
		ath5k_state_reset(hw, 0);
	up(&(sc)->sc_lock);
}

	int
ath5k_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	struct ath5k_softc *sc = hw->priv;
	int error;
	sc->sc_channel = conf->channel;
	sc->sc_channels[sc->sc_channel].freq = conf->freq;
	sc->sc_channels[sc->sc_channel].val = conf->channel_val;
	/* XXX: How about conf->chan ? */
	sc->sc_mode = conf->phymode;
	/* XXX: How about conf->mode ? */
	sc->sc_beacon_interval = (conf->beacon_int * 1000) >> 10;

	/* TODO:
	 * -Set regulatory domain on sc and 
	 * if it's changed reinitialize channels
	 * -Handle global slot time
	 * -Handle tx power/antenna setup
	 * -Handle dfs stuff
	 * -If !radio_enabled configure hw and pause
	 *  interrupt processing
	 */

	if (sc->sc_intr_pause || !conf->radio_enabled)
		return 0;
	/* TODO:
	 * -Verify what kind of reset we need and call
	 *  ath5k_state_reset propertly
	 */

	error = ath5k_state_reset(hw, 2);
	if (error)
		return error;
	return 0;
}

int
ath5k_config_interface(struct ieee80211_hw *hw, int if_id,
	struct ieee80211_if_conf *conf)
{
	struct ath5k_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;

	/* TODO:
	 * Need to review sc and sc_bss
	 * -Get the virtual interface using if_id
	 * -Configure if type
	 * -If AP handle beacon stuff
	 * (can we handle both a managed virtual
	 * if and a master one at the same time?
	 * can we tweak beacon timers etc ?
	 * I remember hostapd had such a functionality.
	 * if not we must do a check here to verify all
	 * virtual if's are in master mode)
	 * -Set assoc id
	 * -Reset hw
	 */

	if (conf->bssid)
		ath5k_hw_set_associd(ah, conf->bssid, 0 /* FIXME: aid */);
	return ath5k_state_reset(hw, 0);
}

u64
ath5k_get_tsf(struct ieee80211_hw *hw)
{
	struct ath5k_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;
	return ath5k_hw_get_tsf64(ah);
}

void
ath5k_reset_tsf(struct ieee80211_hw *hw)
{
	struct ath5k_softc *sc = hw->priv;
	struct ath_hal *ah = sc->sc_ah;
	ath5k_hw_reset_tsf(ah);
}

