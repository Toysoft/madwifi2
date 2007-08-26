/*
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * Copyright (c) 2004-2005 Atheros Communications, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/cache.h>
#include <linux/pci.h>
#include <asm/uaccess.h>

static void	ath_rx_tasklet(unsigned long data);
static void	ath_tx_tasklet_q0(unsigned long data);

/* support for module parameters */
static char *ifname = "ath";
module_param(ifname, charp, 0);

MODULE_PARM_DESC(ifname, "Interface name prefix (default: ath)");

struct ath_pci_softc {
	struct ath_softc	aps_sc;
#ifdef CONFIG_PM
	u32			aps_pmstate[16];
#endif
};

static struct pci_device_id ath_pci_id_table[] __devinitdata = {
	{ PCI_ATH(5210) }, /* 5210 */
	{ PCI_ATH(5311) }, /* 5311 */
	{ PCI_ATH(5211) }, /* 5211 */
	{ PCI_ATH(5212) }, /* 5212 */
	/* 3com 3CRDAG675 5212 */
	{ PCI_VDEVICE(3COM, PCI_PRODUCT_3COM_3CRDAG675) },
	/* 3com 3CRPAG175 5212 */
	{ PCI_VDEVICE(3COM_2, PCI_PRODUCT_3COM2_3CRPAG175) },
	{ PCI_ATH(5210_AP) }, /* 5210 early */
	{ PCI_ATH(5212_IBM) }, /* IBM minipci 5212 */
	{ PCI_ATH(5212_0014) }, /* 5212 combatible */
	{ PCI_ATH(5212_0015) }, /* 5212 combatible */
	{ PCI_ATH(5212_0016) }, /* 5212 combatible */
	{ PCI_ATH(5212_0017) }, /* 5212 combatible */
	{ PCI_ATH(5212_0018) }, /* 5212 combatible */
	{ PCI_ATH(5212_0019) }, /* 5212 combatible */
#if 0 /* The following AR521x devices need to be tested, then added here */
	{ PCI_ATH(5210_DEFAULT) }, /* AR5210 (no eeprom) */
	{ PCI_ATH(5212_DEFAULT) }, /* AR5210 (no eeprom) */
	{ PCI_ATH(5211_DEFAULT) }, /* AR5210 (no eeprom) */
	{ PCI_ATH(5212_FPGA) }, /* AR5212 (emulation board) */
	{ PCI_ATH(5211_LEGACY) }, /* AR5211 (emulation board) */
	{ PCI_ATH(5211_FPGA11B) }, /* AR5211 (emulation board) */
#endif
	{ PCI_ATH(2413) }, /* 2413 Griffin-lite */
	{ PCI_ATH(5413) }, /* 5413 Eagle */
	{ PCI_ATH(5424) }, /* 5424 Condor (PCI-E)*/
	{ 0 }
};

static int
ath_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	unsigned long phymem;
	void __iomem *mem;
	struct ath_pci_softc *sc;
	struct net_device *dev;
	const char *athname;
	u_int8_t csz;
	u32 val;

	if (pci_enable_device(pdev))
		return (-EIO);

	/* XXX 32-bit addressing only */
	if (pci_set_dma_mask(pdev, 0xffffffff)) {
		printk(KERN_ERR "ath_pci: 32-bit DMA not available\n");
		goto bad;
	}

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &csz);
	if (csz == 0) {
		/*
		 * We must have csz setup properly for rx buffer
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

	phymem = pci_resource_start(pdev, 0);
	if (!request_mem_region(phymem, pci_resource_len(pdev, 0), "ath")) {
		printk(KERN_ERR "ath_pci: cannot reserve PCI memory region\n");
		goto bad;
	}

	mem = ioremap(phymem, pci_resource_len(pdev, 0));
	if (!mem) {
		printk(KERN_ERR "ath_pci: cannot remap PCI memory region\n") ;
		goto bad1;
	}

	sc = kmalloc(sizeof(struct ath_pci_softc), GFP_KERNEL);
	if (sc == NULL) {
		printk(KERN_ERR "ath_pci: no memory for device state\n");
		goto bad2;
	}
	memset(sc, 0, sizeof(struct ath_pci_softc));

	/*
	 * Mark the device as detached to avoid processing
	 * interrupts until setup is complete.
	 */
	sc->aps_sc.sc_invalid = 1;
	sc->sc_cachelsz = csz << 2; /* convert to bytes */

	dev = &sc->aps_sc.sc_dev;	/* XXX blech, violate layering */

	/* use variable interface name prefix */
	strncpy(dev->name, ifname, IFNAMSIZ - sizeof("%d") - 1);
	strncat(dev->name, "%d", sizeof("%d"));

	dev->irq = pdev->irq;
	dev->priv = sc;
	sc->aps_sc.sc_iobase = mem;

	SET_NETDEV_DEV(dev, &pdev->dev);

	sc->aps_sc.sc_bdev = (void *) pdev;

	pci_set_drvdata(pdev, dev);

	if (request_irq(dev->irq, ath_intr, IRQF_SHARED, dev->name, dev)) { 
		printk(KERN_WARNING "%s: request_irq failed\n", dev->name);
		goto bad3;
	}

	if (ath_attach(id->device, dev) != 0)
		goto bad4;

	athname = ath5k_hw_mac_name(id->vendor, id->device);
	printk(KERN_INFO "%s: %s: mem=0x%lx, irq=%d\n",
		dev->name, athname ? athname : "Atheros ???", phymem, dev->irq);

	/* ready to process interrupts */
	sc->aps_sc.sc_invalid = 0;

	return 0;
bad4:
	free_irq(dev->irq, dev);
bad3:
	kfree(sc);
bad2:
	iounmap(mem);
bad1:
	release_mem_region(phymem, pci_resource_len(pdev, 0));
bad:
	pci_disable_device(pdev);
	return (-ENODEV);
}

static void
ath_pci_remove(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct ath_pci_softc *sc = dev->priv;

	ath_detach(dev);
	if (dev->irq)
		free_irq(dev->irq, dev);
	iounmap(sc->aps_sc.sc_iobase);
	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));
	pci_disable_device(pdev);
	free_netdev(dev);
}

#ifdef CONFIG_PM
static int
ath_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *dev = pci_get_drvdata(pdev);

	ath_suspend(dev);
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);

	return (0);
}

static int
ath_pci_resume(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	u32 val;
	int err;

	err = pci_set_power_state(pdev, PCI_D0);
	if (err)
		return err;

	err = pci_enable_device(pdev);
	if (err)
		return err;

	pci_restore_state(pdev);
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
	ath_resume(dev);

	return (0);
}
#else
#define ath_pci_suspend		NULL
#define ath_pci_resume		NULL
#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(pci, ath_pci_id_table);

static struct pci_driver ath_pci_drv_id = {
	.name		= "ath_pci",
	.id_table	= ath_pci_id_table,
	.probe		= ath_pci_probe,
	.remove		= ath_pci_remove,
	.suspend	= ath_pci_suspend,
	.resume		= ath_pci_resume,
};

static char *version = "0.1";
static char *dev_info = "ath5k_pci";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_AUTHOR("Luis R. Rodriguez <mcgrof@winlab.rutgers.edu>");
MODULE_DESCRIPTION("Support for Atheros 802.11 wireless LAN cards.");
MODULE_SUPPORTED_DEVICE("Atheros WLAN cards");
MODULE_LICENSE("Dual BSD/GPL");

static int __init
init_ath_pci(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);

	if (pci_register_driver(&ath_pci_drv_id) < 0) {
		printk("ath_pci: No devices found, driver not installed.\n");
		pci_unregister_driver(&ath_pci_drv_id);
		return (-ENODEV);
	}
	return (0);
}
module_init(init_ath_pci);

static void __exit
exit_ath_pci(void)
{
	pci_unregister_driver(&ath_pci_drv_id);
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_pci);

/* Interrupt handler */
irqreturn_t
ath_intr(int irq, void *dev_id) 
{
	struct net_device *dev = dev_id;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	enum ar5k_int status;

	if (sc->sc_invalid) {
		 /* The hardware is not ready/present, don't touch anything.
		 * Note this can happen early on if the IRQ is shared. */
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid; ignored\n", __func__);
		return IRQ_NONE;
	}
	if (!ath5k_hw_irq_pending(ah)) /* shared irq, not for us */
		return IRQ_NONE;
	if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: if_flags 0x%x\n",
			__func__, dev->flags);
		ath5k_hw_getisr(ah, &status); /* clear ISR */
		ath5k_hw_set_intr(ah, 0); /* disable further intr's */
		return IRQ_HANDLED;
	}
	
	/*
	* Figure out the reason(s) for the interrupt.  Note
	* that the hal returns a pseudo-ISR that may include
	* bits we haven't explicitly enabled so we mask the
	* value to insure we only process bits we requested.
	*/
	ath5k_hw_get_isr(ah, &status); /* NB: clears ISR too */
	DPRINTF(sc, ATH_DEBUG_INTR, "%s: status 0x%x\n", __func__, status);
	/* Discard any unwanted bits from ISR */
	status &= sc->sc_imask;

#if 0
        if ((status & AR5K_ISR_RXPHY) && hal->ah_radar.r_enabled == TRUE)
                ath5k_hw_radar_alert(hal);
#endif

	if (status & AR5K_INT_FATAL) {
		/* Fatal errors are unrecoverable. Typically these are caused 
		 * by DMA errors. */
		sc->sc_stats.ast_hardware++;
		/* Disable interrupts until reset */
		ath5k_hw_set_intr(ah, 0);
		tasklet_schedule(&sc->sc_resettq);
	} else if (status & AR5K_INT_RXORN) {
		sc->sc_stats.ast_rxorn++;
		/* Disable interrupts until reset */
		ath5k_hw_set_intr(ah, 0);
		tasklet_schedule(&sc->sc_resettq);
	} else {
		if (status & AR5K_INT_SWBA) {
			/* Software beacon alert--time to send a beacon.
			 * Handle beacon transmission directly; deferring
			 * this is too slow to meet timing constraints
			 * under load. */
			/* XXX: Consider moving to tasklet and using
			 * tasklet_hi_schedule(). The kernel will then
			 * run this tasklet more urgently than networking,
			 * SCSI, timers or anything else */
			ath_beacon_send(dev);
		}
		if (status & AR5K_INT_RXEOL) {
			/*
			* NB: the hardware should re-read the link when
			*     RXE bit is written, but it doesn't work at
			*     least on older hardware revs.
			*/
			sc->sc_stats.ast_rxeol++;
			/* XXX: check why line bellow is there */
			sc->sc_rxlink = NULL;
		}
		if (status & AR5K_INT_TXURN) {
			sc->sc_stats.ast_txurn++;
			/* bump tx trigger level */
			ath_hal_updatetxtriglevel(ah, TRUE);
		}
		if (status & AR5K_INT_RX) {
			/* XXX: consider adding (from madwifi-trunk):
			 * sc->sc_tsf = ath_hal_gettsf64(ah);
			 * ath_uapsd_processtriggers(sc);
			 */
			tasklet_schedule(&sc->sc_rxtq);
		}
		if (status & AR5K_INT_TX)
			tasklet_schedule(&sc->sc_txtq);
		if (status & AR5K_INT_BMISS) {
			sc->sc_stats.ast_bmiss++;
			tasklet_schedule(&sc->sc_bmisstq);
		}
		if (status & AR5K_INT_MIB) {
			sc->sc_stats.ast_mib++;
			/*
			* Disable interrupts until we service the MIB
			* interrupt; otherwise it will continue to fire.
			*/
			ath5k_hw_set_intr(ah, 0);
			/*
			* Let the hal handle the event.  We assume it will
			* clear whatever condition caused the interrupt.
			*/
			ath_hal_mibevent(ah,
				&ATH_NODE(sc->sc_ic.ic_bss)->an_halstats);
			ath5k_hw_set_intr(ah, sc->sc_imask);
		}
	}
	
	return IRQ_HANDLED;
}

static void
ath_reset_tasklet(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	printk(KERN_WARN "ath5k: hardware error; resetting\n");
	ath_reset(dev);
}

static void
ath_rx_tasklet(unsigned long data)
{
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_desc + \
		((_pa) - (_sc)->sc_desc_daddr)))
	struct net_device *dev = (struct net_device *)data;
	struct ath_buf *bf;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	struct sk_buff *skb;
	struct ieee80211_node *ni;
	struct ath_node *an;
	int len, type;
	u_int phyerr;
	AR5K_STATUS status;

	DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s\n", __func__);
	do {
		bf = STAILQ_FIRST(&sc->sc_rxbuf);
		if (bf == NULL) {		/* XXX ??? can this happen */
			if_printf(dev, "%s: no buffer!\n", __func__);
			break;
		}
		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at the end */
			break;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {		/* XXX ??? can this happen */
			if_printf(dev, "%s: no skbuff!\n", __func__);
			continue;
		}
		/* XXX sync descriptor memory */
		/*
		 * Must provide the virtual address of the current
		 * descriptor, the physical address, and the virtual
		 * address of the next descriptor in the h/w chain.
		 * This allows the HAL to look ahead to see if the
		 * hardware is done with a descriptor by checking the
		 * done bit in the following descriptor and the address
		 * of the current descriptor the DMA engine is working
		 * on.  All this is necessary because of our use of
		 * a self-linked list to avoid rx overruns.
		 */
		status = ath5k_hw_rx_desc_proc(ah, ds,
				bf->bf_daddr, PA2DESC(sc, ds->ds_link));
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RECV_DESC)
			ath_printrxbuf(bf, status == AR5K_OK); 
#endif
		if (status == AR5K_EINPROGRESS)
			break;
		STAILQ_REMOVE_HEAD(&sc->sc_rxbuf, bf_list);
		if (ds->ds_rxstat.rs_more) {
			/*
			 * Frame spans multiple descriptors; this
			 * cannot happen yet as we don't support
			 * jumbograms.  If not in monitor mode,
			 * discard the frame.
			 */
#ifndef ERROR_FRAMES
			/*
			 * Enable this if you want to see
			 * error frames in Monitor mode.
			 */
			if (ic->ic_opmode != IEEE80211_M_MONITOR) {
				sc->sc_stats.ast_rx_toobig++;
				goto rx_next;
			}
#endif
			/* fall thru for monitor mode handling... */
		} else if (ds->ds_rxstat.rs_status != 0) {
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_CRC)
				sc->sc_stats.ast_rx_crcerr++;
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_FIFO)
				sc->sc_stats.ast_rx_fifoerr++;
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_PHY) {
				sc->sc_stats.ast_rx_phyerr++;
				phyerr = ds->ds_rxstat.rs_phyerr & 0x1f;
				sc->sc_stats.ast_rx_phy[phyerr]++;
				goto rx_next;
			}
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_DECRYPT) {
				/*
				 * Decrypt error.  If the error occurred
				 * because there was no hardware key, then
				 * let the frame through so the upper layers
				 * can process it.  This is necessary for 5210
				 * parts which have no way to setup a ``clear''
				 * key cache entry.
				 *
				 * XXX do key cache faulting
				 */
				if (ds->ds_rxstat.rs_keyix == AR5K_RXKEYIX_INVALID)
					goto rx_accept;
				sc->sc_stats.ast_rx_badcrypt++;
			}
			if (ds->ds_rxstat.rs_status & AR5K_RXERR_MIC) {
				sc->sc_stats.ast_rx_badmic++;
				/*
				 * Do minimal work required to hand off
				 * the 802.11 header for notifcation.
				 */
				/* XXX frag's and qos frames */
				len = ds->ds_rxstat.rs_datalen;
				if (len >= sizeof (struct ieee80211_frame)) {
					pci_dma_sync_single_for_cpu(sc->sc_bdev,
					    bf->bf_skbaddr, len,
					    PCI_DMA_FROMDEVICE);
					ieee80211_notify_michael_failure(ic,
						(struct ieee80211_frame *) skb->data,
					    sc->sc_splitmic ?
					        ds->ds_rxstat.rs_keyix-32 :
					        ds->ds_rxstat.rs_keyix
					);
				}
			}

			// TODO: correct?
			ic->ic_devstats->rx_errors++;

			/*
			 * accept error frames on the raw device
			 * or in monitor mode if we ask for them.
			 * we'll explicity drop them after capture.
			 */
			if (sc->sc_rxfilter & AR5K_RX_FILTER_PHYERROR)
				goto rx_accept;

			/*
			 * Reject error frames, we normally don't want
			 * to see them in monitor mode (in monitor mode
			 * allow through packets that have crypto problems).
			 */
			if ((ds->ds_rxstat.rs_status &~
			     (AR5K_RXERR_DECRYPT|AR5K_RXERR_MIC)) ||
			    sc->sc_ic.ic_opmode != IEEE80211_M_MONITOR)
				goto rx_next;
		}
rx_accept:
		/*
		 * Sync and unmap the frame.  At this point we're
		 * committed to passing the sk_buff somewhere so
		 * clear buf_skb; this means a new sk_buff must be
		 * allocated when the rx descriptor is setup again
		 * to receive another frame.
		 */
		len = ds->ds_rxstat.rs_datalen;
		pci_dma_sync_single_for_cpu(sc->sc_bdev,
			bf->bf_skbaddr, len, PCI_DMA_FROMDEVICE);
		pci_unmap_single(sc->sc_bdev, bf->bf_skbaddr,
			sc->sc_rxbufsize, PCI_DMA_FROMDEVICE);
		bf->bf_skb = NULL;

		sc->sc_stats.ast_ant_rx[ds->ds_rxstat.rs_antenna]++;
		
		if (len < IEEE80211_ACK_LEN) {
			DPRINTF(sc, ATH_DEBUG_RECV,
				"%s: runt packet %d\n", __func__, len);
			sc->sc_stats.ast_rx_tooshort++;
			dev_kfree_skb(skb);
			goto rx_next;
		}

		KASSERT(len <= skb_tailroom(skb), 
			("not enough tailroom (%d vs %d)",
			 len, skb_tailroom(skb)));

		skb_put(skb, len);
		skb->protocol = __constant_htons(ETH_P_CONTROL);

		if (sc->sc_rawdev_enabled && 
		    (sc->sc_rawdev.flags & IFF_UP)) {
			struct sk_buff *skb2;
			skb2 = skb_copy(skb, GFP_ATOMIC);
			if (skb2) {
				ath_rx_capture(&sc->sc_rawdev, ds, skb2);
			}
		}
		
		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
			/*
			 * Monitor mode: discard anything shorter than
			 * an ack or cts, clean the skbuff, fabricate
			 * the Prism header existing tools expect,
			 * and dispatch.
			 */
			/* XXX TSF */

			ath_rx_capture(dev, ds, skb);
			goto rx_next;
		}


		/*
		 * At this point we have no need for error frames
		 * that aren't crypto problems since we're done
		 * with capture.
		 */
		if (ds->ds_rxstat.rs_status &~
		    (AR5K_RXERR_DECRYPT|AR5K_RXERR_MIC)) {
			dev_kfree_skb(skb);
			goto rx_next;
		}
		/*
		 * From this point on we assume the frame is at least
		 * as large as ieee80211_frame_min; verify that.
		 */
		if (len < IEEE80211_MIN_LEN) {
			DPRINTF(sc, ATH_DEBUG_RECV, "%s: short packet %d\n",
				__func__, len);
			sc->sc_stats.ast_rx_tooshort++;
			dev_kfree_skb(skb);
			goto rx_next;
		}

		/*
		 * Normal receive.
		 */
		if (IFF_DUMPPKTS(sc, ATH_DEBUG_RECV)) {
			ieee80211_dump_pkt(skb->data, len,
				   sc->sc_hwmap[ds->ds_rxstat.rs_rate].ieeerate,
				   ds->ds_rxstat.rs_rssi);
		}

		skb_trim(skb, skb->len - IEEE80211_CRC_LEN);

		/*
		 * Locate the node for sender, track state, and then
		 * pass the (referenced) node up to the 802.11 layer
                 * for its use.  If the sender is unknown spam the
                 * frame; it'll be dropped where it's not wanted.
                 */
                if (ds->ds_rxstat.rs_keyix != AR5K_RXKEYIX_INVALID &&
                    (ni = sc->sc_keyixmap[ds->ds_rxstat.rs_keyix]) != NULL) {
                        /*
                         * Fast path: node is present in the key map;
                         * grab a reference for processing the frame.
                         */
			DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: node %s in keymap\n",
				__func__, ether_sprintf(ni->ni_macaddr));

                        an = ATH_NODE(ieee80211_ref_node(ni));
                        ATH_RSSI_LPF(an->an_avgrssi, ds->ds_rxstat.rs_rssi);
                        type = ieee80211_input(ic, skb, ni,
                                ds->ds_rxstat.rs_rssi, ds->ds_rxstat.rs_tstamp);
                } else {
                        /*
                         * Locate the node for sender, track state, and then
                         * pass the (referenced) node up to the 802.11 layer
                         * for its use.
                         */
                        ni = ieee80211_find_rxnode(ic,
                    		(struct ieee80211_frame_min *)skb->data);
                        /*
                         * Track rx rssi and do any rx antenna management.
                         */
                        an = ATH_NODE(ni);
                        ATH_RSSI_LPF(an->an_avgrssi, ds->ds_rxstat.rs_rssi);
                        /*
                         * Send frame up for processing.
                         */
                        type = ieee80211_input(ic, skb, ni,
                                ds->ds_rxstat.rs_rssi, ds->ds_rxstat.rs_tstamp);
                        if (ni != ic->ic_bss) {
                                u_int16_t keyix;
                                /*
                                 * If the station has a key cache slot assigned
                                 * update the key->node mapping table.
                                 */
                                keyix = ni->ni_ucastkey.wk_keyix;
                                if (keyix != IEEE80211_KEYIX_NONE &&
                                    sc->sc_keyixmap[keyix] == NULL)
                                        sc->sc_keyixmap[keyix] =
                                                ieee80211_ref_node(ni);
                        }
                }
                ieee80211_free_node(ni);
		if (sc->sc_diversity) {
			/*
			 * When using fast diversity, change the default rx
			 * antenna if diversity chooses the other antenna 3
			 * times in a row.
			 */
			if (sc->sc_defant != ds->ds_rxstat.rs_antenna) {
				if (++sc->sc_rxotherant >= 3)
					ath_setdefantenna(sc,
						ds->ds_rxstat.rs_antenna);
			} else
				sc->sc_rxotherant = 0;
		}

		if (sc->sc_softled) {
			/*
			 * Blink for any data frame.  Otherwise do a
			 * heartbeat-style blink when idle.  The latter
			 * is mainly for station mode where we depend on
			 * periodic beacon frames to trigger the poll event.
			 */
			if (type == IEEE80211_FC0_TYPE_DATA) {
				sc->sc_rxrate = ds->ds_rxstat.rs_rate;
				ath_led_event(sc, ATH_LED_RX);
			} else if (jiffies - sc->sc_ledevent >= sc->sc_ledidle)
				ath_led_event(sc, ATH_LED_POLL);
		}
rx_next:
		STAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
	} while (ath_rxbuf_init(sc, bf) == 0);

	/* rx signal state monitoring */
	ath_hal_rxmonitor(ah, &ATH_NODE(ic->ic_bss)->an_halstats);

#undef PA2DESC
}

/*
 * Deferred processing of transmit interrupt; special-cased
 * for a single hardware transmit queue (e.g. 5210 and 5211).
 */
static void
ath_tx_tasklet_q0(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	ath_tx_processq(sc, &sc->sc_txq[0]);
	ath_tx_processq(sc, sc->sc_cabq);

	sc->sc_tx_timer = 0;

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);

        // TODO: okay???
	/*
	 * Don't wakeup unless we're associated; this insures we don't
	 * signal the upper layer it's ok to start sending data frames.
	 */
	/* XXX use a low watermark to reduce wakeups */
	if (ic->ic_state == IEEE80211_S_RUN)
		netif_wake_queue(dev);

	if (sc->sc_rawdev_enabled)
		netif_wake_queue(&sc->sc_rawdev);
}

static void
ath_beacon_tasklet(unsigned long data)
{
	struct ath_softc *sc = (struct ath_softc *)data;
	int needmark;
	ath_beacon_send(sc, &needmark);
}


/*
 * Setup a hardware transmit queue.
 */
static struct ath_txq *
ath_txq_setup(struct ath_softc *sc, int qtype, int subtype)
{
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;
	int qnum;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = subtype;
	qi.tqi_aifs = AR5K_TXQ_USEDEFAULT;
	qi.tqi_cw_min = AR5K_TXQ_USEDEFAULT;
	qi.tqi_cw_max = AR5K_TXQ_USEDEFAULT;
	/*
	 * Enable interrupts only for EOL and DESC conditions.
	 * We mark tx descriptors to receive a DESC interrupt
	 * when a tx queue gets deep; otherwise waiting for the
	 * EOL to reap descriptors.  Note that this is done to
	 * reduce interrupt load and this only defers reaping
	 * descriptors, never transmitting frames.  Aside from
	 * reducing interrupts this also permits more concurrency.
	 * The only potential downside is if the tx queue backs
	 * up in which case the top half of the kernel may backup
	 * due to a lack of tx descriptors.
	 */
	qi.tqi_flags = AR5K_TXQ_FLAG_TXEOLINT_ENABLE | AR5K_TXQ_FLAG_TXDESCINT_ENABLE;
	qnum = ath_hal_setuptxqueue(ah, qtype, &qi);
	if (qnum == -1) {
		/*
		 * NB: don't print a message, this happens
		 * normally on parts with too few tx queues
		 */
		return NULL;
	}
	if (qnum >= ARRAY_SIZE(sc->sc_txq)) {
		printk("%s: hal qnum %u out of range, max %u!\n",
			sc->sc_dev.name, qnum, 
			(unsigned int) ARRAY_SIZE(sc->sc_txq));
		ath_hal_releasetxqueue(ah, qnum);
		return NULL;
	}
	if (!ATH_TXQ_SETUP(sc, qnum)) {
		struct ath_txq *txq = &sc->sc_txq[qnum];

		txq->axq_qnum = qnum;
		txq->axq_depth = 0;
		txq->axq_intrcnt = 0;
		txq->axq_link = NULL;
		STAILQ_INIT(&txq->axq_q);
		spin_lock_init(&(txq)->axq_lock);
		sc->sc_txqsetup |= 1<<qnum;
	}
	return &sc->sc_txq[qnum];
}

/*
 * Setup a hardware data transmit queue for the specified
 * access control.  The hal may not support all requested
 * queues in which case it will return a reference to a
 * previously setup queue.  We record the mapping from ac's
 * to h/w queues for use by ath_tx_start and also track
 * the set of h/w queues being used to optimize work in the
 * transmit interrupt handler and related routines.
 */
static int
ath_tx_setup(struct ath_softc *sc, int ac, int haltype)
{
	struct ath_txq *txq;

	if (ac >= ARRAY_SIZE(sc->sc_ac2q)) {
		printk("%s: AC %u out of range, max %u!\n",
			sc->sc_dev.name, ac, 
			(unsigned int) ARRAY_SIZE(sc->sc_ac2q));
		return 0;
	}
	txq = ath_txq_setup(sc, AR5K_TX_QUEUE_DATA, haltype);
	if (txq != NULL) {
		sc->sc_ac2q[ac] = txq;
		return 1;
	} else
		return 0;
}

/*
 * Update WME parameters for a transmit queue.
 */
static int
ath_txq_update(struct ath_softc *sc, int ac)
{
#define	ATH_EXPONENT_TO_VALUE(v)	((1<<v)-1)
#define	ATH_TXOP_TO_US(v)		(v<<5)
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_txq *txq = sc->sc_ac2q[ac];
	struct wmeParams *wmep = &ic->ic_wme.wme_chanParams.cap_wmeParams[ac];
	struct ath_hal *ah = sc->sc_ah;
	AR5K_TXQ_INFO qi;

	ath_hal_gettxqueueprops(ah, txq->axq_qnum, &qi);
	qi.tqi_aifs = wmep->wmep_aifsn;
	qi.tqi_cw_min = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmin);
	qi.tqi_cw_max = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmax);	
	qi.tqi_burst_time = ATH_TXOP_TO_US(wmep->wmep_txopLimit);

	if (!ath_hal_settxqueueprops(ah, txq->axq_qnum, &qi)) {
		if_printf(&sc->sc_dev, "unable to update hardware queue "
			"parameters for %s traffic!\n",
			ieee80211_wme_acnames[ac]);
		return 0;
	} else {
		ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
		return 1;
	}
#undef ATH_TXOP_TO_US
#undef ATH_EXPONENT_TO_VALUE
}

