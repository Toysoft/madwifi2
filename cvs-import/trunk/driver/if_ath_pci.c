/*-
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
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
 * $Id$
 */
#include "opt_ah.h"

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/cache.h>

#include <linux/pci.h>

#include "if_athvar.h"
#include "if_ath_bus.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0))
/*
 * PCI initialization uses Linux 2.4.x version and
 * older kernels do not support this
 */
#error Atheros PCI version requires at least Linux kernel version 2.4.0
#endif /* kernel < 2.4.0 */

/*
 * User a static table of PCI id's for now.  While this is the
 * "new way" to do things, we may want to switch back to having
 * the HAL check them by defining a probe method.
 */
static struct pci_device_id ath_pci_id_table[] __devinitdata = {
	{ 0x168c, 0x0007, PCI_ANY_ID, PCI_ANY_ID },
	{ 0x168c, 0x0012, PCI_ANY_ID, PCI_ANY_ID },
	{ 0x168c, 0x0013, PCI_ANY_ID, PCI_ANY_ID },
	{ 0xa727, 0x0013, PCI_ANY_ID, PCI_ANY_ID },	/* 3com */
	{ 0x10b7, 0x0013, PCI_ANY_ID, PCI_ANY_ID },	/* 3com 3CRDAG675 */
	{ 0x168c, 0x1014, PCI_ANY_ID, PCI_ANY_ID },	/* IBM minipci 5212 */
	{ 0 }
};

static int
ath_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	unsigned long phymem;
	unsigned long mem;
	struct ath_bus_softc *sc;
	struct net_device *dev;
	const char *athname;
	u_int8_t csz;
#ifdef SOFTLED
	struct ath_softc *ath_sc;
#endif

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
		 * Linux 2.4.18 (at least) writes the cache line size
		 * register as a 16-bit wide register which is wrong.
		 * We must have this setup properly for rx buffer
		 * DMA to work so force a reasonable value here if it
		 * comes up zero.
		 */
		csz = L1_CACHE_BYTES / sizeof(u_int32_t);
		printk("ath_pci: cache line size not set; forcing %u\n", csz);
		pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, csz);
	}
	/*
	 * The default setting of latency timer yields poor results,
	 * set it to the value used by other systems.  It may be worth
	 * tweaking this setting more.
	 */
	pci_write_config_byte(pdev, PCI_LATENCY_TIMER, 0xa8);

	pci_set_master(pdev);

	phymem = pci_resource_start(pdev, 0);
	if (!request_mem_region(phymem, pci_resource_len(pdev, 0), "ath")) {
		printk(KERN_ERR "ath_pci: cannot reserve PCI memory region\n");
		goto bad;
	}

	mem = (unsigned long) ioremap(phymem, pci_resource_len(pdev, 0));
	if (!mem) {
		printk(KERN_ERR "ath_pci: cannot remap PCI memory region\n") ;
		release_mem_region(phymem, pci_resource_len(pdev, 0));
		goto bad1;
	}
	if ((sc = ath_bus_getsc()) == NULL) {
		goto bad2;
	}
	dev = &sc->aps_sc.sc_ic.ic_dev; /* XXX blech, violate layering */
	dev->irq = pdev->irq;
	dev->mem_start = mem;
	dev->mem_end = mem + pci_resource_len(pdev, 0);
	sc->aps_sc.sc_bdev = (void *) pdev;
	pci_set_drvdata(pdev, dev);

	if (request_irq(dev->irq, ath_intr, SA_SHIRQ, dev->name, dev)) {
		printk(KERN_WARNING "%s: request_irq failed\n", dev->name);
		goto bad3;
	}

	if (ath_attach(id->device, dev) != 0)
		goto bad4;

	athname = ath_hal_probe(id->vendor, id->device);
	printk(KERN_INFO "%s: %s: mem=0x%lx, irq=%d\n",
		dev->name, athname ? athname : "Atheros ???", phymem, dev->irq);
#ifdef SOFTLED
	ath_sc = dev->priv;
	ath_sc->sc_ic.ic_beaconCnt = 0;
	/* Check to see if we have an IBM card */
	printk ("Vendor = 0x%x   id=0x%x subsys: %d\n",id->vendor,id->device,
		id->subdevice);
	if ((id->vendor == 0x168c) && 
	    ((id->device == 0x1014) || (id->device==0x12))) {
		ath_sc->sc_ic.ic_ledPin = 0;
		ath_sc->sc_ic.ic_rfKillPin = 1;
	} else {
		ath_sc->sc_ic.ic_ledPin = 1;
		ath_sc->sc_ic.ic_rfKillPin = 0;
	}
	ath_hal_gpioCfgOutput(ath_sc->sc_ah, ath_sc->sc_ic.ic_ledPin);
	ath_hal_gpioSet(ath_sc->sc_ah,ath_sc->sc_ic.ic_ledPin,0);
	ath_sc->sc_ic.ic_caps |= IEEE80211_C_SOFTLED;
#endif
	
	/* ready to process interrupts */
	sc->aps_sc.sc_invalid = 0;

	return 0;
bad4:
	free_irq(dev->irq, dev);
bad3:
	ath_bus_freesc(sc);
bad2:
	iounmap((void *) mem);
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
	struct ath_bus_softc *sc = dev->priv;
	
	ath_detach(dev);
	if (dev->irq)
		free_irq(dev->irq, dev);
	iounmap((void *) dev->mem_start);
	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));
	pci_disable_device(pdev);
	ath_bus_freesc(sc);
}

#ifdef CONFIG_PM
static int
ath_pci_suspend(struct pci_dev *pdev, u32 state)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct ath_bus_softc *sc = dev->priv;
	
	ath_suspend(dev);
	pci_save_state(pdev, sc->aps_pmstate);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, 3);

	return (0);
}

static int
ath_pci_resume(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct ath_bus_softc *sc = dev->priv;

	pci_enable_device(pdev);
	pci_restore_state(pdev, sc->aps_pmstate);
	ath_resume(dev);

	return (0);
}
#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(pci, ath_pci_id_table);

static struct pci_driver ath_pci_drv_id = {
	.name		= "ath_pci",
	.id_table	= ath_pci_id_table,
	.probe		= ath_pci_probe,
	.remove		= ath_pci_remove,
#ifdef CONFIG_PM
	.suspend	= ath_pci_suspend,
	.resume		= ath_pci_resume,
#endif /* CONFIG_PM */
	/* Linux 2.4.6 has save_state and enable_wake that are not used here */
};

extern char *dev_info;
extern char *version;

/* PCI specific module initialization */
int 
init_ath(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);

	if (pci_register_driver(&ath_pci_drv_id) <= 0) {
		printk("ath_pci: No devices found, driver not installed.\n");
		pci_unregister_driver(&ath_pci_drv_id);
		return (-ENODEV);
	}
	return (0);
}

void
exit_ath(void)
{
	pci_unregister_driver(&ath_pci_drv_id);
}

/* return bus cachesize in 4B word units */
void
bus_read_cachesize(struct ath_softc *sc, u_int8_t *csz)
{
	pci_read_config_byte(sc->sc_bdev, PCI_CACHE_LINE_SIZE, csz);
}
