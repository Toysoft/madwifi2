/*
 * Copyright (c) 2002, 2003 Sam Leffler.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Sam Leffler ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL John Hay BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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

#include <linux/pci.h>

#include "if_athvar.h"

/*
 * Much of this code is cribbed from Jouni Malinens hostap driver.
 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0))
/*
 * PCI initialization uses Linux 2.4.x version and
 * older kernels do not support this
 */
#error Atheros PCI version requires at least Linux kernel version 2.4.0
#endif /* kernel < 2.4.0 */

struct ath_pci_softc {
	struct ath_softc	aps_sc;
	struct module		*aps_module;
#ifdef CONFIG_PM
	u32			aps_pmstate[16];
#endif
};

/*
 * User a static table of PCI id's for now.  While this is the
 * "new way" to do things, we may want to switch back to having
 * the HAL check them by defining a probe method.
 */
static struct pci_device_id ath_pci_id_table[] __devinitdata = {
	{ 0x168c, 0x0007, PCI_ANY_ID, PCI_ANY_ID },
	{ 0x168c, 0x0011, PCI_ANY_ID, PCI_ANY_ID },
	{ 0x168c, 0x0013, PCI_ANY_ID, PCI_ANY_ID },
	{ 0 }
};

static int
ath_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	unsigned long phymem;
	unsigned long mem;
	struct ath_pci_softc *sc;
	struct net_device *dev;

	if (pci_enable_device(pdev))
		return (-EIO);

	/* XXX 32-bit addressing only */
	if (pci_set_dma_mask(pdev, 0xffffffff)) {
		printk(KERN_ERR "ath_pci: 32-bit DMA not available\n");
		goto bad;
	}

	pci_set_master(pdev);

	phymem = pci_resource_start(pdev, 0);
	if (!request_mem_region(phymem, pci_resource_len(pdev, 0), "ath")) {
		printk(KERN_ERR "ath_pci: cannot reserve PCI memory region\n");
		goto bad;
	}

	mem = (unsigned long) ioremap(phymem, pci_resource_len(pdev, 0));
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

	sc->aps_module = THIS_MODULE;
	dev = &sc->aps_sc.sc_ic.ic_dev;	/* XXX blech, violate layering */
	memcpy(dev->name, "ath%d", sizeof("ath%d"));

	dev->irq = pdev->irq;
	dev->mem_start = mem;
	dev->mem_end = mem + pci_resource_len(pdev, 0);
	dev->priv = sc;

	sc->aps_sc.sc_pci_dev = pdev;

	pci_set_drvdata(pdev, dev);

	if (request_irq(dev->irq, ath_intr, SA_SHIRQ, dev->name, dev)) {
		printk(KERN_WARNING "%s: request_irq failed\n", dev->name);
		goto bad3;
	}

	if (ath_attach(id->device, dev) != 0)
		goto bad4;

	printk(KERN_INFO "%s: Atheros PCI: mem=0x%lx, irq=%d\n",
		dev->name, phymem, dev->irq);

	return 0;
bad4:
	free_irq(dev->irq, dev);
bad3:
	kfree(sc);
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
	struct ath_pci_softc *sc = dev->priv;

	ath_detach(dev);
	if (dev->irq)
		free_irq(dev->irq, dev);
	iounmap((void *) dev->mem_start);
	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));
	pci_disable_device(pdev);
	kfree(sc);
}

#ifdef CONFIG_PM
static int
ath_pci_suspend(struct pci_dev *pdev, u32 state)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	struct ath_pci_softc *sc = dev->priv;

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
	struct ath_pci_softc *sc = dev->priv;

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

/*
 * Module glue.
 */
#include "version.h"
static char *version = ATH_PCI_VERSION " (Sam Leffler <sam@errno.com>)";
static char *dev_info = "ath_pci";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("Support for Atheros 802.11 wireless LAN PCI cards.");
MODULE_SUPPORTED_DEVICE("Atheros WLAN PCI cards");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");		/* XXX really BSD only */
#endif

static int __init
init_ath_pci(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);

	if (pci_register_driver(&ath_pci_drv_id) <= 0) {
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
