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

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,44))
#include <linux/tqueue.h>
#else
#include <linux/workqueue.h>
#endif

#include <linux/ioport.h>
#include <linux/pci.h>

#include "if_athvar.h"

/*
 * Much of this code is cribbed from Jouni Malinens hostap driver.
 */

static char *version = ATH_VERSION " (Sam Leffler <sam@errno.com>)";
static char *dev_info = "ath_pci";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("Support for Atheros 802.11 wireless LAN PCI cards.");
MODULE_SUPPORTED_DEVICE("Atheros WLAN PCI cards");
#ifdef MODULE_LICENSE
MODULE_LICENSE("BSD");
#endif


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0))
/*
 * PCI initialization uses Linux 2.4.x version and
 * older kernels do not support this
 */
#error Atheros PCI version requires at least Linux kernel version 2.4.0
#endif /* kernel < 2.4.0 */


/* FIX: do we need mb/wmb/rmb with memory operations? */

static struct pci_device_id atheros_pci_id_table[] __devinitdata = {
	{ 0x168c, 0x0007, PCI_ANY_ID, PCI_ANY_ID },
	{ 0x168c, 0x0011, PCI_ANY_ID, PCI_ANY_ID },
	{ 0x168c, 0x0013, PCI_ANY_ID, PCI_ANY_ID },
	{ 0 }
};

static int
ath_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	unsigned long phymem;
	unsigned long mem = 0;
	local_info_t *local = NULL;
	struct net_device *dev = NULL;
	static int cards_found /* = 0 */;
	int irq_registered = 0;

	if (pci_enable_device(pdev))
		return (-EIO);

	phymem = pci_resource_start(pdev, 0);
	if (!request_mem_region(phymem, pci_resource_len(pdev, 0), "Atheros")) {
		printk(KERN_ERR "ath_pci_probe: cannot reserve PCI memory region\n");
		goto err_out_disable;
	}

	mem = (unsigned long) ioremap(phymem, pci_resource_len(pdev, 0));
	if (!mem) {
		printk(KERN_ERR "ath_pci_probe: cannot remap PCI memory region\n") ;
		goto fail;
	}

	pci_set_master(pdev);

	local = ath_init_local_data(&ath_pci_funcs, cards_found);
	if (local == NULL)
		goto fail;
	cards_found++;

	dev = local->dev;

        dev->irq = pdev->irq;
        dev->mem_start = mem;
        dev->mem_end = mem + pci_resource_len(pdev, 0);

	if (ath_init_dev(local))
		goto fail;

	ath_pci_cor_sreset(local);

	pci_set_drvdata(pdev, dev);

	if (request_irq(dev->irq, ath_interrupt, SA_SHIRQ, dev->name, dev)) {
		printk(KERN_WARNING "%s: request_irq failed\n", dev->name);
		goto fail;
	} else
		irq_registered = 1;

	if (ath_hw_config(dev, 1)) {
		printk(KERN_DEBUG "%s: hardware initialization failed\n",
		       dev_info);
		goto fail;
	}

	printk(KERN_INFO "%s: Atheros PCI: mem=0x%lx, irq=%d\n",
		dev->name, phymem, dev->irq);
	return 0;
fail:
	ath_free_local_data(local);
	if (irq_registered && dev)
		free_irq(dev->irq, dev);
	if (mem)
		iounmap((void *) mem);
	release_mem_region(phymem, pci_resource_len(pdev, 0));

err_out_disable:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,4))
	pci_disable_device(pdev);
#endif
	return (-ENODEV);
}

static void
ath_pci_remove(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	local_info_t *local = (local_info_t *) dev->priv;
	unsigned long mem_start;

	/* Reset the hardware, and ensure interrupts are disabled. */
	ath_pci_cor_sreset(local);
	hfa384x_disable_interrupts(dev);

	if (dev->irq)
		free_irq(dev->irq, dev);

	mem_start = dev->mem_start;
	ath_free_local_data(local);

	iounmap((void *) mem_start);

	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,4))
	pci_disable_device(pdev);
#endif
}

#ifdef CONFIG_PM
static int
ath_pci_suspend(struct pci_dev *pdev, u32 state)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	local_info_t *local = (local_info_t *) dev->priv;

	if (netif_running(dev)) {
		hostap_netif_stop_queues(dev);
		netif_device_detach(dev);
	}
	ath_hw_shutdown(dev, 0);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,6))
	pci_save_state(pdev, local->pci_save_state);
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,4))
	pci_disable_device(pdev);
#endif
	pci_set_power_state(pdev, 3);

	return (0);
}

static int
ath_pci_resume(struct pci_dev *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);
	local_info_t *local = (local_info_t *) dev->priv;

	pci_enable_device(pdev);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,6))
	pci_restore_state(pdev, local->pci_save_state);
#endif
	ath_hw_config(dev, 0);
	if (netif_running(dev)) {
		netif_device_attach(dev);
		netif_start_queue(dev);
	}

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

static int
__init init_ath_pci(void)
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

static void
__exit exit_ath_pci(void)
{
	pci_unregister_driver(&ath_pci_drv_id);
	printk(KERN_INFO "%s: Driver unloaded\n", dev_info);
}
module_exit(exit_ath_pci);
