/*-
 * Copyright (c) 2002-2004 Sam Leffler, Errno Consulting
 * Copyright (c) 2004 Atheros Communications, Inc.
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

#include "if_athvar.h"
#include "if_ath_bus.h"

/*
 * Bus independent probe (attach) function.
 *
 * pdev is a device structure pointer (for pci, this is a struct pci_dev).
 */
struct ath_bus_softc *
ath_bus_getsc(void)
{
	struct ath_bus_softc *sc;
	struct net_device *dev;

	sc = kmalloc(sizeof(struct ath_bus_softc), GFP_KERNEL);
	if (sc == NULL) {
		printk(KERN_ERR "ath_dev_probe: no memory for device state\n");
		return (NULL);
	}
	memset(sc, 0, sizeof(struct ath_bus_softc));
	sc->aps_sc.sc_invalid = 1;
	dev = &sc->aps_sc.sc_ic.ic_dev;	/* XXX blech, violate layering */
	memcpy(dev->name, "ath%d", sizeof("ath%d"));
	dev->priv = sc;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,5,41)
	dev->owner = THIS_MODULE;
#else
	SET_MODULE_OWNER(dev);
#endif
	return (sc);
}

void
ath_bus_freesc(struct ath_bus_softc *sc)
{
        kfree(sc);
}
