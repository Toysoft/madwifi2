/*-
 * Copyright (c) 2002-2006 Sam Leffler, Errno Consulting
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

/* Don't use virtualized timer in Linux 2.6.20+ */
#define USE_REAL_TIME_DELAY

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <linux/sysctl.h>
#include <linux/proc_fs.h>

#include <asm/io.h>

#include <ah.h>
#include <ah_os.h>

#ifdef AH_DEBUG
static	int ath_hal_debug = 0;
#endif

int	ath_hal_dma_beacon_response_time = 2;	/* in TUs */
int	ath_hal_sw_beacon_response_time = 10;	/* in TUs */
int	ath_hal_additional_swba_backoff = 0;	/* in TUs */

struct ath_hal *
_ath_hal_attach(u_int16_t devid, HAL_SOFTC sc,
		HAL_BUS_TAG t, HAL_BUS_HANDLE h, HAL_STATUS *s)
{
	struct ath_hal *ah = ath_hal_attach(devid, sc, t, h, s);

	if (ah)
#ifndef __MOD_INC_USE_COUNT
		if (!try_module_get(THIS_MODULE)) {
			printk(KERN_WARNING "%s: try_module_get failed\n",
					__func__);
			_ath_hal_detach(ah);
			return NULL;
		}
#else
		MOD_INC_USE_COUNT;
#endif
	return ah;
}

void
_ath_hal_detach(struct ath_hal *ah)
{
	(*ah->ah_detach)(ah);
#ifndef __MOD_INC_USE_COUNT
	module_put(THIS_MODULE);
#else
	MOD_DEC_USE_COUNT;
#endif
}

/*
 * Print/log message support.
 */

void __ahdecl
ath_hal_vprintf(struct ath_hal *ah, const char* fmt, va_list ap)
{
	char buf[1024];					/* XXX */
	vsnprintf(buf, sizeof(buf), fmt, ap);
	printk("%s", buf);
}

void __ahdecl
ath_hal_printf(struct ath_hal *ah, const char* fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	ath_hal_vprintf(ah, fmt, ap);
	va_end(ap);
}
EXPORT_SYMBOL(ath_hal_printf);

/*
 * Format an Ethernet MAC for printing.
 */
const char* __ahdecl
ath_hal_ether_sprintf(const u_int8_t *mac)
{
	static char etherbuf[18];
	snprintf(etherbuf, sizeof(etherbuf), "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return etherbuf;
}

#ifdef AH_ASSERT
void __ahdecl
ath_hal_assert_failed(const char* filename, int lineno, const char *msg)
{
	printk("Atheros HAL assertion failure: %s: line %u: %s\n",
		filename, lineno, msg);
	panic("ath_hal_assert");
}
#endif /* AH_ASSERT */

#ifdef AH_DEBUG
/* Store the current function name (should be called by wrapper functions)
 * useful for debugging and figuring out, which hal function sets which 
 * registers */
char *ath_hal_func = NULL;
EXPORT_SYMBOL(ath_hal_func);
#endif

#ifdef AH_DEBUG_ALQ
/*
 * ALQ register tracing support.
 *
 * Setting hw.ath.hal.alq=1 enables tracing of all register reads and
 * writes to the file /tmp/ath_hal.log.  The file format is a simple
 * fixed-size array of records.  When done logging set hw.ath.hal.alq=0
 * and then decode the file with the ardecode program (that is part of the
 * HAL).  If you start+stop tracing the data will be appended to an
 * existing file.
 *
 * NB: doesn't handle multiple devices properly; only one DEVICE record
 *     is emitted and the different devices are not identified.
 */
#include "alq.h"

static	struct alq *ath_hal_alq = NULL;
static	u_int ath_hal_alq_lost;		/* count of lost records */
static	const char *ath_hal_logfile = "/tmp/ath_hal.log";
static	u_int ath_hal_alq_qsize = 8*1024;

#define		MSG_MAXLEN	64

static int
ath_hal_setlogging(int enable)
{
	int error;

	if (enable) {
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		error = alq_open(&ath_hal_alq, ath_hal_logfile,
				MSG_MAXLEN, ath_hal_alq_qsize, (enable == 1 ? 0x7fffffff : enable));
		ath_hal_alq_lost = 0;
		printk("ath_hal: logging to %s %s\n", ath_hal_logfile,
			error == 0 ? "enabled" : "could not be setup");
	} else {
		if (ath_hal_alq)
			alq_close(ath_hal_alq);
		ath_hal_alq = NULL;
		printk("ath_hal: logging disabled\n");
		error = 0;
	}
	return error;
}

/*
 * Deal with the sysctl handler api changing.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define	AH_SYSCTL_ARGS_DECL \
	ctl_table *ctl, int write, struct file *filp, void *buffer, \
		size_t *lenp
#define	AH_SYSCTL_ARGS		ctl, write, filp, buffer, lenp
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8) */
#define	AH_SYSCTL_ARGS_DECL \
	ctl_table *ctl, int write, struct file *filp, void *buffer,\
		size_t *lenp, loff_t *ppos
#define	AH_SYSCTL_ARGS		ctl, write, filp, buffer, lenp, ppos
#endif

static int
sysctl_hw_ath_hal_log(AH_SYSCTL_ARGS_DECL)
{
	int error, enable;

	ctl->data = &enable;
	ctl->maxlen = sizeof(enable);
	enable = (ath_hal_alq != NULL);
	error = proc_dointvec(AH_SYSCTL_ARGS);
	if (error || !write)
		return error;
	else
		return ath_hal_setlogging(enable);
}

void ath_hal_logprintf(const char *fmt, ...)
{
	va_list ap;
	struct ale *ale;
	
	if (!ath_hal_alq)
		return;

	ale = alq_get(ath_hal_alq, ALQ_NOWAIT);
	if (!ale) {
		ath_hal_alq_lost++;
		return;
	}

	memset(ale->ae_data, 0, MSG_MAXLEN);
	va_start(ap, fmt);
	vsnprintf(ale->ae_data, MSG_MAXLEN, fmt, ap);
	va_end(ap);

	alq_post(ath_hal_alq, ale);
}
EXPORT_SYMBOL(ath_hal_logprintf);

static void ath_hal_logmsg(struct ath_hal *ah, u8 write, u_int reg, u_int32_t val)
{
	struct ale *ale;
	
	if (!ath_hal_alq)
		return;

	ale = alq_get(ath_hal_alq, ALQ_NOWAIT);
	if (!ale) {
		ath_hal_alq_lost++;
		return;
	}
	
	memset(ale->ae_data, 0, MSG_MAXLEN);
	sprintf(ale->ae_data, "%s:0x%05x = 0x%08x - %s\n", (write ? "W" : "R"), reg, val, (ath_hal_func ?: "unknown"));
	alq_post(ath_hal_alq, ale);
}

#elif defined(AH_DEBUG) || defined(AH_REGOPS_FUNC)

static void ath_hal_logmsg(struct ath_hal *ah, u8 write, u_int reg, u_int32_t val)
{
	if (!ah)
		return;
	ath_hal_printf(ah, "%s:0x%04x = 0x%08x - %s\n", (write ? "W" : "R"), reg, val, (ath_hal_func ?: "unknown"));
}


/*
 * Memory-mapped device register read/write.  These are here
 * as routines when debugging support is enabled and/or when
 * explicitly configured to use function calls.  The latter is
 * for architectures that might need to do something before
 * referencing memory (e.g. remap an i/o window).
 *
 * This should only be called while holding the lock, sc->sc_hal_lock.
 *
 * NB: see the comments in ah_osdep.h about byte-swapping register
 *     reads and writes to understand what's going on below.
 */
void __ahdecl
ath_hal_reg_write(struct ath_hal *ah, u_int reg, u_int32_t val)
{
#ifdef AH_DEBUG
	if (ath_hal_debug > 1)
		ath_hal_logmsg(ah, 0, reg, val);
#endif
	_OS_REG_WRITE(ah, reg, val);
}
EXPORT_SYMBOL(ath_hal_reg_write);

/* This should only be called while holding the lock, sc->sc_hal_lock. */
u_int32_t __ahdecl
ath_hal_reg_read(struct ath_hal *ah, u_int reg)
{
 	u_int32_t val;

	val = _OS_REG_READ(ah, reg);
#ifdef AH_DEBUG
	if (ath_hal_debug > 1)
		ath_hal_logmsg(ah, 1, reg, val);
#endif
	return val;
}
EXPORT_SYMBOL(ath_hal_reg_read);
#endif /* AH_DEBUG || AH_REGOPS_FUNC */

#if 0
void __ahdecl
HALDEBUG(struct ath_hal *ah, const char* fmt, ...)
{
	if (ath_hal_debug) {
		__va_list ap;
		va_start(ap, fmt);
		ath_hal_vprintf(ah, fmt, ap);
		va_end(ap);
	}
}


void __ahdecl
HALDEBUGn(struct ath_hal *ah, u_int level, const char* fmt, ...)
{
	if (ath_hal_debug >= level) {
		__va_list ap;
		va_start(ap, fmt);
		ath_hal_vprintf(ah, fmt, ap);
		va_end(ap);
	}
}
#endif /* 0 */

/*
 * Delay n microseconds.
 */
void __ahdecl
ath_hal_delay(int n)
{
	udelay(n);
}

u_int32_t __ahdecl
ath_hal_getuptime(struct ath_hal *ah)
{
	return ((jiffies / HZ) * 1000) + (jiffies % HZ) * (1000 / HZ);
}
EXPORT_SYMBOL(ath_hal_getuptime);

/*
 * Allocate/free memory.
 */

void * __ahdecl
ath_hal_malloc(size_t size)
{
	void *p;
	p = kmalloc(size, GFP_KERNEL);
	if (p)
		OS_MEMZERO(p, size);
	return p;

}

void __ahdecl
ath_hal_free(void* p)
{
	kfree(p);
}

void __ahdecl
ath_hal_memzero(void *dst, size_t n)
{
	memset(dst, 0, n);
}
EXPORT_SYMBOL(ath_hal_memzero);

void * __ahdecl
ath_hal_memcpy(void *dst, const void *src, size_t n)
{
	return memcpy(dst, src, n);
}
EXPORT_SYMBOL(ath_hal_memcpy);

int __ahdecl
ath_hal_memcmp(const void *a, const void *b, size_t n)
{
	return memcmp(a, b, n);
}
EXPORT_SYMBOL(ath_hal_memcmp);

static ctl_table ath_hal_sysctls[] = {
#ifdef AH_DEBUG
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "debug",
	  .mode		= 0644,
	  .data		= &ath_hal_debug,
	  .maxlen	= sizeof(ath_hal_debug),
	  .proc_handler	= proc_dointvec
	},
#endif
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "dma_beacon_response_time",
	  .data		= &ath_hal_dma_beacon_response_time,
	  .maxlen	= sizeof(ath_hal_dma_beacon_response_time),
	  .mode		= 0644,
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "sw_beacon_response_time",
	  .mode		= 0644,
	  .data		= &ath_hal_sw_beacon_response_time,
	  .maxlen	= sizeof(ath_hal_sw_beacon_response_time),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "swba_backoff",
	  .mode		= 0644,
	  .data		= &ath_hal_additional_swba_backoff,
	  .maxlen	= sizeof(ath_hal_additional_swba_backoff),
	  .proc_handler	= proc_dointvec
	},
#ifdef AH_DEBUG_ALQ
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "alq",
	  .mode		= 0644,
	  .proc_handler	= sysctl_hw_ath_hal_log
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "alq_size",
	  .mode		= 0644,
	  .data		= &ath_hal_alq_qsize,
	  .maxlen	= sizeof(ath_hal_alq_qsize),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "alq_lost",
	  .mode		= 0644,
	  .data		= &ath_hal_alq_lost,
	  .maxlen	= sizeof(ath_hal_alq_lost),
	  .proc_handler	= proc_dointvec
	},
#endif
	{ 0 }
};
static ctl_table ath_hal_table[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "hal",
	  .mode		= 0555,
	  .child	= ath_hal_sysctls
	}, { 0 }
};
static ctl_table ath_ath_table[] = {
	{ .ctl_name	= DEV_ATH,
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_hal_table
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{ .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_hal_sysctl_header;

static void
ath_hal_sysctl_register(void)
{
	static int initialized = 0;

	if (!initialized) {
		ath_hal_sysctl_header =
			ATH_REGISTER_SYSCTL_TABLE(ath_root_table);
		initialized = 1;
	}
}

static void
ath_hal_sysctl_unregister(void)
{
	if (ath_hal_sysctl_header)
		unregister_sysctl_table(ath_hal_sysctl_header);
}

/*
 * Module glue.
 */
#include "version.h"
static char *dev_info = "ath_hal";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("Atheros Hardware Access Layer (HAL)");
MODULE_SUPPORTED_DEVICE("Atheros WLAN devices");
#ifdef MODULE_VERSION
MODULE_VERSION(TARGET ": " ATH_HAL_VERSION);
#endif
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

EXPORT_SYMBOL(ath_hal_probe);
EXPORT_SYMBOL(_ath_hal_attach);
EXPORT_SYMBOL(_ath_hal_detach);
EXPORT_SYMBOL(ath_hal_init_channels);
EXPORT_SYMBOL(ath_hal_getwirelessmodes);
EXPORT_SYMBOL(ath_hal_computetxtime);
EXPORT_SYMBOL(ath_hal_mhz2ieee);
EXPORT_SYMBOL(ath_hal_process_noisefloor);

#ifdef MMIOTRACE
extern void (*kmmio_logmsg)(struct ath_hal *ah, u8 write, u_int reg, u_int32_t val);
#endif

static int __init
init_ath_hal(void)
{
	const char *sep;
	int i;
#ifdef MMIOTRACE
	kmmio_logmsg = ath_hal_logmsg;
#endif

	printk(KERN_INFO "%s: %s (", dev_info, ath_hal_version);
	sep = "";
	for (i = 0; ath_hal_buildopts[i] != NULL; i++) {
		printk("%s%s", sep, ath_hal_buildopts[i]);
		sep = ", ";
	}
	printk(")\n");
	ath_hal_sysctl_register();
	return (0);
}
module_init(init_ath_hal);

static void __exit
exit_ath_hal(void)
{
#ifdef MMIOTRACE
	kmmio_logmsg = NULL;
#endif
	ath_hal_sysctl_unregister();
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_hal);
