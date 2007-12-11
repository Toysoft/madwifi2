/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) IBM Corporation, 2005
 *               Jeff Muizelaar, 2006
 *
 * Derived from the read-mod example from relay-examples by Tom Zanussi.
 */
#include <linux/module.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <linux/version.h>
#include <linux/kallsyms.h>
#include <asm/pgtable.h>

#include "kmmio.h"
#include "mmio-kernel-api.h"
#include "pf_in.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
enum {
	false   = 0,
	true    = 1
};
typedef _Bool bool;
#endif

void (*kmmio_logmsg)(void *ah, u8 write, u_int reg, u_int32_t val) = NULL;
EXPORT_SYMBOL(kmmio_logmsg);

struct {
	unsigned long addr;
	unsigned long ip;
	enum reason_type type;
	unsigned long val;
	int index;
	int active_traces;
} pf_reason[NR_CPUS] = { [0 ... NR_CPUS-1] = { 0, 0, NOT_ME, 0, 0, 0} };

static struct mm_io_header_rw cpu_trace[NR_CPUS];

/* This app's relay channel/control files will appear in /debug/mmio-trace */
#define APP_DIR		"mmio-trace"
/* the marker injection file in /proc */
#define MARKER_FILE     "mmio-marker"

#define MODULE_NAME     "mmio"

/*#define MAX_EVENT_SIZE	256*/

/* app data */
static int suspended = 0;
static struct proc_dir_entry *proc_marker_file;

/* module parameters */
static unsigned long filter_offset = 0;

module_param(filter_offset, ulong, 0);

static inline void print_pte(unsigned long address) 
{
	pgd_t *pgd = pgd_offset_k(address);
	pud_t *pud = pud_offset(pgd, address);
	pmd_t *pmd = pmd_offset(pud, address);
	if (pmd_large(*pmd)) {
		printk(KERN_EMERG MODULE_NAME
			": 4MB pages are not currently supported: %lx\n",
			address);
		BUG();
	}
	printk(KERN_DEBUG MODULE_NAME ": pte for 0x%lx: 0x%lx 0x%lx\n",
	       address, pte_val(*pte_offset_kernel(pmd, address)),
	       pte_val(*pte_offset_kernel(pmd, address)) & _PAGE_PRESENT);
};

void pre(struct kmmio_probe *p, struct pt_regs *regs, unsigned long addr)
{
	const unsigned long cpu = smp_processor_id();
	const unsigned long instptr = instruction_pointer(regs);
	const enum reason_type type = get_ins_type(instptr);

	/* it doesn't make sense to have more than one active trace per cpu */
	if (pf_reason[cpu].active_traces) {
		/* for some reason the pre/post pairs have been called in an
		   unmatched order */
		printk(KERN_EMERG MODULE_NAME
		       ": unexpected fault for address: %lx\n", addr);
		printk(KERN_EMERG "last fault for address: %lx\n",
		       pf_reason[cpu].addr);
		print_pte(addr);
#ifdef __i386__
		print_symbol(KERN_EMERG "faulting EIP is at %s\n", regs->eip);
		print_symbol(KERN_EMERG "last faulting EIP was at %s\n",
		             pf_reason[cpu].ip);
		printk(KERN_EMERG "eax: %08lx   ebx: %08lx   ecx: %08lx   "
		       "edx: %08lx\n", regs->eax, regs->ebx, regs->ecx,
		       regs->edx);
		printk(KERN_EMERG "esi: %08lx   edi: %08lx   ebp: %08lx   "
		       "esp: %08lx\n", regs->esi, regs->edi, regs->ebp,
		       regs->esp);
#else
		print_symbol(KERN_EMERG "faulting RIP is at %s\n", regs->rip);
		print_symbol(KERN_EMERG "last faulting RIP was at %s\n",
		             pf_reason[cpu].ip);
		printk(KERN_EMERG "rax: %016lx   rcx: %016lx   rdx: %016lx\n",
		       regs->rax, regs->rcx, regs->rdx);
		printk(KERN_EMERG "rsi: %016lx   rdi: %016lx   rbp: %016lx  "
		       " rsp: %016lx\n", regs->rsi, regs->rdi, regs->rbp,
		       regs->rsp);
#endif
		BUG();
	} else {
		pf_reason[cpu].active_traces++;
	}

	pf_reason[cpu].type = type;
	pf_reason[cpu].addr = addr;
	pf_reason[cpu].ip = instptr;

	cpu_trace[cpu].header.type = MMIO_MAGIC;
	cpu_trace[cpu].header.pid = 0;
	cpu_trace[cpu].header.data_len = sizeof(struct mm_io_rw);
	cpu_trace[cpu].rw.address = addr;

	/* only record the program counter when requested
	 * this may be needed for clean-room reverse engineering */
	cpu_trace[cpu].rw.pc = instptr;

	switch (type)
	{
		case REG_READ:
			cpu_trace[cpu].header.type |=
				(MMIO_READ << MMIO_OPCODE_SHIFT) |
				(get_ins_mem_width(instptr)
					<< MMIO_WIDTH_SHIFT);
			break;
		case REG_WRITE:
			cpu_trace[cpu].header.type |=
				(MMIO_WRITE << MMIO_OPCODE_SHIFT) |
				(get_ins_mem_width(instptr)
					<< MMIO_WIDTH_SHIFT);
			cpu_trace[cpu].rw.value =
				get_ins_reg_val(instptr, regs);
			break;
		case IMM_WRITE:
			cpu_trace[cpu].header.type |=
				(MMIO_WRITE << MMIO_OPCODE_SHIFT) |
				(get_ins_mem_width(instptr)
					<< MMIO_WIDTH_SHIFT);
			cpu_trace[cpu].rw.value = get_ins_imm_val(instptr);
			break;
		default:
		{
			unsigned char *ip = (unsigned char *)instptr;
			cpu_trace[cpu].header.type |=
				(MMIO_UNKNOWN_OP << MMIO_OPCODE_SHIFT);
			cpu_trace[cpu].rw.value = (*ip)<<16 |
				*(ip+1)<<8 | *(ip+2);
		}
	}

}

void post(struct kmmio_probe *p, unsigned long condition, struct pt_regs *regs)
{
	const unsigned long cpu = smp_processor_id();

	/* this should always return the active_trace count to 0 */
	pf_reason[cpu].active_traces--;
	if (pf_reason[cpu].active_traces) {
		printk(KERN_EMERG MODULE_NAME ": unexpected post handler");
		BUG();
	}

	switch (pf_reason[cpu].type) {
		case REG_READ:
			cpu_trace[cpu].rw.value =
				get_ins_reg_val(pf_reason[cpu].ip, regs);
			break;
		default:
			break;
	}
	/*if ((cpu_trace[cpu].header.type & MMIO_OPCODE_MASK) == 0)
		printk("post(): opcode missing!\n");*/
//	relay_write(chan, &cpu_trace[cpu], sizeof(struct mm_io_header_rw));
	if (kmmio_logmsg)
		kmmio_logmsg(NULL, (pf_reason[cpu].type != REG_READ), cpu_trace[cpu].rw.address - p->addr, cpu_trace[cpu].rw.value);
}

/**
 *	module init - creates channel management control files
 *
 *	Returns 0 on success, negative otherwise.
 */

struct remap_trace {
	struct list_head list;
	struct kmmio_probe probe;
};
static LIST_HEAD(trace_list);

static void do_ioremap_trace_core(unsigned long offset,
				unsigned long size, void __iomem *addr)
{
	struct remap_trace *trace = kmalloc(sizeof(*trace), GFP_KERNEL);
	*trace = (struct remap_trace) { .probe = {
		.addr = (unsigned long)addr,
		.len = size,
		.pre_handler = pre,
		.post_handler = post,
	}
	};

	list_add_tail(&trace->list, &trace_list);
	printk(KERN_DEBUG MODULE_NAME ": registering probe for %lx-%lx\n",
		offset, offset+size);
	register_kmmio_probe(&trace->probe);
	printk(KERN_DEBUG MODULE_NAME ": register done\n");
}

#define ISA_START_ADDRESS       0xa0000
#define ISA_END_ADDRESS         0x100000

static void __iomem *ioremap_trace_core(unsigned long offset,
				unsigned long size, void __iomem *addr)
{
	if ((filter_offset) && (offset != filter_offset))
		return addr;

	/* Don't trace the low PCI/ISA area, it's always mapped.. */
	if ((offset >= ISA_START_ADDRESS)
			&& (offset + size - 1 < ISA_END_ADDRESS)) {
		printk(KERN_NOTICE MODULE_NAME
			": Ignoring map of low PCI/ISA area (0x%lx-0x%lx)\n",
				offset, offset + size);
		return addr;
	}
	do_ioremap_trace_core(offset, size, addr);
	return addr;
}

void __iomem * __ioremap_trace(
	unsigned long offset, unsigned long size, unsigned long flags)
{
	void __iomem * p;
	printk(KERN_DEBUG MODULE_NAME ": __ioremap(0x%lx, 0x%lx, 0x%lx)\n",
			offset, size, flags);
	p = ioremap_trace_core(offset, size, __ioremap(offset, size, flags));
	/*printk(" = %p\n", p);*/
	return p;
}

void __iomem * ioremap_nocache_trace(unsigned long offset, unsigned long size)
{
	void __iomem * p;
	printk(KERN_DEBUG MODULE_NAME ": ioremap_nocache(0x%lx, 0x%lx)\n",
			offset, size);
	p = ioremap_trace_core(offset, size, ioremap_nocache(offset, size));
	/*printk(" = %p\n", p);*/
	return p;
}

void iounmap_trace(volatile void __iomem *addr)
{
	struct remap_trace *trace;
	struct remap_trace *tmp;

	list_for_each_entry_safe(trace, tmp, &trace_list, list) {
		if ((unsigned long)addr == trace->probe.addr) {
			unregister_kmmio_probe(&trace->probe);
			list_del(&trace->list);
			kfree(trace);
			break;
		}
	}
	iounmap(addr);
	printk(KERN_DEBUG MODULE_NAME ": Unmap %p done\n", addr);
}

EXPORT_SYMBOL(__ioremap_trace);
EXPORT_SYMBOL(ioremap_nocache_trace);
EXPORT_SYMBOL(iounmap_trace);

static int __init init(void)
{
	init_kmmio();

	return 0;
}

static void __exit cleanup(void)
{
	cleanup_kmmio();
}

module_init(init);
module_exit(cleanup);
MODULE_LICENSE("GPL");
