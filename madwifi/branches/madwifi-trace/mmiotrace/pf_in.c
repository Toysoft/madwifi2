/******************************************************************************
 *  Fault Injection Test harness (FI)
 *  Copyright (C) Intel Crop.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 ******************************************************************************
 */

/*  $Id: pf_in.c,v 1.1.1.1 2002/11/12 05:56:32 brlock Exp $ 
 *  Copyright by Intel Crop., 2002 
 *  Louis Zhuang (louis.zhuang@intel.com)
 */

/**
 * @file pf_hooks.c
 */

#include <linux/module.h>
/*
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/io.h>
*/
#define PREFIX_NAME "FI_PF"
/*#include <linux/fi/fi.h>
#include <linux/fi/fi_interface.h>
#include <linux/fi/fi_internal.h>
*/
#include "pf_in.h"

#define NR_ELEMENTS(array) (sizeof(array)/sizeof(array[0]))

#ifdef __i386__
/* IA32 Manual 3, 2-1 */
static unsigned char prefix_codes[] = {
	0xF0, 0xF2, 0xF3, 0x2E, 0x36, 0x3E, 0x26, 0x64,
	0x65, 0x2E, 0x3E, 0x66, 0x67
};
#else
static unsigned char prefix_codes[] = {
	0x66, 0x67, 0x2E, 0x3E, 0x26, 0x64, 0x65, 0x36,
	0xF0, 0xF3, 0xF2,
	/* REX Prefixes */
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
};
#endif

static int skip_prefix(unsigned char *addr, int *shorted, int *enlarged, int *rexr)
{
	int i;
	unsigned char *p = addr;
	*shorted = 0;
	*enlarged = 0;
	*rexr = 0;

restart:
	for (i = 0; i < NR_ELEMENTS(prefix_codes); i++) {
		if (*p == prefix_codes[i]) {
			if (*p == 0x66) *shorted = 1;
#ifdef __amd64__
			if ((*p & 0xf8) == 0x48) *enlarged = 1;
			if ((*p & 0xf4) == 0x44) *rexr = 1;
#endif
			p++;
			goto restart;
		}
	}

	return p-addr;
}

static int get_opcode(unsigned char *addr, unsigned int *opcode)
{
	int len;

	if (*addr == 0x0F) { /*0x0F is extension instruction*/
		*opcode = *(unsigned short*)addr;
		len = 2;
	} else {
		*opcode = *addr;
		len = 1;
	}

	return len;
}

#ifdef __i386__
/* IA32 Manual 3, 3-432*/
static unsigned int reg_rop[] = {0x8A, 0x8B, 0xB60F, 0xB70F, 0xBE0F, 0xBF0F};
static unsigned int reg_wop[] = {0x88, 0x89};
static unsigned int imm_wop[] = {0xC6, 0xC7};
#else
/* AMD64 Manual 3, Appendix A*/
static unsigned int reg_rop[] = {0x8A, 0x8B, 0xB60F, 0xB70F, 0xBE0F, 0xBF0F};
static unsigned int reg_wop[] = {0x88, 0x89};
static unsigned int imm_wop[] = {0xC6, 0xC7};
#endif

#define CHECK_OP_TYPE(opcode, array, type) \
	for(i = 0; i < NR_ELEMENTS(array); i++) { \
		if (array[i] == opcode) { \
			rv = type; \
			goto exit; \
		} \
	}

enum reason_type get_ins_type(unsigned long ins_addr)
{
	unsigned int opcode;
	unsigned char *p;
	int shorted, enlarged, rexr;
	int i;
	enum reason_type rv = OTHERS;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);

	CHECK_OP_TYPE(opcode, reg_rop, REG_READ);
	CHECK_OP_TYPE(opcode, reg_wop, REG_WRITE);
	CHECK_OP_TYPE(opcode, imm_wop, IMM_WRITE);

exit:
	return rv;
}
#undef CHECK_OP_TYPE

#ifdef __i386__
/* IA32 Manual 3, 3-432*/
static unsigned int rw8[] = {0x88, 0x8A, 0xC6};
static unsigned int rw32[]= {0x89, 0x8B, 0xC7, 0xB60F, 0xB70F, 0xBE0F, 0xBF0F};
#else
/* AMD64 */
static unsigned int rw8[] = {0xC6, 0x88, 0x8A};
static unsigned int rw32[] = {0xC7,0x89, 0x8B, 0xB60F, 0xB70F, 0xBE0F, 0xBF0F};
#endif

unsigned int get_ins_reg_width(unsigned long ins_addr)
{
	unsigned int opcode;
	unsigned char *p;
	int i, shorted, enlarged, rexr;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);
    
	for (i = 0; i < NR_ELEMENTS(rw8); i++)
		if (rw8[i] == opcode)
			return 1;

	for (i = 0; i < NR_ELEMENTS(rw32); i++)
		if (rw32[i] == opcode)
			return shorted ? 2 : (enlarged ? 8 : 4);

	PERROR("Unknow opcode=0x%02x\n", opcode);
	return 0;
}

#ifdef __i386__
static unsigned int mw8[] = {0x88, 0x8A, 0xC6, 0xB60F, 0xBE0F};
static unsigned int mw16[] = {0xB70F, 0xBF0F};
static unsigned int mw32[] = {0x89, 0x8B, 0xC7};
static unsigned int mw64[] = {};
#else
/* 8 bit only */
static unsigned int mw8[] = {0xC6, 0x88, 0x8A, 0xB60F, 0xBE0F};
/* 16 bit only */
static unsigned int mw16[] = {0xB70F, 0xBF0F};
/* 16 or 32 bit */
static unsigned int mw32[] = {0xC7};
/* 16, 32 or 64 bit */
static unsigned int mw64[] = {0x89, 0x8B};
#endif

unsigned int get_ins_mem_width(unsigned long ins_addr) {
	unsigned int opcode;
	unsigned char *p;
	int i, shorted, enlarged, rexr;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);

	for (i = 0; i < NR_ELEMENTS(mw8); i++)
		if (mw8[i] == opcode)
			return 1;

	for (i = 0; i < NR_ELEMENTS(mw16); i++)
		if (mw16[i] == opcode)
			return 2;

	for (i = 0; i < NR_ELEMENTS(mw32); i++)
		if (mw32[i] == opcode)
			return shorted ? 2 : 4;

	for (i = 0; i < NR_ELEMENTS(mw64); i++)
		if (mw64[i] == opcode)
			return shorted ? 2 : (enlarged ? 8 : 4);

	PERROR("Unknow opcode=0x%02x\n", opcode);
	return 0;

	/* ?!? */
	return get_ins_reg_width(ins_addr);
}

/* define register ident in mod/rm byte after undefine it in ptrace*/
#ifdef __i386__
#undef  EAX
#define EAX 0
#define ECX 1
#define EDX 2
#undef  EBX
#define EBX 3
#define ESP 4
#define EBP 5
#undef  ESI
#define ESI 6
#undef  EDI
#define EDI 7

#define AL 0
#define CL 1
#define DL 2
#define BL 3
#define AH 4
#define CH 5
#define DH 6
#define BH 7
#else
#undef RAX
#undef RBX
#undef RCX
#undef RDX
#undef RSI
#undef RDI
#undef RSP
#undef RBP

#define RAX 0
#define RCX 1
#define RDX 2
#define RBX 3
#define RSP 4
#define RBP 5
#define RSI 6
#define RDI 7
#define R8  8
#define R9  9
#define R10  10
#define R11  11
#define R12  12
#define R13  13
#define R14  14
#define R15  15

#define AL 0
#define CL 1
#define DL 2
#define BL 3
#define AH 4
#define CH 5
#define DH 6
#define BH 7
#define R8B 8
#define R9B 9
#define R10B 10
#define R11B 11
#define R12B 12
#define R13B 13
#define R14B 14
#define R15B 15
#endif

#ifdef __i386__
static unsigned char *get_reg_w8(int no, regs_t *regs)
{
	unsigned char *rv = NULL;

	switch (no) {
	case AL:
		rv = (unsigned char *)&regs->eax;
		break;
	case BL:
		rv = (unsigned char *)&regs->ebx;
		break;
	case CL:
		rv = (unsigned char *)&regs->ecx;
		break;
	case DL:
		rv = (unsigned char *)&regs->edx;
		break;
	case AH:
		rv = 1+(unsigned char *)&regs->eax;
		break;
	case BH:
		rv = 1+(unsigned char *)&regs->ebx;
		break;
	case CH:
		rv = 1+(unsigned char *)&regs->ecx;
		break;
	case DH:
		rv = 1+(unsigned char *)&regs->edx;
		break;
	default:
		PERROR("Error reg no# %d\n", no);
		break;
	}

	return rv;
}

static long *get_reg_w32(int no, regs_t *regs)
{
	long *rv = NULL;

	switch (no) {
	case EAX:
		rv = &regs->eax;
		break;
	case EBX:
		rv = &regs->ebx;
		break;
	case ECX:
		rv = &regs->ecx;
		break;
	case EDX:
		rv = &regs->edx;
		break;
	case ESP:
		rv = &regs->esp;
		break;
	case EBP:
		rv = &regs->ebp;
		break;
	case ESI:
		rv = &regs->esi;
		break;
	case EDI:
		rv = &regs->edi;
		break;
	default:
		PERROR("Error reg no# %d\n", no);
	}

	return rv;
}
#else // AMD64
unsigned char *get_reg_w8(int no, regs_t *regs)
{
	unsigned char *rv = NULL;

	switch (no) {
	case AL:
		rv = (unsigned char *)&regs->rax;
		break;
	case BL:
		rv = (unsigned char *)&regs->rbx;
		break;
	case CL:
		rv = (unsigned char *)&regs->rcx;
		break;
	case DL:
		rv = (unsigned char *)&regs->rdx;
		break;
	case AH:
		rv = 1+(unsigned char *)&regs->rax;
		break;
	case BH:
		rv = 1+(unsigned char *)&regs->rbx;
		break;
	case CH:
		rv = 1+(unsigned char *)&regs->rcx;
		break;
	case DH:
		rv = 1+(unsigned char *)&regs->rdx;
		break;
	case R8B:
		rv = (unsigned char *)&regs->r8;
		break;
	case R9B:
		rv = (unsigned char *)&regs->r9;
		break;
	case R10B:
		rv = (unsigned char *)&regs->r10;
		break;
	case R11B:
		rv = (unsigned char *)&regs->r11;
		break;
	case R12B:
		rv = (unsigned char *)&regs->r12;
		break;
	case R13B:
		rv = (unsigned char *)&regs->r13;
		break;
	case R14B:
		rv = (unsigned char *)&regs->r14;
		break;
	case R15B:
		rv = (unsigned char *)&regs->r15;
		break;
	default:
		PERROR("Error reg no# %d\n", no);
		break;
	}
	return rv;
}

static long *get_reg_w32(int no, regs_t *regs)
{
	long *rv = NULL;

	switch (no) {
	case RAX:
		rv = &regs->rax;
		break;
	case RBX:
		rv = &regs->rbx;
		break;
	case RCX:
		rv = &regs->rcx;
		break;
	case RDX:
		rv = &regs->rdx;
		break;
	case RSP:
		rv = &regs->rsp;
		break;
	case RBP:
		rv = &regs->rbp;
		break;
	case RSI:
		rv = &regs->rsi;
		break;
	case RDI:
		rv = &regs->rdi;
		break;
	case R8:
		rv = &regs->r8;
		break;
	case R9:
		rv = &regs->r9;
		break;
	case R10:
		rv = &regs->r10;
		break;
	case R11:
		rv = &regs->r11;
		break;
	case R12:
		rv = &regs->r12;
		break;
	case R13:
		rv = &regs->r13;
		break;
	case R14:
		rv = &regs->r14;
		break;
	case R15:
		rv = &regs->r15;
		break;
	default:
		PERROR("Error reg no# %d\n", no);
	}

	return rv;
}
#endif

unsigned long get_ins_reg_val(unsigned long ins_addr, regs_t *regs)
{
	unsigned int opcode;
	unsigned char mod_rm;
	int reg;
	unsigned char *p;
	int i, shorted, enlarged, rexr;
	unsigned long rv;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);
	for (i = 0; i < NR_ELEMENTS(reg_rop); i++)
		if (reg_rop[i] == opcode) {
			rv = REG_READ;
			goto do_work;
		}

	for (i = 0; i < NR_ELEMENTS(reg_wop); i++)
		if (reg_wop[i] == opcode) {
			rv = REG_WRITE;
			goto do_work;
		}

	PERROR("Not a register instruction, opcode=0x%02x\n", opcode);
	goto err;

do_work:
	mod_rm = *p;
	reg = ((mod_rm >> 3) & 0x7) | (rexr << 3);
	switch (get_ins_reg_width(ins_addr)) {
	case 1:
		return *get_reg_w8(reg, regs);

	case 2:
		return *(unsigned short*)get_reg_w32(reg, regs);

	case 4:
		return *(unsigned int*)get_reg_w32(reg, regs);

#ifdef __amd64__
	case 8:
		return *(unsigned long*)get_reg_w32(reg, regs);
#endif

	default:
		PERROR("Error width# %d\n", reg);
	}

err:
	return 0;
}


void set_ins_reg_val(unsigned long ins_addr, regs_t *regs, unsigned long val)
{
	unsigned int opcode;
	unsigned char mod_rm;
	int reg;
	unsigned char *p;
	int i, shorted, enlarged, rexr;
	unsigned long rv;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);
	for (i = 0; i < NR_ELEMENTS(reg_rop); i++)
		if (reg_rop[i] == opcode) {
			rv = REG_READ;
			goto do_work;
		}

	for (i = 0; i < NR_ELEMENTS(reg_wop); i++)
		if (reg_wop[i] == opcode) {
			rv = REG_WRITE;
			goto do_work;
		}

	PERROR("Not a register instruction, opcode=0x%02x\n", opcode);
	goto err;

do_work:
	mod_rm = *p;
	reg = ((mod_rm >> 3) & 0x7) | (rexr << 3);

	switch (get_ins_reg_width(ins_addr)) {
	case 1:
		*get_reg_w8(reg, regs) = val;
		break;

	case 2:
		*(unsigned short*)get_reg_w32(reg, regs) = val;
		break;

	case 4:
		*(unsigned int*)get_reg_w32(reg, regs) = val;
		break;

#ifdef __amd64__
	case 8:
		*(unsigned long*)get_reg_w32(reg, regs) = val;
		break;
#endif

	default:
		PERROR("Error width, reg=%d\n", reg);
	}
err:
	return;
}

unsigned long get_ins_imm_val(unsigned long ins_addr) {
	unsigned int opcode;
	unsigned char mod_rm;
	unsigned char mod;
	unsigned char *p;
	int i, shorted, enlarged, rexr;
	unsigned long rv;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);
	for (i = 0; i < NR_ELEMENTS(imm_wop); i++)
		if (imm_wop[i] == opcode) {
			rv = IMM_WRITE;
			goto do_work;
		}

	PERROR("Not a imm instruction, opcode=0x%02x\n", opcode);
	goto err;

do_work:
	mod_rm = *p;
	mod = mod_rm >> 6;
	p++;
	switch (mod) {
	case 0:
		/* if r/m is 5 we have a 32 disp (IA32 Manual 3, Table 2-2)  */
		/* AMD64: XXX Check for address size prefix? */
		if ((mod_rm & 0x7) == 0x5)
			p += 4;
		break;

	case 1:
		p += 1;
		break;

	case 2:
		p += 4;
		break;

	case 3:
	default:
		PERROR("it is not a memory access instruction at 0x%lx, rm_mod=0x%02x\n", ins_addr, mod_rm);
	}

	switch (get_ins_reg_width(ins_addr)) {
	case 1:
		return *(unsigned char *)p;

	case 2:
		return *(unsigned short*)p;

	case 4:
		return *(unsigned int*)p;

#ifdef __amd64__
	case 8:
		return *(unsigned long*)p;
#endif

	default:
		PERROR("Error width%s\n", ".");
	}

err:
	return 0;
}

void set_ins_imm_val(unsigned long ins_addr, unsigned long val)
{
	unsigned int opcode;
	unsigned char mod_rm;
	unsigned char mod;
	unsigned char *p;
	int i, shorted, enlarged, rexr;
	unsigned long rv;

	p = (unsigned char *)ins_addr;
	p += skip_prefix(p, &shorted, &enlarged, &rexr);
	p += get_opcode(p, &opcode);
	for (i = 0; i < NR_ELEMENTS(imm_wop); i++)
		if (imm_wop[i] == opcode) {
			rv = IMM_WRITE;
			goto do_work;
		}

	PERROR("Not a imm instruction, opcode=0x%02x\n", opcode);
	goto err;

do_work:
	mod_rm = *p;
	mod = mod_rm >> 6;
	p++;
	switch (mod) {
	case 0:
		/* if r/m is 5 we have a 32 disp (IA32 Manual 3, Table 2-2)  */
		/* AMD64: XXX Check for address size prefix? */
		if ((mod_rm & 0x7) == 0x5)
			p += 4;
		break;

	case 1:
		p += 1;
		break;

	case 2:
		p += 4;
		break;

	case 3:
	default:
		PERROR("it is not a memory access instruction, rm_mod=0x%02x\n", mod_rm);
	}

	switch (get_ins_reg_width(ins_addr)) {
	case 1:
		*(unsigned char *)p = val;
		break;

	case 2:
		*(unsigned short*)p = val;
		break;

	case 4:
		*(unsigned int*)p = val;
		break;

#ifdef __adm64__
	case 8:
		*(unsigned long*)p = val;
		break;
#endif

	default:
		PERROR("Error width%s\n", ".");
	}

err:
	return;
}
