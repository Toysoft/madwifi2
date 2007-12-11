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

#ifndef __PF_H_
#define __PF_H_


#define PF_MAJOR 1
#define PF_MINOR 1
#define PF_PATCH 0
//#include <stdio.h>
//#include <signal.h>
#define PERROR printk

/* 
 * Delcare typedefs
 */

enum reason_type{
	NOT_ME,	/* page fault is not in regions */
	NOTHING,	/* access others point in regions */
	REG_READ,	/* read from addr to reg */ 
	REG_WRITE,	/* write from reg to addr */
	IMM_WRITE,	/* write from imm to addr */
	OTHERS	/* Other instructions can not intercept */
};

/*
 * Declare functions in pf_interface.c 
 */
void register_pf_interface(void);
void unregister_pf_interface(void);


/*
 * Declare functions in pf_region.c
 */
void pf_lock_region_irqsave(unsigned long *flags);
void pf_lock_region(void);
void pf_unlock_region_irqrestore(unsigned long flags);
void pf_unlock_region(void);
int pf_find_region(unsigned long addr);
int pf_add_region(unsigned long addr);
void pf_release_region(int ri);
int pf_is_removed_regions(unsigned long addr);
void pf_clr_pte_bits(unsigned long addr, unsigned long bitmask);
void pf_set_pte_bits(unsigned long addr, unsigned long bitmask);
/*
 * Declare functions in pf_in.c
 */
typedef struct pt_regs regs_t;
//typedef struct sigcontext regs_t;
enum reason_type get_ins_type(unsigned long ins_addr);
unsigned int get_ins_reg_width(unsigned long ins_addr);
unsigned int get_ins_mem_width(unsigned long ins_addr);
unsigned long get_ins_reg_val(unsigned long ins_addr, regs_t *regs);
void set_ins_reg_val(unsigned long addr, regs_t *regs, unsigned long val);
unsigned long get_ins_imm_val(unsigned long ins_addr);
void set_ins_imm_val(unsigned long ins_addr, unsigned long val);

/*
 * Declare functions in pf_utils.c
 */
unsigned long fi_phy_to_virt(unsigned long phy_addr);
#if 0
/*
 * Declare functions in pf.c
*/
int pf_register(struct watchpoint wp, int id, struct intercept_hook *hook);
int pf_unregister(int id);
int pf_corrupt(int id, __u32 dirty, void *data);
#endif
#endif//__PF_H_
