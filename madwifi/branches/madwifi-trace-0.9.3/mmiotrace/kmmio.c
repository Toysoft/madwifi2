/* Support for MMIO probes.
 * Benfit many code from kprobes
 * (C) 2002 Louis Zhuang <louis.zhuang@intel.com>.
*/

#include <linux/spinlock.h>
#include <linux/hash.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/ptrace.h>
#include <linux/preempt.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/errno.h>
#include <asm/tlbflush.h>

#include "kmmio.h"

#define KMMIO_HASH_BITS 6
#define KMMIO_TABLE_SIZE (1 << KMMIO_HASH_BITS)

static LIST_HEAD(kmmio_probes);

#define KMMIO_PAGE_HASH_BITS 4
#define KMMIO_PAGE_TABLE_SIZE (1 << KMMIO_PAGE_HASH_BITS)
static struct list_head kmmio_page_table[KMMIO_PAGE_TABLE_SIZE];

unsigned int kmmio_count = 0;
unsigned int handler_registered = 0;
static spinlock_t kmmio_lock = SPIN_LOCK_UNLOCKED;
static int cpu = -1;
static struct kmmio_fault_page *cur_page = NULL;
static struct kmmio_probe *cur_probe = NULL;
static unsigned long kmmio_saved_eflags;
static int large_page = 0;

int kmmio_page_fault_notifier(struct notifier_block *nb, unsigned long val, void *args);
int kmmio_die_notifier(struct notifier_block *nb, unsigned long val, void *args);

struct notifier_block nb_page_fault = {
	.notifier_call = kmmio_page_fault_notifier
};

struct notifier_block nb_die = {
	.notifier_call = kmmio_die_notifier
};

/* Locks kmmio: irqs must be disabled */
void lock_kmmio(void)
{
	spin_lock(&kmmio_lock);
}

void unlock_kmmio(void)
{
	spin_unlock(&kmmio_lock);
}

int init_kmmio(void)
{
	int i;
    
	/* FIXME allocate the probe table, currently defined statically */
	/* initialize all list heads */
	for (i = 0; i < KMMIO_PAGE_TABLE_SIZE; i++)
		INIT_LIST_HEAD(&kmmio_page_table[i]);
    
	register_die_notifier(&nb_die);

	return 0;
}

void cleanup_kmmio(void)
{
	if (handler_registered)
		unregister_page_fault_notifier(&nb_page_fault);

	unregister_die_notifier(&nb_die);
}

/* You have to be holding the kmmio_lock */
/* this is basically a dynamic stabbing problem:
 * Could use the existing prio tree code or
 * Possible better implementations:
 * The Interval Skip List: A Data Structure for Finding All Intervals That Overlap a Point (might be simple)
 * Space Efficient Dynamic Stabbing with Fast Queries - Mikkel Thorup */
struct kmmio_probe *get_kmmio_probe(unsigned long addr)
{
	struct kmmio_probe *p;
	list_for_each_entry(p, &kmmio_probes, list) {
		if (addr >= p->addr && addr <= (p->addr + p->len))
			return p;
	}
	return NULL;
}

int register_kmmio_probe(struct kmmio_probe *p)
{
	int ret = 0;
	unsigned long size = 0;

	spin_lock_irq(&kmmio_lock);
	kmmio_count++;
	if (get_kmmio_probe(p->addr)) {
		ret = -EEXIST;
		goto out;
	}
	list_add(&p->list, &kmmio_probes);
	/*printk("adding fault pages...\n");*/
	while (size < p->len) {
		if (add_kmmio_fault_page(p->addr + size)) {
			printk(KERN_ERR "mmio: Unable to set page fault.\n");
		}
		size += PAGE_SIZE;
	}

	if (!handler_registered) {
		register_page_fault_notifier(&nb_page_fault);
		handler_registered++;
	}

out:
	spin_unlock_irq(&kmmio_lock);
	global_flush_tlb();
	return ret;
}

void unregister_kmmio_probe(struct kmmio_probe *p)
{
	unsigned long size = 0;

	spin_lock_irq(&kmmio_lock);
	while (size < p->len) {
		release_kmmio_fault_page(p->addr + size);
		size += PAGE_SIZE;
	}
	list_del(&p->list);
	kmmio_count--;
	spin_unlock_irq(&kmmio_lock);
}

struct kmmio_fault_page *get_kmmio_fault_page(kmmio_addr_t page)
{
	struct list_head *head, *tmp;

	page &= PAGE_MASK;
	head = &kmmio_page_table[hash_long(page, KMMIO_PAGE_HASH_BITS)];
	list_for_each(tmp, head) {
		struct kmmio_fault_page *p
			= list_entry(tmp, struct kmmio_fault_page, list);
		if (p->page == page)
			return p;
	}

	return NULL;
}

int add_kmmio_fault_page(kmmio_addr_t page)
{
	struct kmmio_fault_page *f;

	page &= PAGE_MASK;
	f = get_kmmio_fault_page(page);
	if (f) {
		f->count++;
		return 0;
	}

	f = (struct kmmio_fault_page *)kmalloc(sizeof(*f), GFP_ATOMIC);
	if (!f)
		return -1;

	f->count = 1;
	f->page = page;
	list_add(&f->list,
		 &kmmio_page_table[hash_long(f->page, KMMIO_PAGE_HASH_BITS)]);

	arm_kmmio_fault_page(f->page, NULL);

	return 0;
}

void release_kmmio_fault_page(kmmio_addr_t page)
{
	struct kmmio_fault_page *f;

	page &= PAGE_MASK;
	f = get_kmmio_fault_page(page);
	if (!f)
		return;

	f->count--;
	if(!f->count) {
		disarm_kmmio_fault_page(f->page, NULL);
		list_del(&f->list);
	}
}

int kmmio_page_fault_notifier(struct notifier_block *nb, unsigned long val, void *args)
{   
	unsigned long address;
	struct die_args *arg = args;
	    
	if (val == DIE_PAGE_FAULT) {
		/* get address */
#if defined(__i386__)
		address = read_cr2();
#else
		__asm__("movq %%cr2,%0":"=r" (address));
#endif
		if (is_kmmio_active())
			if(kmmio_handler(arg->regs, address) == 1)
				return NOTIFY_STOP;
        
	}
	return NOTIFY_DONE;
} 

int kmmio_die_notifier(struct notifier_block *nb, unsigned long val, void *args)
{
	struct die_args *arg = args;

	if (val == DIE_DEBUG)
		if(post_kmmio_handler(arg->err, arg->regs) == 1)
			return NOTIFY_STOP;

	return NOTIFY_DONE;
}

/*
 * Interrupts are disabled on entry as trap3 is an interrupt gate
 * and they remain disabled thorough out this function.
 */
int kmmio_handler(struct pt_regs *regs, unsigned long addr)
{
	struct kmmio_fault_page *tmp_page = NULL;

	/* We're in an interrupt, but this is clear and BUG()-safe. */
	preempt_disable();

	lock_kmmio();

	tmp_page = get_kmmio_fault_page(addr);
	if (!tmp_page) {
		/* this page fault is not caused by kmmio */
		/* XXX some pending fault on other cpu may cause problem! */
		unlock_kmmio();
		goto no_kmmio;
	}
	cur_page = tmp_page;
	cpu = smp_processor_id();

	cur_probe = get_kmmio_probe(addr);
	kmmio_saved_eflags = (regs->eflags & (TF_MASK|IF_MASK));

	if (cur_probe && cur_probe->pre_handler) {
		cur_probe->pre_handler(cur_probe, regs, addr);
	}

	regs->eflags |= TF_MASK;
	regs->eflags &= ~IF_MASK;

	/* We hold lock, now we set present bit in PTE and single step. */
	disarm_kmmio_fault_page(cur_page->page, &large_page);

	return 1;

no_kmmio:
	preempt_enable_no_resched();
	return 0;
}

/*
 * Interrupts are disabled on entry as trap1 is an interrupt gate 
 * and they remain disabled thorough out this function.  
 * And we hold kmmio lock.
 */
int post_kmmio_handler(unsigned long condition, struct pt_regs *regs)
{
	if (!is_kmmio_active())
		return 0;
	if (smp_processor_id() != cpu)
		return 0;

	if (cur_probe && cur_probe->post_handler) {
		cur_probe->post_handler(cur_probe, condition, regs);
	}

	arm_kmmio_fault_page(cur_page->page, &large_page);

	regs->eflags &= ~TF_MASK;
	regs->eflags |= kmmio_saved_eflags;

	cpu = -1;

	unlock_kmmio();
	preempt_enable_no_resched();

        /*
	 * if somebody else is singlestepping across a probe point, eflags
	 * will have TF set, in which case, continue the remaining processing
	 * of do_debug, as if this is not a probe hit.
	 */
	if (regs->eflags & TF_MASK)
		return 0;

	return 1;
}

void arm_kmmio_fault_page(kmmio_addr_t page, int *large)
{
	unsigned long address = page & PAGE_MASK;
	pgd_t *pgd = pgd_offset_k(address);
	pud_t *pud = pud_offset(pgd, address);
	pmd_t *pmd = pmd_offset(pud, address);
	pte_t *pte = pte_offset_kernel(pmd, address);

	if (pmd_large(*pmd)) {
		set_pmd(pmd, __pmd(pmd_val(*pmd) & ~_PAGE_PRESENT));
		if (large)
			*large = 1;
	} else
		set_pte(pte, __pte(pte_val(*pte) & ~_PAGE_PRESENT));

	__flush_tlb_one(page);
}

void disarm_kmmio_fault_page(kmmio_addr_t page, int *large)
{
	unsigned long address = page & PAGE_MASK;
	pgd_t *pgd = pgd_offset_k(address);
	pud_t *pud = pud_offset(pgd, address);
	pmd_t *pmd = pmd_offset(pud, address);
	pte_t *pte = pte_offset_kernel(pmd, address);

	if (large && *large) {
		set_pmd(pmd, __pmd(pmd_val(*pmd) | _PAGE_PRESENT));
		*large = 0;
	} else
		set_pte(pte, __pte(pte_val(*pte) | _PAGE_PRESENT));

	__flush_tlb_one(page);
}

/* the function is only used to make virt map to bus */
/* Unused
void *kmmio_invert_map(void *virt_addr, unsigned long bus_addr)
{
	long offset;
	pte_t *pte;

	if((unsigned long)virt_addr & ~PAGE_MASK)
		BUG();

	offset = bus_addr & ~PAGE_MASK;
	bus_addr &= PAGE_MASK;
	pte = get_pte((unsigned long)virt_addr);

	set_pte(pte, __pte((pte_val(*pte) & ~PAGE_MASK) | bus_addr));
	return virt_addr+offset;
}
EXPORT_SYMBOL_GPL(kmmio_invert_map);
*/
