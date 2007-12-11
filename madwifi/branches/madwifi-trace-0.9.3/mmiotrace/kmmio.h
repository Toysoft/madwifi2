#ifndef _LINUX_KMMIO_H
#define _LINUX_KMMIO_H
#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
#include <linux/kdebug.h>
#else
#include <asm/kdebug.h>
#endif

struct kmmio_probe;
struct kmmio_fault_page;
struct pt_regs;

typedef unsigned long kmmio_addr_t;
typedef void (*kmmio_pre_handler_t)(struct kmmio_probe *, 
				    struct pt_regs *, 
				    unsigned long addr);
typedef void (*kmmio_post_handler_t)(struct kmmio_probe *, 
				     unsigned long condition, 
				     struct pt_regs *);
struct kmmio_probe {
	struct list_head list;

	/* start location of the probe point */
	kmmio_addr_t addr;

	/* length of the probe region */
	unsigned long len;

	 /* Called before addr is executed. */
	kmmio_pre_handler_t pre_handler;

	/* Called after addr is executed, unless... */
	kmmio_post_handler_t post_handler;
};

struct kmmio_fault_page {
	struct list_head list;

	/* location of the fault page */
	kmmio_addr_t page;

	int count;
};

void lock_kmmio(void);
void unlock_kmmio(void);

/* kmmio is active by some kmmio_probes? */
static inline int is_kmmio_active(void)
{
	extern unsigned int kmmio_count;
	return kmmio_count;
}

/* Init kmmio */
int init_kmmio(void);
void cleanup_kmmio(void);
/* Get the kmmio at this addr (if any).  Must have called lock_kmmio */
struct kmmio_probe *get_kmmio_probe(kmmio_addr_t addr);
struct kmmio_fault_page *get_kmmio_fault_page(kmmio_addr_t page);
int add_kmmio_fault_page(kmmio_addr_t page);
void release_kmmio_fault_page(kmmio_addr_t page);

int register_kmmio_probe(struct kmmio_probe *p);
void unregister_kmmio_probe(struct kmmio_probe *p);

void arm_kmmio_fault_page(kmmio_addr_t page, int *large);
void disarm_kmmio_fault_page(kmmio_addr_t page, int *large);
int post_kmmio_handler(unsigned long condition,
                       struct pt_regs *regs);
int kmmio_handler(struct pt_regs *regs, unsigned long addr);

#endif /* _LINUX_KMMIO_H */
