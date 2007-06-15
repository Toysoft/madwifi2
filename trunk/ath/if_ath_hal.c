/* Wrapper macros/functions for the binary HAL to comply with local coding
 * convention.  Provides function-style calling convention using either macros
 * or wrapper functions for function pointers in the HAL.
 *
 * The typical convention is ath_hal_foo(ah,p1,p2,p3,...) turns into
 * ah->ah_foo(p1,p2,p3,...) where ah_foo is a function pointer and a member
 * of the struct ath_hal (usually named ah). */

#include "opt_ah.h"

#ifdef ATH_HALOPS_TRACEABLE

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <asm/uaccess.h>
/* Include header file for declarations */
#include "ath/if_ath_hal.h"

/* Include header file for implementations (if necessary) */
#define TRACEABLE_IMPL
#include "ath/if_ath_hal.h"
#undef  TRACEABLE_IMPL

#endif /* #ifdef ATH_HALOPS_TRACEABLE */
