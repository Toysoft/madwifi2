#ifndef ATH_LINUX_H
#define ATH_LINUX_H

#include <sys/queue.h> /* for TAILQ_* macros */

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/version.h>
#undef __KERNEL_STRICT_NAMES
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include <asm/atomic.h>
#include <asm/uaccess.h> /* for copy_to/from_user */


#include <linux/netdevice.h>
#include <linux/skbuff.h>

#ifndef ifr_media
#define	ifr_media	ifr_ifru.ifru_ivalue
#endif
struct ifmediareq {
  char    ifm_name[IFNAMSIZ];     /* if name, e.g. "en0" */
  int     ifm_current;            /* current media options */
  int     ifm_mask;               /* don't care mask */
  int     ifm_status;             /* media status */
  int     ifm_active;             /* active options */
  int     ifm_count;              /* # entries in ifm_ulist array */
  int     *ifm_ulist;             /* media words */
};
#define	SIOCSIFMEDIA	_IOWR('i', 55, struct ifreq)	/* set net media */
#define	SIOCGIFMEDIA	_IOWR('i', 56, struct ifmediareq) /* get net media */
#include "if_media.h"

#define  ETHER_IS_MULTICAST(addr) (*(addr) & 0x01)

#define ETHER_ADDR_LEN          6       /* length of an Ethernet address */
/*
 * Structure of a 10Mb/s Ethernet header.
 */
struct  ether_header {
  u_char  ether_dhost[ETHER_ADDR_LEN];
  u_char  ether_shost[ETHER_ADDR_LEN];
  u_short ether_type;
};

struct llc {
  u_int8_t llc_dsap;
  u_int8_t llc_ssap;
  union {
    struct {
      u_int8_t control;
      u_int8_t format_id;
      u_int8_t class;
      u_int8_t window_x2;
    } type_u __packed;
    struct {
      u_int8_t num_snd_x2;
      u_int8_t num_rcv_x2;
    } type_i __packed;
    struct {
      u_int8_t control;
      u_int8_t num_rcv_x2;
    } type_s __packed;
    struct {
      u_int8_t control;
      /*
       * We cannot put the following fields in a structure because
       * the structure rounding might cause padding.
       */
      u_int8_t frmr_rej_pdu0;
      u_int8_t frmr_rej_pdu1;
      u_int8_t frmr_control;
      u_int8_t frmr_control_ext;
      u_int8_t frmr_cause;
    } type_frmr __packed;
    struct {
      u_int8_t  control;
      u_int8_t  org_code[3];
      u_int16_t ether_type;
    } type_snap __packed;
    struct {
      u_int8_t control;
      u_int8_t control_ext;
    } type_raw __packed;
  } llc_un /* XXX __packed ??? */;
} __packed;

#define LLC_SNAP_LSAP   0xaa
#define llc_control             llc_un.type_u.control
#define llc_snap                llc_un.type_snap
#define LLC_UI          0x3


#ifndef KASSERT
#define KASSERT(x)
#endif


/* this is really gcc-specific... ie: unportable to other compilers. */
#define if_printf(if,str,...) printk (str, ## __VA_ARGS__)
#define printf(str,...) printk (str, ## __VA_ARGS__)


/**
 * simulate BSD malloc/free signature.
 */
#define MALLOC_DEFINE(a,b,c)
#define M_NOWAIT GFP_ATOMIC
#define M_ZERO (0x80000000)
static __inline void *
kmalloc_zeroify (size_t size, int flags)
{
	void *ptr = kmalloc (size, flags & ~M_ZERO);
	if (ptr) {
		memset (ptr, 0, size);
	}
	return ptr;
}
#define malloc(size,type,f2) (f2 & M_ZERO)? kmalloc_zeroify (size, f2):kmalloc (size, f2)
#define free(x,type) kfree(x)
#define copyout(from,to,size) copy_to_user(to,from,size);

#define le16toh(x) le16_to_cpu (x)
#define htole16(x) cpu_to_le16 (x)

#if !defined (htons) && !(defined (__htons))

#define ___htonl(x) __cpu_to_be32(x)
     #define ___htons(x) __cpu_to_be16(x)
     #define ___ntohl(x) __be32_to_cpu(x)
     #define ___ntohs(x) __be16_to_cpu(x)
 
#if defined(__KERNEL__) || (defined (__GLIBC__) && __GLIBC__ >= 2)
     #define htonl(x) ___htonl(x)
     #define ntohl(x) ___ntohl(x)
#else
#define htonl(x) ((unsigned long)___htonl(x))
     #define ntohl(x) ((unsigned long)___ntohl(x))
#endif
#define htons(x) ___htons(x)
     #define ntohs(x) ___ntohs(x)

#endif /* bad hack because kernel headers are broken in non-optimized mode. */



struct ieee80211com;

/**
 * linux-specific crap.
 */
static __inline const char * 
ieee80211_get_device_name (struct ieee80211com *ic)
{
	struct net_device *dev = (struct net_device *) ic;
	return dev->name;
}

#include <linux/random.h>

static __inline u_int32_t
arc4random (void)
{
	char buffer[4];
	get_random_bytes (buffer, 4);
	return (u_int32_t) (buffer[0] << 0) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
}

static __inline u_int8_t *
ieee80211_get_broadcast (struct ieee80211com *ic)
{
	struct net_device *dev = (struct net_device *) ic;
	return (u_int8_t *)dev->broadcast;
}

static __inline u_int8_t *
ieee80211_get_address (struct ieee80211com *ic)
{
	struct net_device *dev = (struct net_device *) ic;
	return (u_int8_t *)dev->dev_addr;
}

const char* ether_sprintf(const u_int8_t *mac);

int ieee80211_linuxmoduleref (struct ieee80211com *ic);
void ieee80211_linuxmoduleunref (struct ieee80211com *ic);

#endif /* ATH_LINUX_H */
