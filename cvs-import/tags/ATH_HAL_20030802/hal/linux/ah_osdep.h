/*-
 * Copyright (c) 2002, 2003 Sam Leffler, Errno Consulting, Atheros
 * Communications, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the following conditions are met:
 * 1. The materials contained herein are unmodified and are used
 *    unmodified.
 * 2. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following NO
 *    ''WARRANTY'' disclaimer below (''Disclaimer''), without
 *    modification.
 * 3. Redistributions in binary form must reproduce at minimum a
 *    disclaimer similar to the Disclaimer below and any redistribution
 *    must be conditioned upon including a substantially similar
 *    Disclaimer requirement for further binary redistribution.
 * 4. Neither the names of the above-listed copyright holders nor the
 *    names of any contributors may be used to endorse or promote
 *    product derived from this software without specific prior written
 *    permission.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT,
 * MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
 * FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES.
 *
 * $Id$
 */
#ifndef _ATH_AH_OSDEP_H_
#define _ATH_AH_OSDEP_H_
/*
 * Atheros Hardware Access Layer (HAL) OS Dependent Definitions.
 */

/*
 * When building the HAL proper we use no GPL-contaminated include
 * files and must define these types ourself.  Beware of these being
 * mismatched against the contents of <linux/types.h>
 */
#ifndef _LINUX_TYPES_H
typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long long int64_t;

typedef unsigned char u_int8_t;
typedef unsigned short u_int16_t;
typedef unsigned int u_int32_t;
typedef unsigned long long u_int64_t;

typedef unsigned int size_t;
typedef unsigned int u_int;
typedef	void *va_list;
#endif

/*
 * Linux/BSD gcc compatibility shims.
 */
#define	__printflike(_a,_b) \
	__attribute__ ((__format__ (__printf__, _a, _b)))
#define	__va_list	va_list 
#define	OS_INLINE	__inline

typedef void* HAL_SOFTC;
typedef int HAL_BUS_TAG;
typedef void* HAL_BUS_HANDLE;
typedef u_int32_t HAL_BUS_ADDR;			/* XXX architecture dependent */

/*
 * Delay n microseconds.
 */
extern	void ath_hal_delay(int);
#define	OS_DELAY(_n)	ath_hal_delay(_n)

/*
 * Memory zero, copy (non-overlapped), compare, and sort.
 */
#define	OS_MEMZERO(_a, _size)		memset((_a), 0, (_size))
#define	OS_MEMCPY(_dst, _src, _size)	memcpy((_dst), (_src), (_size))
#define	OS_MACEQU(_a, _b) \
	(memcmp((_a), (_b), IEEE80211_ADDR_LEN) == 0)

struct ath_hal;
extern	u_int32_t ath_hal_getuptime(struct ath_hal *);
#define	OS_GETUPTIME(_ah)	ath_hal_getuptime(_ah)

/*
 * Register read/write; we assume the registers
 * will always be memory-mapped.
 */
#ifndef AH_DEBUG
#define	OS_REG_WRITE(_ah, _reg, _val) do {				\
	*((volatile u_int32_t *)((_ah)->ah_sh + (_reg))) = (_val);	\
} while (0)
#define	OS_REG_READ(_ah, _reg) \
	(*((volatile u_int32_t *)((_ah)->ah_sh + (_reg))))
#else
#define	OS_REG_WRITE(_ah, _reg, _val)	ath_hal_reg_write(_ah, _reg, _val)
#define	OS_REG_READ(_ah, _reg)		ath_hal_reg_read(_ah, _reg)

extern	void ath_hal_reg_write(struct ath_hal *ah, u_int reg, u_int32_t val);
extern	u_int32_t ath_hal_reg_read(struct ath_hal *ah, u_int reg);
#endif
#define	OS_MARK(_ah, _id, _v)

/*
 * Linux-specific attach/detach methods needed for module reference counting.
 *
 * XXX We can't use HAL_STATUS because the type isn't defined at this
 *     point (circular dependency); we wack the type and patch things
 *     up in the function.
 */
extern	struct ath_hal *_ath_hal_attach(u_int16_t devid, HAL_SOFTC,
		HAL_BUS_TAG, HAL_BUS_HANDLE, void* status);
extern	void ath_hal_detach(struct ath_hal *);

#endif /* _ATH_AH_OSDEP_H_ */
