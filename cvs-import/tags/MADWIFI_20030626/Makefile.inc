#
# Copyright (c) 2002, 2003 Sam Leffler, Errno Consulting
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce at minimum a disclaimer
#    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
#    redistribution must be conditioned upon including a substantially
#    similar Disclaimer requirement for further binary redistribution.
# 3. Neither the names of the above-listed copyright holders nor the names
#    of any contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# Alternatively, this software may be distributed under the terms of the
# GNU General Public License ("GPL") version 2 as published by the Free
# Software Foundation.
#
# NO WARRANTY
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGES.
#
# $Id$
#

DESTDIR=
#
# Define this to build the HAL from source code
#
#HAL_SRC=	true
#
#
# Define these for cross-compilation; e.g.
#
#TOOLPREFIX=	/export/tools/bin/mips-linux-
#TARGET_OPTS=	-g -O -G 0 -mno-abicalls -fno-pic -mcpu=r4600 -mips2 -Wa,--trap
#
CC=		${TOOLPREFIX}gcc
LD=		${TOOLPREFIX}ld
STRIP=		${TOOLPREFIX}strip
OBJCOPY=	${TOOLPREFIX}objcopy
TARGET_OPTS=	-g -O
#
NM=		nm
AWK=		awk

#
#
# Path to the Linux kernel, include files, and where to install the modules.
#
VERSION=	2.4.20
KERNEL_PATH=	${DEPTH}/../linux-${VERSION}
LINUX=		${KERNEL_PATH}/include
MODULE_PATH=	/lib/modules/${VERSION}/net

#
# Path to the HAL source code.
#
HAL=	${DEPTH}/hal
#
# Path to the HAL build area.
#
ATH_HAL=${DEPTH}/ath_hal
#
# Path to the 802.11 include files
#
WLAN=	${DEPTH}/wlan
#
# Path to the device driver.
#
DRIVER=	${DEPTH}/driver

COPTS=	-Wall ${TARGET_OPTS} \
	-include ${DEPTH}/include/compat.h -I${DEPTH}/include

include ${KERNEL_PATH}/.config

ifdef CONFIG_SMP
COPTS+=	-D__SMP__ -DSMP
endif

ifndef CONFIG_FRAME_POINTER
COPTS+=	-fomit-frame-pointer
endif

ifdef CONFIG_MODVERSIONS
MODOPTS=-DMODVERSIONS -include $(KERNEL_PATH)/include/linux/modversions.h

GENKSYMS=	/sbin/genksyms
ifdef CONFIG_SMP
GENKSYMS+=	-p smp_
endif
endif
