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

ifndef DESTDIR
DESTDIR=
endif
#
OS=		linux
#
# TARGET defines the target platform ISA per GNU rules.
# It must match one of the target platforms supported by
# the HAL.  To set the set of target platforms look at
# hal/linux/*.inc.  You can set this in the environment
# to override the default setting.
#
ifndef TARGET
TARGET=		i386-elf
endif
#
# ARCH defines the target platform architecture.  We derive
# it from TARGET unless it is explicitly set in the environment.
# The rules for deriving it are taken from the Linux kernel for
# consistency.
#
ifndef ARCH
ARCH=		$(shell echo ${TARGET} | sed \
		-e s/i.86-.*/i386/ \
		-e s/sun4u-.*/sparc64/ \
		-e s/powerpc-.*/ppc/ \
		-e s/arm.-.**/arm/ \
		-e s/sa110-.*/arm/)
endif
#
NM=		nm
AWK=		awk

#
# Path to the Linux kernel, include files, and where to install the modules.
#
# KERNEL_VERSION is the target kernel's version; it's used to enable
# build techniques and some code.  If KERNEL_VERSIO is not set in the
# environment then it is taken from the running system.
#
# KERNEL_PATH is the path to the target kernel's build/source area.
# This is used to obtain the kernel configuration and include files.
# If KERNEL_PATH is not set in the environment then it is derived from
# KERNEL_VERSION.
#
# KERNEL_CONF is the filename of the target kernel's make configuration
# file.  If this is not set in the environment then it is derived from
# KERNEL_PATH.
#
ifndef KERNEL_VERSION
KERNEL_VERSION=	$(shell uname -r)
endif
ifndef KERNEL_PATH
KERNEL_PATH=	/lib/modules/${KERNEL_VERSION}/build
endif
ifndef KERNEL_CONF
KERNEL_CONF=	${KERNEL_PATH}/.config
endif
MODULE_PATH=	/lib/modules/${KERNEL_VERSION}/net
#
# A numeric version of the target system's version string, suitable
# for doing if checks in make files.
#
LINUX_VERSION=	$(shell echo $(KERNEL_VERSION) | \
		awk '{ split($$0,a,"."); print a[1]*100+a[2]; }')
#
# Path to the HAL source code.
#
ifndef HAL
HAL=	${DEPTH}/hal
endif
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

#
# Basic build components.
#
COPTS=	-Wall -Wno-trigraphs -O2 -D__KERNEL__ -DMODULE
#
# Define this for some cross-build environments
#
COPTS+=	-D__linux__
#
INCS=	-include ${DEPTH}/include/compat.h -I${DEPTH}/include
#
# NB: some architectures include platform/foo.h files,
#     so add a special include search path
#
KERNEL_INCS=	-I${KERNEL_PATH}/include \
		-I${KERNEL_PATH}/arch/${ARCH}
CLEANFILES=	.depend

include ${KERNEL_CONF}

ifdef CONFIG_SMP
COPTS+=	-D__SMP__ -DSMP
endif

ifndef CONFIG_FRAME_POINTER
COPTS+=	-fomit-frame-pointer
endif

ifdef CONFIG_MODVERSIONS
GENKSYMS=	/sbin/genksyms
ifdef CONFIG_SMP
GENKSYMS+=	-p smp_
endif
endif

#
# 2.4, 2.5, 2.6 handling.
# Wish we had proper conditionals to use here..
#
ifneq (${LINUX_VERSION}, 204)
MODOPTS=	-DKBUILD_BASENAME=${KMODNAME} -DKBUILD_MODNAME=${KMODNAME}
ifdef CONFIG_MODVERSIONS
MODOPTS+=	-DMODVERSIONS \
		-include $(KERNEL_PATH)/include/linux/modsetver.h
endif
KMODSUF=	ko
KERNEL_INCS+=	-I${KERNEL_PATH}/include/asm/mach-default
CLEANFILES+=	*.mod.c
else
ifdef CONFIG_MODVERSIONS
MODOPTS=	-DMODVERSIONS \
		-include $(KERNEL_PATH)/include/linux/modversions.h
endif
KMODSUF=	o
endif
#
include ${HAL}/${OS}/${TARGET}.inc
