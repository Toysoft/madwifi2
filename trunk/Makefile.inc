#
# Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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
# $Id: //depot/sw/linuxsrc/src/802_11/madwifi/madwifi/Makefile.inc#11 $
#

ifeq ($(obj),)
obj=	.
endif

# other locales sometimes cause trouble
export LC_ALL = POSIX

# OS is the target operating system. Currently only Linux is supported.
OS=		linux

# TARGET defines the target platform architecture. It must match one of
# the target platforms supported by the HAL. The default target is the
# host machine architecture. You can override TARGET on the make command
# line or in the environment. See hal/linux/*.inc for the list of
# supported targets.
TARGET ?= $(shell uname -m | sed -e s/i.86/i386/ -e s/ppc/powerpc-be/)-elf

# TOOLPREFIX is a string that is prepended to all toolchain executables,
# such as gcc, ld, as, objcopy etc.  This is used for cross-compiling.
# Default is native compilation.
TOOLPREFIX ?= ""

# BUS defines the bus type to which the wireless devices is attached.
# Currently, the valid BUS types are PCI and AHB.  If BUS is not
# defined, then, we assume the bus type is PCI
ifndef BUS
BUS=		PCI
endif

# DESTDIR is used as path prefix during installation.
DESTDIR ?=

# KERNELPATH is the path to the target kernel's build/source area.
# This is used to obtain the kernel configuration and include files.
# If automatic determination doesn't work as desired, you have to
# manually set it, for example with:
# make KERNELPATH=/path/to/kernel/source

# TOOLPATH is the path which contains the crosscompile toolchain (?)

ifeq ($(strip ${BUS}),AHB)
# Bus type AHB forces the target platform to be mipsisa32.
TARGET :=	mipsisa32-be-elf

# Bus type AHB requires KERNELPATH and TOOLPATH to be set manually.
ifndef KERNELPATH
$(error KERNELPATH must be defined for bus type AHB.)
endif
ifndef TOOLPATH
$(error TOOLPATH must be defined for bus type AHB.)
endif

else

KERNELPATH ?= $(shell readlink -f /lib/modules/`uname -r`/build)

ifeq ($(KERNELPATH),)
KERNELPATH = /usr/src/linux
$(warning Default KERNELPATH not found, using $(KERNELPATH))
endif

endif

# sanity check: does KERNELPATH exist?
ifeq ($(wildcard $(KERNELPATH)),)
$(error KERNELPATH: $(KERNELPATH) does not exist)
endif

# KERNELRELEASE is the target kernel's version.  It's always taken from
# the kernel build tree.  Kernel Makefile doesn't always know the exact
# kernel version (especially for vendor stock kernels), so we get it
# from linux/version.h instead.
KERNELRELEASE = $(shell sed -ne 's/"//g;s/^\#define UTS_RELEASE //p' \
		  $(KERNELPATH)/include/linux/version.h)

# KERNELCONF is the name of the file that holds the configuration
# of the target kernel.
KERNELCONF ?=	${KERNELPATH}/.config

# sanity check: does KERNELCONF exist?
ifeq ($(wildcard $(KERNELCONF)),)
$(error KERNELCONF: $(KERNELCONF) does not exist.)
endif

# SYSTEMMAP is the name of the System.map file for the target
# kernel. This will be needed when DESTDIR is defined in order
# to run depmod properly.
SYSTEMMAP ?= 	${KERNELPATH}/System.map

# MODULEPATH nominates the directory where the modules will be
# installed to
ifeq ($(strip ${BUS}),AHB)
MODULEPATH := 	${KERNELPATH}/arch/mips/ar531x/RAMDISK/rootdir/lib/modules/${KERNELRELEASE}/net
else
MODULEPATH ?=	/lib/modules/${KERNELRELEASE}/net
endif

# Some settings that depend on actual kernel release
ifneq ($(filter-out 2.4%,$(KERNELRELEASE)),)
export-objs	:=
list-multi	:=
KMODSUF		:= ko
else
KMODSUF		:= o
endif

NM=		nm
AWK=		awk
UUDECODE ?= $(shell which uudecode 2>/dev/null)


#
# Path to the HAL source code.
#
ifeq ($(HAL),)
HAL=	${obj}/${DEPTH}/hal
endif
#
# Path to the HAL build area.
#
ATH_HAL=${DEPTH}/ath_hal
#
# Path to the 802.11 include files.
#
WLAN=	${DEPTH}/net80211
#
# Path to the Atheros device driver.
#
ATH=	${DEPTH}/ath
#
# Path to the rate control algorithm.
#
ifeq ($(ATH_RATE),)
ATH_RATE=	ath_rate/sample
endif

COPTS+=	-Werror
INCS=	-include ${obj}/${DEPTH}/include/compat.h -I${obj}/${DEPTH}/include

ifeq ($(strip ${BUS}),AHB)
INCS+= -isystem ${TOOLPATH}/include
endif

include ${HAL}/public/${TARGET}.inc
include ${obj}/${DEPTH}/BuildCaps.inc

SYMBOLSDIR=${DEPTH}/symbols