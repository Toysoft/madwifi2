# $Id$
#
# Copyright (c) 2002, 2003 Sam Leffler, Errno Consulting
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

DESTDIR=
#
# Path to the Linux kernel and kernel include files.
#
VERSION=	2.4.20
KERNEL_PATH=	${DEPTH}/../linux-${VERSION}
LINUX=		${KERNEL_PATH}/include
#
# Path to the HAL source code.
#
HAL=	${DEPTH}/hal
#
# Path to the build area for the HAL.
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

COPTS=	-g -O -Wall -include ${DEPTH}/include/compat.h -I${DEPTH}/include

include ${KERNEL_PATH}/.config

ifdef CONFIG_SMP
COPTS+=	-D__SMP__ -DSMP
endif

ifndef CONFIG_FRAME_POINTER
COPTS+=	-fomit-frame-pointer
endif

ifdef CONFIG_MODVERSIONS
MODOPTS=-DMODVERSIONS -include $(KERNEL_PATH)/include/linux/modversions.h
MODOPTS+= -include ${WLAN}/if_ieee80211.ver
MODOPTS+= -include ${WLAN}/if_media.ver
endif
