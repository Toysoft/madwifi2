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

#
# Makefile for the Atheros WLAN driver.
#
DEPTH=	..

KMODNAME=	ath_pci

include ${DEPTH}/Makefile.inc

ifdef CONFIG_MODVERSIONS
MODOPTS+=	-include ${WLAN}/if_ieee80211.ver
ALL_VER+=	${WLAN}/if_ieee80211.ver
MODOPTS+=	-include ${WLAN}/if_media.ver
ALL_VER+=	${WLAN}/if_media.ver
endif

SRCS=	if_ath.c if_ath_pci.c
OBJS=	if_ath.o if_ath_pci.o

INCS+=	-I. -I${HAL} -I${HAL}/linux -I${ATH_HAL} \
	-I${WLAN} ${KERNEL_INCS} -I${DEPTH}

ifdef HAL_OBJDIR
INCS+=	-I${HAL_OBJDIR}		# for opt_ah.h
endif

ifneq (${LINUX_VERSION}, 204)
OBJS+=		if_ath.mod.o if_ath_pci.mod.o
endif

ALL=	${KMODNAME}.${KMODSUF}
CFLAGS=	${INCS} ${MODOPTS} ${COPTS}
LDFLAGS=

all:	${ALL}

install: all
	test -d ${DESTDIR}/${MODULE_PATH} || mkdir ${DESTDIR}/${MODULE_PATH}
	cp ${ALL} ${DESTDIR}/${MODULE_PATH}

${KMODNAME}.${KMODSUF}: ${OBJS} Makefile
	${LD} -o $@ -r ${LDFLAGS} ${OBJS}
if_ath_pci.o:	version.h
ifdef CONFIG_MODVERSIONS
if_ath.o:	${ALL_VER}
if_ath_pci.o:	${ALL_VER}
endif

ifneq (${LINUX_VERSION}, 204)
if_ath_pci.mod.c: if_ath_pci.o
	${KERNEL_PATH}/scripts/modpost ${KERNEL_PATH}/vmlinux if_ath_pci.o
if_ath_pci.mod.o:	if_ath_pci.mod.c

if_ath.mod.c: if_ath.o
	${KERNEL_PATH}/scripts/modpost ${KERNEL_PATH}/vmlinux if_ath.o
if_ath.mod.o:	if_ath.mod.c
endif

clean:
	rm -f ${OBJS} ${ALL} ${CLEANFILES}

depend: beforedepend .depend afterdepend
.depend: ${SRCS} Makefile
	rm -f .depend
	-${CC} -M ${CFLAGS} ${SRCS} > .depend
beforedepend:
afterdepend:
-include .depend
