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

include ${DEPTH}/Makefile.inc

SRCS=	ah_osdep.c if_ath.c if_ath_pci.c
OBJS=	ah_osdep.o if_ath.o if_ath_pci.o

ifdef HAL_SRC
HALINCS=-I${DEPTH}/hal -I${ATH_HAL}
else
HALINCS=-I${ATH_HAL}
endif
CFLAGS=	-I. ${HALINCS} -I${WLAN} -I${LINUX} \
	-D_BUILD_STANDALONE_ -D__KERNEL__ -DMODULE ${MODOPTS} ${COPTS}
#LDFLAGS=-x -S
LDFLAGS=

ALL=	ath_pci.o

all:	${ALL}

install: all
	test -d ${DESTDIR}/lib/modules/${VERSION}/net || \
		mkdir ${DESTDIR}/lib/modules/${VERSION}/net
	cp ${ALL} ${DESTDIR}/lib/modules/${VERSION}/net

ath_pci.o: ${OBJS} ${ATH_HAL}/hal.o Makefile
	${LD} -o $@ -r ${LDFLAGS} ${OBJS} ${ATH_HAL}/hal.o
if_ath_pci.o:	version.h
ifdef CONFIG_MODVERSIONS
if_ath.o:	${WLAN}/if_ieee80211.ver ${WLAN}/if_media.ver
if_ath_pci.o:	${WLAN}/if_ieee80211.ver ${WLAN}/if_media.ver
endif

clean:
	rm -f ${OBJS} ${ALL} .depend

depend: beforedepend .depend afterdepend
.depend: ${SRCS} ${ATH_HAL}/opt_ah.h Makefile
	rm -f .depend
	-${CC} -M ${CFLAGS} ${SRCS} > .depend
beforedepend:
afterdepend:
-include .depend
