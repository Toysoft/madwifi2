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

#
# Makefile for the Atheros WLAN driver.
#
DEPTH=	..

include ${DEPTH}/Makefile.inc

SRCS=	ah_osdep.c if_ath.c if_ath_pci.c
OBJS=	ah_osdep.o if_ath.o if_ath_pci.o ${ATH_HAL}/hal.o

CFLAGS=	-I. -I${ATH_HAL} -I${HAL} -I${WLAN} -I${LINUX} \
	-D_BUILD_STANDALONE_ -D__KERNEL__ -DMODULE ${MODOPTS} ${COPTS}
#LDFLAGS=-x -S
LDFLAGS=

ALL=	ath_pci.o

all:	${ALL}

install: all
	test -d ${DESTDIR}/lib/modules/${VERSION}/net || \
		mkdir ${DESTDIR}/lib/modules/${VERSION}/net
	cp ${ALL} ${DESTDIR}/lib/modules/${VERSION}/net

ath_pci.o: ${OBJS} Makefile
	${LD} -o $@ -r ${LDFLAGS} ${OBJS}
if_ath_pci.o:	version.h
ifdef CONFIG_MODVERSIONS
if_ath.o:	${WLAN}/if_ieee80211.ver ${WLAN}/if_media.ver
if_ath_pci.o:	${WLAN}/if_ieee80211.ver ${WLAN}/if_media.ver
endif

clean:
	rm -f ${OBJS} ${ALL} .depend

depend: beforedepend .depend afterdepend
ifdef CONFIG_MODVERSIONS
.depend: ${SRCS} ${ATH_HAL}/opt_ah.h \
	${WLAN}/if_ieee80211.ver ${WLAN}/if_media.ver
else
.depend: ${SRCS} ${ATH_HAL}/opt_ah.h
endif
	rm -f .depend
	gcc -M ${CFLAGS} ${SRCS} > .depend
-include .depend
