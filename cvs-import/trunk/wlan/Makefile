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
# Makefile for the 802.11 WLAN module.
#
DEPTH=	..

include ${DEPTH}/Makefile.inc

ifdef CONFIG_MODVERSIONS
MODOPTS+=	-include ${WLAN}/if_ieee80211.ver
ALL_VER+=	if_ieee80211.ver
MODOPTS+=	-include ${WLAN}/if_media.ver
ALL_VER+=	if_media.ver
endif

SRCS_80211=	if_ieee80211subr.c if_ieee80211wireless.c
OBJS_80211=	if_ieee80211subr.o if_ieee80211wireless.o
SRCS=	${SRCS_80211} if_media.c rc4.c
OBJS=	${OBJS_80211} if_media.o rc4.o

INCS=	-I. -I${LINUX}
CFLAGS=	${INCS} -D__KERNEL__ -DMODULE ${MODOPTS} ${COPTS}

ALL=	wlan.o

all:	${ALL}

wlan.o: ${OBJS}
	${LD} -o $@ -r -x ${OBJS}

if_ieee80211subr.o: version.h

ifdef CONFIG_MODVERSIONS
if_ieee80211.ver: ${SRCS_80211}
	rm -f $@
	for i in ${SRCS_80211}; do \
		${CC} ${INCS} ${COPTS} -E -D__GENKSYMS__ $$i | \
			${GENKSYMS} -k ${VERSION} >> $@; \
	done
if_media.ver: if_media.c
	${CC} ${INCS} ${COPTS} -E -D__GENKSYMS__ $< | \
		${GENKSYMS} -k ${VERSION} > $@

${OBJS}: if_ieee80211.ver if_media.ver
endif

install: all
	test -d ${DESTDIR}/lib/modules/${VERSION}/net || \
		mkdir ${DESTDIR}/lib/modules/${VERSION}/net
	cp wlan.o ${DESTDIR}/lib/modules/${VERSION}/net

clean:
	rm -f ${OBJS} ${ALL} ${ALL_VER} .depend

depend: beforedepend .depend afterdepend
.depend: ${SRCS} ${ALL_VER}
	rm -f .depend
	${CC} -M ${CFLAGS} ${SRCS} > .depend
beforedepend:
afterdepend:
-include .depend
