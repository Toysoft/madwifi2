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
# Makefile for the Atheros HAL Module.
#
DEPTH=	..

include ${DEPTH}/Makefile.inc

#
# When we have source code use those include files so we
# know we're in sync.  Otherwise, use the ``released'' .h
# files that correspond to hal.o.
#
ifndef HAL_SRC
HALINCS=-I${ATH_HAL}
else
HALINCS=-I${DEPTH}/hal -I${ATH_HAL}
endif

ALL=	ath_hal.o
# NB: add to SRCS so we get dependency info, but don't add to OBJS
SRCS=	ah_osdep.c
INCS=	-I. ${HALINCS} -I${LINUX}
COPTS+= -D_BUILD_STANDALONE_
CFLAGS=	${INCS} -D__KERNEL__ -DMODULE ${MODOPTS} ${COPTS}

ifdef CONFIG_MODVERSIONS
MODOPTS+=	-include ${ATH_HAL}/ath_hal.ver
ALL_VER=	ath_hal.ver
endif

all:	${ALL}

ifdef HAL_SRC
#
# Got source code; just use the HAL makefile smarts.
#
include ${DEPTH}/hal/Makefile.inc
else
#
# No HAL source code; just use the hal.o to generate a module.
#
.PRECIOUS: hal.o opt_ah.h
endif

ath_hal.o: ah_osdep.o hal.o version.h Makefile
	${LD} -o $@ -r -x ah_osdep.o hal.o

ifdef CONFIG_MODVERSIONS
ath_hal.ver: ah_osdep.c
	rm -f $@
	${CC} ${INCS} ${COPTS} -E -D__GENKSYMS__ ah_osdep.c | \
			${GENKSYMS} -k ${VERSION} >> $@
ah_osdep.o: ath_hal.ver
endif

install: all
	test -d ${DESTDIR}/lib/modules/${VERSION}/net || \
		mkdir ${DESTDIR}/lib/modules/${VERSION}/net
	cp ${ALL} ${DESTDIR}/lib/modules/${VERSION}/net
clean:
	rm -f ${ALL} ${OBJS} ah_osdep.o .depend ${ALL_VER}

depend: beforedepend .depend afterdepend
.depend: ${SRCS} ${ALL_VER} opt_ah.h
	rm -f .depend
	${CC} -M ${CFLAGS} ${SRCS} > .depend
beforedepend:
afterdepend:
-include .depend
