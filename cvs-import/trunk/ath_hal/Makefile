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
# NB: we can't support MODVERSIONS because it requires building
#     from source which may not be available
#
DEPTH=	..

KMODNAME=	ath_hal

include ${DEPTH}/Makefile.inc

HALINCS=-I${HAL} -I${HAL}/${OS} -I${ATH_HAL}

SRCS=	${HAL}/${OS}/ah_osdep.c
OBJS=	ah_osdep.o
INCS+=	${HALINCS} ${KERNEL_INCS} -I${DEPTH}

ifneq (${LINUX_VERSION}, 204)
OBJS+=		ah_osdep.mod.o
CLEANFILES+=	ah_osdep.mod.c
endif

ALL=	${KMODNAME}.${KMODSUF}
CFLAGS=	${INCS} ${MODOPTS} ${COPTS}

all:	${ALL}

${KMODNAME}.${KMODSUF}: ${OBJS} ${HAL_OBJDIR}hal.o ${HAL}/version.h Makefile
	${LD} -o $@ -r -x ${OBJS} ${HAL_OBJDIR}hal.o

ifndef HAL_OBJDIR
#
# Use release version of HAL.
#
# NB: binary HAL's are distributed in a uuencode'd file; on some
#     systems uuencode is not installed by default and you must
#     specially install it from the sharutils package.
#
UUDECODE?=uudecode
OPT_AH_H=	opt_ah.h
CLEANFILES+=	hal.o opt_ah.h

hal.o:	${HAL}/${OS}/${TARGET}.hal.o.uu
	${UUDECODE} < $<
opt_ah.h: ${HAL}/${OS}/${TARGET}.opt_ah.h
	cp ${HAL}/${OS}/${TARGET}.opt_ah.h opt_ah.h
else
#
# Use private version of HAL.
#
OPT_AH_H=	${HAL_OBJDIR}/opt_ah.h
HALINCS+=	-I${HAL_OBJDIR}		# for opt_ah.h
endif

ah_osdep.o: ${HAL}/${OS}/ah_osdep.c
	${CC} -c -o $@ ${CFLAGS} $<

ifneq (${LINUX_VERSION}, 204)
ah_osdep.mod.c: ah_osdep.o
	${KERNEL_PATH}/scripts/modpost ${KERNEL_PATH}/vmlinux $<
ah_osdep.mod.o: ah_osdep.mod.c
	${CC} -c -o $@ ${CFLAGS} $<
endif

install: all
	test -d ${DESTDIR}/${MODULE_PATH} || mkdir ${DESTDIR}/${MODULE_PATH}
	cp ${ALL} ${DESTDIR}/${MODULE_PATH}

clean:
	rm -f ${ALL} ${OBJS} ${CLEANFILES}

depend: beforedepend .depend afterdepend
.depend: ${SRCS} ${OPT_AH_H}
	rm -f .depend
	${CC} -M ${CFLAGS} ${SRCS} > .depend
beforedepend:
afterdepend:
-include .depend
