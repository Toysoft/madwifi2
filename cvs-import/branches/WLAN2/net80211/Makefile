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

KMODNAME=	wlan

include ${DEPTH}/Makefile.inc

SRCS_80211= ieee80211.c ieee80211_node.c ieee80211_input.c ieee80211_crypto.c ieee80211_output.c ieee80211_proto.c
OBJS_80211= ieee80211.o ieee80211_node.o ieee80211_input.o ieee80211_crypto.o ieee80211_output.o ieee80211_proto.o
SRCS_BSD= if_media.c rc4.c
OBJS_BSD= if_media.o rc4.o
SRCS_LINUX= linux/if_stats.c linux/if_proc.c linux/if_wireless.c
OBJS_LINUX= linux/if_stats.o linux/if_proc.o linux/if_wireless.o
SRCS=	${SRCS_80211} ${SRCS_BSD} ${SRCS_LINUX} ieee80211-mod.c
OBJS=	${OBJS_80211} ${OBJS_BSD} ${OBJS_LINUX} ieee80211-mod.o

INCS+=	-I. -I${DEPTH} ${KERNEL_INCS}

CLEANFILES=	.depend

ALL=	${KMODNAME}.${KMODSUF}
CFLAGS=	${INCS} ${MODOPTS} ${COPTS} -gdwarf-2

MODULE_OBJECT_FILE_LIST= \
 ieee80211_crypto.o \
 ieee80211_input.o \
 ieee80211_output.o \
 ieee80211-mod.o

all:	${ALL}

${KMODNAME}.${KMODSUF}: ${OBJS}
	${LD} -o $@ -r -x ${OBJS}


ifdef CONFIG_MODVERSIONS
GEN_VERSION=	$(shell echo $(KERNEL_VERSION) | sed -e "s/-.*//")
ieee80211.ver: ${SRCS_80211}
	rm -f $@
	for i in ${SRCS_80211}; do \
		${CC} ${INCS} ${COPTS} -E -D__GENKSYMS__ $$i | \
			${GENKSYMS} -k ${GEN_VERSION} >> $@; \
	done
if_media.ver: if_media.c
	${CC} ${INCS} ${COPTS} -E -D__GENKSYMS__ $< | \
		${GENKSYMS} -k ${GEN_VERSION} > $@

${OBJS}: ieee80211.ver if_media.ver
endif


install: all
	test -d ${DESTDIR}/${MODULE_PATH} || mkdir ${DESTDIR}/${MODULE_PATH}
	cp ${ALL} ${DESTDIR}/${MODULE_PATH}

clean:
	rm -f ${OBJS} ${ALL} ${ALL_VER} ${CLEANFILES}

depend: beforedepend .depend afterdepend
.depend: ${SRCS} ${ALL_VER}
	rm -f .depend
	${CC} -M ${CFLAGS} ${SRCS} > .depend
beforedepend:
afterdepend:
-include .depend
