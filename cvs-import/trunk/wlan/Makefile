#	$P4$
#
# Makefile for the 802.11 WLAN module.
#
DEPTH=	..

include ${DEPTH}/Makefile.inc

SRCS_80211=	if_ieee80211subr.c if_ieee80211ioctl.c if_ieee80211wireless.c
OBJS_80211=	if_ieee80211subr.o if_ieee80211ioctl.o if_ieee80211wireless.o
SRCS=	${SRCS_80211} if_media.c rc4.c
OBJS=	${OBJS_80211} if_media.o rc4.o

INCS=	-I. -I${LINUX}
CFLAGS=	${INCS} -D__KERNEL__ -DMODULE ${MODOPTS} ${COPTS}

ALL=	wlan.o
ifdef CONFIG_MODVERSIONS
ALL_VER=if_ieee80211.ver if_media.ver
endif

all:	${ALL} ${ALL_VER}

wlan.o: ${OBJS}
	${LD} -o $@ -r -x ${OBJS}

ifdef CONFIG_MODVERSIONS
GENKSYMS=	/sbin/genksyms
ifdef CONFIG_SMP
GENKSYMS+=	-p smp_
endif

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
	cp wlan.o ${DESTDIR}/lib/modules/${VERSION}/net

clean:
	rm -f ${OBJS} ${ALL} ${ALL_VER} .depend

depend:	${ALL_VER}
	gcc -M ${CFLAGS} ${SRCS} > .depend

sinclude .depend
