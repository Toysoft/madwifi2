#	$P4$
#
# Makefile for the Atheros WLAN driver.
#
DEPTH=	..

include ${DEPTH}/Makefile.inc

SRCS=	ah_osdep.c if_ath.c if_ath_cs.c if_ath_pci.c

OBJS_COMMON=	ah_osdep.o if_ath.o
OBJS_CS=	${OBJS_COMMON} ${ATH_HAL}/hal.o if_ath_cs.o ${WLAN}/wlan.o
OBJS_PCI=	${OBJS_COMMON} ${ATH_HAL}/hal.o if_ath_pci.o ${WLAN}/wlan.o
OBJS=		${OBJS_COMMON} if_ath_cs.o if_ath_pci.o

CFLAGS=	-I. -I${ATH_HAL} -I${HAL} -I${WLAN} -I${LINUX} \
	-D_BUILD_STANDALONE_ -D__KERNEL__ -DMODULE ${COPTS}

#ALL=	ath_cs.o ath_pci.o
ALL=	ath_pci.o

all:	${ALL}

install: all
	cp ${ALL} ${DESTDIR}/lib/modules/${VERSION}/net

ath_cs.o: ${OBJS_CS} Makefile
	${LD} -o $@ -r -x -S ${OBJS_CS}
ath_pci.o: ${OBJS_PCI} Makefile
	${LD} -o $@ -r -x -S ${OBJS_PCI}

clean:
	rm -f ${OBJS} ath_cs.o ath_pci.o .depend

depend:	${ATH_HAL}/opt_ah.h
	gcc -M ${CFLAGS} ${SRCS} > .depend

sinclude .depend
