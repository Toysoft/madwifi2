#	$P4$
#
# Makefile for the Atheros WLAN driver.
#

#
# Path to the HAL source code.
#
HAL=	../hal
#
# Path to the build area for the HAL.
#
ATH_HAL=../ath_hal
#
# Path to the Linux kernel include files.
#
LINUX=	../../linux-2.4.20/include

SRCS=	ah_osdep.c if_ath.c if_ath_cs.c if_ath_pci.c

OBJS_COMMON=	ah_osdep.o if_ath.o ${ATH_HAL}/hal.o
OBJS_CS=	${OBJS_COMMON} if_ath_cs.o
OBJS_PCI=	${OBJS_COMMON} if_ath_pci.o

CFLAGS=	-I. -I${ATH_HAL} -I${HAL} -I${LINUX} \
	-D_BUILD_STANDALONE_ -D__KERNEL__ -g -O

all:	ath_cs.o ath_pci.o

ath_cs.o: ${OBJS_CS} Makefile
	${LD} -o $@ -r -x -S ${OBJS_CS}
ath_pci.o: ${OBJS_PCI} Makefile
	${LD} -o $@ -r -x -S ${OBJS_PCI}

clean:
	rm -f ${OBJS} ath_cs.o ath_pci.o .depend

depend:	${ATH_HAL}/opt_ah.h
	gcc -M ${CFLAGS} ${SRCS} > .depend

sinclude .depend
