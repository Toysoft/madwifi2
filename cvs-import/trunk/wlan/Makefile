#	$P4$
#
# Makefile for the 802.11 WLAN module.
#
DESTDIR=

VERSION=	2.4.20

#
# Path to the Linux kernel include files.
#
LINUX=	../../linux-${VERSION}/include

SRCS=	if_ieee80211subr.c if_media.c rc4.c
OBJS=	if_ieee80211subr.o if_media.o rc4.o

#CFLAGS=	-I. -I${LINUX} -D__KERNEL__ -DMODULE -g -O
CFLAGS=	-I. -I${LINUX} -D__KERNEL__ -g -O

all:	wlan.o

wlan.o: ${OBJS}
	${LD} -o $@ -r -x ${OBJS}

install: all
	cp wlan.o ${DESTDIR}/lib/modules/${VERSION}/net

clean:
	rm -f ${OBJS} wlan.o .depend

depend:
	gcc -M ${CFLAGS} ${SRCS} > .depend

sinclude .depend
