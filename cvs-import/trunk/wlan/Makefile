#	$P4$
#
# Makefile for the 802.11 WLAN module.
#

#
# Path to the Linux kernel include files.
#
LINUX=	../../linux-2.4.20/include

SRCS=	if_ieee80211subr.c rc4.c
OBJS=	if_ieee80211subr.o rc4.o

CFLAGS=	-I. -I${LINUX} -D__KERNEL__ -DMODULE -g -O

all:	wlan.o

wlan.o: ${OBJS}
	${LD} -o $@ -r -x ${OBJS}

clean:
	rm -f ${OBJS} wlan.o .depend

depend:
	gcc -M ${CFLAGS} ${SRCS} > .depend

sinclude .depend
