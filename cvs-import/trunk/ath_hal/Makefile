#	$P4$
#
# Makefile for the Atheros HAL.
#
# To build a copy of the HAL do
#	make depend
#	make
# to generate hal_syms.o and hal.o.  The latter has all but the
# required symbols stripped from it and is the intended form for
# distribution.  The hal_syms.o file is intended for debugging.
#
DEPTH=	..

include ${DEPTH}/Makefile.inc

AH_SRCS=${HAL}/ah.c ${HAL}/ah_regdomain.c
AH_OBJS=ah.o ah_regdomain.o

AR5210=	${HAL}/ar5210
AR5210_SRCS=${AR5210}/ar5210_attach.c ${AR5210}/ar5210_beacon.c \
	${AR5210}/ar5210_interrupts.c ${AR5210}/ar5210_keycache.c \
	${AR5210}/ar5210_misc.c ${AR5210}/ar5210_power.c \
	${AR5210}/ar5210_recv.c ${AR5210}/ar5210_reset.c \
	${AR5210}/ar5210_xmit.c
AR5210_OBJS=ar5210_attach.o ar5210_beacon.o ar5210_interrupts.o \
	ar5210_keycache.o ar5210_misc.o ar5210_power.o ar5210_recv.o \
	ar5210_reset.o ar5210_xmit.o

AR5211=	${HAL}/ar5211
AR5211_SRCS=${AR5211}/ar5211_attach.c ${AR5211}/ar5211_beacon.c \
	${AR5211}/ar5211_interrupts.c ${AR5211}/ar5211_keycache.c \
	${AR5211}/ar5211_misc.c ${AR5211}/ar5211_power.c \
	${AR5211}/ar5211_recv.c ${AR5211}/ar5211_reset.c \
	${AR5211}/ar5211_xmit.c
AR5211_OBJS=ar5211_attach.o ar5211_beacon.o ar5211_interrupts.o \
	ar5211_keycache.o ar5211_misc.o ar5211_power.o ar5211_recv.o \
	ar5211_reset.o ar5211_xmit.o

SRCS=	${AH_SRCS} ${AR5210_SRCS} ${AR5211_SRCS} ${AR5212_SRCS}
OBJS=	${AH_OBJS} ${AR5210_OBJS} ${AR5211_OBJS} ${AR5212_OBJS}

CFLAGS=	-I. -I${HAL} -D_BUILD_STANDALONE_ ${COPTS}

hal.o:	hal_syms.o Makefile
	strip -x -o $@ \
		--keep-symbol=ath_hal_attach \
		--keep-symbol=ath_hal_init_channels \
		--keep-symbol=ath_hal_ghz2ieee \
		--keep-symbol=ath_hal_ieee2ghz \
	$<

hal_syms.o:	${OBJS}
	${LD} -o $@ -r -x -S ${OBJS}

clean:
	rm -f ${OBJS} hal.o hal_syms.o opt_ah.h .depend

ah.o:	${HAL}/ah.c
	${CC} -c -o $@ ${CFLAGS} $<
ah_regdomain.o:	${HAL}/ah_regdomain.c
	${CC} -c -o $@ ${CFLAGS} $<

ar5210_attach.o: ${AR5210}/ar5210_attach.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_beacon.o: ${AR5210}/ar5210_beacon.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_interrupts.o: ${AR5210}/ar5210_interrupts.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_keycache.o: ${AR5210}/ar5210_keycache.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_misc.o:	${AR5210}/ar5210_misc.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_power.o:	${AR5210}/ar5210_power.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_recv.o:	${AR5210}/ar5210_recv.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_reset.o:	${AR5210}/ar5210_reset.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5210_xmit.o:	${AR5210}/ar5210_xmit.c
	${CC} -c -o $@ ${CFLAGS} $<

ar5211_attach.o: ${AR5211}/ar5211_attach.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_beacon.o: ${AR5211}/ar5211_beacon.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_interrupts.o: ${AR5211}/ar5211_interrupts.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_keycache.o: ${AR5211}/ar5211_keycache.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_misc.o:	${AR5211}/ar5211_misc.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_power.o:	${AR5211}/ar5211_power.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_recv.o:	${AR5211}/ar5211_recv.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_reset.o:	${AR5211}/ar5211_reset.c
	${CC} -c -o $@ ${CFLAGS} $<
ar5211_xmit.o:	${AR5211}/ar5211_xmit.c
	${CC} -c -o $@ ${CFLAGS} $<

opt_ah.h:
	echo "#define AH_SUPPORT_AR5210	1" > opt_ah.h
	echo "#define AH_SUPPORT_AR5211	1" >> opt_ah.h
#	echo "#define AH_SUPPORT_AR5212	1" >> opt_ah.h

depend:	opt_ah.h
	gcc -M ${CFLAGS} ${SRCS} > .depend

sinclude .depend
