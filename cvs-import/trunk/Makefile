#	$P4$
#
# Makefile for the HAL-based Atheros driver.
#
DEPTH=	.

include Makefile.inc

all:
	cd ath_hal; make
	cd ${WLAN}; make
	cd ${DRIVER}; make

install:
	cd ${DRIVER}; make install

clean:
	cd ${DRIVER}; make clean
	cd ${WLAN}; make clean
	cd ${ATH_HAL}; make clean

depend:
	cd ${ATH_HAL}; make depend
	cd ${DRIVER}; make depend
	cd ${WLAN}; make depend
