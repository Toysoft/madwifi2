#	$P4$
#
# Makefile for the HAL-based Atheros driver.
#
include Makefile.inc

all:	driver/ath_pci.o

driver/ath_pci.o:	wlan/wlan.o ath_hal/hal.o
	cd driver; make ath_pci.o
wlan/wlan.o:
	cd wlan; make wlan.o
ath_hal/hal.o:
	cd ath_hal; make hal.o

install: all
	cd driver; make install

clean:
	cd driver; make clean
	cd wlan; make clean
	cd ath_hal; make clean

depend:
	cd ath_hal; make depend
	cd driver; make depend
	cd wlan; make depend
