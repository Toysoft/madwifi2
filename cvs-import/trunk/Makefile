#	$P4$
#
# Makefile for the HAL-based Atheros driver.
#
DEPTH=	.

include Makefile.inc

# NB: the order is important here
DIRS=	${ATH_HAL} ${WLAN} ${DRIVER}

all:
	for i in ${DIRS}; do \
		(cd $$i; make); \
	done

install:
	cd ${WLAN}; make install
	cd ${DRIVER}; make install
	depmod -a

clean:
	for i in ${DIRS}; do \
		(cd $$i; make clean); \
	done

depend:
	for i in ${DIRS}; do \
		(cd $$i; make depend); \
	done
