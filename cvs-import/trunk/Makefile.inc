#	$P4$
#
DESTDIR=
#
# Path to the Linux kernel and kernel include files.
#
VERSION=	2.4.20
KERNEL_PATH=	${DEPTH}/../linux-${VERSION}
LINUX=		${KERNEL_PATH}/include
#
# Path to the HAL source code.
#
HAL=	${DEPTH}/hal
#
# Path to the build area for the HAL.
#
ATH_HAL=${DEPTH}/ath_hal
#
# Path to the 802.11 include files
#
WLAN=	${DEPTH}/wlan
#
# Path to the device driver.
#
DRIVER=	${DEPTH}/driver

COPTS=	-g -O -Wall

include ${KERNEL_PATH}/.config
