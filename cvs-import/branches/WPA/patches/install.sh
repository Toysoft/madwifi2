# /bin/sh

#
# Shell script to integrate madwifi sources into a Linux
# source tree so it can be built statically.  Typically this
# is done to simplify debugging with tools like kgdb.
#
KERNEL_VERSION=`uname -r`
KERNEL_PATH=${1:-/lib/modules/${KERNEL_VERSION}/build}

MKDIR()
{
	DIR=$1
	test -d $DIR || { echo "Creating $DIR"; mkdir $DIR; }
}

PATCH()
{
	cmp -s $1 $2 || patch $1 < $2.patch
}

INSTALL()
{
	DEST=$1; shift
	cp $* $DEST
}

WIRELESS=${KERNEL_PATH}/drivers/net/wireless
ATH=${WIRELESS}/ath
MKDIR ${ATH}
echo "Copy ath driver bits..."
FILES=`ls ../ath/*.[ch] | sed '/mod.c/d'`
INSTALL ${ATH} ${FILES}
INSTALL ${ATH}/Makefile ../ath/Makefile.kernel

ATH_HAL=${WIRELESS}/ath_hal
MKDIR ${ATH_HAL}
echo "Copy ath_hal bits..."
INSTALL ${ATH_HAL}/Makefile ../ath_hal/Makefile.kernel

MKDIR ${WIRELESS}/hal
echo "Copy hal bits..."
INSTALL ${WIRELESS} -r ../hal

if [ -f Kconfig.wireless-${KERNEL_VERSION}.patch ]; then
	PATCH ${WIRELESS}/Kconfig Kconfig.wireless-${KERNEL_VERSION}
else
	echo "No patch file for your ${KERNEL_VERSION} kernel."
	echo "Look at Kconfig.wireless to see how to change"
	echo "${WIRELESS}/Kconfig to suit."
fi
PATCH ${WIRELESS}/Makefile Makefile.wireless

NET=${KERNEL_PATH}/net
NET80211=${NET}/net80211
MKDIR ${NET80211}
echo "Copy net80211 bits..."
FILES=`ls ../net80211/*.[ch] | sed '/mod.c/d'`
INSTALL ${NET80211} ${FILES}
INSTALL ${NET80211}/Makefile ../net80211/Makefile.kernel 
MKDIR ${NET80211}/compat
echo "Setting up compatibility bits..."
INSTALL ${NET80211}/compat ../include/compat.h
MKDIR ${NET80211}/compat/sys
INSTALL ${NET80211}/compat/sys ../include/sys/*.h

if [ -f Kconfig.net-${KERNEL_VERSION}.patch ]; then
	PATCH ${NET}/Kconfig Kconfig.net-${KERNEL_VERSION}
else
	echo "No patch file for your ${KERNEL_VERSION} kernel."
	echo "Look at Kconfig.net to see how to change"
	echo "${NET}/Kconfig to suit."
fi
PATCH ${NET}/Makefile Makefile.net
