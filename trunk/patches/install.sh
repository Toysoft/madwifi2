#! /bin/sh
#
# Shell script to integrate madwifi sources into a Linux
# source tree so it can be built statically.  Typically this
# is done to simplify debugging with tools like kgdb.
#

set -e

die()
{
	echo "FATAL ERROR: $1" >&2
	exit 1
}

DEPTH=..
KERNEL_VERSION=`uname -r`

if test -n "$1"; then
	KERNEL_PATH="$1"
else if test -e /lib/modules/${KERNEL_VERSION}/source; then
	KERNEL_PATH="/lib/modules/${KERNEL_VERSION}/source"
else if test -e /lib/modules/${KERNEL_VERSION}/build; then
	KERNEL_PATH="/lib/modules/${KERNEL_VERSION}/build"
else
	die "Cannot guess kernel source location"
fi
fi
fi

PATCH()
{
	patch -N $1 < $2
}

INSTALL()
{
	DEST=$1; shift
	cp $* $DEST
}

#
# Location of various pieces.  These mimic what is in Makefile.inc
# and can be overridden from the environment.
#
SRC_HAL=${HAL:-${DEPTH}/hal}
test -d ${SRC_HAL} || die "No hal directory ${SRC_HAL}"
SRC_NET80211=${WLAN:-${DEPTH}/net80211}
test -d ${SRC_NET80211} || die "No net80211 directory ${SRC_NET80211}"
SRC_ATH=${ATH:-${DEPTH}/ath}
test -d ${SRC_ATH} || die "No ath directory ${SRC_ATH}"
SRC_ATH_RATE=${DEPTH}/ath_rate
test -d ${SRC_ATH_RATE} ||
	die "No rate control algorithm directory ${SRC_ATH_RATE}"
SRC_COMPAT=${DEPTH}/include
test -d ${SRC_COMPAT} || die "No compat directory ${SRC_COMPAT}"

WIRELESS=${KERNEL_PATH}/drivers/net/wireless
test -d ${WIRELESS} || die "No wireless directory ${WIRELESS}"

if test -f ${WIRELESS}/Kconfig; then
	kbuild=2.6
	kbuildconf=Kconfig
	makedef=LINUX26
else if test -f ${WIRELESS}/Config.in; then
	kbuild=2.4
	kbuildconf=Config.in
	makedef=LINUX24
else
	die "Kernel build system is not supported"
fi
fi

echo "Copying top-level files"
MADWIFI=${WIRELESS}/madwifi
rm -rf ${MADWIFI}
mkdir -p ${MADWIFI}
make -s -C ${DEPTH} svnversion.h KERNELCONF=/dev/null ARCH=. TARGET=i386-elf
INSTALL ${MADWIFI} ${FILES} ${DEPTH}/*.h
INSTALL ${MADWIFI} ${DEPTH}/BuildCaps.inc
cat >>${MADWIFI}/BuildCaps.inc <<EOF

EXTRA_CFLAGS += \$(COPTS)

ifdef CONFIG_CPU_BIG_ENDIAN
EXTRA_CFLAGS += -DAH_BYTE_ORDER=AH_BIG_ENDIAN
else
EXTRA_CFLAGS += -DAH_BYTE_ORDER=AH_LITTLE_ENDIAN
endif

$makedef := 1
EOF


echo "Copying ath driver files"
DST_ATH=${MADWIFI}/ath
mkdir -p ${DST_ATH}
FILES=`ls ${SRC_ATH}/*.[ch] | sed '/mod.c/d'`
INSTALL ${DST_ATH} ${FILES}
INSTALL ${DST_ATH}/Makefile ${SRC_ATH}/Makefile.kernel


echo "Copying ath_rate files"
DST_ATH_RATE=${MADWIFI}/ath_rate
mkdir -p ${DST_ATH_RATE}
RATEALGS="amrr onoe sample"
for ralg in $RATEALGS; do
	mkdir -p ${DST_ATH_RATE}/$ralg
	FILES=`ls ${SRC_ATH_RATE}/$ralg/*.[ch] | sed '/mod.c/d'`
	INSTALL ${DST_ATH_RATE}/$ralg ${FILES}
	INSTALL ${DST_ATH_RATE}/$ralg/Makefile ${SRC_ATH_RATE}/$ralg/Makefile.kernel
done

echo "Copying Atheros HAL files"
DST_HAL=${MADWIFI}/hal
mkdir -p ${DST_HAL}
INSTALL ${DST_HAL} ${SRC_HAL}/ah.h
INSTALL ${DST_HAL} ${SRC_HAL}/ah_desc.h
INSTALL ${DST_HAL} ${SRC_HAL}/ah_devid.h
INSTALL ${DST_HAL} ${SRC_HAL}/version.h
mkdir -p ${DST_HAL}/linux
INSTALL ${DST_HAL}/linux ${SRC_HAL}/linux/ah_osdep.c
INSTALL ${DST_HAL}/linux ${SRC_HAL}/linux/ah_osdep.h
mkdir -p ${DST_HAL}/public
INSTALL ${DST_HAL}/public ${SRC_HAL}/public/*.opt_ah.h
INSTALL ${DST_HAL}/public ${SRC_HAL}/public/*.hal.o.uu


echo "Copying net80211 files"
DST_NET80211=${MADWIFI}/net80211
mkdir -p ${DST_NET80211}
FILES=`ls ${SRC_NET80211}/*.[ch] | sed '/mod.c/d'`
INSTALL ${DST_NET80211} ${FILES} ${DEPTH}/*.h
INSTALL ${DST_NET80211}/Makefile ${SRC_NET80211}/Makefile.kernel


echo "Copying compatibility files"
DST_COMPAT=${MADWIFI}/include
mkdir -p ${DST_COMPAT}
INSTALL ${DST_COMPAT} ${SRC_COMPAT}/*.h
mkdir -p ${DST_COMPAT}/sys
INSTALL ${DST_COMPAT}/sys ${SRC_COMPAT}/sys/*.h


echo "Patching the build system"
INSTALL ${MADWIFI} $kbuild/Makefile
if test "$kbuild" = 2.6; then
INSTALL ${MADWIFI} $kbuild/Kconfig
sed -i '/madwifi/d;/^endmenu/i\
source "drivers/net/wireless/madwifi/Kconfig"' ${WIRELESS}/Kconfig
sed -i '$a\
obj-$(CONFIG_ATHEROS) += madwifi/
/madwifi/d;' ${WIRELESS}/Makefile
else
INSTALL ${MADWIFI} $kbuild/Config.in
sed -i '$a\
source drivers/net/wireless/madwifi/Config.in
/madwifi/d' ${WIRELESS}/Config.in
sed -i '/madwifi/d;/include/i\
subdir-$(CONFIG_ATHEROS) += madwifi\
obj-$(CONFIG_ATHEROS) += madwifi/madwifi.o' ${WIRELESS}/Makefile
fi

DST_DOC=${KERNEL_PATH}/Documentation
if test -f $kbuild/Configure.help.patch; then
	grep -q 'CONFIG_ATHEROS' ${DST_DOC}/Configure.help || \
		PATCH ${DST_DOC}/Configure.help $kbuild/Configure.help.patch
fi


echo "Done"
