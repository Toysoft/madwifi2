#!/bin/sh
# Purpose: provide support for removing modules from previous
# MadWifi installations.
#
# Author: Kel Modderman

if [ $(id -u) != 0 ]; then
	echo
	echo "You must run this script as root."
	echo
	exit 1
fi

QUIET=${1}

SCRIPTS=$(dirname $0)
if [ -z "$SCRIPTS" ]; then
	SCRIPTS=.
fi

if [ -r "${SCRIPTS}"/../install.log ]; then
	. ${SCRIPTS}/../install.log
fi

DEST="${DESTDIR}"
KERNVER="${KERNELRELEASE:-$(uname -r)}"
MODDIR="${DEST}/lib/modules/${KERNVER}"

PATTERN="^.*\/(ath_(hal|pci|rate_(onoe|amrr|sample))\.k?o)|(wlan(_(acl|ccmp|scan_(ap|sta)|tkip|wep|xauth))?\.k?o)$"
OLD_MODULES=$(find ${MODDIR} -type f -regex '.*\.k?o' | grep -w -E "${PATTERN}")
if [ -n "${OLD_MODULES}" ]; then
	if [ "${QUIET}" = "noask" ]; then
		rm -f ${OLD_MODULES} || exit 1
	else
		echo
		echo "Old MadWifi modules found"
		echo
		while true; do
			echo -n "List old MadWifi modules? [y],n "
			read REPLY
			case ${REPLY} in
				n|N|[Nn][Oo])
					break
					;;
	
				y|Y|[Yy][Ee][Ss]) 	
					for m in ${OLD_MODULES}; do
						echo ${m}
					done
					echo
					break
					;;
	
				*) 
					continue
					;;
			esac
		done
	
		while true; do
			echo -n "Remove old MadWifi modules? [y],n "
			read REPLY
			case ${REPLY} in
				n|N|[Nn][Oo])
					exit 1
					;;
	
				y|Y|[Yy][Ee][Ss]) 	
					rm -f ${OLD_MODULES} || exit 1
					echo
					echo "Removed old MadWifi modules"
					echo
					break
				;;
	
				*) 
					continue
					;;
			esac
		done
	fi

	if [ "${KERNVER}" = $(uname -r) ]; then
		unset UNLOADED FAILED
		for m in ath_pci ath_rate_amrr ath_rate_onoe ath_rate_sample ath_hal \
			 wlan_wep wlan_tkip wlan_ccmp wlan_acl wlan_xauth wlan_scan_sta \
			 wlan_scan_ap wlan; do
			if grep -q ^${m} /proc/modules;	then
				modprobe -r ${m} 2>/dev/null || FAILED="${FAILED}${m} "
				UNLOADED="${UNLOADED}${m} "
			else
				continue
			fi
		done
					
		if [ -n "${FAILED}" ]; then
			echo "Failed to unload: ${FAILED}"
			echo "Please run $0 again"
			echo
			exit 1
		elif [ -n "${UNLOADED}" ] && [ "${QUIET}" != "noask" ]; then
			echo "Successfully unloaded MadWifi modules"
			echo "[ ${UNLOADED}]"
			echo
		fi
	fi
else
	if [ "${QUIET}" != "noask" ]; then
		echo
		echo "No MadWifi modules found"
		echo
	fi
fi

exit 0
