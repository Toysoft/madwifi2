#!/bin/sh
# Purpose: provide support for removing modules from previous
# MadWifi installations.
#
# Author: Kel Modderman

SCRIPTS=$(dirname $0)
if [ -z "$SCRIPTS" ]; then
	SCRIPTS=.
fi

if [ -r "${SCRIPTS}"/../install.log ]; then
	. ${SCRIPTS}/../install.log
fi

DEST=${DESTDIR}
KERNVER=${KERNELRELEASE:-$(uname -r)}
MODDIR=${DEST}/lib/modules/${KERNVER}

while [ "$#" -gt 0 ]; do
	case ${1} in
		*.o|*.ko)
			OLD_MODULES="${OLD_MODULES}${1} "
			shift
			;;
		
		noask)
			QUIET="noask"
			shift
			;;

		*)
			echo
			echo "Bad input: ${1}"
			echo
			exit 1
			;;
	esac
done

if [ -z "${OLD_MODULES}" ];then
	PATTERN="^.*\/(ath_(hal|pci|rate_(onoe|amrr|sample))\.k?o)|(wlan(_(acl|ccmp|scan_(ap|sta)|tkip|wep|xauth))?\.k?o)$"
	OLD_MODULES=$(find ${MODDIR} -type f -regex '.*\.k?o' 2>/dev/null | grep -w -E "${PATTERN}")
fi

if [ -n "${OLD_MODULES}" ]; then
	if [ "${QUIET}" = "noask" ]; then
		rm -f ${OLD_MODULES} || exit 1
	else
		echo
		echo "Old MadWifi modules found. Remove them?"
		
		while true; do
			echo
			echo -n "[l]ist, [r]emove, e[x]it (l,r,[x]) ? "
			echo
			read REPLY
			case ${REPLY} in
				l|L)
					for m in ${OLD_MODULES}; do echo ${m}; done
					continue
					;;
	
				r|R) 	
					rm -f ${OLD_MODULES} || exit 1
					echo
					echo "Removed old MadWifi modules"
					echo
					break
					;;
	
				*) 
					exit 1
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
