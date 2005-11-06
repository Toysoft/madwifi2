#!/bin/bash
# Purpose: provide support for removing modules from previous
# MadWifi installations.
#
# Author: Kel Modderman

if (($UID))
then
	echo
	echo "You must run this script as root."
	echo
	exit 1
fi

SCRIPTS=$(dirname $0)
if [[ -z $SCRIPTS ]]
then
	SCRIPTS=.
fi

[[ -r ${SCRIPTS}/../install.log ]] && source ${SCRIPTS}/../install.log

DEST="${DESTDIR}"
KERNVER="${KERNELRELEASE:-$(uname -r)}"
MODDIR="${DEST}/lib/modules/${KERNVER}"

function unload_madwifi ()
{
	unset UNLOADED FAILED
	# Tweaked to circumvent inter-module dependencies
	for m in ath{_pci,_rate_{amrr,onoe,sample},_hal} wlan{_{wep,tkip,ccmp,acl,xauth,scan_{sta,ap}},}; do
		if grep -q ^${m} /proc/modules
		then
			UNLOADED="${UNLOADED}${m} "
			rmmod ${m} &>/dev/null || FAILED="${FAILED}${m} "
		else
			continue
		fi
	done

	if [[ ${UNLOADED} ]]
	then
		echo "Successfully unloaded: $UNLOADED"
	else
		echo "No modules were unloaded."
	fi

	if [[ ${FAILED} ]]
	then
		echo
		echo "Failed to unload: ${FAILED}"
		echo "Please run $0 again"
		echo
		exit 1
	fi
}

function find_old_madwifi ()
{
	if [[ ! -d ${MODDIR} ]]
	then
		break
	else	
		PATTERN="^.*\/(ath_(hal|pci|rate_(onoe|amrr|sample))\.k?o)|(wlan(_(acl|ccmp|scan_(ap|sta)|tkip|wep|xauth))?\.k?o)$"
		find ${MODDIR} -type f -regex '.*\.k?o' | grep -w -E "${PATTERN}"
	fi
}

OLD_MODULES=$(find_old_madwifi)
if [[ ${OLD_MODULES} ]]
then
	echo
	echo "Old MadWifi modules found"
	echo
	while true; do
		echo -n "List old MadWifi modules? [y],n "
		read REPLY
		case ${REPLY} in
			n|N)
				break
				;;

			y|Y) 	
				for m in ${OLD_MODULES}; do
					echo ${m}
				done
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
			n|N)
				exit 1
				;;

			y|Y) 	
				if [[ ${KERNVER} == $(uname -r) ]]
				then
					unload_madwifi
				fi
				rm -f ${OLD_MODULES} || exit 1
				echo
				echo "Old modules removed"
				echo
				exit 0
			;;

			*) 
				continue
				;;
		esac
	done
else
	echo
	echo "No MadWifi modules found"
	echo
	exit 0
fi
