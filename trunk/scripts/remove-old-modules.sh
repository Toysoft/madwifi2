#!/bin/bash
# Purpose: provide support for removing modules from previous
# MadWifi installations.
#
# Author: Kel Modderman

if [[ ${UID} != 0 ]]
then
	echo "You must run this script as root."
	exit 1
fi

KERNVER=${KERNELRELEASE:-"$(uname -r)"}
KERNDIR=/lib/modules/${KERNVER}

function unload_madwifi ()
{
	unset UNLOADED FAILED
	# Tweaked to circumvent inter-module dependencies
	for m in ath{_pci,_{rate{_{amrr,onoe,sample}}},_hal} wlan{_{wep,tkip,ccmp,acl,xauth,scan{_{sta,ap}}},}; do
		if grep -q ^${m} /proc/modules
		then
			UNLOADED="${UNLOADED}${m} "
			rmmod ${m} &>/dev/null || FAILED="${FAILED}${m} "
		else
			continue
		fi
	done
	
	if [[ -z ${UNLOADED} ]]
	then
		echo "No modules were unloaded."
	else
		echo "Successfully unloaded: $UNLOADED"
	fi
	
	if [[ -n ${FAILED} ]]
	then
		echo "Failed to unload: ${FAILED}"
		echo "Please run $0 again"
		exit 1
	fi
}

function find_old_madwifi ()
{
	if [[ ! -d ${KERNDIR} ]]
	then
		echo "Module directory for target kernel release cannot be found."
		exit 1
	else
		for m in ath{_{pci,hal},_{rate{_{amrr,onoe,sample}}}} wlan{,_{wep,tkip,ccmp,acl,xauth,scan{_{sta,ap}}}}; do
			find "${KERNDIR}" -type f -name ${m}.*o
		done
	fi
}

function rm_previous_madwifi ()
{
	OLD_MODULES="$(find_old_madwifi)"
	if [[ -n ${OLD_MODULES} ]]
	then
		echo "Old MadWifi modules found at ${KERNDIR}"
		while true; do
			read -p "List old MadWifi modules? [y],n "
                	case ${REPLY} in
				n)
					break
					;;
                        	
				y) 	
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
			read -p "Remove old MadWifi modules? [y],n "
                	case ${REPLY} in
				n)
					exit 1
					;;
                        	
				y) 	
					if [[ ${KERNVER} == "$(uname -r)" ]]
					then
						unload_madwifi
					fi
					echo "Removing modules."
					rm -f ${OLD_MODULES} && exit 0 || exit 1
					;;
                                
				*) 
					continue
					;;
                	esac
		done
	else
		echo "No MadWifi modules found at ${KERNDIR}"
		exit 0
	fi
}

rm_previous_madwifi
