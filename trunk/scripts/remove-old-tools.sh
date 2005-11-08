#!/bin/sh
# Purpose: provide support for removing tools and manpages from previous
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

if [ -r "${SCRIPTS}"/../tools/install.log ]; then
	. ${SCRIPTS}/../tools/install.log
fi

DEST="${DESTDIR}"
BIN="${BINDIR:-/usr/local/bin}"
BINPATH="${DEST}${BIN}"
MAN="${MANDIR:-/usr/local/man}"
MANPATH="${DEST}${MAN}/man8"
TOOLS="athstats 80211stats athkey athchans athctrl athdebug 80211debug wlanconfig"

REGEX_TOOLS=$(echo ${TOOLS} | sed 's/\ /\\|/g')
REGEX_MAN=$(echo ${TOOLS} | sed 's/\ /\.8\\|/g')
FOUND_TOOLS=$(find ${BINPATH} -type f -regex ".*\(${REGEX_TOOLS}\)" 2>/dev/null)
FOUND_MAN=$(find ${MANPATH} -type f -regex ".*\(${REGEX_MAN}.8\)" 2>/dev/null)

# This subshell ignores blank space of empty var's
FOUND=$(echo ${FOUND_TOOLS} ${FOUND_MAN})
if [ -n "${FOUND}" ]; then
	if [ "${QUIET}" = "noask" ]; then
		rm -f ${FOUND} || exit 1
	else
		echo
		echo "Old MadWifi tools found"
		echo
		while true; do
			echo -n "List old MadWifi tools? [y],n "
			read REPLY
			case ${REPLY} in
				n|N|[Nn][Oo])
					break
					;;
     		
				y|Y|[Yy][Ee][Ss]) 	
					for t in ${FOUND}; do
						echo ${t}
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
			echo -n "Remove old MadWifi tools? [y],n "
			read REPLY
			case ${REPLY} in
				n|N|[Nn][Oo])
					exit 1
					;;
	                       	
				y|Y|[Yy][Ee][Ss]) 	
					rm -f ${FOUND} || exit 1
					echo
					echo "Removed old MadWifi tools"
					echo
					break
					;;
                               
				*) 
					continue
					;;
        	       	esac
		done
	fi	
else
	if [ "${QUIET}" != "noask" ]; then
		echo
		echo "No old MadWifi tools found."
		echo
	fi
fi

exit 0
