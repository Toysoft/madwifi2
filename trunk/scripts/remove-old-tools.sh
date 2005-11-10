#!/bin/sh
# Purpose: provide support for removing tools and manpages from previous
# MadWifi installations.
#
# Author: Kel Modderman

SCRIPTS=$(dirname $0)
if [ -z "$SCRIPTS" ]; then
	SCRIPTS=.
fi

if [ -r "${SCRIPTS}"/../tools/install.log ]; then
	. ${SCRIPTS}/../tools/install.log
fi

DEST=${DESTDIR}
BIN=${BINDIR:-/usr/local/bin}
BINPATH=${DEST}${BIN}
MAN=${MANDIR:-/usr/local/man}
MANPATH=${DEST}${MAN}/man8
TOOLS="athstats 80211stats athkey athchans athctrl athdebug 80211debug wlanconfig"


while [ "$#" -gt 0 ]; do
	case ${1} in
		*ath*|*80211*|*wlanconfig*)
			FOUND="${FOUND}${1} "
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

if [ -z "${FOUND}" ]; then
	REGEX_TOOLS=$(echo ${TOOLS} | sed 's/\ /\\|/g')
	REGEX_MAN=$(echo ${TOOLS} | sed 's/\ /\.8\\|/g').8
	FOUND_TOOLS=$(find ${BINPATH} -type f -regex ".*\(${REGEX_TOOLS}\)" 2>/dev/null)
	FOUND_MAN=$(find ${MANPATH} -type f -regex ".*\(${REGEX_MAN}\)" 2>/dev/null)
	# This subshell ignores blank space of empty var's
	FOUND=$(echo ${FOUND_TOOLS} ${FOUND_MAN})
fi

if [ -n "${FOUND}" ]; then
	if [ "${QUIET}" = "noask" ]; then
		rm -f ${FOUND} || exit 1
	else
		echo
		echo "Old MadWifi tools found. Remove them?"
		
		while true; do
			echo
			echo -n "[l]ist, [r]emove, e[x]it (l,r,[x]) ? "
			echo
			read REPLY
			case ${REPLY} in
				l|L)
					for t in ${FOUND}; do echo ${t}; done
					continue
					;;
     		
				r|R) 	
					rm -f ${FOUND} || exit 1
					echo
					echo "Removed old MadWifi tools"
					echo
					exit 0
					;;
	                                
				*) 
					exit 1
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
