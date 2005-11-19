#!/bin/sh

if [ "$#" -ne 2 ] ; then
	echo "Purpose:"
	echo "Locate all madwifi-related userspace tools in a given directory"
	echo "(including all its subdirectories)."
	echo
	echo "Usage:"
	echo "$0 <bindir> <mandir>"
	echo
	echo "<bindir>: name of the directory where binaries are installed"
	echo "<mandir>: name of the directory where manpages are installed"
	echo
	exit 1
fi

SCRIPTS=$(dirname $0)
if [ -z "$SCRIPTS" ]; then
	SCRIPTS=.
fi

if [ -e "${SCRIPTS}"/../noask ]; then
	QUIET="noask"
fi

BINPATH=${1}
MANPATH=${2}

TOOLS="athstats 80211stats athkey athchans athctrl athdebug 80211debug wlanconfig"

if [ -d "${BINPATH}" ]; then
	REGEX_TOOLS=$(echo ${TOOLS} | sed 's/\ /\\|/g')
	FOUND_TOOLS=$(find ${BINPATH} -type f -regex ".*\(${REGEX_TOOLS}\)" 2>/dev/null)
fi

if [ -d "${MANPATH}" ]; then
	REGEX_MAN=$(echo ${TOOLS} | sed 's/\ /\.8\\|/g').8
	FOUND_MAN=$(find ${MANPATH} -type f -regex ".*\(${REGEX_MAN}\)" 2>/dev/null)
fi

if [ -n "${BINPATH}" -o -n "${MANPATH}" ]; then
	FOUND=$(echo ${FOUND_TOOLS} ${FOUND_MAN})
fi

if [ -n "${FOUND}" ]; then
	if [ "${QUIET}" = "noask" ]; then
		sh ${SCRIPTS}/remove-old-tools.sh noask ${FOUND}
		exit
	fi
	echo
	echo "WARNING:"
	echo "It seems that there are tools left from previous MadWifi installations."
	echo "You should consider removing them before you continue, or else you might"
	echo "experience problems during operation. Remove old tools?"
	
	while true; do
		echo
		echo -n "[l]ist, [r]emove, [i]gnore or e[x]it (l,r,i,[x]) ? "
		echo
		read REPLY
		case ${REPLY} in
			l|L)
				for t in ${FOUND}; do echo ${t}; done
				continue
				;;
			
			r|R)
				sh ${SCRIPTS}/remove-old-tools.sh noask ${FOUND}
				exit
				;;
		
			i|I)
				exit 0
				;;
	
			*)
				exit 1
				;;
		esac
	done
fi

exit 0
