#!/bin/sh

if [ "$#" -ne 1 ] ; then
	echo "Purpose:"
	echo "Locate all madwifi-related kernel modules in a given directory"
	echo "(including all its subdirectories)."
	echo
	echo "Usage:"
	echo "$0 <dir>"
	echo
	echo "<dir>: name of the directory used for the search"
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

SEARCH=${1}

if [ -d "${SEARCH}" ]; then
	PATTERN="^.*\/(ath_(hal|pci|rate_(onoe|amrr|sample))\.k?o)|(wlan(_(acl|ccmp|scan_(ap|sta)|tkip|wep|xauth))?\.k?o)$"
	OLD_MODULES=$(find ${SEARCH} -type f -regex '.*\.k?o' 2>/dev/null | grep -w -E "${PATTERN}")
fi

if [ -n "${OLD_MODULES}" ]; then
	if [ "${QUIET}" = "noask" ]; then
		sh ${SCRIPTS}/remove-old-modules.sh noask ${OLD_MODULES}
		exit
	fi
	echo
	echo "WARNING:"
	echo "It seems that there are modules left from previous MadWifi installations."
	echo "You should consider removing them before you continue, or else you might"
	echo "experience problems during operation. Remove old modules?"
	
	while true; do
		echo
		echo -n "[l]ist, [r]emove, [i]gnore or e[x]it (l,r,i,[x]) ? "
		echo
		read REPLY
		case ${REPLY} in
			l|L)
				for m in ${OLD_MODULES}; do echo ${m}; done
				continue
				;;
			
			r|R)
				sh ${SCRIPTS}/remove-old-modules.sh noask ${OLD_MODULES}
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
