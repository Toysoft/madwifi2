#!/bin/sh

function usage ()
{
	echo "Purpose:"
	echo "Locate all madwifi-related kernel modules in a given directory"
	echo "(including all its subdirectories)."
	echo
	echo "Usage:"
	echo "$0 <dir> [noask]"
	echo
	echo "<dir>: name of the directory used for the search"
	echo "noask: automatically answer 'yes' to all questions."
	echo
}

if [ "$#" -gt 2 -o "$#" -lt 1 ] ; then
	usage >&2
	exit 1
fi

QUIET=$2
SEARCH=$1

function probe_for_existing_madwifi ()
{
	PATTERN="^.*\/(ath_(hal|pci|rate_(onoe|amrr|sample))\.k?o)|(wlan(_(acl|ccmp|scan_(ap|sta)|tkip|wep|xauth))?\.k?o)$"

	find $SEARCH -type f -regex '.*\.k?o' | \
	grep -w -E "$PATTERN" > /dev/null 2>&1 && {
	echo
	echo "WARNING:"
	echo "It seems that there are modules left from previous MadWifi installations."
	echo "You should consider removing them before you continue, or else you might"
	echo "experience problems during operation."
	echo

	if [ "$QUIET" = "noask" ]; then
		REPLY='y'
	else
		read -p "Remove old modules? [y,N]"
	fi
	case $REPLY in
		y|Y|yes)
		scripts/remove-old-modules.sh $QUIET
		exit 0
		;;
		*)
		read -p "Continue anyway? [y,N] "
		case $REPLY in
			y|Y)
			exit 0
			;;
			*)
			exit 1
			;;
		esac
		;;
	esac
}
}

if [ ! -d "$SEARCH" ]; then
	break
else
	probe_for_existing_madwifi
fi

exit 0
