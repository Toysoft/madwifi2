#!/bin/sh

function usage {
    echo "Purpose:"
    echo "Locate all madwifi-related kernel modules in a given directory"
    echo "(including all its subdirectories)."
    echo
    echo "Usage:"
    echo "$0 <dir>"
    echo
    echo "<dir>: name of the directory used for the search"
    echo
}

if [ "$#" -ne 1 ]; then
    usage >&2
    exit 1
fi

SEARCH=$1

if [ ! -d "$SEARCH" ]; then
    echo "ERROR: $1 is no valid directory" >&2
    exit 1
fi

PATTERN="^.*\/(ath_(hal|pci|rate_(onoe|amrr|sample))\.k?o)|(wlan(_(acl|ccmp|scan_(ap|sta)|tkip|wep|xauth))?\.k?o)$"

find $SEARCH -type f -regex '.*\.k?o' | \
grep -w -E "$PATTERN" > /dev/null 2>&1 && {
    echo
    echo "WARNING:"
    echo "It seems that there are modules left from previous MadWifi installations."
    echo "You should consider to remove them before you continue, else you might"
    echo "experience problems during operation."
    echo
    
    read -p "Continue anyway? [y,N] "
    case $REPLY in
        y|Y)
		exit 0
		;;
	*)
		exit 1
		;;
    esac
}

exit 0
