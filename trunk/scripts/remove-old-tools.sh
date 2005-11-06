#!/bin/bash
# Purpose: provide support for removing tools and manpages from previous
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

[[ -r ${SCRIPTS}/../tools/install.log ]] && source ${SCRIPTS}/../tools/install.log

DEST="${DESTDIR}"
BIN="${BINDIR:-/usr/local/bin}"
BINPATH="${DEST}${BIN}"
MAN="${MANDIR:-/usr/local/man}"
MANPATH="${DEST}${MAN}/man8"

# Last space is important!
TOOLS="athstats 80211stats athkey athchans athctrl athdebug 80211debug wlanconfig "

function find_tools ()
{
if [[ ! -d ${BINPATH} ]]
then
	break
else
	for t in ${TOOLS}; do
		if [[ -e ${BINPATH}/${t} ]]
		then
			echo "${BINPATH}/${t}"
		fi
	done
fi

if [[ ! -d ${MANPATH} ]]
then
	break
else
	find ${MANPATH} -type f -regex ".*\(${TOOLS// /.8\|}\)" 2>/dev/null
fi
}

FOUND_TOOLS=$(find_tools)
if [[ ${FOUND_TOOLS} ]]
then
	echo
	echo "Old MadWifi tools found"
	echo
	while true; do
		echo -n "List old MadWifi tools? [y],n "
		read REPLY
		case ${REPLY} in
			n|N)
				break
				;;
     	
			y|Y) 	
				for t in ${FOUND_TOOLS}; do
					echo ${t}
				done
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
			n|N)
				exit 1
				;;
                       	
			y|Y) 	
				rm -f ${FOUND_TOOLS} || exit 1
				echo
				echo "Old tools removed"
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
	echo "No old tools found."
	echo
	exit 0
fi
