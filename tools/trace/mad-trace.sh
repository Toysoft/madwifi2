#!/bin/sh

IF=ath0
WLANDEV=wifi0
TMP=/tmp/mad-trace
CONNECT=$1

count=1

# trace <command> <logfilename>
trace() {
	echo 1 > /proc/sys/dev/ath/hal/alq
	#filename=`printf "%02d\n" $count`-$2.log
	filename=$2.log
	echo "'$1' -> $filename"
	eval $1
	sleep 1
	echo 0 > /proc/sys/dev/ath/hal/alq
	sed 's/\x00//g' /tmp/ath_hal.log > $TMP/$filename
	rm /tmp/ath_hal.log
	let count++
}

load_hal_debug() {
	modprobe ath_hal
	echo 2 > /proc/sys/dev/ath/hal/debug
}

run_ath_info() {
	#Get memory address for ath_info...
	mem=`lspci -v | grep -A 3 'Atheros\|168c' | grep Memory | awk '{print $3}'`

	#Run ath_info and save output...
	echo "running ath_info -d 0x$mem..."
	ath_info -d 0x$mem > $TMP/ath_info.log
	mv ath-eeprom-dump.bin $TMP/
}

create_tar() {
	chip=`grep "MAC Revision" /tmp/mad-trace/ath_info.log | awk '{print $3}'`
	tar cvzf /tmp/$chip-mad-trace-$1.tgz $TMP
}

### "main" ###

rm -rf $TMP
mkdir -p $TMP

madwifi-unload

if [ -n "$CONNECT" ]; then
	echo "*** testbed connection tracing ***"
	echo "* be sure that you have set up the testbed (AP) correctly "
	echo "* b mode: essid bbb channel 10"
	echo "* g mode: essid ggg channel 10"
	echo "* a mode: essid aaa channel 60"
	echo "* gturbo mode: essid gturbo channel 6"
	echo "* aturbo mode: essid aturbo channel 42"
	echo "***********************************"

	load_hal_debug
	trace "modprobe ath_pci" "attach"
else
	echo "*** offline tracing ***************"
	echo "* (no testbed setup) to get more detailed results you can"
	echo "* use ./mad-trace.sh <MODE> for tracing connections in a testbed"
	echo "* <MODE> can be a b g gturbo aturbo"
	echo "***********************************"
fi


if [ "$CONNECT" == "g" ]; then
	trace "iwpriv $IF mode 3" "g-iwpriv-mode-3"
	iwconfig $IF essid ggg channel 10
	trace "ifconfig $IF up; while iwconfig | grep Not-Associated; do sleep 1; done" "g-ifconfig"
elif [ "$CONNECT" == "a" ]; then
	trace "iwpriv $IF mode 1" "a-iwpriv-mode-1"
	iwconfig $IF essid aaa channel 60
	trace "ifconfig $IF up; while iwconfig | grep Not-Associated; do sleep 1; done" "a-ifconfig"
elif [ "$CONNECT" == "b" ]; then
	trace "iwpriv $IF mode 3" "b-iwpriv-mode-3"
	iwconfig $IF essid bbb channel 10
	trace "ifconfig $IF up; while iwconfig | grep Not-Associated; do sleep 1; done" "b-ifconfig"
elif [ "$CONNECT" == "aturbo" ]; then
	iwconfig $IF essid aturbo channel 42
	trace "ifconfig $IF up; while iwconfig | grep Not-Associated; do sleep 1; done" "at-ifconfig"
elif [ "$CONNECT" == "gturbo" ]; then
	iwconfig $IF essid gturbo channel 6
	trace "ifconfig $IF up; while iwconfig | grep Not-Associated; do sleep 1; done" "gt-ifconfig"
fi
echo "* connection established. done."

if [ -n "$CONNECT" ]; then
	madwifi-unload

	run_ath_info
	create_tar "connect"

	exit
fi


### no testbed setup. trace all bands and modes ###

for opmode in "sta" "ap" "adhoc"; do
	for mode in 1 2 3; do
		count=1
		common="$opmode-mode$mode"
		load_hal_debug
		trace "modprobe ath_pci autocreate=none" "$common-$count-modprobe"
		trace "wlanconfig $IF create wlandev $WLANDEV wlanmode $opmode" "$common-$count-wlanconfig"
		trace "iwpriv $IF mode $mode" "$common-$count-set-mode"
		trace "ifconfig $IF up" "$common-$count-ifup"
		if [ $mode == 1 ]; then
			trace "iwconfig $IF channel 36" "$common-$count-chan36"
			trace "iwconfig $IF channel 40" "$common-$count-chan40"
			trace "iwconfig $IF channel 44" "$common-$count-chan44"
			trace "iwconfig $IF channel 48" "$common-$count-chan48"
		else
			trace "iwconfig $IF channel 1" "$common-$count-chan1"
			trace "iwconfig $IF channel 6" "$common-$count-chan6"
		fi
		trace "ifconfig $IF down" "$common-$count-ifdown"
		madwifi-unload
	done
done

run_ath_info
create_tar "all-modes"
