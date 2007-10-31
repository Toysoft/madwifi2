#!/bin/sh

IF=ath0
WLANDEV=wifi0

count=1

# trace <command> <logfilename>
trace() {
	echo 1 > /proc/sys/dev/ath/hal/alq
	#filename=`printf "%02d\n" $count`-$2.log
	filename=$2.log
	echo "'$1' -> $filename"
	$1
	sleep 1
	echo 0 > /proc/sys/dev/ath/hal/alq
	sed 's/\x00//g' /tmp/ath_hal.log > /tmp/$filename
	rm /tmp/ath_hal.log
	let count++
}

load_hal_debug() {
	modprobe ath_hal
	echo 2 > /proc/sys/dev/ath/hal/debug
}

madwifi-unload

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