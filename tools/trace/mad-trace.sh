#!/bin/sh

IF=ath0
WLANDEV=wifi0
TMP=/tmp/mad-trace

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


### "main" ###

rm -rf $TMP
mkdir -p $TMP
rm -f /tmp/mad-trace.tgz

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

run_ath_info

chip=`grep "MAC Revision" /tmp/mad-trace/ath_info.log | awk '{print $3}'`
tar cvzf /tmp/$chip-mad-trace.tgz $TMP
