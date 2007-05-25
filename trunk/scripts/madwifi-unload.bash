#!/bin/bash

for module in ath_{pci,rate_{amrr,minstrel,onoe,sample},hal} wlan{_{wep,tkip,ccmp,acl,xauth,scan_{sta,ap}},}
do
	grep -q ^$module /proc/modules && modprobe -r $module || true
done
