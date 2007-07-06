#!/usr/bin/perl
#
# Copyright (c) 2007 Michael Taylor
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce at minimum a disclaimer
#    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
#    redistribution must be conditioned upon including a substantially
#    similar Disclaimer requirement for further binary redistribution.
# 3. Neither the names of the above-listed copyright holders nor the names
#    of any contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# Alternatively, this software may be distributed under the terms of the
# GNU General Public License ("GPL") version 2 as published by the Free
# Software Foundation.
#
# NO WARRANTY
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGES.
#
# $Id: foo $
#
# This script is invoked at build time to regenerate a wrapper for the HAL
# binary API that adds locking and (optionally) tracing with human readable 
# names.
#
use strict;
use warnings;
require 'dumpvar.pl';

#
# This section contains the output file paths
#
my $path_to_hal  = 'hal';
my $path_to_ath  = 'ath';
my $hal_h        = 'ah.h';
my $if_ath_hal_h = 'if_ath_hal.h';

#
# This section defines the name translation from the binary HAL's function
# pointers to our API names.
#
my %hal_function_name_to_madwifi_name = (
	"ah_beaconInit"               => "ath_hal_beaconinit",
	"ah_disablePhyErrDiag"        => "ath_hal_disablePhyDiag",
	"ah_enablePhyErrDiag"         => "ath_hal_enablePhyDiag",
	"ah_enableReceive"            => "ath_hal_rxena",
	"ah_fillTxDesc"               => "ath_hal_filltxdesc",
	"ah_getAckCTSRate"            => "ath_hal_getackctsrate",
	"ah_setAckCTSRate"            => "ath_hal_setackctsrate",
	"ah_updateMibCounters"        => "ath_hal_updatemibcounters",
	"ah_getAntennaSwitch"         => "ath_hal_getantennaswitch",
	"ah_setAntennaSwitch"         => "ath_hal_setantennaswitch",
	"ah_getAckTimeout"            => "ath_hal_getacktimeout",
	"ah_getBssIdMask"             => "ath_hal_getbssidmask",
	"ah_getCapability"            => "ath_hal_getcapability",
	"ah_getChanNoise"             => "ath_hal_get_channel_noise",
	"ah_getCTSTimeout"            => "ath_hal_getctstimeout",
	"ah_getDefAntenna"            => "ath_hal_getdefantenna",
	"ah_getDiagState"             => "ath_hal_getdiagstate",
	"ah_getInterrupts"            => "ath_hal_intrget",
	"ah_getKeyCacheSize"          => "ath_hal_keycachesize",
	"ah_getMacAddress"            => "ath_hal_getmac",
	"ah_getPendingInterrupts"     => "ath_hal_getisr",
	"ah_getPowerMode"             => "ath_hal_getPowerMode",
	"ah_getRateTable"             => "ath_hal_getratetable",
	"ah_getRfGain"                => "ath_hal_getrfgain",
	"ah_getRxDP"                  => "ath_hal_getrxbuf",
	"ah_getRxFilter"              => "ath_hal_getrxfilter",
	"ah_getSlotTime"              => "ath_hal_getslottime",
	"ah_getTsf32"                 => "ath_hal_gettsf32",
	"ah_getTsf64"                 => "ath_hal_gettsf64",
	"ah_getTxDP"                  => "ath_hal_gettxbuf",
	"ah_getTxIntrQueue"           => "ath_hal_gettxintrtxqs",
	"ah_getTxQueueProps"          => "ath_hal_gettxqueueprops",
	"ah_gpioCfgOutput"            => "ath_hal_gpioCfgOutput",
	"ah_gpioSet"                  => "ath_hal_gpioset",
	"ah_gpioGet"                  => "ath_hal_gpioget",
	"ah_gpioSetIntr"              => "ath_hal_gpiosetintr",
	"ah_gpioCfgInput"             => "ath_hal_gpiCfgInput",
	"ah_isInterruptPending"       => "ath_hal_intrpend",
	"ah_isKeyCacheEntryValid"     => "ath_hal_keyisvalid",
	"ah_numTxPending"             => "ath_hal_numtxpending",
	"ah_perCalibration"           => "ath_hal_calibrate",
	"ah_phyDisable"               => "ath_hal_phydisable",
	"ah_disable"                  => "ath_hal_disable",
	"ah_procMibEvent"             => "ath_hal_mibevent",
	"ah_procRxDesc"               => "ath_hal_rxprocdesc",
	"ah_procTxDesc"               => "ath_hal_txprocdesc",
	"ah_radarWait"                => "ath_hal_radar_wait",
	"ah_releaseTxQueue"           => "ath_hal_releasetxqueue",
	"ah_reqTxIntrDesc"            => "ath_hal_txreqintrdesc",
	"ah_reset"                    => "ath_hal_reset",
	"ah_resetKeyCacheEntry"       => "ath_hal_keyreset",
	"ah_resetStationBeaconTimers" => "ath_hal_beaconreset",
	"ah_resetTsf"                 => "ath_hal_resettsf",
	"ah_resetTxQueue"             => "ath_hal_resettxqueue",
	"ah_rxMonitor"                => "ath_hal_rxmonitor",
	"ah_setAckTimeout"            => "ath_hal_setacktimeout",
	"ah_setBssIdMask"             => "ath_hal_setbssidmask",
	"ah_setCapability"            => "ath_hal_setcapability",
	"ah_setChannel"               => "ath_hal_setchannel",
	"ah_setCoverageClass"         => "ath_hal_setcoverageclass",
	"ah_setCTSTimeout"            => "ath_hal_setctstimeout",
	"ah_setDecompMask"            => "ath_hal_setdecompmask",
	"ah_setDefAntenna"            => "ath_hal_setdefantenna",
	"ah_setInterrupts"            => "ath_hal_intrset",
	"ah_setKeyCacheEntry"         => "ath_hal_keyset",
	"ah_setKeyCacheEntryMac"      => "ath_hal_keysetmac",
	"ah_setLedState"              => "ath_hal_setledstate",
	"ah_setMacAddress"            => "ath_hal_setmac",
	"ah_setMulticastFilter"       => "ath_hal_setmcastfilter",
	"ah_setMulticastFilterIndex"  => "ath_hal_setmcastfilterindex",
	"ah_setPCUConfig"             => "ath_hal_setopmode",
	"ah_setPowerMode"             => "ath_hal_setpower",
	"ah_setRxDP"                  => "ath_hal_putrxbuf",
	"ah_setRxFilter"              => "ath_hal_setrxfilter",
	"ah_setRegulatoryDomain"      => "ath_hal_setregulatorydomain",
	"ah_setSlotTime"              => "ath_hal_setslottime",
	"ah_setStationBeaconTimers"   => "ath_hal_beacontimers",
	"ah_setTxDP"                  => "ath_hal_puttxbuf",
	"ah_setTxQueueProps"          => "ath_hal_settxqueueprops",
	"ah_setTxPowerLimit"          => "ath_hal_settxpowlimit",
	"ah_setBeaconTimers"          => "ath_hal_setbeacontimers",
	"ah_setupRxDesc"              => "ath_hal_setuprxdesc",
	"ah_setupTxDesc"              => "ath_hal_setuptxdesc",
	"ah_setupTxQueue"             => "ath_hal_setuptxqueue",
	"ah_setupXTxDesc"             => "ath_hal_setupxtxdesc",
	"ah_startPcuReceive"          => "ath_hal_startpcurecv",
	"ah_startTxDma"               => "ath_hal_txstart",
	"ah_stopDmaReceive"           => "ath_hal_stopdmarecv",
	"ah_stopPcuReceive"           => "ath_hal_stoppcurecv",
	"ah_stopTxDma"                => "ath_hal_stoptxdma",
	"ah_updateCTSForBursting"     => "ath_hal_updateCTSForBursting",
	"ah_updateTxTrigLevel"        => "ath_hal_updatetxtriglevel",
	"ah_waitForBeaconDone"        => "ath_hal_waitforbeacon",
	"ah_writeAssocid"             => "ath_hal_setassocid",
	"ah_clrMulticastFilterIndex"  => "ath_hal_clearmcastfilter",
	"ah_detectCardPresent"        => "ath_hal_detectcardpresent" );
#
# List any functions that should NOT be generated here (such as those that conflict with
# other functions, perhaps.
#
my @hal_functions_not_to_wrap = ( "ah_detach" );

#
# Boilerplate text
#
my $header_comment = <<EOF
/*-
 * Copyright (c) 2007 Michael Taylor
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
	without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */
/* **************************************************************
 * *   WARNING: THIS IS A GENERATED FILE.  PLEASE DO NOT EDIT   *
 * ************************************************************** */
EOF
;

#
# This text is generated verbatim at the top of the .h file,
# before generated content is added.
#
my $header_for_h = <<EOF
$header_comment

#include "if_ath_hal_macros.h"

#ifndef _IF_ATH_HAL_H_
#define _IF_ATH_HAL_H_
EOF
;
my $footer_for_h = <<EOF

#include "if_ath_hal_wrappers.h"

#endif /* #ifndef _IF_ATH_HAL_H_ */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
EOF
;

#
# This text is generated verbatim at the top of the .c file
#
my $header_for_c = <<EOF
$header_comment

#ifdef ATH_HALOPS_TRACEABLE

EOF
;
#include "opt_ah.h"
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <asm/uaccess.h>

#
# This text is generated verbatim at the bottom of the .c file
#
my $footer_for_c = <<EOF
#endif /* #ifdef ATH_HALOPS_TRACEABLE */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
EOF
;

# Include settings, calculate a few new ones
my $path_to_ah_h = "$path_to_hal/$hal_h";
my $path_to_if_ath_hal_h = "$path_to_ath/$if_ath_hal_h";

# Parsed Function Data 

# list of declarations in document order
my @hal_prototypes = ();
# hash of string->string (hal's function name to return type)
my %hal_functionname_to_return_type = ();
# hash of string->list of strings (ordered list of parameter names)
my %hal_functionname_to_parameter_name_array = ();
# hash of string->list of strings (ordered list of parameter types)
my %hal_functionname_to_parameter_types_array = ();

# Open the files we need
if(!open AH_H, "<$path_to_ah_h") {
   die "Cannot open $path_to_ah_h: $!";
}
if(!open ATH_HAL_API_H, ">$path_to_if_ath_hal_h") {
   close AH_H;
   die "Cannot open $path_to_if_ath_hal_h: $!";
}

# Parse and scrub the hal structure's member function declarations 
my $line_continued = 0;
my $line_buffer = "";
foreach (<AH_H>) {
   chomp($_);
   s/\s+$//g;
   s/^\s+//g;
   s/\s+/ /g;
   if (/__ahdecl\s*\(.*/ || $line_continued) {
      $line_buffer .= "$_";
      if (/__ahdecl.*;/ || ($line_continued && /;/)) {
	 push @hal_prototypes, $line_buffer;
	 $line_buffer = "";
	 $line_continued = 0;
      }
      else {
	 $line_buffer .= " ";
	 $line_continued = 1;
      }
   }
}

# Now pick apart the return type, parameter types, and parameter names for each HAL function
foreach my $proto (@hal_prototypes) {
   $proto =~ /^((?:(?:const|struct)\s*)*[^\s]+(?:[\s]*\*)?)[\s]*__ahdecl\(\*([^\)]*)\)\((.*)\);/;
   my $return_type   = $1;
   my $member_name   = $2;
   my $parameterlist = $3;
   if(! grep{/$member_name/} @hal_functions_not_to_wrap ) {
      $hal_functionname_to_return_type{"$member_name"} = $return_type;
      @{$hal_functionname_to_parameter_name_array{"$member_name"}} = ();
      @{$hal_functionname_to_parameter_types_array{"$member_name"}} = ();
      my @parameters = split /,\s?/, $parameterlist;
      my $argnum = 0;
      my $first = 1;
      foreach(@parameters) {
	 $_ =~ s/ \*/\* /;
	 $_ =~ /^((?:(?:const|struct|\*)\s*)*)([^\s]+\*?)\s*([^\s]*)\s*/;
	 my $type = "$1$2";
	 my $name = "$3";
	 if(0 == length($name)) {
	    if($argnum == 0 && $type =~ /ath_hal/) {
	       $name = "ah";
	    }
	    else {
	       $name = "a" . $argnum;
	    }
	 }
	 
	 push @{$hal_functionname_to_parameter_name_array{$member_name}}, $name;
	 push @{$hal_functionname_to_parameter_types_array{$member_name}}, $type;
	 $first = 0;
	 $argnum++;
      }
   }
}

# Generate the header file
print ATH_HAL_API_H $header_for_h;

for my $member_name (keys %hal_functionname_to_return_type) {
   my $api_return_type   = $hal_functionname_to_return_type{$member_name};
   my $api_name      	 = $member_name;
   if(exists $hal_function_name_to_madwifi_name{$member_name}) {
      $api_name = $hal_function_name_to_madwifi_name{$member_name};
   }   
   print ATH_HAL_API_H "static inline " . $api_return_type . " " . $api_name . "(";
   my @names = @{$hal_functionname_to_parameter_name_array{$member_name}};
   my @types = @{$hal_functionname_to_parameter_types_array{$member_name}};
   for my $i (0..$#names) {
      if($i) {
	 print ATH_HAL_API_H ", ";
      }
      print ATH_HAL_API_H $types[$i] . " " . $names[$i];
   }
   print ATH_HAL_API_H ")\n\t{";
   if(! ($api_return_type =~ /void/ )) {
      print ATH_HAL_API_H "\n\t\t" . $api_return_type . " ret;";
   }
   print ATH_HAL_API_H "\n\t\tATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));";
   print ATH_HAL_API_H "\n\t\t";
   if(! ($api_return_type =~ /void/ )) {
      print ATH_HAL_API_H "ret = ";
   }

   print ATH_HAL_API_H "ah->$member_name(";
   for my $j (0..$#names) {
      if($j) {
	 print ATH_HAL_API_H ", ";
      }
      print ATH_HAL_API_H $names[$j];
   }
   print ATH_HAL_API_H ");";
   print ATH_HAL_API_H "\n\t\tATH_HAL_UNLOCK_IRQ(GET_ATH_SOFTC(ah));";
   if(! ($api_return_type =~ /void/ )) {
      print ATH_HAL_API_H "\n\t\treturn ret;";
   }
   print ATH_HAL_API_H "\n\t}\n";
}
print ATH_HAL_API_H $footer_for_h;

# Close up the files
close AH_H;
close ATH_HAL_API_H;

