#
# Copyright (c) 2002-2006 Sam Leffler, Errno Consulting
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
# $Id: //depot/sw/branches/sam_hal/public/mipsisa32-be-elf.inc#2 $
#

#
# Compilation configuration for building big-endian mipsisa32-elf.
#

# Configuration for building big-endian MIPS32 using the OpenWrt mips toolchain
# gcc version 4.1.2
# binutils version 2.17

CC=	${TOOLPREFIX}gcc
LD=	${TOOLPREFIX}ld
STRIP=	${TOOLPREFIX}strip
OBJCOPY=${TOOLPREFIX}objcopy
NM=	${TOOLPREFIX}nm

#
# Force register read/write operations to go through a function.
#
AH_REGOPS_FUNC=1

COPTS+=	-DAH_BYTE_ORDER=AH_BIG_ENDIAN
COPTS+=	-G 0 -mno-abicalls -fno-pic -march=mips32 -Wa,--trap \
	-fno-strict-aliasing -fno-common -fomit-frame-pointer -mlong-calls \
	-isystem ${KERNELPATH}/include
