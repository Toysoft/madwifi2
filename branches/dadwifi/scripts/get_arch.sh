#!/bin/sh

# Report ARCH for the given Linux .config file.
# Argument 1 should be path to .config

test -r "$1" || { echo ""; exit 1; }
. "$1"

# Calculate ARCH first.  This covers all architectures supported by
# Linux 2.4 and 2.6, whether they are supported by HAL or not.
# Note that more specific entries must follow less specific ones, e.g.
# CONFIG_X86_64 overrides CONFIG_X86.
eval ARCH_${CONFIG_ALPHA}=alpha
eval ARCH_${CONFIG_ARM}=arm
eval ARCH_${CONFIG_ARM26}=arm26
eval ARCH_${CONFIG_CRIS}=cris
eval ARCH_${CONFIG_FRV}=frv
eval ARCH_${CONFIG_H8300}=h8300
eval ARCH_${CONFIG_X86}=i386
eval ARCH_${CONFIG_IA64}=ia64
eval ARCH_${CONFIG_M32R}=m32r
eval ARCH_${CONFIG_M68K}=m68k
eval ARCH_${CONFIG_M68KNOMMU}=m68knommu
eval ARCH_${CONFIG_MIPS}=mips
eval ARCH_${CONFIG_MIPS64}=mips64
eval ARCH_${CONFIG_PARISC}=parisc
eval ARCH_${CONFIG_PPC}=ppc
eval ARCH_${CONFIG_PPC64}=ppc64
eval ARCH_${CONFIG_PPC_MERGE}=powerpc
eval ARCH_${CONFIG_ARCH_S390}=s390
eval ARCH_${CONFIG_ARCH_S390X}=s390x
eval ARCH_${CONFIG_SUPERH}=sh
eval ARCH_${CONFIG_CPU_SH5}=sh64
eval ARCH_${CONFIG_SUPERH64}=sh64
eval ARCH_${CONFIG_SPARC32}=sparc
eval ARCH_${CONFIG_SPARC64}=sparc64
eval ARCH_${CONFIG_UML}=um
eval ARCH_${CONFIG_V850}=v850
eval ARCH_${CONFIG_X86_64}=x86_64
eval ARCH_${CONFIG_XTENSA}=xtensa
ARCH=${ARCH_y}
test -z "${ARCH}" && { echo ""; exit 1; }

echo "${ARCH}"
exit 0
