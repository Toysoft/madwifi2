/*-
 * Copyright (c) 2007 Nick Kossifidis <mickflemm@gmail.com>
 * 
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/* So here is how it works:
 * 
 * First compile...
 * 
 * gcc ath_info.c -o ath_info
 * 
 * then find card's physical address
 * 
 * lspci -v
 * 
 * 02:02.0 Ethernet controller: Atheros Communications, Inc. AR5212 802.11abg NIC (rev 01)
 *         Subsystem: Fujitsu Limited. Unknown device 1234
 *         Flags: bus master, medium devsel, latency 168, IRQ 23
 *         Memory at c2000000 (32-bit, non-prefetchable) [size=64K]
 *         Capabilities: [44] Power Management version 2
 * 
 * address here is 0xc2000000
 * 
 * load madwifi-ng or madwifi-old if not already loaded (be sure the 
 * interface is down!)
 * 
 * modprobe ath_pci
 * 
 * and we run the thing...
 * 
 * ./ath_info 0xc2000000
 * 
 * Use at your own risk, entering a false device address will have really 
 * nasty results! */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#define AR5K_PCI_MEM_SIZE 0x10000
#define AR5K_ELEMENTS(_array)	(sizeof(_array) / sizeof(_array[0]))

/*
 * Common silicon revision/version values
 */
enum ath5k_srev_type {
	AR5K_VERSION_VER,
	AR5K_VERSION_REV,
	AR5K_VERSION_RAD,
};

struct ath5k_srev_name {
	const char		*sr_name;
	enum ath5k_srev_type	sr_type;
	u_int			sr_val;
};

#define AR5K_SREV_NAME	{						\
	{ "5210  ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5210 },	\
	{ "5311  ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311 },	\
	{ "5311a ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311A },\
	{ "5311b ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311B },\
	{ "5211  ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5211 },	\
	{ "5212  ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5212 },	\
	{ "5213  ",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213 },	\
	{ "xxxxx ",	AR5K_VERSION_VER,	AR5K_SREV_UNKNOWN },	\
	{ "5110  ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5110 },	\
	{ "5111  ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111 },	\
	{ "2111  ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2111 },	\
	{ "5112  ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112 },	\
	{ "5112a ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112A },	\
	{ "2112  ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112 },	\
	{ "2112a ",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112A },	\
	{ "xxxxx ",	AR5K_VERSION_RAD,	AR5K_SREV_UNKNOWN },	\
}

#define AR5K_SREV_UNKNOWN	0xffff

#define AR5K_SREV_VER_AR5210	0x00
#define AR5K_SREV_VER_AR5311	0x10
#define AR5K_SREV_VER_AR5311A	0x20
#define AR5K_SREV_VER_AR5311B	0x30
#define AR5K_SREV_VER_AR5211	0x40
#define AR5K_SREV_VER_AR5212	0x50
#define AR5K_SREV_VER_AR5213	0x55
#define AR5K_SREV_VER_UNSUPP	0x60

#define AR5K_SREV_RAD_5110	0x00
#define AR5K_SREV_RAD_5111	0x10
#define AR5K_SREV_RAD_5111A	0x15
#define AR5K_SREV_RAD_2111	0x20
#define AR5K_SREV_RAD_5112	0x30
#define AR5K_SREV_RAD_5112A	0x35
#define AR5K_SREV_RAD_2112	0x40
#define AR5K_SREV_RAD_2112A	0x45
#define AR5K_SREV_RAD_UNSUPP	0x50

/*
 * Silicon revision register
 */
#define AR5K_SREV		0x4020			/* Register Address */
#define AR5K_SREV_REV		0x0000000f	/* Mask for revision */
#define AR5K_SREV_REV_S		0
#define AR5K_SREV_VER		0x000000ff	/* Mask for version */
#define AR5K_SREV_VER_S		4

/*
 * PHY chip revision register
 */
#define	AR5K_PHY_CHIP_ID		0x9818

/*
 * PHY register
 */
#define	AR5K_PHY_BASE			0x9800
#define	AR5K_PHY(_n)			(AR5K_PHY_BASE + ((_n) << 2))
#define AR5K_PHY_SHIFT_2GHZ		0x00004007
#define AR5K_PHY_SHIFT_5GHZ		0x00000007

#define AR5K_PCICFG			0x4010			/* Register Address */
#define AR5K_PCICFG_EEAE		0x00000001	/* Eeprom access enable [5210] */
#define AR5K_PCICFG_CLKRUNEN		0x00000004	/* CLKRUN enable [5211+] */
#define AR5K_PCICFG_EESIZE		0x00000018	/* Mask for EEPROM size [5211+] */
#define AR5K_PCICFG_EESIZE_S		3
#define AR5K_PCICFG_EESIZE_4K		0		/* 4K */
#define AR5K_PCICFG_EESIZE_8K		1		/* 8K */
#define AR5K_PCICFG_EESIZE_16K		2		/* 16K */
#define AR5K_PCICFG_EESIZE_FAIL		3		/* Failed to get size (?) [5211+] */

#define AR5K_EEPROM_BASE	0x6000

#define AR5K_EEPROM_MAGIC		0x003d		/* Offset for EEPROM Magic number */
#define AR5K_EEPROM_MAGIC_VALUE		0x5aa5	   /* Default - found on EEPROM*/
#define AR5K_EEPROM_MAGIC_5212		0x0000145c /* 5212 */
#define AR5K_EEPROM_MAGIC_5211		0x0000145b /* 5211 */
#define AR5K_EEPROM_MAGIC_5210		0x0000145a /* 5210 */

/*
 * EEPROM data register
 */
#define AR5K_EEPROM_DATA_5211	0x6004
#define AR5K_EEPROM_DATA_5210	0x6800
#define	AR5K_EEPROM_DATA	(mac_version == AR5K_SREV_VER_AR5210 ? \
				AR5K_EEPROM_DATA_5210 : AR5K_EEPROM_DATA_5211)

/*
 * EEPROM command register
 */
#define AR5K_EEPROM_CMD		0x6008			/* Register Addres */
#define AR5K_EEPROM_CMD_READ	0x00000001	/* EEPROM read */
#define AR5K_EEPROM_CMD_WRITE	0x00000002	/* EEPROM write */
#define AR5K_EEPROM_CMD_RESET	0x00000004	/* EEPROM reset */

/*
 * EEPROM status register
 */
#define AR5K_EEPROM_STAT_5210	0x6c00			/* Register Address [5210] */
#define AR5K_EEPROM_STAT_5211	0x600c			/* Register Address [5211+] */
#define	AR5K_EEPROM_STATUS	(mac_version == AR5K_SREV_VER_AR5210 ? \
				AR5K_EEPROM_STAT_5210 : AR5K_EEPROM_STAT_5211)
#define AR5K_EEPROM_STAT_RDERR	0x00000001	/* EEPROM read failed */
#define AR5K_EEPROM_STAT_RDDONE	0x00000002	/* EEPROM read successful */
#define AR5K_EEPROM_STAT_WRERR	0x00000004	/* EEPROM write failed */
#define AR5K_EEPROM_STAT_WRDONE	0x00000008	/* EEPROM write successful */

#define AR5K_EEPROM_REG_DOMAIN		0x00bf	/* Offset for EEPROM regulatory domain */	
#define AR5K_EEPROM_INFO_BASE		0x00c0  /* Offset for EEPROM header */
#define AR5K_EEPROM_INFO_MAX		(0x400 - AR5K_EEPROM_INFO_BASE)
#define AR5K_EEPROM_INFO_CKSUM		0xffff
#define AR5K_EEPROM_INFO(_n)		(AR5K_EEPROM_INFO_BASE + (_n))
#define AR5K_EEPROM_MODE_11A		0
#define AR5K_EEPROM_MODE_11B		1
#define AR5K_EEPROM_MODE_11G		2

#define AR5K_EEPROM_VERSION		AR5K_EEPROM_INFO(1)

#define AR5K_EEPROM_HDR			AR5K_EEPROM_INFO(2)	/* Header that contains the device caps */
#define AR5K_EEPROM_HDR_11A(_v)		(((_v) >> AR5K_EEPROM_MODE_11A) & 0x1)	/* Device has a support */
#define AR5K_EEPROM_HDR_11B(_v)		(((_v) >> AR5K_EEPROM_MODE_11B) & 0x1)	/* Device has b support */
#define AR5K_EEPROM_HDR_11G(_v)		(((_v) >> AR5K_EEPROM_MODE_11G) & 0x1)	/* Device has g support */
#define AR5K_EEPROM_HDR_T_2GHZ_DIS(_v)	(((_v) >> 3) & 0x1)	/* Disable turbo for 2Ghz (?) */
#define AR5K_EEPROM_HDR_T_5GHZ_DBM(_v)	(((_v) >> 4) & 0x7f)	/* Max turbo power for a/XR mode (eeprom_init) */
#define AR5K_EEPROM_HDR_DEVICE(_v)	(((_v) >> 11) & 0x7)
#define AR5K_EEPROM_HDR_T_5GHZ_DIS(_v)	(((_v) >> 15) & 0x1)	/* Disable turbo for 5Ghz (?) */
#define AR5K_EEPROM_HDR_RFKILL(_v)	(((_v) >> 14) & 0x1)	/* Device has RFKill support */

/* Misc values available since EEPROM 4.0 */
#define AR5K_EEPROM_MISC0		0x00c4
#define AR5K_EEPROM_EARSTART(_v)	((_v) & 0xfff)
#define AR5K_EEPROM_EEMAP(_v)		(((_v) >> 14) & 0x3)
#define AR5K_EEPROM_MISC1		0x00c5
#define AR5K_EEPROM_TARGET_PWRSTART(_v)	((_v) & 0xfff)
#define AR5K_EEPROM_HAS32KHZCRYSTAL(_v)	(((_v) >> 14) & 0x1)

/*
 * Read data by masking
 */
#define AR5K_REG_MS(_val, _flags)	\
	(((_val) & (_flags)) >> _flags##_S)

/*
 * Read from a device register
 */
#define AR5K_REG_READ(_reg)		\
	(*((volatile unsigned long int *)(mem + (_reg))))

/*
 * Write to a device register
 */
#define AR5K_REG_WRITE(_reg, _val)	\
	(*((volatile unsigned long int *)(mem + (_reg))) = (_val))

#define AR5K_REG_ENABLE_BITS(_reg, _flags)	\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) | (_flags))

#define AR5K_REG_DISABLE_BITS(_reg, _flags)	\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) & ~(_flags))

#define AR5K_TUNE_REGISTER_TIMEOUT		20000


static u_int32_t
ath5k_hw_bitswap(u_int32_t val, u_int bits)
{
	u_int32_t retval = 0, bit, i;

	for (i = 0; i < bits; i++) {
		bit = (val >> i) & 1;
		retval = (retval << 1) | bit;
	}

	return (retval);
}

/*
 * Get the PHY Chip revision
 */
u_int16_t
ath5k_hw_radio_revision(u_int16_t mac_version, void *mem, u_int8_t chip)
{
	int i;
	u_int32_t srev;
	u_int16_t ret;

	/*
	 * Set the radio chip access register
	 */
	switch (chip) {
	case 0:
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_2GHZ);
		break;
	case 1:
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);
		break;
	default:
		return (0);
	}

	usleep(2000);

	/* ...wait until PHY is ready and read the selected radio revision */
	AR5K_REG_WRITE(AR5K_PHY(0x34), 0x00001c16);

	for (i = 0; i < 8; i++)
		AR5K_REG_WRITE(AR5K_PHY(0x20), 0x00010000);

	if (mac_version == AR5K_SREV_VER_AR5210) {
		srev = AR5K_REG_READ(AR5K_PHY(256) >> 28) & 0xf;

		ret = (u_int16_t) ath5k_hw_bitswap(srev, 4) + 1;
	} else {
		srev = (AR5K_REG_READ(AR5K_PHY(0x100)) >> 24) & 0xff;

		ret = (u_int16_t) ath5k_hw_bitswap(((srev & 0xf0) >> 4) | 
				((srev & 0x0f) << 4), 8);
	}

	/* Reset to the 5GHz mode */
	AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);

	return (ret);
}

/*
 * Read from EEPROM
 */
int
ath5k_hw_eeprom_read(void *mem, u_int32_t offset, u_int16_t *data, 
		u_int8_t mac_version)
{
	u_int32_t status, timeout;

	/*
	 * Initialize EEPROM access
	 */
	if (mac_version == AR5K_SREV_VER_AR5210) {
		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, AR5K_PCICFG_EEAE);
		(void)AR5K_REG_READ(AR5K_EEPROM_BASE + (4 * offset));
	} else {
		AR5K_REG_WRITE(AR5K_EEPROM_BASE, offset);
		AR5K_REG_ENABLE_BITS(AR5K_EEPROM_CMD,
				AR5K_EEPROM_CMD_READ);
	}

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = AR5K_REG_READ(AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_RDDONE) {
			if (status & AR5K_EEPROM_STAT_RDERR)
				return 1;
			*data = (u_int16_t)
			    (AR5K_REG_READ(AR5K_EEPROM_DATA) & 0xffff);
			return (0);
		}
		usleep(15);
	}

	return 1;
}

const char *
ath5k_hw_get_part_name(enum ath5k_srev_type type, u_int32_t val)
{
	struct ath5k_srev_name names[] = AR5K_SREV_NAME;
	const char *name = "xxxxx ";
	int i;

	for (i = 0; i < AR5K_ELEMENTS(names); i++) {
		if (names[i].sr_type != type ||
		    names[i].sr_val == AR5K_SREV_UNKNOWN)
			continue;
		if ((val & 0xff) < names[i + 1].sr_val) {
			name = names[i].sr_name;
			break;
		}
	}

	return (name);
}

int
main(int argc, char *argv[])
{
	u_int32_t dev_addr;
	u_int16_t eeprom_header, srev, phy_rev_5ghz, phy_rev_2ghz;
	u_int16_t eeprom_version, mac_version, regdomain, has_crystal, ee_magic;
	u_int8_t error, has_a, has_b, has_g, has_rfkill, eeprom_size;
	void *mem;
	int fd;

	if ((argc < 2) || (argc > 2)) {
		printf("Usage: ath5k_info <phys address> \n");
		return -1;
	}

	dev_addr = strtoul(argv[1], NULL, 0);

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		printf("Open of /dev/mem failed!\n");
		return -2;
	}

	mem = mmap(0, AR5K_PCI_MEM_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_FILE, fd, dev_addr);

	if (mem == (void *) -1) {
		printf("Mmap of device at 0x%08X for 0x%X bytes failed!\n",
			dev_addr, AR5K_PCI_MEM_SIZE);
		return -3;
	}

	srev = AR5K_REG_READ(AR5K_SREV);
	mac_version = AR5K_REG_MS(srev, AR5K_SREV_VER) << 4;

	/* Verify eeprom magic value first */
	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_MAGIC, &ee_magic,
					mac_version);

	if (error) {
		printf("Unable to read EEPROM Magic value !\n");
		return -1;
	}

	if (ee_magic != AR5K_EEPROM_MAGIC_VALUE) {
		printf("Invalid EEPROM Magic number !\n");
		return -1;
	}

	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_HDR, &eeprom_header,
					mac_version);

	if (error) {
		printf("Unable to read EEPROM Header !\n");
		return -1;
	}

	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_VERSION, &eeprom_version,
					mac_version);

	if (error) {
		printf("Unable to read EEPROM version !\n");
		return -1;
	}

	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_REG_DOMAIN, &regdomain,
					mac_version);

	if (error) {
		printf("Unable to read Regdomain !\n");
		return -1;
	}

	if (eeprom_version >= 0x4000) {
		error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_MISC0, 
				&has_crystal, mac_version);

		if (error) {
			printf("Unable to read EEPROM Misc data !\n");
			return -1;
		}

		has_crystal = AR5K_EEPROM_HAS32KHZCRYSTAL(has_crystal);
	} else {
		has_crystal = 2;
	}

	eeprom_size = AR5K_REG_MS(AR5K_REG_READ(AR5K_PCICFG), 
			AR5K_PCICFG_EESIZE);

	has_a = AR5K_EEPROM_HDR_11A(eeprom_header);
	has_b = AR5K_EEPROM_HDR_11B(eeprom_header);
	has_g = AR5K_EEPROM_HDR_11G(eeprom_header);
	has_rfkill = AR5K_EEPROM_HDR_RFKILL(eeprom_header);

	if (has_a)
		phy_rev_5ghz = ath5k_hw_radio_revision(mac_version, mem, 1);
	else
		phy_rev_5ghz = 0;

	if (has_b)
		phy_rev_2ghz = ath5k_hw_radio_revision(mac_version, mem, 0);
	else
		phy_rev_2ghz = 0;

	printf(" -==Device Information==-\n");

	printf("MAC Version:  %s(0x%x) \n",
		ath5k_hw_get_part_name(AR5K_VERSION_VER, mac_version),
		mac_version);
		
	printf("MAC Revision: %s(0x%x) \n",
		ath5k_hw_get_part_name(AR5K_VERSION_VER, srev),
		srev);

	/* Single-chip PHY with a/b/g support */
	if (has_b && !phy_rev_2ghz) {
		printf("PHY Revision: %s(0x%x) \n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_5ghz),
			phy_rev_5ghz);
		phy_rev_5ghz = 0;
	}

	/* Single-chip PHY with b/g support */
	if (!has_a) {
		printf("PHY Revision: %s(0x%x) \n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_2ghz),
			phy_rev_2ghz);
		phy_rev_2ghz = 0;
	}

	/* Different chip for 5Ghz and 2Ghz */
	if (phy_rev_5ghz) {
		printf("5Ghz PHY Revision: %s(0x%x) \n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_5ghz),
			phy_rev_5ghz);
	}
	if (phy_rev_2ghz) {
		printf("2Ghz PHY Revision: %s(0x%x) \n",
			ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_2ghz),
			phy_rev_2ghz);
	}

	printf(" -==EEPROM Information==-\n");

	printf("EEPROM Version:     %x.%x \n",
		(eeprom_version & 0xF000) >> 12, eeprom_version & 0xFFF);

	printf("EEPROM Size: ");

	if (eeprom_size == 0)
		printf("       4K\n");
	else if (eeprom_size == 1)
		printf("       8K\n");
	else if (eeprom_size == 2)
		printf("       16K\n");
	else
		printf("       ??\n");
	
	printf("Regulatory Domain:  0x%X \n", regdomain);

	printf(" -==== Capabilities ====-\n");

	printf("|  802.11a Support: ");
	if (has_a)
		printf("yes  |\n");
	else
		printf("no   |\n");

	printf("|  802.11b Support: ");
	if (has_b)
		printf("yes  |\n");
	else
		printf("no   |\n");

	printf("|  802.11g Support: ");
	if (has_g)
		printf("yes  |\n");
	else
		printf("no   |\n");

	printf("|  RFKill  Support: ");
	if (has_rfkill)
		printf("yes  |\n");
	else
		printf("no   |\n");

	if (has_crystal != 2) {
		printf("|  32KHz   Crystal: ");
		if (has_crystal)
			printf("yes  |\n");
		else
			printf("no   |\n");
	}
	printf(" ========================\n");
	return 0;
}

