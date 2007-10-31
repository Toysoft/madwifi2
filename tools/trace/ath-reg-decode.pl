#!/usr/bin/perl

use Data::Dumper;
use Getopt::Long;

my %regs;

# default options (see below)
my $CHIP = "5212";
my $REG_FILE_NAME = "/tmp/ath_registers.wiki.txt";
my $OUT_FORMAT = "txt";
my $LOOKUP = "";
my $DUMP = 0;
my $BATCH = 0;

# defines the order in which the registers are looked up, in case of
# different definitions for different chipsets
my %chip_lookup = (
	"5212" => "5212 5212+ 5211+ 5210+ 5112 5112+ 5111+ all",
	"5211" => "5211 5211+ 5210+ 5111+ all",
	"5210" => "5210 5110 all",
);


### "main" ###

GetOptions (
	'chip=s' => \$CHIP,
	# --chip -c specify chipset: 5212 (default), 5211, 5210

	'out=s' => \$OUT_FORMAT,
	# --out -o <format> output format
	# <format> is different for processing a trace or dumping:
	#
	# traces <default>:
	#	R: 0x9858 = 0x7ec00d2e - AR5K_PHY_SIG .111 111. 11.. .... .... 11.1 ..1. 111.
	#
	# traces: --out initval:
	#	{ AR5K_PHY_PCDAC_TXPOWER(2),          0x0fff0fff },
	#
	# --dump <default> is just a whitespace separated list:
	#	0x9c1c AR5K_PHY_CURRENT_RSSI        [5111+] PHY current
	#
	# --dump --out case
	#	case 0x9c10: return "AR5K_PHY_IQRES_CAL_PWR_I"; break;
	#
	# --dump --out define
	#	#define AR5K_PHY_IQRES_CAL_CORR         0x9c18  /* [5111+] PHY timing IQ */
	#
	# --dump --out wiki
	#
	#	|| 0x989c || AR5K_RF_BUFFER || all || RF Buffer register ||

	'regfile=s' => \$REG_FILE_NAME,
	# --regfile -r <filename>
	# default: /tmp/ath_registers.wiki.txt
	# file name with registers (will be downloaded from wiki if it does not exist)

	'lookup=s' => \$LOOKUP,
	# --lookup -l <register address>
	# just lookup one register address

	'dump' => \$DUMP,
	# --dump -d
	# dump all infos from file

	'batch' => \$BATCH,
	# --batch -b logs/*.log
	# batch process multiple files: filename -> filename.$OUT_FORMAT.txt
);

check_download_reg_file();
parse_reg_file();

if ($LOOKUP) {
	# just lookup one register
	$LOOKUP =~ s/^0x//;
	$t = lookup_name($LOOKUP);
	print "0x$LOOKUP: $t->{'name'}\t$t->{'desc'}\n"
}
elsif ($DUMP) {
	foreach_reg();
}
elsif ($BATCH) {
	foreach $f (@ARGV) {
		open IN, "<", $f;
		print STDERR "converting $f -> $f.$OUT_FORMAT\n";
		open(STDOUT, ">$f.$OUT_FORMAT") || die "Can't redirect stdout";
		while (<IN>) {
			match_decode($_);
		}
		close FH;
		close STDOUT;
	}
}
else {
	#convert file from stdin or last argument
	while (<>) {
		match_decode($_);
	}
}


### functions ###

sub match_decode($) {
	if (/^.*(.):0x0+(\w{4}) = 0x(\w{8}) - (.*)/) {
		decode($1,$2,$3,$4);
	}
	else {
		s/\0//g;
		print;
	};
}

sub check_download_reg_file() {
	return if -f $REG_FILE_NAME;
	print "trying to get register list from madwifi.org wiki...\n";
	`wget -O $REG_FILE_NAME 'http://madwifi.org/wiki/DevDocs/AtherosRegisters?format=txt'`;
}

sub parse_reg_file() {
	open(REGS, '<', $REG_FILE_NAME );
	while (<REGS>) {
		if  (/^\s*\|\|([^\|]*)\|\|\s*(\S+)\s*\|\|([^\|]*)\|\|\s*(.*)\s+\|\|/) {
			my ($reg, $name, $chips, $desc) = ($1, $2, $3, $4);
			my $t;
			$reg =~ s/ //g; $chips =~ s/ //g;
			#print "($reg) ($name) ($chips) ($desc)\n";

			if ($reg =~ m/-/) {
				# special handling of ranges
				$regs->{"ranges"}->{$reg} = {} unless defined $regs->{"ranges"}->{$reg};
				$t = $regs->{"ranges"}->{$reg};
			}
			else {
				$regs->{$reg} = {} unless defined $regs->{$reg};
				$t = $regs->{$reg};
			}

			if ($chips eq "") {
				# no chip is specified -> all
				$t->{"all"}->{"name"} = $name;
				$t->{"all"}->{"desc"} = $desc;
				$t->{"all"}->{"line"} = $_;
			} else {
				#eg: [5210,5211] -> two entries
				foreach $i (split(/[, ]/,$chips)) {
					$t->{$i}->{"name"} = $name;
					$t->{$i}->{"desc"} = $desc;
					$t->{$i}->{"line"} = $_;
				}
			}
		}
	}
	close REGS;
	#print Dumper($regs);
}

my $last_ee_addr, $last_ee_op, $last_ee_status;
sub print_eeprom_access($$$) {
	my($mode, $reg, $val) = @_;

	if ($reg == 0x6000) { $last_ee_addr = $val; return; }
	if ($reg == 0x6008) { $last_ee_op = $val; return; }
	if ($reg == 0x600c) { $last_ee_stat = $val; return; };
	if ($reg == 0x6004) {
		printf "%s EEPROM AT 0x%03x: 0x%04x",
			$last_ee_op == 1 ? "READ" : "WRITE",
			$last_ee_addr, $val;
		print " STATUS $last_ee_stat" if ($last_ee_stat != 2);
		print "\n";
	}
}

sub lookup_name($) {
	my($reg) = @_;
	foreach my $c (split(/[, ]/,$chip_lookup{$CHIP})) {
		if (defined($regs->{"0x$reg"}->{$c}->{"name"})) {
			return $regs->{"0x$reg"}->{$c};
		}
	}
	# not found? try in ranges...
	foreach my $r (keys(%{$regs->{"ranges"}})) {
		if ($r =~ /0x(\S*)\s*-\s*0x(\S*)/) {
			if (hex($reg) >= hex($1) && hex($reg) <= hex($2)) {
				foreach my $c (split(/[, ]/,$chip_lookup{$CHIP})) {
					if (defined($regs->{"ranges"}->{"$r"}->{$c}->{"name"})) {
						# append (INDEX) to name
						my $i = {};
						$i->{"desc"} = $regs->{"ranges"}->{"$r"}->{$c}->{"desc"};
						$i->{"name"} = $regs->{"ranges"}->{"$r"}->{$c}->{"name"};
						$i->{"name"} .= "(" . (hex($reg) - hex($1))/4 . ")";
						return $i;
					}
				}
			}
		}
	}
	return 0;
}

sub show_bits($) {
	my($val) = @_;
	my $i, $ret="";
	$val = hex($val);

	for ($i=31; $i>=0; $i--) {
		$ret .= (($val>>$i & 1) ? "1" : ".");
		$ret .= " " if (($i&3)==0);
	}

	return $ret;
}

sub decode($$$$) {
	my($mode, $reg, $val, $func) = @_;
	my $dec, $bits;

	if (hex($reg) >= 0x6000 && hex($reg) <= 0x6010) {
		print_eeprom_access($mode,hex($reg),hex($val));
	}
	else {
		$dec = lookup_name($reg);
		# different output formats
		if ($OUT_FORMAT eq "initval") {
			if ($mode eq "W") {
				$dec->{"name"} = "0x$reg" unless $dec;
				$space = 35 - length($dec->{"name"});
				printf "\t{ %s,%*s0x%s },\t/* %s */\n", $dec->{"name"}, $space, "", $val, $func;
			}
		}
		else {
			$dec->{'name'} = "unknown" unless $dec;
			$bits = show_bits($val);
			printf "%s: 0x%s = 0x%s - %-30s %s (%s)\n", $mode, $reg, $val, $dec->{'name'}, $bits, $func;
		}
	}
}

sub foreach_reg_do($$$$) {
	my($reg, $name, $chip, $desc) = @_;
	if ($OUT_FORMAT eq "define") {
		printf "#define %-30s\t$reg\t/* [$chip] $desc */\n", $name;
	}
	elsif ($OUT_FORMAT eq "case") {
		print "\tcase $reg: return \"$name\"; break;\n";
	}
	elsif ($OUT_FORMAT eq "wiki") {
		printf "|| %-13s || %-30s || %-6s || $desc ||\n", $reg, $name, $chip;
	}
	else {
		printf "%-15s\t%-30s\t[$chip]\t$desc\n", $reg, $name;
	}
}

sub foreach_reg() {
	foreach my $r (sort keys(%{$regs})) {
		if ($r =~ "ranges") {
			foreach my $ra (sort keys(%{$regs->{"ranges"}})) {
				foreach my $c (sort keys(%{$regs->{"ranges"}->{$ra}})) {
					foreach_reg_do($ra, $regs->{"ranges"}->{$ra}->{$c}->{"name"},
						$c, $regs->{"ranges"}->{$ra}->{$c}->{"desc"});
				}
			}
		}
		else {
			foreach my $c (sort keys(%{$regs->{$r}})) {
				foreach_reg_do($r, $regs->{$r}->{$c}->{"name"},
					$c, $regs->{$r}->{$c}->{"desc"});
			}
		}
	}
}
