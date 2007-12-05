#!/usr/bin/perl

use Data::Dumper;
use Getopt::Long;

my %regs;
my %cross;

# default options (see below)
my $CHIP = "5212";
my $REG_FILE_NAME = "/tmp/ath_registers.wiki.txt";
my $OUT_FORMAT = "txt";
my $LOOKUP = "";
my $DUMP = 0;
my $BATCH = 0;
my $CROSS = 0;
my $INIT = 0;

# defines the order in which the registers are looked up, in case of
# different definitions for different chipsets
my %chip_lookup = (
	"5212" => "5212 5212+ 5211+ 5210+ 5112 5112+ 5111+ 5111 all",
	"5211" => "5211 5211+ 5210+ 5111+ 5111 all",
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

	'cross' => \$CROSS,
	# cross reference only

	'init=s' => \$INIT,
	# --init <rf | mode>
	# create inittables from 6 files as next args: regs a b g ta tg
	
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
elsif ($INIT) {
	make_init_from_files(@ARGV[0], @ARGV[1], @ARGV[2], @ARGV[3], @ARGV[4], @ARGV[5]);
}
elsif ($BATCH) {
	my $oldout;
	foreach $f (@ARGV) {
		open IN, "<", $f;
		if (!$CROSS) {
			print STDERR "converting $f -> $f.$OUT_FORMAT\n";
			open $oldout, ">&STDOUT" or die "Can't dup STDOUT: $!";
			open STDOUT, ">$f.$OUT_FORMAT" or die "Can't redirect stdout";
		}
		my $line = 0;
		while (<IN>) {
			match_decode($f, $line++, $_);
		}
		if (!$CROSS) {
			close FH;
			close STDOUT;
			open STDOUT, ">&", $oldout or die "Can't dup \$oldout: $!";
		}
	}
}
else {  #convert file from stdin or last argument
	my $line = 0;
	while (<>) {
		match_decode("stdin", $line++, $_);
	}
}

if ($CROSS) {
	dump_cross();
}

### functions ###

sub match_decode($$$) {
	my($file, $lineno, $line) = @_;
	if ( $line =~ /^.*(.): ?0x0?(\w{4}) = 0x(\w{8}) - (.*)/) {
		my $mode = $1;
		my $reg = $2;
		my $val = $3;
		my $func = $4;
		# allow us to re-convert already converted dumps
		if ($func =~ /.*\((.*)\)$/) {
			$func = $1;
		}
		if ($CROSS) {
			crossref($file, $lineno, $mode, $reg, $val, $func);
		}
		else {
			# this prints the line as well
			decode($mode, $reg, $val, $func);
		}
	}
	else {
		s/\0//g;
		print;
	}
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
	my $return = {};
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
						$return->{"desc"} = $regs->{"ranges"}->{"$r"}->{$c}->{"desc"};
						$return->{"name"} = $regs->{"ranges"}->{"$r"}->{$c}->{"name"};
						$return->{"name"} .= "(" . (hex($reg) - hex($1))/4 . ")";
						return $return;
					}
				}
			}
		}
	}
	# still no name -> convert PHY regs to AR5K_PHY(x)
	if (hex($reg) >= 0x9800 && hex($reg) <= 0xa20c) {
		$return->{"desc"} = $regs->{"0x9800"}->{"all"}->{"desc"};
		$return->{"name"} = $regs->{"0x9800"}->{"all"}->{"name"};
		$return->{"name"} .= "(" . (hex($reg) - 0x9800)/4 . ")";
		return $return;
	}
	
	$return->{"desc"} = "unknown";
	$return->{"name"} = "unknown";
	return $return;
}

sub show_bits($) {
	my($val) = @_;
	my $i, $ret="";
	$val = hex($val);

	for ($i=31; $i>=0; $i--) {
		$ret .= (($val>>$i & 1) ? "1" : ".");
		$ret .= " " if (($i&3)==0 && i!=0);
	}

	return $ret;
}

sub noise_to_dbm($) {
	my ($val) = @_;
	my $noi = hex($val);
	
	$noi = (($noi >> 19) & 0x1ff);
	if ($noi & 0x100) {
		$noi = - (($noi ^ 0x1ff) + 1);
		return "$noi dBm"
	} else {
		return "";
	}
}

sub decode($$$$) {
	my($mode, $reg, $val, $func) = @_;
	my $dec, $bits;
	my $nam, $extra = 0;

	if (hex($reg) >= 0x6000 && hex($reg) <= 0x6010) {
		print_eeprom_access($mode,hex($reg),hex($val));
	}
	else {
		$dec = lookup_name($reg);
		# different output formats
		if ($OUT_FORMAT eq "initval") {
			if ($mode eq "W") {
				$dec->{"name"} = "0x$reg" if ($dec->{"name"} =~ "unknown");
				$space = 35 - length($dec->{"name"});
				printf "\t{ %s,%*s0x%s },\t/* %s */\n", $dec->{"name"}, $space, "", $val, $func;
			}
		}
		else {
			$bits = show_bits($val);

			if (hex($reg) == 0x9864 && $mode eq "R") {
				$extra = noise_to_dbm($val);
			}

			$nam = $dec->{'name'};
			$nam .= " (" . $extra . ")" if ($extra);

			printf "%s: 0x%s = 0x%s - %-30s %s (%s)\n", $mode, $reg, $val, $nam, $bits, $func;
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

sub crossref($$$$) {
	my($file, $line, $mode, $reg, $val, $func) = @_;
	$cross->{$reg}->{$val}->{$mode}->{$func}->{$file} .= "$line, ";
}

sub dump_cross() {
	foreach my $r (sort keys(%{$cross})) {
		my $dec = lookup_name("$r");
		print "0x$r $dec->{'name'} ($dec->{'desc'})\n";

		foreach my $v (sort keys(%{$cross->{$r}})) {
			printf "\t0x$v [%s]\n", show_bits($v);

			foreach my $m (sort keys(%{$cross->{$r}->{$v}})) {
				foreach my $func (sort keys(%{$cross->{$r}->{$v}->{$m}})) {
					printf "\t\t$m $func\n";

					foreach my $file (sort keys(%{$cross->{$r}->{$v}->{$m}->{$func}})) {
						print "\t\t\t$file (lines ";
						print $cross->{$r}->{$v}->{$m}->{$func}->{$file};
						print ")\n";
					}
				}
			}
		}
	}
}

sub make_init_from_files($$$$$$) {
	my($regsf, $af, $bf, $gf, $taf, $tgf) = @_;
	
	open REGS, "<", $regsf or die "can't open file '$regsf'";
	open A, "<", $af or die "can't open file '$af'";
	open B, "<", $bf or die "can't open file '$bf'";
	open G, "<", $gf or die "can't open file '$gf'";
	open TA, "<", $taf or die "can't open file '$taf'";
	open TG, "<", $tgf or die "can't open file '$tgf'";
	while (<REGS>) {
		$r = $_; $r =~ s/\n//g;
		$dec = lookup_name($r);
		$r = $dec->{"name"} unless $dec->{"name"} =~ "unknown" ;
		$a = <A>; $a =~ s/\n//g;
		$b = <B>; $b =~ s/\n//g;
		$g = <G>; $g =~ s/\n//g;
		$ta = <TA>; $ta =~ s/\n//g;
		$tg = <TG>; $tg =~ s/\n//g;
		if ($INIT eq "mode") {
			print "\t{ $r,\n";
			print "\t\t{ $a, $ta, $b, $g, $tg } },\n";
		} elsif ($INIT == "rf") {
			print "\t{ 0, $r,\n";
			print "\t\t{ $a, $ta, $b, $g, $tg } },\n";
		}
	}
	close REGS;
	close A;
	close B;
	close G;
	close TA;
	close TG;
}
