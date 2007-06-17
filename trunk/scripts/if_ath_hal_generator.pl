#!/usr/bin/perl
strict;
use warnings;
require 'dumpvar.pl';

# Workaround for perl's warning mechanism, avoids 'possible typo' for every setting used only one time.
$header_for_c = undef;
$footer_for_c = undef;
$header_for_h = undef; 
$footer_for_h = undef;
$if_ath_hal_c = undef;
$if_ath_hal_h = undef;
$path_to_hal = undef;
$hal_functions_not_to_wrap = undef;
$hal_h = undef;

# Include settings, calculate a few new ones
require "scripts/if_ath_hal_settings.pl";
$path_to_ah_h = "$path_to_hal/$hal_h";
$path_to_if_ath_hal_h = "$path_to_ath/$if_ath_hal_h";
$path_to_if_ath_hal_c = "$path_to_ath/$if_ath_hal_c";

# Parsed Function Data 

# list of declarations in document order
@hal_prototypes = ();
# hash of string->string (hal's function name to return type)
%hal_functionname_to_return_type = ();
# hash of string->list of strings (ordered list of parameter names)
%hal_functionname_to_parameter_name_array = ();
# hash of string->list of strings (ordered list of parameter types)
%hal_functionname_to_parameter_types_array = ();

# Open the files we need
if(!open AH_H, "<$path_to_ah_h") {
   die "Cannot open $path_to_ah_h: $!";
}
if(!open ATH_HAL_API_H, ">$path_to_if_ath_hal_h") {
   close AH_H;
   die "Cannot open $path_to_if_ath_hal_h: $!";
}
if(!open ATH_HAL_API_C, ">$path_to_if_ath_hal_c") {
   close AH_H;
   close ATH_HAL_API_H;
   die "Cannot open $path_to_if_ath_hal_c: $!";
}

# Parse and scrub the hal structure's member function declarations 
$line_continued = 0;
$line_buffer = "";
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
foreach $proto (@hal_prototypes) {
   $proto =~ /^((?:(?:const|struct)\s*)*[^\s]+(?:[\s]*\*)?)[\s]*__ahdecl\(\*([^\)]*)\)\((.*)\);/;
   my $return_type   = $1;
   my $member_name   = $2;
   my $parameterlist = $3;
   if(! grep{/$member_name/} @hal_functions_not_to_wrap ) {
      $hal_functionname_to_return_type{"$member_name"} = $return_type;
      @{$hal_functionname_to_parameter_name_array{"$member_name"}} = ();
      @{$hal_functionname_to_parameter_types_array{"$member_name"}} = ();
      my @parameters = split /,\s?/, $parameterlist;
      $argnum = 0;
      $first = 1;
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

for $member_name (keys %hal_functionname_to_return_type) {
   my $api_return_type   = $hal_functionname_to_return_type{$member_name};
   my $api_name      	 = $member_name;
   if(exists $hal_function_name_to_madwifi_name{$member_name}) {
      $api_name = $hal_function_name_to_madwifi_name{$member_name};
   }   
   print ATH_HAL_API_H "__hal_wrapper " . $api_return_type . " " . $api_name . "(";
   my @names = @{$hal_functionname_to_parameter_name_array{$member_name}};
   my @types = @{$hal_functionname_to_parameter_types_array{$member_name}};
   for $i (0..$#names) {
      if($i) {
	 print ATH_HAL_API_H ", ";
      }
      print ATH_HAL_API_H $types[$i] . " " . $names[$i];
   }
   print ATH_HAL_API_H ")\n\tIMPLEMENTATION({";
   if(! ($api_return_type =~ /void/ )) {
      print ATH_HAL_API_H "\n\t\t" . $api_return_type . " ret;";
   }
   print ATH_HAL_API_H "\n\t\tATH_HAL_LOCK_IRQ(GET_ATH_SOFTC(ah));";
   print ATH_HAL_API_H "\n\t\t";
   if(! ($api_return_type =~ /void/ )) {
      print ATH_HAL_API_H "ret = ";
   }

   print ATH_HAL_API_H "ah->$member_name(";
   for $j (0..$#names) {
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
   print ATH_HAL_API_H "\n\t})\n";
}
print ATH_HAL_API_H $footer_for_h;

#
# Generate the implementation file
# 
print ATH_HAL_API_C $header_for_c;
print ATH_HAL_API_C "/* Include header file for declarations */\n";
print ATH_HAL_API_C "#include \"$path_to_if_ath_hal_h\"\n";
print ATH_HAL_API_C "\n";
print ATH_HAL_API_C "/* Include header file for implementations (if necessary) */\n";
print ATH_HAL_API_C "#define TRACEABLE_IMPL\n";
print ATH_HAL_API_C "#include \"$path_to_if_ath_hal_h\"\n";
print ATH_HAL_API_C "#undef  TRACEABLE_IMPL\n";
print ATH_HAL_API_C "\n";
print ATH_HAL_API_C $footer_for_c;

# Close up the files
close AH_H;
close ATH_HAL_API_H;
close ATH_HAL_API_C;

