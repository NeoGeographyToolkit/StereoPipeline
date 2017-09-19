#!/usr/bin/perl
######################################################################
# Reader for the DMS Applanix GPS data for Operation IceBridge.
# The file format is "SBET".
# Author:  Bruce Raup, braup@nsidc.org
######################################################################

use strict;
use warnings;

# get basename of program
my $progname = $0;
$progname =~ s{^.*/}{};

use Getopt::Std;

# Get Subversion revision number
my $version = '$Revision: 15550 $';
($version) = $version =~ /^\$Revision:\s*(\d+\.?\d*)/;

# Set defaults above definition of $usage so that defaults can be shown there.

my $usage = "$progname version $version
Usage:
  $progname  -h            (prints this help message and exits)
OR
  $progname  [-q] [-r] [-n N] infile
where
  -q    Quiet mode
  -n    Output only every nth point
  -r    Raw mode.  Don't convert lon/lat coordinates to degrees
  --    explicitly ends options.  Arguments starting with '-' (e.g. negative
        numbers) thereafter will not be interpreted as options.

This program reads a binary SBET file and dumps it as ASCII to standard output.

";

# Get command-line options.  Letters that require arguments should be followed
# by a ':' in the call to getopts.
my %opt = ();
if (! getopts('hqn:r', \%opt) ) {
  die "$usage\n";
}

die "$usage\n" if $opt{h};

# main program

if (! defined $ARGV[0]) {
  print STDERR "No input file specified on command line.\n\n";
  die "$usage\n";
}
if (! -f $ARGV[0]) {
  print STDERR "Couldn't find input file $ARGV[0]\n\n";
  die "$usage\n";
}
dump_sbet_file($ARGV[0]);

# End of main

##############################################################################
## Subroutines
##############################################################################

##############################################################################
## dump_sbet_file
##############################################################################
sub dump_sbet_file {
  my $infile = shift;
  my $rec_length = 136; # bytes. 17 doubles

  my $point_count = 0;

  my $n_points;
  if ($opt{n}) {
    $n_points = $opt{n};
    unless ($n_points > 0) {
      die "Bad value for -n argument.\n\n$usage\n";
    }
  }

  open my $F, "<", $infile or die "Couldn't open $infile for reading: $!";
  my $rec;
  while ($rec_length == read($F, $rec, $rec_length)) {
    if (!defined $n_points || $point_count % $n_points == 0) {
      print_sbet_rec_as_ascii($rec);
    }
    ++$point_count;
  }
}

##############################################################################
## print_sbet_rec_as_ascii
##############################################################################
sub print_sbet_rec_as_ascii {
  my $binrec = shift;
  my $rec_field_pattern = 'd17';
  my ($time, $lat, $lon, $alt, $x_vel, $y_vel, $z_vel,
      $roll, $pitch, $heading, $wander, $x_force, $y_force, $z_force,
      $x_ang_rate, $y_ang_rate, $z_ang_rate) = unpack('d17', $binrec);

  unless ($opt{r}) {
    my $r2d = 180 / 3.141592654;
    $lat *= $r2d;
    $lon *= $r2d;
  }
  print join('  ', $time, $lat, $lon, $alt, $x_vel, $y_vel, $z_vel,
      $roll, $pitch, $heading, $wander, $x_force, $y_force, $z_force,
      $x_ang_rate, $y_ang_rate, $z_ang_rate), "\n";
}

# Internal documentation below.  See it nicely formatted by typing
#   perldoc thisfile
# at the system command line.  For help on this format, see the man page
# perlpod(1) (that is, type 'man perlpod' at the command line.)

__END__

=head1 NAME

skeleton_program - skeleton program for perl

=head1 SYNOPSIS

Copy to new location and edit.

=head1 DESCRIPTION

This file can (should) be used as a starter template for new perl programs.

=head1 EXAMPLES

=head1 AUTHOR

Bruce H. Raup (braup@nsidc.org)
