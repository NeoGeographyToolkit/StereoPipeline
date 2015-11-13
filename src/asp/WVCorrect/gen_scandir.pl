#!/usr/bin/perl
use strict;        # insist that all variables be declared
use diagnostics;   # expand the cryptic warnings
use File::Basename;
use Cwd;
use File::Spec;

MAIN:{

  # For each of the run directories starting with given prefix, find
  # if the left and right images are forward or reverse scan.

  if (scalar(@ARGV) < 1){
    print "Usage: $0 prefix\n";
    exit(0);
  }

  my $currDir = cwd;
  my $execDir = File::Spec->rel2abs(dirname(__FILE__));

  my $prefix = $ARGV[0];
  print "Prefix is $prefix\n";
  my @lines;
  foreach my $dir (<$prefix*>){

    chdir $currDir;

    next unless ($dir =~ /^$prefix/);
    next unless (-d $dir);

    chdir $dir;
    print "Now in $dir\n";
    my $ans = qx($execDir/print_files.pl);
    print "answer is $ans\n";
    $ans =~ s/\s*$//g;
    my @files = split(/\s+/, $ans);
    my $line = $dir;

    foreach my $file (@files){
      print "---line is $file\n";

      $file = $file . ".xml";
      next unless (-f $file);

      open(NFILE, "<$file");
      my $text = join("", <NFILE>);
      close(NFILE);
      my $scandir = 1;
      if ($text =~ /\<SCANDIRECTION\>Reverse\<\/SCANDIRECTION\>/is){
        $scandir = 0;
      }

      my $pitch = 0.0;
      if ($text =~ /\<DETPITCH\>(.*?)\<\/DETPITCH\>/is){
        $pitch = $1;
      }

      $line = "$line $scandir $pitch";
    }
    $line =~ s/^.*?_//g;# matlab does not like letters
    push(@lines, $line);
    print "$line\n";
  }

  chdir $currDir;
  my $file = "scandir_" . $prefix . ".txt";
  print "Writing: $file\n";
  open(FILE, ">$file");
  foreach my $line (@lines){
    print FILE "$line\n";
  }
  close(FILE);

}
