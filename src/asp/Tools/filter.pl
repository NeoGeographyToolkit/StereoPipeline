#!/usr/bin/perl
use strict;        # insist that all variables be declared
use diagnostics;   # expand the cryptic warnings

# This script is a temporary aid for reconstruct.sh,
# and it will be removed soon.
MAIN:{

  # Get a list from standard input. Keep only the items
  # satisfying the filters coming from file.
  
  if (scalar(@ARGV) < 1){
    print "Usage: $0 filterFile.txt\n";
    exit(0);
  }

  my $skip = (scalar(@ARGV) > 1); # false by default
  
  my ($items, $tmp)  = &parse(<STDIN>);
  open(FILE, "<$ARGV[0]");
  my ($filters, $order) = &parse(<FILE>);
  close(FILE);

  my %filteredVals;
  foreach my $key ( sort {$a cmp $b} keys %$items ){
    if ($skip){
      # Skip the images satisfying the filter
      next if (exists $filters->{$key});
    }else{
      # Show the images satisfying the filter
      next unless (exists $filters->{$key});
    }
    
    $filteredVals{$key} = $items->{$key};
  }

  if ($skip){
    # Print the values not in the filter. We don't have any order, so just
    # use the lexicographic order.
    foreach my $key ( sort {$a cmp $b} keys %filteredVals ){
      print $filteredVals{$key} . "\n";
    }
  }else{
    # Print the values in the same order as in the filter
    foreach my $key ( sort { $order->{$a} <=> $order->{$b} } keys %filteredVals ){
      print $filteredVals{$key} . "\n";
    }
  }

}

sub parse{
  
  my (%hash, %order);
  my $count = 0;
  foreach my $line (@_){
    $line =~ s/\n//g;
    next unless ($line =~ /(AS1\d-M-\d+)/);
    $hash {$1} = $line;
    $order{$1} = $count;
    $count++;
    #print "--$1--$line--\n";
  }

  return (\%hash, \%order);
}
