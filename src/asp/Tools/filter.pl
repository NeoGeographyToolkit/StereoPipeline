#!/usr/bin/perl
# __BEGIN_LICENSE__
#  Copyright (c) 2009-2012, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__


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
