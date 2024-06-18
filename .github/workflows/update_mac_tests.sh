#!/bin/bash

# Given the latest test results for Mac, as fetched (automatically) from the
# cloud as an artifact, update the reference results and push the updated tests
# back to the cloud.

# The tarball having the tests
f=StereoPipelineTest.tar.gz

# Check if it exists
if [ ! -f "$f" ]; then
  # Maybe it is in a subdir. That happens when fetching the artifact.
  ans=$(ls */$f | head -n 1)
  if [ "$ans" != "" ]; then
    cd $(dirname $ans)
  fi
fi

# Check again
if [ ! -f "$f" ]; then  
  echo "Error: File: $f does not exist"
  exit 1
fi

# Extract
echo "Extracting $f"
tar xzfv $f > /dev/null 2>&1 # this is verbose
if [ ! -d "StereoPipelineTest" ]; then
  echo "Error: Directory: StereoPipelineTest does not exist"
  exit 1
fi

# Update the failed tests (each 'gold' is overwritten with 'run').
echo "Updating the tests"
for f in StereoPipelineTest/ss*/run; do 
  g=${f/run/gold}
  /bin/rm -rfv $g
  /bin/mv -fv $f $g
done

# Must make all scripts in bin and individual tests executable
chmod a+x StereoPipelineTest/bin/* StereoPipelineTest/*/*sh

echo "Creating a new tarball"
binaries=StereoPipelineTest.tar
tar cfv $binaries StereoPipelineTest 

echo "Pushing the updated tarball to the cloud"
repo=git@github.com:NeoGeographyToolkit/StereoPipelineTest.git  
gh=/home/oalexan1/miniconda3/envs/gh/bin/gh
tag=0.0.1
$gh release -R $repo delete $tag # wipe old tarball
notes="Update test results"
$gh release -R $repo create $tag $binaries --title $tag --notes "$notes" # upload new

