#!/bin/bash

# Given the latest test results for Mac, as fetched (automatically) from the
# cloud as an artifact, update the reference results and push the updated tests
# back to the cloud.

# The tarball having the tests
data=StereoPipelineTest.tar

# Check if it exists
if [ ! -f "$data" ]; then
  # Maybe it is in a subdir. That happens when fetching the artifact.
  ans=$(ls */$data | head -n 1)
  if [ "$ans" != "" ]; then
    cd $(dirname $ans)
  fi
fi

# Check again
if [ ! -f "$data" ]; then  
  echo "Error: File: $data does not exist"
  exit 1
fi

# Extract
echo "Extracting $data"
tar xfv $data > /dev/null 2>&1 # this is verbose
if [ ! -d "StereoPipelineTest" ]; then
  echo "Error: Directory: StereoPipelineTest does not exist"
  exit 1
fi

# Here may need to do some manual inspections

# Update the failed tests (each 'gold' is overwritten with 'run').
# This assumes that the "run" directories are trusted to be correct.
echo "Updating the tests"
for f in StereoPipelineTest/ss*/run; do 
  g=${f/run/gold}
  /bin/rm -rfv $g
  /bin/mv -fv $f $g
done

# If the scripts need to be updated, do it here, manually

# Must make all scripts in bin and individual tests executable
chmod a+x StereoPipelineTest/bin/* StereoPipelineTest/*/*sh

echo "Creating a new tarball"
tar cfv $data StereoPipelineTest 

# Make sure the gh tool is executable
gh=$(ls -d $HOME/*conda3/envs/gh/bin/gh)
if [ ! -x "$gh" ]; then
  echo "Error: Cannot find gh"
  exit 1
fi

echo "Pushing the updated tarball to the cloud"
repo=git@github.com:NeoGeographyToolkit/StereoPipelineTest.git  
tag=0.0.1
echo Wipe the old tests and upload the new ones
$gh release -R $repo delete $tag -y # wipe old tarball
notes="Update test results"
$gh release -R $repo create $tag $data --title $tag --notes "$notes" # upload new

