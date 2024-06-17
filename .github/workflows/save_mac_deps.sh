#!/bin/bash

# Fetch the latest dependencies as saved in an artifact in the clound.
# Save them to a permanent location as a release.

# Check usage
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 workflow tag"
    exit 1
fi

# The workflow that saved the dependencies as artifact
workflow=$1; shift

# The tag to use to save the dependencies. Must then use this tag
# to fetch the dependencies in build_test.sh and build_isis.sh.
tag=$1; shift 

gh=/home/oalexan1/miniconda3/envs/gh/bin/gh
repo=git@github.com:NeoGeographyToolkit/StereoPipeline.git

# Query the ${workflow}. Must check that that the top-most run is successful
$gh run list -R $repo --workflow=${workflow}

# Find the latest id, then fetch the artifacts for it
ans=$($gh run list -R $repo --workflow=${workflow} | grep -v STATUS | head -n 1)
completed=$(echo $ans | awk '{print $1}')
success=$(echo $ans | awk '{print $2}')
id=$(echo $ans | awk '{print $7}')
echo Completed is $completed
echo Success is $success
echo Id is $id
if [ "$success" != "success" ]; then
  echo "Error: The ${workflow} workflow did not succeed"
  exit 1
fi 

echo Fetching the build with id $id from the cloud 
echo $gh run download -R $repo $id
/bin/rm -rf ASP-dependencies-macOS # Must wipe this first, or else the download can fail
$gh run download -R $repo $id

# Must be careful with the line below. This is set in the ${workflow} file.
binaries=ASP-dependencies-macOS/asp_deps.tar.gz
if [ ! -f "$binaries" ]; then
  echo "Error: File: $binaries does not exist"
  exit 1
fi 

# Add the tarball of dependencies as a release
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git
# Wipe old version
$gh release -R $repo delete $tag 
notes="$tag"
/usr/bin/time $gh release -R $repo create $tag $binaries --title $tag --notes "$notes"
