#!/bin/bash

# To update the Mac dependencies, or create new ones, run the ssh.yml action
# (for interactive use) or build_isis.yml (batch). Some dependencies can be
# fetched with conda, and others need to be built. They should be installed in 
# /Users/runner/miniconda3/envs. 
# See https://stereopipeline.readthedocs.io/en/latest/building_asp.html
# for more details.

# In manual mode, when done, and before exiting, save the dependencies as follows:
#   mkdir -p ~/work/StereoPipeline/packages
#   /usr/bin/time tar cfz ~/work/StereoPipeline/packages/asp_deps.tar.gz \
#     /Users/runner/miniconda3/envs

# Then, from a local machine, which need not be a mac, run this script. 

# The tag set here must match the tag in build_test.sh and build_isis.sh. If
# changing here, must later change in the other places.

# tag=mac_conda_env8 
# workflow="ssh.yml" # manual workflow
# #workflow="build_isis.yml" # automatic workflow
# $HOME/projects/StereoPipeline/.github/workflows/save_mac_deps.sh $workflow $tag

# This script will overwrite the dependencies. If in doubt, use it with a new
# tag, as the dependencies are very hard to recreate.

# For linux, the dependencies from the local machine can be saved as follows.
# tag=linux_conda_env7
# $HOME/projects/StereoPipeline/.github/workflows/save_linux_deps.sh $tag

# Check usage
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 workflow tag"
    exit 1
fi

# The workflow that saved the dependencies as artifact (ssh.yml or build_isis.yml)
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
echo Stage: $completed
echo Status: $success
echo Id: $id
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
# Can use a new tag here, or overwrite the existing tarball
# If making a new one, must make sure to update the tag in build_test.sh and build_isis.sh
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git

# Wipe any old version
echo If present, deleting the old release for tag: $tag
$gh release -R $repo delete $tag 2>/dev/null # hide any error message for missing release

# Upload the new version
notes="$tag"
echo Uploading a new version for tag: $tag
/usr/bin/time $gh release -R $repo create $tag $binaries --title $tag --notes "$notes"
