#!/bin/bash

# To update the Mac dependencies or create new ones, in manual (interactive)
# mode, run the ssh.yml / ssh_mac_arm.yml action to get ssh access to a Mac
# cloud instance.

# Then, fetch/build/update the dependencies. See 
# https://stereopipeline.readthedocs.io/en/latest/building_asp.html
# for a high-level overview.

# Ideally all dependencies are built and then installed as conda packages.
# The script build_helper.sh has the commands for how build dependencies
# manually, if needed to understand failures when using conda.

# The updated dependencies should be installed in /Users/runner/miniconda3/envs. 

# When done, and before exiting, save the dependencies, such as:
#   mkdir -p ~/work/StereoPipeline/packages
#   cd $HOME
#   /usr/bin/time tar cfz ~/work/StereoPipeline/packages/asp_deps.tar.gz \
#     *conda3/envs

# After quitting the action (exiting the shell), the tarball will be saved as an
# artifact. Upload progress can be monitored in GitHub Actions. 

# Then, from a local machine, which need not be a Mac, run this script. 
# It will fetch the tarball from the cloud and then push it as a release
# to permanent location, with given tag.

# This tarball will be used to build VisionWorkplace and ASP. See the script
# build_test.sh.

# The tag set here must match the tag in build_test.sh and build_helper.sh. If
# changing here, must later change in the other places.

# This script will overwrite the dependencies. If in doubt, use it with a new
# tag, as the dependencies are very hard to recreate.

# How to run this script:

# For Mac x64:
# tag=asp_deps_mac_x64_v1
# workflow="ssh.yml" # manual workflow
# #workflow="build_isis.yml" # automatic workflow
# $HOME/projects/StereoPipeline/.github/workflows/save_mac_deps.sh $workflow $tag

# For Mac Arm64:
# tag=asp_deps_mac_arm64_v1
# workflow="ssh_mac_arm.yml" # manual workflow
# $HOME/projects/StereoPipeline/.github/workflows/save_mac_deps.sh $workflow $tag

# For linux, the dependencies from the local machine can be saved as follows.
# tag=asp_deps_linux_v1
# $HOME/projects/StereoPipeline/.github/workflows/save_linux_deps.sh $tag

# Check usage
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 workflow tag"
    exit 1
fi

# The workflow that saved the dependencies as artifact. Options:
# ssh.yml, ssh_mac_arm.yml, build_isis.yml
workflow=$1; shift

# The tag to use to save the dependencies. Must then use this tag
# to fetch the dependencies in build_test.sh and build_helper.sh.
tag=$1; shift 

# The GitHub CLI tool. Can be installed in a new conda environment
# named 'gh' as follows:
# conda create -n gh -c conda-forge gh

gh=$(ls -d $HOME/*conda3/envs/gh/bin/gh)
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
# If making a new one, must make sure to update the tag in build_test.sh and build_helper.sh
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git

# Wipe any old version
echo If present, deleting the old release for tag: $tag
$gh release -R $repo delete $tag -y 2>/dev/null # hide any error message for missing release

# Upload the new version
notes="Full tarball of ASP dependencies (tag: $tag)"
echo Uploading a new version for tag: $tag
/usr/bin/time $gh release -R $repo create $tag $binaries --title $tag --notes "$notes"
