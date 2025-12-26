#!/bin/bash

# Run this on a local machine to save the current dependencies as a release.
# This will wipe the old version with the same tag. See save_mac_deps.sh for
# more info.

# Check usage 
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 tag"
    exit 1
fi

tag=$1; shift

# Create the tarball of dependencies. This likely includes a pre-built ASP
# itself, but that is not a problem.
# Note: Must updated below the names of both envs if they change.
tarball=asp_deps.tar.gz
cd $HOME
/usr/bin/time tar cfz $tarball \
  miniconda3/envs/asp_deps     \
  miniconda3/envs/python_isis9

# Set up GitHub CLI
gh=$HOME/miniconda3/envs/gh/bin/gh
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git

# Run $gh auth to authenticate

# Wipe old version
$gh release -R $repo delete $tag 

# Save the tarball as a release
notes="Full tarball of latest ASP dev build dependencies"
/usr/bin/time $gh release -R $repo create $tag $tarball --title $tag --notes "$notes"
