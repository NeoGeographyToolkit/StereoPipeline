#!/bin/bash

# Run this on a local machine to save the current dependencies as a release.

# Check usage 
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 tag"
    exit 1
fi

tag=$1; shift

# Create the tarball of dependencies. This likely includes
# a pre-built ASP itself, but that is not a problem.
tarball=asp_deps_linux.tar.gz
cd $HOME
/usr/bin/time tar cfz $tarball \
  miniconda3/envs/asp_deps     \
  miniconda3/envs/python_isis8

# Set up GitHub CLI
gh=$HOME/miniconda3/envs/gh/bin/gh
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git

# Run $gh auth to authenticate

# Wipe old version
$gh release -R $repo delete $tag 

# Save the tarball as a release
notes="Full tarball of ASP dependencies (tag: $tag)"
/usr/bin/time $gh release -R $repo create $tag $tarball --title $tag --notes "$notes"
