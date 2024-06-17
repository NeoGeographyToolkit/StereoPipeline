#!/bin/bash

# Run this on a local machine to save the current dependencies as a release.

# Check usage 
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 tag"
    exit 1
fi

tag=$1; shift

# Create the tarball of dependencies
tarball=asp_deps_linux.tar.gz
/usr/bin/time tar cfz $tarball \
  /home/oalexan1/miniconda3/envs/asp_deps \
  /home/oalexan1/miniconda3/envs/python_isis8

gh=/home/oalexan1/miniconda3/envs/gh/bin/gh
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git

# Wipe old version
$gh release -R $repo delete $tag 
notes="$tag"

# Save the tarball as a release
/usr/bin/time $gh release -R $repo create $tag $tarball --title $tag --notes "$notes"
