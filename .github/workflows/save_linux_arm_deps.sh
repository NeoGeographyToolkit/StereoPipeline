#!/bin/bash

# Save the linux-aarch64 ASP dependencies as a BinaryBuilder release, for the
# build_test_linux_arm.yml cloud nightly. Run on a linux-aarch64 machine (or in
# the linux-arm Docker container) that has the asp_deps + python_isis10 conda
# envs built from the nasa-ames channel (our =asp* stack: isis asp_2, ale asp_6,
# usgscsm asp_5, VW asp_7, breakup asp_1, stereo-pipeline asp_7). Analogous to
# save_mac_deps.sh, but uses conda-pack (envs are NOT relocatable by plain tar).
#
# Usage: ./save_linux_arm_deps.sh [tag]   (default tag: asp_deps_linux_arm_v1)
#
# The deps tarball EXCLUDES VW + ASP (lib/libVw*, lib/libAsp*, include/vw,
# include/asp) - the nightly rebuilds those from source into a separate
# installDir. It KEEPS the inline deps (libnabo, libpointmatcher, fgr/FGR, geoid/
# libegm2008) + theia/voxblox/texrecon/cgal headers - the ASP source build needs
# them (build_test_linux_arm.sh only builds libelas inline). BLAS is openblas
# (aarch64 blis 2.0 aborts at runtime in a VM).

set -e
tag=${1:-asp_deps_linux_arm_v1}

# conda-pack must be installed (conda install -n base conda-pack)
echo "Packing asp_deps (excluding VW/ASP)..."
conda pack -n asp_deps -o asp_deps.tar.gz --n-threads 4 --force \
  --exclude "lib/libVw*" --exclude "lib/libAsp*" \
  --exclude "include/vw/*" --exclude "include/asp/*"

echo "Packing python_isis10..."
conda pack -n python_isis10 -o python_isis10.tar.gz --n-threads 4 --force

# GitHub release assets cap at 2 GB. Split asp_deps if needed into 2 GB parts
# named asp_deps_p1.tar.gz, asp_deps_p2.tar.gz (build_test_linux_arm.sh does
# `cat asp_deps_p*.tar.gz | tar xzf -`). If it fits, just rename to _p1.
sz=$(stat -c %s asp_deps.tar.gz 2>/dev/null || stat -f %z asp_deps.tar.gz)
if [ "$sz" -gt 1900000000 ]; then
    echo "asp_deps.tar.gz is $sz bytes (> ~1.9 GB) - splitting into 2 GB parts"
    split -b 1900M -d -a 1 asp_deps.tar.gz asp_deps_p
    mv asp_deps_p0 asp_deps_p1.tar.gz
    [ -f asp_deps_p1 ] && mv asp_deps_p1 asp_deps_p2.tar.gz
else
    mv -f asp_deps.tar.gz asp_deps_p1.tar.gz
fi

# Upload to a BinaryBuilder release (full clobber of the same tag).
gh=$(ls -d $HOME/*conda3/envs/gh/bin/gh 2>/dev/null | head -1)
repo=NeoGeographyToolkit/BinaryBuilder
$gh release delete "$tag" -R "$repo" --yes 2>/dev/null || true
$gh release create "$tag" -R "$repo" -t "$tag" \
  -n "linux-aarch64 ASP deps (conda-pack; openblas; VW/ASP excluded)" \
  asp_deps_p1.tar.gz python_isis10.tar.gz
[ -f asp_deps_p2.tar.gz ] && $gh release upload "$tag" -R "$repo" asp_deps_p2.tar.gz
echo "Uploaded $tag to $repo"
