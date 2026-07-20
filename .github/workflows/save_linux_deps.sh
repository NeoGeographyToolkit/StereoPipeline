#!/bin/bash

# Save the linux-x86_64 ASP dependencies as a BinaryBuilder release, for the
# Linux Intel cloud nightly. Run on a linux-x86_64 machine (e.g. lunokhod1) that
# has the asp_deps + python_isis10 conda envs. Analogous to save_linux_arm_deps.sh
# (aarch64) and save_mac_deps.sh, but for x86_64. Uses conda-pack, since a conda
# env is NOT relocatable by a plain tar (conda-pack rewrites the baked prefix; the
# consumer runs the bundled conda-unpack).
#
# Usage: ./save_linux_deps.sh [tag]   (default tag: asp_deps_linux_v2)
#
# The deps tarball EXCLUDES VW + ASP (lib/libVw*, lib/libAsp*, include/vw,
# include/asp) - the nightly rebuilds those from source. It KEEPS the inline deps
# (libnabo, libpointmatcher, fgr/FGR, geoid/libegm2008) and the stereo correlation
# plugins in plugins/stereo (mgm, mgm_multi, msmw, msmw2, elas).
#
# asp_deps must NOT contain any editable (pip install -e) package. Editable dev
# installs of ale/usgscsm/etc. leave the env borrowing the package from a source
# tree (a dangling .pth) and make conda-pack fail. If you develop such a package,
# install it honestly and non-editable instead:
#   pip uninstall -y <pkg>; pip install --no-deps --no-build-isolation ~/projects/<pkg>
# Do NOT paper over an editable install with --ignore-editable-packages.

set -e
tag=${1:-asp_deps_linux_v2}

# conda-pack must be installed (conda install -n base conda-pack, or pip).
echo "Packing asp_deps (excluding VW/ASP)..."
conda pack -n asp_deps -o asp_deps.tar.gz --n-threads -1 --force \
  --ignore-missing-files \
  --exclude "lib/libVw*" --exclude "lib/libAsp*" \
  --exclude "include/vw/*" --exclude "include/asp/*"

echo "Packing python_isis10..."
conda pack -n python_isis10 -o python_isis10.tar.gz --n-threads -1 --force \
  --ignore-missing-files

# GitHub release assets cap at 2 GB. Split asp_deps if needed into ~2 GB parts
# named asp_deps_p1.tar.gz, asp_deps_p2.tar.gz (build_test.sh does
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
  -n "linux-x86_64 ASP deps (conda-pack; VW/ASP excluded)" \
  asp_deps_p1.tar.gz python_isis10.tar.gz
[ -f asp_deps_p2.tar.gz ] && $gh release upload "$tag" -R "$repo" asp_deps_p2.tar.gz
echo "Uploaded $tag to $repo"
