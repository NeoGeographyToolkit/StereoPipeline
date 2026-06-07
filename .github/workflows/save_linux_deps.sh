#!/bin/bash

# SUPERSEDED by conda-pack. Use the commands below instead.
# The old approach (plain tar of the conda env) did not handle prefix
# relocation. conda-pack rewrites hardcoded paths so the tarball is
# portable to any machine.

# First, wipe VW and ASP libs, headers, and tools from asp_deps so only
# the build dependencies remain (VW/ASP are rebuilt from source by the
# nightly build). Then conda-pack both envs and upload.

# ---- wipe VW/ASP from asp_deps ----
# conda activate asp_deps
# rm -f $CONDA_PREFIX/lib/libVw*.so $CONDA_PREFIX/lib/libAsp*.so
# rm -rf $CONDA_PREFIX/include/vw $CONDA_PREFIX/include/asp
# # Wipe ASP tools (list from StereoPipeline/install/bin)
# for t in $(ls ~/projects/StereoPipeline/install/bin/); do
#   rm -f "$CONDA_PREFIX/bin/$t"
# done
# # Wipe VW tools
# for t in colormap convert_pinhole_model correlate hillshade \
#          ipfind ipmatch print_exif undistort_image image2qtree; do
#   rm -f "$CONDA_PREFIX/bin/$t"
# done

# ---- conda-pack ----
# conda-pack -n asp_deps -o asp_deps_p1.tar.gz --ignore-missing-files
# conda-pack -n python_isis10 -o python_isis10.tar.gz --ignore-missing-files
# # Split if over 2 GB (GitHub release asset limit):
# # split -b 1900m asp_deps_p1.tar.gz asp_deps_p
# # mv asp_deps_paa asp_deps_p1.tar.gz; mv asp_deps_pab asp_deps_p2.tar.gz

# ---- upload ----
# gh=$(ls -d $HOME/*conda3/envs/gh/bin/gh)
# repo=NeoGeographyToolkit/BinaryBuilder
# tag=asp_deps_linux_v2
# $gh release -R $repo delete $tag --yes
# $gh api -X DELETE repos/$repo/git/refs/tags/$tag
# $gh release -R $repo create $tag --title $tag --notes "..."
# $gh release -R $repo upload $tag asp_deps_p1.tar.gz asp_deps_p2.tar.gz \
#   python_isis10.tar.gz
