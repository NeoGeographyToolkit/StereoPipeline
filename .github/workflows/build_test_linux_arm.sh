#!/bin/bash

# Run by GitHub Actions to build and test the Linux aarch64 (arm64) version of
# ASP, on a runs-on: ubuntu-22.04-arm runner. This is a linux-arm-specific copy
# of build_test.sh (which is Mac-only). The Linux x86_64 nightly uses
# BinaryBuilder build.py/Packages.py instead; this cloud path mirrors the Mac
# cloud model. See env_update_06_2026_coordination.sh for the 4-bot file matrix.

build_failed=0

# Base of the StereoPipeline repo (where this runs).
aspRepoDir=$(pwd)
if [ "$(basename $aspRepoDir)" != "StereoPipeline" ]; then
    echo "Error: Directory: $aspRepoDir is not StereoPipeline"
    exit 1
fi
baseDir=$(dirname $aspRepoDir)
installDir=$baseDir/install
packageDir=$baseDir/packages
testDir=$baseDir/StereoPipelineTest

# Throw an error unless on Linux aarch64
isLinux=$(uname -s | grep Linux)
if [ "$isLinux" = "" ]; then
    echo "This script is for Linux only"
    exit 1
fi
# NOTE: on Linux uname -m is "aarch64" (NOT "arm64" as on Mac).
isArm64=$(uname -m | grep -E 'aarch64|arm64')
if [ "$isArm64" = "" ]; then
    echo "This script is for Linux aarch64 only (uname -m = $(uname -m))"
    exit 1
fi
echo "Platform: Linux aarch64"
tag=asp_deps_linux_arm_v1
envName=asp_deps

# Fetch and unpack the ASP dependencies (conda-pack tarballs; relocatable - the
# bundled conda-unpack rewrites the baked prefix on this runner). asp_deps may be
# split into parts (GitHub 2 GB asset cap); the cat-glob handles 1+ parts. The
# python env is a separate conda-pack tarball. NO codesign needed on Linux.
bbUrl=https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}
envParent="$HOME/miniconda3/envs"
mkdir -p "$envParent/asp_deps" "$envParent/python_isis10"
wget -nv --tries=5 --timeout=30 ${bbUrl}/asp_deps_p1.tar.gz
wget -nv --tries=2 --timeout=30 ${bbUrl}/asp_deps_p2.tar.gz 2>/dev/null   # may not exist (single part) - ok
wget -nv --tries=5 --timeout=30 ${bbUrl}/python_isis10.tar.gz
cat asp_deps_p*.tar.gz | tar xzf - -C "$envParent/asp_deps"
"$envParent/asp_deps/bin/conda-unpack"
tar xzf python_isis10.tar.gz -C "$envParent/python_isis10"
"$envParent/python_isis10/bin/conda-unpack"
# Free the downloaded dependency tarballs now that they are unpacked, to save
# runner disk (the arm runner is disk-constrained).
rm -f asp_deps_p*.tar.gz python_isis10.tar.gz
df -h

# Locate the env
envPath=$(ls -d $HOME/*conda3/envs/${envName})
if [ ! -d "$envPath" ]; then
    echo "Error: Directory: $envPath does not exist"
    exit 1
fi
export PATH=$envPath/bin:$PATH

# Activate the env (sets CONDA_PREFIX + compiler env; the deps env is conda-pack
# relocated, so use the env's own activate script).
source "$envPath/bin/activate"

# Linker: conda ISIS leaves some symbols ASP references (e.g.
# Isis::FileName::expanded()) undefined in libisis (resolved at runtime). On
# Linux .so undefined symbols are allowed by default, but pass the GNU flag
# explicitly (belt-and-suspenders) plus -lfmt -lm -L for the ASP cmake ABI probe.
# No forced linker (the Mac Intel strictness issue does not apply on Linux).
LF="-Wl,--allow-shlib-undefined -lfmt -lm -L$envPath/lib"
# Use a bash array so the multi-word $LF survives as ONE value per -D flag.
# (A plain string expanded unquoted splits on spaces and cmake rejects the
# stray "-L.../lib" as an unknown argument.)
cmake_opts=(-DCMAKE_SHARED_LINKER_FLAGS="$LF" -DCMAKE_EXE_LINKER_FLAGS="$LF")

# Compiler (conda aarch64 gcc)
cc_comp=aarch64-conda-linux-gnu-gcc
cxx_comp=aarch64-conda-linux-gnu-g++
echo cc_comp=$cc_comp
echo cxx_comp=$cxx_comp

# Build libelas from source (not shipped in the deps tarball; ported to ARM via
# sse2neon).
mkdir -p $baseDir
cd $baseDir
git clone https://github.com/NeoGeographyToolkit/libelas.git
cd libelas
mkdir -p build
cd build
$envPath/bin/cmake ..                                      \
  -DTIFF_LIBRARY_RELEASE=$envPath/lib/libtiff.so           \
  -DTIFF_INCLUDE_DIR=$envPath/include                      \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp               \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp            \
  -DCMAKE_CXX_FLAGS=-I$envPath/include
make -j$(nproc)
ans=$?
if [ "$ans" -ne 0 ]; then
    echo "Error: libelas build failed"
    build_failed=1
fi
mkdir -p $envPath/plugins/stereo/elas/bin
/bin/cp -fv elas $envPath/plugins/stereo/elas/bin/elas

# Build VisionWorkbench (master has the aarch64 SSE fix: VW_ENABLE_SSE=0)
mkdir -p $baseDir
cd $baseDir
git clone https://github.com/visionworkbench/visionworkbench.git
cd visionworkbench
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp   \
  "${cmake_opts[@]}"
echo Building VisionWorkbench
make -j$(nproc) install > /dev/null 2>&1
out_build_vw=$(pwd)/output_build_vw.txt
make install > $out_build_vw 2>&1
tail -n 500 $out_build_vw
echo Log of VW build will be saved with the artifacts in $(basename $out_build_vw)

# Build StereoPipeline (master has the aarch64 SSE + OpenMP fixes)
cd $aspRepoDir
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DVISIONWORKBENCH_INSTALL_DIR=$installDir       \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp   \
   "${cmake_opts[@]}"
echo Building StereoPipeline
make -j$(nproc) install > /dev/null 2>&1
ans=$?
if [ "$ans" -ne 0 ]; then
    echo "Error: StereoPipeline build failed"
fi
out_build_asp=$(pwd)/output_build_asp.txt
make install > $out_build_asp 2>&1
tail -n 500 $out_build_asp
echo Log of ASP build will be saved with the artifacts in $(basename $out_build_asp)

# Package with BinaryBuilder.
echo Packaging the build
cd $baseDir
git clone https://github.com/NeoGeographyToolkit/BinaryBuilder
cd BinaryBuilder
num=$(ls -d $HOME/*conda3/envs/python* | wc -l)
if [ "$num" -ne 1 ]; then
    echo "Error: Expected exactly one python env, found $num"
    exit 1
fi
export ISISROOT=$envPath
export LD_LIBRARY_PATH=$installDir/lib:$envPath/lib:$LD_LIBRARY_PATH
./make-dist.py $installDir \
  --asp-deps-dir $envPath  \
  --python-env $(ls -d $HOME/*conda3/envs/python*)
if [ $? -ne 0 ]; then
    echo "Error: make-dist.py failed"
    build_failed=1
fi
mkdir -p $packageDir
mv -fv Stereo* $packageDir

# Extract the tarball to test it
cd $packageDir
tarBall=$(ls StereoPipeline-*.tar.bz2 | head -n 1)
if [ "$tarBall" == "" ]; then
  echo Cannot find the packaged ASP tarball
  build_failed=1
fi
/usr/bin/time tar xjf $tarBall > /dev/null 2>&1

binDir=$packageDir/$tarBall
binDir=${binDir/.tar.bz2/}
binDir=$binDir/bin
export PATH=$binDir:$PATH
echo "Binaries are in $binDir"
if [ ! -d "$binDir" ]; then
    echo "Error: Directory: $binDir does not exist. Build failed."
    build_failed=1
fi

# Fetch the shared test data + reference results
cd $baseDir
echo Testing the build.
# Use the system CA bundle explicitly. By this point the conda env is active
# (source activate above), and its wget/openssl cannot verify GitHub's cert
# ("Unable to locally verify the issuer's authority"), whereas the deps wgets
# above (run before activation) use the system certs and succeed.
wget -nv --tries=5 --timeout=30 --ca-certificate=/etc/ssl/certs/ca-certificates.crt https://github.com/NeoGeographyToolkit/StereoPipelineTest/releases/download/0.0.1/StereoPipelineTest.tar
if [ ! -f "StereoPipelineTest.tar" ]; then
    echo "Error: File: StereoPipelineTest.tar does not exist. Test failed."
    build_failed=1
fi
tar xf StereoPipelineTest.tar > /dev/null 2>&1
# Free the tarball after extraction, to save runner disk.
rm -f StereoPipelineTest.tar

if [ ! -d "$testDir" ]; then
    echo "Error: Directory: $testDir does not exist"
    build_failed=1
fi
cd $testDir

reportFile=$(pwd)/output_test.txt
rm -f $reportFile
ans=0
for d in ss*; do
    if [ ! -d "$d" ]; then continue; fi
    cd $d
    echo Running test in $(pwd)
    ./run.sh > output.txt 2>&1
    ./validate.sh >> output.txt 2>&1
    ans0=$?
    tail -n 20 output.txt
    echo "Test $d returned $ans0"
    echo "Test $d returned $ans0" >> $reportFile
    if [ "$ans0" -ne 0 ]; then ans=1; fi
    cd ..
done
echo ans is $ans
echo test_status $ans >> $reportFile

if [ "$ans" -eq 0 ]; then
    echo "All tests passed"
else
    echo "Some tests failed"
fi

mkdir -p $packageDir
echo Copying the build
(cd $testDir/..; tar cf $packageDir/$(basename $testDir).tar $(basename $testDir))
echo Copying the logs
cp -rfv $out_build_vw $out_build_asp $reportFile $packageDir
rm -rfv $(dirname $binDir) > /dev/null 2>&1

if [ "$build_failed" -ne 0 ]; then
    echo "Build or packaging failed (see errors above)"
    exit 1
fi
exit $ans
