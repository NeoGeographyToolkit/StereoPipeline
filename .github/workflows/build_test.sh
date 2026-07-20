#!/bin/bash

# This is run by GitHub Actions to build and test the Mac version of ASP.
# See build_helper.sh for detailed build commands for ASP and its dependencies.

# Track infrastructure failures separately from test failures.
# Infrastructure failures (build, packaging) are fatal.
# Test validation failures are reported but not fatal.
build_failed=0

# Record the location where the script is running, which should
# be the base of the StereoPipeline repo. This must happen first.
aspRepoDir=$(pwd) # same as $HOME/work/StereoPipeline/StereoPipeline
if [ "$(basename $aspRepoDir)" != "StereoPipeline" ]; then
    # Check that base dir is StereoPipeline
    echo "Error: Directory: $aspRepoDir is not StereoPipeline"
    exit 1
fi
# Other variables
baseDir=$(dirname $aspRepoDir) # one level up
installDir=$baseDir/install

# packageDir will later be uploaded, as set in the yml file
packageDir=$baseDir/packages
testDir=$baseDir/StereoPipelineTest

# Throw an error unless on a Mac
isMac=$(uname -s | grep Darwin)
if [ "$isMac" = "" ]; then
    echo "This script is for Mac only"
    exit 1
fi

# See if this is Arm64 or Intel x64
isArm64=$(uname -m | grep arm64)

# The ASP dependencies at the location below are updated using the script
# save_mac_deps.sh. See that script for more info. Sometimes the names and
# versions of these change during development.
if [ "$isArm64" != "" ]; then
    echo "Platform: Arm64 Mac"
    tag=asp_deps_mac_arm64_v4
    envName=asp_deps
else
    echo "Platform: Intel Mac"
    tag=asp_deps_mac_x64_v4
    envName=asp_deps
fi

# Fetch and unzip the ASP dependencies
bbUrl=https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}
if [ "$isArm64" != "" ]; then
    # ARM64: conda-pack tarballs (relocatable; the bundled conda-unpack rewrites
    # the baked prefix on this runner). asp_deps is one part (< 2 GB after
    # conda-pack); the cat-glob handles 1 or 2 parts if it ever grows past the
    # GitHub 2 GB asset limit. The python env is a separate conda-pack tarball.
    # Then re-sign all Mach-O: conda-unpack patches binaries, which invalidates
    # their code signature, and Apple Silicon SIGKILLs binaries with a bad
    # signature (exit 137). An ad-hoc codesign fixes it. (Intel does not need
    # this; x86_64 macOS does not enforce signatures.)
    # conda is not on PATH this early in build_test.sh, so derive the envs dir
    # from $HOME (the env-discovery glob below also expects $HOME/*conda3/envs).
    envParent="$HOME/miniconda3/envs"
    wget ${bbUrl}/asp_deps_p1.tar.gz > /dev/null 2>&1
    wget ${bbUrl}/asp_deps_p2.tar.gz > /dev/null 2>&1   # may not exist (single part) - ok
    wget ${bbUrl}/python_isis10.tar.gz > /dev/null 2>&1
    mkdir -p "$envParent/asp_deps"
    cat asp_deps_p*.tar.gz | tar xzf - -C "$envParent/asp_deps"
    "$envParent/asp_deps/bin/conda-unpack"
    mkdir -p "$envParent/python_isis10"
    tar xzf python_isis10.tar.gz -C "$envParent/python_isis10"
    "$envParent/python_isis10/bin/conda-unpack"
    for e in asp_deps python_isis10; do
        find "$envParent/$e" -type f \( -name '*.dylib' -o -name '*.so' \) -print0 2>/dev/null \
            | xargs -0 -P8 codesign --force -s - 2>/dev/null
        find "$envParent/$e/bin" -type f -perm +111 -print0 2>/dev/null \
            | xargs -0 -P8 codesign --force -s - 2>/dev/null
    done
    # conda-pack drops some unversioned .dylib symlinks that VW/ASP cmake's
    # find_external_library requires (only the versioned lib survives the pack).
    # Recreate them from the first versioned lib found.
    ( cd "$envParent/asp_deps/lib" 2>/dev/null || exit 0
      for stem in libopenblas libnabo libpointmatcher; do
          if [ ! -e "$stem.dylib" ]; then
              v=$(ls "$stem".*.dylib 2>/dev/null | head -1)
              [ -n "$v" ] && ln -sf "$v" "$stem.dylib"
          fi
      done )
else
    # Intel x64: conda-pack tarballs (relocatable; the bundled conda-unpack
    # rewrites the baked prefix on this runner). asp_deps is one part (~1.6 GB
    # < 2 GB after conda-pack); the cat-glob handles 1 or 2 parts if it ever
    # grows past the GitHub 2 GB asset limit. The python env is a separate
    # conda-pack tarball.
    # conda is not on PATH this early in build_test.sh, so derive the envs dir
    # from $HOME (the env-discovery glob below also expects $HOME/*conda3/envs).
    envParent="$HOME/miniconda3/envs"
    wget ${bbUrl}/asp_deps_p1.tar.gz > /dev/null 2>&1
    wget ${bbUrl}/asp_deps_p2.tar.gz > /dev/null 2>&1   # may not exist (single part) - ok
    wget ${bbUrl}/python_isis10.tar.gz > /dev/null 2>&1
    mkdir -p "$envParent/asp_deps"
    cat asp_deps_p*.tar.gz | tar xzf - -C "$envParent/asp_deps"
    "$envParent/asp_deps/bin/conda-unpack"
    mkdir -p "$envParent/python_isis10"
    tar xzf python_isis10.tar.gz -C "$envParent/python_isis10"
    "$envParent/python_isis10/bin/conda-unpack"
fi

# The env can be in miniconda3 or anaconda3  
envPath=$(ls -d $HOME/*conda3/envs/${envName})
if [ ! -d "$envPath" ]; then
    echo "Error: Directory: $envPath does not exist"
    exit 1
fi

# Check a representative lib from each asp_* dep package is the host arch (a
# wrong-arch dylib is silently ignored by ld and later looks like undefined
# symbols). Check all, not just isis - they share a build string across arches.
wantArch=$(uname -m); [ "$wantArch" = "arm64" ] || wantArch="x86_64"
archCheckLibs="lib/libcore.dylib lib/libtexture_reconstruction.dylib lib/libmveCore.dylib lib/libtheia.dylib lib/libale.dylib lib/csmplugins/libusgscsm.dylib"
for rel in $archCheckLibs; do
    [ -e "$envPath/$rel" ] || continue
    gotArch=$(lipo -archs "$envPath/$rel" 2>/dev/null)
    case " $gotArch " in
        *" $wantArch "*) echo "Deps arch OK: $rel=[$gotArch], host=$wantArch" ;;
        *) echo "Error: DEPS TARBALL ARCH MISMATCH - $rel is [$gotArch] but host is $wantArch."
           echo "Re-pack with an isolated CONDA_PKGS_DIRS, force-reinstalling all asp_* packages."
           exit 1 ;;
    esac
done

export PATH=$envPath/bin:$PATH

# Activate the env. conda may not be on PATH this early (the workflow does not run
# setup-miniconda before build_test.sh), and the deps env is conda-pack relocated,
# so use the env's own activate script. This sets CONDA_PREFIX and the compiler
# environment - the conda clang wrapper needs CONDA_PREFIX to find its sysroot and
# headers, otherwise cmake's compiler checks (exp2, log2, ...) fail and the build
# cannot configure.
source "$envPath/bin/activate"

# No special linker flags needed. The deps tarball provides x86_64 isis libs and
# ASP links them with the default toolchain. (The earlier "undefined isis symbol"
# failures were an ARCH MISMATCH: the deps tarball had arm64 isis libs that ld
# silently ignores on this x86_64 runner - caused by a shared conda pkgs-cache
# collision at pack time. Fixed by packing the deps with an isolated x86_64
# pkgs cache. See env_update_06_2026_mac_intel.sh.)
cmake_opts=""

# Set up the compiler
if [ "$(uname)" = "Darwin" ]; then
    cc_comp=clang
    cxx_comp=clang++
else
    cc_comp=x86_64-conda-linux-gnu-gcc
    cxx_comp=x86_64-conda-linux-gnu-g++
fi
echo cc_comp=$cc_comp
echo cxx_comp=$cxx_comp

# Build libelas from source. The asp_deps tarball does not ship elas on Mac
# ARM64 (the old code used x86 SSE intrinsics).
mkdir -p $baseDir
cd $baseDir
git clone https://github.com/NeoGeographyToolkit/libelas.git
cd libelas
mkdir -p build
cd build
$envPath/bin/cmake ..                                      \
  -DTIFF_LIBRARY_RELEASE=$envPath/lib/libtiff.dylib        \
  -DTIFF_INCLUDE_DIR=$envPath/include                      \
  -DCMAKE_CXX_FLAGS=-I$envPath/include
make -j10
ans=$?
if [ "$ans" -ne 0 ]; then
    echo "Error: libelas build failed"
    build_failed=1
fi
mkdir -p $envPath/plugins/stereo/elas/bin
/bin/cp -fv elas $envPath/plugins/stereo/elas/bin/elas

# Build visionworkbench
mkdir -p $baseDir
cd $baseDir
git clone https://github.com/visionworkbench/visionworkbench.git
cd visionworkbench
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.10             \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp   \
  $cmake_opts
echo Building VisionWorkbench
make -j10 install > /dev/null 2>&1 # this is too verbose

# Log of the build, for inspection, in case it fails.
# This will resume from earlier.
out_build_vw=$(pwd)/output_build_vw.txt
make install > $out_build_vw 2>&1
tail -n 500 $out_build_vw
echo Log of VW build will be saved with the artifacts in $(basename $out_build_vw)

# Build StereoPipeline
cd $aspRepoDir
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.13             \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DVISIONWORKBENCH_INSTALL_DIR=$installDir       \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp   \
   $cmake_opts
echo Building StereoPipeline
make -j10 install > /dev/null 2>&1 # this is too verbose
ans=$?
if [ "$ans" -ne 0 ]; then
    echo "Error: StereoPipeline build failed"
    # Do not exit so we can save the build log
fi

# Log of the build, for inspection in case it fails
out_build_asp=$(pwd)/output_build_asp.txt
make install > $out_build_asp 2>&1
tail -n 500 $out_build_asp
echo Log of ASP build will be saved with the artifacts in $(basename $out_build_asp)

# Bugfix for duplicate LC_PATH failure. Wipe all values of LC_PATH.
for lib in $installDir/lib/*dylib; do
    for f in $(otool -l $lib | grep -A 3 LC_RPATH | grep path | awk '{print $2}'); do
        install_name_tool -delete_rpath  $f $lib
    done
done
export DYLD_LIBRARY_PATH=$installDir/lib:$DYLD_LIBRARY_PATH

# Package with BinaryBuilder. The Mac Arm and Mac x64 use
# different paths to the python environment.
echo Packaging the build
cd $baseDir
# Clone BinaryBuilder BEFORE setting DYLD_LIBRARY_PATH to include conda
# libs, as conda's libiconv conflicts with system git.
/usr/bin/git clone https://github.com/NeoGeographyToolkit/BinaryBuilder
cd BinaryBuilder
num=$(ls -d $HOME/*conda3/envs/python* | wc -l)
# Must have exactly one python env
if [ "$num" -ne 1 ]; then
    echo "Error: Expected exactly one python env, found $num"
    exit 1
fi
export ISISROOT=$envPath # needed for Mac Arm
# Do not add $envPath/lib to DYLD_LIBRARY_PATH. Conda's libiconv and ICU libs
# shadow system frameworks and crash CoreFoundation (Qt6 static init dies with
# "unrecognized selector" in CFStringGetFileSystemRepresentation on Sequoia).
# Use DYLD_FALLBACK_LIBRARY_PATH instead - it only kicks in when rpath/install_name
# lookup fails, so system libs are never shadowed.
export DYLD_FALLBACK_LIBRARY_PATH=$envPath/lib
# Qt6 crashes on macOS 15 during os version check in its static initializer.
# SYSTEM_VERSION_COMPAT=1 tells macOS to report version in a compatible way.
export SYSTEM_VERSION_COMPAT=1
./make-dist.py $installDir \
  --asp-deps-dir $envPath  \
  --python-env $(ls -d $HOME/*conda3/envs/python*)
if [ $? -ne 0 ]; then
    echo "Error: make-dist.py failed"
    build_failed=1
fi
# Prepare the package for upload
mkdir -p $packageDir
mv -fv Stereo* $packageDir

# Extract the tarball so we can test it
cd $packageDir
tarBall=$(ls StereoPipeline-*.tar.bz2 | head -n 1)
if [ "$tarBall" == "" ]; then
  echo Cannot find the packaged ASP tarball
  build_failed=1
fi
/usr/bin/time tar xjf $tarBall > /dev/null 2>&1 # this is verbose

# Path to executables
binDir=$packageDir/$tarBall
binDir=${binDir/.tar.bz2/}
binDir=$binDir/bin
export PATH=$binDir:$PATH
echo "Binaries are in $binDir"
if [ ! -d "$binDir" ]; then
    echo "Error: Directory: $binDir does not exist. Build failed."
    build_failed=1
fi

# TODO(oalexan1): Run the tests as a different step in the .yml file.

# Extract the tests. This tarball has both the scripts, test data,
# and the expected results.
# TODO(oalexan1): Must fetch the StereoPipelineTest repo and update
# the scripts extracted from the tarball.
cd $baseDir
echo Testing the build.
wget https://github.com/NeoGeographyToolkit/StereoPipelineTest/releases/download/0.0.1/StereoPipelineTest.tar > /dev/null 2>&1 # this is verbose

# Check if we got the tarball
if [ ! -f "StereoPipelineTest.tar" ]; then
    echo "Error: File: StereoPipelineTest.tar does not exist. Test failed."
    build_failed=1
fi
tar xfv StereoPipelineTest.tar > /dev/null 2>&1 # this is verbose

# Note: If the test results change, a new tarball with latest scripts and test
# results must be uploaded. That is done by running the script:
# StereoPipeline/.github/workflows/update_mac_tests.sh in the local directory.
# The nightly build script fetches the testa data with the latest and reference
# results in tarball StereoPipelineTest.tar. That artifact will be uploaded
# further down.

# Go to the test dir
if [ ! -d "$testDir" ]; then
    echo "Error: Directory: $testDir does not exist"
    build_failed=1
fi
cd $testDir

# Run the tests. Failed to install pytest, despite trying hard.
# Just run them manually.
reportFile=$(pwd)/output_test.txt
rm -f $reportFile
ans=0
for d in ss*; do 
    # Skip unless a directory
    if [ ! -d "$d" ]; then continue; fi

    cd $d
    echo Running test in $(pwd)
    ./run.sh > output.txt 2>&1
    ./validate.sh >> output.txt 2>&1
    ans0=$?
    tail -n 20 output.txt # this can be verbose
    echo "Test $d returned $ans0"
    echo "Test $d returned $ans0" >> $reportFile
    if [ "$ans0" -ne 0 ]; then ans=1; fi # keep record of failures
    cd ..
done
echo ans is $ans

# Set the test status. This is parsed after the build is downloaded.
echo test_status $ans >> $reportFile 

if [ "$ans" -eq 0 ]; then
    echo "All tests passed"
else
    # Do not quit, as we want to save the test results
    echo "Some tests failed"
fi

# Create the artifacts dir that will be saved
mkdir -p $packageDir
    
# Save the resulting test results as part of the artifacts. See above for how 
# to use this to update the test results in the cloud.
echo Copying the build
(cd $testDir/..; tar cf $packageDir/$(basename $testDir).tar $(basename $testDir))

# Save these logs as part of the artifacts
echo Copying the logs
cp -rfv $out_build_vw $out_build_asp $reportFile $packageDir

# Wipe the extracted tarball so we do not upload it
# TODO(oalexan1): Consider extracting it to a different location to start with
rm -rfv $(dirname $binDir) > /dev/null 2>&1

# Exit with failure if build/packaging broke (infrastructure failure)
if [ "$build_failed" -ne 0 ]; then
    echo "Build or packaging failed (see errors above)"
    exit 1
fi

# Exit with test status so CI reports failure when tests fail
exit $ans
