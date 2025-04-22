#!/bin/bash

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
if [ "$isMac" == "" ]; then
    echo "This script is for Mac only"
    exit 1
fi

# See if this is Arm64 or Intel x64
isArm64=$(uname -m | grep arm64)

# The ASP dependencies at the location below are updated using the script
# save_mac_deps.sh. See that script for more info.
# TODO(oalexan1): Reconcile these two
if [ "$isArm64" != "" ]; then
    echo "Platform: Arm64 Mac"
    tag=asp_deps_mac_arm64_v1
    env_name=isis_dev
else
    echo "Platform: Intel Mac"
    tag=asp_deps_mac_x64_v1
    envName=asp_deps
fi

# Fetch and unzip
wget https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}/asp_deps.tar.gz > /dev/null 2>&1 # this is verbose
/usr/bin/time tar xzf asp_deps.tar.gz -C $HOME > /dev/null 2>&1 # this is verbose

# The env can be in miniconda3 or anaconda3  
envPath=$(ls -d $HOME/*conda3/envs/${envName})
if [ ! -d "$envPath" ]; then
    echo "Error: Directory: $envPath does not exist"
    exit 1
fi
export PATH=$envPath/bin:$PATH

# Set up the compiler
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi
echo cc_comp=$cc_comp
echo cxx_comp=$cxx_comp

# Build visionworkbench
# TODO(oalexan1): Use here BinaryBuilder/build.py, just as for Linux.

mkdir -p $baseDir
cd $baseDir
git clone https://github.com/visionworkbench/visionworkbench.git
cd visionworkbench
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_VERBOSE_MAKEFILE=ON                     \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.10             \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp
echo Building VisionWorkbench
make -j10 install > /dev/null 2>&1 # this is too verbose

# Log of the build, for inspection, in case it fails.
# This will resume from earlier.
out_build_vw=$(pwd)/output_build_vw.txt
make install > $out_build_vw 2>&1
tail -n 500 $out_build_vw

# Build StereoPipeline
cd $aspRepoDir
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_VERBOSE_MAKEFILE=ON                     \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.13             \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DVISIONWORKBENCH_INSTALL_DIR=$installDir       \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp
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

# Package with BinaryBuilder. The Mac Arm and Mac x84 use
# different paths to the python environment.
echo Packaging the build
cd $baseDir
git clone https://github.com/NeoGeographyToolkit/BinaryBuilder
cd BinaryBuilder
./make-dist.py $installDir \
  --asp-deps-dir $envPath  \
  --python-env $(ls -d $HOME/*conda3/envs/python*)
# Prepare the package for upload
mkdir -p $packageDir
mv -fv Stereo* $packageDir

# Extract the tarball so we can test it
cd $packageDir
tarBall=$(ls StereoPipeline-*.tar.bz2 | head -n 1)
if [ "$tarBall" == "" ]; then
  echo Cannot find the packaged ASP tarball
  # Do not exit so we can save the build log
fi
/usr/bin/time tar xjf $tarBall > /dev/null 2>&1 # this is verbose

# Path to executables
binDir=$packageDir/$tarBall
binDir=${binDir/.tar.bz2/}
binDir=$binDir/bin
export PATH=$binDir:$PATH
echo "Binaries are in $binDir"
if [ ! -d "$binDir" ]; then
    echo "Error: Directory: $binDir does not exist"
    exit 1
fi

# TODO(oalexan1): Run the tests as a different step in the .yml file.

# Extract the tests. This tarball has both the scripts, test data,
# and the expected results.
# TODO(oalexan1): Must fetch the StereoPipelineTest repo and update
# the scripts extracted from the tarball.
cd $baseDir
echo Build done. Now testing.
wget https://github.com/NeoGeographyToolkit/StereoPipelineTest/releases/download/0.0.1/StereoPipelineTest.tar > /dev/null 2>&1 # this is verbose
# Check if we got the tarball
if [ ! -f "StereoPipelineTest.tar" ]; then
    echo "Error: File: StereoPipelineTest.tar does not exist"
    exit 1
fi
tar xfv StereoPipelineTest.tar > /dev/null 2>&1 # this is verbose

# Note: If the test results change, a new tarball with latest scripts and test
# results must be uploaded. That is done by running the script:
# StereoPipeline/.github/workflows/update_mac_tests.sh in the local directory
# having the Mac artifact fetched from the cloud, that is, the directory having
# the tarball StereoPipelineTest.tar.gz. That artifact will be uploaded further
# down.

# Go to the test dir
if [ ! -d "$testDir" ]; then
    echo "Error: Directory: $testDir does not exist"
    exit 1
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
    pwd
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
    
# Save the resulting test results as part of the artifacts
# This helps with debugging later
(cd $testDir/..; tar czf $packageDir/$(basename $testDir).tar.gz $(basename $testDir))

# Save these logs as part of the artifacts
cp -rfv $out_build_vw $out_build_asp $reportFile $packageDir

# Wipe the extracted tarball so we do not upload it
# TODO(oalexan1): Consider extracting it to a different location to start with
rm -rfv $(dirname $binDir) > /dev/null 2>&1
