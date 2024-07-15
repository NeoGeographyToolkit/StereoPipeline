#!/bin/bash

# Fetch the ASP depenedencies
tag=mac_conda_env8
wget https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}/asp_deps.tar.gz > /dev/null 2>&1 # this is verbose
/usr/bin/time tar xzf asp_deps.tar.gz -C / > /dev/null 2>&1 # this is verbose

# The ASP dependencies at the location above are updated using the script
# save_mac_deps.sh. See that script for how to do the preparations. 
# Here is how the script is called, from a local machine (which need not be a Mac).
# The tag set here must match the tag above, and also in build_isis.sh.
# If changing here, must later change in the other places.
# tag=mac_conda_env8 
# workflow="ssh.yml" # manual workflow
# #workflow="build_isis.yml" # automatic workflow
# $HOME/projects/StereoPipeline/.github/workflows/save_mac_deps.sh $workflow $tag

# For linux, the dependencies from the local machine can be saved as follows.
# tag=linux_conda_env7
# $HOME/projects/StereoPipeline/.github/workflows/save_linux_deps.sh $tag

# Check that base dir is StereoPipeline
aspRepoDir=$(pwd) # same as $HOME/work/StereoPipeline/StereoPipeline
if [ "$(basename $aspRepoDir)" != "StereoPipeline" ]; then
    echo "Error: Directory: $aspRepoDir is not StereoPipeline"
    exit 1
fi

envName=asp_deps
envPath=$HOME/miniconda3/envs/${envName}
export PATH=$envPath/bin:$PATH
if [ ! -d "$envPath" ]; then
    echo "Error: Directory: $envPath does not exist"
    exit 1
fi

baseDir=$(dirname $aspRepoDir) # one level up
installDir=$baseDir/install

# packageDir will later be uploaded, as set in the yml file
packageDir=$baseDir/packages
testDir=$baseDir/StereoPipelineTest

# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi

# Build visionworkbench
mkdir -p $baseDir
cd $baseDir
git clone https://github.com/visionworkbench/visionworkbench.git
cd visionworkbench
mkdir -p build
cd build
$envPath/bin/cmake ..                             \
  -DASP_DEPS_DIR=$envPath                         \
  -DCMAKE_VERBOSE_MAKEFILE=ON                     \
  -DCMAKE_INSTALL_PREFIX=$installDir              \
  -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp
echo Building VisionWorkbench
make -j10 install > /dev/null 2>&1 # this is too verbose

# Log of the build, for inspection in case it fails
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
    #exit 1
fi

# Log of the build, for inspection in case it fails
out_build_asp=$(pwd)/output_build_asp.txt
make install > $out_build_asp 2>&1
tail -n 500 $out_build_asp

# Package with BinaryBuilder
echo Packaging the build
cd $baseDir
git clone https://github.com/NeoGeographyToolkit/BinaryBuilder
cd BinaryBuilder
./make-dist.py $installDir \
  --asp-deps-dir $envPath  \
  --python-env $HOME/miniconda3/envs/python_isis8
# Prepare the package for upload
mkdir -p $packageDir
mv -fv Stereo* $packageDir

# Extract the tarball so we can test it
cd $packageDir
tarBall=$(ls StereoPipeline-*.tar.bz2 | head -n 1)
if [ "$tarBall" == "" ]; then
  echo Cannot find the packaged ASP tarball
  # Do not exit so we can save the build log
  #exit 1
fi
tar xjfv $tarBall > /dev/null 2>&1 # this is verbose

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

# TODO(oalexan1): Run the tests as a diferent step in the .yml file.

# Extract the tests. This tarball has both the scripts, test data,
# and the expected results.
# TODO(oalexan1): Must fetch the StreoPipelineTest repo and update
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
# StereoPipeline/.github/workflows/update_mac_tests.sh

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
