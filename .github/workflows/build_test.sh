#!/bin/bash

# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi

# Set up some variables
envName=asp_deps_3.4.0_alpha
aspRepoDir=$(pwd) # $HOME/work/StereoPipeline/StereoPipeline
# Check that base dir is StereoPipeline
if [ "$(basename $aspRepoDir)" != "StereoPipeline" ]; then
    echo "Error: Directory: $aspRepoDir is not StereoPipeline"
    exit 1
fi

baseDir=$(dirname $aspRepoDir) # one level up
installDir=$baseDir/install

envPath=/usr/local/miniconda/envs/${envName} 
if [ ! -d "$envPath" ]; then
  # Create the conda environment. When the environemnt changes, wipe its cached
  # version for this action, then this will recreate it.
  # Note the variable $envName. This must also be the environment name in the
  # .yaml file.
  conda env create -f conda/${envName}_osx_env.yaml
fi

# # For local testing
# if [ ! -d "$envPath" ]; then
#     envPath=$HOME/miniconda/envs/${envName}
# fi
# if [ ! -d "$envPath" ]; then
#     envPath=$HOME/miniconda3/envs/${envName}
# fi

if [ ! -d "$envPath" ]; then
    echo "Error: Directory: $envPath does not exist"
    exit 1
fi

# packageDir will later be uploaded, as set in the yml file
packageDir=$baseDir/packages
testDir=$baseDir/StereoPipelineTest


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
make -j10 && make install

# Log of the build, for inspection in case it fails
out_build_vw=$(pwd)/output_build_vw.txt
make > $out_build_vw 2>&1

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
make -j10 && make install

# Log of the build, for inspection in case it fails
out_build_asp=$(pwd)/output_build_asp.txt
make > $out_build_asp 2>&1

# Now package with BinaryBuilder
cd $baseDir
git clone https://github.com/NeoGeographyToolkit/BinaryBuilder
cd BinaryBuilder
./make-dist.py $installDir \
  --asp-deps-dir $envPath  \
  --python-env /usr/local/miniconda/envs/python_isis8

# Prepare the package for upload
mkdir -p $packageDir
mv -fv Stereo* $packageDir

# Extract the tarball so we can test it
cd $packageDir
tarBall=$(ls StereoPipeline-*.tar.bz2 | head -n 1)
if [ "$tarBall" == "" ]; then
  echo Cannot find the packaged ASP tarball
  exit 1
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
cd $baseDir
wget https://github.com/NeoGeographyToolkit/StereoPipelineTest/releases/download/0.0.1/StereoPipelineTest.tar  > /dev/null 2>&1 # this is verbose
tar xfv StereoPipelineTest.tar > /dev/null 2>&1 # this is verbose

# Go to test dir
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
    ./run.sh > /dev/null 2>&1
    # TODO(oalexan1): The validate.sh script can print a lot of verbose text.
    # Add a pipe to the head function. Then use ${PIPESTATUS[0]} to find the
    # exit code of this script.
    ./validate.sh
    ans0=$?
    echo "Test $d returned $ans0"
    echo "Test $d returned $ans0" >> ../$reportFile
    if [ "$ans0" -ne 0 ]; then ans=1; fi # keep record of failures
    cd ..
done
echo ans is $ans

if [ "$ans" -eq 0 ]; then
    echo "All tests passed"
else
    # TOODO(oalexan1): Make this a failure. For now just
    # save the results.
    echo "Some tests failed"
    # exit 1
fi
    
# Save the resulting test results as part of the artifacts
# This helps with debugging later
# TODO(oalexan1): Consider saving the test artifacts to a different file
mkdir -p $packageDir
# TODO(oalexan1): Consider creating this as a single tar file
cp -rfv $testDir $packageDir > /dev/null 2>&1

# Save these logs as part of the artifacts
cp -rfv $out_build_vw $out_build_asp $reportFile $packageDir

# Wipe the extracted tarball so we do not upload it
# TODO(oalexan1): Consider extracting it to a different location to start with
rm -rfv $(dirname $binDir) > /dev/null 2>&1
