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

# Fetch the ASP depenedencies
wget https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/mac_conda_env6/asp_deps.tar.gz
tar xzf asp_deps.tar.gz -C / > /dev/null 2>&1 # this is verbose

# How to update the dependencies. Read very carefully and update as needed.
if [ 1 -eq 0 ]; then

  # Log in with ssh.yml, and create the needed dependencies, per the ASP manual,
  # section on building ASP. Or fetch existing ones as above.
  # Update the dependencies as needed. Then archive them as follows:
  
  mkdir -p ~/work/StereoPipeline/packages
  /usr/bin/time tar cfz ~/work/StereoPipeline/packages/asp_deps.tar.gz /usr/local/miniconda/envs/asp_deps /usr/local/miniconda/envs/python_isis8
  
  # When ssh.yml exits, it will cache ~/work/StereoPipeline/packages
  # This can be fetched on a local machine, then the desired tarball
  # can be pushed to the cloud updating the above wget link.
  # That goes as follows:
  gh=/home/oalexan1/miniconda3/envs/gh/bin/gh
  repo=git@github.com:NeoGeographyToolkit/StereoPipeline.git

  # Query the ssh.yml. Must check that that the top-most run is successful
  $gh run list -R $repo --workflow=ssh.yml
  
  ans=$($gh run list -R $repo --workflow=ssh.yml | grep -v STATUS | head -n 1)
  completed=$(echo $ans | awk '{print $1}')
  success=$(echo $ans | awk '{print $2}')
  id=$(echo $ans | awk '{print $7}')
  echo Completed is $completed
  echo Success is $success
  echo Id is $id
  if [ "$success" != "success" ]; then
    echo "Error: The ssh.yml workflow did not succeed"
  else 
    echo Fetching the build with id $id from the cloud 
    echo $gh run download -R $repo $id
    /bin/rm -rf ssh-test-macOS # Must wipe this first, or else the download can fail
    $gh run download -R $repo $id

    # Must be careful with the line below. This comes from ssh.yml
    binaries=ssh-test-macOS/asp_deps.tar.gz
    if [ ! -f "$binaries" ]; then
      echo "Error: File: $binaries does not exist"
      exit 1
    fi 
    
    repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git
    tag=mac_conda_env6 # Must be the same tag as in the wget link above
    # Wipe old version
    $gh release -R $repo delete $tag 
    notes="Mac conda env6"
    # Add the binaries
    /usr/bin/time $gh release -R $repo create $tag $binaries --title $tag --notes "$notes"
  fi
fi
  
# Set up some variables
envName=asp_deps
#envName=asp_deps_3.4.0_alpha
aspRepoDir=$(pwd) # $HOME/work/StereoPipeline/StereoPipeline
# Check that base dir is StereoPipeline
if [ "$(basename $aspRepoDir)" != "StereoPipeline" ]; then
    echo "Error: Directory: $aspRepoDir is not StereoPipeline"
    exit 1
fi

# Install some tools
# TODO(oalexan1): Must have a short environment.yml file
# having all the dependencies ASP needs.
echo Installing some tools
conda init bash
source ~/.bash_profile
conda activate $envName
conda install -c conda-forge -y parallel pbzip2

# These need installing for now
conda install -c nasa-ames-stereo-pipeline -c usgs-astrogeology -c conda-forge geoid=1.0_isis7 htdp=1.0_isis7 -y

baseDir=$(dirname $aspRepoDir) # one level up
installDir=$baseDir/install

envPath=/usr/local/miniconda/envs/${envName}
if [ ! -d "$envPath" ]; then
    echo "Error: Directory: $envPath does not exist"
    exit 1
fi

# The logic below is turned off for now, as the env was
# created and cached manually by logging into the cloud machine.
#if [ ! -d "$envPath" ]; then
#   # Create the conda environment. When the environemnt changes, wipe its cached
#   # version for this action, then this will recreate it.
#   # Note the variable $envName. This must also be the environment name in the
#   # .yaml file.
#   conda env create -f conda/${envName}_osx_env.yaml
# fi

# # For local testing
# if [ ! -d "$envPath" ]; then
#     envPath=$HOME/miniconda/envs/${envName}
# fi
# if [ ! -d "$envPath" ]; then
#     envPath=$HOME/miniconda3/envs/${envName}
# fi
# if [ ! -d "$envPath" ]; then
#     echo "Error: Directory: $envPath does not exist"
#     exit 1
# fi

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
echo Building VisionWorkbench
make -j10 install > /dev/null 2>&1 # this is too verbose

# Log of the build, for inspection in case it fails
out_build_vw=$(pwd)/output_build_vw.txt
make install > $out_build_vw 2>&1
tail -n 500 $out_build_vw

# Temporary fix for the csm frame camera
perl -pi -e "s#private:#public:#g" $envPath/include/usgscsm/UsgsAstroFrameSensorModel.h

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
  --python-env /usr/local/miniconda/envs/python_isis8

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
# results must be uploaded. Here's how that is done.
# TODO(oalexan1): This must be a tool, and documented in StereoPipelineTest.
if [ 1 -eq 0 ]; then
  # Fetch the latest artifact of this run. It will have the current test results,
  # since later in this script we have a step that saves the test results.
  # Fetching of the artifact can be done with gh, as in an example above.
  # Inspect all tests. Update the failed ones (each 'gold' is overwritten with 'run').
  # Make the new 'run' directory the new 'gold'. Do not keep the 'run' directories.
  # Push this updated tarball to the cloud. 
  
  # Go to the directory having the StereoPipelineTest.tar.gz (after fetching the artifact)
  f=StereoPipelineTest.tar.gz
  # Check if it exists
  if [ ! -f "$f" ]; then
    echo "Error: File: $f does not exist"
    exit 1
  fi
  # Extract
  tar xzfv $f > /dev/null 2>&1 # this is verbose
  if [ ! -d "StereoPipelineTest" ]; then
    echo "Error: Directory: StereoPipelineTest does not exist"
    exit 1
  fi
  for f in StereoPipelineTest/ss*/run; do 
    g=${f/run/gold}
    /bin/rm -rfv $g
    /bin/mv -fv $f $g
  done
  # Must make all scripts in bin and individual tests executable
  chmod a+x StereoPipelineTest/bin/* StereoPipelineTest/*/*sh
  # Create a new tarball
  binaries=StereoPipelineTest.tar
  tar cfv $binaries StereoPipelineTest 
  repo=git@github.com:NeoGeographyToolkit/StereoPipelineTest.git  
  gh=/home/oalexan1/miniconda3/envs/gh/bin/gh
  tag=0.0.1
  $gh release -R $repo delete $tag # wipe old tarball
  notes="Update test results"
  $gh release -R $repo create $tag $binaries --title $tag --notes "$notes" # upload new
fi

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
