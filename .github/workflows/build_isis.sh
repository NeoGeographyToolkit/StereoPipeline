#!/bin/bash


# This is a scratch pad of commands used to build ASP dependencies for OSX
# in the cloud, while connecting with ssh.yml. It will go away once
# conda packages are released for the needed version of these.

cd
echo Now in $(pwd)

# Fetch the ASP dependencies, as described in build_test.sh. Update the needed
# packages as further down this document. Then archive them as in build_test.sh,
# and continue the rest of the instructions from there.

# Set up the conda env
conda init bash
source /Users/runner/.bash_profile
echo listing envs
conda env list
conda activate asp_deps

# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi

# Must create an ssh key to be able to clone the repos
ssh-keygen -t rsa
# Add the key /Users/runner/.ssh/id_rsa.pub to github in Settings -> SSH and GPG keys

# Build only the rig_calibrator component of MultiView 
git clone https://github.com/NeoGeographyToolkit/MultiView.git --recursive
cd MultiView
mkdir -p build && cd build
export PREFIX=/usr/local/miniconda/envs/asp_deps
$PREFIX/bin/cmake                              \
  -DBUILD_RIG_CALIBRATOR_ONLY=ON               \
  -DCMAKE_VERBOSE_MAKEFILE=TRUE                \
  -DMULTIVIEW_DEPS_DIR=${PREFIX}               \
  -DCMAKE_INSTALL_PREFIX=${PREFIX}             \
  -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp    \
  -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp \
  ..

make -j 10 install

# See instructions in build_test.sh for how to upload the built packages

# Turn on the steps below only if starting from scratch
if [ 1 -eq 0 ]; then 
  echo Wiping old env
  /bin/rm -rf /usr/local/miniconda/envs/asp_deps

  # Fetch the isis env
  /bin/rm -f isis_environment.yml
  wget https://raw.githubusercontent.com/NeoGeographyToolkit/StereoPipeline/master/.github/isis_environment.yml

  # Create the asp_deps env
  echo Creating a new asp_deps env
  conda env create -n asp_deps -f isis_environment.yml 
  conda activate asp_deps
fi

# Install some needed tools
cd
conda install -c conda-forge -y parallel pbzip2

# Install the needed packages
cd
conda install -c nasa-ames-stereo-pipeline -c usgs-astrogeology -c conda-forge geoid=1.0_isis7 htdp=1.0_isis7 -y

# Build ale
cd
#git clone git@github.com:DOI-USGS/ale.git --recursive
git clone https://github.com/DOI-USGS/ale.git --recursive
cd ale
git reset --hard 775ff21
conda activate asp_deps
# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi
export PREFIX=/usr/local/miniconda/envs/asp_deps
mkdir -p build && cd build
cmake .. -DALE_USE_EXTERNAL_EIGEN=ON -DALE_USE_EXTERNAL_JSON=ON -DUSGSCSM_EXTERNAL_DEPS=ON -DCMAKE_VERBOSE_MAKEFILE=TRUE -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp -DALE_BUILD_DOCS=OFF -DCMAKE_INSTALL_PREFIX=${PREFIX} -DALE_BUILD_TESTS=OFF -DUSGSCSM_BUILD_TESTS=OFF -DALE_USE_EXTERNAL_JSON=ON -DALE_USE_EXTERNAL_JSON=ON -DALE_USE_EXTERNAL_EIGEN=ON -DALE_BUILD_TESTS=OFF
make -j 20 install

# Continue with building usgscsm with the same env as above
cd 
#git clone git@github.com:USGS-Astrogeology/usgscsm.git --recursive
#git clone https://github.com/DOI-USGS/usgscsm.git --recursive
git clone git@github.com:oleg-alexandrov/usgscsm.git --recursive
cd usgscsm
#git reset --hard 0f065ca
git checkout height_radtan_fixes
mkdir -p build && cd build
cmake .. -DALE_USE_EXTERNAL_EIGEN=ON -DALE_USE_EXTERNAL_JSON=ON -DUSGSCSM_EXTERNAL_DEPS=ON -DCMAKE_VERBOSE_MAKEFILE=TRUE -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp -DUSGSCSM_BUILD_DOCS=OFF -DALE_BUILD_DOCS=OFF -DCMAKE_INSTALL_PREFIX=${PREFIX} -DALE_BUILD_TESTS=OFF -DUSGSCSM_BUILD_TESTS=OFF -DALE_USE_EXTERNAL_JSON=ON -DALE_USE_EXTERNAL_JSON=ON -DALE_USE_EXTERNAL_EIGEN=ON -DALE_BUILD_TESTS=OFF
make -j 20 install

# Build ISIS3
cd
echo Will build ISIS3
conda activate asp_deps
#git clone https://github.com/DOI-USGS/ISIS3.git     
git clone https://github.com/oleg-alexandrov/ISIS3.git
cd ISIS3
git checkout 1b129bd84
mkdir build
cd build
export ISISROOT=$PWD
export PREFIX=/usr/local/miniconda/envs/asp_deps
cmake -GNinja -DJP2KFLAG=OFF -Dpybindings=OFF \
 -DbuildTests=OFF -DCMAKE_BUILD_TYPE=Release  \
 -DCMAKE_INSTALL_PREFIX=$PREFIX ../isis
export NINJAJOBS=2
/usr/bin/time ninja install -j 2

# Must do usgscsm
# Init the shell and activate the asp_deps env
cd
export PREFIX=/usr/local/miniconda/envs/asp_deps
git clone https://github.com/DOI-USGS/usgscsm.git
cd usgscsm
git checkout 1.7.0
perl -pi -e "s#private:#public:#g" include/usgscsm/UsgsAstroFrameSensorModel.h 
mkdir -p build && cd build
$PREFIX/bin/cmake .. -DUSGSCSM_EXTERNAL_DEPS=ON -DCMAKE_INSTALL_PREFIX=${PREFIX}  \
    -DCMAKE_BUILD_TYPE=Release -DUSGSCSM_BUILD_TESTS=OFF
make -j 20 && make install

# libnabo
cd
export PREFIX=/usr/local/miniconda/envs/asp_deps
git clone https://github.com/oleg-alexandrov/libnabo.git
cd libnabo
mkdir build && cd build
cmake                                          \
  -DCMAKE_BUILD_TYPE=Release                   \
  -DCMAKE_CXX_FLAGS='-O3 -std=c++11'           \
  -DCMAKE_C_FLAGS='-O3'                        \
  -DCMAKE_INSTALL_PREFIX:PATH=${PREFIX}        \
  -DEIGEN_INCLUDE_DIR=${PREFIX}/include/eigen3 \
  -DCMAKE_PREFIX_PATH=${PREFIX}                \
  -DBoost_DIR=${PREFIX}/lib                    \
  -DBoost_INCLUDE_DIR:PATH=${PREFIX}/include   \
  -DBUILD_SHARED_LIBS=ON                       \
  -DCMAKE_VERBOSE_MAKEFILE=ON                  \
  ..
make -j10 install

# libpointmacher
cd 
export PREFIX=/usr/local/miniconda/envs/asp_deps
conda activate asp_deps
git clone https://github.com/oleg-alexandrov/libpointmatcher.git
cd libpointmatcher
mkdir build && cd build
cmake                                          \
  -DCMAKE_BUILD_TYPE=Release                   \
  -DCMAKE_CXX_FLAGS='-O3 -std=c++11'           \
  -DCMAKE_C_FLAGS='-O3'                        \
  -DCMAKE_INSTALL_PREFIX:PATH=${PREFIX}        \
  -DEIGEN_INCLUDE_DIR=${PREFIX}/include/eigen3 \
  -DCMAKE_PREFIX_PATH=${PREFIX}                \
  -DBoost_DIR=${PREFIX}/lib                    \
  -DBoost_INCLUDE_DIR:PATH=${PREFIX}/include   \
  -DLIBNABO_INSTALL_DIR=${PREFIX}              \
  -DBUILD_SHARED_LIBS=ON                       \
  -DCMAKE_VERBOSE_MAKEFILE=ON                  \
  -DUSE_SYSTEM_YAML_CPP=OFF                    \
  -DBoost_NO_BOOST_CMAKE=OFF                   \
  -DCMAKE_VERBOSE_MAKEFILE=ON                  \
  -DBoost_DEBUG=ON                             \
  -DBoost_DETAILED_FAILURE_MSG=ON              \
  -DCMAKE_CXX_COMPILER_ARCHITECTURE_ID=x64     \
  -DBoost_NO_SYSTEM_PATHS=ON                   \
  ..
make -j 10 install

# fgr
cd
git clone https://github.com/oleg-alexandrov/FastGlobalRegistration.git
cd FastGlobalRegistration
export PREFIX=/usr/local/miniconda/envs/asp_deps
export SRC_DIR=$(pwd)
mkdir build && cd build
CUSTOM_SOURCE_DIR=${SRC_DIR}/source
INC_FLAGS="-I${PREFIX}/include/eigen3 -I${PREFIX}/include -O3 -L${PREFIX}/lib -lflann_cpp -llz4 -O3 -std=c++11"
cmake                                        \
  -DCMAKE_BUILD_TYPE=Release                 \
  -DCMAKE_CXX_FLAGS="${INC_FLAGS}"           \
  -DCMAKE_INSTALL_PREFIX:PATH=${PREFIX}      \
  -DCMAKE_PREFIX_PATH=${PREFIX}              \
  -DCMAKE_VERBOSE_MAKEFILE=ON                \
  -DFastGlobalRegistration_LINK_MODE=SHARED  \
  ${CUSTOM_SOURCE_DIR}
make -j10
# Install
INC_DIR=${PREFIX}/include/FastGlobalRegistration
mkdir -p ${INC_DIR}
/bin/cp -fv ${CUSTOM_SOURCE_DIR}/FastGlobalRegistration/app.h ${INC_DIR}
LIB_DIR=${PREFIX}/lib
mkdir -p ${LIB_DIR}
/bin/cp -fv FastGlobalRegistration/libFastGlobalRegistrationLib* ${LIB_DIR}

#s2p
cd
export PREFIX=/usr/local/miniconda/envs/asp_deps
conda activate asp_deps
conda install -c conda-forge -y fftw=3.3.10   
git clone https://github.com/oleg-alexandrov/s2p.git --recursive
cd s2p
# update recursive submodules
git submodule update --init --recursive
export CFLAGS="-I$PREFIX/include -O3 -DNDEBUG -march=native"
export LDFLAGS="-L$PREFIX/lib"
# Fix for missing liblzma
#perl -pi -e "s#(/[^\s]*?lib)/lib([^\s]+).la#-L\$1 -l\$2#g" ${PREFIX}/lib/*.la
baseDir=$(pwd)
# Extension
if [ "$(uname)" = "Darwin" ]; then
    EXT='.dylib'
else
    EXT='.so'
fi
# Build the desired programs
cd 3rdparty/mgm
perl -pi -e "s#CFLAGS=#CFLAGS=$CFLAGS #g" Makefile
perl -pi -e "s#LDFLAGS=#LDFLAGS=$LDFLAGS #g" Makefile 
make -j${CPU_COUNT}
cd $baseDir
# msmw
cd 3rdparty/msmw
mkdir -p build
cd build
cmake .. -DCMAKE_C_FLAGS="$CFLAGS" -DCMAKE_CXX_FLAGS="$CFLAGS" \
    -DPNG_LIBRARY_RELEASE="${PREFIX}/lib/libpng${EXT}"     \
    -DTIFF_LIBRARY_RELEASE="${PREFIX}/lib/libtiff${EXT}"   \
    -DZLIB_LIBRARY_RELEASE="${PREFIX}/lib/libz${EXT}"      \
    -DJPEG_LIBRARY="${PREFIX}/lib/libjpeg${EXT}"
make -j${CPU_COUNT}
cd $baseDir
# msmw2
cd 3rdparty/msmw2
mkdir -p build
cd build
cmake ..                                                   \
    -DCMAKE_C_FLAGS="$CFLAGS" -DCMAKE_CXX_FLAGS="$CFLAGS"  \
    -DPNG_LIBRARY_RELEASE="${PREFIX}/lib/libpng${EXT}"     \
    -DTIFF_LIBRARY_RELEASE="${PREFIX}/lib/libtiff${EXT}"   \
    -DZLIB_LIBRARY_RELEASE="${PREFIX}/lib/libz${EXT}"      \
    -DJPEG_LIBRARY="${PREFIX}/lib/libjpeg${EXT}"
make -j${CPU_COUNT}
cd $baseDir
# Install the desired programs
BIN_DIR=${PREFIX}/plugins/stereo/mgm/bin
mkdir -p ${BIN_DIR}
/bin/cp -fv 3rdparty/mgm/mgm ${BIN_DIR}
BIN_DIR=${PREFIX}/plugins/stereo/msmw/bin
mkdir -p ${BIN_DIR}
/bin/cp -fv \
    3rdparty/msmw/build/libstereo/iip_stereo_correlation_multi_win2 \
    ${BIN_DIR}/msmw
BIN_DIR=${PREFIX}/plugins/stereo/msmw2/bin
mkdir -p ${BIN_DIR}
/bin/cp -fv \
    3rdparty/msmw2/build/libstereo_newversion/iip_stereo_correlation_multi_win2_newversion \
    ${BIN_DIR}/msmw2

# libelas
cd 
export PREFIX=/usr/local/miniconda/envs/asp_deps
conda activate asp_deps
git clone https://github.com/NeoGeographyToolkit/libelas.git
cd libelas
# Set the env
export CFLAGS="-I$PREFIX/include -O3 -DNDEBUG -ffast-math -march=native"
export LDFLAGS="-L$PREFIX/lib"
# Fix for missing liblzma
#perl -pi -e "s#(/[^\s]*?lib)/lib([^\s]+).la#-L\$1 -l\$2#g" ${PREFIX}/lib/*.la
# Extension
if [ "$(uname)" = "Darwin" ]; then
    EXT='.dylib'
else
    EXT='.so'
fi
# build
mkdir -p build
cd build
cmake .. -DTIFF_LIBRARY_RELEASE="${PREFIX}/lib/libtiff${EXT}" \
    -DTIFF_INCLUDE_DIR="${PREFIX}/include"                    \
    -DCMAKE_CXX_FLAGS="-I${PREFIX}/include"
make -j${CPU_COUNT}
# Copy the 'elas' tool to the plugins subdir meant for it
BIN_DIR=${PREFIX}/plugins/stereo/elas/bin
mkdir -p ${BIN_DIR}
/bin/cp -fv elas ${BIN_DIR}/elas

# multiview
cd
conda activate asp_deps
export PREFIX=/usr/local/miniconda/envs/asp_deps
conda install -c conda-forge rocksdb=8.5.3 rapidjson=1.1.0 \
  ilmbase=2.5.5 openexr=2.5.5 -y
git clone https://github.com/NeoGeographyToolkit/MultiView.git --recursive
cd MultiView
# Must have ssh authentication set up for github
git submodule update --init --recursive
mkdir -p build && cd build
# # For OSX use a custom location for TBB. This is a fix for a conflict with embree.
# # When that package gets updated to version 3 or 4 this may become unnecesary.
# opt=""
# if [[ $target_platform =~ osx.* ]]; then
# 	opt="-DTBB_LIBRARY=${PREFIX}/lib/libtbb.12.dylib -DTBB_MALLOC_LIBRARY=${PREFIX}/lib/libtbbmalloc.2.dylib"
# fi
cmake ..                                    \
    -DCMAKE_BUILD_TYPE=Release              \
    -DMULTIVIEW_DEPS_DIR=${PREFIX}          \
    -DCMAKE_VERBOSE_MAKEFILE=ON             \
    -DCMAKE_CXX_FLAGS='-O3 -std=c++11'      \
    -DCMAKE_C_FLAGS='-O3'                   \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}
make -j4
make install

# Make the python env
echo Creating a new python_isis8 env
/bin/rm -rf /usr/local/miniconda/envs/python_isis8
conda create -n python_isis8 python=3.12.0 numpy=1.26.2 -y

# Build visionworkbench
cd 
conda activate asp_deps
# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi
export PREFIX=/usr/local/miniconda/envs/asp_deps
git clone https://github.com/visionworkbench/visionworkbench.git
cd visionworkbench
mkdir -p build
cd build
$PREFIX/bin/cmake ..                         \
  -DASP_DEPS_DIR=$PREFIX                     \
  -DCMAKE_VERBOSE_MAKEFILE=ON                \
  -DCMAKE_INSTALL_PREFIX=$PREFIX             \
  -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp \
  -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp
echo Building VisionWorkbench
make -j10 install

# Build StereoPipeline
cd
conda activate asp_deps
# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi
export PREFIX=/usr/local/miniconda/envs/asp_deps
git clone https://github.com/NeoGeographyToolkit/StereoPipeline.git
cd StereoPipeline
mkdir -p build
cd build
$PREFIX/bin/cmake ..                         \
  -DASP_DEPS_DIR=$PREFIX                     \
  -DCMAKE_VERBOSE_MAKEFILE=ON                \
  -DCMAKE_INSTALL_PREFIX=$PREFIX             \
  -DVISIONWORKBENCH_INSTALL_DIR=$PREFIX      \
  -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp  \
  -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp
echo Building StereoPipeline
make -j10 install > /dev/null 2>&1 # this is too verbose

# Package with BinaryBuilder
echo Packaging the build
cd
conda activate asp_deps
export PREFIX=/usr/local/miniconda/envs/asp_deps
git clone https://github.com/NeoGeographyToolkit/BinaryBuilder
cd BinaryBuilder
./make-dist.py $PREFIX   \
  --asp-deps-dir $PREFIX \
  --python-env $(dirname $PREFIX)/python_isis8

# Archive the conda env in the packages dir. This dir
# is set in the .yml file. It will be saved as 
# an artifact.
echo Will archive the conda env
packageDir=$HOME/work/StereoPipeline/packages
mkdir -p $packageDir
cd /usr/local/miniconda
/usr/bin/time tar czf $packageDir/asp_deps_osx.tar.gz envs

