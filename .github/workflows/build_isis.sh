#!/bin/bash

# This script builds the ASP dependencies for OSX in the cloud. If fetches the
# existing ones, builds/updates what is needed, and saves the updated
# dependencies as a tarball, which is then uploaded as an artifact.

# Offline, the artifact can be used to overwrite the existing dependencies. That
# is described in build_test.sh.

# Move from the source dir to the home dir
cd

# Set up the compiler
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=x86_64-conda_cos6-linux-gnu-gcc
  cxx_comp=x86_64-conda_cos6-linux-gnu-g++
fi

# Fetch the ASP depenedencies. Must keep $tag in sync with build_test.sh.
tag=mac_conda_env8
wget https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}/asp_deps.tar.gz > /dev/null 2>&1 # this is verbose
/usr/bin/time tar xzf asp_deps.tar.gz -C / > /dev/null 2>&1 # this is verbose

# Build ale. It is assumed the compiler is set up as above. May need to save the
# curent ~/.ssh/id_rsa.pub key to Github in the user settings for recursive
# cloning of the submodules to work.
cd
git clone https://github.com/DOI-USGS/ale.git --recursive
cd ale
git submodule update --recursive # if refreshing the repo later
#git rebase origin/main
git reset --hard 0ba7b24
export PREFIX=$HOME/miniconda3/envs/asp_deps
export PATH=$PREFIX/bin:$PATH
mkdir -p build && cd build
cmake ..                                         \
  -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp   \
  -DALE_USE_EXTERNAL_EIGEN=ON                    \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.10            \
  -DALE_USE_EXTERNAL_JSON=ON                     \
  -DALE_BUILD_DOCS=OFF                           \
  -DALE_BUILD_TESTS=OFF                          \
  -DCMAKE_VERBOSE_MAKEFILE=TRUE                  \
  -DCMAKE_INSTALL_PREFIX=${PREFIX}
make -j 20 install

# Build usgscsm. It is assumed the compiler is set up as above.
cd 
git clone https://github.com/DOI-USGS/usgscsm.git --recursive
cd usgscsm
git submodule update --recursive # if refreshing the repo later
#git rebase origin/main
git reset --hard 568ea46
mkdir -p build && cd build
export PREFIX=$HOME/miniconda3/envs/asp_deps
export PATH=$PREFIX/bin:$PATH
cmake ..                                         \
  -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp      \
  -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp   \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.10            \
  -DUSGSCSM_EXTERNAL_DEPS=ON                     \
  -DUSGSCSM_BUILD_DOCS=OFF                       \
  -DUSGSCSM_BUILD_TESTS=OFF                      \
  -DCMAKE_VERBOSE_MAKEFILE=TRUE                  \
  -DCMAKE_INSTALL_PREFIX=${PREFIX}
make -j 20 install

# Stop here this time. Other dependencies have been built.
exit 0

# Build ISIS3
cd
git clone https://github.com/DOI-USGS/ISIS3.git     
cd ISIS3
# Use latest, not 8.0.3, as that one does not compile.
mkdir build
cd build
export ISISROOT=$PWD
export PREFIX=$HOME/miniconda3/envs/asp_deps
export PATH=$PREFIX/bin:$PATH
ext=.so
if [ "$(uname)" = "Darwin" ]; then
    ext=.dylib
fi
/bin/rm -fv $PREFIX/version # to ensure the build does not fail
$PREFIX/bin/cmake                                  \
 -GNinja                                           \
 -DJP2KFLAG=OFF                                    \
 -Dpybindings=OFF                                  \
 -DbuildTests=OFF -DCMAKE_BUILD_TYPE=Release       \
 -DEMBREE_INCLUDE_DIR=$PREFIX/include              \
 -DEMBREE_LIBRARY=$PREFIX/lib/libembree3$ext       \
 -DPCL_INCLUDE_DIR=$PREFIX/include/pcl-1.13        \
 -DOPENCV_INCLUDE_DIR:PATH=$PREFIX/include/opencv4 \
 -DCMAKE_INSTALL_PREFIX=$PREFIX                    \
 ../isis
export NINJAJOBS=4; /usr/bin/time ninja install -j $NINJAJOBS # osx
#/usr/bin/time ninja install -j 14 # linux

# Create a tarball with the updated packages. It will be uploaded as an
# artifact. The destination directory is set in the .yml file.
#
# See build_test.sh for how to use this artifact to save the updated packages to
# a permanent location.
mkdir -p ~/work/StereoPipeline/packages
/usr/bin/time tar cfz ~/work/StereoPipeline/packages/asp_deps.tar.gz \
    /Users/runner/miniconda3/envs

# Done for now. Other packages have been built before.  
exit 0

# Must create an ssh key to be able to clone the repos
# ssh-keygen -t rsa
# Add the key /Users/runner/.ssh/id_rsa.pub to github in Settings -> SSH and GPG keys

# Turn on the steps below only if starting from scratch
if [ 1 -eq 0 ]; then 
  echo Wiping old env
  /bin/rm -rf /Users/runner/miniconda3/envs/asp_deps

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

# libnabo
cd
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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

# Multiview
cd
conda activate asp_deps
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
conda install -c conda-forge                      \
  rocksdb=8.5.3 rapidjson=1.1.0                   \
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
$PREFIX/bin/cmake ..                              \
    -DCMAKE_BUILD_TYPE=Release                    \
    -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp     \
    -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp  \
    -DMULTIVIEW_DEPS_DIR=${PREFIX}                \
    -DCMAKE_VERBOSE_MAKEFILE=ON                   \
    -DCMAKE_CXX_FLAGS='-O3 -std=c++11 -Wno-error' \
    -DCMAKE_C_FLAGS='-O3 -Wno-error'              \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}
make -j4
make install

# Build OpenImageIO. This is for debugging. Normally
# it would be built as part of MultiView.
cd
git clone https://github.com/NeoGeographyToolkit/oiio.git
cd oiio
mkdir build && cd build
#export PREFIX=$HOME/miniconda3/envs/asp_deps
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
$PREFIX/bin/cmake ..                              \
    -DCMAKE_BUILD_TYPE=Release                    \
    -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp     \
    -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp  \
    -DMULTIVIEW_DEPS_DIR=${PREFIX}                \
    -DCMAKE_VERBOSE_MAKEFILE=ON                   \
    -DCMAKE_CXX_FLAGS='-O3 -std=c++11 -Wno-error' \
    -DCMAKE_C_FLAGS='-O3'                         \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}              \
    -DBUILD_SHARED_LIBS=ON                        \
    -DUSE_PYTHON=OFF                              \
    -DUSE_OPENCV=OFF                              \
    -DUSE_QT=OFF                                  \
    -DUSE_DICOM=OFF                               \
    -DUSE_NUKE=OFF                                \
    -DUSE_LIBRAW=OFF                              \
    -DOIIO_BUILD_TESTS=OFF                        \
    -DOIIO_BUILD_TOOLS=OFF                        \
    -DUSE_OPENGL=OFF                              \
    -DBUILD_DOCS=OFF                              \
    -DINSTALL_DOCS=OFF                            \
    -DINSTALL_FONTS=OFF                           \
    -DOIIO_THREAD_ALLOW_DCLP=OFF                  \
    -DEMBEDPLUGINS=OFF                            \
    -DPROJECT_IS_TOP_LEVEL=OFF                    \
    -DUSE_TBB=OFF                                 \
    -DUSE_FIELD3D=OFF                             \
    -DUSE_OPENVDB=OFF                             \
    -DUSE_QT=OFF                                  \
    -DUSE_OCIO=OFF

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
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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
export PREFIX=/Users/runner/miniconda3/envs/asp_deps
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
