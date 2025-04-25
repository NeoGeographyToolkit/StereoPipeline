#!/bin/bash

# This is a debugging script that builds the ASP dependencies for OSX in the
# cloud. It also shows how the result of building can be uploaded as an
# artifact, and later moved to a permanent location.

# This script is not meant to be run directly. Each block of code must be inspected,
# edited, and run separately.
 
# This helps do a dress rehearsal for the build process, before using
# conda-build which is very slow and error-prone.

# It is shown how packages can be compiled from source, and further down
# how to use conda-build for each of them.

# After the dependencies are updated with this script, they can be saved for the
# future with the script save_mac_deps.sh. See that script for more info.

# Alternatively, from within the ssh session on GitHub, the dependencies can be
# saved as follows. This needs fetching gh and doing auth.
conda create -n gh -c conda-forge gh
gh=$(ls -d $HOME/*conda3/envs/gh/bin/gh)
$gh auth login
binaries=~/work/StereoPipeline/packages/asp_deps.tar.gz # save in the right dir
mkdir -p $(dirname $binaries)
cd $HOME
/usr/bin/time tar cfz $binaries $(ls -d miniconda3/envs/*)
repo=git@github.com:NeoGeographyToolkit/BinaryBuilder.git
tag=asp_deps_mac_x64_v3
$gh release -R $repo delete $tag -y # Wipe the old release. Careful here.
$gh release -R $repo create $tag $binaries --notes "$tag" --title "$tag" 

# Move from the source dir to the home dir
cd

# Set up the compiler. Using a known compiler that is in the environment ensures
# there are no surprises when later conda-build is employed with the same
# compiler.
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
  cc_comp=clang
  cxx_comp=clang++
else
  cc_comp=gcc
  cxx_comp=g++
fi

# Fetch the ASP dependencies. Must keep $tag in sync with build_test.sh.
tag=asp_deps_mac_x64_v3
cd $HOME
wget https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}/asp_deps.tar.gz > /dev/null 2>&1 # this is verbose
/usr/bin/time tar xzf asp_deps.tar.gz > /dev/null 2>&1 # this is verbose

conda init bash
source ~/.bash_profile

# Build ale. It is assumed the compiler is set up as above. May need to save the
# current ~/.ssh/id_rsa.pub key to Github in the user settings for recursive
# cloning of the submodules to work.
cd
git clone https://github.com/DOI-USGS/ale.git --recursive
cd ale
git submodule update --recursive # if refreshing the repo later
#git rebase origin/main
git reset --hard 0ba7b24
export PREFIX=$HOME/miniconda3/envs/isis8.3.0
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
export PREFIX=$HOME/miniconda3/envs/isis8.3.0
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
export PREFIX=$HOME/miniconda3/envs/isis8.3.0
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
# See the top of the document for how to save / fetch a tarball with dependencies.

# Done for now. Other packages have been built before.  
exit 0

# Must create an ssh key to be able to clone the repos
# ssh-keygen -t rsa
# Add the key /Users/runner/.ssh/id_rsa.pub to github in Settings -> SSH and GPG keys

# Turn on the steps below only if starting from scratch
if [ 1 -eq 0 ]; then 
  echo Wiping old env
  /bin/rm -rf /Users/runner/miniconda3/envs/isis8.3.0

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
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
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

# libpointmatcher
cd 
export PREFIX=$HOME/miniconda3/envs/isis8.3.0
git clone https://github.com/oleg-alexandrov/libpointmatcher.git
cd libpointmatcher
mkdir build && cd build
cmake                                          \
  -DCMAKE_BUILD_TYPE=Release                   \
  -DCMAKE_CXX_COMPILER_ARCHITECTURE_ID=x64     \
  -DCMAKE_CXX_FLAGS="-O3 -std=c++17"           \
  -DCMAKE_C_FLAGS='-O3'                        \
  -DCMAKE_INSTALL_PREFIX=${PREFIX}             \
  -DCMAKE_VERBOSE_MAKEFILE=ON                  \
  -DCMAKE_PREFIX_PATH=${PREFIX}                \
  -DCMAKE_VERBOSE_MAKEFILE=ON                  \
  -DBUILD_SHARED_LIBS=ON                       \
  -DEIGEN_INCLUDE_DIR=${PREFIX}/include/eigen3 \
  -DBoost_DIR=${PREFIX}/lib                    \
  -DBoost_INCLUDE_DIR=${PREFIX}/include        \
  -DBoost_NO_BOOST_CMAKE=OFF                   \
  -DBoost_DEBUG=ON                             \
  -DBoost_DETAILED_FAILURE_MSG=ON              \
  -DBoost_NO_SYSTEM_PATHS=ON                   \
  ..
make -j 10 install

# fgr
cd
git clone https://github.com/oleg-alexandrov/FastGlobalRegistration.git
cd FastGlobalRegistration
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
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
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
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
export PREFIX=$(ls -d ~/*conda3/envs/isis8.3.0)
export PATH=$PREFIX/bin:$PATH
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
cmake ..                                               \
  -DTIFF_LIBRARY_RELEASE="${PREFIX}/lib/libtiff${EXT}" \
  -DTIFF_INCLUDE_DIR="${PREFIX}/include"               \
  -DCMAKE_CXX_FLAGS="-I${PREFIX}/include"
  
make -j${CPU_COUNT}
# Copy the 'elas' tool to the plugins subdir meant for it
BIN_DIR=${PREFIX}/plugins/stereo/elas/bin
mkdir -p ${BIN_DIR}
/bin/cp -fv elas ${BIN_DIR}/elas

# Multiview
cd
conda activate asp_deps
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
conda install -c conda-forge                      \
  rocksdb=8.5.3 rapidjson=1.1.0                   \
  ilmbase=2.5.5 openexr=2.5.5 imath -y
# Note: For Mac may need to build openexr from source  
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
    -DCMAKE_OSX_DEPLOYMENT_TARGET=10.13           \
    -DCMAKE_VERBOSE_MAKEFILE=ON                   \
    -DCMAKE_CXX_FLAGS='-O3 -std=c++11 -Wno-error' \
    -DCMAKE_C_FLAGS='-O3 -Wno-error'              \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}

make -j4
make install

# PDAL
git clone https://github.com/PDAL/PDAL.git
cd PDAL
git checkout 2.6.0
mkdir build
cd build
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
if [ "$(uname)" = "Darwin" ]; then
    EXT='.dylib'
else
    EXT='.so'
fi
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
    cc_comp=clang
    cxx_comp=clang++
else
    cc_comp=gcc
    cxx_comp=g++
fi
ldflags="-Wl,-rpath,${PREFIX}/lib -L${PREFIX}/lib -lgeotiff -lcurl -lssl -lxml2 -lcrypto -lzstd -lz"
cmake ${CMAKE_ARGS}                                      \
  -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp              \
  -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp           \
  -DBUILD_SHARED_LIBS=ON                                 \
  -DCMAKE_BUILD_TYPE=Release                             \
  -DCMAKE_INSTALL_PREFIX=$PREFIX                         \
  -DCMAKE_PREFIX_PATH=$PREFIX                            \
  -DDIMBUILDER_EXECUTABLE=$DIMBUILDER                    \
  -DBUILD_PLUGIN_I3S=OFF                                 \
  -DBUILD_PLUGIN_TRAJECTORY=OFF                          \
  -DBUILD_PLUGIN_E57=OFF                                 \
  -DBUILD_PLUGIN_PGPOINTCLOUD=ON                         \
  -DBUILD_PLUGIN_ICEBRIDGE=OFF                           \
  -DBUILD_PLUGIN_NITF=OFF                                \
  -DBUILD_PLUGIN_TILEDB=ON                               \
  -DBUILD_PLUGIN_HDF=ON                                  \
  -DBUILD_PLUGIN_DRACO=OFF                               \
  -DENABLE_CTEST=OFF                                     \
  -DWITH_TESTS=OFF                                       \
  -DWITH_ZLIB=ON                                         \
  -DWITH_ZSTD=ON                                         \
  -DWITH_LASZIP=ON                                       \
  -DWITH_LAZPERF=ON                                      \
  -DCMAKE_VERBOSE_MAKEFILE=ON                            \
  -DCMAKE_CXX17_STANDARD_COMPILE_OPTION="-std=c++17"     \
  -DCMAKE_VERBOSE_MAKEFILE=ON                            \
  -DWITH_TESTS=OFF                                       \
  -DCMAKE_EXE_LINKER_FLAGS="$ldflags"                    \
  -DDIMBUILDER_EXECUTABLE=dimbuilder                     \
  -DBUILD_PLUGIN_DRACO:BOOL=OFF                          \
  -DOPENSSL_ROOT_DIR=${PREFIX}                           \
  -DLIBXML2_INCLUDE_DIR=${PREFIX}/include/libxml2        \
  -DLIBXML2_LIBRARIES=${PREFIX}/lib/libxml2${EXT}        \
  -DLIBXML2_XMLLINT_EXECUTABLE=${PREFIX}/bin/xmllint     \
  -DGDAL_LIBRARY=${PREFIX}/lib/libgdal${EXT}             \
  -DGDAL_CONFIG=${PREFIX}/bin/gdal-config                \
  -DZLIB_INCLUDE_DIR:PATH=${PREFIX}/include              \
  -DZLIB_LIBRARY:FILEPATH=${PREFIX}/lib/libz${EXT}       \
  -DCURL_INCLUDE_DIR=${PREFIX}/include                   \
  -DPostgreSQL_LIBRARY_RELEASE=${PREFIX}/lib/libpq${EXT} \
  -DCURL_LIBRARY_RELEASE=${PREFIX}/lib/libcurl${EXT}     \
  -DPROJ_INCLUDE_DIR:PATH=${PREFIX}/include              \
  -DPROJ_LIBRARY:FILEPATH=${PREFIX}/lib/libproj${EXT}    \
  ..

# OpenEXR
# Build from source, to ensure the proper version of ilmbase is used
# git clone git@github.com:NeoGeographyToolkit/openexr-feedstock.git
# conda build -c nasa-ames-stereo-pipeline -c usgs-astrogeology -c conda-forge openexr-feedstock
# conda install -c nasa-ames-stereo-pipeline -c usgs-astrogeology -c conda-forge --force-reinstall openexr 
wget https://github.com/AcademySoftwareFoundation/openexr/archive/v2.5.5.tar.gz
cd openexr-2.5.5
mkdir build && cd build
conda activate isis_dev
export PREFIX=$(ls -d ~/*conda3/envs/{asp_deps,isis_dev})
if [ ! -d "$PREFIX" ]; then
  echo "Error: $PREFIX does not exist. Exiting."
  #exit 1
fi
$PREFIX/bin/cmake ..  \
  -DCMAKE_BUILD_TYPE=Release \
   -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp     \
   -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp  \
  -DCMAKE_PREFIX_PATH=$PREFIX \
  -DCMAKE_VERBOSE_MAKEFILE=ON    \
  -DCMAKE_CXX_FLAGS='-O3 -std=c++11 -w' \
  -DCMAKE_C_FLAGS='-O3 -w' \
  -DCMAKE_INSTALL_PREFIX=${PREFIX} \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.10

# Build OpenImageIO. This is for debugging. Normally it would be built as part
# of MultiView. Note: The lengthy command below gets truncated if pasted in a
# terminal on github.
conda install -c conda-forge 'cmake>=3.20' compilers 
cd ~/work/StereoPipeline/
#git clone https://github.com/NeoGeographyToolkit/oiio.git
git clone git@github.com:NeoGeographyToolkit/oiio.git
cd oiio
mkdir -p build && cd build
export PREFIX=$HOME/miniconda3/envs/isis8.3.0
if [ ! -d "$PREFIX" ]; then
  echo "Error: $PREFIX does not exist. Exiting."
  #exit 1
fi
$PREFIX/bin/cmake ..  \
  -Wdev \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$PREFIX \
  -DMULTIVIEW_DEPS_DIR=${PREFIX} \
  -DCMAKE_VERBOSE_MAKEFILE=ON    \
  -DCMAKE_CXX_FLAGS='-O3 -std=c++11 -w' \
  -DCMAKE_C_FLAGS='-O3 -w' \
  -DCMAKE_INSTALL_PREFIX=${PREFIX} \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=10.10 \
  -DBUILD_SHARED_LIBS=ON \
  -DUSE_PYTHON=OFF  \
  -DUSE_OPENCV=OFF  \
  -DUSE_QT=OFF      \
  -DUSE_DICOM=OFF   \
  -DUSE_NUKE=OFF    \
  -DUSE_LIBRAW=OFF  \
  -DOIIO_BUILD_TESTS=OFF \
  -DOIIO_BUILD_TOOLS=OFF \
  -DUSE_OPENGL=OFF  \
  -DOIIO_THREAD_ALLOW_DCLP=OFF \
  -DEMBEDPLUGINS=OFF \
  -DUSE_QT=OFF      \
  -DUSE_FIELD3D=OFF \
  -DUSE_OPENVDB=OFF \
  -DUSE_OCIO=OFF    \
  -DUSE_TBB=OFF     \
  -DPROJECT_IS_TOP_LEVEL=OFF \
  -DBUILD_DOCS=OFF   \
  -DINSTALL_DOCS=OFF  \
  -DINSTALL_FONTS=OFF

# Build theia
cd ~/work/StereoPipeline
conda install -c conda-forge vlfeat
conda install -c conda-forge rapidjson=1.1.0
conda install -c conda-forge rocksdb=8.5.3 gflags glog ceres-solver mesalib
# On linux, install mesa-libgl-cos6-x86_64

git clone	git@github.com:NeoGeographyToolkit/TheiaSfM.git
cd TheiaSfM
mkdir -p build && cd build
export PREFIX=$HOME/miniconda3/envs/isis8.3.0
if [ ! -d "$PREFIX" ]; then
  echo "Error: $PREFIX does not exist. Exiting."
  #exit 1
fi
$PREFIX/bin/cmake ..                                   \
    -DCMAKE_BUILD_TYPE=Release                         \
    -DMULTIVIEW_DEPS_DIR=${PREFIX}                     \
    -DCMAKE_VERBOSE_MAKEFILE=ON                        \
    -DCMAKE_CXX_FLAGS='-O3 -std=c++11 -Wno-error'      \
    -DCMAKE_C_FLAGS='-O3 -Wno-error'                   \
    -DCMAKE_MODULE_PATH=$PREFIX/share/pcl-1.13/Modules \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}

# Build Multiview
# Dependencies. Use precisely same compiler that will be used in the conda recipe
git submodule update --init --recursive
conda install -c conda-forge vlfeat \
  gflags=2.2.2 glog=0.7.1 \
  ceres-solver=2.2.0 \
  vlfeat \
 'clang >=16,<17' 'clangxx >=16,<17'
conda install -c conda-forge \
   rapidjson=1.1.0 \
   rocksdb=8.5.3 
# Build directory
mkdir build && cd build
# For OSX use a custom location for TBB. This is a fix for a conflict with embree.
# When that package gets updated to version 3 or 4 this may become unnecessary.
opt=""
if [[ $target_platform =~ osx.* ]]; then
	opt="-DTBB_LIBRARY=${PREFIX}/lib/libtbb.12.dylib -DTBB_MALLOC_LIBRARY=${PREFIX}/lib/libtbbmalloc.2.dylib"
fi
isMac=$(uname -s | grep Darwin)
if [ "$isMac" != "" ]; then
    cc_comp=clang
    cxx_comp=clang++
else
    cc_comp=gcc
    cxx_comp=g++
fi
# Enforce a compiler we know to work
cmake ..                                               \
    -DCMAKE_BUILD_TYPE=Release                         \
    -DCMAKE_C_COMPILER=${PREFIX}/bin/$cc_comp          \
    -DCMAKE_CXX_COMPILER=${PREFIX}/bin/$cxx_comp       \
    -DMULTIVIEW_DEPS_DIR=${PREFIX}                     \
    -DCMAKE_VERBOSE_MAKEFILE=ON                        \
    -DCMAKE_CXX_FLAGS='-O3 -std=c++11'                 \
    -DCMAKE_C_FLAGS='-O3'                              \
    -DCMAKE_MODULE_PATH=$PREFIX/share/pcl-1.13/Modules \
    -DCMAKE_INSTALL_PREFIX=${PREFIX}                   \
	$opt
# Build
make -j${CPU_COUNT}
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
conda install -c conda-forge openblas
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
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
export PREFIX=/Users/runner/miniconda3/envs/isis8.3.0
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

# Activate conda
source  /Users/runner/.bash_profile 
conda activate anaconda 
conda search -c nasa-ames-stereo-pipeline  --override-channels  --platform osx-64

# Install anaconda client and conda build separately
# create a new tool env for that
conda create -n anaconda -c conda-forge -c defaults -y anaconda-client conda-build
conda activate anaconda

# Authenticate
cat ~/.ssh/id_rsa.pub 
# Add to github ssh keys

# Save current dependencies
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml

# See the top of document for how to save / fetch a tarball with dependencies

# geoid
git clone https://github.com/NeoGeographyToolkit/geoid-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml geoid-feedstock
conda activate anaconda
conda build -c nasa-ames-stereo-pipeline -c conda-forge geoid-feedstock
anaconda upload /Users/runner/miniconda3/conda-bld/osx-64/geoid-1.0_asp3.5.0-2.conda         
/Users/runner/miniconda3/bin/conda  install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 geoid=1.0_asp3.5.0

# ilmbase
git clone https://github.com/NeoGeographyToolkit/ilmbase-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml ilmbase-feedstock
conda build -c conda-forge -c nasa-ames-stereo-pipeline ilmbase-feedstock
~/miniconda3/bin/anaconda upload /Users/runner/miniconda3/conda-bld/osx-64/ilmbase-2.5.5-h01edc0c_1.conda
#conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 ilmbase=2.5.5
/Users/runner/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0  nasa-ames-stereo-pipeline::ilmbase=2.5.5   
/Users/runner/miniconda3/envs/anaconda/bin/anaconda upload /Users/runner/miniconda3/conda-bld/osx-64/ilmbase-2.5.5-h01edc0c_0.conda
/Users/runner/miniconda3/bin/conda  install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 ilmbase=2.5.5

# openexr
https://github.com/NeoGeographyToolkit/openexr-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml openexr-feedstock
~/miniconda3/bin/anaconda upload upload /Users/runner/miniconda3/conda-bld/osx-64/openexr-2.5.5-ha5a8b8e_0.conda
/Users/runner/miniconda3/bin/conda  install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0  nasa-ames-stereo-pipeline::openexr=2.5.5

# htdp
git clone https://github.com/NeoGeographyToolkit/htdp-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml htdp-feedstock
conda build -c conda-forge -c nasa-ames-stereo-pipeline htdp-feedstock
~/miniconda3/bin/anaconda upload upload /Users/runner/miniconda3/conda-bld/osx-64/htdp-1.0_asp3.5.0-1.conda
/Users/runner/miniconda3/bin/conda  install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0  nasa-ames-stereo-pipeline::htdp=1.0_asp3.5.0

# visionworkbench
git clone https://github.com/NeoGeographyToolkit/visionworkbench-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml visionworkbench-feedstock
conda build -c conda-forge -c nasa-ames-stereo-pipeline visionworkbench-feedstock 2>&1 | tee output.txt
~/miniconda3/bin/anaconda upload upload /Users/runner/miniconda3/conda-bld/osx-64/visionworkbench-asp3.5.0-0.conda
~/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 visionworkbench

# libnabo
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml
git clone https://github.com/NeoGeographyToolkit/libnabo-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml libnabo-feedstock 
conda activate anaconda
conda build -c nasa-ames-stereo-pipeline -c conda-forge libnabo-feedstock
~/miniconda3/bin/anaconda upload /Users/runner/miniconda3/conda-bld/osx-64/libnabo-asp3.5.0-h01edc0c_1.conda
~/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 libnabo

# fgr
git clone https://github.com/NeoGeographyToolkit/fgr-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml fgr-feedstock
conda activate anaconda
conda build -c nasa-ames-stereo-pipeline -c conda-forge fgr-feedstock
anaconda upload  /Users/runner/miniconda3/conda-bld/osx-64/fgr-asp3.5.0-h01edc0c_0.conda 
~/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 fgr

# libpointmatcher
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml
git clone https://github.com/NeoGeographyToolkit/libpointmatcher-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml libpointmatcher-feedstock
conda activate anaconda
conda build -c nasa-ames-stereo-pipeline -c conda-forge libpointmatcher-feedstock
anaconda upload /Users/runner/miniconda3/conda-bld/osx-64/libpointmatcher-asp3.5.0-ha5a8b8e_0.conda
~/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 libpointmatcher

# pdal
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml
git clone https://github.com/NeoGeographyToolkit/pdal-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml pdal-feedstock
# Do not use ASP GDAL so exclude the ASP channel
conda activate anaconda
conda build -c conda-forge pdal-feedstock

# s2p
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml
git clone https://github.com/NeoGeographyToolkit/s2p-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml s2p-feedstock
conda build -c nasa-ames-stereo-pipeline -c conda-forge s2p-feedstock
anaconda upload  /Users/runner/miniconda3/conda-bld/osx-64/s2p-subset-asp3.5.0-h01edc0c_0.conda 

# libelas
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml
git clone https://github.com/NeoGeographyToolkit/libelas-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml libelas-feedstock
conda build -c nasa-ames-stereo-pipeline -c conda-forge libelas-feedstock
~/miniconda3/bin/anaconda upload /Users/runner/miniconda3/conda-bld/osx-64/libelas-asp3.5.0-h01edc0c_0.conda 
~/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 libelas

# Build Multiview with conda. Ensure that the same compile tools
# are used as above.
cd ~/work/StereoPipeline
conda activate isis8.3.0; conda env export > isis8.3.0.yaml
git clone https://github.com/NeoGeographyToolkit/multiview-feedstock.git
python StereoPipeline/conda/update_versions.py isis8.3.0.yaml multiview-feedstock
conda build -c nasa-ames-stereo-pipeline -c conda-forge multiview-feedstock
~/miniconda3/bin/anaconda upload  /Users/runner/miniconda3/conda-bld/osx-64/multiview-asp_3.5.0-py310_0.conda 
~/miniconda3/bin/conda install -c nasa-ames-stereo-pipeline -c conda-forge -n isis8.3.0 multiview
