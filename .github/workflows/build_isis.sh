#!/bin/bash

# Must wipe any cache before running this script, to ensure 
# the conda env at the end of this gets cached

cd
echo Now in $(pwd)

conda init bash
source /Users/runner/.bash_profile

echo listing envs
conda env list

echo Wiping old env
/bin/rm -rf /usr/local/miniconda/envs/asp_deps

/bin/rm -f isis_environment.yml
wget https://raw.githubusercontent.com/NeoGeographyToolkit/StereoPipeline/master/.github/isis_environment.yml
echo Creating a new asp_deps env
conda env create -n asp_deps -f isis_environment.yml 

conda activate asp_deps

conda install -c conda-forge parallel -y
conda install -c conda-forge rocksdb=8.5.3 rapidjson=1.1.0 -y

#conda install -c nasa-ames-stereo-pipeline -c usgs-astrogeology -c conda-forge libnabo=asp3.4.0_alpha libpointmatcher=asp3.4.0_alpha -y

# Make the python env
echo Creating a new python_isis8 env
/bin/rm -rf /usr/local/miniconda/envs/python_isis8
conda create -n python_isis8 python=3.12.0 numpy=1.26.2 -y

echo Will build ISIS3
conda activate asp_deps
git clone https://github.com/DOI-USGS/ISIS3.git     
cd ISIS3

mkdir build
cd build
export ISISROOT=$PWD
export PREFIX=/usr/local/miniconda/envs/asp_deps
cmake -GNinja -DJP2KFLAG=OFF -Dpybindings=OFF \
 -DbuildTests=OFF -DCMAKE_BUILD_TYPE=Release  \
 -DCMAKE_INSTALL_PREFIX=$PREFIX ../isis

export NINJAJOBS=2
/usr/bin/time ninja install -j 2

# Archive the conda env in the packages dir. This dir
# is set in the .yml file. It will be saved as 
# an artifact.
echo Will archive the conda env
packageDir=$HOME/work/StereoPipeline/packages
mkdir -p $packageDir
cd /usr/local/miniconda
/usr/bin/time tar czf $packageDir/asp_deps_osx.tar.gz envs
