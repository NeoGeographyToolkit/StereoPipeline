#!/bin/bash
# __BEGIN_LICENSE__
#  Copyright (c) 2009-2012, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__


# This script calls recontruct.cc to do albedo reconstruction.
# It must be run as:
# ./reconstruct.sh labelStr
# The results will go to albedo_$labelStr
if [ "$#" -lt 2 ]; then 
    echo Usage: $0 settingsFile labelStr
    exit
fi
settingsFile=$1
labelStr=$2

# Output directoy
resDir=albedo_$labelStr

numProc=8              # number of processes to use
if [ "$PBS_NODEFILE" != "" ]; then useSuperComp=1; else useSuperComp=0; fi

# Executables
VISION_WORKBENCH_PATH=$HOME/visionworkbench/src/vw/tools
STEREO_PIPELINE_PATH=$HOME/StereoPipeline/src/asp/Tools
reconstruct="$STEREO_PIPELINE_PATH/reconstruct"
image2qtree="$VISION_WORKBENCH_PATH/image2qtree"

# Steps of the algorithm:
# 0 Initial setup
# 1 For each DRG, compute the weights and shadows
# 2 For each albedo tile, compute the DEM
# 3 For each DRG, compute the reflectance and exposure
# 4 For each albedo tile, initialize the albedo
# 5 For each DRG, update the exposure
# 6 For each albedo tile, update albedo
# Steps 5 and 6 are repeated MAX_NUM_ITER times (as specified in the settings file)
# See the documentation at
# https://babelfish.arc.nasa.gov/trac/irg/wiki/MapMakers/albedo

# Validation
if [ ! -f "$settingsFile" ]; then echo "ERROR: File $settingsFile does not exist.";      exit; fi
if [ ! -x "$reconstruct"  ]; then echo "ERROR: Program $reconstruct is not executable."; exit; fi
if [ ! -x "$image2qtree"  ]; then echo "ERROR: Program $image2qtree is not executable."; exit; fi

imagesList="$resDir/imagesList.txt"
blankTilesList="$resDir/blankTilesList.txt" # To do: this is duplicated in the code!
options="-c $settingsFile -r $resDir -f $imagesList" 

# Parse some values from the settings file. Make sure to strip the comments
# as to not confuse the parsing.
DRG_DIR=$(        cat $settingsFile | perl -pi -e "s/\#.*?\n/\n/g" | grep DRG_DIR        | awk '{print $2}')
USE_TILES=$(      cat $settingsFile | perl -pi -e "s/\#.*?\n/\n/g" | grep USE_TILES      | awk '{print $2}')
MAX_NUM_ITER=$(   cat $settingsFile | perl -pi -e "s/\#.*?\n/\n/g" | grep MAX_NUM_ITER   | awk '{print $2}')
COMPUTE_ERRORS=$( cat $settingsFile | perl -pi -e "s/\#.*?\n/\n/g" | grep COMPUTE_ERRORS | awk '{print $2}')

# Temporary fix: For robustness, rebuild the indices of images each time it is run to avoid
# invalid indices.
#rm -fv $DRG_DIR/index.txt $DEM_DIR/index.txt $DEM_DIR/index.txt

# Wipe the results directory.
rm -rfv $resDir

# This run will take as input the box in which to compute the albedo, and will
# generate the list of DRG images intersecting that box, will create the albedo tiles,
# and also the index files if missing.
refImage=$(ls $DRG_DIR/*.tif | head -n 1) # Need this to get the GeoReference
if [ "$refImage" = "" ]; then echo "Error: No images."; exit; fi
overrideOptions="--initial-setup"
run="$wrapperCmd $reconstruct $options $overrideOptions -i $refImage"
echo Will run $run
$run
status=$?
# Must check the exit status and not continue if the above command failed
if [ $status -ne 0 ]; then exit; fi

# Create the list of drg images and tiles as expected by reconstruct.cc.
DRG_FILES=$(cat $imagesList | awk '{print $2}')
if [ $USE_TILES -ne 0 ]; then
    TILE_FILES=$(cat $blankTilesList | awk '{print $2}')
fi

numIter=$(perl -e "print 4 + 2*$MAX_NUM_ITER")
if [ "$numIter" = "" ]; then echo "The value of MAX_NUM_ITER is invalid"; exit; fi

# We need to do an extra iteration if we compute the error
if [ $COMPUTE_ERRORS -ne 0 ]; then ((numIter++)); fi;

for ((i = 1; i <= $numIter; i++)); do

    # See if this is the last iteration
    if [ $i -eq $numIter ]; then isLastIter=1; else isLastIter=0; fi

    # If we have to compute the errors, we do it at the last iteration
    computeErrors=0
    if [ $COMPUTE_ERRORS -ne 0 ] && [ $isLastIter -ne 0 ]; then computeErrors=1; fi

    # Decide if we iterate over tiles or over images
    tilesIter=0
    ((rem = i%2)) # remainder modulo 2
    if [ $USE_TILES -ne 0 ] && ( [ $rem -eq 0 ] || [ $computeErrors -ne 0 ] ); then
        # Iterate over albedo tiles
        tilesIter=1
        VALS="$TILE_FILES"
    else
        # Iterate over input DRGs
        VALS="$DRG_FILES"
    fi

    iterFile="$imagesList"

    # Enforce that iteration 7 uses same options as iteration 5, 8 as 6, etc.
    # This because iterations 5, 7, 9, ... update the exposure, while iterations
    # 6, 8, 10, ... update the albedo.
    rem=$i
    while [ $rem -gt 6 ]; do ((rem = rem - 2)); done 
    overrideOptions=""
    if [ $rem -eq 1 ]; then overrideOptions="--save-weights --compute-shadow"; fi
    if [ $rem -eq 2 ]; then overrideOptions="--init-dem";                      fi
    if [ $rem -eq 3 ]; then overrideOptions="--init-exposure";                 fi
    if [ $rem -eq 4 ]; then overrideOptions="--init-albedo";                   fi
    if [ $rem -eq 5 ]; then overrideOptions="--update-exposure";               fi
    if [ $rem -eq 6 ]; then overrideOptions="--update-albedo";                 fi
    
    # The case when we compute errors needs to be handled a bit differently.
    if [ $computeErrors -ne 0 ]; then overrideOptions="--compute-errors"; fi

    # The last iteration.
    if [ $isLastIter -ne 0 ]; then overrideOptions="$overrideOptions --is-last-iter"; fi
    
    # Run all jobs for the current stage
    image="{}" # will be filled in by xarg
    run="$wrapperCmd $reconstruct $options $overrideOptions -i $image"
    echo Will run $run
    # The 'xargs' and 'parallel' commands expect different formats
    if [ $useSuperComp -eq 1 ]; then
    currDir=$(pwd)    
    echo $VALS | perl -pi -e "s#\s+#\n#g" | parallel -P $numProc -u --sshloginfile $PBS_NODEFILE "cd $currDir; $run"
    else
    xargs="xargs -n 1 -P $numProc -I {} $run"
    echo $VALS | perl -pi -e "s#\s+#\n#g" | $xargs
    fi
    
done # end all iterations

# Visualize the results at the resolution they were generated
if [ $useSuperComp -eq 0 ]; then 
    if [ $USE_TILES -ne 0 ]; then
        make_kml_tiles.pl $resDir/albedo
        make_kml_tiles.pl $resDir/error
    else
        make_kml.pl $resDir
    fi
fi

# Visualize the results by doing extra sampling
if [ 0 -eq 1 ]; then 
    shoDir=$resDir/albedo_sub4
    mkdir $shoDir
    for i in $resDir/albedo/*.tif;
      do 
      echo $i;
      o=${i/albedo\//albedo_sub4\/} 
      echo "oFile = $o" 
      gdal_translate -outsize 25% 25% $i $o 
    done
    $VISION_WORKBENCH_PATH/image2qtree -m kml $shoDir/*.tif -o $shoDir
fi

