#!/bin/bash

# This script calls recontruct.cc to do albedo reconstruction.
# It must be run as:
# ./reconstruct.sh labelStr
# The results will go to albedo_$labelStr
if [ "$#" -eq 0 ]; then 
    echo Usage: $0 labelStr
    exit
fi
labelStr=$1

useTiles=0           # If 0, create albedo images, not tiles
sub=64               # must be a power of 2, from 1 to 64.
numProc=2            # number of processes to use
pixelPadding=5       # by how much to pad each tile, in pixels

# Simulation box, used when there are tiles, the convention is: W:E:S:N
simBox="6:10:-10:-9" # two tiles, gold testcase
#simBox="-180:180:-40:40" # all albedo 
tileSize=4 # tile size, in degrees

# Executable
recExe="$HOME/StereoPipeline/src/asp/Tools/reconstruct"
# Input directories
drgDir=../data/DIM_input_sub$sub
demDir=../data/DEM_input_sub$sub
demTilesDir=../data/DEM_tiles_sub$sub
cubDir=../data/cubes
# Output directories
resDir=../data/albedo_$labelStr
outCubDir=$resDir/cubes # A subset of the data in the cub directory for the given run
expDir=$resDir/exposure

# Temporary fix: For robustness, rebuild the indices of images each time it is run
# to avoid invalid indices.
rm -fv $drgDir/index.txt $demDir/index.txt $demTilesDir/index.txt

# Wipe and recreate the results directory.
rm -rfv $resDir
mkdir $resDir
echo $simBox > $resDir/simBox.txt

echo Results directory is $resDir
echo simBox is $simBox
echo tileSize is $tileSize
echo Sub is $sub
echo useTiles is $useTiles

# photometry_init_0_settings.txt     -- init albedo tiles
# photometry_init_1_settings.txt     -- for each DRG, compute weights and shadows
# photometry_init_2_settings.txt     -- for each albedo tile, compute DEM
# photometry_init_3_settings.txt     -- for each DRG, compute reflectance and exposure
# photometry_init_4_settings.txt     -- for each albedo tile, init albedo
# photometry_init_5_settings.txt     -- for each DRG, update exposure
# photometry_init_6_settings.txt     -- for each albedo tile, update albedo
# Repeat steps 5 and 6.

imagesList="$resDir/imagesList.txt" # will create this below, and will use it in subseq. iterations
blankTilesList="$resDir/blankTilesList.txt" # To do: this is duplicated in the code!

# Path to the reconstruct.cc executable
options1="--drg-directory $drgDir --dem-tiles-directory $demTilesDir -d $demDir"
options2="-s $outCubDir -e $expDir -r $resDir -b $simBox"
options3="-t $useTiles --tile-size $tileSize --pixel-padding $pixelPadding -f $imagesList"
options="$options1 $options2 $options3"

# This run will take as input the box in which to compute the albedo, and will
# generate the list of DRG images intersecting that box, will create the albedo
# tiles, and also the index files if missing.
refImage=$(ls $drgDir/*.tif | head -n 1) # Need this to get the GeoReference
if [ "$refImage" = "" ]; then echo "Error: No images."; exit; fi
settings=photometry_init_0_settings.txt
run="$recExe $options -c $settings -i $refImage"
$run

# Note that if we don't use tiles, we use the file filter.txt below
# in order to select which images to do. If this file does not 
# exist, run all images.
if [ $useTiles -eq 0 ]; then 
    if [ -e filter.txt ]; then 
        cat $drgDir/index.txt | ./filter.pl filter.txt > $imagesList
    else
        cat $drgDir/index.txt > $imagesList
    fi
fi

# Filter out the images which don't have sun/spacecraft info
cat $imagesList | ./filter.pl $cubDir/sunpos.txt > tmp.txt
mv -f tmp.txt $imagesList

# Create the list of drg images as expected by reconstruct.cc.
# Also create the corresponding sun/spacecraft position files
# for the current run.
TIFFS=$(ls $drgDir/*tif       | ./filter.pl $imagesList | perl -pi -e "s#\n# #g")
cat $cubDir/spacecraftpos.txt | ./filter.pl $imagesList > $outCubDir/spacecraftpos.txt 
cat $cubDir/sunpos.txt        | ./filter.pl $imagesList > $outCubDir/sunpos.txt

# Options to reconstruct.cc
#-d denotes the DEM directory
#-s denotes the spatial information directory used to determine the sun 
#   and satelite position
#-e denotes the exposure time directory
#-c denotes the configuration filename
#-r denotes the results directory

# See the documentation at
# https://babelfish.arc.nasa.gov/trac/irg/wiki/MapMakers/albedo

for ((i = 1; i <= 4; i++)); do

    tilesIter=0
    ((rem = i%2)) # remainder modulo 2
    if [ $useTiles -ne 0 ] && [ $rem -eq 0 ]; then
        # Iterate over albedo tiles
        tilesIter=1
        VALS=$(cat $blankTilesList | perl -pi -e "s#^\d*\s*(.*?tif).*?\n#\$1 #g")
    else
        # Iterate over input DRGs
        VALS="$TIFFS"
    fi

    # Run all jobs for the current stage
    settings=photometry_init_"$i"_settings.txt
    xargs="xargs -n 1 -P $numProc -I {} $recExe $options -c $settings -i {}"
    echo $VALS | perl -pi -e "s#\s+#\n#g" | $xargs
    
done # end all iterations

# Visualize the results by doing extra sampling
shoDir=$resDir/albedo_sub4
mkdir $shoDir
for i in $resDir/albedo/*.tif; do 
    echo $i;
    o=${i/albedo\//albedo_sub4\/} 
    echo "oFile = $o" 
    gdal_translate -outsize 25% 25% $i $o 
done
~/visionworkbench/src/vw/tools/image2qtree -m kml $shoDir/*.tif -o $shoDir

