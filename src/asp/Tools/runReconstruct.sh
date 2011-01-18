#!bin/sh

#TIFFS=`ls ../data/orbit33/DRG_sub4/*.tif | head -3`
#TIFFS=`ls ../data/orbit33/DRG_sub4/*.tif | tail -3`  
TIFFS=`ls ../data/orbit33/DRG_sub4/*tif`
#echo $TIFFS

#-d denotes the DEM directory
#-s denotes the spatial information directory used to determine the sun and satelite position
#-e denotes the exposure time directory
#-c denotes the configuration filename
#-r denotes the results directory

#single core implementation
../src/asp/Tools/reconstruct -d ../data/orbit33/dem_sub4 -s  ../data/orbit33 -e ../data/orbit33 -r ../results/orbit33_sub4 -c photometry_settings.txt $TIFFS

#paralel implementation
#echo ../data/orbit33/DRG_sub4/*.tif|xargs -n 1 -I {} -P 2 ../src/asp/Tools/reconstruct -d ../data/orbit33/dem_sub4  -i {} -s ../data/orbit33 -e ../data/orbit33 -r ../results/orbit33_sub4 -c photometry_init_settings.txt $TIFFS
#echo ../data/orbit33/DRG_sub4/*.tif|xargs -n 1 -I {} -P 2 ../src/asp/Tools/reconstruct -d ../data/orbit33/dem_sub4  -i {} -s ../data/orbit33 -e ../data/orbit33 -r ../results/orbit33_sub4 -c photometry_settings.txt $TIFFS



