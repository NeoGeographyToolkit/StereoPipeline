#!/bin/sh

export DATA_DIR=/home/mfsmith3/a15
RESULTS_DIR=$HOME/results/a15/sub16
RECONSTRUCT=./reconstruct
SUB=_sub16
NUM_PROCESSORS=14

TIFFS_FILE=/tmp/tiffs-$USER.txt

#ls $DATA_DIR/DRG$SUB/*.tif | head -3 > $TIFFS_FILE
ls $DATA_DIR/DRG$SUB/*.tif > $TIFFS_FILE

TIFFS=$(cat $TIFFS_FILE)

#-d denotes the DEM directory
#-s denotes the spatial information directory used to determine the sun and satelite position
#-e denotes the exposure time directory
#-c denotes the configuration filename
#-r denotes the results directory
#-i gives the stereo pair to work on (default: work on all pairs)

SETTINGS=photometry_init_${1}_settings.txt

#single core implementation
# $RECONSTRUCT -d $DATA_DIR/DEM$SUB -s $DATA_DIR/meta -e $DATA_DIR/meta -r $RESULTS_DIR -c $SETTINGS $TIFFS

#paralel implementation
cat $TIFFS_FILE | xargs -n 1 -t -I{} -P$NUM_PROCESSORS $RECONSTRUCT -d $DATA_DIR/DEM$SUB -s $DATA_DIR/meta -e $DATA_DIR/meta -r $RESULTS_DIR -c $SETTINGS -i {} $TIFFS
