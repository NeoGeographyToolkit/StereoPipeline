#!/bin/sh
BASE_DIR=$HOME/projects/StereoPipeline/reconstructTest
#TIFFS=`ls $BASE_DIR/data/orbit33/DRG_sub4/*.tif | head -3`
TIFFS=$BASE_DIR/data/orbit33/DRG_sub4/*.tif
./reconstruct -d $BASE_DIR $TIFFS
