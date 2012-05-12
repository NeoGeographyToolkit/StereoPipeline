#!/bin/sh
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
