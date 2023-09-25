#!/bin/bash

echo now in run_test.sh

dir=~/work/StereoPipeline/install/StereoPipeline-OSX
mkdir -p $dir

echo Created file > $dir/test.txt

echo content of file is $(cat $dir/test.txt)
