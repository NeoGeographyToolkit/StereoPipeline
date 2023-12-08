#!/bin/bash

cd
echo Now in $(pwd)
conda init bash
source /Users/runner/.bash_profile

export NINJAJOBS=2
ninja build -j 2

