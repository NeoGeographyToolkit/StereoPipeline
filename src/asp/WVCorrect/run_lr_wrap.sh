#!/bin/bash

# A wrapper around run_lr.sh, to make it easier to invoke it right with what stereo_gui dumps out
# when selecting a region. This wrapper will take as inputs the following ugly arguments:

# $(pwd) tag corr Crop src win for  WV02_20170511151556_103001006882D700_17MAY11151556-P1BS-501479705100_01_P002.ntf: -874 2797 36397 8486 Crop src win for  WV02_20170511151722_1030010069726100_17MAY11151722-P1BS-501493169020_01_P002.ntf: -552 6652 36618 7884 

# and will launch run_lr.sh for "left", while looking up the camera models.

# Instead of bothering with "right", just redo this while opening the
# images in the gui in reverse order. Note that the left crop win
# better fit inside the right crop win completely.

# Here, 'tag' is just a string label, and 'corr' is 0 when we we are working on making
# corrections, and is 1, when corrections are already in wv_correct and we want
# to verify how well they work.

execDir=$(dirname $0)

runDir=$1
shift

cd $runDir

tag=$1
shift

corr=$1
shift

shift; shift; shift; shift; # pass on the text: Crop src win for 

left_img=$(echo $1 | perl -p -e "s#:##g")
left_xml=${left_img/.ntf/.xml}
shift

a=$1; shift
b=$1; shift
c=$1; shift
d=$1; shift

shift; shift; shift; shift; # pass on the text: Crop src win for 

right_img=$(echo $1 | perl -p -e "s#:##g")
right_xml=${right_img/.ntf/.xml}
shift

e=$1; shift
f=$1; shift
g=$1; shift
h=$1; shift

echo $left_img $left_xml $right_img $right_xml $a $b $c $d $e $f $g $h $tag

echo Working in $runDir

out=output_${tag}_corr${corr}.txt

echo Writing output to $out

$execDir/run_lr.sh "$a $b $c $d" "$e $f $g $h" $left_img $left_xml $right_img $right_xml $tag $corr  > $out 2>&1





