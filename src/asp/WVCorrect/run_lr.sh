#!/bin/bash

if [ "$#" -lt 4 ]; then 
    echo Usage: $0 left_crop_win right_crop_win left_image right_image left_cam right_cam; [left | right]
    exit; 
fi

execDir=/home/smcmich1/repo/StereoPipeline/src/asp/WVCorrect

echo execDir is $execDir

left_crop_win=$1
right_crop_win=$2
left_image=$3
right_image=$4
left_cam=$5
right_cam=$6

side=""
if [ "$#" -gt 6 ]; then
    side=$7
fi

#export ASP_PYTHON_MODULES_PATH=$HOME/projects/BinaryBuilder/StereoPipelinePythonModules/lib64/python2.6/site-packages:$HOME/projects/BinaryBuilder/StereoPipelinePythonModules/lib64/python2.6/site-packages/GDAL-1.10.0-py2.6-linux-x86_64.egg/osgeo:$HOME/projects/BinaryBuilder/StereoPipelinePythonModules/lib

#bundle_adjust $left_image $right_image $left_cam $right_cam -o run_ba/run

base_cmd="parallel_stereo --corr-seed-mode 1 --subpixel-kernel 13 13
  --part-of-multiview-run --alignment-method homography
  --subpixel-mode 1 --corr-max-levels 2 --disable-fill-holes
  --corr-timeout 300 --bundle-adjust-prefix run_ba/run"

projwin=$(gdalinfo -proj4 dem.tif 2>/dev/null |grep -i "proj=" | perl -pi -e "s#\'##g")
if [ "$projwin" != "" ]; then
    projwin=$projwin # Wipe quotes
    echo "Found projwin from dem: $projwin"
else
    projwin="+proj=stere +lat_0=90 +lat_ts=70 +lon_0=-45 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs "
    echo "Using default projwin $projwin"
fi

for i in 1 2; do

    if [ "$i" == "1" ]; then

        if [ "$side" == "right" ]; then continue; fi # Skip if right was requested

        dir=run_lr
        leftc=""
        rightc=""
        if [ "$left_crop_win" != "" ];  then
            leftc="--left-image-crop-win $left_crop_win"
        else
            rm -rfv ./$dir
        fi

        if [ "$right_crop_win" != "" ]; then rightc="--right-image-crop-win $right_crop_win"; fi

        rm -rf $dir
        cmd="$base_cmd $leftc $rightc $left_image $right_image $left_cam $right_cam $dir/run"

        echo $cmd
    else

        if [ "$side" == "left" ]; then continue; fi # Skip if left was requested

        dir=run_rl
        leftc=""
        rightc=""
        if [ "$right_crop_win" != "" ]; then
            leftc="--left-image-crop-win $right_crop_win";
        else
            rm -rfv ./$dir
        fi

        if [ "$left_crop_win" != "" ];  then rightc="--right-image-crop-win $left_crop_win"; fi

        cmd="$base_cmd $leftc $rightc $right_image $left_image $right_cam $left_cam $dir/run"

        echo $cmd
    fi

    $cmd

    point2dem $dir/run-PC.tif --errorimage --tr 2 --t_srs "$projwin"

    rm -fv ./$dir/run-PC.tif ./$dir/run-D.tif ./$dir/run-RD.tif

    disparitydebug $dir/run-F.tif
    stereo_gui --create-image-pyramids-only $dir/run-F-H.tif $dir/run-F-V.tif

    # This needs to be manually run for each data set using good bounds!
    $execDir/disp_avg $dir/run-F.tif $dir/dx.txt $dir/dy.txt
done

exit

# To do: Force that all runs be redone below!

# PATH to the ASP build compiled for merope.
source ~/.bashenv

export PATH=$HOME/projects/StereoPipeline/src/asp/Tools:$HOME/projects/visionworkbench/src/vw/tools:$HOME/projects/base_system/bin:$HOME/projects/packages/bin:$HOME/bin:$PATH
# Path for merope
v=$(stereo_fltr 2>&1 |grep -i "bin/sed" | perl -pi -e "s#\s##g")
if [ "$v" != "" ]; then
    export PATH=$HOME/projects/StereoPipeline2/src/asp/Tools:$PATH
fi


dir=$1
if [ "$dir" != "" ]; then cd $dir; fi

name=$2
opts=$3
tag=""
# p=$5
# s=$6

runDir="fixed"$tag"$name" # "_"$p"_"$s
outFile=output_"$runDir".txt
echo runDir=$runDir
echo Will write to $(pwd)/$outFile
exec &> $outFile 2>&1

ans=$($execDir/print_files.pl)
l=$(echo $ans | awk '{print $1}')
r=$(echo $ans | awk '{print $2}')

turnon0=1
#a=$(grep -i scandir $l.xml | grep -i reverse)
#if [ "$a" != "" ]; then turnon0=1; echo will do left-right; fi
turnon1=1
#a=$(grep -i scandir $r.xml | grep -i reverse)
#if [ "$a" != "" ]; then turnon1=1; echo will do right-left; fi

# Turn on the correction of artifacts
turnon=0

if [[ ! $opts =~ left-image-crop-win ]]; then
    echo "Must specify crop-win as input"
    exit 1;
fi
win=$(echo $opts | perl -pi -e 's#^.*?left-image-crop-win\s+(\d+\s+\d+\s+\d+\s+\d+).*?$#$1#g')

#win2="0 0 7168 7168"  # temporary!!!
win2="0 0 50600 21504"
#rm -fv *crop* *proj_crop*
if [ ! -f $l"_crop.tif" ]; then
    gdal_translate.pl -srcwin $win2 $l.ntf $l"_crop.tif"
fi

if [ ! -f $r"_crop.tif" ]; then
    gdal_translate.pl -srcwin $win2 $r.ntf $r"_crop.tif"
fi

#rm -f *ntf

if [ "$turnon" -eq 1 ]; then

    runDir=run_fixed"$tag""$name"_flip1
    hill=$runDir/run-crop-hill.tif
    rm -rfv $runDir
    mkdir -p $runDir
    wv_correct "$l"_crop.tif "$l".xml $runDir/"$l"_crop_shift.tif
    wv_correct "$r"_crop.tif "$r".xml $runDir/"$r"_crop_shift.tif

    runDir=run_fixed"$tag""$name"_flip0
    hill=$runDir/run-crop-hill.tif
    rm -rfv $runDir
    mkdir -p $runDir
    wv_correct "$l"_crop.tif "$l".xml $runDir/"$l"_crop_shift.tif
    wv_correct "$r"_crop.tif "$r".xml $runDir/"$r"_crop_shift.tif

    time_run.sh stereo $runDir/"$l"_crop_shift.tif $runDir/"$r"_crop_shift.tif "$l".xml "$r".xml $runDir/run $opts
    gdal_translate.pl -srcwin $win $runDir/run-F.tif $runDir/run-crop-F.tif
    $execDir/find_avg_disp $runDir/run-crop-F.tif $runDir/dx.txt $runDir/dy.txt
    gdal_translate.pl -srcwin $win $runDir/run-PC.tif $runDir/run-crop-PC.tif
    point2dem -r Earth  $runDir/run-crop-PC.tif  --errorimage
    gdaldem hillshade   $runDir/run-crop-DEM.tif $hill
    gdal_translate.pl -outsize 50% 50% $hill $runDir/run-crop-hill_sub2.tif
    image2qtree.pl $runDir/run-crop-hill_sub2.tif
    gdal_translate.pl -outsize 20% 20% $runDir/run-crop-IntersectionErr.tif $runDir/run-crop-IntersectionErr_20pct.tif
    colormap.pl $runDir/run-crop-IntersectionErr_20pct.tif
    rm -fv $(ls $runDir/*.tif | grep -v crop-F.tif | grep -v Intersect | grep -v hill)

    runDir=run_fixed"$tag""$name"_flip1
    hill=$runDir/run-crop-hill.tif

    time_run.sh stereo $runDir/"$r"_crop_shift.tif $runDir/"$l"_crop_shift.tif "$r".xml "$l".xml $runDir/run $opts
    gdal_translate.pl -srcwin $win $runDir/run-F.tif $runDir/run-crop-F.tif
    $execDir/find_avg_disp $runDir/run-crop-F.tif $runDir/dx.txt $runDir/dy.txt
    gdal_translate.pl -srcwin $win $runDir/run-PC.tif $runDir/run-crop-PC.tif
    point2dem -r Earth  $runDir/run-crop-PC.tif  --errorimage
    gdaldem hillshade   $runDir/run-crop-DEM.tif $hill
    gdal_translate.pl -outsize 50% 50% $hill $runDir/run-crop-hill_sub2.tif
    image2qtree.pl $runDir/run-crop-hill_sub2.tif
    gdal_translate.pl -outsize 20% 20% $runDir/run-crop-IntersectionErr.tif $runDir/run-crop-IntersectionErr_20pct.tif
    colormap.pl $runDir/run-crop-IntersectionErr_20pct.tif
    rm -fv $(ls $runDir/*.tif | grep -v crop-F.tif | grep -v Intersect | grep -v hill)
fi

runDir0=runv"$tag""$name"_flip0
if [ "$turnon0" -eq 1 ]; then
    hill=$runDir0/run-crop-hill.tif
    dx=$runDir0/dx.txt
    dy=$runDir0/dy.txt

    rm -rfv $runDir0
    if [ -f "$dx" ] && [ -f "$dy" ]; then
        echo Will skip $runDir0 as files exist
    else
        time_run.sh stereo "$l"_crop.tif "$r"_crop.tif "$l".xml "$r".xml $runDir0/run $opts
        gdal_translate.pl -srcwin $win $runDir0/run-F.tif $runDir0/run-crop-F.tif
        $execDir/find_avg_disp $runDir0/run-crop-F.tif $dx $dy
        gdal_translate.pl -srcwin $win $runDir0/run-PC.tif $runDir0/run-crop-PC.tif
        point2dem -r Earth  $runDir0/run-crop-PC.tif --errorimage
        gdaldem hillshade   $runDir0/run-crop-DEM.tif $hill
        gdal_translate.pl -outsize 50% 50% $hill $runDir0/run-crop-hill_sub2.tif
        image2qtree.pl $runDir0/run-crop-hill_sub2.tif
        gdal_translate.pl -outsize 20% 20% $runDir0/run-crop-IntersectionErr.tif $runDir0/run-crop-IntersectionErr_20pct.tif
        colormap.pl $runDir0/run-crop-IntersectionErr_20pct.tif
        rm -fv $(ls $runDir0/*.tif | grep -v crop-F.tif | grep -v Intersect | grep -v hill)
    fi
fi

runDir1=runv"$tag""$name"_flip1
if [ "$turnon1" -eq 1 ]; then
    hill=$runDir1/run-crop-hill.tif
    dx=$runDir1/dx.txt
    dy=$runDir1/dy.txt

    rm -rfv $runDir1
    if [ -f "$dx" ] && [ -f "$dy" ]; then
        echo Will skip $runDir1 as files exist
    else
        time_run.sh stereo "$r"_crop.tif "$l"_crop.tif "$r".xml "$l".xml $runDir1/run $opts
        gdal_translate.pl -srcwin $win $runDir1/run-F.tif $runDir1/run-crop-F.tif
        $execDir/find_avg_disp $runDir1/run-crop-F.tif $dx $dy
        gdal_translate.pl -srcwin $win $runDir1/run-PC.tif $runDir1/run-crop-PC.tif
        point2dem -r Earth  $runDir1/run-crop-PC.tif --errorimage
        gdaldem hillshade   $runDir1/run-crop-DEM.tif $hill
        gdal_translate.pl -outsize 50% 50% $hill $runDir1/run-crop-hill_sub2.tif
        image2qtree.pl $runDir1/run-crop-hill_sub2.tif
        gdal_translate.pl -outsize 20% 20% $runDir1/run-crop-IntersectionErr.tif $runDir1/run-crop-IntersectionErr_20pct.tif
        colormap.pl $runDir1/run-crop-IntersectionErr_20pct.tif
        rm -fv $(ls $runDir1/*.tif | grep -v crop-F.tif | grep -v Intersect | grep -v hill)
    fi
fi
