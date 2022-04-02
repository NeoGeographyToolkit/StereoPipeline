.. _image_align:

image_align
------------

The program ``image_align`` aligns a second image to a first image. In
the produced aligned second image, each feature has the same row and
column coordinates as in the first image.

Several alignment transforms are supported. The alignment transform is
determined with subpixel precision. Bilinear interpolation is used
to apply the alignment transform.

If the first image is georeferenced, the second aligned image will use
the same georeference as the first one.  The first image and second
aligned image can then be blended with ``dem_mosaic``
(:numref:`dem_mosaic`).

The images are expected to have a single band. If having more than
one, only the first band will be read. The pixel value types can be
integer or floating point (see below for details). The processing is
done in double precision. The default output pixel value value is
float32, as casting to int may result in precision loss.

Optionally, the alignment transform can be saved, or a custom 
alignment transform can be applied instead of the one found
automatically. The interest point matches which determine the
alignment transform can be saved as well.

The work on this tool was funded by NASA proposal 19-PDART19_2-0094,
and was motivated by the CASP-GO software
(https://github.com/mssl-imaging/CASP-GO,
:cite:`sidiropoulos2018automatic`).

See the related tool ``pc_align`` (:numref:`pc_align`) for alignment
of point clouds.

Example::

    image_align image1.tif image2.tif -o image2_align.tif

Usage::
  
    image_align [options] <reference image> <source image> \
      -o <aligned source image>

Command-line options for image_align:

--output-image, -o <string (default: "")>
    Specify the output image.

--alignment-transform <string (default: "translation")>
    Specify the transform to use to align the second image to the
    first. Options: ``translation``, ``rigid`` (translation + rotation),
    ``similarity`` (translation + rotation + scale), ``affine``,
    ``homography``.

--output-prefix <string (default: "")>
    If set, save the interest point matches and computed transform
    using this prefix.

--output-data-type, -d <string (default: "float32")>
    The data type of the output file. Options: ``uint8``, ``uint16``,
    ``uint32``, ``int16``, ``int32``, ``float32``, ``float64``. The
    values are carefully clamped with integer types to avoid overflow.

--ip-per-image <integer (default: 0)>
    How many interest points to detect in each image (default: automatic 
    determination).

--num-ransac-iterations <integer (default: 1000)>
    How many iterations to perform in RANSAC when finding interest point 
    matches.

--inlier-threshold <integer (default: 5)>    
    The inlier threshold (in pixels) to separate inliers from outliers when 
    computing interest point matches. A smaller threshold will result in fewer 
    inliers.

--input-transform <string (default: "")>    
    Instead of computing an alignment transform, read and apply the one from 
    this file. Must be stored as a 3x3 matrix.

--threads <integer (default: 0)>
    Set the number of threads to use. Zero means use as many threads
    as there are cores, unless an explicit value is specified in
    ``.vwrc.``

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--version, -v
    Display the version of software.

--help, -h
    Display this help message.
