.. _image_align:

image_align
------------

The program ``image_align`` aligns a second image to a first image. In
the produced aligned second image, each feature has the same row and
column coordinates as in the first image.

Several alignment transforms are supported. The alignment transform is
determined with subpixel precision and is applied using bilinear
interpolation.

If the first image is georeferenced, the second aligned image will use
the same georeference as the first one.  The first image and second
aligned image can then be blended with ``dem_mosaic``
(:numref:`dem_mosaic`).

The images are expected to have a single band and have float or
integer values. If the images have more than one band, only the first
one will be read. The processing is done in double precision. The
default output pixel value value is ``float32``, as casting to integer
may result in precision loss.

The alignment transform can be saved, and a custom alignment transform
can be applied instead of the one found automatically. The interest
point matches which determine the alignment transform can be saved as
well.

This tool extends the co-registration functionality of CASP-GO
(:numref:`casp_go`).

Example::

    image_align image1.tif image2.tif -o image2_align.tif

Usage::
  
    image_align [options] <reference image> <source image> \
      -o <aligned source image>

Application for alignment of DEMs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Given a DEM, it can be treated as an image with float values, or an
image can be produced from it with the ``hillshade`` command
(:numref:`hillshade`), or, if the DEM is obtained from a stereo point
cloud with ``point2dem``, an orthoimage in one-to-one correspondence
with this DEM can be created with the ``--orthoimage`` option of this
tool (:numref:`point2dem`).

In either case, given two DEMs, ``dem1.tif`` and ``dem2.tif``, their
corresponding images can be aligned to each other as::

    image_align image1.tif image2.tif --output-prefix run \
      --alignment-method translation -o image2_align.tif

Then, the alignment transform can be used to align the second DEM
to the first, as::

    image_align dem1.tif dem2.tif             \
      --input-transform run/run-transform.txt \
      -o dem2_align.tif 

It appears that applying this tool on the DEMs themselves may result
in more accurate results than if applied on their hillshaded images.

If the DEMs have very different grids and projections, regridding them
with ``gdalwarp``may make them easier to align.
  
Note that the alignment transform is a 3x3 matrix and can be examined
and edited.  Its inputs and outputs are 2D pixels in ``homogeneous
coordinates``, that is, of the form (x, y, 1). It is able to model
affine and homography transforms in the pixel plane.

See the related tool ``pc_align`` (:numref:`pc_align`) for alignment
of point clouds.

Command-line options for image_align
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--output-image, -o <string (default: "")>
    Specify the output image.

--output-prefix <string (default: "")>
    If set, save the interest point matches and computed transform
    (in plain text) using this prefix.

--alignment-transform <string (default: "translation")>
    Specify the transform to use to align the second image to the
    first. Options: ``translation``, ``rigid`` (translation + rotation),
    ``similarity`` (translation + rotation + scale), ``affine``,
    ``homography``.

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
