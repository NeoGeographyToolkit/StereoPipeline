.. _image_align:

image_align
------------

The program ``image_align`` aligns a second image to a first image. In
the produced aligned second image, each feature has the same row and
column coordinates as in the first image. It can return a transform
in pixel space and one in planet's coordinate system.

Several alignment transforms are supported, including ``rigid``,
``translation``, ``similarity``,  etc. The alignment transform is determined
with subpixel precision and is applied using bilinear interpolation.

Features are matched among the images using either interest points
or a disparity produced with ASP's correlation algorithms.

If the first image is georeferenced, the second aligned image will use
the same georeference as the first one.  The first image and second
aligned image can then be blended with ``dem_mosaic``
(:numref:`dem_mosaic`).

The images are expected to have a single band and have float or
integer values. If the images have more than one band, only the first
one will be read. The processing is done in double precision. The
default output pixel value value is ``float32``, as casting to integer
may result in precision loss.

Since the first image is kept fixed, if portions of the second aligned
image move higher or more to the left than the upper-left corner of
the first image, those extra portions will be cut. In that case it is
suggested to reverse the order of images when invoking this tool.

The alignment transform can be saved, and a custom alignment transform
can be applied instead of the one found automatically. The interest
point matches which determine the alignment transform can be saved as
well.

This tool extends the co-registration functionality of CASP-GO
(:numref:`casp_go`).

Examples
~~~~~~~~

::
   
    image_align --alignment-transform rigid        \
        image1.tif image2.tif -o image2_align.tif

Using a disparity produced from correlation::

    parallel_stereo --correlator-mode --stereo-algorithm asp_mgm \
      --subpixel-mode 2 image1.tif image2.tif run/run-corr

    image_align image1.tif image2.tif                            \
      --output-image image2_align.tif --output-prefix run/run    \
      --disparity-params "run/run-corr-F.tif 1000000"

Here, the option ``--correlator-mode`` enforces that the disparity is
between the initial images rather than internally aligned versions of
them. Also, no triangulation happens, so ``parallel_stereo`` stops
at computing the filtered disparity ``F.tif``.

Note that ``--subpixel-mode 2`` will be quite slow but produce good
results. See :numref:`running-stereo` for the choices when it comes to
stereo algorithms and subpixel methods, and :numref:`correlator-mode`
for the image correlator functionality.

The disparity will be computed from the first to second image, but the
alignment transform is from the second to first image, so the disparity
and this transform will show opposite trends.

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
      --alignment-transform rigid -o image2_align.tif

Then, the alignment transform can be used to align the second DEM
to the first, as::

    image_align dem1.tif dem2.tif             \
      --input-transform run/run-transform.txt \
      -o dem2_align.tif 

It appears that applying this tool on the DEMs themselves may result
in more accurate results than if applied on their hillshaded images.
(Consider also using for hillshading the tool ``gdaldem hillshade``,
:numref:`gdal_tools`.)

If the DEMs have very different grids and projections, regridding them with
``gdalwarp`` may make them more similar and easier to align (invoke this tool
with cubic spline interpolation).
  
Note that the alignment transform is a 3x3 matrix and can be examined
and edited.  Its inputs and outputs are 2D pixels in *homogeneous
coordinates*, that is, of the form (*x*, *y*, *1*). It is able to model
affine and homography transforms in the pixel plane.

See the related tool ``pc_align`` (:numref:`pc_align`) for alignment
of point clouds. That one is likely to perform better than
``image_align``, as it makes use of the 3D nature of of point clouds,
the inputs need not be gridded, and one of the clouds can be sparse.

.. _image_align_ecef_trans:

Determination of ECEF transform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the images are georeferenced, this program can find the approximate
3D transform around the planet that brings the second image in
alignment with the first one. It is assumed that there exist DEMs
associated with these images, from which the 3D coordinates of the
locations of interest point matches are determined, and the best-fit
3D transform is computed.

Example::

    image_align img1.tif img2.tif \
      -o img2_align.tif           \
      --alignment-transform rigid \
      --ecef-transform-type rigid \
      --dem1 dem1.tif             \
      --dem2 dem2.tif             \
      --output-prefix run/run
 
This will save ``run/run-ecef-transform.txt`` in the ``pc_align``
format (rotation + translation + scale, 
:numref:`alignmenttransform`). This transform can be passed to
``pc_align`` in order to transform a point cloud
(:numref:`prevtrans`), and to ``bundle_adjust`` if desired to
transform cameras (:numref:`ba_pc_align`).

It is important to keep in mind that the ECEF transform is from the second cloud
to the first, hence ``pc_align`` should have the clouds *in the same order* as for
``image_align`` in order to use this transform.

The inverse of this transform is saved as well, if desired to transform the
clouds or cameras from the coordinate system of the first image to the one of
the second image.
 
If no DEMs exist, the images themselves can be used in their
place. The grayscale values will be interpreted as heights above the
datum in meters. The ``image_calc`` program (:numref:`image_calc`)
can modify these values before the DEMs are passed to ``image_align``.

If only DEMs exist, their hillshaded versions (:numref:`hillshade`) can be
used as images. As earlier, the more similar visually the images are, the 
better the results.

It is suggested to use ``--alignment-transform rigid`` and
``--ecef-transform-type rigid`` if it is thought that a rotational component
exists, and the value ``translation`` for these options if no rotation is
expected.

Note that this will produce a rotation + translation around planet
center, rather than a local "in-plane" transform, so it can be hard to
interpret. A similarity transform can be used when there is a difference in
scale.

Note that this transform is an approximation. It is not possible to
precisely convert a 2D transform between images to a 3D transform
in ECEF unless the underlying terrain is perfectly flat.

Usage
~~~~~

::
  
    image_align [options] <reference image> <source image> \
      -o <aligned source image>

Command-line options for image_align
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--output-image, -o <string (default: "")>
    Specify the output image.

--output-prefix <string (default: "")>
    If set, save the interest point matches and computed transform
    (in plain text) using this prefix.

--alignment-transform <string (default: "rigid")>
    Specify the transform to use to align the second image to the
    first. Options: ``translation``, ``rigid`` (translation + rotation),
    ``similarity`` (translation + rotation + scale), ``affine``,
    ``homography``.

--output-data-type, -d <string (default: "float32")>
    The data type of the output file. Options: ``uint8``, ``uint16``,
    ``uint32``, ``int16``, ``int32``, ``float32``, ``float64``. The
    values are clamped (and also rounded for integer types) to avoid
    overflow.

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

--disparity-params <string (default: "")>
    Find the alignment transform by using, instead of interest points,
    a disparity, such as produced by ``parallel_stereo --correlator-mode``. 
    Specify as a string in quotes, in the format: "disparity.tif num_samples".

--input-transform <string (default: "")>    
    Instead of computing an alignment transform, read and apply the one from 
    this file. Must be stored as a 3x3 matrix.

--ecef-transform-type <string (default: "")>
    Save the ECEF transform corresponding to the image alignment
    transform to ``<output prefix>-ecef-transform.txt``. The type can
    be: 'translation', 'rigid' (rotation + translation), or 'similarity'
    (rotation + translation + scale). See :numref:`image_align_ecef_trans`
    for an example.

--dem1 <string (default: "")>
    The DEM associated with the first image. To be used with
    ``--ecef-transform-type``.

--dem2 <string (default: "")>
    The DEM associated with the second image. To be used with
    ``--ecef-transform-type``.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
