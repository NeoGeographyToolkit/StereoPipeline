.. _image_mosaic:

image_mosaic
------------

The program ``image_mosaic`` aligns multiple input images into a single output
image. Currently it only supports a horizontal sequence of images such as
scanned Corona images (:numref:`kh4`).

It is expected that the second input image is a continuation on the right of the
first image, and so on. Otherwise, this program must be called with the images
in reverse order, or by setting the ``--reverse`` option.

Example
~~~~~~~

::

     image_mosaic input1.tif input2.tif \
       --ot Float32 --blend-radius 2000 \
       --overlap-width 5000             \
       -o output.tif

More examples are in :numref:`kh4`, :numref:`kh7`, and :numref:`kh9`.
       
Handling failure
~~~~~~~~~~~~~~~~

In case of failure, inspect the input images. This tool assumes the second
image can be appended to the right of the first image. It expects no rotation
between the images.

This program can fail if not enough interest points are found to align the
images. It will try a couple of attempts with a larger value of
``--ip-per-tile`` before giving up.

Try using an even larger value of this parameter than what the program attempted
and printed on the screen.

Also consider adjusting ``--inlier-threshold`` and ``--num-ransac-iterations``
if the produced transform is not accurate. A lower inlier threshold will result
in a more accurate transform but a higher chance of failure.

Usage
~~~~~

::

     image_mosaic [options] <images> -o output_file_path

Command-line options
~~~~~~~~~~~~~~~~~~~~

--t_orientation <horizontal>
    Specify the image layout.  Currently only supports horizontal.

--reverse
    Mosaic the images in reverse order.

--rotate
    After mosaicking, rotate the image by 180 degrees around its
    center.

--rotate-90
    After mosaicking, rotate the image by 90 degrees clockwise
    around its center.

--rotate-90-ccw
    After mosaicking, rotate the image by 90 degrees counter-clockwise
    around its center.

--use-affine-transform
    Solve for full affine transforms between segments instead of a
    simpler rotate+translate transform.

-o, --output-image <string>
    Specify the output file path. Required.

--overlap-width <number-of-pixels (default: 2000)>
    The width of the expected overlap region in the images, in
    pixels.

--blend-radius <number-of-pixels>
    The width in pixels over which blending is performed. Default
    is calculated based on the overlap width.

--band <integer (default: 1)>
    Specify a band (channel) to use for multi-channel images. The band count
    starts from 1.

--ot <type (default: Float32)>
    Output data type. Supported types: Byte, UInt16, Int16, UInt32,
    Int32, Float32. If the output type is a kind of integer, values
    are rounded and then clamped to the limits of that type.

--input-nodata-value <double>
    Override the input nodata value.

--output-nodata-value double
    Specify the output nodata value.

--ip-per-tile integer
    How many interest points to detect in each :math:`1024^2` image
    tile (default: automatic determination).

--output-prefix <string>
    If specified, save here the interest point matches used in
    mosaicking.

--num-ransac-iterations <integer (default: 1000)>
    How many iterations to perform in RANSAC when finding interest point 
    matches.

--inlier-threshold <integer (default: 10)>
    The inlier threshold (in pixels) to separate inliers from outliers when
    computing interest point matches. A smaller threshold will result in fewer
    inliers.

--save-matches-as-txt
    Save match files as plain text instead of binary. See :numref:`txt_match`.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

--cog
    Write a cloud-optimized GeoTIFF (COG). See :numref:`cog_output`.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
