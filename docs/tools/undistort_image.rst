.. _undistort_image:

undistort_image
---------------

The ``undistort_image`` program takes as input an image and a pinhole
model .tsai file describing the image. The tool will generate a copy of
the input image with the lens distortion specified in the pinhole model
file removed. It will also save the corresponding pinhole camera model
file without the distortion.

Usage::

    undistort_image [options] <input image> <camera model> -o <output image>

See an example in :numref:`sfmgeneric`.

Command-line options for undistort_image:

-o, --output-file <filename>
    Specify the output file.

--output-nodata-value <double (default: smallest float value)>
    Set the output nodata value.  Only applicable if the output is
    a single-channel image with pixels that are float or double.

--preserve-pixel-type
    Save the undistorted image with integer pixels if so is the
    input. This may result in reduced accuracy.

--interpolation-method <bilinear|bicubic (default: bilinear)>
    Interpolation method.

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
