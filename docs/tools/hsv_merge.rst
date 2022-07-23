.. _hsv_merge:

hsv_merge
---------

Replaces the intensity information in an RGB image with the provided
grayscale image by temporarily converting to HSV. Both input image must
be the same size.

Mimics ``hsv_merge.py`` by Frank Warmerdam and Trent Hare. Use it to
combine results from gdaldem.

Usage::

     hsv_merge [options] <rgb_image> <gray_image>

Command-line options for hsv_merge:

-o, --output-file <filepath>
    Specify the output file. Required.

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
