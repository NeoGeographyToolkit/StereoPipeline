.. _pansharp:

pansharp
--------

This tool reads in a high resolution grayscale file and a low resolution
RGB file and produces a high resolution RGB file. The output image will
be at the resolution of the grayscale image and will cover the region
where the two images overlap. Both images must have georeferencing
information. This can either be projection information in the image
metadata or it can be a separate Worldview format XML camera file
containing four ground control points (if using the tool with Digital
Globe images).

Usage::

    pansharp [options] <grayscale image file> <color image file> <output image file>

Command-line options for pansharp:

--min-value
    Manually specify the bottom of the input data range.

--max-value
    Manually specify the top of the input data range.

--gray-xml
    Look for georeference data here if not present in the grayscale image.

--color-xml
    Look for georeference data here if not present in the RGB image.

--nodata-value
    The nodata value to use for the output RGB file.

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
