.. _colormap:

colormap
--------

The ``colormap`` tool reads a DEM or some other single-channel image,
and writes a corresponding color-coded image that can be used
for visualization.

Usage::

    colormap [options] <input DEM>

Example::

    colormap --min -5 --max 10 image.tif

This will produce ``image_CMAP.tif``, with the "hottest" color
corresponding to pixel values at least 10, and the "coolest" color
representing pixel values less than or equal to -5.

See :numref:`visualising` for a discussion of ASP's visualization
tools, including this one.

Command-line options for ``colormap``:

-s, --shaded-relief-file <filename>
    Specify a shaded relief image (grayscale) to apply to the
    colorized image.

-o, --output-file <filename>
    Specify the output file.

--colormap-style <arg>
    Specify the colormap style.  Options: binary-red-blue (default),
    jet, or the name of a file having the colormap, similar to the
    file used by gdaldem.

--nodata-value <arg>
    Remap the DEM default value to the min altitude value.

--min <arg>
    Minimum height of the color map.

--max <arg>
    Maximum height of the color map.

--moon
    Set the min and max height to good values for the Moon.

--mars
    Set the min and max height to good values for Mars.

--legend
    Generate an unlabeled legend, will be saved as ``legend.png``.

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
