.. _image_calc:

image_calc
----------

This tool can be used to perform simple, per-pixel arithmetic on one or
more input images. An arithmetic operation specified on the command line
is parsed and applied to each pixel, then the result is written to disk.
The tool supports multiple input images but each must be the same size
and data type. Input images are restricted to one channel (band). For images with
more than one channel, only the first channel will be read.

The following symbols are allowed in the arithmetic string: +, -, \*, /,
(), min(), max(), pow(), abs(), sign(), and var_N, where N is the index of one of
the input images (N\ :math:`\ge`\ 0). The tool also supports certain conditional
operations: lt, gt, lte, gte, eq (<, >, <=, >=, == respectively).  These must be
used in a format like "lt(var_0, 0.003, var_1, 0)", which translates to
"if var_0 < 0.003 then var_1 else 0".

An example arithmetic string to be passed via ``-c`` is::

    -abs(var_0) + min(58, var_1, var_2) / 2

The tool respects the normal PEMDAS order of operations *except* that
it parses equal priority operations from right to left, *not* the
expected left to right.  Parentheses can be used to enforce any
preferred order of evaluation.

Examples
~~~~~~~~

Apply operation and save pixels as float32
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

     image_calc -c "pow(var_0/3.0, 1.1)" input_image.tif \
      -o output_image.tif -d float32

Invalidate values no more than a threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    thresh=5.2
    image_calc -c "max($thresh, var_0)" -d float32 \
        --output-nodata-value $thresh              \
        input.tif -o output.tif


Apply a mask to an image
^^^^^^^^^^^^^^^^^^^^^^^^

::

    image_calc -c "var_0 * var_1" -d float32 \
        --output-nodata-value 0              \
        input.tif mask.tif -o output.tif

Here it is assumed that the image and the mask have the same
dimensions, the mask has value 1 for pixels to keep and 0 for pixels
to discard, and that the output pixels with value 0 are invalid.

Create a mask
^^^^^^^^^^^^^

::

    image_calc -c "sign(max(var_0, 0))" -d float32 \
        input.tif -o output.tif

Positive values will become 1, and the rest will become 0. 

Add a value to the geoheader metadata
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

     image_calc -c "var_0" --mo 'VAR1=VAL1' -d float32 \
       input.tif -o output.tif

If this variable already exists, its value will be overwritten. Other
existing variables will be preserved. Use ``gdalinfo`` to view the
metadata.

Subtract 360 degrees from the longitudes in a GeoTiff file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    image_calc -c "var_0" input.tif -o output.tif \
      --longitude-offset -360 -d float32 


Usage
~~~~~

::

     image_calc [options] -c <arithmetic formula> <inputs> -o <output>

Command-line options
~~~~~~~~~~~~~~~~~~~~

-c, --calc <string>
    The arithmetic string in quotes (required).

-d, --output-data-type <type (default: float64)>
    The data type of the output file. Options: uint8, uint16, uint32,
    int16, int32, float32, float64.

--input-nodata-value <double>
    Set the nodata value for the input images, overriding the value in
    the images, if present.

--output-nodata-value <double>
    Manually specify a nodata value for the output image. By default
    it is read from the first input which has it, or, if missing, it
    is set to data type min.

-o, --output-file <string>
    Specify the output file instead of using a default.

--mo <string>
    Write metadata to the output file.  Provide as a string in quotes
    if more than one item, separated by a space, such as
    ``'VAR1=VALUE1 VAR2=VALUE2'``.  Neither the variable names nor the
    values should contain spaces.

--longitude-offset <double (default: not specified)>
    Add this value to the longitudes in the geoheader (can be used to
    offset the longitudes by 360 degrees).

--no-georef
   Remove any georeference information (useful with subsequent
   GDAL-based processing).

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
