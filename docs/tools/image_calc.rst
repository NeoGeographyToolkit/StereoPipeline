.. _image_calc:

image_calc
----------

This tool can be used to perform simple, per-pixel arithmetic on one or
more input images. An arithmetic operation specified on the command line
is parsed and applied to each pixel, then the result is written to disk.
The tool supports multiple input images but each must be the same size
and data type. Input images are restricted to one channel.

The following symbols are allowed in the arithmetic string: +, -, \*, /,
(), min(), max(), pow(), abs(), sign(), and var_N where N is the index of one of
the input images (N\ :math:`\ge`\ 0). The tool also supports certain conditional
operations: lt, gt, lte, gte, eq (<, >, <=, >=, == respectively).  These must be
used in a format like "lt(var_0, 0.003, var_1, 0)", which translates to
"if var_0 < 0.003 then var_1 else 0".
An example arithmetic string is:
"-abs(var_0) + min(58, var_1, var_2) / 2". The tool respects the normal
PEMDAS order of operations *except* that it parses equal priority
operations from right to left, *not* the expected left to right.
Parentheses can be used to enforce any preferred order of evaluation.


Usage::

     image_calc [options] -c <arithmetic formula> <inputs> -o <output>

Example::

     image_calc -c "pow(var_0/3.0, 1.1)" input_image.tif -o output_image.tif -d float32

Command-line options for image_calc:

--help
    Display the help message.

-c, --calc
    The arithmetic string in quotes (required).

-d, --output-data-type <type (default: float64)>
    The data type of the output file. Options: uint8, uint16, uint32,
    int16, int32, float32, float64.

--input-nodata-value <arg>
    Set the nodata value for the input images, overriding the value in
    the images, if present.

--output-nodata-value
    Manually specify a nodata value for the output image (default
    is data type min).

-o, --output-file <name>
    Specify the output file instead of using a default.

--mo <string>
    Write metadata to the output file.  Provide as a string in quotes
    if more than one item, separated by a space, such as
    ``'VAR1=VALUE1 VAR2=VALUE2'``.  Neither the variable names nor the
    values should contain spaces.

--no-georef
   Remove any georeference information (useful with subsequent
   GDAL-based processing).
