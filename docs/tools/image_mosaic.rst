.. _image_mosaic:

image_mosaic
------------

The program ``image_mosaic`` aligns multiple input images into a single
output image. Currently it only supports a horizontal sequence of images
such as scanned Corona images. An example of using this tool is in
:numref:`kh4`.

Usage::

     image_mosaic [options] <images> -o output_file_path

Command-line options for image_mosaic:

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

--band <integer (default: 0)>
    Specify a band to use for multi-channel images.

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

--tile-size <integer(=256, 256)>
    The size of image tiles used for processing. The amount of image
    blending is limited by the tile size, so this will be increased
    automatically if it is too small for the overlap width.

-h, --help
    Display the help message.
