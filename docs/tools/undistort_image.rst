.. _undistort_image:

undistort_image
---------------

The ``undistort_image`` program takes as input an image and a pinhole
model .tsai file describing the image. The tool will generate a copy of
the input image with the lens distortion specified in the pinhole model
file removed. It will also save the corresponding pinhole camera model
file without the distortion.

Usage::

     > undistort_image [options] <input image> <camera model> -o <output image>

Command-line options for undistort_image:

-h, --help
    Display the help message.

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
