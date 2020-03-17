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

--help  
    Display the help message.

-o, --output-file <filepath>
    Specify the output file. Required!
