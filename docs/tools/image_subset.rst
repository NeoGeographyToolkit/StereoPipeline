.. _image_subset:

image_subset
------------

Given a set of overlapping georeferenced images, this program will extract a 
subset that results in almost the same coverage as the original images.

This program is slow, as the algorithm complexity is proportional to the square
of number of input images and the number of output pixels. It is best used with
at most 100-200 images, each of dimensions of 1000 - 2000 pixels.

Overview
~~~~~~~~

A threshold determines if a pixel is considered to add to the coverage.

This program finds the image that contributes the most pixels no less than
the threshold, then the image that contributes the most additional pixels, etc.

The produced subset is saved to an output list, with the image name and number
of contributing pixels on each line. The images are sorted in decreasing order
of pixel contribution.

Background and example
~~~~~~~~~~~~~~~~~~~~~~

This program was developed as an auxiliary tool for Shape-from-shading
processing (:numref:`sfs_usage`). For that, it is desired to have a very large
number of images of diverse illumination to be able to coregister them all.
However, once that is done, just a representative subset of images is needed for
SfS, as using the full set can be prohibitive. 

The following way of invoking this tool is suggested. First, break up the input
image set into several groups, by Sun azimuth (:numref:`sfs_azimuth`), with 50 - 150
images in each group. 

For the images in each group, create a list of mapprojected images at a low
resolution, for example, at 1/16 of original image resolution. This can be
accomplished by using ``sub`` images produced by ``stereo_gui``
(:numref:`stereo_gui`), whose resolution can be further reduced with a command
such as::

  gdalwarp -r cubicspline -tr <gridx> <gridy> input.tif output.tif

Then, for each group, this program can be called as::

  image_subset            \
    --threshold 0.01      \
    --image-list list.txt \
    -o subset.txt 

Note, as before, that this program can take many hours. A progress bar is
displayed and helps track its advancement.

A good threshold can be found by clicking on pixels of representative images in
``stereo_gui`` and observing the pixel values printed in the terminal.

Validation
~~~~~~~~~~

When this program finishes, overlay the images in the produced list and inspect
if they cover the desired area in decreasing order of pixel contribution. 

The program ``dem_mosaic`` (:numref:`dem_mosaic`) with the ``--max`` option
can be helpful in determining if the produced subset coverage is about
the same as for the original images.

Command-line options
~~~~~~~~~~~~~~~~~~~~

--image-list <string (default: "")>
    The list of input images.

--output-list, -o <string (default: "")>
    The file having the produced image subset, and for each image the number of
    contributing pixels (sorted in decreasing order of contribution).
    
--threshold <double (default: NaN)>
    The image threshold. Pixels no less than this will contribute to the coverage.
    
-v, --version
    Display the version of software.

-h, --help
    Display this help message.

