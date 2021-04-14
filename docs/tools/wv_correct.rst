.. _wv_correct:

wv_correct
----------

An image taken by one of Digital Globe’s World View satellite cameras is
formed of several blocks as tall as the image, mosaicked from left to
right, with each block coming from an individual CCD sensor
:cite:`digital-globe:camera`. Either due to imperfections in
the camera or in the subsequent processing the image blocks are offset
in respect to each other in both row and column directions by a subpixel
amount. These so-called *CCD boundary artifacts* are not visible in the
images but manifest themselves as discontinuities in the the DEMs
obtained with ASP.

The tool named ``wv_correct`` is able to significantly attenuate these
artifacts (see :numref:`ccd-artifact-example` in the
Digital Globe tutorial for an example). This tool should be used on raw
Digital Globe images before calling ``dg_mosaic`` and ``mapproject``.

It is important to note that both the positions of the CCD offsets and
the offset amounts were determined empirically without knowledge of
Digital Globe’s mosaicking process; this is why we are not able to
remove these artifacts completely.

For PAN images, the WV01 and WV02 datasets are supported, for most TDI
for the forward and reverse scan directions (if a certain combination
is not supported, the tool will print a warning and will write on
output the uncorrected input image).

For WV03 PAN images, CCD artifacts are less noticeable than for 
WV01 and WV02, and they are not corrected at this time.

Usage::

    wv_correct [options] <input image> <input camera model> <output image>

Command-line options for wv_correct:

--ot <string (default: Float32)>
    Output data type. Supported types: Byte, UInt16, Int16, UInt32,
    Int32, Float32. If the output type is a kind of integer, values
    are rounded and then clamped to the limits of that type.

-h, --help
    Display the help message.

--threads <integer (default: 0)>
    Set the number threads to use. 0 means use the default defined
    in the program or in the .vwrc file.
