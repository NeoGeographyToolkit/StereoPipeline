.. _wv_correct:

wv_correct
----------

An image taken by one of Maxar (DigitalGlobe) World View satellite cameras
is formed of several blocks as tall as the image, mosaicked from left
to right, with each block coming from an individual CCD sensor
:cite:`digital-globe:camera`. Either due to imperfections in the
camera or in the subsequent processing, the image blocks are offset in
respect to each other in both row and column directions by a subpixel
amount. These so-called *CCD boundary artifacts* are not visible in
the images but manifest themselves as discontinuities in the the DEMs
obtained with ASP.

The tool named ``wv_correct`` is able to significantly attenuate these
artifacts (see :numref:`ccd-artifact-example` in the
Digital Globe tutorial for an example). This tool should be used on raw
Digital Globe images before calling ``dg_mosaic`` and ``mapproject``.

It is important to note that both the positions of the CCD offsets and
the offset amounts were determined empirically without knowledge of
Digital Globe's mosaicking process; this is why we are not able to
remove these artifacts completely.

For PAN images, the WV01 and WV02 datasets are supported, for most TDI
for the forward and reverse scan directions. For WV03 PAN images, CCD
artifacts are less noticeable than for WV01 and WV02, and they are not
corrected at this time.

For multispectral images, only a few select TDI are supported for band
3 of WV02 data. If a certain combination of spacecraft/TDI is not
supported, the tool will print a warning and will write on output the
uncorrected input image.

The ASP source code repository has additional documentation and tools for how to
tabulate the corrections for the cases not yet covered by this tool.

Note for WV-2 images
^^^^^^^^^^^^^^^^^^^^

Maxar (DigitalGlobe) WorldView-2 images with a processing (generation) date
(rather than acquisition date) of May 26, 2022 or newer have much-reduced CCD
artifacts, and for those this tool will in fact make the solution worse, not
better. 

ASP of version later than 3.2.0 and any development build after 2023-06-21 will
automatically detect this and will not apply the correction (a copy of the input
image will be written on output). A warning will be printed in that case. This
holds for both PAN and multi-spectral images.

The above-mentioned scenario does not apply to WorldView-1, 3, or GeoEye-1.

Usage
^^^^^

::

    wv_correct [options] <input image> <input camera model> <output image>

Examples
^^^^^^^^

Example for PAN images::

    wv_correct pan.tif pan.xml pan_corr.tif

Example for multispectral images (first extract the third band)::

   gdal_translate -co TILED=YES -co COMPRESS=LZW \
     -co BIGTIFF=IF_SAFER -b 3 ms.tif ms_b3.tif
    wv_correct --band 3 ms_b3.tif ms.xml ms_b3_corr.tif

Example if per-column corrections are available, for either PAN or
multispectral images::

    wv_correct --dx dx.txt --dy dy.txt image.tif image.xml image_corr.tif

Command-line options for wv_correct
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

--ot <string (default: Float32)>
    Output data type. Supported types: Byte, UInt16, Int16, UInt32,
    Int32, Float32. If the output type is a kind of integer, values
    are rounded and then clamped to the limits of that type.

--band <integer (default: 0)>
    For multi-spectral images, specify the band to correct. Required
    unless --dx and --dy are set.

--dx <string (default: "")> 
    For PAN or multi-spectral images, specify the plain text file
    having per-column corrections in the x direction, one per line,
    overriding the pre-computed table.

--dy  <string (default: "")>
    As above, but for the y direction.

--print-per-column-corrections
  Print on standard output the per-column corrections about to apply
  (for multispectral images).

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
