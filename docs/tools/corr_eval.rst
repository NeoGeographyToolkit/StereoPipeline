.. _corr_eval:

corr_eval
---------

The ``corr_eval`` program takes as input left and right aligned images
and a disparity, as produced by ``parallel_stereo``
(:numref:`parallel_stereo`), and creates an image of same dimensions as
the left image, having a measure of the quality of the disparity at
each pixel.  

The input disparity can be any of ``D.tif``, ``B.tif``, ``RD.tif``, or
``F.tif``. (:numref:`outputfiles` describes these.)

Two quality metrics are supported, for the moment.

- Normalized cross-correlation (NCC). For each left aligned image
  pixel find the normalized cross-correlation between the patch of
  dimensions given by the kernel size around that pixel and
  corresponding patch in the right aligned image, as determined by the
  disparity at that pixel. A higher value means a more reliable
  disparity.

  Since the disparity is floating-point, bilinear interpolation is
  used in the right image (unless the flag ``--round-to-int`` is
  set, when the disparity is rounded to int and no interpolation
  happens). Pixels with no-data values and out-of-range pixels are
  excluded from the calculation. Note that NCC is not
  the zero-normalized cross-correlation, so there is no subtraction
  from each pixel of the mean of all pixels in the patch.

- Average of standard deviations of left and right matching patches.
 
The output image has no-data values at pixels where it could not
compute the desired metric.

Usage::

    corr_eval [options] <L.tif> <R.tif> <Disp.tif> <output prefix>

Example for a disparity created with ``--stereo-algorithm asp_bm``::

    corr_eval --prefilter-mode 2 --kernel-size 21 21 --metric ncc \
      run/run-L.tif run/run-R.tif run/run-RD.tif run/run

Example for a disparity created with the ``asp_mgm`` algorithm::

    corr_eval --prefilter-mode 0 --kernel-size 5 5 --metric ncc \
      run/run-L.tif run/run-R.tif run/run-RD.tif run/run

This will create ``run/run-ncc.tif``.

It appears that always using ``--prefilter-mode 2`` makes this tool
produce better results. This is to be investigated further. Consider
also experimenting with ``--kernel-size``.

The external stereo algorithms (:numref:`stereo_algos_full`) do not
use the prefilter mode, and each of them specifies the correlation
kernel size in its own way.

See also the somewhat-related image correlator tool which can find the
disparity of two images without assuming camera information
(:numref:`correlator-mode`).

Command-line options for ``corr_eval``:

--kernel-size <integer integer (default: 21 21)>
    The dimensions of image patches. These must be positive odd
    numbers.

--metric <string (default: ncc)>
    The metric to use to evaluate the quality of correlation. Options:
    ``ncc``, ``stddev``.

--prefilter-mode <integer (default: 0)>
    Prefilter mode. This is the same prefilter as in stereo
    correlation (:numref:`stereodefault`) with the ``asp_bm``
    method. Options: 0 (none), 1 (subtracted mean), 2 (LoG).

--prefilter-kernel-width <float (default: 1.5)>
    The diameter of the Gaussian convolution kernel for prefilter
    modes 1 and 2. A value of 1.5 works well for ``LoG``, and 25 - 30 is 
    suggested for ``subtracted mean``.

--sample-rate <integer (default: 1)>
    Compute the quality image only at one out of this many rows and
    columns, for speed. The output image size does not change. To shrink
    it (say by 2x), run ``gdal_translate -r average -outsize 50% 50% in.tif out.tif``.

--round-to-int
    Round the disparity to integer and skip interpolation when finding
    the right image patches. This make the program faster by a factor
    of about 2, without changing significantly the output image.

--threads <integer (default: 0)>  
    Set the number of threads to use. By default use the number of
    threads as given in .vwrc, which can be 8 or 16. (The actual
    number will be printed when this program starts.) 

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
