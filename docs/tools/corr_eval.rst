.. _corr_eval:

corr_eval
---------

The ``corr_eval`` program takes as input left and right aligned images
and a disparity, as produced by ``parallel_stereo``
(:numref:`stereodefault`), and creates an image of same dimensions as
the left image, having a measure of the quality of the disparity at
each pixel.  

The input disparity can be any of ``D.tif``, ``B.tif``, ``RD.tif``, or
``F.tif``.

The output image has nodata values at pixels it could not compute the
desired metric.

Several quality metrics are supported.

- Normalized cross-correlation (NCC). For each left aligned image
  pixel find the normalized cross-correlation between the patch of
  dimensions given by the kernel size around that pixel and
  corresponding patch in the right aligned image, as determined by the
  disparity at that pixel. A higher value means a more reliable
  disparity.

  Since the disparity is floating-point, bilinear interpolation is
  used in the right image. Pixels with nodata values and out-of-range
  pixels are excluded. Note that NCC is not the zero-normalized
  cross-correlation, so there is no subtraction from each pixel of the
  mean of all pixels in the patch.

- Average of standard deviations of left and right matching patches.
 
Usage::

    corr_eval [options] <L.tif> <R.tif> <Disp.tif> <output prefix>

Example::

    corr_eval --kernel-size 21 21 --metric ncc            \
      run/run-L.tif run/run-R.tif run/run-RD.tif run/run

This will create ``run/run-ncc.tif``.

Command-line options for corr_eval:

--kernel-size <(*integer integer*) (default: 21 21)>
    The dimensions of image patches. These must be positive odd
    numbers.

--metric <(*string*) (default: ncc)>
    The metric to use to evaluate the quality of correlation. Options:
    ``ncc``, ``stddev``.

--prefilter-mode arg <(*integer*) (default: 0)>
    Pre-filter mode. This is the same prefilter as used in stereo
    preprocessing (:numref:`stereodefault`) with the ``asp_bm``
    method. Options: 0 (none), 1 (subtracted mean), 2 (LoG).

--prefilter-kernel-width arg (*float*) (default = 1.5)
    This defines the diameter of the Gaussian convolution kernel used
    for the pre-filtering modes 1 and 2 above. A value of 1.5 works
    well for LoG and 25 - 30 works well for the subtracted mean.

-h, --help
    Display the help message.

