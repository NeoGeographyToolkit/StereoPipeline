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

- Normalized cross-correlation. For each left image pixel find the
  normalized cross-correlation between the window of dimensions given by
  the kernel size around that pixel and corresponding window in the
  right image as determined by the disparity at that pixel. A higher
  value means a more reliable disparity. 

  Since the disparity is floating-point, bilinear interpolation is
  used in the right image. Pixels with nodata values and out-of-range
  pixels are excluded. Note that NCC is not the zero-normalized
  cross-correlation, so there is no subtraction from each pixel of the
  mean of all pixels in the window.

Usage::

    corr_eval [options] <L.tif> <R.tif> <Disp.tif> <output.tif>

Example::

    corr_eval --kernel-size 21 21 run/run-L.tif run/run-R.tif \
      run/run-RD.tif run/run-RD-ncc.tif

Command-line options for corr_eval:

--kernel-size <(*integer integer*) (default: 21 21)>
    The dimensions of image patches. These must be positive odd
    numbers.

-h, --help
    Display the help message.

