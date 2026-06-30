.. _corr_eval:

corr_eval
---------

The ``corr_eval`` program takes as input left and right aligned images and a
disparity, as produced by :ref:`parallel_stereo`, and creates an image of the
same dimensions as the left image, with a measure of the quality of the
disparity at each pixel.

The input disparity can be any of ``D.tif``, ``B.tif``, ``RD.tif``, or ``F.tif``
(:numref:`outputfiles`). It is suggested to use the most refined disparity,
namely ``F.tif``. This is particularly important for the localization uncertainty
metrics (:numref:`corr_eval_uncertainty`).

The output image has no-data values at pixels where the metric could not be
computed.

See also the related image correlator, which can find the disparity of two images
without camera information (:numref:`correlator-mode`).

Examples
~~~~~~~~

Evaluate the quality of the disparity produced with the ``asp_bm`` block-matching
(normalized cross-correlation) algorithm (:numref:`stereo_alg_overview`)::

    corr_eval --prefilter-mode 2  \
      --kernel-size 21 21         \
      --metric ncc                \
      run/run-L.tif run/run-R.tif \
      run/run-F.tif               \
      run/run

This writes ``run/run-ncc.tif``. A higher value in this image is a more reliable
match (the left and right patches agree well) and a lower value a poorer one.

The options above should be in agreement with how the disparity in ``F.tif`` was
produced during stereo correlation (:numref:`corr_section`). Hence here the
prefilter mode is set to 2, to apply the LoG prefilter for consistency. See
:numref:`corr_eval_prefilter` for details.

Evaluate the quality of the disparity created with the ``asp_mgm``
algorithm (:numref:`stereo_alg_overview`)::

    corr_eval --prefilter-mode 0  \
      --kernel-size 5 5           \
      --metric ncc                \
      run/run-L.tif run/run-R.tif \
      run/run-F.tif               \
      run/run

Here ``--prefilter-mode 0`` (no prefilter) is used, because ``asp_mgm`` does not
apply the LoG prefilter. It uses its own cost function instead. The prefilter
should not be applied in the evaluation if it was not applied when producing the
disparity.

A different kind of metric is the per-pixel localization uncertainty. This
example computes the ``cramer_rao`` metric, in pixels, from the curvature of the
correlation peak::

    corr_eval --prefilter-mode 2  \
      --kernel-size 21 21         \
      --metric cramer_rao         \
      run/run-L.tif run/run-R.tif \
      run/run-F.tif               \
      run/run

This writes ``run/run-cramer_rao.tif``. A small value in this image is a sharp,
well-localized match and a large value a flat, low-texture one. The metrics are
defined in :numref:`corr_eval_uncertainty`.

Metrics
~~~~~~~

The metric is selected with ``--metric``. The following are supported.

- ``ncc``: Normalized cross-correlation. For each left pixel, the NCC between the
  patch of dimensions given by ``--kernel-size`` around that pixel and the
  corresponding patch in the right image, as determined by the disparity. The
  value is between -1 and 1. A high value (close to 1) means the two patches agree
  well, hence a reliable disparity, while a low value means a poor match. Bilinear
  interpolation is used in the right image, unless ``--round-to-int`` is set.
  No-data and out-of-range pixels are excluded. This is not the zero-normalized
  cross-correlation (there is no subtraction of the patch mean).

- ``stddev``: The average of the standard deviations of the left and right
  matching patches.

- ``parabola_curvature`` and ``cramer_rao``: a per-pixel localization uncertainty.
  See :numref:`corr_eval_uncertainty`. For these, unlike for ``ncc``, a *small*
  value is the good one (a well-localized match) and a large value is poor.

.. _corr_eval_uncertainty:

Localization uncertainty
^^^^^^^^^^^^^^^^^^^^^^^^

The ``parabola_curvature`` and ``cramer_rao`` metrics produce a per-pixel
localization uncertainty ``sigma``, in pixels, from the curvature of the
correlation peak. These are the dense version of the two estimators in the
``--metric`` option of :ref:`sparse_disp`
(:numref:`sparse_disp_uncertainty`).

- ``parabola_curvature``: ``sigma = 1/sqrt(k)``, where ``k`` is the curvature of
  the NCC peak, found by evaluating the NCC one pixel off the disparity in each
  direction (``k = 2 C - C(+1) - C(-1)`` per axis, with ``C`` the peak value). A
  sharp peak gives a small ``sigma``, a flat (low-texture) peak a large one. This
  uses the peak geometry only.

- ``cramer_rao``: ``sigma = sqrt((1-C)/k)``. This additionally downweights
  low-correlation matches via the residual ``1-C``, so a match that is locally
  sharp but agrees poorly between the images still gets a larger ``sigma``.

Both are uncalibrated proxies, useful for ranking reliability rather than as
absolute error bars. Two cautions:

- The curvature is measured by sampling the correlation one pixel to each side of
  the given disparity, which assumes that disparity sits at the peak. So a
  sub-pixel disparity is needed (``RD.tif`` or ``F.tif``, not the integer
  ``D.tif``). An integer, constant, or otherwise approximate disparity lands off
  the peak, especially in textured areas, and corrupts the curvature.

- These metrics require ``--prefilter-mode 2`` (the LoG band-pass), regardless of
  the stereo algorithm, so the curvature reflects texture and not low-frequency
  content. This also matches :ref:`sparse_disp`, which always applies this filter
  internally. It is essential for cross-modal images (for example lidar intensity
  versus optical imagery), where otherwise the radiometric difference dominates
  (:numref:`corr_eval_prefilter`).

The same estimators applied to sparse matches are in
:numref:`sparse_disp_uncertainty`.

.. _corr_eval_prefilter:

Prefilter mode
~~~~~~~~~~~~~~

Before the metric is computed, the input images can be prefiltered, with
``--prefilter-mode`` and ``--prefilter-kernel-width``. This is the same prefilter
as in stereo correlation with the ``asp_bm`` algorithm
(:numref:`stereodefault`). The modes are:

- ``0``: no prefilter.
- ``1``: subtracted mean. A kernel width of 25 - 30 is suggested.
- ``2``: Laplacian of Gaussian (LoG), with a kernel width of about 1.5. This is a
  band-pass filter that removes the low-frequency illumination and radiometric
  differences and keeps the texture.

The prefilter matters, and should match how the disparity was produced (see the
examples above). For ``asp_bm``, which itself uses the LoG prefilter, use
``--prefilter-mode 2``. For ``asp_mgm`` or another algorithm, which use their own
cost function rather than this prefilter (:numref:`stereo_algos_full`), a smaller
kernel and no prefilter can be more appropriate.

For cross-modal images, ``--prefilter-mode 2`` is important regardless of the
algorithm, so the correlation is driven by shared texture and not by the
intensity difference between the two modalities. In general ``--prefilter-mode 2``
tends to produce better results. It is worth experimenting with ``--kernel-size``
as well.

Command-line options
~~~~~~~~~~~~~~~~~~~~

--kernel-size <integer integer (default: 21 21)>
    The dimensions of image patches. These must be positive odd
    numbers.

--metric <string (default: ncc)>
    The metric to use to evaluate the quality of correlation. Options:
    ``ncc`` (the correlation peak value), ``stddev``, and ``parabola_curvature``
    and ``cramer_rao`` (the per-pixel localization uncertainty ``sigma`` from the
    peak curvature, the same two estimators as the ``--metric`` option
    of :ref:`sparse_disp`).

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
