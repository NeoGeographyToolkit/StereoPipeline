.. _external_algorithms:

Options for external stereo algorithms
======================================

In addition to stereo correlation algorithms implemented within ASP,
which are documented in :numref:`stereodefault`, ASP ships and
provides access to externally implemented algorithms.

.. _original_mgm_options:

Options for MGM
---------------

This section describes the options and environemntal vaiables accepted by the
``mgm`` program. How it is invoked by ASP is discussed in :numref:`original_mgm`.

Usage::

  mgm <options> left_image.tif right_image.tif output_disparity.tif \
      [cost_function.tif [back_flow.tif]]

Command-line options
^^^^^^^^^^^^^^^^^^^^

-r (default = -30): 
    Minimum horizontal disparity value. (The images are assumed
    to be rectified, which eliminates the vertical disparity.)

-R (default = 30): 
    Maximum horizontal disparity value. 

-O (default = 4):
    Number of search directions. Options: 2, 4, 8, 16. 

-P1 (default = 8)
    SGM regularization parameter P1.

-P2 (default = 32): 
    SGM regularization parameter P2.

-p (default = none): 
    Prefilter algorithm. Options: none, census, sobelx, gblur. The
    ``census`` mode uses the window of dimensions ``CENSUS_NCC_WIN``.

-t (default = ad): 
    Distance function. Options: census, ad, sd, ncc, btad, btsd. For
    ``ncc`` the window of dimensions ``CENSUS_NCC_WIN`` is used. The
    ``bt`` option is the Birchfield-Tomasi distance.

-truncDist (default = inf): 
    Truncate distances at nch * truncDist.

-s (default = none):
    Subpixel refinement algorithm. Options: none, vfit, parabola,
    cubic.

-aP1 (default = 1): 
    Multiplier factor of P1 when sum \|I1 - I2\|^2 < nch * aThresh^2.

-aP2 (default = 1): 
    Multiplier factor of P2 as above.

-aThresh (default = 5):
   Threshold for the multiplier factors.

-m FILE (default = none): 
    A file with minimum input disparity.

-M FILE (default = none):
    A file with maximum input disparity.
 
-l FILE (default = none): 
    Write here the disparity without the left-to-right test.

Environmental variables for mgm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These should be set on the command line before ``mgm`` is invoked.
(ASP does that automatically.)

CENSUS_NCC_WIN=3: 
    Size of the window for the census prefilter algorithm and NCC
    (normalized cross-correlation).

TESTLRRL=1: 
    If 1, do left-to-right and right-to-left consistency checks.

MEDIAN=0:
     Radius of the median filter postprocessing.

TSGM=4:
    Regularity level.

TSGM_ITER=1: 
    Number of iterations.

TSGM_FIX_OVERCOUNT=1: 
    If 1, fix overcounting of the data term in the energy.

TSGM_DEBUG=0:
    If 1, print debug information.

TSGM_2LMIN=0:
    Use the improved TSGM cost only for TSGM=2. Overrides the TSGM
    value.

USE_TRUNCATED_LINEAR_POTENTIALS=0: 
    If 1, use the Felzenszwalb-Huttenlocher truncated linear
    potential. Then P1 and P2 change meaning. The potential they
    describe becomes V(p,q) = min(P2, P1*\|p-q\|).

Options for OpenCV SGBM
-----------------------

The ``parallel_stereo`` program can invoke the OpenCV 
semi-global block-matching algorithm (SGBM) if called with::

    --alignment-method local_homography --stereo-algorithm     \ 
      "opencv_sgbm"

Alternatively, the full string having this algorithm and its 
options can be used, which, with default values, results in the
invocation::

    --alignment-method local_homography --stereo-algorithm     \ 
      "opencv_sgbm -mode hh -block_size 3 -P1 8 -P2 32          
       -prefilter_cap 63 -uniqueness_ratio 10 -speckle_size 100 
       -speckle_range 32 -disp12_diff 1"

Or just options that are desired to be different from the defaults can
be specified, as this::

    --alignment-method local_homography --stereo-algorithm     \ 
      "opencv_sgbm -block_size 7" 

Options:

-mode (default = hh):
    Choose among several flavors of SGBM. Use ``hh`` to run the
    full-scale two-pass dynamic programming algorithm. It will consume
    O(image_width * image_height * num_disparities) bytes. Use the
    ``sgbm`` value for the less-memory intensive mode, and ``3way``
    for yet another flavor which OpenCV does not document.

-block_size (default = 3):
    Block size to use to match blocks from left to right image. It
    must be an odd number >=1. Normally, it should be somewhere in
    the 3 - 11 range.

-P1 (default = 8): 
    Multiplier for the first parameter controlling the disparity
    smoothness. This parameter is used for the case of slanted
    surfaces. This is multiplied by num_image_channels block_size *
    block_size, and ASP uses num_image_channels = 1. It is used as the
    penalty on the disparity change by plus or minus 1 between
    neighbor pixels.

-P2 (default = 32):
    Multiplier for the second parameter controlling the disparity
    smoothness. This is multiplied by num_image_channels block_size *
    block_size, and ASP uses num_image_channels = 1. This parameter is
    used for "solving" the depth discontinuities problem. The larger
    the values are, the smoother the disparity is. This parameter is
    the penalty on the disparity change by more than 1 between
    neighbor pixels. The algorithm requires P2 > P1.

-disp12_diff (default = 1):
    Maximum allowed difference (in integer pixel units) in the
    left-to-right vs right-to-left disparity check. Set it to a
    non-positive value to disable the check.

-prefilter_cap (default = 63):
    Truncation value for the prefiltered image pixels. The algorithm
    first computes the x-derivative at each pixel and clips its value by
    [-prefilter_cap, prefilter_cap] interval. The result values are
    passed to the Birchfield-Tomasi pixel cost function.

-uniqueness_ratio (default = 10):
    Margin in percentage by which the best (minimum) computed cost
    function value should "win" the second best value to consider the
    found match correct. Normally, a value within the 5 - 15 range is
    good enough.

-speckle_size (default = 100):
    Maximum size of smooth disparity regions to consider their noise
    speckles and invalidate. Set it to 0 to disable speckle
    filtering. Otherwise, set it somewhere in the 50 - 200 range.

-speckle_range (default = 32): 
    Maximum disparity variation within each connected component. If
    you do speckle filtering, set the parameter to a positive value,
    it will be implicitly multiplied by 16. Normally, 1 or 2 is good
    enough.

Options for OpenCV BM
---------------------

The simpler and not as performing block-matching (BM) algorithm of
OpenCV can be invoked in a very similar manner, with the algorithm
name passed to ``-stereo-algorithm`` being ``opencv_bm``. It accepts
the same parameters except ``-P1`` and ``-P2``, and uses in addition 
the option:

-texture_thresh (default = 10):
    The disparity is only computed for pixels whose "texture" measure
    is no less than this value. Hence lowering this will result in the
    disparity being computed at more pixels but it may be more
    erroneous.

The full default string of options that is used by
``--stereo-algorithm`` is::

    "opencv_bm -block_size 21 -texture_thresh 10 -prefilter_cap 31 
     -uniqueness_ratio 15 -speckle_size 100 -speckle_range 32 
     -disp12_diff 1"

and any of these can be modified as for the SGBM algorithm. Notice
how the BM algorithm has to use a bigger block size than SGBM.
