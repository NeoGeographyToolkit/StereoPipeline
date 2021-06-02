.. _external_algorithms:

Options for external stereo algorithms
======================================

In addition to stereo correlation algorithms implemented within ASP,
which are documented in :numref:`stereodefault`, ASP ships and
provides access to externally implemented algorithms.

.. _original_mgm_options:

Options for MGM
---------------

This section describes the full list of options accepted by the
``mgm`` program. How it is invoked by ASP
is discussed in :numref:`original_mgm`.

Usage::

  mgm <options> left_image.tif right_image.tif output_disparity.tif \
      [cost_function.tif [back_flow.tif]]

-r (default=-30): 
    Minimum horizontal disparity value. (The images are assumed
    to be rectified, which eliminates the vertical disparity.)

-R (default=30): 
    Maximum horizontal disparity value. 

-O (default=4):
    Number of search directions. Options: 2, 4, 8, 16. 

-P1 (default=8)
    SGM regularization parameter P1.

-P2 (default=32): 
    SGM regularization parameter P2.

-p (default=none): 
    Prefilter algorithm. Options: none, census, sobelx, gblur. The
    ``census`` mode uses the window of dimensions ``CENSUS_NCC_WIN``.

-t (default=ad): 
    Distance function. Options: census, ad, sd, ncc, btad, btsd. For
    ``ncc`` the window of dimensions ``CENSUS_NCC_WIN`` is used. The
    ``bt`` option is the Birchfield-Tomasi distance.

-truncDist (default=inf): 
    Truncate distances at nch * truncDist.

-s (default=none):
    Subpixel refinement algorithm. Options: none, vfit, parabola,
    cubic.

-aP1 (default=1): 
    Multiplier factor of P1 when sum \|I1 - I2\|^2 < nch * aThresh^2.

-aP2 (default=1): 
    Multiplier factor of P2 as above.

-aThresh (default=5):
   Threshold for the multiplier factors.

-m FILE (default=none): 
    A file with minimum input disparity.

-M FILE (default=none):
    A file with maximum input disparity.
 
-l FILE (default=none): 
    Write here the disparity without the left-to-right test.

Environmental variables
-----------------------

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
