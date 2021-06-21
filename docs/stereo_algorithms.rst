.. _stereo_algos_full:

The stereo algorithms in ASP in detail
======================================

Here we will discuss in a lot of detail ASP's stereo algorithms. For a
brief summary see :numref:`stereo_algos`. For how to add new such
algorithms, see :numref:`adding_algos`.

.. _asp_bm:

Block-matching
--------------

Block-matching is ASP's oldest and default algorithm. It can be
invoked with both ``stereo`` and ``parallel_stereo`` with the option::

    --stereo-algorithm asp_bm

and works with any alignment method (:numref:`image_alignment`).

For each pixel in the left image, the algorithm matches a small block
around this pixel with another similar block in the right image. The
block size is given by the value of ``--corr-kernel``. The obtained
correspondence is then refined based on the value of
``--subpixel-mode``, using a block given by ``--subpixel-kernel``. The
option ``--corr-timeout`` can be used to ensure long-running block
matching operations are stopped after a given time.

The related block-matching algorithm in OpenCV that ASP can invoke is
discussed in :numref:`opencv_bm`.

.. _asp_sgm:

Semi-Global Matching and More Global Matching algorithms
--------------------------------------------------------

ASP implements the popular Semi-Global Matching (SGM) algorithm
introduced in :cite:`hirschmuller_sgm_original`, and the More Global
Matching (MGM) algorithm :cite:`facciolo2015mgm`, which is a
modification of SGM, and usually produces higher quality
results. These should be invoked with ``parallel_stereo``, with the
option ``--stereo-algorithm`` being passed the value ``asp_sgm`` and
``asp_mgm``, respectively.

It is suggested to use these algorithms with ``--alignment-method
local_epipolar``, when piecewise alignment between left and right
images is computed which results in the disparity being 1D and faster
to find (:numref:`image_alignment`).

However, the versions of SGM and MGM implemented by ASP can
perform a full 2D disparity search, similar to what is done in the
NG-fSGM algorithm :cite:`xiang_2016_low_complexity_fsgm`. Since ASP
processes a wide variety of cameras with varying degrees of metadata
quality, the standard assumption with SGM that the disparity search
can be performed only along a one-dimensional epipolar line does not
hold when the alignment method is not ``local_epipolar`` or for
map-projected images.

The other major change is that ASP's implementation uses a
multi-resolution hierarchical search combined with a compressed memory
scheme similar to what is used in the SGM algorithm
:cite:`rothermel2012sure_isgm`.

The MGM algorithm reduces the amount of high frequency artifacts in
textureless regions at the cost of a longer run time. ASP also offers
the option of a hybrid SGM/MGM mode (``--stereo-algorithm final_mgm``)
where MGM is used only for the final resolution level which obtains
results somewhere between the pure SGM and MGM options.

The greatest advantage of the SGM algorithm over the ASP block-matching
algorithm is an improved ability to find disparity matches
in areas of repetitive or low texture. SGM can also discern finer
resolution features than the standard correlation algorithm since it
tends to use much smaller matching kernels. Along with these advantages
come several disadvantages. First, SGM is computationally expensive and
requires a lot of memory. Second, in some situations it can produce
noticeable artifacts at tile boundaries. Third, it can sometimes produce
inaccurate results in textureless regions. With careful parameter
selection and usage these disadvantages can be mitigated.

MGM is currently limited to using 8 simultaneous threads but SGM does
not have a limit.

It is suggested to use these algorithms with default options. If desired,
customizations can be done as follows.

-  The ``sgm-collar-size`` option can be increased from the default to allow
   for more padding for each tile. This decreases the chances of seeing
   artifacts along tile borders but increases processing time and memory
   usage. With the tiles being bigger, local alignment may be less
   effective.

-  Set the ``corr-tile-size`` option to determine the tile size (before
   the padding is applied), keeping in mind that larger tile sizes
   produce better results but consume more memory.

-  Set the ``processes`` option keeping in mind memory constraints as
   discussed earlier. Each process will run one simultaneous SGM
   instance and consume memory.

-  The ``corr-memory-limit-mb`` parameter limits the number of megabytes
   of memory that can be used by SGM/MGM. This limit is per-process. To
   be safe, make sure that you have more RAM available than the value of
   this parameter multiplied by the number of processes.

-  ``job-size-w`` and ``job-size-h`` are set equal to
   ``corr-tile-size``. If the former two are explicitly set, they should
   be equal to each other, and then the latter parameter will be set to
   the same value.

Each process spawned by ``parallel_stereo`` can use multiple threads with
``threads-singleprocess`` without affecting the stereo results.

When SGM or MGM is specified, certain stereo parameters have their
default values replaced with values that will work with SGM. You can
still manually specify these options.

-  Cost Mode (default 4). Mean absolute distance (MAD)
   (``cost-mode <= 2``) usually does not work well. The census transform
   mode (``cost-mode 3``) :cite:`zabih1994census` tends to
   perform better overall but can produce artifacts on featureless
   terrain. The ternary census transform mode (``cost-mode 4``)
   :cite:`hua2016texture` is a modification of the census
   transform that is more stable on low contrast terrain but may be less
   accurate elsewhere.

-  Kernel size. SGM kernels must always be symmetric. The SGM algorithm
   works with much smaller kernel sizes than the regular integer
   correlator so the default large kernel is not recommended. The MAD
   cost mode can be used with any odd kernel size (including size 1) but
   the census cost modes can only be used with kernel sizes 3, 5, 7, and
   9. Size 7 is usually a good choice.

-  Xcorr-Threshold. By default, this is disabled in order to nearly
   halve the (long) run time of the SGM algorithm. Set
   ``xcorr-threshold`` to >= 0 to turn it back on. If you set the
   ``min-xcorr-level`` parameter to 1 you can perform cross correlation
   on the smaller resolution levels without spending the time to run it
   on the largest resolution level.

-  The median and texture filters in the ``stereo_fltr`` tool (defaults
   3, 11, 0.13). These filters were designed specifically to clean up
   output from the SGM algorithm and are especially useful in
   suppressing image artifacts in low-texture portions of the image. A
   median filter size of 3 and a texture filter size of 11 are good
   starts but the best values will depend on your input images. The
   ``texture-smooth-scale`` parameter will have to be adjusted to taste,
   but a range of 0.13 to 0.15 is typical for icy images. These values
   are enabled by default and must be manually disabled. If your images
   have good texture throughout it may be best to disable these filters.

-  The ``prefilter-mode`` setting is ignored when using SGM.

-  The ``subpixel-mode`` If not set or set to values 7-12 SGM will
   perform subpixel interpolation during the stereo correlation step and
   will not do additional work in the stereo refinement step. This means
   that after dealing with the long SGM processing time you do not need
   to follow it up with a slow subpixel option! If desired, you can
   specify modes 1-4 to force those subpixel operations to be performed
   after the default SGM subpixel method.

.. figure:: images/correlation/icebridge_example_crop.png
   :name: corr-sgm-example

   A section of a NASA IceBridge image on the left with a pair of
   hill-shaded DEMs to the right it showing the difference between default
   ASP processing (upper right) and processing using the SGM algorithm
   (lower right).

:numref:`corr-sgm-example` shows a comparison between two
stereo modes. The DEM on the left was generated using the default stereo
parameters and ``--subpixel-mode 3``. The DEM on the right was generated
using the command::

     stereo --stereo-algorithm asp_sgm --threads 1 --xcorr-threshold -1 \
       --corr-kernel 7 7 --corr-tile-size 6400 --cost-mode 4      \
       --median-filter-size 3  --texture-smooth-size 13           \
       --texture-smooth-scale 0.13

Some grid pattern noise is visible in the image produced using SGM.
Using ``--stereo-algorithm asp_mgm`` should reduce it. And, as mentioned
earlier, for large images which won't fit in memory,
``--corr-tile-size`` can be set to a value like 4096, and
``parallel_stereo`` should be used.

.. _original_mgm:

Original implementation of MGM
------------------------------

ASP ships the MGM algorithm as implemented by its authors
(:cite:`facciolo2015mgm`) at::

    https://github.com/gfacciol/mgm

To use it, run::

    parallel_stereo --alignment-method local_epipolar \
      --stereo-algorithm mgm                          \
      --corr-tile-size 1024 --sgm-collar-size 512     \
      left.tif right.tif left.xml right.xml

In this mode, locally aligned portions of the input left and right
images are saved to disk, the MGM program (named ``mgm``) is
called for each such pair, it writes the computed disparity
back to disk, which is then ingested by ASP.

To be more specific, a global affine epipolar alignment of the left
and right images is computed first, then the aligned images are broken
up into tiles, with each tile being by default 1024 x 1024 pixels with
a 512 pixel padding (hence the total tile size is 2048 x 2048), local
epipolar alignment is computed for each tile, the combination of the
global and subsequent local alignment is applied to each original
image to get the locally aligned image tiles, and those are written to
disk, to be passed to ``mgm``.

The ``mgm`` program has its own options. Some are environmental
variables, to be set before the tool is called, such as
``CENSUS_NCC_WIN=5``, while others are passed to the ``mgm``
executable on the command line, for example, ``-t census``. To
communicate any such options to this program, invoke
``parallel_stereo`` (for example) with::

    --stereo-algorithm 'mgm CENSUS_NCC_WIN=5 -t census'

ASP will ensure these will be passed correctly to ``mgm``.
By default, ASP uses::

    MEDIAN=1 CENSUS_NCC_WIN=5 USE_TRUNCATED_LINEAR_POTENTIALS=1 TSGM=3 \
      -s vfit -t census -O 8

These are adjusted depending on which ones the user chooses to override.

The ``CENSUS_NCC_WIN`` parameter is is one of the more notable parameters,
as it determines the size of the window to use for correlation, so it
corresponds to the option ``--corr-kernel`` of ASP-implemented
algorithms.

ASP automatically finds the minimum and maximum estimated disparity,
and it passes it to ``mgm`` via the ``-r`` and ``-R`` switches.

Options for mgm
~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

These should be set on the command line before ``mgm`` is invoked.
(ASP does that automatically.)

CENSUS_NCC_WIN=3:
    Size of the window for the census prefilter algorithm and NCC
    (normalized cross-correlation).

TESTLRRL=1:
    If 1, do left-to-right and right-to-left consistency checks.

MEDIAN=0:
     Radius of the median filter post-processing.

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


.. _opencv_sgbm_options:

OpenCV SGBM
-----------

The ``parallel_stereo`` program can invoke the OpenCV
semi-global block-matching algorithm (SGBM) if called with::

    --alignment-method local_epipolar \
    --stereo-algorithm "opencv_sgbm"

Alternatively, the full string having this algorithm and its
options can be used, as::

    --alignment-method local_epipolar                           \
    --stereo-algorithm                                          \
      "opencv_sgbm -mode hh -block_size 3 -P1 8 -P2 32
       -prefilter_cap 63 -uniqueness_ratio 10 -speckle_size 100
       -speckle_range 32 -disp12_diff 1"

If an invocation as follows is used::

    --alignment-method local_epipolar                 \
    --stereo-algorithm "opencv_sgbm -block_size 7"

ASP will use the earlier values for all the options except
``-block_size`` which will be set to 7. Hence, the user can explicitly
specify options whose values are desired to be different than the
default choices.

SGBM options
~~~~~~~~~~~~

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

.. _opencv_bm:

OpenCV BM
---------

The simpler and not as performing block-matching (BM) algorithm of
OpenCV can be invoked in a very similar manner to SGBM, with the
algorithm name passed to ``--stereo-algorithm`` being
``opencv_bm``. It accepts the same parameters except ``-P1`` and
``-P2``, and uses in addition the option:

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

.. _msmw:

Multi-Scale Multi-Window stereo matching
----------------------------------------

ASP provides access to the ``Multi-Scale Multi-Window`` (MSMW) stereo
matching algorithm :cite:`buades2015reliable`, by invoking its two
implementations ``msmw`` and ``msmw2`` from::

    https://github.com/centreborelli/s2p

(see the ``3rdparty`` directory). While that repository is released
under the AGPL-3.0 license and ASP is under the more permissive Apache
II license, ASP invokes that functionality as an external program via
a system call, so its license does not conflict with ours.

Options for msmw
~~~~~~~~~~~~~~~~

To invoke the ``msmw`` algorithm, run ``parallel_stereo`` with the
option::

    --alignment-method local_epipolar \
    --stereo-algorithm msmw

By default, ASP invokes this program as if it is called with::

    --stereo-algorithm "msmw -i 1 -n 4 -p 4 -W 5 -x 9 -y 9 -r 1
      -d 1 -t -1 -s 0 -b 0 -o 0.25 -f 0 -P 32"

In addition ASP, automatically calculates and passes to ``msmw``
values for the ``-m`` and ``-M`` options which correspond to
estimated minimum and maximum disparity values.

Any options explicitly specified by the user, such as::

    --stereo-algorithm "msmw -x 7 -y 7"

are substituted in the earlier string before ASP invokes this tool.

The meaning of these switches is as follows.

-m:
    Minimum disparity.
-M:
    Maximum disparity.

-x (default = 0):
  Width of the window (block) to match from the left to right
  image. Must be set to a positive odd value.

-y (default = 0):
    Matching window height. Must be set to a positive odd value.

-w (default = 0):
    Flag for weighting window.

-W (default = 5):
    Flag for using windows as lists (5x5 windows only). A non-zero
    value indicates how many of the orientations should be considered.
    (Note: Not sure what all this means.)

-i (default = 1):
    Type of distance.

-p (default = 1):
    Number of precisions for single scale.

-P (default = 1):
    Factor of disparity refinement by cubic interpolation.

-n (default = 3):
    Number of scales.

-f (default = 0):
    Standard deviation noise.

-r (default = 0):
    Reciprocity value.

-g (default = 0):
    Subpixel reciprocity flag.

-R (default = 0):
    Dual reciprocity value.

-l (default = 0):
   Inverse reciprocity value.

-d (default = 0):
    Mindist value.

-t (default = 0):
    Mindist dilatation.

-s (default = 0):
    Self-similarity value.

-b (default = 0):
    Integral of derivatives.

-v (default = 0):
    Variance value.

-o (default = 0):
    Remove isolated flag.

-O (default = 0):
    Remove isolated grain (number pixels).

-C (default = -1):
    Filter using the cost, train removing a fraction of the accepted
    points (e.g. 0.05).

-a (default = 0):
    Use Laplacian of the image instead of the image itself.


Options for msmw2
~~~~~~~~~~~~~~~~~

This flavor of the MSMW algorithm is called analogously, with::

    --stereo-algorithm msmw2

ASP fills in its options as if it is called as::

    --stereo-algorithm "msmw2 -i 1 -n 4 -p 4 -W 5 -x 9 -y 9
      -r 1 -d 1 -t -1 -s 0 -b 0 -o -0.25 -f 0 -P 32 -D 0 -O 25 -c 0"

As earlier, any of these can be overridden. Compared to ``msmw`` this
tool has the additional options:

-D (default = 0):
    Regression mindist.

-c (default = 0):
    Combine last scale with the previous one to densify the result.

.. _libelas:

LIBELAS stereo algorithm
------------------------

ASP ships and can invoke the ``LIBELAS`` (Library for Efficient
Large-scale Stereo Matching) algorithm :cite:`Geiger2010ACCV`,
described at::

    http://www.cvlibs.net/software/libelas/

We implemented an interface around this library to overcome its
assumption of the disparity being always positive, and added other
minor changes. Our fork having these additions is at::

    https://github.com/NeoGeographyToolkit/libelas

This software is released under GPL. ASP does not link to it directly,
rather it gets invoked as via a system call, with its inputs and
outputs being on disk.

To invoke it, run::

    parallel_stereo --alignment-method local_epipolar \
    --stereo-algorithm libelas                        \
    <other options>

If desired to override the values of any of its parameters, those can
be passed as follows::

    --stereo-algorithm "libelas -ipol_gap_width 100"

(This particular parameter is used to fill holes in the disparity,
with a larger value resulting in bigger holes being filled.)

The algorithm options, and their defaults, as used by ASP, are as
follows.

-disp_min (default = 0):
    Minimum disparity (ASP estimates this unless the user overrides it).

-disp_max (default = 255):
    Maximum disparity (ASP estimates this unless the user overrides it).

-support_threshold (default = 0.85):
    Maximum uniqueness ratio (best vs. second-best support match).

-support_texture (default = 10):
    Minimum texture for support points.

-candidate_stepsize (default = 5):
    Step size of regular grid on which support points are matched.

-incon_window_size (default = 5):
    Window size of inconsistent support point check.

-incon_threshold (default = 5):
    Disparity similarity threshold for support point to be considered
    consistent.

-incon_min_support (default = 5):
    Minimum number of consistent support points.

-add_corners (default = 0):
    Add support points at image corners with nearest neighbor
    disparities.

-grid_size (default = 20):
    Size of neighborhood for additional support point extrapolation.

-beta (default = 0.02):
    Image likelihood parameter.

-gamma (default = 3):
    Prior constant.

-sigma (default = 1):
    Prior sigma.

-sradius (default = 2):
    Prior sigma radius.

-match_texture (default = 1):
    Minimum texture for dense matching.

-lr_threshold (default = 2):
    Disparity threshold for left-right consistency check.

-speckle_sim_threshold (default = 1):
    Similarity threshold for speckle segmentation.

-speckle_size (default = 200):
    Speckles larger than this get removed.

-ipol_gap_width (default = 3):
    Fill holes in disparity of height and width at most this value.

-filter_median (default = 0):
    If non-zero, use an approximate median filter.

-filter_adaptive_mean (default = 1):
    If non-zero, use an approximate adaptive mean filter.

-postprocess_only_left (default = 0):
    If non-zero, saves time by not postprocessing the right image.

-verbose (default = 0):
    If non-zero, print some information about the values of the
    options being used, as well as what the input and output files
    are.

.. _adding_algos:

Adding new algorithms to ASP
----------------------------

ASP makes it possible for anybody to add their own algorithm to be
used for stereo correlation without having to recompile ASP itself.

Any such algorithm must a be program to be invoked as::

    myprog <options> left_image.tif right_image.tif \
      output_disparity.tif

Here, as often assumed in the computer vision community, the input
images ``left_image.tif`` and ``right_image.tif`` are expected to be
small image clips with epipolar alignment applied to them so that the
epipolar lines are horizontal and the resulting disparity only need to
be searched in the ``x`` direction (along each row). The images must
have the same size. (ASP will take care of preparing these images.)

The images must be in the TIF format, with pixel values being of the
``float`` type, and no-data pixels being set to ``NaN``. The output
disparity is expected to satisfy the same assumptions and be of
dimensions equal to those of the input images.

The options passed to this program are expected to have no other
characters except letters, numbers, space, period, underscore, plus,
minus, and equal signs.

Such a program and the libraries it depends on (if any) should be
copied somewhere within ASP's top-level directory, then this program
should be registered with ASP by adding a line to the file::

    plugins/stereo/plugin_list.txt

in that directory, in the format::

    myprog plugins/stereo/myprog/bin/myprog  plugin/stereo/myprog/lib

The entries here are the program name (in lowercase), path to the
program, and path to any libraries apart from those shipped with ASP
(the last entry is optional). All paths are relative to the ASP
top-level directory.

Then, ASP can invoke this program by calling it, for example, as::

    parallel_stereo  --alignment-method local_epipolar  \
      --stereo-algorithm "myprog <options>"             \
      <images> <cameras> <output prefix>

The program will be called for each pair of locally aligned tiles
obtained from these input images, with one subdirectory for each such
pair of inputs. That subdirectory will also have the output disparity
produced by the program. All such disparities will be read back by
ASP, blended together, then ASP will continue with the steps of
disparity filtering and triangulation.

It may be helpful to visit one of such subdirectories, examine the
``stereo_corr`` log file which will show how precisely the program was
called, and also look at its input image tiles and output disparity
stored there.

