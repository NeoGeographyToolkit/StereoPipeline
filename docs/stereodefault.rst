.. _stereodefault:

The ``stereo.default`` file
===========================

The ``stereo.default`` file contains configuration parameters that the
``parallel_stereo`` and ``stereo`` programs use to process images. The
``stereo.default`` file is loaded from the current working directory,
unless a different file is specified with the ``-s`` option. The
file extension is not important. 

As mentioned in :numref:`cmdline`, all the ``parallel_stereo``
parameters can also be specified on the command line, by prepending
them with two dashes.

A sample ``stereo.default.example`` file is included in the top-level
directory of the Stereo Pipeline software distribution. That
configuration is optimized for speed. See :numref:`nextsteps` for
various speed-vs-accuracy tradeoffs.

Listed below are the parameters used by ``parallel_stereo``, grouped
by processing stage.

.. _stereo-default-preprocessing:

Preprocessing
-------------

Interest point determination
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ip-per-tile
    How many interest points to detect in each :math:`1024^2` image
    tile (default: automatic determination).

ip-per-image
    How many interest points to detect in each image (default: automatic 
    determination, usually 5000). It is overridden by --ip-per-tile if
    provided.

ip-detect-method
    What type of interest point detection algorithm to use for image
    alignment. 0 = Custom OBAloG implementation (default), 1 = SIFT
    implementation from OpenCV, 2 = ORB implementation from OpenCV. If
    the default method does not perform well, try out one of the other
    two methods.

epipolar-threshold
    Maximum distance in pixels from the epipolar line to search for
    matches for each interest point. Due to the way ASP finds matches,
    reducing this value can actually increase the number of interest
    points detected. If image alignment seems to be working well but
    you are not getting enough interest points to get a good search
    range estimate, try setting this value to a small number, perhaps
    in the low double digits.

ip-inlier-factor (default = 1.0/15)
    A higher factor will result in more interest points, but perhaps
    also more outliers. It is important to note that this parameter
    overlaps somewhat in scope and effect with ``epipolar-threshold``
    and sometimes not both are active. It is suggested to experiment
    with both, as well as with ``ip-uniqueness-threshold`` below, which
    has a different justification but also somewhat similar effects.

ip-uniqueness-threshold (default = 0.7)
    A higher threshold will result in more interest points, but perhaps
    less unique ones.

ip-triangulation-max-error *double*
    When matching IP, filter out any pairs with a triangulation error
    higher than this.

ip-num-ransac-iterations *int(=100)*
    How many RANSAC iterations to do in interest point matching.

force-reuse-match-files
    Force reusing the match files even if older than the images or
    cameras.

.. _image_alignment:

Image alignment
~~~~~~~~~~~~~~~

alignment-method (= affineepipolar, local_epipolar, homography, epipolar, none) 
    (default = affineepipolar)

    When ``alignment-method`` is set to ``local_epipolar``,
    the images are divided into small tiles of size
    ``--corr-tile-size`` expanded by a padding of
    ``--sgm-collar-size`` on all sides, epipolar alignment is
    applied to each pair of tiles, making the stereo disparity
    horizontal, then a desired 1D correlation algorithm (specified via
    ``--stereo-algorithm``) finds this disparity :cite:`de2014automatic`. 
    Then the local alignment is undone for each disparity, the
    resulting disparities are merged and blended across the tiles,
    ASP's subpixel refinement is applied, if set via
    ``--subpixel-mode``, the combined disparity is filtered, and
    triangulation is performed. This mode works only with
    ``parallel_stereo``.

    When ``alignment-method`` is set to ``affineepipolar``, ``parallel_stereo``
    will attempt to pre-align the images by detecting tie-points using
    feature matching, and using those to transform the images such
    that pairs of conjugate epipolar lines become collinear and
    parallel to one of the image axes. The effect of this is
    equivalent to rotating the original cameras which took the
    pictures.

    When ``alignment-method`` is set to ``homography``, ``parallel_stereo`` will
    attempt to pre-align the images by automatically detecting
    tie-points between images using a feature matching. Tie points are
    stored in a ``*.match`` file that is used to compute a linear
    homography transformation of the right image so that it closely
    matches the left image. Note: the user may exercise more control
    over this process by using the ``ipfind`` and
    ``ipmatch`` tools.

    When ``alignment-method`` is set to ``epipolar``, ``parallel_stereo`` will
    apply a 3D transform to both images so that their epipolar lines will
    be horizontal. This speeds of stereo correlation as it greatly
    reduces the area required for searching.

    *Epipolar alignment is only available when calculating the stereo
    matches using the pinhole stereo session (i.e. when using
    ``stereo -t pinhole``), and cannot be used when processing other
    camera types.*

global-alignment-threshold (*float*) (default = 10)
    Maximum distance from inlier interest point matches to the
    epipolar line when calculating the global affine epipolar
    alignment.

local-alignment-threshold (*float*) (default = 2)
    Maximum distance from inlier interest point matches to the
    epipolar line when calculating the local affine epipolar
    alignment.

alignment-num-ransac-iterations (*integer*) (default = 1000)
    How many RANSAC iterations to use for global or local epipolar
    alignment.

outlier-removal-params (*double, double*) (default = 95.0, 3.0)
    Outlier removal params (percentage and factor) to be used in
    filtering interest points and the disparity with the
    box-and-whisker algorithm. Set the percentage to 100 to turn this
    off.

disparity-range-expansion-percent (*integer*) (default = 20)
    Expand the disparity range estimated from interest points by this
    percentage before computing the stereo correlation with local
    epipolar alignment.

Other pre-processing options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

force-use-entire-range (default = false)
    By default, the Stereo Pipeline will normalize ISIS images so that
    their maximum and minimum channel values are :math:`\pm`\ 2
    standard deviations from a mean value of 1.0. Use this option if
    you want to *disable* normalization and force the raw values to
    pass directly to the stereo correlations algorithms.

    For example, if the ISIS ``histeq`` tool has already been used to
    normalize the images, then use this option to disable
    normalization as a (redundant) pre-processing step.

individually-normalize (default = false)
    By default, the maximum and minimum valid pixel value is
    determined by looking at both images. Normalized with the same
    "global" min and max guarantees that the two images will retain
    their brightness and contrast relative to each other.

    This option forces each image to be normalized to its own maximum
    and minimum valid pixel value. This is useful in the event that
    images have different and non-overlapping dynamic ranges. You can
    sometimes tell when this option is needed: after a failed stereo
    attempt one of the rectified images (``*-L.tif`` and ``*-R.tif``)
    may be either mostly white or black. Activating this option may
    correct this problem.

    Note: Photometric calibration and image normalization are steps
    that can and should be carried out beforehand using ISIS's own
    utilities. This provides the best possible input to the stereo
    pipeline and yields the best stereo matching results.

nodata-value (default = none)
    Pixels with values less than or equal to this number are treated as
    no-data. This overrides the nodata values from input images.

datum (default = WGS_1984)
    Set the datum to use with RPC camera models. Options: WGS_1984,
    D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA
    (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth
    (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).

no-datum
    Do not assume a reliable datum exists, such as for irregularly
    shaped bodies.

skip-rough-homography 
    Skip the step of performing datum-based rough homography if it
    fails.

disable-correct-velocity-aberration
    Turn off velocity aberration correction for Optical Bar and
    non-ISIS linescan cameras (:numref:`sensor_corrections`).

disable-correct-atmospheric-refraction
    Turn off atmospheric refraction correction for Optical Bar and
    non-ISIS linescan cameras.

left-image-crop-win xoff yoff xsize ysize
    Do stereo in a sub-region of the left image [default: use the
    entire image].

right-image-crop-win xoff yoff xsize ysize
    When combined with ``left-image-crop-win``, do stereo in given
    subregions of left and right images. The crop windows can be
    determined using ``stereo_gui``. It is important to note that when
    both of these are specified, we explicitly crop the input images to
    these regions, which does not happen when ``left-image-crop-win``
    alone is specified. In that case we use the full images but only
    restrict the computation to the specified region.

left-image-clip: (*string*) (default = "")
    If --left-image-crop-win is used, replaced the left image
    cropped to that window with this clip.

right-image-clip: (*string*) (default = "")
    If --right-image-crop-win is used, replaced the right image
    cropped to that window with this clip.

.. _corr_section:

Correlation
-----------

stereo-algorithm (*string*) (default = "asp_bm")
    Use this option to switch between the different stereo 
    correlation algorithms supported by ASP. Options: ``asp_bm``,
    ``asp_sgm``, ``asp_mgm``, ``asp_final_mgm``, ``mgm`` (original
    author implementation), ``opencv_sgbm``, ``libelas``, ``msmw``,
    ``msmw2``, and ``opencv_bm``. See :numref:`stereo_algos` for their
    description.

prefilter-mode (= 0,1,2) (default = 2)
    This selects the pre-processing filter to be used to prepare
    images before they are fed to the initialization stage of the
    pipeline.

    0 - None

    1 - Subtracted mean
       This takes a preferably large Gaussian kernel and subtracts its
       value from the input image. This effectively reduces low frequency
       content in the image. The result is correlation that is immune to
       translations in image intensity.

    2 - LoG filter
       Takes the Laplacian of Gaussian of the image, This provides some
       immunity to differences in lighting conditions between a pair of
       images by isolating and matching on blob features in the image.

   For all of the modes above, the size of the filter kernel is
   determined by the ``prefilter-kernel-width`` parameter below.

   The choice of pre-processing filter must be made with thought to the
   cost function being used (see ``cost-mode``, below). LoG filter
   preprocessing provides good immunity to variations in lighting
   conditions and is usually the recommended choice.

prefilter-kernel-width (*float*) (default = 1.4)
    This defines the diameter of the Gaussian convolution kernel used
    for the preprocessing modes 1 and 2 above. A value of 1.4 works
    well for LoG and 25-30 works well for Subtracted Mean.

corr-seed-mode (=0,1,2,3)
    (default = 1)
    This integer parameter selects a strategy for how to solve for the
    low-resolution integer correlation disparity, which is used to seed
    the full-resolution disparity later on.

    0 - None
       Don't calculate a low-resolution variant of the disparity image.
       The search range provided by ``corr-search`` is used directly in
       computing the full-resolution disparity.

    1 - Low-resolution disparity from stereo
       Calculate a low-resolution version of the disparity from the
       integer correlation of subsampled left and right images. The
       low-resolution disparity will be used to narrow down the search
       range for the full-resolution disparity.

       This is a useful option despite the fact that our integer
       correlation implementation does indeed use a pyramid approach. Our
       implementation cannot search infinitely into lower resolutions due
       to its independent and tiled nature. This low-resolution disparity
       seed is a good hybrid approach.

    2 - Low-resolution disparity from an input DEM
       Use a lower-resolution DEM together with an estimated value for
       its error to compute the low-resolution disparity, which will then
       be used to find the full-resolution disparity as above. These
       quantities can be specified via the options
       ``disparity-estimation-dem`` and
       ``disparity-estimation-dem-error`` respectively. This option is
       not compatible with map projected input images.

    3 - Disparity from full-resolution images at a sparse number of points.
       This is an advanced option for terrain having snow and no
       large-scale features. It is described in :numref:`sparse-disp`.

    For large images, bigger than MOC-NA, using the low-resolution
    disparity seed is a definitive plus. Smaller images such as Cassini
    ISS or MER images should just shut this option off to save storage
    space.

corr-sub-seed-percent (*float*) (default=0.25)
    When using ``corr-seed-mode 1``, the solved-for or user-provided
    search range is grown by this factor for the purpose of computing
    the low-resolution disparity.

min-num-ip (*integer*) (default = 20)
    Automatic search range estimation will quit if at least this many
    interest points are not detected.

cost-mode (= 0,1,2,3,4)
    (default = 2 for ASP_BM and 4 for ASP_SGM and ASP_MGM)
    This defines the cost function used during integer correlation.
    Squared difference is the fastest cost function. However it comes
    at the price of not being resilient against noise. Absolute
    difference is the next fastest and is a better choice. Normalized
    cross correlation is the slowest but is designed to be more robust
    against image intensity changes and slight lighting differences.
    Normalized cross correlation is about 2x slower than absolute
    difference and about 3x slower than squared difference. The census
    transform :cite:`zabih1994census` and ternary census
    transform :cite:`hua2016texture` can only be used with
    the ASP_SGM and ASP_MGM correlators. See :numref:`asp_sgm` for
    details.

    | 0 - absolute difference
    | 1 - squared difference
    | 2 - normalized cross correlation
    | 3 - census transform
    | 4 - ternary census transform

corr-kernel (*integer integer*) (default = 25 25)
    These option determine the size (in pixels) of the correlation
    kernel used in the initialization step. A different size can be set
    in the horizontal and vertical directions, but square correlation
    kernels are almost always used in practice.

corr-search (*integer integer integer integer*)
    These parameters determine the size of the initial correlation
    search range. The ideal search range depends on a variety of
    factors ranging from how the images were pre-aligned to the
    resolution and range of disparities seen in a given image pair.
    This search range is successively refined during initialization, so
    it is often acceptable to set a large search range that is
    guaranteed to contain all of the disparities in a given image.
    However, setting tighter bounds on the search can sometimes reduce
    the number of erroneous matches, so it can be advantageous to tune
    the search range for a particular data set.

    If this option is not provided, ``parallel_stereo`` will make an attempt to
    guess its search range using interest points.

    These four integers define the minimum horizontal and vertical
    disparity and then the maximum horizontal and vertical disparity.

corr-search-limit (*integer integer integer integer*)
    Set these parameters to constrain the search range that ``parallel_stereo``
    automatically computes when ``corr-search`` is not set. This
    setting is useful when you have a good idea of the alignment
    quality in the vertical direction but not in the horizontal
    direction. For example, when using pinhole frame cameras with
    epipolar alignment the actual vertical search range may be much
    smaller than the automatically computed search range.

elevation-limit (*float float*)
    Notify ASP that all elevations are expected to fall in this range
    relative to the datum. Currently only used to restrict the search
    range estimate in nadir epipolar alignment cases.

corr-max-levels (*integer*) (default = 5)
    The maximum number of additional (lower) resolution levels to use
    when performing integer correlation. Setting this value to zero
    just performs correlation at the native resolution.

xcorr-threshold (*integer*) (default = 2)
    Integer correlation to a limited sense performs a correlation
    forward and backwards to double check its result. This is one of
    the first filtering steps to insure that we have indeed converged
    to a global minimum for an individual pixel. The
    ``xcorr-threshold`` parameter defines an agreement threshold in
    pixels between the forward and backward result.

    Optionally, this parameter can be set to a negative number. This will
    signal the correlator to only use the forward correlation result.
    This will drastically improve speed at the cost of additional noise.

min-xcorr-level (*integer*) (default = 0)
    When using the cross-correlation check controlled by
    xcorr-threshold, this parameter sets the minimum pyramid resolution
    level that the check will be performed at. By default the check
    will be performed at every resolution level but you may wish to
    increase this value to save time by not doubling up on processing
    the largest levels.

    Currently this feature is not enabled when using the default
    block-matching correlation method. In that case cross correlation is
    only ever performed on the last resolution level.

remove-outliers-by-disparity-params (*double double*) (default = 100 3)
    Outlier removal based on the disparity of interest points
    (difference between right and left pixel), when estimating the
    disparity search range. For example, the 10% and 90% percentiles of
    disparity are computed, and this interval is made three times
    bigger. Interest points whose disparity fall outside the expanded
    interval are removed as outliers. Instead of the default 100 and 3
    one can specify pct and factor, without quotes.

rm-quantile-percentile (*double*) (default = 0.85)
    See rm-quantile-multiple for details.

rm-quantile-multiple (*double*) (default = -1)
    Used for filtering disparity values in D_sub. Disparities greater
    than MULTIPLE*PERCENTILE (of the histogram) will be discarded. If
    this value is set greater than zero, this filtering method will be
    used instead of the method using the values RM_MIN_MATCHES and
    RM_THRESHOLD. This method will help filter out clusters of pixels
    which are too large to be filtered out by the neighborhood method
    but that have disparities significantly greater than the rest of
    the image.

corr-timeout (*integer*) (default = 900)
    Correlation timeout for an image tile, in seconds.

corr-blob-filter (*integer*) (default = 0)
    Set to apply a blob filter in each level of pyramidal integer
    correlation. When the correlator fails it often leaves "islands" of
    erroneous disparity results. Using this blob filter to remove them
    cleans up the final stereo output and can even reduce processing
    times by preventing the correlator from searching at large,
    incorrect disparity amounts. The value provided is the size of
    blobs in pixels that will be removed at the full image resolution.

corr-tile-size (*integer*) (default = 1024)
    Manually specifies the size of image tiles used by the correlator
    for multi-threaded processing. Typically there is no need to adjust
    this value but it is very important when using semi-global
    matching. See :numref:`asp_sgm` for details. This
    value must be a multiple of 16.

sgm-collar-size (*integer*) (default = 512)
    Specify the size of a region of additional processing around each
    correlation tile when using SGM or MGM processing. This helps
    reduce seam artifacts at tile borders when processing an image that
    needs to be broken up into tiles at the cost of additional
    processing time. This has no effect if the entire image can fit in
    one tile.

sgm-search-buffer (*integer integer*) (default = 4 4)
    This option determines the size (in pixels) searches around the
    expected disparity location in successive levels of the correlation
    pyramid. A smaller value will decrease run time and memory usage
    but will increase the chance of blunders. It is not recommended to
    reduce either value below 2.

corr-memory-limit-mb (*integer*) (default = 6144)
    Restrict the amount of memory used by the correlation step to be
    slightly above this value. This only really affects SGM/MGM which
    use a pair of large memory buffer in their computation. The total
    memory usage of these buffers is compared to this limit, and if it
    is greater then smaller search ranges will be used for uncertain
    pixels in order to reduce memory usage. If the required memory is
    still over this limit then the program will error out. The unit is
    in megabytes.

Subpixel refinement
-------------------

subpixel-mode (*integer*) (default = 1)
    This parameter selects the subpixel correlation method. Parabola
    subpixel is very fast but will produce results that are only
    slightly more accurate than those produced by the initialization
    step. Bayes EM (mode 2) is very slow but offers the best quality.
    When tuning ``stereo.default`` parameters, it is expedient to start
    out using parabola subpixel as a “draft mode.” When the results are
    looking good with parabola subpixel, then they will look even
    better with subpixel mode 2. For inputs with little noise, the
    affine method (subpixel mode 3) may produce results equivalent to
    Bayes EM in a shorter time. Phase correlation (subpixel mode 4) is
    uses a frequency domain technique. It is slow and is best may not
    produce better results than mode 2 but it may work well in some
    situations with flat terrain.

    Subpixel modes 5 and 6 are experimental. Modes 7-12 are only used as
    part of SGM/MGM correlation. These are much faster than subpixel
    modes 2-4 and if selected (with SGM/MGM) will be the only subpixel
    mode performed. They interpolate between the SGM/MGM integer results
    and should produce reasonable values. The default blend method for
    SGM/MGM is a custom algorithm that should work well but the you may
    find that one of the other options is better for your data.

    Subpixel modes 1-4 can be used in conjunction with SGM/MGM. In this
    case subpixel mode 12 will be used first, followed by the selected
    subpixel mode. Depending on your data this may produce better results
    than using just the SGM/MGM only methods. You may get bad artifacts
    combining mode 1 with SGM/MGM.

    | 0 - no subpixel refinement
    | 1 - parabola fitting 
    | 2 - affine adaptive window, Bayes EM weighting 
    | 3 - affine window 
    | 4 - phase correlation 
    | 5 - Lucas-Kanade method (experimental)
    | 6 - affine adaptive window, Bayes EM with Gamma Noise Distribution (experimental) 
    | 7 - SGM None 
    | 8 - SGM linear 
    | 9 - SGM Poly4 
    | 10 - SGM Cosine 
    | 11 - SGM Parabola 
    | 12 - SGM Blend 

    For a visual comparison of the quality of these subpixel modes, refer
    back to :numref:`correlation`.

subpixel-kernel (*integer integer*) (default = 35 35)
    Specify the size of the horizontal and vertical size (in pixels) of
    the subpixel correlation kernel. It is advantageous to keep this
    small for parabola fitting in order to resolve finer details. However
    for the Bayes EM methods, keep the kernel slightly larger. Those
    methods weight the kernel with a Gaussian distribution, thus the
    effective area is small than the kernel size defined here.

phase-subpixel-accuracy (*integer*) (default = 20)
    Set the maximum resolution of the phase subpixel correlator. The
    maximum resolution is equal to 1.0 / this value. Larger values
    increase accuracy but also computation time.

.. _filter_options:

Filtering
---------

filter-mode (*integer*) (default = 1)
    This parameter sets the filter mode. Three modes are supported as
    described below. Here, by neighboring pixels for a current pixel we
    mean those pixels within the window of half-size of
    ``rm-half-kernel`` centered at the current pixel.

    0
       No filtering.

    1
       Filter by discarding pixels at which disparity differs from mean
       disparity of neighbors by more than ``max-mean-diff``.

    2
       Filter by discarding pixels at which percentage of neighboring
       disparities that are within ``rm-threshold`` of current disparity
       is less than ``rm-min-matches``.

rm-half-kernel (*integer integer*) (default = 5 5)
    This setting adjusts the behavior of an outlier rejection scheme
    that “erodes” isolated regions of pixels in the disparity map that
    are in disagreement with their neighbors.

    The two parameters determine the size of the half kernel that is used
    to perform the automatic removal of low confidence pixels. A
    5 |times| 5 half kernel would result in an
    11 |times| 11 kernel with 121 pixels in it.

max-mean-diff (*integer*) (default = 3)
    This parameter sets the *maximum difference* between the current
    pixel disparity and the mean of disparities of neighbors in order
    for a given disparity value to be retained (for ``filter-mode`` 1).

rm-min-matches (*integer*) (default = 60)
    This parameter sets the *percentage* of neighboring disparity
    values that must fall within the inlier threshold in order for a
    given disparity value to be retained (for ``filter-mode`` 2).

rm-threshold (*double*) (default = 3)
    This parameter sets the inlier threshold for the outlier rejection
    scheme. This option works in conjunction with RM_MIN_MATCHES above.
    A disparity value is rejected if it differs by more than
    RM_THRESHOLD disparity values from RM_MIN_MATCHES percent of pixels
    in the region being considered (for ``filter-mode`` 2).

rm-cleanup-passes (*integer*) (default = 1)
    Select the number of outlier removal passes that are carried out.
    Each pass will erode pixels that do not match their neighbors. One
    pass is usually sufficient.

median-filter-size (*integer*) (default = 0)
    Apply a median filter of the selected kernel size to the subpixel
    disparity results. This option can only be used if
    ``rm-cleanup-passes`` is set to zero.

texture-smooth-size (*integer*) (default = 0)
    Apply an adaptive filter to smooth the disparity results inversely
    proportional to the amount of texture present in the input image.
    This value sets the maximum size of the smoothing kernel used (in
    pixels). This option can only be used if ``rm-cleanup-passes`` is
    set to zero.

texture-smooth-scale (*float*) (default = 0.15)
    Used in conjunction with ``texture-smooth-size``, this value helps
    control the regions of the image that will be smoothed. A larger
    value will result in more smoothing being applied to more of the
    image. A smaller value will leave high-texture regions of the image
    unsmoothed.

enable-fill-holes (default = false)
    Enable filling of holes in disparity using an inpainting method.
    Obsolete. It is suggested to use instead point2dem’s analogous
    functionality.

fill-holes-max-size (*integer*) (default = 100,000)
    Holes with no more pixels than this number should be filled in.

edge-buffer-size (*integer*) (default = -1)
    Crop to be applied around image borders during filtering. If not
    set, default to subpixel kernel size.

erode-max-size (*integer*) (default = 0)
    Isolated blobs with no more pixels than this number should be
    removed.

gotcha-disparity-refinement
    Turn on the experimental Gotcha disparity refinement
    :cite:`tao2018massive`. It refines and overwrites F.tif. See the
    option ``casp-go-param-file`` for customizing its behavior.

casp-go-param-file (*string*) (default = ""):
    The parameter file to use with Gotcha (and in the future other
    CASP-GO functionality) when invoking the
    ``gotcha-disparity-refinement`` option. The default is to use the
    file ``share/CASP-GO_params.xml`` shipped with ASP.

.. _triangulation_options:

Post-processing (triangulation)
-------------------------------

near-universe-radius (*float*) (default = 0.0)

far-universe-radius (*float*) (default = 0.0)
    These parameters can be used to remove outliers from the 3D
    triangulated point cloud. The points that will be kept are those
    whose distance from the universe center (see below) is between
    ``near-universe-radius`` and ``far-universe-radius``, in meters.

universe-center (default = none)
    Defines the reference location to use when filtering the output
    point cloud using the above near and far radius options. The
    available options are:

    None
       Disable filtering.

    Camera
       Use the left camera center as the universe center.

    Zero
       Use the planet center as the universe center.

bundle-adjust-prefix (*string*)
    Use the camera adjustments obtained by previously running
    bundle_adjust with this output prefix.

min-triangulation-angle (*double*)
    The minimum angle, in degrees, at which rays must meet at a
    triangulated point to accept this point as valid. It must be 
    positive. The internal default is somewhat less than 1 degree.

point-cloud-rounding-error (*double*)
    How much to round the output point cloud values, in meters (more
    rounding means less precision but potentially smaller size on
    disk). The inverse of a power of 2 is suggested. Default:
    :math:`1/2^{10}` meters (about 1mm) for Earth and proportionally
    less for smaller bodies.

save-double-precision-point-cloud (default = false)
    Save the final point cloud in double precision rather than bringing
    the points closer to origin and saving as float (marginally more
    precision at twice the storage).

compute-error-vector (default = false)
    When writing the output point cloud, save the 3D triangulation
    error vector (the vector between the closest points on the rays
    emanating from the two cameras), rather than just its length. In
    this case, the point cloud will have 6 bands (storing the
    triangulation point and triangulation error vector) rather than the
    usual 4. When invoking ``point2dem`` on this 6-band point cloud and
    specifying the ``--errorimage`` option, the error image will
    contain the three components of the triangulation error vector in
    the North-East-Down coordinate system.

    The next several parameters are used for jitter correction for
    DigitalGlobe/Maxar images. A usage tutorial is given in :numref:`jitter`.

image-lines-per-piecewise-adjustment (*integer*) (default = 0)
    A positive value, e.g., 1000, will turn on using piecewise camera
    adjustments to help reduce jitter effects. Use one adjustment per
    this many image lines.

piecewise-adjustment-percentiles (*float float*) (default = 5 95)
    A narrower range will place the piecewise adjustments for jitter
    correction closer together and further from the first and last lines
    in the image.

piecewise-adjustment-interp-type (*integer*) (default = 1)
    How to interpolate between adjustments. [1 Linear, 2 Using Gaussian
    weights]

piecewise-adjustment-camera-weight (*float*) (default = 1.0)
    The weight to use for the sum of squares of adjustments component of
    the cost function. Increasing this value will constrain the
    adjustments to be smaller.

num-matches-for-piecewise-adjustment (*integer*) (default = 90000)
    How many matches among images to create based on the disparity for
    the purpose of solving for jitter using piecewise adjustment.

    These last two options are used internally.

compute-piecewise-adjustments-only (default = false)
    Compute the piecewise adjustments as part of jitter correction, and
    then stop.

skip-computing-piecewise-adjustments (default = false)
    Skip computing the piecewise adjustments for jitter, they should
    have been done by now.


Bathymetry correction options
-----------------------------

These are options are used to infer the depth of shallow-water bodies
(see :numref:`shallow_water_bathy`).

Pre-processing stage
~~~~~~~~~~~~~~~~~~~~
left-bathy-mask (*string*)
    Mask to use for the left image when doing bathymetry.

right-bathy-mask (*string*)
    Mask to use for the right image when doing bathymetry.


Triangulation stage
~~~~~~~~~~~~~~~~~~~

bathy-plane (*string*)
    The file storing the water plane used for bathymetry having the coefficients 
    a, b, c, d with the plane being a*x + b*y + c*z + d = 0. Separate
    bathy planes can be used for the left and right images, to be passed in
    as 'left_plane.txt right_plane.txt'.

refraction-index (*double*) (default = 0.0) 
    The index of refraction of water to be used in bathymetry correction.
    (Must be specified and bigger than 1.)

output-cloud-type arg (*string*) (default = all)
    When bathymetry correction is used, return only the triangulated cloud of 
    points where the bathymetry correction was applied (option:
    'bathy'), where it was not applied (option: 'topo'), or the full
    cloud (option: 'all').

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
