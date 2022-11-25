.. _bundle_adjust:

bundle_adjust
-------------

The ``bundle_adjust`` program performs bundle adjustment on a given
set of images and cameras. An introduction to bundle adjustment, and
some advanced usage, including solving for intrinsics, can be found in
:numref:`bundle_adjustment`. If it is desired to process a large
number of images, consider using ``parallel_bundle_adjust``
(:numref:`parallel_bundle_adjust`).

This tool can use several underlying least-squares minimization
algorithms, the default is Google's *Ceres Solver*
(http://ceres-solver.org/).

Usage::

     bundle_adjust <images> <cameras> <optional ground control points> \
       -o <output prefix> [options]

Example (for ISIS)::

     bundle_adjust file1.cub file2.cub file3.cub -o run_ba/run

Example (for Digital Globe Earth data, using ground control points)::

     bundle_adjust file1.tif file2.tif file1.xml file2.xml gcp_file.gcp \
       --datum WGS_1984 -o run_ba/run --num-passes 2

Here, we invoked the tool with two passes, which also enables removal of
outliers by reprojection error (:numref:`bundle_adjustment`) 
and disparity (the options below have more detail).

Examples for RPC cameras. With the cameras stored separately::

    bundle_adjust -t rpc left.tif right.tif left.xml right.xml \
      -o run_ba/run

With the cameras embedded in the images::

    bundle_adjust -t rpc left.tif right.tif -o run_ba/run

Example (for generic pinhole camera data, using optional estimated camera
positions)::

     bundle_adjust file1.JPG file2.JPG file1.tsai file2.tsai   \
        -o run_ba/run -t nadirpinhole --inline-adjustments     \
        --datum WGS_1984 --camera-positions nav_data.csv       \
        --csv-format "1:file 6:lat 7:lon 9:height_above_datum"

Here we assumed that the cameras point towards some planet's surface and
used the ``nadirpinhole`` session. If this assumption is not true one
should use the ``pinhole`` session, though this one often does not
perform as well when finding interest points in planetary context.

This tool will write the adjustments to the cameras as ``*.adjust``
files starting with the specified output prefix
(:numref:`adjust_files`). In order for ``stereo`` to use the adjusted
cameras, it should be passed this output prefix via the option
``--bundle-adjust-prefix``. For example::

     stereo file1.cub file2.cub run_stereo/run \
       --bundle-adjust-prefix run_ba/run

The same option can be used with mapprojection (this example has the
cameras in .xml format)::

     mapproject input-DEM.tif image.tif camera.xml mapped_image.tif \
       --bundle-adjust-prefix run_ba/run

If the ``--inline-adjustments`` option is used, no separate adjustments
will be written, rather, the tool will save to disk copies of the input
cameras with adjustments already applied to them. These output cameras
can then be passed directly to stereo::

     stereo file1.JPG file2.JPG run_ba/run-file1.tsai \
       run_ba/run-file2.tsai run_stereo/run

The ``bundle_adjust`` program can read camera adjustments from a
previous run, via ``--input-adjustments-prefix string``. It can also
apply to the input cameras a transform as output by ``pc_align``, via
``--initial-transform string``. This is useful if a DEM produced by
ASP was aligned to a ground truth, and it is desired to apply the same
alignment to the cameras that were used to create that DEM. The
initial transform can have a rotation, translation, and scale, and it
is applied after the input adjustments are read, if those are
present. An example is shown in (:numref:`ba_pc_align`).

The match files created by ``bundle_adjust`` can be used later by
other ``bundle_adjust`` or ``parallel_stereo`` invocations, with the
options ``--match-files-prefix`` and ``--clean-match-files-prefix``.

.. _bagcp:

Ground control points
~~~~~~~~~~~~~~~~~~~~~

A number of plain-text files containing ground control points (GCP)
can be passed as inputs to ``bundle_adjust``. These can either be
created by hand, or using ``stereo_gui`` (:numref:`creatinggcp`).

A GCP file must end with a ``.gcp`` extension, and contain one ground
control point per line. Each line must have the following fields:

-  ground control point id (integer)

-  latitude (in degrees)

-  longitude (in degrees)

-  height above datum (in meters), with the datum itself specified
   separately

-  :math:`x, y, z` standard deviations (three positive floating point
   numbers, smaller values suggest more reliable measurements)

On the same line, for each image in which the ground control point is
visible there should be:

-  image file name

-  column index in image (float)

-  row index in image (float)

-  column and row standard deviations (two positive floating point
   numbers, smaller values suggest more reliable measurements)

The fields can be separated by spaces or commas. Here is a sample
representation of a ground control point measurement::

   5 23.7 160.1 427.1 1.0 1.0 1.0 image1.tif 124.5 19.7 1.0 1.0 image2.tif 254.3 73.9 1.0 1.0

When the ``--use-lon-lat-height-gcp-error`` flag is used, the three
standard deviations are interpreted as applying not to :math:`x, y, z`
but to latitude, longitude, and height above datum (in this order).
Hence, if the latitude and longitude are known accurately, while the
height less so, the third standard deviation can be set to something
larger.

Such a ``.gcp`` file then can be passed to ``bundle_adjust`` 
as shown earlier, with one or more images and cameras, and the 
obtained adjustments can be used with ``stereo`` or ``mapproject``
as described above.

.. _ba_out_files:

Output files
~~~~~~~~~~~~

If the ``--datum`` option is specified, ``bundle_adjust`` will write
the triangulated world position for every feature being matched in two
or more images, and the mean absolute residuals (that is, reprojection
errors, :numref:`bundle_adjustment`) for each position, before and
after optimization. The files are named

::

     {output-prefix}-initial_residuals_pointmap.csv

and

::

     {output-prefix}-final_residuals_pointmap.csv

Such files can be inspected to see at which pixels the reprojection
error is large.

Residuals corresponding to GCP will be printed at the end
of these files and flagged with the string ``# GCP``. 

These residuals can be plotted with ``stereo_gui``
(:numref:`plot_csv`).

One can also invoke ``point2dem`` with the ``--csv-format``
option to grid these files for visualization in the GUI. Here is a
sample file::

   # lon, lat, height_above_datum, mean_residual, num_observations
   -55.11690935, -69.34307716, 4.824523817, 0.1141333633, 2

The field ``num_observations`` counts how many images each point gets
projected into.

The initial and final mean and median of residual error norms for the
pixels each camera are written to ``residuals_stats.txt`` files in
the output directory.

The command::

    geodiff --absolute --csv-format '1:lon 2:lat 3:height_above_datum' \
      {output-prefix}-final_residuals_pointmap.csv dem.tif

(:numref:`geodiff`) can be used to evaluate how well the residuals
agree with a given DEM.  That can be especially useful if bundle
adjustment was invoked with the ``--heights-from-dem`` option.

As a finer-grained metric, initial and final ``raw_pixels.txt`` files
will be written, having the row and column residuals (reprojection
errors) for each pixel in each camera.

The convergence angle percentiles for each pair of images having matches
are saved to::

    {output-prefix}-convergence_angles.txt

If the options ``--heights-from-dem`` and
``--save-mapproj-match-points-offsets`` are specified, the file::

    {output-prefix}-mapproj_match_offsets.txt

will be written. For each pair of images having matches, the matches
are mapprojected onto the DEM, and percentiles of their disagreements
are computed, for each image vs the rest, and per image pair, in DEM
pixel units. This helps with evaluating image co-registration.

.. _adjust_files:

Format of .adjust files
~~~~~~~~~~~~~~~~~~~~~~~

Unless ``bundle_adjust`` is invoked with the ``--inline-adjustments``
option, when it modifies the cameras in-place, it will save the camera
adjustments in ``.adjust`` files using the specified output prefix.
Such a file stores a translation *T* as *x y z* (measured in
meters) and a rotation *R* as a quaternion in the order *w x y
z*. The rotation is around the camera center *C* for pixel (0, 0)
(for a linescan camera the camera center depends on the pixel).

Hence, if *P* is a point in ECEF, that is, the world in which the camera
exists, and an adjustment is applied to the camera, projecting *P* 
in the original camera gives the same result as projecting::

    P' = R * (P - C) + C + T

in the adjusted camera. 

Note that currently the camera center *C* is not exposed in the
``.adjust`` file, so external tools cannot recreate this
transform. This will be rectified at a future time.

Command-line options for bundle_adjust
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

-o, --output-prefix <filename>
    Prefix for output filenames.

--cost-function <string (default: Cauchy)>
    Choose a cost function from: Cauchy, PseudoHuber, Huber, L1, L2

--robust-threshold <double (default:0.5)>
    Set the threshold for robust cost functions. Increasing this
    makes the solver focus harder on the larger errors.

--datum <string>
    Set the datum. This will override the datum from the input
    images and also ``--t_srs``, ``--semi-major-axis``, and
    ``--semi-minor-axis``.
    Options:

    - WGS_1984
    - D_MOON (1,737,400 meters)
    - D_MARS (3,396,190 meters)
    - MOLA (3,396,000 meters)
    - NAD83
    - WGS72
    - NAD27
    - Earth (alias for WGS_1984)
    - Mars (alias for D_MARS)
    - Moon (alias for D_MOON)

--semi-major-axis <float (default: 0)>
    Explicitly set the datum semi-major axis in meters.

--semi-minor-axis <float (default: 0)>
    Explicitly set the datum semi-minor axis in meters.

-t, --session-type <string>
    Select the stereo session type to use for processing. Usually
    the program can select this automatically by the file extension, 
    except for xml cameras. See :numref:`parallel_stereo_options` for
    options.

--min-matches <integer (default: 30)>
    Set the minimum number of matches between images that will be considered.

--max-pairwise-matches <integer (default: 10000)>
    Reduce the number of matches per pair of images to at most this
    number, by selecting a random subset, if needed. This happens
    when setting up the optimization, and before outlier filtering.

--num-iterations <integer (default: 100)>
    Set the maximum number of iterations.

--parameter-tolerance <double (default: 1e-8)>
    Stop when the relative error in the variables being optimized
    is less than this.

--overlap-limit <integer (default: 0)>
    Limit the number of subsequent images to search for matches to
    the current image to this value.  By default try to match all
    images. See also ``--auto-overlap-params``.

--overlap-list <string>
    A file containing a list of image pairs, one pair per line,
    separated by a space, which are expected to overlap. Matches
    are then computed only among the images in each pair.

--auto-overlap-params <string (default: "")>
    Determine which camera images overlap by finding the lon-lat
    bounding boxes of their footprints given the specified DEM, expanding
    them by a given percentage, and see if those intersect. A higher
    percentage should be used when there is more uncertainty about the
    input camera poses. Example: 'dem.tif 15'.

--auto-overlap-buffer <double (default: not set)>
    Try to automatically determine which images overlap. Used only if
    this option is explicitly set. Only supports Worldview style XML
    camera files. The lon-lat footprints of the cameras are expanded
    outwards on all sides by this value (in degrees), before checking
    if they intersect.

--match-first-to-last
    Match the first several images to last several images by extending
    the logic of ``--overlap-limit`` past the last image to the earliest
    ones.

--tri-weight <double (default: 0.0)>
    The weight to give to the constraint that optimized triangulated
    points stay close to original triangulated points. A positive
    value will help ensure the cameras do not move too far, but a
    large value may prevent convergence. It is suggested to use 
    0.1 to 0.5 divided by image gsd for the value. Does not apply to GCP or
    points constrained by a DEM. This adds a robust cost function 
    with the threshold given by ``--robust-threshold``. Set
    ``--camera-weight`` and other weights to 0 when using this. 

--rotation-weight <double (default: 0.0)>
    A higher weight will penalize more rotation deviations from the
    original configuration.

--translation-weight <double (default: 0.0)>
    A higher weight will penalize more translation deviations from
    the original configuration.

--camera-weight <double(=1.0)>
    The weight to give to the constraint that the camera
    positions/orientations stay close to the original values (only
    for the Ceres solver). A higher weight means that the values
    will change less. The options ``--rotation-weight`` and
    ``--translation-weight`` can be used for finer-grained control and
    a stronger response.
        
--ip-per-tile <integer>
    How many interest points to detect in each :math:`1024^2` image tile.
    If this option isn't given, it will default to an automatic determination.

--ip-per-image <integer>
    How many interest points to detect in each image (default:
    automatic determination). It is overridden by ``--ip-per-tile`` if
    provided.

--ip-detect-method <integer (default: 0)>
    Choose an interest point detection method from: 0=OBAloG, 1=SIFT,
    2=ORB.

--epipolar-threshold <double (default: -1)>
    Maximum distance from the epipolar line to search for IP matches.
    If this option isn't given, it will default to an automatic determination.

--ip-inlier-factor <double (default: 1.0/15)>
    A higher factor will result in more interest points, but perhaps
    also more outliers.

--ip-uniqueness-threshold <double (default: 0.8)>
    A higher threshold will result in more interest points, but
    perhaps less unique ones.

--nodata-value <double(=NaN)>
    Pixels with values less than or equal to this number are treated
    as no-data. This overrides the no-data values from input images.

--individually-normalize
    Individually normalize the input images instead of using common
    values.

--inline-adjustments
    If this is set, and the input cameras are of the pinhole or
    panoramic type, apply the adjustments directly to the cameras,
    rather than saving them separately as .adjust files.

--input-adjustments-prefix <string>
    Prefix to read initial adjustments from, written by a previous
    invocation of this program.

--initial-transform <string>
    Before optimizing the cameras, apply to them the 4 |times| 4 rotation
    + translation transform from this file. The transform is in
    respect to the planet center, such as written by pc_align's
    source-to-reference or reference-to-source alignment transform.
    Set the number of iterations to 0 to stop at this step. If
    ``--input-adjustments-prefix`` is specified, the transform gets
    applied after the adjustments are read.

--fixed-camera-indices <string>
    A list of indices, in quotes and starting from 0, with space
    as separator, corresponding to cameras to keep fixed during the
    optimization process.

--fixed-image-list
    A file having a list of images (separated by spaces or newlines)
    whose cameras should be fixed during optimization.

--fix-gcp-xyz
    If the GCP are highly accurate, use this option to not float
    them during the optimization.

--use-lon-lat-height-gcp-error
    When having GCP, interpret the three standard deviations in the
    GCP file as applying not to x, y, and z, but rather to latitude,
    longitude, and height.

--solve-intrinsics
    Optimize intrinsic camera parameters. Only used for pinhole
    cameras.

--intrinsics-to-float <arg>
    If solving for intrinsics and desired to float only a few of
    them, specify here, in quotes, one or more of: focal_length,
    optical_center, other_intrinsics. Not specifying anything, will
    float all of them, if ``--solve-intrinsics`` is specified.

--intrinsics-to-share <arg>
    If solving for intrinsics and desired to share only a few of
    them, specify here, in quotes, one or more of: focal_length,
    optical_center, other_intrinsics. By default all of the intrinsics
    are shared, so to not share any of them pass in a blank string.

--intrinsics-limits <arg>
    Set a string in quotes that contains min max ratio pairs for
    intrinsic parameters. For example, "0.8 1.2" limits the parameter
    to changing by no more than 20 percent. The first pair is for
    focal length, the next two are for the center pixel, and the
    remaining pairs are for other intrinsic parameters. If too many
    pairs are passed in the program will throw an exception and
    print the number of intrinsic parameters the cameras use. Cameras
    adjust all of the parameters in the order they are specified
    in the camera model unless it is specified otherwise in
    :numref:`pinholemodels`.  Unfortunately, setting limits can
    greatly slow down the solver.

--num-passes <integer (default: 2)>
    How many passes of bundle adjustment to do, with given number
    of iterations in each pass. For more than one pass, outliers will
    be removed between passes using ``--remove-outliers-params``, 
    and re-optimization will take place. Residual files and a copy of
    the match files with the outliers removed (``*-clean.match``) will
    be written to disk.

--num-random-passes <integer (default: 0)>
    After performing the normal bundle adjustment passes, do this
    many more passes using the same matches but adding random offsets
    to the initial parameter values with the goal of avoiding local
    minima that the optimizer may be getting stuck in. Only the
    results for the optimization pass with the lowest error are
    kept.

--remove-outliers-params <'pct factor err1 err2' (default: '75.0 3.0 2.0 3.0')>
    Outlier removal based on percentage, when more than one bundle
    adjustment pass is used.  Triangulated points (that are not
    GCP) with reprojection error in pixels larger than: 
    *min(max(<pct>-th percentile \* <factor>, <err1>), <err2>)*
    will be removed as outliers.  Hence, never remove errors smaller
    than *<err1>* but always remove those bigger than *<err2>*. Specify as
    a list in quotes. Also remove outliers based on distribution
    of interest point matches and triangulated points.

--remove-outliers-by-disparity-params <pct factor>
    Outlier removal based on the disparity of interest points
    (difference between right and left pixel), when more than one
    bundle adjustment pass is used.  For example, the 10% and 90%
    percentiles of disparity are computed, and this interval is
    made three times bigger.  Interest points (that are not GCP)
    whose disparity fall outside the expanded interval are removed
    as outliers. Instead of the default 90 and 3 one can specify
    *pct* and *factor*, without quotes.

--elevation-limit <min max>
    Remove as outliers interest points (that are not GCP) for which
    the elevation of the triangulated position (after cameras are
    optimized) is outside of this range. Specify as two values.

--lon-lat-limit <min_lon min_lat max_lon max_lat>
    Remove as outliers interest points (that are not GCP) for which
    the longitude and latitude of the triangulated position (after
    cameras are optimized) are outside of this range.  Specify as
    four values.

--reference-terrain <filename>
    An externally provided trustworthy 3D terrain, either as a DEM
    or as a lidar file, very close (after alignment) to the stereo
    result from the given images and cameras that can be used as a
    reference, instead of GCP, to optimize the intrinsics of the
    cameras.

--max-num-reference-points <integer (default: 100000000)>
    Maximum number of (randomly picked) points from the reference
    terrain to use.

--disparity-list <'filename12 filename23 ...'>
    The unaligned disparity files to use when optimizing the
    intrinsics based on a reference terrain. Specify them as a list
    in quotes separated by spaces.  First file is for the first two
    images, second is for the second and third images, etc. If an
    image pair has no disparity file, use 'none'.

--max-disp-error <double (default: -1)>
    When using a reference terrain as an external control, ignore
    as outliers xyz points which projected in the left image and
    transported by disparity to the right image differ by the
    projection of xyz in the right image by more than this value
    in pixels.

--reference-terrain-weight <double (default: 1)>
    How much weight to give to the cost function terms involving
    the reference terrain.

--heights-from-dem <string>
    If the cameras have already been bundle-adjusted and aligned
    to a known high-quality DEM, in the triangulated xyz points
    replace the heights with the ones from this DEM, and fix those
    points unless ``--heights-from-dem-weight`` is positive.

--heights-from-dem-weight <double (default: 1.0)>
    How much weight to give to keep the triangulated points close
    to the DEM if specified via ``--heights-from-dem``. If the weight
    is not positive, keep the triangulated points fixed. This value
    should be inversely proportional with ground sample distance, as
    then it will convert the measurements from meters to pixels, which
    is consistent with the reprojection error term.

--heights-from-dem-robust-threshold <double (default: 0.5)> 
    If positive, this is the robust threshold to use keep the
    triangulated points close to the DEM if specified via
    ``--heights-from-dem``. This is applied after the point differences
    are multiplied by ``--heights-from-dem-weight``. It should
    help with attenuating large height difference outliers.

--save-mapproj-match-points-offsets
    If ``--heights-from-dem`` is specified, mapproject matching
    interest points onto this DEM and compute several percentiles of
    their discrepancy for each image vs the rest, and per image pair,
    in units of DEM's pixels.

--reference-dem <string>
    If specified, constrain every ground point where rays from
    matching pixels intersect to be not too far from the average of
    intersections of those rays with this DEM. This is being tested.

--reference-dem-weight <double (default: 1.0)>
    Multiply the xyz differences for the ``--reference-dem`` option by
    this weight. This is being tested.

--reference-dem-robust-threshold <double (default: 0.5)> 
    Use this robust threshold for the weighted xyz differences
    with the ``--reference-dem`` option. This is being tested.

--csv-format <string>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center), 
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-proj4``; the height above datum is in
    meters).  Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-proj4 <string>
    The PROJ.4 string to use to interpret the entries in input CSV
    files, if those files contain Easting and Northing fields.

--min-triangulation-angle <degrees (default: 0.1)>
    The minimum angle, in degrees, at which rays must meet at a
    triangulated point to accept this point as valid. It must
    be a positive value.

--ip-triangulation-max-error <float>
    When matching IP, filter out any pairs with a triangulation
    error higher than this.

--forced-triangulation-distance <meters>
    When triangulation fails, for example, when input cameras are
    inaccurate, artificially create a triangulation point this far
    ahead of the camera, in units of meters.

--ip-num-ransac-iterations <iterations (default: 1000)>
    How many RANSAC iterations to do in interest point matching.

--save-cnet-as-csv
    Save the initial control network containing all interest points
    in the format used by ground control points, so it can be
    inspected.

--camera-positions <filename>
    CSV file containing estimated positions of each camera. Only
    used with the inline-adjustments setting to initialize global
    camera coordinates. If used, the csv-format setting must also
    be set. The "file" field is searched for strings that are found
    in the input image files to match locations to cameras.

--disable-pinhole-gcp-init
    Do not try to initialize pinhole camera coordinates using provided
    GCP coordinates. Set this if you only have one image per GCP
    or if the pinhole initialization process is not producing good
    results.

--transform-cameras-using-gcp
    Use GCP, even those that show up in just an image, to transform
    cameras to ground coordinates.  Need at least two images to
    have at least 3 GCP each. If at least three GCP each show up
    in at least two images, the transform will happen even without
    this option using a more robust algorithm.

--position-filter-dist <max_dist (default: -1.0)>
    If estimated camera positions are used, this option can be used
    to set a threshold distance in meters between the cameras.  If
    any pair of cameras is farther apart than this distance, the
    tool will not attempt to find matching interest points between
    those two cameras.

--force-reuse-match-files
    Force reusing the match files even if older than the images or
    cameras.

--skip-matching
    Only use image matches which can be loaded from disk. This implies
    ``--force-reuse-match-files``.

--match-files-prefix <string (default: "")>
    Use the match files from this prefix instead of the current
    output prefix. This implies ``--skip-matching``.

--clean-match-files-prefix <string (default: "")>
    Use as input match files the \*-clean.match files from this prefix.
    This implies ``--skip-matching``.

--enable-rough-homography
    Enable the step of performing datum-based rough homography for
    interest point matching. This is best used with reasonably
    reliable input cameras and a wide footprint on the ground.

--skip-rough-homography
    Skip the step of performing datum-based rough homography.  This
    obsolete option is ignored as it is the default.

--enable-tri-ip-filter
    Enable triangulation-based interest points filtering. This is
    best used with reasonably reliable input cameras.

--disable-tri-ip-filter
    Disable triangulation-based interest points filtering. This
    obsolete option is ignored as is the default.

--no-datum
    Do not assume a reliable datum exists, such as for irregularly
    shaped bodies.

--mapprojected-data <string>
    Given map-projected versions of the input images and the DEM they
    were mapprojected onto, create interest point matches among the
    mapprojected images, unproject and save those matches, then
    continue with bundle adjustment. Existing match files will be
    reused. Specify the mapprojected images and the DEM as a string in
    quotes, separated by spaces. See :numref:`mapip` for an example.

--save-intermediate-cameras
    Save the values for the cameras at each iteration.

--apply-initial-transform-only
    Apply to the cameras the transform given by ``--initial-transform``.
    No iterations, GCP loading, or image matching takes place.

--image-list
    A file containing the list of images, when they are too many to
    specify on the command line. Use space or newline as
    separator. See also ``--camera-list`` and
    ``--mapprojected-data-list``.

--camera-list
    A file containing the list of cameras, when they are too many to
    specify on the command line. If the images have embedded camera
    information, such as for ISIS, this file must be empty but must
    be specified if ``--image-list`` is specified.

--mapprojected-data-list
    A file containing the list of mapprojected images and the DEM (see
    ``--mapprojected-data``), when they are too many to specify on the
    command line.

--proj-win
    Flag as outliers input triangulated points not in this proj
    win (box in projected units as provided by ``--proj_str``). This
    should be generous if the input cameras have significant errors.

--proj-str
    To be used in conjunction with  ``--proj_win``.

--save-vwip
    Save .vwip files (intermediate files for creating .match
    files). For ``parallel_bundle_adjust`` these will be saved in
    subdirectories, as they depend on the image pair.
    Must start with an empty output directory for this to work.

--enable-correct-velocity-aberration
    Turn on velocity aberration correction for Optical Bar and
    non-ISIS linescan cameras (:numref:`sensor_corrections`).
    This option impairs the convergence of bundle adjustment.

--enable-correct-atmospheric-refraction
    Turn on atmospheric refraction correction for Optical Bar and
    non-ISIS linescan cameras. This option impairs the convergence of
    bundle adjustment.

--threads <integer (default: 0)>
    Set the number threads to use. 0 means use the default defined
    in the program or in ``~/.vwrc``. Note that when using more
    than one thread and the Ceres option the results will vary
    slightly each time the tool is run.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB, for each process.

--dg-use-csm
    Use the CSM model with DigitalGlobe linescan cameras (``-t
    dg``). No corrections are done for velocity aberration or
    atmospheric refraction.

-v, --version
    Display the version of software.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
