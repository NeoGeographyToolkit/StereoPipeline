.. _bundle_adjust:

bundle_adjust
-------------

The ``bundle_adjust`` program performs bundle adjustment on a given
set of images and cameras. An introduction to bundle adjustment, and
some advanced usage, including solving for intrinsics, can be found in
:numref:`bundle_adjustment`. If it is desired to process a large
number of images, consider using ``parallel_bundle_adjust``
(:numref:`parallel_bundle_adjust`).

This tool solves a least squares problem (:numref:`how_ba_works`). It
uses Google's `Ceres Solver <http://ceres-solver.org/>`_.

Usage::

     bundle_adjust <images> <cameras> <optional ground control points> \
       -o <output prefix> [options]

Examples
~~~~~~~~

Example for ISIS cameras (:numref:`planetary_images`)::

     bundle_adjust --camera-weight 0 --tri-weight 0.1 \
       file1.cub file2.cub file3.cub -o run_ba/run

The above choices for camera weight and triangulation weight are a recent
implementation and suggested going forward, but not yet the defaults. These are
helpful in preventing the cameras from drifting too far from initial locations.

Example for Maxar (DigitalGlobe) Earth data (:numref:`dg_tutorial`). Ground
control points are used (:numref:`bagcp`)::

     bundle_adjust --camera-weight 0 --tri-weight 0.1       \
       file1.tif file2.tif file1.xml file2.xml gcp_file.gcp \
       --datum WGS_1984 -o run_ba/run --num-passes 2

Here, we invoked the tool with two passes, which also enables removal
of outliers (see option ``--remove-outliers-params``, :numref:`ba_options`).

Examples for RPC cameras (:numref:`rpc`). With the cameras stored separately::

    bundle_adjust -t rpc left.tif right.tif left.xml right.xml \
      -o run_ba/run

With the cameras embedded in the images::

    bundle_adjust -t rpc left.tif right.tif -o run_ba/run

Example for generic Pinhole cameras (:numref:`pinholemodels`),
using optional estimated camera positions::

     bundle_adjust file1.JPG file2.JPG file1.tsai file2.tsai   \
        -o run_ba/run -t nadirpinhole --inline-adjustments     \
        --datum WGS_1984 --camera-positions nav_data.csv       \
        --csv-format "1:file 6:lat 7:lon 9:height_above_datum"

Here we assumed that the cameras point towards some planet's surface and
used the ``nadirpinhole`` session. If this assumption is not true, one
should use the ``pinhole`` session or the ``--no-datum`` option.

Large-scale bundle adjustment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Bundle adjustment has been tested extensively and used successfully
with thousands of frame (pinhole) cameras and with close to 1000
linescan cameras. 

This tool provides options for constraints relative to a known ground,
can constrain the camera positions and orientations, and can apply an
alignment transform to the cameras (:numref:`ba_pc_align`).

Attention to choices of parameters and solid validation is needed in
such cases. The tool creates report files with various metrics
that can help judge how good the solution is (:numref:`ba_out_files`).

Large-scale usage of bundle adjustment is illustrated in the SkySat
processing example (:numref:`skysat`), with many Pinhole cameras, and
with a large number of linescan Lunar images with variable illumination
(:numref:`sfs-lola`). See :numref:`bundle_adjustment` for how to solve
for intrinsics.

See also the related jitter-solving tool (:numref:`jitter_solve`),
and the rig calibrator (:numref:`rig_calibrator`).

Use of the results
~~~~~~~~~~~~~~~~~~

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

.. _how_ba_works:

How bundle adjustment works
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Features are matched across images. Rays are cast though matching
features using the cameras, and triangulation happens, creating
points on the ground. More than two rays can meet at one triangulated
point, if a feature was successfully identified in more than two
images. The triangulated point is projected back in the cameras. The
sum of squares of differences (also called residuals) between the
pixel coordinates of the features and the locations where the
projections in the cameras occur is minimized. To not let outliers
dominate, a robust "loss" function is applied to each error term to
attenuate the residuals if they are too big. 
See the `Google Ceres <http://ceres-solver.org/nnls_modeling.html>`_
documentation on robust cost functions.

The option ``--cost-function`` controls the type of loss function, and
``--robust-threshold`` option is used to decide at which value of the
residuals the attenuation starts to work. The option
``--min-triangulation-angle`` is used to eliminate triangulated points
for which all the rays converging to it are too close to being
parallel. Such rays make the problem less well-behaved. The option
``--remove-outliers-params`` is used to filter outliers if more than
one optimization pass is used. See :numref:`ba_options` for more
options. See :numref:`bundle_adjustment` for a longer explanation.

The variables of optimization are the camera positions and
orientations, and the triangulated points on the ground. The
intrinsics can be optimized as well, either as a single set for all
cameras or individually (:numref:`floatingintrinsics`).
Triangulated points can be constrained
via ``--tri-weight`` or ``--heights-from-dem``. 

Ground control points can be used to incorporate measurements as part
of the constraints.

.. _bagcp:

Ground control points
~~~~~~~~~~~~~~~~~~~~~

File format
^^^^^^^^^^^

A number of plain-text files containing ground control points (GCP)
can be passed as inputs to ``bundle_adjust``. These can either be
created by hand, or using ``stereo_gui`` (:numref:`creatinggcp`).

A GCP file must end with a ``.gcp`` extension, and contain one ground
control point per line. Each line must have the following fields:

-  ground control point id (integer)

-  latitude (in degrees)

-  longitude (in degrees)

-  height above datum (in meters), with the datum itself specified
   separately, via ``--datum``

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

Effect on optimization
^^^^^^^^^^^^^^^^^^^^^^

Each ground control point will result in the following terms being
added to the cost function:

.. math::

    \frac{(x-x_0)^2}{std_x^2} + \frac{(y-y_0)^2}{std_y^2} + \frac{(z-z_0)^2}{std_z^2}

Here, :math:`(x_0, y_0, z_0)` is the input GCP, :math:`(x, y, z)` is
its version being optimized, and the standard deviations are from
above. No robustified bound is applied to these error terms (see
below). 

Note that the cost function normally contains sums of squares of
pixel differences (:numref:`how_ba_works`), 
while these terms are dimensionless, if the
numerators and denominators are assumed to be in meters. Care should
be taken that these terms not be allowed to dominate the cost function
at the expense of other terms.

The sums of squares of differences between projections into the
cameras of the GCP and the pixel values specified in the GCP file will
be added to the bundle adjustment cost function, with each difference
being divided by the corresponding pixel standard deviation. To
prevent these from dominating the problem, each such error has a
robust cost function applied to it, just as done for the regular
reprojection errors without GCP. See the `Google Ceres
<http://ceres-solver.org/nnls_modeling.html>`_ documentation on robust
cost functions. See also ``--cost-function`` and ``--robust-threshold``
option descriptions (:numref:`ba_options`).

The GCP pixel residuals (divided by the pixel standard deviations)
will be saved as the last lines of the report files ending in
``pointmap.csv`` (see :numref:`ba_out_files` for more
details). Differences between initial and optimized GCP will be
printed on screen.

To not optimize the GCP, use the option ``--fix-gcp-xyz``.

Creating or transforming pinhole cameras using GCP
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If for a given image the intrinsics of the camera are known, and also
the longitude and latitude (and optionally the heights above the
datum) of its corners (or of some other pixels in the image), the
``bundle_adjust`` tool can create an initial camera position and
orientation, and hence a complete pinhole camera. See
:numref:`imagecorners` for more details.

If desired to use GCP to apply a transform to a given
self-consistent camera set, see :numref:`sfm_world_coords`.

.. _ba_out_files:

Output files
~~~~~~~~~~~~

Camera projection errors and triangulated points
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the ``--datum`` option is specified or auto-guessed based on images
and cameras, ``bundle_adjust`` will write the triangulated world
position for every feature being matched in two or more images, and
the mean absolute residuals (that is, reprojection errors,
:numref:`bundle_adjustment`) for each position, before the first and
after the last optimization pass, in geodetic coordinates. The files
are named

::

     {output-prefix}-initial_residuals_pointmap.csv

and

::

     {output-prefix}-final_residuals_pointmap.csv

Here is a sample file::

   # lon, lat, height_above_datum, mean_residual, num_observations
   -55.11690935, -69.34307716, 4.824523817, 0.1141333633, 2

The field ``num_observations`` counts in how many images each
triangulated point is seen.

Such files can be plotted and overlayed with ``stereo_gui``
(:numref:`plot_csv`) to see at which triangulated points the
reprojection errors are large and their geographic locations.

Residuals corresponding to GCP will be printed at the end
of these files and flagged with the string ``# GCP``. 

The command::

    geodiff --absolute --csv-format '1:lon 2:lat 3:height_above_datum' \
      {output-prefix}-final_residuals_pointmap.csv dem.tif

(:numref:`geodiff`) can be used to evaluate how well the residuals
agree with a given DEM.  That can be especially useful if bundle
adjustment was invoked with the ``--heights-from-dem`` option.

One can also invoke ``point2dem`` with the above ``--csv-format``
option to grid these files to create a coarse DEM (also for the
error residuals).

The final triangulated positions can be used for alignment with
``pc_align`` (:numref:`pc_align`). Then, use
``--min-triangulation-angle 15.0`` with bundle adjustment or some
other higher value, to filter out unreliably triangulated points.
(This still allows, for example, to have a triangulated point
obtained by the intersection of three rays, with some
of those rays having an angle of at least this while some a much
smaller angle.)

The initial and final mean and median of residual error norms for the
pixels each camera, and their count, are written to
``residuals_stats.txt`` files in the output directory.

As a finer-grained metric, initial and final ``raw_pixels.txt`` files
will be written, having the row and column residuals (reprojection
errors) for each pixel in each camera.

.. _ba_conv_angle:

Convergence angles
^^^^^^^^^^^^^^^^^^

The convergence angle percentiles for rays emanating from matching 
interest points and intersecting on the ground (:numref:`stereo_pairs`)
are saved to::

    {output-prefix}-convergence_angles.txt

There is one entry for each pair of images having matches.

.. _ba_cam_pose:

Camera positions and orientations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the cameras are Pinhole and a datum exists, the camera names,
camera centers (in meters, in ECEF coordinates), as well as
the rotations from each camera to world North-East-Down
(NED) coordinates at the camera center are saved to::

     {output-prefix}-initial-cameras.csv
     {output-prefix}-final-cameras.csv

(before and after optimization; in either case, after any initial
transform and/or adjustments are applied). These are useful for
analysis when the number of cameras is large and the images are
acquired in quick succession (such as for SkySat data,
:numref:`skysat`). Note that such a rotation determines a camera's
orientation in NED coordinates. A conversion to geodetic coordinates
for the position and to Euler angles for the orientation may help
with this data's interpretation.
     
.. _ba_mapproj_dem:

Registration errors on the ground
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the option ``--mapproj-dem`` (with a DEM file as a value) is
specified, each pair of interest point matches (after bundle
adjustment and outlier removal) will be projected onto this DEM, and
the midpoint location and distance between these points will be
found. This data will be saved to::


    {output-prefix}-mapproj_match_offsets.txt

having the longitude, latitude, and height above datum of the
midpoint, and the above-mentioned distance between these projections
(in meters).

Ideally these distances should all be zero if the mapprojected images
agree perfectly. This makes it easy to see which camera images are
misregistered.

This file is very analogous to the ``pointmap.csv`` file, except that
these errors are measured on the ground in meters, and not in the cameras
in pixels. This file can be displayed and colorized in ``stereo_gui``
as a scatterplot (:numref:`plot_csv`).

In addition, more condensed statistics will be saved as well. The file::

    {output-prefix}-mapproj_match_offset_stats.txt

will be written. It will have the percentiles,(25%, 50%, 75%, 85%,
95%) of these disagreements for each image against the rest, and for each
pair of images, also in units of meter. 

This stats file is very useful at estimating the quality of registration
with the optimized cameras between the images and to the ground.

.. _adjust_files:

Format of .adjust files
~~~~~~~~~~~~~~~~~~~~~~~

Unless ``bundle_adjust`` is invoked with the ``--inline-adjustments``
option, when it modifies the cameras in-place, it will save the camera
adjustments in ``.adjust`` files using the specified output prefix.
Such a file stores a translation *T* as *x, y, z* (measured in
meters) and a rotation *R* as a quaternion in the order *w, x, y,
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

Adjustments are relative to the initial cameras, so a starting
adjustment has the zero translation and identity rotation (quaternion
1, 0, 0, 0).  Pre-existing adjustments can be specified with
``--input-adjustments-prefix``.

.. _ba_options:

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
    See the `Google Ceres <http://ceres-solver.org/nnls_modeling.html>`_
    documentation on robust cost functions.

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
    here 0.1 to 0.5 divided by image gsd. Does not apply to GCP or
    points constrained by a DEM via ``--heights-from-dem``. This adds
    a robust cost function with the threshold given by
    ``--tri-robust-threshold``. Set ``--camera-weight`` to 0 when
    using this.

--tri-robust-threshold <double (default: 0.1)>
    Use this robust threshold to attenuate large
    differences between initial and optimized triangulation points,
    after multiplying them by ``--tri-weight``.

--rotation-weight <double (default: 0.0)>
    A higher weight will penalize more camera rotation deviations from the
    original configuration.  This adds to the cost function
    the per-coordinate differences between initial and optimized
    normalized camera quaternions, multiplied by this weight, and then
    squared. No robust threshold is used to attenuate this term.

--translation-weight <double (default: 0.0)>
    A higher weight will penalize more camera center deviations from
    the original configuration. This adds to the cost function
    the per-coordinate differences between initial and optimized
    camera positions, multiplied by this weight, and then squared. No
    robust threshold is used to attenuate this term.

--camera-weight <double(=1.0)>
    The weight to give to the constraint that the camera
    positions/orientations stay close to the original values. A higher
    weight means that the values will change less. The options
    ``--rotation-weight`` and ``--translation-weight`` can be used for
    finer-grained control.
        
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
    In that case multiply the differences between the triangulated
    points and their corresponding DEM points by this weight
    in bundle adjustment. It is strongly suggested to pick positive
    and small values of ``--heights-from-dem-weight`` and
    ``--heights-from-dem-robust-threshold`` with this option.
    See :numref:`heights_from_dem`.

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

--mapproj-dem <string (default: "")>
    If specified, mapproject every pair of matched interest points
    onto this DEM and compute their distance, then percentiles of such
    distances for each image pair and for each image vs the
    rest. This is done after bundle adjustment and outlier removal.
    Measured in meters. See :numref:`ba_mapproj_dem` for more details.

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
    A triangulated point will be accepted as valid only if at
    least two of the rays which converge at it have a triangulation
    angle of at least this (measured in degrees). 

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

--init-camera-using-gcp
    Given an image, a pinhole camera lacking correct position and
    orientation, and a GCP file, find the pinhole camera with given
    intrinsics most consistent with the GCP (:numref:`imagecorners`).

--transform-cameras-with-shared-gcp
    Given at least 3 GCP, with each seen in at least 2 images,
    find the triangulated positions based on pixels values in the GCP,
    and apply a rotation + translation + scale transform to the entire
    camera system so that the the triangulated points get
    mapped to the ground coordinates in the GCP.

--transform-cameras-using-gcp
    Given a set of GCP, with at least two images having at least three
    GCP each (but with each GCP not shared among the images),
    transform the cameras to ground coordinates. This is not as robust
    as ``--transform-cameras-with-shared-gcp``.

--disable-pinhole-gcp-init
    Do not try to initialize pinhole camera coordinates using provided
    GCP coordinates. This ignored as is now the default. See also:
    ``--init-camera-using-gcp``.

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
    shaped bodies or when at the ground level. This is also helpful
    when the input cameras are not very accurate, as this option
    is used to do some camera-based filtering of interest points.

--mapprojected-data <string>
    Given map-projected versions of the input images and the DEM they
    were mapprojected onto, create interest point matches among the
    mapprojected images, unproject and save those matches, then
    continue with bundle adjustment. Existing match files will be
    reused. Specify the mapprojected images and the DEM as a string in
    quotes, separated by spaces. The DEM must be the last file.
    See :numref:`mapip` for an example.

--save-intermediate-cameras
    Save the values for the cameras at each iteration.

--apply-initial-transform-only
    Apply to the cameras the transform given by ``--initial-transform``.
    No iterations, GCP loading, image matching, or report generation
    take place. Using ``--num-iterations 0`` and without this option
    will create those.

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
    command line. The DEM must be the last entry.

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
