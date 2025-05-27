.. _bundle_adjust:

bundle_adjust
-------------

The ``bundle_adjust`` program performs bundle adjustment on a given
set of images and cameras. An introduction to bundle adjustment, and
some advanced usage, including solving for intrinsics, can be found in
:numref:`bundle_adjustment`. 

If it is desired to process a large number of images, consider using
``parallel_bundle_adjust`` (:numref:`parallel_bundle_adjust`).

This tool solves a least squares problem (:numref:`how_ba_works`). It
uses Google's `Ceres Solver <http://ceres-solver.org/>`_.

Usage::

     bundle_adjust <images> <cameras> <optional ground control points> \
       -o <output prefix> [options]

.. _ba_examples:

Examples
~~~~~~~~

ISIS cameras 
^^^^^^^^^^^^

See :numref:`moc_tutorial` for an introduction to these cameras.

::

     bundle_adjust --camera-weight 0 \
       --tri-weight 0.1              \
       --tri-robust-threshold 0.1    \
       file1.cub file2.cub file3.cub \
       -o run_ba/run

The above choices for camera weight and triangulation weight have in the
meantime become the defaults. These are helpful in preventing the cameras from
drifting too far from initial locations.

How to use the adjusted cameras is shown in :numref:`ba_use`.

.. _maxar_gcp:

Maxar Earth cameras and GCP
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here we use Maxar (DigitalGlobe) Earth data (:numref:`dg_tutorial`) and ground
control points (:numref:`bagcp`)::

    bundle_adjust                \
      --camera-weight 0          \
      --tri-weight 0.1           \
      --num-passes 2             \
      --datum WGS_1984           \
      file1.tif file2.tif        \
      file1.xml file2.xml        \
      gcp1.gcp gcp2.gcp gcp3.gcp \
      --fix-gcp-xyz              \
      -o run_ba/run 

How to use the adjusted cameras is shown in :numref:`ba_use`.

Using the proper value for ``--datum`` is very important, otherwise the
longitude-latitude-height values in the GCP files will not be interpreted
correctly.

There can be more than one GCP file. The option ``--fix-gcp-xyz`` fixes the GCP
coordinates during optimization. This is useful when the GCP are very accurate.
Otherwise set per-GCP sigma.

The residuals for the GCP will be printed at the end of the
``final_residuals_pointmap.csv`` file, and flagged with the string ``# GCP``
(:numref:`ba_err_per_point`).

We invoked the tool with two passes, which also enables removal
of outliers (see option ``--remove-outliers-params``, :numref:`ba_options`).

.. _ba_rpc:

RPC cameras and image lists
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Examples for RPC cameras (:numref:`rpc`). With the cameras stored separately::

    bundle_adjust -t rpc left.tif right.tif left.xml right.xml \
      -o run_ba/run

With the cameras embedded in the images::

    bundle_adjust -t rpc left.tif right.tif -o run_ba/run

How to use the adjusted cameras is shown in :numref:`ba_use`. How to produce RPC
cameras with the adjustments applied to them is discussed in
:numref:`rpc_and_ba`.

The images can be also passed in via ``--image-list`` and cameras with 
``--camera-list``. When the cameras are embedded in the images, the
``--camera-list`` option accepts the image files instead.

Pinhole cameras
^^^^^^^^^^^^^^^

::

     bundle_adjust -t nadirpinhole \
       --inline-adjustments        \
        --camera-weight 0          \
        --tri-weight 0.1           \
        --tri-robust-threshold 0.1 \
        --datum WGS_1984           \
        file1.JPG file2.JPG        \
        file1.tsai file2.tsai      \
        -o run_ba/run

See :numref:`pinholemodels` for the pinhole camera model format.

Here we assumed that the cameras point towards planet's surface and used the
``nadirpinhole`` session. If this assumption is not true, one should use the
``pinhole`` session or the ``--no-datum`` option.

The value of ``--datum`` should reflect the planetary body being imaged. If not
set, some functionality will not be available. It will be auto-guessed, either
based on camera files, input DEM, or camera center (the latter only for Earth,
Mars, Moon).

The option ``--inline-adjustments`` will save save to disk the optimized cameras
with adjustments already applied to them. These can be passed directly to
``parallel_stereo``, without using the original cameras and the adjustments as in
:numref:`ba_use`.

CSM cameras
^^^^^^^^^^^

::

     bundle_adjust file1.cub file2.cub              \
        file1.json file2.json                       \
        -t csm                                      \
        --camera-weight 0                           \
        --tri-weight 0.1 --tri-robust-threshold 0.1 \
        -o run_ba/run

CSM cameras (:numref:`csm`) can be stored in .json files or in .cub files. After
bundle adjustment, updated .json camera files will be written to disk, in
addition to .adjust files. See :numref:`csm_state` and :numref:`embedded_csm`.

Later, use either the original cameras with the computed adjustments
(:numref:`ba_use`), or the updated cameras without the adjustments.

The datum will be read from the camera files.

Other cameras
^^^^^^^^^^^^^

Bundle adjustment supports many other camera models. See :numref:`examples`
for the various sensor types.

.. _ba_use:

Use of the results
~~~~~~~~~~~~~~~~~~

This program will write the adjustments to the cameras as ``*.adjust`` files
starting with the specified output prefix (:numref:`adjust_files`). 

In order for ``parallel_stereo`` to use the adjusted cameras, it should be
passed the ``bundle_adjust`` output prefix via the option
``--bundle-adjust-prefix``. For example::

     parallel_stereo file1.cub file2.cub run_stereo/run \
       --bundle-adjust-prefix run_ba/run

The same option can be used with mapprojection (:numref:`mapproject`) and some
other tools. Example (for cameras in .xml format):: 

    mapproject --bundle-adjust-prefix run_ba/run \
      input-DEM.tif image.tif camera.xml mapped_image.tif

For Pinhole (:numref:`pinholemodels`) and OpticalBar (:numref:`panoramic`)
cameras, if the ``--inline-adjustments`` option is used, no separate adjustments
will be written, rather, the tool will save to disk copies of the input cameras
with adjustments already applied to them. These output cameras can then be
passed directly to ``parallel_stereo``::

    parallel_stereo                               \
      file1.JPG file2.JPG                         \
      run_ba/run-file1.tsai run_ba/run-file2.tsai \
      run_stereo/run

When cameras are of CSM type (:numref:`csm`), self-contained optimized cameras
will be written to disk (:numref:`csm_state`). These can also be appended to the
.cub files (:numref:`embedded_csm`).

To pass adjustments to ``bundle_adjust`` itself, use the option
``--input-adjustments-prefix``.

Camera adjustments and applying a transform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``bundle_adjust`` program can read camera adjustments from a previous run,
via ``--input-adjustments-prefix string``. Their format is described in
:numref:`adjust_files`. 

It can also apply to the input cameras a transform as output by ``pc_align``,
via ``--initial-transform string``. This is useful if a DEM produced by ASP was
aligned to a ground truth, and it is desired to apply the same alignment to the
cameras that were used to create that DEM. 

The initial transform can have a rotation, translation, and scale, and it is
applied after the input adjustments are read, if those are present. An example
is shown in (:numref:`ba_pc_align`). 

.. _ba_validation:

Validation
~~~~~~~~~~

The first report file to check after a run concludes is::

    {output-prefix}-final_residuals_stats.txt

(:numref:`ba_errors_per_camera`). It will have the mean and median pixel
reprojection error for each camera, and their count.

The errors should be under 1 pixel, ideally under 0.5 pixels. The count must
be at least a dozen, and ideally more. Otherwise bundle adjustment did
not work well. 

A fine-grained metric is the *triangulation error*, computed densely across the
images with stereo (:numref:`triangulation_error`). A systematic pattern in this
error may suggest the need to refine the camera intrinsics
(:numref:`floatingintrinsics`).

Other report files are described in :numref:`ba_out_files`.

Handling failures
~~~~~~~~~~~~~~~~~

This program will fail if the illumination changes too much between images (see
also :numref:`sfs_azimuth`).

Various approaches of creation of interest point matches are presented in 
:numref:`ba_ip` (the existing ones should be deleted first). Use ``stereo_gui``
(:numref:`stereo_gui_pairwise_matches`) to inspect the matches.

To make the program work harder at reducing big pixel reprojection errors, the
``--robust-threshold`` can be increased, perhaps to 2.0. This may result in the 
smallest reprojection errors increasing. 

.. _ba_constraints:

Constraints
~~~~~~~~~~~

The primary goal of bundle adjustment is to minimize the pixel reprojection
errors, so that the cameras are consistent with each other and with triangulated
points. 

To ensure the cameras and triangulated points do not drift, ground constraints
are set by default. They are meant to be rather soft, to not prevent the
reduction in reprojection errors.

.. _ba_ground_constraints:

Ground constraints
^^^^^^^^^^^^^^^^^^

The option ``--tri-weight`` constrains how much the triangulated points move.
This is a soft constraint and given less priority than reducing the pixel
reprojection errors in the cameras. Its default value is 0.1. An example is in
:numref:`skysat_stereo`.

This constraint adapts appropriately to the number of interest points and the
local average ground sample distance.

The measured distances between the initial and final triangulated points are
saved to a file (:numref:`ba_tri_offsets`) and should be inspected. Also check
the pixel reprojection errors per camera (:numref:`ba_errors_per_camera`).

The implementation is follows. The distances between initially triangulated
points and those being optimized points are computed, then divided by the local
averaged ground sample distance (GSD) (to make them into pixel units, like the
reprojection errors). These are multiplied by ``--tri-weight``. Then, the robust
threshold given by ``--tri-robust-threshold`` is applied, with a value of 0.1,
to attenuate the big residuals. This threshold is smaller than the pixel
reprojection error threshold (``--robust-threshold``), whose default value is
0.5, to ensure that this constraint does not prevent the optimization from
minimizing the pixel reprojection errors.

Triangulated points that are constrained via a DEM (option
``--heights-from-dem``, :numref:`heights_from_dem`), that is, those that are
close to a valid portion of this DEM, are not affected by the triangulation
constraint.

GCP can be used as well (:numref:`bagcp`).

.. _ba_cam_constraints:

Camera constraints
^^^^^^^^^^^^^^^^^^

If the position uncertainties per camera are known, the option
``--camera-position-uncertainty`` can be used. This sets hard
constraints on how much each camera position can move horizontally and
vertically, in meters, in the local North-East-Down coordinate system of each
camera. See :numref:`ba_options` for usage.

When using hard constraints in bundle adjustment, caution should be exercised as
they can impact the optimization process. It is not recommended to set
uncertainties below 1 - 10 meters, as this may result in slow convergence or even
failure to converge. It is better to overestimate the uncertainties in either
case.

It is suggested to examine the camera change report
(:numref:`ba_camera_offsets`) and pixel reprojection report
(:numref:`ba_errors_per_camera`) to see the effect of this constraint. 

The option ``--camera-position-weight``, with a default of 0.0 (so it is off by
default), offers a soft constraint, and is given less priority than reducing the
pixel reprojection errors. This was shown to impede the optimization process.
Use instead the option ``--camera-position-uncertainty``.

This weight is a multiplier, representing the ratio of strength of the camera
position constraint versus the pixel reprojection error constraint. Internally
the constraint adapts to the mean local ground sample distance, number of
interest points, and per-pixel uncertainty (1 sigma). The implementation is very
analogous to the triangulation constraint (:numref:`ba_ground_constraints`).

An additional modifier to this constraint is the option
``--camera-position-robust-threshold``. This is a robust threshold, with a
default of 0.1, that will attenuate big differences in camera position. Its
documentation has more details. 
 
It is suggested not to use the option ``--rotation-weight``, as camera position
and ground constraints are usually sufficient.

Use cases
~~~~~~~~~

Large-scale bundle adjustment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Bundle adjustment has been tested extensively and used successfully with
thousands of frame (pinhole) cameras and with close to 1000 linescan cameras. 

Large-scale usage of bundle adjustment is illustrated in the SkySat
processing example (:numref:`skysat`), with many Pinhole cameras, and
with a large number of linescan Lunar images with variable illumination
(:numref:`sfs-lola`). 

Attention to choices of parameters and solid validation is needed in
such cases. The tool creates report files with various metrics
that can help judge how good the solution is (:numref:`ba_out_files`).

See also the related jitter-solving program (:numref:`jitter_solve`),
and the rig calibrator (:numref:`rig_calibrator`).

Solving for intrinsics
^^^^^^^^^^^^^^^^^^^^^^

See :numref:`bundle_adjustment` for how to solve for intrinsics. In particular,
see :numref:`kaguya_ba` for the case when there exist several
sensors, each with its own intrinsics parameters.

.. _ba_ip:

Well-distributed interest points
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When different parts of the image have different properties, such as rock vs snow,
additional work may be needed to ensure interest points are created somewhat
uniformly. For that, use the option ``--matches-per-tile``::

    bundle_adjust image1.tif image2.tif       \
        image1.tsai image2.tsai               \
        --ip-per-tile 300                     \
        --matches-per-tile 100                \
        --max-pairwise-matches 20000          \
        --camera-weight 0 --tri-weight 0.1    \
        --remove-outliers-params '75 3 10 10' \
        -o run_ba/run 

For very large images, the number of interest points and matches per tile (whose
size is 1024 pixels on the side) should be decreased from the above. 

If the images have very different perspectives, it is suggested to create the
interest points based on mapprojected images (:numref:`mapip`.)

Uniformly distributed interest points can be produced from stereo disparity.

See :numref:`custom_ip` for more details.

Controlling where interest points are placed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A custom image or mask can be used to define a region where interest points
are created (:numref:`limit_ip`). 

Using mapprojected images
^^^^^^^^^^^^^^^^^^^^^^^^^

For images that have very large variation in elevation, it is suggested to use
bundle adjustment with the option ``--mapprojected-data`` for creating interest
point matches. An example is given in :numref:`mapip`.

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

The variables of optimization are the camera positions and orientations, and the
triangulated points on the ground. The intrinsics can be optimized as well,
either as a single set for all cameras or individually
(:numref:`floatingintrinsics`), or per group of cameras (:numref:`kaguya_ba`).

Triangulated points can be constrained via ``--tri-weight``
(:numref:`ba_ground_constraints`) or ``--heights-from-dem`` (with a
*well-aligned* prior DEM, :numref:`heights_from_dem`). The camera positions can
be constrained as well (:numref:`ba_cam_constraints`).

Ground control points can be employed to incorporate measurements as part of the
constraints (:numref:`bagcp`).

.. _bagcp:

Ground control points
~~~~~~~~~~~~~~~~~~~~~

Ground control points consist of known points on the ground, together with their
pixel locations in one or more images. Their use is to refine, initialize, or
transform to desired coordinates the camera poses (:numref:`ba_use_gcp`).

GCP can be created with ``stereo_gui`` (:numref:`creatinggcp`), ``gcp_gen``
(:numref:`gcp_gen`),  and ``dem2gcp`` (:numref:`dem2gcp`).

File format
^^^^^^^^^^^

A GCP file must end with a ``.gcp`` extension, and contain one ground
control point per line. Each line must have the following fields:

-  ground control point id (integer)

-  latitude (in degrees)

-  longitude (in degrees)

-  height above datum (in meters), with the datum itself specified
   separately, via ``--datum``

-  :math:`x, y, z` standard deviations (*sigma* values, three positive floating
   point numbers, smaller values suggest more reliable measurements, measured in
   meters)

On the same line, for each image in which the ground control point is
visible there should be:

-  image file name

-  column index in image (float, starting from 0)

-  row index in image (float, starting from 0)

-  column and row standard deviations (also called *sigma* values, two positive
   floating point numbers, smaller values suggest more reliable measurements, in
   units of pixel)

The fields can be separated by spaces or commas. Here is a sample
representation of a ground control point measurement::

   5 23.7 160.1 427.1 1.0 1.0 1.0 image1.tif 124.5 19.7 1.0 1.0 image2.tif 254.3 73.9 1.0 1.0

.. _ba_use_gcp:

Uses of GCP
^^^^^^^^^^^

One or more ``.gcp`` files can be passed to ``bundle_adjust`` as shown in
:numref:`ba_examples`, together with one or more images and cameras, to refine the
camera poses.  The option ``--datum`` must be set correctly to interpret the GCP.

GCP can also be employed to initialize the cameras (:numref:`camera_solve_gcp`), or
to transform them as a group, with the ``bundle_adjust`` options
``--transform-cameras-with-shared-gcp`` and ``--transform-cameras-using-gcp``. 
For use with SfM, see :numref:`sfm_world_coords`.

The option ``--fix-gcp-xyz`` fixes the GCP coordinates during optimization. This
is useful when the GCP are very accurate. Otherwise set per-GCP sigma.

The option ``--save-cnet-as-csv`` can be invoked to save the entire control
network in the GCP format, before any optimization. This can be useful for
comparing with any manually created GCP.

When the ``--use-lon-lat-height-gcp-error`` flag is set, the three
standard deviations are interpreted as applying not to :math:`x, y, z`
but to latitude, longitude, and height above datum (in this order).
Hence, if the latitude and longitude are known accurately, while the
height less so, the third standard deviation can be set to something
larger.

See :numref:`ba_out_files` for the output files, including for
more details about reports for GCP files.

Effect on optimization
^^^^^^^^^^^^^^^^^^^^^^

Each ground control point will result in the following terms being
added to the cost function:

.. math::

    \frac{(x-x_0)^2}{\sigma_x^2} + \frac{(y-y_0)^2}{\sigma_y^2} + \frac{(z-z_0)^2}{\sigma_z^2}

Here, :math:`(x_0, y_0, z_0)` is the input GCP, :math:`(x, y, z)` is its version
being optimized, and the sigma values are the standard deviations from
above. No robust cost function is applied to these error terms (see below). 

Note that the cost function normally contains sums of squares of
pixel differences (:numref:`how_ba_works`), 
while these terms are dimensionless, if the
numerators and denominators are assumed to be in meters. Care should
be taken that these terms not be allowed to dominate the cost function
at the expense of other terms.

The sums of squares of differences between projections into the cameras of the
GCP and the pixel values specified in the GCP file will be added to the bundle
adjustment cost function, with each difference being divided by the
corresponding pixel standard deviation (sigma). To prevent these from dominating
the problem, each such error has a robust cost function applied to it, just as
done for the regular reprojection errors without GCP. See the `Google Ceres
<http://ceres-solver.org/nnls_modeling.html>`_ documentation on robust cost
functions. See also ``--cost-function`` and ``--robust-threshold`` option
descriptions (:numref:`ba_options`).

The GCP pixel reprojection errors will be saved as the last lines of the report
files ending in ``pointmap.csv``. Differences between initial and optimized GCP
will be printed in a report file as well. See :numref:`ba_out_files` for more
details.

To not optimize the GCP, use the option ``--fix-gcp-xyz``.

.. _control_network:

Control network
~~~~~~~~~~~~~~~

.. _ba_match_files:

Match files
^^^^^^^^^^^

By default, ``bundle_adjust`` will create interest point matches between all
pairs of images (see also ``--auto-overlap-params``). These matches are
assembled into a *control network*, in which a triangulated point is associated
with features in two or more images. The match files are saved with the
specified output prefix and a ``.match`` extension. 

The naming convention for the match files is::

    <output prefix>-<image1>__<image2>.match
  
where the image names are without the directory name and extension. Excessively
long image names will be truncated.

These files can be used later by other ``bundle_adjust`` invocations, also by
``parallel_stereo`` and ``jitter_solve``, with the option ``--match-files-prefix``.

Bundle adjustment also produces outlier-filtered versions of these files, that
end with ``-clean.match``. These can be used with the option
``--clean-match-files-prefix``. The option ``--remove-outliers-params`` controls
the outlier filtering.

Any such files can be inspected with ``stereo_gui``
(:numref:`stereo_gui_pairwise_matches`) and converted to text with 
``parse_match_file.py`` (:numref:`parse_match_file`).

.. _jigsaw_cnet:

ISIS control network
^^^^^^^^^^^^^^^^^^^^

This program can read and write the ISIS binary control network format,
if invoked with the option ``--isis-cnet filename.net``. This format makes it 
possible to handle a very large number of control points. 

In this case, ``bundle_adjust`` will also write an updated version of this file,
with the name ``<output prefix>.net`` (instead of match files). 

If GCP are provided via a .gcp file (:numref:`bagcp`), these will be added to
the optimization and to the output ISIS control network file.

To have different formats for the input and output control networks, use the
option ``--output-cnet-type``. 

The ``stereo_gui`` program (:numref:`stereo_gui_isis_cnet`) can visualize
such a control network file. 

See :numref:`jigsaw_cnet_details` for more technical details. See also ASP's
``jigsaw`` tutorial (:numref:`jigsaw`).

.. _ba_nvm:

NVM format
^^^^^^^^^^

The ``bundle_adjust`` program can read and write the `VisualSfM NVM format
<http://ccwu.me/vsfm/doc.html#nvm>`_ for a control network. This helps in
interfacing with ``rig_calibrator`` (:numref:`rc_bundle_adjust`) and
``theia_sfm`` (:numref:`theia_sfm`). Usage::

    bundle_adjust --nvm input.nvm \
      --image-list images.txt     \
      --camera-list cameras.txt   \
      --inline-adjustments        \
      -o ba/run

This will write the file ``ba/run.nvm`` having the inlier interest point matches.

The NVM file assumes that the interest points are shifted relative to the optical
center of each camera. The optical centers are kept in a separate file ending with
``_offsets.txt``.

The NVM format can be used with any cameras supported by ASP. To export to this
format, use ``--output-cnet-type nvm``. Unless this option is explicitly set,
the output format is the same as the input format.

If the cameras are of Pinhole (:numref:`pinholemodels`) or CSM Frame
(:numref:`csm_frame`) type, the camera poses will be read from the NVM file as
well, and the optimized poses will be saved to such a file, unless invoked with
``--no-poses-from-nvm``. The optical centers will be read and written as well. The
input cameras must still be provided as above, however, so that the intrinsics
can be read. With the option ``--inline-adjustments``, the updated Pinhole
camera files will be written separately as well.

For all other types, no camera pose information will be read or written to the
NVM file, and the optical centers will be set to half the image dimensions.

.. _ba_out_files:

Output files
~~~~~~~~~~~~

.. _ba_errors_per_camera:

Reprojection errors per camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The initial and final mean and median pixel reprojection error (distance from
each interest point and camera projection of the triangulated point) for each
camera, and their count, are written to::

  {output-prefix}-initial_residuals_stats.txt
  {output-prefix}-final_residuals_stats.txt
 
It is very important to ensure all cameras have a small final reprojection
error, ideally under 1 pixel, as otherwise this means that the cameras are not
well-registered to each other, or that systematic effects exist, such as
uncorrected lens distortion.

See :numref:`ba_mapproj_dem` for an analogous report at the ground level
and :numref:`ba_err_per_point` for finer-grained reporting.

.. _ba_camera_offsets:

Camera position changes
^^^^^^^^^^^^^^^^^^^^^^^

If the ``--datum`` option is specified or auto-guessed based on images
and cameras, the file::

    {output-prefix}-camera_offsets.txt

will be written. It will have, for each camera, the horizontal and vertical
component of the difference in camera center before and after optimization, in
meters. This is after applying any initial adjustments or transform to the
cameras (:numref:`ba_pc_align`). The local North-East-Down coordinate system of
each camera determines the horizontal and vertical components.

This file is useful for understanding how far cameras may move and can help with
adding camera constraints (:numref:`ba_cam_constraints`).

For linescan cameras, the camera centers will be for the upper-left image pixel.

.. _ba_tri_offsets:

Changes in triangulated points
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The distance between each initial triangulated point (after applying any
initial adjustments or alignment transform, but before any DEM constraint) and
final triangulated point (after optimization) are computed (in ECEF, in meters).
The mean, median, and count of these distances, per camera, are saved to::

    {output-prefix}-triangulation_offsets.txt

This is helpful in understanding how much the triangulated points move. An
unreasonable amount of movement may suggest imposing stronger constraints on the
triangulated points (:numref:`ba_ground_constraints`).

.. _ba_conv_angle:

Convergence angles
^^^^^^^^^^^^^^^^^^

The convergence angle percentiles for rays emanating from matching 
interest points and intersecting on the ground (:numref:`stereo_pairs`)
are saved to::

    {output-prefix}-convergence_angles.txt

There is one entry for each pair of images having matches.

.. _ba_err_per_point:

Reprojection errors per triangulated point
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the ``--datum`` option is specified or auto-guessed based on images and
cameras, ``bundle_adjust`` will write the triangulated ground position for every
feature being matched in two or more images, and the mean pixel reprojection
error :numref:`bundle_adjustment`) for each position, before the first and after
the last optimization pass, in geodetic coordinates. The files are named

::

     {output-prefix}-initial_residuals_pointmap.csv
     {output-prefix}-final_residuals_pointmap.csv

Here is a sample file::

   # lon, lat, height_above_datum, mean_residual, num_observations
   -55.11690935, -69.34307716, 4.824523817, 0.1141333633, 2

The field ``num_observations`` counts in how many images each
triangulated point is seen.

Such files can be plotted and overlaid with ``stereo_gui``
(:numref:`plot_csv`) to see at which triangulated points the
reprojection errors are large and their geographic locations.

Pixel reprojection errors corresponding to GCP will be printed at the end of
these files and flagged with the string ``# GCP``. 

During the optimization the pixel differences are divided by pixel sigma.
This is undone when the pixel reprojection errors are later computed.

The command::

    geodiff --absolute --csv-format '1:lon 2:lat 3:height_above_datum' \
      {output-prefix}-final_residuals_pointmap.csv dem.tif

(:numref:`geodiff`) can be called to evaluate how well the residuals agree with
a given DEM. That can be especially useful if bundle adjustment was invoked with
the ``--heights-from-dem`` option (:numref:`heights_from_dem`).

One can also invoke ``point2dem`` (:numref:`point2dem`) with the above
``--csv-format`` option to grid these files to create a coarse DEM (also for the
error residuals).

The final triangulated positions can be used for alignment with
``pc_align`` (:numref:`pc_align`). Then, use
``--min-triangulation-angle 10.0`` with bundle adjustment or some
other higher value, to filter out unreliable triangulated points.
(This still allows, for example, to have a triangulated point
obtained by the intersection of three rays, with some
of those rays having an angle of at least this while some a much
smaller angle.)

GCP report
^^^^^^^^^^

If GCP are present, the file ``{output-prefix}-gcp_report.txt`` will be saved to
disk, having the initial and optimized GCP coordinates, and their difference,
both in ECEF and longitude-latitude-height above datum. 

The reprojection error file may be more helpful than this GCP report file
(:numref:`ba_err_per_point`). The GCP are flagged with the string ``# GCP`` at
the end of that file.

.. _ba_out_cams:

Image and camera lists
^^^^^^^^^^^^^^^^^^^^^^

The list of input images will be saved to::

    {output-prefix}-image_list.txt

The list of optimized cameras (such as for CSM cameras or with the option
``--inline-adjustments`` for Pinhole cameras) will be saved to::

    {output-prefix}-camera_list.txt

If only ``.adjust`` files are saved, these will be in the list instead.

This is convenient because both ``bundle_adjust`` and ``jitter_solve`` can read
such lists with the ``--image-list`` and ``--camera-list`` options (but not if
``.adjust`` files are saved).

.. _ba_error_propagation:

Error propagation
^^^^^^^^^^^^^^^^^

When the option ``--propagate-errors`` is specified, propagate the errors
(uncertainties) from the input cameras to the triangulated point for each pair
of inlier interest point matches. The produced uncertainties will be separated
into horizontal and vertical components relative to the datum. Statistical
measures will be produced for each pair of images.

The same logic as in stereo triangulation is employed (:numref:`error_propagation`),
but for the sparse set of interest point matches rather than for the dense image
disparity. Since the produced uncertainties depend only weakly on the
triangulated surface, computing them for a sparse set of features, and
summarizing the statistics, as done here, is usually sufficient.

Specify ``--horizontal-stddev`` (a single value for all cameras, measured in
meters), to use this as the input camera ground horizontal uncertainty.
Otherwise, as in the above-mentioned section, the input errors will be read from
camera files, if available.

The produced errors are saved to the file::

    {output-prefix}-triangulation_uncertainty.txt

This file will have, for each image pair having matches, the median horizontal
and vertical components of the triangulation uncertainties, the mean of each
type of uncertainty, the standard deviations, and number of samples used
(usually the same as the number of inliner interest points). All errors are in
meters.

This operation will use the cameras after bundle adjustment. Invoke with
``--num-iterations 0`` for the original cameras.

It is instructive to compare these uncertainties with their dense counterparts,
as produced by ``point2dem`` (:numref:`export_stddev`).

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

If the option ``--mapproj-dem`` (with a DEM file as a value) is specified, for
each pair of interest point matches (after bundle adjustment and outlier
removal) rays will be traced with the optimized cameras, that will be
intersected with this DEM. Ideally, these rays should meet the DEM at the
same location, so distance between the intersection points measures the amount
of misregistration.

The file::

    {output-prefix}-mapproj_match_offset_stats.txt

will have the percentiles (25%, 50%, 75%, 85%, 95%) of these distances for all
matches in each image against all other images, in meters, and their count.

Do *not* use this option for any initial evaluation of bundle adjustment. 
Inspect instead the files mentioned earlier in :numref:`ba_out_files`. 

This very advanced metric is only helpful if the images are expected to be 
well-registered to each other and to the DEM, which is not the case without 
explicit prior alignment. This also expects a rather accurate DEM, and
for bundle adjustment to be invoked with the option ``--heights-from-dem``
(:numref:`heights_from_dem`).

The 50th percentiles should be on the order of 1 GSD or less.

The file::

    {output-prefix}-mapproj_match_offset_pair_stats.txt

saves such measurements for every pair of images. 

The full report will be saved to::


    {output-prefix}-mapproj_match_offsets.txt

having the longitude, latitude, and height above datum of the midpoint of each
pair of intersection points with the DEM, and the distance between them (in
meters).

This file is very analogous to the ``pointmap.csv`` file, except that
these errors are measured on the ground in meters, and not in the cameras
in pixels. This file can be displayed and colorized in ``stereo_gui``
as a scatterplot (:numref:`plot_csv`).

.. _adjust_files:

Format of .adjust files
~~~~~~~~~~~~~~~~~~~~~~~

The ``bundle_adjust`` program normally saves external adjustments to the input
cameras, or in some cases it creates standalone cameras with adjustments applied
internally (:numref:`ba_use`). 

An external adjustment is stored in a ``.adjust`` file. It has a translation *T*
as *x, y, z* (measured in meters) and a rotation *R* as a quaternion in the
order *w, x, y, z*. The rotation is around the camera center *C* for pixel (0,
0) (for a linescan camera the camera center depends on the pixel). These are
applied on top of initial cameras.

Hence, if *P* is a point in ECEF, that is, the world in which the camera
exists, and an adjustment is applied to the camera, projecting *P* 
in the original camera gives the same result as projecting::

    P' = R * (P - C) + C + T

in the adjusted camera. 

Note that currently the camera center *C* is not exposed in the
``.adjust`` file, so external tools cannot recreate this
transform. This will be rectified at a future time.

Adjustments are relative to the initial cameras, so a nominal
adjustment has the zero translation and identity rotation (quaternion
1, 0, 0, 0).  

.. _ba_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

-o, --output-prefix <filename>
    Prefix for output filenames.

--cost-function <string (default: Cauchy)>
    Choose a cost function from: Cauchy, PseudoHuber, Huber, L1, L2

--robust-threshold <double (default:0.5)>
    Set the threshold for the robust reprojection error cost function.
    Increasing this makes the solver focus harder on the larger errors while
    becoming more sensitive to outliers. See the `Google Ceres
    <http://ceres-solver.org/nnls_modeling.html>`_ documentation on robust cost
    functions.

--datum <string (default: "")>
    Set the datum. This will override the datum from the input images and also
    ``--t_srs``, ``--semi-major-axis``, and ``--semi-minor-axis``. If not set or
    inferred from the images or camera models, the datum will be auto-guessed
    based on camera centers for Earth, Mars, and Moon. Options:

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
    except for xml cameras. See :numref:`ps_options` for
    options.

--min-matches <integer (default: 5)>
    Set the minimum number of matches between images that will be considered.

--max-pairwise-matches <integer (default: 10000)>
    Reduce the number of matches per pair of images to at most this
    number, by selecting a random subset, if needed. This happens
    when setting up the optimization, and before outlier filtering.

--num-iterations <integer (default: 1000)>
    Set the maximum number of iterations.

--parameter-tolerance <double (default: 1e-8)>
    Stop when the relative error in the variables being optimized is less than
    this. When ``--solve-intrinsics`` is used, the default is 1e-12.

--overlap-limit <integer (default: 0)>
    Limit the number of subsequent images to search for matches to
    the current image to this value.  By default try to match all
    images. See also ``--auto-overlap-params``.

--overlap-list <string>
    A file containing a list of image pairs, one pair per line,
    separated by a space, which are expected to overlap. Matches
    are then computed only among the images in each pair. The order 
    in which pairs are specified is not important.

--auto-overlap-params <string (default: "")>
    Determine which camera images overlap by finding the bounding boxes of their
    ground footprints given the specified DEM, expanding them by a given
    percentage, and see if those intersect. A higher percentage should be used
    when there is more uncertainty about the input camera poses. Example:
    'dem.tif 15'. Using this with ``--mapprojected-data`` will restrict the
    matching only on the overlap regions (expanded by this percentage).

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

--tri-weight <double (default: 0.1)>
    The weight to give to the constraint that optimized triangulated points stay
    close to original triangulated points. A positive value will help ensure the
    cameras do not move too far, but a large value may prevent convergence. It
    is suggested to use here 0.1 to 0.5. This will be divided by ground sample
    distance (GSD) to convert this constraint to pixel units, since the
    reprojection errors are in pixels. See also ``--tri-robust-threshold``. Does
    not apply to GCP or points constrained by a DEM.
    
--tri-robust-threshold <double (default: 0.1)>
    The robust threshold to attenuate large differences between initial and
    optimized triangulation points, after multiplying them by ``--tri-weight``
    and dividing by GSD. This is less than ``--robust-threshold``, as the
    primary goal is to reduce pixel reprojection errors, even if that results in
    big differences in the triangulated points. It is suggested to not modify
    this value, and adjust instead ``--tri-weight``.

--camera-position-uncertainty <string (default: "")>
    A file having on each line the image name and the horizontal and vertical
    camera position uncertainty (1 sigma, in meters). This strongly constrains
    the movement of cameras to within the given values, potentially at the
    expense of accuracy. See :numref:`ba_cam_constraints` for details. 
    See also ``--camera-position-uncertainty-power``.

--camera-position-uncertainty-power <double (default: 2.0)>
    A higher value makes the cost function rise more steeply when
    ``--camera-position-uncertainty`` is close to being violated. This is an
    advanced option. The default should be good enough.

--camera-position-weight <double (default: 0.0)>
    A soft constraint to keep the camera positions close to the original values.
    It is meant to prevent a wholesale shift of the cameras. It can impede the
    reduction in reprojection errors. It adjusts to the ground sample distance
    and the number of interest points in the images. The computed discrepancy is
    attenuated with ``--camera-position-robust-threshold``. See
    :numref:`ba_cam_constraints` for details.
 
--camera-position-robust-threshold <double (default: 0.1)>
    The robust threshold to attenuate large discrepancies between initial and
    optimized camera positions with the option ``--camera-position-weight``.
    This is less than ``--robust-threshold``, as the primary goal is to reduce
    pixel reprojection errors, even if that results in big differences in the
    camera positions. It is suggested to not modify this value, and adjust
    instead ``--camera-position-weight``.
       
--rotation-weight <double (default: 0.0)>
    A higher weight will penalize more camera rotation deviations from the
    original configuration.  This adds to the cost function
    the per-coordinate differences between initial and optimized
    normalized camera quaternions, multiplied by this weight, and then
    squared. No robust threshold is used to attenuate this term.

--camera-weight <double (default: 0.0)>
    The weight to give to the constraint that the camera positions/orientations
    stay close to the original values. A higher weight means that the values will
    change less. This option is deprecated. Use instead ``--camera-position-weight``
    and ``--tri-weight``.
        
--ip-per-tile <integer (default: unspecified)>
    How many interest points to detect in each :math:`1024^2` image
    tile (default: automatic determination). This is before matching. 
    Not all interest points will have a match. See also ``--matches-per-tile``.

--ip-per-image <integer>
    How many interest points to detect in each image (default:
    automatic determination). It is overridden by ``--ip-per-tile`` if
    provided.

--ip-detect-method <integer (default: 0)>
    Choose an interest point detection method from: 0 = OBAloG
    (:cite:`jakkula2010efficient`), 1 = SIFT (from OpenCV), 2 = ORB (from
    OpenCV). The SIFT method, unlike OBALoG, produces interest points that are
    accurate to subpixel level. Remove any existing ``.vwip`` files before
    recomputing interest points with a different method. See also
    :numref:`custom_ip`.

--matches-per-tile <int (default: unspecified)>
    How many interest point matches to compute in each image tile (of size
    normally :math:`1024^2` pixels). Use a value of ``--ip-per-tile`` a few
    times larger than this. See an example in :numref:`ba_examples`. See also
    ``--matches-per-tile-params``.

--matches-per-tile-params <int int (default: 1024 1280)>
    To be used with ``--matches-per-tile``. The first value is the image tile
    size for both images. A larger second value allows each right tile to
    further expand to this size, resulting in the tiles overlapping. This may be
    needed if the homography alignment between these images is not great, as
    this transform is used to pair up left and right image tiles.

--inline-adjustments
    If this is set, and the input cameras are of the pinhole or
    panoramic type, apply the adjustments directly to the cameras,
    rather than saving them separately as .adjust files.

--input-adjustments-prefix <string (default: "")>
    Prefix to read initial adjustments from, written by a previous
    invocation of this program.

--isis-cnet <string (default: "")>
    Read a control network having interest point matches from this binary file
    in the ISIS control network format. This can be used with any images and
    cameras supported by ASP. See also ``--output-cnet-type``.

--nvm <string (default: "")>
    Read a control network having interest point matches from this file in the
    NVM format. This can be used with any images and cameras supported by ASP.
    For Pinhole or CSM frame cameras, the (optimized) camera poses will be
    read from / written to NVM as well (:numref:`ba_nvm`). See also
    ``--output-cnet-type``, ``--no-poses-from-nvm``.

--output-cnet-type <string (default: "")>
    The format in which to save the control network of interest point matches.
    Options: ``match-files`` (match files in ASP's format), ``isis-cnet`` (ISIS
    jigsaw format), ``nvm`` (plain text VisualSfM NVM format). If not set, the same
    format as for the input is used.

--no-poses-from-nvm
    Do not read the camera poses from the NVM file or write them to such a file.
    Applicable only with the option ``--nvm`` and Pinhole camera models.
    
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
    When having GCP (or a DEM constraint), constrain the triangulated points in the
    longitude, latitude, and height space, instead of ECEF. The standard deviations
    in the GCP file (or DEM uncertainty) are applied accordingly.

--solve-intrinsics
    Optimize intrinsic camera parameters. Only used for pinhole, optical bar,
    and CSM (frame and linescan) cameras. This implies ``--inline-adjustments``.
    See :numref:`heights_from_dem` for an example.

--intrinsics-to-float <string (default: "")>
    If solving for intrinsics and is desired to float only a few of them,
    specify here, in quotes, one or more of: ``focal_length``,
    ``optical_center``, ``other_intrinsics`` (same as ``distortion``). Not
    specifying anything will float all of them. Also can specify ``all`` or
    ``none``. See :numref:`ba_frame_linescan` for controlling these per
    each group of cameras sharing a sensor.

--intrinsics-to-share <string (default: "")>
    If solving for intrinsics and desired to share only a few of them across all
    cameras, specify here, in quotes, one or more of: ``focal_length``,
    ``optical_center``, ``other_intrinsics`` (same as ``distortion``). By
    default all of the intrinsics are shared, so to not share any of them pass
    in an empty string. Also can specify as ``all`` or ``none``. If sharing
    intrinsics per sensor, this option is ignored, as then the sharing is more
    fine-grained (:numref:`kaguya_ba`).

--intrinsics-limits <arg>
    Set a string in quotes that contains min max ratio pairs for intrinsic
    parameters. For example, "0.8 1.2" limits the parameter to changing by no
    more than 20 percent. The first pair is for focal length, the next two are
    for the center pixel, and the remaining pairs are for other intrinsic
    parameters. If too many pairs are passed in the program will throw an
    exception and print the number of intrinsic parameters the cameras use.
    Cameras adjust all of the parameters in the order they are specified in the
    camera model unless it is specified otherwise in :numref:`pinholemodels`.
    Setting limits can greatly slow down the solver.

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

--remove-outliers-params <'pct factor err1 err2' (default: '75.0 3.0 5.0 8.0')>
    Outlier removal based on percentage, when more than one bundle adjustment
    pass is used.  Triangulated points (that are not GCP) with reprojection
    error in pixels larger than::
    
        min(max(pct-th percentile * factor, err1), err2)
    
    will be removed as outliers.  Hence, never remove pixel
    reprojection errors smaller than ``err1`` but always remove those bigger
    than ``err2``. Specify as a list in quotes. Also remove outliers based on
    distribution of interest point matches and triangulated points.

--elevation-limit <min max>
    Remove as outliers interest points (that are not GCP) for which
    the elevation of the triangulated position (after cameras are
    optimized) is outside of this range. Specify as two values.

--lon-lat-limit <min_lon min_lat max_lon max_lat>
    Remove as outliers interest points (that are not GCP) for which
    the longitude and latitude of the triangulated position (after
    cameras are optimized) are outside of this range.  Specify as
    four values.

--heights-from-dem <string (default: "")>
    Assuming the cameras have already been bundle-adjusted and aligned to a
    known DEM, constrain the triangulated points to be close to the DEM. See
    also ``--heights-from-dem-uncertainty`` and :numref:`heights_from_dem`.

--heights-from-dem-uncertainty <double (default: 10.0)>
    The DEM uncertainty (1 sigma, in meters). A smaller value constrains more the
    triangulated points to the DEM specified via ``--heights-from-dem``.
    The discrepancy between a triangulated point and corresponding point on the 
    DEM is divided by this uncertainty to make it dimensionless, before being
    added to the cost function (:numref:`how_ba_works`). See also 
    ``--heights-from-dem-robust-threshold``.

--heights-from-dem-robust-threshold <double (default: 0.1)> 
    The robust threshold to use keep the triangulated points close to the DEM if
    specified via ``--heights-from-dem``. This is applied after the point
    differences are divided by ``--heights-from-dem-uncertainty``. It will
    attenuate large height difference outliers. It is suggested to not modify
    this value, and adjust instead ``--heights-from-dem-uncertainty``.

--mapproj-dem <string (default: "")>
    If specified, mapproject every pair of matched interest points onto this DEM
    and compute their distance, then percentiles of such distances for each
    image vs the rest and each image pair. This is done after bundle adjustment
    and outlier removal. Measured in meters. See :numref:`ba_mapproj_dem` for
    more details. Not related to ``--mapprojected-data``.

--csv-format <string>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center), 
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-srs``; the height above datum is in
    meters).  Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-srs <string>
    The PROJ or WKT string for interpreting the entries in input CSV
    files.

--update-isis-cubes-with-csm-state
    Save the model state of optimized CSM cameras as part of the .cub
    files. Any prior version and any SPICE data will be deleted.
    Mapprojected images obtained with prior version of the cameras
    must no longer be used in stereo.

--save-adjusted-rpc
    In addition to external adjustments to the input cameras, save RPC cameras
    with the adjustments applied to them, in XML format. This recomputes the RPC
    models (:numref:`rpc_and_ba`).
            
--min-triangulation-angle <degrees (default: 0.1)>
    Filter as outlier any triangulation point for which all rays converging to
    it have an angle less than this (measured in degrees). This happens on
    loading the match files and after each optimization pass. This should be
    used cautiously with very uncertain input cameras. See also
    ``--forced-triangulation-distance`` and ``--max-triangulation-angle``.

--forced-triangulation-distance <meters>
    When triangulation fails, for example, when input cameras are inaccurate or
    the triangulation angle is too small, artificially create a triangulation
    point this far ahead of the camera, in units of meters. Some of these may
    later be filtered as outliers. Can also set a very small value for
    ``--min-triangulation-angle`` in this case.

--ip-num-ransac-iterations <iterations (default: 1000)>
    How many RANSAC iterations to do in interest point matching.

--save-cnet-as-csv
    Save the initial control network containing all interest points in the
    format used by ground control points, so it can be inspected
    (:numref:`stereo_gui_vwip_gcp`). The triangulated points are before
    optimization.

--camera-positions <filename>
    CSV file containing estimated position of each camera, in ECEF
    coordinates. For this to work well the camera must travel not along linear
    path, as this data will be used to find an alignment transform. Only used
    with the inline-adjustments setting to initialize global camera coordinates.
    If used, the csv-format setting must also be set. The "file" field is
    searched for strings that are found in the input image files to match
    locations to cameras. See :numref:`sfmicebridge` for an example.

--init-camera-using-gcp
    Given an image, a pinhole camera lacking correct position and
    orientation, and a GCP file, find the pinhole camera with given
    intrinsics most consistent with the GCP (:numref:`camera_solve_gcp`).

--transform-cameras-with-shared-gcp
    Given at least 3 GCP, with each seen in at least 2 images,
    find the triangulated positions based on pixels values in the GCP,
    and apply a rotation + translation + scale transform to the entire
    camera system so that the triangulated points get
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
    Force reusing the match files even if older than the images or cameras. Then
    the order of images in each interest point match file need not be the same
    as for input images. Additional match files will be created if needed.

--skip-matching
    Only use the match files that be loaded from disk. This implies ``--force-reuse-match-files``. 

--match-files-prefix <string (default: "")>
    Use the match files from this prefix instead of the current output prefix.
    See the naming convention in :numref:`ba_match_files`. This implies
    ``--skip-matching``. The order of images in each interest point match file
    need not be the same as for input images. See also
    ``--clean-match-files-prefix``.

--clean-match-files-prefix <string (default: "")>
    Use as input the ``*-clean.match`` files from this prefix.
    This implies ``--skip-matching``. The order of images in each interest
    point match file need not be the same as for input images.
    Only one of ``--match-files-prefix`` and ``--clean-match-files-prefix``
    can be set.

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
    Do not assume a reliable datum exists, such as for irregularly shaped bodies
    or when at the ground level. This is also helpful when the input cameras are
    not very accurate, as the datum is used to do some camera-based filtering of
    interest points.

--mapprojected-data <string (default: "")>
    Given map-projected versions of the input images and the DEM they were
    mapprojected onto, create interest point matches between the mapprojected
    images, unproject and save those matches, then continue with bundle
    adjustment. Existing match files will be reused. Specify the mapprojected
    images and the DEM as a string in quotes, separated by spaces. The order
    must be same as for the input images. The DEM must be the last file. See
    :numref:`mapip` for an example. It is suggested to use this with
    ``--auto-overlap-params.`` See also ``--mapprojected-data-list``. 
   
--save-intermediate-cameras
    Save the values for the cameras at each iteration.

--apply-initial-transform-only
    Apply to the cameras the transform given by ``--initial-transform``.
    No iterations, GCP loading, image matching, or report generation
    take place. Using ``--num-iterations 0`` and without this option
    will create those.

--image-list
    A file containing the list of images, when they are too many to specify on
    the command line. Use in the file a space or newline as separator. When
    solving for intrinsics for several sensors, pass to this option several
    lists, with comma as separator between the file names (no space). An example
    is in :numref:`kaguya_ba`. See also ``--camera-list`` and
    ``--mapprojected-data-list``.

--camera-list
    A file containing the list of cameras, when they are too many to
    specify on the command line. If the images have embedded camera
    information, such as for ISIS, this file may be omitted, or
    specify the image names instead of camera names.

--mapprojected-data-list
    A file containing the list of mapprojected images and the DEM (see
    ``--mapprojected-data``), when they are too many to specify on the command
    line. The order must be the same as for input images. The DEM must be the
    last entry.

--proj-win
    Flag as outliers input triangulated points not in this proj
    win (box in projected units as provided by ``--proj_str``). This
    should be generous if the input cameras have significant errors.

--proj-str
    To be used in conjunction with  ``--proj-win``.

--weight-image <string (default: "")>
    Given a georeferenced image with float values, for each initial triangulated
    point find its location in the image and closest pixel value. Multiply the
    reprojection errors in the cameras for this point by this weight value. The
    solver will focus more on optimizing points with a higher weight. Points
    that fall outside the image and weights that are non-positive, NaN, or equal
    to nodata will be ignored. See :numref:`limit_ip` for details.

--propagate-errors
    Propagate the errors from the input cameras to the triangulated
    points for all pairs of match points, and produce a report having
    the median, mean, standard deviation, and number of samples for
    each camera pair (:numref:`ba_error_propagation`).

--horizontal-stddev <double (default: 0.0)>
    If positive, propagate this stddev of horizontal ground plane camera
    uncertainty through triangulation for all cameras. To be used with
    ``--propagate-errors``.
   
--epipolar-threshold <double (default: -1)>
    Maximum distance from the epipolar line to search for IP matches.
    If this option isn't given, it will default to an automatic determination.

--ip-inlier-factor <double (default: 0.2)>
    Inlier factor used to remove outliers with homography filtering and RANSAC.
    A higher factor will result in more interest points, but perhaps also more
    outliers.

--ip-uniqueness-threshold <double (default: 0.8)>
    A higher threshold will result in more interest points, but
    perhaps less unique ones.

--nodata-value <double(=NaN)>
    Pixels with values less than or equal to this number are treated
    as no-data. This overrides the no-data values from input images.

--individually-normalize
    Individually normalize the input images instead of using common
    values.

--min-distortion <double (default: 1e-7)>
    When lens distortion is optimized, all initial distortion parameters
    that are smaller in magnitude than this value are set to this value. This is
    to ensure the parameters are big enough to be optimized. Can be negative.
    Applies to Pinhole cameras (all distortion models) and CSM
    (radial-tangential distortion only). Does not apply to optical bar models.

--reference-terrain <filename>
    An externally provided trustworthy reference terrain to use as a constraint.
    It can be either a DEM or a point cloud in CSV format. It must be
    well-aligned with the input cameras (:numref:`reference_terrain`).

--reference-terrain-weight <double (default: 1)>
    How much weight to give to the cost function terms involving
    the reference terrain.

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

--max-triangulation-angle <double (default: -1.0)>
    Filter as outlier any triangulation points for which the maximum angle of
    rays converging to it are more than this (measured in degrees). Set to a
    positive value. See also ``--min-triangulation-angle``.
    
--ip-triangulation-max-error <float>
    When matching IP, filter out any pairs with a triangulation error higher
    than this. The triangulation error is the shortest distance between rays.
    This gets used only when the usual triangulation error filtering fails.

--flann-method <string (default = "auto")>
    Choose the FLANN method for matching interest points. Options: ``kmeans``:
    slower but deterministic, ``kdtree``: faster (up to 6x) but not
    deterministic (starting with FLANN 1.9.2). The default (``auto``) is to use
    ``kmeans`` for 25,000 features or less and ``kdtree`` otherwise. This does
    not apply to ORB feature matching.

--ip-nodata-radius <integer (default: 4)>
    Remove IP near nodata with this radius, in pixels.

--accept-provided-mapproj-dem
    Accept the DEM provided on the command line as the one mapprojection was
    done with, even if it disagrees with the DEM recorded in the geoheaders of
    input images.

--save-vwip
    Save ``.vwip`` files (interest point matches per image, before matching).
    This option is currently ignored as ``.vwip`` are always saved.

--threads <integer (default: 0)>
    Set the number threads to use. 0 means use the default defined
    in the program or in ``~/.vwrc``. Note that when using more
    than one thread and the Ceres option the results will vary
    slightly each time the tool is run.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB, for each process.

--aster-use-csm
    Use the CSM model with ASTER cameras (``-t aster``).
    
-v, --version
    Display the version of software.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
