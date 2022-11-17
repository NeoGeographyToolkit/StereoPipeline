.. _jitter_solve:

jitter_solve
-------------

The ``jitter_solve`` program takes as input several overlapping images
and linescan camera models in CSM format (such as for LRO NAC, CTX,
HiRISE, Airbus Pleiades, etc.) and adjusts each individual camera position
and orientation in the linescan model to make them more consistent to
each other and to the ground.

The goal is to reduce the effect of unmeasured perturbations in the
linescan sensor as it acquires the data. This is quite analogous to
what ``bundle_adjust`` does (:numref:`bundle_adjust`), except that the
latter tool has just a single position and orientation per camera,
instead of a sequence of them.

Usage::

     jitter_solve <images> <cameras> <input adjustments> \
       -o <output prefix> [options]

Ground constraints
~~~~~~~~~~~~~~~~~~

While the cameras are adjusted to reduce the jitter and make them
self-consistent, this can result in the camera system moving away from
the initial location, or perhaps in warping in any eventually produced
DEM.

Hence, ground constraints are very important. This tool uses several kinds
of constraints. 

The first is an intrinsic one, in which triangulated ground points are
kept, during optimization, close to their initial values.  This works
well when the images have very good overlap, and ideally there are
more than two of them. Its weight is given by ``--tri-weight``. See
:numref:`jitter_options` for more details.

The second kind of ground constraint ties the solution to an external
DEM, which may be at a lower resolution than the images. It is then
expected that this external DEM is *well-aligned* with the input
cameras. This option is named ``--heights-from-dem``, and it is
controlled via ``--heights-from-dem-weight`` and
``--heights-from-dem-robust-threshold``.

The second approach is preferred and ensures the DEM obtained
later with jitter-corrected cameras is consistently close to 
the external DEM.

To tie to the ground portions of images which do not overlap
with other images, one can use the options ``--num-anchor-points``,
``--anchor-weight``, and ``--anchor-dem``.

Resampling the poses
~~~~~~~~~~~~~~~~~~~~

Often times, the number of tabulated camera positions and orientations
in the CSM file is very small. For example, for Airbus Pleiades, the
position is sampled every 30 seconds, while acquiring the whole image
can take only 1.6 seconds. For CTX the opposite problem happens, the
orientations are sampled too finely, resulting in too many variables
to optimize.

Hence, it is strongly suggested to resample the provided positions and
orientations before the solver optimizes them. Use the options:
``--num-lines-per-position`` and ``--num-lines-per-orientation``. The
estimated number of lines per position and orientation will be printed
on screen, before and after resampling.

It is suggested to use perhaps 1000 lines per position and
orientation.

Example 
~~~~~~~

A CTX stereo pair will be used which has quite noticeable jitter.
See :numref:`jitter_multiple_images` for a discussion of multiple images.

Input images
^^^^^^^^^^^^

The pair consists of images with ids::

    J03_045820_1915_XN_11N210W
    K05_055472_1916_XN_11N210W

See :numref:`ctx_example` for how to prepare the image files and
:numref:`create_csm_linescan` for how to create CSM camera models.

All produced images and cameras were stored in a directory named
``img``.

Reference datasets
^^^^^^^^^^^^^^^^^^

The MOLA dataset from:

    https://ode.rsl.wustl.edu/mars/datapointsearch.aspx

is used for alignment. The data for the following (very generous)
longitude-latitude extent was fetched: 146E to 152E, and 7N to 15N.
The obtained CSV file was saved as ``mola.csv``.

A gridded DEM produced from this unorganized set of points
is shipped with the ISIS data. It is gridded at 463 meters
per pixel, which is quite coarse compared to CTX images,
which are at 6 m/pixel, but it is good enough to constrain
the cameras when solving for jitter. A clip can be cut out of 
it with the command::

    gdal_translate -co compress=lzw -co TILED=yes              \
     -co INTERLEAVE=BAND -co BLOCKXSIZE=256 -co BLOCKYSIZE=256 \
     -projwin -2057237.6 1077503.1 -1546698.4 275566.33        \
     $ISISDATA/base/dems/molaMarsPlanetaryRadius0005.cub       \
     ref_dem_shift.tif

This one has a 190 meter vertical shift relative to the preferred Mars
radius of 3396190 meters, which can be removed as follows::

    image_calc -c "var_0-190" -d float32 ref_dem_shift.tif \
      -o ref_dem.tif

As a sanity check, one can take the absolute difference of this DEM
and the MOLA csv file as::

    geodiff --absolute --csv-format 1:lon,2:lat,5:radius_m \
      mola.csv ref_dem.tif

This will give a median difference of 3 meters, which is about right,
given the uncertainties in these datasets.

Uncorrected DEM creation
^^^^^^^^^^^^^^^^^^^^^^^^

Bundle adjustment is run first::

    bundle_adjust                               \
      --ip-per-image 20000                      \
      --max-pairwise-matches 10000              \
      --tri-weight 0.05                         \
      --camera-weight 0                         \
      --remove-outliers-params '75.0 3.0 20 20' \
      img/J03_045820_1915_XN_11N210W.cal.cub    \
      img/K05_055472_1916_XN_11N210W.cal.cub    \
      img/J03_045820_1915_XN_11N210W.cal.json   \
      img/K05_055472_1916_XN_11N210W.cal.json   \
      -o ba/run

The triangulation weight was used to help the cameras from drifting.
Outlier removal was allowed to be more generous (hence the values of
20 pixels above) as perhaps due to jitter some triangulated points
obtained from interest point matches may not project perfectly in the
cameras.

Stereo is run next. The ``local_epipolar`` alignment
(:numref:`running-stereo`) here did a flawless job, unlike
``affineepipolar`` alignment which resulted in some blunders.
::

    parallel_stereo                           \
      --bundle-adjust-prefix ba/run           \
      --stereo-algorithm asp_mgm              \
      --num-matches-from-disp-triplets 40000  \
      --alignment-method local_epipolar       \
      img/J03_045820_1915_XN_11N210W.cal.cub  \
      img/K05_055472_1916_XN_11N210W.cal.cub  \
      img/J03_045820_1915_XN_11N210W.cal.json \
      img/K05_055472_1916_XN_11N210W.cal.json \
      stereo/run
    point2dem --errorimage stereo/run-PC.tif

Note how above we chose to create dense interest point matches from
disparity. They will be used to solve for jitter. The option used
``--num-matches-from-disp-triplets`` can be very slow for images
larger than 50,000 pixels on the side, or so. Then, use
``--num-matches-from-disparity``.

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices for stereo. Close to the poles a polar
stereographic projection may be preferred in ``point2dem``
(:numref:`point2dem`).

This DEM was aligned to MOLA and recreated, as::

    pc_align --max-displacement 400           \
      --csv-format 1:lon,2:lat,5:radius_m     \
      stereo/run-DEM.tif mola.csv             \
      --save-inv-transformed-reference-points \
      -o stereo/run-align
    point2dem stereo/run-align-trans_reference.tif

The value in ``--max-displacement`` may need tuning
(:numref:`pc_align`).

This transform was applied to the cameras, to make them aligned to
MOLA (:numref:`ba_pc_align`)::

    bundle_adjust                                                \
      --input-adjustments-prefix ba/run                          \
      --initial-transform stereo/run-align-inverse-transform.txt \
      img/J03_045820_1915_XN_11N210W.cal.cub                     \
      img/K05_055472_1916_XN_11N210W.cal.cub                     \
      img/J03_045820_1915_XN_11N210W.cal.json                    \
      img/K05_055472_1916_XN_11N210W.cal.json                    \
      --apply-initial-transform-only                             \
    -o ba_align/run

Solving for jitter
^^^^^^^^^^^^^^^^^^

Then, jitter was solved for, using the aligned cameras::

    jitter_solve                               \
      img/J03_045820_1915_XN_11N210W.cal.cub   \
      img/K05_055472_1916_XN_11N210W.cal.cub   \
      img/J03_045820_1915_XN_11N210W.cal.json  \
      img/K05_055472_1916_XN_11N210W.cal.json  \
      --input-adjustments-prefix ba_align/run  \
      --max-pairwise-matches 100000            \
      --match-files-prefix stereo/run-disp     \
      --num-lines-per-position    1000         \
      --num-lines-per-orientation 1000         \
      --max-initial-reprojection-error 20      \
      --heights-from-dem ref_dem.tif           \
      --heights-from-dem-weight 0.05           \
      --heights-from-dem-robust-threshold 0.05 \
      --num-iterations 50                      \
      --anchor-weight 0                        \
      --tri-weight 0                           \
    -o jitter/run

It was found that using about 1000 lines per pose (position and
orientation) sample gave good results, and if using too few lines, the
poses become noisy. Dense interest point matches appear necessary for
a good result, though perhaps the number produced during stereo could
be lowered.

The constraint relative to the reference DEM is needed, to make sure
the DEM produced later agrees with the reference one.  Otherwise, the
final solution may not be unique, as a long-wavelength perturbation
consistently applied to all obtained camera trajectories may work just
as well.

The model states (:numref:`csm_state`) of optimized cameras are saved
with names like::

    jitter/run-*.adjusted_state.json

Then, stereo can be redone, just at the triangulation stage, which
is much faster than doing it from scratch. The optimized cameras were
used::

    parallel_stereo                                                 \
      --prev-run-prefix stereo/run                                  \
      --stereo-algorithm asp_mgm                                    \
      --alignment-method local_epipolar                             \
      img/J03_045820_1915_XN_11N210W.cal.cub                        \
      img/K05_055472_1916_XN_11N210W.cal.cub                        \
      jitter/run-J03_045820_1915_XN_11N210W.cal.adjusted_state.json \
      jitter/run-K05_055472_1916_XN_11N210W.cal.adjusted_state.json \
      stereo_jitter/run
      point2dem --errorimage stereo_jitter/run-PC.tif

To validate the results, first the ray intersection error
(:numref:`point2dem`) was plotted, before and after solving for
jitter. These were colorized as::

    colormap --min 0 --max 10 stereo/run-IntersectionErr.tif
    colormap --min 0 --max 10 stereo_jitter/run-IntersectionErr.tif

The result is below.

.. figure:: ../images/jitter_intersection_error.png
   :name: ctx_jitter_intersection_error

   The colorized intersection error (max shade of red is 10 m)
   before and after optimization for jitter.

Then, the absolute difference was computed between the sparse MOLA
dataset and the DEM after alignment and before solving for jitter, and
the same was done with the DEM produced after solving for it::

    geodiff --absolute                                  \
      --csv-format 1:lon,2:lat,5:radius_m               \
      stereo/run-align-trans_reference-DEM.tif mola.csv \
      -o stereo/run

    geodiff --absolute                                  \
      --csv-format 1:lon,2:lat,5:radius_m               \
      stereo_jitter/run-DEM.tif mola.csv                \
      -o stereo_jitter/run

Similar commands are used to find differences with the
reference DEM::

    geodiff --absolute ref_dem.tif                \
      stereo/run-align-trans_reference-DEM.tif -o \
      stereo/run
    colormap --min 0 --max 20 stereo/run-diff.tif

    geodiff --absolute ref_dem.tif                \
      stereo_jitter/run-DEM.tif                   \
      -o stereo_jitter/run
    colormap --min 0 --max 20 stereo_jitter/run-diff.tif

Plot with::

    stereo_gui --colorize --min 0 --max 20 \
       stereo/run-diff.csv                 \
       stereo_jitter/run-diff.csv          \
       stereo/run-diff_CMAP.tif            \
       stereo_jitter/run-diff_CMAP.tif     \
       stereo_jitter/run-DEM.tif           \
       ref_dem.tif

DEMs can later be hillshaded. 

.. figure:: ../images/jitter_dem_diff.png
   :name: ctx_jitter_dem_diff_error

   From left to right are shown colorized absolute differences of
   (a) jitter-unoptimized but aligned DEM and MOLA (b)
   jitter-optimized DEM and MOLA
   (c) unoptimized DEM and reference DEM (d) jitter-optimized
   DEM and reference DEM. Then, (e) hillshaded optimized DEM (f)
   hillshaded reference DEM . The max shade of red is 20 m difference.

It can be seen that the banded systematic error due to jitter is gone,
both in the intersection error maps and DEM differences. The produced
DEM still disagrees somewhat with the reference, but we believe that
this is due to the reference DEM being very coarse, per plots (e) and
(f) in the figure.

.. _jitter_multiple_images:

Using multiple images
^^^^^^^^^^^^^^^^^^^^^

At a future time an analysis can be done where more images
for that area are used. The following overlap with the above pair
quite well::

    B19_016902_1913_XN_11N210W
    F04_037367_1929_XN_12N211W
    N14_067737_1928_XI_12N210W
    P06_003347_1894_XI_09N210W

Bundle adjustment can be run on all of them, and pairwise DEMs can be
created from the pairs with a convergence angle between 10 and 30 degrees
(``bundle_adjust`` saves the list of convergence angles). 

Then, the obtained DEMs could be merged with ``dem_mosaic``, which
will hopefully result in a solid high-resolution reference DEM due to
jitter canceling out.  Then, jitter could be solved either
simultaneously for all these, or in pairs, and the logic in the 
earlier example could be repeated, but with a higher quality reference
DEM.

.. _jitter_out_files:

Output files
~~~~~~~~~~~~

The optimized CSM model state files (:numref:`csm_state`), which
reduce the jitter and also incorporate the initial
adjustments as well, are saved in the output directory, which in the
example above is named ``jitter``. 

This program will write, just like ``bundle_adjust`` (:numref:`ba_out_files`),
the triangulated world position for every feature being matched in two
or more images, and the mean absolute residuals (reprojection errors)
for each position, before and after optimization. The files are named::

     {output-prefix}-initial_residuals_pointmap.csv

and::

     {output-prefix}-final_residuals_pointmap.csv

Such CSV files can be colorized and overlaid with ``stereo_gui``
(:numref:`plot_pointmap`) to see at which pixels the residual error is
large.

If anchor points are used, the pixel residuals at those points are saved
as well, to::

     {output-prefix}-initial_residuals_anchor_points.csv

and::

     {output-prefix}-final_residuals_anchor_points.csv

Each of those is the norm of reprojection error in the camera for an
anchor point. When being optimized, those reprojection errors are
multiplied by the anchor weight. In this file they are saved without
that weight multiplier.

These can be plotted colorized in ``stereo_gui`` as well,
for example, with::

    stereo_gui --colorize --min 0 --max 0.5 --plot-point-radius 2 \
      {output-prefix}-final_residuals_anchor_points.csv

.. _jitter_options:

Command-line options for jitter_solve
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-o, --output-prefix <filename>
    Prefix for output filenames.

-t, --session-type <string>
    Select the stereo session type to use for processing. Usually
    the program can select this automatically by the file extension, 
    except for xml cameras. See :numref:`parallel_stereo_options` for
    options.

--robust-threshold <double (default:0.5)>
    Set the threshold for robust cost functions. Increasing this
    makes the solver focus harder on the larger errors.

--min-matches <integer (default: 30)>
    Set the minimum number of matches between images that will be
    considered.

--max-pairwise-matches <integer (default: 10000)>
    Reduce the number of matches per pair of images to at most this
    number, by selecting a random subset, if needed. This happens
    when setting up the optimization, and before outlier filtering.

--num-iterations <integer (default: 100)>
    Set the maximum number of iterations.

--parameter-tolerance <double (default: 1e-8)>
    Stop when the relative error in the variables being optimized
    is less than this.

--input-adjustments-prefix <string>
    Prefix to read initial adjustments from, written by ``bundle_adjust``.
    Not required. Cameras in .json files in ISD or model state format
    can be passed in with no adjustments. 

--num-lines-per-position
    Resample the input camera positions and velocities, using this
    many lines per produced position and velocity. If not set, use the
    positions and velocities from the CSM file as they are.

--num-lines-per-orientation
    Resample the input camera orientations, using this many lines per
    produced orientation. If not set, use the orientations from the
    CSM file as they are.

--tri-weight <double (default: 0.0)>
    The weight to give to the constraint that optimized triangulated
    points stay close to original triangulated points. A positive
    value will help ensure the cameras do not move too far, but a
    large value may prevent convergence. Does not apply to GCP or
    points constrained by a DEM. This adds a robust cost function 
    with the threshold given by ``--robust-threshold``. 
    The suggested value is 0.1 to 0.5 divided by the image ground
    sample distance.

--heights-from-dem <string>
    If the cameras have already been bundle-adjusted and aligned
    to a known DEM, in the triangulated points obtained from 
    interest point matches replace the heights with the ones from this
    DEM before optimizing them while tying the points to this DEM via
    ``--heights-from-dem-weight`` and
    ``--heights-from-dem-robust-threshold``.

--heights-from-dem-weight <double (default: 0.5)>
    How much weight to give to keep the triangulated points close
    to the DEM if specified via ``--heights-from-dem``. This value
    should be about 0.1 to 0.5 divided by the image ground sample
    distance, as then it will convert the measurements from meters to
    pixels, which is consistent with the pixel reprojection error term.

--heights-from-dem-robust-threshold <double (default: 0.5)> 
    The robust threshold to use keep the triangulated points close to
    the DEM if specified via ``--heights-from-dem``. This is applied
    after the point differences are multiplied by
    ``--heights-from-dem-weight``. It should help with attenuating
    large height difference outliers. It is suggested to make this 
    equal to ``--heights-from-dem-weight``.

--match-files-prefix <string (default: "")>
    Use the match files from this prefix. Matches are typically dense
    ones produced by stereo or sparse ones produced by bundle
    adjustment.

--clean-match-files-prefix <string (default: "")>
    Use as input match files the \*-clean.match files from this
    prefix.

--max-initial-reprojection-error <integer (default: 10)> 
    Filter as outliers triangulated points project using initial cameras with 
    error more than this, measured in pixels. Since jitter corrections are 
    supposed to be small and cameras bundle-adjusted by now, this value 
    need not be too big.

--num-anchor-points <integer (default: 0)>
    How many anchor points to create tying each pixel to a point on
    a DEM along the ray from that pixel to the ground. These points
    will be uniformly distributed across each input image. (This is
    being tested.) Set also ``--anchor-weight`` and ``--anchor-dem``.

--anchor-weight <double (default: 0.0)>
    How much weight to give to each anchor point. Anchor points are
    obtained by intersecting rays from initial cameras with the DEM
    given by ``--heights-from-dem``. A larger weight will make it
    harder for the cameras to move, hence preventing unreasonable
    changes.

--anchor-dem <string (default: "")>
    Use this DEM to create anchor points.

--quat-norm-weight <double (default: 1.0)>
    How much weight to give to the constraint that the norm of each
    quaternion must be 1.

--rotation-weight <double (default: 0.0)>
    A higher weight will penalize more deviations from the
    original camera orientations.

--translation-weight <double (default: 0.0)>
    A higher weight will penalize more deviations from
    the original camera positions.

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

--min-triangulation-angle <degrees (default: 0.1)>
    The minimum angle, in degrees, at which rays must meet at a
    triangulated point to accept this point as valid. It must
    be a positive value.

--overlap-limit <integer (default: 0)>
    Limit the number of subsequent images to search for matches to
    the current image to this value.  By default try to match all
    images.

--match-first-to-last
    Match the first several images to last several images by extending
    the logic of ``--overlap-limit`` past the last image to the earliest
    ones.

--threads <integer (default: 0)>
    Set the number threads to use. 0 means use the default defined
    in the program or in ``~/.vwrc``. Note that when using more
    than one thread and the Ceres option the results will vary
    slightly each time the tool is run.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB, for each process.

-h, --help
    Display the help message.

-v, --version
    Display the version of software.


.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN

