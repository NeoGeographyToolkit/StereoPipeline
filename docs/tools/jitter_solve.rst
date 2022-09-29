.. _jitter_solve:

jitter_solve
-------------

The ``jitter_solve`` program takes as input several overlapping images
and linescan camera models in CSM format (such as for LRO NAC, CTX,
HiRISE, etc.) and adjusts each individual camera position and orientation
in the linescan model to make them more consistent to each other
and to the ground.

The goal is to reduce the effect of unmeasured perturbations in the
linescan sensor as it acquires the data. This is quite analogous to
what ``bundle_adjust`` does (:numref:`bundle_adjust`), except that the
latter tool has just a single position and orientation per camera,
instead of a sequence of them.

This tool is in development. It gives promising results, but needs
more large-scale testing. 

Usage::

     jitter_solve <images> <cameras> <input adjustments> \
       -o <output prefix> [options]

Example
~~~~~~~

Bundle adjustment is used first to remove any large
misregistration. Here we use a triplet of images acquired with LRO NAC
which have a lot of overlap. See :numref:`lro_nac_no_stitch` for how
to prepare the .cub files, and :numref:`create_csm_linescan` for how
to create CSM camera models.

::

    A=M1101118606LE.cal.echo
    B=M1101097181LE.cal.echo
    C=M1101075756LE.cal.echo

     bundle_adjust --ip-per-image 20000 \
      --max-pairwise-matches 10000      \
      $A.cub $B.cub $C.cub              \
      $A.json $B.json $C.json           \
      -o ba/run  

While one could jump into the jitter-solving step right away, it is
strongly suggested to first align the cameras to a third-party
(perhaps lower-resolution) reference DEM which contains the footprints
of all images. This one can also be used for validation of jitter
correction later.

For that, run ``parallel_stereo`` with the bundle-adjusted
cameras, using a stereo pair with a good convergence angle
(:numref:`stereo_pairs`) among the above such as::

    parallel_stereo left.cub right.cub \
      left.json right.json             \
      --stereo-algorithm asp_mgm       \
      --subpixel-mode 3                \
      --bundle-adjust-prefix ba/run    \
      stereo_ba/run
    point2dem --errorimage stereo_ba/run-PC.tif

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices for stereo.

Align this DEM to the reference DEM, which will call ``ref.tif``, with
``pc_align`` (:numref:`pc_align`). Likely ``ref.tif`` should be the
second argument in that alignment call, as it is perhaps at
lower-resolution.  Care is needed with the ``--max-displacement``
option and the choice of the alignment method.

Apply the alignment transform to the cameras (:numref:`ba_pc_align`),
obtaining the directory ``ba_align`` having the aligned camera
adjustments. Then solve for jitter::

    jitter_solve $A.cub $B.cub $C.cub         \
      $A.json $B.json $C.json                 \
      --input-adjustments-prefix ba_align/run \
      --clean-match-files-prefix ba/run       \
      --max-initial-reprojection-error 100    \
      --num-iterations 100                    \
      --quat-norm-weight 5                    \
      --anchor-weight 0.1                     \
      --heights-from-dem dem.tif              \
      --heights-from-dem-weight 0.5           \
      --heights-from-dem-robust-threshold 0.5 \
      -o jitter/run

If in doubt about the quality of that reference DEM or of alignment,
lower the values specified via ``--heights-from-dem-weight``
and ``--heights-from-dem-robust-threshold``.

The model states (:numref:`csm_state`) of optimized cameras are saved
with names like::

    jitter/run-*.adjusted_state.json

The anchor points
~~~~~~~~~~~~~~~~~

The most important purpose of the reference DEM is its use it with the
``--anchor-weight`` option.  Before this solver starts, rays are
traced from a uniformly sampled set of pixels in each camera and
intersect that DEM. The reprojection errors of those DEM "anchor"
points is at this stage zero, by definition. The reprojection errors
for the anchor points, multiplied by the anchor weight, are then added
to the optimization cost function. The goal of these terms is to
prevent the cameras from moving too far. This cost function is not too
sensitive to whether the DEM is precisely aligned with the cameras.

What will actually drive the optimization are the reprojection errors
from 3D points obtained by triangulating rays emanating from
interest point matches in the cameras. These are zero if the rays
perfectly intersect, and the magnitude of these errors is proportional
to how bad the jitter is.

Ideally, this solver will result in self-consistent cameras, so lower
reprojection errors based on interest points, with the anchor weight
preventing the solution from going wild, so reprojection errors
for the anchor points would increase only somewhat.

Having at least 3 (and ideally more) overlapping images and a
well-aligned input DEM will result in a more accurate solution.

Note that above the clean interest point matches created by bundle
adjustment are used.

Validation
~~~~~~~~~~

Create DEMs with the bundle-adjusted and aligned cameras (adjustments
are in ``ba_align``), that is, before solving for jitter, and after it
(cameras are in ``jitter``).  For that, resume stereo at the
triangulation stage in both cases::

    parallel_stereo left.cub right.cub     \
      left.json right.json                 \
      --prev-run-prefix stereo_ba/run      \
      --stereo-algorithm asp_mgm           \
      --subpixel-mode 3                    \
      --bundle-adjust-prefix ba_align/run  \
      stereo_ba_align/run
    point2dem --errorimage stereo_ba_align/run-PC.tif

    parallel_stereo left.cub right.cub     \
      jitter/run-left.adjusted_state.json  \
      jitter/run-right.adjusted_state.json \
      --stereo-algorithm asp_mgm           \
      --subpixel-mode 3                    \
      --prev-run-prefix stereo_ba/run      \
      stereo_jitter/run
    point2dem --errorimage stereo_jitter/run-PC.tif

Colorize the obtained error images ``stereo*/*IntersectionErr.tif`` using
``colormap`` (:numref:`colormap`) with same min and max values, then
overlay them them in ``stereo_gui`` (:numref:`stereo_gui`).

This should show if the intersection error went down, which correlates with
the jitter effect being reduced.

Validate the obtained aligned DEMs against the preexisting DEM
``ref.tif``.  First this can be done visually by hillshading them in
``stereo_gui`` (:numref:`stereo_gui`) and looking for any obvious
shifts, then absolute differences of these DEM can be found with
``geodiff --absolute`` (:numref:`geodiff`), which can be colorized
with ``colormap`` (:numref:`colormap`) with same min and max values,
and these can also be overlayed in ``stereo_gui``.

Otherwise, compare to a sparse dataset like MOLA or LOLA. It is
assumed that this dataset is aligned as well to the reference DEM and
produced DEMs.  In this case, the ``geodiff`` tool can be used for
differencing as before, and the produced errors can be gridded with
the ``--csv-format`` option of ``point2dem`` (:numref:`point2dem`) and
colorized as before.

.. _jitter_out_files:

Output files
~~~~~~~~~~~~

The optimized CSM model state files (:numref:`csm_state`), which
hopefully reduce the jitter and also incorporate the initial
adjustments as well, are saved in the output directory, which in the
example above is named ``jitter``. 

This program will write, just like ``bundle_adjust`` (:numref:`ba_out_files`),
the triangulated world position for every feature being matched in two
or more images, and the mean absolute residuals (reprojection errors)
for each position, before and after optimization. The files are named

::

     {output-prefix}-initial_residuals_pointmap.csv

and

::

     {output-prefix}-final_residuals_pointmap.csv

Such files can be inspected to see at which pixels the residual error
is large. They can also be gridded with ``point2dem`` as above. For
the height field one can pick the 4th column in these files, which has
the residuals.


Command-line options for jitter_solve
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

--overlap-limit <integer (default: 0)>
    Limit the number of subsequent images to search for matches to
    the current image to this value.  By default try to match all
    images.

--match-first-to-last
    Match the first several images to last several images by extending
    the logic of ``--overlap-limit`` past the last image to the earliest
    ones.

--input-adjustments-prefix <string>
    Prefix to read initial adjustments from, written by ``bundle_adjust``.
    Not required. Cameras in .json files in ISD or model state format
    can be passed in with no adjustments. 

--quat-norm-weight <double (default: 1.0)>
    How much weight to give to the constraint that the norm of each
    quaternion must be 1.

--anchor-weight <double (default: 1.0)>
    How much weight to give to each anchor point. Anchor points are
    obtained by intersecting rays from initial cameras with the DEM
    given by ``--heights-from-dem``. A larger weight will make it
    harder for the cameras to move, hence preventing unreasonable
    changes.

--rotation-weight <double (default: 0.0)>
    A higher weight will penalize more deviations from the
    original camera orientations.

--translation-weight <double (default: 0.0)>
    A higher weight will penalize more deviations from
    the original camera positions.

--heights-from-dem <string>
    If the cameras have already been bundle-adjusted and aligned
    to a known high-quality DEM, in the triangulated xyz points
    replace the heights with the ones from this DEM, and fix those
    points unless ``--heights-from-dem-weight`` is positive.

--heights-from-dem-weight <double (default: 1.0)>
    How much weight to give to keep the triangulated points close
    to the DEM if specified via ``--heights-from-dem``. This value
    should be inversely proportional with ground sample distance, as
    then it will convert the measurements from meters to pixels, which
    is consistent with the reprojection error term.

--heights-from-dem-robust-threshold <double (default: 0.5)> 
    The robust threshold to use keep the
    triangulated points close to the DEM if specified via
    ``--heights-from-dem``. This is applied after the point differences
    are multiplied by ``--heights-from-dem-weight``. It should
    help with attenuating large height difference outliers.

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

--match-files-prefix <string (default: "")>
    Use the match files from this prefix.

--clean-match-files-prefix <string (default: "")>
    Use as input match files the \*-clean.match files from this prefix.

--max-initial-reprojection-error <integer (default: 10)> 
    Filter as outliers triangulated points project using initial cameras with 
    error more than this, measured in pixels. Since jitter corrections are 
    supposed to be small and cameras bundle-adjusted by now, this value 
    need not be too big.

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

