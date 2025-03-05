.. _pc_align:

pc_align
--------

The ``pc_align`` programs aligns two point clouds. The supported algorithms are
Iterative Closest Point (:numref:`pc_icp`), Nuth and Kaab (:numref:`nuth`), Fast
Global Registration (:numref:`fgr`), and feature-based alignment
(:numref:`pc_hillshade`).

Some of the provided ICP implementations can handle a scale change, in addition
to rotations and translations. 

Usage::

     pc_align --max-displacement <float> [other options]    \
       <reference cloud> <source cloud> -o <output prefix>}

The denser cloud must be the first one to be passed to this tool. This
program is very sensitive to the value of ``--max-displacement``
(:numref:`pc_align_max_displacement`).

An example is in :numref:`pc-align-example`. Validation and error metrics are
discussed in :numref:`pc_align_validation` and :numref:`pc_align_error`.

See the related program ``image_align`` (:numref:`image_align`).

Several important things need to be kept in mind if ``pc_align`` is to
be used successfully and give accurate results, as described below.

The input point clouds
~~~~~~~~~~~~~~~~~~~~~~

Due to the nature of ICP, the first input point cloud, that is, the
reference (fixed) cloud, should be denser than the second, source
(movable) point cloud, to get the most accurate results. This is not a
serious restriction, as one can perform the alignment this way and then
simply invert the obtained transform if desired (``pc_align`` outputs
both the direct and inverse transform, and can output the reference
point cloud transformed to match the source and vice-versa).

The user can choose how many points to pick from the reference and
source point clouds to perform the alignment. The amount of memory and
processing time used by ``pc_align`` is directly proportional to these
numbers, ideally the more points the better. Pre-cropping to judiciously
chosen regions may improve the accuracy and/or run-time.

.. _pc_align_max_displacement:

The max displacement option
~~~~~~~~~~~~~~~~~~~~~~~~~~~

In many typical applications, the source and reference point clouds are
already roughly aligned, but the source point cloud may cover a larger
area than the reference. The user should provide to ``pc_align`` the
expected maximum distance (displacement) source points may move by as
result of alignment, using the option ``--max-displacement``. 

This number will help remove source points too far from the reference point
cloud which may not match successfully and may degrade the accuracy. If
in doubt, this value can be set to something large but still reasonable,
as the tool is able to throw away a certain number of unmatched
outliers. 

At the end of alignment, ``pc_align`` will display the
*observed* maximum displacement, a multiple of which can be used to seed
the tool in a subsequent run. If an initial transform is applied to the
source cloud (:numref:`prevtrans`), the outliers are thrown
out *after* this operation. The observed maximum displacement is also
between the source points with this transform applied and the source
points after alignment to the reference.

.. _align-method:

Alignment method
~~~~~~~~~~~~~~~~

The alignment method can be set with the option ``--alignment-method``
(:numref:`pc_align_options`). The default is ``point-to-plane`` ICP.

.. _pc_icp:

ICP algorithms
^^^^^^^^^^^^^^

ASP provides several flavors of the Iterative Closest Point (ICP) algorithm,
with the implementation given by the `libpointmatcher
<https://github.com/ethz-asl/libpointmatcher>`_ library
(:cite:`Pomerleau12comp`).
    
The default alignment method is Point-to-Plane ICP, which may be more
robust to large translations than Point-to-Point ICP, though the latter
can be good enough if the input point clouds have small alignment errors
and it is faster and uses less memory as well. The tool also accepts an
option named ``--highest-accuracy`` which will compute the normals for
Point-to-Plane ICP at all points rather than about a tenth of them. This
option is not necessary most of the time, but may result in better
alignment at the expense of using more memory and processing time.

The default alignment transform is rigid, that is, a combination of rotation and
translation. It is also possible to solve for a scale change, by setting
``--alignment-method`` to ``similarity-point-to-plane`` or
``similarity-point-to-point``. The first of these works better than the second
one.

If the translation between the point clouds is very large, see
:numref:`pc_hillshade`.

.. _nuth:

Nuth and Kaab
^^^^^^^^^^^^^

The Nuth and Kaab alignment method (:cite:`nuth2011co`) can be sub-grid-size
accurate. It is accessible with ``--alignment-method nuth``. The implementation
is based on `dem_align.py  <https://github.com/dshean/demcoreg>`_.

It is assumed that:

  - The input clouds are dense and detailed DEMs with notable relief
  - The DEMs have a lot of overlap
  - The alignment transform is a pure translation in projected coordinates (both
    horizontal and vertical).

If the last two assumptions do not hold, consider using a different alignment
algorithm first (for example, feature-based alignment in combination with ICP,
:numref:`pc_hillshade`). The resulting aligned source point cloud needs to be
regridded with ``point2dem``, and then the alignment further refined with this
method.

The order of inputs should be so that the the reference DEM (the first input)
has a grid size that is no bigger than of the second DEM. The second DEM
will be interpolated to the grid of the first one.

Both DEMs should be in projected coordinates, so with the grid size measured in
meters, and with the same datum. Otherwise, regridding can be done with
``gdalwarp -r cubic`` (:numref:`gdal_tools`). LAS files can be regridded with
``point2dem`` (:numref:`point2dem`).

The produced alignment transform will be converted to a rotation + translation
transform around the planet center (ECEF coordinates), for consistency with the
other alignment methods. It will be an ECEF translation if the option
``--compute-translation-only`` is set.

The DEMs should fit fully in memory, with a solid margin. 

Large DEMs with good relief could be regridded (with cubic interpolation) to a
2x coarser grid, which would still result in a good alignment. That goes as
follows, for any input DEM::

  gdal_translate -r average -outsize 50% 50% input.tif output.tif

Any produced transform with lower-resolution DEMs can be applied to the original
DEMs (:numref:`prevtrans`).

Additional options can be passed in via ``--nuth-options``
(:numref:`nuth_options`).

This alignment method does not support the ``--initial-transform`` option,
because it computes the alignment transform in projected coordinates of the
reference DEM, and in that space an external ECEF transform cannot be applied
exactly.

.. _fgr:

FGR algorithm
^^^^^^^^^^^^^

The `Fast Global Registration
<https://github.com/IntelVCL/FastGlobalRegistration>`_ (FGR) algorithm can be
called with ``--alignment-method fgr``, and is customizable via
``--fgr-options`` (:numref:`pc_align_options`).

This approach can perform better than ICP when the clouds are close enough to
each other but there is a large number of outliers, since it does a cross-check.

When the clouds are far, another algorithm can be employed to bring them 
closer first (:numref:`prevtrans`).

.. _pc_hillshade:

Feature-based alignment
^^^^^^^^^^^^^^^^^^^^^^^

If the clouds differ by a large translation or scale factor, alignment can fail.
If the clouds are DEMs, one may specify the option
``--initial-transform-from-hillshading`` which will hillshade the two
DEMs, find interest point matches among them, and use that to compute an initial
transform between the clouds, which may or may not contain scale.

This transform can be passed as an initial guess to the other alignment
algorithms (:numref:`prevtrans`). See an example in :numref:`kh4_align`. 

The related correlation-based alignment method is described in
:numref:`pc_corr`.
 
This functionality is implemented with ASP's ``hillshade``, ``ipfind``, and
``ipmatch`` tools. The ``pc_align`` options ``--hillshade-options``,
``--ipfind-options``, and ``--ipmatch-options`` can be used to pass options to
to these programs, such as to increase the number interest points being found,
if the defaults are not sufficient. See :numref:`pc_align_options`.

The match file having the correspondences between the two hillshaded DEMs is
saved in the output directory and can be inspected. It can also be created
or edited manually (:numref:`manual-align`).

If the two clouds look too different for interest point matching to work, they
perhaps can be re-gridded to use the same (coarser) grid, as described in
:numref:`regrid`. The produced transform will be applicable to the original
clouds.

.. _pc_corr:

Correlation-based alignment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given two DEMs with the same grid size that look visually similar when
hillshaded, the dense image correlation can be found between the hillshaded
images, and that can be employed to align the clouds. That goes as follows.

Hillshading happens with the ``hillshade`` program (:numref:`hillshade`)::

    hillshade ref.tif -o ref_hill.tif
    hillshade src.tif -o src_hill.tif

Image correlation is performed (:numref:`correlator-mode`)::

    parallel_stereo --correlator-mode    \
      --ip-per-image 40000               \
      ref_hill.tif src_hill.tif          \
      --num-matches-from-disparity 40000 \
      run_corr/run

Stereo correlation can take a long time. It can be run over several nodes
(:numref:`pbs_slurm`). The option ``--max-disp-spread`` can help with reducing
the search range (:numref:`corr_section`). A value like 50 is likely adequate. 

This produces a dense match file (:numref:`dense_ip`), that can 
be passed to ``pc_align``::

    matchFile=run_corr/run-disp-ref_hill__src_hill.match
    pc_align                                     \
      --max-displacement -1                      \
      --num-iterations 0                         \
      --max-num-reference-points 1000000         \
      --match-file $matchFile                    \
      --initial-transform-from-hillshading rigid \
      --initial-transform-ransac-params 1000 3   \
      --save-transformed-source-points           \
      ref.tif src.tif                            \
      -o run_align/run

The resulting aligned cloud ``run_align/run-trans_source.tif`` can be regridded
with ``point2dem`` and same grid size and projection as the input DEMs, and
evaluate if it moved as expected. 

This method will fail if the input DEMs do not overlap a lot when overlaid with
georeference information. 

The related method in :numref:`pc_hillshade` uses sparse features from
hillshading, and can handle a large translation between the clouds.

.. _pc_least_squares:

Least squares
^^^^^^^^^^^^^

Another option is to use least squares (with outlier handling using a
robust cost function) to find the transform, if the reference cloud is a
DEM. This is an *experimental mode* that is *not recommended*.

For this, one should specify the alignment method as ``least-squares`` or
``similarity-least-squares`` (the latter also solves for scale). It is suggested
that the input clouds be very close or otherwise the ``--initial-transform``
option be used, for the method to converge, and use perhaps on the order of
10-20 iterations and a smaller value for ``--max-num-source-points`` (perhaps a
few thousand) for this approach to converge reasonably fast.

File formats
~~~~~~~~~~~~

The input point clouds can be in one of several formats: ASP’s point
cloud format (the output of ``stereo``), DEMs as GeoTIFF or ISIS cub
files, LAS files, or plain-text CSV files (with .csv or .txt extension).

By default, CSV files are expected to have on each line the latitude and
longitude (in degrees), and the height above the datum (in meters),
separated by commas or spaces. Alternatively, the user can specify the
format of the CSV file via the ``--csv-format`` option. Entries in the
CSV file can then be (in any order) (a) longitude, latitude (in
degrees), height above datum (in meters), (b) longitude, latitude,
distance from planet center (in meters or km), (c) easting, northing and
height above datum (in meters), in this case a PROJ or WKT string must be set
via ``--csv-srs``, (d) Cartesian coordinates :math:`(x, y, z)`
measured from planet center (in meters). The precise syntax is described
in the table below. The tool can also auto-detect the LOLA RDR
PointPerRow format.

Any line in a CSV file starting with the pound character (#) is ignored.

If none of the input files have a geoheader with datum information, and
the input files are not in Cartesian coordinates, the datum needs to be
specified via the ``--datum`` option, or by setting
``--semi-major-axis`` and ``--semi-minor-axis``.

.. _alignmenttransform:

The alignment transform
~~~~~~~~~~~~~~~~~~~~~~~

The transform obtained by ``pc_align`` is output to a text file as
a 4 |times| 4 matrix with the upper-left 3 |times| 3 submatrix being
the rotation (and potentially also a scale, per :numref:`align-method`)
and the top three elements of the right-most column being the
translation. It is named ``<output prefix>-transform.txt``.

This transform, if applied to the source point cloud,
will bring it in alignment with the reference point cloud.  The
transform assumes the 3D Cartesian coordinate system with the origin
at the planet center (known as ECEF). This matrix can be supplied
back to the tool as an initial guess (:numref:`prevtrans`). 

The inverse transform, from the reference cloud to the source cloud is saved
as well, as ``<output prefix>-inverse-transform.txt``. 

These two transforms can be used to move cameras from one cloud's coordinate
system to another one's, as shown in :numref:`ba_pc_align`.

.. _prevtrans:

Applying an initial transform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``pc_align``-produced transform (:numref:`alignmenttransform`) can be
supplied back to the tool as an initial guess via the ``--initial-transform``
option, with the same clouds as earlier, or some supersets or subsets of them.
If it is desired to apply this transform without further refinement, one can
specify ``--num-iterations 0``.

An initial transform can be found, for example, based on hillshading the two
clouds (:numref:`pc_hillshade`).

To illustrate applying a transform, consider a DEM, named ``dem.tif``, obtained
with ASP, from which just a portion, ``dem_crop.tif`` is known to have reliable
measurements, which are stored, for example, in a file called ``meas.csv``.
Hence, ``pc_align`` is first used on the smaller DEM, as::

    pc_align <other options> dem_crop.tif meas.csv -o run/run

Then, the command::

    pc_align                                    \
      --max-displacement -1                     \
      --num-iterations 0                        \
      --max-num-reference-points 1000           \
      --max-num-source-points 1000              \
      --save-transformed-source-points          \
      --save-inv-transformed-reference-points   \
      --initial-transform run/run-transform.txt \
      --csv-format <csv format string>          \
      dem.tif meas.csv                          \
      -o run_full/run

will transform the full ``dem.tif`` into the coordinate system of ``meas.csv``,
and ``meas.csv`` into the coordinate system of ``ref.tif`` with no further
iterations. The number of input points here is small, for speed, as they will
not be used.

See also :numref:`ba_pc_align` for how to use such transforms with cameras.

If an initial transform is used, with zero or more iterations, the
output transform produced by such an invocation will be from the source
points *before* the initial transform, hence the output alignment
transform will incorporate the initial transform.

Using ``--max-displacement -1`` should be avoided, as that will do 
no outlier filtering in the source cloud. Here that is not necessary,
as this invocation simply moves the DEM according to the specified
transform.

If a good initial alignment is found, it is suggested to use a smaller
value for ``--max-displacement`` to refine the alignment, as the
clouds will already be mostly on top of each other after the initial
transform is applied.

Applying an initial specified translation or rotation 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

One can apply to the source cloud an initial shift, expressed in the
North-East-Down coordinate system at the centroid of the source
points, before the alignment algorithm is invoked. Hence, if it is
desired to first move the source cloud North by 5 m, East by 10 m, and
down by 15 m relative to the point on planet surface which is the
centroid of the source points, the continue with alignment, one can
invoke ``pc_align`` with::


    --initial-ned-translation "5 10 15"

(Notice the quotes.)

The option ``--initial-rotation-angle`` can be used analogously.

As in :numref:`prevtrans`, one can simply stop after such an
operation, if using zero iterations. In either case, such initial
transform will be incorporated into the transform file output by
``pc_align``, hence that one will go from the source cloud before
user's initial transform to the reference cloud.

Interpreting the transform
~~~~~~~~~~~~~~~~~~~~~~~~~~

The alignment transform, with its origin at the center of the planet,
can result in large movements on the planet surface even for small
angles of rotation. Because of this it may be difficult to interpret
both its rotation and translation components.

The ``pc_align`` program outputs the translation component of this
transform, defined as the vector from the centroid of the original
source points (before any initial transform applied to them) to the
centroid of the source points with the computed alignment transform
applied to them. This translation component is displayed in three ways
(a) Cartesian coordinates with the origin at the planet center, (b)
Local North-East-Down coordinates at the centroid of the source points
(before any initial transform), and (c) Latitude-Longitude-Height
differences between the two centroids. If the effect of the transform is
small (e.g., the points moved by at most several hundred meters) then
the representation in the form (b) above is most amenable to
interpretation as it is in respect to cardinal directions and height
above ground if standing at a point on the planet surface.

This program prints to screen the Euler angles of the rotation
transform, and also the axis of rotation and the angle measured against
that axis. It can be convenient to interpret the rotation as being
around the center of gravity of the reference cloud, even though it was
computed as a rotation around the planet center, since changing the
point around which a rigid transform is applied will only affect its
translation component, which is relative to that point, but not the
rotation matrix.

.. _pc_align_error:

Error metrics and outliers
~~~~~~~~~~~~~~~~~~~~~~~~~~

The tool outputs to CSV files the lists of errors together with their
locations in the source point cloud, before the alignment of the source
points (but after applying any initial transform), and also after the
alignment computed by the tool. They are named
``<output prefix>-beg_errors.csv`` and
``<output prefix>-end_errors.csv``. An error is defined as the distance
from a source point used in alignment to the closest reference point
(measured in meters). 

The format of output CSV files is the same as of input CSV files, or as
given by ``--csv-format``, although any columns of extraneous data in
the input files are not saved on output. The first line in these
files shows the names of the columns.

See :numref:`plot_csv` for how to visualize these files. By default,
this tool shows the 4th column in these files, which is the absolute
error difference. Run, for example::

    stereo_gui --colorbar run/run-end_errors.csv

The program prints to screen and saves to a log file the 16th, 50th, and
84th error percentiles as well as the means of the smallest 25%, 50%,
75%, and 100% of the errors.

When the reference point cloud is a DEM, a more accurate computation of
the errors from source points to the reference cloud is used. A source
point is projected onto the datum of the reference DEM, its longitude
and latitude are found, then the DEM height at that position is
interpolated. That way we determine the closest point on the reference
DEM that interprets the DEM not just as a collection of points but
rather as a polyhedral surface going through those points. These errors
are what is printed in the statistics. To instead compute errors as done
for other type of point clouds, use the option ``--no-dem-distances``.

By default, when ``pc_align`` discards outliers during the computation
of the alignment transform, it keeps the 75% of the points with the
smallest errors. As such, a way of judging the effectiveness of the tool
is to look at the mean of the smallest 75% of the errors before and
after alignment.

.. _pc_align_validation:

Evaluation of aligned clouds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``pc_align`` program can save the source cloud after being aligned to the
reference cloud and vice-versa, via ``--save-transformed-source-points`` and
``--save-inv-transformed-reference-points``. 

To validate that the aligned source cloud is very close to the reference cloud,
DEMs can be made out of them with ``point2dem`` (:numref:`point2dem`), and those
can be overlaid as georeferenced images in ``stereo_gui`` (:numref:`stereo_gui`)
for inspection. A GIS tool can be used as well.

Alternatively, the ``geodiff`` program (:numref:`geodiff`) can 
compute the (absolute) difference between aligned DEMs, which can
be colorized with ``colormap`` (:numref:`colormap`), or colorized on-the-fly
and displayed with a colorbar in ``stereo_gui`` (:numref:`colorize`).

The ``geodiff`` tool can take the difference between a DEM and a CSV file as
well. The obtained error differences can be visualized in ``stereo_gui``
(:numref:`plot_csv`).

Output point clouds and convergence history
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The transformed input point clouds (the source transformed to match
the reference, and the reference transformed to match the source) can
also be saved to disk if desired. If an input point cloud is in CSV,
ASP point cloud format, or LAS format, the output transformed cloud
will be in the same format. If the input is a DEM, the output will be
an ASP point cloud, since a gridded point cloud may not stay so after
a 3D transform. 

As an example, assume that ``pc_align`` is run as::

    pc_align --max-displacement 100              \
      --csv-format '1:x 2:y 3:z'                 \      
      --save-transformed-source-points           \
      --save-inv-transformed-reference-points    \
      ref_dem.tif source.csv                     \
      -o run/run 

This will save ``run/run-trans_reference.tif`` which is a point cloud
in the coordinate system of the source dataset, and
``run/run-trans_source.csv`` which is in reference coordinate system
of the reference dataset.

The ``point2dem`` program (:numref:`point2dem`) can re-grid the
obtained point cloud back to a DEM.

Care is needed, as before, with setting ``--max-displacement``.

The convergence history for ``pc_align`` (the translation and rotation
change at each iteration) is saved to disk with a name like::

    <output prefix>-iterationInfo.csv
 
and can be used to fine-tune the stopping criteria.

.. _manual-align:

Manual alignment
~~~~~~~~~~~~~~~~

If automatic alignment fails, for example, if the clouds are too
different, or they differ by a scale factor, a manual alignment can be
computed as an initial guess transform (and one can stop there if
``pc_align`` is invoked with 0 iterations). 

For that, the input point clouds should be first converted to DEMs using
``point2dem``, unless in that format already. Then, ``stereo_gui`` can be called
to create manual point correspondences (interest point matches) from the
reference to the source DEM (:numref:`stereo_gui_edit_ip`). The DEMs should be
displayed in the GUI with the reference DEM on the left, and should be
hillshaded. 

Once the match file is saved to disk, it can be passed to ``pc_align`` via the
``--match-file`` option, which will compute an initial transform (whose type is
set with ``--initial-transform-from-hillshading``), before continuing with
alignment. This transform can also be used for non-DEM clouds once it is found
using DEMs obtained from those clouds. Note that both a rigid and similarity
transform is supported, both for the initial transform and for the alignment.

.. _regrid:

Regrid a DEM
~~~~~~~~~~~~

Given a DEM, if one invokes ``pc_align`` as follows::

    pc_align dem.tif dem.tif --max-displacement -1 --num-iterations 0 \
       --save-transformed-source-points -o run/run

this will create a point cloud out of the DEM. This cloud can then be re-gridded
using ``point2dem`` (:numref:`point2dem`), with desired grid size and projection. 

Alternatively, the ``gdalwarp`` program (:numref:`gdal_tools`) can be employed
for regridding, with an option such as ``-r cubic``. 

The ``point2dem`` approach is preferable if the output grid size is very coarse,
as this tool does binning in a neighborhood, rather than interpolation.

.. _ba_pc_align:

Applying a transform to cameras
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If ``pc_align`` is used to align a DEM obtained with ASP to a preexisting
reference DEM or other cloud, the obtained alignment transform can be applied to
the cameras used to create the ASP DEM, so the cameras then become aligned with
the reference. That is accomplished by running bundle adjustment with the
options ``--initial-transform`` and ``--apply-initial-transform-only``.

Please note that the way this transform is applied depends on the 
order of clouds in ``pc_align`` and on whether the cameras have
been bundle-adjusted or not. Precise commands are given below.

First, assume, for example, that the reference is ``ref.tif``, and
the ASP DEM is created *without* bundle adjustment, as::

    parallel_stereo left.tif right.tif left.xml right.xml output/run
    point2dem --auto-proj-center output/run-PC.tif

It is very important to distinguish the cases when the obtained DEM is
the first or second argument of ``pc_align``.

If the ASP DEM ``output/run-DEM.tif`` is aligned to the reference
as::

    pc_align --max-displacement 1000 ref.tif output/run-DEM.tif \
      -o align/run

then, the alignment is applied to cameras the following way::

    bundle_adjust left.tif right.tif left.xml right.xml \
      --initial-transform align/run-transform.txt       \
      --apply-initial-transform-only -o ba_align/run

This should create the adjusted cameras incorporating the alignment
transform::

     ba_align/run-left.adjust, ba_align/run-right.adjust

(see :numref:`adjust_files` for discussion of .adjust files). 

If ``pc_align`` was invoked with the two clouds in reverse order, the
transform to use is::

    align/run-inverse-transform.txt

The idea here is that ``run-transform.txt`` goes from the second cloud
passed to ``pc_align`` to the first, hence, ``bundle_adjust`` invoked
with this transform would move cameras from second cloud's coordinate
system's to first. And vice-versa, if ``run-inverse-transform.txt`` is
used, cameras from first clouds's coordinate system would be moved to
second's.

After applying a transform this way, the cameras that are now aligned
with the reference can be used to mapproject onto it, hopefully
with no registration error, as::

    mapproject ref.tif left.tif left_map.tif \
      --bundle-adjust-prefix ba_align/run

and in the same way for the right image. Overlaying the produced
images is a very useful sanity check.
    
If, the initial stereo was done with cameras that already
were bundle-adjusted, with output prefix ``initial_ba/run``,
so the stereo command had the option::

  --bundle-adjust-prefix initial_ba/run

we need to integrate those initial adjustments with this alignment
transform. To do that, again need to consider two cases, as before.

If the just-created stereo DEM is the second argument to ``pc_align``,
then run the slightly modified command::

    bundle_adjust left.tif right.tif left.xml right.xml \
      --initial-transform align/run-transform.txt       \
      --input-adjustments-prefix initial_ba/run         \
      --apply-initial-transform-only -o ba_align/run

Otherwise, if the stereo DEM is the first argument to ``pc_align``, use instead
``align/run-inverse-transform.txt`` as input to ``--initial-transform``.

Note that this way bundle adjustment will not do any further camera refinements
after the initial transform is applied.

A stereo run can be reused after the cameras have been modified as above, with
the option ``--prev-run-prefix``. Only triangulation will then be redone. Ensure
the option ``--bundle-adjust-prefix ba_align/run`` is used to point to the new
cameras. See :numref:`bathy_reuse_run` and :numref:`mapproj_reuse`.

Troubleshooting
~~~~~~~~~~~~~~~

Remember that filtering is applied only to the source point cloud. If
you have an input cloud with a lot of noise, make sure it is being used
as the source cloud.

If you are not getting good results with ``pc_align``, something that
you can try is to convert an input point cloud into a smoothed DEM. Use
``point2dem`` to do this and set ``--search-radius-factor`` if needed to
fill in holes in the DEM. For some input data this can significantly
improve alignment accuracy.

.. _pc_align_options:

Command-line options for pc_align
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--num-iterations <integer (default: 1000)>
    Maximum number of iterations.

--max-displacement <float>
    Maximum expected displacement (horizontal + vertical) of source
    points as result of alignment, in meters (after the initial guess
    transform is applied to the source points).  Used for removing
    gross outliers in the source (movable) point cloud.

-o, --output-prefix <string (default: "")>
    Specify the output file prefix.

--outlier-ratio <float (default: 0.75)>
    Fraction of source (movable) points considered inliers (after
    gross outliers further than max-displacement from reference
    points are removed).

--max-num-reference-points <integer (default: 10^8)>
    Maximum number of (randomly picked) reference points to use.

--max-num-source-points <integer (default: 10^5)>
    Maximum number of (randomly picked) source points to use (after
    discarding gross outliers).

--alignment-method <string (default: "point-to-plane")>
    Alignment method. Options: ``point-to-plane``, ``point-to-point``,
    ``similarity-point-to-plane``, ``similarity-point-to-point``
    (:numref:`pc_icp`), ``nuth`` (:numref:`nuth`), ``fgr`` (:numref:`fgr`),
    ``least-squares``, ``similarity-least-squares``
    (:numref:`pc_least_squares`).

--highest-accuracy
    Compute with highest accuracy for point-to-plane (can be much slower).

--datum <string>
    Sets the datum for CSV files.
    Options:

    * WGS_1984
    * D_MOON (1,737,400 meters)
    * D_MARS (3,396,190 meters)
    * MOLA (3,396,000 meters)
    * NAD83
    * WGS72
    * NAD27
    * Earth (alias for WGS_1984)
    * Mars (alias for D_MARS)
    * Moon (alias for D_MOON)

--semi-major-axis <float>
    Explicitly set the datum semi-major axis in meters.

--semi-minor-axis <float>
    Explicitly set the datum semi-minor axis in meters.

--csv-format <string>
    Specify, in quotes, the format of input CSV files as a list of entries
    ``column_index:column_type`` (indices start from 1). Examples: ``'1:x 2:y
    3:z'`` (a Cartesian coordinate system with origin at planet center is
    assumed, with the units being in meters), ``'5:lon 6:lat 7:radius_m'``
    (longitude and latitude are in degrees, the radius is measured in meters
    from planet center), ``'3:lat 2:lon 1:height_above_datum'``, ``'1:easting
    2:northing 3:height_above_datum'`` (for the latter need to also set
    ``--csv-srs``). The height above datum is in meters. Can also use
    ``radius_km`` for ``column_type``, when it is again measured from planet
    center.

--csv-srs <string>
    The PROJ or WKT string to use to interpret the entries in input CSV
    files.

--compute-translation-only
    Compute the transform from source to reference point cloud as
    a translation only (no rotation).

--save-transformed-source-points
    Apply the obtained transform to the source points so they match the
    reference points and save them. The transformed point cloud can be
    gridded with ``point2dem`` (:numref:`point2dem`).

--save-inv-transformed-reference-points
    Apply the inverse of the obtained transform to the reference
    points so they match the source points and save them.

--initial-transform <string>
    The file containing the transform to be used as an initial
    guess. It can come from a previous run of the tool.

--initial-ned-translation <string>
    Initialize the alignment transform based on a translation with
    this vector in the North-East-Down coordinate system around the
    centroid of the reference points. Specify it in quotes, separated
    by spaces or commas.

--initial-rotation-angle <double (default: 0.0)>
    Initialize the alignment transform as the rotation with this angle
    (in degrees) around the axis going from the planet center to the
    centroid of the point cloud. If ``--initial-ned-translation`` is
    also specified, the translation gets applied after the rotation.

--initial-transform-from-hillshading <string>
    If both input clouds are DEMs, find interest point matches among their
    hillshaded versions, and use them to compute an initial transform to apply
    to the source cloud before proceeding with alignment
    (:numref:`pc_hillshade`).  Specify here the type of transform, as one of:
    ``rigid`` (rotation + translation), ``translation``, or ``similarity``
    (rotation + translation + scale). See the options further down for tuning
    this. The alignment algorithm can refine the scale later if set to
    ``similarity-point-to-plane``, etc.

--hillshade-options
    Options to pass to the ``hillshade`` program when computing the
    transform from hillshading. Default: 
    ``--azimuth 300 --elevation 20 --align-to-georef``.

--ipfind-options
    Options to pass to the ``ipfind`` program when computing the
    transform from hillshading. Default: ``--ip-per-image 1000000
    --interest-operator sift --descriptor-generator sift``.

--ipmatch-options
    Options to pass to the ``ipmatch`` program when computing the
    transform from hillshading. Default: ``--inlier-threshold 100
    --ransac-iterations 10000 --ransac-constraint similarity``.

--initial-transform-ransac-params <num_iter factor (default: 10000 1.0)>
    When computing an initial transform based on hillshading, use
    this number of RANSAC iterations and outlier factor. A smaller
    factor will reject more outliers. 

--match-file
    Compute an initial transform from the source to the reference point cloud
    given interest point matches from the reference to the source DEM in this
    file. This file can be produced manually, in ``stereo_gui``
    (:numref:`manual-align`), or automatically, as in :numref:`pc_hillshade` or
    :numref:`pc_corr`. See also ``--initial-transform-from-hillshading``
    and ``--initial-transform-ransac-params``.

--nuth-options <string (default: "")>
    Options to pass to the Nuth and Kaab algorithm. Set in quotes. 
    See :ref:`nuth_options` for more details.
    
--fgr-options <string>
    Options to pass to the Fast Global Registration (FGR) algorithm. Set in
    quotes. Default: "div_factor: 1.4 use_absolute_scale: 0 max_corr_dist: 0.025
    iteration_number: 100 tuple_scale: 0.95 tuple_max_cnt: 10000".

--diff-rotation-error <float (default: 1e-8)>
    Change in rotation amount below which the algorithm will stop
    (if translation error is also below bound), in degrees.

--diff-translation-error <float (default: 1e-3)>
    Change in translation amount below which the algorithm will
    stop (if rotation error is also below bound), in meters.

--no-dem-distances
    For reference point clouds that are DEMs, don't take advantage
    of the fact that it is possible to interpolate into this DEM
    when finding the closest distance to it from a point in the
    source cloud (the text above has more detailed information).
    
--skip-shared-box-estimation
    Do not estimate the shared bounding box of the two clouds. This estimation
    can be costly for large clouds but helps with eliminating outliers.
    
--config-file <file.yaml>
    This is an advanced option. Read the alignment parameters from
    a configuration file, in the format expected by libpointmatcher,
    over-riding the command-line options.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.

.. _nuth_options:

Options for Nuth and Kaab 
~~~~~~~~~~~~~~~~~~~~~~~~~

The Nuth and Kaab algorithm (:numref:`nuth`) accepts the regular ``pc_align``
options ``--max-displacement``, ``--num-iterations``,
``--compute-translation-only``, ``--threads``.

In addition, it can be tuned via the ``--nuth-options`` argument. Its value is a
string in quotes, with spaces as separators. Example:: 

    --nuth-options "--slope-lim 0.1 40.0 --tol 0.01"

Default values will be used for any unspecified options. The options are:

--slope-lim <float float (default: 0.1 40.0)>
    Minimum and maximum surface slope limits to consider (degrees).
    
--tol <float (default: 0.01)>
    Stop when the addition to the alignment translation at given iteration has
    magnitude below this tolerance (meters).

--max-horizontal-offset <float>
    Maximum expected horizontal translation magnitude (meters). Used to filter
    outliers. If not set, use the value in ``--max-displacement``.

--max-vertical-offset <float>
    Maximum expected vertical translation in meters (meters). Used to filter
    outliers. If not set, use the value in ``--max-displacement``.

--num-inner-iter <integer (default: 10)>
    Maximum number of iterations for the inner loop, when finding the best fit
    parameters for the current translation.
    
.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
