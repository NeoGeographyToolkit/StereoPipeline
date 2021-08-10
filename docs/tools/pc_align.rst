.. _pc_align:

pc_align
--------

This tool can be used to align two point clouds. The algorithms employed
are one of the several flavors of Iterative Closest Point (ICP), based
on the ``libpointmatcher`` library :cite:`Pomerleau12comp`

https://github.com/ethz-asl/libpointmatcher

It also implements the Fast Global Registration algorithm

https://github.com/IntelVCL/FastGlobalRegistration

In addition, it supports feature-based alignment (terrains are
hillshaded and interest point matches are found among them), and
alignment using least squares. It can handle a scale change in addition
to rotations and translations. For joint alignment of more than two
clouds, the related tool ``n_align`` can be used (:numref:`n_align`).

Usage::

     pc_align --max-displacement <float> [other options]    \
       <reference cloud> <source cloud> -o <output prefix>}

An example of using this tool is in :numref:`pc-align-example`.

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

In many typical applications, the source and reference point clouds are
already roughly aligned, but the source point cloud may cover a larger
area than the reference. The user should provide to ``pc_align`` the
expected maximum distance (displacement) source points may move by as
result of alignment, using the option ``--max-displacement``. This
number will help remove source points too far from the reference point
cloud which may not match successfully and may degrade the accuracy. If
in doubt, this value can be set to something large but still reasonable,
as the tool is able to throw away a certain number of unmatched
outliers. At the end of alignment, ``pc_align`` will display the
*observed* maximum displacement, a multiple of which can be used to seed
the tool in a subsequent run. If an initial transform is applied to the
source cloud (:numref:`prevtrans`), the outliers are thrown
out *after* this operation. The observed maximum displacement is also
between the source points with this transform applied and the source
points after alignment to the reference.

The user can choose how many points to pick from the reference and
source point clouds to perform the alignment. The amount of memory and
processing time used by ``pc_align`` is directly proportional to these
numbers, ideally the more points the better. Pre-cropping to judiciously
chosen regions may improve the accuracy and/or run-time.

.. _align-method:

Alignment method
~~~~~~~~~~~~~~~~

The default alignment method is Point-to-Plane ICP, which may be more
robust to large translations than Point-to-Point ICP, though the latter
can be good enough if the input point clouds have small alignment errors
and it is faster and uses less memory as well. The tool also accepts an
option named ``--highest-accuracy`` which will compute the normals for
Point-to-Plane ICP at all points rather than about a tenth of them. This
option is not necessary most of the time, but may result in better
alignment at the expense of using more memory and processing time.

The default alignment transform is rigid, that is, a combination of
rotation and translation. With Point-to-Point ICP, it is also possible
to solve for a scale change (to obtain a so-called
``similarity transform``). It is suggested this approach be used only
when a scale change is expected. It can be turned on by setting
``--alignment-method similarity-point-to-point``. (This method works
best if an initial alignment is first performed with, for example, the
Point-to-Plane approach, to determine the rotation and translation part
of the transform, and then that one can be used as an initial guess in
order to solve for the scale as well.)

For very large scale difference or translation among the two clouds,
both of these algorithms may fail. If the clouds are DEMs, one may
specify the option ``--initial-transform-from-hillshading string``
which will hillshade the two DEMs, find interest point matches among
them, and use that to compute an initial transform between the
clouds (:numref:`prevtrans`), which may or may not contain scale,
after which the earlier algorithms will be applied to refine the
transform.  This functionality is implemented with ASP’s ``hillshade``,
``ipfind``, and ``ipmatch`` tools, and ``pc_align`` has options to
pass flags to these programs, such as to increase the number interest
points being found, if the defaults are not sufficient. If the two
clouds look too different for interest point matching to work, they
perhaps can be re-gridded to use the same (coarser) grid, as described
in :numref:`regrid`, to obtain the initial transform which can then
be applied to the original clouds.

A non-ICP algorithm supported by ASP is *Fast Global Registration*,
accessible with ``--alignment-method fgr``, and customizable using the
``--fgr-options`` field (see the table below for more details). This
approach can perform better than ICP when the clouds are close enough to
each other but there is a large number of outliers, since it does a
cross-check, so it can function with very large ``--max-displacement``.
It does worse if the clouds need a big shift to align.

This one is being advertised as less sensitive to outliers, hence it
should give good results with a larger value of the maximum
displacement.

Another option is to use least squares (with outlier handling using a
robust cost function) to find the transform, if the reference cloud is a
DEM. For this, one should specify the alignment method as
``least-squares`` or ``similarity-least-squares`` (the latter also
solves for scale). It is suggested that the input clouds be very close
or otherwise the ``--initial-transform`` option be used, for the method
to converge, and use perhaps on the order of 10-20 iterations and a
smaller value for ``--max-num-source-points`` (perhaps a few thousand)
for this approach to converge reasonably fast.

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
height above datum (in meters), in this case a PROJ.4 string must be set
via ``--csv-proj4``, (d) Cartesian coordinates :math:`(x, y, z)`
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
translation. This transform, if applied to the source point cloud,
will bring it in alignment with the reference point cloud.  The
transform assumes the 3D Cartesian coordinate system with the origin
at the planet center (known as ECEF). This matrix can be supplied
back to the tool as an initial guess (:numref:`prevtrans`). The
inverse transform is saved to a file as well.

.. _prevtrans:

Applying an initial transform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The transform output by ``pc_align`` can be supplied back to the tool as
an initial guess via the ``--initial-transform`` option, with the same
or different clouds. If it is desired to simply apply this transform to
the clouds without further work, one can specify ``--num-iterations 0``.
This may be useful, for example, in first finding the alignment
transform over a smaller, more reliable region (e.g., over rock,
excluding moving ice), then applying it over the entire available
dataset.

Alternatively, one can apply to the source cloud an initial shift,
expressed in the North-East-Down coordinate system at the centroid of
the source points, before the alignment algorithm is invoked. Hence, if
it is desired to move the source cloud North by 5 m, East by 10 m, and
down by 15 m relative to the point on planet surface which is the
centroid of the source points, one can invoke ``pc_align`` with
``--initial-ned-translation ’5 10 15’`` (notice the quotes).

The option ``--initial-rotation-angle`` can be used for similar
purposes.

If an initial transform is used, the alignment transform output by the
program will be from the source points *before* the initial transform,
hence the output alignment transform will incorporate the initial
transform.

If a good initial alignment is found, it is suggested to use a smaller
value for ``--max-displacement``, as the clouds will already be mostly
on top of each other after the initial transform is applied.

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

Error metrics and outliers
~~~~~~~~~~~~~~~~~~~~~~~~~~

The tool outputs to CSV files the lists of errors together with their
locations in the source point cloud, before the alignment of the source
points (but after applying any initial transform), and also after the
alignment computed by the tool. They are named
``<output prefix>-beg_errors.csv`` and
``<output prefix>-end_errors.csv``. An error is defined as the distance
from a source point used in alignment to the closest reference point.
The format of output CSV files is the same as of input CSV files, or as
given by ``--csv-format``, although any columns of extraneous data in
the input files are not saved on output.

The program prints to screen and saves to a log file the 16th, 50th, and
84th error percentiles as well as the means of the smallest 25%, 50%,
75%, and 100% of the errors.

When the reference point cloud is a DEM, a more accurate computation of
the errors from source points to the reference cloud is used. A source
point is projected onto the datum of the reference DEM, its longitude
and latitude are found, then the DEM height at that position is
interpolated. That way we determine a “closest” point on the reference
DEM that interprets the DEM not just as a collection of points but
rather as a polyhedral surface going through those points. These errors
are what is printed in the statistics. To instead compute errors as done
for other type of point clouds, use the option ``--no-dem-distances``.

By default, when ``pc_align`` discards outliers during the computation
of the alignment transform, it keeps the 75% of the points with the
smallest errors. As such, a way of judging the effectiveness of the tool
is to look at the mean of the smallest 75% of the errors before and
after alignment.

Output point clouds and convergence history
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The transformed input point clouds (the source transformed to match the
reference, and the reference transformed to match the source) can also
be saved to disk if desired. If an input point cloud is in CSV or ASP
point cloud format, the output transformed cloud will be in the same
format. If the input is a DEM, the output will be an ASP point cloud,
since a gridded point cloud may not stay so after a 3D transform. The
``point2dem`` program can be used to re-grid the obtained point cloud
back to a DEM.

The convergence history for ``pc_align`` (the translation and rotation
change at each iteration) is saved to disk and can be used to fine-tune
the stopping criteria.

.. _manual-align:

Manual alignment
~~~~~~~~~~~~~~~~

If automatic alignment fails, for example, if the clouds are too
different, or they differ by a scale factor, a manual alignment can be
computed as an initial guess transform (and one can stop there if
``pc_align`` is invoked with 0 iterations). For that, the input point
clouds should be first converted to DEMs using ``point2dem``, unless in
that format already. Then, ``stereo_gui`` can be called to create manual
point correspondences (interest point matches) from the reference to the
source DEM (hence they should be displayed in the GUI in this order,
from left to right, and one can hillshade them to see features better).
Once the match file is saved to disk, it can be passed to ``pc_align``
via the ``--match-file`` option, which will compute an initial transform
before continuing with alignment. This transform can also be used for
non-DEM clouds once it is found using DEMs obtained from those clouds.

.. _regrid:

Creating a point cloud from a DEM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Given a DEM, if one invokes ``pc_align`` as follows::

    pc_align dem.tif dem.tif --max-displacement -1 --num-iterations 0 \
       --save-transformed-source-points -o run/run

this will create a point cloud out of the DEM. This cloud can then be
re-gridded using ``point2dem`` at a lower resolution or with a different
projection.

.. _ba_pc_align:

Applying the pc_align transform to cameras
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If ``pc_align`` is used to align a DEM obtained with ASP to a
preexisting reference DEM, the obtained alignment transform can be
applied to the cameras used to create the ASP DEM, so the cameras then
become aligned with the pre-existing DEM. That is accomplished by
running bundle adjustment with the options ``--initial-transform``
and ``--apply-initial-transform-only``.


As an example, assume the reference DEM is ``ref.tif``, and the 
ASP DEM is created as::

    parallel_stereo left.tif right.tif left.xml right.xml output/run
    point2dem output/run-PC.tif

The ASP DEM ``output/run-DEM.tif`` is aligned to the reference DEM
as::

    pc_align --max-displacement 1000 ref.tif output/run-DEM.tif \
      -o align/run

The alignment is applied to cameras the following way::

    bundle_adjust left.tif right.tif left.xml right.xml \
      --initial-transform align/run-transform.txt       \
      --apply-initial-transform-only -o ba_align/run

This should create the adjusted cameras incorporating the alignment
transform::

     ba_align/run-left.adjust, ba_align/run-right.adjust

If ``pc_align`` was invoked with the two DEMs in reverse order, the
transform to use is::

    align/run-inverse-transform.txt

As an application, the cameras can now be mapprojected onto the 
reference DEM, hopefully with no registration error as::

    mapproject ref.tif left.tif left_map.tif \
      --bundle-adjust-prefix ba_align/run

and in the same way for the right image.
    
If, however, the initial stereo was done with cameras that already
were bundle adjusted, so the stereo command had the option::

  --bundle-adjust-prefix initial_ba/run

we need to integrate those initial adjustments with this alignment
transform. To do that, run the slightly modified command::

    bundle_adjust left.tif right.tif left.xml right.xml \
      --initial-transform align/run-transform.txt       \
      --input-adjustments-prefix initial_ba/run         \
      --apply-initial-transform-only -o ba_align/run

Note that this way bundle adjustments will not do any camera
refinements after the initial transform is applied.

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

Command-line options for pc_align
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
--num-iterations <integer (default: 1000)>
    Maximum number of iterations.

--max-displacement <float>
    Maximum expected displacement of source points as result of
    alignment, in meters (after the initial guess transform is
    applied to the source points).  Used for removing gross outliers
    in the source (movable) point cloud.

-o, --output-prefix <filename>
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

--alignment-method <string (default: point-to-plane)>
    The type of iterative closest point method to use.  Choices: point-to-plane,
    point-to-point, similarity-point-to-point, fgr, least-squares,
    similarity-least-squares

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
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center),
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-proj4``; the height above datum is in
    meters). Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-proj4 <string>
    The PROJ.4 string to use to interpret the entries in input CSV
    files, if those files contain Easting and Northing fields.

--compute-translation-only
    Compute the transform from source to reference point cloud as
    a translation only (no rotation).

--save-transformed-source-points
    Apply the obtained transform to the source points so they match
    the reference points and save them.

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
    If both input clouds are DEMs, find interest point matches among
    their hillshaded versions, and use them to compute an initial
    transform to apply to the source cloud before proceeding with
    alignment.  Specify here the type of transform, as one of:
    'similarity' (rotation + translation + scale), 'rigid' (rotation
    + translation) or 'translation'.

--hillshade-options
    Options to pass to the ``hillshade`` program when computing the
    transform from hillshading. Default: 
    ``--azimuth 300 --elevation 20 --align-to-georef``.

--ipfind-options
    Options to pass to the ``ipfind`` program when computing the
    transform from hillshading. Default: ``--ip-per-image 1000000
    --interest-operator sift --descriptor-generator sift``

--ipmatch-options
    Options to pass to the ``ipmatch`` program when computing the
    transform from hillshading. Default: ``--inlier-threshold 100
    --ransac-iterations 10000 --ransac-constraint similarity``

--match-file
    Compute an initial transform from the source to the reference
    point cloud using manually selected point correspondences
    (obtained for example using stereo_gui). The type of transform
    can be set via ``--initial-transform-from-hillshading string``

--initial-transform-outlier-removal-params <pct factor (default: 75.0 3.0)>
    When computing an initial transform based on features, either
    via the ``--initial-transform-from-hillshading`` or ``--match-file``
    options, remove outliers when this transform is applied by
    excluding the errors larger than this percentile times this
    factor.

--fgr-options
    Options to pass to the Fast Global Registration algorithm, if
    used. Default: ``div_factor: 1.4 use_absolute_scale: 0
    max_corr_dist: 0.025 iteration_number: 100 tuple_scale: 0.95
    tuple_max_cnt: 10000``

--diff-rotation-error <float (default: 10^{-8})>
    Change in rotation amount below which the algorithm will stop
    (if translation error is also below bound), in degrees.

--diff-translation-error <float (default: 10^{-3})>
    Change in translation amount below which the algorithm will
    stop (if rotation error is also below bound), in meters.

--no-dem-distances
    For reference point clouds that are DEMs, don’t take advantage
    of the fact that it is possible to interpolate into this DEM
    when finding the closest distance to it from a point in the
    source cloud (the text above has more detailed information).

--config-file <file.yaml>
    This is an advanced option. Read the alignment parameters from
    a configuration file, in the format expected by libpointmatcher,
    over-riding the command-line options.

--threads <integer (default: 0)>
    Set the number threads to use. 0 means use the default as set
    by OpenMP. Only some parts of the algorithm are multi-threaded.

-h, --help 
    Display the help message.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
