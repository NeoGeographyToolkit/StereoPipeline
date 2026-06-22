.. _outputfiles:

Guide to output files
=====================

The ``parallel_stereo`` program (:numref:`parallel_stereo`) generates a variety
of intermediate files that are useful for debugging. These are listed below,
along with brief descriptions about the contents of each file.

Some of these files are stored at the location given by the specified
output prefix, while others are in subdirectories of that location
corresponding to individual tiles created by ``parallel_stereo``.

The files are listed based on the stereo stage they are created at
(:numref:`parallel_stereo`).

.. _pprc_files:

Files created in preprocessing
------------------------------

\*-cropped\*.tif
    Cropped versions of the input images, before alignment, when options
    ``--left-image-crop-win`` and/or ``--right-image-crop-win`` are used.

\*.vwip
    Interest point matches per image, before matching.

    If the images are ``left.cub`` and ``right.cub``, these files end in
    ``left.vwip`` and ``right.vwip``. Several interest point detection modes are
    available (see ``--ip-detect-method`` in :numref:`stereodefault`). Very long
    image names are shortened (:numref:`match_file_naming`).

    The ``.vwip`` files can be visualized in ``stereo_gui``
    (:numref:`stereo_gui_vwip_gcp`).

\*.match - image-to-image interest point matches (tie-points)
    The match file lists a select group of unique points out of the previous
    ``.vwip`` files that have been identified and matched in a pair of images.
    For example, if the input images are ``left.cub`` and ``right.cub``, the
    match file will end in ``left__right.match``. Very long image names are
    shortened (:numref:`match_file_naming`).

    The ``.vwip`` and ``.match`` files are meant to serve as cached tie-point
    information, and they help speed up the pre-processing phase of the Stereo
    Pipeline. If these files exist, then the ``parallel_stereo`` program will
    skip over the interest point alignment stage and instead use the cached
    tie-points contained in the ``*.match`` files, unless the images or cameras
    are newer, or if invoked with the options ``--left-image-crop-win`` or
    ``--right-image-crop-win``. In the rare case that these files get
    corrupted, they should be deleted and ``parallel_stereo`` should be rerun.

    The ``.match`` files can be visualized in ``stereo_gui``
    (:numref:`stereo_gui_view_ip`), and can be converted to plain text for
    inspection with ``parse_match_file.py`` (:numref:`parse_match_file`).

    This program supports plain-text match files (:numref:`txt_match`).

\*-L.tif - rectified left input image
    Left input image of the stereo pair, after the pre-processing
    step, which may involve cropping, normalization of pixel values,
    and alignment.

\*-R.tif - rectified right input image
    Right input image of the stereo pair, after the pre-processing
    step, which may involve cropping, normalization of pixel values,
    and alignment.

\*-lMask.tif - mask for left rectified image
    This file and \*-rMask.tif contain binary masks for the input
    images. They are used throughout the stereo process to mask
    out pixels where there is no input data.

\*-rMask.tif - mask for right rectified image
    See \*-lMask.tif, above.

\*-align-L.txt and R.txt - left and right alignment matrices
    These 3 |times| 3 affine transformation matrices that are used to warp the
    left and right images to roughly align them. These are only generated if
    ``alignment-method`` is not ``none`` or ``epipolar``
    (:numref:`stereodefault`).

    The older .exr format for these is still supported on reading but will be
    removed in the future.

\*bathy_mask\*.tif - data related to water-land masks, for stereo with
    shallow water (:numref:`bathy_intro`).

\*-L_sub.tif, \*-R_sub.tif, \*-lMask_sub.tif, \*-rMask_sub.tif are
    low-resolution versions of the aligned left and right input images
    and corresponding masks.

\*-stereo.default - backup of the Stereo Pipeline settings file
    This is a copy of the ``stereo.default`` file used by ``parallel_stereo``.
    It is stored alongside the output products as a record of the
    settings that were used for this particular stereo processing task.

.. _out_corr_files:

Files created during correlation
--------------------------------

\*-D_sub.tif - Low-resolution initial disparity (:numref:`d_sub`).
    Computed at the correlation stage. Not recomputed when a run is
    resumed. The options ``--corr-seed-mode 2`` and ``3`` also produce
    \*-D_sub_spread.tif, which has the spread of this disparity. It is
    in the same format as ``D.tif`` (below).

\*-D.tif - Full-resolution disparity map produced from the low-resolution disparity

    The disparity shows the amount of horizontal and vertical shift between left
    and right images, in units of pixel (:numref:`stereo_corr`). The ``D.tif``
    file contains a preliminary disparity that is used to seed the subsequent
    sub-pixel correlation. It is largely unfiltered, and may contain some bad
    matches.

    Disparity map files are stored in TIF format as 3-channel, 32-bit
    floating point images. Channel 0 = horizontal disparity, channel 1 =
    vertical disparity, and channel 2 = good pixel mask.

    The ``disparitydebug`` program (:numref:`disparitydebug`) can help inspect
    scaled versions of these. Or the raw bands can be extracted and visualized
    as in :numref:`mask_disparity`.

\*-L-R-disp-diff.tif - the discrepancy between left-to-right and right-to-left
    disparities. See option ``--save-left-right-disparity-difference``
    in :numref:`stereodefault` for more details.

\*-PC_sub.tif - triangulated point cloud image.
   Made from the low-resolution disparity ``D_sub.tif`` (created after
   filtering this disparity; will be written unless disparity
   filtering is disabled, see ``outlier-removal-params``).

Files created during blending
-----------------------------

\*-B.tif - disparity map blending the D.tif results from all tiles. Will be
    produced unless using the ``asp_bm`` stereo algorithm without local
    epipolar alignment.  It is in the same format as ``D.tif`` (above).

Files created during refinement
-------------------------------

\*-RD.tif - disparity map after sub-pixel correlation
    This file contains the disparity map after sub-pixel refinement.
    Pixel values now have sub-pixel precision, and some outliers have
    been rejected by the sub-pixel matching process.  It is
    in the same format as ``D.tif`` (above).

.. _out_fltr_files:

File created during filtering
-----------------------------

\*-F-corrected.tif - intermediate data product
    Only created when ``alignment-method`` is not ``none``. This is
    ``*-F.tif`` with effects of interest point alignment removed.

\*-F.tif - filtered disparity map
    The filtered, sub-pixel disparity map with outliers removed (and
    holes filled with the inpainting algorithm if ``FILL_HOLES`` is
    on). This is the final version of the disparity map. It is
    in the same format as ``D.tif`` (above).

\*-GoodPixelMap.tif - map of good pixels.
    An image showing which pixels were matched by the stereo
    correlator (gray pixels), and which were filled in by the hole filling
    algorithm (red pixels).

.. _triangulation_files:

Files created at triangulation
------------------------------

\*-PC.tif - point cloud image
    The point cloud image is generated by the triangulation phase of
    Stereo Pipeline. Each pixel in the point cloud image corresponds to
    a pixel in the left input image (\*-L.tif). The point cloud has four
    channels, the first three are the Cartesian coordinates of each
    point, and the last one has the intersection error of the two rays
    which created that point (:numref:`triangulation_error`). By default,
    the origin of the Cartesian coordinate system being used is a
    point in the neighborhood of the point cloud.
    This makes the values of the points in the cloud
    relatively small, and we save them in single precision (32 bits).
    This origin is saved in the point cloud as well using the tag
    ``POINT_OFFSET`` in the GeoTiff header. To output point clouds using
    double precision with the origin at the planet center (ECEF), call
    ``stereo_tri`` with the option
    ``--save-double-precision-point-cloud``. This can effectively
    double the size of the point cloud.

    If the option ``--compute-error-vector`` (:numref:`triangulation_options`)
    or ``--propagate-errors`` (:numref:`error_propagation`) is set,
    the point cloud will have 6 channels. The first 3 channels store,
    as before, the triangulated points.

\*-PC-center.txt - the point cloud local origin (add this to cloud points
    to convert them to ECEF). Stored in plain text. Has the same information as
    the ``POINT_OFFSET`` header in ``PC.tif``.

.. _stereo_diag:

Diagnostics files
-----------------

\*-stereo-status.txt - processing status file

    As ``parallel_stereo`` runs, it updates this file. It records the current
    processing step (:numref:`entrypoints`), the number of done tiles, and the
    total number of tiles (both for the current step). See :numref:`ps_tiling`
    for more details on tiling.

    For the preprocessing and filtering stages tiling is not done. Then, the number
    of tiles is set to 1, and the number of done tiles is 0 if this stage is
    in progress, and 1 if it is done.

    The status is also printed to standard output as each tile gets done (unless
    there is only one tile).

\*-tiles.shp - shapefile having the tiles

    This file saves the tiles used in processing (:numref:`ps_tiling`). Each is
    shown as a rectangle (the units are described below). This file is produced
    only by ``parallel_stereo``, and not by ``stereo``. The padding of each tile
    (for ``asp_mgm`` for example) is not included.

    A file named ``*-tiles.qml`` is also created. With this one present, when
    the shapefile is opened in QGIS, the tile index (an integer starting with
    zero) will be printed inside each tile.

    The ``stereo_gui`` program (:numref:`plot_poly`) can also read and display
    this shapefile and the indices.

    Note that the actual tile list is saved in ``*-dirList.txt``.

    When the images are mapprojected, the shapefile is saved in the projection
    of the ``L.tif`` image and can be overlaid on top ``L.tif`` and ``R.tif`` in
    QGIS and ``stereo_gui``.

    Otherwise the shapefile is in pixel units. The y coordinate is then written
    with the negative sign, so that the shapefile appears correctly on top of
    ``L.tif`` and ``R.tif`` in QGIS and ``stereo_gui``.

.. _out_log_files:

Other files created at all stages
---------------------------------

\*-log* - log files
    Each program invoked by ``parallel_stereo`` writes a log file containing the
    command name, build information, and various messages output by that
    program. Those are saved to the output prefix location, or to tile
    subdirectories, depending on the stage of processing.

    The tiles are deleted after a successful run, which makes the log files in
    subdirectories go away. See the ``--keep-only`` option
    (:numref:`ps_options`) for how to keep all data, including the log files.

\*-<program name>-resource-usage.txt - resource usage files
    For Linux, write such a file for each ``parallel_stereo`` subprocess. It
    contains the elapsed time and memory usage, as output by ``/usr/bin/time``.
    These are written to tile subdirectories, and are deleted after a successful
    run. See the ``--keep-only`` option for how to keep all files.

Inspection and properties of the output files
---------------------------------------------

All the output images that are single-band can be visualized in
``stereo_gui`` (:numref:`stereo_gui`). The disparities can be first
split into the individual horizontal and vertical disparity files
using ``disparitydebug`` (:numref:`disparitydebug`), then they can be
seen in this viewer as well.

If the input images are map-projected (georeferenced) and the
alignment method is ``none``, all the output images listed above, will
also be georeferenced, and hence can be overlaid in ``stereo_gui`` on
top of the input images (the outputs of ``disparitydebug`` will then
be georeferenced as well).

The point cloud file saves the datum (and projection if available)
inferred from the input images, regardless of whether these images
are map-projected or not.

The ``point2mesh`` (:numref:`point2mesh`) and ``point2dem``
(:numref:`point2dem`) programs can be used to convert the point cloud
to formats that are easier to visualize.

.. _txt_match:

Plain text match files
----------------------

ASP programs store interest point matches between two images as a match file, in
either binary format with a ``.match`` extension, or in plain text format, with a
``.txt`` extension. The latter is supported as of build 2026/02
(:numref:`release`).

Plain text matches are provided for use with external logic for interest point
matching. The next sections describe how to use such files and their format.

When there are multiple images, one may use pairwise match files or a control
network format. See :numref:`control_network`.

If the match files are in mapprojected coordinates, see :numref:`txt_map_match`.

Turn on plain text matches
~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, ASP programs work with binary match files. The switch
``--matches-as-txt`` will enable reading and writing plain text match files
instead. In that case, all existing ``.match`` files will be ignored.

This applies to :ref:`bundle_adjust`, :ref:`parallel_stereo`,
:ref:`jitter_solve`, :ref:`image_align`, :ref:`gcp_gen`, and :ref:`ipmatch`.

Conversions between plain text match files and binary match files (ending in
``.match``) can be done with ``ipmatch`` (:numref:`ipmatch_convert`). Do not use
the ``parse_match_file.py`` (:numref:`parse_match_file`) program as that one has
a different purpose.

Naming convention
~~~~~~~~~~~~~~~~~

The general match-file naming convention, with examples, is described in
:numref:`ba_match_files`. For plain text, the only difference is the extension.

Given two images ``input/image1.tif`` and ``input/image2.tif``, and given an
output prefix such as ``out/run``, the plain-text match file name will be::

    out/run-image1__image2.txt

Binary match files will have the same format but will end in ``.match``.

The ``bundle_adjust`` program needs to be invoked with the input images and::

  --match-files-prefix out/run

to read the above file. The same option is also available for
``parallel_stereo`` and ``jitter_solve``.

If the image names are long enough that the resulting file name would exceed the
file system limit, the long parts are shortened in a reproducible way. See
:numref:`match_file_naming`.

.. _txt_format:

File format
~~~~~~~~~~~

Each line in a plain-text match file will have six numbers, in float precision,
separated by spaces::

    x1 y1 unc1 x2 y2 unc2

Here, ``x1 y1`` are the coordinates of an interest point in the first image
(column and row, starting from 0), ``unc1`` is its uncertainty (in pixels), and
``x2 y2 unc2`` are the corresponding values for the second image. In bundle
adjustment each pixel is weighted by the inverse of its uncertainty. The
uncertainties must be positive.

Note that this is not the same format as in ``parse_match_file.py``
(:numref:`parse_match_file`).

Inspection
~~~~~~~~~~

To view plain text match files use ``stereo_gui`` (:numref:`stereo_gui_view_ip`)
as::

    stereo_gui               \
      --matches-as-txt       \
      image1.tif image2.tif  \
      --match-file out/run-image1__image2.txt

.. _txt_map_match:

Mapprojected images and plain text match files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When ``bundle_adjust`` creates interest point matches from mapprojected images
(:numref:`mapip`), both the match files among the mapprojected images and the
unprojected camera-level match files are read and written in plain text as well.
Both follow the usual match-file naming convention (:numref:`ba_match_files`),
with a ``.txt`` extension rather than ``.match``.

An externally computed match file among mapprojected images can be provided this
way. For it to be picked up, in addition to ``--matches-as-txt``, the option
``--mapprojected-data`` must be set (as described in :numref:`mapip`), as this is
what makes ``bundle_adjust`` look at the mapprojected match files and unproject
them to the cameras. Without it, mapprojected match files are not consulted, with
or without ``--matches-as-txt``.

If there exist match files between both raw (camera-level) and mapprojected
images, the raw ones will be read. Match files between raw images should be
deleted in order for them to be recreated from match files between mapprojected
images.

.. _match_file_naming:

Shortening of long file names
-----------------------------

The ``.vwip`` files (having interest points before matching) and ``.match``
files (having interest point matches) are defined in :numref:`pprc_files`. Their
naming convention is described in :numref:`ba_match_files`.

If, combined with the output prefix, such a name would exceed the maximum length
a file system allows for a single file name (255 bytes on most systems,
including ext4, XFS, and APFS), the long parts are shortened. This avoids
failures when working with very long image names, such as the per-framelet
products of some push-frame cameras.

The two image names in a ``.match`` file, and the single name in a ``.vwip``
file, are shortened independently. Each name that is too long is reduced to a
leading portion of itself followed by a 64-bit hash (16 hexadecimal digits) of
the full original name. A fixed FNV-1a hash is used, not a standard-library
hash, so the result is identical across platforms (Linux, macOS) and across
runs. Distinct inputs therefore map to distinct names, with collisions being
negligibly unlikely (below one in a billion for over 100,000 names). Most data,
including names of about 100 characters, is well under the limit and is left
unchanged.

If the output prefix itself is extremely long (specifically the part after the
last directory separator), that part is shortened as well, so the resulting
match file name stays within the limit. The directory portion of the prefix is
left unchanged.

The same rule is applied whenever a name is generated, both when these files are
written and when they are later looked up (including with ``--match-files-prefix``
and ``--clean-match-files-prefix``, :numref:`bundle_adjust`). Hence the names
stay consistent and the files are found without any user action. The
``-clean.match`` files (:numref:`ba_match_files`) use the same shortened base,
differing only by the ending.

This applies to all ASP programs handling interest points, including
:ref:`parallel_stereo`, :ref:`bundle_adjust`, :ref:`ipmatch`, etc.

As an example, consider the image file names below. These are inspired by the
CaSSIS mission, whose products have unusually long names, and are made
intentionally longer here so that, combined with the output prefix, the result
exceeds the file system limit and the hashing is triggered::

    cas_cal_sc_20180506T223500-20180506T223504-2014-16-PAN-272560849-stack_deband_norm_radcor_destripe-frame00000of01000-band0.cub
    cas_cal_sc_20180506T223500-20180506T223504-2014-16-PAN-272560849-stack_deband_norm_radcor_destripe-frame00001of01000-band0.cub

With the output prefix ``out/run``, the produced match file name is::

    out/run-cas_cal_sc_20180506T223500-20180506T223504-2014-16-PAN-272560849-stack_deband_norm_radcor_destrip_2392540bc8f7935e__cas_cal_sc_20180506T223500-20180506T223504-2014-16-PAN-272560849-stack_deband_norm_radcor_destrip_d92f863fa99371d7.match

where each long name was reduced to a leading part plus a 16-digit hash of its
full name (here ``2392540bc8f7935e`` and ``d92f863fa99371d7``).

.. _csv_format:

Format of CSV files
-------------------

Several ASP tools read and write point data as plain text CSV files, with
the entries separated by commas or spaces. This includes ``pc_align``
(:numref:`pc_align`), ``bundle_adjust`` (:numref:`bundle_adjust`), ``geodiff``
(:numref:`geodiff`), ``point2dem`` (:numref:`point2dem`), and others.

The columns are described with the ``--csv-format`` option, in quotes, as a
list of entries ``column_index:column_type``, with the index starting from 1.
The columns can be in any order, and any extra columns are ignored. The
recognized column types are:

- ``x``, ``y``, ``z``: Cartesian (ECEF) coordinates, in meters, with the origin
  at the planet center.
- ``lon``, ``lat``: longitude and latitude, in degrees.
- ``height_above_datum``: height above the datum, in meters. The datum is set
  by the tool or via ``--datum`` (and ``--semi-major-axis`` /
  ``--semi-minor-axis``).
- ``radius_m``, ``radius_km``: distance from the planet center, in meters or
  kilometers.
- ``easting``, ``northing``: projected coordinates, in meters. A PROJ or WKT
  string must then be set via ``--csv-srs``.

The same format applies to all tools, and to the CSV files these tools write on
output.

What follows are several concrete examples. Each is shown with one tool, but the
same ``--csv-format`` value applies to that kind of data in any tool that reads
CSV files.

Longitude, latitude, height above datum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For a CSV file with longitude, latitude (in degrees), and height above the datum
(in meters) as the first three entries, in this order, the format is
``'1:lon 2:lat 3:height_above_datum'``.

As an illustration, find the difference between a DEM and such a file with
``geodiff`` (:numref:`geodiff`)::

    geodiff dem.tif file.csv                          \
      --csv-format '1:lon 2:lat 3:height_above_datum' \
      -o run

Easting, northing, height above datum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For projected measurements, the format is along the lines of ``'1:easting
2:northing 3:height_above_datum'`` (in meters), and a PROJ or WKT string must be
set via ``--csv-srs`` to interpret the easting and northing values. This may
need to be adjusted for your specific fields and their order.

As an illustration, align a DEM to such a file with ``pc_align``
(:numref:`pc_align`)::

    pc_align --max-displacement 100                            \
      --csv-format '1:easting 2:northing 3:height_above_datum' \
      --csv-srs 'EPSG:32610'                                   \
      dem.tif meas.csv                                         \
      -o run/run

LOLA, with radius in km
~~~~~~~~~~~~~~~~~~~~~~~

For a LOLA RDR PointPerRow table, as fetched from the `ODE web tool
<https://ode.rsl.wustl.edu/moon/tools?displaypage=lolardr>`_, the measurements
are longitude, latitude (in degrees), and radius from the planet center (in km),
in columns 2, 3, and 4. The format is then ``'2:lon 3:lat 4:radius_km'`` (the
first column in a LOLA RDR file is the observation time, in UTC).

As an illustration, use a LOLA cloud as a reference terrain in ``bundle_adjust``
(:numref:`bundle_adjust`)::

    bundle_adjust <images> <cameras>         \
      --reference-terrain lola.csv           \
      --csv-format '2:lon 3:lat 4:radius_km' \
      -o ba/run

The ``pc_align`` program (:numref:`pc_align`) auto-detects the LOLA RDR
PointPerRow format, so for an unmodified file from that tool the
``--csv-format`` option can be omitted.

.. _mola_csv:

MOLA, with radius in m
~~~~~~~~~~~~~~~~~~~~~~

MOLA point data for Mars can be downloaded for a given longitude-latitude extent
from the `ODE web tool <https://ode.rsl.wustl.edu/mars/datapointsearch.aspx>`_,
as a CSV file. The measurements include longitude, latitude (in degrees), and
radius from the planet center, in meters.

Unlike the LOLA RDR table above, the column order is not fixed, as it depends on
the fields selected in the query. Inspect the downloaded file and set
``--csv-format`` to match it, for example ``'1:lon 2:lat 3:radius_m'`` or
``'1:lon 2:lat 5:radius_m'``.

As an illustration, grid a MOLA cloud into a DEM with ``point2dem``
(:numref:`point2dem`)::

    point2dem -r mars                       \
      --stereographic                       \
      --auto-proj-center                    \
      --csv-format '1:lon 2:lat 5:radius_m' \
      mola.csv

The Mars datum and the MOLA data flavors (Topography, Radius, Areoid) carry some
subtleties; these are discussed in :numref:`molacmp`.

.. _poly_files:

Format of polygon files
-----------------------

The ``stereo_gui`` program can read and write polygons in the shapefile format,
and also in plain text with a ``.txt`` or ``.csv`` extension
(:numref:`plot_poly`). Here the plain text format is described.

The x and y coordinates are stored as columns side-by side. Individual polygons
are separated by an empty line. A color for the polygons is specified as a line
of the form: ``color = red``. The given color applies to all polygons on
subsequent lines until overridden by another such statement. How to create and
save such files is shown in :numref:`plot_poly`.

When such polygons are saved, a header will be added to the file, consisting of
lines starting with the pound sign, containing the WKT string for the
georeference, the value of ``--csv-format`` to interpret the vertices, and the
style (usually set to ``poly``). This allows for overlaying polygons with
different georeferences in ``stereo_gui``.

The plain text polygon file supports text labels. They should be on lines that
start with the text ``anno`` (annotation), followed by a space, then the x and y
coordinates, separated by spaces, then the text label.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
