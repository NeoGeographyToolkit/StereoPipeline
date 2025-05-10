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

Files created in preprocessing
------------------------------

\*-cropped\*.tif 
    Cropped versions of the input images, before alignment, when
    ``--left-image-crop-win`` and/or ``--right-image-crop-win`` are used.

\*.vwip 
    Interest point matches per image, before matching.
    
    If the images are ``left.cub`` and ``right.cub``, these files are called
    ``left.vwip`` and ``right.vwip``. They can also be generated (with finer
    grained-control over detection algorithm options) with ``ipfind``
    (:numref:`ipfind`).
    
    The ``.vwip`` files can be visualized in ``stereo_gui``
    (:numref:`stereo_gui_vwip_gcp`).

\*.match - image-to-image interest point matches (tie-points)
    The match file lists a select group of unique points out of the previous
    ``.vwip`` files that have been identified and matched in a pair of images.
    For example, if your images are ``left.cub`` and ``right.cub`` you'll get a
    ``left__right.match`` file. 

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
    shallow water (:numref:`shallow_water_bathy`).

\*-L_sub.tif, \*-R_sub.tif, \*-lMask_sub.tif, \*-rMask_sub.tif are
    low-resolution versions of the aligned left and right input images
    and corresponding masks.

\*-stereo.default - backup of the Stereo Pipeline settings file
    This is a copy of the ``stereo.default`` file used by ``parallel_stereo``.
    It is stored alongside the output products as a record of the
    settings that were used for this particular stereo processing task.

Files created during correlation
--------------------------------

\*-D_sub.tif - Low-resolution initial disparity (:numref:`d_sub`). 
    Computed at the correlation stage. Not recomputed when a run is
    resumed. The options ``--corr-seed-mode 2`` and ``3`` also produce
    \*-D_sub_spread.tif, which has the spread of this disparity. It is 
    in the same format as ``D.tif`` (below).
    
\*-D.tif - Full-resolution disparity map produced from the low-resolution disparity.
    It contains integer values of disparity that are used to seed the
    subsequent sub-pixel correlation phase. It is largely unfiltered,
    and may contain some bad matches.

    Disparity map files are stored in TIF format as 3-channel, 32-bit
    floating point images. Channel 0 = horizontal disparity, channel 1 =
    vertical disparity, and channel 2 = good pixel mask.
    
    The ``disparitydebug`` program (:numref:`disparitydebug`) can help inspect
    these. Or the bands can be extracted and visualized as in
    :numref:`mask_disparity`.

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

.. _poly_files:

Format of polygon files
-----------------------

The ``stereo_gui`` program can read and write polygons stored in plain text with
a ``.txt`` or ``.csv`` extension. The x and y coordinates are stored as columns
side-by side. Individual polygons are separated by an empty line. A color for
the polygons is specified as a line of the form: ``color = red``. The given
color applies to all polygons on subsequent lines until overridden by another
such statement. How to create and save such files is shown in :numref:`plot_poly`.

When such polygons are saved, a header will be added to the file, consisting of
lines starting with the pound sign, containing the WKT string for the
georeference, the value of ``--csv-format`` to interpret the vertices, and the
style (usually set to ``poly``). This allows for overlaying polygons with
different georeferences in ``stereo_gui``.


Inspection and properties of the output files
---------------------------------------------

All the output images that are single-band can be visualized in
``stereo_gui`` (:numref:`stereo_gui`). The disparities can be first
split into the individual horizontal and vertical disparity files
using ``disparitydebug`` (:numref:`disparitydebug`), then they can be
seen in this viewer as well.

If the input images are map-projected (georeferenced) and the
alignment method is ``none``, all the output images listed above, will
also be georeferenced, and hence can be overlayed in ``stereo_gui`` on
top of the input images (the outputs of ``disparitydebug`` will then
be georeferenced as well).

The point cloud file saves the datum (and projection if available)
inferred from the input images, regardless of whether these images
are map-projected or not.

The ``point2mesh`` (:numref:`point2mesh`) and ``point2dem``
(:numref:`point2dem`) programs can be used to convert the point cloud
to formats that are easier to visualize.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
