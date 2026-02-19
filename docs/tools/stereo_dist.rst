.. _stereo_dist:

stereo_dist
-----------

The ``stereo_dist`` program performs distributed stereo processing.  Nearly all
steps are run on small tiles in parallel across a set of computing nodes. This
also includes the preprocessing and filtering steps, which are serial in
:ref:`parallel_stereo`. DEM creation per tile and mosaicking of produced DEMs are
also distributed.

This program requires that input images be mapprojected, or the ``--mapproject``
option be used to mapproject them automatically. GNU Parallel manages the jobs.
It is expected that all nodes can connect to each other using ssh without
password and that they share the same storage space.

.. _stereo_dist_example:

Example with mapprojected images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    stereo_dist                       \
      left_map.tif right_map.tif      \
      left.json right.json            \
      run/run                         \
      --dem input_dem.tif             \
      --tile-size 2048                \
      --tile-padding 256              \
      --nodes-list machines.txt       \
      --processes 5                   \
      --threads-multiprocess 4        \
      --threads-singleprocess 16      \
      --point2dem-options '--tr 2.0'

The input images must be mapprojected (:numref:`mapproj-example`). The
mapprojection DEM should be set with ``--dem``. All output files
(:numref:`outputfiles`) will start with the provided prefix (``run/run`` above).

The result of this program will be the stereo DEM. The workflow in
:numref:`stereo_dist_workflow` has more details.

The option ``--point2dem-options`` must include ``--tr`` and the grid size
(typically in meters) to ensure a consistent grid size across all tiles that are
later merged. This is passed to :ref:`point2dem`.

This program can be run on PBS and SLURM systems in a manner analogous to
:ref:`parallel_stereo`. See :numref:`pbs_slurm` for details.

.. _stereo_dist_mapproject:

Example with raw images
~~~~~~~~~~~~~~~~~~~~~~~

With the ``--mapproject`` option, raw (not mapprojected) images can be specified
as inputs. The program will mapproject both images onto the DEM before running
stereo. This program can be invoked as::

    stereo_dist           \
      --mapproject        \
      left.cub right.cub  \
      run/run             \
      --dem input_dem.tif \
      --tile-size 2048    \
      --tile-padding 256  \
      --point2dem-options '--tr 2.0'

Here we omitted the cameras as those are contained in the cub files.

The projection is auto-determined based on the DEM and the left image
(:numref:`mapproj_auto_proj`).

The grid size is set to the minimum of those estimated from the input images.

The mapprojection region is the intersection of the regions for the two
images.

Any of these can be explicitly set with ``--t_srs``, ``--tr``, and ``--t_projwin``
(:numref:`stereo_dist_options`).

The mapprojected images are saved as ``<output prefix>-left_map.tif`` and
``<output prefix>-right_map.tif``. Then, the same processing is done as
with pre-existing mapprojected images.

.. _stereo_dist_workflow:

Workflow
~~~~~~~~

The processing is as follows:

- The images are mapprojected (with the ``--mapproject`` option only).

- Statistics is computed for the input mapprojected images. This is a serial
  process but is quite fast.

- The list of tiles is created and saved with a name based on the output prefix
  and ending in ``-distTileList.txt``.

- All tiles are run in parallel with the :ref:`stereo_tile` program. That
  program crates a subdirectory for the given tile and provides the global
  statistics via a symbolic link. It then does preprocessing, correlation,
  refinement, filtering, triangulation, and DEM creation. These operations are
  described in :numref:`entrypoints`.

- The per-tile DEMs are mosaicked into a single output DEM using
  :ref:`dem_mosaic`. This is done in parallel as well, for subsets of DEMs,
  whose results are then mosaicked together.

Unlike :ref:`parallel_stereo`, the blend step for disparities is skipped. Each
tile is processed fully independently, and blending only happens between DEMs
during the final mosaic.

The ``--entry-point`` and ``--stop-point`` options can be invoked to run only a
portion of these steps. See :numref:`stereo_dist_options` for the step numbers.

Any options that are not specific to this program are passed directly to 
:ref:`stereo_tile` and the stereo executables (:numref:`cmdline`).

.. _stereo_dist_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--dem <string>
    Input DEM. Required. Used for mapprojection (with ``--mapproject``)
    and for stereo triangulation.

--mapproject
    Mapproject the input images onto the DEM before running stereo. The
    projection, grid size, and bounding box are auto-determined to be
    consistent for both images. See :numref:`stereo_dist_mapproject`.

--t_srs <string>
    Output projection for mapprojection. A PROJ string, EPSG code, or path
    to a WKT file. Auto-determined from the left image if not set. Only
    used with ``--mapproject``.

--tr <float>
    Output grid size (ground sample distance) for mapprojection, in units of
    the projection. Auto-determined as the finer of the two images if not
    set. Only used with ``--mapproject``.

--t_projwin <xmin ymin xmax ymax>
    Bounding box for mapprojection in projected coordinates. Auto-determined
    as the intersection of the two image footprints if not set. Only used
    with ``--mapproject``.

--tile-size <integer (default: 2048)>
    Size of each tile (in pixels) for distributed processing. This is before the
    padding is added.

--tile-padding <integer (default: 256)>
    Padding around each tile (in pixels) to avoid boundary artifacts.

--point2dem-options <string>
    Options to pass to :ref:`point2dem`. Must include ``--tr`` to set
    the grid size for consistent tiling.

--nodes-list <filename>
    A file containing the list of computing nodes, one per line. If not
    provided, run on the local machine.

--processes <integer>
    The number of processes to use per node.

--threads-multiprocess <integer>
    The number of threads to use per process when running multiple processes.

--threads-singleprocess <integer>
    The number of threads to use when running a single process.

-e, --entry-point <integer (default: 0)>
    Stereo pipeline entry point. Values: 0=pprc, 1=corr, 2=blend (skipped,
    kept for compatibility with :ref:`parallel_stereo` steps), 3=rfne,
    4=fltr, 5=tri, 6=cleanup, 7=dem, 8=mosaic.

--stop-point <integer (default: 9)>
    Stereo pipeline stop point (stop before this step).

--parallel-options <string (default: "--sshdelay 0.2")>
    Options to pass directly to GNU Parallel.

--verbose
    Display the commands being executed.

--dry-run
    Do not launch the jobs, only print the commands that should be run.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
