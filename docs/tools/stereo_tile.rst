.. _stereo_tile:

stereo_tile
-----------

This is an auxiliary program invoked by :ref:`stereo_dist` to process a single
tile or to mosaic a block of DEMs. It runs the stereo pipeline stages
(preprocessing through DEM creation, :numref:`entrypoints`) on one tile, writing
results to a tile subdirectory. It can also run :ref:`dem_mosaic` on one block of
DEMs, as part of the final mosaic step in ``stereo_dist``.

.. _stereo_tile_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--tile-index <integer>
    Index of the tile to process (0-based). Required for tile processing.

-e, --entry-point <integer (default: 0)>
    Stereo pipeline entry point. Values: 0=pprc, 1=corr, 2=blend, 3=rfne,
    4=fltr, 5=tri, 6=cleanup, 7=dem.

--stop-point <integer (default: 9)>
    Stereo pipeline stop point (stop before this step).

--dem <string>
    Input DEM for mapprojection. Required for tile processing.

--point2dem-options <string>
    Options to pass to :ref:`point2dem`.

-s, --stereo-file <string (default: ./stereo.default)>
    Explicitly specify the ``stereo.default`` file to use
    (:numref:`stereodefault`). This file is optional and need not exist.

--corr-seed-mode <integer (from 0 to 3)>
    Correlation seed strategy (:numref:`corr_section`). Default is 0.

--sparse-disp-options <string>
    Options to pass directly to sparse_disp (:numref:`sparse_disp`).

--dem-mosaic-index <integer>
    Index of the mosaic block to process (0-based). Requires
    ``--dem-mosaic-master``. Invoked by ``stereo_dist`` for parallel DEM
    mosaicking.

--dem-mosaic-master <string>
    Master file enumerating for each block to mosaic the input DEM list file and
    output DEM path, one block per line. Used with ``--dem-mosaic-index``.

--verbose
    Display the commands being executed.

--dry-run
    Do not launch the jobs, only print the commands that should be run.

-v, --version
    Display the version of the software.

-h, --help
    Display this help message.
