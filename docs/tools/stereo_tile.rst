.. _stereo_tile:

stereo_tile
-----------

This is an auxiliary program invoked by :ref:`stereo_dist` to process a single
tile. It runs the stereo pipeline stages (preprocessing through DEM creation,
:numref:`entrypoints`) on one tile, writing results to a tile subdirectory.

.. _stereo_tile_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--tile-index <integer>
    Index of the tile to process (0-based). Required.

--dem <string>
    Input DEM for mapprojection. Required.

--point2dem-options <string>
    Options to pass to :ref:`point2dem`.

-e, --entry-point <integer (default: 0)>
    Stereo pipeline entry point. Values: 0=pprc, 1=corr, 2=blend, 3=rfne,
    4=fltr, 5=tri, 6=cleanup, 7=dem.

--stop-point <integer (default: 9)>
    Stereo pipeline stop point (stop before this step).

-s, --stereo-file <string (default: ./stereo.default)>
    Explicitly specify the stereo.default file to use (:numref:`stereodefault`).
    This is optional.

--corr-seed-mode <integer (from 0 to 3)>
    Correlation seed strategy (:numref:`corr_section`).

--sparse-disp-options <string>
    Options to pass directly to sparse_disp (:numref:`sparse_disp`).

--verbose
    Display the commands being executed.

--dry-run
    Do not launch the jobs, only print the commands that should be run.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
