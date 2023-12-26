.. _parallel_sfs:

parallel_sfs
------------

The program ``parallel_sfs`` is a wrapper around ``sfs``
(:numref:`sfs`) meant to divide the input DEM into tiles with overlap,
run ``sfs`` on each tile as multiple processes, potentially on
multiple machines, and then merge the results into a single output
DEM. It has the same options as ``sfs``, and a few additional ones, as
outlined below.

Examples for how to invoke it are in the :ref:`SfS usage <sfs_usage>`
chapter.

Usage::

    parallel_sfs -i <input DEM> -n <max iterations> -o <output prefix> \
      <images> [other options]

Command-line options for ``parallel_sfs``:

--tile-size <integer (default: 300)>
    Size of approximately square tiles to break up processing into
    (not counting the padding).

--padding <integer (default: 50)>
    How much to expand a tile in each direction. This helps with
    reducing artifacts in the final mosaicked SfS output.

--processes <integer>
    Number of processes to use on each node (the default is for the
    program to choose).

--num-processes <integer>
    Same as ``--processes``. Used for backwards compatibility.

--nodes-list <filename>
    A file containing the list of computing nodes, one per line.
    If not provided, run on the local machine. See also
    :numref:`pbs_slurm`.

--threads <integer (default: 8)>
    How many threads each process should use. This will be changed to 
    1 for ISIS cameras when ``--use-approx-camera-models`` is not set 
    (:numref:`sfs`), as ISIS is single-threaded. Not all parts of the
    computation benefit from parallelization.

--parallel-options <string (default: "--sshdelay 0.2")>
    Options to pass directly to GNU Parallel.

--resume
    Resume a partially done run. Only process the tiles for which the
    desired per-tile output files are missing or invalid (as checked
    by ``gdalinfo``).

--suppress-output
    Suppress output of sub-calls.

-v, --version
    Display the version of software.

-h, --help
    Display the help message.
