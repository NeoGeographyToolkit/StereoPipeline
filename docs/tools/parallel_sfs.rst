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

Command-line options for parallel_sfs:

--tile-size <integer (default: 300)>
    Size of approximately square tiles to break up processing into
    (not counting the padding).

--padding <integer (default: 50)>
    How much to expand a tile in each direction. This helps with
    reducing artifacts in the final mosaicked SfS output.

--num-processes <integer>
    Number of processes to use (the default program tries to choose
    best).

--nodes-list <filename>
    A file containing the list of computing nodes, one per line.
    If not provided, run on the local machine.

--threads <integer (default: 1)>
    How many threads each process should use. The sfs executable
    is single-threaded in most of its execution, so a large number
    will not help here.

--suppress-output
    Suppress output of sub-calls.
