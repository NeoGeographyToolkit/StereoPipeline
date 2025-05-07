.. _parallel_bundle_adjust:

parallel_bundle_adjust
----------------------

The ``parallel_bundle_adjust`` program is a modification of
``bundle_adjust`` (:numref:`bundle_adjust`) designed to distribute
some of the preprocessing steps over multiple processes and multiple
computing nodes. It uses GNU Parallel to manage the jobs in the same
manner as ``parallel_stereo`` (:numref:`parallel_stereo`).

An example is in :numref:`skysat_bundle_adjustment`.

Setting up the nodes list is discussed in :numref:`pbs_slurm`.

Processing steps
~~~~~~~~~~~~~~~~

The ``parallel_bundle_adjust`` tool has three processing steps: 0: statistics
computation, 1: interest point matching, 2: optimization.

Steps 0 and 1 produce the ``*-stats.tif`` and ``*.match`` files.

Only the first two steps can be done in parallel. In the last step,
``bundle_adjust`` is invoked as a single process, with the data produced so far. 

Use of results
~~~~~~~~~~~~~~

The files created by ``parallel_bundle_adjust`` can be used by later 
invocations of ``bundle_adjust`` or with ``parallel_stereo`` with the 
options ``--match-files-prefix`` and ``--clean-match-files-prefix``
(:numref:`ba_options`).

If ``bundle_adjust`` is called with the same output prefix as
``parallel_bundle_adjust``, and without the options above, it will try to see if
some match files are missing and need to be created. To avoid that, use the
options ``--force-reuse-match-files`` and ``--skip-matching``.

Parallelization
~~~~~~~~~~~~~~~

The default number of processes per node is 1/4 of the number of cores on the
head node, and the default number of threads per process is the number of cores
on the head node over the number of processes.

The number of launched jobs over all nodes is number of nodes times number of
processes per node.

Command-line options
~~~~~~~~~~~~~~~~~~~~

These options are in addition to the ones for ``bundle_adjust``
(:numref:`ba_options`).

--nodes-list <filename>
    The list of computing nodes, one per line. If not provided, run
    on the local machine.

-e, --entry-point <integer (default: 0)>
    Bundle adjustment entry point (start at this stage).
    Options: statistics and interest points per image = 0, 
    interest point matching = 1, optimization = 2.

--stop-point <integer(default: 3)>
    Bundle adjustment stop point (stop *before* this stage).
    Options: statistics = 0, matching = 1, optimization = 2, 
    all = 3.

--parallel-options <string (default: "--sshdelay 0.2")>
    Options to pass directly to GNU Parallel.

--verbose
    Display the commands being executed.

--processes <integer>
    The number of processes per node. The default is a quarter of the number of
    cores on the head node.

--threads <integer>
    The number of threads per process. The default is the number of cores on the
    head node over the number of processes.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB, for each process.

-v, --version
    Display the version of software.

-h, --help
    Display the help message.
