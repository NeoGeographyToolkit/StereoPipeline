.. _parallel_bundle_adjust:

parallel_bundle_adjust
----------------------

The ``parallel_bundle_adjust`` program is a modification of
``bundle_adjust`` (:numref:`bundle_adjust`) designed to distribute
some of the preprocessing steps over multiple processes and multiple
computing nodes. It uses GNU Parallel to manage the jobs in the same
manner as ``parallel_stereo``.  For information on how to set up and
use the node list see :numref:`parallel_stereo`.

The ``parallel_bundle_adjust`` tool has three processing steps:
statistics, matching, and optimization. Only the first two steps can
be done in parallel, and in fact after you have run steps 0 and 1 in a
folder with ``parallel_bundle_adjust``, you could just call regular
``bundle_adjust`` to complete processing in the folder. Steps 0 and 1
produce the ``*-stats.tif`` and ``*.match`` files that are used in the last
step.

In particular, if you would like to repeat bundle adjustment, regular
``bundle_adjust`` would suffice as well, since the supporting data are
already created. If you would like to do some other experiments with
the cameras or the options of the tools, in a different output folder,
you can first copy there the existing content of an existing run, and
as before, call ``bundle_adjust``, rather than its parallel version.
(Also consider using ``--force-reuse-match-files`` and
``--skip-matching``, to prevent ``bundle_adjust`` to try again to
compute match files if the cameras are too new or if some match file
computation failed before and if it is likely to fail again if
re-attempted.)

The match files created by this tool can be used by
``bundle_adjust`` and ``parallel_stereo`` via the options
``--match-files-prefix`` and ``--clean-match-files-prefix``.

Command-line options for ``parallel_bundle_adjust``:

--nodes-list <filename>
    The list of computing nodes, one per line. If not provided, run
    on the local machine.

-e, --entry-point <integer (default: 0)>
    Bundle adjustment entry point (start at this stage).
    Options: statistics = 0, matching = 1, optimization = 2.

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
