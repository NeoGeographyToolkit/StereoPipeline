.. _parallel_stereo:

parallel_stereo
---------------

The ``parallel_stereo`` program is a modification of ``stereo`` designed
to distribute the stereo processing over multiple computing nodes. It
uses GNU Parallel to manage the jobs, a tool which is distributed along
with Stereo Pipeline. It expects that all nodes can connect to each
other using ssh without password and that they share the same storage
space. ``parallel_stereo`` can also be useful when processing
extraterrestrial data on a single computer. This is because ISIS camera
models are restricted to a single thread, but ``parallel_stereo`` can
run multiple processes in parallel to reduce computation times.

At the simplest, ``parallel_stereo`` can be invoked exactly like
``stereo``, with the addition of the list of nodes to use (if using
multiple nodes).

::

     parallel_stereo --nodes-list machines.txt <other stereo options>

It will create the same output files as ``stereo``. Internally some of
them will be GDAL VRT files, that is, plain text virtual mosaics of
files created by individual processes, with the actual files in
subdirectories; ASP and GDAL tools are able to use these virtual files
in the same way as regular binary TIF files.

If your jobs are launched on a cluster or supercomputer, the name of the
file containing the list of nodes may exist as an environmental
variable. For example, on NASA's Pleiades Supercomputer, which uses the
Portable Batch System (PBS), the list of nodes can be retrieved as
$PBS_NODEFILE.

It is important to note that when invoking this tool only the
correlation, blending, subpixel refinement, and triangulation stages of
stereo (:numref:`stereo_dec`) are spread over multiple
machines, with the preprocessing and filtering stages using just one
node, as they require global knowledge of the data. In addition, not all
stages of stereo benefit equally from parallelization. Most likely to
gain are stages 1 and 2 (correlation and refinement) which are the most
computationally expensive.

For these reasons, while ``parallel_stereo`` can be called to do all
stages of stereo generation from start to finish in one command, it may
be more resource-efficient to invoke it using a single node for stages 0
and 3, many nodes for stages 1 and 2, and just a handful of nodes for
stage 4 (triangulation). For example, to invoke the tool only for stage
2, one uses the options::

     --entry-point 2 --stop-point 3

By default, stages 1, 2, and 4 of ``parallel_stereo`` use as many
processes as there are cores on each node, and one thread per process.
These can be customized as shown in the options below.

-h, --help
    Display the help message.

--nodes-list <filename>
    The list of computing nodes, one per line. If not provided, run
    on the local machine.

--ssh <filename>
    Specify the path to an alternate version of the ssh tool to use.

-e, --entry-point <integer (from 0 to 4)>
    Stereo Pipeline `entry point <entrypoints>`_ (start at this stage).

--stop-point <integer (from 1 to 5)>  Stereo Pipeline stop point (stop at
                                      the stage *right before* this).

--corr-seed-mode <integer (from 0 to 3)>  Correlation seed strategy
                                          (:numref:`corr_section`).

--sparse-disp-options <string (default: "")>
    Options to pass directly to sparse_disp (:numref:`sparse-disp`). Use quotes around this string.

--verbose
    Display the commands being executed.

--job-size-w <integer (default: 2048)>
    Pixel width of input image tile for a single process.

--job-size-h <integer (default: 2048)>
    Pixel height of input image tile for a single process.

--processes <integer>
    The number of processes to use per node.

--threads-multiprocess <integer>
    The number of threads to use per process.

--threads-singleprocess <integer>
    The number of threads to use when running a single process (for
    pre-processing and filtering).

--dry-run
    Do not launch the jobs, only print the commands that should be
    run.

--parallel-options <string (default: "")>
    Options to pass directly to GNU Parallel. For example, "--sshdelay 10 --controlmaster".
   
