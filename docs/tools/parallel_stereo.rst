.. _parallel_stereo:

parallel_stereo
---------------

The ``parallel_stereo`` program is the primary tool of the Ames Stereo
Pipeline.  It takes a stereo pair of images that overlap with
corresponding cameras and creates an output point cloud image that can
be processed into a visualizable mesh or a DEM using :ref:`point2mesh`
and :ref:`point2dem` respectively.

This program can distribute the stereo processing over multiple
computing nodes if invoked with the ``--nodes-list`` option. It uses
GNU Parallel to manage the jobs, a tool which is shipped with the
Stereo Pipeline. It expects that all nodes can connect to each other
using ssh without password and that they share the same storage space.

Usage::

    parallel_stereo [options] <images> [<cameras>] <output_file_prefix>

See :numref:`moc_tutorial` and :numref:`dg_tutorial` for more details.
Many examples of this program are in :numref:`examples`.

This tool will create a set of output files (:numref:`outputfiles`).
Internally some of them will be GDAL VRT files, that is, plain text
virtual mosaics of files created by individual processes, with the
actual files in subdirectories; ASP and GDAL tools are able to use
these virtual files in the same way as regular binary TIF files.

If the jobs are launched on a cluster or supercomputer, the name of the
file containing the list of nodes may exist as an environmental
variable. For example, on NASA's Pleiades Supercomputer, which uses the
Portable Batch System (PBS), the list of nodes can be retrieved as
$PBS_NODEFILE.

This program operates only on single channel (grayscale)
images. Multi-channel images need to first be converted to grayscale
or a single channel should be extracted with ``gdal_translate``
(otherwise the first channel will be picked quietly).

.. _entrypoints:

Entry points
~~~~~~~~~~~~

The ``parallel_stereo`` tool is written in Python, and invokes
separate C++ executables for various steps in processing. The
``--entry-point`` and ``--stop-point`` options can be used to run only
a portion of these steps. 

Only the correlation, blending, subpixel refinement, and triangulation
stages of ``parallel_stereo`` are spread over multiple machines, with
the preprocessing and filtering stages using just one node, as they
require global knowledge of the data. In addition, not all stages of
stereo benefit equally from parallelization. Most likely to gain are
stages 1 and 3 (correlation and refinement) which are the most
computationally expensive.

The invoked C++ executables have their own command-line options
(:numref:`stereodefault`). Those options can be passed to
``parallel_stereo`` which will in turn pass them on as needed. By
invoking each executable with no options, it will display the list of
options it accepts.

The steps run by ``parallel_stereo`` are as follows.

Step 0 (Preprocessing)
    Runs ``stereo_pprc``. Normalizes the two images and aligns them by
    locating interest points and matching them in both images. The
    program is designed to reject outlier interest points. This stage
    writes out the pre-aligned images and the image masks.

Step 1 (Stereo correlation)
    Runs ``stereo_corr``. Performs correlation using various
    algorithms which can be specified via ``--stereo-algorithm``.
    It writes a disparity map.

Step 2 (Blend)
    Runs ``stereo_blend``. Blend the borders of adjacent disparity map
    tiles obtained during stereo correlation. Needed for all stereo
    algorithms except the classical ``ASP_BM`` when run without local
    epipolar alignment.

Step 3 (Sub-pixel refinement)
    Runs ``stereo_rfne``. Performs sub-pixel correlation that refines
    the disparity map. Note that all stereo algorithms except
    ``ASP_BM`` already do their own refinement at step 1, however
    further refinement can happen at this step if the
    ``--subpixel-mode`` option is set.

Step 4 (Outlier rejection)
    Runs ``stereo_fltr``. Performs filtering of the disparity map and
    (optionally) fills in holes using an inpainting algorithm. Also
    also creates a "good pixel" map.

Step 5 (Triangulation)
    Runs ``stereo_tri``. Generates a 3D triangulated point cloud from
    the disparity map by intersecting rays traced from the cameras.


.. _parallel_stereo_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--nodes-list <filename>
    The list of computing nodes, one per line. If not provided, run
    on the local machine.

-t, --session-type <string>
    Select the stereo session type to use for processing. Usually the
    program can select this automatically by the file extension except
    for xml cameras. Options:
    
    - nadirpinhole
    - pinhole
    - isis
    - dg
    - rpc
    - spot5
    - aster
    - perusat
    - opticalbar
    - csm
    - pinholemappinhole
    - isismapisis
    - dgmaprpc
    - rpcmaprpc
    - spot5maprpc
    - astermaprpc
    - opticalbarmapopticalbar
    - csmmapcsm

-e, --entry-point <integer (from 0 to 5)>
    Stereo Pipeline entry point. Start at this stage. See
    :numref:`entrypoints`.

--stop-point <integer (from 1 to 6)>  Stereo Pipeline stop point (stop at
                                      the stage *right before* this).

--corr-seed-mode <integer (from 0 to 3)>  Correlation seed strategy
                                          (:numref:`corr_section`).

--sparse-disp-options <string (default: "")>
    Options to pass directly to sparse_disp
    (:numref:`sparse-disp`). Use quotes around this string.

--job-size-w <integer (default: 2048)>
    Pixel width of input image tile for a single process. For
    alignment method ``local_epipolar`` or algorithms apart from
    ``ASP_BM``, if not explicitly set, it is overridden by corr-tile-size
    + 2 * sgm-collar-size. See also :numref:`image_alignment`.

--job-size-h <integer (default: 2048)>
    Pixel height of input image tile for a single process.
    See also ``--job-size-w``.

--processes <integer>
    The number of processes to use per node.

--threads-multiprocess <integer>
    The number of threads to use per process when running multiple
    processes.

--threads-singleprocess <integer>
    The number of threads to use when running a single process (for
    pre-processing and filtering).

--verbose
    Display the commands being executed.

--dry-run
    Do not launch the jobs, only print the commands that should be
    run.

--ssh <filename>
    Specify the path to an alternate version of the ssh tool to use.

--parallel-options <string (default: "")>
    Options to pass directly to GNU Parallel. Example:
    "--sshdelay 1 --controlmaster".

-h, --help
    Display the help message.

