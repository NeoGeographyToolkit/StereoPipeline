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
GNU Parallel to manage the jobs, a program that is shipped with the
Stereo Pipeline. It expects that all nodes can connect to each other
using ssh without password and that they share the same storage space.

Usage::

    parallel_stereo [options] <images> [<cameras>] <output_file_prefix>

See :numref:`tutorial` for more details. Many examples of this
program are in :numref:`examples`.

This tool will create a set of output files (:numref:`outputfiles`). Internally
some of them will be GDAL VRT files, that is, plain text virtual mosaics of
files created by individual processes, with the actual files in subdirectories;
ASP and GDAL tools are able to use these virtual files in the same way as
regular binary TIF files. The files in subdirectories are combined into a single
file at the end of the run, and the subdirectories are deleted (option
``--keep-only``).

See :numref:`pbs_slurm` for how to set up this tool
for PBS and SLURM systems.

This program operates only on single channel (grayscale)
images. Multi-channel images need to first be converted to grayscale
or a single channel should be extracted with ``gdal_translate`` 
with the ``-b`` option.

Processes and threads
~~~~~~~~~~~~~~~~~~~~~

It is suggested that after this program is started, one examine how
well it uses the CPUs and memory on all nodes, especially at the
correlation stage (:numref:`entrypoints`). 

One may want to set the ``--processes``, ``--threads-multiprocess``,
and ``--threads-singleprocess`` options
(:numref:`parallel_stereo_options`), also ``--corr-memory-limit-mb``
(:numref:`stereodefault`). 

Make sure that ``--nodes-list`` is set, otherwise only the head node
will be used.

Note that the SGM and MGM algorithms can be quite memory-intensive. For these,
by default, the number of threads is set to 8, and the number of processes is
the number of cores divided by the number of threads, on each node. Otherwise,
the default is to use as many processes as there are cores.

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

The steps run by ``parallel_stereo`` are as follows. The output
files created by these steps are described in :numref:`outputfiles`.

Step 0 (Preprocessing)
    Runs ``stereo_pprc``. Normalizes the two images and aligns them by
    locating interest points and matching them in both images. The
    program is designed to reject outlier interest points. This stage
    writes out the pre-aligned images and the image masks. It also 
    computes the convergence angle for this stereo pair (for
    non-mapprojected images and with alignment method ``homography``,
    ``affineepipolar``, or ``local_epipolar``).

Step 1 (Stereo correlation)
    Runs ``stereo_corr``. Performs correlation using various
    algorithms which can be specified via ``--stereo-algorithm``.
    It writes a disparity map ending in ``D.tif``.

Step 2 (Blend)
    Runs ``stereo_blend``. Blend the borders of adjacent disparity map
    tiles obtained during stereo correlation. Needed for all stereo
    algorithms except the classical ``ASP_BM`` when run without local
    epipolar alignment. The result is the file ending in ``B.tif``.

Step 3 (Sub-pixel refinement)
    Runs ``stereo_rfne``. Performs sub-pixel correlation that refines
    the disparity map. Note that all stereo algorithms except
    ``ASP_BM`` already do their own refinement at step 1, however
    further refinement can happen at this step if the
    ``--subpixel-mode`` option is set. This produces a file ending in
    ``RD.tif``.

Step 4 (Outlier rejection)
    Runs ``stereo_fltr``. Performs filtering of the disparity map and
    (optionally) fills in holes using an inpainting algorithm. It creates
    ``F.tif``. Also computes ``GoodPixelMap.tif``.

Step 5 (Triangulation)
    Runs ``stereo_tri``. Generates a 3D triangulated point cloud from
    the disparity map by intersecting rays traced from the cameras.
    The output filename ends in ``PC.tif``.

It is important to note that since ``parallel_stereo`` can use a lot
of computational and storage resources, all the intermediate data up
to but not including triangulation can often be reused, if only the
cameras or camera adjustments change (for example, if the cameras got
moved, per :numref:`ba_pc_align`). Such reuse is discussed in
:numref:`bathy_reuse_run` (in the context of stereo with shallow
water).

If the program failed during correlation, such as because of
insufficient memory, it can be told to resume without recomputing the
existing good partial results with the option ``--resume-at-corr``.

.. _parallel_stereo_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--nodes-list <filename>
    The list of computing nodes, one per line. If not provided, run
    on the local machine. Alternatively, specify the full ssh command
    for each node, including the port, so one command per line.
    See examples for PBS and SLURM in :numref:`pbs_slurm`.

-t, --session-type <string>
    Select the stereo session type to use for processing. Usually the
    program can select this automatically by the file extension except
    for xml cameras. Options and when to use:
    
    - nadirpinhole -- for satellites/aircraft with pinhole cameras
      (:numref:`skysat`, :numref:`sfmicebridge`)
    - pinhole      -- ground-level cameras, not assuming a datum (:numref:`mer-example`)
    - isis         -- with planetary images stored in .cub files (:numref:`moc_tutorial`)
    - dg           -- with Digital Globe exact linescan cameras (:numref:`dg_tutorial`), which are implemented as CSM (:numref:`dg_csm`)
    - rpc          -- with any RPC cameras (:numref:`rpc`)
    - spot5        -- SPOT 5 exact linescan camera (:numref:`spot5`)
    - aster        -- exact ASTER camera model (:numref:`aster`)
    - perusat      -- PeruSat-1 exact linescan camera model (:numref:`perusat1`)
    - opticalbar   -- Optical Bar cameras (:numref:`kh4`)
    - csm          -- Community Sensor Model (:numref:`csm`)
    - pleiades     -- Pleiades satellites (:numref:`pleiades`)

    The next sessions are for mapprojected images
    (:numref:`mapproj-example`). If ``-t`` is specified
    as earlier, the sessions will be converted to the entities below
    automatically.

    - pinholemappinhole
    - isismapisis
    - dgmaprpc
    - rpcmaprpc
    - spot5maprpc
    - astermaprpc
    - opticalbarmapopticalbar
    - csmmapcsm / csmmaprpc
    - pleiadesmappleiades

-e, --entry-point <integer (from 0 to 5)>
    Stereo Pipeline entry point. Start at this stage. See
    :numref:`entrypoints`.

--stop-point <integer (from 1 to 6)> 
    Stereo Pipeline stop point (stop at the stage *right before*
    this).

--corr-seed-mode <integer (from 0 to 3)>
    Correlation seed strategy (:numref:`corr_section`).

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
    processes, for the correlation, subpixel refinement, and triangulation steps
    (:numref:`entrypoints`).

--threads-singleprocess <integer>
    The number of threads to use when running a single process (for
    the pre-processing and filtering steps, :numref:`entrypoints`).

--resume-at-corr
   Start at the correlation stage and skip recomputing the valid low
   and full-res disparities for that stage. Do not change
   ``--left-image-crop-win``, etc, when running this.

--prev-run-prefix
    Start at the triangulation stage while reusing the data from this 
    prefix. The new run can use different cameras, bundle adjustment
    prefix, or bathy planes (if applicable). Do not change crop
    windows, as that would invalidate the run. See
    :numref:`bathy_reuse_run` for an example.

--keep-only <string (default: "all_combined")>
    If set to ``all_combined``, which is the default, at the end of a
    successful run combine the results from subdirectories into ``.tif``
    files with the given output prefix, and delete the
    subdirectories. If set to ``essential``, keep only ``PC.tif`` and the
    files needed to recreate it (those ending with ``.exr``, ``-L.tif``,
    ``-F.tif``). If set to ``unchanged``, keep the run directory as it
    is. For fine-grained control, specify a quoted list of suffixes of
    files to keep, such as ``".exr .match -L.tif -PC.tif"``.
                                      
--verbose
    Display the commands being executed.

--dry-run
    Do not launch the jobs, only print the commands that should be
    run.

--ssh <filename>
    Specify the path to an alternate version of the ssh tool to use.

--parallel-options <string (default: "--sshdelay 0.2")>
    Options to pass directly to GNU Parallel.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
