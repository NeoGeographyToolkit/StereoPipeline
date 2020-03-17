.. _stereo:

stereo
------

The ``stereo`` program is the primary tool of the Ames Stereo Pipeline.
It takes a stereo pair of images that overlap and creates an output
point cloud image that can be processed into a visualizable mesh or a
DEM using :ref:`point2mesh` and :ref:`point2dem` respectively.

Usage::

     ISIS> stereo [options] <images> [<cameras>] <output_file_prefix>

Options:

-h, --help  Display the help message.

-t, --session-type <type_name>
    Select the stereo session type to use for processing. Usually the program can select
    this automatically by the file extension.  Options:
    
    - nadirpinhole
    - pinhole
    - isis
    - dg
    - rpc
    - spot5
    - aster
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

-s, --stereo-file <filename (default: ./stereo.default)>
    Define the stereo.default file to use.

-e, --entry-point <integer (from 0 to 5)>
    Stereo Pipeline :ref:`entry point <entrypoints>` (start at this stage).

--stop-point <integer (from 1 to 6)>  Stereo Pipeline stop point (stop at
                                      the stage *right before* this).

--corr-seed-mode <integer (from 0 to 3)>  Correlation seed strategy
                                          (:numref:`corr_section`).

--threads <integer (default: 0)>  Set the number of threads to use.  Zero
                                  means use as many threads as there are cores.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tiff-comp <string (None|LZW|Deflate|Packbits)>
    TIFF compression method.


Example (for ISIS)::

     stereo file1.cub file2.cub results/run

For ISIS, a .cub file has both image and camera information, as such no
separate camera files are specified.

Example (for Digital Globe Earth images)::

     stereo file1.tif file2.tif file1.xml file2.xml results/run

Multiple input images are also supported (:numref:`multiview`).

This tool is is primarily designed to process USGS ISIS ``.cub`` files
and Digital Globe data. However, Stereo Pipeline does have the
capability to process other types of stereo image pairs (e.g., image
files with a CAHVOR camera model from the NASA MER rovers). If you would
like to experiment with these features, please contact the developers for more
information.

The ``<output_file_prefix>`` is prepended to all output data files. For
example, setting ``<output_file_prefix>`` to ‘``out``’ will yield files
with names like ``out-L.tif`` and ``out-PC.tif``. To keep the Stereo
Pipeline results organized in sub-directories, we recommend using an
output prefix like ‘``results-10-12-09/out``’ for
``<output_file_prefix>``. The ``stereo`` program will create a directory
called ``results-10-12-09/`` and place files named ``out-L.tif``,
``out-PC.tif``, etc. in that directory.

More information about additional options that can be passed to
``stereo`` via the command line or via the ``stereo.default``
configuration file can be found in :numref:`stereodefault`. ``stereo``
creates a set of intermediate files, they are described in
:numref:`outputfiles`.

.. _entrypoints:

Entry Points
~~~~~~~~~~~~

The ``stereo -e <number>`` option can be used to restart a ``stereo`` job
partway through the stereo correlation process. Restarting can be useful
when debugging while iterating on ``stereo.default`` settings.

Stage 0 (Preprocessing)
    Normalizes the two images and aligns them by locating interest
    points and matching them in both images. The program is designed
    to reject outlying interest points. This stage writes out the
    pre-aligned images and the image masks.

Stage 1 (Disparity Map Initialization)
    Performs pyramid correlation and builds a rough disparity map
    that is used to seed the sub-pixel refinement phase.

Stage 2 (Blend)
    Blend the borders of adjacent tiles. Only needed for parallel
    stereo with the SGM/MGM algorithms. Skipped otherwise.

Stage 3 (Sub-pixel Refinement)
    Performs sub-pixel correlation that refines the disparity map.

Stage 4 (Outlier Rejection and Hole Filling)
    Performs filtering of the disparity map and (optionally) fills
    in holes using an inpainting algorithm. This phase also creates
    a “good pixel” map.

Stage 5 (Triangulation)
    Generates a 3D point cloud from the disparity map.

.. _stereo_dec:

Decomposition of Stereo
~~~~~~~~~~~~~~~~~~~~~~~

The ``stereo`` executable is a Python script that makes calls to
separate C++ executables for each entry point.

Stage 0 (Preprocessing) calls ``stereo_pprc``. Multi-threaded.

Stage 1 (Disparity Map Initialization) calls ``stereo_corr``.
Multi-threaded.

Stage 2 (Blend) class ``stereo_blend``. Multi-threaded.

Stage 3 (Sub-pixel Refinement) class ``stereo_rfne``. Multi-threaded.

Stage 4 (Outlier Rejection and Hole Filling) calls ``stereo_fltr``.
Multi-threaded.

Stage 5 (Triangulation) calls ``stereo_tri``. Multi-threaded, except for
ISIS input data.

All of the sub-programs have the same interface as ``stereo``. Users
processing a large number of stereo pairs on a cluster may find it
advantageous to call these executables in their own manner. An example
would be to run stages 0-4 in order for each stereo pair. Then run
several sessions of ``stereo_tri`` since it is single-threaded for ISIS.

It is important to note that each of the C++ stereo executables invoked
by ``stereo`` have their own command-line options. Those options can be
passed to ``stereo`` which will in turn pass them to the appropriate
executable. By invoking each executable with no options, it will display
the list of options it accepts.

As explained in more detail in :numref:`perform-stereo`, each such
option has the same syntax as used in ``stereo.default``, while
being prepended by a double hyphen (``--``). A command line option
takes precedence over the same option specified in ``stereo.default``.
:numref:`stereodefault` documents all options for the individual
sub-programs.

Note that the stereo tools operate only on single channel (grayscale)
images. If you need to run stereo on multi-channel images you must first
convert them to grayscale or extract a single channel to operate on.
