.. _stereo:

stereo
------

The ``stereo`` program takes as input a stereo pair of images that
overlap with corresponding camera models and creates an output point cloud
image that can be processed into a visualizable mesh or a DEM using
:ref:`point2mesh` and :ref:`point2dem` respectively.

This program is deprecated and can be used only with ASP's original
block matching algorithm (``ASP_BM``). It is suggested to use instead
``parallel_stereo`` (:numref:`parallel_stereo`).

Usage::

    stereo [options] <images> [<cameras>] <output_file_prefix>

Example (for ISIS)::

     stereo file1.cub file2.cub results/run

Example (for Digital Globe Earth images)::

     stereo file1.tif file2.tif file1.xml file2.xml results/run

See :numref:`tutorial` for more details, and :numref:`examples` for
many usage examples.

Command-line options
~~~~~~~~~~~~~~~~~~~~

-t, --session-type <type_name>
    Select the stereo session type to use for processing. Usually the program can select
    this automatically by the file extension. See :numref:`parallel_stereo` for options.

-s, --stereo-file <filename (default: ./stereo.default)>
    Define the stereo.default file to use.

-e, --entry-point <integer (from 0 to 5)>
    Stereo Pipeline entry point. Start at this stage. See
    :numref:`entrypoints`.

--stop-point <integer (from 1 to 6)>  Stereo Pipeline stop point (stop at
                                      the stage *right before* this).

--corr-seed-mode <integer (from 0 to 3)>
    Correlation seed strategy (:numref:`corr_section`).

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.

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

-h, --help  Display the help message.

More information about additional options that can be passed to
``stereo`` via the command line or via the ``stereo.default``
configuration file can be found in :numref:`stereodefault`. ``stereo``
creates a set of intermediate files, they are described in
:numref:`outputfiles`.
