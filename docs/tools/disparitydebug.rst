.. _disparitydebug:

disparitydebug
--------------

The ``disparitydebug`` program produces visualizable images from
disparity maps created with ``parallel_stereo`` and ``stereo``. These
are named ``D_sub.tif``, ``D.tif``, ``RD.tif``, and ``F.tif`` (see
:numref:`outputfiles` for what each is).

The disparity map files can be useful for debugging because they
contain raw disparity values as measured by the correlator; however
they cannot be directly visualized or opened in a conventional image
browser.  The ``disparitydebug`` tool converts a single disparity map
file into two normalized TIFF image files (``*-H.tif`` and
``*-V.tif``, containing the horizontal and vertical, or line and
sample, components of disparity, respectively) that can be viewed
using any image display program, including with the ``stereo_gui``
tool shipped with ASP (:numref:`stereo_gui`).

The ``disparitydebug`` program will also print out the range of
disparity values in a disparity map, that can serve as useful summary
statistics when tuning the search range settings in the
``stereo.default`` file (:numref:`search_range`).

If the input images are map-projected (georeferenced), the outputs of
``disparitydebug`` will also be georeferenced.

Examples
~~~~~~~~

::

    disparitydebug run/run-D_sub.tif

View the obtained horizontal and vertical disparities with::

    stereo_gui run/run-D_sub-H.tif run/run-D_sub-V.tif 

Another example of using this tool (and a figure) is given in
:numref:`diagnosing_problems`, when discussing how to examine a produced run.

.. _raw_disp:

Raw disparity extraction
~~~~~~~~~~~~~~~~~~~~~~~~

To extract the horizontal and vertical bands from a disparity without
normalization, run a command such as::

    disparitydebug --raw run/run-F.tif

Invalid values are set to a no-data value (-1e+6) that is saved in the geoheader
of the output files.

This option is available in build 1/2026 and later.

Command-line options
~~~~~~~~~~~~~~~~~~~~

-o, --output-prefix <string (default: "")>
    Specify the output file prefix. This is set automatically if not provided.

--raw 
    Save the raw disparity values without any normalization. Invalid pixels are
    set to no-data.

--save-norm 
    Save the norm of the disparity instead of its two bands.

--save-norm-diff
    Save the maximum of norms of differences between a disparity and its four
    neighbors.
        
--normalization <(integer integer integer integer) (default = auto)>
    Normalization range. Specify in the format: hmin vmin hmax vmax.

--roi <(integer integer integer integer) (default = auto)>
    Region of interest. Specify in the format: xmin ymin xmax ymax.

-t, --output-filetype <string (default: tif)>
    Specify the output file type.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
