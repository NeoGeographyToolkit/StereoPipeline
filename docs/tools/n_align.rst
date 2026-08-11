.. _n_align:

n_align
-------

This tool jointly aligns a set of two or more point clouds, extending the
functionality of ``pc_align`` (:numref:`pc_align`). It implements the ICP flavor
from :cite:`toldo2010global`, more precisely
`this MATLAB algorithm <https://searchcode.com/file/13619767/Code/matlab/GlobalProcrustesICP/globalProcrustes.m>`_.

This program does not scale well for large clouds, so in practice ``pc_align``
is preferred, applied pairwise, relative to some larger reference cloud.

Example
~~~~~~~

Align three clouds to their common centroid::

    n_align --max-num-points 10000     \
      cloud1.tif cloud2.tif cloud3.tif \
      -o run/run

This writes one transform per input cloud, ``run-transform-0.txt``,
``run-transform-1.txt``, and ``run-transform-2.txt``. With
``--save-transformed-clouds``, the aligned clouds are saved as well.

.. _n_align_convergence:

Convergence and initial transforms
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This algorithm is not the same as the ones in ``pc_align``. It is expected to be
more robust to outliers, as it uses a cross-check, but it may not handle a large
translation between the clouds as well. In that case, first use ``pc_align`` to
align every other cloud to the first one, then pass the resulting transforms to
this tool as initial guesses, for joint refinement. For three clouds::

     --initial-transforms 'identity.txt
                           run_12/run-transform.txt
                           run_13/run-transform.txt'

Here ``identity.txt`` holds the 4 |times| 4 identity matrix (the transform from
the first cloud to itself), and ``run_12/run`` is the ``pc_align`` output prefix
for the first and second clouds, and so on. The transforms written on output
incorporate these initial guesses. The list of transforms can be on one line or
several, separated by newlines and/or whitespace. Do not add backslash line
continuations, as within the quotes a backslash becomes part of a file name.

This tool is less sensitive than ``pc_align`` to the order of the clouds, as any
two are compared against each other. The number of iterations and points used
strongly affect the run-time and accuracy. Cropping all clouds to the same
region tends to improve both.

.. _n_align_files:

Input files
~~~~~~~~~~~

This tool reads the same cloud types as ``pc_align`` and can also read GDAL
virtual file system paths (:numref:`vsi_paths`): the ASP point cloud format,
DEMs, LAS (including LAZ and COPC), and CSV.

For a COPC file, which can be very large, set ``--copc-win`` to the region to
read, in the projection of the file. All COPC inputs are cropped to this one
region, so they should overlap there. Use ``--copc-read-all`` to read the whole
file instead.

Usage
~~~~~

::

     n_align <cloud files> -o <output prefix>

Command-line options for n_align
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--num-iterations <arg (default: 100)>
    Maximum number of iterations.

--max-num-points <arg (default: 1000000)>
    Maximum number of (randomly picked) points from each cloud to
    use.

--copc-win <minx miny maxx maxy>
    Specify the region to read from any input COPC LAZ file, as all clouds
    should be cropped to the same region. The units are based on the projection
    in the file. This is required unless ``--copc-read-all`` is set. Specify as
    minx miny maxx maxy, or minx maxy maxx miny, with no quotes.

--copc-read-all
    Read the full input COPC files, ignoring the ``--copc-win`` option.

--csv-format <string>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center),
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-srs``; the height above datum is in
    meters).  Can also use radius_km for column_type, when it is
    again measured from planet center. See :numref:`csv_format` for details.

--csv-srs <proj string>
    The PROJ or WKT string to use to interpret the entries in input CSV
    files.

--datum <string>
    Use this datum for CSV files instead of auto-detecting it.  Options:

    - WGS_1984
    - D_MOON (1,737,400 meters)
    - D_MARS (3,396,190 meters)
    - MOLA (3,396,000 meters)
    - NAD83
    - WGS72
    - NAD27
    - Earth (alias for WGS_1984)
    - Mars (alias for D_MARS)
    - Moon (alias for D_MOON)

--semi-major-axis <arg (default: 0)>
    Explicitly set the datum semi-major axis in meters.

--semi-minor-axis <arg (default: 0)>
    Explicitly set the datum semi-minor axis in meters.

-o, --output-prefix <arg>
    Specify the output prefix. The computed alignment transforms
    and, if desired, the transformed clouds, will be saved to names
    starting with this prefix.

--save-transformed-clouds
    Apply the obtained alignment transforms to the input clouds and
    save them.

--initial-transforms-prefix <arg>
    The prefix of the transforms to be used as initial guesses. The
    naming convention is the same as for the transforms written on
    output.

--initial-transforms <arg>
    Specify the initial transforms as a list of files separated by
    spaces and in quotes, that is, as ``'trans1.txt ... trans_n.txt'``.

--relative-error-tolerance <tolerance (default: 1e-10)>
    Stop when the change in the error divided by the error itself
    is less than this.

--align-to-first-cloud
    Align the other clouds to the first one, rather than to their
    common centroid.

--verbose
    Print the alignment error after each iteration.

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

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
