.. _n_align:

n_align
-------

This tool can be used to jointly align a set of two or more point
clouds, hence it extends the functionality of ``pc_align``
(:numref:`pc_align`). It implements the ICP flavor from
:cite:`toldo2010global`, more exactly, the MATLAB algorithm
at

https://searchcode.com/file/13619767/Code/matlab/GlobalProcrustesICP/globalProcrustes.m

It is hoped that joint alignment will give less biased results than
pairwise alignment for the clouds.

Usage::

     n_align <cloud files> -o <output prefix>

This tool supports the same types of data on input and output as
``pc_align``.

Even for two clouds this algorithm is not the same as the ones that are
part of ``pc_align``. This algorithm is expected to be more robust to
outliers than the regular ICP in ``pc_align`` since it uses a
cross-check. Yet, it may not handle a large translation difference
between the clouds as well. In that case, given a set of clouds, one can
first use ``pc_align`` to align all other clouds to the first one, then
invoke this algorithm for joint alignment while passing the obtained
alignment transforms as an argument to this tool, to be used as initial
guesses. The option to use for this, as shown below for simplicity for
three clouds, is::

     --initial-transforms 'identity.txt run_12/run-transform.txt run_13/run-transform.txt'

where the file ``identity.txt`` contains the 4 |times| 4 identity
matrix (the transform from the first cloud to itself), and ``run_12/run``
is the output prefix for ``pc_align`` when invoked on the first and
second clouds, etc. The final transforms output by this tool will
incorporate the initial guesses.

This tool should be less sensitive than ``pc_align`` to the order of the
clouds since any two of them are compared against each other. The number
of iterations and number of input points used will dramatically affect
its performance, and likely the accuracy. Cropping all clouds to the
same region is likely to to improve both run-time and the results.

Command-line options for n_align:

--num-iterations <arg (default: 100)>
    Maximum number of iterations.

--max-num-points <arg (default: 1000000)>
    Maximum number of (randomly picked) points from each cloud to
    use.

--csv-format <string>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center), 
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-proj4``; the height above datum is in
    meters).  Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-proj4 <proj string>
    The PROJ.4 string to use to interpret the entries in input CSV
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
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
