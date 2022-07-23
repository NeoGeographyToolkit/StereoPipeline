.. _aster2asp:

aster2asp
---------

The ``aster2asp`` tool takes as input a directory containing ASTER
images and associated metadata, and creates TIF and XML files that can
then be passed to ``stereo`` to create a point cloud.

An example for how fetch the data and use this tool is given in
:numref:`aster`.

The tool can only process Level 1A ASTER images. The input should be a
directory containing visible and near-infrared (VNIR) nadir (Band3N) and
backward (Band3B) images, together with plain text files containing
values for the satellite positions, sight vectors, longitudes,
latitudes, lattice points, and radiometric correction tables. These
files are described in :cite:`abrams2002aster`.

Usage::

     aster2asp <input directory> -o <output prefix>

The tool will apply the existing radiometric corrections to the the
images, and save two images with Float32 pixels with names like
``out-Band3N.tif`` and ``out-Band3B.tif``. Based on the metadata
mentioned earlier, it will create approximate RPC camera models in XML
format (:numref:`rpc`) for the left and right cameras,
following :cite:`girod2015improvement`, with names of the
form ``out-Band3N.xml`` and ``out-Band3B.xml`` (we do not perform yet
any jitter corrections as described in that paper).

These can then be passed to ``stereo`` as::

     stereo -t rpc out-Band3N.tif out-Band3B.tif     \
        out-Band3N.xml out-Band3B.xml out_stereo/run

(See :numref:`nextsteps` for a discussion about various stereo
algorithms and speed-vs-quality choices.)

It is important to note that the tool expects the minimum and maximum
simulation box heights (in meters, above the datum) in which to compute
the RPC approximation. The defaults are 0 and 8000, corresponding to sea
level and the highest location on Earth. Narrowing down these numbers
(if it is known what range of terrain heights is expected) may result in
slightly more accurate models.

Command-line options for aster2asp:

-o, --output-prefix <arg>
    Specify the output prefix.

--min-height <arg (default: 0)>
    The minimum height (in meters) above the WGS84 datum of the
    simulation box in which to compute the RPC approximation.

--max-height <arg (default: 8000)>
    The maximum height (in meters) above the WGS84 datum of the
    simulation box in which to compute the RPC approximation.

--num-samples <arg (default: 100)>
    How many samples to use between the minimum and maximum heights.

--penalty-weight <arg (default: 0.1)>
    Penalty weight to use to keep the higher-order RPC coefficients
    small. Higher penalty weight results in smaller such coefficients.

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
