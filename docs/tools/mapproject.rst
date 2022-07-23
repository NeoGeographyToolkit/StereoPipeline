.. _mapproject:

mapproject
----------

The tool ``mapproject`` is used to orthorectify (map-project) a camera
image onto a DEM or datum. (ASP is able to use map-projected images to
run stereo, see :numref:`mapproj-example`.)

The ``mapproject`` program can be run using multiple processes and can
be distributed over multiple machines. This is particularly useful for
ISIS cameras, as in that case any single process must use only one
thread due to the limitations of ISIS. The tool splits the image up
into tiles, distributes the tiles to sub-processes, and then merges
the tiles into the requested output image. If the input image is small
but takes a while to process, smaller tiles can be used to
start more simultaneous processes (use the parameters ``--tile-size``
and ``--processes``).

It is important to note that processing more tiles at a time may
actually slow things down, if all processes write to the same disk and
if processing each tile is dominated by the speed of writing to disk.
Hence some benchmarking may be necessary for your camera type and
storage setup.

The grid size, that is the dimension of pixels on the ground, set via
the ``--tr`` option, should be in units as expected by the projection
string obtained either from the DEM to project onto, or, if specified,
from the ``--t_srs`` option. If the grid size is not set, it will be
estimated as the mean *ground sampling distance (GSD)*.  See the
``--tr`` option for how this affects the extent of the output image.

Examples:

Mapproject assuming the ``longlat`` projection and setting the grid
size in degrees::

     mapproject --tr 0.0001 DEM.tif image.tif camera.tsai output.tif

Map-project a .cub file (it has both image and camera information)::

     mapproject -t isis --ppd 256 DEM.tif image.cub output.tif

Map-project an image file with associated .xml camera file::

     mapproject -t rpc --mpp 20 DEM.tif image.tif image.xml output.tif

Mapproject onto a datum rather than a DEM::

     mapproject WGS84 image.tif image.xml output.tif

Valid datum names include WGS84, NAD83, NAD27, D_MOON, D_MARS, and
MOLA.

If processing DigitalGlobe images, both the rigorous DG model
(``-t dg``) and its RPC approximation (``-t rpc``) from the XML metadata
file can be used for map projection. In practice, the latter is
recommended for most applications. The former is slightly more accurate,
but much slower.

If desired to change the range of longitudes from [0, 360] to [-180,
180], or vice-versa, post-process obtained mapprojected image with
``image_calc`` (:numref:`image_calc`).

Usage::

     mapproject [options] <dem> <camera-image> <camera-model> <output-image>

Command-line options for mapproject:

--nodata-value <float(default: -32768)>
    No-data value to use unless specified in the input image.

--t_srs <proj4 string>
    Specify the output projection (PROJ.4 string). If not provided,
    use the one from the DEM.

--tr <float>
    Set the output file resolution (ground sample distance) in target
    georeferenced units per pixel. The center of each output pixel
    will be at integer multiples of this grid size (hence the output
    image will extend for an additional half a pixel at each edge).

--mpp <float>
    Set the output file resolution in meters per pixel.

--ppd <float>
    Set the output file resolution in pixels per degree.

--datum-offset <float>
    When projecting to a datum instead of a DEM, add this elevation
    offset to the datum.

-t, --session-type <pinhole|isis|rpc>
    Select the stereo session type to use for processing. Choose
    ``rpc`` if it is desired to later do stereo with the ``dg`` session.

--t_projwin <xmin ymin xmax ymax>
    Limit the map-projected image to this region, with the corners
    given in georeferenced coordinates (xmin ymin xmax ymax). Max
    is exclusive.

--t_pixelwin <xmin ymin xmax ymax>
    Limit the map-projected image to this region, with the corners
    given in pixels (xmin ymin xmax ymax). Max is exclusive.

--bundle-adjust-prefix <name>
    Use the camera adjustment obtained by previously running
    bundle_adjust with this output prefix.

--ot <type (default: Float32)>
    Output data type, when the input is single channel. Supported
    types: Byte, UInt16, Int16, UInt32, Int32, Float32. If the
    output type is a kind of integer, values are rounded and then
    clamped to the limits of that type. This option will be ignored
    for multi-channel images, when the output type is set to be the
    same as the input type.

--nearest-neighbor
    Use nearest neighbor interpolation instead of bicubic
    interpolation.

--mo <string>
    Write metadata to the output file. Provide as a string in quotes
    if more than one item, separated by a space, such as
    ``VAR1=VALUE1 VAR2=VALUE2``.  Neither the variable names nor
    the values should contain spaces.

--processes <integer>
    Number of processes to use on each node (the default is for the
    program to choose).

--num-processes <integer>
    Same as --processes. Used for backwards compatibility.

--nodes-list
    List of available computing nodes.

--tile-size
    Size of square tiles to break up processing into. Each tile is run
    by an individual process. The default is 1024 pixels for ISIS
    cameras, as then each process is single-threaded, and 5120 pixels
    for other cameras, as such a process is multi-threaded, and disk
    I/O becomes a bigger consideration.

--enable-correct-velocity-aberration
    Turn on velocity aberration correction for Optical Bar and
    non-ISIS linescan cameras (:numref:`sensor_corrections`).
    This option impairs the convergence of bundle adjustment.

--enable-correct-atmospheric-refraction
    Turn on atmospheric refraction correction for Optical Bar and
    non-ISIS linescan cameras. This option impairs the convergence of
    bundle adjustment.

--query-projection
    Display the computed projection information and quit.

--no-geoheader-info
    Do not write information in the geoheader. Otherwise mapproject will
    write the camera model type, the bundle adjustment prefix used,
    the rotation and translation from the .adjust file, the DEM it
    mapprojected onto, and the value of the ``--mo`` option.

--suppress-output
    Suppress output from sub-processes.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB, for each process.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display the help message.
