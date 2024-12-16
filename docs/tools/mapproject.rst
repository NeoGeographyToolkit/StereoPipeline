.. _mapproject:

mapproject
----------

The tool ``mapproject`` is used to orthorectify (map-project) a camera image
onto a DEM or datum. ASP is able to use map-projected images to run stereo, see
:numref:`mapproj-example`.

The ``mapproject`` program can be run using multiple processes and can be
distributed over multiple machines (options ``--nodes-list`` and
``--processes``). 

This is particularly useful for ISIS cameras, as in that case any single process
must use only one thread due to the limitations of ISIS. The tool splits the
image up into tiles, distributes the tiles to sub-processes, and then merges the
tiles into the requested output image. If the input image is small but takes a
while to process, smaller tiles can be used to start more simultaneous processes
(use the parameters ``--tile-size`` and ``--processes``).

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

If the resulting mapprojected images are used for stereo, it is very strongly
suggested to use the same grid size for all images (:numref:`mapproj-res`).

Examples
~~~~~~~~

Mapproject assuming the ``longlat`` projection and setting the grid
size in degrees::

     mapproject --tr 0.0001 DEM.tif image.tif camera.tsai output.tif

Map-project a .cub file (it has both image and camera information) for the Moon.
Use a custom stereographic projection::

    proj="+proj=stere +lat_0=-85.3643 +lon_0=31.2387 +R=1737400 +units=m +no_defs"

The grid size is set to 1 meter/pixel::

    mapproject --tr 1.0 --t_srs "$proj" DEM.tif image.cub output.tif

Map-project an image file with associated .xml camera file. Use bundle-adjusted cameras
(:numref:`bundle_adjust`)::

     mapproject -t rpc --bundle-adjust-prefix ba/run \
       DEM.tif image.tif image.xml output.tif

See :numref:`rpc` for other ways of specifying the camera model.

Mapproject using the CSM camera model (:numref:`csm`)::

    mapproject DEM.tif image.cub camera.json output.tif

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

.. _mapproj_metadata:

Saved metadata
~~~~~~~~~~~~~~

The output image will have the following metadata saved to its geoheader:
   
   * ``INPUT_IMAGE_FILE``, the input image name. 
   * ``BUNDLE_ADJUST_PREFIX``, the bundle adjustment prefix. Set to ``NONE`` if not present.
   * ``CAMERA_MODEL_TYPE``, this is the session name, such as set with ``-t rpc``.
   * ``CAMERA_FILE``, the camera file used on input. Can be empty if the camera is contained within the input image.
   * ``DEM_FILE``, the DEM used in mapprojection.

These metadata values are used to undo the mapprojection in stereo triangulation (:numref:`mapproj_reuse`). The geoheader can be inspected with ``gdalinfo`` (:numref:`gdal_tools`).

In addition, if the cameras have been bundle-adjusted, the translation and
quaternion rotation from the .adjust file will be saved to the fields
``ADJUSTMENT_TRANSLATION`` and ``ADJUSTMENT_QUATERNION``. This is useful for
having mapprojection be reproducible if the separately stored ``.adjust`` files
are not available.

These fields are editable with ``image_calc`` (:numref:`image_calc_metadata`),
but this is not recommended except for very experimental work.

Usage
~~~~~

::

     mapproject [options] <dem> <camera-image> <camera-model> <output-image>

.. _mapproj_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--nodata-value <float(default: -32768)>
    No-data value to use unless specified in the input image.

--t_srs <string (default: "")>
    Specify the output projection as a GDAL projection string (WKT, GeoJSON, or
    PROJ). If not provided, use the one from the DEM.

--tr <float>
    Set the output file resolution (ground sample distance) in target
    georeferenced units per pixel. This may be in degrees or meters,
    depending on your projection. The center of each output pixel
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
    Use nearest neighbor interpolation instead of bicubic interpolation. *This
    is not recommended, as it can result in artifacts.*

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
    List of available computing nodes to use. If not set, use the local
    machine. See also :numref:`pbs_slurm`.

--tile-size
    Size of square tiles to break up processing into. Each tile is run
    by an individual process. The default is 1024 pixels for ISIS
    cameras, as then each process is single-threaded, and 5120 pixels
    for other cameras, as such a process is multi-threaded, and disk
    I/O becomes a bigger consideration.

--query-projection
    Display the computed projection information and estimated ground
    sample distance (pixel size on the ground), and quit.

--query-pixel <double double>
    Trace a ray from this input image pixel (values start from 0) to the ground.
    Print the intersection point with the DEM as lon, lat, height, then as DEM
    column, row, height. Quit afterwards.
    
--parallel-options <string (default: "--sshdelay 0.2")>
    Options to pass directly to GNU Parallel.

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

--aster-use-csm
    Use the CSM model with ASTER cameras (``-t aster``).
    
--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display the help message.
