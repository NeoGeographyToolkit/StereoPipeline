.. _mapproject:

mapproject
----------

The tool ``mapproject`` is used to orthorectify (mapproject) a camera image
onto a DEM or datum. ASP is able to use mapprojected images to run stereo, see
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

If the resulting mapprojected images are used for stereo, all mapprojected
images should have the same grid size and projection (:numref:`mapproj-res`).

.. _mapproj_auto_proj:

Determination of projection
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the option ``--t_srs`` to set a desired output projection. The projection
should be local to the area of interest, in units of meter.

If this is not set, the projection from the DEM will be used, unless it is the
``longlat`` projection. In that case a good output projection is
auto-determined.

For Earth, with the WGS84 datum, the auto-determined projection is UTM
with an auto-computed zone, except for latitudes above 84° North and below 80°
South, where the `NSDIC polar stereographic projections
<https://nsidc.org/data/user-resources/help-center/guide-nsidcs-polar-stereographic-projection>`_
are used.

For other Earth datums and other planetary bodies, the automatic determination
produces a local stereographic projection. The projection center is found
by a median calculation based on of a sample of image pixels.
Or consider using the cylindrical equal area projection.

To ensure the automatic projection determination is always invoked, overriding
all other cases from above, use ``--t_srs auto``.

All mapprojected images passed to stereo should use the same projection and grid
size (:numref:`mapproj-example`).

Examples
~~~~~~~~

Earth image with auto projection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mapproject an image with a pinhole camera (:numref:`pinholemodels`), with a grid
size of 2 meters, for Earth (WGS84)::

     mapproject --tr 2.0 DEM.tif image.tif camera.tsai output.tif

If the DEM has a ``longlat`` projection, a projection in meters is found first 
(:numref:`mapproj_auto_proj`).

Moon image with a custom projection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mapproject a .cub file (:numref:`moc_tutorial`). Such a file has both image and
camera information. The planetary body is the the Moon. Use a custom
stereographic projection::

    proj="+proj=stere +lat_0=-85.364 +lon_0=31.238 +R=1737400 +units=m +no_defs"

The grid size is set to 1 meter/pixel::

    mapproject --tr 1.0 --t_srs "$proj" DEM.tif image.cub output.tif

RPC camera with bundle adjustment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mapproject an image file with an RPC camera model (:numref:`rpc`) in XML format.
Use bundle-adjusted cameras (:numref:`bundle_adjust`)::

     mapproject -t rpc --bundle-adjust-prefix ba/run \
       DEM.tif image.tif camera.xml output.tif

Here, the grid size is auto-determined.

See :numref:`rpc` for other ways of specifying the RPC camera model.

CSM camera
^^^^^^^^^^

Mapproject with the CSM camera model (:numref:`csm`)::

    mapproject -t csm DEM.tif image.cub camera.json output.tif

.. _mapproj_refmap:

Preexisting projection and grid size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The projection and grid size of a given mapprojected image can be borrowed when
mapprojecting another image::

    mapproject -t rpc                \
      --ref-map image1_map.tif       \
      DEM.tif image2.tif camera2.xml \
      image2_map.tif
      
This becomes important for stereo, when the two input mapprojected images
must share these attributes (:numref:`mapproj-example`).

Multiple camera models
^^^^^^^^^^^^^^^^^^^^^^

A DigitalGlobe / Maxar camera file has both an exact linescan model and 
an approximate RPC model. The RPC model is somewhat faster to use. 

To choose between these with ``mapproject``, invoke it either with ``-t dg``
or ``-t rpc``. See :numref:`dg_tutorial` for more information.

Mapproject with no DEM
^^^^^^^^^^^^^^^^^^^^^^

Mapproject onto the surface of zero height above a datum::

     mapproject -t rpc WGS84 image.tif image.xml output.tif

Valid datum names include WGS84, NAD83, NAD27, D_MOON, D_MARS, and
MOLA.

.. _mapproj_metadata:

Saved metadata
~~~~~~~~~~~~~~

The output image will have the following metadata saved to its geoheader:
   
   * ``INPUT_IMAGE_FILE``, the input image name. 
   * ``BUNDLE_ADJUST_PREFIX``, the bundle adjustment prefix. Set to ``NONE`` if not present.
   * ``CAMERA_MODEL_TYPE``, this is the session name, such as set with ``-t rpc``.
   * ``CAMERA_FILE``, the camera file used on input. Can be empty if the camera is contained within the input image.
   * ``DEM_FILE``, the DEM used in mapprojection.

These metadata values are used to undo the mapprojection in stereo triangulation
(:numref:`mapproj_reuse`). The geoheader can be inspected with ``gdalinfo``
(:numref:`gdal_tools`).

In addition, if the cameras have been bundle-adjusted, the translation and
quaternion rotation from the .adjust file will be saved to the fields
``ADJUSTMENT_TRANSLATION`` and ``ADJUSTMENT_QUATERNION``. This is useful for
having mapprojection be reproducible if the separately stored ``.adjust`` files
are not available.

These fields are editable with ``image_calc`` (:numref:`image_calc_metadata`),
but this is not recommended.

Usage
~~~~~

::

     mapproject [options] <dem> <camera-image> <camera-model> <output-image>

.. _mapproj_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--t_srs <string (default: "")>
    Specify the output projection as a GDAL projection string (WKT, GeoJSON, or
    PROJ). See :numref:`mapproj_auto_proj` for details.

--tr <float>
    Set the output file resolution (ground sample distance) in target
    georeferenced units per pixel. This may be in meters or degrees, depending
    on your projection. The center of each output pixel will be at integer
    multiples of this grid size (hence the output image will extend for an
    additional half a pixel at each edge).

-t, --session-type <string>
    Select the stereo session type to use for processing. 
    See :numref:`ps_options` for the list of types.

--t_projwin <xmin ymin xmax ymax>
    Limit the mapprojected image to this region, with the corners
    given in georeferenced coordinates (xmin ymin xmax ymax). Max
    is exclusive.

--t_pixelwin <xmin ymin xmax ymax>
    Limit the mapprojected image to this region, with the corners
    given in pixels (xmin ymin xmax ymax). Max is exclusive.

--bundle-adjust-prefix <name>
    Use the camera adjustment obtained by previously running
    bundle_adjust with this output prefix.

--ref-map <filename>
    Read the projection and grid size from this mapprojected image
    (:numref:`mapproj_refmap`).

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
    
--mpp <float>
    Set the output file resolution in meters per pixel.

--ppd <float>
    Set the output file resolution in pixels per degree.

--datum-offset <float>
    When projecting to a datum instead of a DEM, add this elevation
    offset to the datum.
    
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

--nodata-value <float(default: -32768)>
    No-data value to use unless specified in the input image.

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
