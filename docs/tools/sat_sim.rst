.. _sat_sim:

sat_sim
-------

The ``sat_sim`` satellite simulator program models a satellite traveling around
a planet and taking pictures. It can either create Pinhole camera
models (:numref:`pinholemodels`) or read them from disk. In either case it
creates synthetic images for the given cameras. 

The inputs are a DEM and georeferenced image (ortho image) of the area of
interest. If the input cameras are not specified, the orbit is determined by
given endpoints. It is represented as a straight edge in the projected
coordinate system of the DEM, which results in an arc around the planet. 

The images are created with bicubic interpolation in the ortho image and are
saved with float pixels. Missing pixels will have nodata values.

If the cameras are created from scratch, the camera view can follow a custom
path on the surface (:numref:`sat_sim_custom_path`).

Example (use given cameras)
^^^^^^^^^^^^^^^^^^^^^^^^^^^
::
  
    sat_sim --dem dem.tif --ortho ortho.tif          \
    --camera-list camera_list.txt                    \
    --image-size 800 600                             \
    -o run/run

The camera names in the list should be one per line. The produced image names
will be created from camera names by keeping the filename (without directory
name) and replacing the extension with ``.tif``. They will start with specified
output prefix. Hence, if the input camera is ``path/to/camera.tsai``, the output
image will be ``run/run-camera.tif``.

The value of ``--image-size`` should be chosen so that the ground sample
distance of the produced images is close to the ground sample distance of the
input ortho image. 

To see how a created image projects onto the ground, run ``mapproject``
(:numref:`mapproject`) as::

    mapproject dem.tif run/run-camera.tif path/to/camera.tsai \
      camera.map.tif

Example (generate nadir-pointing cameras)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::
  
    sat_sim --dem dem.tif --ortho ortho.tif              \
    --first 397.1 400.7 450000 --last 397.1 500.7 450000 \
    --num 5                                              \
    --focal-length 450000 --optical-center 500 500       \
    --image-size 1000 1000                               \
    -o run/run

The camera orientations are with the *x*, *y* and *z* axes pointing along
satellite track, across track, and towards the planet, respectively.

The focal length and camera elevations above the datum should be chosen
carefully. In this example, the camera is 450,000 m above the ground and the
focal length is 450,000 pixels. If the magnitude of DEM heights is within
several hundred meters, this will result in the ground sample distance being
around 1 meter per pixel.

The produced image and camera names will be along the lines of::
    
    run/run-10000.tif
    run/run-10000.tsai

.. _sat_sim_custom_path:

Example (follow custom ground path)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given two locations on the DEM, each specified by the column and row of DEM
pixel, to ensure that the center of the camera footprint travels along the straight
edge (in DEM pixel coordinates) between these, add to the tool options along the
lines of::

    --first-ground-pos 484.3 510.7 \
    --last-ground-pos 332.5 893.6    

This will result in the camera roll and pitch changing gradually to keep the
desired view.

Command-line options
^^^^^^^^^^^^^^^^^^^^

--dem <string (default="")>
    Input DEM file.

--ortho <string (default="")>
    Input georeferenced image file. 

-o, --output-prefix <string (default="")>
    Specify the output prefix. All the files that are saved will start with this
    prefix.

--camera-list <string (default="")>
    A file containing the list of pinhole cameras to create synthetic images
    for. Then these cameras will be used instead of generating them. Specify one
    file per line. The options ``--first``, ``--last``, ``--num``, ``--focal-length``,
    and ``--optical-center`` will be ignored.

--first <float, float, float>
    First camera position, specified as DEM pixel column and row, and height
    above the DEM datum.

--last <float, float, float>
    Last camera position, specified as DEM pixel column and row, and height
    above the DEM datum.

--num <int (default=0)>
    Number of cameras to generate, including the first and last ones. Must be
    positive. The cameras are uniformly distributed along the straight edge from
    first to last (in projected coordinates).

--first-ground-pos <float, float>
    Coordinates of first camera ground footprint center (DEM column and row). If
    not set, the cameras will look straight down (perpendicular to along and
    across track directions).

--last-ground-pos <float, float>
    Coordinates of last camera ground footprint center (DEM column and row). If
    not set, the cameras will look straight down (perpendicular to along and
    across track directions).

--focal-length <double>
    Output camera focal length in units of pixel.

--optical-center <float, float>
    Output camera optical center (image column and row).

--image-size <int, int>
    Output camera image size (width and height).

--dem-height-error-tol <float (default: 0.001)>
    When intersecting a ray with a DEM, use this as the height error tolerance
    (measured in meters). It is expected that the default will be always good
    enough.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use the value in
    ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--tif-compress <string (default = "LZW")>
    TIFF compression method. Options: None, LZW, Deflate, Packbits.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
