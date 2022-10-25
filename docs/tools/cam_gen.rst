.. _cam_gen:

cam_gen
-------

This tool will create a Pinhole or Optical Bar camera model given
camera’s optical center, focal length, pixel pitch, the
longitude-latitude coordinates of the camera image corners (or some
other pixels) projected onto a DEM, and the DEM itself. A datum (and a
height above it) can be used instead of the DEM. Normally all these
inputs are known only approximately, so the output camera model will not
be quite precise either, yet it could be good enough to refine later
with bundle adjustment, which can also make use of the GCP file that
this tool creates.

This program can be used with historical images for which camera
position and orientation is not known. If the corners of the image on
the ground are not known, they could be guessed in Google Earth. A good
DEM to infer the heights from, at least for Earth, is the SRTM dataset.
:numref:`skysat` makes use of ``cam_gen`` for SkySat images.

Usage::

      cam_gen [options] <image-file> -o <camera-file>

Example::

     cam_gen --refine-camera --lon-lat-values                             \
       '-122.389 37.627,-122.354 37.626,-122.358 37.612,-122.393 37.613'  \
        --reference-dem dem.tif --focal-length 553846.153846              \
        --optical-center 1280 540 --pixel-pitch 1                         \
        img.tif -o img.tsai --gcp-file img.gcp --gcp-std 1e-2             \

Here we assume that the pixel pitch is 1, hence both the focal length
and the optical center are in units of pixels. If the focal length and
pixel pitch are given in meters, and one assumes the optical center to
be the center of the image, then the optical center passed to this tool
should be half of the image width and height, with both multiplied by
the pixel pitch, to make them in meters as well.

Some other pixels can be used instead of corners, if using the
``--pixel-values`` option.

Note that for Optical Bar cameras the camera parameters must be passed
in using the ``--sample-file`` option instead of specifying them all
manually. (See :numref:`kh9` for a longer discussion.)

It is strongly suggested to mapproject the image onto the obtained
camera to verify if it projects where expected::

     mapproject dem.tif img.tif img.tsai img_map.tif

The output ``img_map.tif`` can be overlayed onto the hillshaded DEM in
``stereo_gui``.

The camera obtained using this tool (whether with or without the
``--refine-camera`` option) can be further optimized in
``bundle_adjust`` using the GCP file written above as follows::

     bundle_adjust img.tif img.tsai img.gcp -o run/run --datum WGS84 \
       --inline-adjustments --robust-threshold 10000

It is suggested that this is avoided by default. One has to be a bit
careful when doing this optimization to ensure some corners are not
optimized at the expense of others. This is discussed in :numref:`imagecorners`.

One can invoke ``orbitviz`` as::

     orbitviz img.tif img.tsai -o orbit.kml

to visualize the computed camera above the ground in Google Earth.

This tool can also create a Pinhole camera approximating any camera
supported by ASP, such as from ISIS cubes, RPC cameras, etc., as long as
the intrinsics are known, as above. For that, it will shoot rays from
the image corners (and also some inner points) using the provided camera
that will intersect a reference DEM determining the footprint on the
ground, and then the best-fit pinhole model will be created based on
that. Here’s an example for ISIS cameras::

     cam_gen image.cub --input-camera image.cub --focal-length 1000       \
       --optical-center 500 300 --pixel-pitch 1 --height-above-datum 4000 \
       --gcp-std 1 --datum WGS84 --refine-camera --reference-dem dem.tif  \
       -o output.tsai --gcp-file output.gcp 

Here we passed the image as the input camera, since for ISIS cubes (and
also for some RPC cameras) the camera information is not stored in a
separate camera file.

Command-line options for cam_gen:

-o, --output-camera-file <file.tsai>
    Specify the output camera file with a .tsai extension.

--camera-type <pinhole|opticalbar (default: pinhole)>
    Specify the camera type

--lon-lat-values <string>
    A (quoted) string listing numbers, separated by commas or spaces,
    having the longitude and latitude (alternating and in this
    order) of each image corner. The corners are traversed in the
    order 0,0 w,0 w,h, 0,h where w and h are the image width and
    height.

--pixel-values <string>
    A (quoted) string listing numbers, separated by commas or spaces,
    having the column and row (alternating and in this order) of
    each pixel in the raw image at which the longitude and latitude
    is known. By default this is empty, and will be populated by
    the image corners traversed as earlier.

--reference-dem <filename>
    Use this DEM to infer the heights above datum of the image corners.

--datum <string>
    Use this datum to interpret the longitude and latitude, unless a
    DEM is given.
    Options:

    * WGS_1984
    * D_MOON (1,737,400 meters)
    * D_MARS (3,396,190 meters)
    * MOLA (3,396,000 meters)
    * NAD83
    * WGS72
    * NAD27
    * Earth (alias for WGS_1984)
    * Mars (alias for D_MARS)
    * Moon (alias for D_MOON)

--height-above-datum <float (default: 0)>
    Assume this height above datum in meters for the image corners
    unless read from the DEM.

--sample-file <filename>
    Instead of manually specifying all of the camera parameters,
    specify a sample camera model file on disk to read them from
    (see :numref:`kh9`, :numref:`file_format`, and
    :numref:`panoramic`).

--focal-length <float (default: 0)>
    The camera focal length.

--optical-center <float (default: 0 0)>
    The camera optical center.

--pixel-pitch <float (default: 0)>
    The camera pixel pitch.

--refine-camera
    After a rough initial camera is obtained, refine it using least
    squares.

--frame-index <filename>
    A file used to look up the longitude and latitude of image
    corners based on the image name, in the format provided by the
    SkySat video product.

--gcp-file <filename>
    If provided, save the image corner coordinates and heights in
    the GCP format to this file.

--gcp-std <double (default: 1)>
    The standard deviation for each GCP pixel, if saving a GCP file.
    A smaller value suggests a more reliable measurement, hence
    will be given more weight.

--input-camera <filename>
    Create the output pinhole camera approximating this camera.
    
--cam-height <float (default: 0)>
    If both this and --cam-weight are positive, enforce that the output camera is at this height above datum. For SkySat, if not set, read this from the frame index. 
    
--cam-weight <float (default: 0)>
    If positive, try to enforce the option --cam-height with this weight (bigger weight means try harder to enforce). Highly experimental.

-t, --session-type <string>
    Select the input camera model type. Normally this is auto-detected,
    but may need to be specified if the input camera model is in
    XML format. See :numref:`parallel_stereo_options` for options.

--bundle-adjust-prefix <path>
    Use the camera adjustment obtained by previously running
    bundle_adjust when providing an input camera.

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
