.. _cam_gen:

cam_gen
-------

This tool can create Pinhole (:numref:`pinholemodels`), Optical Bar
(:numref:`panoramic`), and CSM (:numref:`csm`) camera models, given camera's
optical center, focal length, pixel pitch, the longitude-latitude coordinates of
the camera image corners (or some other pixels) as measured on a DEM. It can
also approximate any camera supported by ASP.

A datum (and a height above it) can be used instead of the DEM. Normally all
these inputs are known only approximately, so the output camera model will not
be quite precise either, yet it could be good enough to refine later with bundle
adjustment, which can also make use of the GCP file that this tool creates.

This program can be used with historical images (:numref:`kh4`) for which camera
position and orientation is not known. If the corners of the image on the ground
are not known, they could be guessed in Google Earth. :numref:`skysat` makes use
of ``cam_gen`` for SkySat images.

The accuracy of this tool decreases as the field of view becomes very narrow. In
that case it is suggested to use it to approximate a good known camera rather
than creating a camera from scratch given corner ground coordinates
(:numref:`cam_gen_prior`).

See :numref:`initial_terrain` for how to find a good DEM to infer the heights
from.

Usage::

      cam_gen [options] <image-file> -o <camera-file>

Examples
~~~~~~~~

.. _cam_gen_pinhole:

Pinhole cameras
^^^^^^^^^^^^^^^

::

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

This procedure is not as accurate as approximating an existing camera
(:numref:`cam_gen_prior`).
  
Some other pixels can be used instead of corners, if using the
``--pixel-values`` option.

Optical bar cameras
^^^^^^^^^^^^^^^^^^^

For optical bar cameras, the camera parameters must be passed in using the
``--sample-file`` option instead of specifying them all manually. This is 
discussed in :numref:`kh9`.

.. _cam_gen_prior:

Fit a prior camera
^^^^^^^^^^^^^^^^^^

This tool can also create a Pinhole camera approximating any camera supported by
ASP, such as from ISIS cubes, RPC cameras, etc., as long as the intrinsics are
known, as above. For that, it will shoot rays from the image corners (and also
some inner points) using the provided camera that will intersect the provided
DEM, determining the footprint on the ground. This will be used to find the
best-fit pinhole model. 

In this case, the corner longitude-latitude coordinates need not be specified.

Here is an example for ISIS cameras::

     cam_gen image.cub --input-camera image.cub     \
       --focal-length 1000 --optical-center 500 300 \
       --pixel-pitch 1                              \
       --gcp-std 1 --refine-camera                  \
       --reference-dem dem.tif                      \
       --height-above-datum 4000                    \
       -o output.tsai --gcp-file output.gcp 

Here we passed the image as the input camera, since for ISIS cubes (and
also for some RPC cameras) the camera information is not stored in a
separate camera file.

This does not model distortion. For that, one has to produce CSM cameras
(:numref:`cam_gen_frame`).

Ensure the correct datum is passed for your planet, if a DEM is not used on
input. For example: ``--datum D_MARS``. 

The ``--height-above-datum`` option will not be used if the input DEM covers the
image ground footprint.

.. _cam_gen_frame:

CSM Frame cameras
^^^^^^^^^^^^^^^^^

This program can create a CSM Frame camera (:numref:`csm`) that approximates any
camera supported by ASP. 

In this mode, distortion is modeled as well. An additional solver pass can be
invoked, which can refine the intrinsics, that is, the focal length, optical
center, and the distortion coefficients, in addition to the camera pose. See the
``--distortion`` option in :numref:`cam_gen_options` for the distortion model.

Good initial guesses, especially for the focal length and optical center, are
still expected.

Example::

  cam_gen input.tif                             \
    --input-camera input.xml                    \
    --reference-dem dem.tif                     \
    --focal-length 30000                        \
    --optical-center 3000 2000                  \
    --pixel-pitch 1                             \
    --refine-camera                             \
    --refine-intrinsics focal_length,distortion \
    -o output.json                              \

The pixel pitch must always be 1, so the focal length and optical center must be
in units of pixel.

It is suggested to not optimize the optical center, as that correlates with the
camera pose and can lead to an implausible solution. The ``--distortion`` option
need not be set, as the program will try to figure that out.

If invoked with ``--refine-intrinsics none``, the provided intrinsics will be
passed to the CSM model, but then only the camera pose will be refined. This
is different than just using ``--refine-camera`` alone, which does not support
distortion.

If the camera model is contained within the image, pass the image to
``--input-camera``.

To transfer the intrinsics produced by the invocation above to another camera
acquired with the same sensor, run::

  cam_gen input2.tif            \
    --input-camera input2.xml   \
    --reference-dem dem.tif     \
    --pixel-pitch 1             \
    --refine-camera             \
    --refine-intrinsics none    \
    --sample-file output.json   \
    -o output2.json             \

The produced camera intrinsics can be jointly refined with other frame or
linescan cameras using ``bundle_adjust`` (:numref:`ba_frame_linescan`).
 
The ``cam_test`` program (:numref:`cam_test`) can evaluate the agreement between
the input and output cameras.

.. _cam_gen_linescan:

CSM linescan cameras
^^^^^^^^^^^^^^^^^^^^

This program can take as input a linescan camera, such as WorldView
(:numref:`dg_tutorial`), Pleiades (:numref:`pleiades`), ASTER (:numref:`aster`),
and CSM (:numref:`csm`), and convert it to the CSM linescan model state format
(:numref:`csm_state`). This allows one to use ASP with a combination of
linescan cameras from different vendors and also with Frame cameras
(:numref:`ba_frame_linescan`).

An example is as follows::

    cam_gen --camera-type linescan       \
      input.tif --input-camera input.xml \
      -o output.json

The option ``--bundle-adjust-prefix`` can be used to apply an adjustment to the
camera on loading.

The ``cam_test`` program (:numref:`cam_test`) can verify the agreement between
the input and output cameras. Do not specify the ``--bundle-adjust-prefix``
option for such experiments, as the original camera does not have the adjustment
applied to it, the produced one does, and ``cam_test`` will apply such an
adjustment to both.

If desired to create linescan cameras to given specifications, use instead
``sat_sim`` (:numref:`sat_sim`).
          
Further refinement
~~~~~~~~~~~~~~~~~~

The camera obtained using this tool (whether with or without the
``--refine-camera`` option) can be re-optimized in
``bundle_adjust`` using the GCP file written above as follows::

     bundle_adjust img.tif img.tsai img.gcp -o run/run --datum WGS84 \
       --inline-adjustments --robust-threshold 10000

It is suggested that this is avoided by default. One has to be a bit careful
when doing this optimization to ensure some corners are not optimized at the
expense of others. This is discussed in :numref:`imagecorners`.

Validation
~~~~~~~~~~

It is strongly suggested to mapproject the image onto the obtained
camera to verify if it projects where expected::

     mapproject dem.tif img.tif img.tsai img_map.tif

The output ``img_map.tif`` can be overlaid onto the hillshaded DEM in
``stereo_gui``.

Use ``cam_test`` program (:numref:`cam_test`) for sanity checks.

The ``sfm_view`` program (:numref:`sfm_view`) can be used to visualize the
cameras in orbit.

One can invoke ``orbitviz`` (:numref:`orbitviz`)::

     orbitviz img.tif img.tsai -o orbit.kml

to create a KML file that can then be opened in Google Earth. It will display
the cameras above the planet. 

.. _cam_gen_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

-o, --output-camera-file <string (default: "")>
    Specify the output camera file with a .tsai or .json extension.

--camera-type <string (default: "pinhole")>
    Specify the output camera type. Options: ``pinhole``,  ``opticalbar``,
    ``linescan``. For linescan usage see :numref:`cam_gen_linescan`.

--lon-lat-values <string (default: "")>
    A (quoted) string listing numbers, separated by commas or spaces,
    having the longitude and latitude (alternating and in this
    order) of each image corner or some other list of pixels given
    by ``--pixel-values``. If the corners are used, they are traversed
    in the order (0, 0) (w, 0) (w, h), (0, h) where w and h are the
    image width and height.

--pixel-values <string (default: "")>
    A (quoted) string listing numbers, separated by commas or spaces,
    having the column and row (alternating and in this order) of
    each pixel in the raw image at which the longitude and latitude
    is known and given by ``--lon-lat-values``. By default this is
    empty, and will be populated by the image corners traversed as 
    mentioned at the earlier option.

--reference-dem <string (default: "")>
    Use this DEM to infer the heights above datum of the image corners.

--datum <string (default: "")>
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

--height-above-datum <float (default: 0.0)>
    Assume this height above datum in meters for the image corners
    unless read from the DEM.

--sample-file <string (default: "")>
    Read the camera intrinsics from this file. Required for optical bar cameras.
    See :numref:`kh9`, :numref:`file_format`, and :numref:`panoramic`.

--focal-length <float (default: 0.0)>
    The camera focal length.

--optical-center <float (default: NaN NaN)>
    The camera optical center. If not specified for pinhole cameras,
    it will be set to image center (half of image dimensions) times
    the pixel pitch. The optical bar camera always uses the image
    center.

--pixel-pitch <float (default: 0.0)>
    The camera pixel pitch.

--distortion <string (default: "")>
    Distortion model parameters. It is best to leave this blank and have the
    program determine them. By default, the OpenCV `radial-tangential lens
    distortion
    <https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html>`_ model is
    used. Then, can specify 5 numbers, in quotes, in the order k1,
    k2, p1, p2, k3. Also supported is the transverse model, which needs 20
    values. These are the coefficients of a pair of polynomials of degree 3 in x
    and y. Only applicable when creating CSM Frame cameras. The default is zero
    distortion. See also ``--distortion-type``.

--distortion-type <string (default: "radtan")>
    Set the distortion type. Options: ``radtan``, ``transverse``. Only
    applicable when creating CSM Frame cameras (:numref:`cam_gen_frame`).

--refine-camera
    After a rough initial camera is obtained, refine it using least squares.
    This does not support distortion. For CSM Frame cameras, a more powerful
    solver is available, see option ``--refine-intrinsics``.

--refine-intrinsics <string (default: "")>
    Refine the camera intrinsics together with the camera pose. Specify, in
    quotes or with comma as separator, one or more of: ``focal_length``,
    ``optical_center``, ``other_intrinsics`` (same as ``distiortion``).
    Also can set as ``all`` or ``none``. In the latter mode only the camera pose
    is optimized. Applicable only with option ``--input-camera`` and when
    creating a CSM Frame camera model (:numref:`cam_gen_frame`). 
        
--frame-index <string (default: "")>
    A file used to look up the longitude and latitude of image
    corners based on the image name, in the format provided by the
    SkySat video product.

--gcp-file <string (default: "")>
    If provided, save the image corner coordinates and heights in
    the GCP format to this file.

--gcp-std <double (default: 1.0)>
    The standard deviation for each GCP pixel, if saving a GCP file.
    A smaller value suggests a more reliable measurement, hence
    will be given more weight.

--input-camera <string (default: "")>
    Create the output pinhole camera approximating this camera.
    If with a ``_pinhole.json`` suffix, read it verbatim, with no
    refinements or taking into account other input options. Example
    in :numref:`skysat_stereo`.

--cam-height <float (default: 0.0)>
    If both this and ``--cam-weight`` are positive, enforce that the output
    camera is at this height above datum.
    
--cam-weight <float (default: 0.0)>
    If positive, try to enforce the option ``--cam-height`` with this weight (a
    bigger weight means try harder to enforce).

--cam-ctr-weight <float (default: 0.0)>
    If positive, try to enforce that during camera refinement the camera center
    stays close to the initial value (a bigger weight means try harder to
    enforce this, a value like 1000 is good enough).

-t, --session-type <string (default: "")>
    Select the input camera model type. Normally this is auto-detected,
    but may need to be specified if the input camera model is in
    XML format. See :numref:`parallel_stereo_options` for options.

--bundle-adjust-prefix <string (default: "")>
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
    Tell GDAL to not create BigTIFF files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
