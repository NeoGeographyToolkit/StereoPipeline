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
path on the surface with varying orientation (:numref:`sat_sim_custom_path`), or
the cameras can have a fixed orientation, without
(:numref:`sat_sim_roll_pitch_yaw`) and with
(:numref:`sat_sim_roll_pitch_yaw_ground`) ground constraints.

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

.. _sat_sim_nadir:

Example (generate nadir-pointing cameras)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::
  
    sat_sim --dem dem.tif --ortho ortho.tif              \
    --first 397.1 400.7 450000 --last 397.1 500.7 450000 \
    --num 5                                              \
    --focal-length 450000 --optical-center 500 500       \
    --image-size 1000 1000                               \
    -o run/run

See :numref:`sat_sim_roll_pitch_yaw` for how to apply a custom rotation
to the cameras.

The first and last cameras will be located as specified by ``--first`` and
``--last``.

In this example, the camera is 450,000 m above the ground and the
focal length is 450,000 pixels. If the magnitude of DEM heights is within
several hundred meters, this will result in the ground sample distance being
around 1 meter per pixel.

The produced image and camera names will be along the lines of::
    
    run/run-10000.tif
    run/run-10000.tsai

.. figure:: ../images/sfm_view_nadir_clip.png
   :name: sat_sim_illustration_nadir_clip
   :alt:  sat_sim_illustration_nadir_clip
   
   Illustration of ``sat_sim`` creating nadir-looking cameras.

.. _sat_sim_custom_path:

Example (follow custom ground path)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given two locations on the DEM, each specified by the column and row of DEM
pixel, to ensure that the center of the camera footprint travels along the straight
edge (in DEM pixel coordinates) between these, add to the tool options along the
lines of::

    --first-ground-pos 484.3 510.7 \
    --last-ground-pos 332.5 893.6    

This will result in the camera orientation changing gradually to keep the
desired view.

.. figure:: ../images/sfm_view.png
   :name: sat_sim_illustration
   :alt:  Illustration of ``sat_sim`` looking at a ground point.

   An example of several generated cameras looking at the same ground point. 
   Plotted with ``sfm_view`` (:numref:`sfm_view`).

.. _sat_sim_roll_pitch_yaw:

Fixed camera orientation
^^^^^^^^^^^^^^^^^^^^^^^^

When custom cameras are created (not read from disk), and unless the
``--first-ground-pos`` and ``--last-ground-pos`` options are specified, the
cameras will look straight down (nadir, perpendicular to along and across track
directions). 

If desired to have a custom orientation, use the ``--roll``, ``--pitch`` and
``--yaw`` options (measured in degrees, all three must be specified). When all
these are set to 0 (the default is ``NaN``) the default nadir-looking orientation is
recovered. 

As an example, if the pitch is 90 degrees and the other angles are zero, the
camera will look along the track rather than down. If a non-zero yaw is set, the
camera will rotate around the view axis.

Example invocation::

    sat_sim --dem dem.tif --ortho ortho.tif              \
    --first 397.1 400.7 450000 --last 397.1 500.7 450000 \
    --num 5                                              \
    --roll 0 --pitch 25 --yaw 0                          \
    --focal-length 450000 --optical-center 500 500       \
    --image-size 1000 1000                               \
    -o run/run

The rotations are applied to the camera body in the roll, pitch, and yaw order.
So, the combined rotation matrix is::

    R = yawRot * pitchRot * rollRot

(the application is from right to left). The camera-to-ECEF rotation is produced
by further multiplying this matrix on the left by the rotation from the satellite
body to ECEF.

It is important to note that the satellite and the camera use different coordinate
systems. The satellite orientation is with the *x*, *y* and *z* axes pointing along
satellite track, across track, and towards the planet, respectively.

For the camera, it is preferable for the rows of pixels to be parallel to the
across track direction, and for the columns to be parallel to the along track
direction. So, the camera *y* direction is along the track, the camera *x*
direction is the negative of the across-track direction, and *z* points towards
the ground as before.

.. _sat_sim_roll_pitch_yaw_ground:

Pose and ground constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Given an orbital trajectory, a path on the ground, and a desired fixed camera
orientation (roll, pitch, yaw), this tool can find the correct endpoints along
the satellite orbit, then use those to generate the cameras (positioned
between those endpoints), with the center of the camera ground footprint following 
the desired ground path. Example::

    sat_sim --dem dem.tif --ortho ortho.tif                 \
      --first 397.1 400.7 450000 --last 397.1 500.7 450000  \
      --first-ground-pos 397.1 400.7                        \
      --last-ground-pos 397.1 500.7                         \
      --roll 0 --pitch 25 --yaw 0                           \
      --num 5                                               \
      --focal-length 450000 --optical-center 500 500        \
      --image-size 1000 1000                                \
      -o run/run

Here, unlike in :numref:`sat_sim_nadir`, we will use ``--first`` and ``--last``
only to identify the orbit. The endpoints to use on it will be found
given that we have to satisfy the orientation constraints in ``--roll``,
``--pitch``, ``--yaw`` and the ground path constraints in ``--first-ground-pos``
and ``--last-ground-pos``. 

Unlike in :numref:`sat_sim_custom_path`, the camera orientations will not change.

It is not important to know very accurately the values of ``--first-ground-pos``
and ``--last-ground-pos``. The trajectory of the camera center ground footprint
will be computed, its endpoints closest to these two ground coordinates will be
found, which in turn will be used to find the orbital segment endpoints.

.. figure:: ../images/sfm_view_nadir_off_nadir.png
   :name: sat_sim_illustration_nadir_off_nadir
   :alt:  sat_sim_illustration_nadir_off_nadir
   
   Illustration of ``sat_sim`` creating two sets of cameras, with different 
   fixed orientations for each, with both sets looking at the same ground path.
   A separate invocation of ``sat_sim`` is needed for each set. 

.. _sat_sim_jitter_model:

Jitter modelling
^^^^^^^^^^^^^^^^

As a satellite moves in orbit, it vibrates ever so slightly. The effect of this
on the acquired images is called *jitter*, and it occurs for both linescan and
Pinhole cameras. See :numref:`jitter_solve` for how jitter is solved for when
the cameras are linescan. Here we will discuss modeling jitter for synthetic
Pinhole cameras.

We assume the jitter is a periodic perturbation of the roll, pitch, and yaw
angles. We will use the same period for each of these, but individual amplitudes.
For example, to model along-track jitter only, the amplitudes for the other 
angles can be set to zero.

The jitter frequency will be measured in Hz. For example, *f* = 45 Hz (45
oscillations per second). If the satellite velocity is *v* meters per second,
the jitter period in meters is :math:`T = v / f`.

Denote by :math:`A_i` the jitter amplitude, in degrees (i = 1, 2, 3, for roll,
pitch, and yaw). The jitter
perturbation is modeled as:

.. math::
    
        A_i \sin\left(d \frac{2 \pi}{T}\right)

Some care is needed to define the parameter *d*. We set it to be the distance
from the starting orbit point as specified by ``--first`` to the current camera
center (both in ECEF, along the curved orbit). This starting point is *before*
adjusting the orbital segment for roll, pitch, yaw, and ground constraints
(:numref:`sat_sim_roll_pitch_yaw_ground`). 

This way the jitter amplitude at the adjusted starting point (first camera
position) is uncorrelated between several sets of cameras along the same orbit
but with different values of roll, pitch yaw.

Specifying the jitter amplitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The jitter amplitude is usually very small and not easy to measure or interpret.
For this reason, we will define it indirectly, via the effect of jitter on the
*horizontal uncertainty* of the intersection of a ray emanating from the camera
center with the datum (see also :numref:`error_propagation`).

Consider a nadir-facing camera with the camera center at height *D* meters above
the datum. If the ray pointing straight down from that camera intersects the
datum at a certain point, and then that ray is perturbed by :math:`A_i` degrees, the
intersection point will move horizontally by

.. math::
    
      H_i = D \tan\left( \frac{\pi}{180} A_i \right)

This is the horizontal ground uncertainty of the intersection point. It is a
rather intuitive concept and many vendors publish it for their cameras. For
example, if the camera ground sample distance (pixel size on the ground) is 1
m/pixel, a horizontal uncertainty of 0.1 m or less is very good. If the camera
orientation is found using a star-tracker or some other estimations in orbit,
and no bundle adjustment (:numref:`bundle_adjust`) is performed, the horizontal
uncertainty will likely be much larger, for example on the order of 1-4 meters. 

In either case, this number is easy to understand, and the jitter amplitude
can be defined as the value of :math:`A_i` that produces the desired horizontal
uncertainty:

.. math::
    
      A_i = \frac{180}{\pi} \arctan\left( \frac{H_i}{D} \right)

The height above datum for the starting and ending points of the orbital segment
is the third value in ``--first`` and ``--last``. These values can, in
principle, be different, and then a linearly interpolated value will be used at
each camera position (and note that the orbital segment endpoints are adjusted,
per :numref:`sat_sim_roll_pitch_yaw_ground`).

As an illustration of using this functionality, consider the ``sat_sim``
invocation as in :numref:`sat_sim_roll_pitch_yaw_ground`, and add the options::

    --velocity 7500 --jitter-frequency 45 \
    --horizontal-uncertainty 2.0 2.0 2.0

See :numref:`sat_sim_options` for more information on
these options.

.. _sat_sim_options:

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

--roll <double>
    Camera roll angle, in degrees. See :numref:`sat_sim_roll_pitch_yaw` for
    details.

--pitch <double>
    Camera pitch angle, in degrees. See :numref:`sat_sim_roll_pitch_yaw` for
    details.

--yaw <double>
    Camera yaw angle, in degrees. See :numref:`sat_sim_roll_pitch_yaw` for  details.

--velocity <double>
    Satellite velocity, in meters per second. Used for modeling jitter. A value of
    around 8000 m/s is typical for a satellite like SkySat in Sun-synchronous orbit
    (90 minute period) at an altitude of about 450 km. For WorldView, the velocity
    is around 7500 m/s, with a higher altitude and longer period.

--horizontal-uncertainty <float, float, float>
    Camera horizontal uncertainty on the ground, in meters, at nadir
    orientation. Specify as three numbers, used for roll, pitch, and yaw. The
    angular uncertainty in the camera orientation for each of these angles is
    found as ``tan(angular_uncertainty) = horizontal_uncertainty /
    satellite_elevation_above_datum``, then converted to degrees. See 
    :numref:`sat_sim_jitter_model` for details.

--jitter-frequency <float>
    Jitter frequency, in Hz. Used for modeling jitter (satellite vibration). The
    jitter amplitude will be the angular horizontal uncertainty (see
    ``--horizontal-uncertainty``.

--no-images
    Create only cameras, and no images. Cannot be used with ``--camera-list``.
    
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
