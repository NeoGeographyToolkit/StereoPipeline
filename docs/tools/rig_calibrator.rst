.. _rig_calibrator:

rig_calibrator
--------------

The ``rig_calibrator`` program takes as input image and/or depth+image
datasets acquired with a rig of *N* cameras. It finds the relationship among
the rig sensors, the pose of each camera image, and refines the
intrinsics of each sensor. 

This tool was extensively tested with actual hardware and can model
many real-world issues encountered with a rig. Its output can be used
to create a fused surface mesh with seamless texture from each of its
sensors.
 
A solved example having all inputs, outputs, and commands, will be provided.

Capabilities
^^^^^^^^^^^^

- The cameras on the rig may be purely image cameras, or may have a depth
  component. In the latter case, the transform from a camera's depth to image
  coordinate system is modeled.
- No calibration target is assumed, so the image/depth data are acquired in situ.
- The solved-for camera poses and relationships among sensors can be registered 
  to real-world coordinates via control points.
- A preexisting mesh of the surface being imaged can be used as a constraint (rays
  corresponding to the same feature must intersect close to the mesh).
- All images acquired with one sensor are assumed to share intrinsics.
  The user may choose which intrinsics of which sensor are optimized
  or kept fixed, while the rig transforms and camera poses are optimized.
- The intrinsics of the sensors and each camera image pose can also be
  optimized without the rig assumption. In that case the transforms
  among the sensors on the rig are not modeled. 
- The sensors on the rig may acquire data simultaneously or not. In
  the latter case one sensor is expected to acquire data frequently
  enough to be used to bracket data from the other sensors in time
  using bilinear interpolation of the camera poses.
- A known time offset among the clocks of the various sensors on the 
  rig is modeled and can be optimized. (By default no offset is
  assumed.)  
- Several quality metrics are printed on output, and for
  each image with its optimized camera a textured mesh is created, for
  examination of any misalignments.
 
Input data conventions
^^^^^^^^^^^^^^^^^^^^^^

Each rig sensor should have a name, such as ``ref_cam``, ``alt_cam``,
etc.

Each image file must be stored according to the convention::

    <parent dir>/<sensor name>/<timestamp>.<extension>

For example, two images acquired at time 1004.6, can be named::

    my_images/ref_cam/10004.6.jpg
    my_images/alt_cam/10004.6.jpg

The exact value of the timestamps is not important, any integer
or double precision value would do, as long as all sensors on the rig
acquire images at the same time, and images have the same timestamp
only if taken at the same time. 

If each sensor acquires images independently, accurate
timestamps are important, and one of the sensors, named the ``reference``
sensor should acquire images frequently enough to help bracket the
other sensors in time using pose interpolation.

The images are expected to be 8 bit, with .jpg, .png, or .tif extension.

If some sensors also have depth data, the same convention is followed,
with the file extension being .pc. Example::

    my_images/alt_cam/10004.6.pc

All such depth cloud files will be loaded automatically alongside
images if present. The format of this file will be described later.  

Configuration file
^^^^^^^^^^^^^^^^^^

What is known about the rig should be specified in a plain text file,
with the following syntax::

  # Lines starting this way are comments
  ref_sensor_name: ref_cam

  # For each sensor on the rig, specify the following:
  sensor_name: <string>
  focal_length: <double> # units of pixel
  optical_center: <double, double> # units of pixel
  distortion_type: <string> # 'none', 'fisheye', or 'radtan'
  distortion_coeffs: <n doubles> # n = 0: none, 1: fisheye, 4/5: radtan
  image_size: <int, int>
  undistorted_image_size: <int, int> 
  ref_to_sensor_transform: <0 or 12 doubles>
  depth_to_image_transform: <0 or 12 doubles>
  ref_to_sensor_timestamp_offset: <double>

Here, ``ref_to_sensor_transform`` has the rotation (9 doubles, stored
row after row) and translation (3 doubles) transform from the
reference sensor to the sensor with given name, while
``depth_to_image_transform`` is the transform from the depth to image
coordinate systems of a given depth+image sensor. These can be set to
the identity transform (example below) if not known or not applicable.
The value ``ref_to_sensor_timestamp_offset``, measured in seconds, is
what should be added to the reference camera clock to get the time in
current sensor's clock. Set to 0 if the clocks are synchronized.

Educated guess can be provided for the quantities that are not known.
This tool can be used to optimize the focal length, optical center, an
distortion coefficients. The undistorted image size also need not be
known accurately.

A file in the same format will be written in the output directory,
with the name::

  <output dir>/rig_config.txt

This time the transforms among the rig sensors will be known,
while other values may change. 

Such a file can be read with the option ``--rig_config``.

Example::

  ref_sensor_name: nav_cam

  sensor_name: nav_cam
  focal_length: 621.04422
  optical_center: 580.56426999999996 495.51236
  distortion_type: fisheye
  distortion_coeffs: 1.0092038999999999
  image_size: 1280 960
  undistorted_image_size: 3000 1800
  ref_to_sensor_transform: 1 0 0 0 1 0 0 0 1 0 0 0
  depth_to_image_transform: 1 0 0 0 1 0 0 0 1 0 0 0
  ref_to_sensor_timestamp_offset: 0

Camera poses
^^^^^^^^^^^^

If estimated poses for each camera image exist, for example, obtained
from a previous run, those can be specified in a plain text file, with
each line in the following format::

 # <image name> <camera to world transform (rotation + translation)>
 my_images/ref_cam/10004.6.jpg <12 doubles>

If these are not known, the Theia structure-from-motion program (shipped
with this software) can be used to find the initial poses which this tool
will then optimize and/or register.

In either case a file having the output camera poses will be saved
by this tool, with the name::

  <output dir>/images.txt

It can be read by the program with the ``--image_list`` option.

A solved example
^^^^^^^^^^^^^^^^

An example using ``rig_calibrator`` on images acquired with the
Astrobee robot in the lab can be found at:

This robot has three cameras: ``nav_cam`` (wide field of view, using
the fisheye distortion model), ``sci_cam`` (narrow field of view,
using the radtan distortion model), and ``haz_cam`` (has depth
measurements, with one depth xyz value per pixel, narrow field of
view, using the radtan distortion model).

We assume the intrinsics of each sensor are resonably well-known (but
will be optimized later), and we do not know each camera's pose. The
first step is then determining these, for which we use the
``theia_sfm.py`` tool, as follows::

    theia_sfm.py --rig_config rig_input/rig_config.txt        \
      --images 'rig_input/nav_cam/*tif rig_input/haz_cam/*tif \
        rig_input/sci_cam/*tif'                               \
      --out_dir rig_theia

This tool will use the Theia flags file in ``share/theia_flags.txt``,
which can be copied to a new name, edited, and passed to this
progam via ``--theia_fags``.

It will write the solved camera poses to ``rig_theia/cameras.nvm``.

Best practices
^^^^^^^^^^^^^^

It is suggested to not optimize the intrinsics of each sensor until later
in the process, as otherwise there are too many variables to optimize
at the same time, hence to focus first on finding each camera's pose
and the transforms among the rig sensors, even if the results are imperfect. 

Optimizing the camera poses (without control points or a preexisting
mesh constraint) can change the scale of things. If it is desired
to keep those fixed and only optimize the transorms among the rig sensors,
use the option ``--camera_poses_to_float ""``.

The output directory of each tool invocation will write the rig
configuration so far, the poses of all images, and can also write the
images and depth clouds themselves. These can be used as inputs for a
subsequent invocation, if needed to fine-tune things.  For that, the
first invocation should use the options ``--out_dir`` and
``--save_images_and_clouds``, and the second one should use
``--image_list prev_dir/images.txt`` and ``--rig_config
prev_dir/rig_config.txt``.

How to use this tool
^^^^^^^^^^^^^^^^^^^^

The precise way the tool is used depends on how much is known
about the rig properties. 

Setting up the known rig configuration

One sensor is declared to be the reference sensor. The choice may be arbitrary 
if the sensors are very similar. Otherwise, it can be the one 
which is more likely to acquire images frequently enough or which 
has a larger field of view and/or resolution making it more likely for
feature matches from this sensor to the rest to succeed. (Feature matches
will be found among all images of all sensors sufficiently close in time, however.)

What is known about the rig should be in a plain text file.



Repeated invocation of the tool

For more complicated situations, when very little is known about the input
cameras, several iterations of this tool may be needed. For example,
one may leave the refinement of intrinsics of each camera for later,
once there is enough confidence that the orientations of the cameras
are reasonably correct.

Subsequent invocations of this tool can read the outputs of a previous
invocations, which save the rig configuration so far, the camera poses,
and interest point matches (as .nvm or .match files).

Determination of scale and registration

If the camera poses for the images are not known, the Theia package is called
to determine them. The obtained poses will be self-consistent, but to make these
be in a real-world coordinate system registration is necessary, which amounts
to applying a rotation, translation, and scale correction to the whole set up.

Before such registration, one already can determine the rig transforms, that is
the transforms among the sensors on the rig, up to a scale factor, however.

If some of the cameras have depth, that information will be automatically taken
into account (assuming ``--depth_tri_weight`` has a positive value), and the 
scale of the clouds will determine the scale of the rig and the scale of
the configuration of camera poses.

For registration, explain about --hugin-file rig6.pto --xyz_file rig6_xyz.txt
