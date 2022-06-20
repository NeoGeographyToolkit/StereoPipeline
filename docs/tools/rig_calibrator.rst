.. _rig_calibrator:

rig_calibrator
--------------

The ``rig_calibrator`` program takes as input image and/or depth+image
datasets acquired with a rig of n cameras, co-registers them to each
other and potentially to another reference, refines the intrinsics and
extrinsics of the cameras, and finds the transforms among the sensors
on the rig.

In more detail:: 

 - The cameras on the rig may be purely image cameras, or may have a depth
   component. In the latter case, the transform from a camera's depth to image
   coordinate system is modeled.
 - No calibration target is assumed, so the image/depth data are assumed acquired in-situ.
 - The solved-for camera poses and relationships among sensors can have a transform
   applied to them to map them to a known coordinate system.
 - A preexisting mesh of the area being imaged can be used as a constraint (rays
   corresponding to the same feature must intersect close to the mesh).
 - All images aqquired with one sensor are assumed to share intrinsics.
   The user may pick and choose which intrinsics of which sensor are optimized
   or kept fixed, while the poses of the cameras are optimized.
 - The camera poses and intrinsics can also be optimized without the rig
   assumption. In that case the transforms among the sensors on the rig
   are not modeled. 
 - The sensors on the rig may acquire data simultaneously or not. In
   the latter case one sensor is expected to acquire data frequently
   enough to be used to bracket data from the other sensors in time
   using bilinear interpolation of the camera poses.
 - A known time offset among the clocks of the various sensors on the 
   rig is modeled and can be optimized. (By deault no offset is assumed.)
 
The produced camera poses and intrinsics can be used to create
seamless textured meshes.

Setting up the known rig configuration

One sensor is declared to be the reference sensor. The choice may be arbitrary 
if the sensors are very similar. Otherwise, it can be the one 
which is more likely to aquire images frequently enough or which 
has a larger field of view and/or resolution making it more likely for
feature matches from this sensor to the rest to succeed. (Feature matches
will be found among all images of all sensors sufficiently close in time, however.)

What is known about the rig should be in a plain text file.

The syntax of the configuration file is the following.

# Lines starting this way are comments.
# Name of the 'reference' sensor. 
ref_sensor_name: ref_cam

# For each sensor on the rig, specify the follwing:
sensor_name: <string>
focal_length: <double>
optical_center: <double, double>
distortion_type: <string> (must be 'none', 'fisheye', or 'radtan')
distortion_coeffs: <n doubles> (n = 0: none, 1: fisheye, 4/5: radtan)
image_size: <int, int>
undistorted_image_size: <int, int> 
ref_to_sensor_transform: <12 doubles> (leave empty if not known)
depth_to_image_transform: <12 doubles> (leave empty if not known)
ref_to_sensor_timestamp_offset: <double>

The undisrtored image size may be known just approxmately.

Here, ``ref_to_sensor_transform`` refers to transform from the
reference sensor and sensor with given id, and
``depth_to_image_transform`` is the transform from the depth to image
coordinate systems of a given depth+image sensor. It can be left empty
if not known or not applicable (when present it may be the identity).

Each transform is given by 12 numbers, representing an affine
transform or particular cases of it, such as rotation + translation,
rotation + translation + scale. The first 9 numbers for the 3x3
matrix, and last 3 for the translation.


Each image file must have the following convention::

    <parent dir>/<sensor name>/<timestamp>.<extension>

For example, two images acquired with the rig sensors ``ref_cam`` and
``alt_cam`` at time 1004.6, must have the names::

    my_images/ref_cam/10004.6.jpg
    my_images/alt_cam/10004.6.jpg

That is to say, all images produced with a given sensor, must be in
that sensor's subdirectory. All images acquired with the rig at the
same time must have the same timestamp. All images must have the same
extension.

If a certain image also has a depth component, it should follow the
same convention, with the extension being .pc. Example::

    my_images/alt_cam/10004.6.pc

All such depth cloud files will be loaded automatically alongside
images if present.

Repeated invocation of the tool

For more complicated situations, when very little is known about the input
cameras, several iterations of this tool may be needed. For example,
one may leave the refinement of intrinsics of each camera for later,
once there is enough confidence that the orientations of the cameras
are reasonably correct.

Subsequent invocations of this tool can read the outputs of a previsous
invocations, which save the rig configuration so far, the camera poses,
and interest point matches (as .nvm or .match files).

Determination of scale and registration

If the camera poses for the images are not known, the package TheiaSFM is called
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
