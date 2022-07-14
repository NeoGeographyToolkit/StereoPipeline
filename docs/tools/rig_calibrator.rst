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
 
See :numref:`rig_calibrator_example` for a solved example having all
inputs, outputs, and commands.

Capabilities
^^^^^^^^^^^^

- The cameras on the rig may be purely image cameras, or may have a depth
  component. In the latter case, the transform from a camera's depth to image
  coordinate system is modeled.
- No calibration target is assumed, so the image/depth data are acquired in situ.
- The solved-for camera poses and relationships among sensors can be registered 
  to real-world coordinates via user-selected control points.
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
- A preexisting mesh of the surface being imaged can be used as a
  constraint (rays corresponding to the same feature must intersect
  close to the mesh). Otherwise one can constrain the triangulated
  points to not move too far from their original values.
- Several quality metrics are printed on output, and for each image
  with its optimized camera a textured mesh is created, for
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
sensor, should acquire images frequently enough to help bracket the
other sensors in time using bilinear pose interpolation.

The images are expected to be 8 bit, with .jpg, .png, or .tif extension.

If some sensors also have depth data, the same convention is followed,
with the file extension being .pc. Example::

    my_images/alt_cam/10004.6.pc

All such depth cloud files will be loaded automatically alongside
images if present. See :numref:`point_cloud_format` for the file
format.

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
  distorted_crop_size: <int, int> 
  undistorted_image_size: <int, int> 
  ref_to_sensor_transform: <12 doubles>
  depth_to_image_transform: <12 doubles>
  ref_to_sensor_timestamp_offset: <double>

Here, ``ref_to_sensor_transform`` has the rotation (9 doubles, stored
row after row) and translation (3 doubles) transform from the
reference sensor to the sensor with given name, while
``depth_to_image_transform`` is the transform from the depth to image
coordinate systems of a given depth+image sensor. These must be set to
the identity transform (example below) if not known or not applicable.
The value ``ref_to_sensor_timestamp_offset``, measured in seconds, is
what should be added to the reference camera clock to get the time in
current sensor's clock. Set to 0 if the clocks are synchronized.

The ``image_size`` field has the image dimensions (width and height).
The ``distorted_crop_size`` has the dimensions of the region whose
center is also the image center in which the given distortion model is
valid.  Normally it should be the whole image. The
``undistorted_image_size`` has a generous overestimate of the image
dimensions after undistortion.

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

Example (only one of the ``N`` sensors is shown)::

  ref_sensor_name: nav_cam

  sensor_name: nav_cam
  focal_length: 621.04422
  optical_center: 580.56426999999996 495.51236
  distortion_type: fisheye
  distortion_coeffs: 1.0092038999999999
  image_size: 1280 960
  distorted_crop_size: 1280 960
  undistorted_image_size: 1500 1500
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

In either case a file in this format having the output camera
poses will be saved by this tool, with the name::

  <output dir>/images.txt

It can be read by the program with the ``--camera_poses`` option.

.. _rig_calibrator_example:

A solved example
^^^^^^^^^^^^^^^^

An example using ``rig_calibrator`` on images acquired in a lab with
cameras mounted on the Astrobee robot
(https://github.com/nasa/astrobee) can be found at:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/rig_calibrator

This robot has three cameras: ``nav_cam`` (wide field of view, using
the fisheye distortion model), ``sci_cam`` (narrow field of view,
using the radtan distortion model), and ``haz_cam`` (has depth
measurements, with one depth xyz value per pixel, narrow field of
view, using the radtan distortion model).

We assume the intrinsics of each sensor are reasonably well-known (but
will be optimized later), and we do not know each camera's pose. The
first step is then determining these, for which we use the
``theia_sfm`` script, as follows::

    theia_sfm --rig_config rig_input/rig_config.txt \
      --images 'rig_input/nav_cam/*tif
                rig_input/haz_cam/*tif 
                rig_input/sci_cam/*tif'                \
      --out_dir rig_theia

It will write the solved camera poses to ``rig_theia/cameras.nvm``.

This tool will use the Theia flags file from ``share/theia_flags.txt``
in the software distribution, which can be copied to a new name,
edited, and passed to this program via ``--theia_fags``.

Next, we run ``rig_calibrator``::

    rig_calibrator                                \
      --rig_config rig_input/rig_config.txt       \
      --nvm rig_theia/cameras.nvm                 \
      --camera_poses_to_float "nav_cam"           \
      --rig_transforms_to_float "sci_cam haz_cam" \
      --intrinsics_to_float ""                    \
      --depth_to_image_transforms_to_float ""     \
      --affine_depth_to_image --bracket_len 3.0   \
      --depth_tri_weight 1000                     \
      --num_iterations 50                         \
      --calibrator_num_passes 2                   \
      --num_overlaps 10                           \
      --registration                              \
      --hugin_file control_points.pto             \
      --xyz_file xyz.txt                          \
      --export_to_voxblox                         \
      --out_dir rig_out

The previously found camera poses are read in. They are registered to
world coordinates. For that, the four corners of a square with known
dimensions visible in a couple of images were picked at control points
in ``Hugin`` (https://hugin.sourceforge.io/) and saved to
``control_points.pto``, and the corresponding measurements of their
coordinates were saved in ``xyz.txt``. See
:numref:`rig_calibrator_registration` for more details.

The ``nav_cam`` camera is chosen to be the reference sensor in the rig
configuration. Its poses are allowed to float, that is, to be
optimized (``--camera_poses_to_float``), and the rig transforms from
this one to the other ones are floated as well
(``--rig_transforms_to_float``). The intrinsics are not floated for now.

The value of ``--depth_tri_weight`` controls how close the
triangulated points should be to the depth measurements (after
adjusting for them being in different coordinate systems). In this
particular case, a positive value here would have been enough to find
the true scale (but not origin and orientation) of the camera
configuration, as it would be inferred from the depth clouds, even if
registration points were not present.

See :numref:`rig_calibrator_command_line` for the full list of options.

The obtained point clouds can be fused into a mesh using ``voxblox_mesh`` 
(:numref:`voxblox_mesh`), using the command::
    
    voxblox_mesh --index rig_out/voxblox/haz_cam/index.txt \
      --output_mesh rig_out/fused_mesh.ply                 \
      --min_ray_length 0.1 --max_ray_length 2.0            \
      --voxel_size 0.005

Here, the output mesh is ``fused_mesh.ply``, points no further than 2
meters from each camera center are used, and the mesh is obtained
after binning the points into voxels of 1 cm in size. See that
project's documentation for more details.

Next, the produced cameras and rig configuration (saved in ``rig_out``) are 
reoptimized, this time the intrinsics are allowed to float, and this mesh is
employed as a constraint. The optimized cameras are used to project the images onto
the mesh, obtaining one ``.obj`` textured mesh file per image::

    rig_calibrator                                       \
      --rig_config rig_out/rig_config.txt                \
      --camera_poses rig_out/cameras.txt                 \
      --mesh rig_out/fused_mesh.ply                      \
      --camera_poses_to_float "nav_cam"                  \
      --rig_transforms_to_float "sci_cam haz_cam"        \
      --intrinsics_to_float                              \
        "nav_cam:focal_length,optical_center,distortion 
         haz_cam:focal_length,optical_center:distortion
         sci_cam:focal_length,optical_center,distortion" \
      --depth_to_image_transforms_to_float ""            \
      --affine_depth_to_image --bracket_len 3.0          \
      --depth_tri_weight 1000                            \
      --depth_mesh_weight 10                             \
      --mesh_tri_weight 10                               \
      --tri_weight 0.1                                   \
      --num_iterations 50                                \
      --calibrator_num_passes 1                          \
      --num_overlaps 10                                  \
      --out_dir rig_out_mesh                             \
      --out_texture_dir rig_out_texture

If in doubt, the value of the three weights above can be lowered.
Especially, a high value of ``--mesh_tri_weight`` can prevent
convergence.

The obtained textured meshes can be inspected for disagreements, by
loading them in MeshLab, as::

    meshlab rig_out_texture/*sci_cam.obj

See :numref:`texrecon` for how to produce a merged textured mesh
given these images and camera poses.

Best practices
^^^^^^^^^^^^^^

It is suggested to not optimize the intrinsics of each sensor until later
in the process, as otherwise there are too many variables to optimize
at the same time, hence to focus first on finding each camera's pose
and the transforms among the rig sensors, even if the results are imperfect. 

Optimizing the camera poses (without control points or a preexisting
mesh constraint) can change the scale of things. If it is desired
to keep those fixed and only optimize the transforms among the rig sensors,
ensure that the value of ``--camera_poses_to_float`` is kept empty.

The output directory of each tool invocation will write the rig
configuration so far, the camera poses of all images, and can also
write the match files. These can be used as inputs for a subsequent
invocation, if needed to fine-tune things.

If the first invocation used the option ``--out_dir run_dir1`` the
second one should set the options ``--camera_poses
run_dir1/cameras.txt`` and ``--rig_config prev_dir/rig_config.txt``.

.. _rig_calibrator_registration:

Determination of scale and registration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To transform the system of cameras to world coordinates, it is
necessary to know the Euclidean coordinates of at least three control
points in the scene, and then to pick the pixel of coordinates of each
of these points in at least two images.

To find the pixel coordinates, open a subset of the reference
camera images in Hugin, such as::

    hugin <image dir>/*.jpg

It will ask to enter a value for the FoV (field of view). That value
is not important since we won't use it. One can input 10 degrees,
for example. 

Go to the "Expert" interface, choose a couple of distinct images, and
click on a desired control point in both images.  Make sure the
left and right image are not the same or highly similar, as that may
result in poor triangulation and registration. Then repeat this
process for all control points.

Save the Hugin project to disk. Create a separate text file which
contains the world coordinates of the control points picked earlier,
with each line in the "x y z" format, and in the same order as the
Hugin project file.  That is to say, if a control point was picked in
several image pairs in Hugin, it must show up also the same number of
times in the text file, in the same order. In the xyz text file all
lines starting with the pound sign (#) are ignored, as well as all
entries on any line beyond three numerical values.

The dataset from :numref:`rig_calibrator_example` has examples
of files used for registration, and shows how to pass these to the tool.

After registration is done, it will print each transformed coordinate
point from the map and its corresponding measured point, as well as the 
error among the two. That will look as follows::

    transformed computed xyz -- measured xyz -- error norm (meters)
    -0.0149 -0.0539  0.0120 --  0.0000  0.0000  0.0000 --  0.0472 img1.jpg img2.jpg
     1.8587  0.9533  0.1531 --  1.8710  0.9330  0.1620 --  0.0254 img3.jpg img4.jpg

Each error norm (last value), is the distance between a measured 3D
point and its computed value based on the registered cameras. If
some of them are too large, may be the measurements have some error,
or the camera poses or intrinsics are not accurate enough.

Note that the registration happens before the optimization, and the
latter can move the cameras around somewhat. To avoid that, or to do
one more registration pass, one can rerun ``rig_calibrator``
with control points as before, previous results (hence adjust
``--rig_config`` and ``--camera_poses``), and zero iterations.

If the images cover a large area, it is suggested to use registration
points distributed over that area. Registration may not always produce
perfect results since a structure-from-motion solution may drift over
large distances.

The software does not force the camera poses to move individually to
fit better the control points. Hence, the cameras are always kept
self-consistent, then the camera configuration has a single
registration transform applied to it to fit the control points.
Hence, the only approach to make the individual cameras conform more
faithfully to what is considered accurate geometry is to use the mesh
constraint.

Quality metrics
^^^^^^^^^^^^^^^

The rig calibrator will print out some statistics showing the residual errors
before and after each optimization pass (before outlier removal at the
end of the pass), as follows::
    
    The 25, 50, 75, and 100th percentile residual stats after opt
    depth_mesh_x_m: 0.0018037 0.0040546 0.011257 0.17554 (742 residuals)
    depth_mesh_y_m: 0.0044289 0.010466 0.025742 0.29996 (742 residuals)
    depth_mesh_z_m: 0.0016272 0.0040004 0.0080849 0.067716 (742 residuals)
    depth_tri_x_m: 0.0012726 0.0054119 0.013084 1.6865 (742 residuals)
    depth_tri_y_m: 0.0010357 0.0043689 0.022755 3.8577 (742 residuals)
    depth_tri_z_m: 0.00063148 0.0023309 0.0072923 0.80546 (742 residuals)
    haz_cam_pix_x: 0.44218 0.99311 2.1193 38.905 (819 residuals)
    haz_cam_pix_y: 0.2147 0.49129 1.3759 95.075 (819 residuals)
    mesh_tri_x_m: 0.0002686 0.00072069 0.014236 6.3835 (5656 residuals)
    mesh_tri_y_m: 9.631e-05 0.00032232 0.057742 9.7644 (5656 residuals)
    mesh_tri_z_m: 0.00011342 0.00031634 0.010118 1.0238 (5656 residuals)
    nav_cam_pix_x: 0.098472 0.28129 0.6482 155.99 (47561 residuals)
    nav_cam_pix_y: 0.11931 0.27414 0.55118 412.36 (47561 residuals)
    sci_cam_pix_x: 0.33381 0.70169 1.4287 25.294 (2412 residuals)
    sci_cam_pix_y: 0.24164 0.52997 0.90982 18.333 (2412 residuals)

These can be helpful in figuring out if the calibration result is
good.  The errors whose name ends in "_m" are in meters and measure
the absolute differences between the depth clouds and mesh
(depth_mesh), between depth clouds and triangulated points
(depth_tri), and between mesh points and triangulated points
(mesh_tri), in x, y, and z, respectively. The ``mesh`` residuals will
be printed only if a mesh is passed on input and if the mesh-related
weights are positive. Some outliers are unavoidable, hence some of
these numbers can be big even if the calibration overall does well
(the robust threshold set via ``--robust_threshold`` does not allow
outliers to dominate).

Source of errors can be, as before, inaccurate intrinsics, camera
poses, or insufficiently good modeling of the cameras. 

When each rig sensor has its own clock, or acquires images at is own
rate, the discrepancy among the clocks (if the timestamp offsets are
not set correctly) or insufficiently tight bracketing (cameras moving
too much between acquisitions meant to serve as brackets) may be source
of errors as well. In this case one can also try the tool with
the ``--no_rig`` option, when the cameras are decoupled and see if this
makes a difference.

.. _point_cloud_format:

Point cloud file format
^^^^^^^^^^^^^^^^^^^^^^^

The depth point clouds (for the depth component of cameras, if
applicable) are saved to disk in binary. The first three entries are
of type int32, having the number of rows, columns and channels (whose
value is 3). Then, one iterates over rows, for each row iterates over
columns, and three float 32 values corresponding to x, y, z
coordinates are read or written. If all three values are zero, this
point is considered to be invalid, but has to be read or written
to ensure there exists one depth point for each corresponding image pixel.

.. _rig_calibrator_command_line:

Command-line options for rig_calibrator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``--affine_depth_to_image`` Assume that the depth-to-image transform for each
  depth + image camera is an arbitrary affine transform rather than a
  rotation times a scale. Type: bool. Default: false.
``--bracket_len`` Lookup non-reference cam images only between consecutive ref
  cam images whose distance in time is no more than this (in seconds),
  after adjusting for the timestamp offset between these cameras. It is
  assumed the rig moves slowly and uniformly during this time. A large
  value here will make the calibrator compute a poor solution but a small
  value may prevent enough images being bracketed. Type: double. Default: 0.6.
``--calibrator_num_passes`` How many passes of optimization to do. Outliers
  will be removed after every pass. Each pass will start with the
  previously optimized solution as an initial guess. Mesh intersections (if
  applicable) and ray triangulation will be recomputed before each pass.)
  Type: int32. Default: 2.
``--camera_poses_to_float`` Specify the cameras of which sensor types can have
  their poses floated. Note that allowing the cameras for all sensors types
  to float can invalidate the registration and scale (while making the
  overall configuration more internally consistent). Hence, one may need to
  use an external mesh as a constraint, or otherwise subsequent
  registration may be needed. Example: 'cam1 cam3'. Type: string. Default: "".
``--tri_weight`` The weight to give to the constraint that optimized
  triangulated points stay close to original triangulated points. A
  positive value will help ensure the cameras do not move too far, but a
  large value may prevent convergence.
``--depth_mesh_weight`` A larger value will give more weight to the constraint
  that the depth clouds stay close to the mesh. Not suggested by default.)
  Type: double. Default: 0.
``--depth_to_image_transforms_to_float`` Specify for which sensors to float the
  depth-to-image transform (if depth data exists). Example: 'cam1 cam3'.)
  Type: string. Default: "".
``--depth_tri_weight`` The weight to give to the constraint that depth
  measurements agree with triangulated points. Use a bigger number as depth
  errors are usually on the order of 0.01 meters while reprojection errors
  are on the order of 1 pixel. Type: double. Default: 1000.
``--tri_weight`` The weight to give to the constraint that optimized triangulated
  points stay close to original triangulated points. A positive value will
  help ensure the cameras do not move too far, but a large value may prevent
  convergence. Type: double. Default: 0.
``--export_to_voxblox`` Save the depth clouds and optimized transforms needed
  to create a mesh with voxblox (if depth clouds exist). Type: bool. Default: false.
``--float_scale`` If to optimize the scale of the clouds, part of
  depth-to-image transform. If kept fixed, the configuration of cameras
  should adjust to respect the given scale. This parameter should not be
  used with ``--affine_depth_to_image`` when the transform is affine, rather
  than rigid and a scale. Type: bool. Default: false.
``--float_timestamp_offsets`` If to optimize the timestamp offsets among the
  cameras. This is experimental. Type: bool. Default: false.
``--hugin_file`` The path to the hugin .pto file used for registration.)
  Type: string. Default: "".
``--camera_poses`` Read the images and world-to-camera poses from this list.
  The same format is used as for when this tool saves the updated poses
  in the output directory. Type: string. Default: "".
``--initial_max_reprojection_error`` If filtering outliers, remove interest
  points for which the reprojection error, in pixels, is larger than this.
  This filtering happens when matches are created, before cameras are
  optimized, and a big value should be used if the initial cameras are not
  trusted. Type: double. Default: 300.
``--intrinsics_to_float`` Specify which intrinsics to float for each sensor.
  Example: 'cam1:focal_length,optical_center,distortion
  cam2:focal_length'. Type: string. Default: "".
``--max_image_to_depth_timestamp_diff`` Use a depth cloud only if it is within
  this distance in time from the nearest image with the same camera.
  Measured in seconds. Type: double. Default: 0.2.
``--max_ray_dist`` The maximum search distance from a starting point along a
  ray when intersecting the ray with a mesh, in meters (if applicable).)
  Type: double. Default: 100.
``--max_reprojection_error`` If filtering outliers, remove interest points for
  which the reprojection error, in pixels, is larger than this. This
  filtering happens after each optimization pass finishes, unless disabled.
  It is better to not filter too aggressively unless confident of the
  solution. Type: double. Default: 25.
``--mesh`` Use this geometry mapper mesh from a previous geometry mapper run to
  help constrain the calibration (e.g., use fused_mesh.ply). Type: string. Default: "".
``--mesh_tri_weight`` A larger value will give more weight to the constraint
  that triangulated points stay close to a preexisting mesh. Not suggested
  by default. Type: double. Default: 0.
``--min_ray_dist`` The minimum search distance from a starting point along a
  ray when intersecting the ray with a mesh, in meters (if applicable).
  Type: double. Default: 0.
``--no_rig`` Do not assumes the cameras are on a rig. Hence, the pose of any
  camera of any sensor type may vary on its own and not being tied to other
  sensor types. See also ``--camera_poses_to_float``. Type: bool. Default: false.
``--num_exclude_boundary_pixels`` Flag as outliers pixels closer than this to
  image boundary, and ignore that boundary region when texturing using the
  optimized cameras with the ``--out_texture_dir`` option. Provide one number
  per sensor, in the same order as in the rig config file. Example: 
  '100 0 0'. For fisheye lenses, this improves a lot the quality of results.
  Type: string. Default: "".
``--num_iterations`` How many solver iterations to perform in calibration.)
  Type: int32. Default: 20.
``--num_match_threads`` How many threads to use in feature detection/matching.
  A large number can use a lot of memory. Type: int32. Default: 8.
``--num_opt_threads`` How many threads to use in the optimization. Type: int32.
  Default: 16.
``--num_overlaps`` How many images (of all camera types) close and forward in
  time to match to given image. Type: int32. Default: 10.
``--nvm`` Read images and camera poses from this nvm file, as exported by
  Theia. Type: string. Default: "".
``--out_dir`` Save in this directory the camera intrinsics and extrinsics. See
  also ``--save-matches``, ``--verbose``. Type: string. Default: "".
``--out_texture_dir`` If non-empty and if an input mesh was provided, project
  the camera images using the optimized poses onto the mesh and write the
  obtained .obj files in the given directory. Type: string. Default: "".
``--parameter_tolerance`` Stop when the optimization variables change by less
  than this. Type: double. Default: 1e-12.
``--refiner_min_angle`` If filtering outliers, remove triangulated points for
  which all rays converging to it make an angle (in degrees) less than
  this. Note that some cameras in the rig may be very close to each other
  relative to the triangulated points, so care is needed here.)
  Type: double. Default: 0.5.
``--registration`` If true, and registration control points for the sparse map
  exist and are specified by ``--hugin_file`` and ``--xyz_file``, register all
  camera poses and the rig transforms before starting the optimization. For
  now, the depth-to-image transforms do not change as result of this, which
  may be a problem. To apply the registration only, use zero iterations.)
  Type: bool. Default: false.
``--rig_config`` Read the rig configuration from file. Type: string. 
  Default: "".
``--rig_transforms_to_float`` Specify the names of sensors whose transforms to
  float, relative to the ref sensor. Use quotes around this string if it
  has spaces. Also can use comma as separator. Example: 'cam1 cam2'.)
  Type: string. Default: "".
``--robust_threshold`` Residual pixel errors and 3D point residuals (the latter
  multiplied by corresponding weight) much larger than this will be
  exponentially attenuated to affect less the cost function.
  Type: double. Default: 3.
``--save_matches`` Save the interest point matches. Stereo Pipeline's viewer
  can be used for visualizing these. Type: bool. Default: false.
``--timestamp_offsets_max_change`` If floating the timestamp offsets, do not
  let them change by more than this (measured in seconds). Existing image
  bracketing acts as an additional constraint. Type: double. Default: 1.
``--use_initial_rig_transforms`` Use the transforms among the sensors of the
  rig specified via ``--rig_config.`` Otherwise derive it from the poses of
  individual cameras. Type: bool. Default: false.
``--xyz_file`` The path to the xyz file used for registration. Type:
  string. Default: "".
``--verbose`` Print a lot of verbose information about how matching goes.)
  Type: bool. Default: false.
