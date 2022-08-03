.. _multi_stereo:

multi_stereo
------------

The ``multi_stereo`` program takes as input a set of images and
cameras, runs pairwise stereo between each image/camera and the next
one in the list, filters the produced points clouds, fuses them,
and creates a mesh.

For the moment this program is very tied to ``rig_calibrator``
(:numref:`rig_calibrator`).  It will become more generic and versatile
with time. In particular, logic is planned for automatically selecting
stereo pairs and for distributing and load-balancing all resulting
processing jobs over multiple machines.

Example
^^^^^^^

Here we will create a mesh of a small portion of the International
Space Station (ISS), based on images acquired with the `Astrobee
<https://github.com/nasa/astrobee>`_ robot (later this example will be
expanded to a full module). This is a good way of testing the limits
of ASP's stereo, because:

 - The camera has a wide field-of-view fisheye lens, whose distortion is strong 
   and hard to model accurately, which then may result in registration errors.

 - The range of camera-to-object distances is much larger than in satellite stereo,
   and there is a wide range of orientations of the encountered surfaces.

 - The camera can move towards an object it looks at, such a wall,
   while having other walls on the side. This makes it tricky to correctly align
   the images. The ideal scenario in stereo is cameras being
   side-by-side and the imaged surface being reasonably far.

 - The ISS is "messy", having cables and laptops sticking out of
   walls, surfaces with weak texture, and areas of low illumination.

 - The images are 8-bit and compressed as JPEG, which may result in artifacts, 
   unlike the lossless high-dynamic range images acquired with satellites.

This required some careful choices of parameters, and a new tool named
``pc_filter`` for filtering blunders according to many geometric
criteria. This will all be explained below.

The 7-image dataset used below, the full recipe, and output mesh, are
all available at:

  https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/multi_stereo

Creation of camera models
~~~~~~~~~~~~~~~~~~~~~~~~~

We broadly follow the tools and approach from section :numref:`rig_calibrator`,
but with a rig consisting of just one camera.

Determination of initial camera poses::

    theia_sfm --rig_config camera_config.txt \
      --images 'images/nav_cam/*jpg'         \
      --out_dir theia_out

Note that the images are stored in the ``nav_cam`` subdirectory, and
each image name consists of a number and an image extension, following
the conventions used by ``rig_calibrator``, even though here we have
just a single sensor acquiring all images.

Refinement of camera poses and registration to world coordinates (this
requires first manually picking some features with known 3D positions
in the images, per :numref:`rig_calibrator_registration`)::

    rig_calibrator                                \
      --rig_config camera_config.txt              \
      --nvm theia_out/cameras.nvm                 \
      --camera_poses_to_float "nav_cam"           \
      --intrinsics_to_float ""                    \
      --num_iterations 50                         \
      --calibrator_num_passes 2                   \
      --num_overlaps 10                           \
      --registration                              \
      --hugin_file control_points.pto             \
      --xyz_file xyz.txt                          \
      --out_dir rig_out
    
Running stereo and mesh creation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As mentioned earlier, the geometry of the scene being imaged requires
some careful choices of parameters for stereo.  Then, this tool calls
several other tools under the hood, so options for those should be set
as well. Here's a recipe which works reasonably well::

    maxDistanceFromCamera=3.5

    stereo_opts="
      --stereo-algorithm asp_mgm
      --alignment-method affineepipolar
      --ip-per-image 10000
      --min-triangulation-angle 0.5 
      --global-alignment-threshold 5   
      --session nadirpinhole 
      --no-datum
      --corr-seed-mode 0
      --corr-search -40 -10 40 10
      --corr-tile-size 5000
      --ip-inlier-factor 0.4"
      
    pc_filter_opts="
      --max-camera-ray-to-surface-normal-angle 75 
      --max-valid-triangulation-error 0.0025   
      --max-distance-from-camera $maxDistanceFromCamera
      --blending-dist 50 --blending-power 1"

    mesh_gen_opts="
      --min_ray_length 0.1
      --max_ray_length $maxDistanceFromCamera
      --voxel_size 0.005"

    multi_stereo --rig_config rig_out/rig_config.txt \
      --camera_poses rig_out/cameras.txt             \
      --undistorted_crop_win '1100 700'              \
      --rig_sensor nav_cam                           \
      --first_step stereo                            \
      --last_step  mesh_gen                          \
      --stereo_options "$stereo_opts"                \
      --pc_filter_options "$pc_filter_opts"          \
      --mesh_gen_options "$mesh_gen_opts"            \
      --out_dir stereo_out

The surface resolution of the cameras is on the order of 1 mm (0.001
meters), the camera is about 1-3 meters from the surface, hence a good
value for the triangulation error was about 0.0025 meters, and the
points in the cloud were binned (before meshing) into voxels of size
0.005 meters. Later some of these choices will be automated, or
scale-independent parameters will be provided.

In future versions of this tool, undistortion of input images may be
optional.

There are three steps happening above, namely:

* stereo: Runs ``parallel_stereo`` (:numref:`parallel_stereo`) and
  writes a point cloud in .tif format for each image/camera
  in the list and the next one. This is the most time-consuming step.

* pc_filter: For each point cloud runs ``pc_filter`` (:numref:`pc_filter`)
  and writes filtered point clouds in .tif and .pcd formats, and a
  textured mesh for that run in .obj format. The .pcd file is in left
  camera's coordinates. The .obj file is for individual stereo run
  inspection purposes.

* mesh_gen: Use ``voxblox_mesh`` (:numref:`voxblox_mesh`) to fuse the
  filtered point clouds in .pcd format and create a mesh in .ply
  format.

See ``--first_step`` and ``--last_step`` in
:numref:`multi_stereo_command_line` for how to choose which processing
steps to run.

Creating a textured mesh
~~~~~~~~~~~~~~~~~~~~~~~~

The obtained mesh can be textured with the original images using the
``texrecon`` tool (:numref:`texrecon`) as::

    texrecon --rig_config rig_out/rig_config.txt \
      --camera_poses rig_out/cameras.txt         \
      --mesh stereo_out/nav_cam/fused_mesh.ply   \
      --rig_sensor nav_cam                       \
      --undistorted_crop_win '1100 700'          \
      --out_dir stereo_out

This produces ``stereo_out/nav_cam/texture.obj``.

Handling issues
^^^^^^^^^^^^^^^

If the produced mesh is noisy, it is suggested to inspect individual
.obj files produced by each stereo pair, the triangulation error of
each filtered point cloud (fourth band, extractable with
``gdal_translate -b 4``), and the blending weight files saved by
``pc_filter``.

One may need to decrease the value of
``--max-valid-triangulation-error``, use less of the boundary image
region (``--undistorted_crop_win``) or redo the bundle adjustment with
``rig_calibrator``.

.. _multi_stereo_command_line:

Command-line options for multi_stereo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

--rig_config <string (default: "")>
    Rig configuration file.
--rig_sensor <string (default: "")>
    Which rig sensor images to use. Must be among the
    sensors specified via ``--rig_config``.
--camera_poses <string (default: "")>
    Read images and camera poses for this sensor from this 
    list.
--out_dir <string (default: "")>
    The directory where to write the stereo output, textured mesh,
    other data.
--stereo_options <string (default: "")>
    Options to pass to ``parallel_stereo``. Use double quotes
    around the full list and simple quotes if needed by an
    individual option, or vice-versa.
--pc_filter_options <string (default: "")>
    Options to pass to ``pc_filter``.
--mesh_gen_options <string (default: "")>
    Options to pass to ``voxblox_mesh`` for mesh generation.
--undistorted_crop_win <string (default: "")>
    The dimensions of the central image region to keep
    after undistorting an image and before using it in
    stereo. Normally 85% - 90% of distorted (actual)
    image dimensions would do. Suggested the Astrobee images: 
    sci_cam: '1250 1000' nav_cam: '1100 776'. haz_cam: '250 200'.
--first_step <string (default: "stereo")>
    Let the first step run by this tool be one of:
    'stereo', 'pc_filter', or 'mesh_gen'. This allows
    resuming a run at a desired step. The stereo
    subdirectories are deleted before that step takes
    place.
--last_step <string (default: "mesh_gen")>
    The last step run by this tool. See ``--first_step``
    for allowed values.
-h, --help
  Show this help message and exit.
