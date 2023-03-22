.. _rig_calibrator_example:

A 3-sensor rig example
^^^^^^^^^^^^^^^^^^^^^^

This is an example using ``rig_calibrator`` on images acquired in a lab with
cameras mounted on the Astrobee robot
(https://github.com/nasa/astrobee). The data that can be used to reproduce this
is available `for download <https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/rig_calibrator>`_.

This robot has three cameras: ``nav_cam`` (wide field of view, using
the fisheye distortion model), ``sci_cam`` (narrow field of view,
using the radtan distortion model), and ``haz_cam`` (has depth
measurements, with one depth xyz value per pixel, narrow field of
view, using the radtan distortion model).

We assume the intrinsics of each sensor are reasonably well-known (but
will be optimized later), and we do not know each camera's pose. The
first step is then determining these, for which we use 
``theia_sfm`` (:numref:`theia_sfm`), as follows::

    theia_sfm --rig_config rig_input/rig_config.txt \
      --images 'rig_input/nav_cam/*tif
                rig_input/haz_cam/*tif 
                rig_input/sci_cam/*tif'             \
      --out_dir rig_theia

It will write the solved camera poses to ``rig_theia/cameras.nvm``.

This tool will use the Theia flags file from ``share/theia_flags.txt``
in the software distribution, which can be copied to a new name,
edited, and passed to this program via ``--theia_fags``.

Next, we run ``rig_calibrator``::

    float="focal_length,optical_center,distortion"
    float_all="nav_cam:${float} haz_cam:${float} sci_cam:${float}" 
    rig_calibrator                                        \
        --rig_config rig_input/rig_config.txt             \
        --nvm rig_theia/cameras.nvm                       \
        --camera_poses_to_float "nav_cam sci_cam haz_cam" \
        --intrinsics_to_float "$float_all"                \
        --depth_to_image_transforms_to_float "haz_cam"    \
        --float_scale                                     \
        --bracket_len 1.0                                 \
        --num_iterations 50                               \
        --calibrator_num_passes 2                         \
        --registration                                    \
        --hugin_file control_points.pto                   \
        --xyz_file xyz.txt                                \
        --export_to_voxblox                               \
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
this one to the other ones are floated as well, when passed in via the 
same option. The intrinsics are optimized as well 
(option ``--intrinsics_to_float``), and the scale of depth clouds
(``--float_scale``).

In this particular case, the real-world scale (but not orientation) would
have been solved for correctly even without registration, as it would
be inferred from the depth clouds. 

Since the ``nav_cam`` camera has a wide field of view, the values
in ``distorted_crop_size`` in the rig configuration are smaller than
actual image dimensions to reduce the worst effects of peripheral
distortion.

One could pass in ``--num_overlaps 3`` to get more interest point 
matches than what Theia finds, but this is usually not necessary.
This number better be kept small, especially if the features
are poor, as it may result in many outliers among images that
do not match well.

See :numref:`rig_calibrator_command_line` for the full list of options.

The obtained point clouds can be fused into a mesh using ``voxblox_mesh`` 
(:numref:`voxblox_mesh`), using the command::
    
    voxblox_mesh --index rig_out/voxblox/haz_cam/index.txt \
      --output_mesh rig_out/fused_mesh.ply                 \
      --min_ray_length 0.1 --max_ray_length 4.0            \
      --voxel_size 0.01

This assumes that depth sensors were present. Otherwise, can needs to
create point clouds with stereo, see :numref:`multi_stereo`.

The output mesh is ``fused_mesh.ply``, points no further than 2
meters from each camera center are used, and the mesh is obtained
after binning the points into voxels of 1 cm in size.

Full-resolution textured meshes can be obtained by projecting and
fusing the images for each sensor with ``texrecon``
(:numref:`texrecon`)::

    for cam in nav_cam sci_cam; do 
      texrecon --rig_config rig_out/rig_config.txt \
        --camera_poses rig_out/cameras.txt         \
        --mesh rig_out/fused_mesh.ply              \
        --rig_sensor ${cam}                        \
        --undistorted_crop_win '1000 800'          \
        --out_dir rig_out/texture
    done

The obtained textured meshes can be inspected for disagreements, by
loading them in MeshLab, as::

    meshlab rig_out/fused_mesh.ply        \
      rig_out/texture/nav_cam/texture.obj \
      rig_out/texture/sci_cam/texture.obj 

See an illustration in :numref:`rig_calibrator_textures`. See a larger
example in  :numref:`sfm_iss`, using two rigs.