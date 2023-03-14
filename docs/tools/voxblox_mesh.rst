.. _voxblox_mesh:

voxblox_mesh
------------

The ``voxblox_mesh`` program takes as input several camera poses, for
each such pose a point cloud in that camera's coordinates, and fuses
them into a mesh. The input point clouds can be created either with
stereo (such as with ``multi_stereo``, :numref:`multi_stereo`) or a
depth sensor (see the example below). 

The produced mesh can be textured with ``texrecon`` (:numref:`texrecon`).

A basic median filter can be applied to the input clouds before fusing them.

This tool is a wrapper around `VoxBlox <https://github.com/ethz-asl/voxblox>`_.
 
Example
^^^^^^^

The dataset:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/rig_calibrator

has an example of how a set of camera images and depth clouds acquired
with a rig were co-registered using the ``rig_calibrator`` program
(:numref:`rig_calibrator`).

With that data and this tool, a fused mesh can be obtained as follows::

    voxblox_mesh --index rig_out/voxblox/haz_cam/index.txt \
      --output_mesh rig_out/fused_mesh.ply                 \
      --min_ray_length 0.1 --max_ray_length 2.0            \
      --voxel_size 0.005

Here, the output mesh is ``fused_mesh.ply``, points no further than 2
meters from each camera center are used, and the mesh is obtained
after binning the points into voxels of 1 cm in size. 

The obtained mesh can be post-processed, by smoothing it, filling in holes,
etc., using several CGAL tools shipped with ASP (:numref:`cgal_tools`).

Format of the inputs
^^^^^^^^^^^^^^^^^^^^

The input index file to this program has two lines for each cloud. The
first is a file having the camera-to-world transform for a point
cloud, as a 4x4 matrix (rotation and translation), and the second one
has the cloud itself, in binary ``.pcd`` format, in camera's
coordinate system. Here is an example::

    data_dir/1_cam2world.txt
    data_dir/1.pcd
    data_dir/2_cam2world.txt
    data_dir/2.pcd

Each .pcd file is in the ``pcl::PointNormal`` format, with each data
point having 6 entries, corresponding to cloud x, y, z, point
intensity, reliability weight given to the point, and the value 0 (the
latter is unused).

Such clouds can be created with the ``pc_filter`` program from ASP's
usual ``PC.tif`` point clouds (:numref:`pc_filter`). One has to make
sure the cloud is transformed to camera coordinates. The command to
create such a ``.pcd`` file is along the lines of::

    pc_filter --transform-to-camera-coordinates   \ 
      --max-distance-from-camera 2.0              \
      --max-camera-ray-to-surface-normal-angle 75 \
      --input-cloud run/run-PC.tif                \
      --input-texture run/run-L.tif               \
      --camera left.tsai                          \
      --output-cloud run/run.pcd

Note how the camera file ``left.tsai`` which was used to create the PC.tif
cloud is passed in as an argument.

.. _voxblox_mesh_command_line:

Command-line options for voxblox_mesh
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``--index`` A list having input camera transforms and point cloud files.
    Type: string. Default: "".
``--output_mesh`` The output mesh file name, in .ply format. Type: string.
    Default: "".
``--voxel_size`` Voxel size, in meters. Type: double. 
    Default: 0.01.
``--min_ray_length`` (The minimum length of a ray from camera center to the
  points. Points closer than that will be ignored. Type: double. Default: -1.
``--max_ray_length`` The maximum length of a ray from camera center to the
    points. Points beyond that will be ignored. Type: double. Default: -1.
``--enable_anti_grazing`` If true, enable anti-grazing. This is an advanced
    option. Type: bool. Default: false.
``--integrator`` Specify how the points should be integrated. Options:
    "simple", "merged", "fast". See the VoxBlox documentation for details.
    Type: string. Default: "merged".
``--min_weight`` The minimum weighting needed for a point to be included in the
    mesh. Type: double. Default: 1e-6.
``--voxel_carving_enabled`` If true, the entire length of a ray is integrated.
    Otherwise only the region inside the truncation distance is used. This is
    an advanced option. Type: bool. Default: false.
``--median_filter`` Filter out depth points that differ in any of the coordinates by more
    than a given threshold from the median of such
    coordinates in a square window of given size. Specify in quotes,
    as: 'window thresh'. The window is an odd integer and in units of pixel
    (given the image storage format of the cloud) and the threshold is
    measured in meters. This assumes that the input .pcd files have more
    than one row and column, rather than the data being
    stored in a single row. Type: string. Default: "".

See also the `VoxBlox documentation
<https://voxblox.readthedocs.io/en/latest/pages/The-Voxblox-Node.html#parameters>`_.
