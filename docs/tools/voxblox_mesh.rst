.. _voxblox_mesh:

voxblox_mesh
------------

The ``voxblox_mesh`` program takes as input several camera poses, and
for each such pose a point cloud in that camera's coordinates, and
fuses them into a (mostly seamless) mesh. 

The input point clouds can be created either with stereo or a depth
sensor.

This tool is a wrapper around the ``VoxBlox`` software at:

    https://github.com/ethz-asl/voxblox

Example
^^^^^^^

The dataset:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/rig_calibrator

has an example of how a set of camera images and clouds acquired with
a rig were co-registered using the ``rig_calibrator`` program
(:numref:`rig_calibrator`).

With that data and this tool, a fused mesh can be obtained as follows::

    max_ray_len=2.0
    voxel_size=0.01
    voxblox_mesh rig_out/voxblox/haz_cam/index.txt \
      rig_out/fused_mesh.ply $max_ray_len $voxel_size

Here, the output mesh is ``fused_mesh.ply``, points no further than 2
meters from each camera center are used, and the mesh is obtained
after binning the points into voxels of 1 cm in size. 

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

In the future there will be updates to these tools to ensure that stereo 
point clouds are fused in a mesh without artifacts.

