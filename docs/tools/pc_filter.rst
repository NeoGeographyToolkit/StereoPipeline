.. _pc_filter:

pc_filter
---------

This program takes as input a four-band ``PC.tif`` point cloud as
created by stereo triangulation (:numref:`outputfiles`) and applies
various outlier filters. It can be especially useful for indoor stereo
datasets, which can have poor texture and very oblique angles, and
hence noisy point clouds.

It is assumed that the input cloud fits fully in memory, and some
filtering options expect that the cloud was created with pinhole
cameras. This may change in future versions.

The output cloud has the same format and dimensions as the input, with
outliers replaced by points with all coordinates equal to 0. The cloud
can also be saved as ``.ply`` and ``.pcd`` files (depending on the
output cloud extension). In those cases the coordinates are saved as
float32, which may result in loss of precision for orbital data.

An .obj file textured mesh can be created from the output cloud using
``point2mesh`` (:numref:`point2mesh`), just as for the original cloud.

Usage::

    pc_filter [options] --input-cloud input.tif --output-cloud output.tif

Command-line options for pc_filter:

--help  
    Display the help message

