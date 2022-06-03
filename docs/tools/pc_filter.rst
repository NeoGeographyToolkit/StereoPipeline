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

Example::

    pc_filter --max-distance-from-camera 1.5      \
      --max-camera-ray-to-surface-normal-angle 75 \
      --input-cloud run/run-PC.tif                \
      --input-texture run/run-L.tif               \
      --camera left.tsai                          \
      --output-cloud run/run-filtered-PC.tif

Command-line options for ``pc_filter``:

--input-cloud <string (default="")>
    Input cloud name. A four-band .tif file as produced by stereo
    triangulation.

--output-cloud <string (default="")>
    Output cloud name. If having a .tif extension, the same format will
    be used as the input. Can also save ``.pcd`` and ``.ply`` files. In that
    case the points will be saved with ``float32`` values, so there may be
    some precision loss. The ``.pcd`` file will store in the field for the
    cloud normal the values image_texture, blending_weight,
    intersection_error, assuming these are computed.

--input-texture <string (default="")>
    If specified, read the texture from this file. Normally this is the
    file ``L.tif`` from the same run which produced the input point
    cloud.

--camera <string (default="")>
    The left or right camera used to produce this cloud. Used for some
    filtering operations.

--max-distance-from-camera <double (default=0.0)>
    If positive, remove points further from camera center than this
    value. Measured in meters.

--max-valid-triangulation-error <double (default=0.0)>
    If positive, points with triangulation error larger than this will
    be removed from the cloud. Measured in meters.

--max-camera-ray-to-surface-normal-angle <double (default=0.0)>
    If positive, points whose surface normal makes an angle with the
    ray back to the camera center greater than this will be removed as
    outliers. Measured in degrees.

--max-camera-dir-to-surface-normal-angle <double (default=0.0)>
    If positive, points whose surface normal makes an angle with the
    camera direction greater than this will be removed as
    outliers. This eliminates surfaces almost parallel to camera view
    direction. Measured in degrees.

--max-camera-dir-to-camera-ray-angle <double (default=0.0)>
    If positive, and a ray emanating from the camera and ending at the
    current point makes an angle with the camera direction bigger than
    this, remove the point as an outlier. In effect, this narrows the
    camera field of view.

--distance-from-camera-weight-power <double (default=0.0)>
    If positive, let the weight of a point be inversely proportional
    to the distance from the camera center to the point, raised to
    this power.

--blending-dist <double (default=0.0)>
    If positive and closer to any boundary of valid points than this
    (measured in point cloud pixels), decrease the weight assigned to
    the given point proportionally to remaining distance to boundary
    raised to a power. In effect, points closer to boundary are given
    less weight. Used in VoxBlox.

--blending-power <double (default=1.0)>
    Use this as the power when setting ``--blending-dist``.

--save-nodata-as-infinity
    If true and saving a ``.pcd`` file, set the x, y, z coordinates of
    an invalid point to infinity rather than to 0. Expected by
    VoxBlox.

--transform-to-camera-coordinates
    Transform the point cloud to the coordinate system of the camera
    provided with ``--camera``. For use with VoxBlox.

-v, --version
    Display the version of software.

-h, --help 
    Display the help message.
