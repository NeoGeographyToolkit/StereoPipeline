.. _point2mesh:

point2mesh
----------

The ``point2mesh`` tool produces a mesh file in .obj format that can be visualized
in any mesh viewer, such as MeshLab, which can be downloaded from 

  https://github.com/cnr-isti-vclab/meshlab/releases

and can be installed and run in user's directory without needing
administrative priveledges.

Unlike DEMs, the 3D mesh is not meant to be used as a finished
scientific product. Rather, it can be used for fast visualization to
create a 3D view of the generated terrain.

The ``point2mesh`` program requires a point cloud file or a DEM, and an
optional texture file. For example, it can be used with
``output-prefix-PC.tif`` and ``output-prefix-L.tif``, as output by
``stereo``, or otherwise with ``output-prefix-DEM.tif`` and
``output-prefix-DRG.tif``, with the latter two output by ``point2dem``.

When a texture file is not provided, a 1D texture is applied in the
local Z direction that produces a rough rendition of a contour map. In
either case, ``point2mesh`` will produce a mesh file in plain text format.

The ``-s`` flag sets the sub-sampling rate, and dictates the degree to
which the 3D model should be simplified. For 3D reconstructions, this
can be essential for producing a model that can fit in memory. The
default value is 10, meaning every 10th point is used in the X and Y
directions. In other words that mean only :math:`1/10^2` of the points
are being used to create the model. Adjust this sampling rate
according to how much detail is desired, but remember that large
models will impact the frame rate of the 3D viewer and affect
performance.

Examples::

     point2mesh -s 2 output-prefix-PC.tif output-prefix-L.tif
     point2mesh -s 2 output-prefix-DEM.tif output-prefix-DRG.tif

To view the resulting ``output-prefix.osgb`` file use ``osgviewer``.

Fullscreen::

    > osgviewer output-prefix.osgb

The ``-t`` output file type option can also take the ``obj`` value, when
it will write the mesh in the Wavefront OBJ format file
(``output-prefix.obj``) that can be read into various 3D graphics
programs.

Command-line options for point2mesh:

-h, --help
    Display the help message.

--simplify-mesh <float>
    Run OSG Simplifier on mesh, 1.0 = 100%.

--smooth-mesh
    Run OSG Smoother on mesh.

--use-delaunay
    Uses the delaunay triangulator to create a surface from the
    point cloud. This is not recommended for point clouds with noise
    issues.

-s, --step <integer (default: 10)>
    Sampling step size for the mesher.

--input-file <pointcloud-file>
    Explicitly specify the input file.

-o, --output-prefix <output-prefix>
    Specify the output prefix.

--texture-file <texture-file>
    Explicitly specify the texture file.

-t, --output-filetype <type (default: osgb)>
    Specify the output file type.  The type 'obj' is possible here.

--center
    Center the model around the origin. Use this option if you are
    experiencing numerical precision issues.
