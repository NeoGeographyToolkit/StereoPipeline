.. _point2mesh:

point2mesh
----------

The ``point2mesh`` tool produces a mesh file in ``.obj`` format that
can be visualized in any mesh viewer, such as Blender or MeshLab (see
:numref:`blender` and :numref:`meshlab` for details).

Unlike DEMs, the 3D mesh is not meant to be used as a finished
scientific product. Rather, it can be used for fast visualization to
create a 3D view of the generated terrain.

The ``point2mesh`` program requires a point cloud file or a DEM, and an
optional texture file. For example, it can be used with
``output-prefix-PC.tif`` and ``output-prefix-L.tif``, as output by
``stereo``, or otherwise with ``output-prefix-DEM.tif`` and
``output-prefix-DRG.tif``, with the latter two output by ``point2dem``
(:numref:`point2dem`).

When a texture file is not provided, a constant texture is applied. (A
mesh viewer will still show a color variation that depends on the
local curvature of the mesh.) In either case, ``point2mesh`` will
produce a mesh file in plain text format.

The ``-s`` (``--point-cloud-step-size``) flag sets the point cloud
sub-sampling rate, and dictates the degree to which the 3D model
should be simplified. For 3D reconstructions, this can be essential
for producing a model that can fit in memory. The default value is 10,
meaning every 10th point is used in the X and Y directions. In other
words that mean only :math:`1/10^2` of the points are being used to
create the model. Adjust this sampling rate according to how much
detail is desired, but remember that large models will impact the
frame rate of the 3D viewer and affect performance.

The ``--texture-step-size`` flag sets the texture sub-sampling rate.
For visualization it may be preferable for the produced cloud to be
rather coarse but for the texture overlaid on it to have higher
resolution. This program enforces that the cloud subsampling rate be a
multiple of the texture subsampling rate, hence the sampled texture
indices are a superset of the point cloud indices.

Examples
~~~~~~~~

::

    point2mesh --center --point-cloud-step-size 4  \
      --texture-step-size 2 output-prefix-PC.tif   \
      output-prefix-L.tif

    point2mesh --center -s 2 output-prefix-DEM.tif \
       output-prefix-DRG.tif

     meshlab output-prefix.obj

These examples use the option ``--center`` to shift the points towards
the origin, as otherwise, given that mesh vertices are measured from
planet center, and hence are large, mesh viewers, which typically use
float32 precision, may render the mesh with artifacts without this
option.

(Note that older versions of MeshLab may have a hard time opening a
mesh if the output prefix is a directory. In that case either open
the mesh from the GUI or change to that directory having the ``.obj``
file first and invoke MeshLab there.)

Command-line options
~~~~~~~~~~~~~~~~~~~~

-s, --point-cloud-step-size <integer (default: 10)>
    Sample by picking one out of these many samples from the point cloud.

--texture-step-size <integer (default: 2)>
    Sample by picking one out of these many samples from the texture.

--input-file <point-cloud-file>
    Explicitly specify the input file.

-o, --output-prefix <output-prefix>
    Specify the output prefix.

--texture-file <texture-file>
    Explicitly specify the texture file.

--center
    Let the origin be the midpoint of the bounding box of the
    cloud. Use this option if you are experiencing numerical precision
    issues.

--precision <integer (default: 17)>
    How many digits of precision to save.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
