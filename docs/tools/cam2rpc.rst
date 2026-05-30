.. _cam2rpc:

cam2rpc
-------

This tool is used to generate an approximate RPC model (:numref:`rpc`) for any
camera model supported by ASP, in a given longitude-latitude-height region for a
given datum, or for a terrain covered by a given DEM. If ``--save-tif-image`` is
specified, the image portion corresponding to the RPC model will be saved in the
TIF format.

The produced RPC camera can be restricted to a given ground or pixel box.
In either case it will be consistent with the image portion that is saved.

The accuracy of RPC models generally degrades if expected to cover very
large regions. Hence, they can be used piecewise, and the obtained
terrain models from ASP can be then mosaicked together using
``dem_mosaic`` (:numref:`dem_mosaic`).

Examples
~~~~~~~~

ISIS cub cameras for Mars
^^^^^^^^^^^^^^^^^^^^^^^^^

::

    cam2rpc input.cub output.xml --session-type isis    \
      --datum D_MARS --save-tif-image                   \
      --height-range -10000 -9000                       \
      --lon-lat-range 141.50 34.43 141.61 34.15         \
      --penalty-weight 0.03 --gsd 1

Pinhole cameras and input DEM
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Example for pinhole cameras, with the lon-lat-height box produced from a DEM.
Here the image is also cropped and the image portion on which the RPC model is
defined is saved::

    cam2rpc                            \
      --session-type nadirpinhole      \
      --dem-file DEM.tif               \
      --save-tif-image                 \
      --image-crop-box 90 70 5511 3675 \
      input.tif input.tsai output.xml

Datum for other planets
^^^^^^^^^^^^^^^^^^^^^^^

For a body without a standard datum name (one not in the ``--datum`` list
below), set the datum by its ellipsoid axes or by a projection string.

Here is how to invoke this program when setting the semi-axes for Europa (radius
1,560,800 m)::

    cam2rpc input.tif input.tsai output.xml               \
      --semi-major-axis 1560800 --semi-minor-axis 1560800 \
      --lon-lat-range 141.55 34.20 141.56 34.23           \
      --height-range -500 1500

Alternatively, use a PROJ, WKT, or GeoJSON string passed to ``--t_srs`` (inline
or as a file). Only the datum is used, not the projection::

    cam2rpc input.tif input.tsai output.xml               \
      --t_srs '+proj=longlat +R=1560800 +no_defs'         \
      --lon-lat-range 141.55 34.20 141.56 34.23           \
      --height-range -500 1500

The chosen datum is saved to the output RPC file (in the ``<RPC_DATUM>``
field) and read back automatically by ASP tools, so no body name is needed
downstream.

Uses
~~~~

The obtained RPC models and (potentially cropped) images can be used with the
ASP tools when invoked with ``--session-type rpc``. The datum is read
automatically from the ``<RPC_DATUM>`` field written by ``cam2rpc``, so no
datum needs to be specified downstream. For RPC files lacking that field
(such as vendor-provided models, which default to WGS84), the datum can be
overridden with the ``--datum`` option.

The RPC models should work with the third-party ``S2P`` and ``SETSM`` stereo
software. Note that both of these packages are for Earth only.

The produced RPC model file can be read by GDAL (including ``gdalinfo``,
:numref:`gdal_tools`) if it has the same name as the image but with the .xml
extension, and no analogously named ``.rpb`` or ``_RPC.txt`` files are present.

Validation
~~~~~~~~~~

It is suggested to mapproject the produced image and camera onto a DEM using
``mapproject`` (:numref:`mapproject`) and compare with the result from the
original image and camera.

In addition, *if the produced image file contains the original upper-left image
corner*, the ``cam_test`` program (:numref:`cam_test`) can be invoked to compare
the original and resulting RPC camera, for example as follows::

    cam_test --image output.tif --cam1 input.tsai --cam2 output.xml \
      --height-above-datum val

Here, ``val`` is a value that is between the minimum and maximum heights used
when the RPC model was computed.

This should result in similar values for the camera directions and pixel
differences, but not for the camera centers, because the RPC model does not have
a well-defined camera center.

Large pixel differences are a sign that the image was not cropped to the region
containing the original upper-left corner (option ``--no-crop`` may help then,
in addition to specifying larger lon-lat bounds or a bigger DEM to fit to).

Usage
~~~~~

::

     cam2rpc [options] <camera-image> <camera-model> <output-rpc>

See also
~~~~~~~~

To export an existing RPC camera to a file in XML format, or to create a
Pinhole camera, use ``cam_gen`` (:numref:`cam_gen`).

To simulate a linescan (or frame) camera together with synthetic images,
use ``sat_sim`` (:numref:`sat_sim`).

Command-line options
~~~~~~~~~~~~~~~~~~~~

--datum <string (default: "")>
    Set the datum. It overrides the datum from the input images and the
    ``--semi-major-axis`` / ``--semi-minor-axis`` values, but is itself
    overridden by ``--t_srs``. Cannot be combined with ``--dem-file``. Options:

    - WGS_1984
    - D_MOON (1,737,400 meters)
    - D_MARS (3,396,190 meters)
    - MOLA (3,396,000 meters)
    - NAD83
    - WGS72
    - NAD27
    - Earth (alias for WGS_1984)
    - Mars (alias for D_MARS)
    - Moon (alias for D_MOON)

--semi-major-axis <double (default: 0)>
    Explicitly set the datum semi-major axis in meters (use equal axes for a
    sphere). Overridden by ``--datum`` or ``--t_srs``. Cannot be combined with
    ``--dem-file``.

--semi-minor-axis <double (default: 0)>
    Explicitly set the datum semi-minor axis in meters. Overridden by
    ``--datum`` or ``--t_srs``. Cannot be combined with ``--dem-file``.

--t_srs <string (default: "")>
    Specify a GDAL projection string instead of the datum (in WKT, GeoJSON, or
    PROJ format; given inline or as a file). Only the datum (ellipsoid) is
    used, not the projection. Overrides ``--datum`` and the semi-axes. Cannot
    be combined with ``--dem-file``.

--lon-lat-range <lon_min lat_min lon_max lat_max (default: 0 0 0 0)>
    The longitude-latitude range in which to compute the RPC model.
    Specify in the format: lon_min lat_min lon_max lat_max.

--height-range <min_height max_height (default: 0 0)>
    Minimum and maximum heights above the datum in which to compute
    the RPC model.

--dem-file <filename (default: "")>
    Compute the longitude-latitude-height box in which to fit the RPC camera as
    the bounding box of the portion of this DEM that is seen by the input
    camera. The datum is taken from this DEM. Cannot be combined with
    ``--datum``, ``--t_srs``, ``--semi-major-axis``, or ``--semi-minor-axis``.

--num-samples <integer (default: 40)>
    How many samples to use in each direction in the
    longitude-latitude-height range.

--penalty-weight <double (default: 0.03)>
    A higher penalty weight will result in smaller higher-order RPC
    coefficients.

--save-tif-image
    Save a TIF version of the input image that approximately
    corresponds to the input longitude-latitude-height range and
    which can be used for stereo together with the RPC model.

--input-nodata-value <double (default: NaN)>
    Set the image input nodata value.

--output-nodata-value <double (default: NaN)>
    Set the image output nodata value. If not specified, the input nodata value
    will be used.

-t, --session-type <string>
    Select the input camera model type. Normally this is auto-detected,
    but may need to be specified if the input camera model is in
    XML format. See :numref:`ps_options` for options.

--bundle-adjust-prefix <string>
    Use the camera adjustment obtained by previously running
    bundle_adjust with this output prefix.

--image-crop-box <minx miny widx widy (default: 0 0 0 0)>
    The output image and RPC model should not exceed this box,
    specified in input image pixels as minx miny widx widy.

--no-crop
    Try to create an RPC model over the entire input image, even
    if the input longitude-latitude-height box covers just a small
    portion of it. Not recommended.

--skip-computing-rpc
    Skip computing the RPC model.

--gsd <double (default: -1)>
    Expected resolution on the ground, in meters. This is needed
    for SETSM.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.

--cache-size-mb <integer (default: 1024)>
    Set the system cache size, in MB.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

--tile-size <arg arg (default: 256 256)>
    Image tile size used for multi-threaded processing.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
