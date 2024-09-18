.. _cam2rpc:

cam2rpc
-------

This tool is used to generate an approximate RPC model for any camera
model supported by ASP, in a given longitude-latitude-height region for
a given datum, or for a terrain covered by a given DEM. If
``--save-tif-image`` is specified, the image portion corresponding to
the RPC model will be saved in the TIF format.

The produced RPC camera can be restricted to a given ground or pixel box.
In either case it will be consistent with the image portion that is saved.

The obtained RPC models and images can be used with ``stereo`` (when the
latter is invoked with ``--session-type rpc`` and the correct datum is
specified via ``--datum``). These can also be passed to the third-party
``S2P`` and ``SETSM`` stereo software, though both of these packages
work for Earth only.

The accuracy of RPC models generally degrades if expected to cover very
large regions. Hence, they can be used piecewise, and the obtained
terrain models from ASP can be then mosaicked together using
``dem_mosaic``.

Examples
~~~~~~~~

Example for ISIS cub cameras for Mars::

    cam2rpc input.cub output.xml --session-type isis    \
      --datum D_MARS --save-tif-image                   \
      --height-range -10000 -9000                       \
      --lon-lat-range 141.50 34.43 141.61 34.15         \
      --penalty-weight 0.03 --gsd 1

Example for pinhole cameras, with the lon-lat-height box produced from a DEM.
Here the image is also cropped and the image portion on which the RPC model is
defined is saved::

    cam2rpc input.tif input.tsai output.xml --session-type nadirpinhole   \
      --dem-file DEM.tif --save-tif-image --image-crop-box 90 70 5511 3675

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

Command-line options
~~~~~~~~~~~~~~~~~~~~

--datum <string>
    Set the datum. This will override the datum from the input
    images and also ``--t_srs``, ``--semi-major-axis``, and
    ``--semi-minor-axis``.
    Options:

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

--semi-major-axis <double>
    Explicitly set the datum semi-major axis in meters.

--semi-minor-axis <double>
    Explicitly set the datum semi-minor axis in meters.

--t_srs <string>
    Specify a GDAL projection string instead of the datum (in WKT, GeoJSON, or
    PROJ format).

--lon-lat-range <lon_min lat_min lon_max lat_max>
    The longitude-latitude range in which to compute the RPC model.
    Specify in the format: lon_min lat_min lon_max lat_max.

--height-range <min_height max_height>
    Minimum and maximum heights above the datum in which to compute
    the RPC model.

--dem-file <filename>
    Compute the longitude-latitude-height box in which to fit the RPC camera as
    the bounding box of the portion of this DEM that is seen by the input
    camera.

--num-samples <integer (default: 40)>
    How many samples to use in each direction in the
    longitude-latitude-height range.

--penalty-weight <float (default: 0.03)>
    A higher penalty weight will result in smaller higher-order RPC
    coefficients.

--save-tif-image
    Save a TIF version of the input image that approximately
    corresponds to the input longitude-latitude-height range and
    which can be used for stereo together with the RPC model.

--input-nodata-value <arg>
    Set the image input nodata value.

--output-nodata-value <arg>
    Set the image output nodata value.

-t, --session-type <string>
    Select the input camera model type. Normally this is auto-detected,
    but may need to be specified if the input camera model is in
    XML format. See :numref:`ps_options` for options.

--bundle-adjust-prefix <string>
    Use the camera adjustment obtained by previously running
    bundle_adjust with this output prefix.

--image-crop-box <minx miny widx widy>
    The output image and RPC model should not exceed this box,
    specified in input image pixels as minx miny widx widy.

--no-crop
    Try to create an RPC model over the entire input image, even
    if the input longitude-latitude-height box covers just a small
    portion of it. Not recommended.

--skip-computing-rpc
    Skip computing the RPC model.

--gsd <arg (default: -1)>
    Expected resolution on the ground, in meters. This is needed
    for SETSM.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

--tile-size <arg arg (default: 256 256)>
    Image tile size used for multi-threaded processing.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
