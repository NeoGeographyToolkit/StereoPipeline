.. _dg_mosaic:

dg_mosaic
---------

This tool can be used when processing Digital Globe images
(:numref:`dg_tutorial`). Such an image product may be packaged
as several sub-images and corresponding camera XML
files. ``dg_mosaic`` will mosaic these sub-images into a single file, and
create the appropriate combined camera XML file.

The tool needs to be applied to the sub-images which form the left
image, and then in the same way to obtain the right image. These can 
then be passed to ``parallel_stereo`` (:numref:`parallel_stereo`).

Digital Globe camera files contain, in addition to the original camera
models, their RPC approximations (:numref:`rpc`).
``dg_mosaic`` outputs both types of combined models. The combined RPC
model can be used to mapproject the mosaicked images with the goal of
computing stereo from them (:numref:`mapproj-example`).

``dg_mosaic`` can also reduce the image resolution while creating the mosaics
(with the camera files modified accordingly). This may result in aliasing
artifacts in produced terrain models, but can be useful for quick previews.

Some older (2009 or earlier) Digital Globe images may exhibit seams upon
mosaicking due to inconsistent image and camera information. The
``--fix-seams`` switch can be used to rectify this problem. Its effect
should be minimal if such inconsistencies are not present.

Digital Globe images can be used as they are as well, but in that case
each left sub-image needs to be paired up with one more more right
sub-images, create individual DEMs, and then mosaic those into a
single output DEM. Hence this program simplifies the data processing.

Only level 1B images are supported (see the `product info 
<https://securewatchdocs.maxar.com/en-us/Orders/Orders_ProductInfo.htm>`_).

Example::

    dg_mosaic WV03_20160925183217*P1BS_R*C1-012844055010_01_P001.tif

This will create ``012844055010_01_P001.r100.tif`` and
``012844055010_01_P001.r100.xml``, where ``r.100`` stands for the full
100% resolution.

Care should be taken to not mosaic together PAN and multispectral
images, which have ``P1BS`` and ``M1BS`` as part of their names.

Command-line options for dg_mosaic:

-h, --help
    Display the help message.

--target-resolution
    Choose the output resolution in meters per pixel on the ground
    (note that a coarse resolution may result in aliasing).

--reduce-percent <integer (default: 100)>
    Render a reduced resolution image and XML based on this percentage.
    This can result in aliasing artifacts.

--skip-rpc-gen
    Skip RPC model generation.

--skip-tif-gen
    Skip TIF file generation.

--rpc-penalty-weight <float (default: 0.1)>
    The weight to use to penalize higher order RPC coefficients
    when generating the combined RPC model. Higher penalty weight
    results in smaller such coefficients.

--output-prefix <name>
    The prefix for the output .tif and .xml files.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB, for each process.

--band integer
    Which band to use (for multi-spectral images).

--input-nodata-value <float>
    Nodata value to use on input; input pixel values less than or
    equal to this are considered invalid.

--output-nodata-value <float>
    Nodata value to use on output.

--ot <Byte|UInt16|Int16|UInt32|Int32|Float32 (default: Float32)>
    Output data type. If the output type is a kind of integer, values
    are rounded and then clamped to the limits of that type.

--fix-seams 
    Fix seams in the output mosaic due to inconsistencies between
    image and camera data using interest point matching.

--ignore-inconsistencies
    Ignore the fact that some of the files to be mosaicked have
    inconsistent EPH/ATT values. Do this at your own risk.

--preview
    Render a small 8 bit png of the input for preview.

-n, --dry-run
    Make calculations, but just print out the commands.
