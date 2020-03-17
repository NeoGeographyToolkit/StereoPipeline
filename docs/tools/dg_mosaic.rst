.. _dg_mosaic:

dg_mosaic
---------

This tool can be used when processing Digital Globe images
(:numref:`dg_tutorial`). A Digital Globe satellite may
take a picture, and then split it into several images and corresponding
camera XML files. ``dg_mosaic`` will mosaic these images into a single
file, and create the appropriate combined camera XML file.

Digital Globe camera files contain, in addition to the original camera
models, their RPC approximations (:numref:`rpc`).
``dg_mosaic`` outputs both types of combined models. The combined RPC
model can be used to map-project the mosaicked images with the goal of
computing stereo from them (:numref:`mapproj-example`).

The tool needs to be applied twice, for both the left and right image
sets.

``dg_mosaic`` can also reduce the image resolution while creating the
mosaics (with the camera files modified accordingly).

Some older (2009 or earlier) Digital Globe images may exhibit seams upon
mosaicking due to inconsistent image and camera information. The
``--fix-seams`` switch can be used to rectify this problem. Its effect
should be minimal if such inconsistencies are not present.

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

--rpc-penalty-weight <float (default: 0.1)>
    The weight to use to penalize higher order RPC coefficients
    when generating the combined RPC model. Higher penalty weight
    results in smaller such coefficients.

--output-prefix <name>
    The prefix for the output .tif and .xml files.

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
