.. _camera_solve:

camera_solve
------------

The ``camera_solve`` tool generates pinhole sensor models (frame
cameras), including camera poses, for input images lacking metadata. See
:numref:`sfm` for an overview and examples of using the tool.

The camera calibration passed with the ``--calib-file`` option
should be a .tsai pinhole camera model file in one of the formats
compatible with ASP. Our supported pinhole camera models are described
in :numref:`pinholemodels`.

You can use a set of estimated camera positions to register camera
models in world coordinates. This method is not as accurate as using
ground control points but it may be easier to use. To do this, use the
``--camera-positions`` parameter to ``bundle-adjust`` via the
``--bundle-adjust-params`` option similar to the example line below. If
you see the camera models shifting too far from their starting positions
try using the ``--camera-weight`` option to restrain their movement.

::

   --bundle-adjust-params '--camera-positions nav.csv \
    --csv-format "1:file 12:lat 13:lon 14:height_above_datum" --camera-weight 1.0'

This tool will generate two .tsai camera model files in the output
folder per input image. The first file, appended with .tsai, is in a
local coordinate system and does not include optimizations for intrinsic
parameters but it may be useful for debugging purposes. The second file,
appended with .final.tsai, contains the final solver results. If ground
control points or estimated camera positions were provided then the
second file will be in a global coordinate system.

Usage::

     > camera_solve [options] <output folder> <Input Image 1> <Input Image 2> ...

Command-line options for camera_solve:

-h, --help
    Display this help message.

--datum <string>
    The datum to use when calibrating. Default is WGS84.

--calib-file <filename>
    Path to an ASP compatible pinhole model file containing camera
    model information. The position and pose information will be
    ignored. If you want to use a unique file for each input image,
    pass a space separated list of files surrounded by quotes.

--gcp-file <filename>
    Path to a ground control point file. This allows the tool to
    generate cameras in a global coordinate system.

--bundle-adjust-params <string>
    Additional parameters (in single quotes) to pass to the
    ``bundle_adjust`` tool.

--theia-overrides <string>
    Override any option in the auto-generated Theia flag file.  Set
    as ``"--option1=val1 --option2=val2 ..."``.

--theia-flagfile <filename>
    Path to a custom Theia flagfile to use settings from. File paths
    specified in this file are ignored.

--overwrite
    Recompute any intermediate steps already completed on disk.

--reuse-theia-matches
    Pass Theia’s IP find results into ASP instead of recomputing
    them to reduce total processing time.

--suppress-output
    Reduce the amount of program console output.

This tool is a wrapper that relies on on two other tools to operate. The
first of these is THEIA, as mentioned earlier, for computing the
relative poses of the cameras. ASP’s ``bundle_adjust`` tool is used to
register the cameras in world coordinates using the ground control
points. If the tool does not provide good results you can customize the
parameters being passed to the underlying tools in order to improve the
results. For ``bundle_adjust`` options, see the description in this
document. For more information about THEIA flagfile options see their
website or edit a copy of the default flagfile generated in the output
folder.
