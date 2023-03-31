.. _camera_solve:

camera_solve
------------

The ``camera_solve`` tool takes as input a set of images acquired with
a camera, and finds each camera's pose (position and orientation).  If
ground control points are provided, the resulting set of cameras is
transformed to be in a desired coordinate system. For examples and an
overview, see :numref:`sfm`.

This tool is a wrapper around the *Theia* structure-from-motion software
(http://theia-sfm.org/), and its goal is create camera models which
can later be used with ASP's bundle adjustment (:numref:`bundle_adjust`)
and stereo (:numref:`tutorial`). 

The camera calibration passed with the ``--calib-file`` option
should be a .tsai pinhole camera model file in one of the formats
compatible with ASP. Our supported pinhole camera models are described
in :numref:`pinholemodels`.

One can use a set of estimated camera positions to register camera
models in world coordinates. This method is not as accurate as using
ground control points but it may be easier to use. To do this, use the
``--camera-positions`` parameter to ``bundle_adjust`` via the
``--bundle-adjust-params`` option similar to the example line below. If
you see the camera models shifting too far from their starting positions
try using the ``--camera-weight`` option to restrain their movement.

This tool will generate two .tsai camera model files in the output
folder per input image. The first file, appended with .tsai, is in a
local coordinate system and does not include optimizations for intrinsic
parameters but it may be useful for debugging purposes. The second file,
appended with .final.tsai, contains the final solver results. If ground
control points or estimated camera positions were provided, then the
second file will be in a global coordinate system.

To customize the options passed to Theia, edit the flag file which is
saved in each output folder and pass it back to ``camera_solve`` via
``--theia-flagfile``, or use the option ``--theia-overrides``.

A related tool is ``theia_sfm`` (:numref:`theia_sfm`).

Example
^^^^^^^

::

    camera_solve                                                \ 
      --bundle-adjust-params '--camera-positions nav.csv        \
      --csv-format "1:file 12:lat 13:lon 14:height_above_datum" \
      --camera-weight 100.0'                                    \
      <other options>

Usage
^^^^^

::

   camera_solve [options] <output folder> <input images>

Command-line options
^^^^^^^^^^^^^^^^^^^^

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
    Pass Theia's computed interest point matches to bundle adjustment
    instead of recreating them (using potentially different methods).

--suppress-output
    Reduce the amount of program console output.

