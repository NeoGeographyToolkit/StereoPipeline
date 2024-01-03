.. _camera_calibrate:

camera_calibrate
----------------

The ``camera_calibrate`` tool can generate camera models suitable for use by
``camera_solve`` (:numref:`camera_solve`), and other ASP tools. This program
only solves for intrinsic camera parameters. To obtain the camera pose you
should use ``camera_solve``. 

This tool is a wrapper around the OpenCV (http://opencv.org/) checkerboard
calibration tool which takes care of converting the output into readily usable
formats. 

When you run the tool, three camera model files will be created in the output
folder: ``solve_cam_params.txt``, ``vw_cam_params.tsai``, and
``ocv_cam_params.yml``. The first file can be used as a camera calibration file
for ``camera_solve``. The second file is a pinhole camera format that
is recognized by ASP but remember that the extrinsic parameters were not solved
for so ASP is limited in what it can do with the camera file. The last file
contains the camera information as formatted by the OpenCV calibration tool. 

If you use the first file as an input to ``camera_solve`` you must remember to
replace the wildcard image path in the file with the one to the images you want
to use solve for (as opposed to the checkerboard images).

In order to use this tool you must provide multiple images of the same
checkerboard pattern acquired with the camera you wish to calibrate. When
calling the tool you must specify the number of *inner* square corners contained
in your checkerboard pattern (width and height can be swapped) so that OpenCV
knows what to look for. 

You must also specify an image wildcard path such as ``"checkers/image*.jpg"``.
This must be in quotes so that the wildcard is not expanded before it is passed
to the tool. If you do not provide the ``--box-size`` parameter, the output
calibration numbers will be unitless.

ASP also ships the ``rig_calibrator`` program (:numref:`rig_calibrator`),
which calibrates the intrinsic and extrinsics of a rig of cameras, without
using a calibration target.

Example::

    camera_calibrate --box-size-cm 4.28625 outputFolder \
      12 12 "images/image*.jpg"

Here, the checkerboard pattern has 13 squares in each direction, so 
12 inner corners. 

Usage::

     camera_calibrate [options] <output folder> \
        <num inner vertical corners>            \
        <num inner horizontal corners>          \
        <image wildcard>

Command-line options for camera_calibrate:

-h, --help
    Display this help message.

--overwrite
    Recompute any intermediate steps already completed on disk.

--suppress-output
    Reduce the amount of program console output.

--box-size-cm <float>
    The size of the checkerboard squares in centimeters.

--duplicate-files
    Make a copy of the VisionWorkbench parameter file for each input camera.
