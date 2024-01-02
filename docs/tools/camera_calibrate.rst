.. _camera_calibrate:

camera_calibrate
----------------

The ``camera_calibrate`` tool can generate camera models suitable for
use by ``camera_solve`` and other ASP tools. This tool only solves for
intrinsic camera parameters; to obtain the camera pose you should use
the ``camera_solve`` tool. This tool is a wrapper around the OpenCV
(http://opencv.org/) checkerboard calibration tool which takes care of
converting the output into readily usable formats. When you run the
tool, three camera model files will be created in the output folder:
``solve_cam_params.txt``, ``vw_cam_params.tsai``, and
``ocv_cam_params.yml``. The first file can be used as a camera
calibration file for the ``camera_solve`` tool. The second file is a
pinhole camera format that is recognized by ASP but remember that the
extrinsic parameters were not solved for so ASP is limited in what it
can do with the camera file. The last file contains the camera
information as formatted by the OpenCV calibration tool. If you use the
first file as an input to ``camera_solve`` you must remember to replace
the wildcard image path in the file with the one to the images you want
to use solve for (as opposed to the checkerboard images).

In order to use this tool you must provide multiple images of the same
checkerboard pattern acquired with the camera you wish to calibrate.
When calling the tool you must specify the number of *inner* square
corners contained in your checkerboard pattern (width and height can be
swapped) so that OpenCV knows what to look for. You must also specify an
image wildcard path such as ``"checkers/image_.jpg"``. You may need to
enclose this parameter in quotes so that your command line does not
expand the wildcard before it is passed to the tool. If you do not
provide the ``–box-size`` parameter the output calibration numbers will
be unitless.

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
    Make a copy of the vw param file for each input camera.
