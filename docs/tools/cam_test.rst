.. _cam_test:

cam_test
--------

This tool compares two camera models for the same image, or a camera
model against itself. It find the camera center and ray direction at a
set of sampled pixels for both cameras and checks how they
differ. Then, it projects pixels to the ground using the first camera
and back-projects the resulting ECEF points into second camera, and
again estimates their discrepancy.

It prints the average time (in milliseconds) for the operation of
projecting from the camera to the ground and back.

Example (compare a PeruSat-1 exact linescan model to its RPC
approximation)::

    cam_test --image input.tif --cam1 exact_cam.xml --cam2 rpc_cam.xml

Example (compare ISIS to CSM cameras)::

    cam_test --image input.cub --cam1 input.cub --cam2 input.json \
      --sample-rate 5000

Example (compare the exact and RPC model stored in the same Digital
Globe file)::

    cam_test --image input.tif --cam1 input.xml --cam2 input.xml \
      --session1 dg --session2 rpc --sample-rate 1000

Evaluate a camera transformed with ``convert_pinhole_model`` 
(:numref:`convert_pinhole_model`). In this case the session names
would be the same but the cameras would differ::

    cam_test --image input.tif --cam1 in.tsai --cam2 out.tsai \
      --session1 pinhole --session2 pinhole

In the following example we evaluate a CSM camera against itself, with
no .cub image file. The image dimensions are contained in the camera
file. This verifies that the ground-to-image and image-to-ground
functions are inverse of each other, up to a certain tolerance.

::

    cam_test --image input.json --cam1 input.json --cam2 input.json \
      --session1 csm --session2 csm --sample-rate 100               \
      --subpixel-offset 0.3

Usage::

    cam_test --image <image file> --cam1 <camera 1 file> \
      --cam2 <camera 2 file> [other options]

Command-line options for cam_test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--image <string>
    Image file.

--cam1 <string>
    Camera 1 file.

--cam2 <string>
    Camera 2 file.

--session1 <string>
    Session to use for camera 1 (if not provided it will be guessed).

--session2 <string>
    Session to use for camera 2 (if not provided it will be guessed).

--sample-rate <integer (default: 100)>
    Use one out of these many pixels when sampling the image.

--subpixel-offset <double (default: 0.0)>
    Add to each integer pixel this offset (in x and y) when sampling
    the image. Sampling at non-integer location may make testing
    more thorough.

--height-above-datum <double (default: 0.0)>
    Let the ground be obtained from the datum for this camera by 
    adding to its radii this value (the units are meters).

--single-pixel <double double (default: nan nan)>
    Instead of sampling pixels from the image use only this pixel.

--print-per-pixel-results
    Print the results at each pixel.

--enable-correct-velocity-aberration
    Turn on velocity aberration correction for Optical Bar and
    non-ISIS linescan cameras (:numref:`sensor_corrections`).
    This option impairs the convergence of bundle adjustment.

--enable-correct-atmospheric-refraction
    Turn on atmospheric refraction correction for Optical Bar and
    non-ISIS linescan cameras. This option impairs the convergence of
    bundle adjustment.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

-h, --help
    Display the help message.

