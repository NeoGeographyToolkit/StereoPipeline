.. _cam_test:

cam_test
--------

This tool compares two camera models for the same image, or a camera
model against itself. It finds the camera center and ray direction at a
set of sampled pixels for both cameras and checks how they
differ. Then, it projects pixels to the datum using the first camera
and back-projects the resulting ECEF points into the second camera, and
estimates the pixel discrepancy.

It prints the average time (in milliseconds) for the operation of
projecting from the camera to the ground and back.

Examples
~~~~~~~~

See :numref:`examples` for information on the camera types used below.

Compare a CSM camera model against itself::

  cam_test --image input.cub --cam1 input.json --cam2 input.json \
    --session1 csm --session2 csm

Compare a PeruSat-1 exact linescan model to its RPC
approximation::

    cam_test --image input.tif --cam1 exact_cam.xml --cam2 rpc_cam.xml

Here the two individual camera types will be auto-guessed as ``perusat`` and
``rpc``, or can be specified as above with ``--session1`` and ``--session2``.

Compare ISIS to CSM cameras::

    cam_test --image input.cub --cam1 input.cub --cam2 input.json \
      --sample-rate 5000

Compare the exact and RPC model stored in the same Digital
Globe file::

    cam_test --image input.tif --cam1 input.xml --cam2 input.xml \
      --session1 dg --session2 rpc --sample-rate 1000

Evaluate a camera transformed with ``convert_pinhole_model`` 
(:numref:`convert_pinhole_model`). In this case the session names
would be the same but the cameras would differ::

    cam_test --image input.tif --cam1 in.tsai --cam2 out.tsai \
      --session1 pinhole --session2 pinhole

Here we evaluate a CSM camera against itself, with
no .cub image file. The image dimensions are contained in the camera
file. This verifies that the ground-to-image and image-to-ground
functions are inverse of each other, up to a certain tolerance.

::

    cam_test --image input.json --cam1 input.json --cam2 input.json \
      --session1 csm --session2 csm --sample-rate 100               \
      --subpixel-offset 0.3

Compare the existing ASTER camera implementation to the new one
that uses the CSM camera model::

    cam_test --image image.tif --cam1 image.xml --cam2 image.xml \
      --session1 aster --session2 aster --aster-vs-csm

Usage
~~~~~

::

    cam_test --image <image file> --cam1 <camera 1 file> \
      --cam2 <camera 2 file> [other options]

Command-line options
~~~~~~~~~~~~~~~~~~~~

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

--single-pixel <double double (default: nan nan)>
    Instead of sampling pixels from the image use only this pixel.

--print-per-pixel-results
    Print the results at each pixel.

--height-above-datum <double (default: 0.0)>
    Let the ground be obtained from the datum for this camera by 
    adding to its radii this value (the units are meters).

--datum <string>
    Set the datum. This will override the datum from the input cameras. 
    Usually needed only for Pinhole cameras for non-Earth planets, when 
    the camera does not have the datum information. Options: WGS_1984, 
    D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA 
    (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth 
    (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).

--bundle-adjust-prefix  <string (default = "")>
    Adjust the cameras using this prefix.

--cam1-bundle-adjust-prefix <string (default = "")>
    Adjust the first camera using this prefix.

--cam2-bundle-adjust-prefix <string (default = "")>
    Adjust the second camera using this prefix.
    
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--aster-use-csm
    Use the CSM model with ASTER cameras (``-t aster``).
    
--aster-vs-csm
    Compare projecting into the camera without and with using the CSM
    model for ASTER.    
    
-h, --help
    Display the help message.
