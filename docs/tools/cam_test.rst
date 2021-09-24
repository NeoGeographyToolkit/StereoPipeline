.. _cam_test:

cam_test
--------

This tool compares two camera models for the same image. It find the
camera center and ray direction at a set of sampled pixels for both
cameras and checks how they differ. Then, it projects pixels to the
ground using the first camera and back-projects the resulting ECEF
points into second camera, and compares the input pixel with the
resulting output pixel.

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

Usage::

    cam_test --image <image file> --cam1 <camera 1 file> \
      --cam2 <camera 2 file> [other options]

Command-line options for cam_test:

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

-h, --help
    Display the help message.

