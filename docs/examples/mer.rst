.. _mer-example:

Mars Exploration Rovers
-----------------------

The Mars Exploration Rovers (MER) have several cameras on board and they
all seem to have a stereo pair. With ASP you are able to process the
PANCAM, NAVCAM, and HAZCAM camera images. ISIS has no telemetry or
camera intrinsic supports for these images. That however is not a
problem as their raw images contain the cameras information in JPL's
CAHV, CAHVOR, and CHAVORE formats.

These cameras are all variations of a simple pinhole camera model so
they are processed with ASP in the ``Pinhole`` session instead of the
usual ``ISIS``. ASP only supports creating of point clouds. *The
\*-PC.tif is a raw point cloud with the first 3 channels being XYZ in
the rover site's coordinate frame*. We don't support the creation of
DEMs from these images and that is left as an exercise for the user.

An example of using ASP with MER data is included in the
``examples/MER`` directory (just type 'make' there).

PANCAM, NAVCAM, HAZCAM
~~~~~~~~~~~~~~~~~~~~~~

All of these cameras are processed the same way. We'll be showing 3D
processing of the front hazard cams. The only new things in the pipeline
is the new executable ``mer2camera`` along with the use of
``alignment-method epipolar``. This example is also provided in the MER
data example directory.

.. figure:: ../images/examples/mer/fh01_combined.png

   Example output possible with the front hazard cameras.

.. _commands-3:

Commands
^^^^^^^^

Download 2f194370083effap00p1214l0m1.img and
2f194370083effap00p1214r0m1.img from the PDS.

::

     ISIS> mer2camera 2f194370083effap00p1214l0m1.img
     ISIS> mer2camera 2f194370083effap00p1214r0m1.img
     ISIS> parallel_stereo 2f194370083effap00p1214l0m1.img     \
                           2f194370083effap00p1214r0m1.img     \
                           2f194370083effap00p1214l0m1.cahvore \
                           2f194370083effap00p1214r0m1.cahvore \
                    fh01/fh01

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices.

It is suggested to filter out points that are not triangulated well
because they are too close to robot's camera or are extremely far
away, using the ``parallel_stereo`` options::

    --universe-center camera --near-universe-radius 0.7 \
       --far-universe-radius 80.0

These are suggested as well::

    --alignment-method epipolar --force-use-entire-range

