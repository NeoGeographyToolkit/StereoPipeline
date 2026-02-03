.. _orbital_rig:

Orbital rig
-----------

This example shows how to produce synthetic images and cameras modeling an
orbital rig with two frame camera sensors, and how to refine the rig parameters,
camera intrinsics, and camera poses. A DEM constraint is to be added.

Input DEM and orthoimage
~~~~~~~~~~~~~~~~~~~~~~~~

We will create the synthetic data for this example with :ref:`sat_sim`. This program
needs as input a DEM and orthoimage of a region of interest. 

To prepare such data, we started with the free ASTER dataset::

  AST_L1A_00404012022185436_20250920182851.hdf

around the San Luis Reservoir in Northern California. A DEM was created with the
workflow in :numref:`aster`. The orthoimage was produced with :ref:`mapproject`,
at the nominal resolution of 15 m / pixel. These reflect the ground sample
distance of the ASTER images.

Synthetic images
~~~~~~~~~~~~~~~~

The synthetic images and cameras were created with ``sat_sim``. This program
can simulate an orbital rig (:numref:`sat_sim_rig`).

The rig was chosen to have one left and one right frame camera, named ``left`` and
``right``. The initial rig configuration is created as in :numref:`msl_init_rig`.
The sensor dimensions were set to 1000 x 1000 pixels, with the principal point at 
the center of the image. No lens distortion was assumed.

The satellite height is set to 700 km. The chosen focal length is 35000 pixels,
which results in an estimated ground sample distance (GSD) of about 20 meters
(GSD is the satellite height divided by the focal length). All these are in line
with what is known about the ASTER instrument.

The ``sat_sim`` command for the nadir images was::

    sat_sim                           \
        --dem aster_dem.tif           \
        --ortho aster_ortho.tif       \
        --rig-config aster_rig.txt    \
        --rig-sensor-ground-offsets   \
        -0.01,0,-4000,0,0.01,0,4000,0 \
        --first 1300 1200 700000      \
        --last  1300 1500 700000      \
        --first-ground-pos 1300 1200  \
        --last-ground-pos  1300 1500  \
        --roll 0 --pitch 0 --yaw 0    \
        --num 3                       \
        --velocity 7500               \
        -o sat_sim/run-nadir

The chosen value of the ``--rig-sensor-ground-offsets`` option places the left
and right sensor centers offset by 0.01 m to the left and right from the rig
center, and the footprints on the ground are separated by 8000 m in the
East-West direction. The satellite itself follows a North-South orbit.

The rig configuration that incorporates these controls is saved as::

    sat_sim/run-nadir-rig_config.txt
    
This has, in addition to the intrinsics as in the input rig, also the
relationship between the rig sensors, in the ``ref_to_sensor_transform`` field.
More details are in :numref:`sat_sim_rig_adjust`.

A similar command is run to create forward-looking images. The value of ``--pitch``
is set to 30 degrees. The output prefix is set to ``sat_sim/run-fwd``. 

The produced images will have names such as::

    sat_sim/run-nadir-0010000.418204756-left.tif
    sat_sim/run-fwd-0009939.411652856-right.tif

following the naming convention in :numref:`rig_data_conv`. The components of
these file names are the output prefix, the time stamp, and the sensor name.
Modeling of time is described in :numref:`sat_sim_time`. The options for this
program are documented in :ref:`sat_sim_options`.

.. figure:: ../images/orbital_rig.png
   :name: orbital_rig_fig
   :alt:  Orbital rig example

   A sample left and right image as produced by the rig (after mapprojection).
   The images have notable overlap. These show some fields and mountain
   foothills in California's Central Valley.

Interest point matches
~~~~~~~~~~~~~~~~~~~~~~

The rig calibrator program expects the camera poses and the interest point
matches between images to be stored in an NVM file (a format for
Structure-from-Motion applications). See :numref:`ba_nvm`.

Since we have 12 input images, and each has to be matched against every other
one, the :numref:`parallel_bundle_adjust` program is run to ensure
parallelization::

    parallel_bundle_adjust       \
      --ip-per-image 10000       \
      --output-cnet-type nvm     \
      sat_sim/*{left,right}.tif  \
      sat_sim/*{left,right}.tsai \
      --camera-weight 1.0        \
      --tri-weight 1.0           \
      --num-iterations 100       \
      -o ba/run

Rig calibration
~~~~~~~~~~~~~~~

Run :ref:`rig_calibrator`::

    rig_calibrator                                  \
      --rig-config sat_sim/run-nadir-rig_config.txt \
      --nvm ba/run.nvm                              \
      --camera-poses-to-float "left right"          \
      --intrinsics-to-float                         \
      "left:focal_length right:focal_length"        \
      --camera-position-weight 1.0                  \
      --tri-weight 1.0                              \
      --save-pinhole-cameras                        \
      --num-iterations 100                          \
      --out-dir rig

Here the data starts perfect, so very few changes are expected. The produced
pinhole cameras (:numref:`pinholemodels`) saved in the output ``rig`` directory
(with the option ``--save-pinhole-cameras``) should be very similar to the
inputs in the ``sat_sim`` directory.

