.. _theia_sfm:

theia_sfm
---------

The ``theia_sfm`` program is a wrapper around the `Theia
<https://github.com/sweeneychris/TheiaSfM>`_ Structure-from-Motion (SfM)
software. It assumes that the there are are several sensors with each acquiring
images. 

The produced SfM solution can be passed on to ``rig_calibrator``
(:numref:`rig_calibrator`), but this program can be used without a rig.

ASP offers another wrapper around Theia, namely ``camera_solve`` 
(:numref:`camera_solve`), whose aim is to work with ASP's ``bundle_adjust``
program (:numref:`bundle_adjust`). 

.. _theia_naming_conv:

Naming convention
~~~~~~~~~~~~~~~~~

To distinguish which images are made with which sensor, either:

 - The image file name (without directory) should contain the sensor name.
 - The name of the immediate directory having the image should be the sensor name.
 - A list should be provided, in which each line has the image name, sensor
   name and image timestamp, separated by spaces.

Examples::

  my_images/image1_cam1.png  

  my_images/cam1/image1.png

where ``cam1`` is the name of the sensor that acquired this image. 

If a list is provided, each line should be of the form::

  my_images/image1.png cam1 <timestamp>

This list should be passed in via ``--image_sensor_list``. The timestamp is not
used but is there for compatibility with ``rig_calibrator``
(:numref:`rig_data_conv`).

All images names (without directory path) must be unique.

Configuration
~~~~~~~~~~~~~

The configuration file contains the intrinsics for each sensor. Is the same as
for ``rig_calibrator`` (:numref:`rig_config`). This program does not use the
transforms between sensors, and there is no concept of reference sensor.

See `Theia's supported camera model types <http://theia-sfm.org/cameras.html>`_.
In the rig configuration those are named ``no_distortion``, ``radtan``,
``fisheye`` and ``fov``.

 .. _theia_sfm_config:

Flags file
~~~~~~~~~~

This program  will use the Theia flags file from ``share/theia_flags.txt`` in
the software distribution, which can be copied to a new name, edited, and passed
to ``theia_sfm`` via ``--theia_flags``.

As an example, for tricky configurations, setting::

  --feature_density=DENSE

in the flags file can create a lot more interest points matches.

Examples
~~~~~~~~

A step-by-step-example is in :numref:`rig_msl`.

Usage
~~~~~

::

    theia_sfm --rig-config <rig config> [images]  \
      [--theia-flags <flag file>] --out-dir <out dir>

The input images can be specified as wildcards::

    theia_sfm --rig-config rig_input/rig_config.txt        \
      --images 'rig_input/cam1/*.tif rig_input/cam2/*.tif' \
      --out-dir rig_theia
 
in a list::

    ls rig_input/cam*/*.jpg > image_list.txt
    theia_sfm --rig-config rig_input/rig_config.txt        \
      --image-list image_list.txt                          \
      --out-dir rig_theia
 
on the command line::

    theia_sfm --rig-config rig_input/rig_config.txt        \
      rig_input/cam1/image1.png rig_input/cam2/image2.png  \
      --out-dir rig_theia


Use ``--image_sensor_list`` instead of ``--list`` if the sensor names
are not part of image names (:numref:`theia_naming_conv`).

Output files
~~~~~~~~~~~~

This tool produces an SfM solution with a name like ``out_dir/cameras.nvm``. The
optical offsets per image are in ``out_dir/cameras_offsets.txt``.  
A reconstruction in binary format is saved to ``out_dir/reconstruction-0``.

The nvm file can be passed in to ``bundle_adjust`` (:numref:`ba_nvm`) and
``rig_calibrator`` (:numref:`rig_calibrator`).

Visualization
~~~~~~~~~~~~~

The created camera poses can be visualized as::

    view_reconstruction --reconstruction out_dir/reconstruction-0

See this program's manual in :numref:`view_reconstruction`.

The interest point matches in the .nvm file can be inspected with 
``stereo_gui`` (:numref:`stereo_gui_nvm`). This will show the images in the
random order produced by Theia. The ``rig_calibrator`` program
(:numref:`rig_calibrator`), which can be used as the next step, will order these
lexicographically.

Manipulating SfM solutions
~~~~~~~~~~~~~~~~~~~~~~~~~~

Several produced solutions can be merged into a larger reconstruction with the
``sfm_merge`` (:numref:`sfm_merge`) program. Portions can be extracted with
``sfm_submap`` (:numref:`sfm_submap`).
 
.. _theia_sfm_command_line:

Command-line options for theia_sfm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

--rig-config <string (default: "")>
    Rig configuration file.
--images <string (default: "")>
    Images, as individual wildcards. Example: 
    ``'dir/cam1/*tif dir/cam2/*tif'``.
--image-list <string (default: "")>
    Use the images from this list, instead of setting ``--images``.
    Images must be separated by a newline.
--image-sensor-list <string (default: "")>
    An alternative way of listing the input images, when the sensor name is
    specified separately in the same file (:numref:`theia_naming_conv`).
--theia-flags <string (default: "")>
    The flags to pass to Theia. By default, the file
    ``share/theia_flags.txt`` in the software distribution is used.
--out-dir <string (default: "")>
    The output directory.
-h, --help
    Show this help message and exit.
