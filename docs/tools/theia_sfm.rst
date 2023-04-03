.. _theia_sfm:

theia_sfm
---------

The ``theia_sfm`` program is a wrapper around the `Theia
<https://github.com/sweeneychris/TheiaSfM>`_ Structure-from-Motion (SfM)
software, to adapt it to work with ``rig_calibrator``
(:numref:`rig_calibrator`). See that page for an example.

ASP offers another wrapper around Theia, namely ``camera_solve`` 
(:numref:`camera_solve`), whose aim is to work with ASP's ``bundle_adjust``
program (:numref:`bundle_adjust`). 

As for ``rig_calibrator``, the images are stored in subdirectories
corresponding to sensor name, but it is not assumed that the image
files (without directory name) represent a timestamp. So, an image
name can be of the form: ``dir/sensor/string.png``.

Usage
~~~~~

::

    theia_sfm --rig_config <rig config> [images]  \
      [--theia_flags <flag file>] --out_dir <out dir>

Examples
~~~~~~~~

The input images can be specified as::

wildcards::

    theia_sfm --rig_config rig_input/rig_config.txt        \
      --images 'rig_input/cam1/*.tif rig_input/cam2/*.tif' \
      --out_dir rig_theia
 
in a list::

    ls rig_input/cam*/*.jpg > image_list.txt
    theia_sfm --rig_config rig_input/rig_config.txt        \
      --image_list image_list.txt                          \
      --out_dir rig_theia
 
on the command line::

    theia_sfm --rig_config rig_input/rig_config.txt        \
      rig_input/cam1/image1.png rig_input/cam2/image2.png  \
      --out_dir rig_theia

Visualization
~~~~~~~~~~~~~
The created camera poses can be visualized as::

    view_reconstruction --reconstruction out_dir/reconstruction-0

The .nvm file can be visualized with ``stereo_gui``
(:numref:`stereo_gui_nvm`).

Manipulating SfM solutions
~~~~~~~~~~~~~~~~~~~~~~~~~~

This tool produces an SfM solution with a name like
``out_dir/cameras.nvm``. Several of these can be merged into a larger
reconstruction with the ``sfm_merge`` (:numref:`sfm_merge`)
program. Portions can be extracted with ``sfm_submap``
(:numref:`sfm_submap`).
 
.. _theia_sfm_command_line:

Command-line options for theia_sfm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

--rig_config <string (default: "")>
    Rig configuration file.
--images <string (default: "")>
    Images, as individual wildcards. Example: 
    ``'dir/cam1/*tif dir/cam2/*tif'``.
--image_list <string (default: "")>
    Use the images from this list, instead of setting ``--images``.
    Images must be separated by a newline.
--theia_flags <string (default: "")>
    The flags to pass to Theia. By default, the file
    ``share/theia_flags.txt`` in the software distribution is used.
--out_dir <string (default: "")>
    The output directory (only the 'cameras.nvm' file in it is needed
    afterwards).
-h, --help
    Show this help message and exit.
