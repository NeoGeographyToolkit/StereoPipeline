.. _theia_sfm:

theia_sfm
---------

The ``theia_sfm`` program is a wrapper around the `Theia
<https://github.com/sweeneychris/TheiaSfM>`_ structure-from-motion
software, to adapt it to work with ``rig_calibrator``
(:numref:`rig_calibrator`). See that page for an example.

ASP offers another wrapper around Theia, namely ``camera_solve`` 
(:numref:`camera_solve`), whose aim is to work with ASP's ``bundle_adjust``
program. 

Usage:: 

    theia_sfm --rig_config <rig config> --images <wildcards> \
      [--theia_flags <flag file>] --out_dir <out dir>

.. _theia_sfm_command_line:

Command-line options for theia_sfm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

--rig_config <string (default: "")>
    Rig configuration file.
--images <string (default: "")>
    Images, as individual wildcards. Example: 
    ``'dir/cam1/*tif dir/cam2/*tif'``.
--theia_flags <string (default: "")>
    The flags to pass to Theia. By default, the file
    ``share/theia_flags.txt`` in the software distribution is used.
--out_dir <string (default: "")>
    The output directory (only the 'cameras.nvm' file in it is needed
    afterwards).
-h, --help
    Show this help message and exit.
