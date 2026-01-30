.. _stereo_sweep:

stereo_sweep
------------

This program runs :ref:`parallel_stereo` on small patches of mapprojected images
with different parameter combinations.

.. _stereo_sweep_example:

Example
~~~~~~~

::

    stereo_sweep                             \
      --param-sweep                          \
        "--corr-kernel 7,7 9,9
        --stereo-algorithm asp_sgm asp_mgm
        --subpixel-mode 1 9"                 \
      --param-sweep                          \
        "--corr-kernel 19,19 21,21
        --stereo-algorithm asp_bm
        --subpixel-mode 1 3"                 \
      left_map_image.tif right_map_image.tif \
      left_cam.xml right_cam.xml             \
      --dem dem.tif                          \
      --point2dem-params "--tr 4.0"          \
      --output-dir stereo_out

Note that the option ``--param-sweep`` is specified twice, with each resulting
in a set of runs. For the first one, there are two possible values for the
correlation kernel, two stereo algorithms, and two subpixel modes. This
results in a total of 8 different runs of ``parallel_stereo``. For the second
``--param-sweep``, there are two kernel sizes, one stereo algorithm, and two
subpixel modes, resulting in 4 runs.

Parameters with multiple values (like ``--corr-kernel 7,7``) use commas to
separate them, with no spaces. This is necessary so ``--param-sweep`` can be
parsed correctly. The commas are replaced with spaces when passing the
arguments to ``parallel_stereo``.

Workflow
~~~~~~~~

This program generates a file named ``lookup.csv`` in the output directory that
enumerates the run directories and corresponding parameter values.

The images are supposed to be mapprojected already (:numref:`mapproj-example`),
with the ``--dem`` option having the mapprojection DEM.

The :ref:`point2dem` command will be run on each output of ``parallel_stereo``.
Arguments to it can be passed via the ``--point2dem-params`` option. See :numref:`stereo_sweep_opts` for more details.

Any other options will be passed directly to ``parallel_stereo``.

.. _stereo_sweep_opts:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--param-sweep <string>
    Parameter sweep. Must be in quotes. Multiple such options can be specified.
    Each defines a different set of parameter combinations to test.
    See :numref:`stereo_sweep_example` for an example.
    
--dem <string>
    Input DEM for mapprojection.

--output-dir <string>
    Output directory.

--point2dem-params <string>
    Parameters to pass to point2dem. If ``--orthoimage`` (with no argument) is
    passed in, the needed ``L.tif`` will be passed to each created point2dem
    command.

--dry-run
    Print commands without executing them.

--help
    Display the help message and exit.
