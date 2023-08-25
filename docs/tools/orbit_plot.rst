.. _orbit_plot:

orbit_plot.py
-------------

The ``orbit_plot.py`` program is a Python script that takes an input one or more
orbital sequences of cameras, and plots the camera orientation as it changes
along the orbit. Each orientation is decomposed into roll, pitch, and yaw
components, that are plotted separately.

If a second set of orbital sequences exists, for example, if the camera
orientations are later optimized, with ``bundle_adjust``
(:numref:`bundle_adjust`) or ``jitter_solve`` (:numref:`jitter_solve`), this
tool can overlay the two sets.

Each orbital sequence consists of several frame (pinhole) cameras, in .tsai
(:numref:`pinholemodels`) or CSM (:numref:`csm_frame`) format, or it can be a
single linescan camera in the CSM model state format (:numref:`csm_state`).  

At some point this tool will also plot the camera positions.

Examples
~~~~~~~~

Plot one dataset
^^^^^^^^^^^^^^^^

The ``orbit_plot`` conda environment should be first installed 
as described in :numref:`orbit_plot_dependencies`.

We assume that ASP's ``bin`` directory is in the path, otherwise
the full path to this script must be specified below.

Plot a single set of cameras along a given orbit::

    ~/miniconda3/envs/orbit_plot/bin/python         \
      $(which orbit_plot.py)                        \
      --dataset  dataset1/                          \
      --label    dataset1                           \
      --orbit-id pinhole-fwd

We assume that the cameras are in the directory ``dataset1/``, and their names
in that directory start with ``pinhole-fwd``. 

The slash (``/``) at the end of the directory name is important, as the two
strings above will be concatenated to find the camera names.

The rest of a camera name can be any string ending in ``.tsai`` or ``.json``.
Hence, only the cameras satisfying this convention will be read.

Plot two datasets
^^^^^^^^^^^^^^^^^

We consider two camera datasets, with the camera names starting with::

    dataset1/pinhole-fwd
    dataset2/run-pinhole-fwd  

The naming convention used above is suggestive of the first dataset being a set
of input cameras, while the second being created from the first using
``bundle_adjust`` or ``jitter_solve``. The string ``pinhole-fwd`` is the orbit
id, and suggests that these cameras are of pinhole type and were created by a
forward-looking camera.

::

    ~/miniconda3/envs/orbit_plot/bin/python         \
      $(which orbit_plot.py)                        \
      --dataset dataset1/,dataset2/run-             \
      --orbit-id pinhole-fwd                        \
      --label Orig,Opt

Notice how above the shared orbit id is specified separately from the dataset
names.

These two datasets will be plotted on top of each other, in red and blue, respectively.

Plot two orbital groups
^^^^^^^^^^^^^^^^^^^^^^^

Here, in addition to a group of pinhole cameras looking forward, before and after bundle adjustment, we also consider a linescan camera looking down, before and after solving for jitter. The linescan camera will have many position and orientation samples.

The only change in the command above is that the orbit id now has the additional value ``linescan-nadir``, so the plot command becomes::

    ~/miniconda3/envs/orbit_plot/bin/python         \
      $(which orbit_plot.py)                        \
      --dataset dataset1/,dataset2/run-             \
      --orbit-id pinhole-fwd,linescan-nadir         \
      --label Orig,Opt

The cameras before optimization will be in directory ``dataset1/``, with the
pinhole camera names starting with ``pinhole-fwd``, and the linescan camera
name starting with ``linescan-nadir``. 

The cameras after optimization will start with ``dataset2/run-``, followed
again by the orbit id.

The resulting plot will have two rows, with the first one showing the pinhole
cameras, before and after optimization, and the second one having the
orientation samples in the linescan camera.

.. _orbit_plot_dependencies:

Dependencies
~~~~~~~~~~~~

This tool needs Python 3 and some additional Python packages to be installed with 
``conda``. 

Conda can be obtained from 

    https://docs.conda.io/en/latest/miniconda.html

Run::

    ./Miniconda3-latest-Linux-x86_64.sh

on Linux, and the appropriate version on OSX (this script needs to be
made executable first). Use the suggested::

    $HOME/miniconda3

directory for installation. 

Activate conda. The needed packages can be installed, for example,
as follows:

::
    
    conda create -n orbit_plot numpy scipy pyproj matplotlib

Command-line options for orbit_plot.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display this help message.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN

