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
 
Example: Plot a single orbital sequence
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here we will consider synthetic cameras, created with ``sat_sim``
(:numref:`sat_sim`).

::

    ~/miniconda3/envs/orbit_plot/bin/python         \
      $(which orbit_plot.py)                        \
      --dataset sim_fn_jitter0.0/,sim_fn_jitter2.0/ \
      --orbit-id pinhole-fwd                        \
      --label Orig,Adjust                           \
      --trim-ratio 0.5

Here it is assumed that ASP's ``bin`` directory is in the path, otherwise
the full path to this Python script must be specified above.

The ``orbit_plot`` conda environment is installed as described below.

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
    conda create -n orbit_plot numpy=1.23.5 scipy=1.9.3 \
      pyproj=3.4.0 matplotlib=3.6.2 -y

Command-line options for orbit_plot.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


-h, --help
    Display this help message.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN

