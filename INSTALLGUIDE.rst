Installation
============

Precompiled binaries are available for the stable releases and the current
development build. Conda packages exist for the stable versions
(:numref:`conda_intro`).

.. _release:

Precompiled binaries
--------------------

Linux
~~~~~
 
Choose either the latest build (recommended) or a stable release
from:

- `<https://github.com/NeoGeographyToolkit/StereoPipeline/releases>`_

No installation steps or administrative rights are necessary.  Extract
the archive, and run the executables in the ``bin`` subdirectory as::

    tar xvf StereoPipeline-3.6.0-2025-12-26-x86_64-Linux.tar.bz2
    ./StereoPipeline-3.6.0-2025-12-26-x86_64-Linux/bin/stereo --help

The result of the last command should be a help message.

To permanently add the ASP executable subdirectory to your PATH, add to your
shell configuration (e.g., ``~/.bashrc``), a line similar to::

    export PATH="${PATH}":"/path/to/StereoPipeline/bin"

Then, run ``source ~/.bashrc`` (or open a new terminal) for the changes to take
effect.

The latest additions are documented in :numref:`news`.

MacOS
~~~~~

ASP is available for the Mac Intel and Mac Arm architectures. The latter is
somewhat experimental but was shown to work well in testing. The Mac Arm package
has all ASP logic except the minor ``libelas`` stereo algorithm
(:numref:`libelas`).

The installation steps are the same as for Linux. It is important to 
note that:

- An error may be shown about not being able to verify the developers. That can
  be overridden in the Privacy & Security settings of the system. Consider using 
  instead the conda-based installation (:numref:`conda_intro`), which should 
  not have this issue.
 
- Running the Intel build the first time will be slow, as Rosetta will
  translate the instructions to the native architecture. Subsequent runs will be
  faster.

Windows
~~~~~~~

ASP does not offer Windows binaries. However, the Linux build can be run on
Windows using the `Windows Subsystem for Linux
<https://learn.microsoft.com/en-us/windows/wsl/install>`_ (WSL). Once a recent
Linux distribution is installed and verified to work, the installation steps are
the same as for Linux.

.. _conda_intro:

Conda-based installation
------------------------

The ASP 3.6.0 release (December 26, 2025) can be installed via conda, together with
ISIS 9.0.0 (:numref:`planetary_images`) for Linux, Mac Intel, and Mac Arm.

The Mac Arm release is experimental but was tested rather thoroughly. Since USGS
did not release an ISIS version for Mac Arm (as of 12/2025), this is shipped
with an unofficial ISIS Arm conda package, hosted on the
``nasa-ames-stereo-pipeline`` channel. This one lacks the Kakadu JPEG2000
library support. Consider using the Intel release under Rosetta 2 for
mission-critical work.

To install ``conda``, see:

    https://docs.conda.io/en/latest/miniconda.html

Make the fetched installation file executable and run it, such as::

    chmod u+x ./Miniconda3-latest-Linux-x86_64.sh
    ./Miniconda3-latest-Linux-x86_64.sh

on Linux, and analogously on OSX. Use the suggested::

    $HOME/miniconda3

directory for installation. 

Configure the conda channels::

    conda config --env --add channels conda-forge
    conda config --env --add channels usgs-astrogeology
    conda config --env --add channels nasa-ames-stereo-pipeline

Do not skip doing each of these three, even if you think you already
have some of these channels.

Run::

    conda config --show channels

to ensure that the order of channels is::

    - nasa-ames-stereo-pipeline
    - usgs-astrogeology
    - conda-forge

*Not having the channels in this order is likely to result in failure to install
ASP.* Do not use the ``defaults`` channel.

Install ASP with the commands::

    conda config --set channel_priority flexible
    conda create -n asp            \
      -c nasa-ames-stereo-pipeline \
      -c usgs-astrogeology         \
      -c conda-forge               \
      -c defaults                  \
      stereo-pipeline=3.6.0

This will create a new environment named ``asp`` and install ASP 3.6.0 together 
with ISIS 9.0.0 and all other dependencies.   

The priority setting is set to ``flexible``, as otherwise conda can get confused 
if the same package (even with old versions) exists in more than one channel.
  
Note that the *latest build* (:numref:`release`) may have more features and
fixes than this official release.

Environment setup
~~~~~~~~~~~~~~~~~

Run::

  conda activate asp
  
and set::

    export ISISROOT=$CONDA_PREFIX

in any new shell. These should put the ASP binaries in the path, and will also
initialize various environmental variables, including ``ISISROOT`` and  
``PROJ_DATA``.

Check if the ``stereo`` command is found by running::

    which stereo

When working with planetary images with ISIS, the ``ISISDATA`` environmental
variable also needs to be set (:numref:`planetary_images`). For more information
see the `ISIS installation instructions
<https://github.com/USGS-Astrogeology/ISIS3>`_.

Alternative approaches
~~~~~~~~~~~~~~~~~~~~~~
  
Consider using ``mamba`` instead of ``conda`` for the installation, as it is
much faster. (Note that recent ``conda`` distributions default to using the
``mamba`` solver.)

ASP can be installed with Docker (`instructions
<https://github.com/uw-cryo/asp-binder>`_).

ASP can be built form source (:numref:`building_asp`).

Post-installation
-----------------
 
The next steps depend on whether it is desired to process planetary (non-Earth),
Earth, or aerial images.
 
.. _planetary_images:

Planetary images
~~~~~~~~~~~~~~~~

To process images from NASA's spacecraft that are exploring other planets,
install ISIS and its data. Summary of the steps:

#. Fetch ISIS binaries and install, following
   https://github.com/DOI-USGS/ISIS3#installation

#. Fetch ISIS data, as detailed at
   https://github.com/DOI-USGS/ISIS3#the-isis-data-area

#. Add the ISIS executables to your path:

   - bash: ``export PATH="/path/to/isis/bin:${PATH}"``
   - csh:  ``setenv PATH "/path/to/isis/bin:${PATH}"``

#. Set the ``ISISDATA`` environmental variable to point to where your
   ISIS data was downloaded, per the installation link above. For
   example, in the ``bash`` shell, this is done as follows::
  
     export ISISDATA="/path/to/isisdata"
   
   Check that you have the directory ``$ISISDATA/base``.

#. Install Stereo Pipeline and set the ``PATH`` variable as above.

#. Try it out. See :numref:`lronac_csm` for a quick Lunar example which does not
   require installing ISIS or it supporting data as above,
   :numref:`moc_tutorial` for an example using Mars images and ISIS data, and
   many other examples in :numref:`examples`.

Earth images
~~~~~~~~~~~~

Processing Earth images is described in the data processing tutorial in
:numref:`dg_tutorial`. See also examples for ASTER (:numref:`aster`), Pleiades
(:numref:`pleiades`), SkySat (:numref:`skysat`), and many more in
:numref:`examples`.

Aerial and historical images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Fetch the software as above. Processing images without accurate camera
pose information is described in :numref:`sfm`. See also examples for 
declassified satellite images in :numref:`kh4`.

.. _system_rec:

System requirements
-------------------

To run ASP, a computer cluster sharing storage and connected via ssh is needed
(:numref:`pbs_slurm`).

As a rule of thumb, for images on the order of 20,000 x 20,000 pixels, a machine
with 40 GB of RAM 16 cores could likely produce a terrain model in 4 - 20 hours.
There is a lot of uncertainty here, and much depends on the choice of the stereo
algorithm (:numref:`stereo_alg_overview`), and if mapprojection is employed
(:numref:`mapproj-example`).

1 TB of storage or more is suggested.

Common errors
-------------

Here are some errors you might see, and what it could mean. Treat these
as templates for problems. In practice, the error messages might be
slightly different.

::

    Error: **ERROR** Unable to initialize camera model in Camera Factory.

    **PROGRAMMER ERROR** Unable to create a shape model from 
      given target and pvl.

    **I/O ERROR** Unable to open [$ISISDATA/<Some/Path/Here>].
    Stereo step 0: Preprocessing failed

You need to set up your ISIS environment or manually set the correct
location for ``ISISDATA`` (:numref:`planetary_images`).

::

    bash: stereo: command not found

You need to add the ``bin`` directory of your deployed Stereo Pipeline
installation to the environmental variable ``PATH``
(:numref:`planetary_images`).

::

    /bin/sh: camrange: command not found

You need to to add the ``bin`` directory of your ISIS installation to your path (:numref:`planetary_images`).

::

    Cache size (500 MB) is larger than the requested maximum cache size

Consider increasing ``--cache-size-mb`` for your program.
This also may be a sign of large input TIF images being stored
in blocks as tall or as wide as the image. The storage scheme of
an image can be examined with the ``gdalinfo -stats`` command,
and an image can be rewritten with square blocks using the command::

    gdal_translate -co compress=lzw -co TILED=yes -co INTERLEAVE=BAND \
      -co BLOCKXSIZE=256 -co BLOCKYSIZE=256 input.tif output.tif

If the new images are used instead, that warning should go away and
the processing time should go down. Both ``gdalinfo`` and
``gdal_translate`` are included with ASP.
