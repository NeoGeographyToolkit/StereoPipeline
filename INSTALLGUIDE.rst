Installation
============

Precompiled binaries are available for the stable releases and the
current development build. Stereo Pipeline can also be compiled from
source, but this is not recommended (:numref:`building_asp`).

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

    tar xvf StereoPipeline-3.4.0-2024-06-19-x86_64-Linux.tar.bz2
    ./StereoPipeline-3.4.0-2024-06-19-x86_64-Linux/bin/stereo --help

The result of the last command should be a help message.

To permanently add the ASP executable subdirectory to your PATH,
add the following line to your shell configuration (e.g.,
``~/.bashrc``), replacing ``/path/to/StereoPipeline/bin`` with the
location on your filesystem::

    export PATH=${PATH}:/path/to/StereoPipeline/bin

The latest additions are documented in :numref:`news`.

MacOS
~~~~~

ASP builds are produced for MacOS, with the Intel processor. ASP was verified to
work on the Arm M1/M2 processor using Rosetta 2.

The installation steps are the same as for Linux. It is important to 
note that:

- An error may be shown about not being able to verify the developers. That can
  be overridden in the Privacy & Security settings of the system.
 
- Running the programs the first time will be slow, as Rosetta will
  translate them to the native architecture. Subsequent runs will be
  faster.

Windows
~~~~~~~

ASP does not offer Windows binaries. However, the Linux build can be run on
Windows using the `Windows Subsystem for Linux
<https://learn.microsoft.com/en-us/windows/wsl/install>`_ (WSL). Once a recent
Linux distribution is installed and verified to work, the installation steps are
the same as for Linux.

Conda and docker
----------------

The latest ASP release (3.4.0) is available only as binaries. The 3.3.0 release
can be installed via conda (:numref:`conda_intro`).

ASP can be installed with Docker (`instructions
<https://github.com/uw-cryo/asp-binder>`_).

Post-installation
-----------------
 
The next steps depend on whether it is desired to process planetary (non-Earth),
Earth, or aerial images.
 
.. _planetary_images:

Planetary images
~~~~~~~~~~~~~~~~

If you plan to process images from NASA's spacecraft that are
exploring other planets, you will need to install ISIS and its data.
Summary of the steps:

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

#. Extract Stereo Pipeline::

     tar xvf StereoPipeline-<VERSION>-<ARCH>-<OS>.tar.bz2

#. Add Stereo Pipeline to your path:

   - bash: ``export PATH="/path/to/StereoPipeline/bin:${PATH}"``
   - csh:  ``setenv PATH "/path/to/StereoPipeline/bin:${PATH}"``

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

.. _conda_intro:

Fetching pre-compiled ASP with conda
------------------------------------

*The latest ASP release (3.4.0, June 19, 2024) is available only as binaries*.
See :numref:`release` for how to fetch them.

The reason is that the latest ISIS conda release (8.0.3) has dependencies that
are incompatible with what ASP expects. This will be rectified in the next ASP
and ISIS releases.

See :numref:`news` for what changed between releases.

The latest ISIS can be installed via conda in a separate location
(:numref:`planetary_images`). ASP's internal libraries are based on the
ISIS source code (circa June 15, 2024), after the ISIS 8.0.3 release.

The ASP 3.3.0 release (August 16, 2023) can be installed via conda, together 
with ISIS 8.0.0.

ASP conda packages do not exist for ARM64 Mac (M1/M2). See
:numref:`release` for how this and platforms may be handled.

To install ``conda``, see:

    https://docs.conda.io/en/latest/miniconda.html

Make the fetched installation file executable and run it, such as::

    chmod u+x ./Miniconda3-latest-Linux-x86_64.sh
    ./Miniconda3-latest-Linux-x86_64.sh

on Linux, and analogously on OSX. Use the suggested::

    $HOME/miniconda3

directory for installation. 

Create an environment for ASP as::

    conda create -n asp
    conda activate asp

Add relevant channels::

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
    - defaults

*Not having the channels in this order is likely to result in failure to install
ASP.*

Install ASP with the command::

    conda install                 \
     -c nasa-ames-stereo-pipeline \
     -c usgs-astrogeology         \
     -c conda-forge               \
     stereo-pipeline==3.3.0

This will install ASP 3.3.0 together with ISIS 8.0.0. Note that the *latest
build* (:numref:`release`) may have more features and fixes than
this official release.

Alternatively, consider using ``mamba`` instead of ``conda``. It is
must faster though it is not always guaranteed to work. 

Run::

  conda activate asp
  
to activate the environment in any new shell. This should put the ASP binaries
in the path, and will also initialize the ``PROJ_DATA`` environment variable
that is needed for the PROJ library. Or, set the PATH variable as in
:numref:`release`.
  
Post-installation
~~~~~~~~~~~~~~~~~

Check that the ``stereo`` command can be found as::

    which stereo

If using ISIS, the environmental variable ISISROOT should be set to
point to this distribution, such as::

    export ISISROOT=$HOME/miniconda3/envs/asp

If you are working with planetary data, you need to complete
the ISIS installation steps from this new ``asp`` conda environment.
Your new ``asp`` environment already has the base ISIS software
installed, but you must run the script which sets the ISIS environment
variables, and also install the appropriate ISIS data files (if you also
have a separate ISIS conda environment, you can use the set-up script
to point the ``asp`` conda environment's ``ISISDATA`` environment
variable to your existing data area).  

For more information see the `ISIS installation instructions
<https://github.com/USGS-Astrogeology/ISIS3>`_ and :numref:`planetary_images`. 

Using a precise list of packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*This section applies to the prior ASP 3.3.0 release* (:numref:`conda_intro`). 

Some variability may exist in the precise dependencies fetched by
conda. For the record, the full environment for this release can be
found as a set of .yaml files in the ``conda`` subdirectory of the
Stereo Pipeline GitHub repository. So, alternatively, the installation
can happen as::

    conda env create -n asp -f asp_3.3.0_linux_env.yaml

or::

    conda env create -n asp -f asp_3.3.0_osx_env.yaml

depending on your platform. Then invoke, as earlier::

    conda activate asp

For how to build ASP, without and with conda, see
:numref:`build_from_source` and :numref:`conda_build`.
