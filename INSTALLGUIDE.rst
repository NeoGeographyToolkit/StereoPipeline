Installation
============

Precompiled binaries are available for the stable releases and the
current development build.  Stereo Pipeline can also be compiled from
source, but this is not recommended (:numref:`building_asp`).

.. _precompiled_binaries:

Precompiled binaries (Linux and macOS)
--------------------------------------

Download the appropriate distribution for your operating system,
choosing either the latest build (recommended) or a stable release
from:

- `<https://github.com/NeoGeographyToolkit/StereoPipeline/releases>`_

No installation steps or administrative rights are necessary.  Extract
the archive, and run the executables in the ``bin`` subdirectory as::

    tar -xvf StereoPipeline-3.2.0-2022-12-30-x86_64-Linux.tar.bz2
    ./StereoPipeline-3.2.0-2022-12-30-x86_64-Linux/bin/stereo --help

The result of the last command should be a help message.

ASP builds are for Linux and OSX (with the Intel processor). ASP was verified to work under Microsoft Windows using the Windows Subsystem for Linux, and under the OSX M1 processor using Rosetta.

See the NEWS file (:numref:`news`) for the most recent additions.

The stable releases are also available via conda
(:numref:`conda_intro`).

To permanently add the ASP executable subdirectory to your PATH, you
can add the following line to your shell configuration (e.g.,
``~/.bashrc``), replacing ``/path/to/StereoPipeline/bin`` with the
location on your filesystem::

    export PATH=${PATH}:/path/to/StereoPipeline/bin

.. _planetary_images:

Planetary images
~~~~~~~~~~~~~~~~

If you plan to process images from NASA's spacecraft that are
exploring other planets, you will probably need to have :term:`ISIS`
installed.  A full ISIS installation is not required for operation of
the main Stereo Pipeline programs (only the ISIS data directory is
needed), but it is required for certain preprocessing steps before
Stereo Pipeline programs are run for planetary data.  If you only want
to process terrestrial images, skip to :numref:`dg_tutorial`.

To perform pre-processing (radiometric calibration, ephemeris
processing, etc.), of non-terrestrial images prior to running Stereo
Pipeline, you will need to install :term:`ISIS`.  Just as with our 
binaries, you can use the ISIS binaries as-is.

ASP has its own self-contained version of the ISIS libraries, and on
occasion it is behind the latest ISIS (see the release notes in
:numref:`news` for the ISIS version ASP has). You should be able to
use a newer version of ISIS to prepare the images and cameras and have
those work with the version of ISIS shipped with ASP, assuming no
major changes have taken place in the data formats or camera models by
the ISIS developers. At the very least, you should be able to install
the older standalone version of ISIS that ASP uses if a failure is
found.  To do so, follow the ISIS installation instructions, but
create a new conda environment (not the one with your current ISIS),
and right before you would run ``conda install isis``, run ``conda
search isis`` to find all of the versions of ISIS available for
installation.  For example, if you wanted to install ISIS 7.1.0, and
it is available in the ``conda search isis`` listing, you can run
``conda install isis=7.1.0`` and then follow the remainder of the ISIS
installation instructions.

In closing, running the Stereo Pipeline executables only requires
that you have downloaded the ISIS secondary data and have
appropriately set the ``ISISDATA`` environment variable. This is
normally performed for the user by starting up the conda ISIS 
environment.

.. _isis_start:

Quick start for ISIS users
~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the packaged ASP tarball
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Fetch Stereo Pipeline from
   https://github.com/NeoGeographyToolkit/StereoPipeline/releases

#. Fetch ISIS binaries and install, following
   https://github.com/USGS-Astrogeology/ISIS3#installation

#. Fetch ISIS data, as detailed at
   https://github.com/USGS-Astrogeology/ISIS3#the-isis-data-area

#. Set the ``ISISDATA`` environmental variable to point to where your
   ISIS data was downloaded, per the installation link above. For
   example, in the ``bash`` shell, this is done as follows::
  
     export ISISDATA="/path/to/isisdata"
   
   Check that you have the directory ``$ISISDATA/base``.

#. Untar Stereo Pipeline::

     tar xzvf StereoPipeline-<VERSION>-<ARCH>-<OS>.tar.gz

#. Add Stereo Pipeline to your path (optional):

   - bash: ``export PATH="/path/to/StereoPipeline/bin:${PATH}"``
   - csh: ``setenv PATH "/path/to/StereoPipeline/bin:${PATH}"``

#. Try it out. See :numref:`lronac_csm` for a quick Lunar example
   which does not require installing ISIS or it supporting data as
   above, and :numref:`moc_tutorial` for an example using Mars images
   and ISIS data.

Installing ASP and ISIS in the same conda environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is discussed further down. 

Quick start for Digital Globe users
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#. Fetch Stereo Pipeline from
   https://github.com/NeoGeographyToolkit/StereoPipeline/releases

#. Untar Stereo Pipeline::

     tar xzvf StereoPipeline-<VERSION>-<ARCH>-<OS>.tar.gz

#. Try it out: Processing Earth images is described in the data processing
   tutorial in :numref:`dg_tutorial`.


Quick start for aerial and historical images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Fetch the software as above. Processing images without accurate camera
pose information is described in :numref:`sfm`.


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
location for ``ISISDATA`` (:numref:`isis_start`).

::

    bash: stereo: command not found

You need to add the ``bin`` directory of your deployed Stereo Pipeline
installation to the environmental variable ``PATH``
(:numref:`isis_start`).

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

ASP's official releases can be fetched with ``conda``, for Linux and OSX with the Intel processor. See (:numref:`precompiled_binaries`) for how other platforms may be handled.

The latest conda release is version 3.2.0, published on December 30, 2022. See
:numref:`news` for what changed since then.  The newer functionality
is available only with the daily build (:numref:`precompiled_binaries`).

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

It is possible that you may already have some of these channels in a
global ``~/.condarc`` file, and you may be tempted to just run the
final add channels command.  If you aren't familiar with conda channel
management, this may have unintended consequences.  Please inspect the
order of the output of the ``--show channels`` command carefully, if
it is not exactly like the above, you can either edit the
``$CONDA_PREFIX/.condarc`` file, or delete it completely, and then run
each of the three ``conda config --env -add channels`` commands
exactly as shown.

You can use the ``--prepend channels`` argument to ``conda config``
but unless you want to add the ``nasa-ames-stereo-pipeline`` channel to
all of your conda environments (which you probably don't), please
make sure you have activated your *asp* environment and make sure to use
the ``--env`` argument to contain the change to the current environment
and not all environments.

Install ASP with the command::

    conda install stereo-pipeline==3.2.0

This will install ASP 3.2.0 together with ISIS 7.1.0. Note that the
latest build (see above) may have more fixes or features than this
official release.

If using ISIS, the environmental variable ISISROOT should be set to
point to this distribution, such as::

    export ISISROOT=$HOME/miniconda3/envs/asp

Check that the ``stereo`` command can be found as::

    which stereo

Some variability may exist in the precise dependencies fetched by
conda. For the record, the full environment for this release can be
found as a set of .yaml files in the ``conda`` subdirectory of the
Stereo Pipeline GitHub repository. So, alternatively, the installation
can happen as::

    conda env create -f asp_3.2.0_linux_env.yaml

or::

    conda env create -f asp_3.2.0_osx_env.yaml

depending on your platform. Then invoke, as earlier::

    conda activate asp

Finally, if you are working with planetary data, you need to complete
the ISIS installation steps from this new ``asp`` conda environment.
Your new ``asp`` environment already has the base ISIS software
installed, but you must run the script which sets the ISIS environment
variables, and also install the appropriate ISIS data files (if you also
have a separate ISIS conda environment, you can use the set-up script
to point the ``asp`` conda environment's ``ISISDATA`` environment
variable to your existing data area).  For more information see 
the `ISIS installation instructions
<https://github.com/USGS-Astrogeology/ISIS3>`_.

For how to build ASP, without and with conda, see
:numref:`build_from_source` and :numref:`conda_build`.
