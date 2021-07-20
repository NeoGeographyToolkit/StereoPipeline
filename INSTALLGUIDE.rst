Installation
============

Precompiled binaries are available for the stable releases and the
current development build.  Stereo Pipeline can also be compiled 
from source, but this is not recommended.

Precompiled binaries (Linux and macOS)
--------------------------------------

Simply download the appropriate distribution for your operating
system, extract, and run the executables in the ``bin`` subdirectory.
No other 'installation' steps or administrative rights are necessary.

- `Stable releases and daily builds
  <https://github.com/NeoGeographyToolkit/StereoPipeline/releases>`_

See the NEWS file (:numref:`news`) for the most recent additions.

To permanently add the ASP executable subdirectory to your PATH, you
can add the following line to your shell configuration (e.g.,
``~/.bashrc``), replacing ``/path/to/StereoPipeline/bin`` with the
location on your filesystem::

    export PATH=${PATH}:/path/to/StereoPipeline/bin

Planetary images
~~~~~~~~~~~~~~~~

If you plan to process images from NASA's spacecraft that are
exploring other planets, you will probably need to have :term:`ISIS`
installed.  A full ISIS installation is not required for operation
of the main Stereo Pipeline programs (only the ISIS data directory
is needed), but it is required for certain preprocessing steps
before Stereo Pipeline programs are run for planetary data.  If you
only want to process terrestrial Digital Globe images, skip to the
`Quick Start for Digital Globe Users`_ section.

To perform pre-processing (radiometric calibration, ephemeris
processing, etc.), of non-terrestrial images prior to running Stereo
Pipeline, you will need to install :term:`ISIS`.  Just as with our 
binaries, you can use the ISIS binaries as-is.

If you need to recompile, you can follow the instructions for
`Building ASP from Source`_ (but we don't recommend it).  If the
current version of ISIS is newer than the version of ISIS that the
Stereo Pipeline is compiled against (listed in the ASP Release
Notes), be assured that we're working on rolling out a new version.
However, since Stereo Pipeline has its own self-contained version
of ISIS's libraries built internally, you should be able to use a
newer version of ISIS with the now dated version of ASP. This is
assuming no major changes have taken place in the data formats or
camera models by the ISIS Developers. At the very least, you should
be able to install older versions of ISIS if a failure is found.
To do so, follow the ISIS installation instructions, but create a
new conda environment (not the one with your current ISIS), and right
before you would run ``conda install isis``, run ``conda search
isis`` to find all of the versions of ISIS available for installation.
For example, if you wanted to install ISIS 5.0.1, and it is available
in the ``conda search isis`` listing, you can run ``conda install
isis=5.0.1`` (to install that specific version of ISIS) and then
follow the remainder of the ISIS installation instructions.

In closing, running the Stereo Pipeline executables only requires
that you have downloaded the ISIS secondary data and have
appropriately set the ``ISISDATA`` environment variable. This is
normally performed for the user by starting up the conda ISIS 
environment.

Quick start for ISIS users
~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the packaged ASP tarball
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Fetch Stereo Pipeline from
   https://github.com/NeoGeographyToolkit/StereoPipeline/releases

#. Fetch ISIS Binaries and install, following
   https://github.com/USGS-Astrogeology/ISIS3#installation

#. Fetch ISIS Data, as detailed at
   https://github.com/USGS-Astrogeology/ISIS3#the-isis-data-area

#. Untar Stereo Pipeline::

     tar xzvf StereoPipeline-<VERSION>-<ARCH>-<OS>.tar.gz

#. Add Stereo Pipeline to Path (optional):

   - bash: ``export PATH="</path/to/StereoPipeline>/bin:${PATH}"``
   - csh: ``setenv PATH "</path/to/StereoPipeline>/bin:${PATH}"``

#. Try It Out: See :numref:`moc_tutorial` for an example.


Installing ASP and ISIS in the same conda environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is discussed further down. 

Quick start for Digital Globe users
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#. Fetch Stereo Pipeline from
   https://github.com/NeoGeographyToolkit/StereoPipeline/releases

#. Untar Stereo Pipeline::

     tar xzvf StereoPipeline-<VERSION>-<ARCH>-<OS>.tar.gz

#. Try It Out: Processing Earth images is described in the data processing
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

   **I/O ERROR** Unable to open [$ISISDATA/<Some/Path/Here>].
   Stereo step 0: Preprocessing failed

You need to set up your ISIS environment or manually set the correct
location for ``ISISDATA``.

::

   bash: stereo: command not found

You need to add the ``bin`` directory of your deployed Stereo Pipeline
installation to the environmental variable ``PATH``.

.. _conda_intro:

Fetching pre-compiled ASP with conda
------------------------------------

Get conda from::

    https://docs.conda.io/en/latest/miniconda.html

Make it executable, and run::

    ./Miniconda3-latest-Linux-x86_64.sh

on Linux, and the appropriate version on OSX. Use the suggested::

    $HOME/miniconda3

directory for installation. 

Create an environment for ASP as::

    conda create -n asp python=3.6
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

We do not recommend using the ``--prepend channels`` argument, as that
will add the ``nasa-ames-stereo-pipeline`` to your default
``~/.condarc`` file and will have consequences for *all* of your conda
environments, which you don't want.

Install ASP with the command::

    conda install stereo-pipeline==2.7.0

This will install ASP 2.7.0 together with ISIS 4.1.0.

Running instead::

    conda install stereo-pipeline==isis5.0.1

will install a development version of ASP together with ISIS 5.0.1.

If using ISIS, the environmental variable ISISROOT should be set to
point to this distribution, such as::

    export ISISROOT=$HOME/miniconda3/envs/asp

Check that the ``stereo`` command can be found as::

    which stereo

Some variability may exist in the precise dependencies fetched by
conda. For the record, the full environment for this release can be
found as a set of .yaml files in the ``conda/`` subdirectory of the
Stereo Pipeline GitHub repository. So, alternatively, the installation
can happen as::

    conda env create -f asp_2.7.0_linux_env.yaml

or::

    conda env create -f asp_2.7.0_osx_env.yaml

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

Building ASP from source
------------------------

This entails downloading all the ASP dependencies with conda first as
pre-compiled binaries, then pulling the VisionWorkbench and Stereo
Pipeline source code from GitHub, and building locally. This is
suggested only for the very adventurous user.

The environments having the ASP dependencies are in the ``conda``
directory of the Stereo Pipeline repository, as above. After
downloading those, one can run on Linux::

    conda env create -f asp_deps_2.7.0_linux_env.yaml

or on the Mac::

    conda env create -f asp_deps_2.7.0_osx_env.yaml

This will create an ``asp_deps`` environment. Activate it with::

    conda activate asp_deps

Some of the .la files created by conda point to other .la files that
are not available. For that reason, those files should be edited to
replace::

    /path/to/libmylibrary.la

with::

    -L/path/to -lmylibrary

This can be done with the following commands::

    cd ~/miniconda3/envs/asp_deps/lib
    mkdir -p  backup
    cp -fv  *.la backup # back these up
    perl -pi -e "s#(/[^\s]*?lib)/lib([^\s]+).la#-L\$1 -l\$2#g" *.la

The Linux environment will also contain the needed C and C++
compilers. On the Mac the compilers provided with conda did not build
ASP correctly, hence it is suggested to use the Apple-provided clang
and clang++.

Next, set up a work directory::

    buildDir=$HOME/build_asp
    mkdir -p $buildDir

Building VisionWorkbench and Stereo Pipeline on Linux::

    cd $buildDir
    ~/miniconda3/envs/asp_deps/bin/git clone \
        git@github.com:visionworkbench/visionworkbench.git
    cd visionworkbench
    git checkout 2.7.0 # check out the desired commit
    mkdir -p build
    cd build
    ~/miniconda3/envs/asp_deps/bin/cmake ..                                                 \
      -DASP_DEPS_DIR=$HOME/miniconda3/envs/asp_deps                                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                                                           \
      -DCMAKE_INSTALL_PREFIX=$buildDir/install                                              \
      -DCMAKE_C_COMPILER=$HOME/miniconda3/envs/asp_deps/bin/x86_64-conda_cos6-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=$HOME/miniconda3/envs/asp_deps/bin/x86_64-conda_cos6-linux-gnu-g++
    make -j10
    make install

    cd $buildDir
    ~/miniconda3/envs/asp_deps/bin/git clone \
    git@github.com:NeoGeographyToolkit/StereoPipeline.git
    cd StereoPipeline
    git checkout 2.7.0 # check out the desired commit
    mkdir -p build
    cd build
    ~/miniconda3/envs/asp_deps/bin/cmake ..                                                 \
      -DASP_DEPS_DIR=$HOME/miniconda3/envs/asp_deps                                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                                                           \
      -DCMAKE_INSTALL_PREFIX=$buildDir/install                                              \
      -DVISIONWORKBENCH_INSTALL_DIR=$buildDir/install                                       \
      -DCMAKE_C_COMPILER=$HOME/miniconda3/envs/asp_deps/bin/x86_64-conda_cos6-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=$HOME/miniconda3/envs/asp_deps/bin/x86_64-conda_cos6-linux-gnu-g++
    make -j10
    make install

Building VisionWorkbench and ASP on OSX (just as above, but omitting the compilers)::

    cd $buildDir
    ~/miniconda3/envs/asp_deps/bin/git clone \
      git@github.com:visionworkbench/visionworkbench.git
    cd visionworkbench
    git checkout 2.7.0 # check out the desired commit
    mkdir -p build
    cd build
    ~/miniconda3/envs/asp_deps/bin/cmake ..                                                 \
      -DASP_DEPS_DIR=$HOME/miniconda3/envs/asp_deps                                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                                                           \
      -DCMAKE_INSTALL_PREFIX=$buildDir/install
    make -j10
    make install

    cd $buildDir
    ~/miniconda3/envs/asp_deps/bin/git clone \
      git@github.com:NeoGeographyToolkit/StereoPipeline.git
    cd StereoPipeline
    git checkout 2.7.0 # check out the desired commit
    mkdir -p build
    cd build
    ~/miniconda3/envs/asp_deps/bin/cmake ..                                                 \
      -DASP_DEPS_DIR=$HOME/miniconda3/envs/asp_deps                                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                                                           \
      -DVISIONWORKBENCH_INSTALL_DIR=$buildDir/install                                       \
      -DCMAKE_INSTALL_PREFIX=$buildDir/install
    make -j10
    make install

Building the documentation
--------------------------

The ASP documentation is encoded in ReStructured Text and is built
with the Sphinx-Doc system (https://www.sphinx-doc.org) with 
sphinxcontrib-bibtex (https://sphinxcontrib-bibtex.readthedocs.io).
These packages can be installed and activated as follows::

    conda create -n sphinx python=3.6 sphinx==3.5.4 \
      sphinxcontrib-bibtex==2.1.4  
    conda activate sphinx

Note that we used a separate conda environment to minimize the chance
of conflict with other dependencies. Also, sphinx version 4 seems to
have trouble compiling our documentation, hence a lower version is
used here.

In order to build the PDF (but not the HTML) document, a full
LaTeX distribution is also necessary, which is not installable with
conda at this time, and whose installation may be specific to your
system.

The ``docs`` directory contains the root of the documentation. Running
``make html`` and ``make latexpdf`` there will create the HTML and PDF
versions of the documentation in the _build subdirectory. In
particular, the PDF document will be at::

  ./_build/latex/asp_book.pdf

Building ASP and its dependencies with conda
--------------------------------------------

This is an advanced topic discussed in :numref:`conda_build`.
