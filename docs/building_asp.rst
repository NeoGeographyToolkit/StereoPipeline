.. _building_asp:

Building and releasing ASP
==========================

This chapter will describe how ASP can be built without and with using conda,
how to build the documentation, and how to prepare a new ASP release. This is
focused towards the developer. Users should read instead the installation guide
in :numref:`installation`.

.. _build_from_source:

Building ASP without conda
--------------------------

The dependencies for the *latest development version* of ASP are a available as
a `binary tarball <https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/>`_.

Building the *prior ASP 3.3.0 release* (:numref:`conda_intro`) without conda
entails downloading all the ASP dependencies with conda first as pre-compiled
binaries, then pulling the VisionWorkbench and Stereo Pipeline source code from
GitHub, and building locally. This is suggested only for the very adventurous
user.

The environments having the ASP dependencies are in the ``conda``
directory of the Stereo Pipeline repository, as above. After
downloading those, one can run on Linux::

    conda env create -n asp_deps -f asp_deps_3.3.0_linux_env.yaml

or on the Mac::

    conda env create -n asp_deps -f asp_deps_3.3.0_osx_env.yaml

This will create an ``asp_deps`` environment. Activate it with::

    conda activate asp_deps

Some of the .la files created by conda point to other .la files that
are not available. For that reason, those files should be edited to
replace::

    /path/to/libmylibrary.la

with::

    -L/path/to -lmylibrary

This can be done as::

    cd ~/miniconda3/envs/asp_deps/lib
    mkdir -p backup
    cp -fv  *.la backup # back these up
    perl -pi -e "s#(/[^\s]*?lib)/lib([^\s]+).la#-L\$1 -l\$2#g" *.la

The `conda-provided compilers
<https://conda.io/projects/conda-build/en/latest/resources/compiler-tools.html>`_
should be installed in the environment, if not present already. Otherwise they can
be installed as::

    conda install -c conda-forge compilers

New versions of clang tend to break a lot third-party code. On occasion it is
necessary to downgrade to compilers, especially ``clang``, which is also needed
to maintain compatibility with ISIS.

Ensure that ``parallel``, ``cmake>=3.15.5`` and ``pbzip2`` are installed::

    conda install -c conda-forge "cmake>=3.15.5" pbzip2 parallel

For Linux only, install the ``chrpath`` tool. 

Set the compiler names. They may differ somewhat from what is in the block
below, so this step may need some adjustments.

::

   if [ "$(uname)" = "Darwin" ]; then
      cc_comp=clang
      cxx_comp=clang++
    else
      cc_comp=x86_64-conda-linux-gnu-gcc
      cxx_comp=x86_64-conda-linux-gnu-g++
    fi

Set up a work directory::

    workDir=$HOME/build_asp
    mkdir -p $workDir

Build VisionWorkbench and Stereo Pipeline version 3.5.0::

    cd $workDir
    envPath=$HOME/miniconda3/envs/asp_deps
    $envPath/bin/git clone                            \
        git@github.com:visionworkbench/visionworkbench.git
    cd visionworkbench
    # Build a specific version
    git checkout 3.5.0
    mkdir -p build
    cd build
    $envPath/bin/cmake ..                             \
      -DASP_DEPS_DIR=$envPath                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                     \
      -DCMAKE_INSTALL_PREFIX=$workDir/install         \
      -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
      -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp
    make -j10 && make install

    cd $workDir
    envPath=$HOME/miniconda3/envs/asp_deps
    $envPath/bin/git clone                            \
    git@github.com:NeoGeographyToolkit/StereoPipeline.git
    cd StereoPipeline
    # Build a specific version
    git checkout 3.5.0
    mkdir -p build
    cd build
    $envPath/bin/cmake ..                             \
      -DASP_DEPS_DIR=$envPath                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                     \
      -DCMAKE_INSTALL_PREFIX=$workDir/install         \
      -DVISIONWORKBENCH_INSTALL_DIR=$workDir/install  \
      -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
      -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp
    make -j10 && make install

.. _conda_build:

Building ASP and its dependencies with conda
--------------------------------------------

This page is meant for advanced users of ASP and maintainers who would
like to use conda to rebuild ASP and all its dependencies. It is
suggested to carefully read :numref:`conda_intro` before this page.

To simplify maintenance, ASP and its dependencies are built upon ISIS
and its dependencies. Hence, in order to create a new conda ASP
package, first one needs to create an environment having the latest
released ISIS, then rebuild ASP's other dependencies and ASP itself,
while ensuring that the dependencies of each of these have their
versions synced up with the ISIS dependency versions.

The rebuilt packages will be uploaded to the ``nasa-ames-stereo-pipeline``
anaconda channel.

Setting up the ISIS environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Search for the latest available ISIS conda package::
  
    conda search -c usgs-astrogeology --override-channels isis

Here it was found that ISIS version 8.3.0 was the latest, which we
will assume throughout the rest of this document. This needs to be
adjusted for your circumstances.

Create a conda environment for this version of ISIS::

     conda create -n isis8.3.0
     conda activate isis8.3.0

Add these channels to conda::

    conda config --env --add channels conda-forge
    conda config --env --add channels usgs-astrogeology

Run::

    conda config --show channels

and verify that ``usgs-astrogeology`` and ``conda-forge`` are in this
order and above all other channels, except perhaps the
``nasa-ames-stereo-pipeline`` channel.

Install the desired version of ISIS::

    conda install isis==8.3.0

Install the version of PDAL that is compatible with current ISIS
(may already exist as part of latest ISIS)::

    conda install -c conda-forge pdal==2.6.0

Save the current environment as follows::

    conda env export > isis8.3.0.yaml

Fetching the build tools
~~~~~~~~~~~~~~~~~~~~~~~~

We will create a new ``tools`` environment to have all the tools we
will need. These could be appended to the earlier environment, but it
is less likely to to have issues with dependency conflicts if these
are kept separate.

It is very strongly suggested to install the same versions of compilers
as what ISIS uses, and ensure the same versions are in every recipe below.

::

    conda create -n tools
    conda activate tools
    conda install -c conda-forge anaconda-client conda-build \
      conda-verify

.. _packages_to_build:

Packages to build
~~~~~~~~~~~~~~~~~

Many additional package need to be built, using ``conda build``. These packages
can be downloaded with ``git clone`` from:

  https://github.com/NeoGeographyToolkit/geoid-feedstock.git
  https://github.com/NeoGeographyToolkit/fgr-feedstock.git
  https://github.com/NeoGeographyToolkit/libnabo-feedstock.git
  https://github.com/NeoGeographyToolkit/libpointmatcher-feedstock.git
  https://github.com/NeoGeographyToolkit/s2p-feedstock.git
  https://github.com/NeoGeographyToolkit/libelas-feedstock.git
  https://github.com/NeoGeographyToolkit/multiview-feedstock
  https://github.com/NeoGeographyToolkit/visionworkbench-feedstock.git

Temporarily, for the ASP 3.5.0 release, a few more dependencies exist::

  https://github.com/NeoGeographyToolkit/ilmbase-feedstock.git
  https://github.com/NeoGeographyToolkit/openexr-feedstock.git
  https://github.com/NeoGeographyToolkit/pdal-feedstock.git

Lastly, the recipe for ASP itself::

  https://github.com/NeoGeographyToolkit/stereopipeline-feedstock.git

Synchronize the versions with the existing environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For each of the above feedstocks, check the ``recipe/meta.yaml`` file
and ensure all dependencies are in sync with what is in the file
``isis8.3.0.yaml`` generated earlier. This can be done automatically
with a provided script in the ASP repository::

     python StereoPipeline/conda/update_versions.py isis8.3.0.yaml \
       gdal-feedstock

and the same for the other packages.

It is very important to note that this script is not fool-proof, and the
changes it makes should be very carefully examined. Also, the versions
of dependencies can be different on Linux and OSX, so the script should
be run separately for each platform.

Having incompatible versions will result in failure when resolving
the dependencies with conda.

It is suggested to examine the changed ``meta.yaml``, and if in doubt,
leave the values as they were before modified by this script. 

In the ``visionworkbench`` and ``stereopipeline`` recipes update the
``git_tag`` value to reflect the desired commit from the Git
history, or leave it as is if desired to build the latest code.

When making an ASP release, one can tag the commit based on
which the release happens in the VisionWorkbench and StereoPipeline
repositories, and then that tag can be used in the ``git_tag`` field.
See :numref:`asp_release_guide` for more details.

Later on, after the packages are built and tested, ensure that all the
changes to the feedstock repositories are checked in.

Build the conda packages
~~~~~~~~~~~~~~~~~~~~~~~~

When building a package that depends on other packages in the
``nasa-ames-stereo-pipeline`` channel, edit its ``meta.yaml`` file and specify
the appropriate version for those dependencies. 

It is very important to also ensure there is a new version for this package at
the top of ``meta.yaml``.

Set the solver to ``libmamba``, for speed::

    conda config --set solver libmamba

This may be the default in more recent versions of conda.
    
Each of the packages above can be built, in the order specified in
:numref:`conda_build_order`, as follows::

    conda build -c nasa-ames-stereo-pipeline -c usgs-astrogeology \
      -c conda-forge fgr-feedstock

Upload the produced packages to the ``nasa-ames-stereo-pipeline`` channel by
first logging in, via the command:

::
    
    anaconda login nasa-ames-stereo-pipeline

The ``anaconda`` tool may have its own dependencies and may need to be 
installed in a different environment.

Run a command along the lines:

::

    anaconda upload \
      $HOME/miniconda3/envs/asp_deps/conda-bld/linux-64/MyPackage.tar.bz2

(Use above the path echoed on the screen by the ``conda build``
command.)

Use the ``--force`` option if desired to overwrite any existing
package with the same name and version. Be careful not to overwrite
a package that is meant to be used with a prior version of ASP.

After a package is uploaded, it can be installed in the existing
``isis8.3.0`` environment as::

    conda install -c nasa-ames-stereo-pipeline \
      -c usgs-astrogeology                     \
      -c conda-forge                           \
      libelas=asp3.3.0

If this is slow, check if the solver is set to ``libmamba``. 
 
To list all packages in that channel, do::

    conda search -c nasa-ames-stereo-pipeline --override-channels

To delete a package from this channel, run::

    anaconda remove nasa-ames-stereo-pipeline/mypackage/myversion
  
.. _conda_build_order:

Order of building the packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is suggested to build the above packages in the order listed
earlier, as some of them depend on others.

Note that ``libpointmatcher`` depends on ``libnabo``, while ``pdal`` depends on
``gdal``, ``visionworkbench`` depends on ``gdal``, and ``multiview`` depends on
``tbb`` (the latter for OSX only). 

The ``stereopipeline`` package depends on all of these so it should be
built the last.

Additional ASP dependencies
~~~~~~~~~~~~~~~~~~~~~~~~~~~

VisionWorkbench and StereoPipeline have a few more conda dependencies
that need to be fetched from ``conda-forge``.

If desired to create an environment in which to build ASP or to update
the one in :numref:`build_from_source`, the dependencies can be looked
up in the ``meta.yaml`` files for these conda packages, after fetching
them according to :numref:`packages_to_build`.

Helper scripts
~~~~~~~~~~~~~~

The ``.github/workflows`` directory in the ``StereoPipeline`` repository has a
few helper scripts that show in detail the commands that are run to build ASP
and its dependencies, from source and with ``conda``.

.. _build_asp_doc:

Building the documentation
--------------------------

The ASP documentation is written in ReStructured Text and is built
with the Sphinx-Doc system (https://www.sphinx-doc.org) with 
sphinxcontrib-bibtex (https://sphinxcontrib-bibtex.readthedocs.io).

Documentation for the latest build and latest release is available online
at https://stereopipeline.readthedocs.io/en/latest/.

To build the documentation locally, install these packages such as:: 

    conda create -n sphinx -c conda-forge python=3.6 \
      sphinx=3.5.4 sphinxcontrib-bibtex=2.1.4  
    conda activate sphinx

Note that we used a separate conda environment to minimize the chance
of conflict with other dependencies. Also, Sphinx version 4 seems to
have trouble compiling our documentation, hence a lower version is
used here.

The ``docs`` directory contains the root of the documentation. Running ``make
html`` will create the HTML version of the documentation in the ``_build``
subdirectory.

Building the PDF documentation is no longer supported. 

If the documentation builds well locally but fails to update on the web, see the
`cloud build status page
<https://readthedocs.org/projects/stereopipeline/builds/>`_.

.. _asp_release_guide:

Releasing a new version of ASP
------------------------------

This is reading for ASP maintainers.

Update the version number
~~~~~~~~~~~~~~~~~~~~~~~~~

Inside both the ASP and VisionWorkbench code, edit ``src/CMakeLists.txt`` and
set the new version, which should be the same for both packages, and in the
format ``x.y.z``. If the value there is ``x.y.z-alpha``, which is used to tag a
pre-release, remove the ``-alpha`` part. Increment one of these digits,
depending on whether this is a major, minor, or bugfix release. See
https://semver.org for guidance.

Update the documentation
~~~~~~~~~~~~~~~~~~~~~~~~

Search all documentation for the old version number for ASP and ISIS (such as
8.3.0) and replace it with the new version numbers. This includes files in the
base directory, not just in ``docs``.

Update NEWS.rst. Add the release date on top, along the lines of prior releases
(see further down in that file). This file must have a detailed log of all
changes, especially those that result in changed behavior or options, and it
should be incrementally updated as changes are made during development.

Update the copyright year in the README.rst file.

Commit and tag
~~~~~~~~~~~~~~

Commit all changes. Tag the release in the VisionWorkbench and ASP repos.
Example:: 

  git tag 3.5.0
  git push origin 3.5.0 # commit to your branch
  git push god    3.5.0 # commit to main branch

(Here it is assumed that ``origin`` points to your own fork and ``god``
points to the parent repository.)

If more commits were made and it is desired to apply this tag to a
different commit, first remove the exiting tag with::

  git tag -d 3.5.0
  git push origin :refs/tags/3.5.0
  git push god    :refs/tags/3.5.0

Build ASP with conda
~~~~~~~~~~~~~~~~~~~~

See :numref:`conda_build`. 
    
Save a record of the conda packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is suggested to save a complete record of all packages that went into this conda
release, as sometimes conda may have issues solving for the dependencies or it may 
return a non-unique solution.

The conda environment having the given ASP release can be exported as::

    conda activate asp
    conda env export > StereoPipeline/conda/asp_3.5.0_linux_env.yaml

This was for Linux, and it works analogously on OSX. How to recreate ASP
from this file is described in :numref:`conda_intro`.

It is suggested to commit these in to the ASP repository, in the ``conda``
subfolder. These files can be checked in after the release is already tagged,
built, and tested.

An example for how to use this file to create the environment having the ASP
dependencies in :numref:`build_from_source`.

.. _build_binaries:

Building self-contained binaries
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In addition to creating a conda package, it is also convenient and ship a
zipped package having all ASP tools and needed libraries (this includes the ISIS
libraries but not the ISIS tools). 

Such a build is created for each release and also daily. These are posted on the
GitHub release page (:numref:`release`). 

ASP uses a custom build system. It can be downloaded with ``git`` from:

    https://github.com/NeoGeographyToolkit/BinaryBuilder

Create a conda environment that has the dependencies for building ASP, as
described in :numref:`build_from_source`. Assume it is called ``asp_deps``.

Install the C, C++, and Fortran compilers (same versions as for ISIS),
``cmake>=3.15.5``, ``pbzip2``, ``parallel``, and for Linux also the ``chrpath``
tool, as outlined on that page.

Go to the directory ``BinaryBuilder``, and run::

    /path/to/python3                                \
      ./build.py                                    \
      --cc <path to C comipler>                     \
      --cxx <path to C++ compiler>                  \
      --gfortran <path to Fortran compiler>         \
      --asp-deps-dir $HOME/miniconda3/envs/asp_deps \
      --build-root build_asp                        \
      --skip-tests                                  \
      visionworkbench stereopipeline

This will fetch and build the latest VisionWorkbench and Stereo Pipeline in
``build_asp/build``, and will install them in ``build_asp/install``.

Create a conda environment having Python and numpy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ISIS expects a full Python distribution to be shipped. To avoid shipping
the entire ``asp_deps`` environment, we create a separate environment
having only Python, numpy, with versions as expected by current ISIS.
Run, for example::

    conda create -c conda-forge -n python_isis8 python=x.y.x numpy=a.b.c

Note that different versions of these may be needed for Linux and OSX.
The ``conda list`` command within the ``asp_deps`` environment 
can be used to look up the desired versions.

Package the build
~~~~~~~~~~~~~~~~~

Run in ``BinaryBuilder`` the command::

    /path/to/python3                                  \
      ./make-dist.py build_asp/install                \
      --asp-deps-dir $HOME/miniconda3/envs/asp_deps   \
      --python-env $HOME/miniconda3/envs/python_isis8

Building and packaging should be done separately for Linux and OSX.

Test ASP
~~~~~~~~

The script ``auto_build/launch_master.sh`` in ``BinaryBuilder`` can be invoked
to build and test ASP. This script and also ``auto_build/utils.sh`` need to be
read carefully and some variables adjusted.

The `StereoPipeline test suite
<https://github.com/NeoGeographyToolkit/StereoPipelineTest>`_ is run. It has
comprehensive tests for the ASP tools.

This functionality creates the daily builds, which are then
uploaded to the GitHub release page (:numref:`release`). 

Prepare the documentation
~~~~~~~~~~~~~~~~~~~~~~~~~

Follow the instructions in :numref:`build_asp_doc`.

Push the release to GitHub
~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a release on `GitHub
<https://github.com/NeoGeographyToolkit/StereoPipeline/releases>`_. Use the tag
for the current release. Add to the release notes a link to the appropriate
NEWS section of the documentation (:numref:`news`). *Only after this save
the release.* 

*Do not delete and recreate the release* (:numref:`zenodo`). It is fine to
upload the binaries after a release is created, and delete and re-upload them.

The GitHub tool ``gh`` can be used to push the binaries to the release. 
Here's an example usage::

  cd BinaryBuilder
  for file in StereoPipeline-3.5.0-Linux.tar.bz2 \
              StereoPipeline-3.5.0-OSX.tar.bz2; do
    gh release upload 3.5.0 $file \
      -R git@github.com:NeoGeographyToolkit/StereoPipeline.git   
  done

Alternatively, these can be uploaded manually.

.. _zenodo:

Zenodo link for the release
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Creating a release will trigger producing the Zenodo permanent link (after a few
minutes). That link cannot be changed, and the text it copies from the release
notes cannot be changed either.

It is very important to *create the release just once for the given tag*.
Otherwise, the Zenodo permanent link will always point to the earliest attempt
at making the release. It is fine to later overwrite the binaries for this
release, or even to upload them later. Just do not delete and recreate the
release page itself.

Do not just rename the latest automatically uploaded daily build, as that will
create an incorrect Zenodo link.

Wait for Zenodo to mint the link for this release, then visit the Zenodo page at
https://zenodo.org/badge/latestdoi/714891 (which will always point to the
'latest' DOI) and find there this release's URL. Put it at the top of
`README.rst
<https://github.com/NeoGeographyToolkit/StereoPipeline/blob/master/README.rst>`_,
in the appropriate ``target`` field. 

*Increment the version in the image field right above that.*

Add this link also to the NEWS.rst page, under the section name for the current
release.

Push this update to GitHub. The new commit will be after the tag for the
release, but that is not an issue. It is best to not change the tag after
the release and Zenodo link got created.

Announce the release
~~~~~~~~~~~~~~~~~~~~

Send an announcement of the new release to the `mailing list
<https://groups.google.com/forum/\#!forum/ames-stereo-pipeline-support>`_ and to
the old stereo-pipeline@lists.nasa.gov, with a link to the NEWS section for the
current release from the documentation.

Post-release work
~~~~~~~~~~~~~~~~~

Update the version number in ``src/CMakeLists.txt`` in boh the VisionWorkbench
and ASP repositories.  

If version 3.5.0 just got released, we expect that the next feature release will
likely be be 3.6.0. The version tag should be updated to 3.6.0-alpha in
anticipation (see https://semver.org for guidance).
