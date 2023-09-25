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

This entails downloading all the ASP dependencies with conda first as
pre-compiled binaries, then pulling the VisionWorkbench and Stereo
Pipeline source code from GitHub, and building locally. This is
suggested only for the very adventurous user.

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
should be installed in the environment, if not present already.

::

    conda install -c conda-forge compilers

Ensure that ``cmake>=3.15.5`` and ``pbzip2`` are installed::

    conda install -c conda-forge "cmake>=3.15.5" pbzip2

For Linux only, install the ``chrpath`` tool. 

Set the compiler names. They may differ somewhat from what is in the block
below, so this step may need some adjustments.

::

    isMac=$(uname -s | grep Darwin)
    if [ "$isMac" != "" ]; then
      cc_comp=clang
      cxx_comp=clang++
    else
      cc_comp=x86_64-conda_cos6-linux-gnu-gcc
      cxx_comp=x86_64-conda_cos6-linux-gnu-g++
    fi

Set up a work directory::

    buildDir=$HOME/build_asp
    mkdir -p $buildDir

Build VisionWorkbench and Stereo Pipeline::

    cd $buildDir
    envPath=$HOME/miniconda3/envs/asp_deps
    $envPath/bin/git clone                            \
        git@github.com:visionworkbench/visionworkbench.git
    cd visionworkbench
    # Uncomment below if desired to build a specific version
    # git checkout 3.3.0
    mkdir -p build
    cd build
    $envPath/bin/cmake ..                             \
      -DASP_DEPS_DIR=$envPath                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                     \
      -DCMAKE_INSTALL_PREFIX=$buildDir/install        \
      -DCMAKE_C_COMPILER=${envPath}/bin/$cc_comp      \
      -DCMAKE_CXX_COMPILER=${envPath}/bin/$cxx_comp
    make -j10 && make install

    cd $buildDir
    envPath=$HOME/miniconda3/envs/asp_deps
    $envPath/bin/git clone                            \
    git@github.com:NeoGeographyToolkit/StereoPipeline.git
    cd StereoPipeline
    # Uncomment below if desired to build a specific version
    # git checkout 3.3.0
    mkdir -p build
    cd build
    $envPath/bin/cmake ..                             \
      -DASP_DEPS_DIR=$envPath                         \
      -DCMAKE_VERBOSE_MAKEFILE=ON                     \
      -DCMAKE_INSTALL_PREFIX=$buildDir/install        \
      -DVISIONWORKBENCH_INSTALL_DIR=$buildDir/install \
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

Here it was found that ISIS version 8.0.0 was the latest, which we
will assume throughout the rest of this document. This needs to be
adjusted for your circumstances.

Create a conda environment for this version of ISIS::

     conda create -n isis8.0.0
     conda activate isis8.0.0

Add these channels to conda::

    conda config --env --add channels conda-forge
    conda config --env --add channels usgs-astrogeology

Run::

    conda config --show channels

and verify that ``usgs-astrogeology`` and ``conda-forge`` are in this
order and above all other channels, except perhaps the
``nasa-ames-stereo-pipeline`` channel.

Install the desired version of ISIS::

    conda install isis==8.0.0

Search and install the latest version of the ``usgscsm`` package,
for example, as::

    conda search -c conda-forge --override-channels usgscsm
    conda install -c conda-forge usgscsm==1.7.0

If that package is too old, consider rebuilding it, following
the recipe at:

    https://github.com/NeoGeographyToolkit/usgscsm-feedstock

See :numref:`packages_to_build` for how to fetch and build this.
  
Save the current environment as follows::

    conda env export > isis8.0.0.yaml

Fetching the build tools
~~~~~~~~~~~~~~~~~~~~~~~~

We will create a new ``tools`` environment to have all the tools we
will need. These could be appended to the earlier environment, but it
is less likely to to have issues with dependency conflicts if these
are kept separate.

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
  https://github.com/NeoGeographyToolkit/htdp-feedstock.git
  https://github.com/NeoGeographyToolkit/laszip-feedstock.git
  https://github.com/NeoGeographyToolkit/fgr-feedstock.git
  https://github.com/NeoGeographyToolkit/libnabo-feedstock.git
  https://github.com/NeoGeographyToolkit/libpointmatcher-feedstock.git
  https://github.com/NeoGeographyToolkit/gdal-feedstock.git
  https://github.com/NeoGeographyToolkit/liblas-feedstock.git
  https://github.com/NeoGeographyToolkit/s2p-feedstock.git
  https://github.com/NeoGeographyToolkit/libelas-feedstock.git
  https://github.com/NeoGeographyToolkit/multiview-feedstock
  https://github.com/NeoGeographyToolkit/visionworkbench-feedstock.git
  https://github.com/NeoGeographyToolkit/stereopipeline-feedstock.git

On OSX, also fetch and build:

  https://github.com/NeoGeographyToolkit/tbb-feedstock.git

This is needed as a workaround for the ``tbb`` conda package
on OSX conflicting with the ``embree`` package which is rather old
but is needed by ``ISIS``. 

Also, per the earlier note, consider rebuilding ``usgscsm`` if
there there are updates in its GitHub repository which are not yet
released on conda-forge.

Synchronize the versions with the existing environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For each of the above feedstocks, check the ``recipe/meta.yaml`` file
and ensure all dependencies are in sync with what is in the file
``isis8.0.0.yaml`` generated earlier. This can be done automatically
with a provided script in the ASP repository::

     python StereoPipeline/conda/update_versions.py isis8.0.0.yaml \
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
history. 

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

Each of the packages above can be built, in the order specified in
:numref:`conda_build_order`, as follows::

    conda build -c nasa-ames-stereo-pipeline -c usgs-astrogeology \
      -c conda-forge gdal-feedstock

Consider using the options ``--no-verify --no-test`` with this tool
if it fails with with unrelated errors at the packaging stage, as
it happened on OSX on occasion. This is a risky option and should
be a measure of last resort.

Upload the produced packages to the ``nasa-ames-stereo-pipeline`` channel by
first logging in, via the command:

::
    
    anaconda login

and specifying the channel as the user name, and then running a
command along the lines:

::

    anaconda upload \
      $HOME/miniconda3/envs/asp_deps/conda-bld/linux-64/mypackage.tar.bz2

(Use above the path echoed on the screen by the ``conda build``
command.)

Use the ``--force`` option if desired to overwrite any existing
package with the same name and version. Be careful not to overwrite
a package that is meant to be used with a prior version of ASP.

After a package is uploaded, it can be installed in the existing
``isis8.0.0`` environment as::

    conda install -c nasa-ames-stereo-pipeline \
      -c usgs-astrogeology                     \
      -c conda-forge                           \
      gdal==3.5_isis7

To list all packages in that channel, do::

    conda search -c nasa-ames-stereo-pipeline --override-channels

To delete a package from this channel, run::

    anaconda remove nasa-ames-stereo-pipeline/mypackage/myversion
  
.. _conda_build_order:

Order of building the packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is suggested to build the above packages in the order listed
earlier, as some of them depend on others.

Note that ``libpointmatcher`` depends on ``libnabo``, while ``liblas`` depends
on ``laszip`` and ``gdal``, ``visionworkbench`` depends on ``gdal``, and
``multiview`` depends on ``tbb`` (the latter for OSX only). 

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

.. _build_asp_doc:

Building the documentation
--------------------------

The ASP documentation is encoded in ReStructured Text and is built
with the Sphinx-Doc system (https://www.sphinx-doc.org) with 
sphinxcontrib-bibtex (https://sphinxcontrib-bibtex.readthedocs.io).
These packages can be installed and activated as follows::

    conda create -n sphinx -c conda-forge python=3.6 \
      sphinx=3.5.4 sphinxcontrib-bibtex=2.1.4  
    conda activate sphinx

Note that we used a separate conda environment to minimize the chance
of conflict with other dependencies. Also, Sphinx version 4 seems to
have trouble compiling our documentation, hence a lower version is
used here.

In order to build the PDF (but not the HTML) document, a full
LaTeX distribution is also necessary, such as TeX Live. 

The ``docs`` directory contains the root of the documentation. Running
``make html`` and ``make latexpdf`` there will create the HTML and PDF
versions of the documentation in the _build subdirectory. In
particular, the PDF document will be at::

  ./_build/latex/asp_book.pdf

If the documentation builds well locally but fails to update 
on the web, see the `build status page <https://readthedocs.org/projects/stereopipeline/builds/>`_.

.. _asp_release_guide:

Releasing a new version of ASP using conda
------------------------------------------

This and subsequent sections are mostly reading for ASP developers.
This content is included with the user documentation as this way it is
easier to refer to relevant sections in the user guide.

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
8.0.0) and replace it with the new version numbers. This includes files in the
base directory, not just in ``docs``.

Update NEWS.rst. Add the release date on top, along the lines of prior releases
(see further down in that file). This file must have a detailed log of all
changes, especially those that result in changed behavior or options, and it
should be incrementally updated as changes are made during development.

Commit and tag
~~~~~~~~~~~~~~

Commit all changes. Tag the release in the VisionWorkbench and ASP repos. Example:: 

  git tag 3.3.0
  git push origin 3.3.0 # commit to your branch
  git push god    3.3.0 # commit to main branch

(Here it is assumed that 'origin' points to your own fork and 'god'
points to the parent repository.)

If more commits were made and it is desired to apply this tag to a
different commit, first remove the exiting tag with::

  git tag -d 3.3.0
  git push origin :refs/tags/3.3.0
  git push god    :refs/tags/3.3.0

Build ASP with conda
~~~~~~~~~~~~~~~~~~~~

See :numref:`conda_build`. 

Test ASP
~~~~~~~~

Fetch the ``stereo-pipeline`` conda package, per :numref:`conda_intro`. Save it,
for example, in a conda environment named ``asp``.

Use the `ASP test framework
<https://github.com/NeoGeographyToolkit/StereoPipelineTest#readme>`_ to test it.
Set the correct paths in the configuration file used for testing, as described
there.
    
Save a record of the conda packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is suggested to save a complete record of all packages that went into this conda
release, as sometimes conda may have issues solving for the dependencies or it may 
return a non-unique solution.

The conda environment having the given ASP release can be exported as::

    conda activate asp
    conda env export > asp_3.3.0_linux_env.yaml

This was for Linux, and it works analogously on OSX. How to recreate ASP
from this file is described in :numref:`conda_intro`.

A file can also be made that lacks the entries for ASP and visionworkbench, so
keeping only the dependencies. It can be saved with a name like
``asp_3.3.0_linux_deps.yaml`` (also edit it and change the name of the
environment). 

It is suggested to commit these in to the ASP repository, in the ``conda``
subfolder. These files can be checked in after the release is already tagged,
built, and tested.

An example for how to use this file to create the environment having the ASP
dependencies in :numref:`build_from_source`.

.. _build_binaries:

Building self-contained binaries
--------------------------------

In addition to creating a conda package, it is also convenient and ship an
archive having all ASP tools and needed libraries (this includes the ISIS
libraries but not the ISIS tools). 

Such a build is created for each release and also daily. These are posted on the
GitHub release page (:numref:`precompiled_binaries`). 

This work is a continuation of the process described in
:numref:`asp_release_guide`.

Use BinaryBuilder
~~~~~~~~~~~~~~~~~

ASP uses a custom build system. It can be downloaded with ``git`` from:

    https://github.com/NeoGeographyToolkit/BinaryBuilder

Create a conda environment that has the dependencies for building ASP, as
described in :numref:`build_from_source`. Assume it is called ``asp_deps``.

Install the C, C++, and Fortran compilers, ``cmake>=3.15.5``, ``pbzip2``,
and for Linux also the ``chrpath`` tool, as outlined on that page.

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

    conda create -n python_isis8 python=x.y.x numpy=a.b.c

Note that different versions of these may be needed for Linux and OSX.
The ``conda list`` command within the ``asp_deps`` environment 
can be used to look up the desired versions.

Prepare the documentation
~~~~~~~~~~~~~~~~~~~~~~~~~

Follow the instructions in :numref:`build_asp_doc`. 

Copy ``asp_book.pdf`` to the ``BinaryBuilder/dist-add/`` directory. Then it will
be added to the packaged build. 

If this operation is not done, likely ``dist-add/`` has the previous release
documentation, which will be shipped instead. So this step must not be missed.

Package the build
~~~~~~~~~~~~~~~~~

Run in ``BinaryBuilder`` the command::

    /path/to/python3                                  \
      ./make-dist.py build_asp/install                \
      --asp-deps-dir $HOME/miniconda3/envs/asp_deps   \
      --python-env $HOME/miniconda3/envs/python_isis8

The same command can be used to package the ``asp`` conda environment created
earlier. Then, one should use instead of ``build_asp/install`` the directory
``$HOME/miniconda3/envs/asp``. The dependencies will still come from
``$HOME/miniconda3/envs/asp_deps``.

Building and packaging should be done separately for Linux and OSX.

Creating a GitHub release
-------------------------

Create a release on `GitHub
<https://github.com/NeoGeographyToolkit/StereoPipeline/releases>`_. Use the tag
for the current release. Upload the binaries (for Linux and OSX,
:numref:`build_binaries`) and pdf documentation (asp_book.pdf,
:numref:`build_asp_doc`). Add to the release notes a link to the appropriate
NEWS section of the documentation (:numref:`news`). *Only after all this save
the release.* 

Zenodo link for the release
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Saving the release will trigger creating the Zenodo permanent link. That link
cannot be changed, and the text it copies from the release notes cannot be
changed either.

It is very important to create the release just once for the given tag. Otherwise,
the Zenodo link will be messed up. It is fine to later overwrite the binaries
for this release, or even to upload them later. Just do not delete and recreate
the release.

Do not just rename the latest automatically uploaded daily build, as that will
create an incorrect Zenodo link.

Wait a few minutes for Zenodo to mint the link for this release, then visit the
Zenodo page at https://zenodo.org/badge/latestdoi/714891 (which will always
point to the 'latest' DOI) and find there this release's URL. Put it at the
top of README.rst, in the appropriate ``target`` field. Increment the version in
the ``image`` field right above that. 

Add this link also to the NEWS.rst page, under the section name for the current
release.

Push this update to GitHub. The new commit will be after the tag for the
release, but that is not an issue. It is best to not change the tag after
the release and Zenodo link got created.

Updating the release from the command line
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The GitHub tool ``gh`` can be used to push files to a release. 
Here's an example usage::

  cd BinaryBuilder
  for file in StereoPipeline-3.3.0-Linux.tar.bz2 \
              StereoPipeline-3.3.0-OSX.tar.bz2   \
              asp_book.pdf; do 
    gh release upload 3.3.0 $file \
      -R git@github.com:NeoGeographyToolkit/StereoPipeline.git   
  done

As before, do not delete and recreate the release, but it is fine
to delete and re-upload the binaries and documentation.

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

If version 3.3.0 just got released, we expect that the next feature
release will be 3.4.0, if a major release, or 3.3.1 if a minor
release. So, the version tag should be updated to 3.3.1-alpha in
anticipation (see https://semver.org for guidance).

Ensure the nightly build and regression (:numref:`nightly`) scripts are modified
to use the latest dependencies.

.. _nightly:

Nightly regression
------------------

The script ``auto_build/launch_master.sh`` in ``BinaryBuilder`` is used to
build and test ASP nightly. If these succeed, the produced daily build is 
automatically uploaded to the GitHub release page.

This script and also ``auto_build/utils.sh`` need to be read carefully and some
variables adjusted.
