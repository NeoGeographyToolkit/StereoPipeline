.. _building_asp:

Building and releasing ASP
==========================

This chapter will describe how ASP can be built from source and with conda, how
to build the documentation, and how to prepare a new ASP release. This is
focused towards the developer. Users should read instead the installation guide
in :numref:`installation`.

.. _build_from_source:

Building ASP from source
------------------------

All dependencies for the *latest development version* of ASP are available as
a `binary tarball
<https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/>`_.

The dependencies for the latest stable version of ASP are in the 
``stereopipeline-feedstock`` repository (:numref:`packages_to_build`).

Assume that all dependencies, including the development tools, are installed
in the ``asp_deps`` conda environment and the ``PATH`` variable is set up
to use them. 

Create a work directory::

    workDir=$HOME/build_asp
    mkdir -p $workDir

Build VisionWorkbench and Stereo Pipeline version <asp version>::

    cd $workDir
    envPath=$HOME/miniconda3/envs/asp_deps
    $envPath/bin/git clone                            \
        git@github.com:visionworkbench/visionworkbench.git
    cd visionworkbench
    # Build a specific version
    git checkout <asp version>
    mkdir -p build
    cd build
    $envPath/bin/cmake ..                             \
      -DASP_DEPS_DIR=$envPath                         \
      -DCMAKE_INSTALL_PREFIX=$workDir/install
    make -j10 && make install

    cd $workDir
    envPath=$HOME/miniconda3/envs/asp_deps
    $envPath/bin/git clone                            \
    git@github.com:NeoGeographyToolkit/StereoPipeline.git
    cd StereoPipeline
    # Build a specific version
    git checkout <asp version>
    mkdir -p build
    cd build
    $envPath/bin/cmake ..                             \
      -DASP_DEPS_DIR=$envPath                         \
      -DCMAKE_INSTALL_PREFIX=$workDir/install         \
      -DVISIONWORKBENCH_INSTALL_DIR=$workDir/install
    make -j10 && make install

Check that the compilers were picked up correctly.

.. _conda_build:

Building ASP and its dependencies with conda
--------------------------------------------

This page is meant for advanced users of ASP and maintainers who would
like to use conda to rebuild ASP and all its dependencies. It is
suggested to carefully read :numref:`conda_intro` before this page.

To simplify maintenance, ASP and its dependencies are built upon ISIS
and its dependencies. The process for this is outlined below.

Setting up the ISIS environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Search for the latest available ISIS conda package::
  
    conda search -c usgs-astrogeology --override-channels isis

Use the latest version found above. See :numref:`planetary_images` for more
about ISIS and how to fetch its data.

Create and activate a conda environment for ISIS::

     conda create -n isis
     conda activate isis

Add these channels to conda::

    conda config --env --add channels conda-forge
    conda config --env --add channels usgs-astrogeology

Run::

    conda config --show channels

and verify that ``usgs-astrogeology`` and ``conda-forge`` are in this
order and above all other channels, except perhaps the
``nasa-ames-stereo-pipeline`` channel.

Install ISIS::

    conda install                 \
      -c usgs-astrogeology        \
      -c conda-forge              \
      --channel-priority flexible \
      isis

Flexible channel priority may be necessary for successful installation.

Save the current environment for reference as follows::

    conda env export > isis.yaml

Build tools
~~~~~~~~~~~

We will create a new ``tools`` environment to have all the tools we
will need. These could be appended to the earlier environment, but
keeping them separate is less likely to cause dependency conflicts.

::

    conda create -n tools
    conda activate tools
    conda install -c conda-forge anaconda-client conda-build

.. _packages_to_build:

Build recipe
~~~~~~~~~~~~

ASP has many dependencies that are source code, rather than pre-existing
packages.

Producing a separate conda package for each turned out to be laborious,
because conda is slow and fragile. The current approach is to build all these
packages and ASP itself in a single script, available at

  https://github.com/NeoGeographyToolkit/stereopipeline-feedstock

To reduce the chance of failures, the process is first tested by building these
packages manually, using the same steps and environment as in the script.

That environment is produced by adding dependencies to the installed ISIS
package. 

The ASP version in this feedstock needs to be updated for each release.

Build command::

  conda activate tools
  conda config --set channel_priority flexible
  conda build                    \
    -c nasa-ames-stereo-pipeline \
    -c usgs-astrogeology         \
    -c conda-forge               \
    stereopipeline-feedstock

The developers can upload the produced packages to the
``nasa-ames-stereo-pipeline`` channel.

After a package is uploaded, it can be installed in the desired environment as::

    conda install                  \
      -c nasa-ames-stereo-pipeline \
      -c usgs-astrogeology         \
      -c conda-forge               \
      -n myEnv                     \
      myPackage=myVersion=myBuildNo

If this is slow, check if the solver is set to ``libmamba``. 
 
To list all packages in the channel, do::

    conda search -c nasa-ames-stereo-pipeline --override-channels

To delete a package from this channel, run::

    anaconda remove nasa-ames-stereo-pipeline/myPackage/myVersion
  
If adding an updated package with the same version, increment the build number.
Otherwise the new package may be confused with a cached version of a prior
build.

.. _helper_scripts:

Helper scripts
~~~~~~~~~~~~~~

The ``.github/workflows`` directory in the ``StereoPipeline`` repository has a
few scripts that show in detail the commands that are run to build ASP daily (on
Mac, in the cloud).

.. _build_asp_doc:

Building the documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~

The ASP documentation is written in ReStructured Text and is built
with `Sphinx <https://www.sphinx-doc.org>`_ and
`sphinxcontrib-bibtex <https://sphinxcontrib-bibtex.readthedocs.io>`_.

See the `online ASP documentation
<https://stereopipeline.readthedocs.io/en/latest/>`_.

To build the documentation locally, install the required packages, for example::

    conda create -n sphinx -c conda-forge sphinx sphinxcontrib-bibtex
    conda activate sphinx

Note that we used a separate conda environment to minimize the chance
of conflict with other dependencies.

The ``docs`` directory contains the root of the documentation. Running
``make html`` there will create the HTML version of the documentation in the
``_build`` subdirectory.

Building the PDF documentation is no longer supported. 

If the documentation builds well locally but fails to update on the web, see the
`cloud build status page
<https://readthedocs.org/projects/stereopipeline/builds/>`_.

.. _asp_release_guide:

Releasing a new version of ASP
------------------------------

This section is for ASP maintainers.

Update the version number
~~~~~~~~~~~~~~~~~~~~~~~~~

Inside *both* the VisionWorkbench and ASP repositories, edit
``src/CMakeLists.txt`` and set the new version, which should be the same for
both packages, and in the format ``x.y.z``. If the value there is
``x.y.z-alpha``, which is used to tag a pre-release, remove the ``-alpha`` part.
Increment one of these digits, depending on whether this is a major, minor, or
bugfix release. See https://semver.org for guidance.

Update the documentation
~~~~~~~~~~~~~~~~~~~~~~~~

Search all documentation for the old version numbers for ASP and ISIS and
replace them with the new version numbers. This includes files in the base
directory, not just in ``docs``.

Update NEWS.rst. Add the release date on top, along the lines of prior releases
(see further down in that file). This file must have a detailed log of all
changes, especially those that result in changed behavior or options, and it
should be incrementally updated as changes are made during development.

Update the copyright year in the README.rst and docs/conf.py files.

Commit and tag
~~~~~~~~~~~~~~

Commit all changes. Tag the release in *both* the VisionWorkbench and
StereoPipeline repos. Example:: 

  git tag <asp version>
  git push origin <asp version> # commit to your branch
  git push god    <asp version> # commit to main branch

(Here it is assumed that ``origin`` points to your own fork and ``god``
points to the parent repository.)

If more commits were made and it is desired to apply this tag to a
different commit, first remove the existing tag with::

  git tag -d <asp version>
  git push origin :refs/tags/<asp version>
  git push god    :refs/tags/<asp version>

Build ASP with conda
~~~~~~~~~~~~~~~~~~~~

See :numref:`conda_build`. 
    
.. _build_binaries:

Building self-contained binaries
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In addition to creating a conda package, it is also convenient to ship a
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
tool, as outlined on that page. The full list of dependencies is in the 
``stereopipeline-feedstock`` repository (:numref:`packages_to_build`).

Go to the directory ``BinaryBuilder``, and run::

    /path/to/python3                                \
      ./build.py                                    \
      --cc <path to C compiler>                     \
      --cxx <path to C++ compiler>                  \
      --gfortran <path to Fortran compiler>         \
      --asp-deps-dir $HOME/miniconda3/envs/asp_deps \
      --build-root build_asp                        \
      --skip-tests                                  \
      visionworkbench stereopipeline

This will fetch and build the latest VisionWorkbench and Stereo Pipeline in
``build_asp/build``, and will install them in ``build_asp/install``.

See :numref:`helper_scripts` for scripts illustrating this process.

Create a conda environment having Python and numpy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ISIS expects a full Python distribution to be shipped. To avoid shipping
the entire ``asp_deps`` environment, we create a separate environment
having only Python and numpy, with versions as expected by current ISIS.
Run, for example::

    conda create -c conda-forge -n python_isis10 \
      python=x.y.z numpy=a.b.c

Note that different versions of these may be needed for Linux and OSX.
The ``conda list`` command within the ``asp_deps`` environment 
can be used to look up the desired versions.

Package the build
~~~~~~~~~~~~~~~~~

Run in ``BinaryBuilder`` the command::

    /path/to/python3                                  \
      ./make-dist.py build_asp/install                \
      --asp-deps-dir $HOME/miniconda3/envs/asp_deps   \
      --python-env $HOME/miniconda3/envs/python_isis10

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

The GitHub ``gh`` program can be invoked to push the binaries to the release.
Example::

  cd BinaryBuilder/asp_tarballs
  for file in                                                \
    StereoPipeline-<asp version>-<date>-x86_64-Linux.tar.bz2 \
    StereoPipeline-<asp version>-<date>-x86_64-OSX.tar.bz2   \
    StereoPipeline-<asp version>-<date>-arm64-OSX.tar.bz2; do

    gh release upload <asp version> $file \
      -R git@github.com:NeoGeographyToolkit/StereoPipeline.git
  done

Alternatively, these can be uploaded from a web browser.

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
<https://groups.google.com/forum/\#!forum/ames-stereo-pipeline-support>`_, with
a link to the NEWS section for the current release from the documentation.

Post-release work
~~~~~~~~~~~~~~~~~

In anticipation of the next feature release, increment the version number in
``src/CMakeLists.txt`` in *both* the VisionWorkbench and ASP repositories, and
append ``-alpha`` to it. See https://semver.org for guidance on versions.
