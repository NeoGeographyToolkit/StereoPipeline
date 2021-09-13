.. _conda_build:

Building ASP and its dependencies with conda
============================================

This page is meant for advanced users of ASP and maintainers who would
like to use conda to rebuild ASP and all its dependencies. It is
suggested to carefully read :numref:`conda_intro` before this page.

To simplify maintenance, ASP and its dependencies are built upon ISIS
and its dependencies. Hence, in order to create a new conda ASP
package, first one needs to create an environment having the latest
released ISIS, then rebuild ASP's other dependencies and ASP itself,
while ensuring that the dependencies of each of these have their
versions synced up with the ISIS dependency versions.

The rebuilt packages will be uploaded to ASP's anaconda channel.

Setting up the ISIS environment
-------------------------------

Search for the latest available ISIS conda package::
  
    conda search -c usgs-astrogeology --override-channels isis

Here it was found that ISIS version 5.0.1 was the latest, which we
will assume throughout the rest of this document. This needs to be
adjusted for your circumstances.

Create a conda environment for this version of ISIS::

     conda create -n isis5.0.1 python=3.6
     conda activate isis5.0.1

Add these channels to conda::

    conda config --env --add channels conda-forge
    conda config --env --add channels usgs-astrogeology

Run::

    conda config --show channels

and verify that ``usgs-astrogeology`` and ``conda-forge`` are in this
order and above all other channels, except perhaps the
``nasa-ames-stereo-pipeline`` channel.

Install the desired version of ISIS::

    conda install isis==5.0.1

Search and install the latest version of the ``usgscsm`` package,
for example, as::

    conda search -c conda-forge --override-channels usgscsm
    conda install -c conda-forge usgscsm==1.5.2

Save the current environment as follows::

    conda env export > isis5.0.1.yaml

Fetching the build tools
------------------------

We will create a new ``tools`` environment to have all the tools we
will need. These could be appended to the earlier environment, but it
is less likely to to have issues with dependency conflicts if these
are kept separate.

::

    conda create -n tools python=3.6
    conda activate tools
    conda install -c conda-forge anaconda-client conda-build \
      conda-verify cmake git

Fetch the recipes to build
--------------------------

The additional recipes that need to be built can be fetched with ``git
clone`` from::

  https://github.com/NeoGeographyToolkit/geoid-feedstock.git
  https://github.com/NeoGeographyToolkit/htdp-feedstock.git
  https://github.com/NeoGeographyToolkit/laszip-feedstock.git
  https://github.com/NeoGeographyToolkit/fgr-feedstock.git
  https://github.com/NeoGeographyToolkit/libnabo-feedstock.git
  https://github.com/NeoGeographyToolkit/libpointmatcher-feedstock.git
  https://github.com/NeoGeographyToolkit/gdal-feedstock.git
  https://github.com/NeoGeographyToolkit/liblas-feedstock.git
  https://github.com/NeoGeographyToolkit/imagemagick-feedstock.git
  https://github.com/NeoGeographyToolkit/theia-feedstock.git
  https://github.com/NeoGeographyToolkit/s2p-feedstock.git
  https://github.com/NeoGeographyToolkit/libelas-feedstock.git
  https://github.com/NeoGeographyToolkit/visionworkbench-feedstock.git
  https://github.com/NeoGeographyToolkit/stereopipeline-feedstock.git

It may be helpful to look at the ``meta.yml`` files for the
visionworkbench and stereopipeline feedstock repositories, and install
the dependencies of those packages in the isis5.0.1 environment created
earlier, except for those that we actually plan to build.

Synchronize the versions with the existing environment
------------------------------------------------------

For each of these, check the ``recipe/meta.yaml`` file and ensure all
dependencies are in sync with what is in the file ``isis5.0.1.yaml``
generated earlier. This can be done automatically with a provided
script in the ASP repository::

     python StereoPipeline/conda/update_versions.py isis5.0.1.yaml \
       gdal-feedstock

and the same for the other packages.

It is very important to note that this script is not fool-proof. For
example, the ``eigen`` version which seems to agree with the current
version of ``ceres`` is 3.3.7 rather than 3.3.9.

It is suggested to examine the changed ``meta.yaml`` with great care,
and if in doubt, leave the values as they were before modified by this
script.

In each of those files manually modify the string ``isis5.0.1`` to
reflect the current ISIS version.

In the ``visionworkbench`` and ``stereopipeline`` recipes update the
``git_tag`` value to reflect the desired commit from the Git
history. (When making an ASP release, one can tag the commit based on
which the release happens in the VisionWorkbench and StereoPipeline
repositories, and then that tag can be used in the ``git_tag`` field.)

Later on, after the packages are built and tested, ensure that all the
changes to the feedstock repositories are checked in.

Build the conda packages
------------------------

Each of the packages above can be built as follows::

    conda build -c nasa-ames-stereo-pipeline -c usgs-astrogeology \
      -c conda-forge gdal-feedstock

and then uploaded to the ``nasa-ames-stereo-pipeline`` channel by
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
package with the same name and version.

After a package is uploaded, it can be installed in the existing
``isis5.0.1`` environment as::

    conda install -c nasa-ames-stereo-pipeline \
      -c usgs-astrogeology                     \
      -c conda-forge                           \
      gdal==isis5.0.1

To list all packages in that channel, do::

    conda search -c nasa-ames-stereo-pipeline --override-channels

To delete a package from this channel, run::

    anaconda remove nasa-ames-stereo-pipeline/mypackage
  
Order of building the packages
------------------------------

It is suggested to build the above packages in the order listed
earlier, as some of them depend on others.

Note that ``libpointmatcher`` depends on ``libnabo``, while ``liblas``
depends on ``laszip`` and ``gdal``, ``theia`` depends on
``imagemagick``, and ``visionworkbench`` depends on ``gdal``. The
``stereopipeline`` package depends on all of these so it should be
built the last.


.. _compilers:

Note on compilers
-----------------

On Linux, the conda packages are set to be built with conda-provided
versions of the C, C++, and Fortran compilers. For OSX, the local
system Clang compilers are used, as the conda-provided
ones turned out to result in problems at runtime.

To install these compilers in a desired environemnt on Linux for use
without ``conda build``, do::

    conda install -c conda-forge gcc_linux-64==11.1.0 \
      gxx_linux-64==11.1.0 gfortran_linux-64==11.1.0

