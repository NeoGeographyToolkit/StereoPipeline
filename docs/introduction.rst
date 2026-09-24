Introduction
============

The NASA Ames Stereo Pipeline (ASP) is a suite of free and open source automated
geodesy and stereogrammetry tools designed for processing images captured from
satellites, around Earth and other planets (:numref:`examples`), robotic rovers
(:numref:`rig_msl`, :numref:`csm_msl`), aerial cameras (:numref:`sfm`), low-cost
satellites (:numref:`skysat`), and historical images (:numref:`kh4`).

It has functionality for 3D terrain creation from stereo (:numref:`tutorial`),
including shallow-water bathymetry (:numref:`bathy_intro`), alignment of point
clouds (:numref:`pc_align`), map projection (:numref:`mapproject`),
structure-from-motion
(:numref:`sfm`), shape-from-shading (:numref:`sfs_usage`), bundle adjustment
(:numref:`bundle_adjust`), solving for jitter (:numref:`jitter_solve`), rig
calibration (:numref:`rig_calibrator`), refining camera intrinsics
(:numref:`floatingintrinsics`), GCP generation (:numref:`gcp_gen`,
:numref:`dem2gcp`), and a versatile GUI shell (:numref:`stereo_gui`).

ASP produces cartographic products, including digital terrain models (DTMs) and
ortho-projected images (:numref:`builddem`), 3D models (:numref:`point2mesh`),
textured meshes (:numref:`sfm_iss`), and bundle-adjusted networks of cameras
(:numref:`control_network`).

ASP's data products are suitable for science analysis, mission planning, and
public outreach.

.. figure:: images/introduction/p19view2_400px.png
   :alt: 3D model of Mars

   This 3D model was generated from a image pair M01/00115 and E02/01461
   (34.66N, 141.29E). The complete stereo reconstruction process takes
   approximately thirty minutes on a 3.0 GHz workstation for input
   images of this size (1024 |times| 8064 pixels). This model,
   shown here without vertical exaggeration, is roughly 2 km wide in the
   cross-track dimension. 

Background
----------

ASP is developed by the Intelligent Robotics Group (IRG) at the NASA Ames
Research Center. It builds on more than two decades of planetary 3D surface
reconstruction, first demonstrated on the Mars Pathfinder mission and since
delivered to the :term:`MPL`, :term:`MER`, :term:`MRO`, and :term:`LRO`
science operations teams.

Software foundations
--------------------

NASA Vision Workbench
~~~~~~~~~~~~~~~~~~~~~

The Stereo Pipeline is built upon the Vision Workbench software which is
a general purpose image processing and computer vision library also
developed by the IRG. Some of the tools discussed in this document are
actually Vision Workbench programs, and any distribution of the Stereo
Pipeline requires the Vision Workbench. This distinction is important
only if compiling this software.

The USGS Integrated Software for Imagers and Spectrometers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For processing non-terrestrial NASA satellite images, Stereo Pipeline must be
installed alongside a copy of the Integrated Software for Imagers and
Spectrometers (:term:`ISIS`, :numref:`planetary_images`). ISIS is however not
required for processing terrestrial images (:numref:`dg_tutorial`).

ISIS is widely used in the planetary science community for processing raw
spacecraft images into high level data products of scientific interest
such as map-projected and mosaicked images
:cite:`2004LPI.35.2039A,1997LPI.28.387G,ISIS_website`.

.. _get-help:

Getting help and reporting bugs
-------------------------------

All bugs, feature requests, and general discussion should be posted on
the ASP support forum:

    https://groups.google.com/forum/#!forum/ames-stereo-pipeline-support

To contact the developers and project manager directly, send an email
to:

    stereo-pipeline-owner@lists.nasa.gov

When you submit a bug report, it may be helpful to attach the logs
output by ``parallel_stereo`` and other tools (:numref:`logging`).

Typographical conventions
-------------------------

Names of programs that are meant to be run on the command line are
written in a constant-width font, like the ``parallel_stereo`` program, as are
options to those programs.

An indented line of constant-width text can be typed into your terminal,
these lines will either begin with a '``>``' to denote a regular shell,
or with '``ISIS>``' which denotes an ISIS-enabled shell (which means you have
to set the ``ISISROOT`` environment variable and have sourced the appropriate
ISIS startup script, as detailed in the ISIS instructions).

::

    > ls

    ISIS> pds2isis

Constant-width text enclosed in greater-than and less-than brackets denotes an 
option or argument that a user will need to supply. For example,
'``stereo E0201461.map.cub M0100115.map.cub out``' is specific, but
'``stereo <left-image> <right-image> out``' indicates that ``<left-image>``
and ``<right-image>`` are not the names of specific files, but dummy
parameters which need to be replaced with actual file names.

Square brackets denote optional options or values to a command, and
items separated by a vertical bar are either aliases for each other, or
different, specific options.  Default arguments or other notes are
enclosed by parentheses, and line continuation with a backslash::

    point2dem [-h|--help] [-r moon|mars] [-s <float(default: 0.0)>] \
              [-o <output prefix>] <output prefix>-PC.tif

The above indicates a run of the ``point2dem`` program. The only
argument that it requires is a point cloud file, which is produced by
the ``parallel_stereo`` program and ends in ``-PC.tif``, although its prefix
could be anything (hence the greater-than and less-than enclosing brackets).
Everything else is in square brackets indicating that they are optional.

Here, ``--help`` and ``-h`` refer to the same thing. Similarly, the
argument to the ``-r`` option must be either ``moon`` or ``mars``. The
``-s`` option takes a floating point value as its argument, and has a
default value of zero. The ``-o`` option takes a filename that will be
used as the output DTM.

Although there are two lines of constant-width text, the backslash at
the end of the first line indicates that the command continues on the
second line. You can either type everything into one long line on your
own terminal, or use the backslash character and a return to continue
typing on a second line in your terminal.

Citing the Ames Stereo Pipeline in your work
--------------------------------------------

In general, use this reference:

  Beyer, Ross A., Oleg Alexandrov, and Scott McMichael. 2018. The Ames
  Stereo Pipeline: NASA's open source software for deriving and processing
  terrain data. *Earth and Space Science*, **5**.
  https://doi.org/10.1029/2018EA000409.

If you are using ASP for application to Earth images, or need a
reference which details the quality of output, then we suggest also
referencing:

  Shean, D. E., O. Alexandrov, Z. Moratto, B. E. Smith, I. R. Joughin, C.
  C. Porter, Morin, P. J. 2016. An automated, open-source pipeline for
  mass production of digital elevation models (DEMs) from very
  high-resolution commercial stereo satellite imagery. *ISPRS Journal of
  Photogrammetry and Remote Sensing.* **116**.

In addition to using the references above, in order to help you better
cite the specific version of ASP that you are using in a work, as of ASP
version 2.6.0, we have started using `Zenodo <https://zenodo.org>`__ to
create digital object identifiers (DOIs) for each ASP release. For
example, the DOI for version 2.6.2 is 10.5281/zenodo.3247734, and you
can cite it like this:

  Beyer, Ross A., Oleg Alexandrov, and Scott McMichael. 2019.
  NeoGeographyToolkit/StereoPipeline: Ames Stereo Pipeline version 2.6.2.
  *Zenodo*. `DOI:
  10.5281/zenodo.3247734 <https://doi.org/10.5281/zenodo.3247734>`__.

Of course, every new release of ASP will have its own unique DOI, and
this link should always point to the `latest
DOI <https://doi.org/10.5281/zenodo.598174>`__ for ASP.

If you publish a paper using ASP, please let us know. We'll cite your
work in this document, in :numref:`papersusingasp`.


Warnings to users of the Ames Stereo Pipeline
---------------------------------------------

Ames Stereo Pipeline is a **research** product. There may be bugs or
incomplete features. We reserve the ability to change the API and
command line options of the tools we provide. Although we hope you
will find this release helpful, you use it at your own risk.

While we are confident that the algorithms used by this software are
robust, the Ames Stereo Pipeline has a lot of adjustable parameters, and
even experienced operators can produce poor results. Inspect every product
before relying on it, and watch for errors from poor input data or unsuitable
settings. We *strongly recommend* that if you have any concerns about the
products that you (or others) create with this software, please just get in
contact with us.
We can help you figure out either how to make the product better, or
help you accurately describe the limitations of the data or the data
products, so that you can use it to confidently make new and wonderful
discoveries.

Please check each release's NEWS file (:numref:`news`) to see a summary of
our recent changes.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
