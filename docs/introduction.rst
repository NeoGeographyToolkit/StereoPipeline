
Introduction
============

The NASA Ames Stereo Pipeline (ASP) is a suite of free and open
source automated geodesy and stereogrammetry tools designed for
processing stereo images captured from satellites (around Earth and
other planets), robotic rovers, aerial cameras, and historical
images, with and without accurate camera pose information. It
produces cartographic products, including digital terrain models
(DTMs), ortho-projected images, 3D models, and bundle-adjusted
networks of cameras. ASP's data products are suitable for science
analysis, mission planning, and public outreach.

.. figure:: images/introduction/p19view2_400px.png
   :alt: 3D model of Mars

   This 3D model was generated from a image pair M01/00115 and E02/01461
   (34.66N, 141.29E). The complete stereo reconstruction process takes
   approximately thirty minutes on a 3.0 GHz workstation for input
   images of this size (1024 |times| 8064 pixels). This model,
   shown here without vertical exaggeration, is roughly 2 km wide in the
   cross-track dimension. 

Background
----------

The Intelligent Robotics Group (IRG) at the NASA Ames Research
Center has been developing 3D surface reconstruction and visualization
capabilities for planetary exploration for more than a decade. First
demonstrated during the Mars Pathfinder Mission, the IRG has delivered
tools providing these capabilities to the science operations teams
of the :term:`MPL` mission, the :term:`MER` mission, the :term:`MRO`
mission, and the :term:`LRO` mission. A critical component technology
enabling this work is the ASP. The Stereo Pipeline generates high
quality, dense, texture-mapped 3D surface models from stereo image
pairs. In addition, ASP provides tools to perform many other
cartography tasks including map projection, point cloud and DEM
registration, automatic registration of cameras, data format
conversion, and data visualization.

Although initially developed for ground control and scientific
visualization applications, the Stereo Pipeline has evolved to address
orbital stereogrammetry and cartographic applications. In particular,
long-range mission planning requires detailed knowledge of planetary
topography, and high resolution topography is often derived from stereo
pairs captured from orbit. Orbital mapping satellites are sent as
precursors to planetary bodies in advance of landers and rovers. They
return a wealth of images and other data that helps mission planners and
scientists identify areas worthy of more detailed study. Topographic
information often plays a central role in this planning and analysis
process.

Our recent development of the Stereo Pipeline coincides with a
period of time when NASA orbital mapping missions are returning
orders of magnitude more data than ever before. Data volumes from
the Mars and Lunar Reconnaissance Orbiter missions now measure in
the tens of terabytes.  There is growing consensus that existing
processing techniques, which are still extremely human intensive
and expensive, are no longer adequate to address the data processing
needs of NASA and the Planetary Science community. To pick an example
of particular relevance, the :term:`HiRISE` instrument has captured
a few thousand stereo pairs. Of these, only about two hundred stereo
pairs have been processed to date; mostly on human-operated, high-end
photogrammetric workstations. It is clear that much more value could
be extracted from this valuable raw data if a more streamlined,
efficient process could be developed.

The Stereo Pipeline was designed to address this very need. By applying
recent advances in computer vision, we have created an *automated*
process that is capable of generating high quality DTMs with minimal human
intervention. Users of the Stereo Pipeline can expect to spend some time
picking a handful of settings when they first start processing a new
type of image, but once this is done, the Stereo Pipeline can be used to
process tens, hundreds, or even thousands of stereo pairs without
further adjustment. With the release of this software, we hope to
encourage the adoption of this tool chain at institutions that run and
support these remote sensing missions. Over time, we hope to see this
tool incorporated into ground data processing systems alongside other
automated image processing pipelines. As this tool continues to mature,
we believe that it will be capable of producing digital elevation models
of exceptional quality without any human intervention.

Human vs. Computer: When to Choose Automation?
----------------------------------------------

When is it appropriate to choose automated stereo mapping over the use
of a conventional, human-operated photogrammetric workstation? This is a
philosophical question with an answer that is likely to evolve over the
coming years as automated data processing technologies become more
robust and widely adopted. For now, our opinion is that you should
*always* rely on human-guided, manual data processing techniques for
producing mission critical data products for missions where human lives
or considerable capital resources are at risk. In particular, maps for
landing site analysis and precision landing absolutely require the
benefit of an expert human operator to eliminate obvious errors in the
DEMs, and also to guarantee that the proper procedures have been
followed to correct satellite telemetry errors so that the data have the
best possible geodetic control.

When it comes to using DTMs for scientific analysis, both techniques have
their merits. Human-guided stereo reconstruction produces DTMs of
unparalleled quality that benefit from the intuition and experience of
an expert. The process of building and validating these DTMs is
well-established and accepted in the scientific community.

However, only a limited number of DTMs can be processed to this level of
quality. For the rest, automated stereo processing can be used to
produce DTMs at a fraction of the cost. The results are not necessarily less
accurate than those produced by the human operator, but they will not
benefit from the same level of scrutiny and quality control. As such,
users of these DTMs must be able to identify potential issues, and be on the
lookout for errors that may result from the improper use of these tools.

We recommend that all users of the Stereo Pipeline take the time to
thoroughly read this documentation and build an understanding of how
stereo reconstruction and bundle adjustment can be best used together to
produce high quality results. You are welcome to contact us if you have
any questions (:numref:`get-help`).

Software Foundations
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

For processing non-terrestrial NASA satellite images, Stereo Pipeline
must be installed alongside a copy of the Integrated Software for
Imagers and Spectrometers (:term:`ISIS`). ISIS is however not required for
processing terrestrial images (DigitalGlobe/Maxar WorldView, etc.).

ISIS is widely used in the planetary science community for processing raw
spacecraft images into high level data products of scientific interest
such as map-projected and mosaicked images
:cite:`2004LPI.35.2039A,1997LPI.28.387G,ISIS_website`.
We chose ISIS because (1) it is widely adopted by the planetary science
community, (2) it contains the authoritative collection of geometric
camera models for planetary remote sensing instruments, and (3) it is
open source software that is easy to leverage.

By installing the Stereo Pipeline, you will be adding an advanced stereo
image processing capability that can be used in your existing ISIS workflow.
The Stereo Pipeline supports the ISIS cube (``.cub``) file format, and can
make use of the camera models and ancillary information (i.e. SPICE
kernels) for imagers on many NASA spacecraft. The use of this single
standardized set of camera models ensures consistency between products
generated in the Stereo Pipeline and those generated by ISIS. Also by
leveraging ISIS camera models, the Stereo Pipeline can process stereo pairs
captured by just about any NASA mission.


.. _get-help:

Getting Help and Reporting Bugs
-------------------------------

All bugs, feature requests, and general discussion should be posted on
the ASP support forum:

    https://groups.google.com/forum/#!forum/ames-stereo-pipeline-support

To contact the developers and project manager directly, send an email
to:

    stereo-pipeline-owner@lists.nasa.gov

When you submit a bug report, it may be helpful to attach the logs
output by ``stereo`` and other tools (:numref:`logging`).

Typographical Conventions
-------------------------

Names of programs that are meant to be run on the command line are
written in a constant-width font, like the ``stereo`` program, as are
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
              [-o <output-filename>] <pointcloud>-PC.tif

The above indicates a run of the ``point2dem`` program. The only
argument that it requires is a point cloud file, which is produced by
the ``stereo`` program and ends in ``-PC.tif``, although its prefix
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

Referencing the Ames Stereo Pipeline in Your Work
-------------------------------------------------

In general, please use this reference for the Ames Stereo Pipeline:

  Beyer, Ross A., Oleg Alexandrov, and Scott McMichael. 2018. The Ames
  Stereo Pipeline: NASA’s open source software for deriving and processing
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

If you publish a paper using ASP, please let us know. We’ll cite your
work in this document, in :numref:`papersusingasp`.


Warnings to Users of the Ames Stereo Pipeline
---------------------------------------------

Ames Stereo Pipeline is a **research** product. There may be bugs or
incomplete features. We reserve the ability to change the API and
command line options of the tools we provide. Although we hope you will
find this release helpful, you use it at your own risk. Please check
each release’s **NEWS** file to see a summary of our recent changes.

While we are confident that the algorithms used by this software are
robust, the Ames Stereo Pipeline has a lot of adjustable parameters, and
even experienced operators can produce poor results. We *strongly
recommend* that if you have any concerns about the products that you (or
others) create with this software, please just get in contact with us.
We can help you figure out either how to make the product better, or
help you accurately describe the limitations of the data or the data
products, so that you can use it to confidently make new and wonderful
discoveries.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
