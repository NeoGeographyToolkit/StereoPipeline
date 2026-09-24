==========================
Ames Stereo Pipeline (ASP)
==========================

.. image:: https://zenodo.org/badge/DOI/latest.svg
   :target: https://zenodo.org/badge/latestdoi/714891

.. image:: https://zenodo.org/badge/DOI/Version%203.7.0.svg
   :target: https://zenodo.org/records/20596670

.. image:: https://readthedocs.org/projects/stereopipeline/badge/?version=latest
   :target: https://stereopipeline.readthedocs.io/en/latest/?badge=latest
   :alt: Documentation for latest version

The NASA Ames Stereo Pipeline (ASP) is a suite of free and open source automated
geodesy and stereogrammetry tools designed for processing stereo images captured
from satellites (around Earth and other planets), robotic rovers, aerial
cameras, and historical images, with and without accurate camera pose
information.

ASP supports a wide range of camera models, including linescan, frame,
pushframe, optical bar, RPC, and the USGS Community Sensor Model (CSM). It can
create cameras from scratch and refine them with bundle adjustment, solving for
pose, jitter, lens distortion, and rig calibration.

It has functionality for 3D terrain creation from stereo, including
shallow-water bathymetry, alignment of point clouds, mapprojection,
structure-from-motion, shape-from-shading, GCP generation, and a
versatile GUI shell.

ASP produces cartographic products, including digital terrain models
(DTMs), ortho-projected images, 3D models, and bundle-adjusted
networks of cameras. These data products are suitable for science
analysis, mission planning, and public outreach.

* ASP is free software released under the Apache Software License 2.0.
* Documentation: https://stereopipeline.readthedocs.io

Installation
============

Precompiled binaries for Linux (Intel and ARM64) and macOS (Intel and ARM64) are
available for the stable releases and the current development build. Download
the appropriate distribution for the operating system and architecture,
extract, and run the executables in the ``bin`` subdirectory.

- `Stable releases, daily builds, and conda packages
  <https://stereopipeline.readthedocs.io/en/latest/installation.html>`_
  
See the `NEWS
<https://stereopipeline.readthedocs.io/en/latest/news.html>`_
for the most recent additions.

To permanently add the ASP executable subdirectory to the PATH, add a
line such as this to a shell configuration file (e.g., ``~/.bashrc``):
``export PATH=${PATH}:/path/to/StereoPipeline/bin``

*ISIS users*: Please install the latest `USGS ISIS
<https://github.com/DOI-USGS/ISIS3>`_ to process NASA
non-terrestrial images. Users wishing to process Earth images, such as Digital
Globe, satellites with RPC cameras, or various frame/pinhole cameras do not need
to download anything else. If ASP is installed with conda, it will install ISIS
in the same environment as well, though it may not be the latest version.

Documentation
=============

The documentation, in HTML format, is at https://stereopipeline.readthedocs.io.

The documentation includes a gentle introduction to using the Stereo Pipeline,
an entry for each tool, and example processing workflows for many supported
sensors.

Support and user community
==========================

All bugs, feature requests, user questions, and general discussion
can be posted on the `ASP support forum
<https://groups.google.com/forum/#!forum/ames-stereo-pipeline-support>`_.

We also encourage the posting of issues on the `GitHub repo
<https://github.com/NeoGeographyToolkit/StereoPipeline>`_, as well as
pull requests.

Credits
=======

ASP was developed within the Autonomous Systems and Robotics area of
the Intelligent Systems Division at NASA's Ames Research Center. It
leverages the Intelligent Robotics Group's (IRG) extensive experience
developing surface reconstruction and tools for planetary exploration
(e.g., the Mars Pathfinder and Mars Exploration Rover missions, and
rover autonomy). It has also been developed in collaboration with the
Adaptive Control and Evolvable Systems (ACES) group, and draws on
their experience developing computer vision techniques for autonomous
vehicle control systems.

See the `list of contributors
<https://stereopipeline.readthedocs.io/en/latest/acknowledgements.html>`_.

Citation
--------

In general, please use this reference for the Ames Stereo Pipeline:

  Beyer, Ross A., Oleg Alexandrov, and Scott McMichael. 2018. The
  Ames Stereo Pipeline: NASA's open source software for deriving and
  processing terrain data, Earth and Space Science, 5.
  https://doi.org/10.1029/2018EA000409.

For application to Earth images, or for a reference detailing the quality
of the output, also cite:

  Shean, D. E., O. Alexandrov, Z. Moratto, B. E. Smith, I. R. Joughin,
  C. C. Porter, Morin, P. J. 2016. An automated, open-source pipeline
  for mass production of digital elevation models (DEMs) from very
  high-resolution commercial stereo satellite imagery. ISPRS Journal
  of Photogrammetry and Remote Sensing, 116.
  https://doi.org/10.1016/j.isprsjprs.2016.03.012.

In addition to the recommended citation, please also cite the DOI for
the specific version of ASP used for processing. Every new release (and
daily build) of ASP has its own unique DOI, which can be found `here
<https://doi.org/10.5281/zenodo.598174>`_.

Additional details for citing ASP in published work are in the ASP
documentation.

License
=======

See LICENSE file for the full text of the license that applies to ASP.

Copyright (c) 2009-2026, United States Government as represented by
the Administrator of the National Aeronautics and Space
Administration. All rights reserved.

ASP is licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License. You
may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
implied. See the License for the specific language governing
permissions and limitations under the License.

Third-party libraries
=====================

This distribution may include some bundled third-party software as a
convenience to the user. This software, located in the ``thirdparty/``
directory of the source code release, is not covered by the
above-mentioned distribution agreement or copyright. Binary releases
distribute third party software in both the ``bin`` and ``lib``
directories. See the included documentation for detailed copyright and
license information for any third-party software or check the
`THIRDPARTYLICENSES
<https://github.com/NeoGeographyToolkit/StereoPipeline/blob/master/THIRDPARTYLICENSES.rst>`_
file. In addition, various pieces of ASP depend on additional
third-party libraries that the user is expected to have installed.
