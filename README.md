# NASA Ames Stereo Pipeline (ASP)

The NASA Ames Stereo Pipeline (ASP) is a suite of free and open source automated geodesy and stereogrammetry tools designed for processing stereo imagery captured from satellites (around Earth and other planets), robotic rovers, aerial cameras, and historical imagery, with and without accurate camera pose information. 

ASP produces cartographic products, including digital elevation models (DEMs), ortho-projected imagery, 3D models, and bundle-adjusted networks of cameras. These data products are suitable for science analysis, mission planning, and public outreach.

## Installation
Precompiled binaries are available for latest release and latest development build. Simply download the appropriate distribution for your operating system, extract, and run the executables in the `bin` subdirectory.  

### Latest binary release (Linux and OS X)
IRG Website: http://ti.arc.nasa.gov/tech/asr/intelligent-robotics/ngt/stereo/  
Github Releases: https://github.com/NeoGeographyToolkit/StereoPipeline/releases

### Latest development build (Linux and OS X)
IRG Website: http://byss.arc.nasa.gov/stereopipeline/daily_build/
See the NEWS file for the most recent additions.

To permanently add the ASP executable subdirectory to your PATH, you can add the following line to your shell configuration (e.g., `~/.bashrc`), replacing `/path/to/StereoPipeline/bin` with the location on your filesystem:  
`export PATH=${PATH}:/path/to/StereoPipeline/bin`

### Installing from source
Given the relatively complex configuration, we do not recommend building from source for most users. Detailed instructions are available in the documentation.

#### Note for ISIS users
Please install USGS ISIS version 3.8.0 or later if you would like to
process imagery from NASA planetary missions. Users wishing to process Earth
images, such as DigitalGlobe, satellites with RPC cameras, or various
frame/pinhole cameras do not need to download anything else.

## Documentation
The primary source of ASP documentation is available as a pdf (asp_book.pdf):  
https://byss.arc.nasa.gov/stereopipeline/daily_build/asp_book.pdf

The book includes a gentle introduction to the Stereo Pipeline, documentation for each tool, and example processing workflows for many supported sensors. 

This pdf is bundled with the binary distributions, and the LaTeX files are distributed in the
docs/book subdirectory of the source archive. A copy of this document in PDF format is also available from wherever you obtained
this package.

## Support and User Community
All bugs, feature requests, user questions, and general discussion should be posted
on the ASP support forum:

https://groups.google.com/forum/#!forum/ames-stereo-pipeline-support

To contact the lead developers and project manager directly, send an
email to: stereo-pipeline-owner@lists.nasa.gov

## Credits

ASP was developed within the Autonomous Systems and Robotics area of
the Intelligent Systems Division at NASA's Ames Research Center. It
leverages the Intelligent Robotics Group's (IRG) extensive experience
developing surface reconstruction and tools for planetary exploration,
e.g., the Mars Pathfinder and Mars Exploration Rover missions, and
rover autonomy. It has also been developed in collaboration with the
Adaptive Control and Evolvable Systems (ACES) group, and draws on
their experience developing computer vision techniques for autonomous
vehicle control systems.

See the AUTHORS file for a complete list of developers.

### Citation

In general, please use this reference for the Ames Stereo Pipeline:
>Beyer, Ross A., Oleg Alexandrov, and Scott McMichael. 2018. The Ames Stereo Pipeline: NASA's open source software for deriving and processing terrain data, Earth and Space Science, 5, https://doi.org/10.1029/2018EA000409.

If you are using ASP for application to Earth images, or need a reference which details the quality of output, then we suggest also referencing:
>Shean, D. E., O. Alexandrov, Z. Moratto, B. E. Smith, I. R. Joughin, C. C. Porter, Morin, P. J. 2016. An automated, open-source pipeline for mass production of digital elevation models (DEMs) from very high-resolution commercial stereo satellite imagery. ISPRS Journal of Photogrammetry and Remote Sensing, 116. https://doi.org/10.1016/j.isprsjprs.2016.03.012.

In addition to the recommended general citations, we recommend that you cite the Zenodo digital object identifier (DOI) for the specific ASP version used for processing. Every new release of ASP will have its own unique DOI, which can be found at this URL: https://doi.org/10.5281/zenodo.598174

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3247734.svg)](https://doi.org/10.5281/zenodo.3247734)

Complete details for how to cite ASP in your published work can be found in the ASP documentation. 

## License 

See LICENSE file for the complete text

A. Copyright and License Summary

Copyright (c) 2009-2018, United States Government as represented by
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

B. Third-Party Libraries

This distribution may include some bundled third-party software as a
convenience to the user. This software, located in the "thirdparty/"
directory of the source code release, is not covered by the
above-mentioned distribution agreement or copyright. Binary releases
distribute third party software in both the "bin" and "lib"
directories. See the included documentation for detailed copyright and
license information for any third-party software or check the
THIRDPARTYLICENSES file. In addition, various pieces of ASP depend on
additional third-party libraries that the user is expected to have
installed.
