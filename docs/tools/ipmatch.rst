.. _ipmatch:

ipmatch
-------

The ``ipmatch`` program reads interest points (IPs) from ``.vwip`` files and
attempts to match them, writing out ``.match`` files containing the
results. Other ASP tools can read the obtained match
files. ``ipmatch`` also produces debug images which can be
useful. Note that this tool does not implement many of the IP matching
steps that are used in ``stereo`` and ``bundle_adjust`` since it
does not use any camera information.

If more than two image/vwip sets are passed in, each possible
combination of images will be matched.

The produced binary match files can be visualized in ``stereo_gui``
(:numref:`stereo_gui`) or converted to plain text for inspection
with ``parse_match_file.py`` (:numref:`parse_match_file`).

Example::

    ipmatch image1.tif image2.tif image1.vwip image2.vwip

The order of images and ``.vwip`` files is flexible.  The first image will 
use the first ``.vwip`` file encountered, etc. The ``.vwip`` files can be 
omitted altogether, and then can be deduced from image names. So the
program can be also called as::

    ipmatch image1.tif image2.tif

Usage::

     ipmatch [options] <images> <vwip files>

Command-line options for ipmatch:

-h, --help
    Display the help message.

--output-prefix <file name>
    Write output files using this prefix.

--matcher-threshold <float (default: 0.6)>
    Threshold for the separation between closest and next closest
    interest points.

--non-kdtree
    Use a non-KDTree version of the matching algorithm.

--distance-metric <L2|Hamming (default: L2)>
    Distance metric to use.  Hamming should only be used for binary
    types like ORB.

--ransac-constraint <similarity|homography|fundamental|none>
    RANSAC constraint type.

--inlier-threshold <float (default: 10)>
    RANSAC inlier threshold.

--ransac-iterations <integer (default: 100)>
    Number of RANSAC iterations.

-d, --debug-image
    Set to write out debug images.
