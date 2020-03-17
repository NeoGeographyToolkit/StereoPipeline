.. _ipmatch:

ipmatch
-------

The ``ipmatch`` reads in interest points (IPs) from .vwip files and
attempts to match them, writing out .match files containing these
results. Other ASP tools can read in these files. ``ipmatch`` also
produces debug images which can be useful. Note that this tool does not
implement many of the IP matching steps that are used in ``stereo_pprc``
and ``stereo_corr`` since it does not use any sensor model information.

If more than two image/vwip sets are passed in, each possible
combination of images will be matched.

Usage::

     ipmatch [options] <image 1> <vwip file 1> <image 2> <vwip file 2> ...

Command-line options for ipmatch:

-h, --help
    Display the help message.

--output-prefix <filepath>
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
