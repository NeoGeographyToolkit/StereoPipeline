.. _ipmatch:

ipmatch
-------

The ``ipmatch`` program reads interest points (IPs) from ``.vwip``
files produced with ``ipfind`` (:numref:`ipfind`) and finds matches
among them, writing out ``.match`` files containing the results. 
``ipmatch`` can also produce debug images with plotted match points. 

If more than two image/vwip sets are passed in, each possible
combination of images will be matched.

The produced binary match files can be visualized in ``stereo_gui``
(:numref:`stereo_gui_view_ip`) or converted to plain text for inspection
with ``parse_match_file.py`` (:numref:`parse_match_file`).

Note that this tool does not implement many of the IP matching steps
that are used in ``parallel_stereo`` and ``bundle_adjust``, since it does not
use any camera information.

Example::

    ipmatch image1.tif image2.tif image1.vwip image2.vwip

The order of images and ``.vwip`` files is flexible.  The first image
will use the first ``.vwip`` file encountered, etc. The ``.vwip``
files can be omitted altogether, and then can be deduced from image
names. So the program can be also called as::

    ipmatch image1.tif image2.tif

Usage::

     ipmatch [options] <images> <vwip files>

Command-line options for ipmatch:

--output-prefix <file name>
    Write output files using this prefix.

--matcher-threshold <float (default: 0.8)>
    Threshold for the separation between closest and next closest
    interest points.

--distance-metric <L2|Hamming (default: L2)>
    Distance metric to use.  Hamming should only be used for binary
    types like ORB.

--ransac-constraint <similarity|homography|fundamental|none>
    RANSAC constraint type.

--inlier-threshold <float (default: 10)>
    RANSAC inlier threshold.

--ransac-iterations <integer (default: 100)>
    Number of RANSAC iterations.

--flann-method <string (default = "kmeans")>
    Choose the FLANN method for matching interest points. The default
    ``kmeans`` is slower but deterministic, while ``kdtree`` is faster but
    not deterministic (starting with FLANN 1.9.2).

--non-flann
    Use an implementation of the interest matcher that is not reliant on FLANN.

-d, --debug-image
    Set to write out debug images.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
