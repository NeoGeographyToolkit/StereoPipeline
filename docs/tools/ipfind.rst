ipfind
------

The ``ipfind`` tool detects interest points (IPs) in images and writes
them out to .vwip files. ASP is able to read these files to recover the
IPs.

This tool is useful in testing out different IP detection settings and
visualizing them (using the option ``--debug-image``).

One can pass multiple input images to the tool and they will be
processed one after another.

This program works in conjunction with ``ipmatch`` (:numref:`ipmatch`)
to match interest points across images.

Usage::

     ipfind [options] <images>

Command-line options for ipfind:

--interest-operator <sift|orb|OBALoG|LoG|Harris|IAGD (default: sift)>
    Choose an interest point detector.

--descriptor-generator <sift|orb|sgrad|sgrad2|patch|pca (default: sift)>
    Choose a descriptor generator. Some descriptors work only with
    certain interest point operators (for example, for 'OBALoG' use
    'sgrad', 'sgrad2', 'patch', and 'pca').

--ip-per-image <integer>
    Set the maximum number of IP to find in the whole image. If not
    specified, use instead the value of ``--ip-per-tile``.

-t, --tile-size <integer (default: 256)>
    The tile size for processing interest points. Useful when working
    with large images.

--ip-per-tile <integer (default: 250)>
    Set the maximum number of IP to find in each tile.

-g, --gain <float (default: 1)>
    Increasing this number will increase the gain at which interest
    points are detected.

--single-scale
    Turn off scale-invariant interest point detection. This option
    only searches for interest points in the first octave of the
    scale space.  Harris and LoG only.

--no-orientation
    Turn off rotational invariance.

--normalize
    Normalize the input. Use for images that have non-standard
    values such as ISIS cube files.

--per-tile-normalize
    Individually normalize each processing tile.

--nodata-radius <integer (default: 1)>
    Don’t detect IP within this many pixels of image borders or
    nodata.

--output-folder <string>
    Write output files to this location.

--num-threads <integer (default: 0)>
    Set the number of threads for interest point detection. If set
    to 0 (default), use the visionworkbench default number of
    threads.

-h, --help
    Display this help message.

-d, --debug-image <0|1|2 (default: 0)>
    Write out a low-resolution or full-resolution debug image with
    interest points on it if the value of this flag is respectively
    1 or 2. The default (0) is to do nothing.

--print-ip <integer (default: 0)>
    Print information for this many detected IP.

--lowe
    Save the interest points in an ASCII data format that is
    compatible with the Lowe-SIFT toolchain.
