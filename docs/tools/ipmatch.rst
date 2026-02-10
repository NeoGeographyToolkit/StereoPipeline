.. _ipmatch:

ipmatch
-------

The ``ipmatch`` program reads interest points (IPs) from ``.vwip`` files
produced with ``ipfind`` (:numref:`ipfind`) and finds matches among them,
writing out ``.match`` files containing the results. It can also produce debug
images with plotted match points. 

If more than two image/``.vwip`` sets are passed in, each possible combination of
images will be matched.

Example
~~~~~~~

::

    ipmatch input/image1.tif input/image2.tif -o output/run

This will read the ``input/image1.vwip`` and ``input/image2.vwip`` files created
by ``ipfind`` and write the match file::

    output/run-image1__image2.match
    
If there are more images, then all combinations of matches will be written out.

These binary files can be inspected with ``stereo_gui``
(:numref:`stereo_gui_pairwise_matches`), and can be converted to plain text with
``parse_match_file.py`` (:numref:`parse_match_file`).

The ``.vwip`` files can be specified explicitly, after the image files.

Note that this tool does not implement many of the IP matching steps that are
used in :ref:`parallel_stereo` and :ref:`bundle_adjust`, since it does not use
any camera information.

.. _ipmatch_convert:

Match file format conversion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Convert a binary match file to plain text::

    ipmatch --binary-to-txt input.match output.txt

The format of the plain text file is described in :numref:`txt_format`.

Convert a plain text match file to binary::

    ipmatch --txt-to-binary input.txt output.match

.. _ipmatch_merge:

Merging match files
~~~~~~~~~~~~~~~~~~~

Merge multiple match files for the same image pair into one::

    ipmatch --merge-match-files input1.match input2.match output.match

Duplicates are identified by left interest point coordinates. The output can
be saved as plain text by adding ``--matches-as-txt``.

Naming convention
~~~~~~~~~~~~~~~~~

The naming convention for the output binary files is::

    <output prefix>-<image1>__<image2>.match
    
where the image names are without the directory name and extension. See
:numref:`ba_match_files` for more details.

Usage
~~~~~

Usage::

     ipmatch [options] <images> <vwip files>

Command-line options
~~~~~~~~~~~~~~~~~~~~

Command-line options for ipmatch:

--output-prefix <string (default: "")>
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

--matches-as-txt
    Read and write match files as plain text instead of binary. See
    :numref:`txt_match`.

--binary-to-txt
    Read a binary match file and write it as plain text. The input and output
    match files must be specified with appropriate extensions.

--txt-to-binary
    Read a plain text match file and write it as binary. The input and output
    match files must be specified with appropriate extensions.

--merge-match-files
    Given several match files for the same image pair, merge them into one.
    Duplicates are removed based on left interest point coordinates.

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
