.. _sfs_blend:

sfs_blend
---------

The ``sfs_blend`` tool is a very specialized DEM blending program
developed for use in conjunction with Shape-from-Shading
(:numref:`sfs`). It replaces in an SfS-produced DEM height values that
are in permanent shadow with values from the initial guess DEM used
for SfS (which is typically the LOLA gridded DEM), with a transition
region between the two DEMs.

Motivation and an example of an invocation of this tool are given in
the :ref:`SfS usage <sfs_usage>` chapter.

Command-line options:

--sfs-dem <arg>
    The SfS DEM to process.

--lola-dem <arg>
    The LOLA DEM to use to fill in the regions in permanent shadow.

--max-lit-image-mosaic <arg>   
    The maximally lit image mosaic to use to determine the permanently
    shadowed regions.

--image-threshold <float>
    The value separating permanently shadowed pixels from lit pixels
    in the maximally lit image mosaic.

--lit-blend-length <float>
    The length, in pixels, over which to blend the SfS and LOLA DEMs
    at the boundary of the permanently shadowed region towards the lit
    region. Because of the weight blur below, the effective blending
    length can be somewhat bigger than this.

--shadow-blend-length <float>
    The length, in pixels, over which to blend the SfS and LOLA DEMs
    at the boundary of the permanently shadowed region towards the
    shadowed region. Because of the weight blur below, the effective
    blending length can be somewhat bigger than this.

--weight-blur-sigma <float (default 0)> 
    The standard deviation of the Gaussian used to blur the weight
    that performs the transition from the SfS to the LOLA DEM. A
    higher value results in a smoother transition (this does not
    smooth the DEMs). The extent of the blur is about 7 times this
    deviation though it tapers fast to 0 before that. Set to 0 to not
    use this operation.

--min-blend-size <int (default 0)>
    Do not apply blending in shadowed areas of dimensions less than
    this, hence keeping there the SfS DEM.

--output-dem <arg>
    The blended output DEM to save.

--output-weight <arg>
    The weight showing the proportion of the SfS DEM in the blend with
    the LOLA DEM (1 is for purely SfS and 0 is for purely LOLA).

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
