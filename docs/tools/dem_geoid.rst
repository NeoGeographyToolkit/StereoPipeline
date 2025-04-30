.. _dem_geoid:

dem_geoid
---------

This tool takes as input a DEM whose height values are relative to the datum
ellipsoid, and adjusts those values to be relative to the equipotential surface
of the planet (geoid on Earth, areoid on Mars, etc.).

This entails subtracting from a DEM height relative to an ellipsoid a correction
value. The program corrects, if need be, for differences between the DEM and
geoid correction datum ellipsoids. Bicubic interpolation is used.

This program can also perform the reverse adjustment.

Supported geoids
~~~~~~~~~~~~~~~~~
Three geoids and one areoid are supported (also custom geoids). The Earth
geoids are EGM96 and EGM2008, relative to the WGS84 datum ellipsoid (see the
`NGA Office of Geomatics
<https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84>`_), and `NAVD88
<https://www.ngs.noaa.gov/GEOID/GEOID09/>`_, which is relative to the NAD83
ellipsoid.

The Mars areoid is `MOLA MEGDR
<https://pds-geosciences.wustl.edu/mgs/mgs-m-mola-5-megdr-l3-v1/mgsl_300x/meg016/>`_.
When importing it into ASP, we adjusted the areoid height values to be relative
to the IAU reference spheroid for Mars of radius 3,396,190 m. The areoid at that
source was relative to the Mars radius of 3,396,000 m. Yet ``dem_geoid`` can
adjust correctly Mars DEMs created in respect to either spheroid.

Custom geoids
~~~~~~~~~~~~~

The ``dem_geoid`` program can work with a provided geoid correction GeoTiff
file, with the option ``--geoid-path`` (:numref:`dem_geoid_cmd_opts`).

As an example, this `Moon geoid correction
<https://github.com/NeoGeographyToolkit/StereoPipeline/releases/download/geoid1.0/gggrx_1200b_meDE430_L002_L900_16ppd.tif>`_,
from :cite:`goossens2020high`, has been adapted to work with this program.

The ``dem_geoid`` program uses bicubic interpolation into the geoid correction
file. The ``gdalwarp`` program (:numref:`gdal_tools`) can be used to resample
any such file to a fine-enough resolution and with the desired interpolation
method before passing it to this program.

Examples
~~~~~~~~

Go from a DEM in respect to the WGS84 datum to one in respect
to the EGM2008 geoid::

     dem_geoid input-DEM.tif --geoid egm2008

The corrected DEM will be saved with the suffix ``-adj.tif``.

.. _dem_geoid_cmd_opts:

Command-line options for dem_geoid
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--nodata-value <float(default: -32768)>
    The value of no-data pixels, unless specified in the DEM.

--geoid <string (default: EGM96)>
    Specify the geoid to use for the given datum. For WGS84 use
    ``EGM96`` or ``EGM2008``. For Mars use ``MOLA`` or leave
    blank.  For NAD83 use ``NAVD88`` or leave blank. When not specified
    it will be auto-detected. See also ``--geoid-path``.

--geoid-path <string (default = "")>
    Specify the path to a custom GeoTiff file having the geoid correction, in
    units of meter. Values from this file will be subtracted from the DEM values
    in order to convert from ellipsoid to geoid heights. 
    
-o, --output-prefix <name>
    Specify the output file prefix.

--double
    Output using double precision (64 bit) instead of float (32 bit).

--reverse-adjustment
    Go from DEM relative to the geoid/areoid to DEM relative to the
    datum ellipsoid.

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
