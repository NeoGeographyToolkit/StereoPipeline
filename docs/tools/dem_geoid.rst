.. _dem_geoid:

dem_geoid
---------

This tool takes as input a DEM whose height values are relative to the
datum ellipsoid, and adjusts those values to be relative to the
equipotential surface of the planet (geoid on Earth, and areoid on
Mars). The program can also apply the reverse of this adjustment. The
adjustment simply subtracts from the DEM height the geoid height
(correcting, if need be, for differences in dimensions between the DEM
and geoid datum ellipsoids).

Three geoids and one areoid are supported. The Earth geoids are EGM96 and
EGM2008, relative to the WGS84 datum ellipsoid (see the `NGA Office of Geomatics
<https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84>`_), and `NAVD88
<https://www.ngs.noaa.gov/GEOID/GEOID09/>`_, which is relative to the NAD83
ellipsoid.

The Mars areoid is `MOLA MEGDR
<https://pds-geosciences.wustl.edu/mgs/mgs-m-mola-5-megdr-l3-v1/mgsl_300x/meg016/>`_.
When importing it into ASP, we adjusted the areoid height values to be relative
to the IAU reference spheroid for Mars of radius 3,396,190 m. The areoid at that
source was relative to the Mars radius of 3,396,000 m. Yet ``dem_geoid`` can
adjust correctly Mars DEMs created in respect to either spheroid.

Example: Go from a DEM in respect to the WGS84 datum to one in respect
to the EGM2008 geoid::

     dem_geoid input-DEM.tif --geoid egm2008

This program will write a new image file with the suffix ``-adj.tif``.

Command-line options for dem_geoid:

--nodata-value <float(default: -32768)>
    The value of no-data pixels, unless specified in the DEM.

--geoid <name (default: EGM96)>
    Specify the geoid to use for the given datum. For WGS84 use
    ``EGM96`` or ``EGM2008``. For Mars use ``MOLA`` or leave
    blank.  For NAD83 use ``NAVD88`` or leave blank. When not specified
    it will be auto-detected.

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
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
