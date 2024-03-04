.. _geodiff:

geodiff
-------

The ``geodiff`` program takes as input two DEMs, or a DEM and a CSV file, and
subtracts the second from the first. The grid is from the first DEM, so the
second one is interpolated into it using bilinear interpolation. When one file
is a CSV, the grid from the DEM is used, regardless of the order of inputs. 

It is important to note that ``geodiff`` is very sensitive to the order of
the two DEMs, due to the fact that the grid comes from the first one.
Ideally the grid of the first DEM would be denser than the one of the
second.

Usage::

    geodiff [options] <dem1> <dem2> [ -o output_file_prefix ]

Examples
~~~~~~~~

Take the absolute difference of two DEMs::

    geodiff --absolute dem1.tif dem2.tif -o run
 
This will create ``run-diff.tif``.

The ``stereo_gui`` program (:numref:`colorize`) can colorize on-the-fly and
display the difference image. The ``colormap`` program (:numref:`colormap`) can
write a colorized image.

Take the difference of a DEM and a CSV file::

    geodiff dem1.tif file.csv                         \
      --csv-format '1:lon 2:lat 3:height_above_datum' \
      -o run

The produced ``run-diff.csv`` file can be visualized and colorized with
``stereo_gui`` (:numref:`plot_csv`), as::

  stereo_gui --colorbar run-diff.csv

This program can also overlay the difference on top of the DEM.

See also
~~~~~~~~

The ``image_calc`` program (:numref:`image_calc`) can perform many operations on
images, as long as they are the same size (use ``gdalwarp``,
:numref:`gdal_tools`, to convert images to the same extent and georeference).

Command-line options
~~~~~~~~~~~~~~~~~~~~

-o, --output-prefix <filename>
    Specify the output prefix.

--absolute
    Output the absolute difference as opposed to just the difference.

--float
    Output using float (32 bit) instead of using doubles (64 bit).

--csv-format <string>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center), 
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-proj4``; the height above datum is in
    meters).  Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-proj4 <proj string>
    The PROJ.4 string to use to interpret the entries in input CSV
    files, if those files contain Easting and Northing fields. If
    not specified, it will be borrowed from the DEM.

--nodata-value <float (default: -32768)>
    The no-data value to use, unless present in the DEM geoheaders.

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
