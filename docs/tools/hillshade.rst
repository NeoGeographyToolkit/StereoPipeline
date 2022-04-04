.. _hillshade:

hillshade
-------------

The ``hillshade`` tool reads in a DEM and outputs an image of that DEM
as though it were a three-dimensional surface, with every pixel shaded
as though it were illuminated by a light from a specified location.

Example::

    hillshade -a 300 -e 30 dem.tif -o hillshaded.tif

View these side-by-side::

    stereo_gui dem.tif hillshaded.tif

Command-line options for ``hillshade``:

--help
    Display a help message.

--input-file <filename>
    Explicitly specify the input file.

-o, --output-file <filename>
    Specify the output file.

--align-to-georef
    Azimuth is relative to geographic East, not +x in the image.

-a, --azimuth <number-in-degrees (default: 300)> 
    Sets the direction that the light source is coming from (in
    degrees). Zero degrees is to the right, with positive degrees
    counter-clockwise.

-e, --elevation <number-in-degrees (default: 20)>
    Set the elevation of the light source (in degrees).

-s, --scale <arg (default: 0)>
    Set the scale of a pixel (in the same units as the DTM height values).

--nodata-value <arg> 
    Remap the DEM default value to the min altitude value.

--blur <arg>
    Pre-blur the DEM with the specified sigma.
