.. _colormap:

colormap
--------

The ``colormap`` tool reads a DEM and writes a corresponding color-coded
height image that can be used for visualization.

Usage::

    colormap [options] <input DEM>

Command-line options for ``colormap``:

--help
    Display a help message.

-s, --shaded-relief-file <filename>
    Specify a shaded relief image (grayscale) to apply to the
    colorized image.

-o, --output-file <filename>
    Specify the output file.

--colormap-style <arg>
    Specify the colormap style.  Options: binary-red-blue (default),
    jet, or the name of a file having the colormap, similar to the
    file used by gdaldem.

--nodata-value <arg>
    Remap the DEM default value to the min altitude value.

--min <arg>
    Minimum height of the color map.

--max <arg>
    Maximum height of the color map.

--moon
    Set the min and max height to good values for the Moon.

--mars
    Set the min and max height to good values for Mars.

--legend
    Generate an unlabeled legend, will be saved as ``legend.png``.
