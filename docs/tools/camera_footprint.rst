.. _camera_footprint:

camera_footprint
----------------

The tool ``camera_footprint`` computes what the footprint of in image
would be if map projected on to a provided datum or DEM and prints it to
the screen. If a KML output path is provided it will also create a KML
file containing the footprint. The KML will show a box with an X pattern
showing the points ASP used to compute the footprint. This tool can be
useful for debugging camera orientations or getting a quick overview of
where images are located.

Usage::

     camera_footprint [options] <camera-image> <camera-model>

Command-line options for camera_footprint:

-h, --help
    Display the help message.

--dem-file <filename>
    Intersect with this DEM instead of a datum.

--datum <string>
    Use this datum to interpret the heights. Options are: WGS_1984,
    D_MOON, D_MARS, and MOLA.

--t_srs <proj string>
    Specify the georeference projection (PROJ or WKT) string.

-t, --session-type
    Select the stereo session type to use for processing. Normally
    this is autodetected.

--bundle-adjust-prefix <string>
    Use the camera adjustment obtained by previously running
    bundle_adjust with this output prefix.

--output-kml <string>
    Write an output KML file at this location.

--quick
    Use a faster but less accurate computation.
