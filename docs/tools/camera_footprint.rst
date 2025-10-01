.. _camera_footprint:

camera_footprint
----------------

The tool ``camera_footprint`` computes what the footprint of in image would be
if mapprojected onto the provided datum or DEM. 

The bounding box of the footprint is printed to the screen. It will be in units
of the DEM projection, or in longitude-latitude if a datum is provided and
``--t_srs`` is not set. The estimated ground sample distance will be printed as
well, in the same units.

If a KML output path is provided it will also create a KML file containing the
footprint. The KML will show a box with an X pattern displaying the points ASP
used to compute the footprint. If the provided DEM is smaller than the image
footprint, more additional point samples may be drawn at the DEM edges
within the image footprint.

This tool can be useful for debugging camera orientations or getting a quick
overview of where images are located.

Example::

    camera_footprint     \
      --dem-file dem.tif \
      image.tif          \
      camera.tsai        \
      --output-kml footprint.kml

To mapproject onto the datum, use an option such as ``--datum WGS_1984``.

Usage::

     camera_footprint [options] <camera-image> <camera-model>

Command-line options for camera_footprint:

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

-h, --help
    Display the help message.

