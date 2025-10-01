.. _camera_footprint:

camera_footprint
----------------

The ``camera_footprint`` program computes what the footprint of an image would be
if mapprojected (:numref:`mapproject`) onto the provided DEM or datum.

The bounding box of the footprint is printed to the screen. It will be in units
of the DEM projection, or in longitude-latitude if a datum is provided. These
can be overridden with ``--t_srs``. The estimated ground sample distance will be
printed as well, in the same units.

This tool can be useful for debugging camera orientations or getting a quick
overview of where the input image is located on the ground.

Save as shapefile
~~~~~~~~~~~~~~~~~

If a shapefile output path is provided, this program will also create a shapefile
containing the convex hull of the footprint. The coordinate system will be determined,
as above, depending on whether ``--t_srs``, ``--dem-file``, or ``--datum`` is used.

Example::
    
    camera_footprint     \
      --dem-file dem.tif \
      image.tif          \
      camera.tsai        \
      --output-shp footprint.shp

Such a shapefile can be displayed and overlaid on top of georeferenced images     
with ``stereo_gui`` (:numref:`plot_poly`).
  
Save as KML
~~~~~~~~~~~

If a KML output path is provided, this program will also create a KML file
containing the footprint.

The KML will show a box with an X pattern displaying the points ASP used to
compute the footprint. If the provided DEM is smaller than the image footprint,
additional point samples may be drawn within the image footprint, as the DEM is
sampled.

The entries in the KML file will be in longitude-latitude coordinates.

Example::

    camera_footprint     \
      --dem-file dem.tif \
      image.tif          \
      camera.tsai        \
      --output-kml footprint.kml

Project onto a datum
~~~~~~~~~~~~~~~~~~~~

If a DEM is not provided, the program will project onto a datum instead. For
that, use an option such as ``--datum WGS_1984`` instead of ``--dem-file``.

Usage
~~~~~

::

     camera_footprint [options] <camera-image> <camera-model>

Command-line options
~~~~~~~~~~~~~~~~~~~~

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

--output-shp <string>
    Save the convex hull of the points sampled on the camera footprint as a
    shapefile with this name.
    
--output-kml <string>
    Write an output KML file at this location.

--quick
    Use a faster but less accurate computation.

-h, --help
    Display the help message.

