.. _bathy_plane_calc:

bathy_plane_calc
----------------

The ``bathy_plane_calc`` program estimates the surface of a body of
water. It can take inputs in two ways. In one, a DEM is given, a
camera model, and a mask obtained from a raw image with that camera
model. The mask has the value 1 on land and 0 where there is water, or
positive values on land and no-data values on water. 

In the second way of specifying the inputs, a DEM and a shapefile
are provided, with the latter's vertices at the water-land
interface.

The output water surface produced by this program is parameterized as
a plane in a local stereographic projection. This plane can be
slightly non-horizontal due to imperfections in the positions and
orientations of the cameras that were used to create the input DEM.

Further context is given in :numref:`shallow_water_bathy`.

.. _bathy_plane_calc_example1:

Example 1
~~~~~~~~~

Using a DEM, a camera file, and a mask for the camera image::

     bathy_plane_calc --mask mask.tif --session-type dg \
       --camera camera.xml --dem dem.tif                \
       --num-samples 10000 --outlier-threshold 0.5      \
       --bathy-plane plane.txt                          \
       --output-inlier-shapefile inliers.shp

Such a mask can be obtained by thresholding an image where the water
shows up darker than the land. A good example for this is the band 7
of Digital Globe multispectral images. The thresholding happens as
follows::

    thresh=155
    image_calc -c "max($thresh, var_0)" --output-nodata-value $thresh \
      image.tif -o mask.tif

It is important that the image be raw, not projected, and if
the image is part of a stereo pair, the corresponding camera
for that image be used. For a stereo pair, this exercise can be
done with both the left image and left camera, then separately
for the right image and right camera.

The DEM itself must be in very good good alignment with the mask used
earlier. For Digital Globe images, it can be created using stereo
either with the left and right PAN images from the same set as the
multispectral images, or with the multispectral images themselves (the
latter's resolution is coarser by a factor of 4 hence the DEM may turn
out to be less precise.)  It is best to not use bundle adjustment or
alignment before any of these steps, as that may result in the MS and
PAN images no longer being aligned to each other.

Above, the ``--session-type`` option determines which camera model to
use (Digital Globe files have both an exact ``dg`` model and an
approximate ``rpc`` model).

Running this command will produce an output as follows::

    Found 5017 / 13490 inliers.
    Max distance to the plane (meters): 6.00301
    Max inlier distance to the plane (meters): 0.499632
    Mean plane height above datum (meters): -22.2469
    Writing: plane.txt

The file ``plane.txt`` will look like this::

  -0.0090 0.0130 0.9998 22.2460
  # Latitude and longitude of the local stereographic projection with the WGS_1984 datum:
  24.5836 -81.7730

The last line has the center of the local stereographic projection in which
the plane is computed, and the first line has the equation of the plane
in that local coordinate system as::

    a * x + b * y + c * z + d = 0.

The value of ``c`` is almost 1 hence this plane is almost perfectly
horizontal in local coordinates and the value of ``-d/c`` gives its
height above the datum (The small deviation from the horizontal may be
due to the orientations of the satellites taking the pictures not
being perfectly known.)

It is important to decide carefully what outlier threshold to use and
to check the number of resulting inliers. If too few, that may mean
that the outlier threshold is too strict. Above, the inliers are saved
as a shapefile and can be inspected. The inliers should be
well-distributed over the entire shoreline.

.. _bathy_plane_calc_example2:

Example 2
~~~~~~~~~

In this example, a set of actual measurements is provided, as the
longitude, latitude, and water height above the WGS_1984 datum,
with the first two measured in degrees and the last one
in meters.

If the water heights are given relative to a geoid, or some other
datum, those need to be converted to WGS_1984.

It is expected that the measurements are given in a CSV file, with
commas or spaces used as separators. Here is an example file, named
``meas.csv``, for Florida Keys::
    
    # ID,lon,lat,NAVD88_m,WGS84_m
    0,-81.598635,24.587748,-0.28662,-23.72363
    1,-81.623768,24.581799,-0.28662,-23.70866
    2,-81.629865,24.578379,-0.2703,-23.69243
    3,-81.674545,24.564429,-0.2703,-23.6798
    4,-81.711308,24.555744,-0.2703,-23.67532
    5,-81.754465,24.55158,-0.2703,-23.67209
    6,-81.756012,24.551758,-0.23154,-23.63346
    7,-81.779985,24.548427,-0.23154,-23.63514

Any lines starting with the pound sign (``#``) will be ignored as
comments. If the first line does not start this way but does not have
valid data it will be ignored as well.

The program is called as follows::

    bathy_plane_calc --water-height-measurements meas.csv \
      --csv-format "2:lon 3:lat 5:height_above_datum"     \
      --num-samples 10000 --outlier-threshold 0.5         \
      --bathy-plane meas_plane.txt                        \
      --output-inlier-shapefile meas_inliers.shp

Note the ``--csv-format`` option, which should be set correctly. As
specified here, it will result in columns 2, 3, and 5, being read,
hence ignoring the ID in column 1 and the NAVD88 measurements in
column 4, and treating the read values as longitude, latitude, and
height above the WGS_1984 datum.

Here we assume that the WorldView images we use for processing later
are well-aligned to the world coordinates in which the above
measurements take place. How well that assumption holds and what to do
when it does not, is still to be determined.
 
.. _bathy_plane_calc_example3:

Example 3
~~~~~~~~~

This example uses a DEM and a shapefile as inputs::

     bathy_plane_calc --shapefile shape.shp --dem dem.tif    \
       --outlier-threshold 0.5                               \ 
       --output-inlier-shapefile inliers.shp                 \
       --bathy-plane plane.txt 

As earlier, it is important to consider carefully what outlier
threshold to use, and to examine the number and distribution of
inliers.

Here it is suggested that the DEM be obtained as in the previous
example, from a stereo pair, and the shapefile delineating the
water-land interface be drawn on top of an orthoimage created with the
same stereo pair. The commands for that can be as follows::

     parallel_stereo -t dg left.tif right.tif left.xml right.xml \
       run/run
     point2dem --orthoimage run/run-PC.tif run/run-L.tif

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices.

Here is an example of a shapefile created on top of an orthoimage:

.. figure:: ../images/examples/bathy/water_outline.png
   :name: bathy_water_plane_example

   Example of a shapefile whose vertices are at the water-land boundary.

Command-line options for bathy_plane_calc
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

--shapefile <filename>
    The shapefile with vertices whose coordinates will be looked up in
    the DEM.

--dem <filename>
    The DEM to use.

--mask <string>
    A input mask, created from a raw camera image and hence having the
    same dimensions, with values of 1 on land and 0 on water, or
    positive values on land and no-data values on water.

--camera <string>
    The camera file to use with the mask.

-t, --session-type <string>
    Select the stereo session type to use for processing. Usually
    the program can select this automatically by the file extension, 
    except for xml cameras. See :numref:`parallel_stereo_options` for
    options.

--outlier-threshold <double>
    A value, in meters, to determine the distance from a sampled point
    on the DEM to the best-fit plane to determine if it will be marked as 
    outlier and not included in the calculation of that plane. The default
    is 0.2.

--num-ransac-iterations <integer>
    Number of RANSAC iterations to use to find the best-fitting plane.
    The default is 1000.

--num-samples <integer>
    Number of samples to pick at the water-land interface if using a
    mask. The default is 10000.

--water-height-measurements <string>
    Use this CSV file having longitude, latitude, and height
    measurements for the water surface, in degrees and meters,
    respectively, relative to the WGS84 datum. The option --csv-format
    must be used.

--csv-format <string>
    Specify the format of the CSV file having water height
    measurements. The format should have a list of entries
    with syntax column_index:column_type (indices start from
    1). Example: '2:lon 3:lat 4:height_above_datum'.

--bathy-plane arg                     
    The output file storing the computed plane as four coefficients
    a, b, c, d, with the plane being a*x + b*y + c*z + d = 0.

--output-inlier-shapefile <string>
    If specified, save at this location the shape file with the inlier
    vertices.

--output-outlier-shapefile <string>
    If specified, save at this location the shape file with the outlier
    vertices.

--save-shapefiles-as-polygons
    Save the inlier and outlier shapefiles as polygons, rather than
    made of of discrete vertices. May be more convenient for processing
    in a GIS tool.

--dem-minus-plane <string (default: "")>
    If specified, subtract from the input DEM the best-fit plane and save the 
    obtained DEM to this GeoTiff file.

--use-ecef-water-surface
    Compute the best fit plane in ECEF coordinates rather than in a
    local stereographic projection. Hence don't model the Earth
    curvature. Not recommended.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
