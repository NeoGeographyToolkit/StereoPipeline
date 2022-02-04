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

     bathy_plane_calc --session-type dg --mask mask.tif  \
       --camera camera.xml --bundle-adjust-prefix ba/run \
       --dem dem.tif --num-samples 10000                 \
       --outlier-threshold 0.5 --bathy-plane plane.txt   \
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

The ``--session-type`` option determines which camera model to
use (Digital Globe files have both an exact ``dg`` model and an
approximate ``rpc`` model).

The DEM itself must be in very good alignment with the mask used
earlier. For Digital Globe images, it can be created using stereo
either with the left and right PAN images from the same set as the
multispectral images, or with the multispectral images themselves (the
latter's resolution is coarser by a factor of 4 hence the DEM may turn
out to be less precise.)  

Great care must be used if bundle adjustment or alignment takes place,
to keep all datasets consistent. If the multispectral images were
bundle-adjusted, the same adjustments can be used with all
multispectral bands. If the DEM is obtained with bundle-adjusted
images multispectral images, then ``--bundle-adjust-prefix`` must be
passed to ``bathy_plane_calc`` as shown in the example above.

If, however, bundle adjustment happened, with one or both
of the MS and PAN pairs, the produced DEMs will no longer be
aligned to each other. Hence, these must be individually aligned
to a chosen reference DEM, the alignments applied to the cameras, 
as discussed in :numref:`ba_pc_align`.

This is further discussed in :numref:`bathy_and_align`.

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

In this example, a set of actual measurements of the water surface is
provided, as the longitude and latitude (in degrees, in decimal
format), and water height above the WGS_1984 datum (ellipsoid
heights), measured in meters.

If the water heights are given relative to a geoid (such as EGM2008),
or some other datum (such as NAD83), those need to be converted to
WGS_1984.

It is expected that the measurements are given in a CSV file, with
commas or spaces used as separators. Here is an example file, named
``meas.csv``, for Florida Keys::
    
   FID,Lon,Lat,WGS84_m
   0,-81.59864018,24.58775288,-23.86539
   1,-81.62377319,24.58180388,-23.84653
   2,-81.62987019,24.57838388,-23.8864
   3,-81.6745502,24.56443387,-23.86815
   4,-81.71131321,24.55574886,-23.86031
   5,-81.75447022,24.55158486,-23.85464
   6,-81.75601722,24.55176286,-23.89892
   7,-81.77999023,24.54843186,-23.89824

Any lines starting with the pound sign (``#``) will be ignored as
comments. If the first line does not start this way but does not have
valid data it will be ignored as well.

The program is called as follows::

    bathy_plane_calc --water-height-measurements meas.csv \
      --csv-format "2:lon 3:lat 4:height_above_datum"     \
      --num-samples 10000 --outlier-threshold 0.5         \
      --bathy-plane meas_plane.txt                        \
      --output-inlier-shapefile meas_inliers.shp

Note the ``--csv-format`` option, which should be set correctly. As
specified here, it will result in columns 2, 3, and 4, being read,
having the longitude, latitude, and height above datum (WGS84
ellipsoid).  The order in which the columns show up is not important,
as long as ``--csv-format`` correctly reflects that. Any extraneous
columns will be ignored, such as the ID in column 1.

Care must be taken to ensure all the measurements, resulting bathy
plane, and any DEMs are in the same coordinate system. This is
discussed further in :numref:`bathy_and_align`.

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

--bundle-adjust-prefix <string>
    Use the camera adjustment at this output prefix, if the cameras
    changed based on bundle adjustment or alignment.

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
