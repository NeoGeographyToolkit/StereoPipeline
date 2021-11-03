.. _bathy_plane_calc:

bathy_plane_calc
----------------

The ``bathy_plane_calc`` program estimates the surface of a body of
water. It can take inputs in two ways. In one, a DEM is given, a
camera model, and a mask obtained from a raw image with that camera
model. The mask has the value 1 on land and 0 where there is water, or
positive values on land and no-data values on water, such as
constructed in :numref:`bathy_mask_creation`. 

In the second way of specifying the inputs, a DEM and a shapefile
are provided, with the latter's vertices at the water-land
interface. Such a shapefile can be created, for example, using a
mapprojected image as a guide for where that interface is.

The output water surface produced by this program is parameterized as
a plane in a local stereographic projection. This plane can be
slightly non-horizontal due to imperfections in the positions and
orientations of the cameras that were used to create the input DEM.

Further motivation for this tool and an example of how to use it in
practice is given in :numref:`water_surface`.

Example 1
~~~~~~~~~

Using a DEM, a camera file, and a mask for the camera image::

     bathy_plane_calc --mask mask.tif --session-type dg \
       --camera camera.xml --dem dem.tif                \
       --output-inlier-shapefile out_shape.shp          \
       --num-samples 500000                             \
       --outlier-threshold 0.5                          \ 
       --bathy-plane plane.txt                          \
       --output-inlier-shapefile inliers.shp

Here, the the session type determines which camera model to use
(Digital Globe files have both an exact ``dg`` model and an
approximate ``rpc`` model).

It will produce an output as follows::

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

Producing an output shapefile having inliers is optional. That one can
be overlayed on top of the DEM. Its edges can be ignored and only the
vertices should be checked if they are well-distributed and at water's
edge. A better display strategy may be adopted going forward.

Example 2
~~~~~~~~~

Using a DEM and a shapefile as inputs::

     bathy_plane_calc --shapefile shape.shp --dem dem.tif    \
       --output-inlier-shapefile out_shape.shp               \
       --bathy-plane plane.txt 

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
    same dimensions, with values of 1 on land and 0 in the water.

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

--bathy-plane arg                     
    The output file storing the computed plane as four coefficients
    a, b, c, d, with the plane being a*x + b*y + c*z + d = 0.

--output-inlier-shapefile <string>
    If specified, save at this location the shape file with the inlier
    vertices.

--num-ransac-iterations <integer>
    Number of RANSAC iterations to use to find the best-fitting plane.
    The default is 1000.

--num-samples <integer>
    Number of samples to pick at the water-land interface if using a
    mask. The default is 500000.

--dem-minus-plane <string (default: "")>
    If specified, subtract from the input DEM the best-fit plane and save the 
    obtained DEM to this GeoTiff file.

--use-ecef-water-surface
    Compute the best fit plane in ECEF coordinates rather than in a
    local stereographic projection. Hence don't model the Earth
    curvature. Not recommended.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
