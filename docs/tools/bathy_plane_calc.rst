.. _bathy_plane_calc:

bathy_plane_calc
----------------

The ``bathy_plane_calc`` program takes as input a shapefile and a DEM,
finds the 3D positions of the vertices of the shapefile in the DEM in
ECEF coordinates using bilinear interpolation, converts those points
to a local stereographic projection, and fits a plane through
them. 

When the vertices in the shapefile are picked at the water-land
interface in the DEM, this would give the surface of the water to be
used for bathymetry correction. The obtained plane can be slightly
non-horizontal due to imperfections in the camera positions and
orientations, and in the input DEM.

Further motivation for this tool and an example of how to use it in
practice is given in :numref:`water_surface`.

Example
~~~~~~~

::

     bathy_plane_calc --shapefile shape.shp --dem dem.tif    \
       --output-inlier-shapefile out_shape.shp               \
       --bathy-plane plane.txt 

It will produce output as follows:

::

    Found 4 / 9 inliers.
    Max distance to the plane (meters): 2.26214
    Max inlier distance to the plane (meters): 0.0131818
    Mean plane height above datum (meters): -21.35657

The file ``plane.txt`` will look like this::

  -0.0090 0.0130 0.9998 21.3523
  # Latitude and longitude of the local stereographic projection with the WGS_1984 datum
  24.583656822372209 -81.773073668899542

The last line has the center of the local stereographic projection in which
the plane is computed, and the first line has the equation of the plane
in that local coordinate system as::

    a * x + b * y + c * z + d = 0.

The value of ``c`` is almost 1 hence this plane is almost perfectly
horizontal in local coordinates and the value of ``-d/c`` gives its
height above the datum (The small deviation from the horizontal may be
due to the orientations of the satellites taking the pictures not
being perfectly known.)
   
Command-line options for bathy_plane_calc
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

--shapefile <filename>
    The shapefile with vertices whose coordinates will be looked up in
    the DEM.

--dem <filename>
    The DEM to use.

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

--dem-minus-plane <string (default: "")>
    If specified, subtract from the input DEM the best-fit plane and save the 
    obtained DEM to this GeoTiff file.

--use-ecef-water-surface
    Compute the best fit plane in ECEF coordinates rather than in a
    local stereographic projection. Hence don't model the Earth
    curvature. Not recommended.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
