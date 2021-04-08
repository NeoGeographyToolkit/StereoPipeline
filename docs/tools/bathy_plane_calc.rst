.. _bathy_plane_calc:

bathy_plane_calc
----------------

The ``bathy_plane_calc`` program takes as input a shapefile and a DEM,
finds the 3D positions of the vertices of the shapefile in the DEM in
ECEF coordinates using bilinear interpolation, and fits a plane
through those 3D points. This would give the surface of the water
to be used for bathymetry correction.

Further motivation for this tool and an example of how to use it in
practice is given in :numref:`water_surface`.

Example::

     bathy_plane_calc --shapefile shape.shp --dem dem.tif     \
        --output-inlier-shapefile out_shape.shp               \
        --bathy-plane plane.txt 

It will produce output as follows:

::

    Found 4 / 9 inliers.
    Max distance to the plane (meters): 2.26214
    Max inlier distance to the plane (meters): 0.0131818
    Mean plane height above datum (meters): -21.3521
    Plane inclination (degrees): 0.225995

Here, the plane inclination is defined as the angle between the plane
normal and the ray going from the Earth center to the mean of all
inlier measurements in ECEF coordinates. The reason the plane
inclination is not zero, so the water surface is not perfectly
horizontal in the local coordinate system, is because of orientation
errors in the input images.

Command-line options for bathy_plane_calc:

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
    Save at this location the shape file with the inlier vertices.

--num-ransac-iterations <integer>
    Number of RANSAC iterations to use to find the best-fitting plane.
    The default is 1000.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
