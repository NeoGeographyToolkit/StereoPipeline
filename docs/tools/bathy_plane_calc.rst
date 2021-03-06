.. _bathy_plane_calc:

bathy_plane_calc
----------------

The ``bathy_plane_calc`` program takes as input a shapefile and a DEM,
finds the 3D positions of the vertices of the shapefile in the DEM in
ECEF coordinates using bilinear interpolation, and fits a plane
through those 3D points. 

The motivation for this tool and an example of how to use it in practice
is given in :numref:`water_surface`. 

Example::

     bathy_plane_calc --shapefile shape.shp --dem dem.tif     \
       --bathy-plane plane.txt

Command-line options for bathy_plane_calc:

-h, --help
    Display the help message.

--shapefile <filename>
    The shapefile with vertices whose coordinates will be looked up in the DEM.

--dem <filename>
    The DEM to use.

--bathy-plane arg                     
    The output file storing the computed plane as four coefficients
    a, b, c, d, with the plane being a*x + b*y + c*z + d = 0.

--outlier-threshold <double>
    A value, in meters, to determine the distance from a sampled point
    on the DEM to the best-fit plane to determine if it will be marked as 
    outlier and not included in the calculation of that plane. The default
    is 0.2.

--num-ransac-iterations <integer>
    Number of RANSAC iterations to use to find the best-fitting plane.
    The default is 1000.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
