.. _icebridgekmztocsv:

icebridge_kmz_to_csv
--------------------

This is a simple tool for use with data from the NASA IceBridge program. 

It takes as input Google Earth compatible .kmz files from:

  http://asapdata.arc.nasa.gov/dms/missions.html 

which display the aircraft position at the point when each DMS frame image was
captured.

This tool exports those camera positions in ECEF to a csv file which can be
passed to ``bundle_adjust`` (:numref:`bundle_adjust`) using the following
parameters::

   --camera-positions ../camera_positions.csv              \
   --csv-format "1:file 2:lon 3:lat 4:height_above_datum"

This list is used to transform the camera positions and orientations from a
local coordinate system to the ECEF coordinate system for the current planet.

This may be useful in conjunction with ``camera_solve`` (:numref:`camera_solve`).
See :numref:`sfmicebridge` for an example.

Usage::

    icebridge_kmz_to_csv <input kmz file> <output csv file>

