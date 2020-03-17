.. _icebridgekmztocsv:

icebridge_kmz_to_csv
--------------------

A simple tool for use with data from the NASA IceBridge program. Google
Earth compatible .kmz files are available at
http://asapdata.arc.nasa.gov/dms/missions.html which display the
aircraft position at the point when each DMS frame image was captured.
This tool exports those positions into a csv file which can be passed
into ``bundle_adjust`` using the following parameters::

   --camera-positions ../camera_positions.csv --csv-format "1:file 2:lon 3:lat 4:height_above_datum"

This may be useful in conjunction with the ``camera_solve`` tool to
allow conversion of camera positions from local to global coordinates.

Usage::

     > icebridge_kmz_to_csv <input kmz file> <output csv file>
