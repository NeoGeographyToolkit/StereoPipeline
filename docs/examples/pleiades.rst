.. _pleiades:

Pleiades
--------

The Airbus Pleiades satellites data have both an exact linescan camera model
and an approximate RPC model. These are stored in separate files. The
names for these start with "DIM" and "RPC", respectively, and end with
".XML". 

ASP supports the linescan model for the 1A/1B satellites. It can also
use the RPC model (:numref:`rpc`), likely for all Pleiades satellites,
including *Neo*. The linescan support is based on the USGS CSM library
(:numref:`csm`).

See :numref:`airbus_tiled` if the input images arrive in multiple
tiles. See :numref:`jitter_pleiades` for an example of solving for
jitter for these cameras.

With the exact model, the stereo command is::

    parallel_stereo -t pleiades --stereo-algorithm asp_mgm           \
        left.tif right.tif left_exact.xml right_exact.xml results/run

For the RPC model the option ``-t rpc`` should be used and the correct
camera files should be passed in. If the ``-t`` option is not
specified, it will be auto-guessed based on the content of the camera
files provided as inputs.

For Pleiades exact linescan camera models the atmospheric correction
and velocity aberration corrections (:numref:`sensor_corrections`) are
disabled. This ensures that the exact and RPC camera models agree (see
below).

ASP supports running stereo with mapprojected Pleiades images
(:numref:`mapproj-example`). All input images must be mapprojected at
the same resolution (which is comparable with the ground sample
distance). The same camera models must be used for mapprojection
as for stereo, so one should not mix the exact and RPC cameras.
Example::

    mapproject --tr 0.000009 -t pleiades                        \
      ref_dem.tif left.tif left_exact.xml left_map.tif 
    mapproject --tr 0.000009 -t pleiades                        \
      ref_dem.tif right.tif right_exact.xml right_map.tif
    parallel_stereo --stereo-algorithm asp_mgm                  \
      left_map.tif right_map.tif left_exact.xml right_exact.xml \
      run_map/run ref_dem.tif
   point2dem run_map/run-PC.tif 

Here it is assumed the images are far from the poles, the input DEM
has the longlat projection, and the grid size (``--tr``) is in degrees
(the value 0.000009 may need adjustment). Otherwise, a polar or UTM
projection needs to be used (option ``--t_srs``) and the grid size
should be set to the known image ground sample distance in
meters.

To compare the linescan and RPC models, run ``cam_test``
(:numref:`cam_test`) as::

     cam_test --image img.tif --cam1 cam_exact.xml --cam2 cam_rpc.xml \
       --session1 pleiades --session2 rpc

This should give great agreement when it comes to pixels projected
from one camera to the ground, then reprojected back to the other
one::

    cam1 to cam2 pixel diff
    Max:    0.00304066

    cam2 to cam1 pixel diff
    Max:    0.00296764

The camera centers computed by the two methods won't agree, because
the RPC camera model does not store the camera center. ASP then
substitutes it with an estimated point on the ray from the camera
center to the ground. This disagreement is not an issue in practice.

Commands similar to the above can be used to compare the exact and RPC
cameras not to each other but against themselves. This tool will also
print timing information for the operation of projecting a pixel to
the ground and back.

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices for stereo.

.. _airbus_tiled:

Pleiades tiled images
~~~~~~~~~~~~~~~~~~~~~

With some Airbus Pleiades data, each of the left and right images
may arrive broken up into .TIF or .JP2 tiles, with names ending in
R1C1.tif, R2C1.tif, etc.

These need to be mosaicked before being used. That can be done as
follows (individually for the left and right stereo image), using
``gdalbuildvrt`` (:numref:`gdal_tools`)::

      gdalbuildvrt vrt.tif *R*C*.tif

This expects any input .tif file to have an associated .tfw (.TFW) file
containing information about how the tiles should be combined.

If both PAN and multispectral tiles are present, use only the PAN ones.

This will create a virtual mosaic, which is just a plain text file
having pointers to the subimages. ASP can use that one as if it was a real image.
If desired, an actual self-contained image can be produced with::

    gdal_translate -co TILED=YES -co BLOCKXSIZE=256 -co BLOCKYSIZE=256 \
      -co BIGTIFF=IF_SAFER vrt.tif image.tif

Note that the size of this image will be comparable to the sum of sizes
of the original tiles.

The Orfeo Toolbox provides functionality for stitching such images as well.

