.. _pleiades:

Pleiades
--------

ASP supports the 1A/1B and NEO satellites from Airbus Pleiades. For NEO, see
:numref:`pleiades_neo` for additional notes. ASP also supports the Pleiades ortho
products, if the projection was done on a surface of constant height  (:numref:`pleiades_projected`).

The Airbus Pleiades data have both an exact linescan camera model and an
approximate RPC model (:numref:`rpc`). These are stored in separate files. The
names for these start with "DIM" and "RPC", respectively, and end with ".XML".
ASP supports both kinds. The USGS CSM library (:numref:`csm`) is used for
linescan models.

See :numref:`airbus_tiled` if the input images arrive in multiple
tiles. See :numref:`jitter_pleiades` for an example of solving for
jitter for these cameras.

Bundle adjustment and stereo with raw images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If desired to process a Pleiades triplet, bundle adjustment
(:numref:`bundle_adjust`) is suggested before stereo. It should be run as::

    bundle_adjust -t pleiades        \
      --camera-weight 0              \
      --tri-weight 0.1               \
      left.tif right.tif             \
      left_exact.xml right_exact.xml \
      -o ba/run

With the exact models, the stereo command, without bundle-adjusted cameras, is::

    parallel_stereo -t pleiades        \
        --stereo-algorithm asp_mgm     \
        --subpixel-mode 9              \
        left.tif right.tif             \
        left_exact.xml right_exact.xml \
        results/run

Then, a DEM is created with ``point2dem`` (:numref:`point2dem`)::

    point2dem results/run-PC.tif
    
See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices for stereo.

For the RPC model the option ``-t rpc`` should be used and the correct
camera files should be passed in. If the ``-t`` option is not
specified, it will be auto-guessed based on the content of the camera
files provided as inputs.

To make use of bundle-adjusted cameras, add the option ``--bundle-adjust-prefix
ba/run`` to the ``parallel_stereo`` command above.

For Pleiades exact linescan camera models the atmospheric correction
and velocity aberration corrections (:cite:`nugent1966velocity`) are
disabled. This ensures that the exact and RPC camera models agree (see
below).

Stereo with mapprojected images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ASP supports running stereo with mapprojected Pleiades images
(:numref:`mapproj-example`). All input images must be mapprojected at
the same resolution (which is comparable with the ground sample
distance, GSD). The same camera models must be used for mapprojection
as for stereo, so one should not mix the exact and RPC cameras.

Example::

    proj="+proj=utm +zone=13 +datum=WGS84 +units=m +no_defs"

    mapproject -t pleiades \
      --tr 0.5             \
      --t_srs "$proj"      \
      ref_dem.tif          \
      left.tif             \
      left_exact.xml       \
      left_map.tif
      
    mapproject -t pleiades \
      --tr 0.5             \
      --t_srs "$proj"      \
      ref_dem.tif          \
      right.tif            \
      right_exact.xml      \
      right_map.tif
      
    parallel_stereo -t pleiades      \
      --stereo-algorithm asp_mgm     \
      --subpixel-mode 9              \
      left_map.tif right_map.tif     \
      left_exact.xml right_exact.xml \
      run_map/run                    \
      ref_dem.tif
      
   point2dem run_map/run-PC.tif 

The projection string above needs to be modified for your area of
interest. It is strongly suggested to use an auto-determined UTM or polar
stereographic projection (:numref:`point2dem_proj`).

The value of the ``--tr`` option is the ground sample distance. It is normally
0.5 to 0.7 meters for Pleiades PAN images. The XML files should have the GSD
value.

To make use of bundle-adjusted cameras, add the option ``--bundle-adjust-prefix
ba/run`` to the ``mapproject`` and ``parallel_stereo`` commands above.

Exact and RPC cameras
~~~~~~~~~~~~~~~~~~~~~

To compare the linescan (exact) and RPC models, run ``cam_test``
(:numref:`cam_test`) as::

     cam_test --image img.tif --cam1 cam_exact.xml --cam2 cam_rpc.xml \
       --session1 pleiades --session2 rpc

This should give great agreement when it comes to pixels projected
from one camera to the ground, then projected back to the other
camera::

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

.. _pleiades_neo:

Pleiades NEO
~~~~~~~~~~~~

Several peculiarities make the Pleiades NEO data different from 1A/1B (:numref:`pleiades`):

- The tabulated positions and orientations may start slightly after the first image line and end slightly before the last image line. If these scenarios are encountered, linear extrapolation based on two nearest values is used to fill in the missing values and a warning is printed for each such operation.
- There is no field for standard deviation of the ground locations of pixels projected from the cameras, so error propagation is not possible unless such a value is specified manually (:numref:`error_propagation`).
- The RPC camera models for a stereo triplet can be rather inconsistent with each other, resulting in large triangulation error. It is suggested to use instead the exact linescan camera model.

.. _pleiades_projected:

Pleiades projected images
~~~~~~~~~~~~~~~~~~~~~~~~~

Airbus offers Pleiades ortho images, that are projected onto a surface of
constant height above a datum. A pair of such images can be used for stereo
and terrain creation.

Each ortho image comes with two XML files. The first, with the ``DIM`` prefix,
stores the projection height, in the ``Bounding_Polygon`` XML field, in the
``H`` subfield. This height is in meters, above the WGS84 ellipsoid. This file
lacks the camera model, unlike the earlier products.

The second XML file starts with the ``RPC`` prefix and contains the RPC camera
model. 

Given two such images forming a stereo pair, the heights should be manually read
from the ``DIM`` files. Then, ``parallel_stereo`` should be invoked with
the RPC camera files, as discussed in :numref:`mapproj_ortho`.

ASP does not support Airbus images that are orthorectified with a 3D terrain
model, as that terrain model is not known.

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

This will create a virtual mosaic, which is just a plain text file having
pointers to the subimages. ASP can use that one as if it were a real image. If
desired, an actual self-contained image can be produced with::

    gdal_translate -co TILED=YES -co BLOCKXSIZE=256 -co BLOCKYSIZE=256 \
      -co BIGTIFF=IF_SAFER vrt.tif image.tif

Note that the size of this image will be comparable to the sum of sizes
of the original tiles.

The Orfeo Toolbox provides functionality for stitching such images as well.

