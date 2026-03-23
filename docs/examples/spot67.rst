.. _spot67:

SPOT 6/7
--------

SPOT 6 (launched 2012) and SPOT 7 (launched 2014) are Airbus Earth observation
satellites with 1.5-meter panchromatic resolution. They are part of the same
SPOT family as SPOT 5 (:numref:`spot5`), but use the DIMAP V2 XML format and a
linescan camera model that closely follows the Pleiades sensor
(:numref:`pleiades`).

ASP expects raw (non-orthorectified) images. The data have both an exact
linescan camera model and an approximate RPC model (:numref:`rpc`), stored in
separate XML files whose names start with "DIM" and "RPC", respectively. ASP
supports both. The USGS CSM library (:numref:`csm`) is used for linescan
models.

The session type is ``-t spot`` (:numref:`ps_options`). If the ``-t``
option is not specified, it will be auto-detected from the camera files.

.. _spot67_stereo:

Bundle adjustment and stereo with raw images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Bundle adjustment (:numref:`bundle_adjust`) is suggested before stereo::

    bundle_adjust -t spot            \
      --camera-weight 0              \
      --tri-weight 0.1               \
      left.tif right.tif             \
      left_exact.xml right_exact.xml \
      -o ba/run

With the exact models, the stereo command, with bundle-adjusted cameras, is::

    parallel_stereo -t spot           \
      --stereo-algorithm asp_mgm      \
      --subpixel-mode 9               \
      --bundle-adjust-prefix ba/run   \
      left.tif right.tif              \
      left_exact.xml right_exact.xml  \
      results/run

Then, a DEM is created with ``point2dem`` (:numref:`point2dem`)::

    point2dem results/run-PC.tif

For steep terrain, it is suggested to run stereo with mapprojected images
(:numref:`spot67_map`).

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices
for stereo.

See :numref:`jitter_pleiades` for an example of solving for jitter with these
cameras. Note the limitations of the jitter solver in
:numref:`jitter_limitations`.

For the RPC model (:numref:`rpc`), the option ``-t rpc`` should be used
and the RPC camera files should be passed in. If the ``-t`` option is
not specified, it will be auto-guessed based on the content of the
camera files provided as inputs.

.. _spot67_map:

Stereo with mapprojected images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ASP supports running stereo with mapprojected SPOT 6/7 images
(:numref:`mapproj-example`).

All input images must be mapprojected at the same resolution (which is
comparable with the ground sample distance, GSD). The same camera models must be
used for mapprojection as for stereo, so one should not mix the exact and RPC
cameras.

Ensure the input DEM used for mapprojection is relative to the ellipsoid
(:numref:`conv_to_ellipsoid`).

Example::

    proj="+proj=utm +zone=13 +datum=WGS84 +units=m +no_defs"

    mapproject -t spot               \
      --tr 1.5                       \
      --t_srs "$proj"                \
      --bundle-adjust-prefix ba/run  \
      ref_dem.tif                    \
      left.tif                       \
      left_exact.xml                 \
      left_map.tif

    mapproject -t spot               \
      --tr 1.5                       \
      --t_srs "$proj"                \
      --bundle-adjust-prefix ba/run  \
      ref_dem.tif                    \
      right.tif                      \
      right_exact.xml                \
      right_map.tif

    parallel_stereo -t spot           \
      --stereo-algorithm asp_mgm      \
      --subpixel-mode 9               \
      --bundle-adjust-prefix ba/run   \
      left_map.tif right_map.tif      \
      left_exact.xml right_exact.xml  \
      run_map/run                     \
      ref_dem.tif

    point2dem run_map/run-PC.tif

The projection string above needs to be modified for your area of interest. It
is strongly suggested to use an auto-determined UTM or polar stereographic
projection (:numref:`point2dem_proj`).

The value of the ``--tr`` option is the ground sample distance. It is normally
1.5 meters for SPOT 6/7 PAN images. The XML files should have the GSD value.

To not use bundle-adjusted cameras, remove the option ``--bundle-adjust-prefix``
from all ``mapproject`` and ``parallel_stereo`` commands above.

.. _spot67_exact_vs_rpc:

Exact and RPC cameras
~~~~~~~~~~~~~~~~~~~~~

To compare the linescan (exact) and RPC models, run ``cam_test``
(:numref:`cam_test`) as::

     cam_test --image img.tif        \
       --cam1 cam_exact.xml          \
       --cam2 cam_rpc.xml            \
       --session1 spot --session2 rpc

With the ESA La Crau sample (PAN band, ``--height-above-datum 200``), the pixel
difference between exact and RPC models was median 0.021, max 0.041 pixels.
This confirms excellent agreement between the two models.

The camera centers computed by the two methods won't agree, because the RPC
camera model does not store the camera center. ASP then substitutes it with an
estimated point on the ray from the camera center to the ground. This
disagreement is not an issue in practice.

Note that SPOT 6/7 RPCs use 0-based pixel offsets, while Pleiades RPCs use
1-based offsets. ASP handles both conventions automatically.
