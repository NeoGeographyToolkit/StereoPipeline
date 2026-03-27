.. _perusat1:

PeruSat-1
---------

PeruSat-1 (launched 2016) is a Peruvian Earth observation satellite with
0.7-meter panchromatic resolution. It provides exact linescan camera models and
RPC-approximated camera models in separate files. The names for these start with
"DIM" and "RPC", respectively, and end with ".XML".

ASP expects raw (non-orthorectified) images. The USGS CSM library (:numref:`csm`)
is used for the linescan model.

The session type is ``-t perusat`` (:numref:`ps_options`). If the ``-t``
option is not specified, it will be auto-detected from the camera files.

For the RPC model (:numref:`rpc`), the option ``-t rpc`` should be used
and the RPC camera files should be passed in.

.. _perusat1_stereo:

Bundle adjustment and stereo with raw images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Bundle adjustment (:numref:`bundle_adjust`) is suggested before stereo::

    bundle_adjust -t perusat         \
      --camera-weight 0              \
      --tri-weight 0.1               \
      left.tif right.tif             \
      left_exact.xml right_exact.xml \
      -o ba/run

With the exact models, the stereo command, with bundle-adjusted cameras, is::

    parallel_stereo -t perusat        \
      --stereo-algorithm asp_mgm      \
      --subpixel-mode 9               \
      --bundle-adjust-prefix ba/run   \
      left.tif right.tif              \
      left_exact.xml right_exact.xml  \
      results/run

Then, a DEM is created with ``point2dem`` (:numref:`point2dem`)::

    point2dem results/run-PC.tif

For steep terrain, it is suggested to run stereo with mapprojected images
(:numref:`perusat1_map`).

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices
for stereo.

See :numref:`jitter_pleiades` for an example of solving for jitter with these
cameras. Note the limitations of the jitter solver in
:numref:`jitter_limitations`. This is available as of build 2026/03
(:numref:`release`).

.. _perusat1_map:

Stereo with mapprojected images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ASP supports running stereo with mapprojected PeruSat-1 images
(:numref:`mapproj-example`). As of build 2026/03 (:numref:`release`),
mapprojection is significantly faster due to the switch to the CSM camera model.

All input images must be mapprojected at the same resolution (which is
comparable with the ground sample distance, GSD). The same camera models must be
used for mapprojection as for stereo, so one should not mix the exact and RPC
cameras.

It is strongly suggested to verify that the input DEM used for mapprojection is
relative to the ellipsoid (:numref:`conv_to_ellipsoid`).

Example::

    proj="+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs"

    mapproject -t perusat              \
      --tr 0.7                         \
      --t_srs "$proj"                  \
      --bundle-adjust-prefix ba/run    \
      ref_dem.tif                      \
      left.tif                         \
      left_exact.xml                   \
      left_map.tif

    mapproject -t perusat              \
      --tr 0.7                         \
      --t_srs "$proj"                  \
      --bundle-adjust-prefix ba/run    \
      ref_dem.tif                      \
      right.tif                        \
      right_exact.xml                  \
      right_map.tif

    parallel_stereo -t perusat        \
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
0.7 meters for PeruSat-1 PAN images. The XML files should have the GSD value.

To not use bundle-adjusted cameras, remove the option ``--bundle-adjust-prefix``
from all ``mapproject`` and ``parallel_stereo`` commands above.

It is strongly suggested to overlay the left and right mapprojected images on
each other and on the input DEM in ``stereo_gui`` (:numref:`stereo_gui`). A
systematic shift likely indicates a vertical datum disagreement between the DEM
and the camera models.

.. _perusat1_notes:

Notes
~~~~~

For PeruSat-1 exact linescan camera models the atmospheric correction and
velocity aberration corrections (:cite:`nugent1966velocity`) are disabled, as
these decrease somewhat the agreement with the RPC models.

DEMs created with the exact and RPC models differ by a systematic
vertical shift of about 15 meters for unknown reasons, even though the
intersection error maps are very similar. Nothing in the sensor manual
or camera metadata suggests the cause of this. The ``pc_align`` tool
(:numref:`pc_align`) can be used to reduce this discrepancy. The mean absolute
difference of the (full-image extent) aligned DEMs is about 0.17
meters.
