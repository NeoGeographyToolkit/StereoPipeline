.. _umbra_sar:

Umbra SAR
---------

Here we describe processing Synthetic Aperture Radar (SAR) images for Earth
produced by `Umbra <https://help.umbra.space/product-guide>`_. An example for
the Moon is in :numref:`csm_minirf`.

Overview
~~~~~~~~

Umbra images are acquired in `Spotlight mode
<https://help.umbra.space/product-guide/umbra-products>`_.

SAR image appearance can vary drastically depending on the perspective. It is
important to choose images that are acquired close in time and have similar
viewing angles. The latter is measured by the provided incidence and azimuth
angles. Another measure is the squint angle.

If the stereo convergence angle (:numref:`stereo_pairs`) is too small, the
produced terrain model may not be accurate. For the example below, this angle is
5.8 degrees (as printed by both ``parallel_stereo`` and ``bundle_adjust``). This
angle correlates well with the larger of the discrepancy in azimuth and
incidence angles between the images. We obtained acceptable results even with
convergence angles as low as 2.5 degrees.

Umbra provides GEC images that are corrected to be relative to an ellipsoid. More
raw products are available, including SICD, that have complex-valued pixels. 

GEC images come with RPC (:numref:`rpc`) camera models embedded in the images,
that we employ. ASP does not support the more rigorous SAR sensor models.

Example
~~~~~~~

.. figure:: ../images/umbra_sar.png
   :name: umbra_sar_fig

   From left to right: hillshaded-terrain model, mapprojected
   (:numref:`mapproject`) SAR image, and triangulation error image
   (:numref:`triangulation_error`). The units in the colorbar on the right are
   in meters.

We downloaded the image pair::

  2024-02-01-03-28-13_UMBRA-06_GEC.tif
  2024-04-03-14-53-17_UMBRA-04_GEC.tif

showing a portion of the Panama Canal. Many other `Umbra datasets
<https://registry.opendata.aws/umbra-open-data/>`_ are available.

To make the notation shorter, we call these ``left.tif`` and ``right.tif``.

It was helpful to run bundle adjustment first (:numref:`bundle_adjust`), to make
the images more self-consistent and reduce the triangulation error
(:numref:`triangulation_error`)::

    bundle_adjust -t rpc                        \
      left.tif right.tif                        \
      --ip-detect-method 1                      \
      --ip-per-tile 4000                        \
      --remove-outliers-params "75.0 3.0 50 50" \
      -o ba/run 

The cameras are embedded in the images, so they are not specified separately.

We found that the SIFT feature detection method (``--ip-detect-method 1``)
worked better than the default (method 0). It was helpful to search for many
more features than usual with the option ``--ip-per-tile``, as SAR images can be
noisy and features hard to find. 

More details on the ``bundle_adjust`` options are in :numref:`ba_options`.

Next, ``parallel_stereo`` (:numref:`parallel_stereo`) was run.
We made use of the corrected cameras and clean (outlier-filtered)
match files produced earlier::

    parallel_stereo -t rpc              \
      --bundle-adjust-prefix ba/run     \
      --clean-match-files-prefix ba/run \
      --stereo-algorithm asp_mgm        \
      --nodes-list machines.txt         \
      left.tif right.tif                \
      stereo/run

The ``asp_mgm`` algorithm worked much better than the default ``asp_bm``
(:numref:`stereo_alg_overview`).

A terrain model was produced with ``point2dem`` (:numref:`point2dem`),
in a local stereographic projection::

    point2dem --stereographic \
      --auto-proj-center      \
      --errorimage            \
      --tr 2.0                \
      stereo/run-PC.tif

To compare with a preexisting terrain, we fetched a portion of the
Copernicus DEM (:numref:`initial_terrain`), that we converted to be
relative to the WGS84 ellipsoid (:numref:`conv_to_ellipsoid`).
We call that dataset ``ref.tif``.

The ASP-created DEM was aligned to the reference DEM with ``pc_align``
(:numref:`pc_align`)::


    pc_align                                  \
      --max-displacement 300                  \
      --save-inv-transformed-reference-points \
      stereo/run-DEM.tif ref.tif              \
      -o align/run

A good value for the ``--max-displacement`` option is perhaps 1.5 times the mean
elevation difference between the two input DEMs, that can be found with
``geodiff`` (:numref:`geodiff`)  and ``gdalinfo -stats``.

The transformed cloud can be gridded back to a DEM as::

  point2dem --tr 2.0 \
    --t_srs "$proj"  \
    align/run-trans_reference.tif
    
Here, the projection string in ``$proj`` can be the same as for the DEM created earlier
(the ``gdalinfo -proj4`` command invoked on that DEM can print it). 

The ``geodiff`` program can take the difference of the now-aligned DEMs.
Other inspections can be done as discussed in :numref:`visualising`.
