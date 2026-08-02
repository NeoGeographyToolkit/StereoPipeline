.. _spot14:

SPOT 1-4 (HRV)
--------------

SPOT 1, 2, 3, and 4 are CNES (Space Agency of France) satellites launched
between 1986 and 1998. Each carried two High Resolution Visible (HRV)
instruments with a panchromatic ground resolution of 10 meters and a
cross-track steering mirror. Unlike the along-track fore/aft stereo of SPOT5
(:numref:`spot5`), an HRV stereo pair is assembled from two acquisitions of the
same area on different dates, with different mirror pointing.

ASP builds a rigorous CSM linescan camera for SPOT 1-4 directly from the DIMAP
metadata, using the same code path and session (``spot5``) as SPOT5. This
requires an ASP build from 2026/8 or later (:numref:`release`).

Camera model
~~~~~~~~~~~~

SPOT 1-4 share the DIMAP v1 format with SPOT5. They differ in two ways, both
handled automatically:

- Attitude is given as two absolute yaw, pitch, and roll samples (a
  ``Raw_Attitudes`` block) rather than a pre-corrected per-line table. These
  are interpolated across the scene. The magnitudes are of the order of
  microradians, so this is adequate; residual low-frequency attitude is
  discussed below.
- The look angles are listed for the first and last detector only. They are
  interpolated linearly to every column.

The geolocation encoded in the metadata is accurate to roughly one kilometer,
so bundle adjustment (:numref:`bundle_adjust`) is always needed, followed by
alignment to a reference (:numref:`pc_align`).

Data access
~~~~~~~~~~~

The SPOT 1-5 archive is free from the CNES SPOT World Heritage (SWH) program at
https://regards.cnes.fr/user/swh. Create an account, accept the ETALAB 2.0
license, and download the level-1A SCENE product in DIMAP format. Do not use the
orthorectified level-2 product, as the rigorous camera needs the raw scene.

Scenes are identified in the catalogue by satellite, the SPOT grid reference K
(column) and J (row), date, instrument (HRV1 or HRV2), and incidence angle. The
scene identifier encodes all of these, for example ``11222858708240840372P`` is
satellite 1 (SPOT-1), K122, J285, 1987-08-24, 08:40:37, instrument 2 (HRV2),
panchromatic. A stereo pair is two scenes of the same K and J acquired with
different incidence angles.

Example stereo pair
~~~~~~~~~~~~~~~~~~~

The scenes below cover the Harrat volcanic field near Badia, northeast Jordan
(K122, J285), a site used for SPOT stereo DEM assessment by Al-Rousan and Petrie
(1998). All three are SPOT-1 HRV2 panchromatic:

.. list-table::
   :header-rows: 1

   * - Scene ID
     - Date
     - Incidence
   * - ``11222858704220824352P``
     - 1987-04-22
     - +2.2° (near-nadir)
   * - ``11222858708240840372P``
     - 1987-08-24
     - +28.2° (east look)
   * - ``11222858709050809462P``
     - 1987-09-05
     - -23.7° (west look)

The 1987-08-24 and 1987-09-05 scenes view from opposite sides, giving a strong
base-to-height ratio of about 0.98. Pairing either off-nadir scene with the
near-nadir 1987-04-22 scene gives a more moderate convergence of about 26°,
which is easier to correlate. The example uses the two ~26° pairs.

After download, each scene arrives as a directory with the image in
``IMAGERY.TIF`` and the metadata in ``METADATA.DIM``. Since bundle adjustment
identifies images by file name, rename each pair so the names are unique, as for
SPOT5 (:numref:`spot5`). Below the images are named ``apr.tif``, ``aug.tif``,
``sep.tif`` with cameras ``apr.dim``, ``aug.dim``, ``sep.dim``.

Bundle adjustment
~~~~~~~~~~~~~~~~~

Adjust all three scenes together, so the near-nadir scene ties the two off-nadir
looks::

    bundle_adjust -t spot5      \
      apr.tif aug.tif sep.tif   \
      apr.dim aug.dim sep.dim   \
      --ip-per-image 30000      \
      -o ba/run

Inspect ``ba/run-final_residuals_stats.txt`` (:numref:`bundle_adjust`). The
median reprojection error should be well under a pixel.

Stereo and DEM
~~~~~~~~~~~~~~

Run stereo on each pair with local epipolar alignment
(:numref:`parallel_stereo`), then make a DEM with the triangulation error
(:numref:`point2dem`)::

    parallel_stereo -t spot5            \
      --alignment-method local_epipolar \
      --stereo-algorithm asp_mgm        \
      --bundle-adjust-prefix ba/run     \
      apr.tif aug.tif apr.dim aug.dim   \
      st_pair1/run
    point2dem --errorimage --t_srs <UTM_EPSG> st_pair1/run-PC.tif

Repeat for the second pair. The triangulation (intersection) error is the key
camera-quality check. After bundle adjustment it should be a small fraction of
the ground sample distance.

Alignment to a reference DEM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A bundle adjustment without ground control leaves an absolute offset, so align
to a reference such as Copernicus GLO-30. Convert the reference to heights above
the ellipsoid first (:numref:`conv_to_ellipsoid`), and regrid both DEMs to a
common projection and grid (about 4 times the image ground sample distance) with
``gdalwarp -r cubicspline``.

Align with pc_align (:numref:`pc_align`)::

    pc_align --max-displacement 1000 \
      --num-iterations 1000          \
      --save-transformed-source-points \
      ref_dem.tif spot_dem.tif       \
      -o align/run
    point2dem --t_srs <UTM_EPSG> align/run-trans_source.tif -o align/spot_aligned

On low-texture terrain the option ``--initial-transform-from-hillshading`` can
lock onto a spurious rotation. If the DEM is already close to the reference, as
after bundle adjustment, plain point-to-plane alignment is more robust. Judge
the result by a hillshade overlay, not by the vertical difference alone
(:numref:`pc_align`).

Results and accuracy
~~~~~~~~~~~~~~~~~~~~

For the Badia pairs, bundle adjustment reduced the median reprojection error
from about 14 pixels to 0.27 pixel. Each DEM had a triangulation error of about
0.25 of the ground sample distance. After alignment, the median vertical
difference to Copernicus was 0.2 meters.

A residual low-frequency tilt remains, visible as a gradient in the horizontal
disparity to the reference and in the difference between the two DEMs. It comes
from SPOT 1-4 carrying only two absolute attitude samples. Integrating the
attitude-rate stream, or a jitter solve (:numref:`jitter_solve`), would reduce
it further.

CCD corrections
~~~~~~~~~~~~~~~

SPOT 2 and SPOT 4 have known detector-to-detector misregistration on the HRV1
instrument that appears as faint striping. If a stereo pair from these
instruments shows striping in the triangulation error or the DEM, it should be
corrected before stereo. Panchromatic scenes from other SPOT 1-4 instruments in
this archive show little striping and can be used directly.
