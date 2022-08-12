.. _experimental:

Experimental features
=====================

.. _jitter:

Managing camera jitter
----------------------

In this section we will talk about big source of inaccuracies in
DigitalGlobe/Maxar images, after CCD artifacts, namely jitter, and how
to correct it.

It is important to note that jitter correction is highly experimental,
and it is not ready for production.

The order in which these corrections need to be handled is the
following. First, CCD artifacts are corrected. Then, optionally, images
are mosaicked with ``dg_mosaic`` and map-projected. And jitter should be
handled last, during stereo. An exception is made for WV03 images, for
which CCD artifacts do not appear to have a significant effect.

Camera jitter has its origin in the fact that the measured position and
orientation of the image-acquiring line sensor as specified in a camera
XML file is usually not perfectly accurate, the sensor in fact wiggles
slightly from where it is assumed to be as it travels through space and
appends rows of pixels to the image. This results in slight errors in
the final DEM created using stereo. Those are most clearly seen in the
intersection error map output by invoking ``point2dem --errorimage``.

ASP provides support for correcting this jitter, at least its
lower-frequency component. During stereo, right before the triangulation
step, so after the left-to-right image disparity is computed, it can
solve for adjustments to apply to the satellite position and
orientation. Those adjustments are placed along-track (hence at several
lines in the image) with interpolation between them. This is quite
analogous to what ``bundle_adjust`` is doing, except that the latter
uses just one adjustment for each image.

This process can be triggered by invoking ``parallel_stereo`` with
``--image-lines-per-piecewise-adjustment arg``. A recommended value here
is 1000, though it is suggested to try several values. A smaller value
of ``arg`` will result in more adjustments being used (each adjustment
being responsible for fewer image lines), hence providing finer-grained
control, though making this number too small may result in over-fitting
and instability. A smaller value here will also require overall more
interest point matches (as computed from the disparity), which is set
via ``--num-matches-for-piecewise-adjustment``.

Jitter correction is more effective if ``parallel_stereo`` is preceded by bundle
adjustment, with the adjusted cameras then being passed to ``parallel_stereo``
via ``--bundle-adjust-prefix``.

If it appears that the adjustments show some instability at the starting
and ending lines due to not enough matches being present (as deduced
from examining the intersection error image), the locations of the first
and last adjustment (and everything in between) may be brought closer to
each other, by modifying ``--piecewise-adjustment-percentiles``. Its
values are by default 5 and 95, and could be set for example to 10 and
90. For very tall images, it may be desirable to use instead values
closer to 0 and 100.

See :numref:`triangulation_options` for the full list of parameters
used in jitter correction.

In order for jitter correction to be successful, the disparity map
(``*-F.tif``) should be of good quality. If that is not the case, it is
suggested to redo stereo, and use, for example, map-projected images,
and in the case of terrain lacking large scale features, the value
``corr-seed-mode 3`` (:numref:`sparse-disp`).

An illustration of jitter correction is given in :numref:`jitter-example`.

.. _jitter-example:

.. figure:: images/jitter.jpg
   :alt: Intersection error map before (left) and after jitter correction.
   :name: fig:jitter-example

   Example of a colorized intersection error map before (left) and after
   jitter correction.


.. _casp_go:

The CASP-GO stereo processing system
------------------------------------

CASP-GO (https://github.com/mssl-imaging/CASP-GO) is a set of
algorithms that are meant to augment certain parts of ASP
:cite:`tao2016optimised, tao2018massive`. Under NASA proposal
19-PDART19_2-0094 we researched incorporating these into ASP.

CASP-GO consists of three algorithms:

- Gotcha disparity refinement. Its purpose is to fix artifacts in
  ASP's older ``asp_bm`` block-matching algorithm
  (:numref:`stereo_algos_full`) at the disparity
  filtering stage. It takes as input and overwrites the ``F.tif``
  disparity (which is described in :numref:`outputfiles`). 
  This algorithm definitely provides some in-filling functionality over
  the older ``asp_bm`` performance, but users may want to experiment
  with other ASP stereo algorithms (like MGM) which may also result in high
  quality disparities.  This logic can be turned on with the 
  ``--gotcha-disparity-refinement`` option of
  ``parallel_stereo``. See below for the parameters which control it.

- Image alignment. This component uses feature detection to find
  interest point matches among orthoimages associated with given DEMs,
  which is then used to compute an alignment transform among the DEMs
  :cite:`sidiropoulos2018automatic`. It was incorporated into ASP and
  further extended as the ``image_align`` tool (:numref:`image_align`).

- Kriging. This logic is meant to produce DEMs with
  fewer holes than ASP's older method in ``point2dem`` (:numref:`point2dem`)
  which used a Delaunay triangulation. It is based on a technique
  called ``kriging``, which is a family of generalized linear least
  square regression algorithms (:cite:`remy2002gstl`), implemented in
  the ``Geostatistics Template Library`` (http://gstl.sourceforge.net/).

  The CASP-GO DEM-creation algorithm functions along the same lines as ASP's
  recent and default implementation in ``point2dem``. The input is a 
  point cloud, the output is a gridded DEM, and weighted averaging
  is used to combine the 3D points to produce the DEM.

  The only difference is that the recent ``point2dem`` implementation (circa 3.1.0)
  computes the weights based on a Gaussian with given sigma and
  neighborhood size, while CASP-GO uses weights produced by the kriging
  procedure with a user-specified covariance.

  CASP-GO's covariance function assigns the same covariance value to all 
  points, which results in the kriging procedure returning constant
  weights. In effect, the resulting algorithm is a particular case of the
  modern approach in ``point2dem``, when the sigma value is very large.

  Thus, no separate implementation for kriging was implemented at this time.

.. For that reason, while kriging seems to be a very interesting technique,
   because CASP-GO did not implement a good covariance function, and since
   it would be quite tricky to assign a nontrivial covariance to
   points in a cloud, we chose to not incorporate this implementation,
   as it does not add to the existing functionality.

The CASP-GO parameter file
~~~~~~~~~~~~~~~~~~~~~~~~~~

CASP-GO's behavior is controlled by a parameter file, which ASP ships
as ``share/CASP-GO_params.xml``, and which can be overridden
with the ``parallel_stereo`` option ``--casp-go-param-file``.

Only the parameters relevant for Gotcha disparity refinement are read
from this file, as we did not implement the kriging algorithm,
and the ``image_align`` tool we added has its own interface.

Here are two sets of values for these parameters, optimized for CTX and
HiRISE cameras, respectively.

CTX::

  ALSC iterations: 8
  Max. eigenvalue: 150
  ALSC kernel:     21
  Grow neighbor:   8

HiRISE::

  ALSC iterations: 8
  Max. eigenvalue: 80
  ALSC kernel:     11
  Grow neighbor:   8



