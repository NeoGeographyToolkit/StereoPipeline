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

