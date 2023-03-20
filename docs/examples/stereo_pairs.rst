.. _stereo_pairs:

Guidelines for selecting stereo pairs
-------------------------------------

When choosing image pairs to process, images that are taken with
similar lighting conditions and significant surface coverage overlap
are best suited for creating terrain models
:cite:`2015LPI462703B`. The images should have sufficient difference
in perspective, hence a reasonably large baseline, or, equivalently, a
non-small convergence angle between the matching rays emanating from
the two cameras, for stereo triangulation to be accurate. Yet, if the
perspectives are very different, it will be challenging to compute the
stereo correlation between images. A convergence angle of 10 to 60
degrees is likely reasonable. 

Depending on the characteristics of the mission data set and the
individual images, the degree of acceptable variation will
differ. Significant differences between image characteristics
increases the likelihood of stereo matching error and artifacts, and
these errors will propagate through to the resulting data products.

The ``parallel_stereo`` and ``bundle_adjust`` programs compute the
convergence angle for input cameras. In stereo that happens at the
preprocessing stage (:numref:`entrypoints`), with the result
printed on the screen and saved to the log file. In ``bundle_adjust``
this computation takes place after the optimization of the cameras
finishes, and the results are saved to a file on disk
(:numref:`ba_out_files`). To find good stereo pairs, one can run
bundle adjustment on a large set of images and pick a pair with a
decent convergence angle.

Although images do not need to be mapprojected before running the
``parallel_stereo`` program, we recommend that you do run ``cam2map`` (or
``cam2map4stereo.py``) beforehand, especially for image pairs that
contain large topographic variation (and therefore large disparity
differences across the scene, e.g., Valles Marineris). mapprojection is
especially necessary when processing HiRISE images. This removes the
large disparity differences between HiRISE images and leaves only the
small detail for the Stereo Pipeline to compute. Remember that ISIS can
work backwards through a mapprojection when applying the camera model,
so the geometric integrity of your images will not be sacrificed if you
mapproject first.

An alternative way of mapprojection, that applies to non-ISIS images
as well, is with the ``mapproject`` tool (:numref:`mapproj-example`).

Excessively noisy images will not correlate well, so images should be
photometrically calibrated in whatever fashion suits your purposes. If
there are photometric problems with the images, those photometric
defects can be misinterpreted as topography.

Remember, in order for ``parallel_stereo`` to process stereo pairs in
ISIS cube format, the images must have had SPICE data associated by
running ISIS's ``spiceinit`` program run on them first.
