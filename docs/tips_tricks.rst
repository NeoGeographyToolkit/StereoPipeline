.. _tips:

Tips and tricks
===============

Here we summarize, in one place, some insights in how to get the most
from ASP, particularly the highest quality results in the smallest
amount of time.

-  Ask for help or if you have questions. We're always glad to share
   what we know, implement suggestions, and fix issues (:numref:`get-help`).

-  Use the GUI (:numref:`stereo_gui`) to get
   comfortable with ASP on a small region and to tune parameters
   (:numref:`stereo_gui`). A solution specific to ISIS
   images is to crop your stereo pair (using the ISIS ``crop`` command being
   careful to retain SPICE information, or ``cam2map4stereo.py`` for map-projected
   images) to a small region of interest.

-  The highest quality results with ASP can be obtained with
   map-projected images (:numref:`mapproj-example`).

-  Run stereo on multiple machines (:numref:`parallel_stereo`).

-  Improve the quality of the inputs to get better outputs.
   Bundle-adjustment can be used to find out the camera positions more
   accurately (:numref:`baasp`). CCD artifact correction
   can be used to remove artifacts from WorldView images
   (:numref:`wvcorrect-example`). 

-  Align the output point cloud to some known absolute reference with
   ``pc_align`` (:numref:`pc-align-example`).

-  Remove noise from the output point cloud. During stereo
   triangulation, points that are further or closer than given distances
   from planet center or left camera center can be removed as outliers
   (:numref:`triangulation_options`).
   During DEM generation (:numref:`point2dem`), points
   with large triangulation error can be removed using
   ``--remove-outliers-params``. Spikes can be removed with
   ``--median-filter-params``. Points close to the boundary, that tend
   to be less accurate, can be eroded (``--erode-length``).

-  During stereo filtering, islands can be removed with
   ``--erode-max-size``.

-  Remove noise from the low-resolution disparity (D_sub) that can
   greatly slow down a run using ``--rm-quantile-percentile`` and
   ``--rm-quantile-multiple``. Some care is needed with these to not
   remove too much information.

-  Fill holes in output orthoimages for nicer display (also in DEMs),
   during DEM and orthoimage generation with ``point2dem`` (:numref:`point2dem`).
   Holes in an existing DEM can also be
   filled using ``dem_mosaic`` (:numref:`dem_mosaic`).

-  To get good results if the images lack large-scale features (such as
   for ice plains) use a different way to get the low-resolution
   disparity (:numref:`sparse_disp`).

-  If a run takes unreasonably long, decreasing the timeout parameter
   may be in order (:numref:`longrun`).

-  Manually set the search range if the automated approach fails
   (:numref:`search_range`).

-  To increase speed, the image pair can be subsampled. For ISIS
   images, the ISIS ``reduce`` command can be used, while for DigitalGlobe/Maxar 
   data one can invoke the ``dg_mosaic`` tool (:numref:`dg_mosaic`,
   though note that this tool may introduce
   aliasing). With subsampling, you are trading resolution for speed, so
   this probably only makes sense for debugging or "previewing" 3D
   terrain. That said, subsampling will tend to increase the signal to
   noise ratio, so it may also be helpful for obtaining 3D terrain out
   of noisy, low quality images.

-  Photometric calibration (using the ISIS tools) can be used to improve
   the input images and hence get higher quality stereo results.

-  If your images have missing or inaccurate camera pose information, and they
   were acquired with frame (pinhole cameras), such data can be solved for using
   structure-from-motion and bundle adjustment (:numref:`sfm`).

-  Shape-from-shading (:numref:`sfs`) can be used to
   further increase the level of detail of a DEM obtained from stereo,
   though this is a computationally expensive process and its results
   are not easy to validate.

We'll be happy to add here more suggestions from community's accumulated
wisdom on using ASP.
