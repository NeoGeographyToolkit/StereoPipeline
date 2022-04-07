.. _correlator-mode:

Image correlator
----------------

The ``parallel_stereo`` program (:numref:`parallel_stereo`) can
function purely as an image correlator, that is, it can find the
disparity between two given images without assuming any cameras are
present and without generating a point cloud.

This mode assumes that the images are already roughly aligned, up to a
translation. Hence, the images can be either raw but with no large
rotation/shear/scale differences, or mapprojected, or to be portions
of aligned images ``L.tif`` and ``R.tif`` (:numref:`outputfiles`) as
created by ASP itself.

See also ``corr_eval`` (:numref:`corr_eval`) a program for estimating
the quality of the disparity at each pixel.

Usage::

    parallel_stereo --correlator-mode <left image> <right image> \
      <output prefix>

Example::

    parallel_stereo --correlator-mode run/run-L.tif run/run-R.tif \
      run_corr/run

This will create the filtered subpixel disparity
``run_corr/run-F.tif``.

All the usual options of ``parallel_stereo`` apply
(:numref:`nextsteps`, :numref:`parallel_stereo`, and
:numref:`stereodefault`). Since the images are assumed to be aligned,
the program will set the alignment method to ``none``. 

If desired to not use an initial low-resolution correlation, set
``--corr-seed-mode 0``. To skip preprocessing (if invoked previously),
or to avoid subpixel refinement or filtering, use the options
``--entry-point`` and ``--stop-point``.

