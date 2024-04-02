.. _disp2ip:

disp2ip
-------

This program has some rather specialized functionality. Given a set of stereo
pairs, which may have the *LoG* (Laplacian of Gaussian, :numref:`stereodefault`)
filter applied to them, the disparities between images in each pair, and
interest point matches between the left images before this filter, it will
augment each interest point based on the appropriate disparity, producing
correspondences with the right images. Input and output interest point matches
are stored in NVM files.

It is assumed that the stereo runs were created with ``affineepipolar``
alignment and that cameras may not be available. This program uses the filtered
disparities ``F.tif`` (:numref:`outputfiles`) as input.

Hence, ``parallel_stereo`` (:numref:`parallel_stereo`) should be invoked, for 
stereo pair with index ``i``, along the lines of::

  parallel_stereo                        \
    --correlator-mode                    \
    --alignment-method affineepipolar    \
    left_log_${i}.tif right_log_${i}.tif \
    stereo_${i}/run

If the images have the LoG filter applied to them, ensure that options
``--prefilter-mode 0``, ``--stereo-algorithm asp_bm``, ``--sgm-collar-size 0``
are passed in (:numref:`stereodefault`).

It may be hard to find features in LoG-filtered images, so consider using
very large values of ``--ip-per-tile`` and ``--matches-per-tile``, such as 50000
or more (this must be decreased for large images).

Several lists should be prepared, and these must be in one-to-one correspondence::

  ls left_log*.png  > left_log.txt
  ls right_log*.png > right_log.txt
  ls left_raw*.png  > left_raw.txt

  ls stereo_*/run-F.tif | perl -p -e 's/-F\.tif//' > stereo.txt

Also ensure that the optical centers for all images are available in a file, as
expected by the option ``--optical-center-list``. 

Then, this program is invoked as::

    disp2ip                                     \
      --left-raw-image-list left_raw.txt        \
      --left-log-image-list left_log.txt        \
      --right-log-image-list right_log.txt      \
      --stereo-prefix-list stereo.txt           \
      --optical-center-list optical_centers.txt \
      --input-nvm left.nvm                      \
      --output-nvm combined.nvm
 
The interest points in the input NVM file are assumed to be shifted relative to
the optical center of those images, with the file ``left_offsets.txt`` (given
the earlier notation) having those optical centers. The program will shift the
produced interest points relative to the optical centers as well, creating
``combined_offsets.txt``.

Use of results
~~~~~~~~~~~~~~~
 
The NVM files follow the convention for ``rig_calibrator``
(:numref:`rig_calibrator_outputs`), and the output of this program is passed to
that tool. 

The ``rig_calibrator`` program, when called with the produced interest point
matches, must use the option ``--use_initial_rig_transforms``, and the rig
configuration in ``--rig_config`` must have valid transforms between the
sensors. That is because ``disp2ip`` is unable to produce the correct poses for
the camera images it adds, and those are populated with a nominal value.
 
The input and produced interest point matches can be inspected with ``stereo_gui`` 
(:numref:`stereo_gui_nvm`).  

Command-line options
~~~~~~~~~~~~~~~~~~~~

--left-raw-image-list <string>
    File containing the list of raw left images, one per line.

--left-log-image-list <string>
    File containing the list of left images after applying the LoG (Laplacian of
    Gaussian) filter, one per line.
    
--right-log-image-list <string>
    File containing the list of right images after applying the LoG filter, one
    per line.
    
--stereo-prefix-list <string>
    File containing the list of stereo prefixes, one per line. Each prefix is
    for a stereo run with a left log and right log image, with affine epipolar
    alignment. Stereo could have been run with ``--correlator-mode``, so without
    cameras.

--optical-center-list <string>
    File containing the list of optical centers for all log images, in pixels.
    On each line must have the image name, optical center column, then row.
    The order of images is not important.
    
--input-nvm <string>
    Input NVM file, having interest point matches between the left raw images.
    
--output-nvm <string>
    Output NVM file, having interest point matches between all LoG-filtered
    images, produced with the help of disparity maps.
