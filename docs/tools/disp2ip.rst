.. _disp2ip:

disp2ip
-------

This program has some rather specialized functionality. It is meant to solve the
following problem. Consider a rover on a planetary body, with a stereo pair of
cameras. To save on bandwidth, only the left raw image and filtered versions of
both left and right images are transmitted to Earth. The filter is usually the
sign of the Laplacian of Gaussian (sLoG), or the Laplacian of Gaussian (LoG).

Filtered images are enough for stereo, but not for interest point matching
between images acquired at different times (rather than simultaneously). This
makes it problematic to find the relative poses of the cameras, which is needed
for ``rig_calibrator`` (:numref:`rig_calibrator`) or bundle adjustment
(:numref:`bundle_adjust`).

The solution to find interest point matches only between the left raw images,
then to use the disparities between the left and right filtered images to find
versions of these interest points in the right images, producing a set of
matches between all filtered images. This is the purpose of this program.

Procedure
~~~~~~~~~

Stereo between filtered images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is assumed by this program that the stereo runs were created with
``affineepipolar`` alignment and that cameras may not be available. This program
uses the disparities ``F.tif`` (:numref:`outputfiles`) as input.

Hence, ``parallel_stereo`` (:numref:`parallel_stereo`) should be invoked, for 
stereo pair with index ``i``, along the lines of::

  parallel_stereo                      \
    --correlator-mode                  \
    --prefilter-mode 0                 \
    --stereo-algorithm asp_bm          \
    --sgm-collar-size 0                \
    --alignment-method affineepipolar  \
    fltr/left/left_filtered_${i}.png   \
    fltr/right/right_filtered_${i}.png \
    stereo_${i}/run

The option ``--prefilter-mode 0`` is very important, if the images
are already filtered, otherwise a second filter would be applied on top.
See :numref:`stereodefault` for more details.

It may be hard to find features in LoG-filtered images so consider using very
large values of ``--ip-per-tile`` and ``--matches-per-tile``, such as 50000 or
more (this must be decreased for large images).

SfM between the left raw images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Structure-from-Motion (SfM) finds the interest point matches between the left
raw images and the relative poses of the cameras::

  theia_sfm --rig_config rig_config.txt \
    --theia_flags theia_flags.txt       \
    --images 'raw/left/left_raw*.png'   \
    --out_dir theia_left

This will produce the NVM file named ``theia_left/cameras.nvm``.

See :numref:`rig_calibrator_example` for more details.

Running this program
^^^^^^^^^^^^^^^^^^^^

Several lists should be prepared, and these must be in one-to-one correspondence::

  ls fltr/left/left_filtered*.png   > left_filtered.txt
  ls fltr/right/right_filtered*.png > right_filtered.txt
  ls raw/left/left_raw*.png         >  left_raw.txt

  ls stereo_*/run-F.tif | perl -p -e 's/-F\.tif//' > stereo.txt

Also ensure that the optical centers for all images are available in a file, as
expected by the option ``--optical-center-list``. For example, this may work,
with the right values::

    # Left images
    for f in $(cat left_filtered.txt); do 
      echo $f 1064 1025 
    done > optical_centers.txt
    # Append the right images
    for f in $(cat right_filtered.txt); do 
      echo $f 1055 1032 
    done >> optical_centers.txt

Then, run::

    disp2ip                                          \
      --left-raw-image-list left_raw.txt             \
      --left-filtered-image-list left_filtered.txt   \
      --right-filtered-image-list right_filtered.txt \
      --stereo-prefix-list stereo.txt                \
      --optical-center-list optical_centers.txt      \
      --input-nvm theia_left/cameras.nvm             \
      --output-nvm combined.nvm
 
The interest points in the input NVM file are assumed to be shifted relative to
the optical center of those images, with the file
``theia_left/camera_offsets.txt`` (given the earlier notation) having those
optical centers. The program will shift the produced interest points relative to
the optical centers as well, creating ``combined_offsets.txt``.

.. _disp2ip_rig:

Use of results
~~~~~~~~~~~~~~
 
The ``rig_calibrator`` program (:numref:`rig_calibrator`), when called with the
produced interest point matches, must use the option
``--use_initial_rig_transforms``, and the rig configuration in ``--rig_config``
must have valid transforms between the sensors (field
``ref_to_sensor_transform``). That is because ``disp2ip`` is unable to produce
the correct poses for the camera images it adds, and those are populated with a
nominal value.

The value of this transform is best determined with the ``rig_calibrator`` program
itself, by running it with the left and right raw images, that here are not
available. The following observed value worked well for a stereo rig::

    ref_to_sensor_transform: 1 0 0 0 1 0 0 0 1 -0.14 0 0

This will be refined when ``rig_calibrator`` is run.
 
The input and produced interest point matches can be inspected with ``stereo_gui`` 
(:numref:`stereo_gui_nvm`).  

.. _disp2ip_filter:

The input filter
~~~~~~~~~~~~~~~~

This program will run equally well if the LoG filter is applied to them (:numref:`stereodefault`), the sign of LoG, or no filter at all. 

What is important is for the left and right filtered images to be consistent, so
a reliable disparity map can be produced, and that the left raw images are
available.

Command-line options
~~~~~~~~~~~~~~~~~~~~

--left-raw-image-list <string>
    File containing the list of raw left images, one per line.

--left-filtered-image-list <string>
    File containing the list of left images after applying the LoG (Laplacian of
    Gaussian) filter, or some other filter, one per line.
    
--right-filtered-image-list <string>
    File containing the list of right images after applying the LoG filter, 
    or some other filter, one per line.
    
--stereo-prefix-list <string>
    File containing the list of stereo prefixes, one per line. Each prefix is
    for a stereo run with a left filtered and right filtered image, with affine epipolar
    alignment. Stereo could have been run with ``--correlator-mode``, so without
    cameras.

--optical-center-list <string>
    File containing the list of optical centers for all filtered images, in pixels.
    On each line must have the image name, optical center column, then row.
    The order of images is not important.
    
--input-nvm <string>
    Input NVM file, having interest point matches between the left raw images.
    
--output-nvm <string>
    Output NVM file, having interest point matches between all filtered
    images, produced with the help of disparity maps.
