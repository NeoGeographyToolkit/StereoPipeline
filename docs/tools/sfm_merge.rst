.. _sfm_merge:

sfm_merge
---------

The ``sfm_merge`` program takes two or more Structure-from-Motion
(SfM) maps in .nvm format, as produced by ``theia_sfm``
(:numref:`theia_sfm`) or ``rig_calibrator``
(:numref:`rig_calibrator`), and merges them into a single map. It
finds correspondences (feature matches) between the images in the maps,
and then transforms the maps to first map's coordinate system.

The input maps may or may not have shared images, but the surfaces
they see must overlap.

If there are more than two maps, the second is merged to the first,
which is then merged with the third, etc.

The produced map must be bundle-adjusted to refine it, using
``rig_calibrator`` (with or without the rig constraint).

The output map can be visualized in ``stereo_gui``
(:numref:`stereo_gui_nvm`).

See also ``sfm_submap`` (:numref:`sfm_submap`), a program to extract
a submap from a larger map. A discussion for how this tool can be used
is in :numref:`map_surgery`.

Example
^^^^^^^

::

    sfm_merge --rig_config rig_config.txt   \
      --num_image_overlaps_at_endpoints 100 \
      map1.nvm map2.nvm -output_map merged.nvm

Increasing the number of overlaps can make the program quite slow as
its complexity is the square of this number. However, if very few
similar images are detected between the maps, they will not be merged
accurately.

The option ``--fast_merge`` can be used when the input maps are known
to have a good number of images in common. 

The option ``--no_transform`` is useful when the maps are
individually registered or when they do not overlap, but in either
case it is desired to integrate them without changing the camera
poses.

Unless the ``--no_shift`` option is used, this invocation will write
a file of the form ``<merged>_offsets.txt`` having the optical center of each
camera. The .nvm file has these offsets subtracted from the features,
and the offsets are needed for plotting the features with ``stereo_gui``.

Handling tracks
^^^^^^^^^^^^^^^

A track is a feature (interest point) seen in multiple images,
corresponding to a triangulated position on the ground. This tool
preserves the tracks from the input maps. Matching of features creates
additional tracks.  Duplicate tracks (showing in multiple input maps
and/or created during matching) are removed.

This tool does not merge tracks that have a subsequence in common, or
eliminate a track if another track exists which is longer than a given
one. These features may be added in future versions.

Command-line options for sfm_merge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

--rig_config <string (default: "")>
  Read the configuration of sensors from this file in the format used for 
  ``rig_calibrator`` (this tool does not use the rig structure). The
  output of this program can be passed back to ``rig_calibrator``
  (with or without a rig constraint).

--output_map <string (default: "")>
  Output file containing the merged map.

--num_image_overlaps_at_endpoints <integer (default: 10)>
  Search this many images at the beginning and end of the first map 
  for matches to this many images at the beginning and end of the 
  second map. Default: 10,

--fast_merge
    When merging maps that have shared images, use their camera poses to 
    find the transform from other maps to first map, and skip finding 
    additional matches among the images.

--fix_first_map
  If true, after merging the maps and reconciling the camera poses for
  the shared images, overwrite the shared poses with those from the
  first map, so it does not change. This is helpful if the first map
  is already registered and evaluated.

--no_shift
  Assume that in the input .nvm files the features are not shifted
  relative to the optical center. The merged map will then be saved
  the same way. 

--no_transform
  Do not compute and apply a transform from the other 
  maps to the first one. This keeps the camera poses as 
  they are (shared poses and features will be reconciled). 
  This will succeed even when the two maps do not overlap.

--close_dist <double (default: -1.0)>
  Two triangulated points are considered to be close if no further
  than this distance, in meters. Used as inlier threshold when
  identifying triangulated points after the maps are
  aligned. Auto-computed, taking into account the extent of
  a tight subset of the triangulated points and printed on screen if
  not set. This is an advanced option.
  
