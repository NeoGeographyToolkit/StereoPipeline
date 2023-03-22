.. _sfm_merge:

sfm_merge
---------

The ``sfm_merge`` program takes as two or more Structure-from-Motion
(SfM) maps in .nvm format, as produced by ``theia_sfm``
(:numref:`theia_sfm`) or ``rig_calibrator``
(:numref:`rig_calibrator`), and merges them into a single map,
by transforming to first map's coordinate system the other maps.

Tracks (sequences of matching interest points) in the input maps
that have shared interest points are merged as well.

The produced map must be bundle-adjusted to refine it, using
``rig_calibrator`` (with or without the rig constraint).

The output map can be visualized in ``stereo_gui``
(:numref:`stereo_gui_nvm`).

Example::

    sfm_merge --num_image_overlaps_at_endpoints 100 \
      map1.nvm map2.nvm -output_map merged.nvm

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

--fix_first_map
  If true, after merging the maps and reconciling the camera poses for
  the shared images, overwrite the shared poses with those from the first map.

--no_shift
  Assume that in the input .nvm files the features are not shifted
  relative to the optical center. The merged map will then be saved
  the same way. 

--close_dist <double (default: -1.0)>
  Two triangulated points are considered to be close if no further
  than this distance, in meters. Used as inlier threshold when
  identifying triangulated points after the maps are
  aligned. Auto-computed, taking into account the extent of
  a tight subset of the triangulated points and printed on screen if
  not set. This is an advanced option.
  