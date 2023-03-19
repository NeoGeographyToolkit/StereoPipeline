.. _sfm_merge:

sfm_merge
---------

TODO(oalexan1): Accept shifted nvm!
TODO(oalexan1): Remove debug info when merging poses, but test first!
TODO(oalexan1): Allow image lists on input.

The ``sfm_merge`` program takes as two or more Structure-from-Motion
(SfM) maps in .nvm format, as produced by ``theia_sfm``
(:numref:`theia_sfm`) or ``rig_calibrator``
(:numref:`rig_calibrator`), and merges them into a single map,
by transforming to first map's coordinate system the other maps.

For the moment, it works only with .nvm files in which the keypoints
are not shifted relative to the optical center.  This
property is preserved by this tool.

The produced map must be bundle-adjusted to refine it, using
``rig_calibrator`` (with or without the rig constraint).

The output map can be visualized in ``stereo_gui``
(:numref:`stereo_gui_nvm`).

Example::

    sfm_merge --num_image_overlaps_at_endpoints 100 \
      map1.nvm map2.nvm -output_map merged.nvm

Command-line options for sfm_merge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``--rig_config``
  Read the configuration of sensors from this file in the format used for 
  ``rig_calibrator`` (this tool does not use the rig structure). The
  output of this program can be passed back to ``rig_calibrator``
  (with or without a rig constraint).

``--output_map``
  Output file containing the merged map.

``--num_image_overlaps_at_endpoints``
  Search this many images at the beginning and end of the first map 
  for matches to this many images at the beginning and end of the 
  second map. Default: 10,

``--fix_first_map``
  If true, after merging the maps and reconciling the camera poses in
  shared images, overwrite the shared poses with those from the first map.

``--no_shift``
  Assume that in the input .nvm files the features are not shifted
  relative to the optical center. The merged map will then be saved
  the same way. The usual behavior is that .nvm file features are
  shifted, then this tool internally undoes the shift.

``--close_dist``
  Two triangulated points are considered to be close if no further
  than this distance, in meters. Used as inlier threshold when
  identifying triangulated points after the maps are
  aligned. Auto-computed, taking into account the extent of
  a tight subset of the triangulated points and printed on screen if
  not set. This is an advanced option.
  
