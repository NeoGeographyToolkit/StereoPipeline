.. _sfm_submap:

sfm_submap
----------

The ``sfm_submap`` program takes as input a Structure-from-Motion
(SfM) map in .nvm format, as produced by ``theia_sfm``
(:numref:`theia_sfm`) or ``rig_calibrator``
(:numref:`rig_calibrator`), and extracts from it a submap
for a desired subset of the images. 

It works with .nvm files in which the keypoints were shifted
relative to the optical center (which is the default for
both ``theia_sfm`` and ``rig_calibrator``) or not (as can be
saved by ``rig_calibrator``, see :numref:`rig_calibrator_outputs`).
This property is preserved by this tool.

The produced map can be visualized in ``stereo_gui``
(:numref:`stereo_gui_nvm`). For now, if the features are shifted
relative to the optical center, the user must produce the list of
shifts, as detailed at the above link, for the features to be plotted
correctly. It is suggested to always use ``rig_calibrator`` with the
option ``--save_nvm_no_shift`` to create an unshifted file to start
with.

Example (specify the images to have in the submap on the command line)::

    sfm_submap -input_map in_map.nvm -output_map out_map.nvm \
      image1.jpg image2.jpg ... 

Example (let those images be in a file, one per line)::

    sfm_submap -input_map in_map.nvm -output_map out_map.nvm \
      -image_list list.txt

The images in the list not present in the input map will be ignored.

Command-line options for sfm_submap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``--input_map`` The input map, in .nvm format. Type: string. Default: "".

``--output_map`` The output map, in .nvm format. Type: string. Default: "".

``--image_list`` A file having the names of the images to be included in
  the submap, one per line.

