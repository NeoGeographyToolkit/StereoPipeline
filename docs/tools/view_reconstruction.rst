.. _view_reconstruction:

view_reconstruction
-------------------

This viewer is part of the Theia SfM suite of tools that ASP ships
(:numref:`theia_sfm`). It can show a 3D point cloud and the camera poses. See
``sfm_view`` (:numref:`sfm_view`) for a program that can show orbital cameras
(with no 3D points).

Zoom in and out with the mouse wheel. Pan with the left mouse button. Rotate
with the right mouse button. 

Example::

    view_reconstruction --reconstruction theia/reconstruction-0 

.. figure:: ../images/view_reconstruction.png
   :name: view_reconstruction_fig
   :alt:  view_reconstruction_tag
   
   Illustration of ``view_reconstruction``. The camera poses are shown as
   red pyramids. The triangulated points are in gray.

