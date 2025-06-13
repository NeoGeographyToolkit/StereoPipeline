.. include:: <isonum.txt>

.. _examples:

Stereo processing examples
==========================

This chapter showcases examples of processing data sets acquired with specific
instruments. For a general introduction, see the tutorial (:numref:`tutorial`).

Structure-from-Motion examples are in :numref:`sfm` (for orbital images with no
rig), and :numref:`rig_examples` (using a rig and robot images).

.. toctree::

   examples/stereo_pairs
   examples/hirise
   examples/ctx
   examples/moc
   examples/mer
   examples/k10
   examples/lronac
   examples/change3
   examples/apollo15
   examples/hrsc
   examples/cassini
   examples/csm
   examples/dawn
   examples/kaguya
   examples/chandrayaan2
   examples/junocam
   examples/isis_minirf
   examples/pbs_slurm
   examples/aster
   examples/dg
   examples/rpc
   examples/perusat1
   examples/pleiades
   examples/spot5
   examples/skysat
   examples/historical
   examples/bathy
   examples/umbra_sar

.. _sfm:

SfM examples
============
   
.. toctree::
   
   sfm

.. _rig_examples:

SfM examples with a rig
=======================

These examples shows how to solve for camera poses using
Structure-from-Motion (SfM) and then create textured meshes. 

The images are acquired using a rig mounted on a robot on the ISS
(:numref:`rig_calibrator_example`, :numref:`sfm_iss`) and with the MSL
Curiosity rover (:numref:`rig_msl`).

Somewhat related examples, but without using a rig or the above
workflow, are in :numref:`sfm` (the images are acquired in orbit using
a satellite and a DEM is produced) and :numref:`mer-example` (a basic and
rather old two-image example for the MER rovers). See also :numref:`csm_msl`
for an example using CSM cameras for the MSL rover, without employing SfM.

.. toctree::

   examples/rig
   examples/sfm_iss
   examples/msl
   
Shape-from-Shading
==================

.. toctree::

   sfs_usage
   examples/sfs_earth
   examples/sfs_ctx
