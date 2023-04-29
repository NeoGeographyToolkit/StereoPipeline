.. include:: <isonum.txt>

.. _examples:

Stereo processing examples
==========================

This chapter showcases a variety of results that are possible when
processing different data sets with the Stereo Pipeline. It is also a
shortened guide that shows the commands used to process specific mission
data. There is no definitive method yet for making elevation models as
each stereo pair is unique. We hope that the following sections serve as
a cookbook for strategies that will get you started in processing your
own data. We recommend that you second check your results against
another source.

Structure-from-Motion examples are in :numref:`rig_examples` (using a rig and robot images)
and in :numref:`sfm` (for orbital images with no rig).

.. toctree::

   examples/stereo_pairs
   examples/hirise
   examples/ctx
   examples/moc
   examples/mer
   examples/k10
   examples/lronac
   examples/lrowac
   examples/apollo15
   examples/hrsc
   examples/cassini
   examples/csm
   examples/dawn
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

.. _rig_examples:

SfM examples using a robot rig
==============================

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
