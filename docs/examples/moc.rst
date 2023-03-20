.. _moc_example:

Mars Global Surveyor MOC-NA
---------------------------

In the Stereo Pipeline Tutorial in :numref:`moc_tutorial`, we showed
you how to process a narrow angle MOC stereo pair that covered a
portion of Hrad Vallis. In this section we will show you more
examples, some of which exhibit a problem common to stereo pairs from
linescan imagers: ``spacecraft jitter`` is caused by oscillations of
the spacecraft due to the movement of other spacecraft hardware. All
spacecraft wobble around to some degree but some are particularly
susceptible.

Jitter causes wave-like distortions along the track of the satellite
orbit in DEMs produced from linescan camera images. This effect can be
very subtle or quite pronounced, so it is important to check your data
products carefully for any sign of this type of artifact. The following
examples will show the typical distortions created by this problem.

Note that the science teams of HiRISE and LROC are actively working on
detecting and correctly modeling jitter in their respective SPICE data.
If they succeed in this, the distortions will still be present in the
raw images, but the jitter will no longer produce ripple artifacts in
the DEMs produced using ours or other stereo reconstruction software.

ASP has its own jitter solver, which was shown to reduce the jitter
for CTX (Mars) and DigitalGlobe (Earth) data (:numref:`jitter_solve`).

Ceraunius Tholus
~~~~~~~~~~~~~~~~

Ceraunius Tholus is a volcano in northern Tharsis on Mars. It can be
found at 23.96 N and 262.60 E. This DEM crosses the volcano's caldera.

.. figure:: ../images/examples/mocna/ceraunius_tholus_mocna_ge_combined.png
   :name: mocna_ceraunius_example

   Example output for MOC-NA of Ceraunius Tholus. Notice the presence
   of severe washboarding artifacts due to spacecraft jitter.

.. _commands-2:

Commands
^^^^^^^^

Download the M08/06047 and R07/01361 images from the PDS.

::

     ISIS> moc2isis f=M0806047.img t=M0806047.cub
     ISIS> moc2isis f=R0701361.img t=R0701361.cub
     ISIS> spiceinit from=M0806047.cub
     ISIS> spiceinit from=R0701361.cub
     ISIS> cam2map4stereo.py M0806047.cub R0701361.cub
     ISIS> parallel_stereo M0806047.map.cub R0701361.map.cub result/output

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices. See :numref:`examples` for other examples.

