.. _cassis:

TGO CaSSIS
----------

The Colour and Stereo Surface Imaging System (*CaSSIS*) is the high-resolution
imager on the ESA `ExoMars Trace Gas Orbiter
<https://en.wikipedia.org/wiki/ExoMars_Trace_Gas_Orbiter>`_ (TGO). It is a
push-frame instrument: it acquires the surface as a sequence of short 2D
framelets rather than a single continuous push-broom line. Each observation
carries two looks of the same ground (a nadir and an oblique pass), which makes
it a stereo instrument.

ASP does not model CaSSIS as a push-broom sensor. Instead, the strip is
decomposed into per-framelet *frame* cameras, each carrying a single frozen
transverse-distortion lens shared across all framelets. These baby-frame cameras
are bundle-adjusted to a CTX reference DEM (heights-from-dem plus a
camera-position leash), the cross-look framelet pairs are correlated in the
mapprojected domain, and the per-pair DEMs are mosaicked. The result is a DEM
registered to CTX with no per-framelet seams.

The example below is for the Jezero crater observation ``MY36_016378_162``, at 18
m. All three panels are cropped to the same footprint and put on one grid.

Comparison to the vendor DTM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The official CaSSIS-team DTM for this observation is delivered in an
equirectangular projection on an ellipsoid, while our reference (CTX) is
stereographic on a sphere. To compare the two fairly, the vendor DTM is
reprojected into the CTX frame, given a vertical shift, and horizontally aligned
to CTX by dense hillshade correlation (a rigid, no-tilt alignment). A
point-to-plane ICP tilt correction was also tried but was rejected: it does not
remove the residual and instead introduces a large lateral slide. What remains
after the best rigid alignment is a genuine large-scale warp in the vendor DTM,
not a registration error.

.. figure:: ../images/cassis_jezero_hillshade.png
   :name: cassis_jezero_hillshade
   :alt: Jezero hillshades: CTX, vendor CaSSIS, our CaSSIS

   Hillshades at 18 m, same footprint and grid. Left: the CTX reference. Middle:
   the vendor CaSSIS DTM (aligned to CTX). Right: our CaSSIS DEM. Our DEM
   reproduces the CTX detail; the vendor DTM is visibly coarser and smoother.

.. figure:: ../images/cassis_jezero_geodiff.png
   :name: cassis_jezero_geodiff
   :alt: Elevation difference to CTX

   Elevation difference to CTX at 18 m (note the different color scales). Left:
   vendor CaSSIS minus CTX, median -3.7 m, NMAD 22.4 m -- a smooth large-scale
   warp that no rigid alignment removes. Right: our CaSSIS minus CTX, median 0.08
   m, NMAD 1.84 m. Our DEM is about 12 times tighter to CTX than the vendor DTM.

.. figure:: ../images/cassis_jezero_dd.png
   :name: cassis_jezero_dd
   :alt: Horizontal registration of our DEM to CTX

   Horizontal registration of our CaSSIS DEM to CTX at 18 m, measured by
   correlating the two hillshades: the horizontal (dd-H) and vertical (dd-V)
   disparity in pixels. Both are sub-pixel everywhere (mean 0.04 / 0.02 px, NMAD
   0.55 / 0.54 px), confirming the DEM is registered to CTX with no seams.
