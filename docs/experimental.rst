.. _experimental:

Experimental features
=====================

.. _casp_go:

The CASP-GO stereo processing system
------------------------------------

CASP-GO (Co-registered ASP using Gotcha Optimisation,
https://github.com/mssl-imaging/CASP-GO) is a set of algorithms that
are meant to augment certain parts of ASP :cite:`tao2016optimised,
tao2018massive, shin2012progressively, otto1989region`. Thse algorithms were developed
at the Imaging Group, Mullard Space Science Laboratory, University
College London, by Yu Tao, under the direction of Jan-Peter Muller,
with funding from the EU-FP7 project titled "iMars: Analysis of Mars
Multi-Resolution Images using Auto-Coregistration, Data Mining and
Crowd Sourcing Techniques", under contract #607379.

Under NASA proposal 19-PDART19_2-0094 we researched incorporating
these algorithms into ASP.

CASP-GO consists of three algorithms:

- Gotcha disparity refinement. Its purpose is to fix artifacts in
  ASP's older ``asp_bm`` block-matching algorithm
  (:numref:`stereo_algos_full`) at the disparity
  filtering stage. It takes as input and overwrites the ``F.tif``
  disparity (which is described in :numref:`outputfiles`). 
  This algorithm definitely provides some in-filling functionality over
  the older ``asp_bm`` performance, but users may want to experiment
  with other ASP stereo algorithms (like MGM) which may also result in high
  quality disparities.  This logic can be turned on with the 
  ``--gotcha-disparity-refinement`` option of
  ``parallel_stereo``. See below for the parameters which control it.

- Image alignment. This component uses feature detection to find
  interest point matches among orthoimages associated with given DEMs,
  which is then used to compute an alignment transform among the DEMs
  :cite:`sidiropoulos2018automatic`. It was incorporated into ASP and
  further extended as the ``image_align`` tool (:numref:`image_align`).

- Kriging. This logic is meant to produce DEMs with
  fewer holes than ASP's older method in ``point2dem`` (:numref:`point2dem`)
  which used a Delaunay triangulation. It is based on a technique
  called ``kriging``, which is a family of generalized linear least
  square regression algorithms (:cite:`remy2002gstl`), implemented in
  the ``Geostatistics Template Library`` (http://gstl.sourceforge.net/).

  The CASP-GO DEM-creation algorithm functions along the same lines as ASP's
  recent and default implementation in ``point2dem``. The input is a 
  point cloud, the output is a gridded DEM, and weighted averaging
  is used to combine the 3D points to produce the DEM.

  The only difference is that the recent ``point2dem`` implementation (circa 3.1.0)
  computes the weights based on a Gaussian with given sigma and
  neighborhood size, while CASP-GO uses weights produced by the kriging
  procedure with a user-specified covariance.

  CASP-GO's covariance function assigns the same covariance value to all 
  points, which results in the kriging procedure returning constant
  weights. In effect, the resulting algorithm is a particular case of the
  modern approach in ``point2dem``, when the sigma value is very large.

  Thus, no separate implementation for kriging was implemented at this time.

.. For that reason, while kriging seems to be a very interesting technique,
   because CASP-GO did not implement a good covariance function, and since
   it would be quite tricky to assign a nontrivial covariance to
   points in a cloud, we chose to not incorporate this implementation,
   as it does not add to the existing functionality.

The CASP-GO parameter file
~~~~~~~~~~~~~~~~~~~~~~~~~~

CASP-GO's behavior is controlled by a parameter file, which ASP ships
as ``share/CASP-GO_params.xml``, and which can be overridden
with the ``parallel_stereo`` option ``--casp-go-param-file``.

Only the parameters relevant for Gotcha disparity refinement are read
from this file, as we did not implement the kriging algorithm,
and the ``image_align`` tool we added has its own interface.

Here are two sets of values for these parameters, optimized for CTX and
HiRISE cameras, respectively.

CTX::

  ALSC iterations: 8
  Max. eigenvalue: 150
  ALSC kernel:     21
  Grow neighbor:   8

HiRISE::

  ALSC iterations: 8
  Max. eigenvalue: 80
  ALSC kernel:     11
  Grow neighbor:   8



