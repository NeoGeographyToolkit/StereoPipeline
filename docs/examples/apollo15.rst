.. _apollo15_example:

Apollo 15 Metric Camera images
------------------------------

Apollo Metric images were all taken at regular intervals, which means
that the same ``stereo.default`` can be used for all sequential pairs of
images. Apollo Metric images are ideal for stereo processing. They
produce consistent, excellent results.

The scans performed by ASU are sufficiently detailed to exhibit film
grain at the highest resolution. The amount of noise at the full
resolution is not helpful for the correlator, so we recommend
subsampling the images by a factor of 4.

Currently the tools to ingest Apollo TIFFs into ISIS are not available,
but these images should soon be released into the PDS for general public
usage.

Ansgarius C
~~~~~~~~~~~

Ansgarius C is a small crater on the west edge of the far side of the
Moon near the equator. It is east of Kapteyn A and B.

.. figure:: ../images/examples/metric/metric_ge_example_combined.png
   :name: metric_example

   Example output possible with Apollo Metric frames AS15-M-2380 and AS15-M-2381.

Commands
^^^^^^^^

Process Apollo TIFF files into ISIS.

::

     ISIS> reduce from=AS15-M-2380.cub to=sub4-AS15-M-2380.cub \
             sscale=4 lscale=4
     ISIS> reduce from=AS15-M-2381.cub to=sub4-AS15-M-2381.cub \
             sscale=4 lscale=4
     ISIS> spiceinit from=sub4-AS15-M-2380.cub
     ISIS> spiceinit from=sub4-AS15-M-2381.cub
     ISIS> parallel_stereo sub4-AS15-M-2380.cub sub4-AS15-M-2381.cub \
             result/output

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices.
