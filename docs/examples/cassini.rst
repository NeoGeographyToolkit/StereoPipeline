.. _cassini_example:

Cassini ISS NAC
---------------

This is a proof of concept showing the strength of building the Stereo
Pipeline on top of ISIS. Support for processing ISS NAC stereo pairs was
not a goal during our design of the software, but the fact that a camera
model exists in ISIS means that it too can be processed by the Stereo
Pipeline.

Identifying stereo pairs from spacecraft that do not orbit their target
is a challenge. We have found that one usually has to settle with images
that are not ideal: different lighting, little perspective change, and
little or no stereo parallax. So far we have had little success with
Cassini's data, but nonetheless we provide this example as a potential
starting point.

Rhea
~~~~

Rhea is the second largest moon of Saturn and is roughly a third the
size of our own Moon. This example shows, at the top right of both
images, a giant impact basin named Tirawa that is 220 miles across. The
bright white area south of Tirawa is ejecta from a new crater. The lack
of texture in this area poses a challenge for our correlator. The
results are just barely useful: the Tirawa impact can barely be made out
in the 3D data while the new crater and ejecta become only noise.

.. figure:: ../images/examples/cassini/cassini_rhea_quad.png
   :name: cassini-example

   Example output of what is possible with Cassini's ISS NAC.  Upper left:
   original left image.  Upper right: original right image.  Lower left: 
   mapprojected left image.  Lower right: 3D Rendering of the point cloud.

Commands
^^^^^^^^

Download the N1511700120_1.IMG and W1567133629_1.IMG images and their
label (.LBL) files from the PDS.

::

     ISIS> ciss2isis f=N1511700120_1.LBL t=N1511700120_1.cub
     ISIS> ciss2isis f=W1567133629_1.LBL t=W1567133629_1.cub
     ISIS> cisscal from=N1511700120_1.cub to=N1511700120_1.lev1.cub
     ISIS> cisscal from=W1567133629_1.cub to=W1567133629_1.lev1.cub
     ISIS> fillgap from=W1567133629_1.lev1.cub to=W1567133629_1.fill.cub

(Note the optional ``fillgap`` command above.)
                                                                        
::

     ISIS> cubenorm from=N1511700120_1.lev1.cub to=N1511700120_1.norm.cub
     ISIS> cubenorm from=W1567133629_1.fill.cub to=W1567133629_1.norm.cub
     ISIS> spiceinit from=N1511700120_1.norm.cub
     ISIS> spiceinit from=W1567133629_1.norm.cub
     ISIS> cam2map from=N1511700120_1.norm.cub to=N1511700120_1.map.cub
     ISIS> cam2map from=W1567133629_1.norm.cub map=N1511700120_1.map.cub \
     ISIS>   to=W1567133629_1.map.cub matchmap=true
     ISIS> parallel_stereo N1511700120_1.map.equ.cub                     \
             W1567133629_1.map.equ.cub result/rhea

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices.
Also consider the following modifications to ``stereo.default``::

       ### PREPROCESSING
       alignment-method none
       force-use-entire-range
       individually-normalize

       ### CORRELATION
       prefilter-mode 2
       prefilter-kernel-width 1.5

       cost-mode 2

       corr-kernel 25 25
       corr-search -55 -2 -5 10

       subpixel-mode 3
       subpixel-kernel 21 21

       ### FILTERING
       rm-half-kernel 5 5
       rm-min-matches 60 # Units = percent
       rm-threshold 3
       rm-cleanup-passes 1
