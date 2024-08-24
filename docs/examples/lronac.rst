.. _lronac-example:

Lunar Reconnaissance Orbiter (LRO) NAC
--------------------------------------

This section will describe in detail how to process an LRO NAC dataset.
(See also :numref:`lronac_csm` for a ready-made example using a different
dataset for which all inputs have already been prepared.)

The site
~~~~~~~~

In this example we will consider a stereo pair that covers the
Taurus-Littrow valley on the Moon where, on December 11, 1972, the
astronauts of Apollo 17 landed. However, this stereo pair does not
contain the landing site. It is slightly west; focusing on the
Lee-Lincoln scarp that is on North Massif. The scarp is an 80 m high
feature that is the only visible sign of a deep fault.

.. figure:: ../images/examples/lrocna/lroc-na-example2_combined.png

   Example output possible with a LROC NA stereo pair, using both
   CCDs from each observation, courtesy of the lronac2mosaic.py tool.

LRO NAC camera design
~~~~~~~~~~~~~~~~~~~~~

LRO has two Narrow Angle Cameras (NAC), with both acquiring image data
at the same time, so each observation consists
of two images, left and right, denoted with ``L`` and ``R``.
These are not meant to be used as a stereo pair, as the camera
center is virtually in the same place for both, and they have very little
overlap. For stereo one needs two such observations, with a
reasonable perspective difference (baseline) among the two.
 
Then stereo can happen by pairing an L or R image from the first
observation with an L or R image from the second. Alternatively, each
observation's L and R images can be stitched first, then stereo happens
between the two stitched images. Both of these approaches will be
discussed below.

Download
~~~~~~~~

Download the experimental data records (EDR) for observations
M104318871 and M104311715 from http://wms.lroc.asu.edu/lroc/search.
Alternatively, search by original IDs of 2DB8 and 4C86 in the
PDS. 

The download will result in four files, named M104318871LE.img,
M104318871RE.img, M104311715LE.img, and M104311715RE.img.

.. _lro_nac_no_stitch:

Preparing the inputs without stitching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The individual ``L`` and ``R`` images in an LRO NAC observation can be
used without stitching if desired to keep the original camera models.
Unstitched cameras can also be converted to CSM (:numref:`csm`), which
will provide a great speed up for stereo, bundle adjustment, and
Shape-from-Shading (:numref:`sfs_usage`).

We convert each .img file to an ISIS .cub camera image, initialize the
SPICE kernels, and perform radiometric calibration and echo
correction. Here are the steps, illustrated on one image::

    f=M104318871LE
    lronac2isis from = ${f}.IMG     to = ${f}.cub
    spiceinit   from = ${f}.cub shape = ellipsoid
    lronaccal   from = ${f}.cub     to = ${f}.cal.cub
    lronacecho  from = ${f}.cal.cub to = ${f}.cal.echo.cub

Note that for these commands to succeed, ISIS and its supporting data
must be downloaded, per :numref:`planetary_images`.

Stitching the LE and RE observations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this case all ISIS preprocessing of the EDRs is performed via the
``lronac2mosaic.py`` command (:numref:`lronac2mosaic`)::

    lronac2mosaic.py M104318871LE.img M104318871RE.img
    lronac2mosaic.py M104311715LE.img M104311715RE.img

This runs ``lronac2isis``, ``lronaccal``, ``lronacecho``,
``spiceinit``, ``noproj``, and ``handmos`` to create a stitched
unprojected image for each observation. In this example we don't
mapproject the images as ASP can usually get good results. More
aggressive terrain might require an additional ``cam2map4stereo.py``
step.

In case of failure, it is suggested to re-run this tool with the option
``--keep`` to keep the intermediate files, and then inspect the ones before
the final step. 

Running stereo
~~~~~~~~~~~~~~

Stereo can then be run either with unstitched or stitched .cub files.
Here's an example with the unstitched LE images::

    parallel_stereo M104318871LE.cub M104311715LE.cub  \
      --alignment-method affineepipolar                \
      run/run

Create a DEM, orthoimage, and error image with ``point2dem``
(:numref:`point2dem`)::
 
    point2dem --stereographic --auto-proj-center \
      --errorimage --orthoimage                  \
        run/run-PC.tif run/run-L.tif
        
Check the stereo convergence angle as printed during preprocessing
(:numref:`stereo_pairs`). That angle is often too small for LRO NAC,
and then the results are not going to be great.

Check the triangulation error (:numref:`triangulation_error`) in the produced
error image. This may suggest that bundle adjustment may be needed
(:numref:`bundle_adjust`).

See :numref:`nextsteps` for a discussion about various stereo
speed-vs-quality choices. Consider using mapprojection
(:numref:`mapproj-example`) for best results for steep slopes.

It is strongly suggested to convert the cameras to CSM 
(:numref:`csm_linescan`). This makes mapprojection faster, 
and also can help solve for jitter (:numref:`jitter_solve`).

Validation and alignment
~~~~~~~~~~~~~~~~~~~~~~~~

A `LOLA <https://ode.rsl.wustl.edu/moon/lrololadataPointSearch.aspx>`_ point
cloud can be downloaded for the area of interest. Download the RDR
``PointPerRow`` product.

The produced DEM can be aligned to LOLA with ``pc_align`` (:numref:`pc_align`),
with a command as::

    pc_align --max-displacement 500          \
     --csv-format 2:lon,3:lat,4:radius_km    \
     --save-inv-transformed-reference-points \
     run/run-DEM.tif LOLA.csv                \
     -o run/run-align
     
Solving for jitter
~~~~~~~~~~~~~~~~~~

LRO NAC images can have jitter. A process as for CTX in :numref:`jitter_ctx`
usually works well to remove it (see also a multi-image example in
:numref:`jitter_multiple_images`).
