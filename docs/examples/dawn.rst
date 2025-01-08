.. _dawn_isis:

Dawn (FC) Framing Camera
------------------------

This is a NASA mission to visit two of the largest objects in the
asteroid belt, Vesta and Ceres. The framing camera on board Dawn is
quite small and packs only a resolution of 1024x1024 pixels. This means
processing time is extremely short. To its benefit, it seems that the
mission planners leave the framing camera on taking shots quite rapidly.
On a single pass, they seem to usually take a chain of FC images that
have a high overlap percentage. This opens the idea of using ASP to
process not only the sequential pairs, but also the wider baseline
shots. Then someone could potentially average all the DEMs together to
create a more robust data product.

For this example, we used the images FC21A0010191_11286212239F1T and
FC21A0010192_11286212639F1T which show the Cornelia crater on
Vesta. We learned about them from the anaglyph shown on the Planetary
Science Blog :cite:`planetaryblog:vesta`.

A different example (using CSM cameras) is in :numref:`csm_frame`.

.. figure:: ../images/examples/dawn/Vesta_figure.png
   :name: dawn-nomap-example

   Example colorized height map and ortho image output.

Commands
~~~~~~~~

First you must download and unzip the Dawn FC images from PDS from
https://sbib.psi.edu/data/PDS-Vesta/pds-vesta.html::

    wget https://sbib.psi.edu/data/PDS-Vesta/HAMO/img-1A/FC21A0010191_11286212239F1T.IMG.gz
    wget https://sbib.psi.edu/data/PDS-Vesta/HAMO/img-1A/FC21A0010192_11286212639F1T.IMG.gz
    gunzip FC21A0010191_11286212239F1T.IMG.gz
    gunzip FC21A0010192_11286212639F1T.IMG.gz

Then, these are converted to ISIS .cub files and ``parallel_stereo`` is run::

    dawnfc2isis from=FC21A0010191_11286212239F1T.IMG \
      to=FC21A0010191_11286212239F1T.cub target=VESTA
    dawnfc2isis from=FC21A0010192_11286212639F1T.IMG \
      to=FC21A0010192_11286212639F1T.cub  target=VESTA
    spiceinit from=FC21A0010191_11286212239F1T.cub
    spiceinit from=FC21A0010192_11286212639F1T.cub
    
    parallel_stereo FC21A0010191_11286212239F1T.cub \
      FC21A0010192_11286212639F1T.cub stereo/stereo
      
A DEM is then created with ``point2dem`` (:numref:`point2dem`)::
    
    point2dem stereo-PC.tif --orthoimage stereo-L.tif \
      --t_srs "+proj=eqc +lat_ts=-11.5 +a=280000 +b=229000 +units=m"

The option ``target=VESTA`` is necessary with ISIS version 5, and is
likely not needed in later versions.

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices when it comes to stereo algorithms.
