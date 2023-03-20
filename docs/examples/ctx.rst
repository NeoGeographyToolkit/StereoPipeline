.. _ctx_example:

Mars Reconnaissance Orbiter CTX
-------------------------------

CTX is a moderately difficult camera to work with. The processing time
can be pretty long when using the Bayes EM subpixel refinement
(``subpixel-mode 2``). Otherwise the disparity between images is
relatively small, allowing efficient computation and a reasonable
processing time.

In this example we use mapprojected images, which is the most reliable
way to align the images for correlation.  Mapprojection is discussed
in :numref:`mapproj_with_cam2map` and :numref:`mapproj-example`.  Note
that mapprojection can slow down the triangulation step, but given
that ``parallel_stereo`` performs the triangulation using multiple
processes, that is not a concern.

This example's recipe is is in the ``examples/CTX`` directory shipped
with ASP (type 'make' there to run it).

.. figure:: ../images/examples/ctx/n_terra_meridiani_ctx_combined.png
   :name: ctx_example_fig

   Example output possible with the CTX imager aboard MRO.

The images are for the North Terra Meridiani region.

Download the CTX images P02_001981_1823_XI_02N356W.IMG and
P03_002258_1817_XI_01N356W.IMG from PDS, at:

    https://ode.rsl.wustl.edu/mars/indexproductsearch.aspx 

The download commands are::

    wget https://pds-imaging.jpl.nasa.gov/data/mro/mars_reconnaissance_orbiter/ctx/mrox_0031/data/P02_001981_1823_XI_02N356W.IMG
    wget https://pds-imaging.jpl.nasa.gov/data/mro/mars_reconnaissance_orbiter/ctx/mrox_0042/data/P03_002258_1817_XI_01N356W.IMG

Convert the .IMG files to ISIS .cub files, initialize the spice information, and calibrate::

    ISIS> mroctx2isis from = P02_001981_1823_XI_02N356W.IMG \
            to = P02_001981_1823.cub
    ISIS> mroctx2isis from = P03_002258_1817_XI_01N356W.IMG \
            to = P03_002258_1817.cub
    ISIS> spiceinit from = P02_001981_1823.cub
    ISIS> spiceinit from = P03_002258_1817.cub
    ISIS> ctxcal from = P02_001981_1823.cub to = P02_001981_1823.cal.cub
    ISIS> ctxcal from = P03_002258_1817.cub to = P03_002258_1817.cal.cub

(Here one can optionally run ``ctxevenodd`` on the ``cal.cub`` files, if needed.)

Run stereo::

    ISIS> cam2map4stereo.py P02_001981_1823.cal.cub P03_002258_1817.cal.cub
    ISIS> parallel_stereo P02_001981_1823.map.cub P03_002258_1817.map.cub \
            results/out

See :numref:`nextsteps` about the next steps, including a discussion
about various speed-vs-quality choices in stereo.

Automated Processing of HiRISE and CTX
--------------------------------------

While he was at the University of Chicago, David Mayer developed a set of
scripts for automating Stereo Pipeline for CTX and HiRISE images.  Those
scripts and more information can now be found at 
https://github.com/USGS-Astrogeology/asp_scripts.

