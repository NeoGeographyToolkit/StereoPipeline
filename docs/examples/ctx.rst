.. _ctx_example:

Mars Reconnaissance Orbiter CTX
-------------------------------

Overview
~~~~~~~~

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

Download
~~~~~~~~

Download the CTX images P02_001981_1823_XI_02N356W.IMG and
P03_002258_1817_XI_01N356W.IMG from PDS, at:

    https://ode.rsl.wustl.edu/mars/indexproductsearch.aspx 

The download commands are::

    wget https://planetarydata.jpl.nasa.gov/img/data/mro/ctx/mrox_0031/data/P02_001981_1823_XI_02N356W.IMG
    wget https://planetarydata.jpl.nasa.gov/img/data/mro/ctx/mrox_0042/data/P03_002258_1817_XI_01N356W.IMG

Creation of cub files
~~~~~~~~~~~~~~~~~~~~~

Install ISIS (:numref:`planetary_images`). 

Convert the .IMG files to ISIS .cub files, initialize the metadata (SPICE), and
calibrate::

    ISIS> mroctx2isis from = P02_001981_1823_XI_02N356W.IMG \
            to = P02_001981_1823.cub
    ISIS> mroctx2isis from = P03_002258_1817_XI_01N356W.IMG \
            to = P03_002258_1817.cub
    ISIS> spiceinit from = P02_001981_1823.cub web = true
    ISIS> spiceinit from = P03_002258_1817.cub web = true
    ISIS> ctxcal from = P02_001981_1823.cub to = P02_001981_1823.cal.cub
    ISIS> ctxcal from = P03_002258_1817.cub to = P03_002258_1817.cal.cub

Optionally run ``ctxevenodd`` on the ``cal.cub`` files.

The `spiceinit
<https://isis.astrogeology.usgs.gov/8.1.0/Application/presentation/Tabbed/spiceinit/spiceinit.html>`_
command initializes the cub file metadata. The option ``web = true`` fetches the
needed data on-the-fly. If it does not work, it is necessary to download this
data manually, from the ``mro`` directory of the `ISIS data area
<https://github.com/DOI-USGS/ISIS3#the-isis-data-area>`_.

Running stereo
~~~~~~~~~~~~~~

Run ``parallel_stereo`` (:numref:`parallel_stereo`) and ``point2dem``
(:numref:`point2dem`)::

    cam2map4stereo.py P02_001981_1823.cal.cub P03_002258_1817.cal.cub
    parallel_stereo                                      \
      --stereo-algorithm asp_mgm --subpixel-mode 9       \
      P02_001981_1823.map.cub P03_002258_1817.map.cub    \
      results/out
    point2dem -r mars --stereographic --auto-proj-center \
      results/out-PC.tif
  
Higher quality results can be obtained by using the ``aspm_mgm`` algorithm and
mapprojection (:numref:`nextsteps`).

It is recommended to bundle-adjust the CTX cameras before stereo
(:numref:`bundle_adjustment`). Then the ``cam2map4stereo.py`` script
cannot be used, but mapprojection can be done with ``mapproject``
(:numref:`mapproj-example`).

Further processing
~~~~~~~~~~~~~~~~~~

It is strongly suggested to use CSM camera models for improved performance
(:numref:`csm`). See :numref:`create_csm_linescan` for how to create CSM camera
models for linescan cameras, including for CTX.

CTX cameras can exhibit jitter. How to correct it is discussed in
:numref:`jitter_ctx`.

Shape-from-Shading with CTX images is illustrated in :numref:`sfs_ctx`.

Automated Processing of HiRISE and CTX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

While he was at the University of Chicago, David Mayer developed a set of
scripts for automating Stereo Pipeline for CTX and HiRISE images.  Those
scripts and more information can now be found at 
https://github.com/USGS-Astrogeology/asp_scripts.

