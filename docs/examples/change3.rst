.. _change3:

Chang'e 3 landing camera
------------------------

This example discusses processing  
`Chang'e 3 <https://en.wikipedia.org/wiki/Chang%27e_3>`_ landing camera images.

This camera was mounted at the bottom of the lander and acquired images during
the descent phase.

The images we inspected had a very small convergence angle
(:numref:`stereo_pairs`), of under 1.5 degrees, which resulted in an unreliable
terrain model. 

Here we show how these images can be registered to LRO NAC images
(:numref:`lronac-example`), how to refine the landing camera intrinsics, and how
to produce a terrain model from a stereo pair consisting of a Chang'e landing
camera image and an LRO NAC image with similar illumination.

The `Chang'e3 landing video <https://www.youtube.com/watch?v=sKYrAM3EJh8>`_ is
very helpful when it comes to deciding which images to process.

Fetching the Chang'e 3 images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The images are available from China's `Lunar Planetary Data Release System
<https://moon.bao.ac.cn/ce5web/searchOrder_dataSearchData.search>`_. Select the
``CE3`` mission, the ``LCAM`` instrument, and the ``2A`` data level. The images
have names of the form::

  CE3_BMYK_LCAM-1801_SCI_N_20131214130807_20131214130807_0001_A.2A

The data is in the PDS3 format, with a plain text metadata header followed by a binary image.
The images can be converted to TIF with ``gdal_translate`` (:numref:`gdal_tools`)
as::

  gdal_translate input.2A 1801.tif

Here, ``1801`` is the index in the sequence. 

.. figure:: ../images/change3_images.png

  Landing camera images 1801, 1831, 1861, and 1891. A crater seen in all four images
  is highlighted with a red box.

Some parts of the lander body are seen in the foreground. Most of those
artifacts can be masked with ``image_calc`` (:numref:`image_calc`) with a
command as::

  image_calc -c "max(var_0,70)" \
    --output-nodata-value 70    \
    -d float32                  \
    input.tif                   \
    -o output.tif 

This sets to no-data any values not exceeding 70. 

A more careful processing could be done by opening an image in an image editor,
manually setting to black (zero pixel value) all undesirable pixels, and then
using the ``image_calc`` ``sign()`` function to create a mask of invalid (value
0) and valid (value 1) pixels. Those could be applied to each image by
multiplication, with ``image_calc`` with the option ``--output-nodata-value 0``.
The same mask would work for all of them.

LRO NAC data
~~~~~~~~~~~~

The Chang'e 3 images will be registered against LRO NAC images. These are larger,
with known camera information, and at higher resolution. 

It was quite tricky to find an LRO NAC image with similar illumination. This
required mapprojecting many such images and visual inspection. We settled on image
``M1154358210RE``. 

How to download and prepare LRO NAC images, including the application of ``lronaccal``
and ``lronacecho``, is described in :numref:`lronac-example`. A CSM camera model
can be produced as in :numref:`create_csm_linescan`. The resulting datasets will
be called ``img/lro.cub`` and ``img/lro.json``.

We will also fetch an `LRO NAC DEM
<https://pds.lroc.asu.edu/data/LRO-L-LROC-5-RDR-V1.0/LROLRC_2001/DATA/SDP/NAC_DTM/CHANGE3/NAC_DTM_CHANGE3.TIF>`_
produced specifically for this landing site. We call it ``ref/ref.tif``.

The LRO NAC image is very large, and sometimes images are also scanned in
reverse direction, appearing mirror-flipped. These result in failure in finding
matching features for registration. To make the work easier, we will mapproject
the needed image portion onto this DEM. 

Since these two datasets are not explicitly co-registered, we will blur the DEM
for mapprojection quite a bit to lessen the effect of artifacts due to
misregistration. Later, for alignment to the ground, we will use the original
DEM. 

The blur is done with ``dem_mosaic`` (:numref:`dem_mosaic`) as::

    dem_mosaic --dem-blur-sigma 10 \
      ref/ref.tif -o ref/ref_blur.tif

Define the extent on the ground and the projection::

    win="3497495 1340957 3503625 1334379"
    proj="+proj=eqc +lat_ts=44 +lat_0=0 +lon_0=180 +x_0=0 +y_0=0 +R=1737400 +units=m +no_defs"

Then, the mapprojection (:numref:`mapproject`) step follows::

    mapproject --tr 2.0 \
      --t_projwin $win  \
      --t_srs "$proj"   \
      ref/ref_blur.tif  \
      img/lro.cub       \
      img/lro.json      \
      img/lro.map.tif
  
The grid size of 2 meters was chosen to be similar to the resolution of the
Chang'e 3 images.

GCP creation
~~~~~~~~~~~~

We will find interest point matches between the Chang'e 3 and LRO NAC images,
based on which we will compute GCP (:numref:`bagcp`), that will be later used to
infer an approximate position and orientation of the Chang'e 3 camera at the
time of image acquisition.

The Chang'e 3 images will be stored in the ``img`` directory, with names 
such as ``img/1801.tif``, ``img/1831.tif``, etc. 

GCP are found with the ``gcp_gen`` program (:numref:`gcp_gen`) as::

    gcp_gen                         \
      --ip-detect-method 1          \
      --inlier-threshold 100        \
      --ip-per-tile 20000           \
      --individually-normalize      \
      --camera-image img/1801.tif   \
      --ortho-image img/lro.map.tif \
      --dem ref/ref.tif             \
      --output-prefix run/run       \
      -o gcp/gcp_1801.gcp

The interest point matches can be visualized with ``stereo_gui``
(:numref:`stereo_gui_view_ip`) as::

    stereo_gui img/1801.tif img/lro.map.tif run/run-1801__lro.map.match

.. figure:: ../images/change3_lro.png

  Interest point matches between Chang'e image 1801 and mapprojected LRO NAC image 
  M1154358210RE. 
