.. _kaguya_tc:

Kaguya Terrain Camera
---------------------

The Kaguya Terrain Camera (TC) is a push-broom imager, with a spatial resolution
of 10 m. The images are acquired from a 100 km altitude above the Moon. It was
part of the JAXA `Kaguya <https://en.wikipedia.org/wiki/SELENE>`_ orbiter.

Kaguya TC has two sensors, named TC1 and TC2, forming a stereo pair. They see
roughly the same region on the ground, with a convergence angle of about 30 degrees
(:numref:`stereo_pairs`). These sensors may have slightly different focal
lengths and distortion coefficients.

Fetching the data
~~~~~~~~~~~~~~~~~

Visit the `product search
<https://darts.isas.jaxa.jp/planet/pdap/selene/product_search.html>`_ page, and
enter a small search region.

Fetch the raw data sets for a desired stereo pair, starting with the *TC1* and
*TC2* prefixes (not the DEM or other products). Both the .img and .lbl files are needed. 

::

    wget https://darts.isas.jaxa.jp/pub/pds3/sln-l-tc-3-w-level2b0-v1.0/20080605/data/TC1W2B0_01_02936N034E0938.img.gz
    wget https://darts.isas.jaxa.jp/pub/pds3/sln-l-tc-3-w-level2b0-v1.0/20080605/data/TC1W2B0_01_02936N034E0938.lbl

    wget https://darts.isas.jaxa.jp/pub/pds3/sln-l-tc-3-w-level2b0-v1.0/20080605/data/TC2W2B0_01_02936N036E0938.img.gz
    wget https://darts.isas.jaxa.jp/pub/pds3/sln-l-tc-3-w-level2b0-v1.0/20080605/data/TC2W2B0_01_02936N036E0938.lbl

Preparing the data
~~~~~~~~~~~~~~~~~~

Unzip the .img.gz files.

Ensure that ISIS is installed, and that ISISROOT and ISISDATA are set, per
:numref:`planetary_images`. The Kaguya kernels can then be downloaded with the
command::

    $ISISROOT/bin/downloadIsisData kaguya $ISISDATA

For each image, run commands along the lines of::

    $ISISROOT/bin/kaguyatc2isis                     \
      from=TC1W2B0_01_02936N034E0938.lbl            \
      to=TC1W2B0_01_02936N034E0938.cub              \
      setnullrange=NO sethrsrange=NO sethisrange=NO \
      setlrsrange=NO setlisrange=NO

    $ISISROOT/bin/spiceinit from=TC1W2B0_01_02936N034E0938.cub \
        web=false attach=TRUE cksmithed=FALSE ckrecon=TRUE     \
        ckpredicted=FALSE cknadir=FALSE spksmithed=true        \
        spkrecon=TRUE spkpredicted=FALSE shape=SYSTEM          \
        startpad=0.0 endpad=0.0

Create CSM cameras (:numref:`csm`)::

    $ISISROOT/bin/isd_generate --only_naif_spice \
        TC1W2B0_01_02936N034E0938.cub            \
        -k TC1W2B0_01_02936N034E0938.cub

Bundle adjustment and stereo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run bundle adjustment (:numref:`bundle_adjust`) and stereo
(:numref:`parallel_stereo`)::

    bundle_adjust                                                   \
      TC1W2B0_01_02936N034E0938.cub TC2W2B0_01_02936N036E0938.cub   \
      TC1W2B0_01_02936N034E0938.json TC2W2B0_01_02936N036E0938.json \
      --tri-weight 0.1 --camera-weight 0.0                          \
      -o ba/run

    parallel_stereo --stereo-algorithm asp_mgm --subpixel-mode 9    \
      TC1W2B0_01_02936N034E0938.cub TC2W2B0_01_02936N036E0938.cub   \
      TC1W2B0_01_02936N034E0938.json TC2W2B0_01_02936N036E0938.json \
      --bundle-adjust-prefix ba/run                                 \
      stereo/run

Run ``point2dem`` (:numref:`point2dem`) to get a DEM. Consider using the
stereographic projection centered at the region of interest::

    point2dem --stereographic --proj-lon 93.8 --proj-lat 3.5 \
      --tr 10 stereo/run-PC.tif

See :numref:`sfs_kaguya_example` for a clip of the produced DEM.

It is suggested to rerun stereo with mapprojected images
(:numref:`mapproj-example`), to get a higher quality output. 

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices
when running stereo. 

Alignment
~~~~~~~~~

The produced DEM can be aligned with ``pc_align`` (:numref:`pc_align`) to the
LOLA RDR product. 

.. _sfs_kaguya:

Shape-from-shading with Kaguya TC
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here it will be illustrated how to run shape-from-shading (:numref:`sfs`) on Kaguya
TC images. First, ensure that the data are fetched and a stereo terrain is created,
per :numref:`kaguya_tc`. 

Shape-from-shading expects a DEM with no holes which is also rather smooth. It
should be at the same ground resolution as the input images, which in this case is 10
meters per pixel. It is best to have it in a local projection, such as stereographic.

We will modify the DEM creation command from above to use a large search radius to fill 
any holes::

    point2dem --stereographic --proj-lon 93.8 --proj-lat 3.5 \
        --tr 10 --search-radius-factor 10 stereo/run-PC.tif

(adjust the projection center for your location).

Inspect the produced DEM ``stereo/run-DEM.tif`` in ``stereo_gui`` in hillshading
mode. Any additional holes can be filled with ``dem_mosaic`` (:numref:`dem_mosaic`),
with options along the lines of ``--fill-search-radius 100 --fill-percent 1``.

It is also suggested to blur it a little, to make it smoother::

    dem_mosaic --dem-blur-sigma 2 stereo/run-filled-dem.tif \
      -o stereo/run-blurred-dem.tif  

Then crop a region with ``gdal_translate`` that has no missing data. 

Mapproject (:numref:`mapproject`) the left and right images with the
corresponding ``.json`` camera files, while using the adjustments in ``ba/run``.
Overlay the resulting georeferenced images in ``stereo_gui``. This is a very
important sanity check to ensure that the cameras are registered correctly. 

Run SfS as::

    parallel_sfs -i stereo/run-cropped-dem.tif \
      TC1W2B0_01_02936N034E0938.cub            \
      TC2W2B0_01_02936N036E0938.cub            \
      TC1W2B0_01_02936N034E0938.json           \
      TC2W2B0_01_02936N036E0938.json           \
      --bundle-adjust-prefix ba/run            \
      --reflectance-type 1                     \
      --blending-dist 10                       \
      --min-blend-size 50                      \
      --allow-borderline-data                  \
      --threads 4                              \
      --save-sparingly                         \
      --crop-input-images                      \
      --smoothness-weight 30000                \
      --initial-dem-constraint-weight 10       \
      --max-iterations 5                       \
      --shadow-thresholds "120 120"            \
      --tile-size 200                          \
      --padding 50                             \
      --processes 10                           \
      -o sfs/run

The initial and final DEM can be inspected in ``stereo_gui``. The ``geodiff``
(:numref:`geodiff`) tool can be used to compare how much the DEM changed.

The choices of parameters above are peculiar to this data set, especially the
smoothness weight. Making this smaller can result in artifacts, but if it is
larger, it may blur the produced DEM too much.

The initial DEM constraint was set rather high to ensure the DEM does not change
much as result of SfS. The shadow threshold depends on the pixel values and can
be very different for other images.

It could have been beneficial to have images with different illumination. This
can add more information, and makes it possible to reduce the value of the
smoothness weight without introducing artifacts, which results in less detail
being lost.

See, for comparison, the parameter choices made for LRO NAC
(:numref:`sfs-lola`). That example, and that entire chapter, also has the most
detailed discussion for how to run SfS.

.. figure:: ../images/sfs_kaguya_example.png
   :name: sfs_kaguya_example
   :alt: SfS with Kaguya TC images

   From left to right: the stereo DEM, SfS DEM (hillshaded), and a mapprojected
   image. 

