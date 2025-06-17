.. _csm:

Community Sensor Model
----------------------

The Community Sensor Model (CSM), established by the U.S. defense
and intelligence community, has the goal of standardizing camera
models for various remote sensor types :cite:`CSMTRD`. It provides
a well-defined application program interface (API) for multiple
types of sensors and has been widely adopted by Earth remote sensing
software systems :cite:`hare2017community,2019EA000713`.

ASP supports and ships the `USGS implementation
<https://github.com/USGS-Astrogeology/usgscsm>`_ of CSM for planetary images,
which provides Linescan, Frame, Pushframe, and Synthetic Aperture Radar (SAR)
implementations.

CSM is handled via dynamically loaded plugins. Hence, if a user has a
new sensor model, ASP should, in principle, be able to use it as soon
as a supporting plugin is added to the existing software, without
having to rebuild ASP or modify it otherwise. In practice, while this
logic is implemented, ASP defaults to using only the USGS
implementation, though only minor changes are needed to support
additional plugins.

Each stereo pair to be processed by ASP should be made up of two
images (for example ``.cub`` or ``.tif`` files) and two plain
text camera files with ``.json`` extension. The CSM information is
contained in the ``.json`` files and it determines which plugin to
load to use with those cameras. 

CSM model state data can also be embedded in ISIS .cub files
(:numref:`embedded_csm`).

.. _csm_frame:

The USGS CSM Frame sensor
~~~~~~~~~~~~~~~~~~~~~~~~~

The USGS CSM *Frame* sensor models a frame camera. All the
pixels get acquired at the same time, unlike for pushbroom and
pushframe cameras, which keep on acquiring image lines as they fly
(those are considered later in the text). Hence, a single camera
center and orientation is present. This model serves the same function
as ASP's own Pinhole camera model (:numref:`pinholemodels`).

:numref:`csm_frame_def` discusses the CSM Frame sensor in some detail,
including the distortion model.

In this example we will consider images acquired with the Dawn
Framing Camera instrument, which took pictures of the Ceres and Vesta
asteroids. This particular example will be for Vesta. Note that one
more example of this sensor is shown in this documentation, in
:numref:`dawn_isis`, which uses ISIS ``.cub`` camera models rather
than CSM ones.

This example is available for `download <https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples>`_.


Creating the input images
^^^^^^^^^^^^^^^^^^^^^^^^^

Fetch the data from PDS then extract it::

    wget https://sbib.psi.edu/data/PDS-Vesta/Survey/img-1B/FC21B0004011_11224024300F1E.IMG.gz
    wget https://sbib.psi.edu/data/PDS-Vesta/Survey/img-1B/FC21B0004012_11224030401F1E.IMG.gz
      
    gunzip FC21B0004011_11224024300F1E.IMG.gz 
    gunzip FC21B0004012_11224030401F1E.IMG.gz

For simplicity of notation, we will rename these to ``left.IMG`` and ``right.IMG``.

Set up the ISIS environment (:numref:`planetary_images`). 

These will need adjusting for your system::

    export ISISROOT=$HOME/miniconda3/envs/isis
    export PATH=$ISISROOT/bin:$PATH
    export ISISDATA=$HOME/isisdata

Create cub files and initialize the kernels::

    dawnfc2isis from = left.IMG  to = left.cub  target = VESTA
    dawnfc2isis from = right.IMG to = right.cub target = VESTA

    spiceinit from = left.cub
    spiceinit from = right.cub

The ``target`` field is likely no longer needed in newer versions of
ISIS.

.. _create_csm_dawn:

Creation of CSM Frame camera files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set::

    export ALESPICEROOT=$ISISDATA

Run::

    isd_generate -k left.cub left.cub
    isd_generate -k right.cub right.cub

This will create ``left.json`` and ``right.json``.

As a sanity check, run ``cam_test`` (:numref:`cam_test`) to see how well the CSM
camera approximates the ISIS camera::

    cam_test --image left.cub  --cam1 left.cub  --cam2 left.json
    cam_test --image right.cub --cam1 right.cub --cam2 right.json

Note that for a handful of pixels these errors may be big. That is a
known issue, and it seems to be due to the fact that a ray traced from
the camera center towards the ground may miss the body of the asteroid.
That should not result in inaccurate stereo results.

Running stereo
^^^^^^^^^^^^^^

::

    parallel_stereo --stereo-algorithm asp_mgm \
      --left-image-crop-win 243 161 707 825    \
      --right-image-crop-win 314 109 663 869   \
      left.cub right.cub left.json right.json  \
      run/run

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices when running stereo.

This is followed by creation of a DEM (:numref:`point2dem`)
and products that can be visualized (:numref:`genhillshade`)::

    point2dem run/run-PC.tif --orthoimage run/run-L.tif 
    hillshade run/run-DEM.tif 
    colormap run/run-DEM.tif -s run/run-DEM_HILLSHADE.tif 

.. figure:: ../images/CSM_Frame.png
   :name: CSM_Frame_example

   The produced colorized DEM and orthoimage for the CSM Frame camera
   example. Likely using mapprojection (:numref:`mapproj-example`)
   may have reduced the number and size of the holes in the DEM.

.. _csm_linescan:

The USGS CSM linescan sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this example we will use the Mars CTX linescan sensor. The images are regular
``.cub`` files as in the tutorial in :numref:`moc_tutorial`, hence the only
distinction compared to that example is that the cameras are stored as ``.json``
files.

We will work with the dataset pair::

     J03_045994_1986_XN_18N282W.cub J03_046060_1986_XN_18N282W.cub

which, for simplicity, we will rename to ``left.cub`` and ``right.cub``
and the same for the associated camera files.

See :numref:`kaguya_tc` for another linescan example for the Kaguya linescan
sensor for the Moon.

.. _create_csm_linescan:

Creation CSM linescan cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Note that this recipe looks a little different for Frame and SAR cameras,
as can be seen in :numref:`create_csm_dawn` and :numref:`create_csm_sar`.

Run the ISIS ``spiceinit`` command on the .cub files as::

    spiceinit from = left.cub
    spiceinit from = right.cub

To create CSM cameras, run::

    isd_generate left.cub
    isd_generate right.cub
    
This will produce ``left.json`` and ``right.json``.

See the `isd_generate manual
<https://astrogeology.usgs.gov/docs/getting-started/using-ale/isd-generate/>`_.

Running stereo
^^^^^^^^^^^^^^

::

    parallel_stereo --stereo-algorithm asp_mgm \
      --subpixel-mode 9                        \
       left.cub right.cub left.json right.json \
       run/run 

Check the stereo convergence angle as printed during preprocessing
(:numref:`stereo_pairs`). If that angle is small, the results are not
going to be great.

See :numref:`nextsteps` for a discussion about various stereo
algorithms and speed-vs-quality choices. 

The fancier MGM algorithm could be used by running this example with
``--stereo-algorithm asp_mgm``.

The actual stereo session used is ``csm``, and here it will be
auto-detected based on the extension of the camera files.

Next, a DEM is produced (:numref:`point2dem`)::
       
    point2dem -r mars --stereographic \
      --proj-lon 77.4 --proj-lat 18.4 \
      run/run-PC.tif

For ``point2dem`` we chose to use a stereographic projection centered at
some point in the area of interest. See :numref:`point2dem_proj`
for how how a projection for the DEM can be auto-determined.

One can also run ``parallel_stereo`` with mapprojected images
(:numref:`mapproj-example`). The first step would be to create a
low-resolution smooth DEM from the previous cloud::

     point2dem -r mars                 \
       --stereographic                 \
       --proj-lon 77.4 --proj-lat 18.4 \
        --tr 120                       \
       run/run-PC.tif                  \
       -o run/run-smooth

followed by mapprojecting onto it and redoing stereo::

    mapproject --tr 6 run/run-smooth-DEM.tif left.cub  \
      left.json left.map.tif
    mapproject --tr 6 run/run-smooth-DEM.tif right.cub \
     right.json right.map.tif
    parallel_stereo --stereo-algorithm asp_mgm         \
      --subpixel-mode 9                                \
      left.map.tif right.map.tif left.json right.json  \
      run_map/run run/run-smooth-DEM.tif

Notice how we used the same resolution for both images when
mapprojecting. That helps making the resulting images more similar and
reduces the processing time (:numref:`mapproj-res`).

.. _csm_wac:

CSM Pushframe sensor
~~~~~~~~~~~~~~~~~~~~

The USGS CSM *Pushframe* sensor models a pushframe camera.  The support for this sensor
is not fully mature, and some artifacts can be seen in the DEMs (per below).
 
What follows is an illustration of using this sensor with Lunar Reconnaissance
Orbiter (LRO) WAC images. 

This example, including the inputs, recipe, and produced terrain model
`can be downloaded <https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/LROWAC>`_.

Fetching the data
^^^^^^^^^^^^^^^^^

We will focus on the monochromatic images for this sensor. Visit:

   https://ode.rsl.wustl.edu/moon/indexproductsearch.aspx

Find the *Lunar Reconnaissance Orbiter -> Experiment Data Record Wide
Angle Camera - Mono (EDRWAM)* option.

Search either based on a longitude-latitude window, or near a notable
feature, such as a named crater.  We choose a couple of images having
the Tycho crater, with download links::

    http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0002/DATA/MAP/2010035/WAC/M119923055ME.IMG
    http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0002/DATA/MAP/2010035/WAC/M119929852ME.IMG

Fetch these with ``wget``.

Creation of .cub files
^^^^^^^^^^^^^^^^^^^^^^

We broadly follow the tutorial at :cite:`ohman2015procedure`. For a
dataset called ``image.IMG``, do::

    lrowac2isis from = image.IMG to = image.cub

This will create so-called *even* and *odd* datasets, with names like
``image.vis.even.cub`` and ``image.vis.odd.cub``.

Run ``spiceinit`` on them to set up the SPICE kernels::

    spiceinit from = image.vis.even.cub
    spiceinit from = image.vis.odd.cub

followed by ``lrowaccal`` to adjust the image intensity::

    lrowaccal from = image.vis.even.cub to = image.vis.even.cal.cub
    lrowaccal from = image.vis.odd.cub  to = image.vis.odd.cal.cub

All these .cub files can be visualized with ``stereo_gui``. It can be
seen that instead of a single contiguous image we have a set of narrow
horizontal framelets, with some of these in the even and some in the odd
cub file. The framelets may also be recorded in reverse.

Production of seamless mapprojected images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is not needed for stereo, but may be useful for readers who would
like to produce image mosaics using ``cam2map``.

::

    cam2map from = image.vis.even.cal.cub to = image.vis.even.cal.map.cub
    cam2map from = image.vis.odd.cal.cub  to = image.vis.odd.cal.map.cub  \
      map = image.vis.even.cal.map.cub matchmap = true

Note how in the second ``cam2map`` call we used the ``map`` and
``matchmap`` arguments. This is to ensure that both of these output
images have the same resolution and projection. In particular, if more
datasets are present, it is suggested for all of them to use the same
previously created .cub file as a map reference.  That because stereo
works a lot better on mapprojected images with the same ground
resolution. For more details see :numref:`mapproj-example` and
:numref:`mapproj_with_cam2map`.

To verify that the obtained images have the same ground resolution, do::

    gdalinfo image.vis.even.cal.map.cub | grep -i "pixel size"
    gdalinfo image.vis.odd.cal.map.cub  | grep -i "pixel size"

(see :numref:`gdal_tools` regarding this tool).

The fusion happens as::

    ls image.vis.even.cal.map.cub image.vis.odd.cal.map.cub  > image.txt
    noseam fromlist = image.txt to = image.noseam.cub SAMPLES=73 LINES=73

The obtained file ``image.noseam.cub`` may still have some small artifacts
but should be overall reasonably good. 

Stitching the raw even and odd images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This requires ISIS newer than version 6.0, or the latest development code.

For each image in the stereo pair, stitch the even and odd datasets::

    framestitch even = image.vis.even.cal.cub odd = image.vis.odd.cal.cub \
      to = image.cub flip = true num_lines_overlap = 2

The ``flip`` flag is needed if the order of framelets is reversed
relative to what the image is expected to show.

The parameter ``num_lines_overlap`` is used to remove a total of this
many lines from each framelet (half at the top and half at the bottom)
before stitching, in order to deal with the fact that the even and odd
framelets have a little overlap, and that they also tend to have artifacts
due to some pixels flagged as invalid in each first and last framelet
row.

The CSM camera models will assume that this parameter is set at 2, so
it should not be modified. Note however that WAC framelets may overlap
by a little more than that, so resulting DEMs may have some artifacts
at framelet borders, as can be seen further down.

Creation of CSM WAC cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set::

    export ALESPICEROOT=$ISISDATA

Run::

    isd_generate -k image.vis.even.cal.cub image.vis.even.cal.cub
    isd_generate -k image.vis.odd.cal.cub  image.vis.odd.cal.cub

These will create ``image.vis.even.cal.json`` and ``image.vis.odd.cal.json``.

Do not use the stitched .cub file as that one lacks camera information. The
obtained .json files can be renamed to follow the same convention as the
stitched .cub images.

Running stereo
^^^^^^^^^^^^^^

::

    parallel_stereo --stereo-algorithm asp_mgm   \
      --left-image-crop-win 341 179 727 781      \
      --right-image-crop-win 320 383 824 850     \
      M119923055ME.cub M119929852ME.cub          \
      M119923055ME.json M119929852ME.json        \
      run/run

As printed by ``stereo_pprc``, the convergence angle is about 27
degrees, which is a good number.

See :numref:`nextsteps` for a discussion about various stereo
speed-vs-quality choices.

A DEM is produced with ``point2dem`` (:numref:`point2dem`), and other products
are made for visualization (:numref:`visualising`)::

    point2dem --stereographic --auto-proj-center \
      run/run-PC.tif --orthoimage run/run-L.tif 
    hillshade run/run-DEM.tif 
    colormap run/run-DEM.tif -s run/run-DEM_HILLSHADE.tif 

.. figure:: ../images/CSM_WAC.png
   :name: CSM_WAC_example

   The produced colorized DEM and orthoimage for the CSM WAC camera
   example. The artifacts are due to issues stitching of even and odd
   framelets.

It can be seen that the stereo DEM has some linear artifacts. That is
due to the fact that the stitching does not perfectly integrate the
framelets.

An improved solution can be obtained by creating a low-resolution
version of the above DEM, mapprojecting the images on it, and then
re-running stereo, per (:numref:`mapproj-example`).

::

    point2dem --stereographic --auto-proj-center       \
      --tr 800 run/run-PC.tif --search-radius-factor 5 \
      -o run/run-low-res
    mapproject --tr 80 run/run-low-res-DEM.tif         \
      M119923055ME.cub M119923055ME.json M119923055ME.map.tif 
    mapproject --tr 80 run/run-low-res-DEM.tif         \
      M119929852ME.cub M119929852ME.json M119929852ME.map.tif 
    parallel_stereo --stereo-algorithm asp_mgm         \
      M119923055ME.map.tif M119929852ME.map.tif        \
      M119923055ME.json M119929852ME.json              \
      run_map/run run/run-low-res-DEM.tif 
    point2dem --stereographic --auto-proj-center       \
      run_map/run-PC.tif --orthoimage run_map/run-L.tif 
    hillshade run_map/run-DEM.tif 
    colormap run_map/run-DEM.tif -s run_map/run-DEM_HILLSHADE.tif 

To create the low-resolution DEM we used a grid size of 800 m,
which is coarser by a factor of about 8 compared to the nominal WAC
resolution of 100 / pixel. 

Note that the same resolution is used when mapprojecting both images; that is
very important to avoid a large search range in stereo later. This is discussed
in more detail in :numref:`mapproj-example`.

.. figure:: ../images/CSM_WAC_mapproj.png
   :name: CSM_WAC_example_mapproj

   The produced colorized DEM and orthoimage for the CSM WAC camera
   example, when mapprojected images are used.

As can be seen in the second figure, there are somewhat fewer artifacts.
The missing lines in the DEM could be filled in if ``point2dem`` was run
with ``--search-radius-factor 4``, for example. 

Given that there exists a wealth of WAC images, one could also try to
get several more stereo pairs with similar illumination, run bundle
adjustment for all of them (:numref:`bundle_adjust`), run pairwise
stereo, create DEMs (at the same resolution), and then merge them with
``dem_mosaic`` (:numref:`dem_mosaic`). This may further attenuate the
artifacts as each stereo pair will have them at different
locations. See :numref:`stereo_pairs` for guidelines about how to
choose good stereo pairs.

.. _csm_minirf:

The USGS CSM SAR sensor for LRO Mini-RF 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This page describes processing data produced with the *Mini-RF* Synthetic
Aperture Radar (SAR) sensor on the LRO spacecraft while making use of CSM
cameras. A SAR example for Earth is in :numref:`umbra_sar`. 

It is challenging to process its data with ASP for several
reasons:

 - The synthetic image formation model produces curved rays going from the
   ground to the pixel in the camera (:cite:`kirk2016semi`). To simplify the
   calculations, ASP finds where a ray emanating from the camera
   intersects the standard Moon ellipsoid with radius 1737.4 km and
   declares the ray to be a straight line from the camera center to this
   point.

 - This sensor very rarely acquires stereo pairs. The convergence angle
   (:numref:`stereo_pairs`) as printed by ``parallel_stereo`` in
   pre-processing is usually less than 5 degrees, which is little and
   results in noisy DEMs. In this example we will use a dataset
   intentionally created with stereo in mind. The images will cover a
   part of Jackson crater (:cite:`kirk2011radargrammetric`).

 - It is not clear if all modeling issues with this sensor were
   resolved. The above publication states that "Comparison of the stereo
   DTM with ~250 m/post LOLA grid data revealed (in addition to
   dramatically greater detail) a very smooth discrepancy that varied
   almost quadratically with latitude and had a peak-to-peak amplitude
   of nearly 4000 m."
  
 - The images are dark and have unusual appearance, which requires
   some pre-processing and a large amount of interest points. 

Hence, ASP's support for this sensor is experimental. The results
are plausible but likely not fully rigorous.

This example, including input images, produced outputs, and a recipe, is available
for download at:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples

No ISIS data are needed to run it.

Creating the input images
^^^^^^^^^^^^^^^^^^^^^^^^^

Fetch the data from PDS::

    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03821_1cd_xku_16n196_v1.img
    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03821_1cd_xku_16n196_v1.lbl
    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03822_1cd_xku_23n196_v1.img
    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03822_1cd_xku_23n196_v1.lbl

These will be renamed to ``left.img``, ``right.img``, etc., to simply
the processing.

Set, per :numref:`planetary_images`, values for ``ISISROOT`` and ``ISISDATA``. Run::

    mrf2isis from = left.lbl  to = left.cub
    mrf2isis from = right.lbl to = right.cub

Run ``spiceinit``. Setting the shape to the ellipsoid makes it easier
to do image-to-ground computations::

    spiceinit from = left.cub  shape = ellipsoid
    spiceinit from = right.cub shape = ellipsoid

.. _create_csm_sar:

Creation of CSM SAR cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set::
  
    export ALESPICEROOT=$ISISDATA

Run ``isd_generate``::

    isd_generate -k left.cub  left.cub
    isd_generate -k right.cub right.cub
          
This will create the CSM camera file ``left.json`` and ``right.json``.

Run ``cam_test`` (:numref:`cam_test`) as a sanity check::

    cam_test --image left.cub  --cam1 left.cub  --cam2 left.json
    cam_test --image right.cub --cam1 right.cub --cam2 right.json

Preparing the images
^^^^^^^^^^^^^^^^^^^^

ASP accepts only single-band images, while these .cub files have four of them.
We will pull the first band and clamp it to make it easier for stereo to find
interest point matches::

    gdal_translate -b 1 left.cub  left_b1.tif
    gdal_translate -b 1 right.cub right_b1.tif

    image_calc -c "min(var_0, 0.5)" left_b1.tif  -d float32 \
      -o left_b1_clamp.tif 
    image_calc -c "min(var_0, 0.5)" right_b1.tif -d float32 \
      -o right_b1_clamp.tif 

Running stereo
^^^^^^^^^^^^^^

It is simpler to first run a clip with ``stereo_gui``
(:numref:`stereo_gui`).  This will result in the following command::

    parallel_stereo --ip-per-tile 3500             \
      --left-image-crop-win 0 3531 3716 10699      \
      --right-image-crop-win -513 22764 3350 10783 \
      --stereo-algorithm asp_mgm --min-num-ip 10   \
      left_b1_clamp.tif right_b1_clamp.tif         \
      left.json right.json run/run  

The stereo convergence angle for this pair is 18.4 degrees which is
rather decent.

Create a colorized DEM and orthoimage (:numref:`point2dem`)::

    point2dem run/run-PC.tif --orthoimage run/run-L.tif 
    hillshade run/run-DEM.tif 
    colormap run/run-DEM.tif -s run/run-DEM_HILLSHADE.tif 

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices when running stereo.

.. figure:: ../images/CSM_MiniRF.png
   :name: CSM_miniRF_example

   The produced colorized DEM and orthoimage for the CSM SAR example. 

.. _csm_msl:

CSM cameras for MSL
~~~~~~~~~~~~~~~~~~~

This example shows how, given a set of Mars Science Laboratory (MSL) Curiosity
rover ``Nav`` or ``Mast`` camera images, CSM camera models can be created. Stereo
pairs are then used (with either ``Nav`` or ``Mast`` data) to make DEMs and
orthoimages.

After recent fixes in ALE (details below), the camera models are accurate enough
that stereo pairs acquired at different rover locations and across different days 
result in consistent DEMs and orthoimages.

See :numref:`rig_msl` for a Structure-from-Motion solution without using CSM
cameras. That one results in self-consistent meshes that, unlike the DEMs
produced here, are not geolocated.

Illustration
^^^^^^^^^^^^

.. figure:: ../images/MSL_Kimberly_images.png
  :name: csm_msl_figure1
  :alt:  MSL Kimberly images

  Four out of the 10 images (5 stereo pairs) used in this example.

.. figure:: ../images/MSL_Kimberly_DEM_DRG.png
  :name: csm_msl_figure2
  :alt:  MSL Kimberly DEM and ortho

  Produced DEM and orthoimage. See :numref:`csm_msl_multiday` for a larger
  example.

Fetch the images and metadata from PDS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

See :numref:`msl_image_prep`. Here we will work with .cub files rather than
converting them to .png. The same Mars day will be used as there (SOL 597). The
datasets for SOL 603 were verified to work as well.

The dataset used in this example (having .LBL, .cub, and .json files) is
available `for download
<https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/MSL_CSM>`_.
It is suggested to recreate the .json files in that dataset as done below.

Download the SPICE data
^^^^^^^^^^^^^^^^^^^^^^^

Fetch the SPICE kernels for MSL (see :numref:`planetary_images` and the links
from there).

.. _csm_msl_create:

Creation of CSM MSL cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set::

    export ALESPICEROOT=$ISISDATA

A full-resolution MSL left ``Nav`` image uses the naming convention::

      NLB_<string>_F<string>.cub

with the right image starting instead with ``NRB``. The metadata files
downloaded from PDS end with ``.LBL``.

A bug in the shipped metakernels requires editing the file::

    $ISISDATA/msl/kernels/mk/msl_v01.tm
    
and replacing::

    /usgs/cpkgs/isis3/data

with your value of $ISISDATA. Ensure that the resulting path still ends with
``/msl/kernels``.
    
A CSM camera file can be created by running::

    isd_generate image.LBL 

This will produce the file ``image.json``. 

If running into issues, invoke this command with the ``-v`` option to see where
it fails.

Simple stereo example
^^^^^^^^^^^^^^^^^^^^^

In this example the camera orientations are not refined using bundle adjustment,
as the camera poses are reasonably good. If desired to do that, one could run
``bundle_adjust`` (:numref:`bundle_adjust`) as::
  
    bundle_adjust --no-datum --camera-weight 0 --tri-weight 0.1 \
      data/*.cub data/*.json -o ba/run

Here and below we use the option ``--no-datum`` as these are ground-level cameras,
when rays emanating from them may not reliably intersect the planet datum.
  
For each stereo pair, run ``parallel_stereo`` (:numref:`parallel_stereo`) as::

    parallel_stereo                 \
      --stereo-algorithm asp_mgm    \
      --subpixel-mode 3 --no-datum  \
      --min-triangulation-angle 1.5 \
      left.cub right.cub            \
      left.json right.json          \
      run/run

If bundle adjustment was used, the above command should be run with the option
``--bundle-adjust-prefix ba/run``. 

The option ``--min-triangulation-angle 1.5`` is highly essential. It filters out
far-away and noisy points. Increasing this will remove more points.  For
terrains with a lot of shadows (such as for the Moon), also consider using the
option ``--no-data-value`` to filter out pixels with low intensity
(:numref:`stereodefault`).                      

This is followed by DEM and orthoimage creation (:numref:`point2dem`) with::

    point2dem --stereographic                \
      --proj-lon 137.402 --proj-lat -4.638   \
      --search-radius-factor 5 --orthoimage  \
      run/run-PC.tif run/run-L.tif
     
Here, the option ``--search-radius-factor 5`` is used to fill the point cloud
when moving further from the rover. A local stereographic projection was used. 

The produced DEMs can be mosaicked together with ``dem_mosaic``
(:numref:`dem_mosaic`) as::

    dem_mosaic */*DEM.tif -o dem_mosaic.tif

For the orthoimages, one can use::

    dem_mosaic --first */*DRG.tif -o ortho_mosaic.tif

The option ``--first`` picks the first encountered image pixel at each location,
rather than  blending them together which may blur the output mosaic. 

See an illustration in :numref:`csm_msl_figure2`, with the input images in :numref:`csm_msl_figure1`. 

.. _csm_msl_multiday:

Multi-day stereo
^^^^^^^^^^^^^^^^

.. figure:: ../images/msl_multiday.png
  :name: msl_multiday
  :alt:  MSL multiday stereo

  A combined DEM and orthoimage produced from 15 stereo pairs from SOL 597 and
  13 stereo pairs from SOL 603. The misregistration half-way down is not due to
  mismatch across days but because of insufficient overlap between two image
  subsets on SOL 603. Here, blue and red correspond to elevations of -5038.921
  and -5034.866 meters.

In this example we take advantage of the fact that there is decent overlap
between images acquired on SOL 597 and SOL 603. They both image the same hill,
called *Kimberly*, in Gale crater, from somewhat different perspectives. Hence
we combine these datasets to increase the coverage.

Good overlap between different days, or even between consecutive rover
stops in the same day, is not guaranteed. Sometimes the low-resolution nav cam
images (:numref:`low_res_msl`) can help with increasing the overlap and
coverage. Lack of good overlap can result in registration errors, as can be seen
in :numref:`msl_multiday`.

For a larger and better-behaved dataset it is suggested to consider the images
from SOL 3551 to 3560. Some effort may be needed to select a good subset.

A workflow can be follows. First, individual DEMs were created and mosaicked,
as in :numref:`csm_msl`. The quality of the produced DEM can be quite uneven,
especially far from the camera. 

Large holes in the initial DEM were filled in with the ``dem_mosaic`` option
``--fill-search-radius`` (:numref:`dem_mosaic_extrapolate`). 

Then, it can be made coarser, for example, as::

    gdalwarp -r cubic -tr 0.1 0.1 input.tif output.tif

(This assumes the projection is local stereographic.)
    
This DEM was then blurred a few times with ``dem_mosaic`` option
``--dem-blur-sigma 10``. This should be repeated until the DEM is smooth enough
and shows no artifacts. The resulting DEM is called ``dem.tif``.

All images were mapprojected onto this DEM using the same local stereographic
projection, and a resolution of 0.01 m::

    proj="+proj=stere +lat_0=-4.638 +lon_0=137.402 +k=1 +x_0=0 +y_0=0 +R=3396190 +units=m +no_defs"
    mapproject --tr 0.01 --t_srs "$proj" \
      dem.tif image.cub image.json image.map.tif

Bundle adjustment was run on the desired set of input images and cameras, while
making use of the mapprojected images to find matches::

  dem=dem.tif
  parallel_bundle_adjust                    \
    --image-list images.txt                 \
    --camera-list cameras.txt               \
    --mapprojected-data-list map_images.txt \
    --camera-weight 0                       \
    --heights-from-dem $dem                 \
    --heights-from-dem-uncertainty 10.0     \
    --heights-from-dem-robust-threshold 0.1 \
    --auto-overlap-params "$dem 15"         \
    -o ba/run

In retrospect, this mapprojection step may be not necessary, and one could
run bundle adjustment with original images.

Then ``parallel_stereo`` was run with mapprojected images, with the option
``--bundle-adjust-prefix ba/run``, to use the bundle-adjusted cameras::

    parallel_stereo                    \
      --stereo-algorithm asp_mgm       \
      --subpixel-mode 9                \
      --max-disp-spread 80             \
      --min-triangulation-angle 1.5    \
      --bundle-adjust-prefix ba/run    \
      left.map.tif right.map.tif       \
      left.json right.json run_map/run \
      $dem

    point2dem --tr 0.01 --stereographic    \
      --proj-lon 137.402 --proj-lat -4.638 \
      --errorimage                         \
      run_map/run-PC.tif                   \
      --orthoimage run_map/run-L.tif

Each run must use a separate output prefix, instead of ``run_map/run``.

Here, the option ``--min-triangulation-angle 1.5`` was highly essential.
It filters out far-away and noisy points. 

Even with this option, the accuracy of a DEM goes down far from the cameras.
Artifacts can arise where the same region is seen from two different locations,
and it is far from either. In this particular example some problematic portions
were cut out with ``gdal_rasterize`` (:numref:`gdal_rasterize_example`).

The produced DEMs were inspected, and the best ones were mosaicked together with
``dem_mosaic``, as follows::

    dem_mosaic --weights-exponent 0.5 */*DEM.tif -o dem_mosaic.tif
 
The option ``--weights-exponent 0.5`` reduced the artifacts in blending.

The orthoimages were mosaicked with::

    dem_mosaic --first */*DRG.tif -o ortho_mosaic.tif
    
It is suggested to sort the input images for this call from best to worst in
terms of quality. In particular, the images where the rover looks down rather
towards the horizon should be earlier in the list.

See the produced DEM and orthoimage in :numref:`msl_multiday`.

Mapprojection
^^^^^^^^^^^^^

The input .cub image files and the camera .json files can be used to create
mapprojected images with the ``mapproject`` program (:numref:`mapproject`). 
The DEM for mapprojection can be the one created earlier with ``point2dem``.
If a third-party DEM is used, one has to make sure its elevations are consistent
with the DEMs produced earlier.

Use the option ``--t_projwin`` to prevent the produced images from extending for
a very long distance towards the horizon.

MSL Mast cameras
^^^^^^^^^^^^^^^^

The same procedure works for creating MSL Mast cameras. To run stereo, first use
``gdal_translate -b 1`` to pull the first band from the input images. This
workflow was tested with the stereo pair ``0706ML0029980010304577C00_DRCL`` and
``0706MR0029980000402464C00_DRCL`` for SOL 706.

.. _low_res_msl:

Low-resolution MSL Nav cam images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In addition to full-resolution Nav camera images (1024 pixels), MSL also
acquires low-resolution Nav images (256 pixels) at separate times. These have
the string ``_D`` as part of their name, instead of ``_F``. Such images were
validated to work, and can produce good DEMs that can plug some gaps in
coverage.

.. _csm_state:

CSM model state
~~~~~~~~~~~~~~~

CSM cameras are stored in JSON files, in one of the following two formats:

* *ISD*: This has the transforms from sensor coordinates to J2000, and from
  J2000 to ECEF. 
* *Model state*: In this file the above-mentioned transforms are combined, and
  other information is condensed or removed. 
   
The model state files have all the data needed to project ground points into the
camera and vice-versa, so they are sufficient for any use in ASP. The model state can
also be embedded in ISIS cubes (:numref:`embedded_csm`).

The `usgscsm_cam_test
<https://github.com/DOI-USGS/usgscsm/blob/main/docs/source/tools/usgscsm_cam_test.rst>`_
program (shipped with ASP) can read any of these and write back a model state.

ASP's bundle adjustment program (:numref:`bundle_adjust`) normally writes plain
text ``.adjust`` files that encode how the position and orientation of the
cameras were modified (:numref:`adjust_files`). If invoked for CSM cameras,
additional files with extension ``.adjusted_state.json`` are saved in the same
output directory, which contain the model state from the input CSM cameras with
the optimization adjustments applied to them. Use zero iterations in
``bundle_adjust`` to save the states of the original cameras.

This functionality is implemented for all USGS CSM sensors, so, for ``frame``,
``linescan``, ``pushframe``, and ``SAR`` models.

The ``cam_gen`` program can convert several linescan camera model types to CSM
model state (:numref:`cam_gen_linescan`). It can also approximate some Pinhole,
RPC, or other cameras with CSM frame cameras in model state format
(:numref:`cam_gen_frame`). 

ASP's ``parallel_stereo`` and bundle adjustment programs can, in addition to CSM
ISD camera model files, also load such model state files, either as previously
written by ASP or from an external source (it will auto-detect the type from the
format of the JSON files). Hence, the model state is a convenient format for
data exchange, while being less complex than the ISD format.

If ``parallel_stereo`` is used to create a point cloud from
images and CSM cameras, and then that point cloud has a transform
applied to it, such as with ``pc_align``, the same transform can be
applied to the model states for the cameras using ``bundle_adjust``
(:numref:`ba_pc_align`).
 
To evaluate how well the obtained CSM camera approximates the ISIS
camera model, run the program ``cam_test`` shipped with ASP
(:numref:`cam_test`) as follows::

    cam_test --sample-rate 100 --image camera.cub \
      --cam1 camera.cub --cam2 camera.json

The pixel errors are expected to be at most on the order of 0.2
pixels.

.. _embedded_csm:

CSM state embedded in ISIS cubes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ASP usually expects CSM cameras to be specified in JSON files. It also accepts
CSM camera model state data (:numref:`csm_state`) embedded in ISIS cubes, if all
of the following (reasonable) assumptions are satisfied: 

 * JSON files are not passed in.
 * The ISIS cubes contain CSM model state data (in the ``CSMState`` group).
 * The ``--session-type`` (or ``--t``) option value is not set to ``isis`` (or
   ``isismapisis``). So, its value should be either ``csm`` (or ``csmmapcsm``),
   or left blank.
 
Hence, if no CSM data is provided, either in the ISIS cubes or separately
in JSON files, or ``--session-type`` is set to ``isis`` (or ``isismapisis``),
ASP will use the ISIS camera models.

The above applies to all ASP tools that read CSM cameras (``parallel_stereo``,
``bundle_adjust``, ``jitter_solve``, ``mapproject``, ``cam_test``).

If ``bundle_adjust`` (:numref:`bundle_adjust`) or ``jitter_solve``
(:numref:`jitter_solve`) is run with CSM cameras, either embedded in ISIS cubes
or specified separately, and the flag ``--update-isis-cubes-with-csm-state`` is
set, then the optimized model states will be saved back to the ISIS cubes, while
the SPICE and other obsolete information from the cubes will be deleted.
(Note that `spiceinit
<https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/spiceinit/spiceinit.html>`_
can restore the cubes.)

Separate model state files in the JSON format will be saved by ``bundle_adjust``
as well, as done without this option.

Note that if images are mapprojected with certain camera files, and then those
camera files are updated in-place, this will result in wrong results if stereo
is run with the old mapprojected images and updated cameras.
  
The `csminit
<https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/csminit/csminit.html>`_
program can also embed a .json model state file into a .cub file (in ISIS
9.0.0 and later). Example::

    csminit from = img.cub state = csm.json

