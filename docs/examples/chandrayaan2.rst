.. _chandrayaan2:

Chandrayaan-2 lunar orbiter
---------------------------

The examples here show how to create 3D terrain models with `Chandrayaan-2 lunar
orbiter <https://en.wikipedia.org/wiki/Chandrayaan-2>`_ data. We will work with
its *Orbiter High Resolution Camera* (OHRC). *A Terrain Mapping Camera-2* (TMC-2)
example will be added at a later time.

For the moment, this exercise expects `ISIS <https://github.com/DOI-USGS/ISIS3>`_
and `ALE <https://github.com/DOI-USGS/ale>`_ to be compiled and installed from
source (to separate locations), and that SPICE kernels be downloaded from the `ISRO
Science Data Archive
<https://pradan.issdc.gov.in/ch2/protected/browse.xhtml?id=spice>`_. The very
latest build of ASP (:numref:`release`) is also required.

All these are temporary and onerous requirements that will be removed in the
near future.

Orbiter High Resolution Camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The OHRC instrument is a high-resolution camera with a 0.25 m ground sample distance.
It can adjust its look angle and acquire stereo pairs (:numref:`stereo_pairs`).

Fetching the data
^^^^^^^^^^^^^^^^^

Raw and calibrated images for OHRC and TMC-2 cameras, as well as orthoimages and
DEMs produced from TMC-2 camera data, can be downloaded from 
`ISRO <https://chmapbrowse.issdc.gov.in/>`_.

The first step when using that interface is selecting the appropriate projection
for displaying the image footprints. Then, choose the instrument (OHRC or
TMC-2), data type (calibrated is suggested, but raw may do), and the area of
interest. 

We selected the region of interest to be 20 to 21 degrees in longitude (East),
and -70 to -67 degrees in latitude (so, South). The OHRC stereo pair we
downloaded consisted of images with prefixes::

	ch2_ohr_nrp_20200827T0030107497_d_img_d18
	ch2_ohr_nrp_20200827T0226453039_d_img_d18

We also got a TMC-2 orthoimage and corresponding DEM with the prefixes::

  ch2_tmc_ndn_20231101T0125121377_d_oth_d18
  ch2_tmc_ndn_20231101T0125121377_d_dtm_d18
    
These are at lower resolution but useful for context.

.. figure:: ../images/chandrayaan2_ohrc_tmc.png

  From left to right: The first and second OHRC image, and their approximate
  extent in the (many times larger) TMC-2 ortho image. Note that the illumination
  in the TMC-2 ortho image is very different.
  
Preprocessing
^^^^^^^^^^^^^

Each calibrated image dataset has ``.img`` and ``.xml`` files, with raw data and
a PDS-4 label. It will be convenient to rename these to ``ohrc/img1.img`` and
``ohrc/img1.xml`` for the first OHRC dataset, and analogously for the second
one.

The `isisimport <https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/isisimport/isisimport.html>`_ command converts the raw image to a .cub file::

    
    isisimport                 \
      from     = ohrc/img1.xml \
      to       = ohrc/img1.cub \
      template = ${template}

(and same for the second image). Here, the ``template`` variable is set such
as::

    template=/path/to/ISIS3/isis/appdata/import/PDS4/Chandrayaan2OHRC.tpl

Then, the SPICE kernels are attached with `spiceinit <https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/spiceinit/spiceinit.html>`_::

    spiceinit from = ohrc/img1.cub

This expects the SPICE kernels for Chandrayaan-2 to exist locally (download link
is above). For more information on ISIS data, see :numref:`planetary_images` and the
links from there. 

Next, the CSM cameras are created (:numref:`csm`). This makes use of the `isd_generate <https://astrogeology.usgs.gov/docs/getting-started/using-ale/isd-generate/>`_ program installed with the latest ALE (link above). The command is::

    export ALESPICEROOT=$ISISDATA
    isd_generate -k ohrc/img1.cub ohrc/img1.cub 

and same for ``img2.cub``. Here the .cub file is specified twice, with the
first file needed to read the SPICE kernels.

It is suggested to do a quick check on the produced ``ohrc/img1.json`` camera
with ``cam_test`` (:numref:`cam_test`).

The images can be inspected with ``stereo_gui`` (:numref:`stereo_gui`), as::

  stereo_gui ohrc/img1.cub ohrc/img2.cub
  
The resulting cub files are very large, on the order of 12,000 x 101,075 pixels.
For exploratory work, these can be cropped, with the ISIS `crop
<https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/crop/crop.html>`_
command, such as::

    crop                            \
      from     = ohrc/img1.cub      \
      to       = ohrc/img1_crop.cub \
      sample   = 1                  \
      line     = 1                  \
      nsamples = 12000              \
      nlines   = 20000

It is very important to ensure that the upper-left pixel (1, 1) is part of the
crop region, as otherwise the resulting images will be inconsistent with the CSM
camera models.

Bundle adjustment
^^^^^^^^^^^^^^^^^

We found that these images have notable pointing error, so bundle adjustment
(:numref:`bundle_adjust`) is needed::

    bundle_adjust                           \
      ohrc/img1_crop.cub ohrc/img2_crop.cub \
      ohrc/img1.json ohrc/img2.json         \
      --ip-per-image 20000                  \
      -o ba/run

This stereo pair was seen to have a decent convergence angle of 25 degrees
(:numref:`ba_conv_angle`).

.. figure:: ../images/chandrayaan2_ohrc_interest_points.png

  The left and right cropped OHRC images, and the interest point matches between
  them (as shown by ``stereo_gui``, :numref:`stereo_gui_view_ip`).

Stereo
^^^^^^

Next, we invoked ``parallel_stereo`` (:numref:`parallel_stereo`) to create a point cloud::

    parallel_stereo                     \
      --stereo-algorithm asp_mgm        \
      --clean-match-files-prefix ba/run \
      ohrc/img1_crop.cub                \
      ohrc/img2_crop.cub                \
      ba/run-img1.adjusted_state.json   \
      ba/run-img2.adjusted_state.json   \
      stereo/run

A DEM, orthoimage, and triangulation error image are made with ``point2dem``
(:numref:`point2dem`), as::

    point2dem           \
      --tr 1.0          \
      --errorimage      \
      stereo/run-PC.tif \
      --orthoimage      \
      stereo/run-L.tif 
      
In a recent version of ASP these will have by default a local stereographic
projection.

.. figure:: ../images/chandrayaan2_ohrc_dem_ortho_err.png

  From left to right: Produced OHRC DEM (range of heights is 370 to 560 meters),
  orthoimage, and triangulation error image (blue = 0 m, red = 0.25 m). This
  looks reasonable enough. There is notable jitter (:numref:`jitter_solve`),
  whose magnitude is about 0.25 m, which is the image GSD, so not too bad. Some
  unmodeled lens distortion also seems evident, which could be solved for
  (:numref:`kaguya_ba`). 

