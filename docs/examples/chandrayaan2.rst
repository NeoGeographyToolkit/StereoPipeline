.. _chandrayaan2:

Chandrayaan-2 lunar orbiter
---------------------------

The example here shows how to create a 3D terrain model with `Chandrayaan-2
lunar orbiter <https://en.wikipedia.org/wiki/Chandrayaan-2>`_ data, using both
the *Orbiter High Resolution Camera* (OHRC) and the *Terrain Mapping Camera-2*
(TMC-2).

This workflow uses the Community Sensor Model (:numref:`csm`). It needs ASP
3.6.0 or later (:numref:`release`), `ISIS
<https://github.com/DOI-USGS/ISIS3>`_ 10.0.0_RC2 (or a later release-candidate
or stable version), `ALE <https://github.com/DOI-USGS/ale>`_ 1.1.3 or later, and
`USGSCSM <https://github.com/DOI-USGS/usgscsm>`_ 2.0.2 or later. All are available as
public conda packages (see below).

Chandrayaan-2 ISIS data should be downloaded as documented further down.

Environment setup
~~~~~~~~~~~~~~~~~

Fetch ISIS, ALE, USGSCSM, and ``rclone`` into a fresh conda environment::

    conda create -n isis10rc2                     \
       -c usgs-astrogeology/label/RC              \
       -c conda-forge                             \
       isis=10.0.0_RC2 ale=1.1.3 usgscsm=2.0.2 rclone

Activate the environment::

    conda activate isis10rc2
    export ISISROOT=$CONDA_PREFIX

Set the location of the ISIS data area (to be downloaded next)::

    export ISISDATA=$HOME/projects/isisdata
    export ALESPICEROOT=$ISISDATA

Install ASP, build 2026/5 or newer (:numref:`release`), as some
bugfixes for the Chandrayaan-2 cameras were incorporated at that
time. Set its path as documented there.

See also the `USGS ISIS TMC documentation
<https://astrogeology.usgs.gov/docs/getting-started/csm-stack/ingesting-tmc2/>`_.

Downloading the Chandrayaan-2 ISIS data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The mission kernels are fetched with ``downloadIsisData``, which is shipped
with ISIS::

    downloadIsisData chandrayaan2 $ISISDATA

Note that the full ``chandrayaan2`` directory is large (about 200 GB), of which
essentially all is reconstructed attitude kernels (``ck``) covering the entire
mission since 2019. For a single OHRC image only one or two ``ck`` files are
needed. Fetching everything except ``ck`` takes only a few hundred MB::

    downloadIsisData chandrayaan2 $ISISDATA --exclude="kernels/ck/**"

The command::

    rclone --config $ISISROOT/etc/isis/rclone.conf \
      ls chandrayaan2:kernels/ck/

lists all available ``ck`` files. This can help pick the ones that span the
acquisition time of the products to be processed.

The ``ck`` files matching the orbit dates of interest can then be fetched
individually with ``rclone``, such as::

    rclone                                          \
      --config $ISISROOT/etc/isis/rclone.conf       \
      copy                                          \
      chandrayaan2:kernels/ck/                      \
      $ISISDATA/chandrayaan2/kernels/ck/            \
      --include="ch2_att_27Jul2020_04Sep2020_v1.bc" \
      --include="ch2_att_27Aug2020_04Oct2020_v1.bc" \
      --no-traverse -P

Orbiter High Resolution Camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The OHRC instrument is a high-resolution camera with a 0.25 m ground sample
distance (GSD). It can adjust its look angle and acquire stereo pairs
(:numref:`stereo_pairs`).

Fetching the data
^^^^^^^^^^^^^^^^^

Raw and calibrated images for OHRC and TMC-2 cameras, as well as orthoimages and
Digital Elevation Models (DEMs) produced from TMC-2 camera data, can be
downloaded from `ISRO <https://chmapbrowse.issdc.gov.in/>`_.

The first step when using that portal is selecting the appropriate projection
for displaying the image footprints. Then, choose the instrument (OHRC or
TMC-2), data type (calibrated is suggested, but raw may do), and the area of
interest.

We selected the region of interest to be between 20 and 21 degrees in longitude,
and -70 to -67 degrees in latitude. The OHRC stereo pair we downloaded consisted
of images with the prefixes::

    ch2_ohr_nrp_20200827T0030107497_d_img_d18
    ch2_ohr_nrp_20200827T0226453039_d_img_d18

We also got a TMC-2 orthoimage and corresponding DEM with the prefixes::

    ch2_tmc_ndn_20231101T0125121377_d_oth_d18
    ch2_tmc_ndn_20231101T0125121377_d_dtm_d18

These are at lower resolution but useful for context.

.. figure:: ../images/chandrayaan2_ohrc_tmc.png

  From left to right: The first and second OHRC images, and their approximate
  extent in the (many times larger) TMC-2 ortho image. Note that the illumination
  in the TMC-2 ortho image is very different.

Preprocessing
^^^^^^^^^^^^^

Each calibrated image dataset has ``.img`` and ``.xml`` files, with raw data and
a PDS-4 label. It will be convenient to rename these to ``ohrc/img1.img`` and
``ohrc/img1.xml`` for the first OHRC dataset, and analogously for the second
one.

The `isisimport <https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/isisimport/isisimport.html>`_ command converts the raw image to a .cub file::

    isisimport from = ohrc/img1.xml to = ohrc/img1.cub

(and same for the second image). The PDS4 template is auto-detected in ISIS
10.

The ``isisimport`` command only works with raw images and not with ortho images.

The SPICE kernels are attached with `spiceinit <https://isis.astrogeology.usgs.gov/Application/presentation/Tabbed/spiceinit/spiceinit.html>`_::

    spiceinit from = ohrc/img1.cub

This expects the SPICE kernels for Chandrayaan-2 to exist locally under
``$ISISDATA/chandrayaan2/`` (see the download instructions above). For more
information on ISIS data, see :numref:`planetary_images` and the links from
there.

Next, CSM cameras are created with `isd_generate
<https://astrogeology.usgs.gov/docs/getting-started/using-ale/isd-generate/>`_
from the ALE package, following the linescan recipe in
:numref:`create_csm_linescan`::

    export ALESPICEROOT=$ISISDATA
    isd_generate -k ohrc/img1.cub ohrc/img1.cub
    isd_generate -k ohrc/img2.cub ohrc/img2.cub

This expects ``$ISISDATA`` and ``$ALESPICEROOT`` to be set as described in
the environment setup section above. Check each produced JSON with
``cam_test`` (:numref:`cam_test`) before proceeding.

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
      nlines   = 50000

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
      --ip-per-image 30000                  \
      -o ba/run

This stereo pair has a convergence angle of about 25 degrees
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

In a recent version of ASP these will, by default, have a local stereographic
projection.

.. figure:: ../images/chandrayaan2_ohrc_dem_ortho_err.png

  From left to right: Produced OHRC DEM (range of heights is 304 to 650 meters),
  orthoimage, and triangulation error image (blue = 0 m, red = 0.5 m). There is
  notable jitter, whose magnitude is on the order of image GSD (0.25 m), which
  is rather high, but which could be corrected (:numref:`jitter_solve`). Some
  unmodeled lens distortion also seems evident, which could be solved for
  (:numref:`kaguya_ba`).

Alignment
^^^^^^^^^

We aligned the produced OHRC DEM to `LOLA
<https://ode.rsl.wustl.edu/moon/lrololadataPointSearch.aspx>`_, which is the
usual global reference coordinate system for the Moon.

The OHRC DEM turned out to be shifted relative to LOLA by about 4 km along the
satellite track, which resulted in failure to align with ``pc_align``
(:numref:`pc_align`).

Manual alignment was first performed (:numref:`manual-align`). The inputs were
the OHRC DEM and a LOLA point cloud, after gridding both with a 10 m grid size
and the same projection with ``point2dem``, and manually picking a few
visually similar features. That brought the cloud notably closer, and the output
transform from that alignment was used for aligning the full clouds as::

    pc_align                                  \
      --max-displacement 250                  \
      --initial-transform init-transform.txt  \
      --csv-format 2:lon,3:lat,4:radius_km    \
      --save-inv-transformed-reference-points \
      stereo/run-DEM.tif lola/lola.csv        \
      -o stereo/run-align

.. figure:: ../images/chandrayaan2_ohrc_lola.png

  The difference between the aligned OHRC DEM and LOLA point cloud. Blue = -5 m,
  red = 5 m. Given that the DEM, in principle, should have a vertical
  uncertainty of under 1 m, this could be better, but at least we are in the
  ballpark.

A terrain model created with the lower-resolution TMC-2 images would likely be
easier to align to LOLA, as it would have a much bigger extent.

Terrain Mapping Camera-2
~~~~~~~~~~~~~~~~~~~~~~~~

The TMC-2 instrument is a 3-line pushbroom camera, with separate forward (fwd),
downward-pointing (nadir), and backward (aft) detectors. The fwd detector looks
~25 degrees ahead of nadir and the aft detector looks ~25 degrees behind, a
setup which is well-suited to stereo with any of these image pairs. The ground
sample distance is about 5 meters at 100 km altitude.

All three detectors record simultaneously, so a substantial ground swath is
imaged by all of them.

Input data
^^^^^^^^^^

We use the fwd/aft pair::

    ch2_tmc_ncf_20231101T0125121344_d_img_d18
    ch2_tmc_nca_20231101T0125121377_d_img_d18

The corresponding pre-existing DTM (``ch2_tmc_ndn_20231101T0125121377``,
mentioned earlier) covers the same orbit pass. These images include the
footprint of the OHRC images from earlier but extend well beyond them.

Creation of cameras
^^^^^^^^^^^^^^^^^^^

We use only the CSM camera models (:numref:`csm`), as it appears that the
non-nadir TMC ISIS camera models in the .cub files are still problematic (as of
5/2026). The fix is to skip ``spiceinit`` entirely and run ``isd_generate``
directly.

ALE 1.1.3 (without ``spiceinit``-attached kernels) needs a metakernel file
under ``$ALESPICEROOT`` to locate SPICE kernels, but the USGS Chandrayaan-2
ISIS data area (as of 5/2026) does not ship one. The workaround is to create
a small metakernel locally at::

    $ISISDATA/chandrayaan2/kernels/mk/ch2_v01.tm

listing the kernel files. The values in ``PATH_VALUES`` should be absolute
due to limitations in ALE, and should be correct for the local file system.
See the NAIF `Metakernel reference
<https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/FORTRAN/req/kernel.html>`_
for the file format and compare with existing ``.tm`` files for other
missions.

With the metakernel in place, the workflow is as follows::

    export ALESPICEROOT=$ISISDATA
    isisimport from = tmc/fwd.xml to = tmc/fwd.cub
    isisimport from = tmc/aft.xml to = tmc/aft.cub
    isd_generate tmc/fwd.cub
    isd_generate tmc/aft.cub

Check each JSON with ``cam_test`` (:numref:`cam_test`).

Bundle adjustment
^^^^^^^^^^^^^^^^^

Bundle-adjust (:numref:`bundle_adjust`) the fwd/aft pair with the JSONs as
cameras::

    bundle_adjust                           \
      tmc/fwd.cub tmc/aft.cub               \
      tmc/fwd.json tmc/aft.json             \
      --num-iterations 100 --num-passes 2   \
      --camera-weight 0 --tri-weight 0.1    \
      --remove-outliers-params "75 3 50 50" \
      --ip-per-image 100000                 \
      --max-pairwise-matches 50000          \
      -o ba/run

It is suggested to inspect the produced report files (:numref:`ba_out_files`).

Stereo
^^^^^^

Run stereo with the bundle-adjusted cameras using ``affineepipolar`` alignment
(:numref:`tutorial`), then create a DEM::

    parallel_stereo                      \
      --alignment-method affineepipolar  \
      --stereo-algorithm asp_mgm         \
      --subpixel-mode 9                  \
      tmc/fwd.cub tmc/aft.cub            \
      ba/run-tmc_fwd.adjusted_state.json \
      ba/run-tmc_aft.adjusted_state.json \
      stereo/run

    point2dem --tr 20    \
      --errorimage       \
      --stereographic    \
      --auto-proj-center \
      stereo/run-PC.tif

The ``--tr 20`` grid size is about 4 x the 5 m TMC ground sample distance. The
``--stereographic --auto-proj-center`` options pick a local stereographic
projection centered on the cloud, which is appropriate for TMC's polar orbits (a
UTM-like default would distort heavily near the poles).

The produced DEM looked reasonably good. No jitter (:numref:`jitter_solve`) was
observed in the triangulation error image (:numref:`point2dem_ortho_err`). The
DEM extent was overestimated, likely due to the extreme aspect ratio of TMC
strips (about 190,000 lines x 4636 samples) and outliers due to shadow. This
extent can be set explicitly with ``--t_projwin`` if known.

Consider also running stereo on mapprojected images
(:numref:`mapproj-example`). The initial DEM used for mapprojection can be
either:

- A prior TMC DTM as provided by ISRO (``ch2_tmc_ndn_*_d_dtm_d18``).
- A LOLA gridded DEM (see :numref:`sfs_initial_terrain`).
- A DEM gridded from LOLA CSV samples with ``point2dem``
  (:numref:`point2dem_csv`).

In either case, the input DEM should first be grown to fill in any holes
(:numref:`dem_mosaic_extrapolate`) and then blurred (``dem_mosaic
--dem-blur-sigma 5``, :numref:`dem_mosaic_blur`) before being used as the
mapprojection reference.

With mapprojection, results were somewhat better, though some staircasing
remains with ``asp_mgm`` and ``--subpixel-mode 9``. We believe this is due to
the large difference in perspective between the fwd and aft cameras and extreme
aspect ratio.

.. figure:: ../images/chandrayaan2_tmc_dem_err.png

  Left: a portion of the colorized hillshaded DEM produced with mapprojected TMC
  images. Color range corresponds to elevations of approximately -1130 to 2400
  meters. Right: triangulation error image (:numref:`point2dem_ortho_err`), with
  the color range from 0 to 5 meters (the image ground sample distance).

It is suggested to also run stereo between the fwd/nadir and nadir/aft
stereo pairs. The ``--alignment-method local_epipolar`` option
(:numref:`image_alignment`) is not recommended for the TMC fwd/aft pair (the
per-tile epipolar refinement does not handle the large change in perspective
well), but may work better on the other stereo pair combinations.

For preliminary investigations, run stereo on a smaller region first
by mapprojecting onto a cropped version of a prior DEM.
