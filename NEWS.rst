Changes since last release
---------------------------

camera_solve (:numref:`camera_solve`):
  * Works on Mac Arm.
  * Removed dependency on OpenImageIO. 

parallel_stereo (:numref:`parallel_stereo`):
  * Added an example for JunoCam images (:numref:`junocam`).
  * Added examples for the Chandrayaan-2 lunar orbiter (:numref:`chandrayaan2`).
  * Rewrote the KH-9 example to take into account the improved modeling of
    optical bar cameras and a new strategy for fixing local warping
    (:numref:`kh9`).
  * Left and right alignment matrices are now saved in plain text format. Older
    .exr files are still read. Support for them will be removed in the next
    release (:numref:`outputfiles`).
  * Erode less at at image boundary during filtering when ``--subpixel-mode`` is
    not between 1 and 6 (option ``--edge-buffer-size``,
    :numref:`filter_options`).
  * Determination of interest point matches is optional when either
    ``--corr-search`` or ``--seed-mode 2`` are set
    (:numref:`stereodefault`).
    
parallel_sfs (:numref:`parallel_sfs`):
  * When albedo and / or haze is modeled, initial estimates for these are
    produced for the full site (:numref:`parallel_sfs_usage`).

parallel_bundle_adjust (:numref:`parallel_bundle_adjust`):
  * Bugfix for a crash when there are no interest point matches.

bundle_adjust (:numref:`bundle_adjust`):
  * Changed the implementation of the camera position constraint
    (:numref:`ba_cam_constraints`).
  * The option ``--auto-overlap-params`` accepts an optional third argument (all
    in quotes) that has the number of subsequent images that overlap to use in
    matching.
  * For the option ``--mapprojected-data``, the DEM specified at the end is
    optional, if it can be looked up from the geoheader of the mapprojected
    images.
    
point2dem (:numref:`point2dem`):
  * Added support for LAS COPC files (:numref:`point2dem_las`).
  * Added the option ``--gdal-tap``.
  * Removed unused options: ``--phi-rotation``, ``--omega-rotation``, 
    ``--kappa-rotation``. A rotation matrix can be applied with ``pc_align``
    instead.
    
pc_align (:numref:`pc_align`):
  * Added support for LAS COPC files (:numref:`pc_align_las`).
  
cam_gen (:numref:`cam_gen`):
  * Can fit a CSM linescan camera to an OpticalBar camera
    (:numref:`opticalbar2csm`).
  * Can fit a CSM frame camera model with radial distortion.  
  * Support pixel pitch that is not just 1 in CSM cameras. 
  * If the input is an ISIS cube and the output is a CSM camera, save the
    ephemeris time, sun position, serial number, and target (planet) name.
  * Added the option ``--camera-center-llh``.

mapproject (:numref:`mapproject`):
  * Added the option ``--gdal-tap``.
  
dem2gcp (:numref:`dem2gcp`):
  * Added the options ``--max-num-gcp``, ``--max-disp``, ``--gcp-sigma-image``.

jitter_solve (:numref:`jitter_solve`):
  * Added documentation on limitations (:numref:`jitter_limitations`).
  * Changed the implementation of the camera position constraint
    (:numref:`jitter_camera`).
  * Added the options ``--fix-gcp-xyz``, ``--use-lon-lat-height-gcp-error``.

point2las (:numref:`point2las`):
  * Added the option ``--dem`` to convert a DEM to LAS.

dem_mosaic (:numref:`dem_mosaic`):
  * Added the option ``--gdal-tap``.
  
misc:
  * Added minimum system requirements for running ASP (:numref:`system_rec`).
  * Made the OpticalBar model 3x faster by switching from minimization in 3D
    to root-finding in 2D with the Newton-Raphson method.
  * Turned off experimental ``--subpixel-mode 6`` as it is failing to run
    (:numref:`subpixel_options`).
  * Unused ``pca`` mode in ``ipfind`` got removed.
  * Bugfix for modifying the creation time of ISIS cubes when it was meant to
    only read them.
  * Bugfix for when ``parallel_stereo`` has the same value for the output prefix
    and bundle adjustment prefix.
  * Bugfix for stereo triangulation when the point cloud is huge and has data
    only in corners.

RELEASE 3.5.0, April 28, 2025
-----------------------------

DOI: `10.5281/zenodo.15298734 <https://zenodo.org/records/15298734>`_

Stable release doc: https://stereopipeline.readthedocs.io/en/stable/index.html

*New platform*: An experimental native Mac M1/M2 Arm64 build is available
(:numref:`release`).
  
bundle_adjust (:numref:`bundle_adjust`):
  * Replaced the algorithm for creating control networks when there are more
    than two images. Notably more features in more than two images now can be
    found.
  * Added the option ``--save-adjusted-rpc`` to save RPC cameras with adjustments
    applied to them (:numref:`rpc_and_ba`).
  * Added the option ``--min-distortion`` to ensure small distortion parameters
    get optimized.
  * Added the option ``--max-triangulation-angle``.
  * Compensate for the images in the input nvm being potentially in different
    order than the images specified on the command line.  
  * The report file measuring statistics of registration errors on the ground
    got broken up into errors per image and per image pair
    (:numref:`ba_mapproj_dem`).
  
parallel_bundle_adjust (:numref:`parallel_bundle_adjust`):
   * The default number of processes per node is 1/4 of the number of cores on
     the head node, and the default number of threads per process is the number
     of cores on the head node over the number of processes.
   * The number of launched jobs is number of nodes times number of processes
     per node. This appears best for load balancing.  
   * Create interest points (before matching) once per image, not each time per
     image pair. This speeds up the processing.
     
mapproject (:numref:`mapproject`):
  * If the input DEM is in the ``longlat`` projection, a projection 
    in meters is auto-determined (:numref:`mapproj_auto_proj`).
  * Added the option ``--ref-map`` to borrow the grid size and projection from
    an existing mapprojected image (:numref:`mapproj_refmap`).
  * Add the option ``--query-pixel``.

jitter_solve (:numref:`jitter_solve`):
  * Do two passes by default. This improves the results.
  * Can model rig constraints between sensors (:numref:`jitter_rig`).
  * Added an example for the Kaguya Terrain Camera (:numref:`jitter_kaguya`).
  * Added the option ``--camera-position-uncertainty`` (:numref:`jitter_camera`).
  * Can constrain against a sparse point cloud (:numref:`jitter_ref_terrain`).
  * Added the option ``--smoothness-weight`` to control high-frequency changes
    in the camera orientations in linescan cameras.
  * Can use GCP files.
  * Can read a control network from an nvm file.
  * Write the stereo convergence angles. Can write registration errors on the
    ground (:numref:`other_jitter_out`).

stereo_gui (:numref:`stereo_gui`):
  * Changing the image threshold updates the display correctly.
  * When creating GCP, ask before quitting without saving them. Save the IP as
    well when GCP are saved.
  * Added the option ``--gcp-sigma`` for creating GCP.  
  * Big speedup when rendering a stack of georeferenced images.

image_calc (:numref:`image_calc`):
  * Added an example for how to extract the horizontal and vertical disparity
    bands while setting invalid disparities to a no-data value
    (:numref:`mask_disparity`).

sat_sim (:numref:`sat_sim`):
  * Added the option ``--rig-sensor-rotation-angles``, to be able to produce
    a rig to desired specifications (:numref:`sat_sim_rig_adjust`).
  * Can apply a periodic or random perturbation to given cameras
    (:numref:`sat_sim_perturb`).

  * Added the option ``--blur-sigma``, to blur the simulated images. This can
    help simulate the effect of degraded images due to fog, motion, etc.
  
parallel_stereo (:numref:`parallel_stereo`):
  * Added an example of processing Umbra SAR images (:numref:`umbra_sar`).
  * Added an example of refining intrinsics and stereo with Chang'e 3 images
    (:numref:`change3`).
  * The initial low-resolution disparity from a DEM works with mapprojected
    images (:numref:`d_sub_dem`).
  * Added a discussion of various ways ASP can make use of existing terrain data
    (:numref:`existing_terrain`).
  * If the number of matches from disparity is much less than requested, try to
    find more matches. This usually brings their number in the ballpark.
  * The option ``--num-matches-from-disparity`` was made equivalent to   
    ``--num-matches-from-disp-triplets``, and the triplet logic now works 
    with mapprojected images (:numref:`dense_ip`).
  * It is possible to mapproject either with ``dg`` or ``rpc`` cameras
    when using mapprojected images in stereo with DigitalGlobe / Maxar
    cameras (:numref:`dg-mapproj`).
  * Enable stereo with vendor-supplied images that have been mapprojected onto
    surfaces of constant height above a datum (:numref:`mapproj_ortho`).
  * Added the option ``--band`` to process a given band (channel) from
    multispectral images (:numref:`stereodefault`).
  * With alignment methods ``none`` and ``epipolar``, the option
    ``--corr-search`` will work even when interest point matching fails
    (:numref:`corr_section`).    
  * Skip tiles for which there is no valid low-resolution disparity.
  * Throw an error if the left and right mapprojected images have different
    resolutions, as this can lead to incorrect results.
  * Print a warning in ``stereo_pprc`` and ``stereo_tri`` if the stereo
    convergence angle is too small.
  * Added the options ``--enable-atmospheric-refraction-correction`` and
    ``--enable-velocity-aberration-correction`` for Pleiades linescan cameras
    (these are enabled by default for WorldView cameras only). It is not clear
    if these corrections improve or not Pleiades accuracy.

sfm (:numref:`sfm`):
  * Added an example for processing data acquired with an UAS, with known
    metadata (:numref:`sfm_uas`).
    
sfs (:numref:`sfs`):
  * Added an SfS example for Earth (:numref:`sfs_earth`).
  * Added a CTX Mars example (:numref:`sfs_ctx`).
  * Added the program ``image_subset`` for selecting a subset of images that
    have almost the same coverage as the full input set
    (:numref:`image_subset`).
  * Added the option ``--sun-angles`` to specify the Sun azimuth and elevation
    angles.
  * Bugfix in modeling atmospheric haze.
  * Removed the ability to work on multiple clips at once, as it was not used.
  * Can ingest a provided albedo map (of same size as the input DEM). Option:
    ``--input-albedo``.
  * Removed the RPC approximation logic. Use instead the option
    ``--use-approx-camera-models`` with ISIS cameras.
  * Removed the option ``--float-cameras``. It is more reliable to optimize
    the cameras beforehand, in bundle adjustment. 
  * Removed obsolete options ``--float-dem-at-boundary``,
    ``--float-sun-position``, ``--coarse-levels``.
  * Have option ``--crop-input-images`` be always on. 

pc_align (:numref:`pc_align`):
  * Added the Nuth and Kaab algorithm (:numref:`nuth`).
  * Added an example of how to use dense image correlation for alignment
    (:numref:`pc_corr`).
  * Speed up the computation of shared bounding box and loading of source
    points.

cam2rpc (:numref:`cam2rpc`):
  * When a DEM is passed in, sample not just the DEM surface but its bounding
    box, to create a more robust RPC model.
  * The produced RPC file has been streamlined to a minimum of metadata.

point2las (:numref:`point2las`):
  * Replaced the option ``--triangulation-error-factor`` for saving the triangulation
    error as a scaled int with the option ``--save-triangulation-error``, that
    saves it in double precision without scaling.
  * Added the options ``--save-intensity-from-image`` and ``--save-stddev``.

point2dem (:numref:`point2dem`):
  * The default projection for WGS84 is now UTM / polar stereographic. 
    For other datums it is local stereographic (:numref:`point2dem_proj`).
  * Adjust the region passed in via the option ``--t_projwin`` so that, as
    usual, the DEM grid coordinates are integer multiples of the grid size.
  * Handle robustly invalid input points.
  * Remove old options ``--use-surface-sampling`` and ``--fsaa``.
  * Bugfix for slow performance with dynamic CRS.
  * Changed the default output nodata-value to -1e+6, as the smallest float
    may not be displayed accurately by some software.
  
gcp_gen (:numref:`gcp_gen`):
  * Make the interest point matching work better by invoking the full 
    machinery and options from ``bundle_adjust``.

image_align (:numref:`image_align`):
  * Let the default alignment method be ``rigid`` rather than ``translation``.

cam_gen (:numref:`cam_gen`):
  * Added the option ``--camera-center``.
  * Can export an RPC camera model to .xml format (:numref:`cam_gen_rpc`).
   
dem_mosaic (:numref:`dem_mosaic`):
  * Added the option ``--weight-list`` for blending DEMs given external weights
    (:numref:`dem_mosaic_external_weights`).
  * Renamed the option ``--dem-list-file`` to ``--dem-list``. The old option
    is kept for backward compatibility.
  * Can handle DEMs with NaN values.

dem_geoid (:numref:`dem_geoid`):
  * Accept a custom geoid correction via ``--geoid-path``. Added support for a
    Moon geoid.

orbit_plot (:numref:`orbit_plot`):
  * Added the options ``--use-rmse``, ``--output-file``.

isis (:numref:`planetary_images`):
  * Upgraded to ISIS 8.3.0.

misc:
  * The logic for triangulation with RPC cameras changed (:numref:`rpc_tri`).
  * In ``bundle_adjust`` and ``jitter_solve``, save the lists of images and
    optimized camera file names (or adjustments). Can be passed in back to
    any of these tools (:numref:`ba_out_cams`).
  * The option ``--flann-method`` in ``bundle_adjust`` and ``stereo`` defaults to
    using the slower but deterministic ``kmeans`` method for a smaller set of
    interest points, and to ``kdtree`` otherwise (:numref:`stereodefault-pprc`).
  * When creating dense interest point matches from disparity and mapprojected
    images, the match file reflects the name of the original unprojected images
    (:numref:`dense_ip`).
  * Bugfix for a crash with the ``asp_sgm`` and ``asp_mgm`` algorithms when the 
    disparity search range is large.
  * Print the stereo convergence angle in ``stereo_pprc`` with mapprojected
    images and with epipolar alignment. These are the remaining cases that were
    not handled before.
  * The ``mapproject`` and ``parallel_sfs`` programs will not fail if the work
    directory has spaces (this fix is a workaround, the bug is in GNU Parallel).
  * Renamed ``--csv-proj4`` to ``--csv-srs``. This accepts any GDAL WKT,
    GeoJSON, or PROJ string. The previous option is still accepted for backward
    compatibility.
  * Support images with up to 12 bands (channels), up from 6.
  * Support files with the .nitf extension.
  * Can handle no-data values larger than valid pixel values.
  * Wiped extremely old and unused SPICE logic.
  * Wiped the unused old option ``--mask-flatfield``. Can use with stereo
    the option ``--nodata-value`` to mask values no more than this value.
  * The ``geodiff`` program output image is with float pixels, rather than
    in double precision.
  * Have the OpenCV interest point detectors respect the ``--threads`` option.
  * Have ``bundle_adjust`` and ``parallel_stereo`` use same
    ``--ip-inlier-factor`` value by default.
  * Bugfix for loading camera adjustments when mapprojected images are passed 
    in, rather than the raw ones.
  * Can read Airbus Pleiades RPC XML files that have both a "global" and a
    "partial" camera model. The global one will be used.
  * Dependence on package ``htdp`` removed. This was needed for
    ``datum_convert``.

RELEASE 3.4.0, June 19, 2024
----------------------------

*This release is is available only as binaries, and not as a conda package*
(:numref:`conda_intro`).

DOI: `10.5281/zenodo.12176190 <https://zenodo.org/records/12176190>`_

Stable release doc: https://stereopipeline.readthedocs.io/en/stable/index.html

New tools:
  * Added ``orbit_plot.py`` (:numref:`orbit_plot`), a tool for plotting
    camera orientations along an orbit (contributed by Shashank Bhushan).
  * Added ``gcp_gen`` (:numref:`gcp_gen`), a program for generating ground
    control points (GCP) based on ortho images. Helps create camera models from
    scratch.  
  * Added ``dem2gcp`` (:numref:`dem2gcp`), a tool that can greatly help solve
    for lens distortion that manifests itself as large horizontal warping in the
    DEM. 

New camera support:
  * Added the ability to use the CSM camera model with ASTER images
    (:numref:`aster_csm`).

New external library support:
  * Migrated to PDAL 2.6.0 from libLAS for LAS input/output (in ``pointlas``,
    ``point2dem``, and ``pc_align``), as libLAS is no longer developed.

WorldView (DigitalGlobe) cameras (:numref:`dg_tutorial`):
  * The WorldView linescan model got moved to a CSM implementation. The
    transitional option ``--dg-use-csm`` was removed. The new implementation is
    about 5x faster for ground-to-image projections.
  * Re-enabled correcting velocity aberration and atmospheric refraction.  
    These corrections are now implemented in the CSM camera model, and, unlike
    before, play nicely with bundle adjustment (:numref:`dg_csm`).
  * The options ``--enable-correct-velocity-aberration`` and
    ``--enable-correct-atmospheric-refraction`` got removed.
  * Non-DG cameras do not use these corrections, as a case for that has not been
    made.

jitter_solve (:numref:`jitter_solve`):
  * Added an example for ASTER cameras (:numref:`jitter_aster`).
  * Added an example with 27 CTX images (:numref:`jitter_multiple_images`).  
  * Added the option ``--weight-image``, to weigh observations based on
    geographic location of triangulated points (:numref:`limit_ip`).
  * Can handle several sensors with very similar positions and orientations
    (:numref:`jitter_rig`).
  * Support reading the ISIS ``jigsaw`` binary control network
    format (:numref:`jitter_ip`).
  * Can read and write CSM model state embedded in ISIS .cub files   
    (:numref:`embedded_csm`).
  * Replaced the option ``--translation-weight`` with
    ``--camera-position-weight``, which is off by default, as it may affect the
    convergence. The new option adapts appropriately to the number of interest
    points and the ground sample distance (:numref:`jitter_camera`).
  * The ``--tri-weight`` constraint is now the default, with a positive value of
    0.1. This is adjusted for GSD (:numref:`jitter_tri_constraint`). 
  * Added report files having the change in camera positions
    (:numref:`jitter_cam_offsets`), triangulated points
    (:numref:`jitter_tri_offsets`), and stats of pixel reprojection errors per
    camera (:numref:`jitter_errors_per_camera`).
  * Replaced the option ``--heights-from-dem-weight`` with
    ``--heights-from-dem-uncertainty`` (1 sigma, in meters). This is more
    physically meaningful (as a rule of thumb, use the inverse of what was
    previously the weight value).
  * Integrated the logic behind ``--reference-dem`` into ``--heights-from-dem``,
    with an approach that combines the strength of both. Removed
    ``--reference-dem``.
  * Can use anchor points with frame cameras.
  * Added ``--num-anchor-points-per-tile``. This helps when different
    images have different sizes but want to ensure the same point density.
  * Added the option ``--anchor-weight-image`` that is used to limit
    where anchor points are placed.
  * The roll and yaw constraints no longer assume linescan camera positions and
    orientations are one-to-one.
  * Order of images in each interest point match file need not be the same
    as for input images.  

bundle_adjust (:numref:`bundle_adjust`):
  * Added the ability to refine the camera intrinsics for several groups of
    cameras, with each group sharing intrinsics (:numref:`kaguya_ba`).
  * Can mix frame and linescan cameras, while controlling for each 
    group of cameras which intrinsics should be optimized
    (:numref:`ba_frame_linescan`).
  * Support reading and writing the ISIS ``jigsaw`` binary control network
    format (:numref:`jigsaw_cnet`).
  * Can read and write CSM model state embedded in ISIS .cub files   
    (:numref:`embedded_csm`).
  * Support reading and writing the NVM format for control networks
    (:numref:`ba_nvm`).
  * Added the option ``--camera-position-weight``, with a default value of 0.0.
    This is an internally adjustable constraint to keep the cameras from moving
    too much. It may prevent the reduction in reprojection error
    (:numref:`ba_cam_constraints`).
  * Remove the option ``--translation-weight``. The translation is now
    automatically controlled by default by the camera position weight.
  * Added the option ``--camera-position-uncertainty`` to set hard constraints
    on the horizontal and vertical uncertainty for each camera
    (:numref:`ba_cam_constraints`).
  * Added report files having the change in camera positions
    (:numref:`ba_camera_offsets`) and triangulated points
    (:numref:`ba_tri_offsets`).
  * The option ``--tri-weight`` is now set by default to 0.1, and adjusted for
    GSD. The option ``--camera-weight`` is by default 0.0. This  work better
    than before at preventing the cameras from moving when optimizing them.
  * Replaced the option ``--heights-from-dem-weight`` with
    ``--heights-from-dem-uncertainty`` (1 sigma, in meters). This is more physically
    meaningful (as a rule of thumb, use the inverse of what was previously the
    weight value).
  * Integrated the logic behind ``--reference-dem`` into ``--heights-from-dem``,
    with an approach that combines the strength of both. Removed 
    ``--reference-dem``.
  * Added the option ``--propagate-errors`` to propagate the uncertainties from
    input cameras to triangulated points (:numref:`ba_error_propagation`).  
  * Added the option ``--weight-image``, to weigh observations based on
    geographic location of triangulated points. (:numref:`limit_ip`). 
  * For ASTER cameras, use the RPC model to find interest points. This does
    not affect the final results but is much faster.
  * When optimizing intrinsics, cameras that do not share distortion can
    have different distortion types and sizes. (:numref:`limit_ip`).
  * Each image passed to ``--mapprojected-data`` reads from its geoheader
    the camera and adjustment prefix for undoing the mapprojection.
  * Fixed a bug when both ``--initial-transform`` and
    ``--input-adjustments-prefix`` are used.
  * Can use the image names in ``--camera-list`` when images contain the camera
    models.
  * The pixel reprojection errors are adjusted correctly for pixel sigma in
    the report files (:numref:`ba_errors_per_camera`, :numref:`ba_err_per_point`).
  * The default outlier removal parameters are more generous, to avoid removing
    valid interest point matches when the input images have distortion (option
    ``--remove-outliers-params``). 
  * The combination of options ``--mapprojected-data`` and
    ``--auto-overlap-params`` will restrict the interest point matching to the
    region of overlap (expanded by the percentage in the latter option). This
    can result in great efficiency gains for large images.
  * Made the Tsai lens distortion agree precisely with OpenCV's implementation
    (:numref:`pinholemodels`). There was a small numerical problem and the K3
    coefficient was not part of the distortion model.
  * Replaced the Tsai lens undistortion implementation, for a 10x speedup.  
  * Added the OpenCV fisheye lens distortion model and also the FOV model
    (:numref:`pinholemodels`). These are for wide-angle lenses. 
  * Bugfix: points for which initial triangulation failed are flagged as
    outliers right away. See ``--forced-triangulation-distance`` for
    fine-grained control.
  * Order of images in each previously created interest point match file need
    not be the same as for input images.
  * RPC lens distortion is now applied to pixels that are normalized by focal
    length, in addition to being offset by the principal point. This is
    consistent with the radial-tangential and fisheye models, and produces a
    more accurate fit to other models. *Previously created models are now
    invalid*.
  * RPC undistortion is now done with a solver rather than using separate
    undistortion coefficients. This much more accurate but slower
    (:numref:`pinholemodels`).
  * Added an example of using RPC distortion for KH-7 cameras, for which 
    an exact model is not available (:numref:`kh7_fig`).
  * Ensure that outlier filtering with ``--min-triangulation-angle`` is done
    after each pass with refined cameras and for all ways of reading a control
    network.
  * Load and save the camera models in parallel, for speed (except for ISIS).
  * Bugfix: if some intrinsics are shared, sync them up before optimization.
  
parallel_stereo (:numref:`parallel_stereo`):
  * Added Kaguya processing example (:numref:`kaguya_tc`).
  * When a run finished successfully, combine the data from subdirectories and
    delete these. See ``--keep-only`` for more options.
  * Made the tiles for the ``asp_mgm`` / ``asp_sgm`` algorithms bigger, with
    smaller padding, which should be about 2x faster (:numref:`ps_tiling`).
  * Added an illustration of several stereo algorithms (:numref:`stereo_alg_fig`).  
  * Fixed a failure when processing images that have very large blocks (on the
    order of several tens of thousands of pixels along some dimension, as shown
    by ``gdalinfo``). A warning, progress bar, and timing info is displayed.
  * For the ``asp_sgm`` and ``asp_mgm`` algorithms allow ``cost-mode`` to
    have the value 3 or 4 only, as other values produce bad results. 
  * Fix a failure when the working directory has a space in its name.
  * Bugfix for memory usage with very large images.

point2dem (:numref:`point2dem`):
  * Added the option ``--auto-proj-center``, to automatically compute the
    projection center for stereographic and other projections
    (:numref:`point2dem_proj`).
  * When the lon-lat projection is used, the output DEM longitude range
    is always in [-180, 180], unless using [0, 360] results in a smaller range
    (such as when crossing the 180 degree meridian).
  * Added the option ``--scalar-error`` to find the norm of the triangulated
    error vector (if applicable).
  * Can read a ground-level point cloud stored as a tif file with 3 bands,
    representing the x, y, and z coordinates of the points, with z being
    vertical (option ``--input-is-projected``).
  * Bugfix for when all heights are equal. A valid DEM is produced.  
  * Do not assume the datum is WGS84 by default, as this can result in
    incorrect DEMs. The datum, projection, or semi-axes must be set    
    (or read from the input PC/LAS file).
    
gdal (:numref:`gdal_tools`):
   * Full support for WKT and GeoJSON for the projection string (option
     ``--t_srs``) in ``point2dem``, ``point2las``, ``mapproject``,
     ``dem_mosaic``, ``cam2rpc``. Can still use PROJ.4 strings. 
   * Georeferenced images with different datums cannot be used together. Use
     ``gdalwarp`` to convert them to a common datum.
   * Upgraded to GDAL 3.8.0 and PROJ 9.3.0.
   
csm (:numref:`csm`):
   * Upgraded to USGSCSM 2.0.1.
   * Fixed several problems in generation of CSM cameras for MSL Curiosity Nav
     and Mast images. Much large-scale testing was performed. Updated the
     example showing how to create stereo from either Nav or Mast stereo pairs
     (:numref:`csm_msl`).
   * A multi-Martian-day example for MSL added (:numref:`csm_msl_multiday`).
   * Added support for the radial and tangential distortion model
     with 3 radial distortion parameters and 2 tangential ones. Tested
     that it agrees with the OpenCV implementation.
   * Fixed a small bug in radial distortion implementation.
      
stereo_gui (:numref:`stereo_gui`):
  * Can show scattered data with a colorbar and axes 
    (:numref:`scattered_points_colorbar`).
  * Renamed ``--colorize-image`` to ``--colorbar``.
  * Right-click on a colorized image to set the range of intensities to
    colorize.
  * Can view ISIS control network files (:numref:`stereo_gui_isis_cnet`).
  * Auto-guess and load ``pc_align`` error files (:numref:`pc_align_error`).
  * When loading an .nvm file with features that are not shifted relative
    to the optical center, must specify ``--no-shift``. This avoids confusion
    as to whether a shift is present or not (:numref:`stereo_gui_nvm`).

colormap (:numref:`colormap`):
  * Added the option ``--hillshade`` to create a hillshaded colormap.
   
image_calc (:numref:`image_calc`):
  * When adding new keywords to metadata geoheader, do not erase the existing
    ones (if a keyword already exists, its value will be modified).
  * Added the ability to create a random image.

pc_align (:numref:`pc_align`):
  * Add the option ``--skip-shared-box-estimation``.
   
historical_helper.py (:numref:`historical_helper`):
  * Added the ability to set a custom path to the needed ``convert``
    executable and described how that tool can be installed.

sfs (:numref:`sfs`):
  * Added two examples for Kaguya TC, for single and multiple illumination
    conditions (:numref:`sfs_kaguya`).
  * Added the option ``--albedo-robust-threshold``.

isis (:numref:`moc_tutorial`):
  * The ISIS libraries are compiled from source, and reflect the code after
    the ISIS 8.0.3 release (:numref:`conda_intro`). 
  * Made the operation of projecting into an ISIS linescan camera 2.2-2.6 times
    faster by using the secant method to find the best sensor line.
  * Expanded the ``jigsaw`` documentation (:numref:`jigsaw`). This is the 
    ISIS bundle adjustment tool. 

cam_gen (:numref:`cam_gen`):
   * Can fit a CSM frame camera to a given input camera, including distortion
     (:numref:`cam_gen_frame`).
   * Can export linescan cameras to CSM format (:numref:`cam_gen_linescan`).
   * Can create cameras given longitude, latitude, height above datum, and roll,
     pitch, yaw angles (:numref:`cam_gen_extrinsics`).

rig_calibrator (:numref:`rig_calibrator`):
   * Can export the interest point matches, cameras, and the OpenCV lens
     distortion model for use with ``bundle_adjust`` (:numref:`rc_bundle_adjust`).
   * Added documentation for how to register the produced cameras to the ground
     for a planet (:numref:`msl_registration`).
   * Can fix the translation and/or rotation component of a rig configuration.  
   * Can constrain camera positions with ``--camera_position_weight``.
   * Added two more naming conventions, to help process existing data
     out-of-the-box. Also for ``theia_sfm`` and ``sfm_merge``.
     :numref:`rig_data_conv`. 
   * Thoroughly validated with an orbital rig (in addition to indoor rigs).   
     
lronac2mosaic.py (:numref:`lronac2mosaic`):
  * Run ``spiceinit`` before calling ``lronaccal``, and re-enable all
    options for the latter command, which were disabled due to a bug
    in ISIS that was fixed in version 7.2.
  * Invoke ``spiceinit`` with ``spksmithed=true``. 
  * Add the option ``--spiceinit-options``.

camera_solve (:numref:`camera_solve`):
  * Switched to cascade matching from brute force matching, which is much faster.
  * Always reuse the Theia SfM matches.
 
dem_mosaic (:numref:`dem_mosaic`):
  * Bugfix for option ``--use-centerline-weights``. 
    
misc:
  * Made all tools that spawn processes in parallel use the option
    ``--parallel-options``, with default ``--sshdelay 0.2``, to avoid
    failure on certain architectures.
  * For ASTER (:numref:`aster`), the model loaded by default is now linescan
    rather than RPC.
  * Fixed a bug in outlier filtering when the interest points are very noisy.   
  * Fixed a couple of runtime errors when using conda packages on OSX.
  * Eliminated a procedure for cleaning the name of an input path that was
    replacing two slashes with one slash, resulting in inconsistencies.
  * Robustly handle 360 degree longitude offsets without classifying
    georeferenced images into [-180, 180] or [0, 360] types.  
  * Fix an error in conversion between projections for non-Earth images.
  * The North-East-Down coordinate system assumes an ellipsoid, not a sphere,
    and takes into account the point elevation. This fixes some small
    inaccuracies in error propagation and reporting in ``pc_align``.
  * The OSX build now gets created and tested via GitHub Actions.
  * Very old jitter adjustment logic was removed. The ``jitter_solve``
    tool must be used instead.
  * For stereo, increased ``--ip-num-ransac-iterations`` from 100 to 1000,
    as for ``bundle_adjust``.  This can make a difference for noisy data.
  * Do not keep auxiliary files with temporary names in the work directory for
    ``parallel_stereo`` and ``parallel_bundle_adjust``. Use run's output
    directory and proper names.
  * Ensure any sequence of quaternions in a CSM linescan model is normalized
    and there is no sign flip along the sequence. Such a flip was resulting
    in incorrectly interpolated camera orientations.  
  * Auto-guess the planet for Pinhole cameras (For Earth, Moon, Mars).   
  * Documented the program ``view_reconstruction``
    (:numref:`view_reconstruction`), with a figure.  
  * Switched by default to a slower but deterministic method for matching
    interest points in ``bundle_adjust`` and ``parallel_stereo``. Normally this
    is not a bottleneck. See ``--flann-method`` in :numref:`stereodefault-pprc`.
  * Made RANSAC multi-threaded. This speeds up interest point matching.
  * Added a sanity check: If the user sets ``--t_srs`` to any tool, it 
    must not be empty.
  * Added sanity checks to ensure no mix-up of datums from different planets in
    different inputs for the stereo tools, ``bundle_adjust``, ``jitter_solve``,
    ``mapproject``, ``cam_test``, and ``cam_gen``.
  * Upgraded to Boost 1.82.0.
  
RELEASE 3.3.0, August 16, 2023
------------------------------

DOI: `10.5281/zenodo.8270270 <https://zenodo.org/record/8270270>`_

Stable release doc: https://stereopipeline.readthedocs.io/en/stable/index.html

New tools:
  * Added ``sfm_merge`` (:numref:`sfm_merge`), a tool to merge several
    SfM reconstructions that may or may not have shared images.
  * Added ``sfm_submap`` (:numref:`sfm_submap`), a tool to extract  
    a submap from a Structure-from-Motion map in .nvm format, 
    as produced by ``theia_sfm`` (:numref:`theia_sfm`) or refined
    with ``rig_calibrator`` (:numref:`rig_calibrator`).
  * Added a couple of small Python scripts for handling ROS bags
    (:numref:`ros_tools`). No ROS binaries are shipped.
  * Added support for the Pleiades NEO exact linescan model
    (:numref:`pleiades_neo`).
  * Added ``sat_sim`` (:numref:`sat_sim`), a tool to create simulated
    satellite images camera models for pinhole or linescan sensors.
  * Added ``sfm_view`` (:numref:`sfm_view`), a tool for viewing orbital 
    Pinhole camera models. It is a modification of the ``umve`` program.
   
csm (:numref:`csm`):
  * Added initial support for using CSM camera models with MSL Curiosity
    (:numref:`csm_msl`).

parallel_stereo (:numref:`parallel_stereo`):
  * Can propagate horizontal ground plane standard deviations (stddev)
    specified for each camera through triangulation, obtaining the
    horizontal and vertical stddev for each triangulated point. 
    For DigitalGlobe RPC and Pleiades linescan cameras the input
    horizontal stddev can be read from camera files. A formula to go
    from known CE90 to input horizontal stddev is provided
    (:numref:`error_propagation`).
  * Can propagate the covariances of satellite positions and
    quaternions to the triangulated point cloud for Maxar
    (DigitalGlobe) linescan cameras (:numref:`error_propagation`).
  * Documented the pre-processing options ``--stddev-mask-kernel``
    and ``--stddev-mask-thresh``
    (:numref:`stereodefault-pprc`). Fixed a bug in writing
    out debug images for this option.
  * The cameras files used in mapprojection can be switched to other
    ones for the purpose of doing stereo or reusing a stereo run
    (:numref:`mapproj_reuse`).
  * Added the option ``--matches-per-tile``, to attempt to guarantee
    that each 1024 x 1024 tile has about this many number of matches.
  * Bugfix for stereo with mapprojected Pleiades images. If the
    mapprojection is done with the exact (non-RPC) cameras, stereo
    must load the exact cameras when undoing the mapprojection.

bundle_adjust (:numref:`bundle_adjust`):
  * Validated that given about a thousand input images acquired with three views
    and much overlap, one can have very tight control of the cameras in bundle
    adjustment, while improving the self-consistency of the camera configuration
    (:numref:`skysat_stereo`).
  * Validated the option ``--reference-dem`` for bundle adjustment. This works
    even when rays obtained during triangulation are parallel.
  * Added the option ``--matches-per-tile``, to attempt to guarantee that each
    1024 x 1024 tile has about this many number of matches (example in
    :numref:`ba_examples`).
  * Bugfix for slow performance and memory usage for a large number of images.

jitter_solve (:numref:`jitter_solve`):
  * Added the options ``--roll-weight`` and ``--yaw-weight`` to control the
    amount of change in these angles relative to the along-track direction. To
    be used with synthetic cameras created with ``sat_sim`` (:numref:`sat_sim`). 
  * Added a section discussing solving for jitter with synthetic camera models
    (:numref:`jitter_sat_sim`).
  * The solver can mix and match linescan and pinhole (frame) camera images if
    the inputs are all in the CSM format (:numref:`jitter_linescan_frame_cam`).
  * Added a section on how to prepare interest point matches
    (:numref:`jitter_ip`).
  * Validated the option ``--reference-dem`` for bundle adjustment. This works
    even when rays obtained during triangulation are parallel.
  * Bugfix for reverse scan direction.
  * Added an example for Pleiades cameras (:numref:`jitter_pleiades`),
    comparing two ways of setting ground constraints.
    
sfs (:numref:`sfs`): 
  * Created an SfS DEM of size 14336 x 11008 pixels, at 1 m pixel with
    420 LRO NAC images with various illuminations and orientations.
    Reliably bundle-adjusted 814 LRO NAC images in which the shadows
    were observed to make a full 360 degree loop, with a total of 614k
    triangulated points. Updated the documentation reflecting
    latest best practices (:numref:`sfs-lola`).
  * Create more detail in the reconstructed DEM in borderline lit
    regions. Option: ``--allow-borderline-data``
    (:numref:`sfs_borderline`).
  * Added the options ``--image-list`` and ``--camera-list`` for when
    the number of images becomes too large to set on the command line.

rig_calibrator (:numref:`rig_calibrator`):
  * Added a detailed tutorial describing how this tool was used to
    create a full 360-degree textured mesh of the JEM ISS module
    (:numref:`sfm_iss`) using data acquired with two rigs (6 sensors).
  * Added an example for the MSL Curiosity rover (:numref:`rig_msl`).
  * Allow multiple rigs to be jointly optimized (the rig constraint
    applies within individual rigs and not between them).
  * Added the option ``--extra_list`` to insert additional images 
    close in time to some of the images already on the rig (for
    the same or different rig sensor). Helps easily grow a map and
    complete a rig.
  * Added the option of keeping a subset of the camera poses fixed (for
    when those have been validated in a smaller map).
  * Images for any of the rig sensors (not just the reference one) can
    be used in registration (but all must be for same sensor).
  * Added the ``--save_pinhole_cameras`` option to save the optimized
    cameras in ASP's Pinhole format (with no distortion for now).
  * Absorb ``--rig_transforms_to_float`` into ``--camera_poses_to_float``. 
  * Save alongside an .nvm file a separate file having the values of
    optical center point that are subtracted from each interest point
    (for plotting in ``stereo_gui``).
  * Merge the interest point tracks created as part of rig calibration
    with the matches read from disk beforehand.
  * Fix for too many valid interest point matches being filtered out.

voxblox_mesh (:numref:`voxblox_mesh`):
  * Added median filtering of input point clouds (option
    ``--median_filter``).
  * Added weighing of depth points given their distance from the
    sensor (option ``--distance_weight``).

multi_stereo (:numref:`multi_stereo`):
  * Left and right images can be acquired with different sensors.
  * Use ``--left`` and ``--right`` to specify which stereo pairs to
    run.

texrecon (:numref:`texrecon`):
  * Can create a texture with images from multiple sensors.
 
point2dem (:numref:`point2dem`): 
  * Added the option ``--propagate-errors`` to grid the stddev values
    computed during stereo triangulation.
  * Added the option ``--input-is-projected`` to specify that the input
    coordinates are already in the projected coordinate system.

stereo_gui (:numref:`stereo_gui`): 
  * Can read, write, edit, and overlay on top of images polygons in
    plain text format in addition to the .shp format. Per-polygon
    colors are supported.
  * Can read nvm files whose features are shifted relative to the 
    optical center, if an ``.nvm`` file is accompanied by an
    ``_offsets.txt`` file.
  * Added the option ``--preview`` to load one image at a time, 
    and quickly cycle through them with the 'n' and 'p' keys.
  * Added the option ``--view-several-side-by-side``
    to view several images side-by-side with a dialog to choose which
    images to show (also accessible from the View menu).
  * Added the option ``--font-size``, with the default of 9. 
  * Added the option ``--lowest-resolution-subimage-num-pixels`` to
    control the behavior of the pyramid of subimages.
  * Noticeable speedup in loading images.
  * Bug fix in loading .nvm files (an SfM format).

image_align (:numref:`image_align`):
  * Can find the 3D alignment around planet center that transforms the
    second georeferenced image to the first one. This transform can be
    used to apply the alignment to cameras and point clouds
    (:numref:`image_align_ecef_trans`).

dem_mosaic (:numref:`dem_mosaic`):
  * Added the option ``--fill-search-radius`` to fill nodata pixels in 
    a DEM with nearby valid values. This is different from
    ``--hole-fill-length``. See an example in :numref:`dem_mosaic_examples`.

wv_correct (:numref:`wv_correct`):
  * Maxar (DigitalGlobe) WorldView-2 images with a processing (generation) date
    (not acquisition date), of May 26, 2022 or newer have much-reduced CCD
    artifacts, and for those this tool will in fact make the solution worse, not
    better. This does not apply to WorldView-1, 3, or GeoEye-1.
  * ASP builds after 2023-06-21 (so, version later than 3.2.0), will detect the
    above-mentioned scenario, and will not apply any correction in that case (a
    copy of the original image will be written instead and a warning will be
    printed). This applies to both PAN and multi-spectral images.

corr_eval (:numref:`corr_eval`):
  * Remove an excessive check. The refined/filtered disparity can be such 
    that left image pixel plus disparity may be outside the right image.
    Don't fail in that case, but just skip the pixel, resulting in empty 
    correlation for that pixel.

cam_test (:numref:`cam_test`):
  * Added the option ``--datum``. Useful for Pinhole cameras as those don't 
    know their datum. 
  * Added a warning if the camera center is below the datum. 

misc: 
  * Upgraded to ISIS 8.0.0 and USGSCSM 1.7.
  * Throw an error for WorldView products that are not Stereo1B or Basic1B.
    That because ASP does not support orthorectified Maxar products.
  * Changed the "pretend" height of the RPC cameras from 10 km 
    above ground to around 100 km. RPC camera models do not store this
    number and it does not make a difference normally, as only ray
    directions to the ground matter. Yet, .adjust
    files created with an earlier version of ASP for RPC cameras
    should be re-created as those use the camera center.
  * The latest version of the Xerces-C XML parser became 10 times
    slower than before, which may affect the speed of
    processing for XML-based camera models.
  * Added back the tool ``view_reconstruction``, for examining
    Theia's SfM solution (:numref:`sfm`).
  * The ``theia_sfm`` tool can write the optical offsets for a given
    nvm file which can be used in plotting such files in ``stereo_gui``. 
  * Added to ``hiedr2mosaic.py`` (:numref:`hiedr2mosaic`) the option
    ``--web`` to invoke ``spiceinit`` with ``web=True``. Contributed
    by Giacomo Nodjoumi.
  * Bugfix for reading .jp2 files. Needed to ship the JPEG2000 driver
    and set GDAL_DRIVER_PATH.
  * Fixed a failure in ``mapproject`` with a small DEM.
  * Bugfix for exporting the TheiaSfM matches in ``camera_solve``.
  * The documentation of the examples chapter was broken up into
    individual pages (:numref:`examples`). 

RELEASE 3.2.0, December 30, 2022
--------------------------------
DOI: `10.5281/zenodo.7497499 <https://doi.org/10.5281/zenodo.7497499>`_

Added functionality for creation of large-scale meshes and fused
textures for small planetary bodies and indoor environments. Added
logic for rig calibration. See individual tools below.

New tools:
  * ``rig_calibrator``: Calibrates a rig of N image and/or
    depth+image cameras. Can also co-register and refine
    intrinsics of camera images acquired with N sensors with no rig
    constraint (:numref:`rig_calibrator`).
  * ``multi_stereo``: Runs multiple stereo pairs and produces
    a fused mesh. Uses ``parallel_stereo``, ``pc_filter``, and 
    ``voxblox_mesh`` (:numref:`multi_stereo`).
  * ``voxblox_mesh``: Fuses point clouds into a seamless oriented
    mesh, with each input point given a weight according to its
    reliability. Based on the third-party VoxBlox software
    (:numref:`voxblox_mesh`).
  * ``texrecon``: Creates seamless textured meshes. Based on
    the third-party MVS-Texturing project (:numref:`texrecon`).
  * ``pc_filter``: Filters outliers in point clouds created with
    pinhole cameras and weighs inliers based on many criteria
    (:numref:`pc_filter`).
  * Added CGAL-based tools for mesh smoothing, hole-filling, remeshing,
    and removal of small connected components (:numref:`cgal_tools`).
  * ``jitter_solve``: A tool for solving for jitter in CSM camera 
    models (:numref:`jitter_solve`). It gives promising results 
    for CTX, Pleiades, and DigitalGlobe data. Examples are provided.

Removed tools:
  * ``datum_convert``: This was an attempt at having a tool applying
    a transform between datums. It is suggested to use GDAL/PROJ instead.
    Note that a datum transform may require fetching transformation grids,
    and without them PROJ will quietly return incorrect results. 

New sensors:
  * Support the Pleiades exact sensor (for 1A/1B). See :numref:`pleiades`.
    Implemented as a wrapper around the CSM linescan camera model.

parallel_stereo (:numref:`parallel_stereo`):
  * Added the options ``--match-files-prefix`` and
    ``--clean-match-files-prefix`` for reusing interest point matches
    from a previous ``bundle_adjust`` or ``parallel_stereo`` run. The
    "clean" interest point matches created by ``bundle_adjust`` may
    have fewer outliers than what stereo can create.
  * Added the option ``--keep-only`` to convert all VRT files to TIF
    (e.g., D.tif), then wipe all files and subdirectories except those
    specified by given suffixes.
  * Added the triangulation option ``--max-valid-triangulation-error``.
  * The option ``--prev-run-prefix`` can be used to start a run
    with bathymetry modeling at the triangulation stage while
    reusing the previous stages of a run without such modeling
    (the needed aligned bathy masks are created, if needed,
    at the triangulation stage, if not done, as usual, at the 
    preprocessing stage).
  * For SGM and MGM use by default 8 threads and number of processes
    equal to number of cores divided by number of threads. Less likely
    to run out of memory that way.
  * Added examples of using PBS and SLURM with ASP
    (:numref:`pbs_slurm`).
  * Added an example of processing SkySat Stereo data
    (:numref:`skysat_stereo`).
  * Documented better the option ``--num-matches-from-disp-triplets``
    for creating dense and uniformly distributed interest point
    matches. Useful for modeling lens distortion.

parallel_bundle_adjust (:numref:`parallel_bundle_adjust`):
  * Do not create subdirectories or symlinks, as that results in a
    massive number of small files. (Unless ``--save-vwip`` is used,
    see below.)
  * Do not save by default .vwip files as those take space and are
    only needed to find .match files. Use the new option
    ``--save-vwip`` to save them. Note that these depend on individual
    image pairs, so ``parallel_bundle_adjust`` saves them in
    subdirectories.

bundle_adjust (:numref:`bundle_adjust`):
  * Save the convergence angle percentiles for each pair of
    images having matches. Useful for understating the configuration
    of cameras.
  * Added the option ``--tri-weight`` (default is 0) to keep triangulated
    points close to their initial values. This looks more promising
    than other weighing used so far at preventing the cameras from
    moving when optimizing them. This assumes input cameras are
    not grossly inaccurate. This adds a robust cost function 
    with the threshold given by ``--tri-robust-threshold``.
  * Added the options ``--image-list``, ``--camera-list``, 
    ``--mapprojected-data-list``, for when the inputs are too many to
    specify on the command line.
  * Added the option ``--fixed-image-list`` to specify a file having a 
    list of image names whose cameras should be fixed during
    optimization.
  * Pinhole cameras are no longer automatically reinitialized or
    transformed based on GCP, but only refined given GCP. So, option
    ``--disable-pinhole-gcp-init`` is the default. Use one of the
    options ``--init-camera-using-gcp`` (:numref:`camera_solve_gcp`),
    ``--transform-cameras-with-shared-gcp``, 
    ``--transform-cameras-using-gcp`` (:numref:`sfm_world_coords`) for
    manipulating cameras using GCP.
  * Bugfix in initializing pinhole cameras based on GCP for off-nadir
    cameras. 
  * When doing multiple passes (which is the default) at each pass
    resume not only with clean matches but also with the cameras
    optimized so far, rather than going to the originals.
  * Can do multiple passes with ``--heights-from-dem``. One should
    be generous with outlier removal thresholds if not sure of 
    the input DEM accuracy (option ``--remove-outliers-params``).
  * Remove outliers based on spatial distribution of triangulated
    points.
  * Bugfix when the number of interest points is 4 million or more.
    The algorithm would just stall. It is now replaced by an OpenMVG
    algorithm.
  * Fold ``--remove-outliers-by-disparity-params`` into 
    ``--remove-outliers-params``.
  * Bugfix in ``residuals_stats.txt``; the mean was correct but the
    median was wrong.
  * Let the default ``--heights-from-dem-weight`` be 1.0, and the
    default ``--heights-from-dem-robust-threshold`` be 0.5. These
    normally need tuning.
  * Added the option ``--mapproj-dem``. If specified, evaluate 
    the disagreement of interest point matches after mapprojecting
    onto this DEM, per interest point match pair, per matching image
    pair, and per image. Useful at evaluating registration without
    mapprojecting the images (:numref:`ba_out_files`).
  * Added report files having the camera positions and orientations
    before and after optimization (for Pinhole cameras only,
    :numref:`ba_cam_pose`).
  * Added options ``--proj-win`` and ``--proj-str`` for restricting
    interest points to given area (useful when having many images
    with footprints beyond area of interest).
  * With ``--match-first-to-last``, write match files from earlier
    to later images, rather than vice-versa. This was a bug, as
    the matches were not being picked up correctly later.
  * For pinhole cameras, can read .adjust files via
    ``--input-adjustments-prefix``, then apply them to existing .tsai
    files via ``--inline-adjustments``. Until now one could do either
    one or the other. Also works with ``--initial-transform``.
  * Added a section describing how bundle adjustment is implemented
    (:numref:`how_ba_works`).

point2dem (:numref:`point2dem`):
  * Added the Tukey outlier removal method option applied to
    triangulation errors (error_thresh = 75th_pct + 1.5 * (75th_pct -
    25th_pct)). Also print out these percentages even for the regular
    outlier removal.

bathymetry (:numref:`shallow_water_bathy`):
  * Added ``scale_bathy_mask.py``, for creating a PAN-sized image
    or mask from an multispectral-sized image or mask, both for
    WorldView data.

mapproject (:numref:`mapproject`):
  * Exposed and documented the ``--query-projection`` option.
 
stereo_gui (:numref:`stereo_gui`):
  * Can plot, overlay on top of images, and colorize scattered points
    stored in a CSV file (:numref:`plot_csv`). Many colormap styles
    are supported. See :numref:`colormap` for the list.
  * Can show side-by-side colorized images with colorbars and coordinate
    axes (:numref:`colorize`).
  * Given a ``bundle_adjust`` output prefix, can select via checkboxes
    any two images to show side-by-side, and automatically load their
    match file or clean match file (options:
    ``--pairwise-matches`` and ``--pairwise-clean-matches``, also
    accessible from the top menu).
  * Visualize pairwise matches read from an nvm file, as created by
    ``rig_calibrator --save_nvm_no_shift``. 
  * Zoom to given proj win from the View menu. Useful for
    reproducibility. Also accessible with the command-line option
    ``--zoom-proj-win``.
  * Bug fix for slow overlaying of images with different datums.
  * When all images have a georeference, start in georeference mode.

corr_eval (:numref:`corr_eval`):
  * Bugfix for excessive memory usage with positive ``--prefilter-mode``.
  * Added a note saying that the user should ensure that this tool uses 
    the same ``--corr-kernel`` and ``--prefilter-mode`` as
    ``parallel_stereo``.
  * Added the option ``--sample-rate``.

cam_gen (:numref:`cam_gen`):
  * Can read Planet's pinhole.json files. Then no further changes
    are made to the produced pinhole camera. 
  * Fix a bug in output camera center determination, when an input
    camera is provided.
  * Bugfix in initializing pinhole cameras based on GCP for off-nadir
    cameras given image corners and no prior camera. 
  * Added the options ``--cam-height`` and ``--cam-weight`` to try
    to keep the camera at a given height above ground.
  * Added the option ``--cam-ctr-weight``, to help fix the camera
    center during refinement.
  * If ``--optical-center`` is not set for pinhole cameras, use the
    image center (half of image dimensions) times the pixel pitch.
    The optical bar camera always uses the image center.

pc_align (:numref:`pc_align`):
  * Fix a bug with loading very large DEMs; it was failing because of
    a 32-bit integer overflow.

colormap (:numref:`colormap`): 
  * Added six colormaps: ``black-body``, ``viridis``, ``plasma``,
    ``kindlmann``, ``rainbow``, ``turbo``. Sources: 
    http://www.kennethmoreland.com/color-advice/ and matplotlib.
 
misc:
  * Upgrade to C++-14, Python 3.9, latest libLAS, OpenCV 4, PCL 1.11,
    Boost 1.72, ISIS 7.1.0, GDAL 3.5, and PROJ 9.1.0. The latter has a
    whole new API, intended to handle properly transformations among
    datums.
  * The ``lronaccal`` tool in ISIS 7.1.0 appears buggy. Try using
    an earlier ISIS version if this is needed.
  * Replaced in some locations ASP's homegrown coordinate transformation
    logic with what is in PROJ.
  * Added the option of using the CSM camera with DigitalGlobe WorldView 
    images in bundle adjustment, stereo, and mapprojection (use with
    ``--t dg``). Option name is ``--dg-use-csm`` and must be set
    consistently for all tools. This speeds up ground-to-image
    computation by a factor of about 20 (which helps with
    mapprojection and bundle adjustment). The result of projecting
    into the camera changes by less than 0.015 pixels from before if
    using this option. That is due to the fact that different
    methods are used for position and orientation interpolation.
    The ``cam_test`` option ``--dg-vs-csm`` can be
    used for evaluating this discrepancy. Each of these methods is
    consistent with itself to within 2e-8 when it comes to projecting
    from camera to ground and back. 
  * Increased the cache size to 1 GB per process for each ASP tool. 
    Added the option ``--cache-size-mb``, to set this. Made the
    warning message refer to this option when the limit is
    hit. Documented this for all tools.
  * Using ``-t pinhole`` now invokes the same logic as ``-t
    nadirpinhole --no-datum``, which is same code path used by other
    sessions. This wipes an old alternative approach. Eliminated much
    other duplicated and mutated code for various sessions at the
    preprocessing stage.
  * Bugfix for D.tif VRTs as created by ``parallel_stereo``.
  * Allow whitespaces in stereo.default before option names. 
  * Fix a crash in ISIS for international users by setting for all ASP
    programs the environmental variables LC_ALL and LANG to en_US.UTF-8.
  * parallel_stereo will accept (but ignore) Unicode in stereo.default.
  * Eliminate internal fudging of ``--ip-uniqueness-threshold``,
    and make it equal to 0.8 for both ``stereo`` and
    ``bundle_adjust``. This was shown to increase the number of
    interest points in situations when not enough were found.
  * The ``historical_helper.py`` program expects a local installation
    of ImageMagick and the ``convert`` tool (available on most systems
    normally).
  
RELEASE 3.1.0, May 18, 2022
----------------------------
DOI: `10.5281/zenodo.6562267 <https://doi.org/10.5281/zenodo.6562267>`_

New camera additions:
  * Added support for the USGSCSM Frame, SAR, and PushFrame sensors
    (until now just the Linescan sensor was supported), together 
    with documentation and examples (for Dawn, MiniRF, and WAC,
    respectively).
  * Added support for ISIS SAR cameras, together with an example in
    the doc.
  * Added support for the PeruSat-1 linescan camera model (so far just
    the RPC model was supported for this satellite).

New tool additions:
  * Added the program ``corr_eval``, for evaluating the quality of
    produced correlation with several metrics. See also the new option
    ``--save-left-right-disparity-difference`` in ``parallel_stereo``.
  * Added the program ``otsu_threshold`` for computing an image
    threshold. It can be used for separating land from water (in
    WorldView multispectral NIR bands), and shadowed from lit areas
    (in Lunar images).
  * The program ``parallel_stereo`` can function as purely an image
    correlation tool, without assuming any camera information, via
    the option ``--correlator-mode``.
  * Added the program ``image_align``. Used to align two images or
    DEMs based on interest point matches or disparity, with given
    alignment transform type (translation, rigid, similarity, affine,
    and homography).

isis:
  * Using ISIS 6.0.0.

csm:
  * Save the camera state on multiple lines. On reading both the
    single-line and multiple-line formats are accepted.
  * Bundle adjustment, mapproject, and SfS with the CSM model can be
    7-15 times faster than done with the corresponding ISIS mode
    for linescan cameras (the latter as reimplemented in ASP itself). 
    It is strongly suggested to use CSM for large-scale processing.
  * Bugfix in CSM linescan implementation for some LRO NAC sensors.
    Also replaced the fixed-point method with the secant method in the 
    ground-to-image logic for CSM linescan cameras, which is faster. 

parallel_stereo:
  * Many fixes for reliability of stereo with local epipolar alignment.
  * Added the option ``--resume-at-corr``. Start at the correlation stage
    and skip recomputing the valid low-res and full-res disparities for
    that stage.
  * Bugfix: Eliminate edge artifacts in stereo refinement (for
    subpixel modes 1, 2, and 3).
  * Print in stereo_pprc the estimated convergence angle for the given
    stereo pair (for alignment methods affineepipolar, local_epipolar, and
    homography).
  * Added the option ``--prev-run-prefix``, which makes parallel_stereo
    start at the triangulation stage while using previous stages
    from this other run. The new run can have different cameras, different
    session (rpc vs dg, isis vs csm), different bundle
    adjustment prefix, and different bathy planes (if applicable).
  * Added option ``--save-left-right-disparity-difference`` to save the
    discrepancy between left-to-right and right-to-left
    disparities, which may help with filtering unreliable
    disparities.
  * Interest point matching with mapprojected images now happens
    at full resolution, which results in a more reliable process
    when there are clouds or if fine features are washed out at
    low resolution.
  * Expanded the doc to address a big gotcha: if left and right
    mapprojected images have somewhat different resolutions, then an
    immense disparity search range can result.
  * Added the option ``--max-disp-spread`` to limit the spread of the
    disparity to this value (useful with clouds in images).
  * Added the option ``--ip-filter-using-dem`` to filter as outliers
    interest point matches whose triangulated height differs by more
    than given value from the height at the same location for the
    given DEM.
  * Added a doc section on handling of images with clouds.
  * Disable by default velocity aberration and atmospheric refraction
    corrections. These are not accurate enough and cause issues with
    convergence of bundle adjustment. Can be enabled with
    ``--enable-correct-velocity-aberration`` and
    ``--enable-correct-atmospheric-refraction``. These improve results
    however with Digital Globe cameras if not doing bundle
    adjustment. (Note that these are still hard-coded as enabled for
    optical bar camera models. This would require further study.)
  * Added ready-made ASTER and LRO NAC examples with sample images,
    cameras, commands, and outputs, all available for
    download. Contributions of more examples are welcome. See
    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples.
  * Bugfix for ASTER cameras; this was fully broken.
  * ASP's SGM and MGM algorithms will always use the cross-check for
    disparity by default, to improve the quality, even if that takes
    more time. It can be turned off with ``--xcorr-threshold -1``.
  * Filter outliers in low-resolution disparity D_sub.tif. Can be
    turned off by setting the percentage in ``--outlier-removal-params``
    to 100.
  * Filtering of interest points based on percentiles (using also
    ``--outlier-removal-params``).
  * Folded ``--remove-outliers-by-disparity-params`` into
    ``--outlier-removal-params``. 
  * Bugfix in disparity search range handling when it is large. 
  * For Linux, in each tile's directory write the elapsed runtime and
    memory usage to ``<tile prefix>-<prog name>-resource-usage.txt``.
  * Removed the ``--local-homography`` option, as it is superseded by 
    ``--alignment-method local_epipolar``, which blends the local results.
  * The stereo tool is deprecated, and can be used only with the
    ASP_BM classical block-matching algorithm when invoked without
    local epipolar alignment. Use parallel_stereo instead. 
  * Added the experimental ``--gotcha-disparity-refinement`` option, under
    NASA proposal 19-PDART19_2-0094.
 
bundle_adjust:
  * Added the option ``--apply-initial-transform-only`` to apply an initial
    transform to cameras while skipping image matching and other
    steps, making the process much faster.
  * Added the option ``--auto-overlap-params`` to automatically compute
    which camera images overlap, if a DEM and camera footprint
    expansion percentage are given. 
  * Added the option ``--max-pairwise-matches`` to put an upper limit on
    the number of matches, as a large number can slow down bundle
    adjustment. The default is 10000, likely a large overestimate (but
    this includes potential outliers). If creating interest points
    from disparity using ``--num-matches-from-disp-triplets``, similar
    values should be used for both of these options.
  * Stop printing warnings about failed triangulation if their number
    goes over 100.
  * Rename verbose ``final_residuals_no_loss_function_pointmap_point_log.csv``
    to ``final_residuals_pointmap.csv`` and
    ``final_residuals_no_loss_function_raw_pixels.txt`` to 
    ``final_residuals_raw_pixels.txt``, etc.
  * Document the useful initial and final ``residuals_stats.txt`` files. 
  * Added new options for reusing a previous run:
    ``--match-files-prefix`` and ``--clean-match-files-prefix``.

sfs:
  * SfS was made to work with any camera model supported by ASP,
    including for Earth. For non-ISIS and non-CSM cameras, the option
    ``--sun-positions`` should be used.
  * Exhaustively tested with the CSM model. It is very recommended to
    use that one instead of ISIS .cub cameras, to get a very large
    speedup and multithreading. 
  * Added a new ``--gradient-weight`` parameter, constraining the 
    first-order derivatives. Can be used in combination with the
    ``--smoothness-weight`` parameter which constrains the second-order
    derivatives. The goal is to avoid a noisy solution without losing
    detail.
  * Much work on expanding the documentation.

mapproject:
  * If the input image file has an embedded RPC camera model, append
    it to the output mapprojected file. (Which makes stereo with
    mapprojected images work correctly in this case.)
  * Always start a process for each tile. The default tile size 
    is set to 5120 for non-ISIS cameras and to 1024 for ISIS. Use
    a large value of ``--tile-size`` to use fewer processes.

bathymetry:
  * Can have different water surfaces in left and right images, so the
    triangulating rays bend at different heights.
  * ``bathy_plane_calc`` can use a mask of pixels above water to find the
    water-land interface, and also a set of actual ``lon, lat, height``
    measurements.
  * Added documentation for how to find water level heights at given 
    times and coordinates using National Ocean Service's tidal zoning
    map.
 
pc_align:
  * Add alignment method similarity-point-to-plane. It works better
    than similarity-point-to-point at finding a scale between the
    clouds when they have a large shift.
  * Bugfix for alignment methods point-to-point and
    similarity-point-to-point.
  * Use RANSAC with ``--initial-transform-from-hillshading``, for increased
    robustness to outliers. Replaced
    ``--initial-transform-outlier-removal-params`` (based on percentiles)
    with ``--initial-transform-ransac-params``.

dem_mosaic:
  * Add the option ``--tap``, to have the output grid be at integer
    multiples of the grid size (like the default behavior of
    ``point2dem`` and ``mapproject``, and ``gdalwarp`` when invoked
    with ``-tap``). If not set, the input grids determine
    the output grid. (The produced DEM will then extend for an
    additional 0.5 x grid_size beyond grid centers on its perimeter.)
  * Do not allow more than one of these operations in a given
    dem_mosaic invocation: fill holes, blur, or erode. These won't
    work when also having more than one input DEM, reprojection is
    desired, or priority blending length is used. This is done to
    avoid confusion about order of operations, and the fact that
    different input DEMs can have different grid sizes and hence the
    input parameters have different effects on each.
  * Bugfix for hole-filling and blurring. Tile artifacts got removed.

stereo_gui: 
  * Can cycle through given images from the View menu, or with the 'n'
    and 'p' keys, when all images are in the same window.
  * Can save a shapefile having points, segments, or polygons. (These
    are distinct classes for a shapefile; the shapefile format
    requires that these not be mixed in the same file.)
  * Noticeable speedup when changing display mode (e.g., from
    side-by-side to overlayed).
  * Bugfix when overlaying shapefiles with different georeferences.
  * Polygon layers can be set to desired colors from the left pane,
    when overlaid.
  * On startup, draw rectangular regions corresponding to values of
    ``--left-image-crop-win`` and ``--right-image-crop-win``, if these
    are passed in as command line arguments together with two images.
  * Quietly accept parallel_stereo options and pass them on if this tool
    is invoked from the GUI.

image_calc:
  * Add the option ``--no-georef`` to remove any georeference
    information in the output image (useful with subsequent GDAL-based
    processing).
  * Added the option ``--longitude-offset`` to help to deal with the
    fact that ASP-produced DEMs and orthoimages may have the
    longitudes in [0, 360] while users may prefer [-180, 180].
  * Bugfix: The ``--input-nodata`` value, if set, now overrides the
    value set in the metadata (the previous value then becomes valid).

Misc:
  * Added the tool ``parse_match_file.py`` to convert a binary match file
    to text and vice-versa.
  * Add the tool ``cam_test`` to compare two different camera models
    for the same image. 
  * Stereo and bundle adjustment with RPC cameras now query the RPC
    model for the datum.
  * The ``cam2rpc`` program saves its datum which is read when needed by
    the RPC model loader.
  * Add the option ``--triangulation-error-factor`` to ``point2las`` to allow
    point cloud triangulation errors multiplied by this factor and
    rounded/clamped appropriately to be stored in the 2-byte intensity
    field in the LAS file.
  * Make symlinks relative in ``parallel_bundle_adjust`` for portability.
  * The mapprojected image saves as metadata the adjustments it was
    created with.
  * Save the low-resolution triangulated point cloud (``PC_sub.tif``) in 
    stereo_corr (based on filtered ``D_sub.tif``).
  * The ``ipmatch`` program can take as input just images, with the 
    .vwip files looked up by extension.
  * Bugfix in handling projections specified via an EPSG code.
  * Bugfix when some environmental variables or the path to ASP
    itself have spaces. (It happens under Microsoft WSL.)
  * Bugfix for the "too many open files" error for large images.
  * Add the build date to the ``--version`` option in the ASP tools
    and to the log files.
  * Bugfix in the original author's MGM implementation, accepted by
    the author.

RELEASE 3.0.0, July 27, 2021
----------------------------
DOI: `10.5281/zenodo.5140581 <https://doi.org/10.5281/zenodo.5140581>`_

New functionality:
  * Added new stereo algorithms: MGM (original author implementation),
    OpenCV SGBM, LIBELAS, MSMW, MSMW2, and OpenCV BM to complement  
    the existing ASP block matching, SGM, and MGM algorithms. See
    https://stereopipeline.readthedocs.io/en/latest/next_steps.html
    for usage. These will be further refined in subsequent releases.
  * Added the ability to perform piecewise local epipolar alignment
    for the input images, to be followed by a 1D disparity search (for
    non-mapprojected images), as suggested by the Satellite Stereo
    Pipeline (S2P) approach. This is still somewhat experimental.
  * Added the ability for a user to plug into ASP any desired stereo
    program working on image clips to which epipolar alignment has
    been applied (as is customary in the computer vision community)
    without rebuilding ASP.
  * Added support for shallow-water bathymetry, so creation of terrain
    models with correct depth determination for well-resolved areas under
    shallow water. To be used with dg, rpc, and nadirpinhole cameras.
  * Added two supporting tools for this: bathy_plane_calc and
    bathy_threshold_calc.py.
  * Added CCD artifact corrections for a few WV02 band 3 multispectral
    images. Apart from the systematic artifacts corrected by this
    logic, these images have a high-frequency unique pattern, and also
    jitter, which are not corrected for. Also added tools and
    documentation to easily tabulate more multispectral bands and TDI.

isis:
  * Upgraded to ISIS 5.0.1.
  * Ship a full Python 3.6 runtime, as expected by ISIS.

csm:
  * Upgraded to USGSCSM 1.5.2 (ASP's own build of it has an additional
    bugfix for LRO NAC not present in the conda-forge package).
  * Validated the CSM model for CTX, HiRISE, and LRO NAC cameras.
  * Added documentation for how to create CSM models from .cub
    cameras.
  * Export the state of a CSM camera after bundle adjustment and
    pc_align (only for linescan cameras supported by ISIS).
 
parallel_stereo
  * Will now throw an error if ``--threads`` is passed in, whose behavior
    was not defined.
  * Bugifx for Python 3.

bundle_adjust:
  * Added the option ``--heights-from-dem-robust-threshold``.
  * Added the option ``--save-intermediate-cameras`` to save the cameras
    at each iteration.
  * Added the option ``--match-first-to-last`` to match the first several
    images to several last images by extending the logic of
    ``--overlap-limit`` past the last image to the earliest ones.

point2las
  * Remove outliers by using a percentile times a factor, in a way
    analogous to point2dem.
   
convert_pinhole_model:
  * Improve the accuracy of the RPC approximation distortion and
    undistortion.

sfs:
  * Added the option ``--shadow-threshold`` to be able to specify
    a single shadow threshold for all images. Also added
    ``--custom-shadow-threshold-list``.
  * Added the option ``--robust-threshold`` for situations when the
    measured image intensity is unreliable.
  * Added the option ``--estimate-height-errors`` to estimate the 
    uncertainty in height at each computed SfS DEM pixel.
    It can be customized via ``--height-error-params``.
  * Added an auxiliary tool named sfs_blend to replace SfS
    pixels with ones from the original LOLA DEM in permanently
    shadowed regions.

stereo_gui:
  * Added the ability to find the contour of a georeferenced image at
    a given threshold. (It can be later edited, saved to disk, etc.) 
  * Bugifxes for polygon drawing logic.
  * Much more responsive for overlaying many images.

image_calc:
  * Support the sign function (can help in creating masks).

pc_align: 
  * Bugifx for ``--initial-transform-from-hillshading`` with outlier
    removal.
  * Add the ``--initial-transform-outlier-removal-params`` to control
    outlier removal when finding matches between DEMs to align
    using features detected in hillshaded images or selected
    manually. 
  * Added ``--initial-rotation-angle``, to initialize the alignment
    transform as the rotation with this angle (in degrees) around
    the axis going from the planet center to the centroid of the point
    cloud.

Misc
 * Moved the daily build to the release area on GitHub, at 
   https://github.com/NeoGeographyToolkit/StereoPipeline/releases
 * Upgraded to GDAL 2.4 and PROJ4 5.2.0. (ISIS constrains updating to
   newer versions of these.)
 * Added the option ``--ip-per-image`` to bundle adjustment and stereo, to
   detect roughly how many interest points should be found per image
   (only a small fraction of them may eventually match across images).
 * The ``--min-triangulation-angle`` in stereo must be always positive if 
   set by the user. Can be set to something very small if desired.
   This is a bug fix for this rarely used option (before, when set to
   0 it would just reset itself to some internal non-small value).  
 * Bugifx for the VisionWorkbench implementation of the
   Levenberg-Marquardt algorithm, it was giving up prematurely in
   challenging situations.
 * Bugifx for affine epipolar alignment. Use the OpenCV function 
   for finding the alignment matrix instead of the ASP one as OpenCV
   can filter outliers which cause issues on rare occasions. 
 * Bugfix: Do not allow a full run to take place in a directory
   where a clip was run, as that will produce incorrect results.
 
RELEASE 2.7.0, July 27, 2020
----------------------------

New functionality
   * Support for ISIS version 4.1.10. Please set ISISDATA instead of
     ISIS3DATA with this version of ISIS and ASP.
   * Support for the Community Sensor Model
     (https://github.com/USGS-Astrogeology/usgscsm)
   * Ability to install ASP with conda. See INSTALLGUIDE.rst for details.
   * Moved the documentation to ReStructured Text, and Sphinx-Doc. See
     the documentation at: https://stereopipeline.readthedocs.io
   * As of this release, we have transitioned to the 
     `Semantic Versioning 2.0.0 standard <https://semver.org>`_ for ASP.

bundle_adjust
   * Can first create interest point matches among mapprojected images
     (automatically or manually) and use those to create matches among
     the unprojected images when the latter are so dissimilar in
     perspective that the direct approach fails. See ``--mapprojected-data``.
  
stereo_gui
   * Bug fix when zooming all images to same region when the region is
     such that all images are seen fully.

sfs
   * Added a new very challenging example at the South Pole with drastic
     illumination changes and using a non-stereo DEM as initial guess.
   * Fixed a bug with craters missing under low light.
   * Fixed a bug with computation of exposures in terrain with many shadows.
   * Print the Sun azimuth angle for all images (useful for sorting them
     by illumination conditions).

hiedr2mosaic.py
   * When hijitreg finds no match points between two CCDs, the program now
     emits a warning message to STDOUT with a suggestion to perhaps
     fiddle with hijitreg manually, and rather than fail with a
     mysterious exception warning, now gracefully falls back to
     assuming that there is no jitter correction between the two
     CCDs that had no matches.

point2dem
   * Use outlier filtering when computing the bounding box of a DEM.
     The same option ``--remove-outliers-params`` controls this
     just as for removing outliers by triangulation error.

mapproject
   * Fixed a bug when finding the extent of the mapprojected
     image when the DEM to project onto spans the whole planet.

point2mesh
   * Only meshes in .obj format are created. This format can be opened
     in Meshlab, Blender, or some other mesh viewer.
   * The osgviewer program is no longer shipped.
   * Fixed a bug with invalid points not being filtered.
   * Fixed a bug with insufficient precision (now it can be set 
     by the user and defaults to 17 digits).
   * Added the option ``--texture-step-size`` to control the sampling
     rate for the texture, in addition to the -s option that controls
     the sampling rate for the point cloud.

Misc
   * Updated to C++ 11.
   * Added phase subpixel correlation accuracy parameter.

RELEASE 2.6.2, June 15, 2019
----------------------------

DOI: https://doi.org/10.5281/zenodo.3247734

New satellites
   * Added support for SkySat, together with a detailed example,
     including how to jointly align and optimize cameras in respect
     to a reference DEM, while optionally refining the intrinsics. 
     This approach may be helpful for other images obtained with frame
     cameras and uncertain positioning information.
   * Added support for CORONA KH-4B, KH-7, and KH-9 declassified images
     and their panoramic (optical bar) camera models, as well as using
     and optimizing camera models with RPC distortion (only RPC is
     supported for KH-7 because it is a linescan camera). An example
     is in the documentation. 
   
New tools
   * Added parallel_bundle_adjust which computes image statistics and
     IP matching in a parallel manner similar to parallel_stereo.
   * Added the cam_gen tool to create a correctly oriented pinhole
     camera model given camera intrinsics, lon-lat coordinates of the
     corners (or some other pixels), and optionally a ground truth
     DEM. It can also parse SkySat's video/frame_index metafile to get
     this data. It can also take as input any camera supported by ASP
     via ``--input-camera`` and create a most-similar pinhole camera
     model with given intrinsics.
   * Added the coverage_fraction tool to provide a coverage estimate
     of the results of a stereo call. 
   * Added the image_mosaic tool which merges together images based on
     interest point matches.  Can be used to stitch together Corona
     scanned images.
   * Added a new tool, n_align, to jointly align n clouds
     (re-implemented from Matlab, works well for small clouds that are
     close to each other).

stereo_rfne
   * Added the option to run a non-SGM subpixel option after
     running SGM/MGM.
   * Added the phase correlation subpixel option. This is a Fourier
     transform based method.

pc_align
   * Added a new approach to finding an initial transform between
     clouds, when they are DEMs, that may be more robust to large
     scale or translation changes, or to noise. It is based on
     hillshading the DEMs and finding interest point matches among
     them, which are then used to find the transform. Can be invoked
     with ``--initial-transform-from-hillshading`` <transform type>.
     Supported transforms are: 'similarity' (rotation + translation +
     scale), 'rigid' (rotation + translation) and 'translation'.
   * Added the expression of the Euler angles in the North-East-Down
     coordinate system around the center of gravity of the source
     cloud.
   * Bug fix: intersection of bounding boxes of the clouds takes
     into account the initial transform applied to the source points.
   * Added a new alignment algorithm, based on 
     https://github.com/IntelVCL/FastGlobalRegistration
     It can be invoked with ``--alignment-method fgr``. It can perform
     better than ICP when the clouds are close enough to each
     other but there is a large number of outliers, when it can
     function with very large ``--max-displacement``. It does worse if the
     clouds need a big shift to align.

bundle_adjust
   * Two passes of bundle adjustment (with outlier filtering after
   * first pass) is now the default. 
   * The flag ``--skip-rough-homography`` is on by default as it usually 
     gives more reliable results. Use ``--enable-rough-homography``
     to turn this option back on (useful when the footprint on the 
     ground and difference in perspective are large).
   * The flag ``--disable-tri-ip-filter`` is also the default as input
     cameras may not be reliable enough for this filter. Can be 
     enabled back with ``--enable-tri-ip-filter``.
   * Added the ``--intrinsics-limits`` option to manually specify 
     intrinsic parameter limits.
   * Added the ``--num-random-passes`` option to allow repeat solving 
     attempts with randomly distorted initial parameters.
   * Added option to automatically guess overlapping images from
     Worldview style XML camera files.
   * Removed the non-Ceres bundle adjustment options.
   * Added the option to share or not share selected intrinsic parameters
     between pinhole cameras when optimizing intrinsics.
   * Improvements in solving simultaneously for both intrinsics and
     extrinsics of n camera images if underlying ground truth
     terrain in the form of a DEM or LIDAR point cloud is
     present. After this bundle adjustment, pairwise stereo and DEM
     creation, the DEMs are well-aligned to the ground truth.
   * Added the flag ``--reference-terrain-weight`` which, when increased,
     helps align better camera images to a given reference terrain. 
   * Added the option ``--heights-from-dem``. It is very helpful in 
     determining an unknown focal length and distortion parameters
     for pinhole cameras.
     It can be used together with ``---heights-from-dem-weight``.
   * Bug fix in outlier filtering for n images.
   * Updated Ceres version from 1.11 to 1.14. When optimizing with 
     multiple threads, results now vary slightly from run to run.
     Results from single threaded runs are deterministic.
   * Added a new ``--parameter-tolerance`` option. Stop when the relative
     error in the variables being optimized is less than this.
   * Documented the ability to create a roughly positioned 
     pinhole camera model from an image if its intrinsics and the 
     longitude and latitude (and optionally height) of its corners
     (or some other pixels) are known.
   * When multiple passes happen with outliers removed, match files
     are not over-written, but a new clean copy of them gets saved.
   * Renamed ``--create-pinhole-cameras`` to ``--inline-adjustments``, and 
     distortion_params to other_intrinsics. This is needed since
     for the panoramic model there will be other intrinsic
     parameters as well.
   * Added the option ``--forced-triangulation-distance`` for when one
     really needs to triangulate with poor cameras. Can be used with 
     a very small but positive value of ``--min-triangulation-angle``.
   * Added the option ``--transform-cameras-using-gcp``. If there
     are at least two images with each having at least 3 GCP
     (each GCP need not show in more than one image), use this
     to convert cameras from an abstract coordinate system to world
     coordinates.
   * Increased the default ``--num-ransac-iterations`` to 1000 from 100
     so that the solver tries harder to find a fit.
     Increased default ``--ip-inlier-factor`` from 1/15 to 0.2 to help
     with getting more interest points for steep terrain.
   * Increased the default ``--ip-uniqueness-threshold`` from 0.7 
     to 0.8 to allow for more interest points.
   * Option to filter interest points by elevation limit and lon-lat limit
     after each pass of bundle adjustment except the last.

dem_mosaic
   * Added normalized median absolute deviation (NMAD) output option.
   * Added the option ``--force-projwin`` to create a mosaic filling
     precisely the desired box specified via ``--t_projwin``.

stereo_gui
   * Added the ability to manually reposition interest points.
   * Can now show non-synchronous .match files (that is, each IP
     need not be present in all images).
   * Added basic functionality for drawing/editing/merging polygons on
   * top of georeferenced images or DEMs. The polygons can be saved as 
     shape files, and then used to cut out portions of images with GDAL.
   * Added the option ``--nodata-value``. Pixels with value less than 
     or equal to this are shown as transparent.
   * Added the ability to view .vwip files (specify one per image).
   * Can view (but not edit) GCP files, via ``--gcp-file`` (creating
     GCP is supported in a separate mode, per the doc).
   * The option ``--dem-file`` specifies a DEM to use when creating
     manually picked GCP and ``--gcp-file`` specifies the name of 
     the GCP file to use upon saving such GCP.

mapproject
   * Added the ``--nearest-neighbor`` option to use that interpolation
     method instead of bicubic.  This is better for labeled images
     which should not be interpolated.

convert_pinhole_model
   * Can create RPC distortion models of any degree, which can be
     further optimized in bundle_adjust. Old RPC distortion files are
     still supported throughout ASP, but not functionality which
     optimizes them. They can be approximately converted to new type
     RPC distortion files with this tool if optimization is desired.

Misc
   * Compiled against USGS ISIS version 3.6.0.
   * Expanded the documentation explaining how to align cameras 
     to a DEM manually (or initialize such cameras) by selecting
     matching points between the images and the DEM.
   * The stereo tools and bundle_adjust will now cache image
     statistics and interest points to files on disk.
   * In stereo and bundle_adjust, when images or cameras are newer
     than the match files, the latter get recomputed unless the tools
     are invoked with ``--force-reuse-match-files``.
   * Added a fix to make stereo work with the ZY3 satellite.
   * For stereo and bundle_adjust, added the ``--no-datum`` option to
     find interest points without assuming a reliable datum exists,
     such as for irregularly shaped bodies. Added the related
     option ``--skip-rough-homography`` to not use the datum in
     rough homography computation. Added the option
     ``--ip-num-ransac-iterations`` for finer control of interest
     point matching. Added ``--ip-triangulation-max-error`` to control
     the triangulation error.
   * The cam2rpc tool accepts ``--t_srs`` and ``--semi-major-axis`` as
     alternatives to ``--datum`` and ``--dem-file``.
   * Add option ``--theia-overrides`` to camera_solve to make it easier
     to customize its behavior via flags.
   * Added an explanation for how the pinhole model works. 
   
RELEASE 2.6.1, August 13, 2018
------------------------------

New satellites
   * Support Cartosat-1 and Perusat-1 RPC cameras.

New tools
   * Added convert_pinhole_model, to convert between various
     existing such models. 
   * Added camera_footprint as a helpful utility to show where
     images will project on to the ground.
   * Documented and improved the ipfind and ipmatch tools.
     ipfind is used to detect interest points in input images,
     either to generate .vwip files for other tools or to 
     experiment with different IP finding settings.
     ipmatch matches the IPs contained in .vwip files to
     create .match files.

New camera models
    * Added simple atmospheric refraction correction to the
      DG and SPOT5 camera models. This can be enabled
      using the "--enable-correct-atmospheric-refraction" option.
    * Added support for pinhole camera models where the lens
      distortion is given by an RPC model (rational polynomial
      coefficients), of degrees 4, 5, and 6. Such a model may be more
      expressive than existing ones, and its coefficients can now be
      optimized using bundle adjustment. An initial model can be
      created with convert_pinhole_model.

stereo_corr
   * Added new options for post-SGM subpixel stereo. Previously only a
     parabola method was used.
   * Added option to perform cross-correlation checks on multiple
     resolution levels while using SGM/MGM.
   * Added option ``--corr-search-limit`` to constrain the automatically
     computed correlation search range.
   * Added ``--corr-memory-limit-mb`` option to limit the memory usage of
     the SGM/MGM algorithms.
   * Improved search range estimation in nadir epipolar alignment
     cases. Added ``--elevation-limit`` option to help constrain this
     search range.
   * Added hybrid SGM/MGM stereo option.
   * Improvements to SGM search range estimation.
   * Added ``--min-num-ip`` option.

bundle_adjust
   * Added the ability to optimize pinhole camera intrinsic
     parameters, with and without having a LIDAR or DEM ground truth
     to be used as reference (the latter is recommended though).
   * The tool is a lot more sensitive now to ``--camera-weight``,
     existing results may change a lot. 
   * Added the parameters ``--rotation-weight`` and ``--translation-weight``
     to penalize large rotation and translation changes.
   * Added the option ``--fixed-camera-indices`` to keep some cameras
     fixed while optimizing others. 
   * Can read the adjustments from a previous invocation of this
     program via ``--input-adjustments-prefix``.
   * Can read each of pc_align's output transforms and apply it
     to the input cameras via ``--initial-transform``, to be able to 
     bring the cameras in the same coordinate system as the aligned
     terrain (the initial transform can have a rotation, translation,
     and scale). If ``--input-adjustments-prefix`` is specified as well,
     the input adjustments are read first, and the pc_align 
     transform is applied on top.
   * Renamed ``--local-pinhole`` to ``--create-pinhole-cameras``.
   * Added the parameter ``--nodata-value`` to ignore pixels at and below
     a threshold.
   * Added the ability to transfer interest points manually picked in
     mapprojected images to the the original unprojected images via
     ``--mapprojected-data``.  
   * Added the flag ``--use-lon-lat-height-gcp-error``. Then, if using
     GCP, the three standard deviations are interpreted as applying
     not to x, y, z but to latitude, longitude, and height above
     datum (in this order). Hence, if the latitude and longitude are
     known accurately, while the height less so, the third standard
     deviation can be set to something much larger.
   * Added the ability to do multiple passes of bundle adjustment,
     removing outliers at each pass based on reprojection error and
     disparity (difference of pixel value between images). This
     works for any number of cameras. Match files are updated with
     outliers removed. Controlled via ``--num-passes``,
     ``--remove-outliers-params`` and ``--remove-outliers-by-disparity-params``.
   * Added the option ``--save-cnet-as-csv``, to save the control
     network containing all interest points in the format used by
     ground control points, so it can be inspected.
   * If ``--datum`` is specified, bundle_adjust will save to disk
     the reprojection errors before and after optimization. 

stereo_gui
   * Can view SPOT5 .BIL files.

pc_align
   * Add the ability to help the tool with an initial translation
     specified as a North-East-Down vector, to be used to correct known
     gross offsets before proceeding with alignment. The option is
     ``--initial-ned-translation``.
   * When pc_align is initialized via ``--initial-transform`` or
     ``--initial-ned-translation``, the translation vector is now computed
     starting from the source points before any of these initial
     transforms are applied, rather than after. The end point of this
     vector is still the source points after alignment to the
     reference. This is consistent with the alignment transform output
     by the tool, which also is from the source points before any
     initial alignment and to the reference points.
   * The translation vector was expressed incorrectly in the
     North-East-Down coordinate system, that is now fixed.

dem_mosaic
   * If the -o option value is specified as filename.tif, all mosaic will be
     written to this exact file, rather than creating tiles. 

point2dem 
   * Added the ability to apply a filter to the cloud points in each circular
     neighborhood before gridding. In addition to the current weighted average
     option, it supports min, max, mean, median, stddev, count, nmad,
     and percentile filters. The ``--search-radius-factor`` parameter can
     control the neighborhood size.
   * Sped up hole-filling in ortho image generation. If this creates
     more holes than before, it is suggested to relax all outlier filtering,
     including via ``--remove-outliers-params``, median filtering, and erosion. 
   * Added the option ``--orthoimage-hole-fill-extra-len`` to make hole-filling
     more aggressive by first extrapolating the cloud.

datum_convert
   * Rewrote the tool to depend on the Proj.4 HTDPGrids grid shift system.
     This fixed some situations where the tool was not working such as WGS84/NAD83
     conversions and also added support for datum realizations (versions).
   * Vertical datum conversion is only supported in simple cases like D_MARS to MOLA.
   * Even with HTDPGrids, datum support with the Proj.4 library is poor and will
     hopefully be improved with future releases.  Until then try to get external
     verification of results obtained with the datum_convert tool.

wv_correct
   * Supports WV2 TDI = 32 in reverse scan direction.

Misc
   * We now compile against USGS ISIS version 3.5.2.
   * The tools mapproject, dem_mosaic, dg_mosaic, and wv_correct support
     the ``--ot`` option, to round the output pixels to several types of
     integer, reducing storage, but perhaps decreasing accuracy.
   * The tools mapproject and image_calc support the ``--mo`` option to
     add metadata to the geoheader in the format 'VAR1=VAL1 VAR2=VAL2',
     etc.
   * Handle properly in bundle_adjust, orbitviz, and stereo 
     with mapprojected images the case when, for RPC cameras,
     these coefficients are stored in _RPC.TXT files.
   * Support for web-based PROJ.4 strings, e.g., 
     point2dem ``--t_srs`` http://spatialreference.org/ref/iau2000/49900/
   * Added ``--max-output-size`` option to point2dem to prevent against
     creation of too large DEMs.
   * Added image download option in hiedr2mosaic.py.
   * Bug fix in cam2map4stereo.py when the longitude crosses 180 degrees.
   * Added support for running sparse_disp with your own Python installation.
   * Bug fix for image cropping with epipolar aligned images.
   * The sfs tool supports the integrability constraint weight from Horn 1990.
   * The software works with both Python versions >= 2.6 and 3. 

RELEASE 2.6.0, May 15, 2017
---------------------------

New stereo algorithms
   * ASP now supports the Semi Global Matching (SGM) and 
     More Global Matching (MGM) stereo algorithms. 
     They do particularly well for Earth imagery, better 
     than the present approaches. They can be invoked with 
     ``--stereo-algorithm`` 1 and 2 respectively. 

New tools
    * Added cam2rpc, a tool to create an RPC model from any
      ASP-supported camera. Such cameras can be used with ASP for
      Earth and planetary data (stereo's ``--datum`` option must be set),
      or passed to third-party stereo tools S2P and SETSM. 
    * Added correct_icebridge_l3_dem for IceBridge.
    * Added fetch_icebridge_data for IceBridge.

parallel_stereo
   * By default, use as many processes as there are cores, and one
     thread per processes.
     
stereo_pprc
   * Large speedup in epipolar alignment.
   * Improved epipolar alignment quality with standard pinhole cameras.
   * Added the options ``--ip-inlier-threshold`` and ``--ip-uniqueness-threshold``
     for finer-grained control over interest point generation.
   * Fix a bug with interest point matching the camera model is RPC
     and the RPC approximation domain does not intersect the datum.
  
stereo_corr
   * Added new option ``--stereo-algorithm``.  Choices 1 and 2 replaces
     the standard integer correlator with a new semi-global matching 
     (SGM) correlator or an MGM correlator respectively.  SGM/MGM is
     slow and memory intensive but it can produce better results
     for some challenging input images, especially for IceBridge.
     See the manual for more details.

stereo_tri
  * Added the option ``--min-triangulation-angle`` to not triangulate
    when rays have an angle less than this. 
 
stereo_gui
  * Zooming in one image can trigger all other side-by-side images to
    zoom to same region.
  * Clicking on a pixel prints image pixel indices, value, and image 
    name. Selecting a region with Control+Mouse prints its bounds in 
    pixels, and, if georeferenced, in projected and degree units. 
  * Added a 1D profile tool for DEMs.
  * Can visualize the pixel locations for a GCP file (by interpreting
    them as interest points).
  * Can save a screenshot of the current view.
  * If all images are in the same window, can show a given image above
    or below all others. Also can zoom to bring any image in full view
    (from the list of images on the left).
  * Options to set the azimuth and elevation when showing hillshaded 
    images.

dem_mosaic
   * Added the option ``--dem-blur-sigma`` to blur the output DEM.
   * Use by default ``--weights-exponent 2`` to improve the blending,
     and increase this to 3 if ``--priority-blending-length`` is specified.
   * Added the options ``--tile-list``, ``--block-max``, and ``--nodata-threshold``. 
   * Display the number of valid pixels written. 
   * Do not write empty tiles. 

geodiff
   * One of the two input files can be in CSV format.

dg_mosaic
    * Save on output the mean values for MEANSUNEL, MEANSUNAZ,
      and a few more.

point2dem
     * Added the parameter ``--gaussian-sigma-factor`` to control the 
       Gaussian kernel width when creating a DEM (to be used together
       with ``--search-radius-factor``).

sfs
    * Improvements, speedups, bug fixes, more documentation, usage
      recipes, much decreased memory usage, together with a lot of
      testing and validation for the Moon.
    * Can run on multiple input DEM clips (which can be chosen as
      representative for the desired large input DEM region and images)
      to solve for adjusted camera positions throughout this region.
    * Added parallel_sfs, to run sfs as multiple processes over
      multiple machines.

bundle_adjust
    * Can optimize the intrinsic parameters for pinhole cameras. The
      focal length, optical center, and distortion parameters can
      be fixed or varied independently of each other. To be used with
      ``--local-pinhole``, ``--solve-intrinsics``, ``--intrinsics-to-float``.
    * Added the option ``--overlap-list``. It can be used to specify which
      image pairs are expected to overlap and hence to be used to
      compute matches.
    * Added the option ``--initial-transform`` to initialize the adjustments
      based on a 4x4 rotation + translation transform, such as coming
      from pc_align. 
    * Added the options ``--ip-inlier-threshold`` and ``--ip-uniqueness-threshold``
      for finer-grained control over interest point generation.

pc_align
   * Can solve for a rotation + translation or for rotation +
     translation + scale using least squares instead of ICP, if the
     first cloud is a DEM. It is suggested that the input clouds be 
     very close or otherwise the ``--initial-transform`` option be used,
     for the method to converge. The option is:
     ``--alignment-method`` [ least-squares | similarity-least-squares ]

Misc
  * Built with ISIS 3.5.0.
  * Minimum supported OS versions are OSX 10.11, RHEL 6, SUSE 12, and
    Ubuntu 14.
  * Ship with GDAL's gdalwarp and gdaldem.
  * Added integration with Zenodo so that this and all future ASP 
	releases will have a DOI.  More info in the asp_book.pdf

RELEASE 2.5.3, August 24, 2016
------------------------------

Highlights:
 
 - Added the ability to process ASTER L1A VNIR images via the tool
   aster2asp that creates image files and both RPC and rigorous
   linescan camera models that can then be passed to stereo.
   The RPC model seems to work just as well as the rigorous one
   and is much faster.

 - Added the ability to process SPOT5 images with stereo,
   bundle_adjust, and mapproject using a rigorous linescan camera model.
 - Added the add_spot_rpc tool to create RPC models for SPOT5
   which allows them to be mapprojected with the RPC model.

pc_align 
   * Can solve for a scale change in addition to a rotation and
     translation to best align two clouds, hence for a similarity
     transform, using option: ``--alignment-method similarity-point-to-point``.

mapproject
   * Added ability to mapproject color images.
   * Added option to mapproject on to a flat datum.

camera_solve
   * Added option to accept multiple input camera models.

Other:

dem_mosaic
   * Fix a bug with mosaicking of DEMs over very large extent.
   * Fix a bug with 360 degree longitude offset.
   * Added the option ``--use-centerline-weights``. It will compute
     blending weights based on a DEM centerline algorithm. Produces 
     smoother weights if the input DEMs don't have holes or complicated
     boundary.

colormap
   * Added a new colormap scheme, 'cubehelix', that works better for
     most color-blind people.

stereo_gui
   * Use transparent pixels for displaying no-data values instead of black.
   * Can delete or hillshade individual images when overlayed.
   * Add control to hide/show all images when in overlay mode.

Misc
   * Make ASP handle gracefully georeferenced images with some pixels
     having projected coordinates outside of the range expected by PROJ.4.
   * Removed the deprecated orthoproject tool. Now mapproject should be used. 
   * Fixed a bug in ``pc_align`` which caused the ``--max-displacement``
     argument to be misread in some situations.
   * Removed some extraneous code slowing down the datum_convert tool.
   * Fixed a bug in point2dem handling the Albers Conic Equal Area projection.
   * Added standard thread/bigtiff/LZW options to image_calc.
 
RELEASE 2.5.2, Feb 29, 2016
---------------------------

Highlights:

Added a constellation of features and tools to support solving for
the positions of input images lacking position information.  Can be used
for aerial imagery with inaccurate or incomplete pose information,
images from low cost drones, historical images lacking metadata, 
and images taken with handheld cameras.

camera_solve
   * New tool which adds support for aerial imagery etc as described above.
   * Uses the THEIA library (http://www.theia-sfm.org/index.html)
     to compute camera positions and orientations where no metadata is available. 
   * Ground control points and estimated camera positions
     can be used to find absolute camera positions.
   * Added section to documentation describing ways to use ASP to 
     process imagery from NASA's IceBridge program.

camera_calibrate
    * A convenience camera calibration tool that is a wrapper around
      the OpenCV checkerboard calibration program with outputs in
      formats for camera_solve and ASP.

bundle_adjust
    * Added several options to support solving for pinhole camera
      models in local coordinates using GCPs or estimated camera positions.
    * Improved filtering options for which images are IP-matched.

orbitviz
    * Significantly improved the accuracy of the plotted camera locations.
    * Added option to load results from camera_solve.

wv_correct
    * Now corrects TDI 8 (Reverse) of WV01 and TDI 8 (Forward 
      and Reverse) and TDI 32 (Forward) of WV02.  Other correction
      behavior is unchanged.

stereo_corr
   * Added the ability to filter large disparities from D_sub that 
     can greatly slow down a run. The options are ``--rm-quantile-percentile``
     and ``--rm-quantile-multiple``. 

undistort_image
    * A new tool to test out pinhole model lens distortion parameters.
    
Lens distortion models:
    * Switched from binary .pinhole file format to updated version of
      the old plain text .tsai file format.
    * Added support for Photometrix camera calibration parameters.
    * New appendix to the documentation describing the .tsai file format
      and supported lens distortion models.
    
Other:

Tools
    * Suppressed pesky aux.xml warning sometimes printed by GDAL.
    * Removed the long-deprecated orthoproject tool.
    * Added icebridge_kmz_to_csv and lvis2kml utilities.

point2las
    * Write correct bounding box in the header.
    * Respect projections that are not lon-lat.

point2dem
    * Increased speed of erode option.
   
docs
    * Mention DERT, a tool for exploring large DEMs.
    * Added new section describing camera_solve tool in detail.

RELEASE 2.5.1, November 13, 2015
--------------------------------

Highlights:

stereo
    * Added jitter correction for Digital Globe linescan imagery.
    * Bug fix for stereo with map-projected images using the RPC
      session (e.g, for map-projected Pleiades imagery).
    * Added OpenCV-based SIFT and ORB interest point finding options.

bundle_adjust
    * Much improved convergence for Digital Globe cameras.
    * Added OpenCV-based SIFT and ORB interest point finding options.

point2dem, point2las, and pc_align
   * The datum (``-r <planet>`` or ``--semi-major-axis``) is optional now.
     The planet will be inferred automatically (together with the
     projection) from the input images if present. This can be useful
     for bodies that are not Moon, Mars, or Earth. The datum and
     projection can still be overridden with ``--reference-spheroid`` (or
     ``--datum``) and ``--t_srs``. 

dem_mosaic
   * Introduce ``--priority-blending-length``, measured in input pixels. 
     If positive, keep unmodified values from the earliest available
     DEM at the current location except a band this wide near its
     boundary where blending will happen. Meant to be used with 
     smaller high-resolution "foreground" DEMs and larger
     lower-resolution "background" DEMs that should be specified later
     in the list. Changing ``--weights-exponent`` can improve transition.

pc_align
  * Added the ability to compute a manual rotation + translation +
    scale transform based on user-selected point correspondences
    from reference to source cloud in stereo_gui.

stereo_gui
   * Added the ability to generate ground control point (GCP) files
     for bundle_adjust by picking features. In addition to the images
     to be bundle-adjusted, one should provide a georeferenced image to find
     the GCP lon-lat, and a reference DEM to find the GCP heights.

Other:

stereo
    * If the input images are map-projected (georeferenced) and 
      alignment method is none, all image outputs of stereo are
      georeferenced as well, such as GoodPixelMap, D_sub, disparity,
      etc. As such, all these data can be overlayed in stereo_gui.
    * The output point cloud saves datum info from input images
      (even when the inputs are not georeferenced). 
    * Increased reliability of interest point detection.
    * Decreased the default timeout to 900 seconds. This still needs
      tuning and a permanent solution is necessary.

point2dem, point2las, and pc_align
  * Accept ``--datum`` (``-r``) ``MOLA``, as a shortcut for the sphere with
     radius 3,396,000 meters.

dem_mosaic
   * Fix an issue with minor jumps across tiles. 
   * Introduce ``--save-dem-weight`` <index>. Saves the weight image that
     tracks how much the input DEM with given index contributed to the
     output mosaic at each pixel (smallest index is 0).
   * Introduce ``--save-index-map``. For each output pixel, save the
     index of the input DEM it came from (applicable only for
     ``--first``, ``--last``, ``--min``, and ``--max``). A text file with the index
     assigned to each input DEM is saved as well.
   * Rename ``--blending-length`` to ``--extra-crop-length``, for clarity. 

dg_mosaic 
   * Added the switch ``--fix-seams`` to use interest point matching
     to fix seams in the output mosaic due to inconsistencies between
     image and camera data. Such artifacts may show up in older
     (2009 or earlier) Digital Globe images.

stereo_gui
   * Added the option ``--match-file`` to view interest point matches.
   * Added the options ``--delete-temporary-files-on-exit`` and
     ``--create-image-pyramids-only``.
   * Can read the georeference of map-projected ISIS cubes.

point2dem
   * Respect ``--t_projwin`` to the letter. 
   * Can create simultaneously DEMs at multiple resolutions (by
     passing multiple values in quotes to ``--dem-spacing``).
   * Fix minor discrepancies in the minor semi-axis for the WGS84,
     NAD83 and WGS72 datums. Now using GDAL/OGR for that.

point2las
   * Save the LAS file with a datum if the input PC had one.

image_calc
   * Fix calculation bug when no-data is present.

pc_align
  * Upgraded to the latest libpointmatcher. This may result in minor
    alignment changes as the core algorithm got modified.
  * Save all PC clouds with datum and projection info, if present. Add
    comment lines with the datum and projection to CSV files.

geodiff
   * Bug fix when the two DEMs have longitudes offset by 360 degrees.

colormap
   * Default style is binary-red-blue. Works better than jet when 
     data goes out of range.

pc_merge
   * Can merge clouds with 1 band. That is, can merge not only PC.tif
     files but also L.tif files, with the goal of using these two
     merged datasets to create a merged orthoimage with point2dem.

point2mesh
   * Can create a mesh from a DEM and an orthoimage (DRG file).

RELEASE 2.5.0, August 31, 2015
------------------------------

Improved speed, coverage, and accuracy for areas with steep slopes
for ISIS, RPC and Pinhole cameras by implementing stereo using
images map-projected onto an existing DEM. This mapprojection is
multi-process and hence much faster than cam2map. This
functionality was previously available only for Digital Globe
images.

New tools:
    * Added stereo_gui, an image viewer and GUI front-end.
      Features:

      - View extremely large images using a pyramid approach.
      - If invoked with the same interface as stereo, can run stereo on 
        selected clips.
      - Load images with int, float, and RGB pixels, including ISIS
        cubes, DEMs, NTF, TIF, and other formats.
      - Can overlay georeferenced images and can toggle individual
        images on and off (like Google Earth).
      - Show images side-by-side, as tiles on grid, or on top of each other.
      - Create and view hillshaded DEMs.
      - View/add/delete interest points.
      - Create shadow thresholds by clicking on shadow pixels (needed
        for sfs).
      - Based on Michael Broxton's vwv tool. 

   * Added sfs, a tool to refine DEMs using shape-from-shading. Can
     optimize the DEM, albedo per pixel, image exposures and camera
     positions and orientations using a multi-resolution pyramid
     approach. Can handle shadows. Tested with LRO NAC lunar images at
     low latitudes and toward poles. It works only with ISIS images.
   * Added image_calc, a tool for performing simple per-pixel arithmetic
     operations on one or more images.
   * Added pc_merge, a tool for concatenating ASP-produced point clouds.
   * Added pansharp, a tool to apply a pansharp algorithm to a matched
     grayscale image and a low resolution color image.
   * Added datum_convert, a tool to transform a DEM to a different
     datum (e.g., NAD27 to WGS84).
   * Added geodiff, a tool for taking the (absolute) difference of two 
     DEMs.
   * Documented the colormap tool. Added a new colormap option based 
     on the paper "Diverging Color Maps for Scientific Visualization" 
     (http://www.sandia.gov/~kmorel/documents/ColorMaps/).
   * Added gdalinfo, gdal_translate, and gdalbuildvrt to the bin
     directory. These executables are compiled with JPEG2000 and
     BigTIFF support, and  can handle NTF images.

docs
   * Added a documentation section on 'tips and tricks', summarizing 
     in one place practices for getting the most out of ASP.

stereo
   * Increase the default correlation timeout to 1800 seconds.
   * Fix failure in interest point matching in certain circumstances.
   * Use bundle-adjusted models (if provided) at all stages of stereo,
     not just at triangulation.
   * Added ``--right-image-crop-win`` in addition to ``--left-image-crop-win``.
     If both are specified, stereo crops both images to desired regions
     before running stereo (this is different from when only 
     ``--left-image-crop-win`` is specified, as then no actual cropping 
     happens, the domain of computation is just restricted to the desired
     area). 
   * Bug fix, remove outliers during search range determination.
   * Added the option ``--ip-per-tile``, to search for more interest points 
     if the default is insufficient.
   * If the input images are georeferenced, the good pixel map will be
     written with a georeference.
 
point2dem
   * Fixed a slight discrepancy in the value of the semi-minor axis in
     the WGS84 and NAD83 datum implementations.
   * Added the option ``--median-filter-params`` <window size> <threshold> to
     remove spikes using a median filter.
   * Added the option ``--erode-length`` <num> to erode pixels from point cloud 
     boundary (after outliers are removed, but before filling in holes).
   * Improved hole-filling, and removed the ``--hole-fill-mode`` and 
     ``--hole-fill-num-smooth-iter``, as there's only one algorithm now. 
   * Improved performance when large holes are to be filled.
   * Can create a DEM from point clouds stored in CSV files containing
     easting, northing, and height above datum (the PROJ.4 string
     needed to interpret these numbers should be set with ``--csv-proj4``).
   * Fixed a bug in creating DEMs from CSV files when different projections
     are used on input and output.
   * Expose to user gnomonic and oblique stereographic projections,
     as well as false easting and false northing (where applicable). 
     This is a shortcut from using explicitly ``--t_srs`` for the PROJ.4 string.
   * The default no-data value is set to the smallest float.
 
pc_align
   * Can ingest CSV files containing easting, northing, and height
     above datum (the PROJ.4 string needed to interpret these numbers
     should be set with ``--csv-proj4``).
   * If the reference point cloud is a DEM, the initial and final errors
     in the statistics, as well as gross outlier removal, are done using
     a new distance function. Instead of finding the distance from a 3D 
     point to the closest point in the cloud, the 3D point is projected 
     onto DEM's datum, its longitude and latitude are found, the
     height in the DEM is interpolated, and and the obtained point on the 
     DEM is declared to be the closest point. This is more accurate
     than the original implementation for coarse DEMs. The old 
     approach is available using the ``--no-dem-distances`` flag.
   * Fix a bug with a 360 degree longitude offset.

point2las
   * Added the ability to specify a custom projection (PROJ.4 string)
     for output LAS files.

dem_mosaic
   * Write GeoTIFF files with blocks of size 256 x 256 as those
     may be faster to process with GDAL tools.
   * Bug fix when the tool is used to re-project.
   * Added the option ``--weights-blur-sigma`` <num> to allow the blending
     weights to be blurred by a Gaussian to increase their smoothness.
   * Added the option ``--weight-exponent`` <num>, to allow weights
     to increase faster than linearly.
   * Added ``--stddev`` option to compute standard deviation.
   * Added the ability to fill holes in the output mosaic.

bundle_adjust
    * Added new parameters, ``--ip-per-tile`` and ``--min-triangulation-angle``.
    * Bug fix in handling situations when a point cannot get projected
      into the camera.
    * Bug fix in the camera adjustment logic. Any .adjust files may 
      need to be regenerated.

image2qtree
   * Bug fixes.
 
cam2map4stereo.py
   * Create temporary files in current directory, to avoid access
     issues to system directories.

mapproject
   * Can run on multiple machines.
   * Use multiple processes for ISIS images, for a huge speedup.
   * Bug fix, the mapprojected image should not go much beyond the DEM
     it is mapprojected onto (where it would have no valid pixels).

dg_mosaic
   * Default penalty weight produces a more accurate fit when creating an 
     RPC model from a DG model.
   * Handle the situation when two images to be mosaicked start at the 
     same output row number.
   * Added ``--target-resolution`` option to specify the output resolution in meters.

Misc.
   * Upgraded to ISIS 3.4.10.
   * Oldest supported OSX version is 10.8.
   * Added documentation for image2qtree and hillshade.

RELEASE 2.4.2, October 6, 2014
------------------------------

ASP can perform multi-view triangulation (using both the
stereo and parallel_stereo tools). The first image is set
as reference, disparities are computed from it to the other 
ones, and joint triangulation is performed.

Added a new tool, dem_mosaic, for mosaicking a large number of 
DEMs, with erosion at boundary, smooth blending, and tiled output.
Instead of blending, the tool can do the first, last, min, max,
mean, median, or count of encountered DEM values.   

dg_mosaic
   * Support for multi-band (multi-spectral) images. Use ``--band`` <num>
     to pick a band to mosaic.
      
stereo
   * Bug fix in interest point matching in certain circumstances.
   * Set the correlation timeout to 600 seconds. This is generous
     and ensures runs don't stall. 
 
point2dem
   * Take as input n clouds and optionally n texture files, create a
     single DEM/orthoimage.
   * Take as input LAS and CSV files in addition to ASP's PC format.
   * Fix a bug in the interplay of hole-filling and outlier removal
     for orthoimage creation.
   * Ensure that the DEM grid is always at integer multiples of the
     grid size. This way, two DEMs with overlapping grids of the same
     size will be exactly on top of each other, minimizing interpolation
     error in subsequent mosaicking.
   * Outlier removal is on by default. Can be disabled by setting 
     the percentage in ``--remove-outliers-params`` to 100.
 
bundle_adjust
   * Use multiple-threads for non-ISIS sessions.
   * Added the parameter ``--overlap-limit`` <num> to limit the number 
     of subsequent images to search for matches to the current image.
   * Added the parameter ``--camera-weight`` <val>, to set the weight to
     give to the constraint that the camera positions/orientations
     stay close to the original values (only for the Ceres solver).

dem_geoid
   * Support the EGM2008 geoid. The geoid surface across all Earth
     is computed with an error of less than 1.5 cm compared to the
     values generated by harmonic synthesis. A 2.5 x 2.5 minute grid
     is used.
   * Converted the EGM geoids shipped with ASP to INT16 and JPEG2000,
     resulting in size reduction of more than 10x. 

wv_correct
    * Corrects TDI of 16, 48, 56, and 64 (forward and reverse scan
      directions) for WV01, TDI of 8 (forward only) for WV01, and TDI
      of 16, 48, 64 (forward and reverse scan directions) for
      WV02. Returns uncorrected images in other cases.

pc_align
    * Fix a crash for very large clouds.  
    * Use a progress bar when loading data.
    * Support LAS files on input and output.

point2las
    * Bug fix when saving LAS files in respect to a datum.

Documentation
    * Move the non-ISIS-specific tutorial sections onto its own
      chapter, to be read by both ISIS and Earth users. Updates and
      cleanup.

RELEASE 2.4.1, 12 July, 2014
----------------------------

Added a new tool, bundle_adjust, which uses Google's ceres-solver
to solve for adjusted camera positions and orientations. Works
for n images and cameras, for all camera types supported by ASP. 

wv_correct
    * Improved corrections for WV01 images of TDI 16.

stereo_rfne
    * Performance bugfix when the integer disparity is noisy.
 
stereo_fltr
    * Fix for large memory usage when removing small islands from
      disparity with ``--erode-max-size``.

stereo_tri
    * Bug fixes for MER cameras.

stereo_tri and mapproject
    * Added the option ``--bundle-adjust-prefix`` to read adjusted
      camera models obtained by previously running bundle_adjust with
      this output prefix.

point2las
    * LAS files can be saved in geo-referenced format in respect 
      to a specified datum (option ``--reference-spheroid``).
 
point2dem
    * Bug fix, longitude could be off by 360 degrees.
    * Robustness to large jumps in point cloud values.

pc_align
    * Ability to read and write CSV files having UTM data (easting,
      northing, height above datum).
    * Read DEMs in the ISIS cube format.

RELEASE 2.4.0, 28 April, 2014
-----------------------------

Added wv_correct, a tool for correcting artifacts in Digital Globe
WorldView-1 and WorldView-2 images with TDI of 16.

Added logging to a file for stereo, pc_align, point2dem, 
point2mesh, point2las, and dem_geoid.

Added a tutorial for processing Digital Globe Earth imagery
and expanded the MOC tutorial.

Bug fixes in mosaicking of Digital Globe images.

parallel_stereo
     * Use dynamic load balancing for improved performance.
     * Automatically determine the optimal number of processes
       and threads for each stage of stereo.

stereo_pprc
     * Added the ``--skip-image-normalization`` option (for non-ISIS 
       images and alignment-method none), it can help with reducing
       the size of data on disk and performance.
       
stereo_rfne
     * Added new affine subpixel refinement mode,
       ``--subpixel-mode 3``. This mode sacrifices the error resistance
       of Bayes EM mode in exchange for reduced computation time.
       For some data sets this can perform as well as Bayes EM in
       about one fifth the time.

stereo_fltr:
     * Hole-filling is disabled by default in stereo_fltr. It is 
       suggested to use instead point2dem's analogous functionality.
       It can be re-enabled using ``--enable-fill-holes``.
     * Added the option ``--erode-max-size`` to remove isolated blobs.
     * Relaxed filtering of disparities, retaining more valid
       disparities. Can be adjusted with ``--filter-mode`` and related
       parameters.

stereo_tri:
    * Added ability to save triangulation error for a DEM as a 3D
      North-East-Down vector rather than just its magnitude.
    * When acting on map-projected images, handle the case when the 
      DEM used for map-projection does not completely encompass the 
      images.
 
pc_align:
    * Read and write CSV files in a wide variety of formats, using 
      the ``--csv-format`` option.
    * Display the translation component of the rigid alignment
      transform in the local North-East-Down coordinate system, as
      well as the centroid of source points used in alignment.
    * Save to disk the convergence history (iteration information).
    * Added the ability to explicitly specify the datum semi-axes.
    * Bug fix for saving transformed clouds for Moon and Mars.
    * More efficient processing of reference and source points
      by loading only points in each cloud within a neighborhood
      of the long/lat bounding box of the other cloud.
    * Make it possible to generate ortho and error images using
      point2dem with the transformed clouds output by pc_align.

point2dem:
     * Replaced the core algorithm. Instead of sampling the point
       cloud surface, which is prone to aliasing, the DEM height at a
       given grid point is obtained as a weighted average of heights
       of all points in the cloud within search radius of the grid
       point, with the weights given by a Gaussian. The cutoff of the
       Gaussian can be controlled using the ``--search-radius-factor``
       option. The old algorithm is still available (but obsoleted)
       using the ``--use-surface-sampling`` option. The new algorithm
       makes the ``--fsaa`` option redundant. 
     * Added the ability to remove outliers by triangulation error,
       either automatically (--remove-outliers) or manually, with 
       given error threshold (--max-valid-triangulation-error).
     * Added two algorithms to fill holes in the output DEM and 
       orthoimage (--hole-fill-mode).
     * The way the default DEM spacing is computed was modified, 
       to make dependent only on the local distribution of points
       in the cloud and robust to outliers. 
     * Can handle highly noisy input point clouds without spikes in 
       memory usage and processing time.
     * Improved memory usage and performance for large point clouds.
     * Bug fix, the DEM was shifted by 1 pixel from true location.

RELEASE 2.3.0, 19 November, 2013
--------------------------------

TOOLS:

- Added pc_align, a tool for aligning point clouds, using the
  libpointmatcher library
  (https://github.com/ethz-asl/libpointmatcher). Sparse and dense
  point clouds are supported, as well as DEMs. Two ICP methods are
  supported, point-to-plane and point-to-point. Memory and processing
  usage are proportional to the desired number of input points
  to use rather than to the overall input data sizes.

- Added lronac2mosaic.py, a tool for merging the LE and RE images
  from the LRONAC camera into a single map-projected image.  The
  output images can be fed into the stereo tool to generate DEMs.

- rpc_maprpoject and orthoproject are combined into a single tool
  for projecting a camera image onto a DEM for any camera model
  supported by Stereo Pipeline. The old orthoproject is kept for 
  backward compatibility for a while.

GENERAL: 

- Stereo Pipeline (almost) daily and fully verified builds for all
  platforms are available for the adventurous user
  (http://byss.arc.nasa.gov/stereopipeline/daily_build/, which was
  later moved to https://github.com/NeoGeographyToolkit/StereoPipeline/releases).
  When requesting support, please provide the output of ``stereo --version``.

- The size of Stereo Pipeline output data has been reduced, by up to
  40%, particularly point clouds and DEMs are between 30% to 70%
  smaller.  Better encoding is used, output data is rounded (up to 1
  mm), and point clouds are offset and saved as float instead of
  double.
  
- Timeout option added for stereo correlation, preventing
  unreasonably long correlation times for certain image tiles.

- Subpixel mosaicking in dg_mosaic uses bilinear interpolation
  instead of nearest neighbor avoiding artifacts in certain
  situations.

- dg_mosaic can generate a combined RPC model in addition to the
  combined DG model. It accepts flags for specifying input and 
  output nodata values.

- point2dem with the ``--fsaa`` option for reducing aliasing at
  low-resolution DEM generation has been improved as to remove the
  erosion of of valid data close to no-data values.

- Bug fixes for parallel_stereo, point2dem, etc. 

RELEASE 2.2.2, 17 MAY 2013
--------------------------
(incremented from 2.2.1 after one more bugfix)

TOOLS:

- stereo_mpi renamed to parallel_stereo and made to work
  on any machines with shared storage, rather than just on 
  supercomputers using Intel's MPI library. Bug fixes for
  homography and affine epipolar alignment modes, etc.

- Bug fix for dem_geoid path to geoids, more robust datum
  identification.

RELEASE 2.2.0, 6 MAY 2013
-------------------------

GENERAL:

- ISIS headers removed from IsisIO's headers.
- Removed unneeded mutex inside inpaint algorithm.
- Interest point matching and description are parallel now.
- Stereo pprc uses separable convolution for anti-aliasing.
- IsisIO made compliant with ISIS 3.4.3's API.
- Blob consolidation (for inpainting) is now parallel.
- Yamaha RMAX code dropped.

SESSIONS:

- RPC mode can now read Astrium data.
- DG added additional safety checks for XML values.
- DG, ISIS, and RPC now have affineepipolar alignment option.
- All sessions had their API changed. We now use Transform objects
  instead of LUTs to reverse mapprojections and alignments.

TOOLS:

- Added dem_geoid, dg_mosaic, and stereo_mpi.
- Added new interest point matching method to stereo.
- Added new DEM seed mode for stereo.
- Point2dem sped up by reducing over rasterization of triangles.
- Added the ``--use-local-homography`` option to stereo_corr. Homography
  transform is applied per tile.
- Fix point2dem where for certain projections we were setting K=0.
- Stereo can now operate using command-line arguments only, without 
  stereo.default.

RELEASE 2.1.0, 8 JANUARY 2013
-----------------------------

GENERAL:

- Added documentation for processing GeoEye, Digital Globe, and Dawn FC data.
- Fixed implementation of internal RANSAC function.
- DEMError has been renamed IntersectionErr. 3D IntersectionErr is
  now recordable in local North East Down format.

SESSIONS:

- Added RPC processing session.
- DG sessions now use bicubic interpolation for mapprojection arithmetic.
- Fixed bug in case where DG XML file had single TLC entry.
- DG sessions now applies velocity aberration corrections.

TOOLS:

- Have point2dem use correct nodata value when writing DRGs.
- Fix segfault issue in point2dem due to triangle clipping.
- Hiedr2mosaic python script now supports missing CCD files and
  start/stop resume on noproj step for bundle adjustment.
- Max pyramid level used for stereo correlation is configurable with
  corr-max-levels option.
- Stereo accepts left-image-crop-win option for processing of
  specific image coordinates.
- Stereo_pprc accepts nodata-threshold and nodata-percentage options
  for masking (possibly shadows).
- Stereo command should now correctly call secondary executables so
  that their dependencies are loaded.

RELEASE 2.0.0, 20 JUNE 2012
---------------------------

GENERAL:

- Modified ASP according to API changes in ISIS 3.4.0.
- Added new interest point matching code. Provides better initial
  guess for search range.
- Complete changed stereo.default format. See stereo.default.example
  for an example.
- Complete rewrote integer correlator for improved speed and less
  memory use.
- Relicense code to be Apache 2 licensed instead of NOSA.

SESSIONS:

- Add normalization options to PINHOLE session.
- Added Digital Globe (DG) session. This supports the linearized
  linescan camera model that is described in the supporting XML file.
- Deleted KEYPOINT session. PINHOLE essentially does all of that.

EXAMPLES:

- Added DEMError output example for MOC.
- Added jigsaw example for MOC.
- Added HiRISE example dataset.

TOOLS:

- Dropped release of isis_adjust and bundlevis.
- Fix int32 overflow in arithmetic for subsampling in preprocessing.
- Remove Python 2.4 incompatible call in cam2map4stereo.py.
- Speed up point2dem texture access by remove unnecessary mutex.
- Add earth mode and fix non spherical support in point2dem.
- Added lronac4staged.py.
- Implemented D_sub or seeded integer correlation in stereo_corr.
- Fourth channel of output PC file is now triangulation error.
- Added ``--t_srs`` option to point2dem.
- Added rpc_mapproject tool. This provides an optional mapprojection
  step that can be used for DG session.
- Allow IAU2000:* projection options to be used by point2dem.
- No-Data is now colored black in GoodPixelMap.
- Make noproj step in hiedr2mosaic parallel.

RELEASE 1.0.5, 27 OCT 2011
--------------------------

Fixed ASP to work with ISIS 3.3.0's new API changes and library
dependencies.

Enabled parallel writing in Pinhole Session.

TOOLS:

- Fix possible infinite loop in stereo_corr's search range.
- Shutoff rotation invariance in automatic search range for better
  quality results. This is possible because the input images are
  already aligned.
- Sub image produced by stereo_pprc are now limited to around 8MB.
- Fix disparity_debug to work with integer disparities as well.
- All ASP tools should now have a '--version' option.
- Bug fix point2dem where rasterizer was accessing outside of
  allocated memory.
- Speed up mask generation in stereo_pprc by avoiding mutex.
- Speed up hole filling in stereo_fltr by avoiding mutex.

RELEASE 1.0.4, 23 MAY 2011
--------------------------

Added support for CAHVORE in pinhole sessions.

TOOLS:

- Hide GDAL warnings caused by our file integrity checks.
- Mostly added standardized options for settings threads and BigTIFF.
- Have orthoproject return same type as input plus alpha channel.
- Improved edge_masking, speeds up stereo_fltr and stereo_pprc.
- Have cam2map4stereo.py explicitly use ISIS's getkey command.
- Fix and optimized point2dem. Remove caching and improved rendering
  times. This should fix BigTIFF problems that have been reported.
- Improve triangulation times slightly when using mapprojected
  linescan cameras.

EXAMPLES:

- Added orthoproject, image2qtree, colormap, hillshade examples to MOC.
- Added K10 example dataset.
- Added MER example dataset.
- Added a non-mapprojected MOC example.
- Added CTX example dataset.

DOCS:

- Append notes from Michael about run times.

VISION WORKBENCH benefits:

- Added threaded writing to colormap and hillshade.
- Fix hillshade problems with int16 DEMs.

RELEASE 1.0.3.1, 16 MARCH 2011
------------------------------

Updated documentation and support text files to insure compatibility
with our third party software.

RELEASE 1.0.3, 11 MARCH 2011
----------------------------

ISISIO:
  Make quaternion interaction compliant with VW changes.

SESSIONS:
  Correct reading of TSAI camera format.

TOOLS:

- Reduce memory footprint of ISIS_Adjust.
- MOC Example rewritten.
- Improve dash script that loads libraries on startup of application.

VISION WORKBENCH benefits:

- KD-Tree search replace with FLANN, a fast approximate nearest
  neighbors. This improves speed of ipmatch, and ip alignment
  option in stereo.
- Removed exception catch in Bayesian affine sub-pixel.
- Fixed type deduction problem on 32 bit systems.
- Pyramid Correlator code cleaned up. Minimal speed improvement.
- Fixed Camera Relation Network's memory leak.
- Fix image2qtree normalization and manual geo-positioning.
- Correct random seed call with faster solution.
- Default raster tile size changed to 256.
- Fix deadlocking in loading of ".vwrc", Vision Workbench's settings file.

KNOWN ISSUES
  OSX seems to do excessive locking during multi-threaded rendering.
  This problem is non-existent in RHEL5 and is still a mystery.

RELEASE 1.0.2, 9 DECEMBER 2010
------------------------------

ISISIO:

- IsisCameraModel support operator<< style printing.
- Correct camera pose return to be consistent with VW.
- Change IsisCameraModel to use shared_ptr to block memory leak.

TOOLS:

- Executables should catch VW and Standard errors and print human readable
  responses.
- Stereo is now a python script that call multiple executables.
- Change correlation progress bar to track total completion.
- Bundle_Adjust and ISIS_Adjust switch from Euler's to quaternions.
- Bundlevis dropped CAHVOR support. Added progress bar. Converted statistics
  with CDFAccumulator.
- Point2dem remove excessive rotation call
- Enforce tile rasterization size to 1024 during integer correlation.
- Select tools should now write their nodata value in the TIFF metadata.

PHOTOMETRYTK
    Still unreleased, and still under development.

RELEASE 1.0.1, 24 MAY 2010
--------------------------

CORE:

- Control Network Loader removed and sent to VW's Bundle Adjustment Module.
- Build system can now use Google PerfTools.
- Kakadu was made optional in build system (ISIS 3.2.x uses this).

ISISIO:

- Optimized IsisCameraModel to use IsisInterface. Custom code can be loaded up
  for individual camera types so we don't have to run through ISIS's entire
  camera model. This allows us not to call GroundMap when the camera is not
  mapprojected.
- Added a series of tests for the IsisCameraModel that perform unit tests
  with MOC and Galileo.
- Added custom project code for Linescan cameras so not to rely on ISIS's
  LineScanCameraGroundMap. This code is a bit more precise.

MPI
   Added new optional module called MPI that builds on top of
   Boost MPI. This is experimental development code and is not used for
   anything in binary release yet.

PHOTOMETRYTK
   Added new optional module call the Photometry Toolkit. This is
   experimental development code and is not use for anything released
   in the binary yet. This code focuses on future research of massive
   mosaics (+100GB) and the ability to perform basic photometric corrections.

SESSIONS
   Pinhole session modified to read CMOD files as well.

TOOLS:

 - Made orthoproject more robust against odd input georeferences.
 - orthoproject's auto scale and crop works again.
 - Point2mesh's texture is written to a different file.
 - Added aligndem and geodiff, experimental DEM alignment utilities.
 - Added a quick experimental DEM profile utility called dem_profile.
 - stereo now detects correlation settings automatically using OBALoG and
   SGrad1 interest point functions.
 - Added cam2map4stereo.py
 - Remove excessive serial number calculations in isis_adjust.
 - Update isis_adjust to VW's new Bundle Adjustment module for a 2x improvement.
 - Stereo should now use LZW compression by default.
 - Point2dem and Stereo have added option to use directory other than /tmp for
   intermediate files.
 - Point2dem now uses MOLA datum instead of its previous truncated value.
 - Added safety check to stereo to make sure user is not supplying the
   same camera.
 - Added point2las, a utility for converting a point cloud to the LAS format.

TESTS
   Switched from CXXTests to GTest framework.

RELEASE 1.0.0, 23 OCTOBER, 2009
-------------------------------

CORE:

 - OrthoRasterizer.h is subject to change for further VW integration
 - MedianFilter.h is untested/unused
 - BundleAdjustUtils.* is subject to deletion for integration with
   ControlNetworkLoader.*

SESSIONS:

 - ISIS Session is the only fully supported session at this time
 - Pinhole Session works but has not been tested for this release
 - Keypoint/RMAX Session status are unknown

SPICEIO
   Subject to deletion in 1.0.1

TOOLS:

 - Point2dem can crash rarely. Still investigating.
 - rmax* utilities are not working
