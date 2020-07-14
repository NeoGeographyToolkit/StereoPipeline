RELEASE 2.7.0, Upcoming!
------------------------

New functionality
   * Support for the Community Sensor Model
     (https://github.com/USGS-Astrogeology/usgscsm)
   * Support for ISIS version 4.1.10. Please set ISISDATA instead of
     ISIS3DATA with this version of ISIS and ASP.
   * Ability to install ASP with conda. See INSTALLGUIDE.rst for details.
 
bundle_adjust
   * Can first create interest point matches among mapprojected images
     (automatically or manually) and use those to create matches among
     the unprojected images when the latter are so dissimilar in
     perspective that the direct approach fails. See --mapprojected-data.
  
stereo_gui
   * Bugfix when zooming all images to same region when the region is
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

point2dem:
   * Use outlier filtering when computing the bounding box of a DEM.
     The same option ``--remove-outliers-params`` controls this
     just as for removing outliers by triangulation error.

mapproject:
   * Fixed a bug when finding the extent of the mapprojected
     image when the DEM to project onto spans the whole planet.

point2mesh:
   * Only meshes in .obj format are created. This format can be opened
     in Meshlab, Blender, or some other mesh viewer.
   * The osgviewer program is no longer shipped.
   * Fixed a bug with invalid points not being filtered.
   * Fixed a bug with insufficient precision (now it can be set 
     by the user and defaults to 17 digits).
   * Added the option --texture-step-size to control the sampling
     rate for the texture, in addition to the -s option that controls
     the sampling rate for the point cloud.

Misc
   * Updated to C++ 11.
   * The Linux build system upgraded to CentOS 7.6, using conda
     for many dependencies, with gcc 5, glibc 2.17, and Python version
     >= 3 or 2.7. Tested on SUSE Linux Enterprise Server 12 and Ubuntu
     18.04.
   * Added phase subpixel correlation accuracy parameter.
   * Updated documentation to ReStructured Text, and Sphinx-Doc.
   * As of this release, we have transitioned to the `Semantic Versioning 2.0.0
     standard <https://semver.org>`_ for ASP.

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
     via --input-camera and create a most-similar pinhole camera
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
     with --initial-transform-from-hillshading <transform type>.
     Supported transforms are: 'similarity' (rotation + translation +
     scale), 'rigid' (rotation + translation) and 'translation'.
   * Added the expression of the Euler angles in the North-East-Down
     coordinate system around the center of gravity of the source
     cloud.
   * Bugfix: intersection of bounding boxes of the clouds takes
     into account the initial transform applied to the source points.
   * Added a new alignment algorithm, based on 
     https://github.com/IntelVCL/FastGlobalRegistration
     It can be invoked with --alignment-method fgr. It can perform
     better than ICP when the clouds are close enough to each
     other but there is a large number of outliers, when it can
     function with very large --max-displacement. It does worse if the
     clouds need a big shift to align.

bundle_adjust
   * Two passes of bundle adjustment (with outlier filtering after
   * first pass) is now the default. 
   * The flag --skip-rough-homography is on by default as it usually 
     gives more reliable results. Use --enable-rough-homography
     to turn this option back on (useful when the footprint on the 
     ground and difference in perspective are large).
   * The flag --disable-tri-ip-filter is also the default as input
     cameras may not be reliable enough for this filter. Can be 
     enabled back with --enable-tri-ip-filter.
   * Added the --intrinsics-limits option to manually specify 
     intrinsic parameter limits.
   * Added the --num-random-passes option to allow repeat solving 
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
   * Added the flag --reference-terrain-weight which, when increased,
     helps align better camera images to a given reference terrain. 
   * Added the option --heights-from-dem. It is very helpful in 
     determining an unknown focal length and distortion parameters
     for pinhole cameras.
     It can be used together with ---heights-from-dem-weight.
   * Bug fix in outlier filtering for n images.
   * Updated Ceres version from 1.11 to 1.14. When optimizing with 
     multiple threads, results now vary slightly from run to run.
     Results from single threaded runs are deterministic.
   * Added a new --parameter-tolerance option. Stop when the relative
     error in the variables being optimized is less than this.
   * Documented the ability to create a roughly positioned 
     pinhole camera model from an image if its intrinsics and the 
     longitude and latitude (and optionally height) of its corners
     (or some other pixels) are known.
   * When multiple passes happen with outliers removed, match files
     are not over-written, but a new clean copy of them gets saved.
   * Renamed --create-pinhole-cameras to --inline-adjustments, and 
     distortion_params to other_intrinsics. This is needed since
     for the panoramic model there will be other intrinsic
     parameters as well.
   * Added the option --forced-triangulation-distance for when one
     really needs to triangulate with poor cameras. Can be used with 
     --min-triangulation-angle 0.
   * Added the option --transform-cameras-using-gcp. If there
     are at least two images with each having at least 3 GCP
     (each GCP need not show in more than one image), use this
     to convert cameras from an abstract coordinate system to world
     coordinates.
   * Increased the default --num-ransac-iterations to 1000 from 100
     so that the solver tries harder to find a fit.
     Increased default --ip-inlier-factor from 1/15 to 0.2 to help
     with getting more interest points for steep terrain with the
     pinhole session.
   * Increased the default --ip-uniqueness-threshold from 0.7 
     to 0.8 to allow for more interest points.
   * Option to filter interest points by elevation limit and lon-lat limit
     after each pass of bundle adjustment except the last.

dem_mosaic
   * Added normalized median absolute deviation (NMAD) output option.
   * Added the option --force-projwin to create a mosaic filling
     precisely the desired box specified via --t_projwin.

stereo_gui
   * Added the ability to manually reposition interest points.
   * Can now show non-synchronous .match files (that is, each IP
     need not be present in all images).
   * Added basic functionality for drawing/editing/merging polygons on
   * top of georeferenced images or DEMs. The polygons can be saved as 
     shape files, and then used to cut out portions of images with GDAL.
   * Added the option --nodata-value. Pixels with value less than 
     or equal to this are shown as transparent.
   * Added the ability to view .vwip files (specify one per image).
   * Can view (but not edit) GCP files, via --gcp-file (creating
     GCP is supported in a separate mode, per the doc).
   * The option --dem-file specifies a DEM to use when creating
     manually picked GCP and --gcp-file specifies the name of 
     the GCP file to use upon saving such GCP.

mapproject
   * Added the --nearest-neighbor option to use that interpolation
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
     are invoked with --force-reuse-match-files.
   * Added a fix to make stereo work with the ZY3 satellite.
   * For stereo and bundle_adjust, added the --no-datum option to
     find interest points without assuming a reliable datum exists,
     such as for irregularly shaped bodies. Added the related
     option --skip-rough-homography to not use the datum in
     rough homography computation. Added the option
     --ip-num-ransac-iterations for finer control of interest
     point matching. Added --ip-triangulation-max-error to control
     the triangulation error.
   * The cam2rpc tool accepts --t_srs and --semi-major-axis as
     alternatives to --datum and --dem-file.
   * Add option --theia-overrides to camera_solve to make it easier
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
      DG and SPOT5 camera models. This can be manually disabled
      using the "--disable-correct-atmospheric-refraction" option.
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
   * Added option --corr-search-limit to constrain the automatically
     computed correlation search range.
   * Added --corr-memory-limit-mb option to limit the memory usage of
     the SGM/MGM algorithms.
   * Improved search range estimation in nadir epipolar alignment
     cases. Added --elevation-limit option to help constrain this
     search range.
   * Added hybrid SGM/MGM stereo option.
   * Improvements to SGM search range estimation.
   * Added --min-num-ip option.

bundle_adjust
   * Added the ability to optimize pinhole camera intrinsic
     parameters, with and without having a LIDAR or DEM ground truth
     to be used as reference (the latter is recommended though).
   * The tool is a lot more sensitive now to --camera-weight,
     existing results may change a lot. 
   * Added the parameters --rotation-weight and --translation-weight
     to penalize large rotation and translation changes.
   * Added the option --fixed-camera-indices to keep some cameras
     fixed while optimizing others. 
   * Can read the adjustments from a previous invocation of this
     program via --input-adjustments-prefix.
   * Can read each of pc_align's output transforms and apply it
     to the input cameras via --initial-transform, to be able to 
     bring the cameras in the same coordinate system as the aligned
     terrain (the initial transform can have a rotation, translation,
     and scale). If --input-adjustments-prefix is specified as well,
     the input adjustments are read first, and the pc_align 
     transform is applied on top.
   * Renamed --local-pinhole to --create-pinhole-cameras.
   * Added the parameter --nodata-value to ignore pixels at and below
     a threshold.
   * Added the ability to transfer interest points manually picked in
     mapprojected images to the the original unprojected images via
     --mapprojected-data.  
   * Added the flag --use-lon-lat-height-gcp-error. Then, if using
     GCP, the three standard deviations are interpreted as applying
     not to x, y, z but to latitude, longitude, and height above
     datum (in this order). Hence, if the latitude and longitude are
     known accurately, while the height less so, the third standard
     deviation can be set to something much larger.
   * Added the ability to do multiple passes of bundle adjustment,
     removing outliers at each pass based on reprojection error and
     disparity (difference of pixel value between images). This
     works for any number of cameras. Match files are updated with
     outliers removed. Controlled via --num-passes,
     --remove-outliers-params and --remove-outliers-by-disparity-params.
   * Added the option --save-cnet-as-csv, to save the control
     network containing all interest points in the format used by
     ground control points, so it can be inspected.
   * If --datum is specified, bundle_adjust will save to disk
     the reprojection errors before and after optimization. 

stereo_gui
   * Can view SPOT5 .BIL files.

pc_align
   * Add the ability to help the tool with an initial translation
     specified as a North-East-Down vector, to be used to correct known
     gross offsets before proceeding with alignment. The option is
     --initial-ned-translation.
   * When pc_align is initialized via --initial-transform or
     --initial-ned-translation, the translation vector is now computed
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
     and percentile filters. The --search-radius-factor parameter can
     control the neighborhood size.
   * Sped up hole-filling in ortho image generation. If this creates
     more holes than before, it is suggested to relax all outlier filtering,
     including via --remove-outliers-params, median filtering, and erosion. 
   * Added the option --orthoimage-hole-fill-extra-len to make hole-filling
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
     the --ot option, to round the output pixels to several types of
     integer, reducing storage, but perhaps decreasing accuracy.
   * The tools mapproject and image_calc support the --mo option to
     add metadata to the geoheader in the format 'VAR1=VAL1 VAR2=VAL2',
     etc.
   * Handle properly in bundle_adjust, orbitviz, and stereo 
     with mapprojected images the case when, for RPC cameras,
     these coefficients are stored in _RPC.TXT files.
   * Support for web-based PROJ.4 strings, e.g., 
     point2dem --t_srs http://spatialreference.org/ref/iau2000/49900/
   * Added --max-output-size option to point2dem to prevent against
     creation of too large DEMs.
   * Added image download option in hiedr2mosaic.py.
   * Bugfix in cam2map4stereo.py when the longitude crosses 180 degrees.
   * Added support for running sparse_disp with your own Python installation.
   * Bugfix for image cropping with epipolar aligned images.
   * The sfs tool supports the integrability constraint weight from Horn 1990.
   * The software works with both Python versions >= 2.6 and 3. 

RELEASE 2.6.0, May 15, 2017
---------------------------

New stereo algorithms
   * ASP now supports the Semi Global Matching (SGM) and 
     More Global Matching (MGM) stereo algorithms. 
     They do particularly well for Earth imagery, better 
     than the present approaches. They can be invoked with 
     --stereo-algorithm 1 and 2 respectively. 

New tools
    * Added cam2rpc, a tool to create an RPC model from any
      ASP-supported camera. Such cameras can be used with ASP for
      Earth and planetary data (stereo's --datum option must be set),
      or passed to third-party stereo tools S2P and SETSM. 
    * Added correct_icebridge_l3_dem for IceBridge.
    * Added fetch_icebridge_data for IceBridge.

parallel_stereo
   * By default, use as many processes as there are cores, and one
     thread per processes.
     
stereo_pprc
   * Large speedup in epipolar alignment.
   * Improved epipolar alignment quality with standard pinhole cameras.
   * Added the options --ip-inlier-threshold and --ip-uniqueness-threshold
     for finer-grained control over interest point generation.
   * Fix a bug with interest point matching the camera model is RPC
     and the RPC approximation domain does not intersect the datum.
  
stereo_corr
   * Added new option --stereo-algorithm.  Choices 1 and 2 replaces
     the standard integer correlator with a new semi-global matching 
     (SGM) correlator or an MGM correlator respectively.  SGM/MGM is
     slow and memory intensive but it can produce better results
     for some challenging input images, especially for IceBridge.
     See the manual for more details.

stereo_tri
  * Added the option --min-triangulation-angle to not triangulate
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
   * Added the option --dem-blur-sigma to blur the output DEM.
   * Use by default --weights-exponent 2 to improve the blending,
     and increase this to 3 if --priority-blending-length is specified.
   * Added the options --tile-list, --block-max, and --nodata-threshold. 
   * Display the number of valid pixels written. 
   * Do not write empty tiles. 

geodiff
   * One of the two input files can be in CSV format.

dg_mosaic
    * Save on output the mean values for MEANSUNEL, MEANSUNAZ,
      and a few more.

point2dem
     * Added the parameter --gaussian-sigma-factor to control the 
       Gaussian kernel width when creating a DEM (to be used together
       with --search-radius-factor).

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
      --local-pinhole, --solve-intrinsics, --intrinsics-to-float.
    * Added the option --overlap-list. It can be used to specify which
      image pairs are expected to overlap and hence to be used to
      compute matches.
    * Added the option --initial-transform to initialize the adjustments
      based on a 4x4 rotation + translation transform, such as coming
      from pc_align. 
    * Added the options --ip-inlier-threshold and --ip-uniqueness-threshold
      for finer-grained control over interest point generation.

pc_align
   * Can solve for a rotation + translation or for rotation +
     translation + scale using least squares instead of ICP, if the
     first cloud is a DEM. It is suggested that the input clouds be 
     very close or otherwise the --initial-transform option be used,
     for the method to converge. The option is:
     --alignment-method [ least-squares | similarity-least-squares ]

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
   bundle_adjust, and map_project using a rigorous linescan camera model.
 - Added the add_spot_rpc tool to create RPC models for SPOT5
   which allows them to be map projected with the RPC model.

pc_align 
   * Can solve for a scale change in addition to a rotation and translation 
     to best align two clouds, hence for a similarity transform. 
     Option: --alignment-method similarity-point-to-point

mapproject
   * Added ability to map project color images.
   * Added option to map project on to a flat datum.

camera_solve
   * Added option to accept multiple input camera models.

Other:

dem_mosaic
   * Fix a bug with mosaicking of DEMs over very large extent.
   * Fix a bug with 360 degree longitude offset.
   * Added the option --use-centerline-weights. It will compute
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
   * Fixed a bug in pc_align which caused the --max-disp argument to be misread
     in some situations.
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
     can greatly slow down a run. The options are --rm-quantile-percentile
     and --rm-quantile-multiple. 

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
    * Bugfix for stereo with map-projected images using the RPC
      session (e.g, for map-projected Pleiades imagery).
    * Added OpenCV-based SIFT and ORB interest point finding options.

bundle_adjust
    * Much improved convergence for Digital Globe cameras.
    * Added OpenCV-based SIFT and ORB interest point finding options.

point2dem, point2las, and pc_align
   * The datum (-r <planet> or --semi-major-axis) is optional now.
     The planet will be inferred automatically (together with the
     projection) from the input images if present. This can be useful
     for bodies that are not Moon, Mars, or Earth. The datum and
     projection can still be overridden with --reference-spheroid (or
     --datum) and --t_srs. 

dem_mosaic
   * Introduce --priority-blending-length, measured in input pixels. 
     If positive, keep unmodified values from the earliest available
     DEM at the current location except a band this wide near its
     boundary where blending will happen. Meant to be used with 
     smaller high-resolution "foreground" DEMs and larger
     lower-resolution "background" DEMs that should be specified later
     in the list. Changing --weights-exponent can improve transition.

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
  * Accept --datum (-r) MOLA, as a shortcut for the sphere with
     radius 3,396,000 meters.

dem_mosaic
   * Fix an issue with minor jumps across tiles. 
   * Introduce --save-dem-weight <index>. Saves the weight image that
     tracks how much the input DEM with given index contributed to the
     output mosaic at each pixel (smallest index is 0).
   * Introduce --save-index-map. For each output pixel, save the
     index of the input DEM it came from (applicable only for
     --first, --last, --min, and --max). A text file with the index
     assigned to each input DEM is saved as well.
   * Rename --blending-length to --extra-crop-length, for clarity. 

dg_mosaic 
   * Added the switch --fix-seams to use interest point matching
     to fix seams in the output mosaic due to inconsistencies between
     image and camera data. Such artifacts may show up in older
     (2009 or earlier) Digital Globe images.

stereo_gui
   * Added the option --match-file to view interest point matches.
   * Added the options --delete-temporary-files-on-exit and
     --create-image-pyramids-only.
   * Can read the georeference of map-projected ISIS cubes.

point2dem
   * Respect --t_projwin to the letter. 
   * Can create simultaneously DEMs at multiple resolutions (by
     passing multiple values in quotes to --dem-spacing).
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
   * Bugfix when the two DEMs have longitudes offset by 360 degrees.

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
   * Added --right-image-crop-win in addition to --left-image-crop-win.
     If both are specified, stereo crops both images to desired regions
     before running stereo (this is different from when only 
     --left-image-crop-win is specified, as then no actual cropping 
     happens, the domain of computation is just restricted to the desired
     area). 
   * Bugfix, remove outliers during search range determination.
   * Added the option --ip-per-tile, to search for more interest points 
     if the default is insufficient.
   * If the input images are georeferenced, the good pixel map will be
     written with a georeference.
 
point2dem
   * Fixed a slight discrepancy in the value of the semi-minor axis in
     the WGS84 and NAD83 datum implementations.
   * Added the option --median-filter-params <window size> <threshold> to
     remove spikes using a median filter.
   * Added the option --erode-length <num> to erode pixels from point cloud 
     boundary (after outliers are removed, but before filling in holes).
   * Improved hole-filling, and removed the --hole-fill-mode and 
     --hole-fill-num-smooth-iter, as there's only one algorithm now. 
   * Improved performance when large holes are to be filled.
   * Can create a DEM from point clouds stored in CSV files containing
     easting, northing, and height above datum (the PROJ.4 string
     needed to interpret these numbers should be set with --csv-proj4).
   * Fixed a bug in creating DEMs from CSV files when different projections
     are used on input and output.
   * Expose to user gnomonic and oblique stereographic projections,
     as well as false easting and false northing (where applicable). 
     This is a shortcut from using explicitly t_srs for the PROJ.4 string.
   * The default no-data value is set to the smallest float.
 
pc_align
   * Can ingest CSV files containing easting, northing, and height
     above datum (the PROJ.4 string needed to interpret these numbers
     should be set with --csv-proj4).
   * If the reference point cloud is a DEM, the initial and final errors
     in the statistics, as well as gross outlier removal, are done using
     a new distance function. Instead of finding the distance from a 3D 
     point to the closest point in the cloud, the 3D point is projected 
     onto DEM's datum, its longitude and latitude are found, the
     height in the DEM is interpolated, and and the obtained point on the 
     DEM is declared to be the closest point. This is more accurate
     than the original implementation for coarse DEMs. The old 
     approach is available using the --no-dem-distances flag.
   * Fix a bug with a 360 degree longitude offset.

point2las
   * Added the ability to specify a custom projection (PROJ.4 string)
     for output LAS files.

dem_mosaic
   * Write GeoTIFF files with blocks of size 256 x 256 as those
     may be faster to process with GDAL tools.
   * Bug fix when the tool is used to re-project.
   * Added the option --weights-blur-sigma <num> to allow the blending
     weights to be blurred by a Gaussian to increase their smoothness.
   * Added the option --weight-exponent <num>, to allow weights
     to increase faster than linearly.
   * Added --stddev option to compute standard deviation.
   * Added the ability to fill holes in the output mosaic.

bundle_adjust
    * Added new parameters, --ip-per-tile and --min-triangulation-angle.
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
   * Bugfix, the mapprojected image should not go much beyond the DEM
     it is mapprojected onto (where it would have no valid pixels).

dg_mosaic
   * Default penalty weight produces a more accurate fit when creating an 
     RPC model from a DG model.
   * Handle the situation when two images to be mosaicked start at the 
     same output row number.
   * Added --target-resolution option to specify the output resolution in meters.

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
   * Support for multi-band (multi-spectral) images. Use --band <num>
     to pick a band to mosaic.
      
stereo
   * Bugfix in interest point matching in certain circumstances.
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
     the percentage in --remove-outliers-params to 100.
 
bundle_adjust
   * Use multiple-threads for non-ISIS sessions.
   * Added the parameter --overlap-limit <num> to limit the number 
     of subsequent images to search for matches to the current image.
   * Added the parameter --camera-weight <val>, to set the weight to
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
    * Bugfix when saving LAS files in respect to a datum.

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
      disparity with --erode-max-size.

stereo_tri
    * Bugfixes for MER cameras.

stereo_tri and mapproject
    * Added the option --bundle-adjust-prefix to read adjusted
      camera models obtained by previously running bundle_adjust with
      this output prefix.

point2las
    * LAS files can be saved in geo-referenced format in respect 
      to a specified datum (option --reference-spheroid).
 
point2dem
    * Bugfix, longitude could be off by 360 degrees.
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
     * Added the --skip-image-normalization option (for non-ISIS 
       images and alignment-method none), it can help with reducing
       the size of data on disk and performance.
       
stereo_rfne
     * Added new affine subpixel refinement mode when 
       --subpixel-mode = 3. This mode sacrifices the error resistance
       of Bayes EM mode in exchange for reduced computation time.
       For some data sets this can perform as well as Bayes EM in
       about one fifth the time.

stereo_fltr:
     * Hole-filling is disabled by default in stereo_fltr. It is 
       suggested to use instead point2dem's analogous functionality.
       It can be re-enabled using --enable-fill-holes.
     * Added the option --erode-max-size to remove isolated blobs.
     * Relaxed filtering of disparities, retaining more valid
       disparities. Can be adjusted with --filter-mode and related
       parameters.

stereo_tri:
    * Added ability to save triangulation error for a DEM as a 3D
      North-East-Down vector rather than just its magnitude.
    * When acting on map-projected images, handle the case when the 
      DEM used for map-projection does not completely encompass the 
      images.
 
pc_align:
    * Read and write CSV files in a wide variety of formats, using 
      the --csv-format option.
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
       Gaussian can be controlled using the --search-radius-factor
       option. The old algorithm is still available (but obsoleted)
       using the --use-surface-sampling option. The new algorithm
       makes the --fsaa option redundant. 
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
  libpointmacher library
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
  (http://byss.arc.nasa.gov/stereopipeline/daily_build/). When
  requesting support, please provide the output of "stereo --version".

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

- point2dem with the --fsaa option for reducing aliasing at
  low-resolution DEM generation has been improved as to remove the
  erosion of of valid data close to no-data values.

- Bugfixes for parallel_stereo, point2dem, etc. 

RELEASE 2.2.2, 17 MAY 2013
--------------------------
(incremented from 2.2.1 after one more bugfix)

TOOLS:

- stereo_mpi renamed to parallel_stereo and made to work
  on any machines with shared storage, rather than just on 
  supercomputers using Intel's MPI library. Bug fixes for
  homography and affine epipolar alignment modes, etc.

- Bugfix for dem_geoid path to geoids, more robust datum
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
  instead of LUTs to reverse map projections and alignments.

TOOLS:

- Added dem_geoid, dg_mosaic, and stereo_mpi.
- Added new interest point matching method to stereo.
- Added new DEM seed mode for stereo.
- Point2dem sped up by reducing over rasterization of triangles.
- Stereo_corr has local_homography option. Homography transform
  applied per tile.
- Fix point2dem where for certain projections we were setting K=0.
- Stereo can now operate on terminal only arguments without stereo.default.

RELEASE 2.1.0, 8 JANUARY 2013
-----------------------------

GENERAL:

- Added documentation for processing GeoEye, Digital Globe, and Dawn FC data.
- Fixed implementation of internal RANSAC function.
- DEMError has been renamed IntersectionErr. 3D IntersectionErr is
  now recordable in local North East Down format.

SESSIONS:

- Added RPC processing session.
- DG sessions now use bicubic interpolation for map projection arithmetic.
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
- Added --t_srs option to point2dem.
- Added rpc_mapproject tool. This provides an optional map_projection
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
- Improve triangulation times slightly when using map projected
  linescan cameras.

EXAMPLES:

- Added orthoproject, image2qtree, colormap, hillshade examples to MOC.
- Added K10 example dataset.
- Added MER example dataset.
- Added a non-map projected MOC example.
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
  map projected.
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
