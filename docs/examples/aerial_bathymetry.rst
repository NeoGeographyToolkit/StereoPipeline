.. _aerial_bathymetry:

Aerial images for shallow-water bathymetry
------------------------------------------

This is an end-to-end example for how to produce bathymetry-corrected
(:numref:`bathy_intro`) digital elevation models (DEM) with images from an
airborne frame-camera survey.

The images were created with a Leica RCD30 camera flown over the Gulf coast near
Sarasota, Florida, at about 2300 m above the water, with a ground sample
distance of about 0.23 m.

Vendor metadata
~~~~~~~~~~~~~~~

The data came with an exterior-orientation table (extrinsics). Each line
has the image name, the camera position in a projected coordinate system, and the
orientation as omega, phi, and kappa angles.

What follows is a simplified example of such a file. The column headers and
the values in the rows below must be one-to-one, with *tabs as separators*. The
order can be variable, as the fields are found by name. Some names can have
spaces, such as ``Image ID``. Other fields (such as standard deviation) are
ignored.

Example::

    Filename     Image ID X        Y         Z      Omega  Phi    Kappa
    img_0003.tif 0        337557.0 3028849.6 2309.6 0.059  -0.071 -136.386

The interior orientation (intrinsics) are provided in an ESRI camera CSV format,
giving the focal length, pixel size, principal point, image dimensions, and lens
distortion::

    CameraModel,FocalLength,PrincipalX,PrincipalY,NRows,NCols,PixelSize,DistortionType,Radial,Tangential
    RCD30,53000,0,0,7788,10336,5.2,DistortionModel,0;0;0;0,0;0

Here the focal length and pixel size are in microns.

The RCD30 delivers already-undistorted imagery (all distortion coefficients are
zero), as is typical for a metric aerial camera. In general, the OpenCV
radial-tangential lens distortion model will be assumed, with the coefficients
in the order K1, K2, K3, P1, P2 (:numref:`pinhole_distortion`).

Neither metadata file states its coordinate system or the frame the angles are
in. Those are set by the vendor's convention, which we set below with the
``--vendor`` option.

ASP also supports parsing metadata from EXIF files (:numref:`sfm_uas`), and
providing the orientations as roll, pitch, yaw (:numref:`cam_gen_extrinsics`).

It is suggested to study such input on a case-by-case basis. Our Pinhole camera
format used for output is described in :numref:`pinholemodels`.

Extracting image bands
~~~~~~~~~~~~~~~~~~~~~~

The RCD30 frames have four bands, in the order red, green, blue, and
near-infrared (NIR). Bundle adjustment and stereo operate on single-band
images, so one band must be extracted from each frame.

Use the green band (band 2) for stereo and bundle adjustment. Green penetrates
water better than red or blue, so it shows the most texture on a shallow water
bottom. Use the NIR band (band 4) for water masking
(:numref:`aerial_bathymetry_mask`), as water is dark in the NIR.

Extract a band with ``gdal_translate``::

    gdal_translate -b 2 -co compress=lzw \
      img_0003.tif img_0003_green.tif
    gdal_translate -b 4 -co compress=lzw \
      img_0003.tif img_0003_nir.tif

The band order can be confirmed with ``gdalinfo`` (:numref:`gdal_tools`).

List the green-band images in ``images.txt``, one per line. That list is used
for camera creation, bundle adjustment, and stereo below. In practice a short
shell loop over all frames does the extraction and writes this list.

Creation of camera models
~~~~~~~~~~~~~~~~~~~~~~~~~

The following creates one ASP Pinhole camera per image. This requires
build 2026/09/10 (:numref:`release`) or later::

    cam_gen --vendor esri                  \
      --extrinsics RCD30_2026_eop.txt      \
      --intrinsics RCD30_2026_cam_esri.csv \
      --image-list images.txt              \
      --output-dir cameras                 \
      --t_srs EPSG:6346

Here ``images.txt`` lists the input images (one per line). Each is matched to an
exterior-orientation record by its file name. The program writes one ``.tsai``
camera per image into the directory ``cameras``, and saves the list of those
cameras, in *the same order* as ``images.txt``, to ``cameras/camera_list.txt``. That
list is passed later to ``bundle_adjust`` (:numref:`bundle_adjust`).

The value of ``--t_srs`` is the projected coordinate system of the positions in
the exterior-orientation file, given as a PROJ, WKT, or EPSG string. Here it is
NAD83(2011), epoch 2010.0, UTM zone 17N (``EPSG:6346``), the datum this survey
and its reference lidar were delivered in. The datum *cannot be inferred* from the
easting and northing alone, so it must be provided. Only the ESRI convention is
supported at this time.

The NAD83(2011) and WGS84 datums differ by about one to two meters in the
continental United States, mostly horizontally. The two share the same
ellipsoid to well under a millimeter, and ASP applies no transform between them.

All commands in this document would work equally well with UTM zone 17N on the
WGS84 datum (``EPSG:32617``). It is suggested to carefully read any vendor
documentation and evaluate any output products for potential coordinate system
mix-ups.

For the ESRI convention the omega, phi, and kappa angles are referenced to the
projected grid, so the grid axes are not aligned with true north away from the
central meridian. ``cam_gen`` accounts for this grid-to-true-north convergence
automatically, computing it from the coordinate system at each camera.

Getting the angle convention wrong produces a constant rotation of every camera
about its optical axis, which is easy to miss in a summary statistic. A visual
validation strategy is described in :numref:`aerial_bathymetry_validate`.

.. _aerial_bathymetry_refdem:

A reference terrain
~~~~~~~~~~~~~~~~~~~

Validation and bundle adjustment both need a prior terrain over the area. A free
global option is the Copernicus 30 m DEM. Its heights are relative to the EGM2008
geoid, so they must be converted to WGS84 ellipsoid heights with ``dem_geoid``
(:numref:`dem_geoid`) before use, as discussed in :numref:`initial_terrain`.

Where available, the USGS 3DEP lidar DEM is a much finer alternative (about 1
m). Like most published USGS products, its heights are relative to the NAVD88
geoid (orthometric heights), not the ellipsoid. All heights in this pipeline,
the cameras and the stereo output, are relative to the ellipsoid. The 3DEP DEM
must therefore be converted to ellipsoidal heights before use, or else
mapprojection, bundle adjustment, and validation are all off by the geoid
separation, which is about 26 m near Sarasota, Florida.

Convert it with ``dem_geoid`` (:numref:`dem_geoid`). The default direction of
that tool subtracts the geoid to produce orthometric heights, so here the
``--reverse-adjustment`` option is used to go the other way, adding the geoid
to produce ellipsoidal heights::

    dem_geoid              \
      3dep_dem.tif         \
      --reverse-adjustment \
      -o 3dep_dem

This writes ``3dep_dem-adj.tif``. Use that ellipsoidal DEM as ``ref_dem.tif`` in
the commands below. For NAD83 the NAVD88 geoid is the default, so no geoid needs
to be specified.

.. figure:: ../images/examples/threedep_colorhs.png
   :name: aerial_bathymetry_3dep
   :width: 60%

   The USGS 3DEP lidar DEM over the site, as a terrain-colored hillshade. The
   barrier island, circular canal development, and bay islands are resolved. Blue
   is low (water and bay), green to tan to white is rising land. Water is flat
   fill, so the bay shows tile-boundary blocks.

.. _aerial_bathymetry_validate:

Validating the input cameras
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before bundle adjustment, confirm that the cameras place the imagery correctly
on the ground. Mapproject a frame onto the reference DEM with its created camera
(:numref:`mapproject`)::

    mapproject                             \
      --t_srs EPSG:6346                    \
      ref_dem.tif                          \
      20251113_155312_032_003.tif          \
      cameras/20251113_155312_032_003.tsai \
      frame_map.tif

Then overlay the mapprojected frame on the DEM's hillshade, for example in
``stereo_gui`` (:numref:`stereo_gui`). If the cameras are right, the landmarks
will agree.

.. figure:: ../images/examples/validation_overlay.png
   :name: aerial_bathymetry_overlay

   Left: a frame mapprojected with its ``cam_gen`` camera. Center: the 3DEP lidar
   DEM hillshade over the same area. Right: the two overlaid.

This check is strongly suggested. Vendors differ in their angle and coordinate
conventions, and a wrong convention can result in gross misalignment.

.. _aerial_bathymetry_ba:

Bundle adjustment
~~~~~~~~~~~~~~~~~

Modern aerial camera systems have reasonably good geolocation and pointing
information. Bundle adjustment (:numref:`bundle_adjust`) refines the cameras to
tighten their consistency.

We found that *not* modeling bathymetry at this stage is acceptable. Most rays
are seen in a single pair of images, and they still geometrically intersect when
meeting under water, even without taking into account refraction, just at a
shallower point. It also helps that the underlying solver uses a robust
threshold (:numref:`ba_optim`) that attenuates any outliers.

We employ :ref:`parallel_bundle_adjust` to parallelize the finding of interest
point matches::

    parallel_bundle_adjust                   \
      --image-list  images.txt               \
      --camera-list cameras/camera_list.txt  \
      --inline-adjustments                   \
      --auto-overlap-params 'ref_dem.tif 15' \
      --min-triangulation-angle 1e-10        \
      --forced-triangulation-distance 2000   \
      --camera-position-uncertainty 100,100  \
      --num-iterations 100                   \
      --num-passes 2                         \
      -o ba/run

Ensure that the image and camera lists are in *the same order*.

The option ``--auto-overlap-params`` uses the prior DEM to decide which images
overlap, rather than trying all pairs (:numref:`ba_options`).

The options ``--min-triangulation-angle`` and
``--forced-triangulation-distance`` (set to roughly the camera height above the
ground, in meters) ensure that triangulated points between cameras with very
little perspective difference are not filtered out.

The value of ``--camera-position-uncertainty`` (here 100 m in the horizontal and
vertical) prevents large movements in camera positions. Too tight a constraint
can prevent convergence.

Inspect the initial and final reprojection errors in the ``pointmap.csv`` files
(:numref:`ba_err_per_point`). Inspect how much the camera positions and
triangulated points moved, as well as the pixel reprojection per camera
(:numref:`ba_out_files`). Pixel errors should be well under a pixel if lens
distortion is modeled correctly. For this dataset, the camera positions moved by
about 0.2 m and the final pixel reprojection error was about 0.3 pixels.

.. _aerial_bathymetry_stereo:

DEM creation
~~~~~~~~~~~~

Unlike with satellite data, a collection of aerial images has many overlapping
stereo pairs. Bundle adjustment writes a report with the pairwise stereo
convergence angle for all overlapping image pairs (:numref:`ba_conv_angle`).

The ``multi_stereo`` program (:numref:`multi_stereo`) automatically runs
pairwise stereo between the image pairs whose convergence angle is within
reasonable bounds (here between 15 and 45 degrees, see also
:numref:`stereo_pairs`).

This requires build 2026/09/10 (:numref:`release`) or later.

Set up the stereo and ``point2dem`` options::

    stereoOpts="--stereo-algorithm asp_mgm --subpixel-mode 9"
    demOpts="--tr 0.9 --t_srs EPSG:6346 --errorimage --orthoimage"

Then run stereo and mosaic the results::

    multi_stereo                           \
      --mode dem_mosaic                    \
      --image-list  ba/run-image_list.txt  \
      --camera-list ba/run-camera_list.txt \
      --conv-angle-list                    \
        ba/run-convergence_angles.txt      \
      --conv-angle-range 15,45             \
      --processes 8                        \
      --threads 4                          \
      --stereo-options "$stereoOpts"       \
      --point2dem-options "$demOpts"       \
      --output-prefix stereo/run

This writes a DEM (:numref:`point2dem`) named ``stereo/run-DEM.tif``, a
triangulation error image (:numref:`triangulation_error`) named
``stereo/run-IntersectionErr.tif``, and an orthoimage (DRG)
at the DEM grid named ``stereo/run-DRG.tif``.

The image and camera lists ``ba/run-image_list.txt`` and
``ba/run-camera_list.txt`` are written by ``bundle_adjust``
(:numref:`ba_out_cams`). The stereo pairs come from the convergence angle report
``ba/run-convergence_angles.txt`` (:numref:`ba_conv_angle`): each pair whose
median convergence angle is within ``--conv-angle-range`` is used.

We set the DEM grid to 0.9 m, about four times the 0.23 m ground sample
distance. The projection is the same as the cameras (here ``EPSG:6346``).

If desired to use mapprojected images (:numref:`mapproj-example`), mapproject
every image with the same grid size (option ``--tr``, :numref:`mapproject`) and
the same projection.

If the input images are mapprojected, add the option ``--dem`` that points to
the DEM for mapprojection (:numref:`multi_stereo_dem_mosaic`).

Set the option ``--nodes-list`` to run on multiple machines
(:numref:`pbs_slurm`).

.. figure:: ../images/examples/aerial_dem_vs_3dep.png
   :name: aerial_dem_3dep
   :width: 100%

   Left: the color-hillshaded mosaicked stereo DEM. Right: the USGS 3DEP lidar
   DEM over the same window, on the same grid, shifted up by about 2 m to match
   this DEM's vertical level (the horizontal registration is already good, and
   only a near-constant vertical offset remains). The developed area agrees in
   both. Areas in deep water are unreliable.

The orthoimage above is at the DEM grid (0.9 m). For one at the full 0.23 m native
ground sample distance, mapproject each image onto the mosaic DEM::

    mapproject --tr 0.23   \
        stereo/run-DEM.tif \
        image.tif          \
        ba/run-image.tsai  \
        image_map.tif

then mosaic the results with::

    dem_mosaic --first *_map.tif -o ortho_mosaic.tif

The ``--first`` option takes the first valid pixel instead of blending, which
avoids smearing image seams and any small residual misregistration in the ortho.

.. figure:: ../images/examples/aerial_ortho_trierr.png
   :name: aerial_ortho_trierr
   :width: 100%

   Left: the orthoimage over the developed area. Right: the triangulation error.
   Over open water there is no texture, so stereo correlation finds no matches and
   the error is high there. That area can be ignored.

.. _aerial_bathymetry_mask:

Water masking
~~~~~~~~~~~~~

Shallow-water bathymetry needs a water mask. The near-infrared (NIR) band is
strongly absorbed by water, so it separates land from water cleanly. Mapproject
and mosaic the NIR band the same way as the orthoimage, then threshold it. The
threshold can be found automatically with Otsu's method or a kernel-density
estimate, both described in :numref:`bathy_thresh`. Turn the threshold into a mask
with ``image_calc`` (:numref:`image_calc`), so that land is a positive value and
water is nodata::

    image_calc -c "sign(var_0 - T)" --output-nodata-value -1 \
      nir_ortho_mosaic.tif -o ortho_water_mask.tif

Here ``T`` is the threshold. Land (NIR above the threshold) becomes positive, and
water (at or below the threshold) becomes nodata.

.. figure:: ../images/examples/aerial_nir_mask.png
   :name: aerial_nir_mask
   :width: 100%

   Left: the mosaicked NIR orthoimage. Water is dark, land is bright. Right: the
   water mask derived from it. Land is kept (green), water is dropped (light). The
   canal interiors are correctly classified as water.

.. _aerial_bathymetry_correct:

Bathymetry correction
~~~~~~~~~~~~~~~~~~~~~

The stereo above ignores refraction at the water surface, so underwater terrain is
too shallow. The bathymetry correction of :numref:`bathy_intro` fixes this. First,
fit one water-surface plane over the whole dataset with ``bathy_plane_calc``
(:numref:`water_surface`), using the global ortho mask and the DEM::

    bathy_plane_calc              \
      --mask ortho_water_mask.tif \
      --dem stereo/run-DEM.tif    \
      --output-plane bathy_plane.txt

Because there is a single mosaicked orthoimage, one global mask and one plane serve
the entire dataset. This is simpler than the per-image left and right masks used for
a single pair in :numref:`bathy_mask_creation`.

Then run the same ``multi_stereo`` command as above, with the water-surface plane,
the saltwater refraction index, and the global water mask added to
``--stereo-options`` (:numref:`bathy_intro`), and a new ``--output-prefix``. The global
ortho mask is passed with ``--ortho-bathy-mask`` (in place of the per-image masks of
:numref:`bathy_mask_creation`)::

    stereoOpts="--stereo-algorithm asp_mgm
                --subpixel-mode 9
                --ortho-bathy-mask ortho_water_mask.tif
                --bathy-plane bathy_plane.txt
                --refraction-index 1.34"
    demOpts="--tr 0.9 --t_srs EPSG:6346
             --errorimage --orthoimage"

    multi_stereo                           \
      --mode dem_mosaic                    \
      --image-list  ba/run-image_list.txt  \
      --camera-list ba/run-camera_list.txt \
      --conv-angle-list                    \
        ba/run-convergence_angles.txt      \
      --conv-angle-range 15,45             \
      --processes 8                        \
      --threads 4                          \
      --stereo-options "$stereoOpts"       \
      --point2dem-options "$demOpts"       \
      --output-prefix stereo_bathy/run

This creates the bathymetry-corrected DEM named ``stereo_bathy/run-DEM.tif``,
and other products as before.

.. figure:: ../images/examples/aerial_bathy_deepen.png
   :name: aerial_bathy_deepen
   :width: 100%

   Left: the bathymetry-corrected DEM, as a terrain-colored hillshade. Right: the
   change from the correction, computed as the corrected DEM minus the DEM before
   correction. Blue is where the water bottom moved deeper, the expected
   refraction signature. Land is unchanged (pale). The value is clamped to 1.5 m.
   The correction deepens the shallow water by up to 1 m or so.
