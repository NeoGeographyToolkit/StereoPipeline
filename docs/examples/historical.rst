.. _kh4:

Declassified satellite images: KH-4B
------------------------------------

ASP has support for the declassified high-resolution CORONA KH-4B images. 
This support is very experimental, and the user is advised to use caution.

These images can be processed using either optical bar (panoramic) camera
models or as pinhole camera models with RPC distortion. Most of the
steps are similar to the example in :numref:`skysat-example`.
The optical bar camera model is based on
:cite:`schenk2003rigorous` and
:cite:`sohn2004mathematical`, whose format is described in
:numref:`panoramic`.

Fetching the data
~~~~~~~~~~~~~~~~~

KH-4B images are available via the USGS Earth Explorer, at

https://earthexplorer.usgs.gov/

(an account is required to download the data). We will work with the
KH-4B image pair::

    DS1105-2248DF076
    DS1105-2248DA082

To get these from Earth Explorer, click on the ``Data Sets`` tab and
select the three types of declassified data available, then in the
``Additional Criteria`` tab choose ``Declass 1``, and in the
``Entity ID`` field in that tab paste the above frames (if no results
are returned, one can attempt switching above to ``Declass 2``, etc).
Clicking on the ``Results`` tab presents the user with information about
these frames.

Clicking on ``Show Metadata and Browse`` for every image will pop-up a
table with meta-information. That one can be pasted into a text file,
named for example, ``DS1105-2248DF076.txt`` for the first image, from
which later the longitude and latitude of each image corner will be
parsed. Then one can click on ``Download Options`` to download the data.

.. _resizing_images:

Resizing the images
~~~~~~~~~~~~~~~~~~~

Sometimes the input images can be so large, that either the ASP tools
or the auxiliary ImageMagick ``convert`` program will fail, or the machine
will run out of memory. 

It is suggested to resize the images to a more manageable size, at least for 
initial processing. This can be done with ``gdal_translate`` (:numref:`gdal_tools`).
as follows::

    gdal_translate -outsize 25% 25% -r average input.tif output.tif

This will reduce the image size by a factor of 4. A factor of 2 can be tried
instead. The ``-r average`` option will average the nearby pixels in the input
image, which will reduce aliasing artifacts.

A camera model (pinhole or optical bar) created at one resolution can be used at
another resolution by adjusting the ``pitch`` parameter (a higher value of pitch
means bigger pixels so lower resolution).

Stitching the images
~~~~~~~~~~~~~~~~~~~~

Each downloaded image will be made up of 2-4 portions, presumably due to
the limitations of the scanning equipment. They can be stitched together
using ASP's ``image_mosaic`` tool (:numref:`image_mosaic`).

For some reason, the KH-4B images are scanned in an unusual order. To
mosaic them, the last image must be placed first, the next to last
should be second, etc. In addition, as seen from the tables of metadata
discussed earlier, some images correspond to the ``Aft`` camera type.
Those should be rotated 180 degrees after mosaicking, hence below we use
the ``--rotate`` flag for that one. The overlap width is manually
determined by looking at two of the sub images in ``stereo_gui``.

With this in mind, image mosaicking for these two images will happen as
follows::

     image_mosaic DS1105-2248DF076_d.tif DS1105-2248DF076_c.tif \
       DS1105-2248DF076_b.tif  DS1105-2248DF076_a.tif           \
       -o DS1105-2248DF076.tif                                  \
       --ot byte --overlap-width 7000 --blend-radius 2000
     image_mosaic DS1105-2248DA082_d.tif DS1105-2248DA082_c.tif \
       DS1105-2248DA082_b.tif  DS1105-2248DA082_a.tif           \
       -o DS1105-2248DA082.tif                                  \
       --ot byte --overlap-width 7000 --blend-radius 2000       \
       --rotate

In order to process with the optical bar camera model these images need
to be cropped to remove the most of empty area around the image. The
four corners of the valid image area can be manually found by clicking
on the corners in ``stereo_gui``. Note that for some input images it can
be unclear where the proper location for the corner is due to edge
artifacts in the film. Do your best to select the image corners such
that obvious artifacts are kept out and all reasonable image sections
are kept in. 

ASP provides a simple Python tool called
``historical_helper.py`` to rotate the image so that the top edge is
horizontal while also cropping the boundaries. This tool requires
installing the ImageMagick software. See :numref:`historical_helper`
for more details.

Pass in the corner coordinates as shown below in the order top-left, top-right,
bot-right, bot-left (column then row). This is also a good opportunity to
simplify the file names going forwards.

::

     historical_helper.py rotate-crop                                     \
       --interest-points '4523 1506  114956 1450  114956 9355  4453 9408' \
       --input-path DS1105-2248DA082.tif                                  \
       --output-path aft.tif
     historical_helper.py rotate-crop                                     \
       --interest-points '6335 1093  115555 1315  115536 9205  6265 8992' \
       --input-path DS1105-2248DF076.tif                                  \
       --output-path for.tif 

See :numref:`resizing_images` if these steps failed, as perhaps the images
were too large.

Fetching a ground truth DEM
~~~~~~~~~~~~~~~~~~~~~~~~~~~

To create initial cameras to use with these images, and to later refine
and validate the terrain model made from them, we will need a ground
truth source. Several good sets of DEMs exist, including SRTM, ASTER,
and TanDEM-X. Here we will work with SRTM, which provides DEMs with a
30-meter post spacing. The bounds of the region of interest are inferred
from the tables with meta-information from above. We will use ``wget``
to fetch https://e4ftl01.cr.usgs.gov/provisional/MEaSUREs/NASADEM/Eurasia/hgt_merge/n31e099.hgt.zip

and also tiles ``n31e100`` and ``n31e101``. After unzipping, these can
be merged and cropped as follows::

     dem_mosaic n*.hgt --t_projwin 99.6 31.5 102 31 -o dem.tif

Determining these bounds and the visualization of all images and DEMs
can be done in ``stereo_gui``.

The SRTM DEM must be adjusted to be relative to the WGS84 datum, as discussed in
:numref:`conv_to_ellipsoid`.

Creating camera files
~~~~~~~~~~~~~~~~~~~~~

ASP provides the tool named ``cam_gen`` that, based on a camera's
intrinsics and the positions of the image corners on Earth's surface
will create initial camera models that will be the starting point for
aligning the cameras.

To create optical bar camera models, an example camera model file is
needed. This needs to contain all of the expected values for the camera,
though image_size, image_center, iC, and IR can be any value since they
will be recalculated. The pitch is determined by the resolution of the
scanner used, which is seven microns. The other values are determined by
looking at available information about the satellite. For the first
image (DS1105-2248DF076) the following values were used::

     VERSION_4
     OPTICAL_BAR
     image_size = 13656 1033
     image_center = 6828 517
     pitch = 7.0e-06
     f = 0.61000001430511475
     scan_time = 0.5
     forward_tilt = 0.2618
     iC = -1030862.1946224371 5468503.8842079658 3407902.5154047827
     iR = -0.95700845635275322 -0.27527006183758934 0.091439638698163225 -0.26345593052063937 0.69302501329766897 -0.67104940475144637 0.1213498543172795 -0.66629027007731101 -0.73575232847574434
     speed = 7700
     mean_earth_radius = 6371000
     mean_surface_elevation = 4000
     motion_compensation_factor = 1.0
     scan_dir = right

For a description of each value, see :numref:`panoramic`. For
the other image (aft camera) the forward tilt was set to -0.2618 and
scan_dir was set to 'left'. The correct values for scan_dir (left or
right) and use_motion_compensation (1.0 or -1.0) are not known for
certain due to uncertainties about how the images were recorded and may
even change between launches of the KH-4 satellite. You will need to
experiment to see which combination of settings produces the best
results for your particular data set.

The metadata table from Earth Explorer has the following entries for
DS1105-2248DF076::

     NW Corner Lat dec   31.266
     NW Corner Long dec  99.55
     NE Corner Lat dec   31.55
     NE Corner Long dec  101.866
     SE Corner Lat dec   31.416
     SE Corner Long dec  101.916
     SW Corner Lat dec   31.133
     SW Corner Long dec  99.55

These correspond to the upper-left, upper-right, lower-right, and
lower-left pixels in the image. We will invoke ``cam_gen`` as follows::

     cam_gen --sample-file sample_kh4b_for_optical_bar.tsai     \
       --camera-type opticalbar                                 \
       --lon-lat-values                                         \
       '99.55 31.266 101.866 31.55 101.916 31.416 99.55 31.133' \
       for.tif --reference-dem dem.tif --refine-camera -o for.tsai

     cam_gen --sample-file sample_kh4b_aft_optical_bar.tsai     \
       --camera-type opticalbar                                 \
       --lon-lat-values                                         \
       '99.566 31.266 101.95 31.55 101.933 31.416 99.616 31.15' \
       aft.tif --reference-dem dem.tif --refine-camera -o aft.tsai

It is very important to note that if, for example, the upper-left image
corner is in fact the NE corner from the metadata, then that corner
should be the first in the longitude-latitude list when invoking this
tool.

Bundle adjustment and stereo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before processing the input images it is a good idea to experiment with
reduced resolution copies in order to accelerate testing. You can easily
generate reduced resolution copies of the images using ``stereo_gui`` as
shown below. 

::

     stereo_gui for.tif aft.tif --create-image-pyramids-only
     ln -s for_sub8.tif  for_small.tif
     ln -s aft_sub8.tif  aft_small.tif
     cp for.tsai for_small.tsai
     cp aft.tsai aft_small.tsai

The new .tsai files need to be adjusted by updating the image_size,
image_center (divide by resolution factor, which is 8 here), and the
pitch (multiply by the resolution factor) to account for the
downsample amount.

You can now run bundle adjustment on the downsampled images::

     bundle_adjust for_small.tif aft_small.tif \
       for_small.tsai aft_small.tsai           \
       -t opticalbar                           \
       --max-iterations 100                    \
       --camera-weight 0                       \
       --tri-weight 0.1                        \
       --tri-robust-threshold 0.1              \
       --disable-tri-ip-filter                 \
       --skip-rough-homography                 \
       --inline-adjustments                    \
       --ip-detect-method 1                    \
       --datum WGS84                           \
       -o ba_small/run

The value of ``--tri-weight`` should be inversely proportional to
ground-sample distance, so low-resolution (coarser) images should use
a lower-value, as then multiplying by this weight will more accurately
bring differences in units of meters to units of pixels.

Validation of cameras
~~~~~~~~~~~~~~~~~~~~~

An important sanity check is to mapproject the images with these
cameras, for example as::

     mapproject dem.tif for.tif for.tsai for.map.tif
     mapproject dem.tif aft.tif aft.tsai aft.map.tif

and then overlay the mapprojected images on top of the DEM in
``stereo_gui``. If it appears that the images were not projected
correctly, or there are gross alignment errors, likely the order of
image corners was incorrect. At this stage it is not unusual that the
mapprojected images are somewhat shifted from where they should be,
that will be corrected later.

This exercise can be done with the small versions of the images and
cameras, and also before and after bundle adjustment.

Running stereo
~~~~~~~~~~~~~~

Followed by stereo::

     parallel_stereo for_small.tif aft_small.tif                        \
       ba_small/run-for_small.tsai ba_small/run-aft_small.tsai          \
       stereo_small_mgm/run --alignment-method affineepipolar           \
       -t opticalbar --skip-rough-homography --disable-tri-ip-filter    \
       --ip-detect-method 1 --stereo-algorithm 2 

If stereo takes too long, and in particular, if the printed disparity
search range is large (its width and height is more than 100 pixels),
it is strongly suggested to run stereo with mapprojected images, per
:numref:`mapproj-example`. Ensure the mapprojected images have the
same resolution, and overlay them on top of the initial DEM first, to
check for gross misalignment.

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices in stereo.

.. _kh4_align:

DEM generation and alignment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Next, a DEM is created::

     point2dem --stereographic --proj-lon 100.50792 --proj-lat 31.520417 \
       --tr 30 stereo_small_mgm/run-PC.tif

Here we chose to use a stereographic projection, with its center
not too far from the area of interest. This has the advantage that the grid
size (``--tr``) is then expressed in meters, which is more intuitive
than if it is in fraction of a degree as when the ``longlat`` projection
is used. 

This will create a very rough initial DEM. It is sufficient however to
align and compare with the SRTM DEM::

     pc_align --max-displacement -1                                      \
       --initial-transform-from-hillshading similarity                   \
       --save-transformed-source-points --num-iterations 0               \
       --max-num-source-points 1000 --max-num-reference-points 1000      \
       dem.tif stereo_small_mgm/run-DEM.tif -o stereo_small_mgm/run

     point2dem --stereographic --proj-lon 100.50792 --proj-lat 31.520417 \
       --tr 30 stereo_small_mgm/run-trans_source.tif

This will hopefully create a DEM aligned to the underlying SRTM. Consider
examining in ``stereo_gui`` the left and right hillshaded files produced
by ``pc_align`` and the match file among them, to ensure tie points among
the two DEMs were found properly (:numref:`stereo_gui_view_ip`). 

There is a chance that this may fail as the two DEMs to align could be too
different. In that case, one can re-run ``point2dem`` to re-create the
DEM to align with a coarser resolution, say with ``--tr 120``, then
re-grid the SRTM DEM to the same resolution, which can be done as::

     pc_align --max-displacement -1 dem.tif dem.tif -o dem/dem             \
       --num-iterations 0 --max-num-source-points 1000                     \
       --max-num-reference-points 1000 --save-transformed-source-points

     point2dem --stereographic --proj-lon 100.50792 --proj-lat 31.520417   \
       --tr 120 dem/dem-trans_source.tif

You can then try to align the newly obtained coarser SRTM DEM to the
coarser DEM from stereo.

Floating the intrinsics
~~~~~~~~~~~~~~~~~~~~~~~

The obtained alignment transform can be used to align the cameras as
well, and then one can experiment with floating the intrinsics, as in
:numref:`skysat`.

Modeling the camera models as pinhole cameras with RPC distortion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once sufficiently good optical bar cameras are produced and the
DEMs from them are reasonably similar to some reference terrain
ground truth, such as SRTM, one may attempt to improve the accuracy
further by modeling these cameras as simple pinhole models with the
nonlinear effects represented as a distortion model given by Rational
Polynomial Coefficients (RPC) of any desired degree (see
:numref:`pinholemodels`). The best fit RPC representation can be
found for both optical bar models, and the RPC can be further
optimized using the reference DEM as a constraint.

To convert from optical bar models to pinhole models with RPC distortion
one does::

    convert_pinhole_model for_small.tif for_small.tsai \
      -o for_small_rpc.tsai --output-type RPC          \
      --camera-to-ground-dist 300000                   \
      --sample-spacing 50 --rpc-degree 2

and the same for the other camera. Here, one has to choose carefully
the camera-to-ground-distance. Above it was set to 300 km.  

The obtained cameras should be bundle-adjusted as before. One can
create a DEM and compare it with the one obtained with the earlier
cameras. Likely some shift in the position of the DEM will be present,
but hopefully not too large. The ``pc_align`` tool can be used to make
this DEM aligned to the reference DEM.

Next, one follows the same process as outlined in :numref:`skysat` and
:numref:`floatingintrinsics` to refine the RPC
coefficients. We will float the RPC coefficients of the left and right
images independently, as they are unrelated. Hence the command we will
use is::

     bundle_adjust for_small.tif aft_small.tif                       \
       for_small_rpc.tsai aft_small_rpc.tsai                         \
       -o ba_rpc/run --max-iterations 200                            \
       --camera-weight 0 --disable-tri-ip-filter                     \
       --skip-rough-homography --inline-adjustments                  \
       --ip-detect-method 1 -t nadirpinhole --datum WGS84            \
       --force-reuse-match-files --reference-terrain-weight 1000     \
       --parameter-tolerance 1e-12 --max-disp-error 100              \
       --disparity-list stereo/run-unaligned-D.tif                   \
       --max-num-reference-points 40000 --reference-terrain srtm.tif \
       --solve-intrinsics                                            \
       --intrinsics-to-share 'focal_length optical_center'           \
       --intrinsics-to-float other_intrinsics --robust-threshold 10  \
       --initial-transform pc_align/run-transform.txt

Here it is suggested to use a match file with dense interest points. The
initial transform is the transform written by ``pc_align`` applied to
the reference terrain and the DEM obtained with the camera models
``for_small_rpc.tsai`` and ``aft_small_rpc.tsai`` (with the reference
terrain being the first of the two clouds passed to the alignment
program). The unaligned disparity in the disparity list should be from
the stereo run with these initial guess camera models (hence stereo
should be used with the ``--unalign-disparity`` option). It is suggested
that the optical center and focal lengths of the two cameras be kept
fixed, as RPC distortion should be able model any changes in those
quantities as well.

One can also experiment with the option ``--heights-from-dem`` instead
of ``--reference-terrain``. The former seems to be able to handle better
large height differences between the DEM with the initial cameras and
the reference terrain, while the former is better at refining the
solution.

Then one can create a new DEM from the optimized camera models and see
if it is an improvement.

.. _kh7:

Declassified satellite images: KH-7
-----------------------------------

KH-7 was an effective observation satellite that followed the Corona
program. It contained an index (frame) camera and a single strip
(pushbroom) camera. ASP does not currently have a dedicated camera model for
this camera, so we will have to try to approximate it with a pinhole
model. Without a dedicated solution for this camera, you may only be
able to get good results near the central region of the image.

For this example we find the following images in Earth Explorer
declassified collection 2::

     DZB00401800038H025001
     DZB00401800038H026001

Make note of the lat/lon corners of the images listed in Earth Explorer,
and note which image corners correspond to which compass locations.

It is suggested to resize the images to a more manageable size. This can
avoid failures in the processing below (:numref:`resizing_images`).

We will merge the images with the ``image_mosaic`` tool. These images have a
large amount of overlap and we need to manually lower the blend radius so that
we do not have memory problems when merging the images. Note that the image
order is different for each image.

::

     image_mosaic DZB00401800038H025001_b.tif  DZB00401800038H025001_a.tif \
       -o DZB00401800038H025001.tif  --ot byte --blend-radius 2000         \
       --overlap-width 10000
     image_mosaic DZB00401800038H026001_a.tif  DZB00401800038H026001_b.tif \
       -o DZB00401800038H026001.tif  --ot byte --blend-radius 2000         \
       --overlap-width 10000

For this image pair we will use the following SRTM images from Earth
Explorer::

     n22_e113_1arc_v3.tif
     n23_e113_1arc_v3.tif
     dem_mosaic n22_e113_1arc_v3.tif n23_e113_1arc_v3.tif -o srtm_dem.tif

The SRTM DEM must be first adjusted to be relative to WGS84
(:numref:`conv_to_ellipsoid`).

Next we crop the input images so they only contain valid image area. We
use, as above, the ``historical_helper.py`` tool. See :numref:`historical_helper`
for how to install the ImageMagick software that it needs.

::

     historical_helper.py rotate-crop                                    \
       --interest-points '1847 2656  61348 2599  61338 33523  1880 33567'\
       --input-path DZB00401800038H025001.tif                            \
       --output-path 5001.tif
     historical_helper.py rotate-crop                                    \
       --interest-points '566 2678  62421 2683  62290 33596  465 33595'  \
       --input-path DZB00401800038H026001.tif                            \
       --output-path 6001.tif

We will try to approximate the KH7 camera using a pinhole model. The
pitch of the image is determined by the scanner, which is 7.0e-06 meters
per pixel. The focal length of the camera is reported to be 1.96 meters,
and we will set the optical center at the center of the image. We need
to convert the optical center to units of meters, which means
multiplying the pixel coordinates by the pitch to get units of meters.

Using the image corner coordinates which we recorded earlier, use the
``cam_gen`` tool to generate camera models for each image, being careful
of the order of coordinates.

::

     cam_gen --pixel-pitch 7.0e-06 --focal-length 1.96                             \
       --optical-center 0.2082535 0.1082305                                        \
       --lon-lat-values '113.25 22.882 113.315 23.315 113.6 23.282 113.532 22.85'  \
       5001.tif --reference-dem srtm_dem.tif --refine-camera -o 5001.tsai
     cam_gen --pixel-pitch 7.0e-06 --focal-length 1.96                             \
       --optical-center 0.216853 0.108227                                          \
       --lon-lat-values '113.2 22.95 113.265 23.382 113.565 23.35 113.482 22.915'  \
       6001.tif --reference-dem srtm_dem.tif --refine-camera -o 6001.tsai

A quick way to evaluate the camera models is to use the
``camera_footprint`` tool to create KML footprint files, then look at
them in Google Earth. For a more detailed view, you can mapproject them
and overlay them on the reference DEM in ``stereo_gui``.

::

     camera_footprint 5001.tif  5001.tsai  --datum  WGS_1984 --quick \
       --output-kml  5001_footprint.kml -t nadirpinhole --dem-file srtm_dem.tif
     camera_footprint 6001.tif  6001.tsai  --datum  WGS_1984 --quick \
       --output-kml  6001_footprint.kml -t nadirpinhole --dem-file srtm_dem.tif

The output files from ``cam_gen`` will be roughly accurate but they may
still be bad enough that ``bundle_adjust`` has trouble finding a
solution. One way to improve your initial models is to use ground
control points. For this test case I was able to match features along
the rivers to the same rivers in a hillshaded version of the reference
DEM. I used three sets of GCPs, one for each image individually and a
joint set for both images. I then ran ``bundle_adjust`` individually for
each camera using the GCPs.

::

    bundle_adjust 5001.tif 5001.tsai gcp_5001.gcp \
      -t nadirpinhole --inline-adjustments        \
      --num-passes 1 --camera-weight 0            \
      --ip-detect-method 1 -o bundle_5001/out     \
      --max-iterations 30 --fix-gcp-xyz

    bundle_adjust 6001.tif 6001.tsai gcp_6001.gcp \
      -t nadirpinhole --inline-adjustments        \
      --num-passes 1 --camera-weight 0            \
      --ip-detect-method 1 -o bundle_6001/out     \
      --max-iterations 30 --fix-gcp-xyz

At this point it is a good idea to experiment with downsampled copies of
the input images before running processing with the full size images.
You can generate these using ``stereo_gui``. Also make copies of the
camera model files and scale the pitch to match the
downsample amount. 

::

     stereo_gui 5001.tif 6001.tif --create-image-pyramids-only
     ln -s 5001_sub16.tif  5001_small.tif
     ln -s 6001_sub16.tif  6001_small.tif
     cp 5001.tsai  5001_small.tsai
     cp 6001.tsai  6001_small.tsai

Now we can run ``bundle_adjust`` and ``parallel_stereo``. If you are using the
GCPs from earlier, the pixel values will need to be scaled to match the
downsampling applied to the input images.

::

    bundle_adjust 5001_small.tif 6001_small.tif              \
       bundle_5001/out-5001_small.tsai                       \
       bundle_6001/out-6001_small.tsai                       \
       gcp_small.gcp -t nadirpinhole -o bundle_small_new/out \
       --force-reuse-match-files --max-iterations 30         \
       --camera-weight 0 --disable-tri-ip-filter             \
       --skip-rough-homography                               \
       --inline-adjustments --ip-detect-method 1             \
       --datum WGS84 --num-passes 2

    parallel_stereo --alignment-method homography                      \
      --skip-rough-homography --disable-tri-ip-filter                  \
      --ip-detect-method 1 --session-type nadirpinhole                 \
      5001_small.tif 6001_small.tif                                    \
      bundle_small_new/out-out-5001_small.tsai                         \
      bundle_small_new/out-out-6001_small.tsai                         \
      st_small_new/out

As in :numref:`kh4`, it is suggested to mapproject the images with these
cameras onto the initial guess DEM, overlay all these in ``stereo_gui``,
and check if they roughly align.

It is suggested to run stereo with mapprojected images
(:numref:`mapproj-example`). See also :numref:`nextsteps` for a
discussion about various speed-vs-quality choices in stereo.

Write the intersection error image to a separate file::

     gdal_translate -b 4 st_small_new/out-PC.tif st_small_new/error.tif

Looking at the error result, it is clear that the simple pinhole model
is not doing a good job modeling the KH7 camera. We can try to improve
things by adding a distortion model to replace the NULL model in the
.tsai files we are using.

::

   BrownConrady
   xp  = -1e-12
   yp  = -1e-12
   k1  = -1e-10
   k2  = -1e-14
   k3  = -1e-22
   p1  = -1e-12
   p2  = -1e-12
   phi = -1e-12

Once the distortion model is added, you can use ``bundle_adjust`` to
optimize them. See the section on solving for pinhole intrinsics in the
KH4B example for details. We hope to provide a more rigorous method of
modeling the KH7 camera in the future.

.. _kh9:

Declassified satellite images: KH-9
-----------------------------------

The KH-9 satellite contained one frame camera and two panoramic cameras,
one pitched forwards and one aft. It is important to check which of these 
sensors your images are acquired with.

The frame camera is a regular pinhole model (:numref:`pinholemodels`). 
The images produced with it could be processed as for KH-7 (:numref:`kh7`), 
SkySat (:numref:`skysat`), or using Structure-from-Motion (:numref:`sfm`). 

This example describes how to process the the panoramic camera images. These
images appear notably distorted at the corners. 
The processing is similar to handling KH-4B (:numref:`kh4`) except that 
the images are much larger.

The ASP support for panoramic images is highly experimental. There is no reliable
way of determining the camera orientation to use below. As of now, sometimes
one may get plausible results, and sometimes this approach will fail. The use
is strongly advised not to spend much time on this data until the support is
improved.

For this example we use the following images from the Earth Explorer
declassified collection 3::

     D3C1216-200548A041
     D3C1216-200548F040

Make note of the lat/lon corners of the images listed in Earth Explorer,
and note which image corners correspond to which compass locations.

It is suggested to resize the images to a more manageable size. This can
avoid failures in the processing below (:numref:`resizing_images`).

We merge the images with the ``image_mosaic`` tool.

::

     image_mosaic D3C1216-200548F040_a.tif D3C1216-200548F040_b.tif D3C1216-200548F040_c.tif \
       D3C1216-200548F040_d.tif  D3C1216-200548F040_e.tif  D3C1216-200548F040_f.tif          \
       D3C1216-200548F040_g.tif  D3C1216-200548F040_h.tif  D3C1216-200548F040_i.tif          \
       D3C1216-200548F040_j.tif  D3C1216-200548F040_k.tif  D3C1216-200548F040_l.tif          \
       --ot byte --overlap-width 3000 -o D3C1216-200548F040.tif
     image_mosaic D3C1216-200548A041_a.tif D3C1216-200548A041_b.tif D3C1216-200548A041_c.tif \
       D3C1216-200548A041_d.tif  D3C1216-200548A041_e.tif  D3C1216-200548A041_f.tif          \
       D3C1216-200548A041_g.tif  D3C1216-200548A041_h.tif  D3C1216-200548A041_i.tif          \
       D3C1216-200548A041_j.tif  D3C1216-200548A041_k.tif --overlap-width 1000               \
       --ot byte -o D3C1216-200548A041.tif  --rotate

These images also need to be cropped to remove most of the area around
the images::

     historical_helper.py rotate-crop --input-path D3C1216-200548F040.tif \
       --output-path for.tif                                              \
       --interest-points '2414 1190  346001 1714  345952 23960  2356 23174'
     historical_helper.py rotate-crop --input-path D3C1216-200548A041.tif \
       --output-path aft.tif                                              \
       --interest-points '1624 1333  346183 1812  346212 24085  1538 23504'

We used, as above, the ``historical_helper.py`` tool. See :numref:`historical_helper`
for how to install the ImageMagick software that it needs.

For this example there are ASTER DEMs which can be used for reference.
They can be downloaded from https://gdex.cr.usgs.gov/gdex/ as single
GeoTIFF files. To cover the entire area of this image pair you may need
to download two files separately and merge them using ``dem_mosaic``.

As with KH-4B, this satellite contains a forward pointing and aft
pointing camera that need to have different values for "forward_tilt" in
the sample camera files. The suggested values are -0.174533 for the aft
camera and 0.174533 for the forward camera. Note that some KH9 images
have a much smaller field of view (horizontal size) than others!

::

     VERSION_4
     OPTICAL_BAR
     image_size = 62546 36633
     image_center = 31273 18315.5
     pitch = 7.0e-06
     f = 1.5
     scan_time = 0.7
     forward_tilt = 0.174533
     iC = -1053926.8825477704 5528294.6575468015 3343882.1925249361
     iR = -0.96592328992496967 -0.16255393156297787 0.20141603042941184 -0.23867502833024612 0.25834753840712932 -0.93610404349651921 0.10013205696518604 -0.95227767417513032 -0.28834146846321851
     speed = 8000
     mean_earth_radius = 6371000
     mean_surface_elevation = 0
     motion_compensation_factor = 1
     scan_dir = right

Camera files are generated using ``cam_gen`` from a sample camera file
as in the previous examples.

::

     cam_gen --sample-file sample_kh9_for_optical_bar.tsai --camera-type opticalbar          \
       --lon-lat-values '-151.954 61.999  -145.237 61.186  -145.298 60.944  -152.149 61.771' \
       for.tif --reference-dem aster_dem.tif --refine-camera  -o for.tsai
     cam_gen --sample-file sample_kh9_aft_optical_bar.tsai --camera-type opticalbar         \
       --lon-lat-values '-152.124 61.913  -145.211 61.156  -145.43 60.938  -152.117 61.667' \
       aft.tif --reference-dem aster_dem.tif --refine-camera  -o aft.tsai

As with KH-4B, it is best to first experiment with low resolution copies
of the images. Don't forget to scale the image size, center location,
and pixel size in the new camera files!

::

     stereo_gui for.tif aft.tif --create-image-pyramids-only
     ln -s for_sub32.tif for_small.tif
     ln -s aft_sub32.tif aft_small.tif
     cp for.tsai for_small.tsai
     cp aft.tsai aft_small.tsai

From this point KH-9 data can be processed in a very similar manner to
the KH-4B example. Once again, you may need to vary some of the camera
parameters to find the settings that produce the best results. For this
example we will demonstrate how to use ``bundle_adjust`` to solve for
intrinsic parameters in optical bar models.

Using the DEM and the input images it is possible to collect rough
ground control points which can be used to roughly align the initial
camera models.

::

     bundle_adjust for_small.tif for_small.tsai    \
       ground_control_points.gcp -t opticalbar     \
       --inline-adjustments --num-passes 1         \
       --camera-weight 0 --ip-detect-method 1      \
       -o bundle_for_small/out --max-iterations 30 \
       --fix-gcp-xyz

     bundle_adjust aft_small.tif aft_small.tsai    \
       ground_control_points.gcp -t opticalbar     \
       --inline-adjustments --num-passes 1         \
       --camera-weight 0 --ip-detect-method 1      \
       -o bundle_aft_small/out --max-iterations 30 \
       --fix-gcp-xyz

Now we can do a joint bundle adjustment. While in this example we
immediately attempt to solve for intrinsics, you can get better results
using techniques such as the ``--disparity-list`` option described in
:numref:`kh4` and :numref:`skysat` along with the reference DEM.
We will try to solve for all intrinsics but will share the focal length
and optical center since we expect them to be very similar. If we get
good values for the other intrinsics we could do another pass where we
don't share those values in order to find small difference between the
two cameras. We specify intrinsic scaling limits here. The first three
pairs are for the focal length and the two optical center values. For an
optical bar camera, the next three values are for ``speed``,
``motion_compensation_factor``, and ``scan_time``. We are fairly
confident in the focal length and the optical center but we only have
guesses for the other values so we allow them to vary in a wider range.

::

    bundle_adjust left_small.tif right_small.tif          \
      bundle_for_small/out-for_small.tsai                 \
      bundle_aft_small/out-aft_small.tsai                 \
      -t opticalbar -o bundle_small/out                   \
      --force-reuse-match-files --max-iterations 30       \
      --camera-weight 0 --disable-tri-ip-filter           \
      --skip-rough-homography --inline-adjustments        \
      --ip-detect-method 1 --datum WGS84 --num-passes 2   \
      --solve-intrinsics                                  \
      --intrinsics-to-float "focal_length optical_center 
        other_intrinsics"                                 \
      --intrinsics-to-share "focal_length optical_center" \
      --ip-per-tile 1000                                  \
      --intrinsics-limits "0.95 1.05 0.90 1.10 0.90 1.10 
         0.5 1.5 -5.0 5.0 0.3 2.0" --num-random-passes 2

These limits restrict our parameters to reasonable bounds but
unfortunately they greatly increase the run time of ``bundle_adjust``.
Hopefully you can figure out the correct values for ``scan_dir`` doing
long optimization runs using the limits. The ``--intrinsic-limits``
option is useful when used in conjunction with the
``--num-random-passes`` option because it also sets the numeric range in
which the random initial parameter values are chosen from. Note that
``--num-passes`` is intended to filter out bad interest points while
``--num-random-passes`` tries out multiple random starting seeds to see
which one leads to the result with the lowest error.
