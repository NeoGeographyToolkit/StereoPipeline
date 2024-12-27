.. _gcp_gen:

gcp_gen
-------

This program creates ground control points (GCP, :numref:`bagcp`) given a raw
camera image, orthoimage, and DEM. The GCP can then be used to initialize or
constrain a camera model for the camera image. This is a very quick and
convenient way of creating cameras that avoids full Structure-from-Motion
(:numref:`sfm`).

The approach is to find interest point matches between the camera image (which
does not have a georeference) and orthoimage (which does), infer the geolocation
of those points from the orthoimage, and their elevation from the DEM.

The context and next steps after using this program are discussed in
:numref:`camera_solve_gcp`.

See the related program named ``dem2gcp`` (:numref:`dem2gcp`).

Example
~~~~~~~

::

    gcp_gen                           \
      --camera-image camera_image.tif \
      --ortho-image ortho_image.tif   \
      --dem dem.tif                   \
      --gcp-sigma 1.0                 \
      --output-prefix run/run         \
      -o gcp.gcp

If given several images, the program should be invoked individually
for each image, thus creating several GCP files. 

For certain datasets, the SIFT interest point detection (method 1) and a smaller
RANSAC threshold turned out to work better. Here's an alternative invocation,
also with more interest points per tile::

    gcp_gen                           \
      --ip-detect-method 1            \
      --inlier-threshold 50           \
      --ip-per-tile 1000              \
      --camera-image camera_image.tif \
      --ortho-image ortho_image.tif   \
      --dem dem.tif                   \
      --output-prefix run/run         \
      -o gcp.gcp

In some cases, ``--ip-detect-method 2`` (ORB) worked out better than SIFT.

Advanced usage
~~~~~~~~~~~~~~

If the extent of the raw camera image is very different than the orthoimage,
or if the camera image appears to be a mirror-flipped version of the orthoimage,
this program can fail. 

In that case, it is recommended to mapproject (:numref:`mapproject`) the raw
camera image onto a DEM, and pass in the mapprojected image as a helper to this
tool, with the option::

    --mapproj-image mapproj_image.tif

If needed, both the mapprojected image and orthoimage can be cropped first to a
shared area.

If the camera image and orthoimage have very different ranges of pixel values,
use the option ``--individual-normalize``.

If no luck, manual selection of interest points can be invoked
(:numref:`creatinggcp`).
    
Validation
~~~~~~~~~~

Run ``stereo_gui``::

  stereo_gui camera_image.tif ortho_image.tif \
    run/run-camera_image__ortho_image.match

to inspect the produced match file (see also :numref:`stereo_gui_view_ip`). It
should show correctly the correspondences. The GCP file can be inspected in
``stereo_gui`` as well (:numref:`stereo_gui_vwip_gcp`).
    
The images and GCP files can be passed together to ``bundle_adjust`` to refine,
transform, or initialize the camera models (:numref:`ba_use_gcp`).

Then, ``mapproject`` (:numref:`mapproject`) can be invoked with the camera
image, updated camera (or the original camera with the option
``--bundle-adjust-prefix``), and the DEM. The resulting orthoimage can be
overlaid on top of the original orthoimage in ``stereo_gui``
(:numref:`stereo_gui`) to visually inspect the agreement.

Alternatively, the residuals for each GCP can be inspected in the
``pointmap.csv`` files produced by ``bundle_adjust``
(:numref:`ba_err_per_point`).
 
Command-line options
~~~~~~~~~~~~~~~~~~~~

--camera-image <string (default: "")>
    The camera image.
    
--ortho-image <string (default: "")>
    The ortho image to geolocate the interest points in.
  
--dem <string (default: "")>
    The DEM to infer the elevations from.
    
--output-gcp, -o <string (default: "")>
    The output GCP file.

--gcp-sigma <double (default: 1.0)>
    The sigma (uncertainty, in meters) to use for the GCPs (:numref:`bagcp`). A
    smaller sigma suggests a more accurate GCP. See also option
    ``--fix-gcp-xyz`` in ``bundle_adjust`` (:numref:`ba_options`).
    
--ip-per-image <integer (default: 20000)>
    How many interest points to detect in each image (the resulting number of
    matches will be much less).

--ip-per-tile <integer (default: 0)>
    How many interest points to detect in each 1024^2 image tile (default:
    automatic determination). This is before matching. Not all interest points
    will have a match. See also ``--matches-per-tile``.

--matches-per-tile <integer (default: 0)>
    How many interest point matches to compute in each image tile (of size
    normally 1024^2 pixels). Use a value of ``--ip-per-tile`` a few times larger
    than this. See also ``--matches-per-tile-params``.
    
--matches-per-tile-params <int int (default: 1024 1280)>
    To be used with ``--matches-per-tile``. The first value is the image tile
    size for both images. A larger second value allows each right tile to
    further expand to this size, resulting in the tiles overlapping. This may be
    needed if the homography alignment between these images is not great, as
    this transform is used to pair up left and right image tiles.
  
--individually-normalize
    Individually normalize the input images instead of using common
    values.
    
--num-ransac-iterations <integer (default: 1000)>
    How many iterations to perform in RANSAC when finding interest point matches.

--inlier-threshold <double (default: 0.0)>
    The inlier threshold (in pixels) to separate inliers from outliers when
    computing interest point matches. A smaller threshold will result in fewer
    inliers. The default is 10% of the image diagonal.

--ip-detect-method <integer (default: 0)>
    Choose an interest point detection method from: 0 = OBAloG
    (:cite:`jakkula2010efficient`), 1 = SIFT (from OpenCV), 2 = ORB (from
    OpenCV). The SIFT method, unlike OBALoG, produces interest points that are
    accurate to subpixel level. See also :numref:`custom_ip`.

--output-prefix <string (default: "")>
    Save the intermediate data, including match files, in this directory. This
    will cache any matches found, and those will be used to create the GCP file.
    The match file needs to be deleted if desired to recompute it.

--match-file <string (default: "")>
    If set, use this match file instead of creating one.          

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
