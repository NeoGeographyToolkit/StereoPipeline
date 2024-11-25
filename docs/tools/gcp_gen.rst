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

This program can fail if the camera image and orthoimage are not similar enough,
or if the orthoimage is a mirror-flipped version of the camera image. Manual
selection of interest points can then be invoked (:numref:`creatinggcp`).

Use the option ``--output-prefix`` to save the interest point matches for
inspection with ``stereo_gui`` (:numref:`stereo_gui_view_ip`). Consider
increasing the number of interest points to detect per image and adjusting the
inlier threshold if the matches are not good enough.

The context and next steps after using this program are discussed in
:numref:`camera_solve_gcp`.

See the related program named ``dem2gcp`` (:numref:`dem2gcp`).

Example
~~~~~~~

::

    gcp_gen --camera-image camera_image.tif \
      --ortho-image ortho_image.tif         \
      --dem dem.tif                         \
      -o gcp.gcp

If given several images, the program should be invoked individually
for each image, thus creating several GCP files. 

Validation
~~~~~~~~~~

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
    
--num-ransac-iterations <integer (default: 1000)>
    How many iterations to perform in RANSAC when finding interest point matches.

--inlier-threshold <double (default: 0.0)>
    The inlier threshold (in pixels) to separate inliers from outliers when
    computing interest point matches. A smaller threshold will result in fewer
    inliers. The default is 10% of the image diagonal.

--output-prefix <string (default: "")>
    If set, save the interest point matches using this prefix (for inspection).

--match-file <string (default: "")>
    If set, use this match file instead of creating one.          

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
