.. _ortho2pinhole:

ortho2pinhole
-------------

Given an orthoimage, a raw image, and a sample pinhole camera model having
camera intrinsics, this program computes a new pinhole camera model that best
reflects how the orthoimage was created from the raw image, so it adds
the extrinsic parameters (camera position and orientation) to the model.

Example invocation::

    ortho2pinhole raw_image.jpg ortho_image.tif \
      sample_model.tsai output_model.tsai       \
      --camera-height 1459.9561                 \
      --orthoimage-height 886.5911

The .tsai pinhole model format is described in :numref:`pinholemodels`.

The option ``--reference-dem`` can be used to specify a DEM from which
to extract the ground height, instead of using the value in
``--orthoimage-height``. 

As a way of verifying the results, the ``mapproject`` program can be used to
project the raw image with the produced camera model onto the reference DEM.
The resulting image should have a similar position and orientation on the ground
as the orthoimage.

More context is given in :numref:`sfmicebridge`.

Command-line options for ``ortho2pinhole``:

--camera-estimate <string (default: "")>
      An estimated camera model used for location and pose estimate only.
          
--max-translation <double (default: 10)>
      The maximum distance the camera solution is allowed to move from
      camera-estimate.
      
--camera-height <double (default: -1)>
      The approximate height above the datum, in meters, at which the camera
      should be. If not specified, it will be read from the orthoimage metadata.
      
--orthoimage-height <double (default: 0)>
      The approximate height above the datum, in meters, at which the orthoimage
      is positioned. We assume flat ground. See also ``--reference-dem``.
      
--reference-dem <string (default: "")>
      If provided, extract from this DEM the heights above the ground rather
      than assuming the value in ``--orthoimage-height``.
      
--ip-per-tile <integer (default: 0)>
      How many interest points to detect in each 1024^2 image tile (default:
      automatic determination).
      
--ip-detect-method <integer (default: 1)>
      Interest point detection algorithm (0: Integral OBALoG, 1: OpenCV SIFT
      (default), 2: OpenCV ORB.  Remove any existing ``.vwip`` files before
      recomputing interest points with a different method.

--minimum-ip <integer (default: 5)>
      Don't create a camera model if fewer than this many interest point matches
      were found.
      
--ip-inlier-factor <double (default: 0.2)>
      Interest points inlier factor.
      
--individually-normalize
      Individually normalize the input images instead of using common values.
      
--skip-image-normalization
      Skip the step of normalizing the values of input images.
      
--short-circuit
      No processing, just copy input intrinsic parameters to camera-estimate and
      write out.
      
--show-error
      Print point error.
      
--keep-match-file
      Don't delete the .match file after running.
      
--write-gcp-file
      Write a ``bundle_adjust``-compatible GCP file (:numref:`bagcp`).
      
--crop-reference-dem
      Crop the reference DEM to a generous area to make it faster to load.

-v, --version
    Display the version of software.

-h, --help
    Display the help message.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
