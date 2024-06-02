.. _dem2gcp:

dem2gcp
-------

This program generates GCP (:numref:`bagcp`) based on densely measuring the
warping between two DEMs, which helps resolve the lens distortion and the
warping with (:numref:`intrinsics_ground_truth`).

The approach is as follows. The dense disparity from an ASP-produced warped DEM
to a reference DEM is found. This program will take as input that disparity
and interest point matches between the raw images, and will produce GCP with
correct ground positions based on the reference DEM. Lastly, bundle adjustment
will solve for the lens distortion.

This program was motivated by the processing of KH-7 images, for which ASP has
no camera model, so the initial produced DEM ends up rather warped. The hope is 
to model and correct the deformation in the sensor plane as lens distortion.
 
ASP DEM creation
~~~~~~~~~~~~~~~~

Prepare the images and Pinhole camera models with no distortion as shown in
:numref:`kh7`. 

It is very strongly suggested to work at a lower-resolution, such as 1/16th of
the original resolution. The exact factor depends on the GSD of raw images and
of the reference DEM. The Pinhole camera models (numref:`pinholemodels`) can be
adjusted for the resolution change by multiplying appropriately the ``pitch``
value *only*.

Bundle-adjust the images and the cameras without optimizing the intrinsics
(:numref:`bundle_adjust`). Use the option ``--inline-adjustments``, and
increase ``--ip-per-tile`` and ``--matches-per-tile`` if not getting enough 
matches.

Mapproject the images at the same appropriate resolution onto a reference DEM
with the cameras produced by bundle adjustment. Use a local projection, such as
stereographic. Run stereo with the mapprojected images
(:numref:`mapproj-example`). Use the ``asp_mgm`` algorithm with
``--subpixel-mode 9`` (:numref:`stereo_alg_overview`). This will ensure the best
quality.

Overlay the mapprojected images, produced DEM, and reference DEM in ``stereo_gui``.
These should be roughly in the same place, but with some warping.

The produced DEM can be aligned with the reference DEM (:numref:`pc_align`), and
the same alignment can be applied to the cameras (:numref:`ba_pc_align`).
Mapproject the images with the latest cameras onto the DEM, and run stereo again
(with option ``--prev-run-prefix`` to reuse the previous run,
:numref:`mapproj_reuse`), and see if the new DEM is better-aligned with the
reference DEM.
 
Ensure ``parallel_stereo`` was invoked to generate dense matches from disparity
(:numref:`dense_ip`). It is suggested to use ``--num-matches-from-disparity
200000`` or so. That is an outrageous number of interest points, but will
help produce sufficient GCP later on. (Their number can be reduced later 
for bundle adjustment with the option ``--max-pairwise-matches``.)

Ensure that the match file is renamed according the *naming convention* for the
original raw images (:numref:`ba_match_files`). Such matches can be produced
after stereo already finished, by re-running ``stereo_tri`` only
(:numref:`entrypoints`).

Comparison with the reference DEM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The level of detail and overall appearance of the hillshaded DEMs in ``stereo_gui``
must be very similar. 

.. figure:: ../images/kh7_dem_vs_ref.png
   :name: kh7_dm_vs_ref
   
   A low-resolution KH-7 DEM produced by ASP (left) and a reference DEM (right).
   These must be visually similar and with enough features for ``dem2gcp`` to work.
   The DEMs can be overlaid to see if there is significant local warping. Difference
   maps are in :numref:`kh7_orig_vs_opt`.

Some hole-filling and blur can be applied to the ASP DEM with ``dem_mosaic``
(:numref:`dem_mosaic_blur` and :numref:`dem_mosaic_grow`).

A command as::

    gdal_translate -r average -outsize 50% 50% dem.tif dem_small.tif
    
can reduce the resolution. This likely will do a better job than ``gdalwarp``,
which uses interpolation. 

The two DEMs must be re-gridded to the same local projection and grid size.
Example (adjust the projection center)::

   proj='+proj=stere +lat_0=27.909 +lon_0=102.226 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
   gdalwarp -tr 20 20 -t_srs "$proj" -r cubicspline dem_in.tif dem_out.tif

It is not necessary for the produced DEMs to have precisely the same extent, but
cropping to similar regions is suggested. 

The DEMs should be hillshaded and inspected in ``stereo_gui``. Select the first
band of each hillshaded DEM produced by this tool, as::

    gdal_translate -b 1 dem_hillshade_a300_e20.tif \
     -o dem_hill_b1.tif

Find the dense disparity from the ASP hillshaded DEM (first band) to the 
reference hill-shaded DEM::

  parallel_stereo      \
    --correlator-mode  \
    --ip-per-tile 500  \
    warped_hill_b1.tif \
    ref_hill_b1.tif
    warp/run

The order here is very important. Increase ``--ip-per-tile`` if not enough matches
are found.

Running ``dem2gcp``
~~~~~~~~~~~~~~~~~~~

:: 

    dem2gcp                                            \
      --warped-dem asp_dem.tif                         \
      --ref-dem ref_dem.tif                            \
      --warped-to-ref-disparity warp/run-F.tif         \
      --left-image left.tif                            \
      --right-image right.tif                          \
      --left-camera left.tsai                          \
      --right-camera right.tsai                        \
      --match-file dense_matches/run-left__right.match \
      --search-len 5                                   \
      --gcp-sigma 1e-2                                 \
      --output-gcp out.gcp
      
Here we used the left and right raw images, the left and right camera models
with no distortion that produced the DEM, and the dense matches between the raw
images. 

.. figure:: ../images/dem2gcp_ip_vs_gcp.png
   :name: dem2gcp_ip_vs_gcp
   
   Interest point matches (left, in red) and produced GCP (right), on top of the raw images.
   Plotted with ``sterero_gui`` (:numref:`stereo_gui`). 

Plotted in :numref:`dem2gcp_ip_vs_gcp` are the interest point matches and the
resulting GCP. Their numbers are likely excessive here, though the bigger concern
is if they are lacking over featureless terrain. 

Solving for lens distortion
~~~~~~~~~~~~~~~~~~~~~~~~~~~

We employ the solution from :numref:`heights_from_dem`, which mostly takes care
of the vertical component of disagreement between the ASP-produced and reference
DEM, and augment it with GCP, that mostly take care of the horizontal component.

The most recent bundle-adjusted and aligned cameras can be converted to use the
RPC lens distortion model (:numref:`rpc_distortion`) as in
:numref:`convert_pinhole_model`.

The small RPC coefficients *must be changed manually to be at least 1e-7*,
otherwise they will not get optimized. A RPC of degree 3 is used for now. 

Command::

    bundle_adjust                                 \
      left_image.tif right_image.tif              \
      left_rpc_camera.tsai right_rpc_camera.tsai  \
      --inline-adjustments                        \
      --solve-intrinsics                          \
      --intrinsics-to-float all                   \
      --intrinsics-to-share none                  \
      --num-iterations 100                        \
      --match-files-prefix dense_matches/run      \
      --max-pairwise-matches 100000               \
      --remove-outliers-params '75.0 3.0 100 100' \
      --heights-from-dem ref_dem.tif              \
      --heights-from-dem-uncertainty 500          \
      out.gcp                                     \
      -o ba_rpc_gcp_ht/run
     
Note how we employ the match file and GCP created earlier. A higher value 
in ``--heights-from-dem-uncertainty`` gives less weight to the vertical 
constraint. Likely it is better to prioritize the GCP instead.

Examine the pixel residuals before and after bundle adjustment
(:numref:`ba_err_per_point`) in ``stereo_gui`` as::

  stereo_gui --colorbar --min 0 --max 10             \
    ba_rpc_gcp_ht/run-initial_residuals_pointmap.csv \
    ba_rpc_gcp_ht/run-final_residuals_pointmap.csv

It should be rather obvious to see which residuals are from the GCP. These are
also flagged in this csv file.

.. figure:: ../images/kh7_orig_vs_opt.png
   :name: kh7_orig_vs_opt

   Difference between the ASP DEM and reference DEM. The vertical range is -1000
   m to 1000 m. From left-to-right: (a) no distortion modeling (b) modeling
   distortion with RPC of degree 3 and optimizing with ``--heights-from-dem``
   (c) additionally, use the GCP produced by ``dem2gcp``. The differences are
   found with ``geodiff`` (:numref:`geodiff`) and plotted with ``stereo_gui``.

Then, one can rerun stereo with the optimized cameras and the original images
(again with the option ``--prev-run-prefix``). The results are in
:numref:`kh7_orig_vs_opt`. The warping is much reduced but not eliminated. 

Ideally, several pairs of images are available that are acquired with precisely
the same camera model (note that there is a whole family of KH-7 cameras). Then,
they would be used jointly in bundle adjustment, while setting
``--intrinsics-to-share all``. This should increase the accuracy across the
board, if the produced DEMs are mountainous and hence it is easy to measure the
discrepancies.
 
One could also use a higher degree for the RPC model, such as 5.

Command-line options
~~~~~~~~~~~~~~~~~~~~
  
--warped-dem <string (default: "")>
    The DEM file produced by stereo, that may have warping due to unmodeled distortion.
    
--ref-dem <string (default: "")>
    The reference DEM file, which is assumed to be accurate.

--warped-to-ref-disparity <string (default: "")>
    The stereo disparity from the warped DEM to the reference DEM (use the first
    band of the hillshaded DEMs as the inputs for the disparity).

--left-image <string (default: "")>
    The left raw image that produced the warped DEM.
    
--right-image <string (default: "")>
    The right raw image that produced the warped DEM.
    
--left-camera <string (default: "")>
    The left camera that was used for stereo.    
    
--right-camera <string (default: "")>
    The right camera that was used for stereo.
    
--match-file <string (default: "")>
    A match file between the left and right raw images with many dense matches.
    
--search-len <int (default: 5)>
    How many DEM pixels to search around to find a valid DEM disparity (pick the
    closest).

--gcp-sigma <double (default: 1.0)>
    The sigma to use for the GCP points. A smaller value will give to GCP more weight.
    
--output-gcp <string (default: "")>
    The produced GCP file with ground coordinates from the reference DEM.
    
-v, --version
    Display the version of software.

-h, --help
    Display this help message.
