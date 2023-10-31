.. _aster:

ASTER
-----

In this example we will describe how to process ASTER Level 1A VNIR
images. A ready-made ASTER example having the input images and
cameras, ASP outputs, and instructions for how to run it, can be found
at:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases

A different worked-out example with illustrations:

    https://github.com/uw-cryo/asp-binder-demo/blob/master/example-aster_on_pangeo_binder_draft.ipynb

Fetching the data
^^^^^^^^^^^^^^^^^
 
ASTER satellite images are freely available from: 

  https://search.earthdata.nasa.gov/search

When visiting that page, select a region on the map, search for
AST_L1A, and choose ``ASTER L1A Reconstructed Unprocessed Instrument
Data V003``. (The same interface can be used to obtain pre-existing
ASTER DEMs.) If too many results are shown, narrow down the choices by
using a range in time or deselecting unwanted items
manually. Examining the data thumbnails is helpful, to exclude those
with clouds, etc. Then click to download.

It is very important that, at the very last step, when finalizing the order
options, choose GeoTIFF as the data format, rather than HDF-EOS. This way the
images and metadata will come already extracted from the HDF file.

Note that ASP cannot process ASTER Level 1B images, as those images lack camera
information.

Data preparation
^^^^^^^^^^^^^^^^

In this example will use the dataset
``AST_L1A_00307182000191236_20160404141337_21031`` near San Luis
Reservoir in Northern California. This dataset will contain TIFF
images and meta-information as text files. We use the tool
:ref:`aster2asp` to parse it::

     aster2asp dataDir -o out

Here, ``dataDir`` is the directory containing the ``*VNIR*tif`` files
produced by unzipping the ASTER dataset (sometimes this creates a new 
directory, and sometimes the files are extracted in the current
one).

This command will create 4 files, named::

     out-Band3N.tif out-Band3B.tif out-Band3N.xml out-Band3B.xml

We refer again to the tool's documentation page regarding details of how
these files were created.

Open the images in ``stereo_gui`` (:numref:`stereo_gui`) as::

    stereo_gui out-Band3N.tif out-Band3B.tif 

and ensure that they are of good quality, or else get another dataset. 

Stereo with raw images
^^^^^^^^^^^^^^^^^^^^^^
 
Run ``parallel_stereo`` (:numref:`parallel_stereo`)::

     parallel_stereo -t aster         \
       --stereo-algorithm asp_mgm     \
       --subpixel-mode 9              \
        out-Band3N.tif out-Band3B.tif \
        out-Band3N.xml out-Band3B.xml \
        out_stereo/run

This used the ``asp_mgm`` algorithm, which is the most accurate algorithm ASP
has. One can also try the option ``--subpixel-mode 2`` which will be much slower
but produce better results.

See :numref:`nextsteps` for a discussion about various stereo algorithms and
speed-vs-quality choices.

This is followed by DEM creation with ``point2dem``::

     point2dem -r earth --stereographic --auto-proj-center \
       out_stereo/run-PC.tif

This will create a DEM named ``out_stereo/run-DEM.tif`` using an auto-guessed
local stereographic projection with auto-guessed resolution (about 15 m / pixel,
the image ground sample distance). See the ``point2dem`` documentation in
:numref:`point2dem` for more details about how to run this program.

Visualize the DEM with ``stereo_gui`` (:numref:`stereo_gui`)::

    stereo_gui --hillshade out_stereo/run-DEM.tif

Stereo with mapprojected images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To improve the results for steep terrain, one may consider doing stereo as
before, followed by mapprojection onto a coarser and smoother version of the
obtained DEM, and then redoing stereo with mapprojected images (per the
suggestions in :numref:`mapproj-example`).

::

     # Initial stereo
     parallel_stereo -t aster         \
       --stereo-algorithm asp_mgm     \
       --subpixel-mode 9              \
        out-Band3N.tif out-Band3B.tif \
        out-Band3N.xml out-Band3B.xml \
        out_stereo/run

     # Create a low-resolution smooth DEM at 200 meters/pixel
     point2dem -r earth --stereographic --auto-proj-center \
       --tr 200 out_stereo/run-PC.tif -o out_stereo/run-200m

     # Mapproject onto this DEM at 15 meters/pixel
     mapproject --tr 15 out_stereo/run-200m-DEM.tif        \
       out-Band3N.tif out-Band3N.xml out-Band3N_proj.tif
     mapproject --tr 15 out_stereo/run-200m-DEM.tif        \
       out-Band3B.tif out-Band3B.xml out-Band3B_proj.tif
     
     # Run parallel_stereo with the mapprojected images
     parallel_stereo -t aster                  \
       --stereo-algorithm asp_mgm              \
       --subpixel-mode 9                       \
       out-Band3N_proj.tif out-Band3B_proj.tif \
       out-Band3N.xml out-Band3B.xml           \
       out_stereo_proj/run                     \
       out_stereo/run-200m-DEM.tif

     # Create the final DEM
     point2dem -r earth --stereographic --auto-proj-center \
      out_stereo_proj/run-PC.tif

It is very important to use the same resolution (option ``--tr``) for both
images when mapprojecting. That helps making the resulting images more similar
and reduces the processing time (:numref:`mapproj-res`). 

One could consider mapprojecting at a higher resolution, for example, at 10
meters/pixel.

It is suggested to also create and inspect the intersection error image
(:numref:`point2dem`). If it is large (comparable to ground sample distance),
the cameras should be bundle-adjusted first (:numref:`bundle_adjust`).

See :numref:`aster_dem_ortho_error` for a figure. 

.. _aster_csm:

Using the CSM model
^^^^^^^^^^^^^^^^^^^

An ASTER camera model consists of a sequence of satellite position samples and
a set of camera directions, sampled at about a dozen image rows and columns.
Interpolation is used in-between.

ASP can, in addition, fit a CSM linescan model (:numref:`csm`) on-the-fly to the
ASTER model. This has the advantage that instead of a set of directions on a grid,
there is one camera orientation at each satellite position sample. This will 
be used to solve for jitter in ASTER cameras (:numref:`jitter_aster`).

This functionality can be turned on with the option ``--aster-use-csm`` in 
stereo, bundle adjustment, mapprojection, and ``cam_test`` (:numref:`cam_test`).
This option is implicitly assumed when solving for jitter as
that tool only works with CSM cameras.

The ``cam_test`` documentation also describes how to compare the existing ASTER
and new CSM-based implementations. 

The bundle adjustment program (:numref:`bundle_adjust`) will optimize and save
the produced CSM models (:numref:`csm_state`), if invoked with this switch. To
save the best-fit CSM models with no further refinement, invoke this tool with
zero iterations. 

