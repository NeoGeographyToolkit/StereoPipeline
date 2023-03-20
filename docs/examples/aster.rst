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
 
ASTER satellite images are freely available from: 

  https://search.earthdata.nasa.gov/search

When visiting that page, select a region on the map, search for
AST_L1A, and choose ``ASTER L1A Reconstructed Unprocessed Instrument
Data V003``. (The same interface can be used to obtain pre-existing
ASTER DEMs.) If too many results are shown, narrow down the choices by
using a range in time or deselecting unwanted items
manually. Examining the data thumbnails is helpful, to exclude those
with clouds, etc. Then click to download.

There are two important things to keep in mind. First, at the very
last step, when finalizing the order options, choose GeoTIFF as the
data format, rather than HDF-EOS. This way the images and metadata
will come already extracted from the HDF file.

Second, note that ASP cannot process ASTER Level 1B images, as those
images lack camera information.

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
 
Next, we run ``parallel_stereo`` (:numref:`parallel_stereo`). We can
use either the exact camera model (``-t aster``), or its RPC
approximation (``-t rpc``). The former is much slower but more
accurate.

::

     parallel_stereo -t aster --subpixel-mode 3      \
        out-Band3N.tif out-Band3B.tif                \
        out-Band3N.xml out-Band3B.xml out_stereo/run

or

::

     parallel_stereo -t rpc --subpixel-mode 3        \
        out-Band3N.tif out-Band3B.tif                \
        out-Band3N.xml out-Band3B.xml out_stereo/run

See :numref:`nextsteps` for a discussion about various stereo
algorithms and speed-vs-quality choices. Particularly,
``--stereo-algorithm asp_mgm`` should produce more detailed results.

This is followed by DEM creation::

     point2dem -r earth --tr 0.0002777 out_stereo/run-PC.tif

The value 0.0002777 is the desired output DEM resolution,
specified in degrees. It is approximately 31 meters/pixel, the same as
the publicly available ASTER DEM, and about twice the 15 meters/pixel
image resolution.

Visualize the DEM with::

    stereo_gui --hillshade out_stereo/run-DEM.tif

To improve the results for steep terrain, one may consider doing
stereo as before, followed by mapprojection onto a coarser and
smoother version of the obtained DEM, and then redoing stereo with
mapprojected images (per the suggestions in :numref:`mapproj-example`). Using
``--subpixel-mode 2``, while much slower, yields the best results. The
flow is as follows::

     # Initial stereo
     parallel_stereo -t aster --subpixel-mode 3      \
        out-Band3N.tif out-Band3B.tif                \
        out-Band3N.xml out-Band3B.xml out_stereo/run               

     # Create a coarse and smooth DEM at 300 meters/pixel
     point2dem -r earth --tr 0.002695                \
       out_stereo/run-PC.tif -o out_stereo/run-300m

     # Mapproject onto this DEM at 10 meters/pixel
     mapproject --tr 0.0000898 out_stereo/run-300m-DEM.tif \
       out-Band3N.tif out-Band3N.xml out-Band3N_proj.tif
     mapproject --tr 0.0000898 out_stereo/run-300m-DEM.tif \
       out-Band3B.tif out-Band3B.xml out-Band3B_proj.tif
     
     # Run parallel_stereo with the mapprojected images
     # and subpixel-mode 2
     parallel_stereo -t aster --subpixel-mode 2          \
       out-Band3N_proj.tif out-Band3B_proj.tif           \
       out-Band3N.xml out-Band3B.xml out_stereo_proj/run \
       out_stereo/run-300m-DEM.tif

     # Create the final DEM
     point2dem -r earth --tr 0.0002777 out_stereo_proj/run-PC.tif

Also consider using ``--stereo-algorithm asp_mgm`` as mentioned earlier.

Here we could have again used ``-t rpc`` instead of ``-t aster``. 

It is very important to use the same resolution (option ``--tr``) for
both images when mapprojecting. That helps making the resulting images
more similar and reduces the processing time
(:numref:`mapproj-res`). The mapprojection resolution was 0.0000898,
which is about 10 meters/pixel.

It is possible to increase the resolution of the final DEM slightly by
instead mapprojecting at 7 meters/pixel, hence using::

     --tr .00006288

or smaller correlation and subpixel-refinement kernels, that is::

     --corr-kernel 15 15 --subpixel-kernel 25 25

instead of the defaults (21 21 and 35 35) but this comes with increased
noise as well, and using a finer resolution results in longer run-time.

We also tried to first bundle-adjust the cameras, using ASP's
``bundle_adjust``. We did not notice a noticeable improvement in
results.

