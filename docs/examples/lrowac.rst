
Lunar Reconnaissance Orbiter (LRO) WAC
--------------------------------------

This example, including the inputs, recipe, and produced terrain model
can be downloaded from:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples/releases/tag/LROWAC

Fetching the data
~~~~~~~~~~~~~~~~~

We will focus on the monochromatic images for this sensor. Visit:

   https://ode.rsl.wustl.edu/moon/indexproductsearch.aspx

Find the *Lunar Reconnaissance Orbiter -> Experiment Data Record Wide
Angle Camera - Mono (EDRWAM)* option.

Search either based on a longitude-latitude window, or near a notable
feature, such as a named crater.  We choose a couple of images having
the Tycho crater, with download links::

    http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0002/DATA/MAP/2010035/WAC/M119923055ME.IMG
    http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0002/DATA/MAP/2010035/WAC/M119929852ME.IMG

Get these with ``wget``.

Creation of .cub files
~~~~~~~~~~~~~~~~~~~~~~

We broadly follow the tutorial at :cite:`ohman2015procedure`. For a
dataset called ``image.IMG``, do::

    lrowac2isis from = image.IMG to = image.cub

This will create so-called *even* and *odd* datasets, with names like
``image.vis.even.cub`` and ``image.vis.odd.cub``.

Run ``spiceinit`` on them to set up the SPICE kernels::

    spiceinit from = image.vis.even.cub
    spiceinit from = image.vis.odd.cub

followed by ``lrowaccal`` to adjust the image intensity::

    lrowaccal from = image.vis.even.cub to = image.vis.even.cal.cub
    lrowaccal from = image.vis.odd.cub  to = image.vis.odd.cal.cub

All these .cub files can be visualized with ``stereo_gui``. It can be
seen that instead of a single contiguous image we have a set of narrow
horizontal framelets, with some of these in the even and some in the odd
cub file. The framelets may also be recorded in reverse.

Production of seamless mapprojected images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is not needed for stereo, but may be useful for readers who would
like to produce image mosaics using ``cam2map``.

::

    cam2map from = image.vis.even.cal.cub to = image.vis.even.cal.map.cub
    cam2map from = image.vis.odd.cal.cub  to = image.vis.odd.cal.map.cub  \
      map = image.vis.even.cal.map.cub matchmap = true

Note how in the second ``cam2map`` call we used the ``map`` and
``matchmap`` arguments. This is to ensure that both of these output
images have the same resolution and projection. In particular, if more
datasets are present, it is suggested for all of them to use the same
previously created .cub file as a map reference.  That because stereo
works a lot better on mapprojected images with the same ground
resolution. For more details see :numref:`mapproj-example` and
:numref:`mapproj_with_cam2map`.

To verify that the obtained images have the same ground resolution, do::

    gdalinfo image.vis.even.cal.map.cub | grep -i "pixel size"
    gdalinfo image.vis.odd.cal.map.cub  | grep -i "pixel size"

(see :numref:`gdal_tools` regarding this tool).

The fusion happens as::

    ls image.vis.even.cal.map.cub image.vis.odd.cal.map.cub  > image.txt
    noseam fromlist = image.txt to = image.noseam.cub SAMPLES=73 LINES=73

The obtained file ``image.noseam.cub`` may still have some small artifacts
but should be overall reasonably good. 

Stitching the raw even and odd images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This requires ISIS newer than version 6.0, or the latest development code.

For each image in the stereo pair, stitch the even and odd datasets::

    framestitch even = image.vis.even.cal.cub odd = image.vis.odd.cal.cub \
      to = image.cub flip = true num_lines_overlap = 2

The ``flip`` flag is needed if the order of framelets is reversed
relative to what the image is expected to show.

The parameter ``num_lines_overlap`` is used to remove a total of this
many lines from each framelet (half at the top and half at the bottom)
before stitching, in order to deal with the fact that the even and odd
framelets have a little overlap, and that they also tend to have artifacts
due to some pixels flagged as invalid in each first and last framelet
row.

The CSM camera models will assume that this parameter is set at 2, so
it should not be modified. Note however that WAC framelets may overlap
by a little more than that, so resulting DEMs may have some artifacts
at framelet borders, as can be seen further down.

Creation of CSM camera models
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This process is still being ironed out. Generally speaking, for the
time being one should fetch the latest ALE and its conda environemnt
from GitHub, at:

    https://github.com/USGS-Astrogeology/ale

then create a script named ``gen_json.py``::

    #!/usr/bin/python

    import os, sys
    import json
    import ale

    prefix = sys.argv[1]

    if prefix.endswith(".cub") or prefix.lower().endswith(".img") \
      or prefix.endswith(".lbl"):
      prefix = os.path.splitext(prefix)[0]

    cub_file = prefix + '.cub'

    print("Loading cub file: " + cub_file)

    kernels = ale.util.generate_kernels_from_cube(cub_file, expand = True)

    usgscsm_str = ale.loads(cub_file, formatter = "ale", \
                        props={"kernels": kernels},
                        verbose = True)

    csm_isd = prefix + '.json'
    print("Saving: " + csm_isd)
    with open(csm_isd, 'w') as isd_file:
      isd_file.write(usgscsm_str)
  
and invoke it with either the ``even`` or ``odd`` .cub file as an argument.
Do not use the stitched .cub file as that one lacks camera information.
The obtained .json files can be renamed to follow the same
convention as the stitched .cub images.

At some point when a new version of ISIS is released (version > 6),
it may have a tool for creation of CSM camera models.

Running stereo
~~~~~~~~~~~~~~

::

    parallel_stereo --stereo-algorithm asp_mgm   \
      --left-image-crop-win 341 179 727 781      \
      --right-image-crop-win 320 383 824 850     \
      M119923055ME.cub M119929852ME.cub          \
      M119923055ME.json M119929852ME.json        \
      run/run

    point2dem run/run-PC.tif --orthoimage run/run-L.tif 
    hillshade run/run-DEM.tif 
    colormap run/run-DEM.tif -s run/run-DEM_HILLSHADE.tif 

As printed by ``stereo_pprc``, the convergence angle is about 27
degrees, which is a good number.

See :numref:`nextsteps` for a discussion about various stereo
speed-vs-quality choices.

.. figure:: ../images/CSM_WAC.png
   :name: CSM_WAC_example

   The produced colorized DEM and orthoimage for the CSM WAC camera
   example. The artifacts are due to issues stitching of even and odd
   framelets.

It can be seen that the stereo DEM has some linear artifacts. That is
due to the fact that the stitching does not perfectly integrate the
framelets.

An improved solution can be obtained by creating a low-resolution
version of the above DEM, mapprojecting the images on it, and then
re-running stereo, per (:numref:`mapproj-example`).

::

    point2dem --tr 0.03 run/run-PC.tif --search-radius-factor 5 -o \
      run/run-low-res
    mapproject --tr 0.0025638 run/run-low-res-DEM.tif              \
      M119923055ME.cub M119923055ME.json M119923055ME.map.tif 
    mapproject --tr 0.0025638 run/run-low-res-DEM.tif              \
      M119929852ME.cub M119929852ME.json M119929852ME.map.tif    
    parallel_stereo --stereo-algorithm asp_mgm                     \
      M119923055ME.map.tif M119929852ME.map.tif                    \
      M119923055ME.json M119929852ME.json                          \
      run_map/run run/run-low-res-DEM.tif    
    point2dem run_map/run-PC.tif --orthoimage run_map/run-L.tif 
    hillshade run_map/run-DEM.tif 
    colormap run_map/run-DEM.tif -s run_map/run-DEM_HILLSHADE.tif 

To create the low-resolution DEM we used a grid size which is about 10
times coarser than the one for the DEM created earlier. Note that the
same resolution is used when mapprojecting both images; that is very
important to avoid a large search range in stereo later. This is discussed
in more detail in :numref:`mapproj-example`.

.. figure:: ../images/CSM_WAC_mapproj.png
   :name: CSM_WAC_example_mapproj

   The produced colorized DEM and orthoimage for the CSM WAC camera
   example, when mapprojected images are used.

As can be seen in the second figure, there are somewhat fewer artifacts.
The missing lines in the DEM could be filled in if ``point2dem`` was run
with ``--search-radius-factor 4``, for example. 

Given that there exists a wealth of WAC images, one could also try to
get several more stereo pairs with similar illumination, run bundle
adjustment for all of them (:numref:`bundle_adjust`), run pairwise
stereo, create DEMs (at the same resolution), and then merge them with
``dem_mosaic`` (:numref:`dem_mosaic`). This may further attenuate the
artifacts as each stereo pair will have them at different
locations. See :numref:`stereo_pairs` for guidelines about how to
choose good stereo pairs.

