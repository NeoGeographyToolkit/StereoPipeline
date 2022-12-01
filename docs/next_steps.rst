.. _nextsteps:

The next steps
==============

This chapter will discuss in more detail ASP's stereo process and other
tools available to either pre-process the input images/cameras or to
manipulate ``parallel_stereo``'s outputs, both in the context of planetary ISIS
data and for Earth images. This includes how to (a) customize
``parallel_stereo``'s settings (b) use ``point2dem`` to create 3D terrain
models, (c) visualize the results, (d) align the obtained point clouds
to another data source, (e) perform 3D terrain adjustments in respect to
a geoid, etc.

.. _running-stereo:

Stereo Pipeline in more detail
------------------------------

Choice of stereo algorithm
~~~~~~~~~~~~~~~~~~~~~~~~~~

The most important choice a user has to make when running ASP is the 
stereo algorithm to use. By default, ASP runs as if invoked with::

   parallel_stereo --alignment-method affineepipolar  \
     --stereo-algorithm asp_bm --subpixel-mode 1      \
     <other options>
    
This invokes block-matching stereo with parabola subpixel mode, which
can be fast but not of high quality. Much better results are likely
produced with::

   parallel_stereo --alignment-method affineepipolar \
     --stereo-algorithm asp_mgm --subpixel-mode 3    \
     <other options>

which uses ASP's implementation of MGM (:numref:`asp_sgm`). For best
results one can use ``--subpixel-mode 2``, but that is very slow.

For steep terrains it is suggested to mapproject the images
(:numref:`mapproj-example`).

ASP also implements local alignment, when the input images are split
into tiles (with overlap) and locally aligned. This makes it possible
to use third-party algorithms in addition to the ones ASP implements. 

With ASP's own MGM algorithm this method can be run as::

   parallel_stereo --alignment-method local_epipolar \
     --stereo-algorithm asp_mgm                      \
     <other options>

ASP also ships with the following third-party stereo algorithms:
MGM (original author implementation), OpenCV SGBM, LIBELAS, MSMW,
MSMW2, and OpenCV BM. For more details see :numref:`stereo_algos`.

For example, the rather solid and reasonably fast OpenCV SGBM
implementation can be called as::

   parallel_stereo --alignment-method local_epipolar \
     --stereo-algorithm opencv_sgbm                  \
     --corr-tile-size 512 --sgm-collar-size 256      \ 
     <other options>

Above we used tiles of size 512 pixels with an extra padding of 256
pixels on each side, for a total size of 1024 pixels, to avoid using
too much memory. The defaults in ``parallel_stereo`` are double these
values, which work well with ASP's MGM which is more conservative with
its use of memory but can be too much for some other implementations.

It is suggested to not specify here ``--subpixel-mode``, in which case
it will use each algorithm's own subpixel implementation. Using
``--subpixel-mode 3`` will refine that result using ASP's subpixel
implementation. Using ``--subpixel-mode 2`` will be much slower but
likely produce even better results.

Next we will discuss more advanced parameters which rarely need to be
set in practice.

.. _settingoptionsinstereodefault:

Setting options in the ``stereo.default`` file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``parallel_stereo`` program can use a ``stereo.default`` file that
contains settings that affect the stereo reconstruction process.  Its
contents can be altered for your needs; details are found in
:numref:`stereodefault`. You may find it useful to save multiple
versions of the ``stereo.default`` file for various processing
needs. If you do this, be sure to specify the desired settings file by
invoking ``parallel_stereo`` with the ``-s`` option. If this option is
not given, the ``parallel_stereo`` program will search for a file
named ``stereo.default`` in the current working directory. If
``parallel_stereo`` does not find ``stereo.default`` in the current
working directory and no file was given with the ``-s`` option,
``parallel_stereo`` will assume default settings and continue.

An example ``stereo.default`` file is available in the top-level
directory of ASP. The actual file has a lot of comments to show you
what options and values are possible. Here is a trimmed version of the
important values in that file.

::

    alignment-method affineepipolar
    stereo-algorithm asp_bm 
    cost-mode 2
    corr-kernel 21 21
    subpixel-mode 1
    subpixel-kernel 21 21

For the ``asp_sgm`` and ``asp_mgm`` algorithms, the default correlation
kernel size is 5 x 5 rather than 21 x 21.

Note that the ``corr-kernel`` option does not apply to the external
algorithms.  Instead, each algorithm has its own options that need to
be set (:numref:`stereo_algos`).

All these options can be overridden from the command line, as described
in :numref:`cmdline`.

Alignment method
^^^^^^^^^^^^^^^^

For raw images, alignment is always necessary, as the left and right
images are from different perspectives.  Several alignment methods are
supported, including ``local_epipolar``, ``affineepipolar`` and
``homography`` (see :numref:`image_alignment` for details).

Alternatively, stereo can be performed with mapprojected images
(:numref:`mapproj-example`). In effect we take a smooth
low-resolution terrain and map both the left and right raw images onto
that terrain. This automatically brings both images into the same
perspective, and as such, for mapprojected images the alignment method
is always set to ``none``.

.. _stereo_algos:

Stereo algorithms
^^^^^^^^^^^^^^^^^

ASP can invoke several algorithms for doing stereo, some internally
implemented, some collected from the community, and the user can add
their own algorithms as well (:numref:`adding_algos`).

The list of algorithms is as follows. (See :numref:`stereo_algos_full`
for a full discussion.) 

**Algorithms implemented in ASP**

asp_bm (or specify the value '0')
   The ASP implementation of Block Matching. Search in the
   right image for the best match for a small image block in the
   left image. This is the fastest algorithm and works well for
   similar images with good texture coverage. How to set the block
   (kernel) size and subpixel mode is described further down.
   See also :numref:`asp_sgm`.

asp_sgm (or specify the value '1')
   The ASP implementation of the Semi-Global Matching (SGM)
   algorithm :cite:`hirschmuller_sgm_original`. This algorithm is
   slow and has high memory requirements but it performs better in
   images with less texture. See :numref:`asp_sgm` for important
   details on using this algorithm.

asp_mgm (or specify the value '2')
   The ASP implementation of the More Global Matching (MGM)
   variant of the SGM algorithm :cite:`facciolo2015mgm` to reduce
   high frequency artifacts in the output image at the cost of
   increased run time. See :numref:`asp_sgm` for important details on
   using this algorithm.

asp_final_mgm (or specify the value '3')
   Use MGM on the final resolution level and SGM on preceding
   resolution levels. This produces a result somewhere in between
   the pure SGM and MGM options.

**External implementations (shipped with ASP)**

mgm
   The MGM implementation by its authors. See :numref:`original_mgm`.

opencv_sgbm
   Semi-global block-matching algorithm from OpenCV 3. See
   :numref:`opencv_sgbm_options`.

libelas
   The LIBELAS algorithm :cite:`Geiger2010ACCV`. See
   :numref:`libelas`.

msmw and msmw2
   Multi-Scale Multi-Window algorithm (two versions provided). See
   :numref:`msmw`.

opencv_bm
   Classical block-matching algorithm from OpenCV 3. See
   :numref:`opencv_bm`.

Correlation parameters
^^^^^^^^^^^^^^^^^^^^^^

The option ``corr-kernel`` in ``stereo.default`` define what
correlation metric *(normalized cross correlation)* we'll be using and
how big the template or kernel size should be *(21 pixels square)*. A
pixel in the left image will be matched to a pixel in the right image
by comparing the windows of this size centered at them.

Making the kernel sizes smaller, such as 15 |times| 15, or even
11 |times| 11, may improve results on more complex features, such as steep
cliffs, at the expense of perhaps introducing more false matches or
noise.

These options only to the algorithms implemented in ASP (those whose
name is prefixed with ``asp_``). For externally implemented
algorithms, any options to them can be passed as part of the
``stereo-algorithm`` field, as discussed in
:numref:`stereo_algos_full`.

Subpixel refinement parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A highly critical parameter in ASP is the value of
``subpixel-mode``. When set to 1, ``parallel_stereo`` performs
parabola subpixel refinement, which is very fast but not very
accurate. When set to 2, it produces very accurate results, but it is
about an order of magnitude slower. When set to 3, the accuracy and
speed will be somewhere in between the other methods.

For the algorithms not implemented in ASP itself, not specifying this
field will result in each algorithm using its own subpixel mode.

The option ``subpixel-kernel`` sets the kernel size to use during
subpixel refinement *(also 21 pixels square)*.

.. _search_range2:

Search range determination
^^^^^^^^^^^^^^^^^^^^^^^^^^

Using these settings alone, ASP will attempt to work out the minimum and
maximum disparity it will search for automatically. However if you wish
to, you can explicitly set the extent of the search range by adding the
option::

    corr-search -80 -2 20 2

The search range determined automatically can then be tightened using
the option ``--max-disp-spread`` (:numref:`stereodefault`) before
full-image resolution happens.

It is suggested that these settings be used only if the run-time is
high or the inputs are difficult. For more details see
:numref:`search_range`. The inner working of stereo correlation can be
found in :numref:`correlation`.

.. _perform-stereo:

Performing stereo correlation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. figure:: images/p19-stereo-output_400px.png
   :name: p19-stereo-output
   :alt:  Outputs of the ``parallel_stereo`` program.

   These are the four viewable ``.tif`` files
   created by the ``parallel_stereo`` program. On the left are the two aligned,
   pre-processed images: (``results/output-L.tif`` and
   ``results/output-R.tif``). The next two are mask images
   (``results/output-lMask.tif`` and ``results/output-rMask.tif``),
   which indicate which pixels in the aligned images are good to use in
   stereo correlation. The image on the right is the "Good Pixel map",
   (``results/output-GoodPixelMap.tif``), which indicates (in gray)
   which were successfully matched with the correlator, and (in red)
   those that were not matched.

As already mentioned, the ``parallel_stereo`` program can be invoked for ISIS
images as::

     ISIS> parallel_stereo left_image.cub right_image.cub \
               -s stereo.default results/output

For DigitalGlobe/Maxar images the cameras need to be specified separately:

::

    parallel_stereo left.tif right.tif left.xml right.xml \
      -s stereo.default results/output

The string ``results/output`` is arbitrary, and in this case we will
simply make all outputs go to the ``results`` directory.

When ``parallel_stereo`` finishes, it will have produced a point cloud image.
:numref:`visualising` describes how to convert it to a digital
elevation model (DEM) or other formats.

The ``parallel_stereo`` program can be used purely for computing the
correlation (disparity) of two images, without cameras
(:numref:`correlator-mode`). 

The quality of correlation can be evaluated with the ``corr_eval``
program (:numref:`corr_eval`).

The ``parallel_stereo`` command can also take multiple input images,
performing multi-view stereo (:numref:`multiview`), though this
approach is rather discouraged as better results can be obtained with
bundle adjustment followed by pairwise stereo and merging of DEMs with
``dem_mosaic``.

Running the GUI frontend
~~~~~~~~~~~~~~~~~~~~~~~~

The ``stereo_gui`` program is a GUI frontend to
``parallel_stereo``. It is invoked with the same options as
``parallel_stereo`` (except for the more specialized ones such as
``--job-size-h``, etc.). It displays the input images, and makes it
possible to zoom in and select smaller regions to run stereo on. The
GUI is described in :numref:`stereo_gui`.

.. _cmdline:

Specifying settings on the command line
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All the settings given via the ``stereo.default`` file can be
over-ridden from the command line. Just add a double hyphen (``--``) in
front the option's name and then fill out the option just as you would
in the configuration file. For options in the ``stereo.default`` file
that take multiple numbers, they must be separated by spaces (like
``corr-kernel 25 25``) on the command line. Here is an example in
which we override the search range and subpixel mode from the command
line.

::

     ISIS> parallel_stereo E0201461.map.cub M0100115.map.cub \
               -s stereo.map --corr-search -70 -4 40 4       \
               --subpixel-mode 0 results/output

Stereo on multiple machines
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the input images are really large it may desirable to distribute
the work over several computing nodes. For that the ``--nodes-list``
option of ``parallel_stereo`` can be used. See
:numref:`parallel_stereo`.

.. _mapproj-example:

Running stereo with mapprojected images
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The way stereo correlation works is by matching a neighborhood of each
pixel in the left image to a similar neighborhood in the right image.
This matching process can fail or become unreliable if the two images
are too different, which can happen for example if the perspectives of
the two cameras are very different, the underlying terrain has steep
portions, or because of clouds and deep shadows. This can result in
large disparity search ranges, long run times, and ASP producing
3D terrains with noise or missing data.

ASP can mitigate this by *mapprojecting* the left and right images onto
some pre-existing low-resolution smooth terrain model without holes, and
using the output images to do stereo. In effect, this makes the images
much more similar and more likely for stereo correlation to succeed.

In this mode, ASP does not create a terrain model from scratch, but
rather uses an existing terrain model as an initial guess, and improves
on it.

.. _initial_terrain:

Choice of initial guess terrain model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For Earth, an existing terrain model can be, for example, the
Copernicus 30 m DEM from:

    https://portal.opentopography.org/raster?opentopoID=OTSDEM.032021.4326.3

or the NASA SRTM DEM (available on the same web site as above),
GMTED2010, USGS's NED data, or NGA's DTED data.

The Copernicus 30 m DEM heights are relative to the EGM96 geoid. Any such 
DEM must be converted to WGS84 ellipsoid heights, for any processing
to be accurate (:numref:`conv_to_ellipsoid`).

There exist pre-made terrain models for other planets as well, for
example the Moon LRO LOLA global DEM and the Mars MGS MOLA
DEM. Additionally, for Mars, consider downloading HRSC DEMs from:

   http://i-mars.eu/imars-datasets.php

or DEMs based on HRSC, CTX, and HiRISE cameras from:

    https://www.cosmos.esa.int/web/psa/ucl-mssl_meta-gsf

Check, as before, if your DEM is relative to the areoid rather than an
ellipsoid (:numref:`conv_to_ellipsoid`). Some Mars DEMs may have an
additional 190 meter vertical offset (such as the dataset
``molaMarsPlanetaryRadius0001.cub`` shipped with ISIS data), which can
be taken care of with ``image_calc`` (:numref:`image_calc`).

Alternatively, a low-resolution smooth DEM can be obtained by running
ASP itself as described in previous sections. In such a run, subpixel
mode may be set to parabola (``subpixel-mode 1``) for speed. To make it
sufficiently coarse and smooth, the resolution can be set to about 40
times coarser than either the default ``point2dem`` resolution or the
resolution of the input images. If the resulting DEM turns out to be
noisy or have holes, one could change in ``point2dem`` the search radius
factor, use hole-filling, invoke more aggressive outlier removal, and
erode pixels at the boundary (those tend to be less reliable).
Alternatively, holes can be filled with ``dem_mosaic``.

.. _conv_to_ellipsoid:

Conversion of initial guess terrain to ellipsoid heights
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is very important that your DEM be relative to a
datum/ellipsoid (such as WGS84), and *not* to a geoid/areoid, such as
EGM96.

A DEM relative to a geoid/areoid must be converted so that its heights
are relative to an ellipsoid. The ``gdalwarp`` program in recent
versions of GDAL and our own ``dem_geoid`` tool (:numref:`dem_geoid`)
can be used to perform the necessary conversions, if needed. For
example, with ``dem_geoid``, one can convert EGM96 heights to WGS84
with the command::

     dem_geoid --geoid egm96 --reverse-adjustment \
       dem.tif -o dem

This will create ``dem-adj.tif``.

.. _mapproj-res:

Resolution of mapprojection
^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is very important to specify the same resolution (ground sample
distance) when mapprojecting the images (option ``--tr`` for
``mapproject``, :numref:`mapproject`), in order for the images to have
the same scale and avoid big search range issues later in correlation.

Normally, ``mapproject`` is rather good at auto-guessing the resolution,
so this tool can be invoked with no specification of the resolution 
for the left image, then then ``gdalinfo`` can be used to find
the obtained pixel size, and that value can be used with the right image.

If these two images have rather different auto-determined resolutions,
it is suggested that the smaller ground sample distance be used for both,
or otherwise something in the middle. 

Using a ground sample distance which is too different than what is
appropriate can result in aliasing in mapprojected images and
artifacts in stereo.

Example for ISIS images
^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: images/stereo_mapproj_400px.png
   :name: mapproj-example-fig
   :alt: DEMs from camera geometry images and from mapprojected images.

   A DEM obtained using plain stereo (left) and stereo with
   mapprojected images (right). Their quality will be comparable for
   relatively flat terrain and the second will be much better for rugged
   terrain. The right image has some artifacts, but those are limited to
   areas close to the boundary. Things can be further improved with
   the ``asp_mgm`` algorithm (:numref:`running-stereo`).

This example illustrates how to run stereo with mapprojected images
for ISIS data. For an alternative approach using ``cam2map``, see
:numref:`mapproj_with_cam2map`.

We start with LRO NAC Lunar images M1121224102LE
and M1121209902LE from ASU's LRO NAC web site (https://wms.lroc.asu.edu/lroc/search), fetching them as::

    wget http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0015/DATA/ESM/2013111/NAC/M1121224102LE.IMG
    wget http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0015/DATA/ESM/2013111/NAC/M1121209902LE.IMG

We convert them to ISIS cubes using the ISIS program ``lronac2isis``,
then we use the ISIS tools ``spiceinit``, ``lronaccal``, and
``lrnonacecho`` to update the SPICE kernels and to do radiometric and
echo correction. This process is described in
:numref:`lro_nac_no_stitch`.  We name the two obtained .cub files
``left.cub`` and ``right.cub``.

Here we decided to run ASP to create the low-resolution DEM needed for
mapprojection, rather than get them from an external source. For speed,
we process just a small portion of the images::

     parallel_stereo left.cub right.cub            \
       --left-image-crop-win 1984 11602 4000 5000  \
       --right-image-crop-win 3111 11027 4000 5000 \
       --job-size-w 1024 --job-size-h 1024         \
       --subpixel-mode 1                           \
       run_nomap/run

(the crop windows can be determined using ``stereo_gui``,
:numref:`image_bounds`). The input images have resolution of about 1
meter, or :math:`3.3 \times 10^{-5}` degrees on the Moon. We create
the low-resolution DEM using a resolution 40 times as coarse, so we
use a grid size of 0.0013 degrees (we use degrees since the default
``point2dem`` projection invoked here is ``longlat``).

::

     point2dem --search-radius-factor 5 --tr 0.0013 run_nomap/run-PC.tif 

As mentioned earlier, some experimentation with the parameters used by
``point2dem`` may be necessary for this low-resolution DEM to be
smooth enough and with no holes.

We used ``--search-radius-factor 5`` to expand the DEM a
bit, to counteract future erosion at image boundary in stereo due to
the correlation kernel size. This is optional.

If this terrain is close to the poles, say within 25 degrees of
latitude, it is advised to use a stereographic projection, centered
either at the nearest pole, or close to the center of the current DEM.
Its center's longitude and latitude can be found with
``gdalinfo -stats``, which can then be passed to ``point2dem`` such as::

     point2dem --stereographic --proj-lon <lon_ctr> --proj-lat <lat_ctr> ...

By calling ``gdalinfo -proj4``, the PROJ.4 string of the obtained DEM
can be found, which can be used in mapprojection later, and with the
resolution switched to meters from degrees (see :numref:`dg-mapproj`
for more details).

Next, we mapproject the images onto this DEM, using the original
resolution of :math:`3.3 \times 10^{-5}` degrees, using
the ``mapproject`` program (:numref:`mapproject`):: 

     mapproject --tr 0.000033 run_nomap/run-DEM.tif           \
       left.cub left_proj.tif                                 \
       --t_projwin 3.6175120 25.5669989 3.6653695 25.4952127
     mapproject --tr 0.000033 run_nomap/run-DEM.tif           \
       right.cub right_proj.tif                               \
       --t_projwin 3.6175120 25.5669989 3.6653695 25.4952127

Notice that we used the same resolution for both images
(:numref:`mapproj-res`), and that we restricted the area of
computation using ``--t_projwin`` to again make the process faster.

Next, we do stereo with these mapprojected images::

     parallel_stereo --job-size-w 1024 --job-size-h 1024 \
       --subpixel-mode 3                                 \
       left_proj.tif right_proj.tif left.cub right.cub   \
       run_map/run run_nomap/run-DEM.tif

Notice that even though we use mapprojected images, we still specified
the original images as the third and fourth arguments. That because we
need the camera information from those files. The fifth argument is the
output prefix, while the sixth is the low-resolution DEM we used for
mapprojection. We have used here ``--subpixel-mode 3`` as this will be
the final point cloud and we want the increased accuracy.

Lastly, we create a DEM at 1 meter resolution::

     point2dem --nodata-value -32768 --tr 0.000033 run_map/run-PC.tif

We could have used a coarser resolution for the final
DEM, such as 4 meters/pixel, since we won't see detail at the level of 1
meter in this DEM, as the stereo process is lossy. This is explained in
more detail in :numref:`post-spacing`.

In :numref:`mapproj-example-fig` we show the effect of using
mapprojected images on accuracy of the final DEM.

Some experimentation on a small area may be necessary to obtain the best
results. Once images are mapprojected, they can be cropped to a small
shared region using ``gdal_translate -projwin`` and then stereo with
these clips can be invoked.

We could have mapprojected the images using the ISIS tool ``cam2map``,
as described in :numref:`mapproj_with_cam2map`.  The current approach
may be preferable since it allows us to choose the DEM to mapproject
onto, and it is faster, since ASP's ``mapproject`` uses multiple
processes, while ``cam2map`` is restricted to one process and one
thread.

.. _dg-mapproj:

Example for DigitalGlobe/Maxar images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In this section we will describe how to run stereo with mapprojected
images for DigitalGlobe/Maxar cameras for Earth. The same process can
be used for any satellite images from any vendor
(:numref:`other-mapproj`).

Unlike the previous section, here we will use an external DEM to
mapproject onto, rather than creating our own. We will use a variant of
NASA SRTM data with no holes. See :numref:`initial_terrain` for how
to fetch such a terrain. We will name this DEM ``ref.tif``. 

It is important to note that ASP expects the input low-resolution DEM
to be in reference to a datum ellipsoid, such as WGS84 or NAD83. If
the DEM is in respect to either the EGM96 or NAVD88 geoids, the ASP
tool ``dem_geoid`` can be used to convert the DEM to WGS84 or NAD83
(:numref:`dem_geoid`).  (The same tool can be used to convert back the
final output ASP DEM to be in reference to a geoid, if desired.)

Not applying this conversion might not properly negate the parallax seen
between the two images, though it will not corrupt the triangulation
results. In other words, sometimes one may be able to ignore the
vertical datums on the input but we do not recommend doing that. Also,
you should note that the geoheader attached to those types of files
usually does not describe the vertical datum they used. That can only be
understood by careful reading of your provider's documents.

Below are the commands for mapprojecting the input and then running
through stereo. You can use any projection you like as long as it
preserves detail in the images. Note that the last parameter in the
stereo call is the input low-resolution DEM. The dataset is the same as
the one used in :numref:`rawdg`.

.. figure:: images/examples/dg/Mapped.png
   :name: fig:dg-map-example
   :figwidth: 100%

   Example colorized height map and ortho image output.

Commands
^^^^^^^^

::

    mapproject -t rpc --t_srs "+proj=eqc +units=m +datum=WGS84" \
      --tr 0.5 ref.tif                                          \
      12FEB12053305-P1BS_R2C1-052783824050_01_P001.TIF          \
      12FEB12053305-P1BS_R2C1-052783824050_01_P001.XML          \
      left_mapped.tif
    mapproject -t rpc --t_srs "+proj=eqc +units=m +datum=WGS84" \
      --tr 0.5 ref.tif                                          \
      12FEB12053341-P1BS_R2C1-052783824050_01_P001.TIF          \
      12FEB12053341-P1BS_R2C1-052783824050_01_P001.XML          \
      right_mapped.tif
    parallel_stereo --subpixel-mode 1                           \
      --alignment-method none                                   \
      left_mapped.tif right_mapped.tif                          \
      12FEB12053305-P1BS_R2C1-052783824050_01_P001.XML          \
      12FEB12053341-P1BS_R2C1-052783824050_01_P001.XML          \
      dg/dg ref.tif

It is very important to specify the argument ``-t rpc`` to
``mapproject``, as otherwise the exact DG model will be used, which is
slower and not what ``parallel_stereo`` expects later.

The same appropriately chosen resolution setting (option ``--tr``)
must be used for both images to avoid long run-times and artifacts
(:numref:`mapproj-res`).

If the ``--t_srs`` option is not specified, it will be read from the
low-resolution input DEM.

The complete list of options for ``mapproject`` is described in
:numref:`mapproject`.

In the ``parallel_stereo`` command, we have used ``subpixel-mode 1`` which is
less accurate but reasonably fast. We have also used
``alignment-method none``, since the images are mapprojected onto the
same terrain with the same resolution, thus no additional alignment is
necessary. More details about how to set these and other ``parallel_stereo``
parameters can be found in :numref:`settingoptionsinstereodefault`.

Note here that any DigitalGlobe/Maxar camera file has two models in
it, the exact linescan model (which we name ``DG``), and its ``RPC``
approximation. Above, we have used the approximate ``RPC`` model for
mapprojection, since mapprojection is just a pre-processing step to
make the images more similar to each other, this step will be undone
during stereo triangulation, and hence using the ``RPC`` model is good
enough, while being much faster than the exact ``DG`` model.

When ``parallel_stereo`` runs with mapprojected images above,
it will run as if invoked with the``-t dgmaprpc`` stereo session, 
signaling that the images were mapprojected with ``RPC`` cameras
but the triangulation happens with the exact ``DG`` cameras.

.. _other-mapproj:

Mapprojection with other camera models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Stereo with mapprojected images can be used with any camera model
supported by ASP, including RPC (:numref:`rpc`), Pinhole
(:numref:`sfmgeneric`), CSM (:numref:`csm`), OpticalBar
(:numref:`kh4`), etc.  The ``mapproject`` command needs to be invoked
with ``-t rpc``, ``-t pinhole``, etc., and normally it auto-detects
this option (except when a camera file has both ``DG`` and ``RPC``
cameras). 

The cameras can also be bundle-adjusted, as discussed later.

As earlier, when invoking ``parallel_stereo``, the first two arguments
should be the mapprojected images, followed by the camera models,
output prefix, and the name of the DEM used for mapprojection.

The session name (``-t``) passed to ``parallel_stereo`` should be
``rpcmaprpc``, ``pinholemappinhole``, or just ``rpc``, ``pinhole``,
etc. Normally this is detected and set automatically, except
for the ``dg`` and ``rpc`` ambiguity, as discussed right above.

The stereo command with mapprojected images when the cameras are
stored separately is along the lines of::

    parallel_stereo -t rpc --stereo-algorithm asp_mgm  \
      left.map.tif right.map.tif left.xml right.xml    \
      run/run ref.tif

or::

    parallel_stereo -t pinhole --stereo-algorithm asp_mgm  \
      left.map.tif right.map.tif left.tsai right.tsai      \
      run/run ref.tif

and when the cameras are embedded in the images, it is::

    parallel_stereo -t rpc --stereo-algorithm asp_mgm \
      left.map.tif right.map.tif run/run ref.tif

If your cameras have been corrected with bundle adjustment
(:numref:`bundle_adjust`), one should pass ``--bundle-adjust-prefix``
to all ``mapproject`` and ``parallel_stereo`` invocations. See also
:numref:`ba_pc_align` for when alignment was used as well.

.. _multiview:

Multi-view stereo
~~~~~~~~~~~~~~~~~

ASP supports multi-view stereo at the triangulation stage. This mode is
somewhat experimental, and not used widely. We have obtained higher
quality results by doing pairwise stereo and merging the results, as
described later on in this section.

In the multiview scenario, the first image is set as reference,
disparities are computed from it to all the other images, and then joint
triangulation is performed :cite:`slabaugh2001optimal`. A
single point cloud is generated with one 3D point for each pixel in the
first image. The inputs to multi-view stereo and its output point cloud
are handled in the same way as for two-view stereo (e.g., inputs can be
mapprojected, the output can be converted to a DEM, etc.).

It is suggested that images be bundle-adjusted (:numref:`baasp`)
before running multi-view stereo.

Example (for ISIS with three images)::

     parallel_stereo file1.cub file2.cub file3.cub results/run

Example (for DigitalGlobe/Maxar data with three mapprojected images)::

     parallel_stereo file1.tif file2.tif file3.tif \
       file1.xml file2.xml file3.xml               \
       results/run input-DEM.tif

For a sequence of images, multi-view stereo can be run several times
with each image as a reference, and the obtained point clouds combined
into a single DEM using ``point2dem`` (:numref:`point2dem`).

The ray intersection error, the fourth band in the point cloud file, is
computed as twice the mean of distances from the optimally computed
intersection point to the individual rays. For two rays, this agrees
with the intersection error for two-view stereo which is defined as the
minimal distance between rays. For multi-view stereo this error is much
less amenable to interpretation than for two-view stereo, since the
number of valid rays corresponding to a given feature can vary across
the image, which results in discontinuities in the intersection error.

Other ways of combining multiple images
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As an alternative to multi-view stereo, point clouds can be generated
from multiple stereo pairs, and then a single DEM can be created with
``point2dem`` (:numref:`builddem`). Or, multiple DEMs can be
created, then combined into a single DEM with ``dem_mosaic``
(:numref:`dem_mosaic`).

In both of these approaches, the point clouds could be registered to a
trusted dataset using ``pc_align`` before creating a combined terrain
model (:numref:`pc-align-example`).

The advantage of creating separate DEMs and then merging them (after
alignment) with ``dem_mosaic``, compared to multiview triangulation, is
that this approach will not create visible seams, while likely it will
still increase the accuracy compared to the individual input DEMs.

.. _diagnosing_problems:

Diagnosing problems
~~~~~~~~~~~~~~~~~~~

Once invoked, ``parallel_stereo`` proceeds through several stages that are
detailed in :numref:`entrypoints`. Intermediate and final output
files are generated as it goes. See :numref:`outputfiles`, page for
a comprehensive listing. Many of these files are useful for diagnosing
and debugging problems.  For example, as :numref:`p19-stereo-output`
shows, a quick look at some of the TIFF files in the ``results/``
directory provides some insight into the process.

Perhaps the most accessible file for assessing the quality of your
results is the good pixel image (``results/output-GoodPixelMap.tif``).
If this file shows mostly good, gray pixels in the overlap area
(the area that is white in both the ``results/output-lMask.tif``
and ``results/output-rMask.tif`` files), then your results are just
fine. If the good pixel image shows lots of failed data, signified
by red pixels in the overlap area, then you need to go back and
tune your ``stereo.default`` file until your results improve. This
might be a good time to make a copy of ``stereo.default`` as you
tune the parameters to improve the results.

.. _p19-disparity:

.. figure:: images/p19-disparity_400px.png
   :alt: Disparity images produced using the ``disparitydebug`` tool.

   Disparity images produced using the
   ``disparitydebug`` tool. The two images on the left are the
   ``results/output-D-H.tif`` and ``results/output-D-V.tif`` files,
   which are normalized horizontal and vertical disparity components
   produced by the disparity map initialization phase. The two images on
   the right are ``results/output-F-H.tif`` and
   ``results/output-F-V.tif``, which are the final filtered,
   sub-pixel-refined disparity maps that are fed into the Triangulation
   phase to build the point cloud image. Since these MOC images were
   acquired by rolling the spacecraft across-track, most of the
   disparity that represents topography is present in the horizontal
   disparity map. The vertical disparity map shows disparity due to
   "wash-boarding", which is not due to topography but because of spacecraft
   movement. Note however that the horizontal and vertical disparity
   images are normalized independently. Although both have the same
   range of gray values from white to black, they represent
   significantly different absolute ranges of disparity.

Whenever ``parallel_stereo``, ``point2dem``, and other executables are run, they
create log files in given tool's results directory, containing a copy of
the configuration file, the command that was run, your system settings,
and tool's console output. This will help track what was performed so
that others in the future can recreate your work.

Another handy debugging tool is the ``disparitydebug`` program
(:numref:`disparitydebug`), which allows you to generate viewable
versions of the intermediate results from the stereo correlation
algorithm. ``disparitydebug`` converts information in the disparity
image files into two TIFF images that contain horizontal and vertical
components of the disparity (i.e.  matching offsets for each pixel in
the horizontal and vertical directions). There are actually three
flavors of disparity map: the ``-D.tif``, the ``-RD.tif``, and
``-F.tif``. You can run ``disparitydebug`` on any of them. Each shows
the disparity map at the different stages of processing.

::

    disparitydebug results/output-F.tif

If the output H and V files from ``disparitydebug`` look good, then the
point cloud image is most likely ready for post-processing. You can
proceed to make a mesh or a DEM by processing ``results/output-PC.tif``
using the ``point2mesh`` or ``point2dem`` tools, respectively.

:numref:`p19-disparity` shows the outputs of ``disparitydebug``.

If the input images are mapprojected (georeferenced) and the alignment
method is ``none``, all images output by stereo are georeferenced as
well, such as GoodPixelMap, D_sub, disparity, etc. As such, all these
data can be overlayed in ``stereo_gui``. ``disparitydebug`` also
preserves any georeference.

.. _longrun:

Dealing with long run-times
~~~~~~~~~~~~~~~~~~~~~~~~~~~

If ``stereo_corr`` takes unreasonably long, it may have encountered a
portion of the image where, due to noise (such as clouds, shadows, etc.)
the determined search range is much larger than what it should be. 

With the default block-matching algorithm, ``--stereo-algorithm
asp_bm``, the option ``--corr-timeout integer`` can be used to limit
how long each 1024 |times| 1024 pixel tile can take. A good value here
could be 300 (seconds) or more if your terrain is expected to have
large height variations.

If using the ``asp_sgm`` or ``asp_mgm`` algorithms, one can use a lower
value for ``--corr-memory-limit-mb`` (:numref:`asp_sgm`).  One may
also tighten ``--outlier-removal-params`` (:numref:`stereodefault`),
or mapproject the images (:numref:`mapproj-example`). A smaller manual
search range can also be specified (:numref:`search_range2`).

If a run failed partially during correlation, it can be resumed with
the ``parallel_stereo`` option ``--resume-at-corr``
(:numref:`parallel_stereo`). A ran can be started at the triangulation
stage after making changes to the cameras while reusing a previous run
with the option ``--prev-run-prefix``.

See also :numref:`handling_clouds` with considers the situation
that clouds are present in the input images. The suggestions there
may apply in other contexts as well.

On Linux, the ``parallel_stereo`` program writes in each output tile
location a file of the form::

    <tile prefix>-<program name>-resource-usage.txt

having the elapsed time and memory usage, as output by ``/usr/bin/time``.
This can guide tuning of parameters to reduce resource usage.

.. _visualising:

Visualizing and manipulating the results
----------------------------------------

When ``parallel_stereo`` finishes, it will have produced a point cloud
image, with a name like ``results/output-PC.tif``
(:numref:`outputfiles`), which can be used to create many kinds of
data products, such as DEMs, orthoimages, textured meshes, LAS files,
colormaps, hillshaded images, etc. DEMs can also be mosaicked,
aligned, etc.

.. _p19-osg:

.. figure:: images/p19-osg_400px.png
   :alt:  A visualization of a mesh.

   A visualization of a mesh.

Building a 3D mesh model
~~~~~~~~~~~~~~~~~~~~~~~~

The ``point2mesh`` command (:numref:`point2mesh`) can be used to
create a 3D textured mesh in the plain text ``.obj`` format that can be
opened in a mesh viewer such as MeshLab. The ``point2mesh`` program
takes the point cloud file and the left normalized image as inputs::

     point2mesh results/output-PC.tif results/output-L.tif

An example visualization is shown in :numref:`p19-osg`.

If you already have a DEM and an ortho image (:numref:`builddem`),
they can be used to build a mesh as well, in the same way as done
above::

     point2mesh results/output-DEM.tif results/output-DRG.tif

.. _builddem:

Building a digital elevation model and ortho image
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``point2dem`` program (:numref:`point2dem`) creates a Digital
Elevation Model (DEM) from the point cloud file.

::

     point2dem results/output-PC.tif

The resulting TIFF file is mapprojected and will contain georeference
information stored as a GeoTIFF header.

The tool will infer the datum and projection from the input images, if
present. You can explicitly specify a coordinate system (e.g., mercator,
sinusoidal) and a reference spheroid (i.e., calculated for the Moon,
Mars, or Earth). Alternatively, the datum semi-axes can be set or a
PROJ.4 string can be passed in.

::

     point2dem -r mars results/output-PC.tif

The output DEM will be named ``results/output-DEM.tif``. It can be
imported into a variety of GIS platforms. The DEM can be transformed
into a hill-shaded image for visualization (:numref:`genhillshade`).
The DEM can be examined in ``stereo_gui``, as::

    stereo_gui --hillshade results/output-DEM.tif

The ``point2dem`` program can also be used to orthoproject raw satellite
images onto the DEM. To do this, invoke ``point2dem`` just as before,
but add the ``--orthoimage`` option and specify the use of the left
image file as the texture file to use for the projection::

     point2dem results/output-PC.tif --orthoimage results/output-L.tif

The texture file ``L.tif`` must always be specified after the point
cloud file ``PC.tif`` in this command.

This produces ``results/output-DRG.tif``, which can be visualized in
``stereo_gui``.  See :numref:`p19-norm_ortho` on the right for the
output image.

To fill in any holes in the obtained orthoimage, one can invoke it with
a larger value of the grid size (the ``--tr`` option) and/or with a
variation of the options::

    --no-dem --orthoimage-hole-fill-len 100 --search-radius-factor 2 

The ``point2dem`` program is also able to accept output projection
options the same way as the tools in GDAL. Well-known EPSG, IAU2000
projections, and custom PROJ.4 strings can applied with the target
spatial reference set flag, ``--t_srs``. If the target spatial reference
flag is applied with any of the reference spheroid options, the
reference spheroid option will overwrite the datum defined in the target
spatial reference set. The following examples produce the same output.
However, the last two results will also show correctly the name of the
datum in the geoheader, not just the values of its axes.

::

    point2dem --t_srs "+proj=longlat +a=3396190 +b=3376200"          \
       results/output-PC.tif

    point2dem --t_srs http://spatialreference.org/ref/iau2000/49900/ \
       results/output-PC.tif

    point2dem --t_srs                                                \
      'GEOGCS["Geographic Coordinate System",                     
         DATUM["D_Mars_2000",
         SPHEROID["Mars_2000_IAU_IAG",3396190,169.894447223611]],
         PRIMEM["Greenwich",0],
         UNIT["degree",0.0174532925199433]]'                         \
      results/output-PC.tif

The ``point2dem`` program can be used in many different ways. The
complete documentation is in :numref:`point2dem`.

.. _p19-norm_ortho:

.. figure:: images/p19-norm_ortho_500px.png
   :alt: Normalized DEM and orthoimage.

   The image on the left is a normalized DEM (generated using 
   the ``point2dem`` option ``-n``), which shows low terrain
   values as black and high terrain values as white. The image on the
   right is the left input image projected onto the DEM (created using
   the ``--orthoimage`` option to ``point2dem``).

Orthorectification of an image from a different source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you have already obtained a DEM, using ASP or some other approach,
and have an image and camera pair which you would like to overlay on top
of this terrain, use the ``mapproject`` tool (:numref:`mapproject`).

Correcting camera positions and orientations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``bundle_adjust`` program can be used to adjust the camera positions
and orientations before running stereo. These adjustments only makes the
cameras self-consistent. For the adjustments to be absolute, it is
necessary to use ``bundle_adjust`` with ground control points. This tool
is described in :numref:`bundle_adjust`.

.. _pc-align-example:

Alignment to point clouds from a different source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Often the 3D terrain models output by ``parallel_stereo`` (point
clouds and DEMs) can be intrinsically quite accurate yet their actual
position on the planet may be off by several meters or several
kilometers, depending on the spacecraft. This can result from small
errors in the position and orientation of the satellite cameras taking
the pictures.

Such errors can be corrected in advance using bundle adjustment, as
described in the previous section. That requires using ground control
points, that may not be easy to collect. Alternatively, the images and
cameras can be used as they are, and the absolute position of the output
point clouds can be corrected in post-processing. For that, ASP provides
a tool named ``pc_align``. It aligns a 3D terrain to a much more
accurately positioned (if potentially sparser) dataset. Such datasets
can be made up of GPS measurements (in the case of Earth), or from laser
altimetry instruments on satellites, such as ICESat/GLASS for Earth,
LRO/LOLA on the Moon, and MGS/MOLA on Mars. Under the hood, ``pc_align``
uses the Iterative Closest Point algorithm (ICP) (both the
point-to-plane and point-to-point flavors are supported, and with
point-to-point ICP it is also possible to solve for a scale change).

The ``pc_align`` tool requires another input, an a priori guess for the
maximum displacement we expect to see as result of alignment, i.e., by
how much the points are allowed to move when the alignment transform is
applied. If not known, a large (but not unreasonably so) number can be
specified. It is used to remove most of the points in the source
(movable) point cloud which have no chance of having a corresponding
point in the reference (fixed) point cloud.

Here is how ``pc_align`` can be called (the denser cloud is specified
first).

.. figure:: images/examples/align_compare_500px.png
   :alt:  pc_align results
   :name: pc-align-fig

   Example of using ``pc_align`` to align a DEM obtained using stereo
   from CTX images to a set of MOLA tracks. The MOLA points are colored
   by the offset error initially (left) and after pc align was applied
   (right) to the terrain model. The red dots indicate more than 100 m
   of error and blue less than 5 m. The ``pc_align`` algorithm
   determined that by moving the terrain model approximately 40 m south,
   70 m west, and 175 m vertically, goodness of fit between MOLA and the
   CTX model was increased substantially.

::

    pc_align --max-displacement 200 --datum MOLA   \
      --save-inv-transformed-reference-points      \
      --csv-format '1:lon 2:lat 3:radius_m'        \
      stereo-PC.tif mola.csv

It is important to note here that there are two widely used Mars datums,
and if your CSV file has, unlike above, the heights relative to a datum,
the correct datum name must be specified via ``--datum``.  :numref:`molacmp`
talks in more detail about the Mars datums.

:numref:`pc-align-fig` shows an example of using ``pc_align``.
The complete documentation for this program is in :numref:`pc_align`.

.. _pc_align_validation:

Validation of alignment
~~~~~~~~~~~~~~~~~~~~~~~

The ``pc_align`` program can save the source cloud after being aligned
to the reference cloud and vice-versa, via
``--save-transformed-source-points`` and
``--save-inv-transformed-reference-points``. To validate that the
aligned source cloud is very close to the reference cloud, DEMs can be
made out of them with ``point2dem``, and those can be overlayed
in ``stereo_gui`` (:numref:`stereo_gui`) for inspection.

Alternatively, the ``geodiff`` program (:numref:`geodiff`) can be used
to compute the (absolute) difference between aligned DEMs, which can
be colorized with ``colormap`` (:numref:`colormap`). The ``geodiff``
tool can take the difference between a DEM and a CSV file as well.

Alignment and orthoimages
~~~~~~~~~~~~~~~~~~~~~~~~~

Two related issues are discussed here. The first is that sometimes,
after ASP has created a DEM, and the left and right images are
mapprojected to it, they are shifted in respect to each other. That is
due to the errors in camera positions. To rectify it, one has to run
``bundle_adjust`` first, then rerun the stereo and mapprojection tools,
with the adjusted cameras being passed to both via
``--bundle-adjust-prefix``.

Note that this approach will create self-consistent outputs, but not
necessarily aligned with pre-existing ground truth. That we deal with
next.

Once an ASP-generated DEM has been aligned to known ground data using
``pc_align``, it may be desired to create orthoimages that are also
aligned to the ground. That can be accomplished in two ways.

The ``point2dem --orthoimage`` approach be used, and one can pass to it
the point cloud after alignment and the ``L`` image before alignment
(all this tool does is copy pixels from the texture image, so position
errors are not a problem).

Alternatively, one can invoke the ``mapproject`` tool again. Yet, there
is a challenge, because this tool uses the original cameras, before
alignment, but will project onto the DEM after alignment, so the
obtained orthoimage location on the ground will be wrong.

The solution is to invoke ``bundle_adjust`` on the two input images
and cameras, while passing to it the transform obtained from
``pc_align`` via the ``--initial-transform`` option. This will shift
the cameras to the right place, and then ``mapproject`` can be called
with the adjusted cameras, using again the ``--bundle-adjust-prefix``
option. If all that is wanted is to shift the cameras, without doing
any actual adjustments, the tool can be invoked with the option
``--apply-initial-transform-only``.

Creating DEMs relative to the geoid/areoid
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The DEMs generated using ``point2dem`` are in reference to a datum
ellipsoid. If desired, the ``dem_geoid`` program can be used to convert
this DEM to be relative to a geoid/areoid on Earth/Mars respectively.
Example usage::

    dem_geoid results/output-DEM.tif

Converting to the LAS format
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If it is desired to use the ``parallel_stereo`` generated point cloud outside of
ASP, it can be converted to the LAS file format, which is a public file
format for the interchange of 3-dimensional point cloud data. The tool
``point2las`` can be used for that purpose (:numref:`point2las`). Example usage::

    point2las --compressed -r Earth results/output-PC.tif

.. _genhillshade:

Generating color hillshade maps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once you have generated a DEM file, you can use the ``colormap`` and
``hillshade`` tools to create colorized and/or shaded relief images.

To create a colorized version of the DEM, you need only specify the DEM
file to use. The colormap is applied to the full range of the DEM, which
is computed automatically. Alternatively you can specify your own min
and max range for the color map.

::

    colormap results/output-DEM.tif -o colorized.tif

See :numref:`colormap` for available colormap styles and illustrations
for how they appear.

To create a hillshade of the DEM, specify the DEM file to use. You can
control the azimuth and elevation of the light source using the ``-a``
and ``-e`` options.

::

    hillshade results/output-DEM.tif -o shaded.tif -e 25 -a 300

To create a colorized version of the shaded relief file, specify the DEM
and the shaded relief file that should be used::

    colormap results/output-DEM.tif -s shaded.tif -o color-shaded.tif

See :numref:`hrad-color` showing the images obtained with these
commands.

The complete documentation for ``colormap`` is in :numref:`colormap`,
and for ``hillshade`` in :numref:`hillshade`.

.. _hrad-color:

.. figure:: images/p19-colorized-shaded_500px.png

   The colorized DEM, the shaded relief image, and the
   colorized hillshade.

Building overlays for Moon and Mars mode in Google Earth
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Sometimes it may be convenient to see how the DEMs and orthoimages
generated by ASP look on top of existing images in Google Earth. ASP
provides a tool named ``image2qtree`` for that purpose. It creates
multi-resolution image tiles and a metadata tree in KML format that can
be loaded into Google Earth from your local hard drive or streamed from
a remote server over the Internet.

The ``image2qtree`` program can only be used on 8-bit image files with
georeferencing information (e.g. grayscale or RGB GeoTIFF images). In
this example, it can be used to process

| ``results/output-DEM-normalized.tif``, ``results/output-DRG.tif``,
  ``shaded.tif``,
| ``colorized.tif``, and ``shaded-colorized.tif``.

These images were generated respectively by using ``point2dem`` with the
``-n`` option creating a normalized DEM, the ``--orthoimage`` option to
``point2dem`` which projects the left image onto the DEM, and the images
created earlier with ``colormap``.

Here's an example of how to invoke this program::

    image2qtree shaded-colorized.tif -m kml --draw-order 100

:numref:`hrad-kml` shows the obtained KML files in Google
Earth.

The complete documentation is in :numref:`image2qtree`.

.. _hrad-kml:

.. figure:: images/p19-googlemars_500px.png

   The colorized hillshade DEM as a KML overlay.

Using DERT to visualize terrain models
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The open source Desktop Exploration of Remote Terrain (DERT) software
tool can be used to explore large digital terrain models, like those
created by the Ames Stereo Pipeline. For more information, visit
https://github.com/nasa/DERT.

.. _blender:

Using Blender to visualize meshes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`point2mesh` program will create ``.obj`` and ``.mtl`` files
that you can import directly into Blender (https://www.blender.org/).
Remember that ``.obj`` files don't particularly have a way to
specify 'units' but the 'units' of an ``.obj`` file written out by ASP
are going to be 'meters.'  If you open a large .obj model created by
ASP (like HiRISE), you'll need to remember to move the default
viewpoint away from the origin, and extend the clipping distance to a
few thousand (which will be a few kilometers), otherwise it may
'appear' that the model hasn't loaded (because 
your viewpoint is inside of it, and you can't see far enough).

The default step size for :ref:`point2mesh` is 10, which only samples
every 10th point, so you may want to read the documentation which
talks more about the ``-s`` argument to :ref:`point2mesh`.  Depending on how
big your model is, even that might be too small, and I'd be very
cautious about using ``-s 1`` on a HiRISE model that isn't cropped
somehow first.

You can also use :ref:`point2mesh` to pull off this trick with
terrain models you've already made (maybe with SOCET or something
else).  Our :ref:`point2mesh` program certainly knows how to read
our ASP ``*-PC.tif`` files, but it can also read GeoTIFFs.  So if
you have a DEM as a GeoTIFF, or an ISIS cube which is a terrain
model (you can use ``gdal_translate`` to convert them to GeoTIFFs),
then you can run :ref:`point2mesh` on them to get ``.obj`` and
``.mtl`` files.

.. _meshlab:

Using MeshLab to visualize meshes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

MeshLab is another program that can view meshes in 
``.obj`` files. It can be downloaded from::

  https://github.com/cnr-isti-vclab/meshlab/releases

and can be installed and run in user's directory without needing
administrative privileges.

Using QGIS to visualize terrain models
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The free and open source geographic information system QGIS
(https://qgis.org) as of version 3.0 has a 3D Map View feature that
can be used to easily visualize perspective views of terrain models.

After you use :ref:`point2dem` to create a terrain model (the
``*-DEM.tif`` file), or both the terrain model and an ortho image
via ``--orthoimage`` (the ``*-DRG.tif`` file), those files can be
loaded as raster data files, and the 'New 3D Map View' under the
View menu will create a new window, and by clicking on the wrench
icon, you can set the DEM file as the terrain source, and are able
to move around a perspective view of your terrain.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
