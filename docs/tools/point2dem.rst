.. _point2dem:

point2dem
---------

The ``point2dem`` program produces a digital elevation model (DEM) in the
GeoTIFF format and/or an orthographic image from a set of point clouds. The
clouds can be created by ``parallel_stereo`` (:numref:`parallel_stereo`), or be
in CSV (:numref:`point2dem_csv`) or LAS (:numref:`point2dem_las`) format. 

The heights in the produced DEM are relative to a datum (ellipsoid). 
They are calculated by weighted averaging around each grid point
of the heights of points in the cloud (see ``--search-radius-factor``).

The grid size is set with ``--tr`` for the given projection. The grid points are
placed at integer multiples of the grid size, and the created DEM has a ground
footprint that is outwardly larger by half a grid pixel than the bounding box of
the grid points. If not set, the grid size is estimated automatically.

A custom extent can be specified with the option ``--t_projwin``. This will be
adjusted to ensure, as above, that the grid points are placed at integer
multiples of the grid size.

The obtained DEMs can be colorized or hillshaded (:numref:`genhillshade`),
visualized with ``stereo_gui`` (:numref:`stereo_gui`), mosaicked with
``dem_mosaic`` (:numref:`dem_mosaic`), or analyzed with GDAL tools
(:numref:`gdal_tools`).

.. _point2dem_proj:

Determination of projection
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the option ``--t_srs`` to set a desired output projection. The projection
should be local to the area of interest, in units of meter.

If this is not set:

 - The ``point2dem`` program inherits the projection from the input images, if
   those are mapprojected (:numref:`mapproj-example`) and the projection is not
   geographic. 

 - If the input is a LAS file having a projection that is not geographic, that
   will be used. 
   
 - If the input is a CSV file, the projection from ``--csv-srs`` will be used. 

If none of these are applicable, in the latest ASP (:numref:`release`),
``point2dem`` automatically finds a good local projection in meters. For ASP
3.4.0 and earlier, the default projection was geographic. 

For Earth, with the WGS84 datum, the auto-determined projection is UTM
with an auto-computed zone, except for latitudes above 84° North and below 80°
South, where the `NSDIC polar stereographic projections
<https://nsidc.org/data/user-resources/help-center/guide-nsidcs-polar-stereographic-projection>`_
are used.

For other Earth datums and other planetary bodies, the automatic determination
produces a local stereographic projection. The projection center is found
by computing the median of a sample of points in the cloud.

To ensure the automatic projection determination is always invoked, overriding
all other cases from above, use ``--t_srs auto``.

See the options ``--stereographic``, ``--orthographic``, ``--proj-lon``,
``--proj-lat`` for other ways to set the projection.

Examples
~~~~~~~~

Local stereographic projection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a DEM for Mars in a local stereographic projection, auto-guessing
the projection center and grid size.

::

    point2dem -r mars    \
      --stereographic    \
      --auto-proj-center \
      run/run-PC.tif

This creates ``run/run-DEM.tif``, which is a GeoTIFF file, with each 32-bit
floating point pixel value being the height above the datum (ellipsoid). The
datum and projection are saved in the geoheader and can be seen with ``gdalinfo
-proj4`` (:numref:`gdal_tools`).

ASP normally auto-guesses the planet (datum), otherwise the option ``-r`` can be
used. 

If desired to change the output no-data value (which can also be inspected with
``gdalinfo``), use the options ``--nodata-value``.

.. _point2dem_ortho_err:

Orthoimage and error image
^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    point2dem -r moon            \
      --auto-proj-center         \
      run/run-PC.tif             \
      --orthoimage run/run-L.tif \
      --errorimage

This produced a lunar DEM. The projection is found as in
:numref:`point2dem_proj`.

The left aligned image was used to create an orthoimage, by orthographically
projecting it onto the DEM. The resulting ``run/run-DRG.tif`` file will be saved
as a GeoTIFF image with the same geoheader as the DEM.

In addition, the file ``run/run-IntersectionErr.tif`` is created,
based on the 4th band of the ``PC.tif`` file, having the gridded
version of the closest distance between the pair of rays intersecting
at each point in the cloud (:numref:`triangulation_error`). This is
also called the *triangulation error*, but it is only one way of
evaluating the quality of the DEM.

Here we have explicitly specified the spheroid (``-r moon``), rather
than have it inferred automatically. The Moon spheroid will have a
radius of 1737.4 km.

Specify a projection string
^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    point2dem --t_srs '+proj=sinu +R=3396190 +no_defs' \
      run/run-PC.tif

This is the sinusoidal projection for Mars. The option ``gdalinfo --proj4``
can find the projection string in a GeoTIFF file.

Custom grid size with geographic projection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

    point2dem -r earth --geographic --tr 0.0001 run/run-PC.tif

It is important to note that here the grid size passed to ``--tr``, is in
degrees, rather than meters, because the projection is geographic. This
projection is *not recommended* except close to the equator.

It is best to let the grid size be computed automatically, so not specifying
``--tr`` at all, or otherwise use a multiple of the automatically determined
grid size (:numref:`post-spacing`).

If desired to change the range of longitudes from [0, 360] to [-180,
180], or vice-versa, post-process obtained DEM with ``image_calc``
(:numref:`image_calc`).

Polar stereographic projection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

     point2dem -r moon             \
       --stereographic             \
       --proj-lon 0 --proj-lat -90 \
       run/run-PC.tif

.. _point2dem_utm:

UTM projection
^^^^^^^^^^^^^^

::

    point2dem --utm 13 run/run-PC.tif

Or::

    proj="+proj=utm +zone=13 +datum=WGS84 +units=m +no_defs"
    point2dem --t_srs "$proj" run/run-PC.tif

The zone for the UTM projection depends on the region of interest. It can be
auto-guessed (:numref:`point2dem_proj`). The `Geoplanner
<https://www.geoplaner.com/>`_ website is a reliable source for UTM zone
information.

See the options ``--sinusoidal``, ``--mercator``, etc., in
:numref:`point2dem_options` for how to set other projections.

.. _point2dem_csv:

CSV files
^^^^^^^^^

The ``point2dem`` program can grid CSV files having longitude, latitude, and 
height values as::

     point2dem -r moon                               \
       --dem-spacing 10                              \
       --csv-format 1:lon,2:lat,3:height_above_datum \
       in.csv                                        \
       -o run/run

This will produce a DEM in projected coordinates (in meters, rather than
degrees), unless the option ``--geographic`` is passed in and the
``--dem-spacing`` is set to a fraction of a degree (:numref:`point2dem_proj`).

For input data in projected coordinates, one can set a projection and the CSV
format::

  proj="+proj=utm +zone=10 +datum=WGS84 +units=m +no_defs"
  format="1:easting,2:northing,3:height_above_datum"
  
then run::

    point2dem -r Earth       \
      --dem-spacing 10       \
      --csv-srs "$proj"      \
      --csv-format "$format" \
      --t_srs "$proj"        \
      in.csv                 \
      -o run/run

.. _point2dem_las:

LAS and COPC
^^^^^^^^^^^^

The ``point2dem`` program can grid LAS files, including compressed
(LAZ) and cloud-optimized (`COPC <https://copc.io/>`_) data. The processing is
done with `PDAL <https://pdal.io/en/latest/>`_, which is shipped with ASP. 
 
For example, to create a DEM from a LAS file, run::

    point2dem -r Earth --tr 10 in.las -o run/run

This assumes that the LAS file is in projected coordinates with the file having
the projection. If the points are in ECEF coordinates, a projection needs to be
set with ``--t_srs``.

For COPC files, which are potentially immense but spatially organized, the
option ``--copc-win`` must be set. It determines the bounds of desired data to
process, in projected coordinates. Example::

    point2dem --tr 2.0                       \
      --copc-win 636400 852260 638180 849990 \
      cloud.laz                              \
      -o run/run 

To process the full file, use the option ``--copc-read-all``. 

The determination of whether an input file is COPC or plain LAZ is done
by peeking at the relevant bits with PDAL.

This program can process LAS files created with ``point2las``
(:numref:`point2las`).
    
Multiple clouds
^^^^^^^^^^^^^^^

Several point clouds of different types can be passed in on input::

     point2dem -r earth                              \
       --dem-spacing 10                              \
       --csv-format 1:lon,2:lat,3:height_above_datum \
       in1.las in2.csv run/run-PC.tif -o combined 

Here LAS, CSV, and TIF point clouds (the latter obtained with
``parallel_stereo``) are fused together into a single DEM. 

The CSV file is in longitude, latitude, and height above datum format, but the
produced DEM will be in a projection in meters, unless borrowed from the LAS
file or explicitly set with ``--t_srs`` (:numref:`point2dem_proj`).

If it is desired to use the ``--orthoimage`` option with multiple
clouds, the clouds need to be specified first, followed by the
``L.tif`` images.

Ground-level or projected data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If a dataset is in a tif file with three bands, representing projected data or
Cartesian values in a local coordinate system, it can be gridded as::

    point2dem --input-is-projected \
      --t_srs <proj string>        \
      --tr 0.1                     \
      data.tif

See ``--input-is-projected`` for more details.

More examples are shown in :numref:`builddem`.

.. _molacmp:

Comparing with MOLA
~~~~~~~~~~~~~~~~~~~

When comparing the output of ``point2dem`` to laser altimeter data, like
MOLA, it is important to understand the different kinds of data that are
being discussed. By default, ``point2dem`` returns planetary radius
values in meters. These are often large numbers that are difficult to
deal with. If you use the ``-r mars`` option, the output terrain model
will be in meters of elevation with reference to the IAU reference
spheroid for Mars: 3,396,190 m. So if a post would have a radius value
of 3,396,195 m, in the model returned with the ``-r mars`` option, that
pixel would just be 5 m.

You may want to compare the output to MOLA data. MOLA data is released
in three 'flavors', namely: Topography, Radius, and Areoid. The MOLA
Topography data product that most people use is just the MOLA Radius
product with the MOLA Areoid product subtracted. Additionally, it is
important to note that all of these data products have a reference value
subtracted from them. The MOLA reference value is NOT the IAU reference
value, but 3,396,000 m.

In order to compare with the MOLA data, you can do one of two different
things. You could operate purely in radius space, and have ``point2dem``
create radius values that are directly comparable to the MOLA radius
data. You can do this by having ``point2dem`` subtract the MOLA
reference value, by using either ``-r mola`` or setting
``--semi-major-axis 3396000`` and ``--semi-minor-axis 3396000``.

Alternatively, to get values that are directly comparable to MOLA
*Topography* data, you will need to run ``point2dem`` with either
``-r mars`` or ``-r mola``, then run the ASP tool ``dem_geoid``
(:numref:`dem_geoid`). This program will convert the DEM height values
from being relative to the IAU reference spheroid or the MOLA spheroid
to being relative to the MOLA Areoid.

The newly obtained DEM will inherit the datum from the unadjusted DEM,
so it could be either of the two earlier encountered radii, but of
course the heights in it will be in respect to the areoid, not to this
datum. It is important to note that one cannot tell from inspecting a
DEM if it was adjusted to be in respect to the areoid or not, so there
is the potential of mixing up adjusted and unadjusted terrain models.

.. _post-spacing:

Post spacing
~~~~~~~~~~~~

Recall that ``parallel_stereo`` creates a point cloud file as its
output and that you need to use ``point2dem`` on to create a GeoTIFF
that you can use in other tools. The point cloud file is the result of
taking the image-to-image matches (which were created from the kernel
sizes you specified, and the subpixel versions of the same, if used)
and projecting them out into space from the cameras, and arriving at a
point in real world coordinates. Since ``stereo`` does this for every
pixel in the input images, the *default* value that ``point2dem`` uses
(if you don't specify anything explicitly) is the input image scale,
because there's an "answer" in the point cloud file for each pixel in
the original image.

However, as you may suspect, this is probably not the best value to use
because there really is not that much "information" in the data. The true
resolution of the output model is dependent on a whole bunch of things
(like the kernel sizes you choose to use) but also can vary from place
to place in the image depending on the texture.

The general rule of thumb is to produce a terrain model that has a
post spacing of about 3x the input image ground scale. This is based
on the fact that it is nearly impossible to uniquely identify a single
pixel correspondence between two images, but a 3x3 patch of pixels
provides improved matching reliability. This depends on the stereo
algorithm as well, however, with the ``asp_mgm`` algorithm producing a
higher effective DEM resolution than ``asp_bm``. As you go to numerically
larger post-spacings on output, you are averaging more point data
(that is probably spatially correlated anyway) together.

So you can either use the ``--dem-spacing`` argument to ``point2dem`` to
do that directly, or you can use your favorite averaging algorithm to
reduce the ``point2dem``-created model down to the scale you want.

If you attempt to derive science results from an ASP-produced terrain
model with the default DEM spacing, expect serious questions from
reviewers.

LAS or CSV clouds
~~~~~~~~~~~~~~~~~

The ``point2dem`` program can take as inputs point clouds in LAS and CSV
formats. These differ from point clouds created by stereo by being, in
general, not uniformly distributed. It is suggested that the user pick
carefully the output resolution for such files (``--dem-spacing``). If
the output DEM turns out to be sparse, the spacing could be increased,
or one could experiment with increasing the value of
``--search-radius-factor``, which will fill in small gaps in the output
DEM by searching further for points in the input clouds.

It is expected that the input LAS files have spatial reference
information such as WKT data. Otherwise it is assumed that the points
are raw :math:`x,y,z` values in meters in reference to the planet
center (ECEF).

Unless the output projection is explicitly set when invoking
``point2dem``, the one from the first LAS file will be used.

For LAS or CSV clouds it is not possible to generate triangulation (ray
intersection) error maps or ortho images.

For CSV point clouds, the option ``--csv-format`` must be set. The option
``--csv-srs`` containing a PROJ or WKT string needs to be specified to interpret
this data. If not provided, the value set in ``--t_srs`` will be used.

Output statistics
~~~~~~~~~~~~~~~~~

When ``point2dem`` concludes, it prints the *percentage of valid
pixels*, which is the number of pixels in the produced floating-point
image that are valid heights (not equal to the no-data value
saved in the geoheader) divided by the total number of pixels, and
then multiplied by 100. Note that if the DEM footprint is rotated in
the image frame, there will be blank regions at image corners, so
normally this percentage can be between 50 and 100 (or so) even when
stereo correlation was fully successful.

.. _point2dem_options:

Command-line options for point2dem
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

--nodata-value <float (default: -1e+6)>
    Set the nodata value.

--use-alpha
    Create images that have an alpha channel.

-n, --normalized
    Also write a normalized version of the DEM (for debugging).

-o, --output-prefix <string (default: "")>
    Specify the output prefix. The output DEM will be 
    ``<output prefix>-DEM.tif``.

--orthoimage
    Write an orthoimage based on the texture files passed in as inputs (after
    the point clouds). Must pass ``<output prefix>-L.tif`` when using this
    option. Produces ``<output prefix>-DRG.tif``.

--errorimage
    Write an additional image, whose values represent the triangulation ray
    intersection error in meters (the closest distance between the rays
    emanating from the two cameras corresponding to the same point on the
    ground). Filename is ``<output prefix>-IntersectionErr.tif``. If stereo
    triangulation was done with the option ``--compute-error-vector``, this
    intersection error will instead have 3 bands, corresponding to the
    North-East-Down coordinates of that vector (:numref:`triangulation_options`),
    unless the option ``--scalar-error`` is set.

--t_srs <string (default: "")>
    Specify the output projection as a GDAL projection string (WKT, GeoJSON, or
    PROJ). If not provided, will be read from the point cloud, if available.
    See :numref:`point2dem_proj` for details.

--t_projwin <xmin ymin xmax ymax>
    Specify a custom extent in georeferenced coordinates. This will be adjusted
    to ensure that the grid points are placed at integer multiples of the grid
    size.

--datum <string>
    Set the datum. This will override the datum from the input
    images and also ``--t_srs``, ``--semi-major-axis``, and
    ``--semi-minor-axis``.
    Options:

    - WGS84 (WGS_1984)
    - WGS72
    - NAD83
    - NAD27
    - Earth (alias for WGS84)
    - D_MOON (1,737,400 meters)
    - D_MARS (3,396,190 meters)
    - MOLA (3,396,000 meters)
    - Mars (alias for D_MARS)
    - Moon (alias for D_MOON)

--reference-spheroid <string (default: "")> 
    This is identical to the datum option.

--semi-major-axis <float (default: 0)>
    Explicitly set the datum semi-major axis in meters.

--semi-minor-axis <float (default: 0)>
    Explicitly set the datum semi-minor axis in meters.

--sinusoidal
    Save using a sinusoidal projection.

--mercator
    Save using a Mercator projection.

--transverse-mercator
    Save using a transverse Mercator projection.

--orthographic
    Save using an orthographic projection.

--stereographic
    Save using a stereographic projection. See also ``--auto-proj-center``.

--oblique-stereographic
    Save using an oblique stereographic projection.

--gnomonic
    Save using a gnomonic projection.

--lambert-azimuthal
    Save using a Lambert azimuthal projection.

--utm <zone>
    Save using a UTM projection with the given zone (:numref:`point2dem_utm`).

--geographic
    Save using the geographic projection (longitude and latitude).
    Recommended only close to the equator.

--proj-lon <float (default: NaN)>
    The center of projection longitude. If not specified, it will be computed
    automatically based on the estimated point cloud median (option
    ``--auto-proj-center``).

--proj-lat <float (default: NaN)>
    The center of projection latitude. See also ``--proj-lon``.

--auto-proj-center
    Automatically compute the projection center, based on the median of a sample
    of points in the cloud, unless ``--proj-lon`` and ``--proj-lat`` are set.
    This is the default in the latest build, but should be set for ASP 3.4.0 and
    earlier.

-s, --tr, --dem-spacing <float (default: 0)>
    Set output DEM resolution (in target georeferenced units per
    pixel). These units may be in meters or degrees, depending on the
    projection. If not specified, it will be computed automatically
    (except for LAS and CSV files). Multiple spacings can be set
    (in quotes) to generate multiple output files.

--search-radius-factor <float>
    Multiply this factor by ``--dem-spacing`` to get the search
    radius. The DEM height at a given grid point is obtained as the weighted
    average of heights of all points in the cloud within search radius of the
    grid point, with the weight given by the Gaussian of the distance from the
    grid point to the cloud point (see ``--gaussian-sigma-factor``). If not
    specified, the default search radius is the maximum of user-set
    ``--dem-spacing`` and internally estimated median DEM spacing, so the
    default factor is about 1.

--gaussian-sigma-factor <float (default: 0)>
    The value :math:`s` to be used in the Gaussian
    :math:`\exp(-s*(x/grid\_size)^2)` when computing the weight to give to a
    cloud point's contribution to a given DEM grid point, with *x* the
    distance in meters between the two. The default is -log(0.25) = 1.3863. A
    smaller value will result in a smoother terrain.

--csv-format <string (default: "")>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center),
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-srs``; the height above datum is in
    meters). Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-srs <string (default: "")>
    The PROJ or WKT string to use to interpret the entries in input CSV files.
    If not specified, ``--t_srs`` will be used. See also
    :numref:`point2dem_proj`.

--filter <string (default: "weighted_average")>
    The filter to apply to the heights of the cloud points within
    a given circular neighborhood when gridding (its radius is
    controlled via ``--search-radius-factor``).
    Options:

    * weighted_average (default),
    * min
    * max
    * mean
    * median
    * stddev
    * count (number of points)
    * nmad (= 1.4826 \* median(abs(X - median(X)))),
    * *n*-pct (where *n* is a real value between 0 and 100, for example,
      ``80-pct``, meaning, 80th percentile). Except for the default, the name of
      the filter will be added to the obtained DEM file name, e.g.,
      ``output-min-DEM.tif`` if ``--filter min`` is used.

--propagate-errors
    Write files with names ``<output prefix>-HorizontalStdDev.tif``
    and ``<output prefix>-VerticalStdDev.tif`` having the gridded
    stddev produced from bands 5 and 6 of the input point cloud,
    if this cloud was created with the ``parallel_stereo`` option
    ``--propagate-errors`` (:numref:`error_propagation`). The same
    gridding algorithm is used as for creating the DEM.

--remove-outliers-params <pct factor (default: 75.0 3.0)>
    Outlier removal based on percentage. Points with triangulation
    error larger than pct-th percentile times factor and points
    too far from the cluster of most points will be removed
    as outliers.

--use-tukey-outlier-removal
    Remove outliers above Q3 + 1.5*(Q3 - Q1). Takes precedence over
    ``--remove-outliers-params``.

--max-valid-triangulation-error <float (default: 0)>
    Outlier removal based on threshold. If positive, points with
    triangulation error larger than this will be removed from the
    cloud. Measured in meters. This option takes precedence over
    ``--remove-outliers-params`` and ``--use-tukey-outlier-removal``.

--scalar-error
    If the point cloud has a vector triangulation error, ensure that the
    intersection error produced by this program is the rasterized norm of
    that vector. See also ``--error-image``.
     
-t, --output-filetype <string (default: tif)>
    Specify the output file type.

--proj-scale <float (default: 1)>
    The projection scale (if applicable).

--false-northing <float (default: 0)>
    The projection false northing (if applicable).

--false-easting <float (default: 0)>
    The projection false easting (if applicable).

--input-is-projected
   Input data is already in projected coordinates, or is a point cloud in
   Cartesian coordinates in a box such as [-10, 10]^3. Need not be spatially
   organized. If both a top and bottom surface exists (such as indoors), one of
   them must be cropped out. Point (0, 0, 0) is considered invalid. Must specify
   a projection to interpret the data and the output grid size.
    
--rounding-error <float (default: 1/2^{10}=0.0009765625)>
    How much to round the output DEM and errors, in meters (more
    rounding means less precision but potentially smaller size on
    disk). The inverse of a power of 2 is suggested. See also 
    ``--point-cloud-rounding-error`` and ``--save-double-precision-point-cloud``
    for when the input point cloud is created (:numref:`triangulation_options`).

--dem-hole-fill-len <integer (default: 0)>
    Maximum dimensions of a hole in the output DEM to fill in, in pixels.
    For large holes, use instead ``dem_mosaic`` (:numref:`dem_mosaic_extrapolate`).

--orthoimage-hole-fill-len <integer (default: 0)>
    Maximum dimensions of a hole in the output orthoimage to fill
    in, in pixels. See also ``--orthoimage-hole-fill-extra-len``.
    For large holes, use instead ``mapproject`` (:numref:`mapproject`).

--orthoimage-hole-fill-extra-len <integer (default: 0)>
    This value, in pixels, will make orthoimage hole filling more
    aggressive by first extrapolating the point cloud. A small value
    is suggested to avoid artifacts. Hole-filling also works better
    when less strict with outlier removal, such as in
    ``--remove-outliers-params``, etc.

--max-output-size <columns rows>
    Creating of the DEM will be aborted if it is calculated to
    exceed this size in pixels.

--median-filter-params <window_size (integer) threshold (float)>
    If the point cloud height at the current point differs by more
    than the given threshold from the median of heights in the
    window of given size centered at the point, remove it as an
    outlier. Use for example 11 and 40.0.

--erode-length <integer (default: 0)>
    Erode input point clouds by this many pixels at boundary (after
    outliers are removed, but before filling in holes).

--copc-win <float float float float>
    Specify the region to read from a COPC LAZ file. The units are based on the
    projection in the file. This is required unless ``--copc-read-all`` is set.
    Specify as ``minx miny maxx maxy``, or ``minx maxy maxx miny``, with no
    quotes. See :numref:`point2dem_las`.

--copc-read-all
    Read the full COPC file, ignoring the ``--copc-win`` option.
        
--x-offset <float (default: 0)>
    Add a longitude offset (in degrees) to the DEM.

--y-offset <float (default: 0)>
    Add a latitude offset (in degrees) to the DEM.

--z-offset <float (default: 0)>
    Add a vertical offset (in meters) to the DEM.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.
