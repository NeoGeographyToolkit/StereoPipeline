.. _point2dem:

point2dem
---------

The ``point2dem`` program produces a digital elevation model (DEM) in
the GeoTIFF format and/or an orthographic image from a set of point
clouds. The clouds can be created by the ``parallel_stereo`` command
(:numref:`parallel_stereo`), or be in LAS or CSV format.

The heights in the produced DEM are relative to a datum (ellipsoid). 
They are calculated by weighted averaging around each grid point
of the heights of points in the cloud.

The output DEM is by default in the geographic coordinate system
(longitude and latitude).  Any projection can be specified via the
``--t_srs`` option. The grid size is set with ``--tr`` for the given 
projection, and the extent with ``--t_projwin``. The grid corners
are placed at integer multiples of the grid size, and the created
DEM has a ground footprint which is half a grid pixel larger
than the bounding box of the grid points.

The obtained DEMs can be colorized or hillshaded 
(:numref:`genhillshade`), visualized with ``stereo_gui``
(:numref:`stereo_gui`) or analyzed using GDAL tools
(:numref:`gdal_tools`).

Examples
~~~~~~~~

Create a DEM only::

    point2dem run/run-PC.tif

This creates ``run/run-DEM.tif``, which is a GeoTIFF file, with each
32-bit floating point pixel value being the height above the datum
(ellipsoid). The datum is saved in the geoheader and can be seen with
``gdalinfo`` (:numref:`gdal_tools`).

ASP normally auto-guesses the datum, otherwise the option ``-r`` can
be used. If desired to change the output no-data value (which can also
be inspected with ``gdalinfo``), use the options ``--nodata-value``.

If desired to change the range of longitudes from [0, 360] to [-180,
180], or vice-versa, post-process obtained DEM with ``image_calc``
(:numref:`image_calc`).

Create a DEM, orthoimage, and intersection error image::

    point2dem run/run-PC.tif -r moon --errorimage \
        --orthoimage run/run-L.tif

This produced the DEM, and also takes the left input image and
orthographically projects it onto the DEM. The resulting
``run/run-DRG.tif`` file will be saved as a GeoTIFF image with the
same geoheader as the DEM.

In addition, the file ``run/run-IntersectionErr.tif`` is created,
based on the 4th band of the ``PC.tif`` file, having the gridded
version of the closest distance between the pair of rays intersecting
at each point in the cloud (:numref:`triangulation_error`). This is
also called the *triangulation error*, but it is only one way of
evaluating the quality of the DEM.

Here we have explicitly specified the spheroid (``-r moon``), rather
than have it inferred automatically. The Moon spheroid will have a
radius of 1737.4 km.

Example with setting the grid size::

    point2dem --tr 0.0001 run/run-PC.tif

It is important to note that the grid size here, passed to ``--tr``,
is in degrees, because the default projection is in degrees. If you
set your projection with ``--t_projwin`` and it is in meters, the
value of ``--tr`` will be in meters too, so a reasonable value may be
``--tr 0.5``, perhaps.  It is best to let the grid size be computed
automatically, so not specifying ``--tr`` at all, or otherwise use a
multiple of the automatically determined grid size
(:numref:`post-spacing`).

Example with stereographic projection (for data close to poles)::

     point2dem --stereographic --proj-lon 0 --proj-lat -90 \
       run/run-PC.tif

Example with multiple input clouds::

     point2dem in1.las in2.csv run/run-PC.tif -o combined \
       --dem-spacing 0.001 --nodata-value -32768

Here LAS, CSV, and TIF point clouds (the latter obtained with
``parallel_stereo``) are fused together into a single DEM.
The option ``--dem-spacing`` is an alias for ``--tr``.

If it is desired to use the ``--orthoimage`` option with multiple
clouds, the clouds need to be specified first, followed by the
``L.tif`` images.

More examples are shown in :numref:`builddem`.

.. _molacmp:

Comparing with MOLA Data
~~~~~~~~~~~~~~~~~~~~~~~~

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
in three ‘flavors,’ namely: Topography, Radius, and Areoid. The MOLA
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

Using with LAS or CSV Clouds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
center.

Unless the output projection is explicitly set when invoking
``point2dem``, the one from the first LAS file will be used.

For LAS or CSV clouds it is not possible to generate intersection error
maps or ortho images.

For CSV point clouds, the option ``--csv-format`` must be set. If such a
cloud contains easting, northing, and height above datum, the option
``--csv-proj4`` containing a PROJ.4 string needs to be specified to
interpret this data (if the PROJ.4 string is set, it will be also used
for output DEMs, unless ``--t_srs`` is specified).

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

Command-line options for point2dem
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

--nodata-value <float (default: -3.40282347e+38)>
    Set the nodata value.

--use-alpha
    Create images that have an alpha channel.

-n, --normalized
    Also write a normalized version of the DEM (for debugging).

-o, --output-prefix <string>
    Specify the output prefix. The output DEM will be 
    ``<output prefix>-DEM.tif``.

--orthoimage
    Write an orthoimage based on the texture files passed in as
    inputs (after the point clouds). Filename is 
    ``<output prefix>-DRG.tif``.

--errorimage
    Write an additional image, whose values represent the
    triangulation ray intersection error in meters (the closest
    distance between the rays emanating from the two cameras
    corresponding to the same point on the ground). Filename
    is ``<output prefix>-IntersectionErr.tif``.

-t, --output-filetype <string (default: tif)>
    Specify the output file type.

--x-offset <float (default: 0)>
    Add a longitude offset (in degrees) to the DEM.

--y-offset <float (default: 0)>
    Add a latitude offset (in degrees) to the DEM.

--z-offset <float (default: 0)>
    Add a vertical offset (in meters) to the DEM.

--rotation-order <string (default: xyz)>
    Set the order of an Euler angle rotation applied to the 3D
    points prior to DEM rasterization.

--phi-rotation <float (default: 0)>
    Set a rotation angle phi.

--omega-rotation <float (default: 0)>
    Set a rotation angle omega.

--kappa-rotation <float (default: 0)>
    Set a rotation angle kappa.

--t_srs <string>
    Specify the output projection (PROJ.4 string). Can also be an
    URL or in WKT format, as for GDAL.

--t_projwin <xmin ymin xmax ymax>
    The output DEM will have corners with these georeferenced
    coordinates. The actual spatial extent (ground footprint) is
    obtained by expanding this box by half the grid size.

--datum <string>
    Set the datum. This will override the datum from the input
    images and also ``--t_srs``, ``--semi-major-axis``, and
    ``--semi-minor-axis``.
    Options:

    - WGS_1984
    - D_MOON (1,737,400 meters)
    - D_MARS (3,396,190 meters)
    - MOLA (3,396,000 meters)
    - NAD83
    - WGS72
    - NAD27
    - Earth (alias for WGS_1984)
    - Mars (alias for D_MARS)
    - Moon (alias for D_MOON)

--reference-spheroid <string>
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
    Save using a stereographic projection.

--oblique-stereographic
    Save using an oblique stereographic projection.

--gnomonic
    Save using a gnomonic projection.

--lambert-azimuthal
    Save using a Lambert azimuthal projection.

--utm <zone>
    Save using a UTM projection with the given zone.

--proj-lat <float>
    The center of projection latitude (if applicable).

--proj-lon <float>
    The center of projection longitude (if applicable).

--proj-scale <float>
    The projection scale (if applicable).

--false-northing <float>
    The projection false northing (if applicable).

--false-easting <float>
    The projection false easting (if applicable).

-s, --tr, --dem-spacing <float (default: 0)>
    Set output DEM resolution (in target georeferenced units per
    pixel). These units may be in degrees or meters, depending on your
    projection. If not specified, it will be computed automatically
    (except for LAS and CSV files). Multiple spacings can be set
    (in quotes) to generate multiple output files.

--search-radius-factor <float>
    Multiply this factor by ``dem-spacing`` to get the search radius.
    The DEM height at a given grid point is obtained as a weighted
    average of heights of all points in the cloud within search
    radius of the grid point, with the weights given by a Gaussian.
    If not specified, the default search radius is max(``dem-spacing``,
    default_dem_spacing), so the default factor is about 1.

--gaussian-sigma-factor <float (default: 0)>
    The value :math:`s` to be used in the Gaussian
    :math:`exp(-s*(x/grid\_size)^2)` when computing the DEM. The
    default is -log(0.25) = 1.3863.  A smaller value will result
    in a smoother terrain.

--csv-format <string>
    Specify the format of input CSV files as a list of entries
    column_index:column_type (indices start from 1).  Examples:
    ``1:x 2:y 3:z`` (a Cartesian coordinate system with origin at
    planet center is assumed, with the units being in meters),
    ``5:lon 6:lat 7:radius_m`` (longitude and latitude are in degrees,
    the radius is measured in meters from planet center),
    ``3:lat 2:lon 1:height_above_datum``,
    ``1:easting 2:northing 3:height_above_datum``
    (need to set ``--csv-proj4``; the height above datum is in
    meters). Can also use radius_km for column_type, when it is
    again measured from planet center.

--csv-proj4 <string>
    The PROJ.4 string to use to interpret the entries in input CSV
    files, if those files contain Easting and Northing fields. If
    not specified, ``--t_srs`` will be used.

--input-is-projected
    Treat the input coordinates as already in the projected coordinate
    system, avoiding the need to convert to geodetic coordinates.

--rounding-error <float (default: 1/2^{10}=0.0009765625)>
    How much to round the output DEM and errors, in meters (more
    rounding means less precision but potentially smaller size on
    disk). The inverse of a power of 2 is suggested.

--dem-hole-fill-len <integer (default: 0)>
    Maximum dimensions of a hole in the output DEM to fill in, in pixels.

--orthoimage-hole-fill-len <integer (default: 0)>
    Maximum dimensions of a hole in the output orthoimage to fill
    in, in pixels. See also ``--orthoimage-hole-fill-extra-len``.

--orthoimage-hole-fill-extra-len <integer (default: 0)>
    This value, in pixels, will make orthoimage hole filling more
    aggressive by first extrapolating the point cloud. A small value
    is suggested to avoid artifacts. Hole-filling also works better
    when less strict with outlier removal, such as in
    ``--remove-outliers-params``, etc.

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

--use-surface-sampling
    Use the older algorithm, interpret the point cloud as a surface
    made up of triangles and sample it (prone to aliasing).

--fsaa
    Oversampling amount to perform antialiasing. Obsolete, can be
    used only in conjunction with ``--use-surface-sampling``.

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.

--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.
