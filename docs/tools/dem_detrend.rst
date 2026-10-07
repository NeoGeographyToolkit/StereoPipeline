.. _dem_detrend:

dem_detrend
-----------

The ``dem_detrend`` program removes a smooth spatial bias (a low-frequency
trend) from a DEM, using scattered ground-control points, such as ICESat-2
ATL24 photon depths or a lidar survey. It fits a thin-plate spline
:cite:`wood2003thin` to the control-point-minus-DEM residual, evaluates that
smooth surface over the whole DEM, and adds it, so the DEM is pulled toward the
control points.

Overview
~~~~~~~~

This program corrects a slowly varying warp that a rigid alignment cannot
remove. The recommended order is to first align the DEM to the control points
with ``pc_align`` (:numref:`pc_align`), which removes a translation, rotation,
and scale, and then run ``dem_detrend`` on the aligned DEM to remove the
residual smooth trend. The control points are a form of prior ground truth
(:numref:`existing_terrain`).

A motivating use is shallow-water bathymetry, where a stereo DEM of the sea
floor (:numref:`bathy_intro`, :numref:`aerial_bathymetry`) is detrended against
ICESat-2 ATL24 depths to reduce a residual depth-dependent bias.

This is a Python program. It runs in the same ``bathy`` conda environment as
``bathy_threshold_calc`` (:numref:`bathy_threshold_calc`).

How it works
~~~~~~~~~~~~

The DEM is sampled at each control point, and the residual (control point minus
DEM height) is formed. Gross outliers are removed with a Tukey rule (1.5 times
the interquartile range), unless ``--outlier-method none`` is set. A reduced-rank
penalized thin-plate spline is fit to the residuals, with the smoothing weight
chosen by generalized cross validation. The fitted surface is then evaluated at
every DEM cell and added back.

The rank is kept low (``--num-knots``, default 150). A DEM bias is a smooth,
low-frequency field. A low rank is faster to evaluate at full resolution and
gives a cleaner surface. A high rank is slower and tends to overfit. Raise
``--num-knots`` only if a genuinely finer trend is wanted, at proportional cost.

Where control points are sparse (ICESat-2 tracks are narrow lines), the trend is
an extrapolation far from the data. The ``--max-distance`` option tapers the
correction to zero beyond a given distance from the control points, so distant
cells are left unchanged.

Preparation
~~~~~~~~~~~

The DEM and the control points must be in the same horizontal projection and the
same vertical datum. The ``x`` and ``y`` columns of the CSV are read as easting
and northing in the DEM's projection, and ``z`` as a height in the DEM's
vertical datum. Convert heights with ``dem_geoid`` (:numref:`dem_geoid`) if
needed, and align first with ``pc_align`` (:numref:`pc_align`), which also reads
such CSV control points.

Example
~~~~~~~

Install the ``bathy`` conda environment (:numref:`dem_detrend_dependencies`),
then run::

    ~/miniconda3/envs/bathy/bin/python $(which dem_detrend) \
        --dem bathy_dem.tif --csv atl24_points.csv          \
        --output-dem bathy_dem_detrend.tif

Here it is assumed that ASP's ``bin`` directory is in the path, otherwise the
full path to this Python script must be specified above.

The program prints robust before-and-after residual statistics (bias, NMAD,
RMSE) at the control points, so the reduction in bias can be confirmed. To
inspect the fitted surface itself, pass ``--output-trend``.

If the CSV has columns in a different order, or is whitespace-separated, set
``--csv-cols`` and ``--csv-delimiter``. A header line, and any line starting
with ``#``, are skipped automatically.

.. _dem_detrend_dependencies:

Dependencies
~~~~~~~~~~~~

This tool needs Python 3 with ``numpy``, ``scipy``, and ``gdal``. These are among
the packages used by ``bathy_threshold_calc.py``
(:numref:`bathy_threshold_calc`), so its ``bathy`` environment can be reused. To
create it, run::

     conda create --name bathy -c conda-forge \
       python gdal numpy scipy matplotlib
     conda activate bathy

Command-line options for dem_detrend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--dem <filename>
    The input DEM to detrend, as a GeoTIFF.

--csv <filename>
    The control points, as a CSV or text file. The ``x`` and ``y`` columns must
    be in the DEM's projection (easting, northing) and ``z`` in its vertical
    datum.

--output-dem <filename>
    The output detrended DEM. Default: the input name with a ``_detrend``
    suffix.

--output-trend <filename>
    Also write the fitted trend (the amount added to the DEM) to this GeoTIFF.

--csv-cols <string (default: "1 2 3")>
    The 1-based column indices in the CSV for ``x``, ``y``, ``z``.

--csv-delimiter <string (default: ",")>
    The CSV field delimiter. Use the empty string for whitespace.

--num-knots <integer (default: 150)>
    The rank of the thin-plate spline. More knots give a finer trend at higher
    cost and risk overfitting.

--num-fit-points <integer (default: 80000)>
    Subsample the control points to this many for the fit.

--max-distance <double (default: 0)>
    Taper the correction to zero beyond this distance (meters) from the control
    points, with a cosine falloff. Zero means no taper, so the trend is applied
    over the whole DEM.

--outlier-method <tukey|none (default: tukey)>
    Reject residual outliers before the fit: ``tukey`` (1.5 times the
    interquartile range) or ``none``.

--tile-rows <integer (default: 256)>
    Evaluate the trend in blocks of this many DEM rows, to limit memory.

--nodata-value <double>
    Output nodata value. Default: inherited from the input DEM.

--seed <integer (default: 0)>
    Random seed for the fit subsample, for reproducibility.

-h, --help
    Display the help message.
