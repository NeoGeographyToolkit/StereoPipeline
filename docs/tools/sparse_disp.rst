.. _sparse_disp:

sparse_disp
-----------

The ``sparse_disp`` program creates a low-resolution initial disparity
(``D_sub.tif``, :numref:`out_corr_files`) from the full-resolution stereo
images, computed at a sparse set of pixels for speed. This is meant for
difficult terrain, such as snow or ice, where the subsampled images used by the
default seed approach appear blank (:numref:`d_sub_corr`).

This program is invoked automatically by :ref:`parallel_stereo` with the option
``--corr-seed-mode 3``, and is customized with the ``parallel_stereo`` option
``--sparse-disp-options``.

An example is in :numref:`sparse_disp_example`.

``sparse_disp`` is written in Python and uses the ``numpy``, ``scipy``, and
``gdal`` modules.

Installation
~~~~~~~~~~~~

As of the 2026/06 build (:numref:`release`), the needed Python modules ship with
ASP, so no separate installation or environment setup are required.

For older builds, the ``numpy``, ``scipy``, and ``gdal`` Python modules must be
available. It is important to use the same version of ``python``, ``numpy``, and
``gdal`` as in ASP. Make adjustments below and then run::

    conda create -n sparse_disp -c conda-forge \
      python=3.12.2 numpy=1.26.4 gdal=3.8.1 scipy

ASP is told where to find these modules via::

    export ASP_PYTHON_MODULES_PATH=$HOME/miniconda3/envs/sparse_disp/lib/python3.x/site-packages

Adjust the ``conda`` installation location and ``python`` version as needed.

If ASP is installed with ``conda`` (:numref:`conda_intro`), and the ISIS version
is at least 9.0.0, the ASP environment already has all the needed modules. Then
``ASP_PYTHON_MODULES_PATH`` can point to the ``site-packages`` directory of the
ASP conda environment.

.. _dsub_match:

Producing sub-pixel matches
~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default ``sparse_disp`` creates only the low-resolution seed disparity. It
can instead be run as a standalone tool to produce sub-pixel *matches*, with
uncertainty, on a uniform coarse grid, at the full resolution of the input
images. This is useful when sparse but accurate correspondences are needed,
without the cost of full dense correlation.

Sample invocation
^^^^^^^^^^^^^^^^^

::

    sparse_disp left.tif right.tif      \
      output/run                        \
      --coarse 100 --fine 100           \
      --subpixel-mode 1                 \
      --metric cramer_rao               \
      --save-match-file                 \
      --match-points-geopackage out.gpkg

Here, ``left.tif`` and ``right.tif`` are full-resolution images, such as the
``L.tif`` and ``R.tif`` produced by :ref:`parallel_stereo`. For best results,
the input images should be in reasonable alignment, or mapprojected with the
same grid size and projection.

The option ``--subpixel-mode 1`` fits a parabola around the correlation peak, so
each offset is computed to sub-pixel accuracy. This is not the best subpixel
method but is rather fast (:numref:`subpixel_options`). The default,
``--subpixel-mode 0``, leaves the offset at the integer peak, which is the
behavior used for the seed disparity.

The sampling rate is set with ``--coarse`` and ``--fine``. These are the coarsest
and finest *spacing* between search points, in pixels. They are a spacing, not a
count of points. A value of ``N`` means one point every ``N`` pixels, so about
``(width / N)`` by ``(height / N)`` points across the image. The grid is refined
from the coarse to the fine spacing only where neighboring disparities disagree.
To produce a *uniform* grid, set both to the same value ``N``.

The spacing is reduced automatically, that is, made finer, when the requested
value is too large for the image. This keeps a minimum number of points across
the smaller dimension. On a small image a value such as 100 may therefore be
lowered, with a printed message.

The usual outputs, ``D_sub.tif`` and ``D_sub_spread.tif``, are still written,
unchanged, unaffected by any of these additional options.

.. _sparse_disp_match:

Match files
^^^^^^^^^^^

The above invocation saves the produced correspondences as interest point
matches, in several formats. Each matched pair carries a localization uncertainty
(:numref:`sparse_disp_uncertainty`).

The option ``--save-match-file`` writes the matches to an ASP ``.match`` file, in
full-resolution left and right pixel coordinates. The file name is formed
automatically from the output prefix and the input image names. This file can be
examined with :ref:`ipmatch` (option ``--binary-to-txt``).

The match file naming convention in :numref:`ba_match_files` is respected, though
this program does not implement the name-shortening of :numref:`match_file_naming`,
so excessively long input image file names should be avoided.

Using ``--matches-as-txt`` saves the matches instead as a plain text file
(:numref:`txt_match`), with the same naming convention but ending in ``.txt``.
Each line is ``x1 y1 unc1 x2 y2 unc2`` (:numref:`txt_format`), where the
uncertainty is the per-match ``sigma``.

.. _sparse_disp_geopackage:

Matches as a GeoPackage
^^^^^^^^^^^^^^^^^^^^^^^

The option ``--match-points-geopackage`` writes the matches to a GeoPackage
(``.gpkg``) of projected points, with fields modeled on :ref:`image_align`
(:numref:`image_align_match_points`).

The fields are an integer ``id``, then ``ref_x``, ``ref_y`` (reference point
projected coordinates), ``src_x``, ``src_y`` (corresponding source point
projected coordinates), ``dx``, ``dy`` (source minus reference offsets), the
pixel columns and rows (``ref_col``, ``ref_row``, ``src_col``, ``src_row``), the
``metric`` field (the value of the chosen ``--metric``, see
:numref:`sparse_disp_uncertainty`), and the correlation peak value as
``quality``.

Here, the reference is the first (template) image and the source is the second
(search) image.

As in :ref:`image_align`, the source point in the GeoPackage is reprojected into
the reference projection, so ``dx``, ``dy`` and the point geometry (the source
location) are all in the reference frame. This file can be inspected or
converted with ``ogrinfo`` and ``ogr2ogr``.

.. _sparse_disp_uncertainty:

Match localization uncertainty
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With ``--subpixel-mode 1``, the parabola fit to the correlation peak also yields a
per-match localization uncertainty ``sigma``, in pixels. It is written to the
``scale`` field of the ``.match`` file (the field :ref:`bundle_adjust` reads as
the per-pixel sigma) and to the ``metric`` field of the GeoPackage. The correlation
peak value, a match-quality score, is written to the ``interest`` and ``quality``
fields, respectively. Two ways of computing ``sigma`` are available, selected with
``--metric``:

- ``parabola_curvature``: ``sigma = sqrt(1/k_x + 1/k_y)``. Here ``k_x`` and
  ``k_y`` are the sharpness (curvature) of the correlation peak along the two
  image axes. Each is the discrete second derivative at the peak,
  ``k = 2*C - C(-1) - C(+1)``. Here ``C`` is the peak value. ``C(-1)`` and
  ``C(+1)`` are the correlation one pixel to each side along that axis. A sharp,
  well-defined peak gives a large ``k`` and a small ``sigma``. A flat peak gives
  a small ``k`` and a large ``sigma``. This uses the peak geometry only.

- ``cramer_rao``: ``sigma = sqrt((1-C) * (1/k_x + 1/k_y))``, where ``C`` is the
  correlation peak value. This is the Cramer-Rao estimate for locating the vertex
  of a noisy parabola :cite:`robinson2004fundamental,rao1945information`. The
  parabola variance ``1/k_x + 1/k_y`` is scaled by the residual ``1-C``, the
  unexplained mismatch. This reduces the weight of low-correlation matches. A
  match that is locally sharp but agrees poorly between the images still gets a
  larger ``sigma``.

Both are uncalibrated localization proxies, useful for *ranking* match reliability
rather than as absolute error bars (the same caveat as the detector scale used for
sparse interest points in :ref:`image_align`). A sharper, higher-correlation peak
corresponds to a more unique, better-pinned match.

The two measures were evaluated on a well-textured Mars CTX pair and on a
cross-modal desert pair (airborne lidar intensity versus NAIP imagery), the
latter containing both textured vegetation and smooth, textureless sand. On the
uniformly textured CTX pair the two measures agree and vary little, as every
match is about equally good.

On the desert pair both correctly flag the textureless areas (smooth sand,
roads) with a large ``sigma`` and the textured areas (shrubs, washes) with a
small one. The Cramer-Rao measure has roughly twice the dynamic range and tracks
the local image texture more closely, since it also accounts for the lower
correlation over textureless ground.

The Cramer-Rao measure is the default, as it is the more sensitive of the two.

The same two estimators are available densely, per pixel, from the ``--metric``
option of :ref:`corr_eval`, applied to a dense disparity rather than to these
sparse matches. These programs give very similar results when the sparse grid is
fine enough and the dense ``corr_eval`` output image is sampled at the same grid
points.

.. _sparse_disp_quality_metrics:

Other quality metrics
^^^^^^^^^^^^^^^^^^^^^

Two further ``--metric`` values measure match quality from the whole correlation
surface, rather than from the peak curvature alone. These are not localization
uncertainties. They need the full correlation surface. So they are available only
in ``sparse_disp``. They are not in the dense :ref:`corr_eval`, which samples the
correlation only at the disparity.

The first is ``peak_ratio``. It is the highest competing correlation peak divided
by the main peak value. The competing peak is the largest value outside a small
neighborhood of the main peak. A small value means a unique match. A large value
means a near-tie with another location. That happens with repetitive texture or an
ambiguous match. This is the correlation-surface analog of the ratio test of Lowe
(2004).

The second is ``snr``. It is a signal-to-noise ratio of the correlation peak. It
measures how far the peak rises above the background level of the surface. A large
value means a sharp, isolated peak well above the noise. A small value means the
peak barely stands out, so the match is unreliable.

It is important to note that the COSI-Corr correlator reports a similar quantity
to ``snr`` as implemented here. The natural definition is the peak minus the
mean background, divided by the peak. That is ``(Cmax - mean_background) /
Cmax``. When applied to the normalized cross-correlation image, that definition
saturates near one. The reason is that the mean background is close to zero.
Then the ratio is close to ``Cmax / Cmax``, which is one almost everywhere. It
carries little information.

The definition of ``snr`` implemented here divides by the standard deviation of
the background instead. That is ``(Cmax - mean_background) / std_background``.
This is the peak height in units of the background noise. It has a useful range.
It discriminates well between sharp and ambiguous matches.

These were tested on a desert data pair. All four metrics broadly agree on which
matches are reliable.

Note that for ``parabola_curvature``, ``cramer_rao``, and ``peak_ratio`` a small
value suggests a more reliable match. The opposite is the case with
``snr``.

The ``peak_ratio`` and ``snr`` metrics are *not* localization sigmas. Their
value in the ``.match`` ``scale`` field is not meaningful to
:ref:`bundle_adjust`.

Command-line options
~~~~~~~~~~~~~~~~~~~~

-x, --xsearch <integer (default: 968)>
    Initial x search range, in pixels.

-y, --ysearch <integer (default: 968)>
    Initial y search range, in pixels.

-t, --template <integer (default: 56)>
    Template size, in pixels.

-c, --coarse <integer>
    Initial search-point spacing, in pixels.

-f, --fine <integer (default: 64)>
    Final search-point spacing, in pixels.

-r, --refine_tol <integer (default: 8)>
    Pixel range tolerance for refinement.

-d, --output_dec_scale <integer (default: 8)>
    If set, also output a second copy of the offsets scaled by this amount.

-p, --output_pad <integer (default: 2)>
    Pad the output search range by this amount.

-s, --sigma_t_min <float (default: 0)>
    Matches are not computed if the standard deviation of the template is below
    this value.

-R, --R_lim_min <float (default: 32)>
    Lower bound on the minimum-neighbor-disparity parameter, used to reject
    points whose neighbors have too large a disparity range.

-l, --R_lim_max <float (default: 128)>
    Upper bound on the minimum-neighbor-disparity parameter.

-m, --mask_file <string>
    A GeoTIFF mask to reject template points that should not be searched
    (1 = search, 0 = skip).

-M, --Max_disp_range <float (default: 64)>
    Limit the unscaled output disparity range to this value.

-P, --processes <integer (default: number of CPUs)>
    The number of processes to use.

--subpixel-mode <integer (default: 0)>
    Sub-pixel refinement of the correlation peak. 0 = none, the peak stays at the
    integer location (as used for the low-resolution seed). 1 = fit a parabola to
    the peak and use its vertex, giving sub-pixel matches.

--metric <string (default: cramer_rao)>
    The per-match quality metric, written with ``--subpixel-mode 1``
    (see :numref:`sparse_disp_uncertainty` and
    :numref:`sparse_disp_quality_metrics`). ``parabola_curvature`` =
    ``sigma = 1/sqrt(k)``, from the parabola curvature ``k`` (small = good).
    ``cramer_rao`` = ``sigma = sqrt((1-C)/k)``, which also gives less weight to
    low-correlation matches (small = good). The same two are in :ref:`corr_eval`.
    ``peak_ratio`` = highest competing peak over the main peak, a uniqueness test
    (small = good). ``snr`` = peak height above the mean background in background
    standard deviations (large = good). ``peak_ratio`` and ``snr`` are
    for ``sparse_disp`` only.

--save-match-file
    In addition to the disparity, write the matches to an ASP ``.match`` file, in
    full-resolution left and right pixel coordinates. The name is formed
    automatically from the output prefix and the input image names. Intended for
    standalone use to produce sparse sub-pixel matches (see ``--subpixel-mode``
    and ``--coarse`` / ``--fine``).

--matches-as-txt
    With ``--save-match-file``, write the matches as a plain text file
    (:numref:`txt_match`), with the format ``x1 y1 unc1 x2 y2 unc2``
    (:numref:`txt_format`), instead of a binary ``.match`` file. The name is the
    same but ends in ``.txt``. The uncertainty is the per-match ``sigma`` (see
    ``--metric``).

--match-points-geopackage <string>
    In addition to the disparity, write the matches to this GeoPackage (``.gpkg``)
    file, as projected points. See :numref:`sparse_disp_geopackage` for the
    format.

--no_epipolar_fltr
    Disable filtering of disparities by distance from the epipolar vector.

-e, --epipolar_axis <integer>
    The axis that is epipolar (0 = x, 1 = y).

-n, --nodata-value <integer>
    The no-data value. Pixels at or below this value are not used.

-w, --fill-dist <float (default: 1000)>
    Fill gaps of this size or larger with smoothed values.

-D, --Debug
    Output debugging information and a text file of correlation-estimate points.
