.. _sparse_disp:

sparse_disp
-----------

The ``sparse_disp`` program creates a low-resolution initial disparity
(``output_prefix-D_sub.tif``) from the full-resolution stereo images, computed
at a sparse set of pixels for speed. This is meant for difficult terrain, such
as snow or ice, where the subsampled images used by the default seed approach
appear blank (:numref:`d_sub_corr`).

It is invoked automatically by ``parallel_stereo`` with ``--corr-seed-mode 3``,
and is customized with the ``parallel_stereo`` option ``--sparse-disp-options``.

For a worked example, see :numref:`sparse_disp_example`.

``sparse_disp`` is written in Python and uses the ``numpy``, ``scipy``, and
``gdal`` modules.

Installation
~~~~~~~~~~~~

As of the 2026-06 ASP build, the needed Python modules ship with ASP, so no
separate environment or ``ASP_PYTHON_MODULES_PATH`` is required. The steps below
are needed only for earlier releases.

To use this tool, the ``numpy``, ``scipy``, and ``gdal`` Python modules must be
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

-l, --R_limax_disp_range <float (default: 128)>
    Upper bound on the minimum-neighbor-disparity parameter.

-m, --mask_file <string>
    A GeoTIFF mask to reject template points that should not be searched
    (1 = search, 0 = skip).

-M, --Max_disp_range <float (default: 64)>
    Limit the unscaled output disparity range to this value.

-P, --processes <integer (default: number of CPUs)>
    The number of processes to use.

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
