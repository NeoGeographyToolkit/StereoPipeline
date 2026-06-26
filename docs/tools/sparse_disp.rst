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
can instead be run as a standalone tool to produce sub-pixel *matches* on a
uniform coarse grid, at the full resolution of the input images. This is useful
when sparse but accurate correspondences are needed, without the cost of full
dense correlation.

Example, for full-resolution sub-pixel matches on a uniform 100 x 100 pixel
grid::

    sparse_disp left.tif right.tif \
      output/run                   \
      --coarse 100 --fine 100      \
      --subpixel-mode 1            \
      --save-match-file

Here, ``left.tif`` and ``right.tif`` are full-resolution images, such as the
``L.tif`` and ``R.tif`` produced by :ref:`parallel_stereo`. For best results,
the input images should be in reasonable alignment, or mapprojected with the
same grid size and projection.

The option ``--subpixel-mode 1`` fits a parabola around the correlation peak, so
each offset is computed to sub-pixel accuracy. This is not the best subpixel
method but is rather fast  (:numref:`subpixel_options`). The default,
``--subpixel-mode 0``, leaves the offset at the integer peak, which is the
behavior used for the seed disparity.

The option ``--save-match-file`` writes the matches to an ASP ``.match`` file, in
full-resolution left and right pixel coordinates. The file name is formed
automatically from the output prefix and the input image names. This file can be
examined with :ref:`ipmatch` (option ``--binary-to-txt``).

The match file naming convention in :numref:`ba_match_files` is respected, though
this program does not have the implementation as in :numref:`match_file_naming`
for excessively long input image files, which should be avoided.

The sampling rate is set with ``--coarse`` and ``--fine``, the coarsest and
finest search-point spacing in pixels. The grid is refined from the coarse to the
fine spacing only where neighboring disparities disagree. To produce a *uniform*
grid at a chosen spacing of ``N`` pixels, set both to ``N``.

The usual outputs, ``D_sub.tif`` and ``D_sub_spread.tif``, are still written,
unchanged, unaffected by any of these additional options. 

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

--subpixel-mode <integer (default: 0)>
    Sub-pixel refinement of the correlation peak. 0 = none, the peak stays at the
    integer location (as used for the low-resolution seed). 1 = fit a parabola to
    the peak and use its vertex, giving sub-pixel matches.

--save-match-file
    In addition to the disparity, write the matches to an ASP ``.match`` file, in
    full-resolution left and right pixel coordinates. The name is formed
    automatically from the output prefix and the input image names. Intended for
    standalone use to produce sparse sub-pixel matches (see ``--subpixel-mode``
    and ``--coarse`` / ``--fine``).

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
