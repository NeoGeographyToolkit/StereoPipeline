.. _sfs_blend:

sfs_blend
---------

The ``sfs_blend`` tool is a very specialized DEM blending program
developed for use in conjunction with Shape-from-Shading
(:numref:`sfs`). It replaces in an SfS-produced DEM height values that
are in permanent shadow with values from the initial guess DEM used
for SfS (which is typically the LOLA gridded DEM), with a transition
region between the two DEMs.

By default the shadowed holes are filled with the LOLA DEM. The
``--shadow-fill-mode`` option offers alternatives that grow a smooth
floor from the surrounding SfS terrain instead (:numref:`sfs_blend_fill`).

Motivation and an example of an invocation of this tool are given in
the :ref:`SfS usage <sfs_usage>` chapter.

.. _sfs_blend_fill:

Shadow fill modes
~~~~~~~~~~~~~~~~~~

Filling the holes with LOLA can produce an artifact where LOLA sits
above the SfS floor, as the filled region then rises above the
surrounding terrain. The grown-floor modes avoid this.

The grown floor (used by all modes except ``lola``) is found per hole
by solving Laplace's equation in the hole with the surrounding SfS as
the boundary, and a forcing equal to the terrain curvature estimated
from the valid ground just outside the hole. This continues the local
shape inward: a flat neighborhood gives a flat floor, a bowl-shaped one
gives a bowl. The forcing is clamped to be non-negative, so the floor
can only dip down or stay level, never rise above its surroundings.

The modes differ in how this grown floor is used:

- ``lola``: fill the hole with the LOLA DEM (the default and original
  behavior).
- ``curved``: fill the hole with the grown floor.
- ``lola-curved``: use the lower of the grown floor and LOLA.
- ``sfs-curved``: keep the SfS and lower it only where it rises above
  the grown floor. The grown floor acts as a ceiling, so the real SfS
  floor and its fine detail are preserved and only spurious bumps are
  clipped. This is usually the best choice.
- ``sfs-lola-curved``: as ``sfs-curved``, but take the lower of the
  grown floor, the SfS, and LOLA, so LOLA can additionally pull the
  floor down where it is genuinely lower.

The seam where the fill meets the lit terrain is softened with a
feathered blur: the blended DEM is blurred (with a standard deviation
of ``--weight-blur-sigma``) and mixed with the original using a weight
that is 1 on the shadow-lit boundary and falls to zero over
``--lit-blend-length`` pixels into the lit region and
``--shadow-blend-length`` pixels into the shadow. That weight is itself
smoothed to avoid banding. A reasonable invocation is::

    sfs_blend                             \
      --sfs-dem sfs.tif                   \
      --lola-dem lola.tif                 \
      --max-lit-image-mosaic max_lit.tif  \
      --image-threshold 0.005             \
      --shadow-fill-mode sfs-lola-curved  \
      --shadow-fill-max-diameter 300      \
      --lit-blend-length 20               \
      --shadow-blend-length 20            \
      --weight-blur-sigma 10              \
      --min-blend-size 25                 \
      --output-dem blend.tif              \
      --output-weight weight.tif

Command-line options:

--sfs-dem <arg>
    The SfS DEM to process.

--lola-dem <arg>
    The LOLA DEM to use to fill in the regions in permanent shadow.

--max-lit-image-mosaic <arg>   
    The maximally lit image mosaic to use to determine the permanently
    shadowed regions.

--image-threshold <float>
    The value separating permanently shadowed pixels from lit pixels
    in the maximally lit image mosaic.

--lit-blend-length <float>
    The length, in pixels, over which to blend the SfS and LOLA DEMs
    at the boundary of the permanently shadowed region towards the lit
    region. Because of the weight blur below, the effective blending
    length can be somewhat bigger than this.

--shadow-blend-length <float>
    The length, in pixels, over which to blend the SfS and LOLA DEMs
    at the boundary of the permanently shadowed region towards the
    shadowed region. Because of the weight blur below, the effective
    blending length can be somewhat bigger than this.

--weight-blur-sigma <float (default 0)>
    The standard deviation of the Gaussian used to blur the weight
    that performs the transition from the SfS to the LOLA DEM. A
    higher value results in a smoother transition (this does not
    smooth the DEMs). The extent of the blur is about 7 times this
    deviation though it tapers fast to 0 before that. Set to 0 to not
    use this operation. In the grown-floor modes this value is also
    the standard deviation of the feathered boundary blur
    (:numref:`sfs_blend_fill`).

--min-blend-size <int (default 0)>
    Do not apply blending in shadowed areas of dimensions less than
    this, hence keeping there the SfS DEM.

--shadow-fill-mode <string (default: lola)>
    What to put in a permanently shadowed hole before the boundary
    blending: ``lola``, ``curved``, ``lola-curved``, ``sfs-curved``,
    or ``sfs-lola-curved`` (:numref:`sfs_blend_fill`).

--shadow-fill-max-diameter <float (default 0)>
    The maximum hole diameter, in pixels, to which the grown floor is
    applied in the grown-floor modes. Larger holes keep the LOLA fill.
    Ignored when ``--shadow-fill-mode`` is ``lola``.

--output-dem <arg>
    The blended output DEM to save.

--output-weight <arg>
    The weight showing the proportion of the SfS DEM in the blend with
    the LOLA DEM (1 is for purely SfS and 0 is for purely LOLA).

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create BigTiff files.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
