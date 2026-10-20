.. _sfs_blend:

sfs_blend
---------

The ``sfs_blend`` tool is a very specialized DEM blending program
developed for use in conjunction with Shape-from-Shading
(:numref:`sfs`). It replaces in an SfS-produced DEM height values that
are in permanent shadow with values from the initial guess DEM used
for SfS (which is typically the LOLA gridded DEM), with a transition
region between the two DEMs.

Motivation and an example of an invocation of this tool are given in
the :ref:`SfS usage <sfs_usage>` chapter.

.. _sfs_blend_fill:

Shadow fill modes
~~~~~~~~~~~~~~~~~

The ``--shadow-fill-mode`` option sets what fills a permanently
shadowed hole. Three modes are supported.

- ``lola``: fill the hole with the LOLA DEM. This is the earliest
  implementation.
  
- ``curved``: fill the hole with a surface grown from the surrounding
  SfS that curves gently downward into the hole. It is found by solving
  Laplace's equation with the surrounding SfS as the boundary and the
  local terrain curvature as the forcing, clamped so the surface can
  only dip down, never rise above its surroundings.

- ``lola-curved`` (the default): the per-pixel minimum of the SfS, the LOLA DEM,
  and the curved surface. The minimum keeps the fill from rising above the
  surrounding terrain even when LOLA itself does that.

The seam where the fill meets the lit terrain is softened with a feathered blur:
the blended DEM is blurred (with a standard deviation of
``--weight-blur-sigma``) and mixed with the original using a weight that is 1 on
the shadow-lit boundary and falls to zero over ``--lit-blend-length`` pixels
into the lit region and ``--shadow-blend-length`` pixels into the shadow. That
weight is itself smoothed to avoid banding. 

A reasonable invocation is::

    sfs_blend                             \
      --sfs-dem sfs.tif                   \
      --lola-dem lola.tif                 \
      --max-lit-image-mosaic max_lit.tif  \
      --image-threshold 0.005             \
      --shadow-fill-mode lola-curved      \
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
    use this operation. In the ``curved`` and ``lola-curved`` modes this
    value is also the standard deviation of the feathered boundary blur
    (:numref:`sfs_blend_fill`).

--min-blend-size <int (default 0)>
    Do not apply blending in shadowed areas whose pixel area is less
    than the square of this value, hence keeping there the SfS DEM.

--shadow-fill-mode <string (default: lola-curved)>
    What to put in a permanently shadowed hole before the boundary
    blending: ``lola``, ``curved``, or ``lola-curved``
    (:numref:`sfs_blend_fill`).

--shadow-fill-max-diameter <float (default 0)>
    The maximum hole diameter, in pixels, for which the curved surface
    is computed in the ``curved`` and ``lola-curved`` modes. Larger
    holes keep the plain LOLA or SfS fill. Ignored when
    ``--shadow-fill-mode`` is ``lola``.

--output-dem <arg>
    The blended output DEM to save.

--output-weight <arg>
    The weight showing the proportion of the SfS DEM in the blend with
    the LOLA DEM (1 is for purely SfS and 0 is for purely LOLA).

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the number of threads set in ~/.vwrc, or the number of cores if
    not set there.
 
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
