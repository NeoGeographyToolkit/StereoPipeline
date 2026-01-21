.. _bathy_water_masking:

Bathy water masking
===================

For shallow water bathymetry (:numref:`shallow_water_bathy`) it is important to
distinguish land pixels from water pixels. This allows the algorithm to focus on
underwater terrain while avoiding false depth estimates from land features.

A simple and commonly used approach is to threshold the near-infrared (NIR)
band 7, where water typically appears darker than land. This method is
described in :numref:`bathy_threshold_use`.

In complex coastal environments with vegetation, shadows, turbid water, or
shallow clear water, the NIR band alone may not provide sufficient separation.
This section describes alternative spectral indices that combine multiple bands
that may improve land-water discrimination.

Multispectral image bands
-------------------------

WorldView satellites capture multispectral imagery with eight bands.

.. list-table:: WorldView-3 multispectral bands
   :widths: 10 30
   :header-rows: 1

   * - Band
     - Name
   * - 1
     - Coastal
   * - 2
     - Blue
   * - 3
     - Green
   * - 4
     - Yellow
   * - 5
     - Red
   * - 6
     - Red Edge
   * - 7
     - NIR1 (Near-infrared 1)
   * - 8
     - NIR2 (Near-infrared 2)

Other vendors provide similar products.

Individual bands can be extracted from a multispectral image with
``gdal_translate`` (:numref:`gdal_tools`). For example, run this for the green
band (band 3)::

     b=3
     gdal_translate -b ${b} -co compress=lzw -co TILED=yes \
       -co BLOCKXSIZE=256 -co BLOCKYSIZE=256               \
       input.TIF input_b${b}.tif

The compression and tiling options help with the performance of ASP processing
later.

Water indices for land-water masking
------------------------------------

The following indices provide alternatives to band 7 (NIR1), as described
in :numref:`bathy_threshold_use`.

NDWI (Normalized Difference Water Index)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

NDWI is computed as:

.. math::

   \text{NDWI} = \frac{\text{Green} - \text{NIR}}{\text{Green} + \text{NIR}}

This index enhances the contrast between water and land. Water typically has
positive values, while land and vegetation have negative values. It is effective
for general water delineation and separates water from soil and terrestrial
vegetation well.

To compute NDWI using ``image_calc`` (:numref:`image_calc`), do::

     image_calc -c "(var_0 - var_1) / (var_0 + var_1)" \
       input_b3.tif input_b7.tif -o ndwi.tif

where ``input_b3.tif`` is the green band and ``input_b7.tif`` is the NIR band.

RNDVI (Reversed Normalized Difference Vegetation Index)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

RNDVI is computed as:

.. math::

   \text{RNDVI} = \frac{\text{Red} - \text{NIR}}{\text{Red} + \text{NIR}}

This is the inverse of the standard NDVI (which is high for vegetation). In
RNDVI, water appears bright (high values) while vegetation appears very dark
(low or negative values). This index is particularly effective in areas with
heavy vegetation along the shoreline, such as mangroves or dense forests, where
standard NIR masking may be ambiguous.

To compute RNDVI using ``image_calc`` run::

     image_calc -c "(var_0 - var_1) / (var_0 + var_1)" \
       input_b5.tif input_b7.tif -o rndvi.tif

where ``input_b5.tif`` is the red band and ``input_b7.tif`` is the NIR band.

OSI (Ocean/Sea Index)
~~~~~~~~~~~~~~~~~~~~~

OSI is computed as:

.. math::

   \text{OSI} = \frac{\text{Green} + \text{Red}}{\text{Blue}}

This index uses the ratio of longer visible wavelengths to blue. It can be
useful for specific water conditions, though it is often less robust in clear
shallow water than NDWI or RNDVI.

The ``image_calc`` command is::

     image_calc -c "(var_0 + var_1) / var_2" \
       input_b3.tif input_b5.tif input_b2.tif -o osi.tif

where ``input_b3.tif`` is the green band, ``input_b5.tif`` is the red band,
and ``input_b2.tif`` is the blue band.

Thresholding
------------

The resulting index images (``ndwi.tif``, ``rndvi.tif``, ``osi.tif``) can be
converted to binary water masks, as in :numref:`bathy_threshold_use`.

This requires computing an appropriate threshold. Here's an example that invokes
``otsu_threshold`` (:numref:`otsu_threshold`) for that purpose, with
``ndwi.tif``:

::

     otsu_threshold ndwi.tif

This will print the computed threshold to standard output. 

Mask creation and important notes
----------------------------------

When creating binary masks from these indices, it is important to note the
following.

**Polarity reversal:** Unlike the raw NIR band (band 7), where water is darker
than land, the spectral indices NDWI and RNDVI make water appear brighter. This
affects how binary masks are created from thresholds.

The mask convention is that **land pixels have value 1** (or positive values)
and **water pixels have value 0** (or nodata).

For the NIR band, water pixels are *below* the threshold and land pixels are
*above*, so the masking command is::

     threshold=225
     image_calc -c "gt(var_0, $threshold, 1, 0)" \
      input_b7.tif -o land_mask.tif

For NDWI and RNDVI, water pixels are *above* the threshold and land pixels are
*below*::

     threshold=0.38
     image_calc -c "lt(var_0, $threshold, 1, 0)" \
      ndwi.tif -o land_mask.tif

Note the reversed comparison operator (``gt`` vs ``lt``) to maintain the
convention that land=1 and water=0. 

**Site-specific performance:** The effectiveness of these indices varies
depending on local water conditions, bottom composition, turbidity, and
shoreline vegetation. 

In testing, NDWI and NIR band 7 showed the most consistent thresholds across
images. RNDVI might be effective for vegetated shorelines but could require
site-specific threshold adjustment. The results with OSI were not as promising
in our experiments (for KeyWest, FL).

It is recommended to compute all indices and visually inspect the produced masks in
``stereo_gui`` (:numref:`stereo_gui`) before selecting the most appropriate one
for your specific site.
