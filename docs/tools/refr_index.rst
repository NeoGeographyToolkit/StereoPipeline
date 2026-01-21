.. _refr_index:

refr_index
----------

The ``refr_index`` program computes the effective refraction index of water for
a specific satellite sensor band and given temperature and salinity. This has
applications for shallow water bathymetry (:numref:`shallow_water_bathy`).

Examples
~~~~~~~~

Single wavelength refraction index::

    refr_index --salinity 35 --temperature 20 \
        --wavelength 550

Provide a spectral response file::

    refr_index --salinity 35 --temperature 20 \
      --spectral-response WV03_Green.csv

The result is printed to standard output.

Methodology
~~~~~~~~~~~

.. _spectral_response:

Spectral response
^^^^^^^^^^^^^^^^^

It is assumed that a satellite band, such as Green for WorldView-3, records
light in a narrow range of wavelengths. The spectral response of the band
contains the sensitivity of the sensor to each wavelength in that band. An
effective wavelength is first computed as the weighted average of the
wavelengths, with the weights given by the spectral response. The refraction
index is then computed with this effective wavelength (:numref:`refr_model`).

The spectral response CSV file must have two columns, with the wavelength (in
nanometers) in the first column, the relative response for that wavelength in
the second one. Use commas, spaces, or tabs as separators. The first line must
be a header and will be ignored.

Wavelengths outside the range 300-1100 nm will be ignored (skipped). Wavelengths
between 300-1100 nm but outside 400-700 nm will be used but a warning will be
printed, as they are outside the validated range for the models
(:numref:`refr_model`).

Example::

    wavelength response
    521 0.78240309
    522 0.78540934
    523 0.79221241
    524 0.79904049
    525 0.80684987

Salinity and temperature
^^^^^^^^^^^^^^^^^^^^^^^^

The salinity is measured in parts per thousand (ppt). Typical seawater has a
salinity of 35 ppt and freshwater has a salinity of 0 ppt.

The temperature is measured in degrees Celsius. The model expects values
between 0°C and 30°C.

.. _refr_model:

Modeling
^^^^^^^^

Two methods are available for computing the refractive index:

* `Quan and Fry (1994)
  <https://github.com/geojames/global_refractive_index_532>`_ (default) -
  directly handles salinity without interpolation.

* `Parrish (2020)
  <https://research.engr.oregonstate.edu/parrish/index-refraction-seawater-and-freshwater-function-wavelength-and-temperature>`_ -
  uses linear interpolation between freshwater (S=0) and seawater (S=35)
  coefficients.

Use the ``--mode`` switch to choose between these two.

.. _refr_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--salinity <float>
    Salinity in parts per thousand (ppt). A good value is 35 ppt. Required.

--temperature <float>
    Temperature in degrees Celsius. Must be between 0 and 30. Required.

--mode <string (default: Quan-Fry)>
    Refractive index equation to use: ``Quan-Fry`` (default) or ``Parrish``. See
    :numref:`refr_model` for details.

--spectral-response <string (default: "")>
    CSV file containing the spectral response of the sensor band. See an example
    in :numref:`spectral_response`. Mutually exclusive with ``--wavelength``.

--wavelength <float>
    Calculate refraction index for single wavelength (nm). Validated range is
    400-700 nm. A warning will be printed for wavelengths between 300-1100 nm
    and outside the narrower range. There will be an error outside the 300-1100
    nm range. Mutually exclusive with ``--spectral-response``.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.