.. _refr_index:

refr_index
----------

The ``refr_index`` program computes the effective refraction index of water for
a specific satellite sensor band and given temperature and salinity. This has
applications for shallow water bathymetry (:numref:`shallow_water_bathy`).

Example
~~~~~~~

::
    
  refr_index --salinity 35 --temperature 20 \ 
    --spectral-response WV03_NIR.csv

The result is printed to standard output.

Methodology
~~~~~~~~~~~

.. _spectral_response:

Spectral response
^^^^^^^^^^^^^^^^^

It is assumed that a satellite band, such as Green for WorldView-3, records
light in a narrow range of wavelengths. The spectral response of the band
contains the sensitivity of the sensor to each wavelength in that band.
The computed effective refraction index is the weighted average of the
refraction indices for each wavelength, with the weights given by the spectral
response. 

The spectral response CSV file must have two columns, with the wavelength (in
nanometers) in the first column, the relative response for that wavelength in
the second one. Use commas, spaces, or tabs as separators. The first line must
be a header and will be ignored. The wavelengths should be in the range
supported by the Parrish empirical equation (typically 400 nm to 800 nm).

Example::

    wavelength response
    350 4.16E-07
    351 3.55E-07
    352 2.93E-07
    353 2.27E-07

Salinity and temperature
^^^^^^^^^^^^^^^^^^^^^^^^

The salinity is measured in parts per thousand (ppt). Typical seawater has a
salinity of 35 ppt and freshwater has a salinity of 0 ppt. The program assumes 
a linear dependence on salinity.

The temperature is measured in degrees Celsius. The model expects values
between 0°C and 30°C.

The refraction index for each wavelength is computed with the 
`Parrish  empirical equation <https://research.engr.oregonstate.edu/parrish/index-refraction-seawater-and-freshwater-function-wavelength-and-temperature>`_.

.. _refr_options:

Command-line options
~~~~~~~~~~~~~~~~~~~~

--salinity <float>
    Salinity in parts per thousand (ppt). A good value is 35 ppt.

--temperature <float>
    Temperature in degrees Celsius. Must be between 0 and 30.

--spectral-response <string>
    CSV file containing the spectral response of the sensor band.
    See an example in :numref:`spectral_response`.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
