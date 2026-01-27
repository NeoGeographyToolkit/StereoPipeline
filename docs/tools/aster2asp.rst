.. _aster2asp:

aster2asp
---------

The ``aster2asp`` tool takes as input an HDF file or a directory containing
ASTER Level 1A data and creates TIF and XML files that can then be passed to
:ref:`parallel_stereo` to create a point cloud.

An example for how to fetch the data and use this tool is given in
:numref:`aster`.

ASTER acquires stereo imagery using two VNIR (visible and near-infrared)
telescopes that view the same ground area from different angles:

- **Band3N**: Nadir-looking telescope (views ground from directly above)
- **Band3B**: Backward-looking telescope (views ground from behind at 27.6 degrees)

Both bands have 15-meter resolution, enabling stereo reconstruction of terrain
elevation.

Input formats
~~~~~~~~~~~~~

The tool supports two ASTER Level 1A data formats:

- **V003 directory format**: Pre-extracted text files with all metadata,
  including latitude and longitude grids. This was the original format
  distributed by NASA.

- **V004 HDF format**: All data stored in HDF4_EOS datasets. The program
  automatically extracts the required datasets and computes latitude and
  longitude grids from the orbital geometry. This functionality is available in
  build 2026-01 or later (:numref:`release`).

The various plain text files containing satellite positions, sight vectors,
etc., are described in :cite:`abrams2002aster`.

Usage
~~~~~

::

     aster2asp <input directory> -o <output prefix>

For V004 HDF format::

     aster2asp <input.hdf> -o <output prefix>

Output
~~~~~~

The tool will apply the existing radiometric corrections to the images, and save
two images with Float32 pixels with names like ``out-Band3N.tif`` and
``out-Band3B.tif``. It will create approximate RPC camera models in XML format
(:numref:`rpc`) for the left and right cameras, following
:cite:`girod2015improvement`, with names of the form ``out-Band3N.xml`` and
``out-Band3B.xml``.

See :numref:`aster` for how to run :ref:`parallel_stereo` with these files.

RPC height range
~~~~~~~~~~~~~~~~

It is important to note that the tool expects the minimum and maximum simulation
box heights (in meters, above the datum) in which to compute the RPC
approximation. The defaults are 0 and 8000, corresponding to sea level and the
highest location on Earth. Narrowing down these numbers (if it is known what
range of terrain heights is expected) may result in slightly more accurate
models.

References
~~~~~~~~~~

For more information about ASTER data formats, see:

- `ASTER User Guide Version 4 <https://lpdaac.usgs.gov/documents/2265/ASTER_User_Guide_V4_pcP80n5.pdf>`_
- `ASTER L1 Product Specifications <https://lpdaac.usgs.gov/documents/175/ASTER_L1_Product_Specifications.pdf>`_

Command-line options
~~~~~~~~~~~~~~~~~~~~

-o, --output-prefix <arg>
    Specify the output prefix.

--min-height <arg (default: 0)>
    The minimum height (in meters) above the WGS84 datum of the
    simulation box in which to compute the RPC approximation.

--max-height <arg (default: 8000)>
    The maximum height (in meters) above the WGS84 datum of the
    simulation box in which to compute the RPC approximation.

--num-samples <arg (default: 100)>
    How many samples to use between the minimum and maximum heights.

--penalty-weight <arg (default: 0.1)>
    Penalty weight to use to keep the higher-order RPC coefficients
    small. Higher penalty weight results in smaller such coefficients.

--keep-tmp-dir
    Keep the temporary directory where HDF data is extracted (for debugging).

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
