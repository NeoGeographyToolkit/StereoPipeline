.. _datum_convert:

datum_convert
-------------

This tool is used to convert a DEM from one datum to another. For
example, a UTM zone 10 DEM with an NAD27 datum can be converted to a UTM
zone 10 DEM with a WGS84 datum. This tool does not convert between
projections, another program such as ``gdalwarp`` (included with ASP) or
ASP’s ``dem_mosaic`` should be used for that. ``datum_convert`` performs
horizontal conversion; vertical conversion is only provided for the
limited case of conversions between datums defined only by the +a and +b
terms (such as our D_MARS and MOLA datums). The underlying Proj.4
library does have some support for vertical datums (see
https://github.com/OSGeo/proj.4/wiki/VerticalDatums) so a motivated user
may be able to apply them successfully. If you do, let us know what
steps you took so we can add them to the manual! ASP ships with some
vertical datum grid files in the ASP/share/proj folder but more can be
found on the Internet. Whenever you perform datum conversions be
careful; the Proj.4 library tends to fail silently by performing an
identity transform on the input data. If your output data exactly
matches your input data this means that something has probably gone
wrong.

The tool will try to automatically find datum information from the input
file but the input datum can be manually specified if the information in
the file is missing or incorrect. Be aware that if the ``--keep-bounds``
option is not set there may be noticeable changes in the image data just
from re-interpolating to the new projected space grid. In the case of
sparsely sampled input images this effect can be much larger than the
changes resulting from the actual datum transformation.

Intuitively, the input and output DEMs should correspond to the same
point cloud in 3D space up to the interpolation errors required to
perform the conversion. In practice datum conversion is a complex task
which may need to account for things like shifting tectonic plates over
time. ASP’s implementation is based on Proj.4 and the HTDPGrids
extension (https://github.com/OSGeo/proj.4/wiki/HTDPGrids). Datum
support in Proj.4 is not robust even with the extension so if it is
critical that you have a very accurate conversion we recommend that you
attempt to verify results obtained using ``datum_convert`` with another
conversion method.

This tool requires the GDAL and NumPy Python packages to run. One way
to get these is to install the ASP Python tools, described at the end
of :numref:`sparse-disp`, and by setting ASP_PYTHON_MODULES_PATH as
mentioned in that section.

Usage::

    datum_convert [options] <input dem> <output dem>

Command-line options for datum_convert:

--help
    Display the help message.

--show-all-datums
    Print out all the datum names which are recognized.

--output-datum <string>
    The datum to convert to. Supported options include: WGS_1984,
    NAD83, WGS72, and NAD27.

--input-datum <string>
    Override the datum of the input file. Supports the same options
    as ``--output-datum``.

--output-datum-year <float (default: 2000.0)>
    Specify the exact date of the output datum in floating point
    format ex: 2003.4.

--input-datum-year <float (default: 2000.0)>
    As ``--output-datum-year``, but for the input file.

--t_srs <proj string>
    Specify the output datum via the PROJ.4 string.

--keep-bounds
    Don’t recompute the projected space boundary. This can help
    reduce changes caused by interpolation.

--nodata-value
    The value of no-data pixels, unless specified in the DEM.

--double
    Create float64 instead of float32 output files.

--show-grid-calc
    Don’t hide the shift grid creation output.

--debug-mode
    Print the converted lon/lat/alt coordinates for each pixel.
    Only useful for investigating exact change that is happening.

--grid-size-lon
    Specify the number of columns in the grid shift file.

--grid-size-lat
    Specify the number of rows in the grid shift file.

--keep-working-files
    Don’t delete intermediate files.
