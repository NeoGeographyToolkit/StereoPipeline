.. _gdal_tools:

GDAL tools
----------

ASP distributes in the ``bin`` directory the following `GDAL
<https://gdal.org/>`_ tools:

- gdal_rasterize
- gdal_translate
- gdalbuildvrt
- gdaldem
- gdalinfo
- gdaltransform
- gdalwarp
- gdaladdo

and a handful of other ones.

These executables are compiled with JPEG2000 and BigTIFF support, and
can handle NTF images in addition to most image formats. They can be
used to see image statistics, crop and scale images, build virtual
mosaics, reproject DEMs, etc.

If ASP was installed via conda, rather than using the release tarball
(:numref:`installation`), make sure to activate that conda environment
before using these tools or set the ``PROJ_LIB`` environmental
variable to point to the ``share/proj`` subdirectory of the ASP conda
environment. Otherwise there will be warnings about failing to find
such a directory. This is a known PROJ issue.

Certain conversion between datums using the GDAL/PROJ tools may need
additional data, which can be stored either locally or fetched via a
network (see the `PROJ documentation
<https://proj.org/usage/network.html>`_).

