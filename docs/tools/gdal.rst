.. _gdal_tools:

GDAL tools
----------

ASP distributes in the ``bin`` directory the following `GDAL
<https://gdal.org/>`_ programs:

- gdal_rasterize (modify images based on shape files)
- gdal_translate (crop, resample)
- gdalbuildvrt (mosaic)
- gdaldem (hillshade)
- gdalinfo (show stats)
- gdaltransform (transforms between coordinate systems)
- gdalwarp (convert to another projection and interpolate)
- gdaladdo (create multi-resolution pyramids)

and a handful of other ones.

These executables are compiled with JPEG2000 and BigTiff support, and
can handle NTF images in addition to most image formats. They can be
used to see image statistics, crop and scale images, build virtual
mosaics, reproject DEMs, etc.

If ASP was installed via conda, rather than using the release tarball
(:numref:`installation`), make sure to activate that conda environment
before using these tools or set the ``PROJ_LIB`` and/or ``PROJ_DATA``
environmental variable to points to the ``share/proj`` subdirectory of
the ASP conda environment. Otherwise there will be warnings about
failing to find such a directory. This is a known PROJ issue.

Certain conversion between datums using the GDAL/PROJ tools may need
additional data, which can be stored either locally or fetched via a
network (see the `PROJ documentation
<https://proj.org/usage/network.html>`_).
