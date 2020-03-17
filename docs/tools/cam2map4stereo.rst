.. _cam2map4stereo.py:

cam2map4stereo.py
-----------------

This program takes similar arguments as the ISIS ``cam2map`` program,
but takes two input images. With no arguments, the program determines
the minimum overlap of the two images, and the worst common resolution,
and then map-projects the two images to this identical area and
resolution.

The detailed reasons for doing this, and a manual step-by-step
walkthrough of what ``cam2map4stereo.py`` does is provided in the
discussion on aligning images on page .

The ``cam2map4stereo.py`` is also useful for selecting a subsection
and/or reduced resolution portion of the full image. You can inspect a
raw camera geometry image in qview after you have run ``spiceinit`` on
it, select the latitude and longitude ranges, and then use
``cam2map4stereo.py``\ ’s ``--lat``, ``--lon``, and optionally
``--resolution`` options to pick out just the part you want.

Use the ``--dry-run`` option the first few times to get an idea of what
``cam2map4stereo.py`` does for you.

Command-line options for cam2map4stereo.py:

-h, --help
    Display the help message.

--manual
    Read the manual.

-m, --map <file.map>
    The mapfile to use for ``cam2map``.

-p, --pixres <PIXRES>
    The pixel resolution mode to use for ``cam2map``.

-r, --resolution <RESOLUTION>
    Resolution of the final map for ``cam2map``.

-i, --interp <INTERP>
    Pixel interpolation scheme for ``cam2map``.

-a, --lat <LAT>
    Latitude range for ``cam2map``, where ``LAT`` is of the form
    *min:max*. So to specify a latitude range between -5 and 10
    degrees, it would look like ``--lat=-5:10``.

-o, --lon <LON> 
    Longitude range for ``cam2map``, where ``LON`` is of the form
    *min:max*. So to specify a longitude range between 45 and 47
    degrees, it would look like ``--lon=40:47``.

-n, --dry-run
    Make calculations, and print the ``cam2map`` command that would
    be executed, but don’t actually run it.

--prefix <path>
    Make all output files use this prefix. Default: no prefix.

-s, --suffix <suffix (default: map)>
    Suffix that gets inserted in the output file names.
