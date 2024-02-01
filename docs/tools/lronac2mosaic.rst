.. _lronac2mosaic:

lronac2mosaic.py
----------------

This tool takes in two LRONAC files (M*LE.IMG and M*RE.IMG) and produces a
single noproj mosaic composed of the two inputs. It performs the following
operations in this process: ``lronac2isis``, ``spiceinit``, ``lronaccal``,
``lronacecho``, an optional crop to the top-most lines, ``noproj``, and
``handmos``.

The offsets used in ``handmos`` are calculated using an ASP internal tool called
``lronacjitreg`` and is similar in functionality to the ISIS command
``hijitreg``. Offsets need to be calculated via feature measurements in image to
correct for imperfections in camera pointing. The angle between LE and RE optics
changes slightly with spacecraft temperature.

Optionally, ``lronac2mosiac.py`` can be given many IMG files all at
once. The tool will then look at image names to determine which should
be paired and mosaicked. The tool will also spawn multiple processes of
ISIS commands were possible to finish the task faster. The max number of
simultaneous processes is limited by the ``--threads`` option.

Usage::

    lronac2mosaic.py [options] <IMG file 1> <IMG file 2>

Command-line options for lronac2mosaic.py:

--manual
    Display the help message.

-o, --output-dir <name>
    Set the output folder (default is input folder).

--stop-at-no-proj
    Stops processing after the noproj steps are complete.

--resume-at-no-proj
    Start after ``spiceinit``, ``lronaccal``, ``lronacecho``, and the
    optional crop. This allows for custom preparation of the inputs.

--spiceinit-options <string (default='web=false spksmithed=true')>
    Options to pass to spiceinit. Specify in quotes.
    
-c, --crop <integer>
    Process only this many first lines of the image.
    
-t, --threads
    Specify the number of threads to use.

-k, --keep
    Keep all intermediate files.
