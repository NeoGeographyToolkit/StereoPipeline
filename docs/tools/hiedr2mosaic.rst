.. _hiedr2mosaic:

hiedr2mosaic.py
---------------

Assemble a collection of HiRISE EDR files into a single image. This runs
the sequence of ISIS preprocessing commands, followed by hijitreg, to
assemble the input images into a single output image. You can either
download the input files yourself and pass them all in or specify a
download folder and pass in only a URL such as
http://hirise-pds.lpl.arizona.edu/PDS/EDR/ESP/ORB_029400_029499/ESP_029421_2300/.
If you use a URL, the program will attempt to download all of the HiRISE
images found at that location and then run the processing script. See
the "Mars Reconnaissance Orbiter HiRISE" section in the examples chapter
for a more detailed explanation.

Usage::

    hiedr2mosaic.py [options] <input files OR a URL>

Command-line options for hiedr2mosaic.py:

--manual
    Display the help message.

-w, --web
    Invokes spiceinit with web=true, to fetch the kernels from the web.

-m, --match
    The CCD number passed as the match argument to noproj (default 5).

--stop-at-no-proj
    Stops processing after the noproj steps are complete.

--resume-at-no-proj
    Restarts processing using the results from ``--stop-at-no-proj``.

--download-folder
    Download input files to this folder. Must pass in a URL instead
    of files.

-t, --threads
    Specify the number of threads to use.

-k, --keep
    Keep all intermediate files.
