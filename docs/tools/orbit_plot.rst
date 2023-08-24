.. _orbit_plot:

orbit_plot
----------

The ``orbit_plot.py`` program is a Python script that takes an input one or more
orbital sequences of cameras, and plots the camera orientation as it changes
along the orbit. Each orientation is decomposed into roll, pitch, and yaw
components, that are plotted separately.

If a second set of orbital sequences exists, for example, if the camera
orientations are later optimized, with ``bundle_adjust``
(:numref:`bundle_adjust`) or ``jitter_solve`` (:numref:`jitter_solve`), this
tool can overlay the two sets.

Each orbital sequence consists of several frame (pinhole) cameras, in .tsai
(:numref:`pinholemodels`) or CSM (:numref:`csm_frame`) format, or it can be a
single linescan camera in the CSM model state format (:numref:`csm_state`).  

At some point this tool will also plot the camera positions.
 
Example
~~~~~~~

Here we will consider synthetic cameras, created with ``sat_sim`` (:numref:`sat_sim`).


Command-line options for bathy_plane_calc
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

--shapefile <filename>
    The shapefile with vertices whose coordinates will be looked up in
    the DEM.

--dem <filename>
    The DEM to use.

--mask <string (default: "")>
    A input mask, created from a raw camera image and hence having the
    same dimensions, with values of 1 on land and 0 on water, or
    positive values on land and no-data values on water.

--camera <string (default: "")>
    The camera file to use with the mask.

--bundle-adjust-prefix <string (default: "")>
    Use the camera adjustment at this output prefix, if the cameras
    changed based on bundle adjustment or alignment.

-t, --session-type <string (default: "")>
    Select the stereo session type to use for processing. Usually
    the program can select this automatically by the file extension, 
    except for xml cameras. See :numref:`parallel_stereo_options` for
    options.

--outlier-threshold <double>
    A value, in meters, to determine the distance from a sampled point
    on the DEM to the best-fit plane to determine if it will be marked as 
    outlier and not included in the calculation of that plane. The default
    is 0.5. Its value should be roughly the expected vertical uncertainty
    of the DEM.


--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN

