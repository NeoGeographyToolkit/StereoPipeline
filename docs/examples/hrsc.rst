.. _hrsc_example:

Mars Express High Resolution Stereo Camera (HRSC)
-------------------------------------------------

The HRSC camera on the Mars Express satellite is a complicated system,
consisting of multiple channels pointed in different directions plus
another super resolution channel. The best option to create DEMs is to
use the two dedicated stereo channels. These are pointed ahead of and
behind the nadir channel and collect a stereo observation in a single
pass of the satellite. Data can be downloaded from the Planetary Data
System (PDS)
http://pds-geosciences.wustl.edu/missions/mars_express/hrsc.htm or you
can use the online graphical tool located at
http://hrscview.fu-berlin.de/cgi-bin/ion-p?page=entry2.ion. Since each
observation contains both stereo channels, one observation is sufficient
to create a DEM.

HRSC data is organized into categories. Level 2 is radiometrically
corrected, level 3 is corrected and mapprojected onto MOLA, and level 4
is corrected and mapprojected on to a DEM created from the HRSC data.
You should use the level 2 data for creating DEMs with ASP. If you would
like to download one of the already created DEMs, it may be easiest to
use the areoid referenced version (.da4 extension) since that is
consistent with MOLA.

What follows is an example for how to process HRSC data. One starts by
fetching the two stereo channels from::

   https://pds-geosciences.wustl.edu/mex/mex-m-hrsc-3-rdr-v4/mexhrs_4000/data/1995/h1995_0000_s13.img
   https://pds-geosciences.wustl.edu/mex/mex-m-hrsc-3-rdr-v4/mexhrs_4000/data/1995/h1995_0000_s23.img

.. figure:: ../images/examples/hrsc/hrsc_example.png
   :name: hrsc_figure

   Sample outputs from a cropped region of HRSC frame 1995.  Left: Cropped input.
   Center: Block matching with subpixel mode 3.  Right: MGM algorithm with cost
   mode 3.

Commands
~~~~~~~~

You may need to download the HRSC kernel files in case using
``web=true`` with ``spiceinit`` does not work. You will also probably
need to include the ``ckpredicted=true`` flag with ``spiceinit``. HRSC
images are large and may have compression artifacts so you should
experiment on a small region to make sure your stereo parameters are
working well. For this frame, the MGM stereo algorithm performed better
than block matching with subpixel mode 3.

It appears that ``hrsc2isis`` is not able to read the level 3 images that were
downloaded above, and PDS no longer offers level 2 images. What seems to work is
edit the .img files and change level 3 to level 2. That can be done with::

    perl -pi -e 's#(PROCESSING_LEVEL_ID\s+=) 3#$1 2#g' *.img

Then run::

     ISIS> hrsc2isis from=h1995_0000_s13.img to=h1995_0000_s13.cub
     ISIS> hrsc2isis from=h1995_0000_s23.img to=h1995_0000_s23.cub
     ISIS> spiceinit from=h1995_0000_s13.cub ckpredicted=true
     ISIS> spiceinit from=h1995_0000_s23.cub ckpredicted=true
     ISIS> parallel_stereo h1995_0000_s13.cub  h1995_0000_s23.cub \
             --stereo-algorithm 2 --cost-mode 3 mgm/out
     ISIS> point2dem --stereographic --auto-proj-center \
             mgm/out-PC.tif         

These are big images, so consider using ``stereo_gui`` to select clips
to work on (:numref:`stereo_gui`).

See :numref:`nextsteps` for other stereo algorithms, and information on
tradeoffs between them.
