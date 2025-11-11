.. _hirise_example:

Mars Reconnaissance Orbiter HiRISE
----------------------------------

HiRISE is one of the most challenging cameras to use when making 3D
models because HiRISE exposures can be several gigabytes each. Working
with this data requires patience as it will take time.

One important fact to know about HiRISE is that it is composed of
multiple linear CCDs that are arranged side by side with some vertical
offsets. These offsets mean that the CCDs will view some of the same
terrain but at a slightly different time and a slightly different angle.
Mosaicking the CCDs together to a single image is not a simple process
and involves living with some imperfections.

One cannot simply use the HiRISE RDR products, as they do not have the
required geometric stability. Instead, the HiRISE EDR products must be
assembled using ISIS ``noproj``. The USGS distributes a script in use
by the HiRISE team that works forward from the team-produced 'balance'
cubes, which provides a de-jittered, noproj'ed mosaic of a single
observation, which is perfectly suitable for use by the Stereo
Pipeline (this script was originally engineered to provide input for
SOCET SET).  However, the 'balance' cubes are not available to the
general public, and so we include a program (``hiedr2mosaic.py``
(:numref:`hiedr2mosaic`), written in Python, that will take PDS
available HiRISE EDR products and walk through the processing steps
required to provide good input images for ``parallel_stereo``.

The program takes all the red CCDs and projects them using the ISIS
``noproj`` command into the perspective of the RED5 CCD. From there,
``hijitreg`` is performed to work out the relative offsets between CCDs.
Finally the CCDs are mosaicked together using the average offset listed
from ``hijitreg`` using the ``handmos`` command, and the mosaic is
normalized with ``cubenorm``. Below is an outline of the processing.

::

       hi2isis           # Import HiRISE IMG to Isis
       hical             # Calibrate
       histitch          # Assemble whole-CCD images from the channels
       spiceinit
       spicefit          # For good measure
       noproj            # Project all images into perspective of RED5
       hijitreg          # Work out alignment between CCDs
       handmos           # Mosaic to single file
       cubenorm          # Normalize the mosaic

To use our script, first download a set of HiRISE data. Here is an
example, using wget to fetch all RED CCDs for a dataset and process
them.

::

     wget -r -l1 -np \
       "http://hirise-pds.lpl.arizona.edu/PDS/EDR/ESP/ORB_029400_029499/ESP_029421_2300/" \
       -A "*RED*IMG"

Alternately, you can pass the ``--download-folder`` option to
``hiedr2mosaic.py`` and pass in the URL of the web page containing the
EDR files as the only positional argument. This will cause the tool to
first download all of the RED CCD images to the specified folder and
then continue with processing.

::

     hiedr2mosaic.py --download-folder hirise_example/ \
       http://hirise-pds.lpl.arizona.edu/PDS/EDR/ESP/ORB_029400_029499/ESP_029421_2300/

Assuming you downloaded the files manually, go to the directory
containing the files. You can run the ``hiedr2mosaic.py`` program
without any arguments to view a short help statement, with the ``-h``
option to view a longer help statement, or just run the program on the
EDR files like so::

       hiedr2mosaic.py *.IMG

If you have more than one observation's worth of EDRs in that directory,
then limit the program to just one observation's EDRs at a time, e.g.
``hiedr2mosaic.py PSP_001513_1655*IMG``. If you run into problems, try
using the ``-k`` option to retain all of the intermediary image files to
help track down the issue. The ``hiedr2mosaic.py`` program will create a
single mosaic file with the extension ``.mos_hijitreged.norm.cub``. Be
warned that the operations carried out by ``hiedr2mosaic.py`` can take
many hours to complete on the very large HiRISE images.

If you get any errors, make sure you have ISIS and its data installed, and the
environmental variable ``ISISDATA`` is set (:numref:`planetary_images`).

An example of using ASP with HiRISE data is included in the
``examples/HiRISE`` directory (just type 'make' there).

The dataset
~~~~~~~~~~~

HiRISE observations
`PSP_001513_1655 <http://hirise.lpl.arizona.edu/PSP_001513_1655>`_ and
`PSP_001777_1650 <http://hirise.lpl.arizona.edu/PSP_001777_1650>`_ are
on the floor of Gusev Crater and cover the area where the MER Spirit
landed and has roved, including the Columbia Hills.

.. figure:: ../images/examples/hirise/chills_hirise_combined.png
   :name: hirise_chills_example

   Example output using HiRISE images PSP_001513_1655 and
   PSP_001777_1650 of the Columbia Hills.

Commands
~~~~~~~~

Download all 20 of the RED EDR ``.IMG`` files for each observation::

    wget -r -l1 -np \
      "http://hirise-pds.lpl.arizona.edu/PDS/EDR/PSP/ORB_001500_001599/PSP_001513_1655/" \
      -A "*RED*IMG"

    wget -r -l1 -np \
      "http://hirise-pds.lpl.arizona.edu/PDS/EDR/PSP/ORB_001700_001799/PSP_001777_1650/" \
      -A "*RED*IMG"

Then process::

     ISIS> hiedr2mosaic.py PSP_001513_1655_RED*.IMG
     ISIS> hiedr2mosaic.py PSP_001777_1650_RED*.IMG
     ISIS> cam2map4stereo.py PSP_001777_1650_RED.mos_hijitreged.norm.cub \
                             PSP_001513_1655_RED.mos_hijitreged.norm.cub
     ISIS> parallel_stereo PSP_001513_1655.map.cub                       \
                    PSP_001777_1650.map.cub result/output

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices.

The ``corr-kernel`` value can usually be safely reduced to 21 pixels
to resolve finer detail and faster processing for images with good
contrast.
