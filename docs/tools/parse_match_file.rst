.. _parse_match_file:

parse_match_file.py
-------------------

This tool reads an ASP match file in binary format as written by ``ipmatch``
(:numref:`ipmatch`), ``bundle_adjust`` (:numref:`bundle_adjust`), or ``stereo``
(:numref:`outputfiles`), and writes it as a text file, with each line having an
interest point and other associated information. 

The program can be invoked in reverse, to create a binary match file from a text
file. Such a match file can be viewed (:numref:`stereo_gui_view_ip`) and edited
(:numref:`stereo_gui_edit_ip`) in ``stereo_gui``.

It is assumed that the version of Python in the path has the ``numpy`` and
``argparse`` packages installed, and that ``parse_match_file.py`` is in the
path.

Examples
~~~~~~~~

::

     python $(which parse_match_file.py) run/run-left__right.match \
       run/run-matches.txt

The reverse of this operation can be performed as::

     python $(which parse_match_file.py) -rev run/run-matches.txt \
       run/run-left__right.match

Other functionality which may be used to understand interest points is
the option ``--save-cnet-as-csv`` in ``bundle_adjust`` which saves the
interest point matches in the plain text format used by ground control
points (GCP). 

.. _parse_match_format:

File format
~~~~~~~~~~~

The first line in the file has the number of matches in the left and right
images. These are always the same.

After this, the first half of the text file saved by this program has interest
points for the left image, and the second half has corresponding points in the
right image.

Each such line has the following fields, separated by spaces::

  x y ix iy orientation scale interest polarity octave scale_lv num_descr [descriptors]

Here are is an example of the first two lines of such a file::

  25 25
  2995.8699 636.7928 2996 637 -2.0879858 2.9508026 0.09294365 0 0 0 0

In this case there will be a total of 25 + 25 = 50 lines having interest points,
after the first line, with each in the format of the second line.

The only important values are the first two, which are the x and y coordinates
of each interest point pixel, and the scale, which is treated as the uncertainty
of the pixel in bundle adjustment (higher scale means less weight given during
optimization). A larger value of the ``interest`` field means it may be more
prominent (salient), though this is not employed in any way.

As of of build 2026/02 (:numref:`release`), ASP supports operating on matches in
text format. That uses however a simpler format (:numref:`txt_match`).

Descriptors
~~~~~~~~~~~

If this program is invoked with the ``--save-descriptors`` option, the interest
point descriptors are also saved in the text file. Otherwise their number is set
to zero, and no descriptors are saved.

Note that in either case the descriptors are not saved for the reverse
operation, when converting from the text file back to the binary file. In
practice that is not important, as descriptors are needed only when the interest
points are matched during creation, and not for later use.

Command-line options
~~~~~~~~~~~~~~~~~~~~

-rev, --reverse
    Convert a text file having matches into an ASP binary match file.

--save-descriptors
    When converting a binary match file to text, save the interest point
    descriptors as well.

-h, --help
    Display the help message.

