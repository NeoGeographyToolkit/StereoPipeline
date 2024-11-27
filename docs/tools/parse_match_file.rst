.. _parse_match_file:

parse_match_file.py
-------------------

This tool reads an ASP match file in binary format as written by
``ipmatch`` (:numref:`ipmatch`), ``bundle_adjust``
(:numref:`bundle_adjust`), or ``stereo`` (:numref:`outputfiles`), and
writes it as a text file, with each line having an interest point and
other associated information. The first half of the file has interest
points for the left image, and the second half has corresponding
points in the right image.

The program can be invoked in reverse, to create a binary match file from a text
file. Such a match file can be viewed (:numref:`stereo_gui_view_ip`) and edited (:numref:`stereo_gui_edit_ip`) in ``stereo_gui``.

It is assumed that the version of Python in the path has the
``numpy`` and ``argparse`` packages installed, and that
``parse_match_file.py`` is in the path.

Example::

     python $(which parse_match_file.py) run/run-left__right.match \
       run/run-matches.txt

The reverse of this operation can be performed as::

     python $(which parse_match_file.py) -rev run/run-matches.txt \
       run/run-left__right.match

Note that the second invocation does not result in exactly the same
match file as the original one, as the descriptors for each interest
point are ignored on reading the text file. In practice that is not
important, as descriptors are needed only when the interest point
matches are created.

Other functionality which may be used to understand interest points is
the option ``--save-cnet-as-csv`` in ``bundle_adjust`` which saves the
interest point matches in the plain text format used by ground control
points (GCP). 
