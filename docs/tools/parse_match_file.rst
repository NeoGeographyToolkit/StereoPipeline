.. _parse_match_file:

parse_match_file.py
-------------------

This tool reads an ASP match file in binary format, as written by
``ipmatch`` (:numref:`ipmatch`), ``bundle_adjust``
(:numref:`bundle_adjust`), or ``stereo`` (:numref:`outputfiles`), and
writes it as a text file, with each line having an interest point and
other associated information (the first half of the file has interest
points for the left image, and the second half has matching interest
points in the right image).

Example::

     parse_match_file.py run/run-left__right.match matches.txt

This tool can perform the reverse of this operation if called as::

     parse_match_file.py -rev matches.txt run/run-left__right.match

Note that the second invocation does not result in exactly the same
match file as the original one, as the descriptors for each interest
point are ignored on reading the text file. In practice that is not
important, as descriptors are needed only when the interest point
matches are created.

Other functionality which may be used to understand interest points is
the option ``--save-cnet-as-csv`` in ``bundle_adjust`` which saves the
interest point matches in the plain text format used by ground control
points (GCP). This tool also saves a file named
``final_residuals_no_loss_function_pointmap_point_log.csv`` in the
output directory which has for each pair of
matching interest points the triangulated world position and the error
of re-projecting such a point back in the cameras
(:numref:`bundle_adjust`).
