.. _historical_helper:

historical_helper.py
--------------------

The ``historical_helper.py`` program is a helper script for processing
historical imagery. See examples for how to use it in :numref:`kh4`
and :numref:`kh7`.

This tool needs the ``convert`` program from the ``ImageMagick`` package.
If ASP was installed with ``conda`` (:numref:`conda_intro`), this will
have a program named ``convert``, but that will be a utility that is part
of the ``embree`` package, rather than the tool we need.

To install the correct ``convert`` tool with conda, run::

    conda create -n imagemagick -c conda-forge imagemagick -y

Then, either prepend the path to ``convert`` to your ``PATH``,
or invoke ``historical_helper.py`` with the option::

    --convert-path $HOME/miniconda3/envs/imagemagick/bin/convert

Usage::

     historical_helper.py [options] <rotate or rotate-crop>

Command-line options:

--input-path <string (default: "")>
    Path of the input file to process.

--output-path <string (default: "")>
    The output file to write.

--interest-points <string (default: "")>
    List of column and row pairs contained in quotes.

--convert-path <string (default: "")>
    Path to the ImageMagick ``convert`` executable to use in processing. If not
    set, the directory having this tool must be prepended to the system path.

-h, --help
    Display this help message.
