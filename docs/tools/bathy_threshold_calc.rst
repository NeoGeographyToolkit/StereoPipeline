.. _bathy_threshold_calc:

bathy_threshold_calc.py
-----------------------

The ``bathy_threshold_calc.py`` program takes as input a
single-channel image, for example, band 7 of a WorldView multispectral
image, and computes the threshold separating the water and land
pixels.

See :numref:`bathy_threshold_use` for further context, and a sample
output and graph.

Example usage
~~~~~~~~~~~~~

Install the ``bathy`` conda environment as described in
:numref:`bathy_threshold_dependencies` below.

This program can be invoked as:

::

    ~/miniconda3/envs/bathy/bin/python $(which bathy_threshold_calc.py) \
        --image image.tif --num-samples 1000000

Here it is assumed that ASP's ``bin`` directory is in the path, otherwise
the full path to this Python script must be specified above.

.. _bathy_threshold_dependencies:

Dependencies
~~~~~~~~~~~~

This tool needs Python 3 and some additional Python packages to be installed with 
``conda``. 

Conda can be obtained from 

    https://docs.conda.io/en/latest/miniconda.html

Run::

    ./Miniconda3-latest-Linux-x86_64.sh

on Linux, and the appropriate version on OSX (this script needs to be
made executable first). Use the suggested::

    $HOME/miniconda3

directory for installation. 

Activate conda. The needed packages can be installed
as follows:

::

     conda create --name bathy -c conda-forge python=3.6 gdal
     conda activate bathy
     conda install -c conda-forge numpy scipy matplotlib

Command-line options for bathy_threshold_calc.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-h, --help
    Display the help message.

--image <filename>
    The single-channel image to use to find the water-land threshold.

--num-samples <integer (default: 1000000)>
    The number of samples to pick from the image (more samples will
    result in more accuracy but will be slower).

--no-plot
    Do not show the plot.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
