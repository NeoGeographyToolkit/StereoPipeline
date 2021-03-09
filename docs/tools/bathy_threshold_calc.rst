.. _bathy_threshold_calc:

bathy_threshold_calc.py
-----------------------

The ``bathy_threshold_calc.py`` program takes as input a
single-channel image, for example, band 7 of a WorldView multispectral
image, and computes the threshold separating the water and land
pixels.

See :numref:`bathy_threshold_use` for further context, and a sample
output and graph.

This tool needs Python 3 and the numpy, scipy, matplotlib, and gdal
packages. Those can be installed in a conda environment as follows:

::

     conda create --name bathy -c conda-forge python=3.6 gdal
     conda activate bathy
     conda install -c conda-forge numpy scipy matplotlib

Example usage:

::

    ~/miniconda3/envs/bathy/bin/python bathy_threshold_calc.py \
        --image image.tif --num-samples 1000000

Command-line options for bathy_threshold_calc.py:

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
