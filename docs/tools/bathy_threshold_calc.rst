.. _bathy_threshold_calc:

bathy_threshold_calc.py
-----------------------

The ``bathy_threshold_calc.py`` program takes as input a
single-channel image, for example, band 7 of a WorldView multispectral
image, and computes the threshold separating the water and land
pixels. 

This program works based on the observation that in such an image the
water appears darker than the land, hence, in a histogram of the
pixels in the image, the water and land appear as two noticeable
peaks, with a good value for the threshold then being the image value
in the valley between those peaks.

For robustness to noise, this histogram is approximated by a
kernel-density estimate (``KDE``) using Gaussian kernels. It is very
important to note that even then this tool may return the wrong
minimum, which it assumes to be the first one.

Therefore, this tool plots the histogram, its kernel density estimate,
and the positions of the minima. The user is responsible to validate
visually where the most appropriate position of the minimum is (along
the horizontal axis).
 
The kernel-density estimate calculation is very time-consuming for
large images, hence it is suggested to sample the image, by using, for
example, a million uniformly sampled pixels, when the calculation
should take a few minutes to complete.

This tool needs Python 3 and the numpy, scipy, matplotlib and osgeo packages.
Those can be installed in a conda environment as follows:

::

     conda create --name bathy -c conda-forge python=3.6 gdal
     conda activate bathy
     conda install -c conda-forge numpy scipy matplotlib

Example:

::

    ~/miniconda3/envs/bathy/bin/python bathy_threshold_calc.py \
        --image image.tif --num-samples 1000000

See :numref:`shallow_water_bathy` for further context, how it is used,
and a sample output and graph.

Command-line options for bathy_threshold_calc.py:

-h, --help
    Display the help message.

--image <filename>
    The single-channel image to use to find the water-land threshold.

--num-samples <integer (default: 1000000)>
    The number of samples to pick from the image (more samples will
    result in more accuracy but will be slower.

.. |times| unicode:: U+00D7 .. MULTIPLICATION SIGN
