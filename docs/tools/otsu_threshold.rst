.. _otsu_threshold:

otsu_threshold
-------------------

The ``otsu_threshold`` program takes as input a single-channel image
and computes the optimal `Otsu threshold
<https://en.wikipedia.org/wiki/Otsu%27s_method>`_. See also 
`this overview <http://www.labbookpages.co.uk/software/imgProc/otsuThreshold.html>`_ 
with some example images and thresholds plotted vs histograms at the very bottom of that page.

If the input image is, for example, band 7 of a WorldView multispectral
image, this will return a value for the threshold separating land from water
(since water shows up rather dark in the infrared, being cooler).
If the input is an image of the Moon, with shadows present, the threshold
will separate shadow pixels from lit pixels.

Note that the value returned by this tool is not necessarily the
position corresponding to the bottom of the "saddle" of the histogram
of this image, if it is bi-modal (see the figure further down). To get
that behavior, use instead ``bathy_threshold_calc.py``
(:numref:`bathy_threshold_calc`).

.. figure:: ../images/otsu_threshold.png
   :name: otsu_threshold_example

   Illustration of the Otsu threshold.

Command-line options for otsu_threshold:

-h, --help
    Display the help message.

--image <filename>
    The single-channel image to use to find the water-land threshold.

--num-samples <integer (default: -1)>
    The number of samples to pick from the image (more samples will
    result in more accuracy but will be slower). If not specified,
    hence set to -1, the full image will be used.

--num-bins <integer (default: 256)>
    Number of bins to use for the histogram. A larger value is
    suggested if the image has some pixels of unexpectedly high
    values.

