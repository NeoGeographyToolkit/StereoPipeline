#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __BEGIN_LICENSE__
#  Copyright (c) 2009-2013, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__

'''
Find the water-land threshold in an image (for example the band 7 of
a WorldView multispectral image by computing a kernel-density
estimate using Gaussian kernels. A good threshold is usually the
first minimum of this estimate.

This tool needs python 3, numpy, scipy, matplotlib, and osgeo.
'''

import sys, time, math, argparse
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as st
from osgeo import gdal
from scipy.signal import argrelextrema

# Try to use sklearn as well, gives very similar results in very similar time.
# Install this with:
# conda install -c conda-forge scikit-learn

use_sklearn = False # off by default
if use_sklearn:
    from sklearn.neighbors import KernelDensity

usage  = "python bathy_threshold_calc.py --image <image> --num-samples <num>."

parser = argparse.ArgumentParser(usage=usage,
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument('--image',  dest = 'image', default = "", 
                    help='The single-channel image to use to find the water-land threshold.')

parser.add_argument("--num-samples",  dest="num_samples", type=int, default = 1000000,
                    help="The number of samples to pick from the image (more samples " +
                    "will result in more accuracy but will be slower).")

parser.add_argument("--no-plot", action="store_true", default=False,
                        dest="no_plot",  help="Do not show the plot.")

(options, args) = parser.parse_known_args(sys.argv)

if options.image == "":
    parser.print_help()
    sys.exit(1)

print("Image file is " + options.image)
print("Number of samples is " + str(options.num_samples))

# Try to read the file using GDAL
try:
    ds = gdal.Open(options.image, gdal.GA_ReadOnly)

    if ds is None:
        print("Could not read the file: " + options.image)
        sys.exit(1)

    if ds.RasterCount != 1:
        print("Expecting one band in " + options.image + ", but got instead: " +
               str(ds.RasterCount) + ".")
        sys.exit(1)
        
    rb = ds.GetRasterBand(1)
    image = rb.ReadAsArray()
    
except Exception as err:
    print("Could not read the file: " + options.image)
    print("It must exist and be a single-band TIF file.")
    sys.exit(1)
    
num_rows = image.shape[0]
num_cols = image.shape[1]

if num_rows <= 0 or num_cols <= 0:
    print("Expecting an image with positive dimensions")
    sys.exit(1)
    
num_vals = num_rows * num_cols
samp_ratio = math.sqrt( float(num_vals) / float(options.num_samples) ) 
num_sub_rows = round(num_rows / samp_ratio)

if num_sub_rows < 1:
    num_sub_rows = 1
if num_sub_rows > num_rows:
    num_sub_rows = num_rows
    
num_sub_cols = round(num_cols / samp_ratio)
if num_sub_cols < 1:
    num_sub_cols = 1
if num_sub_cols > num_cols:
    num_sub_cols = num_cols

print("Number of image rows and columns: " + str(num_rows) + ", " + str(num_cols))
print("Picking a uniform sample of dimensions " + str(num_sub_rows) + ", " + str(num_sub_cols))
print("Please be patient. It make take several minutes to find the answer.")

# Subsample uniformly the image
sub_rows = np.round(np.array(range(num_sub_rows)) * float(num_rows - 1)/float(num_sub_rows - 1))
sub_cols = np.round(np.array(range(num_sub_cols)) * float(num_cols - 1)/float(num_sub_cols - 1))
sub_rows = sub_rows.astype(int)
sub_cols = sub_cols.astype(int)
sub_image = image[sub_rows, :][:, sub_cols]

# Make it into an array
data = sub_image.reshape(-1)
xvals = np.linspace(data.min(), data.max(), 1000)

beg = time.time()
kde = st.gaussian_kde(data)
yvals = kde(xvals)
min_pos  = argrelextrema(yvals, np.less);  min_vals = xvals[min_pos]
end = time.time()
# Note that it is not universal for it to be first minimum. Sometimes
# the second minimum is better!
print("Positions of the minima: ", min_vals)
print("Suggested threshold is the position of the first minimum: ", min_vals[0])
print("Please verify with the graph. There is a chance subsequent minima may work better.")
print("Elapsed time in seconds:", round(10.0*(end - beg))/10.0)

# sklearn, with similar results
if use_sklearn:
    beg2 = time.time()
    kernel = 'gaussian'
    kde2 = KernelDensity(kernel = kernel, bandwidth = 10).fit(data[:, np.newaxis])
    log_dens = kde2.score_samples(xvals[:, np.newaxis])
    yvals2 = np.exp(log_dens).reshape(-1)
    min_pos2 = argrelextrema(yvals2, np.less); min_vals2 = xvals[min_pos2]
    end2 = time.time()
    print("Elapsed time for sklearn kernel estimation in seconds:", round(10.0*(end2 - beg2))/10.0)
    print("Suggested threshold is the position of the first minimum2: ", min_vals2[0])
    print("Positions of the minima2: ", min_vals2)
    
# Plot the kernel-density estimate and highlight the minima
if not options.no_plot:
    plt.figure(1)
    plt.hist(data, bins=100, density=True, label="Data histogram")
    plt.plot(xvals, yvals, label="KDE", c="red")
    plt.vlines(min_vals, ymin=0, ymax=yvals.max(),colors='g', ls="--", label="Minima", alpha=0.7)
    
    if use_sklearn:
        plt.plot(xvals, yvals2, color = 'green', lw = 2,
                 linestyle='-', label="kernel = '{0}'".format(kernel))
    
    plt.legend()
    plt.show()
