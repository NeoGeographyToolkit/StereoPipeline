#!/usr/bin/python

# Given several pairs of one-column files with each pair corresponding
# to the x and y components of CCD artifact correction for a certain
# TDI and scan direction, concatenate the two files in each pair and
# write them as a row in the output TIF image. Use only 4 digits of
# precision.
 
import sys, os, re, argparse

import numpy as np
from osgeo import gdal, osr

usage = "python form_corrections.py <input1-dx.txt input1-dy.txt ...> --output-image image.tif"

parser = argparse.ArgumentParser(usage = usage,
                                 formatter_class = argparse.RawTextHelpFormatter)

parser.add_argument("--output-image",  dest = "output_image", default = None, 
                    help = "Save data in this .tif image.")

(options, other) = parser.parse_known_args(sys.argv)

if len(other) <= 1:
    print("No input correction files were provided.")
    sys.exit(1)

if options.output_image is None:
    print("The output image file was not specified.")
    sys.exit(1)

files = other[1:]
print("Input files: " + " ".join(files))
print("Output image: " + options.output_image)

num_files = len(files)

# Read all the files in pairs
data_rows = []
num_cols = 0
for i in range(int(num_files)):
    x_file = files[i]
    y_file = x_file.replace('x.txt', 'y.txt')
    if len(x_file) < 5 or x_file[-5:] != "x.txt" or len(y_file) < 5 or y_file[-5:] != "y.txt":
        print("Expecting the input files to end with x.txt.")

    x = np.loadtxt(x_file)
    y = np.loadtxt(y_file)
    vals = np.concatenate((x, y))
    data_rows.append(vals)
    if num_cols > 0 and num_cols != len(vals):
        print("All input files must have the same number of values.")
        sys.exit(1)
    num_cols = len(vals)
    
num_rows = len(data_rows)

# GDAL really does not like to reuse the driver.  Pasting the code
# below repeatedly in a python shell does not work well.
driver = gdal.GetDriverByName('GTiff')
dst_filename = options.output_image
dst_ds = driver.Create(dst_filename, num_cols, num_rows, 1, gdal.GDT_Float32)

raster = np.ones( (num_rows, num_cols) ) + 9
for row in range(num_rows):
    raster[row, :] = data_rows[row]

dst_ds.GetRasterBand(1).WriteArray( raster )
dst_ds.FlushCache()

