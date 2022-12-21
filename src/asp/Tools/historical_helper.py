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
Python tool for working with historical data products.
'''
from __future__ import print_function
import sys
import os, shutil, string, errno, argparse
#import simplekml, json
import numpy as np

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
libpath = os.path.abspath(basepath + '/../lib') # path to the lib directory
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

from asp_alg_utils import *

import asp_file_utils, asp_system_utils, asp_cmd_utils, asp_image_utils, asp_string_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]

# Prepend to system LD_LIBRARY_PATH
if "LD_LIBRARY_PATH" not in os.environ:
    os.environ["LD_LIBRARY_PATH"] = ""
os.environ["LD_LIBRARY_PATH"] = libpath + os.pathsep + os.environ["LD_LIBRARY_PATH"]

#------------------------------------------------------------------------------

class BBox:
    '''Simple bounding box class'''
    def __init__(self):
        self.minX = 999999999
        self.maxX = 0
        self.minY = 999999999
        self.maxY = 0

    def __str__(self):
        return ('Min: (%lf, %lf), Max: (%lf, %lf)'
                % (self.minX, self.minY, self.maxX, self.maxY))

    def expandToContain(self, pt):
        if pt[0] < self.minX: self.minX = pt[0]
        if pt[0] > self.maxX: self.maxX = pt[0]
        if pt[1] < self.minY: self.minY = pt[1]
        if pt[1] > self.maxY: self.maxY = pt[1]

    def width(self):
        return self.maxX - self.minX
    def height(self):
        return self.maxY - self.minY

def parseInterestPoints(ipString):
    '''Return the list of IP as a numpy array'''

    parts = ipString.split()
    if len(parts) % 2 == 1:
        raise Exception('The number of IP numbers must be even!')
    numIp = int(len(parts) / 2)

    ip = np.ndarray(shape=(numIp,2), dtype=float)

    for i in range(numIp):
        ip[i,0] = float(parts[2*i  ])
        ip[i,1] = float(parts[2*i+1])

    return ip


def computeAngle(topLeft, topRight):
    '''Compute the angle from the top left to the top right of an image.
       Positive rotation = clockwise.'''
    horizVec    = (1,0)
    measuredVec = topRight - topLeft

    denom   = np.linalg.norm(horizVec)*np.linalg.norm(measuredVec)
    angle   = np.arccos(np.dot(measuredVec,horizVec) / denom)
    degrees = np.rad2deg(angle)

    return degrees


def rotateAndCrop(options):
    '''Apply a rotation and crop to the image so that the output image is horizontal and cropped.'''

    ip    = parseInterestPoints(options.interestPoints)
    numIp = ip.shape[0]
    if numIp != 4:
        raise Exception('rotate-crop requires the following IP: top-left, top-right, bot-right, bot-left')

    degrees = computeAngle(ip[0,:], ip[1,:])
    radians = np.deg2rad(degrees)

    box = BBox()
    rotMatrix = np.array([[np.cos(radians), -np.sin(radians)],
                          [np.sin(radians),  np.cos(radians)]], np.float32)
    for i in range(numIp):
        rotLoc = np.matmul(rotMatrix, ip[i,:])
        box.expandToContain(rotLoc)

    offsetX = box.minX
    offsetY = box.minY
    cropX   = box.width()
    cropY   = box.height()

    # All option to rotate without cropping
    cropCmd = ''
    if options.operation == 'rotate-crop':
        cropCmd = ("-crop '%dx%d+%d+%d'" % (cropX, cropY, offsetX, offsetY))

    cmd = ("convert %s -define tiff:tile-geometry=256x256 -distort ScaleRotateTranslate '0,0 %lf' %s %s"
           % (options.inputPath, degrees, cropCmd, options.outputPath))
    print(cmd)
    os.system(cmd)


def main(argsIn):

#     try:
#         asp_system_utils.checkIfToolExists('convert')
#     except:
#         print('Cannot find the "convert" tool. Install the ImageMagick software, and then add the directory of this tool to PATH.')
#         return -1

    try:

        # Use parser that ignores unknown options
        usage  = "usage: historical_helper [options] <rotate or rotate-crop>"
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--input-path", dest="inputPath", default=None, required=True,
                                              help="Path of the input file to process")

        parser.add_argument("--output-path", dest="outputPath", default=None, required=True,
                                              help="The output file to write.")

        parser.add_argument("--interest-points", dest="interestPoints", default=None,
                                          help="List of col row pairs contained in quotes.")
        parser.add_argument('operation')

        # This call handles all the parallel_mapproject specific options.
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError as msg:
        raise Usage(msg)

    if options.operation == 'rotate-crop' or options.operation == 'rotate':
        rotateAndCrop(options)

    print('Script finished.')

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
