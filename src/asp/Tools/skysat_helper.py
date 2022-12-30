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
Python tools for working with SkySat data products.
'''
from __future__ import print_function
import sys
import os, glob, re, shutil, subprocess, string, time, errno, argparse
import simplekml, json

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

from asp_alg_utils import *

import asp_file_utils, asp_system_utils, asp_cmd_utils, asp_image_utils, asp_string_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]


#------------------------------------------------------------------------------


def addJsonFileToKml(path, kml, color = simplekml.Color.white):
    '''Add one JSON frame bounding box to the KML object'''

    with open(path, 'r') as f:
        contents = f.read()
    parts  = json.loads(contents)
    coords = parts['geometry']['coordinates'][0]
    pol = kml.newpolygon(name=os.path.basename(path),
                         outerboundaryis=coords)
    pol.style.linestyle.color = color
    pol.style.linestyle.width = 5
    pol.style.polystyle.fill  = 0


def convertToTsai(path, demPath):
    '''Given the json metadata, generate a .tsai camera file'''

    # Get the other file paths
    tifPath    = path.replace('metadata.json', 'basic_panchromatic_dn.tif')
    outputPath = tifPath.replace('.tif', '.tsai')

    # Get the image size (for the center coordinate)
    width, height = asp_image_utils.getImageSize(tifPath)

    # Set up the required coordinate string
    with open(path, 'r') as f:
        contents = f.read()
    parts  = json.loads(contents)
    coords = parts['geometry']['coordinates'][0]
    coordString = (('%f %f  %f %f  %f %f  %f %f') % (coords[0][0],  coords[0][1],
                                                     coords[3][0],  coords[3][1],
                                                     coords[2][0],  coords[2][1],
                                                     coords[1][0],  coords[1][1]))

    cmd = (('cam_gen %s  --reference-dem %s  --focal-length 553846.153846  --optical-center %f %f --pixel-pitch 1  --refine-camera  --lon-lat-values "%s"  -o %s') % (tifPath, demPath, width/2.0, height/2.0, coordString, outputPath))
    asp_system_utils.executeCommand(cmd, outputPath=outputPath)

def splitSkysatName(path):
    '''Extract parts of the skysat name'''

    # 20181215_074403_ssc10d3_0004_metadata.json
    result   = dict()
    name     = os.path.basename(path)
    parts    = name.split('_')
    result['date'    ] = parts[0]
    result['imageId' ] = parts[1]
    result['sat'], result['ccd'] = parts[2].split('d')
    result['frameNum'] = parts[3]

    return result

def main(argsIn):

    try:

        # Use parser that ignores unknown options
        usage  = "usage: skysat_helper [options]"
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--input-prefix", dest="inputPrefix", default=None,
                                              help="Input prefix of files to process")

        parser.add_argument("--output-path", dest="outputPath", default=None,
                                              help="The output kml file to write.")

        parser.add_argument("--kml-name", dest="kmlName", default='footprint',
                                              help="The name assigned to the kml tree.")


        parser.add_argument("--dem-path", dest="demPath", default=None,
                                          help="DEM file to use for tsai conversion.")

        # This call handles all the parallel_mapproject specific options.
        options = parser.parse_args(argsIn)

        # Check the required positional arguments.

    except argparse.ArgumentError as msg:
        raise Usage(msg)


    kml = simplekml.Kml()
    kml.document.name = options.kmlName

    colorList = ['ffffffff', 'ff000000', 'ffff0000', 'ff008000', 'ff00ffff', 'orange', 'ff00a5ff']

    count = 0
    imageIdList = []
    paths = glob.glob(options.inputPrefix)
    for path in paths:
        if (os.path.splitext(path)[1] != '.json') or ('metadata' not in path):
            continue
        frameInfo = splitSkysatName(path)
        fid       = frameInfo['imageId']
        if fid not in imageIdList:
            imageIdList.append(fid)
        colorIndex = imageIdList.index(fid) % len(colorList)
        color      = colorList[colorIndex]

        try:
            addJsonFileToKml(path, kml, color)
            count += 1
        except:
            print('Failed to add file: ' + path + ' to the kml footprint.')

        if options.demPath:
            try:
                convertToTsai(path, options.demPath)
            except:
                print('Failed to generate .tsai model for file: ' + path)

    print('Wrote ' + str(count) + ' polygons to file: ' + options.outputPath)
    kml.save(options.outputPath)


    print('Script is finished.')

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
