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

"""
Basic functions for working with images on disk.
"""

import sys, os, re, subprocess, string, time, errno

import asp_string_utils


def stripRgbImageAlphaChannel(inputPath, outputPath):
    """Makes an RGB copy of an RBGA image"""
    cmd = 'gdal_translate ' + inputPath + ' ' + outputPath + ' -b 1 -b 2 -b 3 -co "COMPRESS=LZW" -co "TILED=YES" -co "BLOCKXSIZE=256" -co "BLOCKYSIZE=256"'
    print cmd
    os.system(cmd)



def getImageSize(imagePath):
    """Returns the size [samples, lines] in an image"""

    # Make sure the input file exists
    if not os.path.exists(imagePath):
        raise Exception('Image file ' + imagePath + ' not found!')
       
    # Use subprocess to suppress the command output
    cmd = ['gdalinfo', imagePath]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    textOutput, err = p.communicate()

    # Extract the size from the text
    sizePos    = textOutput.find('Size is')
    endPos     = textOutput.find('\n', sizePos+7)
    sizeStr    = textOutput[sizePos+7:endPos]
    sizeStrs   = sizeStr.strip().split(',')
    numSamples = int(sizeStrs[0])
    numLines   = int(sizeStrs[1])
    
    size = [numSamples, numLines]
    return size

def isIsisFile(filePath):
    """Returns True if the file is an ISIS file, False otherwise."""

    # Currently we treat all files with .cub extension as ISIS files
    extension = os.path.splitext(filePath)[1]
    return (extension == '.cub')




def getImageStats(imagePath):
    """Obtains some image statistics from gdalinfo"""
    
    if not os.path.exists(imagePath):
        raise Exception('Image file ' + imagePath + ' not found!')
    
    # Call command line tool silently
    cmd = ['gdalinfo', imagePath, '-stats']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    textOutput, err = p.communicate()
    
    # Statistics are computed seperately for each band
    bandStats = []
    band = 0
    while (True): # Loop until we run out of bands
        # Look for the stats line for this band
        bandString = 'Band ' + str(band+1) + ' Block='
        bandLoc = textOutput.find(bandString)
        if bandLoc < 0:
            return bandStats # Quit if we did not find it
            
        # Now parse out the statistics for this band
        bandMaxStart  = textOutput.find('STATISTICS_MAXIMUM=', bandLoc)
        bandMeanStart = textOutput.find('STATISTICS_MEAN=',    bandLoc)
        bandMinStart  = textOutput.find('STATISTICS_MINIMUM=', bandLoc)
        bandStdStart  = textOutput.find('STATISTICS_STDDEV=',  bandLoc)
               
        bandMax  = asp_string_utils.getNumberAfterEqualSign(textOutput, bandMaxStart)
        bandMean = asp_string_utils.getNumberAfterEqualSign(textOutput, bandMeanStart)
        bandMin  = asp_string_utils.getNumberAfterEqualSign(textOutput, bandMinStart)
        bandStd  = asp_string_utils.getNumberAfterEqualSign(textOutput, bandStdStart)
            
        # Add results to the output list
        bandStats.append( (bandMin, bandMax, bandMean, bandStd) )
            
        band = band + 1 # Move to the next band
    





