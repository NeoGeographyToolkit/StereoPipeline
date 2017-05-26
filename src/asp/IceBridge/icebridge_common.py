#!/usr/bin/env python
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

# Icebridge utility functions

import os, sys, datetime, time, subprocess, logging

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../IceBridge')  # for dev ASP
pythonpath  = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, asp_alg_utils, asp_geo_utils
asp_system_utils.verify_python_version_is_supported()

#def getImageFrameNumber(filename):
#    '''Return the frame number from an input image file'''
#    return int(filename[11:16])

#def getOrthoFrameNumber(filename):
#    '''Return the frame number from an input ortho file'''
#    return int(filename[12:17])

def getCameraFileName(imageFileName):
    '''Get the camera file name we associate with an input image file'''
    return imageFileName.replace('.tif', '.tsai')

def getFrameNumberFromFilename(f):
    '''Return the frame number of an image or camera file'''
    # Look for a 5 digit number, that is usually the frame name.
    # Other parts of the file, like the date and time stamp
    # have more digits.
    base  = os.path.basename(f)
    base  = base.replace('.', '_') # To deal with the extension
    parts = base.split('_')
    for part in parts:
        if len(part) != 5:
            continue
        if part < '00000' or part > '99999':
            continue
        return int(part)

    raise Exception('Cannot parse the frame number from ' + f)
    return 0

def parseDateTimeStrings(dateString, timeString):
    '''Parse strings in the format 20110323_17433900'''
    
    MILLISECOND_TO_MICROSECOND = 10000
    
    year    = int(dateString[0:4])
    month   = int(dateString[4:6])
    day     = int(dateString[6:8])
    hour    = int(timeString[0:2])
    minute  = int(timeString[2:4])
    second  = int(timeString[4:6])
    usecond = 0
    if len(timeString) > 6:
        usecond = int(timeString[6:8]) * MILLISECOND_TO_MICROSECOND
    
    return datetime.datetime(year, month, day, hour, minute, second, usecond)

# Pull two six or eight digit values from the given file name
# as the time and date stamps.
def parseTimeStamps(fileName):

    fileName = os.path.basename(fileName)
    fileName = fileName.replace('.', '_')
    fileName = fileName.replace('-', '_')
    parts    = fileName.split('_')

    imageDateString = ""
    imageTimeString = ""

    for part in parts:

        if len(part) != 6 and len(part) != 8:
            continue
        
        if len(part) == 6:
            if part < '000000' or part > '999999':
                continue

        if len(part) == 8:
            if part < '00000000' or part > '99999999':
                continue

        if imageDateString == "" and len(part) == 8:
            # The date must always be 8 digits (YYYYMMDD)
            imageDateString = part
            continue

        if imageTimeString == "":
            # The time can be hhmmss or hhmmssff (ff = hundreds of seconds)
            imageTimeString = part
            continue
            
    if imageDateString == "":
        return []

    if imageTimeString == "":
        return []

    return [imageDateString, imageTimeString]


def findMatchingLidarFile(imageFile, lidarFolder):
    '''Given an image file, find the best lidar file to use for alignment.'''
    
    # Look in the paired lidar folder, not the original lidar folder.
    pairedFolder = os.path.join(lidarFolder, 'paired')
    
    vals = parseTimeStamps(imageFile)
    if len(vals) < 2:
        raise Exception('Failed to parse the date and time from: ' + imageFile)
    imageDateTime = parseDateTimeStrings(vals[0], vals[1])
    
    #print 'INPUT = ' + str(imageDateTime)
    
    # Search for the matching file in the lidar folder.
    # - We are looking for the closest lidar time that starts BEFORE the image time.
    # - It is possible for an image to span lidar files, we will address that if we need to!
    bestTimeDelta = datetime.timedelta.max
    bestLidarFile = 'NA'
    lidarFiles    = os.listdir(pairedFolder)
    zeroDelta     = datetime.timedelta()

    for f in lidarFiles:

        if '.csv' not in f: # Skip other files
            continue

        # Extract time for this file
        lidarPath = os.path.join(pairedFolder, f)

        vals = parseTimeStamps(lidarPath)
        if len(vals) < 2: continue # ignore bad files

        lidarDateTime = parseDateTimeStrings(vals[0], vals[1])

        #print 'THIS = ' + str(lidarDateTime)

        # Compare time to the image time
        timeDelta       = abs(imageDateTime - lidarDateTime)
        #print 'DELTA = ' + str(timeDelta)
        # Select the closest lidar time
        # - Since we are using the paired files, the file time is in the middle 
        #   of the (large) file so being close to the middle should make sure the DEM
        #   is fully covered by LIDAR data.
        if timeDelta < bestTimeDelta:
            bestLidarFile = lidarPath
            bestTimeDelta = timeDelta

    if bestLidarFile == 'NA':
        raise Exception('Failed to find matching lidar file for image ' + imageFile)

    return bestLidarFile


