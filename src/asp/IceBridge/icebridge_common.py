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

import os, sys, datetime, time, subprocess, logging, re, hashlib, string
import psutil

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../IceBridge')  # for dev ASP
pythonpath  = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, asp_alg_utils, asp_geo_utils, asp_image_utils
asp_system_utils.verify_python_version_is_supported()

def outputFolder(site, yyyymmdd):
    '''The output folder name.'''
    return site + '_' + yyyymmdd

def makeSymLink(oldFile, newFile, verbose=True):
    '''Safely create a symlink'''
    try:    os.remove(newFile)
    except: pass  
    if verbose:
        print("ln -s " + os.path.abspath(oldFile) + " " + newFile)
    os.symlink(os.path.abspath(oldFile), newFile)

def getSmallestFrame():
    '''Return the smallest possible frame number.'''
    return 0

def getLargestFrame():
    '''Return the largest possible frame number.'''
    return 99999999 # 100 million should be enough

def fileExtension(filename):
    '''Convenience function to get the file extension.'''
    return os.path.splitext(filename)[1]

def hasImageExtension(filename):
    '''Return true if the file is a recognized image extension.'''
    extension = fileExtension(filename).lower()
    validExtensions = ['.tif', '.jpg', '.jpeg', '.ntf']
    if extension in validExtensions:
        return True
    return False

def folderToType(folder):
    '''If input is myRun/ortho, return 'ortho'. Same for 'fireball', 'lidar', etc.'''
    return os.path.basename(folder)

def htmlIndexFile(folder):
    '''Return the html index file for this folder (if appropriate)'''
    return os.path.join(folder, folderToType(folder) + "_index.html")
    
def csvIndexFile(folder):
    '''Return the clean csv version of the html index file for this folder (if appropriate) '''
    return htmlIndexFile(folder) + ".csv"
    
def readIndexFile(parsedIndexPath):
    '''Read an index file having frame number, filename, and url it came from.'''
    frameDict  = {}
    urlDict    = {}
    with open(parsedIndexPath, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 3:
                # Odd index file
                raise Exception("Invalid index file: " + parsedIndexPath)
            
            frameNumber = int(parts[0])
            frameDict[frameNumber] = parts[1].strip()
            urlDict[frameNumber] = parts[2].strip()

    return (frameDict, urlDict)

def isValidImage(filename):
    '''Check that an image file is not corrupted in some way. This check is not enough.'''
    
    if not os.path.exists(filename):
        return False
    
    # Must always wipe .aux.xml. Always. Otherwise, if this function is called first time
    # it may return False, but if called second time it may return True.
    auxFile = filename + '.aux.xml'
    if os.path.exists(auxFile):
        os.remove(auxFile)
        
    gdalinfoPath = asp_system_utils.which("gdalinfo")
    cmd = gdalinfoPath + ' -stats ' + filename
    
    if os.path.exists(auxFile):
        os.remove(auxFile)
        
    p = subprocess.Popen(cmd.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        return False
    
    if error is not None:
        output += error
        
    m = re.match("^.*?(Block\s+failed|Premature\s+end)", output,
                re.IGNORECASE|re.MULTILINE|re.DOTALL)
    if m:
        return False
    
    return True

def isDEM(filename):
    '''Return true if a file is a recognized DEM.'''
    return (len(filename) >= 8 and filename[-8:] == '_DEM.tif')

def isLidar(filename):
    '''Return true if the file is an input (not converted) lidar format'''
    extension = fileExtension(filename)
    return (extension == '.qi') or (extension == '.hdf5') or \
           (extension == '.h5') or (extension == '.TXT')

def isValidLidarCSV(filename):
    '''Check that a lidar csv file is valid. It must have at least threee entries on one line.'''
    
    if not os.path.exists(filename):
        return False

    with open(filename, "r") as ins:
        array = []
        for line in ins:

            # Skip empty lines
            if len(line) == 0: continue

            # Skip lines starting with spaces followed by #
            m = re.match("^\s*\#", line)
            if m:
                continue

            line = string.replace(line, ',',  ' ')
            line = string.replace(line, '\t', ' ')

            vals = line.split(' ')
            num = 0
            for val in vals:
                if len(val) == 0: continue
                num += 1

            if num >= 3:
                return True
            else:
                return False

    return False

def getLidarCsvFormat(filename):
    '''Returns the ASP CSV format string to use for a lidar file'''
    extension = fileExtension(filename)
    if extension == '.TXT': # LVIS
        return '5:lat,4:lon,6:height_above_datum'
    return '1:lat,2:lon,3:height_above_datum' # ATM
    
def getCameraGsdAndBounds(imagePath, cameraPath, logger, referenceDem=None, projString=""):
    '''Compute the GSD and bounding box of a single camera.
       Use the DEM is provided, otherwise use the datum.'''
    
    # Run GSD tool
    tool = asp_system_utils.which('camera_footprint')
    cmd = ('%s --quick --datum wgs84 -t nadirpinhole %s %s' %
            (tool, imagePath, cameraPath))
    if referenceDem:
        cmd += ' --dem-file ' + referenceDem
    cmd = cmd.split()
    if projString:
        cmd.append('--t_srs',)
        cmd.append(projString)
    logger.info(" ".join(cmd))
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    textOutput, err = p.communicate()
    logger.info(textOutput)
    
    # Extract the gsd from the output text
    m = re.findall(r"Computed mean gsd: (\d+\.*[0-9e\-]*)", textOutput)
    if len(m) != 1: # An unknown error occurred, move on.
        raise Exception('Unable to compute GSD for file: ' + cameraPath)
    gsd = float(m[0])

    # Extract the bounding box from the output text
    print textOutput
    m = re.findall(
          r"Origin: \(([0-9e\-\.\+]*), ([0-9e\-\.\+]*)\) width: ([0-9e\-\.\+]*) height: ([0-9e\-\.\+]*)",
          textOutput)
    if (len(m) != 1) and (len(m[0]) != 4): # An unknown error occurred, move on.
        raise Exception('Unable to compute GSD for file: ' + cameraPath)    
    bounds = [float(x) for x in m[0]]

    return (gsd, bounds)

def getCameraGsdAndBoundsRetry(imagePath, cameraPath, logger, referenceDem, projString=""):
    '''As getCameraGsd, but retry with the datum if the DEM fails.'''

    try:
        # Compute GSD using the DEM
        # - Only need the projection string when not using a DEM
        results = getCameraGsdAndBounds(imagePath, cameraPath, logger, referenceDem)
    except:
        # If that failed, try intersecting with the datum.
        logger.info('DEM intersection failed, trying with datum...')
        results = getCameraGsdAndBounds(imagePath, cameraPath, logger, None, projString)
     
    return results

def getFrameRangeFromBatchFolder(folder):
    '''Returns (startFrame, endFrame) for a batch folder'''

    # Extract just the desired folder name    
    m = re.search('batch_[0-9]+_[0-9]+', folder)
    if not m:
        raise Exception('Failed to find batch frames in folder:' + folder)
    text = m.group(0)
    
    # Parse the two frame numbers
    batchLen   = len('batch')
    start      = text.find('batch_')
    next       = text.find('_', start+batchLen+1)
    startFrame = int(text[batchLen+1:next])
    endFrame   = int(text[next+1:])
    
    return (startFrame, endFrame)

def xmlFile(filename):
    '''Return the matching xml file path for the input file.'''
    
    if (len(filename) >= 8 and filename[-7:-4] == 'DEM'): # DEM.tif and DEM.tfw
        #file_DEM.tif and file_DEM.tfw becomes file.xml
        return filename[:-8] + '.xml'
    
    # For other types
    return filename + '.xml'

def xmlToImage(filename):
    if fileExtension(filename) != '.xml':
        raise Exception("Not an XML file: " + filename)
    return filename[:-4]
    
def tfwFile(filename):
    '''Return the matching tfw file path for the input file.'''
    return filename[:-4] + '.tfw'


def isFloat(value):
    '''Return true if the input value can be converted to a float.'''
    try:
      float(value)
      return True
    except:
      return False

def hasValidChkSum(filename, logger):
    '''Some files have an xml file containing the chksum. If so, varify
       its validity. This applies to orthoimages, DEMs, and tfw files.'''

    isTfw = (fileExtension(filename) == '.tfw')
    
    if not os.path.exists(filename):
        logger.info("File does not exist: " + filename)
        return False

    baseFile = os.path.basename(filename)
    
    xml_file = xmlFile(filename)
    if not os.path.exists(xml_file):
        logger.info("File does not exist: " + xml_file)
        return False
    
    expectedChksum = ''
    chkSumCount = 0
    currFile = ''
    with open(xml_file, "r") as xf:
        for line in xf:

            # There can be multiple files
            m = re.match("^.*?\<DistributedFileName\>(.*?)\<", line, re.IGNORECASE)
            if m:
                currFile = m.group(1)
                
            # Encompass both kinds of checksum
            m = re.match("^.*?\<Checksum\>(\w+)(\||\<)", line, re.IGNORECASE)
            if m:
                chkSumCount += 1

                # There can be multiple checksums. The file can give a hint:
                if currFile != '':
                    if currFile == baseFile:
                        expectedChksum = m.group(1)
                else:
                    # Just pick the first chksum
                    if chkSumCount == 1:
                        expectedChksum = m.group(1)
                    
    actualChksum = hashlib.md5(open(filename,'rb').read()).hexdigest()

    if actualChksum != expectedChksum or actualChksum == '' or expectedChksum == '':
        logger.info("Computed chksum: ", actualChksum, filename)
        logger.info("Expected chksum: ", expectedChksum, filename)
        
        return False
    
    return True

def isValidTfw(filename, logger):
    '''This file must have 6 lines of floats and a valid chksum.'''
    
    if fileExtension(filename) != '.tfw':
        return False

    if not hasValidChkSum(filename, logger):
        return False
    
    count = 0
    with open(filename, "r") as xf:
        for line in xf:
            line = line.strip()
            if isFloat(line):
                count += 1
    return (count >= 6)

def parseLatitude(filename):
    '''Find the <PointLatitude> value in the given file.'''

    if not os.path.exists(filename):
        raise Exception("Could not find file: " + filename)

    latitude = None
    with open(filename, "r") as xf:
        for line in xf:
            m = re.match("^.*?\<PointLatitude\>(.*?)\<", line, re.IGNORECASE)
            if m:
                latitude = float(m.group(1))
                break

    if latitude is None:
        raise Exception("Could not parse positive or negative latitude from: " + filename)

    return latitude

def getCameraFileName(imageFileName):
    '''Get the camera file name we associate with an input image file.'''
    return imageFileName.replace('.tif', '.tsai')

# This function works for raw images, camera tsai files, orthoimages,
# DEMs, lvis, atm1, and atm2 files.
def getFrameNumberFromFilename(filename):

    # Match 2009_10_16_<several digits>.JPG
    m = re.match("^.*?\d+\_\d+\_\d+\_(\d+)\.JPG", filename, re.IGNORECASE)
    if m: return int(m.group(1))

    # Match DMS_20111012_145559_00156.tif or .tsai (created by our python scripts)
    m = re.match("^.*?DMS\_\d+\_\d+\_(\d+)\.(tif|tsai)", filename, re.IGNORECASE)
    if m: return int(m.group(1))
    
    # Match DMS_1000109_03939_20091016_23310503_V02.tif (fetched from NSIDC)
    m = re.match("^.*?DMS\_\d+\_(\d+)\w+\.tif", filename, re.IGNORECASE)
    if m: return int(m.group(1))

    # Match IODMS3_20111018_14295436_00347_DEM.tif
    m = re.match("^.*?IODMS[a-zA-Z0-9]*?\_\d+\_\d+\_(\d+)\w+DEM\.tif", filename, re.IGNORECASE)
    if m: return int(m.group(1))
    
    # Match ILVIS2_AQ2015_0929_R1605_060226.TXT
    m = re.match("^.*?ILVIS.*?_(\d+)(.TXT)", filename, re.IGNORECASE)
    if m: return int(m.group(1))

    # Match ILATM1B_20091016_193033.atm4cT3.qi
    # or    ILATM1B_20160713_195419.ATM5BT5.h5
    m = re.match("^.*?ILATM\w+\_\d+\_(\d+)\.\w+\.(h5|qi)", filename, re.IGNORECASE)
    if m: return int(m.group(1))

    raise Exception('Could not parse: ' + filename)

def getTifs(folder, prependFolder=False):
    '''Get tif files in given directory, ignoring _sub files.
    This returns the files without sorting.'''
    files = []
    for f in os.listdir(folder):

        # Skip non-image files and sub-images
        ext = os.path.splitext(f)[1]
        if (ext != '.tif') or ('_sub' in f) or ('pct.tif' in f) or ('_hillshade_' in f):
            continue
        if prependFolder:
            files.append(os.path.join(folder, f))
        else:
            files.append(f)

    return files

def getJpegs(folder):
    '''Get jpeg files in given directory. This returns the files
    without sorting or the folder name prepended to them.'''

    files = []
    for f in os.listdir(folder):

        ext = os.path.splitext(f)[1]
        if ext != '.JPG':
            continue
        files.append(f)

    return files

def getByExtension(folder, ext):
    '''Get files with given extension. This returns the files without
    sorting or the folder name prepended to them.'''
    files = []
    for f in os.listdir(folder):

        curr_ext = os.path.splitext(f)[1]
        if ext != curr_ext:
            continue
        files.append(f)
        
    return files

def getDems(folder):
    '''Get DEM files. This returns the files without sorting or the
    folder name prepended to them.'''

    files = []
    for f in os.listdir(folder):

        if not isDEM(f):
            continue
        files.append(f)

    return files

def getLidar(folder):
    '''Get LIDAR files. This returns the files without sorting or the
    folder name prepended to them.'''

    files = []
    for f in os.listdir(folder):
        if not isLidar(f):
            continue
        files.append(f)

    return files

def getMatchingFrames(inputFiles, candidateFiles):
    '''Given a list of input files and candidate files,
       returns a list of candidate files having the same
       frame numbers as the input files in the same order.
       An entry will be 'None' if there is no matching frame.'''
       
    # Init output structure
    numFiles   = len(inputFiles)
    outputList = []
    for i in range(0,numFiles):
        outputList.append(None)
    numMatched = 0
    
    # Loop through all the candidate files
    for c in candidateFiles:
        candidateFrame = getFrameNumberFromFilename(c)
        # Compare them to each of the input files
        for i in range(0,numFiles):
            if outputList[i]: # Skip matched files
                continue
            inputFrame = getFrameNumberFromFilename(inputFiles[i])
            if inputFrame == candidateFrame: # If the frames match, record the file
                outputList[i] = c
                numMatched += 1
                if numMatched == numFiles: # Quit once all files are matched
                    return outputList
    return outputList
            

def parseDateTimeStrings(dateString, timeString, secondFix, returnSecondOnly):
    '''Parse strings in the format 20110323_17433900.'''

    MILLISECOND_TO_MICROSECOND = 10000
    
    year    = int(dateString[0:4])
    month   = int(dateString[4:6])
    day     = int(dateString[6:8])
    hour    = int(timeString[0:2])
    minute  = int(timeString[2:4])
    second  = int(timeString[4:6])

    if returnSecondOnly:
        return second
    
    if secondFix: # Some files number the seconds from 1-60!
        second  = second - 1
    usecond = 0
    if len(timeString) > 6:
        usecond = int(timeString[6:8]) * MILLISECOND_TO_MICROSECOND
    
    try:
        result = datetime.datetime(year, month, day, hour, minute, second, usecond)
        return result
    except Exception, e:
        raise Exception('Caught exception processing dateString: ' 
                        + dateString +', timeString: ' + timeString
                        +'\n with values: ' + str((year, month, day, hour, minute, second, usecond))
                        +'\n' + str(e))

def secondsSinceMidnightToHHMMSS(secondsSinceMidnight):
    '''Convert from integer number to HHMMSS string.'''

    hours, remainder = divmod(secondsSinceMidnight, 3600)
    minutes, seconds = divmod(remainder, 60)
    
    return ('%02d%02d%02d' % (hours, minutes, seconds))
    

def parseTimeStamps(fileName):
    '''Pull two six or eight digit values from the given file name
       as the time and date stamps.'''

    # Start by handling ILVIS2_AQ2011_1012_R1203_049752.TXT
    m = re.match("^.*?ILVIS\d\_[A-Z][A-Z](\d\d\d\d)\_(\d\d\d\d)\_.*?\_(\d+)\.TXT",
                 fileName, re.IGNORECASE)
    if m:
        lidarDateString = m.group(1) + m.group(2)
        secondsInDay    = int(m.group(3))
        lidarTimeString = secondsSinceMidnightToHHMMSS(secondsInDay)
        return [lidarDateString, lidarTimeString]

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

def lidarFiles(lidarFolder):
    '''Find lidar files in given folder. Note that the folder
    name is not prepended to the file names.'''
    
    # All files in the folder
    allFiles = os.listdir(lidarFolder)

    # See based on existing files if we are dealing with LVIS
    isLVIS = False
    for f in allFiles:
        m = re.match("^.*?ILVIS.*?\d+\.TXT", f, re.IGNORECASE)
        if m:
            isLVIS = True
            
    # Get just the files we converted to csv format or plain text LVIS files
    lidarFiles = []    
    for f in allFiles:
        extension = os.path.splitext(f)[1]
        if 'html.csv' in f:
            continue # skip index.html.csv
        if (not isLVIS and extension == '.csv') or (isLVIS and extension == '.TXT'):
           lidarFiles.append(f)
    lidarFiles.sort()

    lidarExt = '.csv'
    if isLVIS:
        lidarExt = '.TXT'

    return (lidarFiles, lidarExt, isLVIS)

def lidar_pair_prefix():
    return 'LIDAR_PAIR_'

def pairFiles(pairedFolder):
    '''Find the lidar pairs. Here we prepend the folder name to the files.'''

    allFiles   = os.listdir(pairedFolder)
    pairs = []
    
    for f in allFiles:

        # Filter by lidar pair prefix. Do not filter by extension or do it well. That
        # one can be .csv or .TXT.
        if lidar_pair_prefix() not in f:
            continue

        # Extract time for this file
        fullPath = os.path.join(pairedFolder, f)
        pairs.append(fullPath)
        
    return pairs

    
def findMatchingLidarFile(imageFile, lidarFolder):
    '''Given an image file, find the best lidar file to use for alignment.'''
    
    # Look in the paired lidar folder, not the original lidar folder.
    pairedFolder = os.path.join(lidarFolder, 'paired')
    
    vals = parseTimeStamps(imageFile)
    if len(vals) < 2:
        raise Exception('Failed to parse the date and time from: ' + imageFile)
    secondFix = False
    returnSecondOnly = False
    imageDateTime = parseDateTimeStrings(vals[0], vals[1], secondFix, returnSecondOnly)
    
    #print 'INPUT = ' + str(imageDateTime)
    
    # Search for the matching file in the lidar folder.
    # - We are looking for the closest lidar time that starts BEFORE the image time.
    # - It is possible for an image to span lidar files, we will address that if we need to!
    bestTimeDelta = datetime.timedelta.max
    bestLidarFile = 'NA'
    lidarFiles    = pairFiles(pairedFolder)
    zeroDelta     = datetime.timedelta()

    if len(lidarFiles) <= 0:
        raise Exception("Empty directory of pairs in " + pairedFolder)
    
    # First see if we need correction for sometimes seconds going from 1 to 60.
    minSec = 60
    maxSec = 0
    for lidarPath in lidarFiles:
        
        vals = parseTimeStamps(lidarPath)
        if len(vals) < 2: continue # ignore bad files

        secondFix = False
        returnSecondOnly = True
        second = parseDateTimeStrings(vals[0], vals[1], secondFix, returnSecondOnly)
        if second < minSec:
            minSec = second
        if second > maxSec:
            maxSec = second

    if minSec > maxSec:
        raise Exception("Failure parsing " + pairedFolder)

    if minSec <= 0 and maxSec >= 60:
        raise Exception("The second range goes from  " + str(minSec) + " to " + str(maxSec))

    secondFix = False
    if maxSec >= 60:
        secondFix = True
        
    for lidarPath in lidarFiles:

        vals = parseTimeStamps(lidarPath)
        if len(vals) < 2:
            continue # ignore bad files

        try:
            returnSecondOnly = False
            lidarDateTime = parseDateTimeStrings(vals[0], vals[1], secondFix, returnSecondOnly)
        except Exception as e:
            raise Exception('Failed to parse datetime for lidar file: ' + lidarPath + '\n' +
                            'Error is: ' + str(e))

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

def fileNonEmpty(path):
    '''Make sure file exists and is non-empty.'''
    return os.path.exists(path) and (os.path.getsize(path) > 0)

# It is faster to invoke one curl command for multiple files.
# Do not fetch files that already exist. Note that we expect
# that each file looks like outputFolder/name.<ext>,
# and each url looks like https://.../name.<ext>.
def fetchFilesInBatches(baseCurlCmd, batchSize, dryRun, outputFolder, files, urls, logger):
    '''Fetch a list of files in batches using curl.'''

    curlCmd = baseCurlCmd
    numFiles = len(files)

    if numFiles != len(urls):
        raise Exception("Expecting as many files as urls.")
    
    currentFileCount = 0
    for fileIter in range(numFiles):
        
        if not fileNonEmpty(files[fileIter]):
            # Add to the command
            curlCmd += ' -O ' + urls[fileIter]
            currentFileCount += 1 # Number of files in the current download command

        # Download the indicated files when we hit the limit or run out of files
        if ( (currentFileCount >= batchSize) or (fileIter == numFiles - 1) ) and \
               currentFileCount > 0:
            logger.info(curlCmd)
            if not dryRun:
                logger.info("Saving the data in " + outputFolder)
                p = subprocess.Popen(curlCmd, cwd=outputFolder, shell=True)
                os.waitpid(p.pid, 0)
                
            # Start command fresh for the next file
            currentFileCount = 0
            curlCmd = baseCurlCmd
    
# This block of code is just to get a non-blocking keyboard check!
import signal
class AlarmException(Exception):
    pass
def alarmHandler(signum, frame):
    raise AlarmException
def nonBlockingRawInput(prompt='', timeout=20):
    '''Return a key if pressed or an empty string otherwise.
       Waits for timeout, non-blocking.'''
    signal.signal(signal.SIGALRM, alarmHandler)
    signal.alarm(timeout)
    try:
        text = raw_input(prompt)
        signal.alarm(0)
        return text
    except AlarmException:
        pass # Timeout
    signal.signal(signal.SIGALRM, signal.SIG_IGN)
    return ''

def logger_print(logger, msg):
    '''Print to logger, if present. This helps keeps all messages in sync.'''
    if logger is not None:
        logger.info(msg)
    else:
        print(msg)
        
def waitForTaskCompletionOrKeypress(taskHandles, logger = None, interactive=True, quitKey='q',
                                    sleepTime=20):
    '''Block in this function until the user presses a key or all tasks complete.'''

    # Wait for all the tasks to complete
    notReady = len(taskHandles)
    while notReady > 0:

        if interactive:
            # Wait and see if the user presses a key
            msg = 'Waiting on ' + str(notReady) + ' process(es), press '+str(quitKey)+'<Enter> to abort...\n'
            keypress = nonBlockingRawInput(prompt=msg, timeout=sleepTime)
            if keypress == quitKey:
                logger_print(logger, 'Recieved quit command!')
                break
        else:
            logger_print(logger, "Waiting on " + str(notReady) + ' incomplete tasks.')
            time.sleep(sleepTime)
            
        # As long as we have this process waiting, keep track of our resource consumption.
        cpuPercentUsage = psutil.cpu_percent()
        memInfo         = psutil.virtual_memory()
        memUsed         = memInfo[0] - memInfo[1]
        memPercentUsage = float(memUsed) / float(memInfo[0])

        usageMessage = ('CPU percent usage = %f, Memory percent usage = %f' 
                        % (cpuPercentUsage, memPercentUsage))
        logger_print(logger, usageMessage)
            
        # Otherwise count up the tasks we are still waiting on.
        notReady = 0
        for task in taskHandles:
            if not task.ready():
                notReady += 1
    return

def stopTaskPool(pool):
    '''Stop remaining tasks and kill the pool.'''

    PROCESS_POOL_KILL_TIMEOUT = 3
    pool.close()
    time.sleep(PROCESS_POOL_KILL_TIMEOUT)
    pool.terminate()
    pool.join()


def setUpLogger(outputFolder, logLevel, logPathPrefix):
    '''Set up the root logger so all called files will write to the same output file.'''

    # Generate a timestamped log file in the output folder
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    logName   = logPathPrefix +'_'+ timestamp + '.txt'
    logPath   = os.path.join(outputFolder, logName)
    
    logger = logging.getLogger()    # Call with no argument to configure the root logger.
    logger.setLevel(level=logLevel)
    logger.propagate = False # This is a unique logger, don't copy messages to parent modules.
    
    fileHandler = logging.FileHandler(logPath)
    formatter   = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fileHandler.setFormatter(formatter)
    logger.addHandler(fileHandler)
    logger.addHandler(logging.StreamHandler()) # Mirror logging to console

    logger = logging.getLogger(__name__) # We configured root, but continue logging with the normal name.
    return logger

# TODO: Rename to isSouth.
def checkSite(site):
    '''Verify the site is legal and return True if it is in the southern hemisphere.'''
    possibleSites = ['AN', 'GR', 'AL']
    if site not in possibleSites:
        raise Exception("Site must be either AN, GR, or AL.")
    isSouth = (site == 'AN')
    return isSouth

def getElevationLimits(site):
    '''Return the min and max elevation expected at a given site'''
    # Would it work better to compute this on a per-flight or per-DEM basis?
    if site == 'AN':
        return (-50, 4500)
    if site == 'GR':
        return (-50, 3500)
    if site == 'AL':
        return (-50, 3500)

def getEpsgCode(isSouth, asString=True):
    '''Return EPSG code for a location.  See notes in getProjString.'''
    code = 3413
    if isSouth:
        code = 3031
    if asString:
        return 'EPSG:' + str(code)
    return code

def getProjString(isSouth, addQuotes=False):
    '''Return the correct proj string for the pole.  Surrounding quotes are optional'''
    # EPSG 3413 - WGS 84 / NSIDC Sea Ice Polar Stereographic North
    #PROJ_STRING_NORTH = '+proj=stere +lat_0=90 +lat_ts=70 +lon_0=-45 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    # EPSG 3031 - WGS 84 / Antarctic Polar Stereographic
    #PROJ_STRING_SOUTH = '+proj=stere +lat_0=-90 +lat_ts=-71 +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    #s = PROJ_STRING_NORTH
    #if isSouth:
    #    s = PROJ_STRING_SOUTH
    s = getEpsgCode(isSouth, asString=True)
    if addQuotes:
        return '"'+s+'"'
    else:
        return s 


def getReferenceDemName(site):
    '''Returns the DEM name to use for a given location'''

    # Note: The AN and GR DEMs match our projection system for those sites.

    if site == 'AN':
        #return 'krigged_dem_nsidc_ndv0_fill.tif' # Higher resolution
        return 'ramp200dem_wgs_v2.tif' # Used to produce the orthos - EPSG 3031
    if site == 'GR':
        #return 'gimpdem_90m_v1.1.tif' # Higher resolution
        return 'NSIDC_Grn1km_wgs84_elev.tif' # Used to produce the orthos - EPSG 3413
    if site == 'AL':
        # Supposedly these were produced with the SRTM map but that map
        #  does not seem to actually include Alaska.  This may mean the NED
        #  map (60 meter) was used but this would require tile handling logic
        #  so for now we will try to use this single 300m DEM.
        return 'akdem300m.tif'
        # Proj string: '+proj=aea +lat_1=55 +lat_2=65 +lat_0=50 +lon_0=-154 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs '


def readGeodiffOutput(inputPath):
    '''Read in the header from a geodiff output csv file.
       Returns a dictionary containing 'Max', 'Min', 'Mean', and 'StdDev'. '''

    if not os.path.exists(inputPath):
        raise Exception('geodiff output file ' + inputPath + ' does not exist!')

    # Pack the results into a dictionary
    keywords = ['Max', 'Min', 'Mean', 'StdDev']
    results = {}

    ext = os.path.splitext(inputPath)[1]
    if ext == '.csv':
        with open(inputPath, 'r') as f:
            for line in f:
                if '#' not in line: # Quit when we go past the comment lines
                    break
                for word in keywords: # Look for the four values
                    if word in line:
                        parts = line.split(':') # Extract the number
                        if len(parts) != 2:
                            raise Exception('Error parsing geodiff line:\n' + line)
                        results[word] = float(parts[1])
                        break # Go on to the next line in the file
    else: # Handle .tif files
        stats = asp_image_utils.getImageStats(inputPath)[0]
        results['Min'   ] = stats[0]
        results['Max'   ] = stats[1]
        results['Mean'  ] = stats[2]
        results['StdDev'] = stats[3]
        
    return results

def isBatchValid(batchFolder):
    '''Returns true if the given batch has produced a good output DEM.'''

    # The maximum allowed distance between our DEM and the lidar file.
    MAX_LIDAR_DEM_DIFF_METERS = 5

    try:
        diffPath = os.path.join(batchFolder, 'out-diff.csv')
        results  = readGeodiffOutput(diffPath)
        
        return (results['MEAN'] <= MAX_LIDAR_DEM_DIFF_METERS)
    except:
        return False

# For debugging functions
#if __name__ == "__main__":
#    print getFrameRangeFromBatchFolder('/home/test/batch_234_1425/')

