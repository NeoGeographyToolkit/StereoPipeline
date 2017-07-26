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

import os, sys, datetime, time, subprocess, logging, re, hashlib

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

def isValidImage(filename):
    '''Check that an image file is not corrupted in some way. This check is not enough.'''
    
    if not os.path.exists(filename):
        return False
    
    gdalinfoPath = asp_system_utils.which("gdalinfo")
    cmd = gdalinfoPath + ' ' + filename
    
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        return False

    return True

def isDEM(filename):
    '''Return true if a file is a recognized DEM.'''
    return (len(filename) >= 8 and filename[-8:] == '_DEM.tif')

def isLidar(filename):
    extension = fileExtension(filename)
    return (extension == '.qi') or (extension == '.hdf5') or \
           (extension == '.h5') or (extension == '.TXT')

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

def hasValidChkSum(filename):
    '''Some files have an xml file containing the chksum. If so, varify
       its validity. This applies to orthoimages, DEMs, and tfw files.'''

    isTfw = (fileExtension(filename) == '.tfw')
    
    if not os.path.exists(filename):
        return False

    baseFile = os.path.basename(filename)
    
    xml_file = xmlFile(filename)
    if not os.path.exists(xml_file):
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
        print("Computed chksum: ", actualChksum, filename)
        print("Expected chksum: ", expectedChksum, filename)
        
        return False
    
    return True

def isValidTfw(filename):
    '''This file must have 6 lines of floats and a valid chksum.'''
    
    if fileExtension(filename) != '.tfw':
        return False

    if not hasValidChkSum(filename):
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

def getTifs(folder):
    '''Get tif files in given directory, ignoring _sub files.
    This returns the files without sorting or the folder name prepended to them.'''
    files = []
    for f in os.listdir(folder):

        # Skip non-image files and sub-images
        ext = os.path.splitext(f)[1]
        if (ext != '.tif') or ('_sub' in f):
            continue
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

def parseDateTimeStrings(dateString, timeString, secFix=False):
    '''Parse strings in the format 20110323_17433900.'''
    
    MILLISECOND_TO_MICROSECOND = 10000
    
    year    = int(dateString[0:4])
    month   = int(dateString[4:6])
    day     = int(dateString[6:8])
    hour    = int(timeString[0:2])
    minute  = int(timeString[2:4])
    second  = int(timeString[4:6])
    if secFix: # Some files number the seconds from 1-60!
        second  = second - 1
    usecond = 0
    if len(timeString) > 6:
        usecond = int(timeString[6:8]) * MILLISECOND_TO_MICROSECOND
    
    return datetime.datetime(year, month, day, hour, minute, second, usecond)

def parseTimeStamps(fileName):
    '''Pull two six or eight digit values from the given file name
       as the time and date stamps.'''

    # Start by handling ILVIS2_AQ2011_1012_R1203_049752.TXT
    m = re.match("^.*?ILVIS\d\_[A-Z][A-Z](\d\d\d\d)\_(\d\d\d\d)\_.*?\_(\d+)\.TXT",
                 fileName, re.IGNORECASE)
    if m:
        imageDateString = m.group(1) + m.group(2)
        imageTimeString = m.group(3)
        return [imageDateString, imageTimeString]

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

        try:
            lidarDateTime = parseDateTimeStrings(vals[0], vals[1], secFix=True)
        except Exception as e:
            raise Exception('Failed to parse datetime for lidar file: ' + f + '\n' +
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


def waitForTaskCompletionOrKeypress(taskHandles, interactive=True, quitKey='q', sleepTime=20):
    '''Block in this function until the user presses a key or all tasks complete.'''

    # Wait for all the tasks to complete
    notReady = len(taskHandles)
    while notReady > 0:
        
        if interactive:
            # Wait and see if the user presses a key
            msg = 'Waiting on ' + str(notReady) + ' process(es), press '+str(quitKey)+'<Enter> to abort...\n'
            keypress = nonBlockingRawInput(prompt=msg, timeout=sleepTime)
            if keypress == quitKey:
                logger.info('Recieved quit command!')
                break
        else:
            print("Waiting on " + str(notReady) + ' incomplete tasks.')
            time.sleep(sleepTime)
            
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
    
    fileHandler = logging.FileHandler(logPath)
    formatter   = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fileHandler.setFormatter(formatter)
    logger.addHandler(fileHandler)
    logger.addHandler(logging.StreamHandler()) # Mirror logging to console

    logger = logging.getLogger(__name__) # We configured root, but continue logging with the normal name.
    return logger


# For debugging functions
#if __name__ == "__main__":
#    print getFrameRangeFromBatchFolder('/home/test/batch_234_1425/')

