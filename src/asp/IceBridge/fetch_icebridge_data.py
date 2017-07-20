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
Tool for downloading IceBridge data
'''

import sys, os, re, subprocess, optparse, logging
import icebridge_common
import httplib
from urlparse import urlparse

logger = logging.getLogger(__name__)

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = basepath    + os.pathsep + os.environ["PATH"]

#------------------------------------------------------------------------------

# Constants
LIDAR_TYPES = ['lvis', 'atm1', 'atm2']

def checkIfUrlExists(url):
    '''Return true if the given IceBrige folder URL is valid'''
    p    = urlparse(url)
    conn = httplib.HTTPConnection(p.netloc)
    conn.request('HEAD', p.path)
    resp = conn.getresponse()
    # Invalid pages return 404, valid pages should return one of the numbers below.
    #print 'URL response = ' + str(resp.status)
    return (resp.status == 403) or (resp.status == 301)

def makeYearFolder(year, site):
    '''Generate part of the URL.  Only used for images.'''
    return str(year) + '_' + site + '_NASA'

def makeDateFolder(year, month, day, ext, fileType):
    '''Generate part of the URL.'''

    if fileType == 'image':
        datePart = ('%02d%02d%04d%s') % (month, day, year, ext)
        return datePart +'_raw'
    else: # Used for all other cases
        datePart = ('%04d.%02d.%02d%s') % (year, month, day, ext)
        return datePart

def getFolderUrl(year, month, day, ext, site, fileType):
    '''Get full URL to the location where the files are kept.'''

    if fileType == 'image':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE_FTP/IODMS0_DMSraw_v01'
        yearFolder = makeYearFolder(year, site)
        dateFolder = makeDateFolder(year, month, day, ext, fileType)
        folderUrl  = os.path.join(base, yearFolder, dateFolder)
        return folderUrl
    # The other types share more formatting
    if fileType == 'ortho':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODMS1B.001'       
    elif fileType == 'dem':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODMS3.001'
    elif fileType == 'lvis':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/ILVIS2.001/'
    elif fileType == 'atm1':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/ILATM1B.001/'
    elif fileType == 'atm2':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/ILATM1B.002/'
    else:
        raise("Unknown type: " + fileType)
    
    dateFolder = makeDateFolder(year, month, day, ext, fileType)
    folderUrl  = os.path.join(base, dateFolder)
    
    return folderUrl

def readIndexFile(parsedIndexPath):
    frameDict  = {}
    urlDict    = {}
    with open(parsedIndexPath, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) <= 2:
                # Od index file
                raise Exception("Invalid index file.")
            
            frameNumber = int(parts[0])
            frameDict[frameNumber] = parts[1].strip()
            urlDict[frameNumber]   = parts[2].strip()

    return (frameDict, urlDict)

def fetchAndParseIndexFileAux(baseCurlCmd, folderUrl, path, fileType):
    '''Retrieve the index file for a folder of data and create
    a parsed version of it that contains frame number / filename pairs.'''

    curlCmd = baseCurlCmd + ' ' + folderUrl + ' > ' + path
    
    # Download the file
    logger.info(curlCmd)
    p = subprocess.Popen(curlCmd, shell=True)
    os.waitpid(p.pid, 0)
    
    # Find all the file names in the index file and
    #  dump them to a new index file
    logger.info('Extracting file name list from index.html file...')
    with open(path, 'r') as f:
        indexText = f.read()

    if os.path.exists(path): os.remove(path) # don't need this anymore
    
    # Extract just the file names
    fileList = [] # ensure initialization
    if fileType == 'image':
        fileList = re.findall(">[0-9_]*.JPG", indexText, re.IGNORECASE)
    if fileType == 'ortho':
        fileList = re.findall(">DMS\w*.tif<", indexText, re.IGNORECASE) 
    if fileType == 'dem':
        fileList = re.findall(">IODMS\w*DEM.tif", indexText, re.IGNORECASE)
    if fileType == 'lvis':
        fileList = re.findall(">ILVIS\w+.TXT", indexText, re.IGNORECASE)
    if fileType == 'atm1':
        fileList = re.findall(">ILATM1B[0-9_]*.ATM4\w+.qi", indexText, re.IGNORECASE)
        #                      >ILATM1B_20111018_145455.ATM4BT4.qi
        #   or                 >ILATM1B_20091016_165112.atm4cT3.qi
    if fileType == 'atm2':
        # Match ILATM1B_20160713_195419.ATM5BT5.h5 
        fileList = re.findall(">ILATM1B[0-9_]*.ATM\w+.h5", indexText, re.IGNORECASE)
    
    frameDict  = {}
    urlDict    = {}

    # For each entry that matched the regex, record: the frame number and the file name.
    for filename in fileList:
        # Get rid of '>' and '<'
        filename = filename.replace(">", "")
        filename = filename.replace("<", "")
        frame = icebridge_common.getFrameNumberFromFilename2(filename)
        frameDict[frame] = filename
        # note that folderUrl can vary among orthoimages, as sometimes
        # some of them are in a folder for the next day.
        urlDict[frame]   = folderUrl
        
    return (frameDict, urlDict)

# Create a list of all files that must be fetched unless done already.
def fetchAndParseIndexFile(options, baseCurlCmd, outputFolder):

    # If we need to parse the next flight day as well, as expected in some runs,
    # we will fetch two html files, but create a single index out of them.
    dayVals = [0]
    if options.fetchNextDay:
        dayVals.append(1)
        options.refetchIndex = True # Force refetch, to help with old archives
        
    # See if to wipe the index
    if options.type in LIDAR_TYPES:
        filename = 'lidar' + '_index.html'
    else:
        filename = options.type + '_index.html'
        
    indexPath       = os.path.join(outputFolder, filename)
    parsedIndexPath = indexPath + '.csv'

    if options.refetchIndex:
        os.system('rm -f ' + parsedIndexPath)

    if icebridge_common.fileNonEmpty(parsedIndexPath):
        logger.info('Already have the index file ' + parsedIndexPath + ', keeping it.')
        return parsedIndexPath
    
    frameDict  = {}
    urlDict    = {}

    for dayVal in dayVals:

        # Find folderUrl which contains all of the files
        if options.type in LIDAR_TYPES:
            options.allFrames = True # For lidar, always get all the frames!
            
            # For lidar, the data can come from one of three sources.
            # Unfortunately sometimes there is more than one source, and then
            # we need to pick by latitude.
            folderUrls = []
            lidar_types = []
            for lidar in LIDAR_TYPES:
                folderUrl = getFolderUrl(options.year, options.month,
                                         options.day + dayVal, # note here the dayVal
                                         options.ext, options.site, lidar)
                logger.info('Checking lidar URL: ' + folderUrl)
                if checkIfUrlExists(folderUrl):
                    logger.info('Found match with lidar type: ' + lidar)
                    folderUrls.append(folderUrl)
                    lidar_types.append(lidar)

            if len(folderUrls) == 0:
                logger.info('WARNING: Could not find any lidar data for the given date!')

            elif len(folderUrls) == 1:
                # Unique solution
                folderUrl = folderUrls[0]
                options.type = lidar_types[0]

            elif len(folderUrls) >= 2:
                # Multiple solutions. Pick the good one by latitude.
                logger.info("Multiples URLs to search: " + " ".join(folderUrls))
                count = -1
                isGood = False
                isSouth = (options.site == 'AN')
                for folderUrl in folderUrls:
                    count += 1
                    (localFrameDict, localUrlDict) = \
                                     fetchAndParseIndexFileAux(baseCurlCmd, folderUrl,
                                                               indexPath, lidar_types[count])
                    for frame in sorted(localFrameDict.keys()):
                        filename = localFrameDict[frame]
                        xmlFile  = icebridge_common.xmlFile(filename)
                        url      = os.path.join(folderUrl, xmlFile)
                                        
                        # Download the file
                        curlCmd = baseCurlCmd + ' ' + url + ' > ' + xmlFile
                        logger.info(curlCmd)
                        p = subprocess.Popen(curlCmd, shell=True)
                        os.waitpid(p.pid, 0)
                        
                        latitude = icebridge_common.parseLatitude(xmlFile)
                        if os.path.exists(xmlFile): os.remove(xmlFile)

                        if (isSouth and latitude < 0) or ( (not isSouth) and latitude > 0 ):
                            isGood = True
                            options.type = lidar_types[count]

                        # Stop at first file no matter what
                        break

                    if isGood:
                        break

                if not isGood:
                    raise Exception("None of these URLs are good: " + " ".join(folderUrls))
                
        else: # Other cases are simpler
            folderUrl = getFolderUrl(options.year, options.month,
                                     options.day + dayVal, # note here the dayVal
                                     options.ext, options.site, options.type)

        logger.info('Fetching from URL: ' + folderUrl)

        (localFrameDict, localUrlDict) = \
                         fetchAndParseIndexFileAux(baseCurlCmd, folderUrl, indexPath, options.type)

        # Append to the main index
        for frame in sorted(localFrameDict.keys()):
            # If we already have this frame, don't read it from the current day
            # as that's a recipe for mix-up
            if frame not in frameDict.keys():
                frameDict[frame] = localFrameDict[frame]
                urlDict[frame]   = localUrlDict[frame]
        
    # Write the combined index file
    with open(parsedIndexPath, 'w') as f:
        for frame in sorted(frameDict.keys()):
            f.write(str(frame) + ', ' + frameDict[frame] + ', ' + urlDict[frame] + '\n')
            
    return parsedIndexPath

def doFetch(options, outputFolder):
    
    # Verify that required files exist
    home = os.path.expanduser("~")
    if not (os.path.exists(home+'/.netrc') and os.path.exists(home+'/.urs_cookies')):
        logger.error('Missing a required authentication file!  See instructions here:\n' +
                     '    https://nsidc.org/support/faq/what-options-are-available-bulk-downloading-data-https-earthdata-login-enabled')
        return -1
    
    curlPath = asp_system_utils.which("curl")
    curlOpts    = ' -n -L '
    cookiePaths = ' -b ~/.urs_cookies -c ~/.urs_cookies '
    baseCurlCmd = curlPath + curlOpts + cookiePaths

    logger.info('Creating output folder: ' + outputFolder)
    os.system('mkdir -p ' + outputFolder)  

    parsedIndexPath = fetchAndParseIndexFile(options, baseCurlCmd, outputFolder)
    if not icebridge_common.fileNonEmpty(parsedIndexPath):
        # Some dirs are weird, both images, dems, and ortho.
        # Just accept whatever there is, but with a warning.
        logger.info('Warning: Missing index file: ' + parsedIndexPath)

    # Store file information in a dictionary
    # - Keep track of the earliest and latest frame
    logger.info('Reading file list from ' + parsedIndexPath)
    try:
        (frameDict, urlDict) = readIndexFile(parsedIndexPath)
    except:
        # We probably ran into old format index file. Must refetch.
        logger.info('Could not read index file. Try again.')
        options.refetchIndex = True
        parsedIndexPath = fetchAndParseIndexFile(options, baseCurlCmd, outputFolder)
        (frameDict, urlDict) = readIndexFile(parsedIndexPath)

    allFrames = sorted(frameDict.keys())
    firstFrame = icebridge_common.getLargestFrame()    # start big
    lastFrame  = icebridge_common.getSmallestFrame()   # start small
    for frameNumber in allFrames:
        if frameNumber < firstFrame:
            firstFrame = frameNumber
        if frameNumber > lastFrame:
            lastFrame = frameNumber
            
    if options.allFrames:
        options.startFrame = firstFrame
        options.stopFrame  = lastFrame

    # There is always a chance that not all requested frames are available.
    # That is particularly true for Fireball DEMs. Instead of failing,
    # just download what is present and give a warning. 
    if options.startFrame not in frameDict:
        logger.info("Warning: Frame " + str(options.startFrame) + \
                    " is not found in this flight.")
                    
    if options.stopFrame and (options.stopFrame not in frameDict):
        logger.info("Warning: Frame " + str(options.stopFrame) + \
                    " is not found in this flight.")

    allFilesToFetch = [] # Files that we will fetch, relative to the current dir. 
    allUrlsToFetch  = [] # Full url of each file.
    
    # What is the maximum number of files that can be downloaded with one call?
    MAX_IN_ONE_CALL = 100
        
    # Loop through all found frames within the provided range
    currentFileCount = 0
    lastFrame = ""
    if len(allFrames) > 0:
        lastFrame = allFrames[len(allFrames)-1]

    hasTfw = (options.type == 'dem')
    hasXml = ( (options.type in LIDAR_TYPES) or (options.type == 'ortho') or hasTfw )
    numFetched = 0
    for frame in allFrames:
        if (frame >= options.startFrame) and (frame <= options.stopFrame):

            filename = frameDict[frame]
            
            # Some files have an associated xml file. DEMs also have a tfw file.
            currFilesToFetch = [filename]
            if hasXml: 
                currFilesToFetch.append(icebridge_common.xmlFile(filename))
            if hasTfw: 
                currFilesToFetch.append(icebridge_common.tfwFile(filename))

            for filename in currFilesToFetch:    
                url        = os.path.join(urlDict[frame], filename)
                outputPath = os.path.join(outputFolder, filename)
                allFilesToFetch.append(outputPath)
                allUrlsToFetch.append(url)

    if options.maxNumToFetch > 0 and len(allFilesToFetch) > options.maxNumToFetch:
        allFilesToFetch = allFilesToFetch[0:options.maxNumToFetch]
        allUrlsToFetch = allUrlsToFetch[0:options.maxNumToFetch]
                
    icebridge_common.fetchFilesInBatches(baseCurlCmd, MAX_IN_ONE_CALL, options.dryRun, outputFolder,
                                         allFilesToFetch, allUrlsToFetch, logger)

    # Verify that all files were fetched and are in good shape
    failedFiles = []
    for outputPath in allFilesToFetch:

        if options.skipValidate: continue
        
        if not icebridge_common.fileNonEmpty(outputPath):
            logger.info('Missing file: ' + outputPath)
            failedFiles.append(outputPath)
            continue

        if icebridge_common.hasImageExtension(outputPath):
            if not icebridge_common.isValidImage(outputPath):
                logger.info('Found an invalid image. Will wipe it: ' + outputPath)
                if os.path.exists(outputPath): os.remove(outputPath)
                failedFiles.append(outputPath)
                continue
            else:
                logger.info('Valid image: ' + outputPath)

        # Verify the chcksum    
        if hasXml and len(outputPath) >= 4 and outputPath[-4:] != '.xml' \
               and outputPath[-4:] != '.tfw':
            isGood = icebridge_common.hasValidChkSum(outputPath)
            if not isGood:
                xmlFile = icebridge_common.xmlFile(outputPath)
                logger.info('Found invalid data. Will wipe it: ' + outputPath + ' ' + xmlFile)
                if os.path.exists(outputPath): os.remove(outputPath)
                if os.path.exists(xmlFile):    os.remove(xmlFile)
                failedFiles.append(outputPath)
                failedFiles.append(xmlFile)
                continue
            else:
                logger.info('Valid chksum: ' + outputPath)


        if hasTfw and icebridge_common.fileExtension(outputPath) == '.tfw':
            isGood = icebridge_common.isValidTfw(outputPath)
            if not isGood:
                xmlFile = icebridge_common.xmlFile(outputPath)
                logger.info('Found invalid data. Will wipe it: ' + outputPath + ' ' + xmlFile)
                if os.path.exists(outputPath): os.remove(outputPath)
                if os.path.exists(xmlFile):    os.remove(xmlFile)
                failedFiles.append(outputPath)
                failedFiles.append(xmlFile)
                continue
            else:
                logger.info('Valid tfw file: ' + outputPath)
                
            
    numFailed = len(failedFiles)
    if numFailed > 0:
        logger.info("Number of files that could not be processed: " + str(numFailed))
        
    return numFailed

def main(argsIn):

    # Command line parsing
    try:
        usage  = "usage: fetch_icebridge_data.py [options] output_folder"
        parser = optparse.OptionParser(usage=usage)

        parser.add_option("--year",  dest="year", type='int', default=None,
                          help="Number of processes to use (default program tries to choose best)")
        parser.add_option("--month",  dest="month", type='int', default=None,
                          help="Number of processes to use (default program tries to choose best)")
        parser.add_option("--day",  dest="day", type='int', default=None,
                          help="Number of processes to use (default program tries to choose best)")
        parser.add_option("--yyyymmdd",  dest="yyyymmdd", default=None,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_option("--site",  dest="site", default=None,
                          help="Name of the location of the images (AN or GR)")
        parser.add_option("--start-frame",  dest="startFrame", type='int', default=None,
                          help="Frame number or start of frame sequence")
        parser.add_option("--stop-frame",  dest="stopFrame", type='int', default=None,
                          help="End of frame sequence to download.")
        parser.add_option("--all-frames", action="store_true", dest="allFrames", default=False,
                          help="Fetch all frames for this flight.")
        parser.add_option("--fetch-from-next-day-also", action="store_true", dest="fetchNextDay",
                          default=False,
                          help="Sometimes some files are stored in next day's runs as well.")
        parser.add_option("--skip-validate", action="store_true", dest="skipValidate",
                          default=False,
                          help="Skip input data validation.")
        parser.add_option("--dry-run", action="store_true", dest="dryRun",
                          default=False,
                          help="Just print the image/ortho/dem download commands.")
        parser.add_option("--refetch-index", action="store_true", dest="refetchIndex",
                          default=False,
                          help="Force refetch of the index file.")
        parser.add_option("--type",  dest="type", default='image',
                          help="File type to download ([image], ortho, dem, lidar)")
        parser.add_option('--max-num-to-fetch', dest='maxNumToFetch', default=-1,
                          type='int', help='The maximum number to fetch of each kind of file. ' + \
                          'This is used in debugging.')

        # This call handles all the parallel_mapproject specific options.
        (options, args) = parser.parse_args(argsIn)

        # Some of the code does not work unless a specific lidar type is used
        if options.type == 'lidar':
            options.type = LIDAR_TYPES[0]
            
        if len(args) != 1:
            logger.info('Error: Missing output folder.\n' + usage)
            return -1
        outputFolder = os.path.abspath(args[0])

        # Handle unified date option
        options.ext = '' # some dirs have appended to them 'a' or 'b'
        if options.yyyymmdd:
            options.year  = int(options.yyyymmdd[0:4])
            options.month = int(options.yyyymmdd[4:6])
            options.day   = int(options.yyyymmdd[6:8])
            if len(options.yyyymmdd) == 9:
                options.ext = options.yyyymmdd[8]

        if not options.stopFrame:
            options.stopFrame = options.startFrame
        
        # Error checking
        if (not options.year) or (not options.month) or (not options.day):
            logger.error('Error: year, month, and day must be provided.\n' + usage)
            return -1
        
        # Ortho and DEM files don't need this information to find them.
        if (options.type == 'image') and not (options.site == 'AN' or options.site == 'GR'):
            logger.error('Error, site must be AN or GR for images.\n' + usage)
            return -1

        KNOWN_TYPES = ['image', 'ortho', 'dem'] + LIDAR_TYPES
        if not (options.type.lower() in KNOWN_TYPES):
            logger.error('Error, type must be image, ortho, dem, or a lidar type.\n' + usage)
            return -1

    except optparse.OptionError, msg:
        raise Exception(msg)

    # Make several attempts. Stop if there is no progress.
    numPrevFailed = -1
    numFailed = -1
    for attempt in range(10):
        numFailed = doFetch(options, outputFolder)
        
        if numFailed == 0:
            return 0      # Success

        if numFailed == numPrevFailed:
            logger.info("No progress in attempt %d" % (attempt+1))
            return -1

        # Try again
        logger.info("Failed to fetch all in attempt %d, will try again.\n" % (attempt+1))
        numPrevFailed = numFailed

    return -1 # We should not come all the way to here

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


