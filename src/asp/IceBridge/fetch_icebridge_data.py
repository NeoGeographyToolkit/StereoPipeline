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

logging.info('DEBUG')
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
LIDAR_TYPES = ['atm1', 'atm2', 'lvis']
MAX_IN_ONE_CALL = 100 # when fetching in batches

def checkIfUrlExists(url):
    '''Return true if the given IceBrige folder URL is valid'''
    p    = urlparse(url)
    conn = httplib.HTTPConnection(p.netloc)
    conn.request('HEAD', p.path)
    resp = conn.getresponse()
    # Invalid pages return 404, valid pages should return one of the numbers below.
    return (resp.status == 403) or (resp.status == 301)

def makeYearFolder(year, site):
    '''Generate part of the URL.  Only used for images.'''
    return str(year) + '_' + site + '_NASA'

def makeDateFolder(year, month, day, ext, fileType):
    '''Generate part of the URL.'''

    if fileType == 'jpeg':
        datePart = ('%02d%02d%04d%s') % (month, day, year, ext)
        return datePart +'_raw'
    else: # Used for all other cases
        datePart = ('%04d.%02d.%02d%s') % (year, month, day, ext)
        return datePart

def hasGoodLat(latitude, isSouth):
    '''Return true if latitude and isSouth parameters match.'''
    if (isSouth and latitude < 0) or ( (not isSouth) and latitude > 0 ):
        return True
    return False

def fetchAndParseIndexFileAux(isSouth, separateByLat, dayVal,
                              baseCurlCmd, folderUrl, path, fileType):
    '''Retrieve the index file for a folder of data and create
    a parsed version of it that contains frame number / filename pairs.'''

    # Download the html file
    curlCmd = baseCurlCmd + ' ' + folderUrl + ' > ' + path
    logger.info(curlCmd)
    p = subprocess.Popen(curlCmd, shell=True)
    os.waitpid(p.pid, 0)

    # Find all the file names in the index file and
    #  dump them to a new index file
    logger.info('Extracting file name list from index.html file...')
    with open(path, 'r') as f:
        indexText = f.read()

    # Must wipe this html file. We fetch it too often in different
    # contexts.  If not wiped, the code fails to work in some
    # very rare but real situations.
    if os.path.exists(path):
        os.remove(path)
    
    # Extract just the file names
    fileList = [] # ensure initialization
    if fileType == 'jpeg':
        fileList = re.findall(">[0-9_]*.JPG", indexText, re.IGNORECASE)
    if fileType == 'ortho':
        fileList = re.findall(">DMS\w*.tif<", indexText, re.IGNORECASE) 
    if fileType == 'fireball':
        # Fireball DEMs
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

    # Get rid of '>' and '<'
    for fileIter in range(len(fileList)):
        fileList[fileIter] = fileList[fileIter].replace(">", "")
        fileList[fileIter] = fileList[fileIter].replace("<", "")

    # Some runs, eg, https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODMS1B.001/2015.09.24
    # have files for both GR and AN, with same frame number. Those need to be separated
    # by latitude. This is a problem only with orthoimages.
    badXmls = set()
    outputFolder = os.path.dirname(path)
    if separateByLat:
        allFilesToFetch = []
        allUrlsToFetch  = []
        for filename in fileList:
            xmlFile  = icebridge_common.xmlFile(filename)
            url      = os.path.join(folderUrl, xmlFile)
            outputPath = os.path.join(outputFolder, xmlFile)
            allFilesToFetch.append(outputPath)
            allUrlsToFetch.append(url)
            
        dryRun = False
        icebridge_common.fetchFilesInBatches(baseCurlCmd, MAX_IN_ONE_CALL,
                                             dryRun, outputFolder,
                                             allFilesToFetch, allUrlsToFetch,
                                             logger)

        # Mark the bad ones
        for xmlFile in allFilesToFetch:
            latitude = icebridge_common.parseLatitude(xmlFile)
            isGood = hasGoodLat(latitude, isSouth)
            if not isGood:
                badXmls.add(xmlFile)
                
    elif (fileType == 'ortho' or fileType == 'fireball'):
        # Sometimes there is a large gap in the timestamp. That means orthoimages 
        # from previous day are spilling over. If dayVal is 0, we must ignore
        # the spillover images. If dayVal is 1, we must keep the spillover images
        # and igore the others.
        list1 = []
        list2 = []
        isBigGap = False
        prevStamp = -1
        for filename in fileList:
            [imageDateString, imageTimeString] = icebridge_common.parseTimeStamps(filename)
            currStamp = float(imageTimeString)/1000000.0 # hours
            if prevStamp < 0:
                list1.append(filename)
                prevStamp = currStamp
                continue
            
            # Note that once isBigGap becomes true, it stays true
            # even when the gap gets small again
            if currStamp - prevStamp >= 6: # six hour gap is a lot
                isBigGap = True
            if not isBigGap:
                list1.append(filename)
            else:
                list2.append(filename)

            prevStamp = currStamp # for next iteration

        if isBigGap:
           if dayVal == 0:
               fileList = list2[:] # current day
           else:
               fileList = list1[:] # spillover from prev day

    # For each entry that matched the regex, record: the frame number and the file name.
    frameDict = {}
    urlDict   = {}
    badFiles = []
    for filename in fileList:

        if len(badXmls) > 0:
            xmlFile  = os.path.join(outputFolder, icebridge_common.xmlFile(filename))
            if xmlFile in badXmls:
                continue
            
        frame = icebridge_common.getFrameNumberFromFilename(filename)
        if frame in frameDict.keys():
            
            # The same frame must not occur twice.
            if fileType not in LIDAR_TYPES:
                logger.error("Error: Found two file names with same frame number: " + \
                             frameDict[frame] + " and " + filename)
                badFiles.append(filename)
                badFiles.append(frameDict[frame])
                            
        # note that folderUrl can vary among orthoimages, as sometimes
        # some of them are in a folder for the next day.
        frameDict[frame] = filename
        urlDict[frame]   = folderUrl
        
    # Wipe them all, to be sorted later
    for badFile in badFiles:
        if os.path.exists(badFile):
            logger.info("Deleting: " + badFile)
            os.remove(badFile)
        xmlFile  = icebridge_common.xmlFile(badFile)
        if os.path.exists(xmlFile):
            logger.info("Deleting: " + xmlFile)
            os.remove(xmlFile)

    if len(badFiles) > 0:
        raise Exception("Found files with same frame number")
    
    return (frameDict, urlDict)

# These exist both in AN and GR, all mixed up, and have to separate by lat
def isInSeparateByLatTable(yyyymmdd):
    ''''''
    return yyyymmdd in ['20150923', '20150924', '20151004', '20151005',
                        '20151019', '20151020', '20151021', '20151022'];
    
def twoFlightsInOneDay(site, yyyymmdd):
    '''Return true if there are two flights in one day.'''
    
    # For this day, there are GR_20100422a and GR_20100422b
    if site == 'GR' and yyyymmdd == '20100422':
        return True
    return False

def getFolderUrl(yyyymmdd, year, month, day,
                 dayInc, # if to add one to the day
                 site, fileType):
    '''Get full URL to the location where the files are kept.'''

    # Note that yyyymmdd can equal 20100422a.
    ext = ''
    if len(yyyymmdd) == 9:
        ext = yyyymmdd[8]

    if fileType == 'nav':
        # This is the simplest, usually one file per flight.
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE_FTP/IPAPP1B_GPSInsCorrected_v01'
        yearFolder = makeYearFolder(year, site)
        folderUrl  = os.path.join(base, yearFolder)
        return folderUrl

    if fileType == 'jpeg':

        # If yyyymmdd is 20100422, put a or b depending on dayVal
        if twoFlightsInOneDay(site, yyyymmdd):
            if dayInc == 0:
                ext = 'a'
            else:
                ext = 'b'
            dayInc = 0
            
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE_FTP/IODMS0_DMSraw_v01'
        yearFolder = makeYearFolder(year, site)
        dateFolder = makeDateFolder(year, month, day + dayInc, ext, fileType)
        folderUrl  = os.path.join(base, yearFolder, dateFolder)
        return folderUrl
    
    # The other types share more formatting
    if twoFlightsInOneDay(site, yyyymmdd):
        dayInc = 0 # for this particular day, one should not look at the next day
        
    if fileType == 'ortho':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODMS1B.001'       
    elif fileType == 'fireball':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODMS3.001'
    elif fileType == 'atm1':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/ILATM1B.001/'
    elif fileType == 'atm2':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/ILATM1B.002/'
    elif fileType == 'lvis':
        base = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/ILVIS2.001/'
    else:
        raise("Unknown type: " + fileType)
    
    dateFolder = makeDateFolder(year, month, day + dayInc, ext, fileType)
    folderUrl  = os.path.join(base, dateFolder)
    
    return folderUrl


def fetchAndParseIndexFile(options, isSouth, baseCurlCmd, outputFolder):
    '''Create a list of all files that must be fetched unless done already.'''

    # For AN 20091112, etc, some of the ortho images are stored at the
    # beginning of the next day's flight. Need to sort this out, and
    # it is tricky. More comments within the code. 
    fetchNextDay = True
    
    separateByLat = (options.type == 'ortho' and isInSeparateByLatTable(options.yyyymmdd))
    if separateByLat:
        # Here we won't fetch the next day, we will just separate by latitude within
        # a given day
        fetchNextDay = False

    orthoOrFireball = ( (options.type == 'ortho') or (options.type == 'fireball') )

    if fetchNextDay:
        # Normally we fetch for next day only for ortho or fireball. However,
        # for one single special flight, we do it for jpeg too, as then
        # the jpegs are also split. 
       if orthoOrFireball or \
          ((options.type == 'jpeg') and twoFlightsInOneDay(options.site, options.yyyymmdd)):
           fetchNextDay = True
       else:
           fetchNextDay = False
           
    # If we need to parse the next flight day as well, as expected in some runs,
    # we will fetch two html files, but create a single index out of them.
    dayVals = [0]
    if fetchNextDay:
        dayVals.append(1)

    indexPath       = icebridge_common.htmlIndexFile(outputFolder)
    
    currIndexPath   = indexPath
    parsedIndexPath = icebridge_common.csvIndexFile(outputFolder)

    if options.refetchIndex:
        os.system('rm -f ' + indexPath)
        os.system('rm -f ' + parsedIndexPath)

    if icebridge_common.fileNonEmpty(parsedIndexPath):
        logger.info('Already have the index file ' + parsedIndexPath + ', keeping it.')
        return parsedIndexPath
    
    frameDict  = {}
    urlDict    = {}

    # We need the list of jpeg frames. Sometimes when fetching ortho images,
    # and we have to fetch from the next day, don't fetch unless
    # in the jpeg index.
    if len(dayVals) > 1 and options.type != 'jpeg':
        jpegFolder = icebridge_common.getJpegFolder(os.path.dirname(outputFolder))
        jpegIndexPath = icebridge_common.csvIndexFile(jpegFolder)
        (jpegFrameDict, jpegUrlDict) = icebridge_common.readIndexFile(jpegIndexPath)

    orthoStamp = {}
    if options.type == 'fireball':
        # This is a bugfix. Ensure that the fireball DEM has not just
        # the same frame number, but also same timestamp as the ortho.
        orthoFolder = icebridge_common.getOrthoFolder(os.path.dirname(outputFolder))
        orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
        (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath)
        for frame in sorted(orthoFrameDict.keys()):
            filename = orthoFrameDict[frame]
            [imageDateString, imageTimeString] = icebridge_common.parseTimeStamps(filename)
            orthoStamp[frame] = imageTimeString
            
    for dayVal in dayVals:

        if len(dayVals) > 1:
            currIndexPath = indexPath + '.day' + str(dayVal)
            if options.refetchIndex:
                os.system('rm -f ' + currIndexPath)
            
        # Find folderUrl which contains all of the files
        if options.type in LIDAR_TYPES:
            options.allFrames = True # For lidar, always get all the frames!
            
            # For lidar, the data can come from one of three sources.
            # Unfortunately sometimes there is more than one source, and then
            # we need to pick by latitude.
            folderUrls = []
            lidar_types = []
            for lidar in LIDAR_TYPES:
                folderUrl = getFolderUrl(options.yyyymmdd, options.year, options.month,
                                         options.day, dayVal, # note here the dayVal
                                         options.site, lidar)
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
                for folderUrl in folderUrls:
                    count += 1
                    (localFrameDict, localUrlDict) = \
                                     fetchAndParseIndexFileAux(isSouth,
                                                               separateByLat, dayVal,
                                                               baseCurlCmd, folderUrl,
                                                               currIndexPath,
                                                               lidar_types[count])
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

                        if hasGoodLat(latitude, isSouth):
                            isGood = True
                            options.type = lidar_types[count]
                            logger.info("Good latitude " + str(latitude) + ", will use " +
                                        folderUrl + " of type " + lidar_types[count])
                        else:
                            logger.info("Bad latitude " + str(latitude) + ", will not use " +
                                        folderUrl + " of type " + lidar_types[count])
                            
                        # Stop at first file no matter what
                        break

                    if isGood:
                        break

                if not isGood:
                    if options.type in LIDAR_TYPES and options.ignoreMissingLidar:
                        logger.info("No lidar. None of these URLs are good: " +
                                    " ".join(folderUrls))
                    else:
                        raise Exception("None of these URLs are good: " +
                                        " ".join(folderUrls))
                    
        else: # Other cases are simpler
            folderUrl = getFolderUrl(options.yyyymmdd, options.year, options.month,
                                     options.day, dayVal, # note here the dayVal
                                     options.site, options.type)

        logger.info('Fetching from URL: ' + folderUrl)
        (localFrameDict, localUrlDict) = \
                         fetchAndParseIndexFileAux(isSouth,
                                                   separateByLat, dayVal,
                                                   baseCurlCmd, folderUrl,
                                                   currIndexPath, options.type)

        # Append to the main index
        for frame in sorted(localFrameDict.keys()):

            if options.type == 'fireball':
                # This is a bugfix. Ensure that the fireball DEM has not just
                # the same frame number, but also same timestamp as the ortho.
                # Otherwise we may accidentally getting one from next day.
                [imageDateString, imageTimeString] = \
                                  icebridge_common.parseTimeStamps(localFrameDict[frame])
                if frame not in orthoStamp:
                    #logger.info("Missing ortho for fireball: " + localFrameDict[frame])
                    continue
                if abs(int(imageTimeString) - int(orthoStamp[frame])) > 1000:
                    # Apparently a tolerance is needed. Use 10 seconds, so the number 1000.
                    #logger.info("Will not use fireball DEM whose timestamp differs from ortho.")
                    #logger.info("Fireball is: " + localFrameDict[frame])
                    #logger.info("Ortho is:    " + orthoFrameDict[frame])
                    continue
                
            # Fetch from next day, unless already have a value. And don't fetch
            # frames not in the jpeg index.
            if len(dayVals) > 1 and options.type != 'jpeg':
                if not frame in jpegFrameDict.keys(): continue
                if frame in frameDict.keys(): continue
                
            frameDict[frame] = localFrameDict[frame]
            urlDict[frame]   = localUrlDict[frame]
        
    # Write the combined index file
    icebridge_common.writeIndexFile(parsedIndexPath, frameDict, urlDict)
            
    return parsedIndexPath

def lidarFilesInRange(lidarDict, lidarFolder, startFrame, stopFrame):
    '''Fetch only lidar files for the given frame range. Do that as follows.   '''
    '''For each ortho frame in [startFrame, stopFrame], find the lidar         '''
    '''file with the closest timestamp. Collect them all.                      '''
    '''Add the two neighboring ones, to help with finding lidar pairs later.   '''
    
    lidarList = []
    for frame in sorted(lidarDict.keys()):
        lidarList.append(lidarDict[frame])

    orthoFolder = icebridge_common.getOrthoFolder(os.path.dirname(lidarFolder))
    orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
    (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath)
    minLidarIndex = len(lidarList)
    maxLidarIndex = 0
    for frame in sorted(orthoFrameDict.keys()):
        if ((frame < startFrame) or (frame > stopFrame) ): continue
        orthoFrame = orthoFrameDict[frame]
        matchingLidar = icebridge_common.findMatchingLidarFileFromList(orthoFrame, lidarList)

        for index in range(len(lidarList)):
            if lidarList[index] == matchingLidar:
                if minLidarIndex > index:
                    minLidarIndex = index
                if maxLidarIndex < index:
                    maxLidarIndex = index

    # We will fetch neighboring lidar files as well
    if minLidarIndex > 0:
        minLidarIndex = minLidarIndex -1
    if maxLidarIndex + 1 < len(lidarList):
        maxLidarIndex = maxLidarIndex + 1

    lidarsToFetch = set()
    for index in range(minLidarIndex, maxLidarIndex+1):
        lidarsToFetch.add(lidarList[index])

    return lidarsToFetch


def fetchNavData(options, outputFolder):
    '''Fetch all the nav data for a flight.'''

    numFailed = 0

    # The storage convention for these is very easy!
    # - A few dates have two files instead of one.
    folderUrl = getFolderUrl(options.yyyymmdd, options.year, options.month,
                             options.day, False,
                             options.site, options.type)
    filename  = 'sbet_' + options.yyyymmdd + '.out'
    filenameA = 'sbet_' + options.yyyymmdd + 'a.out'
    filenameB = 'sbet_' + options.yyyymmdd + 'b.out'
    
    # Check which urls are accurate for this file
    url = folderUrl + filename
    if checkIfUrlExists(url):
        fileList = [filename]
    else:
        fileList = [filenameA, filenameB]

    # Download the files    
    for f in fileList:
        url        = os.path.join(folderUrl, f)
        outputPath = os.path.join(outputFolder, f)
        # TODO: How to handle refetch?
        if os.path.exists(outputPath):
            continue
        if not icebridge_common.fetchFile(url, outputPath):
            numFailed = numFailed + 1

    return numFailed

def doFetch(options, outputFolder):
    '''The main fetch function.
       Returns the number of failures.'''
    
    # Verify that required files exist
    home = os.path.expanduser("~")
    if not (os.path.exists(home+'/.netrc') and os.path.exists(home+'/.urs_cookies')):
        logger.error('Missing a required authentication file!  See instructions here:\n' +
                     '    https://nsidc.org/support/faq/what-options-are-available-bulk-' +
                     'downloading-data-https-earthdata-login-enabled')
        return -1
    
    curlPath = asp_system_utils.which("curl")
    curlOpts    = ' -n -L '
    cookiePaths = ' -b ~/.urs_cookies -c ~/.urs_cookies '
    baseCurlCmd = curlPath + curlOpts + cookiePaths

    logger.info('Creating output folder: ' + outputFolder)
    os.system('mkdir -p ' + outputFolder)  

    isSouth = (options.site == 'AN')
    
    if options.type == 'nav': # Nav fetching is much less complicated
        return fetchNavData(options, outputFolder)
    
    parsedIndexPath = fetchAndParseIndexFile(options, isSouth, baseCurlCmd, outputFolder)
    if not icebridge_common.fileNonEmpty(parsedIndexPath):
        # Some dirs are weird, both images, fireball dems, and ortho.
        # Just accept whatever there is, but with a warning.
        logger.info('Warning: Missing index file: ' + parsedIndexPath)

    # Store file information in a dictionary
    # - Keep track of the earliest and latest frame
    logger.info('Reading file list from ' + parsedIndexPath)
    try:
        (frameDict, urlDict) = icebridge_common.readIndexFile(parsedIndexPath)
    except:
        # We probably ran into old format index file. Must refetch.
        logger.info('Could not read index file. Try again.')
        options.refetchIndex = True
        parsedIndexPath = fetchAndParseIndexFile(options, isSouth, baseCurlCmd, outputFolder)
        (frameDict, urlDict) = icebridge_common.readIndexFile(parsedIndexPath)

    if options.stopAfterIndexFetch:
        return 0
    
    isLidar = (options.type in LIDAR_TYPES)

    allFrames  = sorted(frameDict.keys())
    
    if not isLidar:
        # The lidar frames use a totally different numbering than the image/ortho/dem frames
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

    if isLidar:
        # Based on image frames, determine which lidar frames to fetch.
        if options.ignoreMissingLidar and len(frameDict.keys()) == 0:
            # Nothing we can do if this run has no lidar and we are told to continue
            logger.info("Warning: missing lidar, but continuing.")
            lidarsToFetch = set()
        else:
            lidarsToFetch = lidarFilesInRange(frameDict, outputFolder,
                                              options.startFrame, options.stopFrame)
        
    # There is always a chance that not all requested frames are available.
    # That is particularly true for Fireball DEMs. Instead of failing,
    # just download what is present and give a warning. 
    if options.startFrame not in frameDict and not isLidar:
        logger.info("Warning: Frame " + str(options.startFrame) +
                    " is not found in this flight.")
                    
    if options.stopFrame and (options.stopFrame not in frameDict) and not isLidar:
        logger.info("Warning: Frame " + str(options.stopFrame) +
                    " is not found in this flight.")

    allFilesToFetch = [] # Files that we will fetch, relative to the current dir. 
    allUrlsToFetch  = [] # Full url of each file.
    
    # Loop through all found frames within the provided range
    currentFileCount = 0
    lastFrame = ""
    if len(allFrames) > 0:
        lastFrame = allFrames[len(allFrames)-1]

    hasTfw = (options.type == 'fireball')
    hasXml = ( isLidar or (options.type == 'ortho') or hasTfw )
    numFetched = 0
    skipCount  = 0
    for frame in allFrames:

        # Skip frame outside of range
        if isLidar:
            if frameDict[frame] not in lidarsToFetch:
                continue
        else:       
            if ((frame < options.startFrame) or (frame > options.stopFrame) ):
                continue
                
        # Handle the frame skip option
        if options.frameSkip > 0: 
            if skipCount < options.frameSkip:
                skipCount += 1
                continue
            skipCount = 0

        filename = frameDict[frame]
        
        # Some files have an associated xml file. Fireball DEMs also have a tfw file.
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

    # Restrict lidar fetch amount according to the parameter
    if (isLidar and options.maxNumLidarToFetch > 0 and 
           len(allFilesToFetch) > options.maxNumLidarToFetch):

        # Ensure an even number, to fetch both the lidar file and its xml
        if options.maxNumLidarToFetch % 2 == 1:
            options.maxNumLidarToFetch += 1
        
        allFilesToFetch = allFilesToFetch[0:options.maxNumLidarToFetch]
        allUrlsToFetch  = allUrlsToFetch [0:options.maxNumLidarToFetch]
                
    icebridge_common.fetchFilesInBatches(baseCurlCmd, MAX_IN_ONE_CALL, options.dryRun,
                                         outputFolder,
                                         allFilesToFetch, allUrlsToFetch, logger)

    # Fetch from disk the set of already validated files, if any
    validFilesList = icebridge_common.validFilesList(os.path.dirname(outputFolder),
                                                     options.startFrame, options.stopFrame)
    validFilesSet = set()
    validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
    numInitialValidFiles = len(validFilesSet)
    
    # Verify that all files were fetched and are in good shape
    failedFiles = []
    for outputPath in allFilesToFetch:

        if options.skipValidate:
            continue
        
        if not icebridge_common.fileNonEmpty(outputPath):
            logger.info('Missing file: ' + outputPath)
            failedFiles.append(outputPath)
            continue

        if icebridge_common.hasImageExtension(outputPath):
            if False:
                # This check is just so slow. Turn it off for now.
                # This will impact only the validation of jpegs,
                # as the other files can be validated via the checksum.
                # Jpegs will be validated when converting them to 1 band images
                if outputPath in validFilesSet and os.path.exists(outputPath):
                    #logger.info('Previously validated: ' + outputPath)   # verbose
                    continue
                else:
                    if not icebridge_common.isValidImage(outputPath):
                        logger.info('Found an invalid image. Will wipe it: ' + outputPath)
                        if os.path.exists(outputPath): os.remove(outputPath)
                        failedFiles.append(outputPath)
                        continue
                    else:
                        logger.info('Valid image: ' + outputPath)
                        validFilesSet.add(outputPath) # mark it as validated

        # Sanity check: XML files must have the right latitude.
        if icebridge_common.fileExtension(outputPath) == '.xml':
            if outputPath in validFilesSet and os.path.exists(outputPath):
                #logger.info('Previously validated: ' + outputPath) #verbose
                continue
            else:
                if os.path.exists(outputPath):
                    try:
                        latitude = icebridge_common.parseLatitude(outputPath)
                        logger.info('Valid file: ' + outputPath)
                        validFilesSet.add(outputPath) # mark it as validated
                    except:
                        # Corrupted file
                        logger.info("Failed to parse latitude, will wipe: " + outputPath)
                        if os.path.exists(outputPath): os.remove(outputPath)
                        failedFiles.append(outputPath)

                    # On a second thought, don't wipe files with wrong latitude, as
                    # next time we run fetch we will have to fetch them again.
                    # Hopefully they will be ignored.
                    #isGood = hasGoodLat(latitude, isSouth)
                    #if not isGood:
                    #    logger.info("Wiping XML file " + outputPath + " with bad latitude " + \
                    #                str(latitude))
                    #    os.remove(outputPath)
                    #    imageFile = icebridge_common.xmlToImage(outputPath)
                    #    if os.path.exists(imageFile):
                    #        logger.info("Wiping TIF file " + imageFile + " with bad latitude " + \
                    #                    str(latitude))
                    #        os.remove(imageFile)
                    
        # Verify the chcksum    
        if hasXml and len(outputPath) >= 4 and outputPath[-4:] != '.xml' \
               and outputPath[-4:] != '.tfw':
            if outputPath in validFilesSet and os.path.exists(outputPath):
                #logger.info('Previously validated: ' + outputPath) # verbose
                continue
            else:
                isGood = icebridge_common.hasValidChkSum(outputPath, logger)
                if not isGood:
                    xmlFile = icebridge_common.xmlFile(outputPath)
                    logger.info('Found invalid data. Will wipe: ' + outputPath + ' ' + xmlFile)
                    if os.path.exists(outputPath): os.remove(outputPath)
                    if os.path.exists(xmlFile):    os.remove(xmlFile)
                    failedFiles.append(outputPath)
                    failedFiles.append(xmlFile)
                    continue
                else:
                    logger.info('Valid file: ' + outputPath)
                    validFilesSet.add(outputPath)

        if hasTfw and icebridge_common.fileExtension(outputPath) == '.tfw':
            if outputPath in validFilesSet and os.path.exists(outputPath):
                #logger.info('Previously validated: ' + outputPath)
                continue
            else:
                isGood = icebridge_common.isValidTfw(outputPath, logger)
                if not isGood:
                    xmlFile = icebridge_common.xmlFile(outputPath)
                    logger.info('Found invalid tfw. Will wipe: ' + outputPath + ' ' + xmlFile)
                    if os.path.exists(outputPath): os.remove(outputPath)
                    if os.path.exists(xmlFile):    os.remove(xmlFile)
                    failedFiles.append(outputPath)
                    failedFiles.append(xmlFile)
                    continue
                else:
                    logger.info('Valid tfw file: ' + outputPath)
                    validFilesSet.add(outputPath)

    # Write to disk the list of validated files, but only if new
    # validations happened.  First re-read that list, in case a
    # different process modified it in the meantime, such as if two
    # managers are running at the same time.
    numFinalValidFiles = len(validFilesSet)
    if numInitialValidFiles != numFinalValidFiles:
        validFilesSet = \
                      icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
        icebridge_common.writeValidFilesList(validFilesList, validFilesSet)

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
        parser.add_option("--start-frame",  dest="startFrame", type='int', 
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame number or start of frame sequence")
        parser.add_option("--stop-frame",  dest="stopFrame", type='int', 
                          default=icebridge_common.getLargestFrame(),
                          help="End of frame sequence to download.")
        parser.add_option("--all-frames", action="store_true", dest="allFrames", 
                          default=False,
                          help="Fetch all frames for this flight.")
        parser.add_option("--skip-validate", action="store_true", dest="skipValidate",
                          default=False,
                          help="Skip input data validation.")
        parser.add_option("--ignore-missing-lidar", action="store_true", dest="ignoreMissingLidar",
                          default=False,
                          help="Keep going if the lidar is missing.")
        parser.add_option("--frame-skip",  dest="frameSkip", type='int', default=0,
                          help="Skip this many frames between downloads.")
        parser.add_option("--dry-run", action="store_true", dest="dryRun",
                          default=False,
                          help="Just print the image/ortho/fireball download commands.")
        parser.add_option("--refetch-index", action="store_true", dest="refetchIndex",
                          default=False,
                          help="Force refetch of the index file.")
        parser.add_option("--stop-after-index-fetch", action="store_true",
                          dest="stopAfterIndexFetch", default=False,
                          help="Stop after fetching the indices.")
        parser.add_option('--max-num-lidar-to-fetch', dest='maxNumLidarToFetch', default=-1,
                          type='int', help='The maximum number of lidar files to fetch. ' + \
                          'This is used in debugging.')

        # This call handles all the parallel_mapproject specific options.
        (options, args) = parser.parse_args(argsIn)

        if len(args) != 1:
            logger.info('Error: Missing output folder.\n' + usage)
            return -1
        outputFolder = os.path.abspath(args[0])

        # TODO: Restore "type" input parameter so that outside users who do not use
        #       our folder convention can use this tool.
        options.type = icebridge_common.folderToType(outputFolder)
        if options.type == 'lidar':
            options.type = LIDAR_TYPES[0]
        print 'Detected type: ' + options.type
            
        # Handle unified date option
        if options.yyyymmdd:
            options.year  = int(options.yyyymmdd[0:4])
            options.month = int(options.yyyymmdd[4:6])
            options.day   = int(options.yyyymmdd[6:8])

        if not options.stopFrame:
            options.stopFrame = options.startFrame
        
        # Error checking
        if (not options.year) or (not options.month) or (not options.day):
            logger.error('Error: year, month, and day must be provided.\n' + usage)
            return -1
        
        # Ortho and Fireball DEM files don't need this information to find them.
        if (options.type == 'jpeg') and not (options.site == 'AN' or options.site == 'GR'):
            logger.error('Error, site must be AN or GR for images.\n' + usage)
            return -1

        KNOWN_TYPES = ['jpeg', 'ortho', 'fireball', 'nav'] + LIDAR_TYPES
        if not (options.type.lower() in KNOWN_TYPES):
            logger.error('Error, type must be image, ortho, fireball, or a lidar type.\n' + usage)
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


