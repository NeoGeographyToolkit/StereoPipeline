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

# Class to help manage a run stored on disk

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing
import re, shutil, time, getpass, glob

import os.path as P

# The path to the ASP python files and tools
basepath      = os.path.dirname(os.path.realpath(__file__)) # won't change, unlike syspath
pythonpath    = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath   = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools

# Prepend to Python path
sys.path.insert(0, basepath)
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)

import icebridge_common, asp_file_utils

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]


class RunHelper():
    '''Class for managing a folder for processing one run (flight)'''

    def __init__(self, site, yyyymmdd, parentFolder=''):
        '''Constructor.
           If optional parent folder is provided, it will be prependend to all returned paths.'''
        self.site          = site
        self.yyyymmdd      = yyyymmdd
        self.parentFolder = parentFolder
        
    def __str__(self):
        '''Define string autoconversion'''
        return self.site +'_'+ self.yyyymmdd

    def name(self):
        '''Return a name in SS_YYYYMMDD format'''
        return str(self)

    def shortName(self):
        '''Return a name in SSYYMMDD format'''
        return self.site + self.yyyymmdd[2:]

    # TODO: Add version numbers to these in a way that is easily handled!
    def getInputTarName(self):
        '''Return the file name used to tar up the downloaded input data'''
        return self.name() + '.tar'

    def getCameraTarName(self):
        '''Return the file name used to tar up the generated camera files'''
        return 'CAMERA_' + self.name() + '.tar'

    def getAlignedCameraTarName(self):
        '''Return the file name used to tar up the generated bundle-adjustd
        and aligned camera files'''
        return 'ALIGNED_CAMERA_' + self.name() + '.tar'

    def getOrthoTarName(self):
        '''Return the file name used to tar up the generated ortho images.'''
        return 'ORTHO_' + self.name() + '.tar'

    def getSummaryTarName(self):
        '''Return the file name used to tar up the generated camera files'''
        return 'SUMMARY_' + self.name() + '.tar'

    def getOutputTarName(self):
        '''Return the file name used to tar up the final results'''
        return 'DEM_' + self.name() + '_V1.tar'

    def getLabelTarName(self):
        '''Return the file name used to tar up the label files'''
        return 'LABELS_' + self.name() + '.tar'

    def getFolder(self):
        '''Returns the folder where this run will be stored'''
        return os.path.join(self.parentFolder, str(self))

    def _internalLoc(self, path):
        '''Returns a folder relative to getFolder()'''
        return os.path.join(self.getFolder(), path)

    # TODO: Merge these with corresponding functions in icebridge_common.
    def getJpegFolder(self):
        return self._internalLoc('jpeg')
    def getImageFolder(self):
        return self._internalLoc('image')
    def getLidarFolder(self):
        return self._internalLoc('lidar')
    def getLidarPairFolder(self):
        return icebridge_common.getPairedLidarFolder(self.getLidarFolder())
    def getNavFolder(self):
        return self._internalLoc('nav')
    def getNavCameraFolder(self):
        return self._internalLoc('nav_camera')
    def getCameraFolder(self):
        return self._internalLoc('camera')
    def getOrthoFolder(self):
        return self._internalLoc('ortho')
    def getProcessFolder(self):
        return self._internalLoc('processed')
    def getAssemblyFolder(self):
        return self._internalLoc('tarAssembly')
    def getPbsLogFolder(self):
        return self._internalLoc('pbsLog')
    def getSummaryFolder(self):
        return self._internalLoc('summary')
    def getLabelFolder(self):
        return self._internalLoc('labeled')

    def getJpegList(self, prependFolder=False):
        '''Return a list containing all the currently stored jpeg files'''
        jpegFolder = self.getJpegFolder()
        jpegs = icebridge_common.getJpegs(jpegFolder)
        if prependFolder:
            jpegs = [os.path.join(jpegFolder, x) for x in jpegs]
        jpegs.sort()
        return jpegs

    def getImageList(self, prependFolder=False):
        '''Return a list containing all the currently stored image files'''
        imageFolder = self.getImageFolder()
        images = icebridge_common.getTifs(imageFolder)
        if prependFolder:
            images = [os.path.join(imageFolder, x) for x in images]
        images.sort()
        return images

    def getLidarList(self, paired=False, prependFolder=False):
        '''Return a list containing all the currently stored lidar files.
           This does not return converted csv files.'''
        lidarFolder = self.getLidarFolder()
        if paired:
            lidarFolder = icebridge_common.getPairedLidarFolder(lidarFolder)
        files = icebridge_common.getLidar(lidarFolder)
        if prependFolder:
            files = [os.path.join(lidarFolder, x) for x in files]
        files.sort()
        return files

    def getBatchFolderList(self):
        '''Return a list of all the batch folders'''
        pFolder       = self.getProcessFolder()
        batchFolders  = os.listdir(pFolder)
        def getFirstFrame(s):
            try:
                parts = s.split('_')
                return int(parts[1])
            except:
                return 99999999
        batchFolders.sort(key=getFirstFrame)
        batchFolders  = [os.path.join(pFolder,x) for x in batchFolders if 'batch_' in x]
        batchFolders  = [x for x in batchFolders if os.path.isdir(x)]
        return batchFolders

    # Get a list of all output files of given type. They may not exist. 
    def getOutputFileList(self, fileType):
        '''Return a list containing all the output DEM files in the run and the associated frames'''
        
        batchList = self.getBatchFolderList()
        output = []
        for batch in batchList:
            frames = icebridge_common.getFrameRangeFromBatchFolder(batch)
            path   = os.path.join(batch, fileType)
            output.append((path, frames))
        return output

    def existingFilesDict(self, fileType, startFrame, stopFrame):
        '''See which output files of given type exist.'''

        fileList = self.getOutputFileList(fileType)
        fileDict = {}
        
        for (filename, frames) in fileList:
            
            # Handle frame range option
            if (frames[0] < startFrame):
                continue
            if (frames[1] > stopFrame):
                break

            if not os.path.exists(filename):
                continue
            fileDict[frames[0]] = filename

        return fileDict
        
    def allSourceDataFetched(self, noNav, verbose=False):
        '''Return true if all the required source data has been downloaded'''
    
        logger = logging.getLogger(__name__)

        # Old file collections use a different naming scheme    
        altIndexName = 'image_index.html.csv'

        # Verify these input folders
        subFolders = [self.getJpegFolder(), self.getLidarFolder(), self.getOrthoFolder()]
        
        for folder in subFolders:
            
            # Make sure all the files specified in the parsed index file are present
            indexFile = icebridge_common.csvIndexFile(folder)
            if not os.path.exists(indexFile):
                # See if the older naming scheme was used
                altPath = os.path.join(folder, altIndexName)
                if os.path.exists(altPath):
                    indexFile = altPath
            
            (fileDict, urlDict) = icebridge_common.readIndexFile(indexFile)
            logger.info("Checking all files listed in: " + indexFile)
            num = len(fileDict.keys())
            count = 0
            for f in fileDict.itervalues():

                # Add the progress, as this operation can be terribly slow
                # when the filesystem is not doing too well, especially on mfe.
                count = count + 1
                if (count - 1) % 1000 == 0:
                    logger.info('Progress: ' + str(count) + '/' + str(num))
                    
                path = os.path.join(folder, f)
                if not os.path.exists(path):
                    logger.error('Missing file ' + path)
                    return False

        if not noNav:
            # Simple nav file check
            navFiles = os.listdir(self.getNavFolder())
            navFiles = [x for x in navFiles if '.out' in navFiles]
            if (len(navFiles) == 0) or (len(navFiles) > 2):
                logger.error('Wrong number of nav files detected!')
                return False
        
        return True # Success!

    def massRenameByGlob(self, startFrame, stopFrame, orthoFrameDict,
                         globStr, logger):
        '''Axuxually function used below.'''
        
        files = glob.glob(globStr)

        for fileName in files:

            # This is rather fragile, try to ignore certain types of files
            if ('_sub' in fileName) or ('pct.tif' in fileName) or ('_hillshade_' in fileName):
                continue
            
            [prefix, dateString, timeString, frameString, suffix] = \
                     icebridge_common.parseParts(fileName)
            if frameString == "":
                logger.info("Could not parse frame and time stamps from: " + fileName)
                continue

            frame = int(frameString)
            if frame < startFrame or frame > stopFrame:
                continue
            if not frame in orthoFrameDict:
                logger.info("Missing ortho for frame: " + frame)
                continue

            [newDateString, newTimeString] = icebridge_common.parseTimeStamps(orthoFrameDict[frame])
            newFile = prefix + icebridge_common.formFilePrefix(newDateString, newTimeString,
                                                               frame) + suffix
            if not os.path.exists(fileName):
                continue
            if fileName == newFile:
                continue
            
            if os.path.exists(newFile):
                logger.info("File exists: " + newFile + ", will wipe " + fileName)
                os.system("rm -f " + fileName)
                continue

            logger.info("Renaming: " + fileName + " to " + newFile)
            os.system("mv -f " + fileName + " " + newFile)
            
    def massRename(self, startFrame, stopFrame, logger):
        '''We changed how the timestamp for images and cameras is computed.
        Make all existing converted images, cameras, nav cameras, and aligned cameras conform.'''

        logger.info("Renaming files with timestamp. This is slow.")
        
        # Need to do a mass rename for incorrect timestamp for:
        # converted images, nav cameras, cameras, and bundle aligned cameras
        outputFolder    = self.getFolder()
        cameraFolder    = icebridge_common.getCameraFolder(outputFolder)
        imageFolder     = icebridge_common.getImageFolder(outputFolder)
        jpegFolder      = icebridge_common.getJpegFolder(outputFolder)
        orthoFolder     = icebridge_common.getOrthoFolder(outputFolder)
        processedFolder = icebridge_common.getProcessedFolder(outputFolder)
        navFolder       = icebridge_common.getNavFolder(outputFolder)
        navCameraFolder = icebridge_common.getNavCameraFolder(outputFolder)
        
        # Need the orthos to get the timestamp
        orthoFolder = icebridge_common.getOrthoFolder(os.path.dirname(jpegFolder))
        orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
        if not os.path.exists(orthoIndexPath):
            raise Exception("Error: Missing ortho index file: " + orthoIndexPath + ".")
        (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath,
                                                                        prependFolder = True)

        logger.info('Renaming camera files...')
        self.massRenameByGlob(startFrame, stopFrame, orthoFrameDict, 
                              os.path.join(cameraFolder, '*DMS*tsai'), logger)

        logger.info('Renaming nav camera files...')
        self.massRenameByGlob(startFrame, stopFrame, orthoFrameDict, 
                              os.path.join(navCameraFolder, '*DMS*tsai'), logger)

        logger.info('Renaming converted images...')
        self.massRenameByGlob(startFrame, stopFrame, orthoFrameDict, 
                              os.path.join(imageFolder, '*DMS*tif'), logger)

        logger.info('Renaming aligned cameras...')
        self.massRenameByGlob(startFrame, stopFrame, orthoFrameDict, 
                              os.path.join(processedFolder, 'batch*',
                                           icebridge_common.alignedBundleStr() + '*DMS*tsai'),
                              logger)
        
    def checkForImages(self, startFrame, stopFrame, logger):
        '''Return true if all the images have been converted from jpeg.'''

        logger.info("Checking if all jpegs have been converted.")
        
        jpegFolder = self.getJpegFolder()
        imageFolder = self.getImageFolder()
        orthoFolder = self.getOrthoFolder()
        
        if not os.path.exists(jpegFolder):
            logger.info("Missing: " + jpegFolder)
            return False
        if not os.path.exists(imageFolder):
            logger.info("Missing: " + imageFolder)
            return False

        jpegIndexPath = icebridge_common.csvIndexFile(jpegFolder)
        if not os.path.exists(jpegIndexPath):
            logger.info("Missing: " + jpegIndexPath)
            return False
        (jpegFrameDict, jpegUrlDict) = icebridge_common.readIndexFile(jpegIndexPath,
                                                                  prependFolder = True)

        # Need the orthos to get the timestamp
        orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
        if not os.path.exists(orthoIndexPath):
            raise Exception("Error: Missing ortho index file: " + orthoIndexPath + ".")
        (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath,
                                                                        prependFolder = True)
        
        # Thorough check for missing images. It is very slow.
        num = len(jpegFrameDict.keys())
        allGood = True
        count = 0
        for frame in sorted(jpegFrameDict.keys()):

            if frame < startFrame or frame > stopFrame: continue
            
            # Add the progress, as this operation can be terribly slow
            # when the filesystem is not doing too well, especially on mfe.
            count = count + 1
            if (count - 1) % 1000 == 0:
                logger.info('Progress: ' + str(count) + '/' + str(num))
                
            inputPath = jpegFrameDict[frame]
            
            if not frame in orthoFrameDict:
                logger.info("Missing ortho for frame: " + frame)
                continue
        
            # Make sure the timestamp and frame number are in the output file name
            try:
                outputPath = icebridge_common.jpegToImageFile(inputPath, orthoFrameDict[frame])
            except Exception, e:
                logger.info(str(e))
                logger.info("Removing bad file: " + inputPath)
                if os.path.exists(inputPath): os.remove(inputPath)

                allGood = False
                continue

            if not os.path.exists(outputPath):
                logger.info("Missing image file: " + outputPath)
                allGood = False
                
        return allGood

    def conversionIsFinished(self, startFrame, stopFrame, verbose=False):
        '''Return true if this run is present and conversion has finished running on it'''
        
        logger = logging.getLogger(__name__)

        # Make sure that there is a camera file for input image file.    
        # - This could be a more expansive check.
        cameraFolder = self.getCameraFolder()
        imageList    = self.getImageList()
        for imageFile in imageList:
            camFile = os.path.join(cameraFolder,
                                   icebridge_common.getCameraFileName(imageFile))

            # Check only within range
            # TODO: Actually we need the cameras to go a bit beyond
            frame = icebridge_common.getFrameNumberFromFilename(camFile)
            if frame < startFrame or frame >= stopFrame:
                continue
            
            if not os.path.exists(camFile):
                if verbose:
                    logger.error('Missing file ' + camFile)
                return False

        # Do a simple check of the converted lidar files

        prependFolder = True
        lidarFolder   = self.getLidarFolder()
        convLidarFile = icebridge_common.getConvertedLidarIndexFile(lidarFolder)
        (lidarDict, dummyUrlDict) = icebridge_common.readIndexFile(convLidarFile,
                                                                   prependFolder)

        pairedLidarFolder = icebridge_common.getPairedLidarFolder(lidarFolder)
        pairedLidarFile   = icebridge_common.getPairedIndexFile(pairedLidarFolder)
        (pairedLidarDict, dummyUrlDict) = icebridge_common.readIndexFile(pairedLidarFile,
                                                                         prependFolder)

        numLidar = len(lidarDict.values())
        numPairedLidar = len(pairedLidarDict.values())
        
        if numLidar != (numPairedLidar+1):
            logger.error('Not enough paired lidar files found')
            return False
        
        # Make sure the lidar files are not empty
        success = True
        for f in lidarDict.values() + pairedLidarDict.values():
            if not asp_file_utils.fileIsNonZero(f):
                logger.error('lidar file ' + f + ' is empty!')
                os.system('rm -f ' + f) # Remove bad files
                success = False

        return success
    

    def getFrameRange(self):
        '''Return the min and max frame currently stored for the run'''

        jpegFolder = icebridge_common.getJpegFolder(self.getFolder())
        jpegIndexPath = icebridge_common.csvIndexFile(jpegFolder)
        if not os.path.exists(jpegIndexPath):
            raise Exception("Error: Missing jpeg index file: " + jpegIndexPath + ".")
        (jpegFrameDict, jpegUrlDict) = icebridge_common.readIndexFile(jpegIndexPath)

        frames = sorted(jpegFrameDict.keys())

        if len(frames) == 0:
            raise Exception("Empty folder: " + jpegFolder)

        return (frames[0], frames[-1])
    
    def setFlag(self, flag):
        '''Set a file based flag to be checked later'''
        os.system('touch '+ self._internalLoc(flag))
        
    def checkFlag(self, flag):
        '''Check if a file based flag has been set'''
        return os.path.exists(self._internalLoc(flag))
        
    def clearFlag(self, flag):
        '''Clear a file based flag'''
        return os.system('rm -rf ' + self._internalLoc(flag))


    def deleteLocalData(self):
        '''Delete everything which has not been archived'''
        os.system('rm -rf ' + self.getFolder())

