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
import re, shutil, time, getpass

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

import icebridge_common

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]


class RunHelper():

    def __init__(self, site, yyyymmdd, parentFolder=''):
        '''Constructor.
           If optional parent folder is provided, it will be prependend to all returned paths.'''
        self.site          = site
        self.yyyymmdd      = yyyymmdd
        self._parentFolder = parentFolder
        
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
        return 'CAMERA_' + self.name() + '.tar.gz'

    def getSummaryTarName(self):
        '''Return the file name used to tar up the generated camera files'''
        return 'SUMMARY_' + self.name() + '.tar'

    def getOutputTarName(self):
        '''Return the file name used to tar up the final results'''
        return 'DEM_' + self.name() + '_V1.tar'

    def getFolder(self):
        '''Returns the folder where this run will be stored'''
        return os.path.join(self._parentFolder, str(self))

    def _internalLoc(self, path):
        '''Returns a folder relative to getFolder()'''
        return os.path.join(self.getFolder(), path)

    def getJpegFolder(self):
        return self._internalLoc('jpeg')
    def getImageFolder(self):
        return self._internalLoc('image')
    def getLidarFolder(self):
        return self._internalLoc('lidar')
    def getLidarPairFolder(self):
        return os.path.join(self.getLidarFolder(), 'paired')
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

    def getLidarList(self, paired=False):
        '''Return a list containing all the currently stored lidar files.
           This does not return converted csv files.'''
        lidarFolder = self.getLidarFolder()
        if paired:
            lidarFolder = os.path.join(lidarFolder, 'paired')
        return icebridge_common.getLidar(lidarFolder)
        

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

    def getOutputDemList(self):
        '''Return a list containing all the output DEM files in the run and the associated frames'''
        
        batchList = self.getBatchFolderList()

        demName = 'out-align-DEM.tif'
        output = []
        for batch in batchList:
            frames = icebridge_common.getFrameRangeFromBatchFolder(batch)
            path   = os.path.join(batch, demName)
            output.append((path, frames))
        return output


    def allSourceDataFetched(self, verbose=False):
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
            for f in fileDict.itervalues():
                path = os.path.join(folder, f)
                if not os.path.exists(path):
                    if verbose:
                        logger.error('Missing file ' + camFile)
                    return False

        return True # Success!


    def conversionIsFinished(self, verbose=False):
        '''Return true if this run is present and conversion has finished running on it'''
        
        logger = logging.getLogger(__name__)

        # Make sure that there is a camera file for input image file.    
        # - This could be a more expansive check.
        cameraFolder = self.getCameraFolder()
        imageList    = self.getImageList()
        for imageFile in imageList:
            camFile = os.path.join(cameraFolder,
                                   icebridge_common.getCameraFileName(imageFile))
            if not os.path.exists(camFile):
                if verbose:
                    logger.error('Missing file ' + camFile)
                return False
        
        # Do a simple check of the lidar files
        lidarFiles       = self.getLidarList(paired=False)
        pairedLidarFiles = self.getLidarList(paired=True )
        if len(lidarFiles) != (len(pairedLidarFiles)+1):
            logger.error('Not enough paired lidar files found')
            return False       

        return True # Success!
    

    def getFrameRange(self):
        '''Return the min and max frame currently stored for the run'''

        # Loop through all the files in the jpeg folder to get the frame range
        jpegFolder = self.getJpegFolder()
        if not os.path.exists(jpegFolder):
            raise Exception('Cannot get frame range from run, data not available: ' + str(run))

        minFrame = 9999999
        maxFrame = 0
        jpegFiles = self.getJpegList(prependFolder=True)
        for jpegPath in jpegFiles:
                        
            # Update frame range
            frame = icebridge_common.getFrameNumberFromFilename(jpegPath)
            if frame < minFrame:
                minFrame = frame
            if frame > maxFrame:
                maxFrame = frame

        return (minFrame, maxFrame)


    def setFlag(self, flag):
        '''Set a file based flag to be checked later'''
        os.system('touch '+ self._internalLoc('flag'))
        
    def checkFlag(self, flag):
        '''Check if a file based flag has been set'''
        return os.path.exists(self._internalLoc('flag'))
        
    def clearFlag(self, flag):
        '''Clear a file based flag'''
        return os.system('rm -rf ' + self._internalLoc('flag'))


    def deleteLocalData(self):
        '''Delete everything which has not been archived'''
        os.system('rm -rf ' + self.getFolder())

