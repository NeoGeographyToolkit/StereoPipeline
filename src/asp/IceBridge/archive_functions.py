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

# Contains functions to store/retrieve data from outside a run folder

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

import icebridge_common, archive_functions

#asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]


REMOTE_INPUT_FOLDER     = 'lou:/u/oalexan1/projects/data/icebridge'
REMOTE_CAMERA_FOLDER    = 'lou:/u/smcmich1/icebridge/camera'
REMOTE_OUTPUT_FOLDER    = 'lou:/u/smcmich1/icebridge/output'



def retrieveRunData(run):
    '''Retrieve the data for the specified run from Lou.'''

    logger = logging.getLogger(__name__)
    
    # Skip retrieval if we already have the data
    if isRunReadyForProcessing(run):
        logger.info('No need to retrieve run ' + str(run) + ', it is already on disk.')
        return

    # First check that we have enough space available

    logger.info('Retrieving data for run ' + str(run))

    fileName = run.getInputTarName()
    louPath  = os.path.join(REMOTE_INPUT_FOLDER, fileName)

    cmd = 'shiftc --wait --sync --verify --extract-tar ' + louPath + ' ' + UNPACK_FOLDER
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        raise Exception('Failed to copy data for run ' + str(run))

    # Retrieve a preprocessed set of camera files if we have it
    fetchCameraFolder(run)
    
    # TODO: Is it safe to set the conversion finished flag if we get the camera folder?
    




def fetchCameraFolder(run):
    '''Fetch a camera folder from the archive if it exists.
       Returns True if we got the file.'''
    
    logger = logging.getLogger(__name__)
    logger.info('Fetching camera folder for ' + str(run))
    
    # Tar up the camera files and send them at the same time using the shiftc command
    cameraFolder = run.getCameraFolder()
    fileName     = run.getCameraTarName()
    louPath      = os.path.join(REMOTE_CAMERA_FOLDER, fileName)

    cmd = 'shiftc  --wait --extract-tar ' + louPath + ' ' + cameraFolder
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        logger.info('Did not find camera file for run.')
        return False
    else:
        logger.info('Finished sending cameras to lou.')
        return True


def packAndSendCameraFolder(run):
    '''Archive the camera folder for later use'''
    
    logger = logging.getLogger(__name__)
    logger.info('Archiving camera folder for run ' + str(run))
    
    # Tar up the camera files and send them at the same time using the shiftc command
    cameraFolder = run.getCameraFolder()
    fileName     = run.getCameraTarName()
    louPath      = os.path.join(REMOTE_CAMERA_FOLDER, fileName)

    cmd = 'shiftc --wait --create-tar ' + cameraFolder + ' ' + louPath
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        raise Exception('Failed to pack/send cameras for run ' + str(run))
    logger.info('Finished sending cameras to lou.')


def packAndSendCompletedRun(run):
    '''Assembles and compresses the deliverable parts of the run'''
    
    logger = logging.getLogger(__name__)
    logger.info('Getting ready to pack up run ' + str(run))
    
    runFolder = run.getFolder()
    
    # TODO: What do we want to deliver?
    # - The aligned DEM file from each batch folder
    # - Some low-size diagnostic information about the run
    # - Camera calibration files used?
    # - Information about how we created the run (process date, ASP version, etc)

    # Use symlinks to assemble a fake file structure to tar up
    assemblyFolder = run.getAssemblyFolder()
    batchFolders   = run.getBatchFolderList()
    
    # For each batch folder, start adding links to files that we want in the tarball
    for batch in batchFolders:
        alignDemFile = os.path.join(batch,'out-align-DEM.tif')
        
        # Need to change the name of these files when they go in the output folder
        (startFrame, stopFrame) = getFrameRangeFromBatchFolder(batch)
        prefix = ('F_%d_%d' % (startFrame, stopFrame))
        prefix = os.path.join(assemblyFolder, prefix)
        
        os.symlink(alignDemFile, prefix+'_aligned_DEM.tif')
    
    
    # Tar up the assembled files and send them at the same time using the shiftc command
    # - No need to use a compression algorithm here
    fileName = run.getOutputTarName()
    louPath  = os.path.join(REMOTE_OUTPUT_FOLDER, fileName)

    # TODO: Don't use the wait command here, and clean up after this transfer is finished!

    logger.info('Sending run to lou...')
    cmd = 'shiftc --dereference --create-tar ' + assemblyFolder + ' ' + louPath
    logger.info(cmd)
    #status = os.system(cmd)
    if status != 0:
        raise Exception('Failed to pack/send results for run ' + str(run))
    logger.info('Finished sending run to lou.')
    
    
    
