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
import asp_system_utils

#asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

REMOTE_INPUT_FOLDER     = 'lfe:/u/oalexan1/projects/data/icebridge'

def stripHost(val):
    # Replace lfe:/path with /path
    m = re.match("^.*?:\s*(.*?)$", val)
    if m:
        return m.group(1)
    else:
        return val
    
if icebridge_common.getUser() == 'smcmich1':
    REMOTE_CAMERA_FOLDER    = 'lfe:/u/smcmich1/icebridge/camera'
    REMOTE_ALIGN_CAM_FOLDER = 'lfe:/u/smcmich1/icebridge/aligned_cameras'
    REMOTE_ORTHO_FOLDER     = 'lfe:/u/smcmich1/icebridge/ortho'
    REMOTE_OUTPUT_FOLDER    = 'lfe:/u/smcmich1/icebridge/output'
    REMOTE_SUMMARY_FOLDER   = 'lfe:/u/smcmich1/icebridge/summaries'
    LUNOKHOD                = 'lunokhod2'
    L_SUMMARY_FOLDER        = LUNOKHOD + ':/home/smcmich1/data/icebridge_summaries'
elif icebridge_common.getUser() == 'oalexan1':
    REMOTE_CAMERA_FOLDER    = 'lfe:/u/oalexan1/projects/data/icebridge/camera'
    REMOTE_ALIGN_CAM_FOLDER = 'lfe:/u/oalexan1/projects/data/icebridge/aligned_cameras'
    REMOTE_ORTHO_FOLDER     = 'lfe:/u/oalexan1/projects/data/icebridge/ortho'
    REMOTE_OUTPUT_FOLDER    = 'lfe:/u/oalexan1/projects/data/icebridge/output'
    REMOTE_SUMMARY_FOLDER   = 'lfe:/u/oalexan1/projects/data/icebridge/summaries'
    LUNOKHOD                = 'lunokhod1'
    L_SUMMARY_FOLDER        = LUNOKHOD + ':/home/oalexan1/projects/data/icebridge/summaries'

def retrieveRunData(run, unpackFolder):
    '''Retrieve the data for the specified run from Lfe.'''

    logger = logging.getLogger(__name__)
    
    # First check that we have enough space available

    logger.info('Retrieving data for run ' + str(run))

    fileName = run.getInputTarName()

    unpackedDir = os.path.join(unpackFolder, os.path.splitext(fileName)[0])
    if os.path.exists(unpackedDir) and os.path.isdir(unpackedDir):
        logger.info("Directory exists, won't fetch from lfe: " + unpackedDir)
        return

    lfePath  = os.path.join(REMOTE_INPUT_FOLDER, fileName)

    cmd = 'shiftc --wait -d -r --verify --extract-tar ' + lfePath + ' ' + unpackFolder
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        raise Exception('Failed to copy data for run ' + str(run))

    # Retrieve a preprocessed set of camera files if we have it
    fetchCameraFolder(run)

def fetchCameraFolder(run):
    '''Fetch a camera folder from the archive if it exists.
       Returns True if we got the file.'''
    
    logger = logging.getLogger(__name__)
    logger.info('Fetching camera folder for ' + str(run))
    
    # Tar up the camera files and send them at the same time using the shiftc command
    cameraFolder = run.getCameraFolder()
    fileName     = run.getCameraTarName()
    lfePath      = os.path.join(REMOTE_CAMERA_FOLDER, fileName)

    cmd = 'shiftc --wait -d -r --extract-tar ' + lfePath + ' .'  
    logger.info(cmd)
    status = os.system(cmd)
    print status
    if status != 0:
        logger.info('Did not find camera file for run.')
        return False
    else:
        logger.info('Finished retrieving cameras from lfe.')
        return True


def packAndSendCameraFolder(run):
    '''Archive the camera folder for later use'''
    
    logger = logging.getLogger(__name__)
    logger.info('Archiving camera folder for run ' + str(run))
    
    # Tar up the camera files and send them at the same time using the shiftc command
    cameraFolder = run.getCameraFolder()
    fileName     = run.getCameraTarName()
    lfePath      = os.path.join(REMOTE_CAMERA_FOLDER, fileName)

    # First remove any existing tar file. Use 2>/dev/null to not print
    # NASA's "You have no expecation of privacy."
    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Do the new file
    runFolder = str(run)
    cmd = 'shiftc --wait -d -r --include=\'^.*?' + \
          os.path.basename(cameraFolder) + '.*?\.tsai$\' --create-tar ' + runFolder + \
          ' ' + lfePath
    
    logger.info(cmd)
    status = os.system(cmd)
    print status
    if status != 0:
        raise Exception('Failed to pack/send cameras for run ' + str(run))
    logger.info('Finished sending cameras to lfe.')

    # Test if this is reversible
    #fetchCameraFolder(run)
    
def packAndSendAlignedCameras(run):
    '''Archive the pc_align-ed cameras for later use'''
    
    logger = logging.getLogger(__name__)
    logger.info('Archiving aligned cameras for run ' + str(run))

    runFolder = str(run)
    
    fileName = run.getAlignedCameraTarName()
    lfePath  = os.path.join(REMOTE_ALIGN_CAM_FOLDER, fileName)
    
    # First remove any existing tar file
    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Create a new archive
    cmd = 'shiftc --wait -d -r --include=\'^.*?' + icebridge_common.alignedBundleStr() + \
          '.*?\.tsai$\' --create-tar ' + runFolder + \
    ' ' + lfePath
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        logger.info('Failed to pack/send aligned cameras for run ' + str(run))
    logger.info('Finished sending aligned cameras to lfe.')

def packAndSendOrthos(run):
    '''Archive the created ortho images.'''

    logger = logging.getLogger(__name__)
    logger.info('Archiving ortho images for run ' + str(run))

    runFolder = str(run)
    
    fileName = run.getOrthoTarName()
    lfePath  = os.path.join(REMOTE_ORTHO_FOLDER, fileName)

    # First remove any existing tar file
    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Create a new archive
    cmd = 'shiftc --wait -d -r --include=\'^.*?' + icebridge_common.orthoFileName() + \
          '$\' --create-tar ' + runFolder + ' ' + lfePath
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        logger.info('Failed to pack/send ortho images for run ' + str(run))
    logger.info('Finished sending ortho images to lfe.')

def packAndSendSummaryFolder(run, folder):
    '''Archive the summary folder in case we want to look at it later'''
    
    logger = logging.getLogger(__name__)
    logger.info('Archiving summary folder for run ' + str(run))
    
    # Create a local tar file
    # - Some fiddling to make the packed folders convenient
    fileName = run.getSummaryTarName()
    cmd = 'tar -chf '+ fileName +' -C '+ folder +'/.. ' + os.path.basename(folder)
    logger.info(cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)

    # This tends to print a very verbose message
    ans = out + '\n' + err
    vals = ans.split('\n')
    if len(vals) < 10:
        print(ans)
    else:
        vals = vals[0:10]
        print("\n".join(vals))
        print("Above output truncated.")

    # Delete any existing copy of the file on lfe
    lfePath  = os.path.join(REMOTE_SUMMARY_FOLDER, fileName)
    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Send the file to lfe using shiftc
    cmd = 'shiftc --wait -d -r ' + fileName + ' ' + lfePath
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        raise Exception('Failed to pack/send summary folder for run ' + str(run))
    logger.info('Finished sending summary to lfe.')

    # Wipe the copy on lunokhod
    l2Path   = os.path.join(L_SUMMARY_FOLDER, fileName)
    cmd      = "ssh " + LUNOKHOD + "  'rm -f "+ stripHost(l2Path) +"' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Make target directory on lunokhod
    cmd = "ssh  " + LUNOKHOD + " 'mkdir -p " + os.path.dirname(stripHost(l2Path)) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Send a copy of the file to Lunokhod for convenience
    cmd = 'scp ' + fileName + ' ' + l2Path + ' 2>/dev/null'
    logger.info(cmd)
    os.system(cmd)

    # Clean up the local tar file
    os.system('rm -f ' + fileName)

def packAndSendCompletedRun(run):
    '''Assembles and compresses the deliverable parts of the run'''
    
    logger = logging.getLogger(__name__)
    logger.info('Getting ready to pack up run ' + str(run))
    
    runFolder = str(run)
    
    # TODO: What do we want to deliver?
    # - The aligned DEM file from each batch folder
    # - Some low-size diagnostic information about the run
    # - Camera calibration files used?
    # - Information about how we created the run (process date, ASP version, etc)

    # Use symlinks to assemble a fake file structure to tar up
    assemblyFolder = run.getAssemblyFolder()
    batchFolders   = run.getBatchFolderList()
    os.system('mkdir -p ' + assemblyFolder)
    
    # For each batch folder, start adding links to files that we want in the tarball
    for batch in batchFolders:
        # Skip folders where we did not produce final output
        finalDemFile = os.path.join(batch, icebridge_common.blendFileName())
        if not os.path.exists(finalDemFile):
            continue
        
        # Need to change the name of these files when they go in the output folder
        (startFrame, stopFrame) = icebridge_common.getFrameRangeFromBatchFolder(batch)
        prefix = ('F_%05d_%05d' % (startFrame, stopFrame))
        prefix = os.path.join(assemblyFolder, prefix)
        target = prefix + '_DEM.tif'
        try:
            if os.path.exists(target):
                os.system("rm -f " + target) # to wipe whatever was there
            os.symlink(finalDemFile, target)
        except Exception, e:
            logger.info(str(e) + " when doing: ln -s " + finalDemFile + " " + target)
    
    # Tar up the assembled files and send them at the same time using the shiftc command
    # - No need to use a compression algorithm here
    fileName = run.getOutputTarName()
    lfePath  = os.path.join(REMOTE_OUTPUT_FOLDER, fileName)

    logger.info('Sending run to lfe...')

    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    cmd = 'shiftc --wait -d -r --dereference --create-tar ' + \
          os.path.join(runFolder, os.path.basename(assemblyFolder)) + ' ' + lfePath

    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        raise Exception('Failed to pack/send results for run ' + str(run))
    logger.info('Finished sending run to lfe.')
    
    
    
