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

import icebridge_common, archive_functions
import asp_system_utils

#asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

REMOTE_INPUT_FOLDER     = 'lfe:/u/oalexan1/projects/data/icebridge'

def robust_shiftc(cmd, logger, attempts = 120):
    '''Try to fetch/submit with shiftc for at least 2 hours, with attempts every minute.
    We worked hard for many hours to produce a run, and we should not let
    a temporary problem with the server nullify our work.'''
    sleep_time = 60

    for attempt in range(attempts):

        if attempt > 0:
            logger.info("Trying attempt: " + str(attempt+1) + " out of " + str(attempts))
            
        logger.info(cmd)
        status = os.system(cmd)
        if status == 0:
            return
        
        if attempt == attempts - 1:
            raise Exception('Failed to copy data after ' + str(attempts) + ' attempts.')
        else:
            attempt += 1 
            logger.info("Will try shiftc again after sleeping for " + str(sleep_time) + \
                        " seconds.")
            time.sleep(sleep_time)

    
def stripHost(val):
    # Replace lfe:/path with /path
    m = re.match("^.*?:\s*(.*?)$", val)
    if m:
        return m.group(1)
    else:
        return val

def get_remote_output_folder(user):
    '''Get the folder containing DEMs for given user.'''
    if user == 'smcmich1':
        return 'lfe:/u/smcmich1/icebridge/output'
    elif user == 'oalexan1':
        return 'lfe:/u/oalexan1/projects/data/icebridge/output'
    else:
        raise Exception("Unknown user: " + user)
    
def get_remote_ortho_folder(user):
    '''Get the folder containing orthos for given user.'''
    if user == 'smcmich1':
        return 'lfe:/u/smcmich1/icebridge/ortho'
    elif user == 'oalexan1':
        return 'lfe:/u/oalexan1/projects/data/icebridge/ortho'
    else:
        raise Exception("Unknown user: " + user)
    
if icebridge_common.getUser() == 'smcmich1':
    REMOTE_CAMERA_FOLDER    = 'lfe:/u/smcmich1/icebridge/camera'
    REMOTE_ALIGN_CAM_FOLDER = 'lfe:/u/smcmich1/icebridge/aligned_cameras'
    REMOTE_ORTHO_FOLDER     = 'lfe:/u/smcmich1/icebridge/ortho'
    REMOTE_SUMMARY_FOLDER   = 'lfe:/u/smcmich1/icebridge/summaries'
    REMOTE_LABEL_FOLDER     = 'lfe:/u/smcmich1/icebridge/labels'
    LUNOKHOD                = 'lunokhod2'
    L_SUMMARY_FOLDER        = LUNOKHOD + ':/home/smcmich1/data/icebridge_summaries'
elif icebridge_common.getUser() == 'oalexan1':
    REMOTE_CAMERA_FOLDER    = 'lfe:/u/oalexan1/projects/data/icebridge/camera'
    REMOTE_ALIGN_CAM_FOLDER = 'lfe:/u/oalexan1/projects/data/icebridge/aligned_cameras'
    REMOTE_SUMMARY_FOLDER   = 'lfe:/u/oalexan1/projects/data/icebridge/summaries'
    REMOTE_LABEL_FOLDER     = 'lfe:/u/oalexan1/projects/data/icebridge/labels'
    LUNOKHOD                = 'lunokhod1'
    L_SUMMARY_FOLDER        = LUNOKHOD + ':/home/oalexan1/projects/data/icebridge/summaries'

REMOTE_OUTPUT_FOLDER = get_remote_output_folder(icebridge_common.getUser())
REMOTE_ORTHO_FOLDER  = get_remote_ortho_folder(icebridge_common.getUser())

def retrieveRunData(run, unpackFolder, useTar, forceTapeFetch, skipTapeCameraFetch, logger):
    '''Retrieve the data for the specified run from Lfe.'''

    logger.info('Retrieving data for run ' + str(run))

    fileName = run.getInputTarName()

    unpackDir = os.path.join(unpackFolder, os.path.splitext(fileName)[0])
    jpegDir = os.path.join(unpackDir, os.path.basename(run.getJpegFolder()))
    if os.path.exists(jpegDir) and os.path.isdir(jpegDir) and (not forceTapeFetch):
        logger.info("Won't fetch from lfe, as we already have: " + jpegDir)
        return

    lfePath  = os.path.join(REMOTE_INPUT_FOLDER, fileName)

    # I have had bad luck with shift to fetch. So in the worst case try tar.

    tar_cmd = 'ssh lfe "cd ' + os.path.dirname(stripHost(lfePath)) + "; tar xfv " + \
              os.path.basename(lfePath) + " -C " + os.path.realpath(unpackFolder) + '"'
    shift_cmd = 'shiftc --wait -d -r --verify --extract-tar ' + lfePath + ' ' + unpackFolder
    
    if useTar:
        # If to use tar right away
        robust_shiftc(tar_cmd, logger, attempts = 1)
    else:
        try:
            # If shiftc does not work, fall back to tar.
            # Don't try too hard on fetching.
            robust_shiftc(shift_cmd, logger, attempts = 10)
        except Exception, e:
            robust_shiftc(tar_cmd, logger, attempts = 1)
            
    # Retrieve a preprocessed set of camera files if we have it
    if not skipTapeCameraFetch:
        fetchCameraFolder(run, logger)
    else:
        logger.info("Skip fetching cameras from tape.")

def fetchProcessedByType(run, unpackFolder, logger, dataType):
    '''Fetch from lfe the latest archive of processed DEMs or orthos by modification time.
    For now we ignore the timestamp.'''
    
    logger.info('Fetching processed ' + dataType + ' data for ' + str(run))

    # Fetch the latest file by modification time as returned by 'ls'. 
    # A given file can be either in Scott's or Oleg's archive. 
    # It can be in both, and it can be multiple instances.
    # Need to list all and fetch the latest.
    if dataType == 'DEM':
        fileName = run.getOutputTarName(useWildCard = True)
        lfePaths = os.path.join(stripHost(get_remote_output_folder('oalexan1')), fileName) + " " + \
                   os.path.join(stripHost(get_remote_output_folder('smcmich1')), fileName)
    elif dataType == 'ORTHO':
        fileName = run.getOrthoTarName(useWildCard = True)
        lfePaths = os.path.join(stripHost(get_remote_ortho_folder('oalexan1')), fileName) + " " + \
                   os.path.join(stripHost(get_remote_ortho_folder('smcmich1')), fileName)
    else:
        raise Exception("Unknown data type: " + dataType)

    # Get the list 
    cmd = 'ssh lfe "ls -altd ' + lfePaths + '"'
    logger.info(cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)

    # Parse the answer and keep the latest. 
    out = out.strip()
    vals = out.split('\n')
    for val in vals:
        logger.info("Found: " + val)
    lfePath = ""
    for val in vals:
        if len(val) >= 1 and val[0] == 'l':
            continue # this is a symlink
        arr = val.split()
        lfePath = arr[-1]
        break # found what we needed

    if lfePath == "":
        logger.info("Could not locate: " + fileName)
        return False

    cmd = 'shiftc --wait -d -r --verify --extract-tar ' + 'lfe:' + lfePath + ' ' + unpackFolder  
    logger.info(cmd)
    status = os.system(cmd)
    
    # Try tar if shift failed
    if status != 0:
        logger.info('Did not sucessfully fetch archived processed data of type: ' + dataType)
        logger.info('Will try with tar.''')

        tar_cmd = 'ssh lfe "cd ' + os.path.dirname(lfePath) + "; tar xfv " + \
                  os.path.basename(lfePath) + " -C " + os.path.realpath(unpackFolder) + '"'
        
        logger.info(tar_cmd)
        status = os.system(tar_cmd)
        if status != 0:
            logger.info('Failed using tar as well.')
            
            return False

    return True

def fetchCameraFolder(run, logger):
    '''Fetch a camera folder from the archive if it exists.  Returns
    True if we got the file. If more than one, return the latest by
    modification time.'''
    
    logger.info('Fetching camera folder for ' + str(run))
    
    # Tar up the camera files and send them at the same time using the shiftc command
    cameraFolder = run.getCameraFolder()
    fileName     = run.getCameraTarName()

    # There could be multiple camera folders, fetch the latest by modification time.
    strippedName = fileName
    m = re.match('^(.*?)\.tar', strippedName)
    if m:
        strippedName = m.group(1)
    m = re.match('^(.*?)' + run.suffix, strippedName)
    if m:
        strippedName = m.group(1)
    cmd = 'ssh lfe "ls -dt ' + stripHost(REMOTE_CAMERA_FOLDER) + '/' + strippedName + '*.tar"'
    logger.info(cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)
    out = out.strip()
    vals = out.split()
    if len(vals) >= 1:
        # Pick the first one, which is the newest
        out = vals[0]
    if out == "":
        logger.info('Did not find camera file for run.')
        return False
    fileName = os.path.basename(out)
    lfePath  = os.path.join(REMOTE_CAMERA_FOLDER, fileName)
    cmd = 'shiftc --wait -d -r --extract-tar ' + lfePath + ' .'  

    # Don't try too hard below, if failed first time that means there are no cameras
    # to fetch
    logger.info(cmd)
    status = os.system(cmd)
    if status != 0:
        logger.info('Did not find camera file for run.')
        return False
    else:
        logger.info('Finished retrieving cameras from lfe.')
        return True

def packAndSendCameraFolder(run, logger):
    '''Archive the camera folder for later use'''
    
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

    # Do the new file. Save the projection bounds, we will need that for later
    # as that file is very time consuming to create.
    cwd = os.getcwd()
    os.chdir(run.parentFolder)
    
    runFolder = str(run)
    cmd = 'shiftc --wait -d -r --include=\'^.*?(' + \
          os.path.basename(icebridge_common.projectionBoundsFile(runFolder)) + \
          '|' + icebridge_common.validFilesPrefix() + '.*|' + \
          os.path.basename(cameraFolder) + '.*?\.tsai)$\' --create-tar ' + runFolder + \
          ' ' + lfePath
    
    robust_shiftc(cmd, logger)

    logger.info("Finished archiving cameras.")
    
    os.chdir(cwd)
    
    # Test if this is reversible
    #fetchCameraFolder(run, logger)
    
def packAndSendAlignedCameras(run, logger):
    '''Archive the pc_align-ed cameras for later use'''
    
    logger.info('Archiving aligned cameras for run ' + str(run))
    cwd = os.getcwd()
    os.chdir(run.parentFolder)
    
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

    robust_shiftc(cmd, logger)

    os.chdir(cwd)
    logger.info('Finished sending aligned cameras to lfe.')

def packAndSendOrthos(run, logger):
    '''Archive the created ortho images.'''

    logger.info('Archiving ortho images for run ' + str(run))

    cwd = os.getcwd()
    os.chdir(run.parentFolder)
    
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
    robust_shiftc(cmd, logger)

    os.chdir(cwd)
    logger.info('Finished sending ortho images to lfe.')

def packAndSendSummaryFolder(run, folder, logger):
    '''Archive the summary folder in case we want to look at it later'''
    
    logger.info('Archiving summary folder for run ' + str(run))
    
    cwd = os.getcwd()
    os.chdir(run.parentFolder)

    fileName = run.getSummaryTarName()

    # Create a local tar file.

    # Turn this approach off, new approach below.
    # - Some fiddling to make the packed folders convenient
    cmd = 'tar -chf '+ fileName +' -C '+ folder +'/.. ' + os.path.basename(folder)
    logger.info(cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)
    # This tends to print a very verbose message
    ans = out + '\n' + err
    vals = ans.split('\n')
    if len(vals) < 10:
        logger.info(ans)
    else:
        vals = vals[0:10]
        logger.info("\n".join(vals))
        logger.info("Above output truncated.")

    # Use shiftc to create a local copy, and we want to include log files too
    #runFolder = str(run)
    #sumName = os.path.basename(run.getSummaryFolder())
    #cmd = 'shiftc --wait -d -r --dereference --include=\'^.*?('  \
    #      + icebridge_common.logFilePrefix() + '|' \
    #      + runFolder + '/' + sumName        + '|' \
    #      + icebridge_common.manager_log_prefix()  \
    #      + ')\' --create-tar ' + runFolder        \
    #      +  ' ' + fileName
    
    #logger.info(cmd)
    #os.system(cmd)
    
    # Delete any existing copy of the file on lfe
    lfePath  = os.path.join(REMOTE_SUMMARY_FOLDER, fileName)
    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Send the file to lfe using shiftc
    cmd = 'shiftc --wait -d -r ' + fileName + ' ' + lfePath
    robust_shiftc(cmd, logger)

    logger.info('Finished sending summary to lfe.')

    if icebridge_common.getUser() != 'oalexan1':
        # Wipe the copy on lunokhod
        l2Path   = os.path.join(L_SUMMARY_FOLDER, fileName)
        cmd      = "ssh " + LUNOKHOD + "  'rm -f "+ stripHost(l2Path) +"' 2>/dev/null"
        logger.info(cmd)
        os.system(cmd)
        
        # Make target directory on lunokhod
        cmd = "ssh  " + LUNOKHOD + " 'mkdir -p " + os.path.dirname(stripHost(l2Path)) + \
              "' 2>/dev/null"
        logger.info(cmd)
        os.system(cmd)
        
        # Send a copy of the file to Lunokhod for convenience
        cmd = 'scp ' + fileName + ' ' + l2Path + ' 2>/dev/null'
        logger.info(cmd)
        os.system(cmd)

    # Clean up the local tar file
    cmd = 'rm -f ' + fileName
    logger.info(cmd)
    os.system(cmd)

    os.chdir(cwd)

def packAndSendCompletedRun(run, logger):
    '''Assembles and compresses the deliverable parts of the run'''
    
    logger.info('Getting ready to pack up run ' + str(run))
    
    cwd = os.getcwd()
    os.chdir(run.parentFolder)
    
    runFolder = str(run)

    # Use symlinks to assemble a fake file structure to tar up
    assemblyFolder = run.getAssemblyFolder()
    batchFolders   = run.getBatchFolderList()
    os.system('mkdir -p ' + assemblyFolder)

    # Wipe any dead symlinks, as maybe this is not the first time the assembly is made
    pattern = os.path.join(assemblyFolder, '*')
    currFiles = glob.glob(pattern)
    for filename in currFiles:
        if not os.path.exists(os.path.realpath(filename)):
            logger.info("Will wipe dead link: " + filename)
            os.system("rm -f " + filename)
            
    # For each batch folder, start adding links to files that we want in the tarball
    for batch in batchFolders:
        # Skip folders where we did not produce final output
        finalDemFile = os.path.join(batch, icebridge_common.blendFileName())
        if not os.path.exists(finalDemFile):
            continue
        
        # Need to change the name of these files when they go in the output folder
        (startFrame, stopFrame) = icebridge_common.getFrameRangeFromBatchFolder(batch)
        # We respect the convention below in push_to_nsidc.py.
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

    cmd = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"

    logger.info(cmd)
    os.system(cmd)

    cmd = 'shiftc --wait -d -r --dereference --create-tar ' + \
          os.path.join(runFolder, os.path.basename(assemblyFolder)) + ' ' + lfePath

    # Command to transfer the files as they are in the batch dirs without symlink
    #cmd = 'shiftc --wait -d -r --include=\'^.*?' + icebridge_common.blendFileName() + \
    #      '$\' --create-tar ' + runFolder + \
    #      ' ' + lfePath

    try:
        robust_shiftc(cmd, logger)
    except Exception, e:
        logger.info(str(e))
        raise Exception('Failed to pack/send results for run ' + str(run) + \
                    '. Maybe not all sym links are valid.')
    
    os.chdir(cwd)
    
    logger.info('Finished sending run to lfe.')

def packAndSendLabels(run, logger):
    '''Archive the labeled images generated by the 3rd party tool.'''
    
    # Archive everything stored in the 'labeled' folder under the run folder.
    
    logger.info('Archiving labled images for run ' + str(run))
    cwd = os.getcwd()
    os.chdir(run.parentFolder)
    
    runFolder = str(run)   
    relRunFolder = os.path.join(runFolder, os.path.basename(run.getLabelFolder)) 
    fileName = run.getLabelTarName()
    lfePath  = os.path.join(REMOTE_LABEL_FOLDER, fileName)
    
    # First remove any existing tar file
    cmd      = "ssh lfe 'rm -f " + stripHost(lfePath) + "' 2>/dev/null"
    logger.info(cmd)
    os.system(cmd)

    # Create a new archive
    cmd = 'shiftc --wait -d -r --create-tar ' + relRunFolder + ' ' + lfePath
    robust_shiftc(cmd, logger)

    os.chdir(cwd)

    logger.info('Finished sending labels to lfe.')



    
    
