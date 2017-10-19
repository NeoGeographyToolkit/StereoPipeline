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

# Fetch all the data for a run and then process all the data.
# See sample usage below.

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing, re, shutil, time
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

import icebridge_common, fetch_icebridge_data, process_icebridge_run
import extract_icebridge_ATM_points, camera_models_from_nav
import asp_system_utils, asp_alg_utils, asp_geo_utils

asp_system_utils.verify_python_version_is_supported()

def convertJpegs(jpegFolder, imageFolder, startFrame, stopFrame, skipValidate, logger):
    '''Convert jpeg images from RGB to single channel.
       Returns false if any files failed.'''

    badFiles = False
    
    logger.info('Converting input images to grayscale...')

    os.system('mkdir -p ' + imageFolder)

    # Loop through all the input images

    jpegIndexPath = icebridge_common.csvIndexFile(jpegFolder)
    if not os.path.exists(jpegIndexPath):
        raise Exception("Error: Missing jpeg index file: " + jpegIndexPath + ".")
    (jpegFrameDict, jpegUrlDict) = icebridge_common.readIndexFile(jpegIndexPath,
                                                                  prependFolder = True)
    
    # Need the orthos to get the timestamp
    orthoFolder = icebridge_common.getOrthoFolder(os.path.dirname(jpegFolder))
    orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
    if not os.path.exists(orthoIndexPath):
        raise Exception("Error: Missing ortho index file: " + orthoIndexPath + ".")
    (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath,
                                                                  prependFolder = True)
    
    if not skipValidate:
        validFilesList = icebridge_common.validFilesList(os.path.dirname(jpegFolder),
                                                         startFrame, stopFrame)
        validFilesSet = set()
        validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
        numInitialValidFiles = len(validFilesSet)
        
    # Fast check for missing images. This is fragile, as maybe it gets
    # the wrong file with a similar name, but an honest check is very slow.
    imageFiles = icebridge_common.getTifs(imageFolder, prependFolder = True)
    imageFrameDict = {}
    for imageFile in imageFiles:
        frame = icebridge_common.getFrameNumberFromFilename(imageFile)
        if frame < startFrame or frame > stopFrame: continue
        imageFrameDict[frame] = imageFile
        
    for frame in sorted(jpegFrameDict.keys()):

        inputPath = jpegFrameDict[frame]
        
        # Only deal with frames in range
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue

        if frame in imageFrameDict.keys() and skipValidate:
            # Fast, hackish check
            continue

        if frame not in orthoFrameDict:
            logger.info("Error: Could not find ortho image for jpeg frame: " + str(frame))
            # Don't want to throw here. Just ignore the missing ortho
            continue
        
        # Make sure the timestamp and frame number are in the output file name
        try:
            outputPath = icebridge_common.jpegToImageFile(inputPath, orthoFrameDict[frame])
        except Exception, e:
            logger.info(str(e))
            logger.info("Removing bad file: " + inputPath)
            os.system('rm -f ' + inputPath) # will not throw
            badFiles = True
            continue
        
        # Skip existing valid files
        if skipValidate:
            if os.path.exists(outputPath):
                logger.info("File exists, skipping: " + outputPath)
                continue
        else:
            if outputPath in validFilesSet and os.path.exists(outputPath):
                #logger.info('Previously validated: ' + outputPath) # very verbose
                validFilesSet.add(inputPath) # Must have this
                continue
            
            if icebridge_common.isValidImage(outputPath):
                #logger.info("File exists and is valid, skipping: " + outputPath) # verbose
                if not skipValidate:
                    # Mark both the input and the output as validated
                    validFilesSet.add(inputPath) 
                    validFilesSet.add(outputPath)
                continue
        
        # Use ImageMagick tool to convert from RGB to grayscale
        
        cmd = ('%s %s -colorspace Gray %s') % \
              (asp_system_utils.which('convert'), inputPath, outputPath)
        logger.info(cmd)

        # Run command and fetch its output
        p = subprocess.Popen(cmd.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            badFiles = True
            logger.error("Command failed.")
            logger.error("Wiping bad files: " + inputPath + " and " + outputPath + '\n'
                         + output)
            os.system('rm -f ' + inputPath) # will not throw
            os.system('rm -f ' + outputPath) # will not throw

        if not os.path.exists(outputPath):
            badFiles = True
            logger.error('Failed to convert jpeg file: ' + inputPath)
            logger.error("Wiping bad files: " + inputPath + " and " + outputPath + '\n'
                         + output)
            os.system('rm -f ' + inputPath) # will not throw
            os.system('rm -f ' + outputPath) # will not throw

        # Check for corrupted files
        if error is not None:
            output += error
        m = re.match("^.*?premature\s+end", output, re.IGNORECASE|re.MULTILINE|re.DOTALL)
        if m:
            badFiles = True
            logger.error("Wiping bad files: " + inputPath + " and " + outputPath + '\n'
                         + output)
            os.system('rm -f ' + inputPath) # will not throw
            os.system('rm -f ' + outputPath) # will not throw

    if not skipValidate:
        # Write to disk the list of validated files, but only if new
        # validations happened.  First re-read that list, in case a
        # different process modified it in the meantime, such as if two
        # managers are running at the same time.
        numFinalValidFiles = len(validFilesSet)
        if numInitialValidFiles != numFinalValidFiles:
            validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList,
                                                                          validFilesSet)
            icebridge_common.writeValidFilesList(validFilesList, validFilesSet)
            
    if badFiles:
        logger.error("Converstion of JPEGs failed. If any files were corrupted, " +
                     "they were removed, and need to be re-fetched.")
    
    return (not badFiles)
            
def correctFireballDems(fireballFolder, corrFireballFolder, startFrame, stopFrame, isNorth,
                        skipValidate, logger):
    '''Fix the header problem in Fireball DEMs'''

    logger.info('Correcting Fireball DEMs ...')

    # Read the existing DEMs
    fireballIndexPath = icebridge_common.csvIndexFile(fireballFolder)
    if not os.path.exists(fireballIndexPath):
        raise Exception("Error: Missing fireball index file: " + fireballIndexPath + ".")
        
    (fireballFrameDict, fireballUrlDict) = \
                        icebridge_common.readIndexFile(fireballIndexPath, prependFolder = True)
    
    if not skipValidate:
        validFilesList = icebridge_common.validFilesList(os.path.dirname(fireballFolder),
                                                         startFrame, stopFrame)
        validFilesSet = set()
        validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
        numInitialValidFiles = len(validFilesSet)

    # Loop through all the input images
    os.system('mkdir -p ' + corrFireballFolder)
    badFiles = False
    for frame in sorted(fireballFrameDict.keys()):

        # Skip if outside the frame range
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue

        inputPath = fireballFrameDict[frame]
        if not icebridge_common.isDEM(inputPath):
            continue

        outputPath = os.path.join(corrFireballFolder, os.path.basename(inputPath))

        # Skip existing valid files
        if skipValidate:
            if os.path.exists(outputPath):
                logger.info("File exists, skipping: " + outputPath)
                continue
        else:
            if outputPath in validFilesSet and os.path.exists(outputPath):
                #logger.info('Previously validated: ' + outputPath) # very vebose
                continue
            
            if icebridge_common.isValidImage(outputPath):
                #logger.info("File exists and is valid, skipping: " + outputPath)
                validFilesSet.add(outputPath) # mark it as validated
                continue
        
        # Run the correction script
        execPath = asp_system_utils.which('correct_icebridge_l3_dem')
        cmd = (('%s %s %s %d') %
               (execPath, inputPath, outputPath, isNorth))
        logger.info(cmd)
        # TODO: Run this as a subprocess and check the return code
        os.system(cmd)
        
        # Check if the output file is good
        if not icebridge_common.isValidImage(outputPath):
            logger.error('Failed to convert dem file, wiping: ' + inputPath + ' ' + outputPath)
            os.system('rm -f ' + inputPath) # will not throw
            os.system('rm -f ' + outputPath) # will not throw
            badFiles = True
        else:
            if not skipValidate:
                validFilesSet.add(outputPath) # mark it as validated
            
    if not skipValidate:
        # Write to disk the list of validated files, but only if new
        # validations happened.  First re-read that list, in case a
        # different process modified it in the meantime, such as if two
        # managers are running at the same time.
        numFinalValidFiles = len(validFilesSet)
        if numInitialValidFiles != numFinalValidFiles:
            validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList,
                                                                          validFilesSet)
            icebridge_common.writeValidFilesList(validFilesList, validFilesSet)

    return (not badFiles)
            
def getCalibrationFileForFrame(cameraLoopkupFile, inputCalFolder, frame, yyyymmdd, site):
    '''Return the camera model file to be used with a given input frame.'''

    # To manually force a certain camera file, use this spot!
    #name = 'IODCC0_2015_GR_NASA_DMS20.tsai'
    #return os.path.join(inputCalFolder, name)

    camera = ''
    
    # Iterate through lines in lookup file
    with open(cameraLoopkupFile, "r") as cf:
        for line in cf:

            # Split line into parts and match site/date entries
            line = line.strip()
            vals = re.split('\s+', line)
            if len(vals) < 3:
                continue
            if vals[0] != site or vals[1] != yyyymmdd:
                continue
            
            curr_camera = vals[2]
            if curr_camera == "":
                raise Exception('Found an empty camera for day and site: ' + yyyymmdd + ' ' + site)
                
            # There is one default camera, and possible a backup camera for a range
            # of frames
            # - Currently these are on separate lines
            m = re.match("^.*?frames\s+(\d+)-(\d+)", line)
            if not m:
                # The default camera, it is always before the backup one in the list.
                # So this map must not be populated yet with this key.
                camera = curr_camera
            else:
                # The backup camera for a range
                startRange = int(m.group(1))
                stopRange  = int(m.group(2))
                if (frame >= startRange) and (frame <= stopRange):
                    camera = curr_camera

            # TODO: Modify file format so each flight has all info on a single line!
            #break # Each frame will match only one line

    if camera == "":
        logger.error('Failed to parse the camera.')
        raise Exception('Failed to parse the camera.')

    # Switch the extension to .tsai
    camera = camera[:-4] + '.tsai'

    return os.path.join(inputCalFolder, camera)

def cameraFromOrthoWrapper(inputPath, orthoPath, inputCamFile, estimatedCameraPath, 
                           outputCamFile, refDemPath, simpleCamera, numThreads):
    '''Generate a camera model from a single ortho file'''

    # Make multiple calls with different options until we get one that works well
    IP_METHOD    = [1, 0, 2, 1, 2, 0] # IP method
    FORCE_SIMPLE = [0, 0, 0, 0, 0, 1] # If all else fails use simple mode
    LOCAL_NORM   = [False, False, False, True, True, False] # If true, image tiles are individually normalized with method 1 and 2
    numAttempts = len(IP_METHOD)
   
    MIN_IP     = 15  # Require more IP to make sure we don't get bogus camera models
    DESIRED_IP = 200 # If we don't hit this number, try other methods before taking the best one.

    # The max distance in meters the ortho2pinhole solution is allowed to move from the input
    #  navigation estimate.
    MAX_TRANSLATION = 7

    bestIpCount = 0
    tempFilePath  = outputCamFile + '_temp' # Used to hold the current best result
    matchPath     = outputCamFile + '.match' # Used to hold the match file if it exists
    tempMatchPath = matchPath + '_temp'

    os.system("ulimit -c 0")  # disable core dumps
    os.system("rm -f core.*") # these keep on popping up
    os.system("umask 022")    # enforce files be readable by others
        
    for i in range(0,numAttempts):

        # Get parameters for this attempt
        ipMethod  = IP_METHOD[i]
        localNorm = LOCAL_NORM[i]

        if FORCE_SIMPLE[i]: # Always turn this on for the final attempt!
            simpleCamera = True

        # Call ortho2pinhole command
        ortho2pinhole = asp_system_utils.which("ortho2pinhole")
        cmd = (('%s %s %s %s %s --reference-dem %s --crop-reference-dem --threads %d --ip-detect-method %d' \
                ' --minimum-ip %d --max-translation %f') 
                % (ortho2pinhole, inputPath, orthoPath, inputCamFile, outputCamFile, 
                   refDemPath, numThreads, ipMethod, MIN_IP, MAX_TRANSLATION) )
        if localNorm:
            cmd += ' --skip-image-normalization'
        if estimatedCameraPath is not None:
            cmd += ' --camera-estimate ' + estimatedCameraPath
        if simpleCamera:
            cmd += ' --short-circuit'

        # Use a print statement as the logger fails from multiple processes
        print(cmd)

        os.system('rm -f ' + matchPath) # Needs to be gone
        p = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
        textOutput, err = p.communicate()
        p.wait()
        print(textOutput)

        if not os.path.exists(outputCamFile): # Keep trying if no output file produced
            continue

        if simpleCamera:
            break # Never need more than one attempt with simpleCamera!

        # Check the number of IP used
        m = re.findall(r"Using (\d+) points to create the camera model.", textOutput)
        if len(m) != 1: # An unknown error occurred, move on.
            continue
        numPoints = int(m[0])
        if numPoints >= DESIRED_IP: # Got a lot of points, quit
            break
        if numPoints > bestIpCount: # Got some points but not many, try other options 
            bestIpCount = numPoints #  to see if we can beat this result.
            shutil.move(outputCamFile, tempFilePath)
            
            if os.path.exists(matchPath):
                shutil.move(matchPath, tempMatchPath)

    if (not simpleCamera) and (numPoints < DESIRED_IP): # If we never got the desired # of points
        shutil.move(tempFilePath, outputCamFile) # Use the camera file with the most points found
        if os.path.exists(tempMatchPath):
            shutil.move(tempMatchPath, matchPath)
        print 'Best number of ortho points = ' + str(bestIpCount)
    else:
        print 'Best number of ortho points = ' + str(numPoints)
    
    os.system('rm -f ' + tempFilePath ) # Clean up these files
    os.system('rm -f ' + tempMatchPath)
    os.system("rm -f core.*") # these keep on popping up
    os.system("rm -f " + outputCamFile + "*-log-*") # wipe logs
              
    if not os.path.exists(outputCamFile):
        # This function is getting called from a pool, so just log the failure.
        print('Failed to convert ortho file: ' + orthoPath)

    # I saw this being recommended, to dump all print statements in the current task
    sys.stdout.flush()

def getCameraModelsFromOrtho(imageFolder, orthoFolder, inputCalFolder,
                             cameraLookupPath, navCameraFolder,
                             yyyymmdd, site,
                             refDemPath, cameraFolder,
                             simpleCameras,
                             startFrame, stopFrame,
                             framesFile,
                             numProcesses, numThreads, logger):
    '''Generate camera models from the ortho files.
       Returns false if any files were not generated.'''
    
    logger.info('Generating camera models from ortho images...')
    
    imageFiles    = icebridge_common.getTifs(imageFolder)
    orthoFiles    = icebridge_common.getTifs(orthoFolder)
    if navCameraFolder != "":
        estimateFiles = icebridge_common.getByExtension(navCameraFolder, '.tsai')
    else:
        estimateFiles = []

    # See if to process frames from file
    filesSet = set()
    if framesFile != "":
        filesSet = icebridge_common.readLinesInSet(framesFile)

    # Make a dictionary of ortho files by frame
    # - The orthoFiles list contains _gray.tif as well as the original
    #   images.  Prefer the gray versions because it saves a bit of time
    #   in the ortho2pinhole process.
    orthoFrames = {}
    for f in orthoFiles:

        frame = icebridge_common.getFrameNumberFromFilename(f)

        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue
        if (framesFile != "") and (str(frame) not in filesSet):
            continue

        # Record this file if it is the first of this frame or
        #  if it is the gray version of this frame.
        if (frame not in orthoFrames) or ('_gray.tif' in f):
            orthoFrames[frame] = f

    # Make a dictionary of estimated camera files by frame
    estimatedFrames = {}
    for f in estimateFiles:
        frame = icebridge_common.getFrameNumberFromFilename(f)
        
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue
        if (framesFile != "") and (str(frame) not in filesSet):
            continue

        estimatedFrames[frame] = f

    imageFiles.sort()

    logger.info('Starting ortho processing pool with ' + str(numProcesses) + ' processes.')
    pool = multiprocessing.Pool(numProcesses)

    # Loop through all input images
    taskHandles = []
    outputFiles = []
    for imageFile in imageFiles:

        # Skip non-image files (including _sub images made by stereo_gui)
        # TODO: Use a function here from icebridge_common. Also replace
        # all similar locations.
        ext = os.path.splitext(imageFile)[1]
        if (ext != '.tif') or ('_sub' in imageFile) or ('pct.tif' in imageFile):
            continue

        # Get associated orthofile
        frame = icebridge_common.getFrameNumberFromFilename(imageFile)

        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue
        if (framesFile != "") and (str(frame) not in filesSet):
            continue

        if not frame in orthoFrames.keys():
            continue
        
        orthoFile = orthoFrames[frame]
        try:
            estimatedCameraFile = estimatedFrames[frame]
            estimatedCameraPath = os.path.join(navCameraFolder, estimatedCameraFile)
        except:
            logger.warning('Missing nav estimated camera for frame ' + str(frame))
            estimatedCameraFile = None
            estimatedCameraPath = None
        
        # Get estimated camera from nav
        
        # Check output file
        inputPath = os.path.join(imageFolder, imageFile)
        orthoPath = os.path.join(orthoFolder, orthoFile)
        
        outputCamFile = os.path.join(cameraFolder,
                                     icebridge_common.getCameraFileName(imageFile))
        outputFiles.append(outputCamFile)
        if os.path.exists(outputCamFile):
            logger.info("File exists, skipping: " + outputCamFile)
            os.system("rm -f " + outputCamFile + "*-log-*") # wipe logs
            continue

        # Determine which input camera file will be used for this frame
        inputCamFile = getCalibrationFileForFrame(cameraLookupPath, inputCalFolder,
                                                  frame, yyyymmdd, site)

        # Add ortho2pinhole command to the task pool
        taskHandles.append(pool.apply_async(cameraFromOrthoWrapper, 
                                            (inputPath, orthoPath, inputCamFile,
                                             estimatedCameraPath, outputCamFile,
                                             refDemPath, simpleCameras, numThreads)))

    # Wait for all the tasks to complete
    logger.info('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, logger, interactive=False,
                                                     quitKey='q')

    # All tasks should be finished
    icebridge_common.stopTaskPool(pool)
    logger.info('Finished ortho processing.')

    # Run a check to see if we got all the output files
    for f in outputFiles:
        if not os.path.exists(f):
            return False
    return True

def getCameraModelsFromNav(imageFolder, orthoFolder, 
                           inputCalFolder, navFolder, navCameraFolder,
                           startFrame, stopFrame, 
                           logger):
    '''Given the folder containing navigation files, generate an
       estimated camera model for each file.'''
   
    # Note: Currently these output files DO NOT contain accurate intrinsic parameters!

    logger.info("Get camera models from nav.")
    
    # All the work is done by the separate file.
    cmd = [imageFolder, orthoFolder, inputCalFolder, navFolder, navCameraFolder,
           str(startFrame), str(stopFrame)]
    logger.info("camera_models_from_nav.py " + " ".join(cmd))
    
    if (camera_models_from_nav.main(cmd) < 0):
        raise Exception('Error generating camera models from nav!')

def convertLidarDataToCsv(lidarFolder, startFrame, stopFrame, 
                          skipValidate, logger):
    '''Make sure all lidar data is available in a readable text format.
       Returns false if any files failed to convert.'''

    logger.info('Converting LIDAR files...')

    lidarIndexPath = icebridge_common.csvIndexFile(lidarFolder)
    (frameDict, urlDict) = icebridge_common.readIndexFile(lidarIndexPath)

    if not skipValidate:
        validFilesList = icebridge_common.validFilesList(os.path.dirname(lidarFolder),
                                                         startFrame, stopFrame)
        validFilesSet = set()
        validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
        numInitialValidFiles = len(validFilesSet)

    convDict = {}
    
    # Loop through all files in the folder
    badFiles = False
    for frame in sorted(frameDict.keys()):

        f = frameDict[frame]
        extension = icebridge_common.fileExtension(f)
        
        # Only interested in a few file types
        if (extension != '.qi') and (extension != '.hdf5') and (extension != '.h5'):
            convDict[frame] = f # these are already in plain text
            continue

        convDict[frame] = os.path.splitext(f)[0] + '.csv'
        outputPath = os.path.join(lidarFolder, convDict[frame])

        # Handle paths
        fullPath = os.path.join(lidarFolder, f)
        if not os.path.exists(fullPath):
            logger.info("Cannot convert missing file: " + fullPath)
            continue

        # If the input is invalid, wipe both it, its xml, and the output
        # Hopefully there will be a subsquent fetch step where it will get
        # refetched.
        if not icebridge_common.hasValidChkSum(fullPath, logger):
            logger.info("Will wipe invalid file: " + fullPath)
            xmlFile = icebridge_common.xmlFile(fullPath)
            os.system('rm -f ' + fullPath) # will not throw
            os.system('rm -f ' + xmlFile) # will not throw
            os.system('rm -f ' + outputPath) # will not throw
            badFiles = True
            continue

        # Skip existing valid files
        if skipValidate:
            if os.path.exists(outputPath):
                logger.info("File exists, skipping: " + outputPath)
                continue
        else:
            if outputPath in validFilesSet and os.path.exists(outputPath):
                #logger.info('Previously validated: ' + outputPath) # verbose
                continue
            if icebridge_common.isValidLidarCSV(outputPath):
                #logger.info("File exists and is valid, skipping: " + outputPath)
                continue
        
        # Call the conversion
        logger.info("Process " + fullPath)
        extract_icebridge_ATM_points.main([fullPath])
        
        # Check the result
        if not icebridge_common.isValidLidarCSV(outputPath):
            logger.error('Failed to parse LIDAR file, will wipe: ' + outputPath)
            os.system('rm -f ' + outputPath) # will not throw            
            badFiles = True
        else:
            if not skipValidate:
                validFilesSet.add(outputPath) # mark it as validated
            
    convLidarFile = icebridge_common.getConvertedLidarIndexFile(lidarFolder)
    if not os.path.exists(convLidarFile):
        logger.info("Writing: " + convLidarFile)
        icebridge_common.writeIndexFile(convLidarFile, convDict, {})
        
    if not skipValidate:
        # Write to disk the list of validated files, but only if new
        # validations happened.  First re-read that list, in case a
        # different process modified it in the meantime, such as if two
        # managers are running at the same time.
        numFinalValidFiles = len(validFilesSet)
        if numInitialValidFiles != numFinalValidFiles:
            validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList,
                                                                          validFilesSet)
            icebridge_common.writeValidFilesList(validFilesList, validFilesSet)

    return (not badFiles)

def pairLidarFiles(lidarFolder, skipValidate, logger):
    '''For each pair of lidar files generate a double size point cloud.
       We can use these later since they do not have any gaps between adjacent files.'''
    
    logger.info('Generating lidar pairs...')

    # Create the output folder
    pairedFolder = icebridge_common.getPairedLidarFolder(lidarFolder)
    os.system('mkdir -p ' + pairedFolder)

    convLidarFile = icebridge_common.getConvertedLidarIndexFile(lidarFolder)
    if not os.path.exists(convLidarFile):
        raise Exception("Missing file: " + convLidarFile)

    (lidarDict, dummyUrlDict) = icebridge_common.readIndexFile(convLidarFile)
    lidarExt = ''
    for frame in lidarDict:
        lidarExt = icebridge_common.fileExtension(lidarDict[frame])

    numLidarFiles = len(lidarDict.keys())

    pairedDict = {}
    
    # Loop through all pairs of csv files in the folder    
    badFiles = False
    lidarKeys = sorted(lidarDict.keys())
    for i in range(len(lidarKeys)-1):
        
        thisFile = lidarDict[lidarKeys[i  ]]
        nextFile = lidarDict[lidarKeys[i+1]]

        date2, time2 = icebridge_common.parseTimeStamps(nextFile)
        
        # Record the name with the second file
        # - More useful because the time for the second file represents the middle of the file.
        outputName = icebridge_common.lidar_pair_prefix() + date2 +'_'+ time2 + lidarExt

        pairedDict[lidarKeys[i]] = outputName
        
        # Handle paths
        path1      = os.path.join(lidarFolder, thisFile)
        path2      = os.path.join(lidarFolder, nextFile)
        outputPath = os.path.join(pairedFolder, outputName)

        if not os.path.exists(path1) or not os.path.exists(path2):
            logger.info("Cannot create " + outputPath + " as we are missing its inputs")
            # If the inputs are missing, but the output is there, most likely it is corrupt.
            # Wipe it. Hopefully a subsequent fetch and convert step will bring it back.
            if os.path.exists(outputPath):
                logger.info("Wiping: " + outputPath)
                os.system('rm -f ' + outputPath) # will not throw
                badFiles = True
            continue
        
        # Skip existing valid files
        if skipValidate:
            if os.path.exists(outputPath):
                logger.info("File exists, skipping: " + outputPath)
                continue
        else:
            if icebridge_common.isValidLidarCSV(outputPath):
                #logger.info("File exists and is valid, skipping: " + outputPath)
                continue

        # Concatenate the two files
        cmd1 = 'cat ' + path1 + ' > ' + outputPath
        cmd2 = 'tail -n +2 -q ' + path2 + ' >> ' + outputPath
        logger.info(cmd1)
        p        = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True)
        out, err = p.communicate()
        logger.info(cmd2)
        p        = subprocess.Popen(cmd2, stdout=subprocess.PIPE, shell=True)
        out, err = p.communicate()

        if not icebridge_common.isValidLidarCSV(outputPath):
            logger.error('Failed to generate merged LIDAR file, will wipe: ' + outputPath)
            os.system('rm -f ' + outputPath) # will not throw
            badFiles = True

    pairedLidarFile = icebridge_common.getPairedIndexFile(pairedFolder)
    if not os.path.exists(pairedLidarFile):
        logger.info("Writing: " + pairedLidarFile)
        icebridge_common.writeIndexFile(pairedLidarFile, pairedDict, {})

    return (not badFiles)


