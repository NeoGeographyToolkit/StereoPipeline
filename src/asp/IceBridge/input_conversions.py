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

import icebridge_common, fetch_icebridge_data, process_icebridge_run, extract_icebridge_ATM_points
import asp_system_utils, asp_alg_utils, asp_geo_utils

asp_system_utils.verify_python_version_is_supported()

def getJpegDateTime(filepath):
    '''Get the date and time from a raw jpeg file.'''
    
    # TODO: For some files it is probably in the name.
    
    # Use this tool to extract the metadata
    cmd      = [asp_system_utils.which('gdalinfo'), filepath]
    p        = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    out, err = p.communicate()
    
    lines = out.split('\n')

    for line in lines:
        if 'EXIF_DateTimeOriginal' not in line:
            continue
        parts = line.replace('=',' ').split()
        dateString = parts[1].strip().replace(':','')
        timeString = parts[2].strip().replace(':','')
        
        return (dateString, timeString)

    raise Exception('Failed to read date/time from file: ' + filepath)

def convertJpegs(jpegFolder, imageFolder, startFrame, stopFrame, skipValidate, logger):
    '''Convert jpeg images from RGB to single channel.
       Returns false if any files failed.'''

    badFiles = False
    
    logger.info('Converting input images to grayscale...')

    # Loop through all the input images
    os.system('mkdir -p ' + imageFolder)
    jpegFiles = os.listdir(jpegFolder)
    for jpegFile in jpegFiles:
        
        inputPath = os.path.join(jpegFolder, jpegFile)
        
        # Skip non-image files
        ext = os.path.splitext(jpegFile)[1]
        if ext != '.JPG':
            continue
        
        # Only deal with frames in range
        frame = icebridge_common.getFrameNumberFromFilename(inputPath)
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue

        # Make sure the timestamp and frame number are in the output file name
        (dateStr, timeStr) = getJpegDateTime(inputPath)
        outputName = ('DMS_%s_%s_%05d.tif') % (dateStr, timeStr, frame)
        outputPath = os.path.join(imageFolder, outputName)

        # Skip existing valid files
        if skipValidate:
            if os.path.exists(outputPath):
                logger.info("File exists, skipping: " + outputPath)
                continue
        else:
            if icebridge_common.isValidImage(outputPath):
                logger.info("File exists and is valid, skipping: " + outputPath)
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

        if not os.path.exists(outputPath):
            badFiles = True
            logger.error('Failed to convert jpeg file: ' + jpegFile)

        # Check for corrupted files
        if error is not None:
            output += error
        m = re.match("^.*?premature\s+end", output, re.IGNORECASE|re.MULTILINE|re.DOTALL)
        if m:
            logger.error("Wiping bad files: " + inputPath + " and " + outputPath +'\n'
                         + output)
            if os.path.exists(inputPath ): os.remove(inputPath)
            if os.path.exists(outputPath): os.remove(outputPath)
            badFiles = True
            
    if badFiles:
        logger.error("Converstion of JPEGs failed. If any files were corrupted, " +
                     "they were removed, and need to be re-fetched.")
    
    return (not badFiles)
            
def correctFireballDems(demFolder, correctedDemFolder, startFrame, stopFrame, isNorth,
                        skipValidate, logger):
    '''Fix the header problem in Fireball DEMs'''

    logger.info('Correcting Fireball DEMs ...')

    # Loop through all the input images
    os.system('mkdir -p ' + correctedDemFolder)
    demFiles = os.listdir(demFolder)
    badFiles = False
    for demFile in demFiles:

        # Skip other files
        inputPath = os.path.join(demFolder, demFile)
        if not icebridge_common.isDEM(inputPath):
            continue

        # Skip if outside the frame range
        frame = icebridge_common.getFrameNumberFromFilename(demFile)
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue

        # Make sure the timestamp and frame number are in the output file name
        outputPath = os.path.join(correctedDemFolder, os.path.basename(inputPath))

        # Skip existing valid files
        if skipValidate:
            if os.path.exists(outputPath):
                logger.info("File exists, skipping: " + outputPath)
                continue
        else:
            if icebridge_common.isValidImage(outputPath):
                logger.info("File exists and is valid, skipping: " + outputPath)
                continue
        
        # Run the correction script
        execPath = asp_system_utils.which('correct_icebridge_l3_dem')
        cmd = (('%s %s %s %d') %
               (execPath, inputPath, outputPath, isNorth))
        logger.info(cmd)
        os.system(cmd)
        
        # Check if the output file is good
        if not icebridge_common.isValidImage(outputPath):
            logger.error('Failed to convert dem file: ' + demFile)
            badFiles = True

    return not badFiles
            

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

def cameraFromOrthoWrapper(inputPath, orthoPath, inputCamFile, outputCamFile,
                           refDemPath, numThreads):
    '''Generate a camera model from a single ortho file'''

    # If the call fails, try it again with different IP algorithm options to see
    #  if we can get it to work.
    IP_OPTIONS = ['1', '0', '2']
   
    MIN_IP     = 10  # Require more IP to make sure we don't get bogus camera models
    DESIRED_IP = 100 # If we don't hit this number, try other methods before taking the best one.

    bestIpCount = 0
    tempFilePath = outputCamFile + '_temp' # Used to hold the current best result
    for ip_option in IP_OPTIONS:

        # Call ortho2pinhole command
        ortho2pinhole = asp_system_utils.which("ortho2pinhole")
        cmd = (('%s %s %s %s %s --reference-dem %s --threads %d --ip-detect-method %s --minimum-ip %d') % (ortho2pinhole, inputPath, orthoPath, inputCamFile, outputCamFile, refDemPath, numThreads, ip_option, MIN_IP))

        # Use a print statement as the logger fails from multiple processes
        print(cmd)

        p = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
        textOutput, err = p.communicate()
        p.wait()
        print(textOutput)
        
        if not os.path.exists(outputCamFile): # Keep trying if no output file produced
            continue
        
        # Check the number of IP used
        m = re.findall(r"Init model with (\d+) points", textOutput)
        if len(m) != 1: # An unknown error occurred, move on.
            continue
        numPoints = int(m[0])
        if numPoints >= DESIRED_IP: # Got a lot of points, quit
            break
        if numPoints > bestIpCount: # Got some points but not many, try other options 
            bestIpCount = numPoints #  to see if we can beat this result.
            shutil.move(outputCamFile, tempFilePath)

    if numPoints < DESIRED_IP: # If we never got the desired # of points
        shutil.move(tempFilePath, outputCamFile) # Use the camera file with the most points found
    
    if not os.path.exists(outputCamFile):
        # This function is getting called from a pool, so just log the failure.
        print('Failed to convert ortho file: ' + orthoPath)

    # I saw this being recommended, to dump all print statements in the current task
    sys.stdout.flush()
                
    # TODO: Clean up the .gcp file?


def getCameraModelsFromOrtho(imageFolder, orthoFolder, inputCalFolder,
                             cameraLookupPath, yyyymmdd, site,
                             refDemPath, cameraFolder, 
                             startFrame, stopFrame,
                             numProcesses, numThreads, logger):
    '''Generate camera models from the ortho files.
       Returns false if any files were not generated.'''
    
    logger.info('Generating camera models from ortho images...')
    
    imageFiles = icebridge_common.getTifs(imageFolder)
    orthoFiles = icebridge_common.getTifs(orthoFolder)
    
    # Make a dictionary of ortho files by frame
    orthoFrames = {}
    for f in orthoFiles:
        frame = icebridge_common.getFrameNumberFromFilename(f)
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue
        orthoFrames[frame] = f

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
        orthoFile = orthoFrames[frame]
        
        # Check output file
        inputPath     = os.path.join(imageFolder, imageFile)
        orthoPath     = os.path.join(orthoFolder, orthoFile)
        outputCamFile = os.path.join(cameraFolder,
                                     icebridge_common.getCameraFileName(imageFile))
        outputFiles.append(outputCamFile)
        if os.path.exists(outputCamFile):
            logger.info("File exists, skipping: " + outputCamFile)
            continue

        # Determine which input camera file will be used for this frame
        inputCamFile = getCalibrationFileForFrame(cameraLookupPath, inputCalFolder,
                                                  frame, yyyymmdd, site)

        # Experimental. Using the lidar file instead of the datum works better in the open water.
        #lidarFile = icebridge_common.findMatchingLidarFile(imageFile, lidarFolder)
        #isSouth = icebridge_common.checkSite(site)
        #lidarDemFile = lidarFile[:-4] + '-DEM.tif'
        #if os.path.exists(lidarDemFile):
        #    refDemPath = lidarDemFile
        #else:
        #    projString = icebridge_common.getProjection(isSouth)
        #    lidarCsvFormatString = icebridge_common.getLidarCsvFormat(lidarFile)
        #    cmd = "point2dem --tr 1 --search-radius-factor 4 --t_srs " + projString + ' --csv-format ' + lidarCsvFormatString + ' --datum wgs84 ' + lidarFile
        #    suppressOutput = False
        #    redo = False
        #    logger.info(cmd)
        #    asp_system_utils.executeCommand(cmd, lidarDemFile, suppressOutput, redo)
        #    
        #    if os.path.exists(lidarDemFile):
        #        refDemPath = lidarDemFile 
        #logger.info("Using dem path: " + refDemPath)
        
        # Add ortho2pinhole command to the task pool
        taskHandles.append(pool.apply_async(cameraFromOrthoWrapper, 
                                            (inputPath, orthoPath, inputCamFile,
                                             outputCamFile, refDemPath, numThreads)))

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

def convertLidarDataToCsv(lidarFolder, logger):
    '''Make sure all lidar data is available in a readable text format.
       Returns false if any files failed to convert.'''

    logger.info('Converting LIDAR files...')

    lidarIndexPath = icebridge_common.csvIndexFile(lidarFolder)
    (frameDict, urlDict) = icebridge_common.readIndexFile(lidarIndexPath)
    
    # Loop through all files in the folder
    badFiles = False
    for frame in frameDict:

        f = frameDict[frame]
        extension = icebridge_common.fileExtension(f)
        
        # Only interested in a few file types
        if (extension != '.qi') and (extension != '.hdf5') and (extension != '.h5'):
           continue

        # Handle paths
        fullPath = os.path.join(lidarFolder, f)
        if not os.path.exists(fullPath):
            logger.info("Cannot convert missing file: " + fullPath)
            continue
        
        outputPath = os.path.join(lidarFolder, os.path.splitext(f)[0]+'.csv')
        if icebridge_common.isValidLidarCSV(outputPath):
            logger.info("File exists and is valid, skipping: " + outputPath)
            continue
        
        # Call the conversion
        logger.info("Process " + fullPath)
        extract_icebridge_ATM_points.main([fullPath])
        
        # Check the result
        if not icebridge_common.isValidLidarCSV(outputPath):
            logger.error('Failed to parse LIDAR file: ' + fullPath)
            badFiles = True
            
    return not badFiles

def pairLidarFiles(lidarFolder, logger):
    '''For each pair of lidar files generate a double size point cloud.
       We can use these later since they do not have any gaps between adjacent files.'''
    
    logger.info('Generating lidar pairs...')

    # Create the output folder
    pairFolder = os.path.join(lidarFolder, 'paired')
    os.system('mkdir -p ' + pairFolder)

    (lidarFiles, lidarExt, isLVIS) = icebridge_common.lidarFiles(lidarFolder)
    
    numLidarFiles = len(lidarFiles)
    
    # Loop through all pairs of csv files in the folder    
    badFiles = False
    for i in range(0, numLidarFiles-1):

        thisFile = lidarFiles[i  ]
        nextFile = lidarFiles[i+1]

        date2, time2 = icebridge_common.parseTimeStamps(nextFile)
        
        # Record the name with the second file
        # - More useful because the time for the second file represents the middle of the file.
        outputName = icebridge_common.lidar_pair_prefix() + date2 +'_'+ time2 + lidarExt

        # Handle paths
        path1      = os.path.join(lidarFolder, thisFile)
        path2      = os.path.join(lidarFolder, nextFile)
        outputPath = os.path.join(pairFolder, outputName)
        if icebridge_common.isValidLidarCSV(outputPath):
            logger.info("Valid lidar file: " + outputPath)
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
            logger.error('Failed to generate merged LIDAR file: ' + outputPath)
            badFiles = True
            
    return badFiles


