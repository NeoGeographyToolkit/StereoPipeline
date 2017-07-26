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
    cmd      = ['gdalinfo', filepath]
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

def convertJpegs(jpegFolder, imageFolder, startFrame, stopFrame):
    '''Convert jpeg images from RGB to single channel.
       Returns false if any files failed.'''

    badFiles = False
    
    logger = logging.getLogger(__name__)
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
        
        # Make sure the timestamp and frame number are in the output file name
        (dateStr, timeStr) = getJpegDateTime(inputPath)
        frame = icebridge_common.getFrameNumberFromFilename(inputPath)
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue
        outputName = ('DMS_%s_%s_%05d.tif') % (dateStr, timeStr, frame)
        outputPath = os.path.join(imageFolder, outputName)

        # Skip existing files
        if os.path.exists(outputPath):
            logger.info("File exists, skipping: " + outputPath)
            continue
        
        # Use ImageMagick tool to convert from RGB to grayscale
        cmd = (('convert %s -colorspace Gray %s') % (inputPath, outputPath))
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
        m = re.match("^.*?premature\s+end", output, re.IGNORECASE)
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
            
def correctFireballDems(demFolder, correctedDemFolder, startFrame, stopFrame, isNorth):
    '''Fix the header problem in Fireball DEMs'''

    logger = logging.getLogger(__name__)
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

        # Skip existing files
        if os.path.exists(outputPath):
            logger.info("File exists, skipping: " + outputPath)
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

def cameraFromOrthoWrapper(inputPath, orthoPath, inputCamFile, outputCamFile, \
                           refDemPath, numThreads):
    '''Generate a camera model from a single ortho file'''

    logger = logging.getLogger(__name__)

    # Call ortho2pinhole command
    ortho2pinhole = asp_system_utils.which("ortho2pinhole")
    cmd = (('%s %s %s %s %s --reference-dem %s --threads %d') % (ortho2pinhole, inputPath, orthoPath,
                                 inputCamFile, outputCamFile, refDemPath, numThreads))
    logger.info(cmd)
    os.system(cmd)
    
    if not os.path.exists(outputCamFile):
        # This function is getting called from a pool, so just log the failure.
        logger.error('Failed to convert ortho file: ' + orthoFile)
            
    # TODO: Clean up the .gcp file?


def getCameraModelsFromOrtho(imageFolder, orthoFolder, inputCalFolder,
                             cameraLookupPath, yyyymmdd, site,
                             refDemPath, cameraFolder, 
                             startFrame, stopFrame,
                             numProcesses, numThreads):
    '''Generate camera models from the ortho files.
       Returns false if any files were not generated.'''
    
    logger = logging.getLogger(__name__)
    logger.info('Generating camera models from ortho images...')
    
    imageFiles = os.listdir(imageFolder)
    orthoFiles = icebridge_common.getTifs(orthoFolder)
    
    # Make a dictionary of ortho files by frame
    orthoFrames = {}
    for f in orthoFiles:
        frame = icebridge_common.getFrameNumberFromFilename(f)
        if not ( (frame >= startFrame) and (frame <= stopFrame) ): continue
        orthoFrames[frame] = f

    logger.info('Starting ortho processing pool with ' + str(numProcesses) +' processes.')
    pool = multiprocessing.Pool(numProcesses)

    # Loop through all input images
    taskHandles = []
    outputFiles = []
    for imageFile in imageFiles:
        
        # Skip non-image files (including junk from stereo_gui)
        ext = os.path.splitext(imageFile)[1]
        if (ext != '.tif') or ('_sub' in imageFile):
            continue

        # Get associated orthofile
        frame = icebridge_common.getFrameNumberFromFilename(imageFile)
        if not ( (frame >= startFrame) and (frame <= stopFrame) ): continue
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
               
        # Add ortho2pinhole command to the task pool
        taskHandles.append(pool.apply_async(cameraFromOrthoWrapper, 
                                            (inputPath, orthoPath, inputCamFile,
                                             outputCamFile, refDemPath, numThreads)))

    # Wait for all the tasks to complete
    logger.info('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, interactive=False,
                                                     quitKey='q')

    # All tasks should be finished, clean up the processing pool
    logger.info('Cleaning up the ortho processing pool...')
    icebridge_common.stopTaskPool(pool)
    logger.info('Finished cleaning up the ortho processing pool')

    # Run a check to see if we got all the output files
    for f in outputFiles:
        if not os.path.exists(f):
            return False
    return True


def convertLidarDataToCsv(lidarFolder):
    '''Make sure all lidar data is available in a readable text format.
       Returns false if any files failed to convert.'''

    logger = logging.getLogger(__name__)
    logger.info('Converting LIDAR files...')
    
    # Loop through all files in the folder
    allFiles = os.listdir(lidarFolder)
    badFiles = False
    for f in allFiles:
        extension = icebridge_common.fileExtension(f)
        
        # Only interested in a few file types
        if (extension != '.qi') and (extension != '.hdf5') and (extension != '.h5'):
           continue

        # Handle paths
        fullPath   = os.path.join(lidarFolder, f)
        outputPath = os.path.join(lidarFolder, os.path.splitext(f)[0]+'.csv')
        if os.path.exists(outputPath):
            continue
        
        # Call the conversion
        extract_icebridge_ATM_points.main([fullPath])
        
        # Check the result
        if not os.path.exists(outputPath):
            logger.error('Failed to parse LIDAR file: ' + fullPath)
            badFiles = True
            
    return not badFiles


def pairLidarFiles(lidarFolder):
    '''For each pair of lidar files generate a double size point cloud.
       We can use these later since they do not have any gaps between adjacent files.'''
    
    logger = logging.getLogger(__name__)
    logger.info('Generating lidar pairs...')
    
    # Create the output folder
    pairFolder = os.path.join(lidarFolder, 'paired')
    os.system('mkdir -p ' + pairFolder)
    
    # All files in the folder
    allFiles = os.listdir(lidarFolder)

    # See based on existing files if we are dealing with LVIS
    isLVIS = False
    for f in allFiles:
        m = re.match("^.*?ILVIS.*?\d+\.TXT", f, re.IGNORECASE)
        if m:
            isLVIS = True
            
    # Get just the files we converted to csv format or plain text LVIS files
    csvFiles = []    
    for f in allFiles:
        extension = os.path.splitext(f)[1]
        if 'html.csv' in f:
            continue # skip index.html.csv
        if (not isLVIS and extension == '.csv') or (isLVIS and extension == '.TXT'):
           csvFiles.append(f)
    csvFiles.sort()
    numCsvFiles = len(csvFiles)

    outExt = '.csv'
    if isLVIS:
        outExt = '.TXT'
    
    # Loop through all pairs of csv files in the folder    
    badFiles = False
    for i in range(0,numCsvFiles-2):

        thisFile = csvFiles[i  ]
        nextFile = csvFiles[i+1]

        #date1, time1 = icebridge_common.parseTimeStamps(thisFile)
        date2, time2 = icebridge_common.parseTimeStamps(nextFile)
        
        # Record the name with the second file
        # - More useful because the time for the second file represents the middle of the file.
        outputName = 'LIDAR_PAIR_' + date2 +'_'+ time2 + outExt

        # Handle paths
        path1      = os.path.join(lidarFolder, thisFile)
        path2      = os.path.join(lidarFolder, nextFile)
        outputPath = os.path.join(pairFolder, outputName)
        if os.path.exists(outputPath):
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
               
        if not os.path.exists(outputPath):
            logger.error('Failed to generate merged LIDAR file: ' + outputPath)
            badFiles = True
            
    return badFiles


