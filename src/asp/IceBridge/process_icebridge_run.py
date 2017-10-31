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

# Process an entire run of icebrige images. Multiple runs will be started in parallel.

# All the image, camera, and lidar files must have date and time stamps,
# like the orthoimages and the Fireball DEMs. As such, raw image
# files must be renamed to be similar to the ortho image files.
# No other files must be present in those directries.
# Image files must be single-channel, so use for example gdal_translate -b 1.

import os, sys, optparse, datetime, multiprocessing, time, logging, subprocess, re
import traceback

# We must import this explicitly, it is not imported by the top-level
# multiprocessing module.
import multiprocessing.pool

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools
toolspath = os.path.abspath(basepath + '/../Tools')  # ASP Tools
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, toolspath)

import icebridge_common, process_icebridge_batch
import asp_system_utils, asp_alg_utils, asp_geo_utils
asp_system_utils.verify_python_version_is_supported()

# TODO: Fix the logging!
logging.info('DEBUG')
logger = logging.getLogger(__name__)

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath   + os.pathsep + os.environ["PATH"]

# This is the file name in the output folder where batch commands will
#  be written to with the --log-batches option.
BATCH_COMMAND_LOG_FILE = 'batch_commands_log.txt'

def processBatch(imageCameraPairs, lidarFolder, referenceDem, skipInterval, outputFolder, extraOptions, 
                 outputResolution, stereoArgs, batchNum, batchLogPath=''):
    '''Processes a batch of images at once'''

    suppressOutput = False
    redo           = False

    argString = ''
    for pair in imageCameraPairs:
        argString += pair[0] + ' '
    for pair in imageCameraPairs:
        argString += pair[1] + ' '

    # Just set the options and call the pair python tool.
    # We can try out bundle adjustment for intrinsic parameters here.
    cmd = ('--lidar-folder %s --reference-dem %s --stereo-image-interval %d --dem-resolution %f' \
           ' --output-folder %s %s %s --stereo-arguments ' 
           % (lidarFolder, referenceDem, skipInterval, outputResolution, outputFolder, argString, extraOptions))
        
    if batchLogPath:
        # With this option we just log the commands to a text file
        # - Setting this option limits to one process so there will be only one 
        #   simultaneous file writer.
        with open(batchLogPath, 'a') as f:
            f.write('python ' + icebridge_common.fullPath('process_icebridge_batch.py') + ' ' + \
                    cmd +'"'+ stereoArgs +'"\n')
        return
    
    try:
        args = cmd.split()
        args += (stereoArgs.strip(),) # Make sure this is properly passed
        process_icebridge_batch.main(args)
    except Exception as e:
        logger.error('Batch processing failed!\n' + str(e) +
                     traceback.print_exc())

def getImageSpacing(orthoFolder, availableFrames, startFrame, stopFrame, forceAllFramesInRange):
    '''Find a good image stereo spacing interval that gives us a good
       balance between coverage and baseline width.
       Also detect all frames where this is a large break after the current frame.'''

    logger.info('Computing optimal image stereo interval...')

    ## With very few cameras this is the only possible way to process them
    #if len(availableFrames) < 3 and not forceAllFramesInRange:
    #    return ([], {}) # No skip, no breaks

    # Retrieve a list of the ortho files
    orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
    if not os.path.exists(orthoIndexPath):
        raise Exception("Error: Missing ortho index file: " + orthoIndexPath + ".")
    (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath,
                                                                    prependFolder = True)

    # From the dictionary create a sorted list of ortho files in the frame range
    breaks     = []
    largeSkips = {}
    orthoFiles = []
    for frame in sorted(orthoFrameDict.keys()):

        # Only process frames within the range
        if not ( (frame >= startFrame) and (frame <= stopFrame) ):
            continue

        orthoPath = orthoFrameDict[frame]
        frame     = icebridge_common.getFrameNumberFromFilename(orthoPath)
        
        if not forceAllFramesInRange:
            if frame not in availableFrames: # Skip frames we did not compute a camera for
                continue
        
        orthoFiles.append(orthoPath)
        
    orthoFiles.sort()
    numOrthos = len(orthoFiles)

    # First load whatever boxes are there
    projectionIndexFile = icebridge_common.projectionBoundsFile(os.path.dirname(orthoFolder))
    logger.info("Reading: " + projectionIndexFile)
    boundsDict = icebridge_common.readProjectionBounds(projectionIndexFile)
    
    # Get the bounding box and frame number of each ortho image
    logger.info('Loading bounding boxes...')
    frames = []
    updatedBounds = False # will be true if some computation got done
    count = 0
    for i in range(0, numOrthos):

        # This can be slow, so add a progress dialong
        count = count + 1
        if (count - 1) % 1000 == 0:
            logger.info('Progress: ' + str(count) + '/' + str(numOrthos))

        thisFrame    = icebridge_common.getFrameNumberFromFilename(orthoFiles[i])
        if thisFrame not in boundsDict:
            imageGeoInfo   = asp_geo_utils.getImageGeoInfo(orthoFiles[i], getStats=False)
            thisBox        = imageGeoInfo['projection_bounds']
            boundsDict[thisFrame] = thisBox
            updatedBounds = True
            
        frames.append(thisFrame)

    # Read this file again, in case some other process modified it in the meantime.
    # This won't happen in production mode, but can during testing with partial sequences.
    boundsDictRecent = icebridge_common.readProjectionBounds(projectionIndexFile)
    for frame in sorted(boundsDictRecent.keys()):
        if not frame in boundsDict.keys():
            boundsDict[frame] = boundsDictRecent[frame]
            updatedBounds = True

    # Save the bounds. There is always the danger that two processes will
    # do that at the same time, but this is rare, as hopefully we get here
    # only once from the manager. It is not a big loss if this file gets messed up.
    if updatedBounds:
        logger.info("Writing: " + projectionIndexFile)
        icebridge_common.writeProjectionBounds(projectionIndexFile, boundsDict)
        
    # Since we are only comparing the image bounding boxes, not their exact corners,
    #  these ratios are only estimates.
    MAX_RATIO   = 0.85  # Increase skip until we get below this...
    MIN_RATIO   = 0.75  # ... but don't go below this value!
    NOTRY_RATIO = 0.35 # Don't bother with overlap amounts less than this

    def getBboxArea(bbox):
        '''Return the area of a bounding box in form of (minX, maxX, minY, maxY)'''
        width  = bbox[1] - bbox[0]
        height = bbox[3] - bbox[2]
        if (width < 0) or (height < 0):
            return 0
        return width*height

    # Iterate over the frames and find the best stereo frame for each    
    for i in range(0, numOrthos-1):
    
        thisFrame = frames[i]
        thisBox   = boundsDict[thisFrame]
        thisArea  = getBboxArea(thisBox)
        interval  = 1
        
        while(True):
            
            # Compute intersection area between this and next image

            nextFrame = frames[i+interval]
            nextBox   = boundsDict[nextFrame]
            intersect = [max(thisBox[0], nextBox[0]), # Min X
                         min(thisBox[1], nextBox[1]), # Max X
                         max(thisBox[2], nextBox[2]), # Min Y
                         min(thisBox[3], nextBox[3])] # Max Y
            area      = getBboxArea(intersect)
            ratio = 0
            if area > 0:
                ratio = area / thisArea
            
            #print str(thisFrame) +', ' + str(nextFrame) + ' -> ' + str(ratio)
            if interval == 1: # Cases for the smallest interval...
                if ratio < NOTRY_RATIO:
                    breaks.append(thisFrame) # No match for this frame
                    logger.info('Detected large break after frame ' + str(thisFrame))
                    break
                if ratio < MIN_RATIO:
                    break # No reason to try increasing skip amounts for this frame
            else: # interval > 1
                if ratio < MIN_RATIO: # Went too small, walk back the interval.
                    interval = interval - 1
                    break
                    
            if ratio > MAX_RATIO: # Too much overlap, increase interval
                interval = interval + 1
            else: # Overlap is fine, keep this interval.
                break

            # Handle the case where we go past the end of frames looking for a match.
            if i+interval >= len(frames):
                interval = interval - 1
                break
        
        if interval > 1: # Only record larger than normal intervals.
            largeSkips[thisFrame] = interval

    
    logger.info('Detected ' + str(len(breaks)) + ' breaks in image coverage.')
    logger.info('Detected ' + str(len(largeSkips)) + ' images with interval > 1.')

    return (breaks, largeSkips)

def list_median(lst):
    sortedLst = sorted(lst)
    lstLen = len(lst)
    index = (lstLen - 1) // 2
    
    if (lstLen % 2):
        return sortedLst[index]
    else:
        return (sortedLst[index] + sortedLst[index + 1])/2.0
    
def getRunMedianGsd(imageCameraPairs, referenceDem, isSouth, frameSkip=1):
    '''Compute the mean GSD of the images across the run'''

    logger.info('Computing mean input image GSD...')

    projString = icebridge_common.getProjString(isSouth)

    # Iterate through the input pairs
    numPairs  = len(imageCameraPairs)
    #medianGsd   = 0.0
    #numChecks = 0.0
    gsdVals = []
    for i in range(0, numPairs, frameSkip):
        pair = imageCameraPairs[i]
        
        # Average in the GSD for each file we can process
        gsd, bounds = icebridge_common.getCameraGsdAndBoundsRetry(pair[0], pair[1], logger, referenceDem, projString)
        # TODO Move on to the next file on failure

        gsdVals.append(gsd)
        #medianGsd   += gsd
        #numChecks += 1.0

    #medianGsd = medianGsd / numChecks
    medianGsd = list_median(gsdVals)
    
    logger.info('Computed input image mean GSD: ' + str(medianGsd))
    return medianGsd
        

class NoDaemonProcess(multiprocessing.Process):
    # make 'daemon' attribute always return False
    def _get_daemon(self):
        return False
    def _set_daemon(self, value):
        pass
    daemon = property(_get_daemon, _set_daemon)

# A pool that allows each process to have its own pool of subprocesses.
# This should be used with care.
class NonDaemonPool(multiprocessing.pool.Pool):
    Process = NoDaemonProcess


def main(argsIn):

    try:
        usage = '''usage: process_icebridge_run.py <image_folder> <camera_folder>
                      <lidar_folder> <output_folder>'''
                      
        parser = optparse.OptionParser(usage=usage)

        # Data selection optios
        parser.add_option('--start-frame', dest='startFrame', default=-1,
                          type='int', help='The frame number to start processing with.')
        parser.add_option('--stop-frame', dest='stopFrame', default=-1,
                          type='int', help='The frame number to finish processing with.')        
        parser.add_option('--south', action='store_true', default=False, dest='isSouth',  
                          help='MUST be set if the images are in the southern hemisphere.')

        # Processing options
        parser.add_option('--stereo-arguments', dest='stereoArgs', default='',
                          help='Additional argument string to be passed to the stereo command.')

        parser.add_option('--bundle-length', dest='bundleLength', default=2,
                          type='int', help='Number of images to bundle adjust and process at once.')
        parser.add_option('--image-stereo-interval', dest='imageStereoInterval', default=None,
                          type='int', help='Advance this many frames to get the stereo pair.  Default is auto-calculate')

        parser.add_option('--solve-intrinsics', action='store_true', default=False,
                          dest='solve_intr',  
                          help='If to float the intrinsics params.')

        #parser.add_option('--dem-resolution', dest='demResolution', default=0.4,
        #                  type='float', help='Generate output DEMs at this resolution.')

        parser.add_option('--max-displacement', dest='maxDisplacement', default=20,
                          type='float', help='Max displacement value passed to pc_align.')


        # Performance options
        parser.add_option('--num-processes', dest='numProcesses', default=1,
                          type='int', help='The number of simultaneous processes to run.')
        parser.add_option('--num-threads', dest='numThreads', default=None,
                          type='int', help='The number threads to use per process.')
        parser.add_option('--num-processes-per-batch', dest='numProcessesPerBatch', default=1,
                          type='int', help='The number of simultaneous processes to run ' + \
                          'for each batch. This better be kept at 1 if running more than one batch.')

        # Action options
        parser.add_option('--interactive', action='store_true', default=False, dest='interactive',  
                          help='If to wait on user input to terminate the jobs.')
        parser.add_option('--log-batches', action='store_true', default=False, dest='logBatches',  
                          help="Just log the batch commands to a file.")
        parser.add_option('--cleanup', action='store_true', default=False, dest='cleanup',  
                          help='If the final result is produced delete intermediate files.')
        parser.add_option('--dry-run', action='store_true', default=False, dest='dryRun',  
                          help="Print but don't launch the processing jobs.")

        parser.add_option('--ortho-folder', dest='orthoFolder', default=None,
                          help='Use ortho files to adjust processing to the image spacing.')
        parser.add_option('--fireball-folder', dest='fireballFolder', default=None,
                          help='Location of fireball DEMs for comparison.')

        parser.add_option('--reference-dem', dest='referenceDem', default=None,
                          help='Reference DEM used to calculate the expected GSD.')

        (options, args) = parser.parse_args(argsIn)

        if len(args) < 4:
            print usage
            return 0

        imageFolder  = args[0]
        cameraFolder = args[1]
        lidarFolder  = args[2]
        outputFolder = args[3]

    except optparse.OptionError, msg:
        raise Usage(msg)

    os.system("ulimit -c 0") # disable core dumps
    os.system("umask 022")   # enforce files be readable by others
    
    # Check the inputs
    for f in [imageFolder, cameraFolder, lidarFolder]:
        if not os.path.exists(f):
            logger.error('Input folder '+ f +' does not exist!')
            return 0

    asp_system_utils.mkdir_p(outputFolder)

    suppressOutput = False
    redo           = False

    logger.info('\nStarting processing...')
    
    # Get a list of all the input files
    imageCameraPairs = icebridge_common.getImageCameraPairs(imageFolder, cameraFolder, 
                                                            options.startFrame, options.stopFrame,
                                                            logger)
    numFiles = len(imageCameraPairs)
    if numFiles < 2:
        raise Exception('Failed to find any image camera pairs!')
    
    # Check that the files are properly aligned
    lastFrame = -1
    availableFrames = []
    for (image, camera) in imageCameraPairs:
        frameNumber = icebridge_common.getFrameNumberFromFilename(image)
        availableFrames.append(frameNumber)
        if (icebridge_common.getFrameNumberFromFilename(camera) != frameNumber):
            logger.error('Error: input files do not align!\n' + str((image, camera)))
            return -1
        if frameNumber <= lastFrame:
            logger.error('Error: input frames not sorted properly!\n')
            return -1
        lastFrame = frameNumber


    # Set the output resolution as the computed mean GSD
    # - Currently we process ten frames total but this should be increased for production!
    # TODO: This should be cashed, and recomputed only when the batch file changes!
    GSD_RESOLUTION_MULTIPLIER = 4.0
    NUM_GSD_FRAMES = 20
    logger.info('Computing GSD with ' + str(NUM_GSD_FRAMES) + ' frames.')
    gsdFrameSkip = len(imageCameraPairs) / NUM_GSD_FRAMES
    if gsdFrameSkip < 1:
        gsdFrameSkip = 1
    medianGsd = getRunMedianGsd(imageCameraPairs, options.referenceDem, options.isSouth, gsdFrameSkip)
    outputResolution = medianGsd * GSD_RESOLUTION_MULTIPLIER
    logger.info('OUTPUT_RESOLUTION: ' + str(outputResolution))
    #return 0

    # Generate a map of initial camera positions
    orbitvizBefore = os.path.join(outputFolder, 'cameras_in.kml')
    vizString  = ''
    for (image, camera) in imageCameraPairs: 
        vizString += camera+' '
    cmd = 'orbitviz_pinhole --hide-labels -o '+ orbitvizBefore +' '+ vizString
    logger.info('Running orbitviz on input files...')

    asp_system_utils.executeCommand(cmd, orbitvizBefore, True, redo) # Suppress (potentially long) output
    #raise Exception('DEBUG')

    # Set up options for process_icebridge_batch
    extraOptions = ''
    if options.numThreads:
        extraOptions += ' --num-threads ' + str(options.numThreads)
    if options.numProcessesPerBatch:
        extraOptions += ' --num-processes-per-batch ' + str(options.numProcessesPerBatch)
    if options.solve_intr:
        extraOptions += ' --solve-intrinsics '
    if options.isSouth:
        extraOptions += ' --south '
    if options.maxDisplacement:
        extraOptions += ' --max-displacement ' + str(options.maxDisplacement)
    if options.fireballFolder:
        extraOptions += ' --fireball-folder ' + str(options.fireballFolder)
    if options.cleanup:
        extraOptions += ' --cleanup '

    # We ran this before, as part of fetching, so hopefully all the data is cached
    forceAllFramesInRange = False
    (breaks, largeSkips) = getImageSpacing(options.orthoFolder, availableFrames,
                                           options.startFrame, options.stopFrame,
                                           forceAllFramesInRange)
    if options.imageStereoInterval: 
        logger.info('Using manually specified image stereo interval: ' +
                    str(options.imageStereoInterval))
        largeSkips = [] # Always use a manually specified skip interval
    else:
        options.imageStereoInterval = 1

    sleepTime = 20

    # If all we are doing is logging commands then one process is sufficient.
    # - Wipe the output file while we are at it.
    batchLogPath = ''
    batchNum = 0

    if options.logBatches:
        options.numProcesses = 1
        options.numProcessesPerBatch = 1
        sleepTime    = 1
        batchLogPath = os.path.join(outputFolder, BATCH_COMMAND_LOG_FILE)
        os.system('rm -f ' + batchLogPath)
        logger.info('Just generating batch log file '+batchLogPath+', no processing will occur.')

    logger.info('Starting processing pool with ' + str(options.numProcesses) +' processes.')
    pool = NonDaemonPool(options.numProcesses)
    
    # Call process_icebridge_batch on each batch of images.
    # - Batch size should be the largest number of images which can be effectively bundle-adjusted.
    taskHandles           = []
    batchImageCameraPairs = []
    frameNumbers          = []
    i = 0 # The frame index that starts the current batch
    while True: # Loop for adding batches

        # Bugfix: arrived at the last feasible frame (maybe there are more but
        # they lack cameras).
        if i >= len(imageCameraPairs):
            break
        
        firstBundleFrame = icebridge_common.getFrameNumberFromFilename(imageCameraPairs[i][0])

        # Determine the frame skip amount for this batch (set by the first frame)
        thisSkipInterval = options.imageStereoInterval
        if firstBundleFrame in largeSkips:
            thisSkipInterval = largeSkips[firstBundleFrame]
        thisBatchSize = options.bundleLength + thisSkipInterval - 1

        # Keep adding frames until we have enough or run out of frames
        j = i # The frame index that ends the batch
        while True:
            # Update conditions
            hitBreakFrame = frameNumber in breaks
            lastFrame     = (frameNumber > options.stopFrame) or (j >= numFiles)
            endBatch      = len(frameNumbers) >= thisBatchSize
        
            if lastFrame or endBatch:
                break # The new frame is too much, don't add it to the batch
        
            # Add frame to the list for the current batch
            frameNumber = icebridge_common.getFrameNumberFromFilename(imageCameraPairs[j][0])
            batchImageCameraPairs.append(imageCameraPairs[j])
            frameNumbers.append(frameNumber)
            
            if hitBreakFrame:
                break # Break after this frame, it is the last one added to the batch.
            
            j = j + 1
        # Done adding frames to this batch

        if len(frameNumbers) <= thisSkipInterval:
            logger.info('Batch from frame: ' + str(firstBundleFrame) + ' is too small to run.  Skipping.')
        else:

            # Submit the batch
            if not options.logBatches:
                logger.info('Processing frame number: ' + str(firstBundleFrame))
                     
            # The output folder is named after the first and last frame in the batch.
            # We count on this convention in blend_dems.py.
            batchFolderName  = ('batch_%05d_%05d_%d' % (frameNumbers[0], frameNumbers[-1], options.bundleLength))
            thisOutputFolder = os.path.join(outputFolder, batchFolderName)

            if not options.logBatches:
                logger.info('Running processing batch in output folder: ' + thisOutputFolder + '\n' + 
                            'with options: ' + extraOptions + ' --stereo-arguments ' + options.stereoArgs)
            
            if not options.dryRun:
                # Generate the command call
                taskHandles.append(pool.apply_async(processBatch, 
                    (batchImageCameraPairs, lidarFolder, options.referenceDem, thisSkipInterval,
                     thisOutputFolder, extraOptions, 
                     outputResolution, options.stereoArgs, batchNum, batchLogPath)))
            batchNum += 1
            
        
        # Reset these lists
        batchImageCameraPairs = []
        frameNumbers          = []
        
        # Advance to the frame that starts the next batch
        if hitBreakFrame:
            # When we hit a break in the frames we need to start the
            # next batch after the break frame
            i = j + 1
        else:
            # Start in the next frame that was not used as a "left" stereo image.
            i = i + options.bundleLength - 1
        
        if lastFrame:
            break # Quit the main loop if we hit the end of the frame list.

    # End of loop through input file pairs
    logger.info('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    
    # Wait for all the tasks to complete
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, logger, options.interactive, 
                                                     quitKey='q', sleepTime=sleepTime)

    # Either all the tasks are finished or the user requested a cancel.
    # Clean up the processing pool
    icebridge_common.stopTaskPool(pool)

    logger.info('Finished process_icebridge_run.') # to avoid ending a log with 'waiting ...'

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



