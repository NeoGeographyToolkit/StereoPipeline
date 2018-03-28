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

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing, re, shutil, time
import os.path as P
import glob

# The path to the ASP python files and tools
basepath      = os.path.dirname(os.path.realpath(__file__))  # won't change, unlike syspath
pythonpath    = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath   = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
binpath       = os.path.abspath(basepath + '/../bin')        # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools
toolspath     = os.path.abspath(basepath + '/../Tools')      # ASP Tools

# Prepend to Python path
sys.path.insert(0, basepath)
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)

import icebridge_common, fetch_icebridge_data, process_icebridge_run, extract_icebridge_ATM_points
import input_conversions
import asp_system_utils, asp_alg_utils, asp_geo_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath      + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath        + os.pathsep + os.environ["PATH"]


def fetchAllRunData(options, startFrame, stopFrame, 
                    jpegFolder, orthoFolder, fireballFolder, lidarFolder, navFolder):
    '''Download all data needed to process a run'''
    
    logger = logging.getLogger(__name__)
    logger.info('Downloading data for the run...')

    baseCommand = (('--yyyymmdd %s --site %s --start-frame %d --stop-frame %d')
                   % (options.yyyymmdd, options.site, startFrame, stopFrame))

    if options.maxNumLidarToFetch is not None and options.maxNumLidarToFetch >= 0:
        baseCommand += ' --max-num-lidar-to-fetch ' + str(options.maxNumLidarToFetch)

    if options.refetchIndex:
        baseCommand += ' --refetch-index' # this was not right in older fetched runs
    if options.refetchNav:
        baseCommand += ' --refetch-nav' # sometimes this was corrupted
        
        
    if options.stopAfterIndexFetch:
        baseCommand += ' --stop-after-index-fetch' 
    if options.skipValidate:
        baseCommand += ' --skip-validate'
    if options.ignoreMissingLidar:
        baseCommand += ' --ignore-missing-lidar'
    if options.dryRun:
        baseCommand += ' --dry-run'

    jpegCommand      = baseCommand + ' ' + jpegFolder
    orthoCommand     = baseCommand + ' ' + orthoFolder
    fireballCommand  = baseCommand + ' ' + fireballFolder
    lidarCommand     = baseCommand + ' ' + lidarFolder
    navCommand       = baseCommand + ' ' + navFolder
    
    # Try to do all the downloads one after another
    # - On a failure the error message should already be printed.
    # - The fetching tool will not redownload existing data.
    if fetch_icebridge_data.main(jpegCommand.split()) < 0:
        return -1
    if fetch_icebridge_data.main(orthoCommand.split()) < 0:
        return -1
    if fetch_icebridge_data.main(fireballCommand.split()) < 0:
        logger.info('Fireball DEM data is optional, continuing run.')
    if not options.noNavFetch:
        if fetch_icebridge_data.main(navCommand.split()) < 0:
            return -1
    # Skip the lidar fetch if the user requested no lidar files
    if (options.maxNumLidarToFetch is None) or (options.maxNumLidarToFetch > 0):
        if fetch_icebridge_data.main(lidarCommand.split()) < 0:
            return -1

    # jpeg and ortho indices must be consistent
    if not options.skipValidate:
        logger.info("Check for consistency between raw and ortho images.")
        jpegIndex  = icebridge_common.csvIndexFile(jpegFolder)
        orthoIndex = icebridge_common.csvIndexFile(orthoFolder)
        
        (jpegFrameDict, jpegUrlDict)   = icebridge_common.readIndexFile(jpegIndex)
        (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndex)
        
        for jpegFrame in jpegFrameDict.keys():
            
            if jpegFrame < startFrame or jpegFrame > stopFrame:
                continue
            
            if jpegFrame not in orthoFrameDict.keys():
                logger.info("Found jpeg frame missing from ortho: " + str(jpegFrame))
                #raise Exception ("Found jpeg frame missing from ortho:" + str(jpegFrame))

        for orthoFrame in orthoFrameDict.keys():

            if orthoFrame < startFrame or orthoFrame > stopFrame:
                continue
            
            if orthoFrame not in jpegFrameDict.keys():
                # This can happen, don't die because of it
                logger.info("Found ortho frame missing from jpeg: " + str(orthoFrame))
                #raise Exception ("Found ortho frame missing from jpeg:" + str(orthoFrame))

    # TODO: Wipe any ortho and jpeg images not in the index, or at least warn about it.
    
    return (jpegFolder, orthoFolder, fireballFolder, lidarFolder)

def validateOrthosAndFireball(options, fileType, logger):
    '''Validate ortho and fireball files within the current frame range. This
    is expected to be in called in parallel for smaller chunks. Lidar files
    will be validated serially. Jpegs get validated when converted to tif.
    Return True if all is good.'''

    badFiles = False
    logger.info("Validating files of type: " + fileType)
    
    if fileType   == 'ortho':
        dataFolder = icebridge_common.getOrthoFolder(options.outputFolder)
    elif fileType == 'fireball':
        dataFolder = icebridge_common.getFireballFolder(options.outputFolder)
    else:
        raise Exception("Unknown file type: " + fileType)

    indexPath = icebridge_common.csvIndexFile(dataFolder)
    if not os.path.exists(indexPath):
        # The issue of what to do when the index does not exist should
        # have been settled by now.
        return (not badFiles)

    # Fetch from disk the set of already validated files, if any
    validFilesList = icebridge_common.validFilesList(options.outputFolder,
                                                     options.startFrame, options.stopFrame)
    validFilesSet = set()
    validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
    numInitialValidFiles = len(validFilesSet)
    
    (frameDict, urlDict) = icebridge_common.readIndexFile(indexPath, prependFolder = True)
    for frame in frameDict.keys():

        if frame < options.startFrame or frame > options.stopFrame:
            continue

        outputPath = frameDict[frame]
        xmlFile = icebridge_common.xmlFile(outputPath)

        if outputPath in validFilesSet and os.path.exists(outputPath) and \
            xmlFile in validFilesSet and os.path.exists(xmlFile):
            #logger.info('Previously validated: ' + outputPath + ' ' + xmlFile)
            continue
        else:
            isGood = icebridge_common.hasValidChkSum(outputPath, logger)
            if not isGood:
                logger.info('Found invalid data. Will wipe: ' + outputPath + ' ' + xmlFile)
                os.system('rm -f ' + outputPath) # will not throw
                os.system('rm -f ' + xmlFile) # will not throw
                badFiles = True
            else:
                logger.info('Valid file: ' + outputPath)
                validFilesSet.add(outputPath)
                validFilesSet.add(xmlFile)
            
        if fileType != 'fireball':
            continue

        # Also validate tfw
        tfwFile = icebridge_common.tfwFile(outputPath)
        xmlFile = icebridge_common.xmlFile(tfwFile)
        if tfwFile in validFilesSet and os.path.exists(tfwFile) and \
            xmlFile in validFilesSet and os.path.exists(xmlFile):
            #logger.info('Previously validated: ' + tfwFile + ' ' + xmlFile)
            continue
        else:
            isGood = icebridge_common.isValidTfw(tfwFile, logger)
            if not isGood:
                logger.info('Found invalid tfw. Will wipe: ' + tfwFile + ' ' + xmlFile)
                os.system('rm -f ' + tfwFile) # will not throw
                os.system('rm -f ' + xmlFile) # will not throw
                badFiles = True
            else:
                logger.info('Valid tfw file: ' + tfwFile)
                validFilesSet.add(tfwFile)
                validFilesSet.add(xmlFile)
        
    # Write to disk the list of validated files, but only if new
    # validations happened.  First re-read that list, in case a
    # different process modified it in the meantime, such as if two
    # managers are running at the same time.
    numFinalValidFiles = len(validFilesSet)
    if numInitialValidFiles != numFinalValidFiles:
        validFilesSet = \
                      icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
        icebridge_common.writeValidFilesList(validFilesList, validFilesSet)

    return (not badFiles)
    
def runFetchConvert(options, isSouth, cameraFolder, imageFolder, jpegFolder, orthoFolder,
                    fireballFolder, corrFireballFolder, lidarFolder, processedFolder,
                    navFolder, navCameraFolder, refDemPath, logger):
    '''Fetch and/or convert. Return 0 on success.'''

    if options.noFetch:
        logger.info('Skipping fetch.')
    else:
        # Call data fetch routine and check the result
        fetchResult = fetchAllRunData(options, options.startFrame, options.stopFrame,
                                      jpegFolder, orthoFolder, fireballFolder, lidarFolder,
                                      navFolder)
        if fetchResult < 0:
            logger.error("Fetching failed!") 
            return -1

        # This step is slow, so run it here as part of fetching and save its result
        # We certainly don't want it to throw any exception at this stage.
        try:
            forceAllFramesInRange = True
            availableFrames = []
            (autoStereoInterval, breaks) = \
                                 process_icebridge_run.getImageSpacing(orthoFolder, availableFrames,
                                                                       options.startFrame,
                                                                       options.stopFrame,
                                                                       forceAllFramesInRange)
        except Exception, e:
            pass
        
    if options.stopAfterFetch or options.dryRun:
        logger.info('Fetching complete, finished!')
        return 0

    # Keep track of how we are doing
    isGood = True
    
    if options.noConvert:        
        logger.info('Skipping convert.')
    else:

        # When files fail in these conversion functions we log the error and keep going

        if not options.skipFastConvert:

            if not options.skipValidate:
                # Validate orthos and dems for this frame range.
                ans = validateOrthosAndFireball(options, 'ortho', logger)
                isGood = (isGood and ans)
                ans = validateOrthosAndFireball(options, 'fireball', logger)
                isGood = (isGood and ans)
            
            # Run non-ortho conversions without any multiprocessing (they are pretty fast)
            # TODO: May be worth doing the faster functions with multiprocessing in the future

            if not options.noLidarConvert:
                ans = input_conversions.convertLidarDataToCsv(lidarFolder,
                                                              options.startFrame, options.stopFrame,
                                                              options.skipValidate,
                                                              logger)
                isGood = (isGood and ans)
                
                ans = input_conversions.pairLidarFiles(lidarFolder, options.skipValidate, logger)
                isGood = (isGood and ans)
                
            ans = input_conversions.correctFireballDems(fireballFolder, corrFireballFolder,
                                                        options.startFrame, options.stopFrame,
                                                        (not isSouth), options.skipValidate,
                                                        logger)
            isGood = (isGood and ans)

            ans = input_conversions.convertJpegs(jpegFolder, imageFolder, 
                                                 options.startFrame, options.stopFrame,
                                                 options.skipValidate, options.cameraMounting,
                                                 logger)
            isGood = (isGood and ans)
            
        if not options.noNavFetch:
            # Single process call to parse the nav files.
            input_conversions.getCameraModelsFromNav(imageFolder, orthoFolder, 
                                                     options.inputCalFolder,
                                                     options.inputCalCamera,
                                                     options.cameraLookupFile,
                                                     navFolder, navCameraFolder,
                                                     options.yyyymmdd, options.site, 
                                                     options.startFrame, options.stopFrame,
                                                     options.cameraMounting,
                                                     logger)
        else:
            navCameraFolder = ""
            options.simpleCameras = False

        if not options.noOrthoConvert:
            # Multi-process call to convert ortho images
            input_conversions.getCameraModelsFromOrtho(imageFolder, orthoFolder,
                                                       options.inputCalFolder,
                                                       options.inputCalCamera,
                                                       options.cameraLookupFile,
                                                       options.noNavFetch,
                                                       navCameraFolder,
                                                       options.yyyymmdd, options.site, 
                                                       refDemPath, cameraFolder, 
                                                       options.simpleCameras,
                                                       options.startFrame, options.stopFrame,
                                                       options.framesFile,
                                                       options.numOrthoProcesses, options.numThreads,
                                                       logger)


    os.system("rm -f core.*") # these keep on popping up
    
    if isGood:
        return 0

    return -1
    
def processTheRun(options, imageFolder, cameraFolder, lidarFolder, orthoFolder,
                  fireballFolder, processedFolder, isSouth, refDemPath):
    
    '''Do all the run processing'''

    # Some care is taken with the --stereo-arguments argument to make sure it is passed correctly.
    processCommand = (('%s %s %s %s --bundle-length %d --fireball-folder %s ' +
                       '--ortho-folder %s --num-processes %d --num-threads %d ' +
                       '--reference-dem %s')
                      % (imageFolder, cameraFolder, lidarFolder, processedFolder,
                         options.bundleLength, fireballFolder, orthoFolder, options.numProcesses,
                         options.numThreads, refDemPath))
    if isSouth:
        processCommand += ' --south'
    if options.startFrame:
        processCommand += ' --start-frame ' + str(options.startFrame)
    if options.stopFrame:
        processCommand += ' --stop-frame ' + str(options.stopFrame)
    if options.logBatches:
        processCommand += ' --log-batches'
    if options.cleanup:
        processCommand += ' --cleanup'
    if options.manyip:
        processCommand += ' --many-ip'
        
    processCommand += ' --stereo-arguments '

    logger = logging.getLogger(__name__)
    logger.info('Process command: process_icebridge_run ' +
                processCommand + options.stereoArgs.strip())
    args = processCommand.split()
    args += (options.stereoArgs.strip(),) # Make sure this is properly passed
    process_icebridge_run.main(args)

def solveIntrinsics_Part1(options, jpegFolder, cameraFolder, navCameraFolder, processedFolder,
                          logger):
    '''Some preliminary work before solving for intrinsics. Here we
    look up the default calibration file, and generate an RPC
    approximation of its distortion model with polynomials of degree
    4. We will then create cameras and stereo DEMs using this initial
    camera file with RPC distortion.'''

    # Sanity checks
    if options.startFrame == icebridge_common.getSmallestFrame() or \
       options.stopFrame == icebridge_common.getLargestFrame():
        raise Exception("When solving for intrinsics, must specify a frame range.")
    if options.bundleLength != 2:
        raise Exception("When solving for intrinsics, we assume bundle length of 2.")
    if (options.stopFrame - options.startFrame) % 2 == 0:
        raise Exception("When solving for intrinsics, must have an even number of frames, " +
                        " so stopFrame - startFrame must be odd.")
    if options.processingSubfolder:
        raise Exception("Processing subfolder not supported when solving for intrinsics.")

    # Generate extra data we will use later to float intrinsics
    options.stereoArgs += "  --num-matches-from-disp-triplets 10000 --unalign-disparity " #  --enable-fill-holes "

    # Create separate directories for cameras and processed data,
    # as these will be distinct than what we will finally be
    # using to do the full run.
    suff = "_camgen"
    cameraFolder    += suff
    navCameraFolder += suff
    processedFolder += suff

    # Get the input calibration file
    defaultCalibFile = ""
    for frame in range(options.startFrame, options.stopFrame+1):
        currCalibFile = input_conversions.getCalibrationFileForFrame(options.cameraLookupFile,
                                                                     options.inputCalFolder,
                                                                     frame, options.yyyymmdd,
                                                                     options.site)
        if defaultCalibFile == "":
            defaultCalibFile = currCalibFile

        if defaultCalibFile != currCalibFile:
            # This is important, the calibration file must be unique
            raise Exception("Found two distinct calibration files: " + defaultCalibFile + \
                            " and " + currCalibFile)

    logger.info("Default calibration file: " + defaultCalibFile)
    if options.inputCalCamera != "":
        defaultCalibFile = options.inputCalCamera
        logger.info("Using instead the user-provided: " + defaultCalibFile)

    # Find the first image in the range
    jpegIndex  = icebridge_common.csvIndexFile(jpegFolder)
    (jpegFrameDict, jpegUrlDict)   = icebridge_common.readIndexFile(jpegIndex,
                                                                    prependFolder = True)
    if options.startFrame not in jpegFrameDict.keys():
        raise Exception("Could not find jpeg image for frame: " + options.startFrame)
    firstImage = jpegFrameDict[options.startFrame]

    # Create the RPC file before optimization
    rpcCalibFile = os.path.join(processedFolder, os.path.basename(defaultCalibFile))
    rpcCalibFile = rpcCalibFile.replace(".tsai", "_INIT_RPC.tsai")
    logger.info("Will approximate camera model " + defaultCalibFile + " with " + \
                options.outputModelType + " model " + rpcCalibFile)
    os.system("mkdir -p " + os.path.dirname(rpcCalibFile))
    cmd = "convert_pinhole_model --input-file " + firstImage + ' --camera-file '   +  \
          defaultCalibFile + ' --output-type ' + options.outputModelType           +  \
          ' --sample-spacing 50 -o ' + rpcCalibFile
    logger.info(cmd)
    os.system(cmd)

    # Use this one from now on
    options.inputCalCamera = rpcCalibFile

    # Return the modified values
    return (options, cameraFolder, navCameraFolder, processedFolder)
    
def solveIntrinsics_Part2(options, imageFolder, cameraFolder, lidarFolder, orthoFolder,
                         processedFolder, isSouth, logger):
    
    '''Create a camera model with optimized intrinsics. By now we
    processed a bunch of images and created bundle-adjusted and
    pc_aligned cameras and DEMs while using a camera model with
    distortion implemented using RPC coefficients which was obtained
    from the photometrics model. We now use the obtained cameras as
    inputs to a bundle adjust problem where we will optimize the
    intrinsics, including the distortion RPC coefficients, using the
    lidar as an external constraint, and many dense IP pairs and
    triplets (no quadruplets yet, even if 4 images overlap).'''

    # Get a list of all the input files
    imageCameraPairs = icebridge_common.getImageCameraPairs(imageFolder, cameraFolder, 
                                                            options.startFrame, options.stopFrame,
                                                            logger)
    
    # The paired lidar file for the first image should be huge enough to contain
    # all images.
    lidarFile = icebridge_common.findMatchingLidarFile(imageCameraPairs[0][0], lidarFolder)
    logger.info('Found matching lidar file ' + lidarFile)
    lidarCsvFormatString = icebridge_common.getLidarCsvFormat(lidarFile)

    numFiles = len(imageCameraPairs)
    if numFiles < 2:
        raise Exception('Failed to find any image camera pairs!')
    if numFiles % 2 != 0:
        raise Exception("When solving for intrinsics, must have an even number of frames to use.")


    # Collect pc_align-ed cameras, unaligned disparities, and dense match files
    images = []
    cameras = []
    dispFiles = []
    for it in range(numFiles/2):
        begFrame = options.startFrame + 2*it
        endFrame = begFrame + 1
        batchFolderName  = icebridge_common.batchFolderName(begFrame, endFrame, options.bundleLength)
        thisOutputFolder = os.path.join(processedFolder, batchFolderName)
        
        # Find all the cameras after bundle adjustment and pc_align.
        pattern = icebridge_common.getAlignedBundlePrefix(thisOutputFolder) + '*.tsai'
        alignedCameras = glob.glob(pattern)
        if len(alignedCameras) != options.bundleLength:
            raise Exception("Expected " + str(options.bundleLength) + " cameras, here's what " +
                            " was obtained instead: " + " ".join(alignedCameras))
        img0 = ""; cam0 = ""; img1 = ""; cam1 = ""
        for cam in alignedCameras:
            frame = icebridge_common.getFrameNumberFromFilename(cam)
            if begFrame == frame:
                img0 = imageCameraPairs[2*it][0]
                cam0 = cam
            if endFrame == frame:
                img1 = imageCameraPairs[2*it+1][0]
                cam1 = cam
        images.append(img0);  images.append(img1)
        cameras.append(cam0); cameras.append(cam1)

        # Unaligned disparity
        stereoFolder = os.path.join(thisOutputFolder, 'stereo_pair_'+str(0))
        currDispFiles = glob.glob(os.path.join(stereoFolder, '*unaligned-D.tif'))
        if len(currDispFiles) != 1:
            raise Exception("Expecting a single unaligned disparity file in " + stereoFolder)
        dispFiles.append(currDispFiles[0])
        
    # Match files
    matchFiles = []
    for it in range(numFiles-1):
        begFrame = options.startFrame + it
        endFrame = begFrame + 1
        batchFolderName  = icebridge_common.batchFolderName(begFrame, endFrame, options.bundleLength)
        thisOutputFolder = os.path.join(processedFolder, batchFolderName)
        stereoFolder = os.path.join(thisOutputFolder, 'stereo_pair_'+str(0))
        DISP_PREFIX = "disp-"
        currMatchFiles = glob.glob(os.path.join(stereoFolder, '*' + DISP_PREFIX + '*.match'))
        if len(currMatchFiles) != 1:
            raise Exception("Expecting a single dense match file in " + stereoFolder)
        matchFiles.append(currMatchFiles[0])

    # Create output directory for bundle adjustment and copy there the match files
    baDir = os.path.join(processedFolder, "bundle_intrinsics")
    baPrefix = os.path.join(baDir, "out")
    os.system("mkdir -p " + baDir)
    for matchFile in matchFiles:
        dstFile = os.path.basename(matchFile)
        dstFile = dstFile.replace(DISP_PREFIX, '')
        dstFile = os.path.join(baDir, dstFile)
        cmd = "cp -f " + matchFile + " " + dstFile
        logger.info(cmd)
        os.system(cmd)

    # The bundle adjustment
    cmd = "bundle_adjust " + " ".join(images) + " " +  " ".join(cameras) + \
            ' --reference-terrain ' + lidarFile + \
            ' --disparity-list "' + " ".join(dispFiles) + '"' + \
            ' --datum wgs84 -t nadirpinhole --create-pinhole-cameras --robust-threshold 2' + \
            ' --camera-weight 1 --solve-intrinsics --csv-format ' + lidarCsvFormatString + \
            ' --overlap-limit 1 --max-disp-error 10 --max-iterations 100 ' + \
            ' --parameter-tolerance 1e-12 -o ' + baPrefix
    logger.info(cmd)
    os.system(cmd)

    # Generate DEMs of residuals before and after optimization
    projString = icebridge_common.getEpsgCode(isSouth, asString=True)
    for val in ['initial', 'final']:
        cmd = 'point2dem --t_srs ' + projString + ' --tr 2'    + \
              ' --csv-format 1:lon,2:lat,4:height_above_datum' + \
              ' ' + baPrefix + '-' + val + '_residuals_no_loss_function_pointmap_point_log.csv'
        logger.info(cmd)
        os.system(cmd)
        cmd = 'point2dem --t_srs ' + projString + ' --tr 2'    + \
              ' --csv-format 1:lon,2:lat,4:height_above_datum' + \
              ' ' + baPrefix + '-' + val +'_residuals_no_loss_function_reference_terrain.txt'
        logger.info(cmd)
        os.system(cmd)

    # Look at the latest written tsai file, that will be the optimized distortion file.
    # Force the initial rotation and translation to be the identity, this is
    # expected by ortho2pinhole.
    outFiles = filter(os.path.isfile, glob.glob(baPrefix + '*.tsai'))
    outFiles.sort(key=lambda x: os.path.getmtime(x))
    optFile = outFiles[-1]
    logger.info("Reading optimized file: " + optFile)
    with open(optFile, 'r') as f:
        lines = f.readlines()
    for it in range(len(lines)):
        lines[it] = lines[it].strip()
        if re.match("^C\s*=\s*", lines[it]):
            lines[it] = "C = 0 0 0"
        if re.match("^R\s*=\s*", lines[it]):
            lines[it] = "R = 1 0 0 0 1 0 0 0 1"

    # Write the final desired optimized RPC file
    logger.info("Writing final optimized file: " + options.outputCalCamera)
    # Below is a bugfix, must take full path to find the dir, otherwise it may fail.
    os.system("mkdir -p " + os.path.dirname(os.path.abspath(options.outputCalCamera)))
    with open(options.outputCalCamera, 'w') as f:
        for line in lines:
            f.write(line + "\n")
        
def main(argsIn):

    try:
        # Sample usage:
        # python full_processing_script.py \
        #  --yyyymmdd 20091016 --site AN --num-processes 1 --num-threads 12 --bundle-length 12 \
        #  --start-frame 350 --stop-frame 353 --skip-validate \
        # --camera-calibration-folder camera_calib  \
        # --reference-dem-folder ref_dem_folder
        # An output folder will be crated automatically (with a name like
        # AN_20091016), or its name can be specified via the --output-folder
        # option.
        usage = '''full_processing_script.py <options>'''
                      
        parser = argparse.ArgumentParser(usage=usage)

        # Run selection
        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL)")

        parser.add_argument("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        parser.add_argument("--camera-lookup-file",  dest="cameraLookupFile", default=None,
                          help="The file to use to find which camera was used for which "  + \
                          "flight. By default it is in the same directory as this script " + \
                          "and named camera_lookup.txt.")
        
        # Processing options
        parser.add_argument('--bundle-length', dest='bundleLength', default=2,
                          type=int, help="The number of images to bundle adjust and process " + \
                          "in a single batch.")
        # TODO: Compute this automatically??
        parser.add_argument('--overlap-limit', dest='overlapLimit', default=2,
                          type=int, help="The number of images to treat as overlapping for " + \
                          "bundle adjustment.")
        
        parser.add_argument('--stereo-arguments', dest='stereoArgs',
                            # set --min-xcorr-level 0 to do the left-to-right 
                            # and right-to-left consistency check at the lowest level.
                            default='--stereo-algorithm 2 --min-xcorr-level 0',
                            help='Extra arguments to pass to stereo.')

        parser.add_argument('--start-frame', dest='startFrame', type=int,
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                          default=icebridge_common.getLargestFrame(),
                          help='Frame to stop on.')
        parser.add_argument('--frames-file', dest='framesFile', default="",
                            help='Specific frames to run ortho2pinhole on within this frame range.')

        parser.add_argument('--max-num-lidar-to-fetch', dest='maxNumLidarToFetch', default=None,
                          type=int, help="The maximum number of lidar files to fetch. " + \
                          "This is used in debugging.")
        
        parser.add_argument("--camera-calibration-folder",  dest="inputCalFolder", default=None,
                          help="The folder containing camera calibration.")

        parser.add_argument("--input-calibration-camera",  dest="inputCalCamera", default="",
                            help="Instead of looking up the calibrated camera in the calibration folder, use this one.")
        
        parser.add_argument("--output-calibration-camera",  dest="outputCalCamera", default="",
                            help="If specified, float the intrinsics and write the optimized model here.")

        parser.add_argument("--output-model-type",  dest="outputModelType", default="RPC",
                            help="Generate a distortion model of type RPC, RPC5, or RPC6.")
        parser.add_argument("--reference-dem-folder",  dest="refDemFolder", default=None,
                          help="The folder containing DEMs that created orthoimages.")

        parser.add_argument("--processing-subfolder",  dest="processingSubfolder", default=None,
                          help="Specify a subfolder name where the processing outputs will go. " + \
                          "fault is no additional folder")
                          
        parser.add_argument("--simple-cameras", action="store_true", dest="simpleCameras", default=False,
                          help="Don't use orthoimages to refine the camera models.")

        # This option is only needed when generating camera models from the nav files.
        parser.add_argument('--camera-mounting', default=0, dest='cameraMounting', type=int,
              help='0=right-forwards, 1=left-forwards, 2=top-forwards, 3=bottom-forwards.')

        # Performance options  
        parser.add_argument('--num-processes', dest='numProcesses', default=1,
                          type=int, help='The number of simultaneous processes to run.')
        parser.add_argument('--num-ortho-processes', dest='numOrthoProcesses', default=-1,
                          type=int, help='The number of simultaneous ortho processes to run.')
        parser.add_argument('--num-threads', dest='numThreads', default=8,
                          type=int, help='The number of threads per process.')

        # Action control
        parser.add_argument("--skip-fetch", action="store_true", dest="noFetch", default=False,
                          help="Skip data fetching.")
        parser.add_argument("--skip-convert", action="store_true", dest="noConvert", default=False,
                          help="Skip data conversion.")
        parser.add_argument("--stop-after-fetch", action="store_true", dest="stopAfterFetch",
                          default=False,
                          help="Stop program after data fetching.")
        parser.add_argument("--stop-after-convert", action="store_true", dest="stopAfterConvert",
                          default=False,
                          help="Stop program after data conversion.")
        parser.add_argument("--skip-validate", action="store_true", dest="skipValidate",
                            default=False,
                            help="Skip input data validation.")
        parser.add_argument("--ignore-missing-lidar", action="store_true", dest="ignoreMissingLidar",
                            default=False,
                            help="Keep going if the lidar is missing.")
        parser.add_argument("--log-batches", action="store_true", dest="logBatches", default=False,
                          help="Log the required batch commands without running them.")
        parser.add_argument('--cleanup', action='store_true', default=False, dest='cleanup',  
                          help='If the final result is produced delete intermediate files.')
        parser.add_argument('--many-ip', action='store_true', default=False, dest='manyip',  
                          help='If to use a lot of IP in bundle adjustment from the beginning.')
        parser.add_argument("--dry-run", action="store_true", dest="dryRun", default=False,
                          help="Set up the input directories but do not fetch/process any imagery.")

        parser.add_argument("--refetch", action="store_true", dest="reFetch", default=False,
                          help="Try fetching again if some files turned out invalid " + \
                          "during conversions.")
        parser.add_argument("--refetch-index", action="store_true", dest="refetchIndex",
                          default=False,
                          help="Force refetch of the index file.")
        parser.add_argument("--refetch-nav", action="store_true", dest="refetchNav",
                          default=False,
                          help="Force refetch of the nav file.")
        parser.add_argument("--stop-after-index-fetch", action="store_true",
                          dest="stopAfterIndexFetch", default=False,
                          help="Stop after fetching the indices.")

        parser.add_argument("--no-nav", action="store_true", dest="noNavFetch",
                            default=False, help="Don't fetch or convert the nav data.")
                       
        parser.add_argument("--no-lidar-convert", action="store_true", dest="noLidarConvert",
                          default=False,
                          help="Skip lidar files in the conversion step.")
        parser.add_argument("--no-ortho-convert", action="store_true", dest="noOrthoConvert",
                          default=False,
                          help="Skip generating camera models in the conversion step.")
        parser.add_argument("--skip-fast-conversions", action="store_true", dest="skipFastConvert",
                          default=False,
                          help="Skips all non-ortho conversions.")
                          
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError, msg:
        parser.error(msg)

    icebridge_common.switchWorkDir()
    
    if options.numOrthoProcesses < 0:
        options.numOrthoProcesses = options.numProcesses
        
    isSouth = icebridge_common.checkSite(options.site)

    # Turned off elevation limits here since they are being set from LIDAR data.
    ## Add the site based elevation limits to the stereoArgs option
    #altLimits = icebridge_common.getElevationLimits(options.site)
    #options.stereoArgs = (' %s --elevation-limit %f %f ' 
    #                      % (options.stereoArgs, altLimits[0], altLimits[1]))
    options.stereoArgs = (' %s ' % (options.stereoArgs))

    if options.cameraLookupFile is None:
        options.cameraLookupFile = P.join(basepath, 'camera_lookup.txt')
    if not os.path.isfile(options.cameraLookupFile):
        raise Exception("Can't find camera file: " + options.cameraLookupFile)
        
    if len(options.yyyymmdd) != 8 and len(options.yyyymmdd) != 9:
        # Make an exception for 20100422a
        raise Exception("The --yyyymmdd field must have length 8 or 9.")

    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)

    if options.stopAfterIndexFetch:
        options.stopAfterFetch = True
        
    os.system('mkdir -p ' + options.outputFolder)
    logLevel = logging.INFO # Record everything
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel,
                                            'icebridge_processing_log_frames_' + \
                                            str(options.startFrame) + "_" + str(options.stopFrame))

    # Make sure we later know what we were doing
    logger.info("full_processing_script.py " + " ".join(argsIn)) 
                
    (out, err, status) = asp_system_utils.executeCommand(['uname', '-a'],
                                                         suppressOutput = True)
    logger.info("Running on machine: " + out)
    logger.info("Work dir is " + os.getcwd())

    os.system("ulimit -c 0") # disable core dumps
    os.system("umask 022")   # enforce files be readable by others
    
    # Perform some input checks and initializations
    # These are not needed unless cameras are initialized 
    if options.inputCalFolder is None or not os.path.exists(options.inputCalFolder):
        raise Exception("Missing camera calibration folder.")
    if options.refDemFolder is None or not os.path.exists(options.refDemFolder):
        raise Exception("Missing reference DEM folder.")
    
    refDemName = icebridge_common.getReferenceDemName(options.site)
    refDemPath = os.path.join(options.refDemFolder, refDemName)
    if not os.path.exists(refDemPath):
        raise Exception("Missing reference DEM: " + refDemPath)
    
    # TODO: CLEAN UP!!!
    # Set up the output folders
    cameraFolder       = icebridge_common.getCameraFolder(options.outputFolder)
    imageFolder        = icebridge_common.getImageFolder(options.outputFolder)
    jpegFolder         = icebridge_common.getJpegFolder(options.outputFolder)
    orthoFolder        = icebridge_common.getOrthoFolder(options.outputFolder)
    fireballFolder     = icebridge_common.getFireballFolder(options.outputFolder)
    corrFireballFolder = icebridge_common.getCorrFireballFolder(options.outputFolder)
    lidarFolder        = icebridge_common.getLidarFolder(options.outputFolder)
    navFolder          = icebridge_common.getNavFolder(options.outputFolder)
    navCameraFolder    = icebridge_common.getNavCameraFolder(options.outputFolder)
    processedFolder    = icebridge_common.getProcessedFolder(options.outputFolder)

    if options.outputCalCamera != "":
        # Prepare to solve for intrinsics. Note that this modifies some things along the way.
        (options, cameraFolder, navCameraFolder, processedFolder) = \
                  solveIntrinsics_Part1(options, jpegFolder, cameraFolder, navCameraFolder,
                                        processedFolder, logger)
        
    # Handle subfolder option.  This is useful for comparing results with different parameters!
    if options.processingSubfolder:
        processedFolder = os.path.join(processedFolder, options.processingSubfolder)
        logger.info('Will write to processing subfolder: ' + options.processingSubfolder)
       
    # If something failed in the first attempt either in fetch or in
    # convert, we will wipe bad files, and try to refetch/re-convert.
    numAttempts = 1
    if options.reFetch and (not options.noFetch):
        numAttempts = 2
    
    for attempt in range(numAttempts):
        if numAttempts > 1:
            logger.info("Fetch/convert attempt: " + str(attempt+1))
        ans = runFetchConvert(options, isSouth, cameraFolder, imageFolder, jpegFolder, orthoFolder,
                              fireballFolder, corrFireballFolder, lidarFolder, processedFolder,
                              navFolder, navCameraFolder, refDemPath, logger)
        if ans == 0:
            break
        
    if options.stopAfterFetch or options.dryRun or options.stopAfterConvert:
        logger.info('Fetch/convert finished!')
        return 0

       
    # Call the processing routine
    processTheRun(options, imageFolder, cameraFolder, lidarFolder, orthoFolder,
                  corrFireballFolder, processedFolder,
                  isSouth, refDemPath)
   
    if options.outputCalCamera != "":
        # Finish solving for intrinscs. 
        solveIntrinsics_Part2(options, imageFolder, cameraFolder, lidarFolder, orthoFolder,
                              processedFolder, isSouth, logger)
        
# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


