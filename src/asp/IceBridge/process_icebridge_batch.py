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

# Run bundle adjustment, stereo, generate DEMs, merge dems, perform alignment, etc,
# for a series of icebridge images

import os, sys, optparse, datetime, logging, multiprocessing, glob, re, shutil

# The path to the ASP python files
basepath      = os.path.dirname(os.path.realpath(__file__))  # won't change, unlike syspath
pythonpath    = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath   = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
binpath       = os.path.abspath(basepath + '/../bin')        # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools
toolspath     = os.path.abspath(basepath + '/../Tools')      # ASP Tools
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, toolspath)

import icebridge_common
import asp_system_utils, asp_alg_utils, asp_geo_utils, asp_image_utils, asp_file_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath   + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath     + os.pathsep + os.environ["PATH"]

# TODO: For the footprint DEM, implement some functions like:
# getFootprintPrefix() to avoid replicate the '-footprint-' string all
# over the place.

def formImageCameraString(inputPairs):
    '''Generate [images] [cameras] string from a list of input pairs.'''
    imagesAndCams = ""
    images        = ""
    cameras       = ""
    for (image, camera) in inputPairs: 
        images  += image;  images  += " "
        cameras += camera; cameras += " "
    imagesAndCams = images + cameras
    return imagesAndCams
    

def makeDemAndCheckError(options, projString, pointCloud, 
                          lidarFile, lidarCsvFormatString,
                          threadText, logger):
    '''Make a DEM from a point cloud and compute the geodiff results with lidar.'''

    suppressOutput = True
    redo = True

    # Generate a DEM file from the point cloud
    cmd = ('point2dem --tr %lf --t_srs %s %s %s' 
           % (options.demResolution, projString, pointCloud, threadText))
    icebridge_common.logger_print(logger, cmd)
    demPath = pointCloud.replace('.tif', '-DEM.tif')
    asp_system_utils.executeCommand(cmd, demPath, suppressOutput, redo)
    
    # Run geodiff with lidar and read in the results
    diffPrefix = pointCloud
    diffPath   = diffPrefix + '-diff.csv'
    cmd = ('geodiff --absolute --csv-format %s %s %s -o %s' %
           (lidarCsvFormatString, demPath, lidarFile, diffPrefix))
    icebridge_common.logger_print(logger, cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, diffPath, suppressOutput, redo)
    icebridge_common.logger_print(logger, out + '\n' + err)
    
    results = icebridge_common.readGeodiffOutput(diffPath)
    
    return (demPath, diffPath, results)

# TODO: May not need this function!
def getOverlapAmount(dem1, dem2, logger):
    '''Count the number of overlapping pixels in two DEMs'''
    
    # Generate a DEM file from the point cloud
    mosaicPrefix = os.path.join(os.path.dirname(dem1), 'overlapTemp')
    mosaicOutput = mosaicPrefix + '-tile-0-count.tif'
    cmd = ('dem_mosaic --count %s %s -o %s' % (dem1, dem2, mosaicPrefix))
    icebridge_common.logger_print(logger, cmd)
    asp_system_utils.executeCommand(cmd, mosaicOutput, True, True)

    # Count up the '2' values

    # Need to be 8 bit for ImageMagick
    u8Mosaic = mosaicPrefix + '-u8_count.tif'
    cmd = ('gdal_translate -ot byte %s %s' % (mosaicOutput, u8Mosaic))
    icebridge_common.logger_print(logger, cmd)
    asp_system_utils.executeCommand(cmd, u8Mosaic, True, True)
    
    # Now count the overlapping pixels
    cmd = ('convert '+u8Mosaic+
           ' -fill black +opaque "gray(2)" -fill white -opaque "gray(2)" -format "%[fx:w*h*mean]" info:')
    icebridge_common.logger_print(logger, cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, u8Mosaic, True, True)    
    if status != 0:
        raise Exception('Failed to compute overlap amount!')
    count = int(out.strip())
    
    # Clean up
    os.remove(mosaicOutput)
    os.remove(u8Mosaic)
    
    return count


def robustPcAlign(options, outputPrefix, lidarFile, lidarDemPath, 
                  demPath, finalAlignedDEM, 
                  projString, lidarCsvFormatString, threadText, logger):
    '''Try pc_align with increasing max displacements until we it completes
       with enough lidar points used in the comparison'''

    # Displacements are still since an initial vertical shift is applied
    DISPLACEMENTS        = [1, 2, 4, 6, 10, 25] # TODO: Increase these numbers for land flights!
    ERR_HEADER_SIZE      = 3
    IDEAL_LIDAR_DIST     = 0.1  # Quit aligning if we get under this error
    MIN_DIST_IMPROVEMENT = 0.25 # Percentage improvement in error to accept a larger max-disp

    lidarCsvFormatString = icebridge_common.getLidarCsvFormat(lidarFile)
    
    alignPrefix   = icebridge_common.getAlignPrefix(options.outputFolder)
    
    pcAlignFolder = os.path.dirname(alignPrefix)
    endErrorPath  = alignPrefix + '-end_errors.csv'
    transformPath = alignPrefix + '-transform.txt'
    alignedPC     = alignPrefix+'-trans_reference.tif'
    alignedDem    = alignedPC.replace('-trans_reference.tif', '-trans_reference-DEM.tif')
    lidarDiffPath = outputPrefix + "-diff.csv"

    # Check if the file is already there
    if ( os.path.exists(alignedDem) or os.path.exists(finalAlignedDEM) ) \
           and os.path.exists(lidarDiffPath) and not redo:
        results = icebridge_common.readGeodiffOutput(lidarDiffPath)
        logger.info("Outputs of pc_align already exist.")
        return alignedDem, lidarDiffPath, results['Mean']

    # Estimate an initial vertical shift between our DEM and the lidar data.
    try:
        diffPrefix = os.path.join(options.outputFolder, 'out')
        diffOutput = diffPrefix + '-diff.tif'
        cmd = ('geodiff %s %s -o %s' % (demPath, lidarDemPath, diffPrefix))
        icebridge_common.logger_print(logger, cmd)
        (out, err, status) = asp_system_utils.executeCommand(cmd, diffOutput, True, True)
        icebridge_common.logger_print(logger, out + '\n' + err)
        diffInfo = asp_image_utils.getImageStats(diffOutput)
        meanDiff = -1.0 * diffInfo[0][2] # This adjustment is made in the down direction.
        os.system('rm -rf ' + diffOutput) # Go ahead and clean this up now
        logger.info('Vertical diff estimate from lidar = ' + str(meanDiff))
    except:
        raise Exception('No overlap between lidar DEM and stereo DEM!')


    bestMeanDiff  = -1
    bestMaxDisp   = -1
    bestNumDiffs  = -1
    bestTransPath = os.path.join(pcAlignFolder, 'best_transform.txt')
    bestDemPath   = os.path.join(pcAlignFolder, 'best_dem.tif')

    # Done computing the desired point count, now align our DEM.
    resultsDict = {}
    for maxDisp in DISPLACEMENTS:
               
        # Call pc_align
        alignOptions = ( ('--max-displacement %f --csv-format %s ' +
                          '--save-inv-transformed-reference-points ' + 
                          '--initial-ned-translation "0 0 %f"') % 
                         (maxDisp, lidarCsvFormatString, meanDiff))
        cmd = ('pc_align %s %s %s -o %s %s' %
               (alignOptions, demPath, lidarFile, alignPrefix, threadText))
        try:
            # Redo must be true here
            icebridge_common.logger_print(logger, cmd)
            (out, err, status) = asp_system_utils.executeCommand(cmd, alignedPC, True, True)
            icebridge_common.logger_print(logger, out + '\n' + err)
            
            # Get diff stats for this attempt
            (thisDem, thisLidarDiffPath, results) = \
                makeDemAndCheckError(options, projString, alignedPC, 
                                     lidarFile, lidarCsvFormatString,
                                     threadText, logger)
            transformPath = thisDem.replace('trans_reference-DEM.tif', 'transform.txt')
            
            #numDiffs = getOverlapAmount(thisDem, lidarDemPath, logger)
        except: 
            icebridge_common.logger_print(logger, 'Alignment failed, trying next displacement.')
            continue # Try the next displacement

        #shutil.copyfile(thisDem, str(maxDisp) + '_dem.tif') # DEBUG
        
        # Keep track of the best displacement result, requiring a minimum
        #  improvement in the mean error to accept a higher-displacement result.
        # - Also don't accept results with less lidar points covered because that is 
        #   a sign that the DEM has been tilted away from the LIDAR path.
        thisDiff = results['Mean']
        numDiffs = results['NumDiffs']
        percent_improvement = (bestMeanDiff - thisDiff) / bestMeanDiff
        icebridge_common.logger_print(logger, '\nDiff results for max displacement ' 
                                               + str(maxDisp) + ' = ' + str(results)
                                               + ', num diffs = ' 
                                               + str(numDiffs)
                                               + ', improvement ratio = ' 
                                               + str(percent_improvement))
        resultsDict[str(maxDisp)] = results
        if ( (bestMeanDiff < 0) or 
             (thisDiff < IDEAL_LIDAR_DIST) or
             ((percent_improvement >= MIN_DIST_IMPROVEMENT) and
              (numDiffs >= bestNumDiffs)) ):
            icebridge_common.logger_print(logger, 
                    'Accepting pc_align result for displacement ' + str(maxDisp))
            bestMeanDiff = thisDiff
            bestMaxDisp  = maxDisp
            bestNumDiffs = numDiffs
            shutil.move(thisDem,       bestDemPath  )
            shutil.move(transformPath, bestTransPath)
            
        if thisDiff < IDEAL_LIDAR_DIST:
            break # No need to check more displacements if this result is great

    # Move the best result into the default file locations.
    icebridge_common.logger_print(logger, 'Best mean diff is ' + str(bestMeanDiff) 
                                  +' with --max-displacement = ' + str(bestMaxDisp)
                                  +' and num diffs = ' + str(bestNumDiffs))
    logger.info(resultsDict)
    shutil.move(bestDemPath,   alignedDem)
    shutil.move(bestTransPath, transformPath)

    # Final success check
    if not os.path.exists(alignedPC):
        raise Exception('pc_align call failed!')

    # Apply the same transform to the footprint DEM. This one is used in blending later.
    finalFootprintDEM = os.path.join(options.outputFolder,
                                     icebridge_common.footprintFileName())
    if os.path.exists(finalFootprintDEM):
        logger.info("File exists: " + finalFootprintDEM)
    else:
        alignOptions = ( ('--max-displacement -1 --csv-format %s ' +   \
                          '--save-inv-transformed-reference-points') % \
                            (lidarCsvFormatString))
        alignPrefixFoot = alignPrefix + '-footprint'
        footDemPath = demPath.replace("DEM.tif", "footprint-DEM.tif")
        cmd = ('pc_align --num-iterations 0 --initial-transform %s %s %s %s -o %s %s' %
               (transformPath, alignOptions, footDemPath, lidarFile, alignPrefixFoot, threadText))
        alignedFootPC = alignPrefixFoot + '-trans_reference.tif'
        try:
            logger.info(cmd) # to make it go to the log, not just on screen
            asp_system_utils.executeCommand(cmd, alignedFootPC, False, True)
            if os.path.exists(footDemPath):
                os.remove(footDemPath) # No longer needed
        except: 
            pass

        try:
            # POINT2DEM on the aligned PC file
            cmd = ('point2dem --search-radius-factor 5 --dem-hole-fill-len 500 --tr %lf --t_srs %s %s %s' 
                   % (options.demResolution, projString, alignedFootPC, threadText))
            alignedFootDEM = alignPrefixFoot + '-trans_reference-DEM.tif'
            logger.info(cmd) # to make it go to the log, not just on screen
            asp_system_utils.executeCommand(cmd, alignedFootDEM, False, True)
        except: 
            pass

        # Move it out of the align directory
        if os.path.exists(alignedFootDEM):
            if os.path.exists(finalFootprintDEM):
                os.remove(finalFootprintDEM)
            logger.info("Renaming " + alignedFootDEM + " to " + finalFootprintDEM)
            os.rename(alignedFootDEM, finalFootprintDEM)

        # Wipe all auxilliary footprint files
        for filename in glob.glob(alignPrefix + '-footprint-*'):
            logger.info("Removing: " + filename)
            os.remove(filename)
        
    return alignedDem, lidarDiffPath, results['Mean']


def blurImage(inputPath, outputPath, suppressOutput, redo):
    '''Create a blurred copy of an image'''

    # Blurring the input image can help reduce the negative impact of jpeg artifacts.
    # - The 6x3 option applies a significant amount of blur but it is still not enough
    #   to clean up all the jpeg artifacts!
    cmd = 'convert ' +inputPath+' -compress LZW -blur 6x3 '+outputPath
    asp_system_utils.executeCommand(cmd, outputPath, suppressOutput, redo)


def robustBundleAdjust(options, inputPairs,
                       suppressOutput, redo,
                       threadText, heightLimitString, logger):
    '''Perform bundle adjustment with multiple retries in case things fail.
       Returns inputPairs with the updated camera models swapped in.'''

    # - Bundle adjust all of the input images in the batch at the same time.
    # - An overlap number less than 2 is prone to very bad bundle adjust results so
    #   don't use less than that.  If there is really only enough overlap for one we
    #   will have to examine the results very carefully!
    MIN_BA_OVERLAP       = 2
    TRANSLATION_WEIGHT   = 4.0
    ROBUST_THRESHOLD     = 2.0
    OVERLAP_EXPONENT     = 0
    MIN_IP_MATCHES       = 22
    SIDE_IP_CROP_PERCENT = 20 # Remove IP in 20% of the sides of the images
    bundlePrefix   = icebridge_common.getBundlePrefix(options.outputFolder)
    baOverlapLimit = options.stereoImageInterval + 3
    if baOverlapLimit < MIN_BA_OVERLAP:
        baOverlapLimit = MIN_BA_OVERLAP

    # Try many attempts until one works
    # - Blurring can help with artifacts.
    # - Many IP can also help, but risks getting false matches and is slower.
    ipMethod    = [1,   0,   2,     1,   0,   2,     1,    0,      1,    0,    2,     1   ]
    ipPerTile   = [500, 500, 500,   500, 500, 500,   500,  500,    2000, 2000, 2000,  2000]
    useBlur     = [0,   0,   0,     1,   1,   1,     0,    0,      0,    0,    0,     0   ]
    epipolarT   = [450, 450, 450,   450, 450, 450,   2000, 2000,   450,  450,  450,   2000]
    normIpTiles = [0,   0,   0,     0,   0,   0,     1,    1,      0,    0,    0,     1   ]

    # This is the rough percentage of the image that we want to have interest points in.
    MIN_IP_COVERAGE = 0.70

    if (len(ipMethod) != len(ipPerTile)) or (len(ipMethod) != len(useBlur)) or \
           (len(ipMethod) != len(epipolarT)):
        raise Exception("Book-keeping error in robust bundle adjustment.")

    imageCameraString = formImageCameraString(inputPairs)

    # Fill inputPairs with output camera names
    for pair in inputPairs:
        pair[1] = bundlePrefix +'-'+ os.path.basename(pair[1])
   
    # Generate a new string with the blurred image files paths
    # - Don't actually generate blurred files unless they are needed
    # - Blurred images will be deleted when the final batch cleanup function is called.
    blurredImageCameraString = imageCameraString
    blurPairs = []
    i = 0
    for pair in inputPairs:
        imagePath   = pair[0]
        blurredPath = bundlePrefix + '_' + str(i)+ '_blurred.tif'
        blurredImageCameraString = blurredImageCameraString.replace(imagePath, blurredPath)
        blurPairs.append((imagePath, blurredPath))
        i = i + 1

    outputCamera = inputPairs[0][1]

    # Loop through all our parameter settings, quit as soon as one works.
    bestNumIpPreElevation = 0
    bestCmd = ''
    bestCmdSuccess = False
    success = False
    successWithoutCoverage = False
    for attempt in range(len(ipPerTile)):
               
        argString = imageCameraString
        if useBlur[attempt]: # Make sure blurred images are created
            for pair in blurPairs:
                blurImage(pair[0], pair[1], False, False)
            argString = blurredImageCameraString                     

        cmd = (('bundle_adjust %s -o %s %s %s --datum wgs84 ' +
                '--translation-weight %0.16g -t nadirpinhole --skip-rough-homography ' +
                '--local-pinhole --overlap-limit %d --robust-threshold %0.16g ' +
                '--ip-detect-method %d --ip-per-tile %d --min-matches %d ' + 
                '--overlap-exponent %0.16g --epipolar-threshold %d --ip-side-filter-percent %d ')
               % (argString, bundlePrefix, threadText, heightLimitString, 
                  TRANSLATION_WEIGHT, baOverlapLimit, ROBUST_THRESHOLD, ipMethod[attempt],
                  ipPerTile[attempt], MIN_IP_MATCHES, OVERLAP_EXPONENT, epipolarT[attempt], SIDE_IP_CROP_PERCENT))

        # This helps in cases where there is a small dark object (like a stream) that prevents
        #  large snow areas from being processed well.  Can lead to false IP.
        if normIpTiles[attempt]:
            cmd += ' --normalize-ip-tiles'

        if options.solve_intr:
            cmd += ' --solve-intrinsics'            
            
        # Disabling triangulation error filtering up front and then applying it post bundle adjust
        #  works well but it is currently only supported in bundle_adjust for two cameras at once!
        if len(inputPairs) == 2:
            cmd += ' --disable-tri-ip-filter --num-passes 3 --remove-outliers-params "75.0 3.0 2.0 3.0" '
    
        # Run the BA command and log errors
        logger.info(cmd) # to make it go to the log, not just on screen
        icebridge_common.logger_print(logger, cmd)
        (out, err, status) = asp_system_utils.executeCommand(cmd, outputCamera, True, redo,
                                                             noThrow=True)
        logger.info(out + '\n' + err)
        
        if status == 0: # Record if we had a success before coverage was checked
            successWithoutCoverage = True
        thisSuccessWithoutCoverage = (status == 0)
        
        if status == 0:
            # Check to see if we got the desired amount of coverage.
            # - TODO: How to do this with >2 cameras?
            m = re.findall(r"IP coverage fraction after cleaning = ([0-9e\-\.\+]*)", out)
            if len(m) == 0: # Handle case where no IP were removed for high residual.
                m = re.findall(r"IP coverage fraction = ([0-9e\-\.\+]*)", out)
            if len(m) > 0: # If this text is available...
                ipCoveragePercentage = float(m[-1]) # Use the last instance of this text.
                logger.info('Read coverage percentage: ' + str(ipCoveragePercentage))
                if ipCoveragePercentage < MIN_IP_COVERAGE:
                    logger.info('Coverage percentage is less than required amount: ' 
                                + str(MIN_IP_COVERAGE) + ', trying more IP options to get more coverage.')
                    status = -1
                
        if status == 0:
            logger.info("Bundle adjustment succeded on attempt " + str(attempt))
            success = True
            break

        # Keep track of the best number of IP before elevation filtering.
        m = re.findall(r"Reduced matches to (\d+)", out)
        if len(m) == 1: # If this text is available...
            numIpPreElevation = int(m[0])
            # Mostly want to save a successful command but use numIpPreElevation as a tiebreaker.
            if ((numIpPreElevation > bestNumIpPreElevation) and not (bestCmdSuccess and not thisSuccessWithoutCoverage)) \
               or (thisSuccessWithoutCoverage and not bestCmdSuccess):
                bestNumIpPreElevation = numIpPreElevation
                bestCmd = cmd
                bestCmdSuccess = thisSuccessWithoutCoverage
    
        # Try again. Carefully wipe only relevant files
        logger.info("Trying bundle adjustment again.")
        for f in glob.glob(bundlePrefix + '*'):
            if 'blurred.tif' in f: # Don't wipe these files
                continue
            logger.info("Wipe: " + f)
            os.remove(f)

    # End bundle adjust attempts

    # If we failed, retry the best attempt
    if not success:
        
        # If we never succeeded even before coverage checking, lift the elevation restriction.
        # - This increases the risk of bad IP's being used but it is better than failing.
        if successWithoutCoverage:
            logger.info("Retrying bundle_adjust without coverage check")
            cmd = bestCmd
        else:
            logger.info("Retrying bundle_adjust without coverage check or elevation restriction")
            cmd = bestCmd.replace(heightLimitString, '')

        icebridge_common.logger_print(logger, cmd)
        (out, err, status) = asp_system_utils.executeCommand(cmd, outputCamera, True, redo,
                                                             noThrow=True)
        logger.info(out + '\n' + err)
        if status == 0:
            logger.info("Bundle adjustment succeded on repeat attempt.")
            success = True

    if not success:
        raise Exception('Bundle adjustment failed!\n')

    # Return image/camera pairs with the camera files replaced with the bundle_adjust output files.
    # - Also return if we used blurred input images or not
    return inputPairs

def applyTransformToCameras(options, inputPairs, suppressOutput, redo,
                            threadText, heightLimitString, logger):
    '''Create cameras with the pc_align transform applied to them,
    # so that we can later generate ortho images.'''

    imagesAndCams = formImageCameraString(inputPairs)

    alignPrefix         = icebridge_common.getAlignPrefix(options.outputFolder)
    bundlePrefix        = icebridge_common.getBundlePrefix(options.outputFolder)
    alignedBundlePrefix = icebridge_common.getAlignedBundlePrefix(options.outputFolder)

    outputCamera = inputPairs[0][1].replace(bundlePrefix, alignedBundlePrefix)

    if os.path.exists(outputCamera):
        logger.info("Transformed cameras already exist.")
        return
    
    initialTransform = alignPrefix + '-inverse-transform.txt'
    if not os.path.exists(initialTransform):
        raise Exception("Cannot locate pc_align transform: " + initialTransform)

    # We will run bundle adjustment with 0 iterations. Hence use whatever match files
    # we had before
    matchFiles = glob.glob(bundlePrefix + '*.match')
    for matchFile in matchFiles:
        newMatchFile = matchFile.replace(bundlePrefix, alignedBundlePrefix)
        icebridge_common.makeSymLink(matchFile, newMatchFile)

    cmd = (('bundle_adjust %s -o %s %s --datum wgs84 ' +
            '-t nadirpinhole --skip-rough-homography '+
            '--local-pinhole --min-matches 0  --max-iterations 0 --ip-per-tile 1000 ' + 
            ' --initial-transform %s')
           % (imagesAndCams, alignedBundlePrefix, threadText,
              initialTransform))

    # Run the BA command and log errors
    logger.info("Applying pc_align transform to cameras.")
    logger.info(cmd) # to make it go to the log, not just on screen
    (out, err, status) = asp_system_utils.executeCommand(cmd, outputCamera, True, redo,
                                                         noThrow=True)
    logger.info(out + '\n' + err)

    # Since input cameras start with out- and output prefix starts with out-, the output
    # cameras will start with out-out-. Fix this.
    for camera in glob.glob(alignedBundlePrefix + '*.tsai'):
        cameraOut = camera.replace(alignedBundlePrefix + '-out', alignedBundlePrefix)
        if camera != cameraOut and os.path.exists(camera) and (not os.path.exists(cameraOut)):
            cmd = 'mv ' + camera + ' ' + cameraOut
            logger.info(cmd)
            os.system(cmd)


def getMatchFiles(options, origInputPairs, index):
    '''Get the path of the match file generated by bundle_adjust for
       a given image pair, and the path to where that match file
       could be copied to the stereo folder in order to be used.'''

    # TODO: These definitions are repeated elsewhere
    bundlePrefix     = icebridge_common.getBundlePrefix(options.outputFolder)
    thisOutputFolder = os.path.join(options.outputFolder, 'stereo_pair_'+str(index))
    thisPairPrefix   = os.path.join(thisOutputFolder,     'out')
    
    # Saturate the pair index at the end of the list.
    # - This serves to handle the case where bundle_adjust was called with a
    #   truncated input image list in a one-DEM multiple-image case.
    pairIndex = index + options.stereoImageInterval
    if pairIndex >= len(origInputPairs):
        pairIndex = len(origInputPairs) - 1

    leftImageName  = os.path.basename(origInputPairs[index    ][0])
    rightImageName = os.path.basename(origInputPairs[pairIndex][0])
    normalMatchPath = ('%s-%s__%s.match' % 
                       (bundlePrefix, leftImageName.replace('.tif',''), 
                                      rightImageName.replace('.tif','')))
    blurMatchPath = ('%s-out_%d_blurred__out_%d_blurred.match' % 
                     (bundlePrefix, index, pairIndex))
    filterMatchPath = bundlePrefix + '-clean.match' # Generated by multi-pass option.

    outputMatchPath = ('%s-%s__%s.match' % 
                       (thisPairPrefix, leftImageName.replace('.tif',''), 
                                        rightImageName.replace('.tif','')))

    if os.path.exists(filterMatchPath):
        if index > 0:
            raise Exception('Error: bundle adjust multi-pass is not compatible with >2 frames!')
        inputMatchPath = filterMatchPath
    else:
        if os.path.exists(normalMatchPath):
            inputMatchPath = normalMatchPath
        else:
            if os.path.exists(blurMatchPath):
                inputMatchPath = blurMatchPath
            else:
                # Match files may not exist if the bundle adjust dir got cleaned up
                return ("", "")
                #raise Exception('Unable to find bundle_adjust match file ' + str(index))

    return (inputMatchPath, outputMatchPath)


def consolidateGeodiffResults(inputFiles, outputPath=None):
    '''Create a summary file of multiple geodiff csv output files'''

    if len(inputFiles) == 0: # No input files, do nothing.
        return None

    # Take the max/min of min/max and the mean of mean and stddev
    keywords = ['Max', 'Min', 'Mean', 'StdDev']
    mergedResult = {'Max':-999999.0, 'Min':999999.0, 'Mean':0.0, 'StdDev':0.0}
    for path in inputFiles:
        results = icebridge_common.readGeodiffOutput(path)
        if results['Max'] > mergedResult['Max']:
            mergedResult['Max'] = results['Max']
        if results['Min'] < mergedResult['Min']:
            mergedResult['Min'] = results['Min']
        mergedResult['Mean'  ] += results['Mean'  ]
        mergedResult['StdDev'] += results['StdDev']
    mergedResult['Mean'  ] = mergedResult['Mean'  ] / float(len(inputFiles))
    mergedResult['StdDev'] = mergedResult['StdDev'] / float(len(inputFiles))
    
    if not outputPath:
        return mergedResult
    
    # If an output path was provided, write out the values in a similar to geodiff format.
    with open(outputPath, 'w') as f:
        f.write('# Max difference:       '+str(mergedResult['Max'   ])+'\n')
        f.write('# Min difference:       '+str(mergedResult['Min'   ])+'\n')
        f.write('# Mean difference:      '+str(mergedResult['Mean'  ])+'\n')
        f.write('# StdDev of difference: '+str(mergedResult['StdDev'])+'\n')

    # Delete all the input diff files to reduce file bloat if we wrote the output file
    if os.path.exists(outputPath): 
        for f in inputFiles:
            os.system('rm -f ' + f)
    
    return mergedResult


def consolidateStats(lidarDiffPath, interDiffPath, fireDiffPath, fireLidarDiffPath,  
                     demPath, outputPath, fractionValidPath, logger, skipGeo = False):
    '''Consolidate statistics into a single file'''

    # Read in the diff results            
    try:
        lidarDiffResults = icebridge_common.readGeodiffOutput(lidarDiffPath)
    except:
        lidarDiffResults = {'Mean':-999}
    try:
        interDiffResults = icebridge_common.readGeodiffOutput(interDiffPath)
    except:
        interDiffResults = {'Mean':-999}
    try:
        fireDiffResults  = icebridge_common.readGeodiffOutput(fireDiffPath)
    except:
        fireDiffResults  = {'Mean':-999}
    try:
        fireLidarDiffResults = icebridge_common.readGeodiffOutput(fireLidarDiffPath)
    except:
        fireLidarDiffResults = {'Mean':-999}
    try:
        with open(fractionValidPath, 'r') as f:
            fractionValid = float(f.read())
    except:
        fractionValid = -1

    # Get DEM stats
    success = True
    if skipGeo:
        success = False
    else:
        try:
            geoInfo = asp_geo_utils.getImageGeoInfo(demPath, getStats=False)
            stats   = asp_image_utils.getImageStats(demPath)[0]
            meanAlt = stats[2]
            centerX, centerY = geoInfo['projection_center']
            
            # Convert from projected coordinates to lonlat coordinates            
            isSouth    = ('+lat_0=-90' in geoInfo['proj_string'])
            projString = icebridge_common.getEpsgCode(isSouth, asString=True)
            PROJ_STR_WGS84 = 'EPSG:4326'
            centerLon, centerLat = asp_geo_utils.convertCoords(centerX, centerY,
                                                               projString, PROJ_STR_WGS84)
        except Exception, e:
            pass
            # Print nothing, comes out too verbose
            #if logger:
            #    #logger.exception('Caught exception getting DEM center coordinates:\n' + str(e))
            #    #logger.info("Not fatal, will continue.") # for clarity in the log use this line
            #else:
            #    #print 'Could not compute DEM center coordinates.'
            success = False

    if not success:
        centerLon = 0
        centerLat = 0
        meanAlt   = -999

    # Write info to summary file        
    with open(outputPath, 'w') as f:
        f.write('%f, %f, %f, %f, %f, %f, %f, %f' % 
                 (centerLon, centerLat, meanAlt, 
                  lidarDiffResults['Mean'], interDiffResults    ['Mean'],
                  fireDiffResults ['Mean'], fireLidarDiffResults['Mean'], fractionValid))


def readConsolidatedStatsFile(consolidatedStatsPath):
    '''Read the contents of the stats file'''

    if not os.path.exists(consolidatedStatsPath):
        raise Exception('Consolidated stats file missing: ' + consolidatedStatsPath)
    with open(consolidatedStatsPath, 'r') as f:
        statsText = f.read()
    parts = split(statsText)
    return [float(x) for x in parts]
    


def lidarCsvToDem(lidarFile, projBounds, projString, outputFolder, threadText, 
                  redo, logger):
    '''Generate a DEM from a lidar file in the given region (plus a buffer)'''

    LIDAR_DEM_RESOLUTION     = 5 # TODO: Vary this
    LIDAR_PROJ_BUFFER_METERS = 100

    lidarCsvFormatString = icebridge_common.getLidarCsvFormat(lidarFile)

    # Buffer out the input bounds
    minX = projBounds[0] - LIDAR_PROJ_BUFFER_METERS # Expand the bounds a bit
    minY = projBounds[1] - LIDAR_PROJ_BUFFER_METERS
    maxX = projBounds[2] + LIDAR_PROJ_BUFFER_METERS
    maxY = projBounds[3] + LIDAR_PROJ_BUFFER_METERS

    # Generate a DEM from the lidar point cloud in this region        
    lidarDemPrefix = os.path.join(outputFolder, 'cropped_lidar')
    cmd = ('point2dem --max-output-size 10000 10000 --t_projwin %f %f %f %f --tr %lf --t_srs %s %s %s --csv-format %s -o %s' 
           % (minX, minY, maxX, maxY,
              LIDAR_DEM_RESOLUTION, projString, lidarFile, threadText, 
              lidarCsvFormatString, lidarDemPrefix))
    lidarDemOutput = lidarDemPrefix+'-DEM.tif'
    icebridge_common.logger_print(logger, cmd)
    (out, err, status) = asp_system_utils.executeCommand(cmd, lidarDemOutput, True, redo, noThrow=True)
    logger.info(out + '\n' + err)
    if status != 0:
        raise Exception('Did not generate any lidar DEM!')
    
    return lidarDemOutput

def cropGdalImage(projBounds, inputPath, outputPath, logger):
    '''Crop out a section of an image'''

    PROJ_BUFFER_METERS = 100
    minX = projBounds[0] - PROJ_BUFFER_METERS # Expand the bounds a bit
    minY = projBounds[1] - PROJ_BUFFER_METERS
    maxX = projBounds[2] + PROJ_BUFFER_METERS
    maxY = projBounds[3] + PROJ_BUFFER_METERS

    cmd = ('gdal_translate %s %s -projwin %f %f %f %f' 
           % (inputPath, outputPath, minX, maxY, maxX, minY))
    logger.info(cmd)
    asp_system_utils.executeCommand(cmd, outputPath, True, False)

def estimateHeightRange(projBounds, projString, lidarFile, options, threadText, 
                        suppressOutput, redo, logger):
    '''Estimate the valid height range in a region based on input height info.
       Also generates the lidar DEM.'''
        
    # Expand the estimate by this much in either direction
    # - If the input cameras are good then this can be fairly small, at least for flat
    #   regions.  Bad cameras are much farther off.
    HEIGHT_BUFFER = 20
    
    # Create a lidar DEM at the region
    lidarDemPath = lidarCsvToDem(lidarFile, projBounds, projString, 
                                 options.outputFolder, threadText, 
                                 redo, logger)
    
    # Get the min and max height of the lidar file
    try:
        lidarMin, lidarMax, lidarMean, lidarStd = asp_image_utils.getImageStats(lidarDemPath)[0]
        logger.info('Found lidar height min = %f, max = %f' % (lidarMin, lidarMax))
    except:
        raise Exception('Failed to generate lidar DEM to estimate height range!')

    minHeight = lidarMin
    maxHeight = lidarMax

    # Get the min/max height in the reference DEM region
    # - The lidar field is narrower than the image so sometimes it can miss terrain features
    #   that show up in the lower resolution DEM.
    try:
        demCropPath = os.path.join(options.outputFolder, 'cropped_ref_dem.tif')
        cropGdalImage(projBounds, options.referenceDem, demCropPath, logger)
        refDemMin, refDemMax, refDemMean, refDemStd = asp_image_utils.getImageStats(demCropPath)[0]
        logger.info('Found ref DEM height min = %f, max = %f' % (refDemMin, refDemMax))
        
        if refDemMin < minHeight:
            minHeight = refDemMin
        if refDemMax > maxHeight:
            maxHeight = refDemMax
            
    except:
        logger.warning('Error generating reference DEM height estimate for ')

    minHeight = minHeight - HEIGHT_BUFFER
    maxHeight = maxHeight + HEIGHT_BUFFER
    
    # Generate the height string
    s = '--elevation-limit ' + str(minHeight) +' '+ str(maxHeight)
    return (s, lidarDemPath)
    

def getWidthAndMemUsageFromStereoOutput(outputText, errorText):
    '''Parse the output from running stereo and return the search range width and
       the memory usage in GB.'''

    successParsintStats = False
    corrSearchWidth = -1
    memUsage        = -1
    elapsed         = "-1"
    out             = outputText + "\n" + errorText
    for line in out.split('\n'):
        m = re.match("^.*?Search\s+Range:.*?Origin:.*?width:\s*(\d+)", line, re.IGNORECASE)
        if m:
            corrSearchWidth = float(m.group(1))
            successParsintStats = True
        m = re.match("^.*?elapsed=(.*?)\s+mem=(\d.*?)\s+.*?time_stereo_corr", line, re.IGNORECASE)
        if m:
            elapsed = m.group(1)
            memUsage = float(m.group(2))
            memUsage = float(round(memUsage/100000.0))/10.0 # convert to GB
            successParsintStats = True
            
    return (corrSearchWidth, memUsage, elapsed, successParsintStats)

def createDem(i, options, inputPairs, prefixes, demFiles, projString,
              heightLimitString, threadText, matchFilePair,
              suppressOutput, redo, logger=None):
    '''Create a DEM from a pair of images'''

    # Since we use epipolar alignment our images should be aligned at least this well.
    VERTICAL_SEARCH_LIMIT = 10
    
    # Get the appropriate image to use as a stereo pair    
    pairIndex = i + options.stereoImageInterval

    thisPairPrefix = prefixes[i]
    argString      = ('%s %s %s %s ' % (inputPairs[i][0],  inputPairs[pairIndex][0], 
                                        inputPairs[i][1],  inputPairs[pairIndex][1]))

    # Testing: Is there any performance hit from using --corr-seed-mode 0 ??
    #          This skips D_sub creation and saves processing time.
    # - This epipolar threshold is post camera model based alignment so it can be quite restrictive.
    # - Note that the base level memory usage ignoring the SGM buffers is about 2 GB so this memory
    #   usage is in addition to that.
    minIpString = '--min-num-ip 40'
    stereoCmd = ('stereo %s %s %s %s -t nadirpinhole --alignment-method epipolar --skip-rough-homography --corr-blob-filter 50 --corr-seed-mode 0 --epipolar-threshold 10 %s ' %
                 (argString, thisPairPrefix, threadText, heightLimitString, minIpString))
    searchLimitString = (' --corr-search-limit -9999 -' + str(VERTICAL_SEARCH_LIMIT) +
                         ' 9999 ' + str(VERTICAL_SEARCH_LIMIT) )
    if '--stereo-algorithm 0' not in options.stereoArgs:
        correlationArgString = (' --xcorr-threshold 2 --corr-kernel 7 7 ' 
                                + ' --corr-tile-size 9000 --cost-mode 4 --sgm-search-buffer 4 2 '
                                + searchLimitString + ' --corr-memory-limit-mb 6000 '
                                + options.stereoArgs
                               )
        #+ ' --corr-blob-filter 100')
        filterArgString = (' --rm-cleanup-passes 0 --median-filter-size 5 ' +
                           ' --texture-smooth-size 17 --texture-smooth-scale 0.14 ')
    else:
        correlationArgString = options.stereoArgs
        filterArgString = ''
         
    stereoCmd += correlationArgString
    stereoCmd += filterArgString
    stereoCmd += ' --check-mem-usage'

    # Call and check status
    triOutput = thisPairPrefix + '-PC.tif'
    icebridge_common.logger_print(logger, stereoCmd)
    (out, err, status) = asp_system_utils.executeCommand(stereoCmd, triOutput, suppressOutput, redo, noThrow=True)
    
    if status != 0:
        # If stereo failed, try it again with the .match file that was created by bundle_adjust.
        icebridge_common.logger_print(logger, 'First stereo attempt failed, will copy .match file from bundle_adjust and retry.')
        
        # Clear any existing .match file then link in the new one.
        os.system('rm -f ' + thisPairPrefix + '*.match')
        if matchFilePair[0] == "":
            # This can happen if the bundle adjust directory got cleaned up. Nothing we can do.
            raise Exception("No usable match files. Stereo call failed.")
        
        icebridge_common.makeSymLink(matchFilePair[0], matchFilePair[1])
        if not os.path.exists(matchFilePair[1]):
            raise Exception('Failed to create .match file symlink: ' + matchFilePair[1])
            
        # With the .match file copied we can retry with the same parameters.
        # - Remove some filtering steps we don't need.
        # - Exception is the height limit string, which we can remove if using existing IP.
        stereoCmd = stereoCmd.replace(minIpString, '--min-num-ip 10')
        stereoCmd = stereoCmd.replace(heightLimitString, ' ')
        icebridge_common.logger_print(logger, stereoCmd)
        os.system('rm -f ' + triOutput) # In case the output cloud exists but is bad
        (out, err, status) = asp_system_utils.executeCommand(stereoCmd, triOutput, suppressOutput, 
                                                             redo, noThrow=True)
        if status != 0:
            # If we fail again give up.
            icebridge_common.logger_print(logger, out + '\n' + err)
            raise Exception('Stereo call failed!')

    # Extract the search range width and memory usage from the output text.
    (corrSearchWidth, memUsage, elapsed, gotMemStats) = getWidthAndMemUsageFromStereoOutput(out, err)

    icebridge_common.logger_print(logger, ("Corr search width: %d mem usage: %f GB elapsed: %s" %
                                          (corrSearchWidth, memUsage, elapsed) ) )

    if i == 0 and gotMemStats:
        # If we could not parse the data, write nothing. Maybe this time
        # we are rerunning things, and did not actually do any work.
        filePath = os.path.join(os.path.dirname(os.path.dirname(thisPairPrefix)),
                                icebridge_common.getRunStatsFile())
        icebridge_common.logger_print(logger, "Writing: " + filePath)
        with open(filePath, 'w') as f:
            f.write( ("%d, %f, %s\n") % (corrSearchWidth, memUsage, elapsed) )
        
    # point2dem on the result of ASP
    # - The size limit is to prevent bad point clouds from creating giant DEM files which
    #   cause the processing node to crash.
    cmd = ('point2dem --max-output-size 10000 10000 --tr %lf --t_srs %s %s %s'
           % (options.demResolution, projString, triOutput, threadText))
    p2dOutput = demFiles[i]
    icebridge_common.logger_print(logger, cmd)
    (out, err, status) =  asp_system_utils.executeCommand(cmd, p2dOutput, suppressOutput, redo, noThrow=True)
    if status != 0:
        icebridge_common.logger_print(logger, out + '\n' + err)
        raise Exception('point2dem call on stereo pair failed!')

    # Require a certain percentage of valid output pixels to go forwards with this DEM
    # - This calculation currently does not work well but anything under this is probably bad.
    # TODO: This validity fraction is NOT ACCURATE and needs to be improved!
    MIN_FRACTION_VALID_PIXELS = 0.35 
    percentageFlagFile = os.path.join(options.outputFolder, 'valid_pixel_fraction.txt')
    fractionValid = 1.0;

    # Try to parse the output text for the percentage or read it from disk if we already logged it.
    m = re.findall(r"Percentage of valid pixels = ([0-9e\-\.\+]+)", out)
    if len(m) == 1:
        fractionValid = float(m[0])
        logger.info('Valid DEM pixel fraction = ' + str(fractionValid))
        with open(percentageFlagFile, 'w') as f: # Log the percentage to disk
            f.write(str(fractionValid))
    else:
        try: # Read the percentage from disk
            with open(percentageFlagFile, 'r') as f:
                fractionValid = float(f.read())
        except:
            logger.warning('Unable to read dem percentage fraction from file ' + percentageFlagFile)

           
    logger.info('Detected valid pixel fraction = ' + str(fractionValid))
    if fractionValid < MIN_FRACTION_VALID_PIXELS:
        raise Exception('Required DEM pixel fraction is ' + str(MIN_FRACTION_VALID_PIXELS) +
                        ', aborting processing on this DEM.')

    # The DEM with larger footprint, not filtered out as agressively. We use
    # the valid pixels in this DEM's footprint as a template where to blend.
    p2dFoot = thisPairPrefix + '-footprint'
    cmd = ( ('point2dem --max-output-size 10000 10000 --tr %lf --t_srs %s %s %s ' +
             ' --remove-outliers-params 75 12 -o %s ')
           % (options.demResolution, projString, triOutput, threadText, p2dFoot))
    p2dFoot = p2dFoot + '-DEM.tif'
    icebridge_common.logger_print(logger, cmd)
    (out, err, status) =  asp_system_utils.executeCommand(cmd, p2dFoot, suppressOutput, redo, noThrow=True)
    if status != 0:
        icebridge_common.logger_print(logger, out + '\n' + err)
        raise Exception('point2dem call on stereo pair failed!')

def cleanBatch(batchFolder, alignPrefix, stereoPrefixes,
               interDiffPaths, fireballDiffPaths):
    '''Clean up all non-output files to conserve space.
       Setting smallFiles will remove additional low size files.'''

    smallFiles = True

    # Delete all of the stereo folders
    for s in stereoPrefixes:
        if smallFiles:
            folder = os.path.dirname(s)
            os.system('rm -rf ' + folder)
        else:
            os.system('rm -rf ' + s + '*.tif')
    
    if smallFiles:
        # Delete bundle_adjust folder. Note that will also wipe the cameras.
        os.system('rm -rf ' + os.path.dirname(icebridge_common.getBundlePrefix(batchFolder)))
        
        # Clean out the pc_align folder
        alignFiles = ['-beg_errors.csv', '-end_errors.csv', '-iterationInfo.csv',
                      '-trans_reference.tif', '-transform.txt', '-inverse-transform.txt'] 

        for currFile in alignFiles:
            os.system('rm -f ' + alignPrefix + currFile)

        logFiles = glob.glob(alignPrefix + '*-log-*')
        for logFile in logFiles:
            os.system('rm -f ' + logFile)

        # Wipe all footprint files except the final result
        footprintFiles = glob.glob(alignPrefix + '*footprint*')
        for footprintFile in footprintFiles:
            if not footprintFile.endswith('out-footprint-trans_reference-DEM.tif'):
                os.system('rm -f ' + footprintFile)

    # Delete the diff images
    for f in (interDiffPaths + fireballDiffPaths):
        os.system('rm -f ' + f)

    # Delete dangling link.
    os.system('rm -f ' + os.path.join(batchFolder, 'out-DEM.tif'))

    # Delete lidar_crop files
    lidar_crop_glob = os.path.join(batchFolder, '*cropped_lidar*')
    for filename in glob.glob(lidar_crop_glob):
        os.system('rm -f ' + filename)

    # Delete the cropped DEM path
    demCropPath = os.path.join(batchFolder, 'cropped_ref_dem.tif')
    os.system('rm -f ' + demCropPath)

    # Wipe the pc_aligned bundle directory except for the final results
    alignedBundlePrefix = icebridge_common.getAlignedBundlePrefix(batchFolder)
    for filename in glob.glob(alignedBundlePrefix + '*'):
        if not filename.endswith('.tsai'):
            os.system('rm -f ' + filename)

    # Wipe the entire align directory, we moved out of it everything by now
    os.system('rm -rf ' + os.path.dirname(alignPrefix))
    
    # Repeating the logic from generate_flight_summary.py.
    consolidatedStatsPath = os.path.join(batchFolder, 'out-consolidated_stats.txt')
    if os.path.exists(consolidatedStatsPath):
        # Then we don't need the individual files
        lidarDiffPath      = os.path.join(batchFolder, 'out-diff.csv')
        interDiffPath      = os.path.join(batchFolder, 'out_inter_diff_summary.csv')
        fireDiffPath       = os.path.join(batchFolder, 'out_fireball_diff_summary.csv')
        fireLidarDiffPath  = os.path.join(batchFolder, 'out_fireLidar_diff_summary.csv')
        percentageFlagFile = os.path.join(batchFolder, 'valid_pixel_fraction.txt')
        for filename in [lidarDiffPath, interDiffPath, fireDiffPath, fireLidarDiffPath, percentageFlagFile]:
            os.system('rm -f ' + filename)

    # Wipe any aux.xml
    for filename in glob.glob(os.path.join(batchFolder, '*aux.xml')):
        os.system('rm -f ' + filename)
    
def main(argsIn):
    '''Handle arguments then call doWork function'''

    try:
        usage = '''usage: process_icebridge_batch <imageA> <imageB> [imageC ...] <cameraA> <cameraB> [cameraC ...]'''
        
        parser = optparse.OptionParser(usage=usage)

        # Data options
        parser.add_option('--south', action='store_true', default=False, dest='isSouth',  
                          help='MUST be set if the images are in the southern hemisphere.')
                          
        parser.add_option('--lidar-folder', default=None, dest='lidarFolder',  
                          help='Use pc-align to match the closest lidar file.')

        parser.add_option('--output-folder', default=None, dest='outputFolder',  
                          help='The folder used for output.')

        parser.add_option('--reference-dem', default=None, dest='referenceDem',  
                          help='Low resolution DEM used for certain checks.')
                          
        parser.add_option('--fireball-folder', default=None, dest='fireballFolder',
                          help='Folder containing fireball DEMs.')

        # Processing options
        parser.add_option('--max-displacement', dest='maxDisplacement', default=20,
                          type='float', help='Max displacement value passed to pc_align.')

        parser.add_option('--solve-intrinsics', action='store_true', default=False,
                          dest='solve_intr',  
                          help='If to float the intrinsics params.')

        parser.add_option('--stereo-arguments', dest='stereoArgs', default='',
                          help='Additional argument string to be passed to the stereo command.')
                          
        parser.add_option('--stereo-image-interval', dest='stereoImageInterval', default=1,
                          type='int', help='Advance this many frames to get the stereo pair. ' + \
                          ' Also sets bundle adjust overlap limit.')

        # Output options
        parser.add_option('--dem-resolution', dest='demResolution', default=0.4,
                          type='float', help='Generate output DEMs at this resolution.')

        parser.add_option('--cleanup', action='store_true', default=False, dest='cleanup',  
                          help='If the final result is produced delete intermediate files.')

        # Performance options
        parser.add_option('--num-threads', dest='numThreads', default=None,
                          type='int', help='The number of threads to use for processing.')

        parser.add_option('--num-processes-per-batch', dest='numProcessesPerBatch', default=1,
                          type='int', help='The number of simultaneous processes to run ' + \
                          'for each batch. This better be kept at 1 if running more than one batch.')

        (options, args) = parser.parse_args(argsIn)

        # Check argument count
        numArgs    = len(args)
        numCameras = (numArgs) / 2
        if ( (2*numCameras - numArgs) != 0) or (numCameras < 2):
            print("Expecting as many images as cameras. Got: " + " ".join(args))
            print usage
            return 0

    except optparse.OptionError, msg:
        raise Usage(msg)

    # Start up the logger, output will go in the output folder.
    #logger = logging.getLogger(__name__)
    logLevel = logging.INFO # Make this an option??
    asp_system_utils.mkdir_p(options.outputFolder)
    logger = icebridge_common.setUpLogger(options.outputFolder, logLevel, 'icebridge_batch_log')

    logger.info('Input arguments: ' + str(argsIn))

    # Run the rest of the code and log any unhandled exceptions.
    try:
        doWork(options, args, logger)
        return 0 # Success!
        
    except Exception, e:
        logger.exception(e) # Failed to generate output file
        
        try: # When we failed to generate the output DEM, 
             #  generate a thumbnail of the first input image to help diagnose problems.
            thumbOutput = os.path.join(options.outputFolder, 'first_image_browse.tif')
            cmd = 'gdal_translate '+args[0]+' '+thumbOutput+' -of GTiff -outsize 40% 40% -b 1 -co "COMPRESS=JPEG"'
            asp_system_utils.executeCommand(cmd, thumbOutput, True, False)
            logger.info('Created browse image ' + thumbOutput)
        except:
            logger.exception('Failed to generate debug input image thumbnail.')
        
        return -1 # Failure!
    

def doWork(options, args, logger):
    '''Do all of the processing.'''

    os.system("ulimit -c 0")  # disable core dumps
    os.system("rm -f core.*") # these keep on popping up
    os.system("umask 022")    # enforce files be readable by others

    numArgs       = len(args)
    numCameras    = (numArgs) / 2
    numStereoRuns = numCameras - options.stereoImageInterval

    # Verify all input files exist
    for i in range(0,numArgs):
        if not os.path.exists(args[i]):
            logger.error('Arg parsing error: Input file '+ args[i] +' does not exist!')
            return 0

    # Parse input files
    inputPairs   = []
    for i in range(0, numCameras):
        image  = args[i]
        camera = args[i + numCameras]
        inputPairs.append([image, camera])
    imageCameraString = ' '.join(args)

    projString = icebridge_common.getProjString(options.isSouth, addQuotes=False)

    suppressOutput = False
    redo           = False

    logger.info('Starting processing...')


    threadText = ''
    if options.numThreads:
        threadText = ' --threads ' + str(options.numThreads) +' '

    # If a lidar folder was specified, find the best lidar file.
    lidarFile = None
    if options.lidarFolder:
        logger.info('Searching for matching lidar file...')
        lidarFile = icebridge_common.findMatchingLidarFile(inputPairs[0][0], options.lidarFolder)
        logger.info('Found matching lidar file ' + lidarFile)
        lidarCsvFormatString = icebridge_common.getLidarCsvFormat(lidarFile)

    outputPrefix  = os.path.join(options.outputFolder, 'out')

    # Check the last output products from this script.  If they exist,
    #  quit now so we don't regenerate intermediate products.
    consolidatedStatsPath = outputPrefix + '-consolidated_stats.txt'
    finalAlignedDEM        = outputPrefix + '-align-DEM.tif'

    try: # If the consolidated stats file exists but is incomplete, remove it so that we go ahead with processing.
        stats = readConsolidatedStatsFile(consolidatedStatsPath)
        if (stats[0] == 0) or (stats[1] == 0) or (stats[2] == -999):
            os.remove(consolidatedStatsPath)
    except:
        pass

    if ( os.path.exists(consolidatedStatsPath) and 
         os.path.exists(finalAlignedDEM) and not redo ):
        logger.info('Final output files already exists: ' + finalAlignedDEM +
                    ' and ' + consolidatedStatsPath + '. Quitting script early.')
    
        # Include the same normal completion message
        logger.info('Finished script process_icebridge_batch!') 
        return
    
    # Check that the output GSD is not set too much lower than the native resolution
    heightLimitString = ''
    lidarDemPath      = None # Path to a DEM created from lidar data.
    if options.referenceDem:
        MAX_OVERSAMPLING = 3.0
        computedGsd = options.demResolution
        meanGsd     = 0
        totalBounds = [99999999, 99999999, -99999999, -999999999] # minX, minY, maxX, maxY
        for i in range(0,numCameras):
            try:
                # Compute the native GSD of the first input camera
                computedGsd, bounds = icebridge_common.getCameraGsdAndBoundsRetry(
                                          inputPairs[i][0], inputPairs[i][1], logger, 
                                          options.referenceDem, projString)
                meanGsd += computedGsd
                # Accumulate the bounding box
                minX = bounds[0]
                minY = bounds[1]
                maxX = minX + bounds[2]
                maxY = minY + bounds[3]
                if totalBounds[0] > minX: totalBounds[0] = minX
                if totalBounds[1] > minY: totalBounds[1] = minY
                if totalBounds[2] < maxX: totalBounds[2] = maxX
                if totalBounds[3] < maxY: totalBounds[3] = maxY
            except:
                logger.warning('Failed to compute GSD for camera: ' + inputPairs[0][1])
        meanGsd = meanGsd / numCameras                
        #print 'GSD = ' + str(meanGsd)
        #print 'TotalBounds = ' + str(totalBounds)
        if options.demResolution < (meanGsd*MAX_OVERSAMPLING):
            logger.warning('Specified resolution ' + str(options.demResolution) + 
                           ' is too fine for camera with computed GSD ' + str(meanGsd) +
                           '.  Switching to native GSD.)')
            options.demResolution = meanGsd*MAX_OVERSAMPLING
        # Undersampling is not as dangerous, just print a warning.
        if options.demResolution > 5*meanGsd:
            logger.warning('Specified resolution ' + str(options.demResolution) + 
                           ' is much larger than computed GSD ' + str(meanGsd))
                           
        if lidarFile:
            # Compute a good height limit from the reference DEM
            # - Can try generating lonlat bounds in the future, but maybe better
            #   to keep these in projected coordinate space.
            (heightLimitString, lidarDemPath) = estimateHeightRange(totalBounds,
                                                    projString, lidarFile,
                                                    options, threadText, 
                                                    suppressOutput, redo, logger)
       
    # BUNDLE_ADJUST
    origInputPairs = inputPairs # All input pairs, non-blurred or otherwise altered.
    
    # For now at least only pass into bundle adjust the images which will be used in stereo.
    # - This lets use use some BA tools which are currently only supported for two images.
    if numStereoRuns == 1:
        prunedInputPairs = [ inputPairs[0], inputPairs[-1]]
    else: # Bundle on all input images
        prunedInputPairs = inputPairs
    inputPairs = robustBundleAdjust(options, prunedInputPairs,
                                    suppressOutput, redo,
                                    threadText, heightLimitString, logger)

    # Record the best match file generated by bundle_adjust for each output pair
    #  and where it can be copied for stereo to re-use it.
    baMatchFiles = []
    for i in range(0,len(prunedInputPairs)-1):
        (inputMatch, outputMatch) = getMatchFiles(options, prunedInputPairs, i)
        baMatchFiles.append( (inputMatch, outputMatch))

    # Generate a map of post-bundle camera positions
    orbitvizAfter = os.path.join(options.outputFolder, 'cameras_out.kml')
    vizString  = ''
    for (image, camera) in inputPairs: 
        vizString += camera + ' '
    cmd = ('orbitviz_pinhole --hide-labels -o ' +orbitvizAfter + ' '+ vizString)
    logger.info(cmd) # to make it go to the log, not just on screen
    asp_system_utils.executeCommand(cmd, orbitvizAfter, suppressOutput, redo)

    # STEREO
    
    # Call stereo seperately on each pair of cameras and create a DEM

    # Check if we have all of the stereo output files
    prefixes              = []
    demFiles              = []
    fireLidarDiffCsvPaths = []                      

    # Load index of fireball DEMs for comparison
    if options.fireballFolder:
        fireballFrameDict = icebridge_common.getCorrectedFireballDems\
                            (os.path.dirname(options.fireballFolder))
        matchingFireballDems = []
        
    atLeastOneDemMissing = False
    for i in range(0, numStereoRuns):
        thisOutputFolder = os.path.join(options.outputFolder, 'stereo_pair_'+str(i))
        thisPairPrefix   = os.path.join(thisOutputFolder,     'out')
        prefixes.append(thisPairPrefix)
        p2dOutput = thisPairPrefix + '-DEM.tif'
        demFiles.append(p2dOutput)
        if not os.path.exists(p2dOutput):
            atLeastOneDemMissing = True

        # Diff with fireball early, in case our run fails
        currCam = inputPairs[i][1]
        frame   = icebridge_common.getFrameNumberFromFilename(currCam)
        if frame not in fireballFrameDict.keys():
            matchingFireballDems.append(None) # Store flag object
            continue
        fireball = fireballFrameDict[frame]
        matchingFireballDems.append(fireball)
        if lidarFile:                           
            prefix  = outputPrefix + '_fireball_lidar_' + str(i)
            csvPath = prefix + "-diff.csv"
            cmd = ('geodiff --absolute --csv-format %s %s %s -o %s' % 
                   (lidarCsvFormatString, fireball, lidarFile, prefix))
            logger.info(cmd) # to make it go to the log, not just on screen
            asp_system_utils.executeCommand(cmd, csvPath, suppressOutput, redo)
            fireLidarDiffCsvPaths.append(csvPath)
        
    # We can either process the batch serially, or in parallel For
    # many batches the former is preferred, with the batches
    # themselves being in parallel.
    if options.numProcessesPerBatch > 1:
        logger.info('Starting processing pool for given batch with ' +
                    str(options.numProcessesPerBatch) + ' processes.')
        pool = multiprocessing.Pool(options.numProcessesPerBatch)
        taskHandles = []
        for i in range(0, numStereoRuns):
            taskHandles.append(pool.apply_async(createDem, 
                                                (i, options, origInputPairs, prefixes, demFiles,
                                                 projString, heightLimitString, threadText, baMatchFiles[i],
                                                 suppressOutput, redo)))
        # Wait for all the tasks to complete
        icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, logger, interactive = False, 
                                                         quitKey='q', sleepTime=20)
        
        # Either all the tasks are finished or the user requested a cancel.
        icebridge_common.stopTaskPool(pool)
    else:
        for i in range(0, numStereoRuns):
            createDem(i, options, origInputPairs, prefixes, demFiles, projString, heightLimitString,
                      threadText, baMatchFiles[i], suppressOutput, redo, logger)

    # If we had to create at least one DEM, need to redo all the post-DEM creation steps
    if atLeastOneDemMissing:
        redo = True

    logger.info('Finished running all stereo instances.')
    
    numDems = len(demFiles)

    # Check the elevation disparities between the DEMs.  High discrepancy
    # usually means there was an alignment error.
    interDiffSummaryPath  = outputPrefix + '_inter_diff_summary.csv'
    interDiffPaths        = []
    
    for i in range(1,numDems):
        try:
            # Call geodiff
            prefix   = outputPrefix + '_inter_dem_' + str(i)
            diffPath = prefix + "-diff.tif"
            cmd = ('geodiff --absolute %s %s -o %s' % (demFiles[0], demFiles[i], prefix))
            logger.info(cmd) # to make it go to the log, not just on screen
            asp_system_utils.executeCommand(cmd, diffPath, suppressOutput, redo)
            
            # Read in and examine the results
            results = icebridge_common.readGeodiffOutput(diffPath)
            interDiffPaths.append(diffPath)
        except:
            pass # Files with no overlap will fail here
            #logger.warning('Difference between dem ' + demFiles[0] + \
            # ' and dem ' + demFiles[i] + ' failed!')

    # Can do interdiff only if there is more than one DEM
    if numDems > 1:
        consolidateGeodiffResults(interDiffPaths, interDiffSummaryPath)
    else:
        logger.info("Only one stereo pair is present, cannot create: " + interDiffSummaryPath)
        
    # DEM_MOSAIC
    allDemPath = outputPrefix + '-DEM.tif'
    if numDems == 1:
        # If there are only two files just skip this step
        icebridge_common.makeSymLink(demFiles[0], allDemPath)

        # For the footprint. We implemented this only for batch size of 2.
        demFoot = demFiles[0].replace("DEM.tif", "footprint-DEM.tif")
        if os.path.exists(demFoot):
            allDemFoot = allDemPath.replace("DEM.tif", "footprint-DEM.tif")
            icebridge_common.makeSymLink(demFoot, allDemFoot)
    else:
        demString = ' '.join(demFiles)
        # Only the default blend method produces good results but the DEMs must not be too 
        #  far off for it to work.
        print projString
        cmd = ('dem_mosaic %s --tr %lf --t_srs %s %s -o %s' 
               % (demString, options.demResolution, projString, threadText, outputPrefix))
        print cmd
        mosaicOutput = outputPrefix + '-tile-0.tif'
        logger.info(cmd) # to make it go to the log, not just on screen
        asp_system_utils.executeCommand(cmd, mosaicOutput, suppressOutput, redo)
        # TODO: Updated fractionValid in this case
        
        # Create a symlink to the mosaic file with a better name
        icebridge_common.makeSymLink(mosaicOutput, allDemPath)

    # More comparisons with Fireball DEMs if available
    fireballDiffPaths     = []
    if options.fireballFolder:
        fireballDiffSummaryPath  =  outputPrefix + '_fireball_diff_summary.csv'
        fireLidarDiffSummaryPath =  outputPrefix + '_fireLidar_diff_summary.csv'
    
        # Loop through matches
        # - Each fireball DEM is compared to our final output DEM as without the pc_align step
        #   the errors will be high and won't mean much.
        for i in range(0,numDems):
            dem = demFiles[i]
            fireball = matchingFireballDems[i]
            if not fireball: # Skip missing fireball file (will be 'None')
                continue
            #try:
            prefix  = outputPrefix + '_fireball_' + str(i)
            diffPath = prefix + "-diff.tif"
            cmd = ('geodiff --absolute %s %s -o %s' % (allDemPath, fireball, prefix))
            logger.info(cmd)
            try:
                asp_system_utils.executeCommand(cmd, diffPath, suppressOutput, redo)
            except Exception, e:
                # This is necessary, sometimes the fireball DEM is wrong
                logger.info('Caught exception doing diff to fireball: ' + str(e))
                logger.info("Not fatal, will continue.") # for clarity in the log, use this line
                continue
            
            results = icebridge_common.readGeodiffOutput(diffPath)
            fireballDiffPaths.append(diffPath)
    
    
            #except:
            #    logger.warning('Difference between dem ' + demFiles[0] + ' and fireball failed!')
        consolidateGeodiffResults(fireballDiffPaths,     fireballDiffSummaryPath )
        consolidateGeodiffResults(fireLidarDiffCsvPaths, fireLidarDiffSummaryPath)

    lidarDiffPath = ''
    if lidarFile:
        # PC_ALIGN

        # - Use function to call with increasing max distance limits
        alignedDem, lidarDiffPath, meanErr = \
                    robustPcAlign(options, outputPrefix,
                                  lidarFile, lidarDemPath, allDemPath, finalAlignedDEM,
                                  projString, lidarCsvFormatString, 
                                  threadText, logger)
        
        # Move the aligned DEM to the main directory, to have one file less
        if os.path.exists(alignedDem):
            logger.info("Moving " + alignedDem + " to " + finalAlignedDEM)
            if os.path.exists(finalAlignedDEM):
                os.remove(finalAlignedDEM)
            os.rename(alignedDem, finalAlignedDEM)
        allDemPath = finalAlignedDEM

        # Create pc-aligned bundle-adjusted cameras
        applyTransformToCameras(options, inputPairs, suppressOutput, redo,
                                threadText, heightLimitString, logger)
                                
    # Consolidate statistics into a one line summary file
    percentageFlagFile = os.path.join(options.outputFolder, 'valid_pixel_fraction.txt')
    consolidateStats(lidarDiffPath, interDiffSummaryPath, 
                     fireballDiffSummaryPath, fireLidarDiffSummaryPath,  
                     allDemPath, consolidatedStatsPath, percentageFlagFile, logger)

    if options.cleanup and os.path.exists(finalAlignedDEM):
        # Delete large files that we don't need going forwards.
        alignPrefix = icebridge_common.getAlignPrefix(options.outputFolder)
        cleanBatch(options.outputFolder,
                   alignPrefix, prefixes, interDiffPaths, fireballDiffPaths)

    logger.info('Finished script process_icebridge_batch!')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

