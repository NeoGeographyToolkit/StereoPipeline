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

# Top level program to process all of the Icebridge data.
# - This program is not sophisticated enough to handle everything and will need to be
#   superceded by another script.

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing
import re, shutil, time, getpass, argparse

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

import icebridge_common, pbs_functions, archive_functions, run_helper, lvis2kml
import process_icebridge_batch
import asp_system_utils, asp_geo_utils, asp_image_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

# TODO: Move this function!
def getLastLog(logPrefix):
    '''Return the path to the latest log file matching a prefix'''
    
    # Get all the files in the folder containing the prefix
    folder   = os.path.dirname(logPrefix)
    logFiles = os.listdir(folder)
    logFiles = [os.path.join(folder, x) for x in logFiles]
    logFiles = [x for x in logFiles if logPrefix in x]

    # No matches found!
    if not logFiles:
        return None
    
    # Go through the remaining files and find the one with the latest modify time
    # - Assumes the modify time is a good replacement for the stamp in the file name.
    latestFile = logFiles[0]
    latestTime = os.path.getmtime(latestFile)
    for log in logFiles:
        thisTime = os.path.getmtime(log)
        if thisTime > latestTime:
            latestTime = thisTime
            latestFile = log

    return latestFile
    
    

def getFailureCause(batchFolder):
    '''Try to figure out the reason that a DEM failed'''
    
    # Define a list of failure reasons and text descriptions
    UNKNOWN       = -1
    SUCCESS       = 0
    FAIL_FILE_MISSING = 1
    FAIL_CAMERA_MISSING = 2
    FAIL_LIDAR_DEM_TOO_LARGE = 3
    FAIL_LIDAR    = 4
    FAIL_BUNDLE   = 5
    FAIL_STEREO_NO_POINTS = 6
    FAIL_STEREO   = 7
    FAIL_STEREO_POINT2DEM = 8
    FAIL_PC_ALIGN = 9
    FAIL_FRACTION_VALID = 10
    FAIL_NO_LIDAR_DEM_OVERLAP = 11
    
    errorSummaries = {} # Human readable error codes
    errorSummaries[UNKNOWN      ] = 'Unknown failure'
    errorSummaries[SUCCESS      ] = 'Success'
    errorSummaries[FAIL_FILE_MISSING] = 'Missing argument file'
    errorSummaries[FAIL_CAMERA_MISSING] = 'Failed to generate camera file'
    #errorSummaries[FAIL_LIDAR_DEM_TOO_LARGE] = 'LIDAR DEM would be too large'
    errorSummaries[FAIL_LIDAR   ] = 'Failed to generate lidar DEM'
    errorSummaries[FAIL_BUNDLE  ] = 'Bundle adjust failed'
    errorSummaries[FAIL_STEREO_NO_POINTS] = 'Generated empty stereo point cloud'
    errorSummaries[FAIL_STEREO  ] = 'Stereo failed'
    errorSummaries[FAIL_STEREO_POINT2DEM] = 'Other stereo point2dem failure'
    errorSummaries[FAIL_PC_ALIGN] = 'pc_align failed'
    errorSummaries[FAIL_FRACTION_VALID] = 'Too few valid pixels'
    errorSummaries[FAIL_NO_LIDAR_DEM_OVERLAP] = 'No lidar-DEM overlap'
    
    errorLogText = {} # Text in the log file that indicates an error occurred
    errorLogText[SUCCESS      ] = 'Finished script process_icebridge_batch!'
    errorLogText[FAIL_FILE_MISSING] = 'Arg parsing error: Input file'
    errorLogText[FAIL_CAMERA_MISSING] = 'Not enough input pairs exist to continue, quitting!'
    #errorLogText[FAIL_LIDAR_DEM_TOO_LARGE] = 'Requested DEM size is too large'
    errorLogText[FAIL_LIDAR   ] = 'Failed to generate lidar DEM to estimate height range!'
    errorLogText[FAIL_BUNDLE  ] = 'Bundle adjustment failed!'
    errorLogText[FAIL_STEREO_NO_POINTS] = 'OrthoRasterize: Input point cloud is empty!'
    errorLogText[FAIL_STEREO  ] = 'Stereo call failed!'
    errorLogText[FAIL_STEREO_POINT2DEM] = 'point2dem call on stereo pair failed!'
    errorLogText[FAIL_PC_ALIGN] = 'Unable to find a good value for max-displacement in pc_align.'
    errorLogText[FAIL_FRACTION_VALID] = 'Required DEM pixel fraction is'
    errorLogText[FAIL_NO_LIDAR_DEM_OVERLAP] = 'No overlap between lidar DEM and stereo DEM'

    foundError = UNKNOWN

    logPrefix = os.path.join(batchFolder, 'icebridge_batch_log_')
    latestLog = getLastLog(logPrefix)
    if not latestLog:
        print("Cannot find log matching: " + logPrefix)
        return (foundError, errorSummaries[foundError])
    
    # Search for errors in chronological order
    logText = ''
    with open(latestLog, 'r') as log:
        logText = log.read()
    
    for code in errorLogText.iterkeys():
        if code in (UNKNOWN, SUCCESS): #Skip these codes
            continue

        if errorLogText[code] in logText:
            foundError = code
            break # Stop at the first error we find
       
    return (foundError, errorSummaries[foundError])
      

def generateFlightSummary(run, options):
    '''Generate a folder containing handy debugging files including output thumbnails'''
    
    # Copy logs to the output folder
    print 'Copying log files...'
    badImageFolder  = os.path.join(options.outputFolder, 'badImages')
    runFolder       = run.getFolder()
    procFolder      = run.getProcessFolder()
    navCameraFolder = run.getNavCameraFolder()
    os.system('mkdir -p ' + options.outputFolder)
    os.system('mkdir -p ' + badImageFolder)
    
    packedErrorLog = os.path.join(runFolder, 'packedErrors.log')
    if os.path.exists(packedErrorLog):
        try:
            shutil.copy(packedErrorLog, options.outputFolder)
        except Exception, e:
            # In case it complains about copying a file onto itself
            print("Warning: " + str(e))
            
    if not options.skipKml:
        # Copy the input camera kml file
        camerasInKmlPath = os.path.join(procFolder, 'cameras_in.kml')
        try:
            shutil.copy(camerasInKmlPath, options.outputFolder)
        except Exception, e:
            # In case it complains about copying a file onto itself
            print("Warning: " + str(e))

        # Copy the input camera kml file
        navCamerasKmlPath = os.path.join(navCameraFolder, 'nav_cameras.kml')
        try:
            shutil.copy(navCamerasKmlPath, options.outputFolder)
        except Exception, e:
            # In case it complains about copying a file onto itself
            print("Warning: " + str(e))
        
        # Create a merged version of all the bundle adjusted camera files
        # - The tool currently includes cameras more than once if they appear
        #   in multiple bundles.
        print 'Merging output camera kml files...'
        cmd = "find "+procFolder+" -name cameras_out.kml"
        p = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, shell=False)
        textOutput, err = p.communicate()
        camKmlFiles = textOutput.replace('\n', ' ')

        # Write the list of files to process to disk. Otherwise, if we just
        # pass the full list on the command line, it may be too big
        # and the call will fail.
        kmlList = os.path.join(options.outputFolder, 'kml_list.txt')
        print("Writing: " + kmlList)
        with open(kmlList, 'w') as f:
            for filename in sorted(camKmlFiles.split()):
                filename = filename.strip()
                if filename == "":
                    continue
                f.write(filename + "\n")
                
        outputKml = os.path.join(options.outputFolder, 'cameras_out.kml')
        scriptPath = icebridge_common.fullPath('merge_orbitviz.py')
        cmd = 'python ' + scriptPath + ' ' + outputKml + ' -list ' + kmlList
        print(cmd)
        os.system(cmd)
        
        # Generate lidar kml files
        print("Generating lidar kml files")
        LIDAR_POINT_SKIP = 1527
        lidarFiles = run.getLidarList(prependFolder=True)
        lidarOutputFolder = os.path.join(options.outputFolder, 'lidar')
        os.system('mkdir -p ' + lidarOutputFolder)
        for f in lidarFiles:
            inputPath = os.path.splitext(f)[0] + '.csv'
            outputPath = os.path.join(lidarOutputFolder, os.path.basename(f)+'.kml')
            args = [inputPath, outputPath, '--skip', str(LIDAR_POINT_SKIP), '--color', 'red']
            if not os.path.exists(outputPath): # Don't recreate these files
                print("Generating: " + outputPath) # This can be very slow, so print what is going on
                try:
                    lvis2kml.main(args)
                except Exception, e:
                    # Do not let this make our life miserable
                    print("Problem: " + str(e))
       
    # Collect per-batch information
    batchInfoPath   = os.path.join(options.outputFolder, 'batchInfoSummary.csv')
    failedBatchPath = os.path.join(options.outputFolder, 'failedBatchList.csv')
    print("Writing statistics to: " + batchInfoPath)
    print("Writing failures to: " + failedBatchPath)
    batchInfoLog = open(batchInfoPath, 'w')
    failureLog   = open(failedBatchPath, 'w')

    # Write the header for the batch log file
    batchInfoLog.write('# startFrame, stopFrame, centerLon, centerLat, meanAlt, ' +
                       'meanLidarDiff, meanInterDiff, meanFireDiff, meanFireLidarDiff, ' +
                       'percentValid, meanBlendDiff, meanBlendDiffInFireballFootprint, ' + \
                       'corrSearchWid, corrMem(GB), corrElapsedTime(minutes)\n')
    failureLog.write('# startFrame, stopFrame, errorCode, errorText\n')

    demList = run.getOutputFileList(icebridge_common.blendFileName())
    for (dem, frames) in demList:

        demFolder = os.path.dirname(dem)

        # Handle frame range option
        if (frames[0] < options.startFrame):
            continue
        if (frames[1] > options.stopFrame):
            break

        # Progress indication
        if frames[0] % 100 == 0:
            print("Frame: " + str(frames[0]))
            batchInfoLog.flush() # for instant gratification
            failureLog.flush()

        # Read in blend results which are not part of the consolidated stats file
        blendDiffPath = os.path.join(demFolder, 'out-blend-DEM-diff.csv')
        try:
            blendDiffResults = icebridge_common.readGeodiffOutput(blendDiffPath)
        except:
            blendDiffResults = {'Mean':-999}

        # Read in blend results which are not part of the consolidated stats file
        # for the blend done in the fireball footprint
        fireballBlendDiffPath = os.path.join(demFolder, 'out-blend-fb-footprint-diff.csv')

        try:
            fireballBlendDiffResults = icebridge_common.readGeodiffOutput(fireballBlendDiffPath)
        except:
            fireballBlendDiffResults = {'Mean':-999}

        runStatsFile = os.path.join(demFolder, icebridge_common.getRunStatsFile())
        statsLine = icebridge_common.readStats(runStatsFile)
        
        # All of the other results should be in a consolidated stats file
        consolidatedStatsPath = os.path.join(demFolder, 'out-consolidated_stats.txt')

        if not os.path.exists(consolidatedStatsPath):
            # Stats file not present, recreate it.

            print 'Recreating missing stats file: ' + consolidatedStatsPath

            # Get paths to the files of interest
            # This logic must be sync-ed up with cleanBatch().
            lidarDiffPath     = os.path.join(demFolder, 'out-diff.csv')
            interDiffPath     = os.path.join(demFolder, 'out_inter_diff_summary.csv')
            fireDiffPath      = os.path.join(demFolder, 'out_fireball_diff_summary.csv')
            fireLidarDiffPath = os.path.join(demFolder, 'out_fireLidar_diff_summary.csv')
            fractionValidPath = os.path.join(demFolder, 'valid_pixel_fraction.txt')
            process_icebridge_batch.consolidateStats(lidarDiffPath, interDiffPath, fireDiffPath,
                                                     fireLidarDiffPath, dem,
                                                     consolidatedStatsPath,
                                                     fractionValidPath, None, options.skipGeo)
        # Now the consolidated file should always be present

        with open(consolidatedStatsPath, 'r') as f:
            statsText = f.read()

        # Write info to summary file
        batchInfoLog.write('%d, %d, %s, %f, %f, %s\n' % 
                           (frames[0], frames[1], statsText,
                            blendDiffResults['Mean'], fireballBlendDiffResults['Mean'], statsLine))

        # Keep a list of batches that did not generate an output DEM
        parts = statsText.split(',')
        if (float(parts[0]) == 0) and (float(parts[1]) == 0) and (float(parts[2]) == -999):

            if os.path.exists(dem): # Handle the case where the statistics are bad for some reason
                errorCode = 0
                errorText = 'Success but statistics are bad'
            else: # A real failure, figure out the cause
                batchFolder = os.path.dirname(dem)
                (errorCode, errorText) = getFailureCause(batchFolder)
                #print str((errorCode, errorText))
            #if errorCode < 0: # Debug code for unknown errors
                #print str((errorCode, errorText))
                #print statsText
                #print batchFolder
                #raise Exception('DEBUG')
            failureLog.write('%d, %d, %d, %s\n' %  (frames[0], frames[1], errorCode, errorText))


        # Make a link to the DEM thumbnail file in our summary folder
        hillshadePath = os.path.join(demFolder, 'out-blend-DEM_HILLSHADE_browse.tif')
        if os.path.exists(hillshadePath):
            thumbName = ('dem_%05d_%05d_browse.tif' % (frames[0], frames[1]))
            thumbPath = os.path.join(options.outputFolder, thumbName)
            icebridge_common.makeSymLink(hillshadePath, thumbPath, verbose=False)
        else:
            # If the DEM thumbnail does not exist, look for the input frame thumbnail.
            inPath    = os.path.join(demFolder, 'first_image_browse.tif')
            thumbName = ('input_%05d_browse.tif' % (frames[0]))
            thumbPath = os.path.join(badImageFolder, thumbName)
            if os.path.exists(inPath):
                icebridge_common.makeSymLink(inPath, thumbPath, verbose=False)                

        # Make a link to the ortho thumbnail file in our summary folder
        orthoPath = os.path.join(demFolder,  icebridge_common.orthoPreviewFileName())
        if os.path.exists(orthoPath):
            thumbName = ('ortho_%05d_%05d_browse.jpg' % (frames[0], frames[1]))
            thumbPath = os.path.join(options.outputFolder, thumbName)
            icebridge_common.makeSymLink(orthoPath, thumbPath, verbose=False)

    # End loop through expected DEMs and writing log files
    batchInfoLog.close()
    failureLog.close()
    
    print 'Finished generating flight summary in folder: ' + options.outputFolder
    print("Wrote: " + batchInfoPath)
    print("Wrote: " + failedBatchPath)

# The parent folder is where the runs AN_... and GR_..., etc., are
# stored. Usually it is the current directory.
def main(argsIn):
    '''Parse arguments and call the processing function'''

    try:
        # Sample usage:
        # python generate_flight_summary.py --yyyymmdd 20091016 --site AN 
        usage = '''generate_flight_summary.py <options>'''
                      
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL).")

        parser.add_argument("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        parser.add_argument("--parent-folder",  dest="parentFolder", default=os.getcwd(),
                            help="The folder having all the runs.")

        parser.add_argument("--skip-kml-gen", action="store_true", dest="skipKml", default=False, 
                            help="Skip combining kml files.")

        parser.add_argument("--skip-geo-center", action="store_true", dest="skipGeo", default=False, 
                            help="Skip computing the center of the tile, which is slow.")
        
        parser.add_argument('--start-frame', dest='startFrame', type=int,
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                          default=icebridge_common.getLargestFrame(),
                          help='Frame to stop on.')

        options = parser.parse_args(argsIn)
        
    except argparse.ArgumentError, msg:
        parser.error(msg)
        
    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)

    run = run_helper.RunHelper(options.site, options.yyyymmdd, options.parentFolder)
    
    generateFlightSummary(run, options)
    
    return 0

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
