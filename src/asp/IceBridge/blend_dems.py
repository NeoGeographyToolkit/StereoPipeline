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

# For each DEM, blend it, within its footprint, with neighboring DEMs.
# That is to say, make several mosaics. First the DEM alone. Then
# blended with the one on the right. Then also with the one on the
# left. Then with second on the right. Then with second on the left.
# Keep the result with the lowest mean error to lidar.

# It creates files of the form:
# processed/batch_2489_2490_2/out-blend-DEM.tif 
# processed/batch_2489_2490_2/out-blend-DEM-diff.csv

# Operate in the range [startFrame, stopFrame) so does not include the
# last one.

# See usage below.

# TODO: Find a better way of picking DEMs to blend. For example,
# include only those which decrease the mean error. Then compare
# if this approach gives lower error than the current way
# which keeps on adding DEMs left and right. 

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing, re, glob
import traceback
import os.path as P

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

import icebridge_common
import asp_system_utils, asp_alg_utils, asp_geo_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath      + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath        + os.pathsep + os.environ["PATH"]

def runBlend(frame, processFolder, lidarFile, fireballDEM, bundleLength,
             threadText, redo, suppressOutput):

    # This will run as multiple processes. Hence have to catch all exceptions:
    try:
        
        demFile, batchFolder = icebridge_common.frameToFile(frame, icebridge_common.alignFileName(),
                                                            processFolder, bundleLength)
        lidarCsvFormatString = icebridge_common.getLidarCsvFormat(lidarFile)

        if demFile == "":
            print("Could not find DEM for frame: " + str(frame))
            return

        # The names for the final results
        finalOutputPrefix = os.path.join(batchFolder, 'out-blend-DEM')
        finalBlend        = finalOutputPrefix + '.tif'
        finalDiff         = finalOutputPrefix + "-diff.csv"

        fireballOutputPrefix = os.path.join(batchFolder, 'out-blend-fb-footprint')
        fireballBlendOutput  = fireballOutputPrefix + '-tile-0.tif'
        finalFireballOutput  = fireballOutputPrefix + '-DEM.tif'
        fireballDiffPath     = fireballOutputPrefix + "-diff.csv"

        if not redo:
            set1Exists = False
            if (os.path.exists(finalBlend) and os.path.exists(finalDiff)):
                print("Files exist: " + finalBlend + " " + finalDiff + ".")
                set1Exists = True
                
            set2Exists = True
            if fireballDEM != "":
                if (os.path.exists(finalFireballOutput) and os.path.exists(fireballDiffPath)):
                    print("Files exist: " + finalFireballOutput + " " + fireballDiffPath + ".")
                    set2Exists = True
                else:
                    set2Exists = False

            if set1Exists and set2Exists:
                return

        # We will blend the dems with frame offsets within frameOffsets[0:index]
        filesToWipe = []
        bestMean = 1.0e+100
        bestBlend  = ''
        bestVals = ''
        bestDiff = ''
        
        # Look at frames with these offsets when blending
        frameOffsets = [0, 1, -1, 2, -2]

        for index in range(len(frameOffsets)):

            # Find all the DEMs up to the current index
            dems = []
            currDemFile = ""
            for val in range(0, index+1):
                offset = frameOffsets[val]
                currDemFile, currBatchFolder = \
                             icebridge_common.frameToFile(frame + offset,
                                                          icebridge_common.alignFileName(),
                                                          processFolder, bundleLength)
                if currDemFile == "":
                    continue
                dems.append(currDemFile)

            if currDemFile == "":
                # The last DEM was not present. Hence this iteration will add nothing new.
                continue

            demString = " ".join(dems)
            outputPrefix = os.path.join(batchFolder, 'out-blend-' + str(index))

            # See if we have a pre-existing DEM to use as footprint
            footprintDEM = os.path.join(batchFolder, 'out-trans-footprint-DEM.tif')
            blendOutput = outputPrefix + '-tile-0.tif'
            if os.path.exists(footprintDEM):
                cmd = ('dem_mosaic --this-dem-as-reference %s %s %s -o %s' 
                       % (footprintDEM, demString, threadText, outputPrefix))
                #filesToWipe.append(footprintDEM) # no longer needed
            else:
                cmd = ('dem_mosaic --first-dem-as-reference %s %s -o %s' 
                       % (demString, threadText, outputPrefix))
                
            print(cmd)

            # Sometimes there is junk left from a previous interrupted run. So if we
            # got so far, recreate all files.
            localRedo = True
            asp_system_utils.executeCommand(cmd, blendOutput, suppressOutput, localRedo)
            filesToWipe.append(blendOutput)
            
            diffPath = outputPrefix + "-diff.csv"
            filesToWipe.append(diffPath)
            
            cmd = ('geodiff --absolute --csv-format %s %s %s -o %s' % 
                   (lidarCsvFormatString, blendOutput, lidarFile, outputPrefix))
            print(cmd)
            asp_system_utils.executeCommand(cmd, diffPath, suppressOutput, redo)
                
            # Read in and examine the results
            results = icebridge_common.readGeodiffOutput(diffPath)
            print("Current mean error to lidar is " + str(results['Mean']))

            if  bestMean > float(results['Mean']):
                bestMean  = float(results['Mean'])
                bestBlend = blendOutput
                bestVals  = demString
                bestDiff  = diffPath

            logFiles = glob.glob(outputPrefix + "*" + "-log-" + "*")
            filesToWipe += logFiles
        
        # Update the filenames of the output files
        print("Best mean error to lidar is " + str(bestMean) + " when blending " + bestVals)
        cmd = "mv " + bestBlend + " " + finalBlend
        print(cmd)
        asp_system_utils.executeCommand(cmd, finalBlend, suppressOutput, redo)
        
        cmd = "mv " + bestDiff   + " " + finalDiff
        print(cmd)
        asp_system_utils.executeCommand(cmd, finalDiff, suppressOutput, redo)
        
        # Generate a thumbnail of the final DEM
        hillOutput = finalOutputPrefix+'_HILLSHADE.tif'
        cmd = 'hillshade ' + finalBlend +' -o ' + hillOutput
        asp_system_utils.executeCommand(cmd, hillOutput, suppressOutput, redo)

        # Generate a low resolution compressed thumbnail of the hillshade for debugging
        thumbOutput = finalOutputPrefix + '_HILLSHADE_browse.tif'
        cmd = 'gdal_translate '+hillOutput+' '+thumbOutput+' -of GTiff -outsize 40% 40% -b 1 -co "COMPRESS=JPEG"'
        asp_system_utils.executeCommand(cmd, thumbOutput, suppressOutput, redo)
        os.remove(hillOutput) # Remove this file to keep down the file count

        # Do another blend, to this DEM's footprint, but not using it
        if fireballDEM != "":
            
            # Find all the DEMs
            dems = []
            for val in range(0, len(frameOffsets)):
                offset = frameOffsets[val]
                currDemFile, currBatchFolder = \
                             icebridge_common.frameToFile(frame + offset,
                                                          icebridge_common.alignFileName(),
                                                          processFolder, bundleLength)
                if currDemFile == "":
                    continue
                dems.append(currDemFile)
                
            demString = " ".join(dems)
            cmd = ('dem_mosaic --this-dem-as-reference %s %s %s -o %s' 
                   % (fireballDEM, demString, threadText, fireballOutputPrefix))
            
            #filesToWipe.append(fireballBlendOutput)

            print(cmd)

            # Sometimes there is junk left from a previous interrupted run. So if we
            # got so far, recreate all files.
            localRedo = True
            asp_system_utils.executeCommand(cmd, fireballBlendOutput, suppressOutput, localRedo)

            #filesToWipe.append(fireballDiffPath)

            cmd = ('geodiff --absolute --csv-format %s %s %s -o %s' % 
                   (lidarCsvFormatString, fireballBlendOutput, lidarFile, fireballOutputPrefix))
            print(cmd)
            asp_system_utils.executeCommand(cmd, fireballDiffPath, suppressOutput, redo)

            # Read in and examine the results
            results = icebridge_common.readGeodiffOutput(fireballDiffPath)
            print("Mean error to lidar in fireball footprint is " + str(results['Mean']))
            
            cmd = "mv " + fireballBlendOutput   + " " + finalFireballOutput
            print(cmd)
            asp_system_utils.executeCommand(cmd, finalFireballOutput, suppressOutput, redo)

            # Generate a thumbnail of the final DEM
            #hillOutput = fireballOutputPrefix+'_HILLSHADE.tif'
            #cmd = 'hillshade ' + finalFireballOutput +' -o ' + hillOutput
            #print(cmd)
            #asp_system_utils.executeCommand(cmd, hillOutput, suppressOutput, redo)

            ## Generate a low resolution compressed thumbnail of the hillshade for debugging
            #thumbOutput = fireballOutputPrefix + '_HILLSHADE_browse.tif'
            #cmd = 'gdal_translate '+hillOutput+' '+thumbOutput+' -of GTiff -outsize 40% 40% -b 1 -co "COMPRESS=JPEG"'
            #asp_system_utils.executeCommand(cmd, thumbOutput, suppressOutput, redo)
            #os.remove(hillOutput) # Remove this file to keep down the file count
            
            logFiles = glob.glob(fireballOutputPrefix + "*" + "-log-" + "*")
            filesToWipe += logFiles

        # Done with dealing with the fireball footprint
        
        # Clean up extra files
        for fileName in filesToWipe:
            if os.path.exists(fileName):
                print("Removing: " + fileName)
                os.remove(fileName)
                
    except Exception as e:
        print('Blending failed!\n' + str(e) + ". " + str(traceback.print_exc()))
            
    sys.stdout.flush()
         
def main(argsIn):

    try:
        # Sample usage:
        # python ~/projects/StereoPipeline/src/asp/IceBridge/blend_dems.py --site GR   \
        #   --yyyymmdd 20120315 --start-frame 2490 --stop-frame 2491 --bundle-length 2 \
        #   --num-threads 8 --num-processes 10
        usage = '''blend_dems.py <options>'''
                      
        parser = argparse.ArgumentParser(usage=usage)

        # Run selection
        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL)")

        parser.add_argument("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        # Processing options
        parser.add_argument('--bundle-length', dest='bundleLength', default=2,
                          type=int, help="The number of images to bundle adjust and process " + \
                          "in a single batch.")

        parser.add_argument('--start-frame', dest='startFrame', type=int,
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                          default=icebridge_common.getLargestFrame(),
                          help='Frame to stop on. This frame will also be processed.')

        parser.add_argument("--processing-subfolder",  dest="processingSubfolder", default=None,
                          help="Specify a subfolder name where the processing outputs will go. "+\
                            "The default is no additional folder.")

        parser.add_argument("--blend-to-fireball-footprint", action="store_true",
                            dest="blendToFireball", default=False,
                            help="Create additional blended DEMs having the same " + \
                            "footprint as Fireball DEMs.")

        # Performance options  
        parser.add_argument('--num-processes', dest='numProcesses', default=1,
                          type=int, help='The number of simultaneous processes to run.')
        parser.add_argument('--num-threads', dest='numThreads', default=8,
                          type=int, help='The number of threads per process.')
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError, msg:
        parser.error(msg)
        
    icebridge_common.switchWorkDir()
    
    os.system("ulimit -c 0") # disable core dumps
    os.system("rm -f core.*") # these keep on popping up
    os.system("umask 022")   # enforce files be readable by others
    
    if len(options.yyyymmdd) != 8 and len(options.yyyymmdd) != 9:
        # Make an exception for 20100422a
        raise Exception("The --yyyymmdd field must have length 8 or 9.")

    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)
        
    os.system('mkdir -p ' + options.outputFolder)
    logLevel = logging.INFO # Make this an option??
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel,
                                            'icebridge_blend_log')

    (out, err, status) = asp_system_utils.executeCommand(['uname', '-a'],
                                                         suppressOutput = True)
    logger.info("Running on machine: " + out)
    
    processFolder = os.path.join(options.outputFolder, 'processed')
    
    # Handle subfolder option.  This is useful for comparing results with different parameters!
    if options.processingSubfolder:
        processFolder = os.path.join(processFolder, options.processingSubfolder)
        logger.info('Reading from processing subfolder: ' + options.processingSubfolder)

    orthoFolder = icebridge_common.getOrthoFolder(options.outputFolder)
    orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
    if not os.path.exists(orthoIndexPath):
        raise Exception("Error: Missing ortho index file: " + orthoIndexPath + ".")
    (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath)
    
    if options.blendToFireball:
        fireballFrameDict = icebridge_common.getCorrectedFireballDems(options.outputFolder)
        
    lidarFolder = icebridge_common.getLidarFolder(options.outputFolder)
    
    threadText = ''
    if options.numThreads:
        threadText = '--threads ' + str(options.numThreads)
    
    redo = False
    suppressOutput = True
    taskHandles  = []
    if options.numProcesses > 1:    
        pool = multiprocessing.Pool(options.numProcesses)

    # Bound the frames
    sortedFrames = sorted(orthoFrameDict.keys())
    if len(sortedFrames) > 0:
        if options.startFrame < sortedFrames[0]:
            options.startFrame = sortedFrames[0]
        if options.stopFrame > sortedFrames[-1] + 1:
            options.stopFrame = sortedFrames[-1] + 1
    else:
        # No ortho files, that means nothing to do
        options.startFrame = 0
        options.stopFrame  = 0 

    for frame in range(options.startFrame, options.stopFrame):

        if not frame in orthoFrameDict:
            logger.info("Error: Missing ortho file for frame: " + str(frame) + ".")
            continue
        
        orthoFile = orthoFrameDict[frame]
        lidarFile = icebridge_common.findMatchingLidarFile(orthoFile, lidarFolder)

        fireballDEM = ""
        if options.blendToFireball:
            if frame in fireballFrameDict:
                fireballDEM = fireballFrameDict[frame]
            else:
                logger.info("No fireball DEM for frame: " + str(frame))
            
        args = (frame, processFolder, lidarFile, fireballDEM, options.bundleLength, threadText,
                redo, suppressOutput)

        # Run things sequentially if only one process, to make it easy to debug
        if options.numProcesses > 1:
            taskHandles.append(pool.apply_async(runBlend, args))
        else:
            runBlend(*args)
        
    if options.numProcesses > 1:
        icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, logger, interactive = False, 
                                                         quitKey='q', sleepTime=20)
        icebridge_common.stopTaskPool(pool)
    
# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


