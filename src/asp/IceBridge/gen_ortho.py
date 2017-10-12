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

# For a given frame, blend the two DEMs containing that frame, and one
# more to the left and to the right to ensure we have enough coverage,
# and then mapproject onto this obtained DEM the current image
# and current bundle-adjusted and aligned camera.

# It creates a file of the form:
# processed/batch_2489_2490_2/out-ortho.tif

# The resolution is auto-determined per each image.

# Operate in the range [startFrame, stopFrame) so does not include the
# last one.

# See usage below.

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

def runOrtho(frame, processFolder, imageFile, bundleLength,
             threadText, redo, suppressOutput):

    os.system("ulimit -c 0") # disable core dumps
    os.system("rm -f core.*") # these keep on popping up
    os.system("umask 022")   # enforce files be readable by others

    # This will run as multiple processes. Hence have to catch all exceptions:
    try:

        alignCamFile, batchFolder = \
                      icebridge_common.frameToFile(frame,
                                                   icebridge_common.alignedBundleStr() + 
                                                   '*' + str(frame) + '.tsai',
                                                   processFolder, bundleLength)

        if alignCamFile == "":
            print("Could not find aligned camera for frame: " + str(frame))
            return

        # To ensure we mapproject the image fully, mosaic the several DEMs
        # around it. Keep the closest 5. Try to grab more first to account
        # for skipped frames.
        frameOffsets = [0, 1, -1, 2, -2, -3, 3, -4, 4]
        dems = []
        for offset in frameOffsets: 
            demFile, batchFolder = icebridge_common.frameToFile(frame + offset,
                                                                icebridge_common.blendFileName(),
                                                                processFolder, bundleLength)

            # If the central DEM is missing, we are out of luck
            if offset == 0 and demFile == "":
                print("Could not find DEM for frame: " + str(frame + offset))
                return

            if demFile == "":
                # Missing DEM
                continue

            if len(dems) >= 5:
                break # too many already
            
            dems.append(demFile)
            
        demList = " ".join(dems)

        # Call this one more time, to get the current batch folder
        currDemFile, batchFolder = icebridge_common.frameToFile(frame,
                                                                icebridge_common.blendFileName(),
                                                                processFolder, bundleLength)

        # The names for the final results
        finalOrtho = os.path.join(batchFolder, icebridge_common.orthoFileName())
        finalOrthoPreview = os.path.join(batchFolder, icebridge_common.orthoPreviewFileName())
        
        if (not redo) and os.path.exists(finalOrtho):
            print("File exists: " + finalOrtho + ".")
        else:
            # See if we have a pre-existing DEM to use as footprint
            mosaicPrefix = os.path.join(batchFolder, 'out-temp-mosaic')
            mosaicOutput = mosaicPrefix + '-tile-0.tif'
            filesToWipe = []
            cmd = ('dem_mosaic --hole-fill-length 100 %s %s -o %s' 
                   % (demList, threadText, mosaicPrefix))
            filesToWipe.append(mosaicOutput) # no longer needed

            print(cmd)
            localRedo = True # The file below should not exist unless there was a crash
            asp_system_utils.executeCommand(cmd, mosaicOutput, suppressOutput, localRedo)

            # TODO: Also blend into this mosaic the "-footprint-" file
            # with a lower priority blending length. Just want to make
            # sure there is always enough real estate to mapproject
            # onto.
            # TODO: Look at more aggressive hole-filling. But need a testcase.
            
            filesToWipe += glob.glob(mosaicPrefix + '*' + '-log-' + '*')

            # First mapproject to create a tif image with 4 channels.
            # Then pull 3 channels and compress them as jpeg, while keeping the
            # image a geotiff.

            tempOrtho = os.path.join(batchFolder, icebridge_common.orthoFileName() + "_tmp.tif")

            # There is no need for this file to exist unless it is stray junk
            if os.path.exists(tempOrtho):
                os.remove(tempOrtho)
                
            # Run mapproject. The grid size is auto-determined.
            cmd = ('mapproject --no-geoheader-info %s %s %s %s %s' 
                   % (mosaicOutput, imageFile, alignCamFile, tempOrtho, threadText))
            print(cmd)
            asp_system_utils.executeCommand(cmd, tempOrtho, suppressOutput, redo)
            filesToWipe.append(tempOrtho)

            # This makes the images smaller than Rose's by a factor of about 4,
            # even though both types are jpeg compressed. Rose's images filtered
            # through this command also get compressed by a factor of 4.
            # I conclude that the jpeg compression used by Rose was not as
            # aggressive as the one used in gdal_translate, but there is no
            # apparent knob to control that. 
            cmd = "gdal_translate -b 1 -b 2 -b 3 -co compress=jpeg " + tempOrtho + " " + finalOrtho
            print(cmd)
            asp_system_utils.executeCommand(cmd, finalOrtho, suppressOutput, redo)
            
            # Clean up extra files
            for fileName in filesToWipe:
                if os.path.exists(fileName):
                    print("Removing: " + fileName)
                    os.remove(fileName)

        if (not redo) and os.path.exists(finalOrthoPreview):
            print("File exists: " + finalOrthoPreview + ".")
        else:
            cmd = 'gdal_translate -scale -outsize 25% 25% -of jpeg ' + finalOrtho + \
                  ' ' + finalOrthoPreview 
            print(cmd)
            asp_system_utils.executeCommand(cmd, finalOrthoPreview, suppressOutput, redo)
            
    except Exception as e:
        print('Ortho creation failed!\n' + str(e) + ". " + str(traceback.print_exc()))

    os.system("rm -f core.*") # these keep on popping up

    # To ensure we print promptly what we did so far
    sys.stdout.flush()
         
def main(argsIn):

    try:
        # Sample usage:
        # python ~/projects/StereoPipeline/src/asp/IceBridge/gen_ortho.py --site GR   \
        #   --yyyymmdd 20120315 --start-frame 2490 --stop-frame 2491 --bundle-length 2 \
        #   --num-threads 8 --num-processes 3. 
        usage = '''gen_ortho.py <options>'''
                      
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

        # Performance options  
        parser.add_argument('--num-processes', dest='numProcesses', default=1,
                          type=int, help='The number of simultaneous processes to run.')
        parser.add_argument('--num-threads', dest='numThreads', default=8,
                          type=int, help='The number of threads per process.')
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError, msg:
        parser.error(msg)
        
    icebridge_common.switchWorkDir()
    
    if len(options.yyyymmdd) != 8 and len(options.yyyymmdd) != 9:
        # Make an exception for 20100422a
        raise Exception("The --yyyymmdd field must have length 8 or 9.")

    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)
        
    os.system('mkdir -p ' + options.outputFolder)
    logLevel = logging.INFO # Make this an option??
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel,
                                            'icebridge_ortho_log')

    (out, err, status) = asp_system_utils.executeCommand(['uname', '-a'],
                                                         suppressOutput = True)
    logger.info("Running on machine: " + out)
    
    processFolder = os.path.join(options.outputFolder, 'processed')
    
    # Handle subfolder option.  This is useful for comparing results with different parameters!
    if options.processingSubfolder:
        processFolder = os.path.join(processFolder, options.processingSubfolder)
        logger.info('Reading from processing subfolder: ' + options.processingSubfolder)

    jpegFolder = icebridge_common.getJpegFolder(options.outputFolder)
    jpegIndexPath = icebridge_common.csvIndexFile(jpegFolder)
    if not os.path.exists(jpegIndexPath):
        raise Exception("Error: Missing jpeg index file: " + jpegIndexPath + ".")
    (jpegFrameDict, jpegUrlDict) = icebridge_common.readIndexFile(jpegIndexPath,
                                                                  prependFolder = True)
    
    threadText = ''
    if options.numThreads:
        threadText = '--threads ' + str(options.numThreads)
    
    redo = False
    suppressOutput = True
    taskHandles  = []
    if options.numProcesses > 1:    
        pool = multiprocessing.Pool(options.numProcesses)

    # Bound the frames
    sortedFrames = sorted(jpegFrameDict.keys())
    if len(sortedFrames) > 0:
        if options.startFrame < sortedFrames[0]:
            options.startFrame = sortedFrames[0]
        if options.stopFrame > sortedFrames[-1] + 1:
            options.stopFrame = sortedFrames[-1] + 1
    else:
        # No jpeg files, that means nothing to do
        options.startFrame = 0
        options.stopFrame  = 0 

    for frame in range(options.startFrame, options.stopFrame):

        if not frame in jpegFrameDict:
            logger.info("Error: Missing jpeg file for frame: " + str(frame) + ".")
            continue

        # Find the right image
        currImage = jpegFrameDict[frame]

        args = (frame, processFolder, currImage, options.bundleLength, threadText,
                redo, suppressOutput)

        # Run things sequentially if only one process, to make it easy to debug
        if options.numProcesses > 1:
            taskHandles.append(pool.apply_async(runOrtho, args))
        else:
            runOrtho(*args)
        
    if options.numProcesses > 1:
        icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, logger, interactive = False, 
                                                         quitKey='q', sleepTime=20)
        icebridge_common.stopTaskPool(pool)
    
# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


