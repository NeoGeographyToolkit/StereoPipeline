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
                    jpegFolder, orthoFolder, fireballFolder, lidarFolder):
    '''Download all data needed to process a run'''
    
    logger = logging.getLogger(__name__)
    logger.info('Downloading data for the run...')

    baseCommand = (('--yyyymmdd %s --site %s --start-frame %d --stop-frame %d --jpeg-folder %s')
                   % (options.yyyymmdd, options.site, startFrame, stopFrame, jpegFolder))

    if options.maxNumLidarToFetch is not None and options.maxNumLidarToFetch >= 0:
        baseCommand += ' --max-num-lidar-to-fetch ' + str(options.maxNumLidarToFetch)

    if options.refetchIndex:
        baseCommand += ' --refetch-index' # this was not right in older fetched runs
        
    if options.stopAfterIndexFetch:
        baseCommand += ' --stop-after-index-fetch' 
    if options.skipValidate:
        baseCommand += ' --skip-validate'
    if options.dryRun:
        baseCommand += ' --dry-run'

    jpegCommand      = baseCommand + ' ' + jpegFolder
    orthoCommand     = baseCommand + ' ' + orthoFolder
    fireballCommand  = baseCommand + ' ' + fireballFolder
    lidarCommand     = baseCommand + ' ' + lidarFolder

    # TODO: Handle runs without DEM or ORTHO data.
    
    # Try to do all the downloads one after another
    # - On a failure the error message should already be printed.
    # - The fetching tool will not redownload existing data.
    if fetch_icebridge_data.main(jpegCommand.split()) < 0:
        return -1
    if fetch_icebridge_data.main(orthoCommand.split()) < 0:
        return -1
    if fetch_icebridge_data.main(fireballCommand.split()) < 0:
        print 'Fireball DEM data is optional, continuing run.'
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
            if jpegFrame not in orthoFrameDict.keys():
                logger.info("Found jpeg frame missing from ortho: " + str(jpegFrame))
                #raise Exception ("Found jpeg frame missing from ortho:" + str(jpegFrame))

        for orthoFrame in orthoFrameDict.keys():
            if orthoFrame not in jpegFrameDict.keys():
                # This can happen, don't die because of it
                logger.info("Found ortho frame missing from jpeg: " + str(orthoFrame))
                #raise Exception ("Found ortho frame missing from jpeg:" + str(orthoFrame))

    # TODO: Wipe any ortho and jpeg images not in the index, or at least warn about it.
    
    return (jpegFolder, orthoFolder, fireballFolder, lidarFolder)

def processTheRun(imageFolder, cameraFolder, lidarFolder, orthoFolder, fireballFolder, processFolder,
                  isSouth, bundleLength, referenceDem, stereoArgs,
                  startFrame, stopFrame, logBatches,
                  numProcesses, numThreads, numProcessesPerBatch):
    '''Do all the run processing'''

    # Some care is taken with the --stereo-arguments argument to make sure it is passed correctly.
    processCommand = (('%s %s %s %s --bundle-length %d --fireball-folder %s ' +
                       '--ortho-folder %s --num-processes %d --num-threads %d ' +
                       '--num-processes-per-batch %d --reference-dem %s')
                      % (imageFolder, cameraFolder, lidarFolder, processFolder,
                         bundleLength, fireballFolder, orthoFolder, numProcesses,
                         numThreads, numProcessesPerBatch, referenceDem))
    if isSouth:
        processCommand += ' --south'
    if startFrame:
        processCommand += ' --start-frame ' + str(startFrame)
    if stopFrame:
        processCommand += ' --stop-frame ' + str(stopFrame)
    if logBatches:
        processCommand += ' --log-batches'
    processCommand += ' --stereo-arguments '

    logger = logging.getLogger(__name__)
    logger.info('Process command: process_icebridge_run ' + processCommand + stereoArgs.strip())
    args = processCommand.split()
    args += (stereoArgs.strip(),) # Make sure this is properly passed
    process_icebridge_run.main(args)

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
        
        parser.add_argument('--stereo-arguments', dest='stereoArgs', default='--stereo-algorithm 2',
                            help='Extra arguments to pass to stereo.')

        parser.add_argument('--start-frame', dest='startFrame', type=int,
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                          default=icebridge_common.getLargestFrame(),
                          help='Frame to stop on.')
        parser.add_argument('--max-num-lidar-to-fetch', dest='maxNumLidarToFetch', default=None,
                          type=int, help="The maximum number of lidar files to fetch. " + \
                          "This is used in debugging.")
        
        parser.add_argument("--camera-calibration-folder",  dest="inputCalFolder", default=None,
                          help="The folder containing camera calibration.")
        parser.add_argument("--reference-dem-folder",  dest="refDemFolder", default=None,
                          help="The folder containing DEMs that created orthoimages.")

        parser.add_argument("--processing-subfolder",  dest="processingSubfolder", default=None,
                          help="Specify a subfolder name where the processing outputs will go. " + \
                          "fault is no additional folder")

        # Performance options  
        parser.add_argument('--num-processes', dest='numProcesses', default=1,
                          type=int, help='The number of simultaneous processes to run.')
        parser.add_argument('--num-ortho-processes', dest='numOrthoProcesses', default=-1,
                          type=int, help='The number of simultaneous ortho processes to run.')
        parser.add_argument('--num-threads', dest='numThreads', default=8,
                          type=int, help='The number of threads per process.')
        parser.add_argument('--num-processes-per-batch', dest='numProcessesPerBatch', default=1,
                          type=int, help='The number of simultaneous processes to run ' + \
                          'for each batch. This better be kept at 1 if running more than one batch.')

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
        parser.add_argument("--log-batches", action="store_true", dest="logBatches", default=False,
                          help="Log the required batch commands without running them.")
        parser.add_argument("--dry-run", action="store_true", dest="dryRun", default=False,
                          help="Set up the input directories but do not fetch/process any imagery.")


        parser.add_argument("--refetch", action="store_true", dest="reFetch", default=False,
                          help="Try fetching again if some files turned out invalid " + \
                          "during conversions.")
        parser.add_argument("--refetch-index", action="store_true", dest="refetchIndex",
                          default=False,
                          help="Force refetch of the index file.")
        parser.add_argument("--stop-after-index-fetch", action="store_true",
                          dest="stopAfterIndexFetch", default=False,
                          help="Stop after fetching the indices.")
                       
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
    logLevel = logging.INFO # Make this an option??
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel,
                                            'icebridge_processing_log')

    (status, out, err) = asp_system_utils.run_return_outputs(['uname', '-a'], verbose=False)
    logger.info("Running on machine: " + out)
    
    # Perform some input checks and initializations
    if not (options.noOrthoConvert and (options.stopAfterConvert or options.stopAfterFetch) ):
        # These are not needed unless cameras are initialized 
        if options.inputCalFolder is None or not os.path.exists(options.inputCalFolder):
            raise Exception("Missing camera calibration folder.")
        if options.refDemFolder is None or not os.path.exists(options.refDemFolder):
            raise Exception("Missing reference DEM folder.")

        refDemName = icebridge_common.getReferenceDemName(options.site)
        refDemPath = os.path.join(options.refDemFolder, refDemName)
        if not os.path.exists(refDemPath):
            raise Exception("Missing reference DEM: " + refDemPath)

    # Set up the output folders
    cameraFolder       = os.path.join(options.outputFolder, 'camera')
    imageFolder        = os.path.join(options.outputFolder, 'image')
    jpegFolder         = os.path.join(options.outputFolder, 'jpeg')
    orthoFolder        = os.path.join(options.outputFolder, 'ortho')
    fireballFolder     = os.path.join(options.outputFolder, 'fireball')
    corrFireballFolder = os.path.join(options.outputFolder, 'corr_fireball')
    lidarFolder        = os.path.join(options.outputFolder, 'lidar') # Paired files go in /paired
    processFolder      = os.path.join(options.outputFolder, 'processed')
    
    # Handle subfolder option.  This is useful for comparing results with different parameters!
    if options.processingSubfolder:
        processFolder = os.path.join(processFolder, options.processingSubfolder)
        logger.info('Will write to processing subfolder: ' + options.processingSubfolder)
       
    if options.noFetch:
        logger.info('Skipping fetch.')
    else:
        # Call data fetch routine and check the result
        fetchResult = fetchAllRunData(options, options.startFrame, options.stopFrame,
                                      jpegFolder, orthoFolder, fireballFolder, lidarFolder)
        if fetchResult < 0:
            logger.error("Fetching failed, quitting the program!") 
            return -1
           
    if options.stopAfterFetch or options.dryRun:
        logger.info('Fetching complete, finished!')
        return 0

    if options.noConvert:        
        logger.info('Skipping convert.')
    else:

        # When files fail in these conversion functions we log the error and keep going

        if not options.skipFastConvert:

            # Run non-ortho conversions without any multiprocessing (they are pretty fast)
            # TODO: May be worth doing the faster functions with multiprocessing in the future

            if not options.noLidarConvert:
                input_conversions.convertLidarDataToCsv(lidarFolder, logger)
                input_conversions.pairLidarFiles(lidarFolder, logger)
         
            input_conversions.correctFireballDems(fireballFolder, corrFireballFolder,
                                                  options.startFrame, options.stopFrame,
                                                  (not isSouth), options.skipValidate,
                                                  logger)

            isGood = input_conversions.convertJpegs(jpegFolder, imageFolder, 
                                                    options.startFrame, options.stopFrame,
                                                    options.skipValidate, logger)
            if not isGood:
                if options.reFetch and (not options.noFetch):
                    # During conversion we may realize some data is bad. 
                    logger.error("Cconversions failed. Trying to re-fetch problematic files.")
                    fetchResult = fetchAllRunData(options, options.startFrame, options.stopFrame,
                                                  jpegFolder, orthoFolder, fireballFolder,
                                                  lidarFolder)
                    if fetchResult < 0:
                        logger.error("Fetching failed, quitting the program!")
                        return -1
                    isGood = input_conversions.convertJpegs(jpegFolder, imageFolder, 
                                                            options.startFrame, options.stopFrame,
                                                            options.skipValidate, logger)
                    if not isGood:
                        logger.error("Jpeg conversions failed, quitting the program!") 
                        return -1
                    
                else:
                    logger.error("Jpeg conversions failed, quitting the program!")
                    return -1

        if not options.noOrthoConvert:
            # Multi-process call to convert ortho images
            input_conversions.getCameraModelsFromOrtho(imageFolder, orthoFolder,
                                                       options.inputCalFolder, 
                                                       options.cameraLookupFile,
                                                       options.yyyymmdd, options.site, 
                                                       refDemPath, cameraFolder, 
                                                       options.startFrame, options.stopFrame,
                                                       options.numOrthoProcesses, options.numThreads,
                                                       logger)
    if options.stopAfterConvert:
        print 'Conversion complete, finished!'
        return 0

    # Call the processing routine
    processTheRun(imageFolder, cameraFolder, lidarFolder, orthoFolder,
                  corrFireballFolder, processFolder,
                  isSouth, options.bundleLength, refDemPath, options.stereoArgs,
                  options.startFrame, options.stopFrame, options.logBatches,
                  options.numProcesses, options.numThreads, options.numProcessesPerBatch)

   

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


