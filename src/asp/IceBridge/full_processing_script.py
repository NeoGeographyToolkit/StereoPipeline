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

louUser = 'oalexan1' # all data is stored under this user name

def fetchAllRunData(yyyymmdd, site, dryRun,
                    startFrame, stopFrame, maxNumToFetch,
                    fetchNextDay, skipValidate, outputFolder,
                    jpegFolder, orthoFolder, demFolder, lidarFolder):
    '''Download all data needed to process a run'''
    
    logger = logging.getLogger(__name__)
    logger.info('Downloading all data for the run! This could take a while.')
    
    baseCommand = (('--yyyymmdd %s --site %s --start-frame %d --stop-frame %d')
                   % (yyyymmdd, site, startFrame, stopFrame))

    if maxNumToFetch >= 0:
        baseCommand += ' --max-num-to-fetch ' + str(maxNumToFetch)

    if fetchNextDay:
        baseCommand += ' --fetch-from-next-day-also'
        baseCommand += ' --refetch-index' # this was not right in older fetched runs
        
    if skipValidate:
        baseCommand += ' --skip-validate'

    if dryRun:
        baseCommand += ' --dry-run'

    jpegCommand  = '--type image ' + baseCommand +' '+ jpegFolder
    orthoCommand = '--type ortho ' + baseCommand +' '+ orthoFolder
    demCommand   = '--type dem   ' + baseCommand +' '+ demFolder
    lidarCommand = '--type lidar ' + baseCommand +' '+ lidarFolder

    # TODO: Handle runs without DEM or ORTHO data.
    
    # Try to do all the downloads one after another
    # - On a failure the error message should already be printed.
    # - The fetching tool will not redownload existing data.
    if fetch_icebridge_data.main(jpegCommand.split()) < 0:
        return -1
    if fetch_icebridge_data.main(orthoCommand.split()) < 0:
        return -1
    if fetch_icebridge_data.main(demCommand.split()) < 0:
        print 'DEM data is optional, continuing run.'
    if fetch_icebridge_data.main(lidarCommand.split()) < 0:
        return -1

    return (jpegFolder, orthoFolder, demFolder, lidarFolder)

def processTheRun(imageFolder, cameraFolder, lidarFolder, orthoFolder, processFolder, isSouth, 
                  bundleLength, startFrame, stopFrame, logBatches, numProcesses, numThreads):
    '''Do all the run processing'''

    processCommand = (('%s %s %s %s --bundle-length %d --stereo-algorithm 1 --ortho-folder %s --num-processes %d --num-threads %d')
                      % (imageFolder, cameraFolder, lidarFolder, processFolder, bundleLength, orthoFolder, numProcesses, numThreads))
    if isSouth:
        processCommand += ' --south'
    if startFrame:
        processCommand += ' --start-frame ' + str(startFrame)
    if stopFrame:
        processCommand += ' --stop-frame ' + str(stopFrame)
    if logBatches:
        processCommand += ' --log-batches'

    logger = logging.getLogger(__name__)
    logger.info('Process command: ' + processCommand)
    process_icebridge_run.main(processCommand.split())

def workDirs():
    currDir = os.getcwd()
    m = re.match("^.*?/" + louUser + "/(.*?)$", currDir)
    if not m:
        raise Exception("Could not match %s in %s " % (louUser, currDir))
    pfePath = '/nobackupnfs2/' + louUser + '/' + m.group(1) # path on pfe
    lfePath = '/u/'            + louUser + '/' + m.group(1) # path on lfe
    
    return (pfePath, lfePath)
    
def tarAndWipe(options, logger):

    logger.info("All files were fetched and checks passed. " +
                "Will tar to lou and wipe the dir.")

    # Per https://www.nas.nasa.gov/hecc/support/kb/using-
    # shift-for-local-transfers-and-tar-operations_512.html
    # one can tar and push to lfe at the same time if
    # connecting to lfe first, from where one can see the pfe
    # filesystem. Avoid though using the suggested shift
    # command. It may be faster, but it detaches and is hard
    # to manage. Note: To untar and transfer in one step, one
    # should as well go to lfe first.

    (pfePath, lfePath) = workDirs()
    
    lfeCmd = 'cd ' + pfePath + '; tar cfv ' + lfePath + '/' + \
             options.outputFolder + '.tar ' + options.outputFolder

    cmd = 'ssh ' + louUser + '@lfe "' + lfeCmd + '"'
    logger.info(cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        raise Exception('Failed to tar and copy to lfe.')
    else:
        logger.info('Success tarring and copying to lfe.')

    if options.outputFolder == "" or options.outputFolder[0] == '.':
        raise Exception('Output folder is not as expected. ' +
                        'Not deleting anything just in case.')

    logger.info('Will wipe: ' + options.outputFolder)
    try:
        shutil.rmtree(options.outputFolder)
    except Exception as e:
        # TODO: Can't wipe it as still logging there
        print("Failed to wipe " + options.outputFolder)

    return 0

def startWithLouArchive(options, logger):

    (pfePath, lfePath) = workDirs()

    # See tarAndWipe() for the logic of how one can work with pfe and lfe
    lfeCmd = 'cd ' + lfePath + '; tar xfv ' + lfePath + '/' + options.outputFolder + '.tar' + \
             ' -C ' + pfePath

    cmd = 'ssh ' + louUser + '@lfe "' + lfeCmd + '"'
    logger.info(cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        raise Exception('Failed to untar lfe archive: ' + options.outputFolder + '.tar')
    else:
        logger.info('Success untarring lfe archive.')

    return 0

def fetchNextDay(outputFolder):

    return outputFolder in ['AN_20091025',
                            'AN_20091105',
                            'AN_20091109',
                            'AN_20091112',
                            'AN_20101026',
                            'AN_20101110',
                            'GR_20120317',
                            'AN_20121107',
                            'AN_20131120',
                            'AN_20131127',
                            'GR_20150327',
                            'GR_20150330',
                            'AN_20150911',
                            'AN_20150926',
                            'GR_20160421',
                            'GR_20160715'];
    
def main(argsIn):

    try:
        # Sample usage:
        # python ~/projects/StereoPipeline/src/asp/IceBridge/full_processing_script.py \
        #  --yyyymmdd 20091016 --site AN --num-processes 1 --num-threads 12 --bundle-length 12 \
        #  --start-frame 350 --stop-frame 353 --skip-validate camera_calib ref_dem_folder
        # An output folder will be crated automatically (with a name like
        # AN_20091016), or its name can be specified via the --output-folder
        # option.
        usage = '''usage: full_processing_script.py <options> <camera_calibration_folder>
        <ref_dem_folder>'''
                      
        parser = optparse.OptionParser(usage=usage)

        # TODO: Add options to allow spreading work across supercomputer nodes

        # Run selection
        parser.add_option("--yyyymmdd",  dest="yyyymmdd", default=None,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_option("--site",  dest="site", default=None,
                          help="Name of the location of the images (AN, GR, or AL)")

        parser.add_option("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        parser.add_option("--camera-lookup-file",  dest="cameraLookupFile", default=None,
                          help="The file to use to find which camera was used for which flight. " + \
                          "By default it is in the same directory as this script and named camera_lookup.txt.")
        
        # Processing options
        parser.add_option('--bundle-length', dest='bundleLength', default=2,
                          type='int', help="The number of images to bundle adjust and process in a single batch.")
        # TODO: Compute this automatically??
        parser.add_option('--overlap-limit', dest='overlapLimit', default=2,
                          type='int', help="The number of images to treat as overlapping for bundle adjustment.")

        # Python treats numbers starting with 0 as being in octal rather than decimal.
        # Ridiculous. So read them as strings and convert to int. 
        parser.add_option('--start-frame', dest='startFrame', default=None, type='int',
                          help='Frame to start with.  Leave this and stop-frame blank to process all frames.')
        parser.add_option('--stop-frame', dest='stopFrame', default=None, type='int',
                          help='Frame to stop on.')
        
        parser.add_option("--processing-subfolder",  dest="processingSubfolder", default=None,
                          help="Specify a subfolder name where the processing outputs will go. " + \
                          "fault is no additional folder")

        # Performance options  
        parser.add_option('--num-processes', dest='numProcesses', default=1,
                          type='int', help='The number of simultaneous processes to run.')
        parser.add_option('--num-threads', dest='numThreads', default=1,
                          type='int', help='The number of threads per process.')

        # Action control
        parser.add_option("--skip-fetch", action="store_true", dest="noFetch", default=False,
                          help="Skip data fetching.")
        parser.add_option("--skip-convert", action="store_true", dest="noConvert", default=False,
                          help="Skip data conversion.")
        parser.add_option("--stop-after-fetch", action="store_true", dest="stopAfterFetch",
                          default=False,
                          help="Stop program after data fetching.")
        parser.add_option("--stop-after-convert", action="store_true", dest="stopAfterConvert",
                          default=False,
                          help="Stop program after data conversion.")
        parser.add_option("--skip-validate", action="store_true", dest="skipValidate", default=False,
                          help="Skip input data validation.")
        parser.add_option("--log-batches", action="store_true", dest="logBatches", default=False,
                          help="Log the required batch commands without running them.")
        parser.add_option("--dry-run", action="store_true", dest="dryRun", default=False,
                          help="Set up the input directories but do not fetch/process any imagery.")


        # The following options are intended for use on the supercomputer                          
        parser.add_option("--tar-and-wipe", action="store_true", dest="tarAndWipe", default=False,
                          help="After fetching all data and performing all conversions and " + \
                          "validations, make a tarball on lou and wipe the directory. "      + \
                          "Only valid on Pleiades!")
        parser.add_option("--start-with-lou-archive", action="store_true",
                          dest="startWithLouArchive", default=False,
                          help="Untar an existing archive from lou, then continue.")
                
        parser.add_option('--max-num-to-fetch', dest='maxNumToFetch', default=-1,
                          type='int', help="The maximum number to fetch of each kind of file. " + \
                          "is is used in debugging.")
        parser.add_option("--no-lidar-convert", action="store_true", dest="noLidarConvert",
                          default=False,
                          help="Skip lidar files in the conversion step.")
        parser.add_option("--no-ortho-convert", action="store_true", dest="noOrthoConvert",
                          default=False,
                          help="Skip generating camera models in the conversion step.")
        parser.add_option("--skip-fast-conversions", action="store_true", dest="skipFastConvert",
                          default=False,
                          help="Skips all non-ortho conversions.")
                          
        (options, args) = parser.parse_args(argsIn)

        if len(args) != 2:
            print usage
            return -1
        inputCalFolder = os.path.abspath(args[0])
        refDemFolder   = os.path.abspath(args[1])

    except optparse.OptionError, msg:
        raise Usage(msg)

    possibleSites = ['AN', 'GR', 'AL']
    if options.site not in possibleSites:
        raise Exception("Site must be either AN, GR, or AL.")
    isSouth = (options.site == 'AN')

    if options.cameraLookupFile is None:
        options.cameraLookupFile = P.join(basepath, 'camera_lookup.txt')
    if not os.path.isfile(options.cameraLookupFile):
        raise Exception("Missing camera file: " + options.cameraLookupFile)
        
    if len(options.yyyymmdd) != 8 and len(options.yyyymmdd) != 9:
        # Make an exception for 20100422a
        raise Exception("The --yyyymmdd field must have length 8 or 9.")

    if options.outputFolder is None:
        options.outputFolder = options.site + '_' + options.yyyymmdd

    # Explicitely go from strings to integers, per earlier note.
    if options.startFrame is None:
        options.startFrame = icebridge_common.getSmallestFrame()
    if options.stopFrame is None:
        options.stopFrame = icebridge_common.getLargestFrame()
    
    os.system('mkdir -p ' + options.outputFolder)
    logLevel = logging.INFO # Make this an option??
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel,
                                            'icebridge_processing_log')

    # Perform some input checks
    if not os.path.exists(inputCalFolder):
        raise Exception("Missing camera calibration folder: " + inputCalFolder)
    if not os.path.exists(refDemFolder):
        raise Exception("Missing reference DEM folder: " + refDemFolder)       
    if not options.yyyymmdd:
        raise Exception("The run date must be specified!")
    if not options.site:
        raise Exception("The run site must be specified!")

    os.system('mkdir -p ' + options.outputFolder)
    
    # Get the low resolution reference DEM to use. Bypass this if only fetching is desired.
    if not options.stopAfterFetch and not options.dryRun:
        if options.site == 'AN':
            #refDemPath = 'krigged_dem_nsidc_ndv0_fill.tif' # Higher resolution
            refDemPath = 'ramp200dem_wgs_v2.tif' # Used to produce the orthos
        if options.site == 'GR':
            #refDemPath = 'gimpdem_90m_v1.1.tif' # Higher resolution
            refDemPath = 'NSIDC_Grn1km_wgs84_elev.tif' # Used to produce the orthos
        if options.site == 'AL':
            # Supposedly these were produced with the SRTM map but that map
            #  does not seem to actually include Alaska.  This may mean the NED
            #  map (60 meter) was used but this would require tile handling logic
            #  so for now we will try to use this single 300m DEM.
            refDemPath = 'akdem300m.tif'
        refDemPath = os.path.join(refDemFolder, refDemPath)
        if not os.path.exists(refDemPath):
            raise Exception("Missing reference DEM: " + refDemPath)
                          
    # Set up the output folders
    cameraFolder  = os.path.join(options.outputFolder, 'camera')
    imageFolder   = os.path.join(options.outputFolder, 'image')
    jpegFolder    = os.path.join(options.outputFolder, 'jpeg')
    orthoFolder   = os.path.join(options.outputFolder, 'ortho')
    demFolder     = os.path.join(options.outputFolder, 'fireball')
    corrDemFolder = os.path.join(options.outputFolder, 'corr_fireball')
    lidarFolder   = os.path.join(options.outputFolder, 'lidar') # Paired files go in /paired
    processFolder = os.path.join(options.outputFolder, 'processed')
    
    # Handle subfolder option.  This is useful for comparing results with different parameters!
    if options.processingSubfolder:
        processFolder = os.path.join(processFolder, options.processingSubfolder)
        logger.info('Will write to processing subfolder: ' + options.processingSubfolder)

    # Sanity checks
    if options.tarAndWipe:
        if not options.stopAfterFetch:
            raise Exception('Must stop after fetch to be able to be tar the inputs.')
        if options.noFetch:
            raise Exception('Must fetch to be able to tar the inputs.')

    if options.maxNumToFetch > 0:
        rem = options.maxNumToFetch % 6
        if rem != 0:
            # This is tricky. Do this so that for every ortho file
            # we fetch its .xml file, and for every dem file
            # we fetch its .xml and .tfw file.
            options.maxNumToFetch = options.maxNumToFetch - rem + 6
            logger.info('Increasing maxNumToFetch to ' + str(options.maxNumToFetch))

    if options.startWithLouArchive:
        startWithLouArchive(options, logger)
        
    if options.noFetch:
        logger.info('Skipping fetch.')
    else:
        # Do one more attempt if at the jpeg conversion stage some files were wiped
        # because they were invalid.
        for attempt in range(2):
            # Call data fetch routine and check the result
            fetchResult = fetchAllRunData(options.yyyymmdd, options.site, options.dryRun,
                                          options.startFrame, options.stopFrame, options.maxNumToFetch,
                                          fetchNextDay(options.outputFolder),
                                          options.skipValidate,
                                          options.outputFolder,
                                          jpegFolder, orthoFolder, demFolder, lidarFolder)
            if fetchResult < 0:
                logger.Error("Fetching failed, quitting the program!")
                return -1
           
    if options.stopAfterFetch or options.dryRun:
        logger.info('Fetching complete, finished!')
        return 0

    if options.noConvert:        
        logger.info('Skipping convert')
    else:

        # When files fail in these conversion functions we log the error and keep going

        if not skipFastConvert:
         
            # Run non-ortho conversions without any multiprocessing (they are pretty fast)
            # TODO: May be worth doing the faster functions with multiprocessing in the future
            input_conversions.correctFireballDems(demFolder, corrDemFolder,
                                                  options.startFrame, options.stopFrame, (not isSouth))

            input_conversions.convertJpegs(jpegFolder, imageFolder, options.startFrame, options.stopFrame)

            if not options.noLidarConvert:
                input_conversions.convertLidarDataToCsv(lidarFolder)
                input_conversions.pairLidarFiles(lidarFolder)


        if not options.noOrthoConvert
            # Multi-process call to convert ortho images
            input_conversions.getCameraModelsFromOrtho(imageFolder, orthoFolder, inputCalFolder, 
                                                       options.cameraLookupFile,
                                                       options.yyyymmdd, options.site, 
                                                       refDemPath, cameraFolder, 
                                                       options.startFrame, options.stopFrame,
                                                       options.numProcesses, options.numThreads)


    # This option happens just after conversion to allow it to be paired with the stopAfterConvert option.
    if options.tarAndWipe:
        tarAndWipe(options, logger)


    if options.stopAfterConvert:
        print 'Conversion complete, finished!'
        return 0

    # Call the processing routine
    processTheRun(imageFolder, cameraFolder, lidarFolder, orthoFolder,
                  processFolder, isSouth,
                  options.bundleLength, options.startFrame, options.stopFrame, options.logBatches,
                  options.numProcesses, options.numThreads)

   

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


