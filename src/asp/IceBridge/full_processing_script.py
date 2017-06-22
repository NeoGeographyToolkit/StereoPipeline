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
# Sample usage:
# python ~/projects/StereoPipeline/src/asp/IceBridge/full_processing_script.py --yyyymmdd 20111018 --site AN AN_2011_10_18 camera_calib/5D_MII_28mm_DMS\#04_03Oct2011.tsai --num-processes 1 --num-threads 12 --bundle-length 12 --start-frame 350 --stop-frame 400

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing
import icebridge_common
import fetch_icebridge_data
import process_icebridge_run
import extract_icebridge_ATM_points

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../IceBridge')  # for dev ASP
pythonpath  = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, asp_alg_utils, asp_geo_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = basepath    + os.pathsep + os.environ["PATH"]

def fetchAllRunData(yyyymmdd, site, frameStart, frameStop, outputFolder,
                    jpegFolder, orthoFolder, demFolder, lidarFolder):
    '''Download all data needed to process a run'''
    
    print 'Downloading all data for the run!  This could take a while...'
    
    
    baseCommand = (('--yyyymmdd %s --site %s --frame %d --frame-stop %d')
                   % (yyyymmdd, site, frameStart, frameStop))
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

def convertJpegs(jpegFolder, imageFolder):
    '''Convert jpeg images from RGB to single channel'''

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
        frameNumber = icebridge_common.getFrameNumberFromFilename(inputPath)
        outputName = ('DMS_%s_%s_%05d.tif') % (dateStr, timeStr, frameNumber)
        outputPath = os.path.join(imageFolder, outputName)
        
        # Skip existing files
        if os.path.exists(outputPath):
            continue
        
        # Use ImageMagick tool to convert from RGB to grayscale
        cmd = (('convert %s -colorspace Gray %s') % (inputPath, outputPath))
        logger.info(cmd)
        os.system(cmd)
        if not os.path.exists(outputPath):
            raise Exception('Failed to convert jpeg file: ' + jpegFile)

def cameraFromOrthoWrapper(inputPath, orthoPath, inputCamFile, outputCamFile):
    '''Generate a camera model from a single ortho file'''

    logger = logging.getLogger(__name__)

    # Call ortho2pinhole command
    cmd = (('ortho2pinhole %s %s %s %s') % (inputPath, orthoPath, inputCamFile, outputCamFile))
    logger.info(cmd)
    os.system(cmd)
    if not os.path.exists(outputCamFile):
        # This function is getting called from a pool, so just log the failure.
        logger.error('Failed to convert ortho file: ' + orthoFile)
            
    # TODO: Clean up the .gcp file?


def getCameraModelsFromOrtho(imageFolder, orthoFolder, inputCamFile, cameraFolder, numProcesses):
    '''Generate camera models from the ortho files'''
    
    logger = logging.getLogger(__name__)
    logger.info('Generating camera models from ortho images...')

    # TODO: DEM/height options for this tool
    
    imageFiles = os.listdir(imageFolder)
    orthoFiles = os.listdir(orthoFolder)
    
    # Make a dictionary of ortho files by frame
    orthoFrames = {}
    for f in orthoFiles:
        # Skip non-image files
        ext = os.path.splitext(f)[1]
        if ext != '.tif':
            continue
        frame = icebridge_common.getFrameNumberFromFilename(f)
        orthoFrames[frame] = f

    logger.info('Starting ortho processing pool with ' + str(numProcesses) +' processes.')
    pool = multiprocessing.Pool(numProcesses)

    # Loop through all input images
    taskHandles = []
    for imageFile in imageFiles:
        
        # Skip non-image files (including junk from stereo_gui)
        ext = os.path.splitext(imageFile)[1]
        if (ext != '.tif') or ('_sub' in imageFile):
            continue
        
        # Get associated orthofile
        frame     = icebridge_common.getFrameNumberFromFilename(imageFile)       
        orthoFile = orthoFrames[frame]
        
        # Check output file
        inputPath     = os.path.join(imageFolder, imageFile)
        orthoPath     = os.path.join(orthoFolder, orthoFile)
        outputCamFile = os.path.join(cameraFolder, icebridge_common.getCameraFileName(imageFile))
        if os.path.exists(outputCamFile):
            continue
               
        # Add ortho2pinhole command to the task pool
        taskHandles.append(pool.apply_async(cameraFromOrthoWrapper, (inputPath, orthoPath, inputCamFile, outputCamFile)))

    # Wait for all the tasks to complete
    logger.info('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, interactive=False, quitKey='q')


    # All tasks should be finished, clean up the processing pool
    logger.info('Cleaning up the ortho processing pool...')
    icebridge_common.stopTaskPool(pool)
    logger.info('Finished cleaning up the ortho processing pool')
    
    
    
    

def convertLidarDataToCsv(lidarFolder):
    '''Make sure all lidar data is available in a readable text format'''

    logger = logging.getLogger(__name__)
    logger.info('Converting LIDAR files...')
    
    # Loop through all lidar files in the folder
    lidarFiles = os.listdir(lidarFolder)
    for f in lidarFiles:
        extension = os.path.splitext(f)[1]
        
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
        if not os.path.exists(outputPath):
            raise Exception('Failed to parse LIDAR file: ' + fullPath)

def pairLidarFiles(lidarFolder):
    '''For each pair of lidar files generate a double size point cloud.
       We can use these later since they do not have any gaps between adjacent files.'''
    
    logger = logging.getLogger(__name__)
    logger.info('Generating lidar pairs...')
    
    # Create the output folder
    pairFolder = os.path.join(lidarFolder, 'paired')
    os.system('mkdir -p ' + pairFolder)
    
    # All files in the folder
    lidarFiles = os.listdir(lidarFolder)
    
    # Get just the files we converted to csv format
    csvFiles = []    
    for f in lidarFiles:
        extension = os.path.splitext(f)[1]
        if extension == '.csv':
           csvFiles.append(f)
    csvFiles.sort()
    numCsvFiles = len(csvFiles)
    
    # Loop through all pairs of csv files in the folder    
    for i in range(0,numCsvFiles-2):

        thisFile = csvFiles[i  ]
        nextFile = csvFiles[i+1]

        #date1, time1 = icebridge_common.parseTimeStamps(thisFile)
        date2, time2 = icebridge_common.parseTimeStamps(nextFile)
        
        # Record the name with the second file
        # - More useful because the time for the second file represents the middle of the file.
        outputName = 'LIDAR_PAIR_' + date2 +'_'+ time2 + '.csv'

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
            raise Exception('Failed to generate merged LIDAR file: ' + outputPath)
    

def processTheRun(imageFolder, cameraFolder, lidarFolder, orthoFolder, processFolder, isSouth, 
                  bundleLength, startFrame, stopFrame, numProcesses, numThreads):
    '''Do all the run processing'''

    processCommand = (('%s %s %s %s --bundle-length %d --stereo-algorithm 1 --ortho-folder %s --num-processes %d --num-threads %d')
                      % (imageFolder, cameraFolder, lidarFolder, processFolder, bundleLength, orthoFolder, numProcesses, numThreads))
    if isSouth:
        processCommand += ' --south'
    if startFrame:
        processCommand += ' --start-frame ' + str(startFrame)
    if stopFrame:
        processCommand += ' --stop-frame ' + str(stopFrame)

    logger = logging.getLogger(__name__)
    logger.info('Process command: ' + processCommand)
    process_icebridge_run.main(processCommand.split())

def setUpLogger(outputFolder, logLevel):
    '''Set up the root logger so all called files will write to the same output file'''

    # Generate a timestamped log file in the output folder
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    logName   = 'icebridge_processing_log_' + timestamp + '.txt'
    logPath   = os.path.join(outputFolder, logName)
    
    logger = logging.getLogger()    # Call with no argument to configure the root logger.
    logger.setLevel(level=logLevel)
    
    fileHandler = logging.FileHandler(logPath)
    formatter   = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fileHandler.setFormatter(formatter)
    logger.addHandler(fileHandler)
    logger.addHandler(logging.StreamHandler()) # Mirror logging to console

    logger = logging.getLogger(__name__) # We configured root, but continue logging with the normal name.
    return logger

def main(argsIn):

    try:
        usage = '''usage: full_processing_script.py <output_folder> <camera_file>'''
                      
        parser = optparse.OptionParser(usage=usage)

        # TODO: Add options to allow spreading work across supercomputer nodes


        # Run selection
        parser.add_option("--yyyymmdd",  dest="yyyymmdd", default=None,
                          help="Specify the year, month, and day in one YYMMDD string.")
        parser.add_option("--site",  dest="site", default=None,
                          help="Name of the location of the images (AN or GR)")

        # Processing options
        parser.add_option('--bundle-length', dest='bundleLength', default=2,
                          type='int', help='The number of images to bundle adjust and process in a single batch.')
        # TODO: Compute this automatically??
        parser.add_option('--overlap-limit', dest='overlapLimit', default=2,
                          type='int', help='The number of images to treat as overlapping for bundle adjustment.')
        parser.add_option('--start-frame', dest='startFrame', default=None,
                          type='int', help='Frame to start with.  Leave this and stop-frame blank to process all frames.')
        parser.add_option('--stop-frame', dest='stopFrame', default=None,
                          type='int', help='Frame to stop on.')

        parser.add_option("--processing-subfolder",  dest="processingSubfolder", default=None,
                          help="Specify a subfolder name where the processing outputs will go.  Default is no additional folder")

        # Performance options  
        parser.add_option('--num-processes', dest='numProcesses', default=1,
                          type='int', help='The number of simultaneous processes to run.')
        parser.add_option('--num-threads', dest='numThreads', default=1,
                          type='int', help='The number of threads per process.')

        # Action control
        parser.add_option("--fetch-only", action="store_true", dest="fetchOnly", default=False,
                          help="Just fetch the data, don't do any processing.")
        parser.add_option("--skip-fetch", action="store_true", dest="noFetch", default=False,
                          help="Skip data fetching.")
        parser.add_option("--skip-convert", action="store_true", dest="noConvert", default=False,
                          help="Skip data conversion.")
        parser.add_option("--stop-after-fetch", action="store_true", dest="stopAfterFetch", default=False,
                          help="Stop program after data fetching.")
        parser.add_option("--stop-after-convert", action="store_true", dest="stopAfterConvert", default=False,
                          help="Stop program after data conversion.")
                          
        (options, args) = parser.parse_args(argsIn)

        if len(args) != 2:
            print usage
            return -1
        
        outputFolder = os.path.abspath(args[0])
        cameraFile   = os.path.abspath(args[1])

    except optparse.OptionError, msg:
        raise Usage(msg)

    os.system('mkdir -p ' + outputFolder)

    logLevel = logging.INFO # Make this an option??
    logger   = setUpLogger(outputFolder, logLevel)

    # Set up the output folders
    cameraFolder  = os.path.join(outputFolder, 'camera')
    imageFolder   = os.path.join(outputFolder, 'image')
    jpegFolder    = os.path.join(outputFolder, 'jpeg')
    orthoFolder   = os.path.join(outputFolder, 'ortho')
    demFolder     = os.path.join(outputFolder, 'fireball')
    lidarFolder   = os.path.join(outputFolder, 'lidar') # Paired files go in /paired
    processFolder = os.path.join(outputFolder, 'processed')
    
    # Handle subfolder option.  This is useful for comparing results with different parameters!
    if options.processingSubfolder:
        processFolder = os.path.join(processFolder, options.processingSubfolder)
        logger.info('Will write to processing subfolder: ' + options.processingSubfolder)

    if options.noFetch:
        logger.info('Skipping fetch.')
    else:
        # Call data fetch routine and check the result
        fetchResult = fetchAllRunData(options.yyyymmdd, options.site, 
                                      options.startFrame, options.stopFrame, outputFolder,
                                      jpegFolder, orthoFolder, demFolder, lidarFolder)
        if fetchResult == -1:
            return -1
        
    if options.stopAfterFetch:
        logger.info('Fetching complete, finished!')
        return 0

    if options.noConvert:        
        logger.info('Skipping convert')
    else:
        convertJpegs(jpegFolder, imageFolder)
        
        # TODO: Handle case where orthofiles are not present!
        # TODO: What arguments need to be passed in here!
        getCameraModelsFromOrtho(imageFolder, orthoFolder, cameraFile, cameraFolder, options.numProcesses)
       
        convertLidarDataToCsv(lidarFolder)
        
        pairLidarFiles(lidarFolder)

    if options.stopAfterConvert:
        print 'Conversion complete, finished!'
        return 0
    
    isSouth = (options.site == 'AN')

    # Call the processing routine
    processTheRun(imageFolder, cameraFolder, lidarFolder, orthoFolder, processFolder, isSouth,
                  options.bundleLength, options.startFrame, options.stopFrame,
                  options.numProcesses, options.numThreads, )

   

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


