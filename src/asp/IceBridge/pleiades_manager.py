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

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing
import re, shutil, time

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

import icebridge_common, pbs_functions, archive_functions, run_helper
import asp_system_utils, generate_flight_summary

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

#=========================================================================
# Parameters

# Constants used in this file

ORTHO_PBS_QUEUE = 'normal'
BATCH_PBS_QUEUE = 'normal'
BLEND_PBS_QUEUE = 'normal'
MAX_ORTHO_HOURS = 3 # Limit is 8 but these seem to complete fast.
MAX_BATCH_HOURS = 8 # devel limit is 2, long limit is 120, normal is 8
MAX_BLEND_HOURS = 2 # These jobs go pretty fast

GROUP_ID = 's1827'

# Wait this long between checking for job completion
SLEEP_TIME = 60

#=========================================================================

# 'wes' = Westmere = 12 cores/24 processors, 48 GB mem, SBU 1.0, Launch from mfe1 only!
# 'san' = Sandy bridge = 16 cores,  32 GB mem, SBU 1.82
# 'ivy' = Ivy bridge    = 20 cores,  64 GB mem, SBU 2.52
# 'has' = Haswell      = 24 cores, 128 GB mem, SBU 3.34
# 'bro' = Broadwell    = 28 cores, 128 GB mem, SBU 4.04

def getParallelParams(nodeType, task):
    '''Return (numProcesses, numThreads, tasksPerJob) for running a certain task on a certain node type'''

    # Define additional combinations and edit as needed.

    if task == 'ortho':
        if nodeType == 'ivy': return (10, 2, 400)
        if nodeType == 'bro': return (14, 4, 500)
        if nodeType == 'wes': return (10, 4, 400)
    
    if task == 'dem':
        if nodeType == 'ivy': return (4, 8, 80)
        if nodeType == 'bro': return (6, 8, 100)
        if nodeType == 'wes': return (3, 8, 75)
    
    if task == 'blend':
        if nodeType == 'ivy': return (10, 2, 1000)
        if nodeType == 'bro': return (14, 4, 1400) # 200 seems to finish in 10 minutes
        if nodeType == 'wes': return (10, 3, 1000) 
    
    raise Exception('No params defined for node type ' + nodeType + ', task = ' + task)

#=========================================================================

def getEmailAddress(userName):
    '''Return the email address to use for a user'''
    
    if userName == 'smcmich1':
        return 'scott.t.mcmichael@nasa.gov'
    if userName == 'oalexan1':
        return 'oleg.alexandrov@nasa.gov'

def sendEmail(address, subject, body):
    '''Send a simple email from the command line'''
    try:
        os.system('mail -s '+subject+' '+address+' <<< '+body)
    except Exception, e:
        print("Could not send mail.")
        
def partialRun(options):
    '''For a partial run we will not archive or cleanup, as many such
    runs could be running at the same time (which is not recommended,
    as it can cause problems in conversions and ortho2pinhole generation.'''
    return (options.startFrame != icebridge_common.getSmallestFrame()) or \
           (options.stopFrame  != icebridge_common.getLargestFrame())

#---------------------------------------------------------------------

def readRunList(path, options):
    '''Reads a list of runs in this format: GR 20110411.
       Output is a list of (site, yyyymmdd) pairs.'''

    runList = []
    with open(path, 'r') as f:
        for line in f:
            if line[0] == '#': # Skip comments
                continue
            parts = line.split() # Site, yyyymmdd
            # Set up a class object to help manage the run
            runList.append(run_helper.RunHelper(parts[0], parts[1], options.unpackDir))
    return runList

def addToRunList(path, run):
    '''Append a run to a list file'''
    with open(path, 'a') as f:
        f.write(path[0]+' '+path[1]+'\n')
    
def getRunsToProcess(allRuns, skipRuns, doneRuns):
    '''Go through the run lists to get the list of runs that
       should be processed.'''
       
    runList = []
    for run in allRuns:
        if (run in skipRuns) or (run in doneRuns):
            continue
        runList.append(run)
    return runList

#---------------------------------------------------------------------


def runFetch(run, options):
    '''Fetch all the data for a run if it is not already available'''

    logger = logging.getLogger(__name__)
    
    logger.info("Checking all data is present.")

    # Check if already done
    allIsFetched = False
    if options.skipChecks:
        allIsFetched = True
    else:
        try:
            allIsFetched = run.allSourceDataFetched()
        except Exception, e:
            logger.warning('Caught error checking fetch status.\n'
                       + str(e))

    # Fetch the archive from lfe, only in the case the directory is not present
    archive_functions.retrieveRunData(run, options.unpackDir)

    pythonPath = asp_system_utils.which('python')
    
    if not options.noRefetchIndex:
        # Go ahead and refetch the indices since it helps to have these up-to-date.
        logger.info("Refetching the indices.")
        cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --refetch-index --stop-after-index-fetch' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder()))
        logger.info(cmd)
        os.system(cmd)

    if allIsFetched:
        logger.info('Fetch is complete.')
        return
    
    # Fetch whatever is missing directly from NSIDC, and force to have the indices
    # regenerated in this case. Hopefully just a few files are missing.
    logger.info("Fetch from NSIDC.")
    cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --refetch-index --stop-after-fetch --skip-validate' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder()))
    logger.info(cmd)
    os.system(cmd)
    
    # Don't need to check results, they should be cleaned out in conversion call.

    run.setFlag('fetch_complete')
            

def runConversion(run, options):
    '''Run the conversion tasks for this run on the supercomputer nodes.
       This will also run through the fetch step to make sure we have everything we need.'''
    
    logger = logging.getLogger(__name__)

    # Check if already done
    try:
        if run.conversionIsFinished():
            logger.info('Conversion is already complete.')
            return
    except Exception, e:
        logger.warning('Caught error checking conversion status, re-running conversion.\n' + str(e))
        
    logger.info('Converting data for run ' + str(run))
    
    # Get the frame range for the data.
    (minFrame, maxFrame) = run.getFrameRange()
    if minFrame < options.startFrame:
        minFrame = options.startFrame
    if maxFrame > options.stopFrame:
        maxFrame = options.stopFrame
        
    logger.info('Detected frame range: ' + str((minFrame, maxFrame)))

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'ortho')
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data
    numFrames    = maxFrame - minFrame + 1
    numOrthoJobs = numFrames / tasksPerJob
    if numOrthoJobs < 1:
        numOrthoJobs = 1
        
    outputFolder = run.getFolder()
    
    scriptPath = icebridge_common.fullPath('full_processing_script.py')
    args       = (' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --stop-after-convert --num-threads %d --num-processes %d --output-folder %s --skip-validate' 
                  % ( options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, numThreads, numProcesses, outputFolder))
    
    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    os.system('mkdir -p ' + pbsLogFolder)
    
    # Submit all the jobs
    currentFrame = minFrame
    for i in range(0, numOrthoJobs):
        jobName    = ('%06d_%s' % (currentFrame, baseName) )
        startFrame = currentFrame
        stopFrame  = currentFrame+tasksPerJob-1
        if (i == numOrthoJobs - 1):
            stopFrame = maxFrame # Make sure nothing is lost at the end
        thisArgs = (args + ' --start-frame ' + str(startFrame) + ' --stop-frame ' + str(stopFrame) )
        if i != 0: # Only the first job will convert lidar files
            thisArgs += ' --no-lidar-convert'

        logPrefix = os.path.join(pbsLogFolder, 'convert_' + jobName)
        logger.info('Submitting conversion job: ' + scriptPath + ' ' + thisArgs)
        pbs_functions.submitJob(jobName, ORTHO_PBS_QUEUE, MAX_ORTHO_HOURS, GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + " " + thisArgs, logPrefix)
        
        currentFrame += tasksPerJob

    # Wait for conversions to finish
    waitForRunCompletion(baseName)

    # Check the results
    # - If we didn't get everything keep going and process as much as we can.
    success = False
    try:
        success = run.conversionIsFinished(verbose=True)
    except Exception, e:
        logger.warning('Caught error checking conversion status.\n' + str(e))

    if not success:
        #raise Exception('Failed to convert run ' + str(run))
        logger.warning('Could not fully convert run ' + str(run))
        
    run.setFlag('conversion_complete')

    # Pack up camera folder and store it for later. Do this only for full runs
    if not partialRun(options):
        try:
            archive_functions.packAndSendCameraFolder(run)
        except Exception,e:
            print 'Caught exception sending camera folder'
            logger.exception(e)

def generateBatchList(run, options, listPath):
    '''Generate a list of all the processing batches required for a run'''

    logger = logging.getLogger(__name__)
    
    refDemName = icebridge_common.getReferenceDemName(run.site)
    refDemPath = os.path.join(options.refDemFolder, refDemName)

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'dem')

    # No actual processing is being done here so it can run on the PFE
    # - This is very fast so we can re-run it every time. (Maybe not...)
    pythonPath = asp_system_utils.which('python')
    scriptPath = icebridge_common.fullPath('full_processing_script.py')
    cmd       = ('%s %s --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --skip-fetch --skip-convert --num-threads %d --num-processes %d --output-folder %s --bundle-length %d --log-batches --start-frame %d --stop-frame %d' 
                  % (pythonPath, scriptPath, options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, numThreads, numProcesses, run.getFolder(), options.bundleLength, options.startFrame, options.stopFrame))

    # For full runs we must cleanup, as otherwise we'll run out of space
    if not partialRun(options):
        cmd += ' --cleanup'

# TODO: Find out what takes so long here!
# - Also fix the logging!

    logger.info(cmd)
    os.system(cmd)


def getOutputFolderFromBatchCommand(batchCommand):
    '''Extract the output folder from a line in the batch file'''
    parts        = batchCommand.split()
    outputFolder = parts[10] # This needs to be kept up to date with the file format!
    return outputFolder

# TODO: Share code with the other function
def filterBatchJobFile(run, batchListPath):
    '''Make a copy of the batch list file which only contains incomplete batches.'''
    
    logger = logging.getLogger(__name__)
    
    runFolder = run.getFolder()
    
    newBatchPath = batchListPath + '_onlyFailed.txt'

    # Make sure each batch produced the aligned DEM file
    #batchOutputName = 'out-blend-DEM.tif'
    batchOutputName = 'out-align-DEM.tif'
    
    with open(batchListPath, 'r') as fIn, open(newBatchPath, 'w') as fOut:
        for line in fIn:
            outputFolder = getOutputFolderFromBatchCommand(line)
            targetPath   = os.path.join(outputFolder, batchOutputName)
            if not os.path.exists(targetPath):
                fOut.write(line)
            
    return newBatchPath
    
def submitBatchJobs(run, options, batchListPath):
    '''Read all the batch jobs required for a run and distribute them across job submissions.
       Returns the common string in the job names.'''

    logger = logging.getLogger(__name__)

    if not os.path.exists(batchListPath):
        logger.error('Failed to generate batch list file: ' + batchListPath)
        raise Exception('Failed to generate batch list file: ' + batchListPath)

    # Number of batches = number of lines in the file
    p = subprocess.Popen(['wc', '-l', batchListPath], stdout=subprocess.PIPE)
    textOutput, err = p.communicate()
    numBatches      = int(textOutput.split()[0])

    logger.info("Reading batch list: " + batchListPath)
    
    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'dem')
    numBatchJobs = numBatches / tasksPerJob
    if numBatchJobs < 1:
        numBatchJobs = 1

    logger.info( ("Num batches: %d, tasks per job: %d, number of jobs: %d" %
                  (numBatches, tasksPerJob, numBatchJobs) ) )

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Call the tool which just executes commands from a file
    scriptPath = icebridge_common.fullPath('multi_process_command_runner.py')

    outputFolder = run.getFolder()
    pbsLogFolder = run.getPbsLogFolder()

    currentBatch = 0
    for i in range(0, numBatchJobs):
        jobName    = ('%06d_%s' % (currentBatch, baseName) )
        startBatch = currentBatch
        stopBatch  = currentBatch+tasksPerJob
        if (i == numBatchJobs-1):
            stopBatch = numBatches-1 # Make sure nothing is lost at the end

        # Specify the range of lines in the file we want this node to execute
        args = ('%s %d %d %d' % (batchListPath, numProcesses, startBatch, stopBatch))

        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting DEM creation job: ' + scriptPath + ' ' + args)
        pbs_functions.submitJob(jobName, BATCH_PBS_QUEUE, MAX_BATCH_HOURS, GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + ' ' + args, logPrefix)
        
        currentBatch += tasksPerJob

    # Waiting on these jobs happens outside this function
    return baseName

def runBlending(run, options):
    '''Blend together a series of batch DEMs'''
    
    logger = logging.getLogger(__name__)
        
    logger.info('Blending DEMs for run ' + str(run))
    
    # Get the frame range for the data.
    (minFrame, maxFrame) = run.getFrameRange()
    if minFrame < options.startFrame:
        minFrame = options.startFrame
    if maxFrame > options.stopFrame:
        maxFrame = options.stopFrame
    
    logger.info('Detected frame range: ' + str((minFrame, maxFrame)))

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'blend')
    
    # Split the blends across multiple nodes using frame ranges
    numFrames    = maxFrame - minFrame + 1
    numBlendJobs = numFrames / tasksPerJob
    if numBlendJobs < 1:
        numBlendJobs = 1
        
    logger.info( ("Num frames: %d, tasks per job: %d, number of blend jobs: %d" %
                  (numFrames, tasksPerJob, numBlendJobs) ) )
    
    outputFolder = run.getFolder()
    
    scriptPath = icebridge_common.fullPath('blend_dems.py')
    args       = ('--site %s --yyyymmdd %s --num-threads %d --num-processes %d --output-folder %s --bundle-length %d ' 
                  % (run.site, run.yyyymmdd, numThreads, numProcesses, outputFolder, options.bundleLength))
    
    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    
    # Submit all the jobs
    currentFrame = minFrame
    for i in range(0, numBlendJobs):
        jobName    = ('%06d_%s' % (currentFrame, baseName) )
        startFrame = currentFrame
        stopFrame  = currentFrame+tasksPerJob # Last frame passed to the tool is not processed
        if (i == numBlendJobs - 1):
            stopFrame = maxFrame+1 # Make sure nothing is lost at the end
        thisArgs = (args + ' --start-frame ' + str(startFrame) + ' --stop-frame ' + str(stopFrame) )
        logPrefix = os.path.join(pbsLogFolder, 'blend_' + jobName)
        logger.info('Submitting blend job: ' + scriptPath +  ' ' + thisArgs)
        pbs_functions.submitJob(jobName, BLEND_PBS_QUEUE, MAX_BLEND_HOURS, GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + ' ' + thisArgs, logPrefix)
        
        currentFrame += tasksPerJob

    # Wait for conversions to finish
    waitForRunCompletion(baseName)

# TODO: Move this
def waitForRunCompletion(text):
    '''Sleep until all of the submitted jobs containing the provided text have completed'''

    print("Wait for completion with: " + text)
    
    jobsRunning = []
    user = icebridge_common.getUser()
    stillWorking = True
    while stillWorking:
        time.sleep(SLEEP_TIME)
        stillWorking = False

        # Look through the list for jobs with the run's date in the name        
        jobList = pbs_functions.getActiveJobs(user)
        for (job, status) in jobList:
            if text in job: # Matching job found so we keep waiting
                stillWorking = True
                # Print a message if this is the first time we saw the job as running
                if (status == 'R') and (job not in jobsRunning):
                    jobsRunning.append(job)
                    print 'Job launched: ' + job

def checkResults(run, batchListPath):
    '''Return (numOutputs, numProduced, errorCount) to help validate our results'''
    
    logger = logging.getLogger(__name__)
    
    # TODO: Check more carefully!
    runFolder = run.getFolder()
    
    packedErrorLog = os.path.join(runFolder, 'packedErrors.log')
    with open(packedErrorLog, 'w') as errorLog:
    
        # Look for errors in the log files
        # TODO: These log files should be in the batch folders now!
        pbsLogFolder = run.getPbsLogFolder()
        logFileList = os.listdir(pbsLogFolder)
        logFileList = [os.path.join(pbsLogFolder, x) for x in logFileList]
        errorCount = 0
        errorWords = ['but crn has', 'LD_PRELOAD cannot be preloaded'] # TODO: Add more!
        for log in logFileList:
            with open(log, 'r') as f:
                for line in f:
                    hasError = False
                    for word in errorWords:
                        if word in line:
                            hasError = True
                    # Count up the errors and copy them to the packed error log.
                    if hasError:
                        errorLog.write(line)
                        errorCount += 1
    logger.info('Counted ' + str(errorCount) + ' errors in log files!')
    
    # Make sure each batch produced the aligned DEM file
    batchOutputName = 'out-blend-DEM.tif'
    #batchOutputName = 'out-align-DEM.tif'
    
    numOutputs  = 0
    numProduced = 0
    with open(batchListPath, 'r') as f:
        for line in f:
            outputFolder = getOutputFolderFromBatchCommand(line)
            targetPath   = os.path.join(outputFolder, batchOutputName)
            numOutputs += 1
            if os.path.exists(targetPath):
                numProduced += 1
            else:
                logger.info('Did not find: ' + targetPath)
                if not os.path.exists(outputFolder):
                    logger.error('Check output folder position in batch log file!')
            
    return (numOutputs, numProduced, errorCount)

def checkRequiredTools():
    '''Verify that we have all the tools we will be calling during the script.'''

    scripts = ['full_processing_script.py',
               'multi_process_command_runner.py',
               'merge_orbitviz.py',
               'process_icebridge_run.py',
               'process_icebridge_batch.py',
               'lvis2kml.py',
               'blend_dems.py']
    tools  = ['ortho2pinhole',
              'camera_footprint']
    
    for tool in tools:
        asp_system_utils.checkIfToolExists(tool)

    for script in scripts:
        if not os.path.exists(icebridge_common.fullPath(script)):
            raise Exception("Could not find: " + script)
        
def main(argsIn):

    try:
        usage = '''usage: pleiades_manager.py <options> '''
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--base-dir",  dest="baseDir", default=os.getcwd(),
                            help="Where all the inputs and outputs are stored.")

        parser.add_argument("--unpack-dir",  dest="unpackDir", default=None,
                            help="Where to unpack the data.")

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", default=None,
                            help="Specify the year, month, and day in one YYYYMMDD string.")

        parser.add_argument("--site",  dest="site", required=True,
                            help="Name of the location of the images (AN, GR, or AL)")
                            
        parser.add_argument("--node-type",  dest="nodeType", default='ivy',
                            help="Node type to use (wes[mfe], san, ivy, has, bro)")

        # If a partial run is desired, the results will not be archived
        parser.add_argument('--start-frame', dest='startFrame', type=int,
                            default=icebridge_common.getSmallestFrame(),
                            help="Frame to start with.  Leave this and stop-frame blank to " + \
                            "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                            default=icebridge_common.getLargestFrame(),
                            help='Frame to stop on.')
        
        parser.add_argument("--camera-calibration-folder",  dest="inputCalFolder", default=None,
                          help="The folder containing camera calibration.")
        parser.add_argument("--reference-dem-folder",  dest="refDemFolder", default=None,
                          help="The folder containing DEMs that created orthoimages.")

        parser.add_argument('--bundle-length', dest='bundleLength', default=2,
                          type=int, help="The number of images to bundle adjust and process in a single batch.")

        parser.add_argument("--recompute-batch-file", action="store_true", 
                            dest="recomputeBatches", default=False, 
                            help="Recompute an existing batch file.")

        parser.add_argument("--no-refetch-index", action="store_true", 
                            dest="noRefetchIndex", default=False, 
                            help="Normally we want the indices refetched, but for quick tests this is slow.")

        parser.add_argument("--skip-checks", action="store_true", 
                            dest="skipChecks", default=False, 
                            help="Skip checking if files exist. This can be very slow for many files.")

        parser.add_argument("--skip-completed-batches", action="store_true", 
                            dest="failedBatchesOnly", default=False, 
                            help="Don't reprocess completed batches.")

        parser.add_argument("--skip-fetch", action="store_true", dest="skipFetch", default=False, 
                            help="Don't fetch.")
        parser.add_argument("--skip-convert", action="store_true", dest="skipConvert", default=False, 
                            help="Don't convert.")
        parser.add_argument("--skip-process", action="store_true", dest="skipProcess", default=False, 
                            help="Don't process the batches.")
        parser.add_argument("--skip-blend", action="store_true", dest="skipBlend", default=False, 
                            help="Skip blending.")
        parser.add_argument("--skip-report", action="store_true", dest="skipReport", default=False, 
                            help="Skip summary report step.")
                          
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError, msg:
        parser.error(msg)

    #ALL_RUN_LIST       = os.path.join(options.baseDir, 'full_run_list.txt')
    #SKIP_RUN_LIST      = os.path.join(options.baseDir, 'run_skip_list.txt')
    #COMPLETED_RUN_LIST = os.path.join(options.baseDir, 'completed_run_list.txt')

    options.logFolder = os.path.join(options.baseDir, 'manager_logs')
    os.system('mkdir -p ' + options.logFolder)

    if options.unpackDir is None:
        options.unpackDir = os.path.join(options.baseDir, 'data')
        
    os.system('mkdir -p ' + options.unpackDir)

    options.summaryFolder = os.path.join(options.baseDir, 'summaries')
    os.system('mkdir -p ' + options.summaryFolder)
    
    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(options.logFolder, logLevel, 'pleiades_manager_log')

    checkRequiredTools() # Make sure all the needed tools can be found before we start

    emailAddress = getEmailAddress(icebridge_common.getUser())

    # TODO: Uncomment when processing more than one run!
    # Get the list of runs to process
    #logger.info('Reading run lists...')
    #allRuns  = readRunList(ALL_RUN_LIST, options)
    #skipRuns = readRunList(SKIP_RUN_LIST, options)
    #doneRuns = readRunList(COMPLETED_RUN_LIST, options)
    #runList  = getRunsToProcess(allRuns, skipRuns, doneRuns)

    runList = [run_helper.RunHelper(options.site, options.yyyymmdd, options.unpackDir)]

    # TODO: Loop is unused, this tool will be replaced by a different one
    # Loop through the incomplete runs
    for run in runList:

        # TODO: Put this in a try/except block so it keeps going on error

        # TODO: Prefetch the next run while waiting on this run!

        fullBatchListPath = os.path.join(run.getProcessFolder(), 'batch_commands_log.txt')
        batchListPath = fullBatchListPath

        if not options.skipFetch:
            # Obtain the data for a run if it is not already done
            runFetch(run, options)       

        if not options.skipConvert:                   
            # Run conversion and archive results if not already done        
            runConversion(run, options)
        
        if os.path.exists(fullBatchListPath) and not options.recomputeBatches:
            logger.info('Re-using existing batch list file.')
        else:
            # Run command to generate the list of batch jobs for this run
            logger.info('Fetching batch list for run ' + str(run))
            generateBatchList(run, options, fullBatchListPath)
    
        if options.failedBatchesOnly:
            logger.info('Assembling batch file with only failed batches...')
            batchListPath = filterBatchJobFile(run, batchListPath)

        if not options.skipProcess:
        
            # Divide up batches into jobs and submit them to machines.
            logger.info('Submitting jobs for run ' + str(run))
            baseName = submitBatchJobs(run, options, batchListPath)
        
            # Wait for all the jobs to finish
            logger.info('Waiting for job completion of run ' + str(run))
            waitForRunCompletion(baseName)
            logger.info('All jobs finished for run '+str(run))
        
        if not options.skipBlend:
            runBlending(run, options)
        
        # TODO: Uncomment when processing multiple runs.
        ## Log the run as completed
        ## - If the run was not processed correctly it will have to be looked at manually
        #addToRunList(COMPLETED_RUN_LIST, run)

        if not options.skipReport:
            # Generate a simple report of the results
            (numOutputs, numProduced, errorCount) = checkResults(run, fullBatchListPath)
            resultText = ('"Created %d out of %d output targets with %d errors."' % 
                          (numProduced, numOutputs, errorCount))
            logger.info(resultText)

            # Generate a summary folder and send a copy to Lou
            # - Currently the summary folders need to be deleted manually, but they should not
            #   take up much space due to the large amount of compression used.
            summaryFolder = os.path.join(options.summaryFolder, run.name())
            genCmd = ['--yyyymmdd', run.yyyymmdd, '--site', run.site, 
                      '--output-folder', summaryFolder, '--parent-folder', run.parentFolder]
            generate_flight_summary.main(genCmd)
            
            # send data to lunokhod and lfe
            if not partialRun(options):
                archive_functions.packAndSendSummaryFolder(run, summaryFolder)
        else:
            resultText = 'Summary skipped'
            errorCount  = 0 # Flag values to let the next condition pass
            numOutputs  = 1
            numProduced = 1

        # Don't pack or clean up the run if it did not generate all the output files.
        # - TODO: Will never produce 100% of outputs
        if (numProduced == numOutputs) and (errorCount == 0):
            print 'TODO: Automatically send the completed files to lou and clean up the run!'

            # TODO: Uncomment this once our outputs are good.
            # Pack up all the files to lou, then delete the local copies.          
            if not partialRun(options):
                archive_functions.packAndSendCompletedRun(run)
                #cleanupRun(run) # <-- Should this always be manual??
            
            # TODO: Don't wait on the pack/send operation to finish!
            
            sendEmail(emailAddress, '"OIB run passed - '+str(run)+'"', resultText)
        else:
            sendEmail(emailAddress, '"OIB run failed - '+str(run)+'"', resultText)
        
        #raise Exception('DEBUG - END LOOP')
        
    # End loop through runs
    logger.info('==== pleiades_manager script has finished! ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



