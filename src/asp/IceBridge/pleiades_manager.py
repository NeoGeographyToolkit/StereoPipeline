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
import re, shutil, time, getpass

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

STEREO_ALGORITHM = 2 # 1 = SGM, 2 = MGM

ORTHO_PBS_QUEUE = 'normal'
BATCH_PBS_QUEUE = 'normal'
MAX_ORTHO_HOURS = 8
MAX_BATCH_HOURS = 8 # devel limit is 2, long limit is 120, normal is 8

GROUP_ID = 's1827'

# Wait this long between checking for job completion
SLEEP_TIME = 60


#=========================================================================

# 'wes' = Westmere = 12 cores/24 processors, 48 GB mem, SBU 1.0, Launch from mfe1 only!
# 'san' = Sandy bridge = 16 cores,  32 GB mem, SBU 1.82
# 'ivy' = Ivy bridge    = 20 cores,  64 GB mem, SBU 2.52
# 'has' = Haswell      = 24 cores, 128 GB mem, SBU 3.34
# 'bro' = Broadwell    = 28 cores, 128 GB mem, SBU 4.04

def getParallelParams(nodeType, ortho=False):
    '''Return (numProcesses, numThreads, tasksPerJob) for running a certain task on a certain node type'''

    # Define additional combinations and edit as needed.

    if ortho:
        if nodeType == 'ivy': return (10, 2, 400)
        if nodeType == 'bro': return (14, 2, 500)
    
    else: # Batch processing

        if nodeType == 'ivy': return (3, 8, 80)
        if nodeType == 'bro': return (4, 8, 100)
    
    # TODO: Blend step
    
    raise Exception('No params defined for node type ' + nodeType + ', ortho = ' + str(ortho))


#=========================================================================

def getUser():
    '''Return the current user name.'''
    return getpass.getuser()

def getEmailAddress(userName):
    '''Return the email address to use for a user'''
    
    if userName == 'smcmich1':
        return 'scott.t.mcmichael@nasa.gov'
    if userName == 'oalexan1':
        return 'oleg.alexandrov@nasa.gov'

def sendEmail(address, subject, body):
    '''Send a simple email from the command line'''
    os.system('mail -s '+subject+' '+address+' <<< '+body)

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

    # Check if already done
    try:
        if run.allSourceDataFetched():
            logger.info('Fetch is already complete.')
            return
    except Exception, e:
        logger.warning('Caught error checking fetch status, re-running fetch.\n'
                       + str(e))

    # Call the fetch command
    archive_functions.retrieveRunData(run, options.unpackDir)
    
    # Go ahead and refetch the indices since it helps to have these up-to-date.
    cmd = ('full_processing_script.py --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --refetch-index --stop-after-index-fetch' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder()))
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
        logger.warning('Caught error checking conversion status, re-running conversion.\n'
                       + str(e))
        
    logger.info('Converting data for run ' + str(run))
            
    
    # Get the frame range for the data.
    (minFrame, maxFrame) = run.getFrameRange()
    
    logger.info('Detected frame range: ' + str((minFrame, maxFrame)))

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, ortho=True)
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data
    numFrames    = maxFrame - minFrame + 1
    numOrthoJobs = numFrames / tasksPerJob
    
    outputFolder = run.getFolder()
    
    scriptPath = asp_system_utils.which('full_processing_script.py')
    args       = ('--camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --stop-after-convert --num-threads %d --num-processes %d --output-folder %s --skip-validate' 
                  % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, numThreads, numProcesses, outputFolder))
    
    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    os.system('mkdir -p ' + pbsLogFolder)
    
    # Submit all the jobs
    currentFrame = minFrame
    for i in range(0, numOrthoJobs):
        jobName    = str(currentFrame) + baseName
        startFrame = currentFrame
        stopFrame  = currentFrame+tasksPerJob-1
        if (i == numOrthoJobs - 1):
            stopFrame = maxFrame # Make sure nothing is lost at the end
        thisArgs = (args + ' --start-frame ' + str(startFrame) + ' --stop-frame ' + str(stopFrame) )
        if i != 0: # Only the first job will convert lidar files
            thisArgs += ' --no-lidar-convert'
        logPrefix = os.path.join(pbsLogFolder, 'convert_' + jobName)
        logger.info('Submitting conversion job with args: '+thisArgs)
        pbs_functions.submitJob(jobName, ORTHO_PBS_QUEUE, MAX_ORTHO_HOURS, GROUP_ID, options.nodeType, scriptPath, thisArgs, logPrefix)
        
        currentFrame += tasksPerJob

    # Wait for conversions to finish
    waitForRunCompletion(baseName)

    # Check the results
    # - If we didn't get everything keep going and process as much as we can.
    if not run.conversionIsFinished(verbose=True):
        #raise Exception('Failed to convert run ' + str(run))
        logger.warning('Could not fully convert run ' + str(run))
        
    run.setFlag('conversion_complete')

    # Pack up camera folder and store it for later
    gotCameras = archive_functions.packAndSendCameraFolder(run)

def generateBatchList(run, options, listPath):
    '''Generate a list of all the processing batches required for a run'''

    logger = logging.getLogger(__name__)
    
    refDemName = icebridge_common.getReferenceDemName(run.site)
    refDemPath = os.path.join(options.refDemFolder, refDemName)

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, ortho=False)

    # No actual processing is being done here so it can run on the PFE
    # - This is very fast so we can re-run it every time. (Maybe not...)
    scriptPath = asp_system_utils.which('full_processing_script.py')
    cmd       = ('%s --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --skip-fetch --skip-convert --num-threads %d --num-processes %d --output-folder %s --bundle-length %d --log-batches' 
                  % (scriptPath, options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, numThreads, numProcesses, run.getFolder(), options.bundleLength))


# TODO: Find out what takes so long here!
# - Also fix the logging!

    logger.info(cmd)
    os.system(cmd)
    

def submitBatchJobs(run, batchListPath):
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
    
    
    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, ortho=False)   
    numBatchJobs = numBatches / tasksPerJob

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Call the tool which just executes commands from a file
    scriptPath = asp_system_utils.which('multi_process_command_runner.py')

    outputFolder = run.getFolder()
    pbsLogFolder = run.getPbsLogFolder()

    currentBatch = 0
    for i in range(0, numBatchJobs):
        jobName    = str(currentBatch) + baseName
        startBatch = currentBatch
        stopBatch  = currentBatch+numBatchesPerNode
        if (i == numBatchJobs-1):
            stopBatch = numBatches-1 # Make sure nothing is lost at the end

        # Specify the range of lines in the file we want this node to execute
        args = ('%s %d %d %d' % (batchListPath, numProcesses, currentBatch, currentBatch+numBatchesPerNode))

        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting batch job with args: '+args)
        pbs_functions.submitJob(jobName, BATCH_PBS_QUEUE, MAX_BATCH_HOURS, GROUP_ID, options.nodeType, scriptPath, args, logPrefix)
        
        currentBatch += tasksPerJob

    # Waiting on these jobs happens outside this function
    return baseName

# TODO: Move this
def waitForRunCompletion(text):
    '''Sleep until all of the submitted jobs containing the provided text have completed'''

    print 'Waiting for jobs with text: ' + text

    jobsRunning = []
    user = getUser()
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
        errorWords = ['but crn has'] # TODO: Add more!
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
    batchOutputName = 'out-align-DEM.tif'
    
    numOutputs  = 0
    numProduced = 0
    with open(batchListPath, 'r') as f:
        for line in f:
            parts        = line.split()
            outputFolder = parts[9] # This needs to be kept up to date with the file format!
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

    tools = ['full_processing_script.py',
             'multi_process_command_runner.py',
             'merge_orbitviz.py',
             'process_icebridge_run.py',
             'process_icebridge_batch.py',
             'lvis2kml.py',
             'ortho2pinhole',
             'camera_footprint'
            ]

    for tool in tools:
        asp_system_utils.checkIfToolExists(tool)

def main(argsIn):

    try:
        usage = '''usage: pleiades_manager.py <options> '''
        parser = argparse.ArgumentParser(usage=usage)

        ## Run selection

        parser.add_argument("--base-dir",  dest="baseDir", default=os.getcwd(),
                            help="Where all the inputs and outputs are stored.")

        parser.add_argument("--unpack-dir",  dest="unpackDir", default=None,
                            help="Where to unpack the data.")

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", default=None,
                            help="Specify the year, month, and day in one YYYYMMDD string.")

        parser.add_argument("--site",  dest="site", required=True,
                            help="Name of the location of the images (AN, GR, or AL)")
                            
        parser.add_argument("--node-type",  dest="nodeType", default='ivy'
                            help="Node type to use (wes[mfe], san, ivy, has, bro)")
        
        parser.add_argument("--camera-calibration-folder",  dest="inputCalFolder", default=None,
                          help="The folder containing camera calibration.")
        parser.add_argument("--reference-dem-folder",  dest="refDemFolder", default=None,
                          help="The folder containing DEMs that created orthoimages.")

        parser.add_argument('--bundle-length', dest='bundleLength', default=2,
                          type=int, help="The number of images to bundle adjust and process in a single batch.")

        parser.add_argument("--stop-before-process", action="store_true", dest="dontProcess", default=False, 
                            help="Don't process the batches.")
        parser.add_argument("--process-only", action="store_true", dest="processOnly", default=False, 
                            help="Don't fetch or convert.")
                          
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

    emailAddress = getEmailAddress(getUser())

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

        batchListPath = os.path.join(run.getProcessFolder(), 'batch_commands_log.txt')

        if not options.processOnly:
            # Obtain the data for a run if it is not already done
            runFetch(run, options)       
                   
            # Run conversion and archive results if not already done        
            runConversion(run, options)

            # Run command to generate the list of batch jobs for this run
            logger.info('Fetching batch list for run ' + str(run))
            batchListPath = generateBatchList(run, options, batchListPath)
        else:
            logger.info('Skipping fetch and ortho2pinhole')

        if options.dontProcess:
            logger.info('Quitting after pre-processing')
            return
       
        # Divide up batches into jobs and submit them to machines.
        logger.info('Submitting jobs for run ' + str(run))
        baseName = submitBatchJobs(run, options, batchListPath)
        
        
        # Wait for all the jobs to finish
        logger.info('Waiting for job completion of run ' + str(run))
        waitForRunCompletion(baseName)
        logger.info('All jobs finished for run '+str(run))
        
        # TODO: Uncomment when processing multiple runs.
        ## Log the run as completed
        ## - If the run was not processed correctly it will have to be looked at manually
        #addToRunList(COMPLETED_RUN_LIST, run)

        # Generate a simple report of the results
        (numOutputs, numProduced, errorCount) = checkResults(run, batchListPath)
        resultText = ('"Created %d out of %d output targets with %d errors."' % 
                      (numProduced, numOutputs, errorCount))
        logger.info(resultText)

        # Generate a summary folder and send a copy to Lou
        # - Currently the summary folders need to be deleted manually, but they should not
        #   take up much space due to the large amount of compression used.
        summaryFolder = os.path.join(options.summaryFolder, run.name())
        generate_flight_summary.generateFlightSummary(run, summaryFolder)
        archive_functions.packAndSendSummaryFolder(run, summaryFolder) # Sends data to Lunokhod2 location

        # Don't pack or clean up the run if it did not generate all the output files.
        if (numProduced == numOutputs) and (errorCount == 0):
            print 'TODO: Automatically send the completed files to lou and clean up the run!'

            # TODO: Uncomment this once our outputs are good.
            # Pack up all the files to lou, then delete the local copies.          
            #archive_functions.packAndSendCompletedRun(run)
            #cleanupRun(run) # <-- Should this always be manual??
            
            # TODO: Don't wait on the pack/send operation to finish!
            
            sendEmail(emailAddress, '"IB run passed - '+str(run)+'"', resultText)
        else:
            sendEmail(emailAddress, '"IB run failed - '+str(run)+'"', resultText)
        
        raise Exception('DEBUG - END LOOP')
        
    # End loop through runs
    logger.info('==== pleiades_manager script has finished! ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



