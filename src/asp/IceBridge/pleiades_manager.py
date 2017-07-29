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
import asp_system_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]


# Constants used in this file
#NODE_TYPE = 'wes' # Westmere = 12 core,  48 GB mem, SBU 1.0, Launch from mfe1 only!
#NODE_TYPE = 'san' # Sandy bridge = 16 core,  32 GB mem, SBU 1.82
NODE_TYPE = 'ivy' # Ivy bridge   = 20 core,  64 GB mem, SBU 2.52
#NODE_TYPE = 'has' # Haswell      = 24 core, 128 GB mem, SBU 3.34
#NODE_TYPE = '???' # Broadwell    = 28 core, 128 GB mem, SBU 4.04

NUM_ORTHO_JOBS      = 2
NUM_ORTHO_PROCESSES = 10
NUM_ORTHO_THREADS   = 2
ORTHO_PBS_QUEUE     = 'normal'
MAX_ORTHO_HOURS     = 2

NUM_BATCH_JOBS      = 4 # The number of PBS requests we will submit.
NUM_BATCH_THREADS   = 8 # MGM is limited to 8 threads
NUM_BATCH_PROCESSES = 2 # TODO: Get a handle on memory usage!
BATCH_PBS_QUEUE     = 'normal'
MAX_BATCH_HOURS     = 8 # devel limit is 2, long limit is 120, normal is 8

CONTACT_EMAIL = 'scott.t.mcmichael@nasa.gov'

GROUP_ID = 's1827'


STEREO_ALGORITHM = 2 # 1 = SGM, 2 = MGM

# Wait this long between checking for job completion
SLEEP_TIME = 30

ALL_RUN_LIST       = '/nobackup/smcmich1/icebridge/full_run_list.txt'
SKIP_RUN_LIST      = '/nobackup/smcmich1/icebridge/run_skip_list.txt'
COMPLETED_RUN_LIST = '/nobackup/smcmich1/icebridge/completed_run_list.txt'

LOG_FOLDER              = '/nobackup/smcmich1/icebridge/manager_logs'
UNPACK_FOLDER           = '/nobackup/smcmich1/icebridge/data'
CALIBRATION_FILE_FOLDER = '/nobackup/smcmich1/icebridge/calib_files'
REFERENCE_DEM_FOLDER    = '/nobackup/smcmich1/icebridge/reference_dems'
SUMMARY_FOLDER          = '/nobackup/smcmich1/icebridge/summaries'


BUNDLE_LENGTH = 10

def getUser():
    '''Return the current user name.'''
    return getpass.getuser()

def submitJob(jobName, pbsQueue, maxHours, scriptPath, args, logPrefix):
    '''Trivial wrapper to fill in the parts of the job command that never change'''
    return pbs_functions.submitJob(jobName, pbsQueue, maxHours, GROUP_ID, NODE_TYPE, scriptPath, args, logPrefix)

def sendEmail(address, subject, body):
    '''Send a simple email from the command line'''
    os.system('mail -s '+subject+' '+address+' <<< '+body)

#---------------------------------------------------------------------

def readRunList(path):
    '''Reads a list of runs in this format: GR 20110411.
       Output is a list of (site, yyyymmdd) pairs.'''

    runList = []
    with open(path, 'r') as f:
        for line in f:
            if line[0] == '#': # Skip comments
                continue
            parts = line.split() # Site, yyyymmdd
            # Set up a class object to help manage the run
            runList.append(run_helper.RunHelper(parts[0], parts[1], UNPACK_FOLDER))
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


def runFetch(run):
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
    archive_functions.retrieveRunData(run, UNPACK_FOLDER)
    
    # Go ahead and refetch the indices since it helps to have these up-to-date.
    cmd = ('full_processing_script.py %s %s --site %s --yyyymmdd %s --output-folder %s --refetch-index --stop-after-index-fetch' % (CALIBRATION_FILE_FOLDER, REFERENCE_DEM_FOLDER, run.site, run.yyyymmdd, run.getFolder()))
    logger.info(cmd)
    os.system(cmd)
    
    # Don't need to check results, they should be cleaned out in conversion call.

    run.setFlag('fetch_complete')
            

def runConversion(run):
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
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data
    numFrames        = maxFrame - minFrame + 1
    numFramesPerNode = numFrames / NUM_ORTHO_JOBS
    numFramesPerNode += 1 # Make sure we don't cut off a few frames at the end
    
    outputFolder = run.getFolder()
    
    scriptPath = asp_system_utils.which('full_processing_script.py')
    args       = ('%s %s --site %s --yyyymmdd %s --stop-after-convert --num-threads %d --num-processes %d --output-folder %s --skip-validate' 
                  % (CALIBRATION_FILE_FOLDER, REFERENCE_DEM_FOLDER, run.site, run.yyyymmdd, NUM_ORTHO_THREADS, NUM_ORTHO_PROCESSES, outputFolder))
    
    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    os.system('mkdir -p ' + pbsLogFolder)
    
    currentFrame = minFrame
    for i in range(0,NUM_ORTHO_JOBS):
        jobName  = str(currentFrame) + baseName
        thisArgs = (args + ' --start-frame ' + str(currentFrame)
                         + ' --stop-frame '  + str(currentFrame+numFramesPerNode-1) )
        if i != 0: # Only the first job will convert lidar files
            thisArgs += ' --no-lidar-convert'
        logPrefix = os.path.join(pbsLogFolder, 'convert_' + jobName)
        logger.info('Submitting conversion job with args: '+thisArgs)
        submitJob(jobName, ORTHO_PBS_QUEUE, MAX_ORTHO_HOURS, scriptPath, thisArgs, logPrefix)
        currentFrame += numFramesPerNode

    # Wait for conversions to finish
    waitForRunCompletion(baseName)

    # Check the results
    if not run.conversionIsFinished(verbose=True):
        raise Exception('Failed to convert run ' + str(run))
        
    run.setFlag('conversion_complete')

    # Pack up camera folder and store it for later
    gotCameras = archive_functions.packAndSendCameraFolder(run)



def generateBatchList(run):
    '''Generate a list of all the processing batches required for a run'''

    logger = logging.getLogger(__name__)
    
    # No actual processing is being done here so it can run on the PFE
    # - This is very fast so we can re-run it every time.
    cmd = ('process_icebridge_run.py %s %s %s %s --ortho-folder %s --num-threads %d  --bundle-length %d --stereo-algorithm %d  --stop-frame 99999999  --log-batches' % (run.getImageFolder(), run.getCameraFolder(), run.getLidarFolder(), run.getProcessFolder(), run.getOrthoFolder(), NUM_BATCH_THREADS, BUNDLE_LENGTH, STEREO_ALGORITHM))
    logger.info(cmd)
    os.system(cmd)
    
    listPath = os.path.join(run.getProcessFolder(), 'batch_commands_log.txt')
    return listPath
    

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
    
    numBatchesPerNode = numBatches / NUM_BATCH_JOBS
    numBatchesPerNode += 1 # Make sure we don't cut off a few batches at the end

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Call the tool which just executes commands from a file
    scriptPath = asp_system_utils.which('multi_process_command_runner.py')

    outputFolder = run.getFolder()
    pbsLogFolder = run.getPbsLogFolder()

    currentBatch = 0
    for i in range(0,NUM_BATCH_JOBS):
        jobName  = str(currentBatch) + baseName

        # Specify the range of lines in the file we want this node to execute
        args = ('%s %d %d %d' % (batchListPath, NUM_BATCH_PROCESSES, currentBatch, currentBatch+numBatchesPerNode))

        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting batch job with args: '+args)
        submitJob(jobName, BATCH_PBS_QUEUE, MAX_BATCH_HOURS, scriptPath, args, logPrefix)
        
        currentBatch += numBatchesPerNode

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
        pbsLogFolder = run.getPbsLogFolder()
        logFileList = os.listdir(pbsLogFolder)
        logFileList = [os.path.join(pbsLogFolder, x) for x in logFileList]
        errorCount = 0
        errorWords = ['error', 'Error'] # TODO: Fix this!
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
            outputFolder = parts[4] # This needs to be kept up to date with the file format!
            targetPath = os.path.join(outputFolder, batchOutputName)
            numOutputs += 1
            if os.path.exists(targetPath):
                numProduced += 1
            
    return (numOutputs, numProduced, errorCount)

def generateSummaryFolder(run, outputFolder):
    '''Generate a folder containing handy debugging files including output thumbnails'''
    
    # Copy logs to the output folder
    os.system('mkdir -p ' + outputFolder)
    runFolder = run.getFolder()
    packedErrorLog = os.path.join(runFolder, 'packedErrors.log')
    shutil.copy(packedErrorLog, outputFolder)
    shutil.copy(packedErrorLog, outputFolder)
    
    # Make thumbnail versions of all the output DEM files
    demList = run.getOutputDemList()
    for (dem, frames) in demList:
        if not os.path.exists(dem):
            continue
        thumbName = ('dem_%d_%d_browse.tif' % (frames[0], frames[1]))
        thumbPath = os.path.join(outputFolder, thumbName)
        cmd = 'gdal_translate '+dem+' '+thumbPath+' -of GTiff -outsize 10% 10% -b 1 -co "COMPRESS=JPEG"'
        os.system(cmd)
        

def main(argsIn):


    try:

        usage = '''usage: pleiades_manager.py <options> '''
                      
        parser = optparse.OptionParser(usage=usage)

        ## Run selection
        #parser.add_option("--yyyymmdd",  dest="yyyymmdd", default=None,
        #                  help="Specify the year, month, and day in one YYYYMMDD string.")

        # TODO: What format for defining the run?
        parser.add_option("--manual-run",  dest="manualRun", default=None,
                          help="Manually specify a single run to process.") 
                          
        (options, args) = parser.parse_args(argsIn)

        #if len(args) != 2:
        #    print usage
        #    return -1


    except optparse.OptionError, msg:
        raise Usage(msg)

    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(LOG_FOLDER, logLevel, 'pleiades_manager_log')

    # Get the list of runs to process
    logger.info('Reading run lists...')
    allRuns  = readRunList(ALL_RUN_LIST      )
    skipRuns = readRunList(SKIP_RUN_LIST     )
    doneRuns = readRunList(COMPLETED_RUN_LIST)
    
    runList = getRunsToProcess(allRuns, skipRuns, doneRuns)

    runList = [run_helper.RunHelper('AN','20101104', UNPACK_FOLDER)] # DEBUG

   
    # Loop through the incomplete runs
    for run in runList:

        # TODO: Put this in a try/except block so it keeps going on error

        # TODO: Prefetch the next run while waiting on this run!

        # Obtain the data for a run if it is not already done
        runFetch(run)       
               
        # Run conversion and archive results if not already done        
        runConversion(run)

        # Run command to generate the list of batch jobs for this run
        logger.info('Fetching batch list for run ' + str(run))
        batchListPath = generateBatchList(run)
        
        # Divide up batches into jobs and submit them to machines.
        logger.info('Submitting jobs for run ' + str(run))
        baseName = submitBatchJobs(run, batchListPath)
        
        
        # Wait for all the jobs to finish
        logger.info('Waiting for job completion of run ' + str(run))
        waitForRunCompletion(baseName)
        logger.info('All jobs finished for run '+str(run))
        
        # Log the run as completed
        # - If the run was not processed correctly it will have to be looked at manually
        addToRunList(COMPLETED_RUN_LIST, run)

        # Generate a simple report of the results
        (numOutputs, numProduced, errorCount) = checkResults(run, batchListPath)
        resultText = ('"Created %d out of %d output targets with %d errors."' % 
                      (numProduced, numOutputs, errorCount))
        logger.info(resultText)

        # Generate a summary folder and send a copy to Lou
        # - Currently the summary folders need to be deleted manually, but they should not
        #   take up much space due to the large amount of compression used.
        summaryFolder = os.path.join(SUMMARY_FOLDER, run.name())
        generateSummaryFolder(run, summaryFolder)
        archive_functions.packAndSendSummaryFolder(run, summaryFolder)
        # TODO: Automatically scp these to lunokhod?

        # Don't pack or clean up the run if it did not generate all the output files.
        if (numProduced == numOutputs) and (errorCount == 0):
            print 'TODO: Automatically send the completed files to lou and clean up the run!'

            # Pack up all the files to lou, then delete the local copies.            
            #archive_functions.packAndSendCompletedRun(run)
            #cleanupRun(run) # <-- Should this always be manual??
            
            # TODO: Don't wait on the pack/send operation to finish!
            
            sendEmail(CONTACT_EMAIL, '"IB run passed - '+str(run)+'"', resultText)
        else:
            sendEmail(CONTACT_EMAIL, '"IB run failed - '+str(run)+'"', resultText)
        
        raise Exception('DEBUG')
        
    # End loop through runs
    logger.info('==== pleiades_manager script has finished! ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



