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

# Top level program that 

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

#asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]




# Constants used in this file
PBS_QUEUE = 'devel' # devel, debug, long
#NODE_TYPE = 'wes' # Westmere = 12 core,  48 GB mem, SBU 1.0, Launch from mfe1 only!
NODE_TYPE = 'san' # Sandy bridge = 16 core,  32 GB mem, SBU 1.82
#NODE_TYPE = 'ivy' # Ivy bridge   = 20 core,  64 GB mem, SBU 2.52
#NODE_TYPE = 'has' # Haswell      = 24 core, 128 GB mem, SBU 3.34
#NODE_TYPE = '???' # Broadwell    = 28 core, 128 GB mem, SBU 4.04
NUM_ORTHO_JOBS      = 1
NUM_ORTHO_PROCESSES = 8
NUM_ORTHO_THREADS   = 2
NUM_BATCH_JOBS      = 1
NUM_BATCH_THREADS   = 8 # MGM is limited to 8 threads
NUM_BATCH_PROCESSES = 3
GROUP_ID = 's1827'
MAX_PROCESS_HOURS = 2#40 # devel limit is 2, long limit is 120, normal is 8

# Wait this long between checking for job completion
SLEEP_TIME = 30

ALL_RUN_LIST       = '/u/smcmich1/icebridge/full_run_list.txt'
SKIP_RUN_LIST      = '/u/smcmich1/icebridge/run_skip_list.txt'
COMPLETED_RUN_LIST = '/u/smcmich1/icebridge/completed_run_list.txt'

LOG_FOLDER              = '/u/smcmich1/icebridge/manager_logs'
UNPACK_FOLDER           = '/u/smcmich1/icebridge/data'
CALIBRATION_FILE_FOLDER = '/u/smcmich1/icebridge/calib_files'
REFERENCE_DEM_FOLDER    = '/u/smcmich1/icebridge/reference_dems'


BUNDLE_LENGTH = 10

def getUser():
    '''Return the current user name.'''
    return getpass.getuser()

def submitJob(jobName, scriptPath, args, logPrefix):
    '''Trivial wrapper to fill in the parts of the job command that never change'''
    return pbs_functions.submitJob(jobName, PBS_QUEUE, MAX_PROCESS_HOURS, GROUP_ID, NODE_TYPE, scriptPath, args, logPrefix)

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

# Do we need this function?
#def getConversionJobName(run, startFrame):
#    '''Get the PBS name to use for a conversion job'''
#    # PBS job names are limited to 15 characters
#    name = str(startFrame) + run[0] + run[1][2:]
#    if len(name) > pbs_functions.MAX_PBS_NAME_LENGTH:
#        raise Exception('Computed name ' +name+' for run '+str(run)+' and frame '+startFrame+' but it is too long')

def conversionIsFinished(run, fullCheck=False):
    '''Return true if this run is present and conversion has finished running on it'''
    outputFolder = getOutputFolder(run)
    if not fullCheck:
        return run.checkFlag('conversion_complete')

    # For a full check, make sure that there is a camera file for
    #  each input image file.    
    cameraFolder = run.getCameraFolder()
    jpegList     = run.getJpegList()
    
    for jpeg in jpegList:
        camFile = os.path.join(cameraFolder,
                               icebridge_common.getCameraFileName(jpeg))
        if not os.path.exists(camFile):
            return False
    
    # Add a lidar file check here if that turns out to be necessary
    return True

def runConversion(run):
    '''Run the conversion tasks for this run on the supercomputer nodes'''
    
    logger = logging.getLogger(__name__)
    
    # Get the frame range for the data.
    (minFrame, maxFrame) = run.getFrameRange()
    
    logger.info('Detected frame range: ' + str((minFrame, maxFrame)))
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data
    numFrames        = maxFrame - minFrame + 1
    numFramesPerNode = numFrames / NUM_ORTHO_JOBS
    numFramesPerNode += 1 # Make sure we don't cut off a few frames at the end
    
    outputFolder = run.getFolder()
    
    # TODO: Is using the default output folder ok here?
    scriptPath = '/u/smcmich1/repo/StereoPipeline/src/asp/IceBridge/full_processing_script.py' # TODO
    args       = ('%s %s --site %s --yyyymmdd %s --skip-fetch --stop-after-convert --num-threads %d --num-processes %d --output-folder %s' 
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
        submitJob(jobName, scriptPath, thisArgs, logPrefix)
        currentFrame += numFramesPerNode

    # Wait for conversions to finish
    waitForRunCompletion(baseName)

    raise Exception('DEBUG')

    # Create a file to mark that conversion is finished for this folder
    run.setFlag('conversion_complete')

    # Pack up camera folder and store it for later
    gotCameras = archive_functions.packAndSendCameraFolder(run)
    
    raise Exception('DEBUG')

def generateBatchList(run):
    '''Generate a list of all the processing batches required for a run'''

    # No actual processing is being done here so it can run on the PFE
    outputFolder = run.getFolder()
    cmd = ('python full_processing_script.py --yyyymmdd %s --site %s --output-folder %s %s %s   --num-processes 1 --num-threads 1  --bundle-length %d  --skip-fetch  --skip-convert' % (run.yyyymmdd, run.site, outputFolder, CALIBRATION_FILE_FOLDER, REFERENCE_DEM_FOLDER, BUNDLE_LENGTH))
    os.system(cmd)
    
    listPath = os.path.join(outputFolder, 'processed/batch_commands_log.txt')
    return listPath
    

    

def submitBatchJobs(run, batchListPath):
    '''Read all the batch jobs required for a run and distribute them across job submissions'''

    if not os.path.exists(batchListPath):
        raise Exception('Failed to generate batch list file: ' + batchListPath)

    # Number of batches = number of lines in the file
    p = subprocess.Popen('wc -l '+batchListPath , stdout=subprocess.PIPE)
    textOutput, err = p.communicate()
    numBatches      = int(textOutput.split()[0])
    
    numBatchesPerNode = numBatches / NUM_BATCH_JOBS
    numBatchesPerNode += 1 # Make sure we don't cut off a few batches at the end

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Call the tool which just executes commands from a file
    scriptPath = 'multi_process_command.py' # TODO

    outputFolder = run.getFolder()
    pbsLogFolder = run.getPbsLogFolder()

    currentBatch = 0
    for i in range(0,NUM_BATCH_JOBS):
        jobName  = str(currentBatch) + baseName

        # Specify the range of lines in the file we want this node to execute
        args = ('%s, %d, %d, %d' % (batchListPath, NUM_BATCH_PROCESSES, currentBatch, currentBatch+numBatchesPerNode))

        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting batch job with args: '+args)
        pbs_functions.submitJob(jobName, scriptPath, args, logPrefix)

    # Waiting on these jobs happens outside this function

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
    
    # TODO: Check more carefully!
    runFolder = run.getFolder()
    
    packedErrorLog = os.path.join(runFolder, 'packedErrors.log')
    with open(packedErrorLog, 'w') as errorLog:
    
        # Look for errors in the log files
        logFileList = [] # TODO, probably in run class
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
                        packedErrorLog.write(line)
                        errorCount += 1
    logger.info('Counted ' + errorCount + ' errors in log files!')
    
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



def main(argsIn):


    try:

        usage = '''usage: plieades_manager.py <options> '''
                      
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
    logger   = icebridge_common.setUpLogger(LOG_FOLDER, logLevel, 'plieades_manager_log')

    # Get the list of runs to process
    logger.info('Reading run lists...')
    allRuns  = readRunList(ALL_RUN_LIST      )
    skipRuns = readRunList(SKIP_RUN_LIST     )
    doneRuns = readRunList(COMPLETED_RUN_LIST)
    
    runList = getRunsToProcess(allRuns, skipRuns, doneRuns)

    runList = [run_helper.RunHelper('GR','20110504', UNPACK_FOLDER)] # DEBUG

   
    # Loop through the incomplete runs
    for run in runList:

        # TODO: Put this in a try/except block so it keeps going

        # Obtain the data for a run
        archive_functions.retrieveRunData(run)
        
        if not run.checkFlag('conversion_complete'):
            # TODO: There needs to be a logging system
            logger.info('Converting data for run ' + str(run))
            runConversion(run)
         
        raise Exception('DEBUG')
         
        # Run command to generate the list of batch jobs for this run
        logger.info('Fetching batch list for run ' + str(run))
        batchListPath = generateBatchList(run)
        
        raise Exception('DEBUG')
        
        # Divide up batches into jobs and submit them to machines.
        logger.info('Submitting jobs for run ' + str(run))
        submitBatchJobs(run, batchListPath)
        
        # TODO: Don't wait for one run to finish before starting the next one!
        
        # Wait for all the jobs to finish
        logger.info('Waiting for job completion of run ' + str(run))
        waitForRunCompletion(run.site) # TODO: Specialize this!
        logger.info('All jobs finished for run '+str(run))
        
        # Log the run as completed
        # - If the run was not processed correctly it will have to be looked at manually
        addToRunList(COMPLETED_RUN_LIST, run)

        # Generate a simple report of the results
        (numOutputs, numProduced, errorCount) = checkResults(run, batchListPath)
        resultText = ('Created %d out of %d output targets with %d errors.' % 
                      (numOutputs, numProduced, errorCount))
        logger.info(resultText)

        # Don't pack or clean up the run if it did not generate all the output files.
        if (numProduced == numOutputs) and (errorCount == 0):
            print 'TODO: Automatically send the completed files to lou and clean up the run!'

            # Pack up all the files to lou, then delete the local copies.            
            #archive_functions.packAndSendCompletedRun(run)
            #cleanupRun(run)
            
            # TODO: Don't wait on the pack/send operation to finish!

        # Notify that the run is done processing
        sendEmail('scott.t.mcmichael@nasa.gov', 'IB run complete: '+str(run), resultText)
        
        raise Exception('DEBUG')
        
    # End loop through runs
    logger.info('==== plieades_manager script has finished! ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



