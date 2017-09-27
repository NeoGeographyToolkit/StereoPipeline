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

CAMGEN_PBS_QUEUE   = 'normal'
BATCH_PBS_QUEUE    = 'normal'
BLEND_PBS_QUEUE    = 'normal'
ORTHOGEN_PBS_QUEUE = 'normal'
MAX_CAMGEN_HOURS   = 3 # Limit is 8 but these seem to complete fast.
MAX_BATCH_HOURS    = 8 # devel limit is 2, long limit is 120, normal is 8
MAX_BLEND_HOURS    = 2 # These jobs go pretty fast
MAX_ORTHOGEN_HOURS = 2 # These jobs go pretty fast

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

    if task == 'camgen':
        if nodeType == 'ivy': return (10, 2, 400)
        if nodeType == 'bro': return (14, 4, 500)
        if nodeType == 'wes': return (10, 4, 400)
        if nodeType == 'san': return (10, 4, 400)
    
    if task == 'dem':
        if nodeType == 'ivy': return (4, 8, 80)
        if nodeType == 'bro': return (6, 8, 100)
        if nodeType == 'wes': return (3, 8, 75)
        if nodeType == 'san': return (3, 8, 75)
    
    if task == 'blend':
        if nodeType == 'ivy': return (10, 2, 1000)
        if nodeType == 'bro': return (14, 4, 1400) # 200 seems to finish in 10 minutes
        if nodeType == 'wes': return (10, 3, 1000) 
        if nodeType == 'san': return (10, 3, 1000) 
    
    if task == 'orthogen':
        # TODO: Need to think more here
        if nodeType == 'ivy': return (10, 2, 400)
        if nodeType == 'bro': return (14, 4, 500)
        if nodeType == 'wes': return (10, 4, 400)
        if nodeType == 'san': return (10, 4, 400)

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
    # Remove any quotes, as that confuses the command line.
    subject = subject.replace("\"", "")
    body    = body.replace("\"", "")
    try:
        cmd = 'mail -s "' + subject + '" ' + address + ' <<< "' + body + '"'
        print(cmd)
        os.system(cmd)
    except Exception, e:
        print("Could not send mail.")
        
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
            # Note that the check below is incomplete, it does not check for Fireball
            allIsFetched = run.allSourceDataFetched()
        except Exception, e:
            allIsFetched = False
            logger.warning('Caught error checking fetch status.\n'
                       + str(e))

    # Fetch the archive from lfe, only in the case the directory is not present
    archive_functions.retrieveRunData(run, options.unpackDir)

    pythonPath = asp_system_utils.which('python')
    
    # Fetch whatever is missing directly from NSIDC, and force to have the indices
    # regenerated in this case. Hopefully just a few files are missing.
    # - This is likely to have to fetch the large nav data file(s)
    if not options.noRefetch:
        logger.info("Fetch from NSIDC.")
        cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s  --stop-after-fetch --start-frame %d --stop-frame %d' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder(), options.startFrame, options.stopFrame))
        
        if options.noNavFetch:
            cmd += ' --no-nav'
            
        if options.skipValidate:
            cmd += ' --skip-validate'

        if not options.noRefetchIndex:
            cmd += ' --refetch-index'
            
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
        if run.conversionIsFinished(options.startFrame, options.stopFrame):
            logger.info('Conversion is already complete.')
            return
    except Exception, e:
        logger.warning('Caught error checking conversion status, re-running conversion.\n' + str(e))
        
    logger.info('Converting data for run ' + str(run))

    # Run just the nav data --> estimated camera conversion on the PFE machine.
    # - This is single threaded and may take a while but is primarily IO driven.
    # - We can't start the slower ortho2pinhole processes until this is finished.
    # - If all of the camera files already exist this call will finish up very quickly.
    if not options.noNavFetch:
        logger.info("Generating estimated camera files from the navigation files.")
        pythonPath = asp_system_utils.which('python')
        cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --skip-fetch --stop-after-convert --no-lidar-convert --no-ortho-convert --skip-fast-conversions  --start-frame %d --stop-frame %d' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder(), options.startFrame, options.stopFrame))
    
        logger.info(cmd)
        os.system(cmd)    
        logger.info("Finished generating estimated camera files from nav.")
    
    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'camgen')
    
    if options.simpleCameras:
        numThreads   = 1 # No need for multiple threads for the simple code
        numProcesses = 6 # Keep low to avoid admin wrath
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data
    numFrames    = options.stopFrame - options.startFrame + 1
    numCamgenJobs = numFrames / tasksPerJob
    if numCamgenJobs < 1:
        numCamgenJobs = 1
        
    outputFolder = run.getFolder()
    
    scriptPath = icebridge_common.fullPath('full_processing_script.py')
    args       = (' --skip-fetch --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --stop-after-convert --num-threads %d --num-processes %d --output-folder %s' 
                  % ( options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, numThreads, numProcesses, outputFolder))
    if options.noNavFetch:
        args += ' --no-nav'
    
    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    os.system('mkdir -p ' + pbsLogFolder)
    
    if options.simpleCameras: # Use the fast ortho2pinhole call locally.
    
        logger.info('Running simple ortho2pinhole step locally.')
        args += ' --simple-cameras --start-frame ' + str(minFrame) + ' --stop-frame ' + str(maxFrame)
        fullCmd = scriptPath +' '+ args
        logger.info(fullCmd)
        os.system(fullCmd)
    
    else: # Use the longer running ortho2pinhole spread across the processing nodes.
                
        baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.
        
        # Submit all the jobs
        currentFrame = options.startFrame
        jobList = []
        for i in range(0, numCamgenJobs):
            jobName    = ('%06d_%s' % (currentFrame, baseName) )
            startFrame = currentFrame
            stopFrame  = currentFrame+tasksPerJob-1
            if (i == numCamgenJobs - 1):
                stopFrame = options.stopFrame # Make sure nothing is lost at the end
            thisArgs = (args + ' --start-frame ' + str(startFrame) + \
                        ' --stop-frame ' + str(stopFrame) )
            if i != 0: # Only the first job will convert lidar files
                thisArgs += ' --no-lidar-convert'

            logPrefix = os.path.join(pbsLogFolder, 'convert_' + jobName)
            logger.info('Submitting camera generation job: ' + scriptPath + ' ' + thisArgs)
            pbs_functions.submitJob(jobName, CAMGEN_PBS_QUEUE, MAX_CAMGEN_HOURS,
                                    options.minutesInDevelQueue,
                                    GROUP_ID,
                                    options.nodeType, '/usr/bin/python2.7',
                                    scriptPath + " " + thisArgs, logPrefix)
            jobList.append(jobName)
            currentFrame += tasksPerJob

        # Wait for conversions to finish
        waitForRunCompletion(baseName, jobList)

    # Check the results
    # - If we didn't get everything keep going and process as much as we can.
    success = False
    try:
        success = run.conversionIsFinished(options.startFrame, options.stopFrame, verbose = True)
    except Exception, e:
        logger.warning('Caught error checking conversion status.\n' + str(e))

    if not success:
        #raise Exception('Failed to convert run ' + str(run))
        logger.warning('Could not fully convert run ' + str(run))
        
    run.setFlag('conversion_complete')


def generateBatchList(run, options, listPath):
    '''Generate a list of all the processing batches required for a run'''

    logger = logging.getLogger(__name__)
    logger.info("Generate batch list: " + listPath)
    
    refDemName = icebridge_common.getReferenceDemName(run.site)
    refDemPath = os.path.join(options.refDemFolder, refDemName)

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'dem')

    # No actual processing is being done here so it can run on the PFE
    # - No start/stop frames here, always generate the full list so it is stable.
    pythonPath = asp_system_utils.which('python')
    scriptPath = icebridge_common.fullPath('full_processing_script.py')
    cmd       = ('%s %s --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --skip-fetch --skip-convert --num-threads %d --num-processes %d --output-folder %s --bundle-length %d --log-batches ' 
                  % (pythonPath, scriptPath, options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, numThreads, numProcesses, run.getFolder(), options.bundleLength))

    # For full runs we must cleanup, as otherwise we'll run out of space
    if not options.skipCleanup:
        cmd += ' --cleanup'

    if options.noNavFetch:
        cmd += ' --no-nav'

# TODO: Find out what takes so long here!
# - Also fix the logging!

    logger.info(cmd)
    os.system(cmd)


def getOutputFolderFromBatchCommand(batchCommand):
    '''Extract the output folder from a line in the batch file'''
    # TODO: Integrate with the more robust logic in icebridge_common.getFrameRangeFromBatchFolder()
    # which can parse a batch command.
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
    
    fIn = open(batchListPath, 'r')
    fOut = open(newBatchPath, 'w')
    for line in fIn:
        outputFolder = getOutputFolderFromBatchCommand(line)
        targetPath   = os.path.join(outputFolder, batchOutputName)
        if not os.path.exists(targetPath):
            fOut.write(line)
    fIn.close()
    fOut.close()

    return newBatchPath
    
def submitBatchJobs(run, options, batchListPath):
    '''Read all the batch jobs required for a run and distribute them across job submissions.
       Returns the common string in the job names.'''

    logger = logging.getLogger(__name__)

    if not os.path.exists(batchListPath):
        logger.error('Failed to generate batch list file: ' + batchListPath)
        raise Exception('Failed to generate batch list file: ' + batchListPath)

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, 'dem')

    # Read the batch list
    logger.info("Reading batch list: " + batchListPath)
    batchLines = []
    with open(batchListPath, 'r') as f:
        text = f.read()
        batchLines = text.split('\n')

    # Find all lines in the batch list in range
    framesInRange = []
    for line in batchLines:
        if line == "":
            continue
        (begFrame, endFrame) = icebridge_common.getFrameRangeFromBatchFolder(line)
        if begFrame >= options.startFrame and begFrame < options.stopFrame:
            framesInRange.append(begFrame)

    frameGroups  = icebridge_common.partitionArray(framesInRange, tasksPerJob)
    numBatches   = len(framesInRange)
    numBatchJobs = len(frameGroups)
    
    logger.info( ("Num batches: %d, tasks per job: %d, number of jobs: %d" %
                  (numBatches, tasksPerJob, numBatchJobs) ) )

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Call the tool which just executes commands from a file
    scriptPath = icebridge_common.fullPath('multi_process_command_runner.py')

    outputFolder = run.getFolder()
    pbsLogFolder = run.getPbsLogFolder()

    jobList = []
    for group in frameGroups:
        if len(group) == 0:
            continue
        firstFrame = group[0]
        lastFrame  = group[-1] + 1 # go one beyond
        jobName    = ('%06d_%s' % (firstFrame, baseName) )

        # Specify the range of lines in the file we want this node to execute
        args = ('--command-file-path %s --start-frame %d --stop-frame %d --num-processes %d' % \
                (batchListPath, firstFrame, lastFrame, numProcesses))

        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting DEM creation job: ' + scriptPath + ' ' + args)

        pbs_functions.submitJob(jobName, BATCH_PBS_QUEUE, MAX_BATCH_HOURS,
                                options.minutesInDevelQueue,
                                GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + ' ' + args, logPrefix)
        jobList.append(jobName)

    # Waiting on these jobs happens outside this function
    return (baseName, jobList)

def runJobs(run, mode, options):
    '''Run a blend or an ortho gen job.'''
    
    # TODO: Also merge with the logic for creating a camera file.
    
    logger = logging.getLogger(__name__)
        
    logger.info('Running task ' + mode + ' for run ' + str(run))
    
    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob) = getParallelParams(options.nodeType, mode)
    
    # Split the jobs across multiple nodes using frame ranges
    numFrames    = options.stopFrame - options.startFrame + 1
    numJobs = numFrames / tasksPerJob
    if numJobs < 1:
        numJobs = 1
        
    logger.info( ("Num frames: %d, tasks per job: %d, number of %s jobs: %d" %
                  (numFrames, tasksPerJob, mode, numJobs) ) )
    
    outputFolder = run.getFolder()

    scriptPath = ""
    extraArgs = ""
    if mode == 'orthogen':
        scriptPath = icebridge_common.fullPath('gen_ortho.py')
        queueName  = ORTHOGEN_PBS_QUEUE
        maxHours   = MAX_ORTHOGEN_HOURS
    elif mode == 'blend':
        queueName  = BLEND_PBS_QUEUE
        maxHours   = MAX_BLEND_HOURS
        scriptPath = icebridge_common.fullPath('blend_dems.py')
        extraArgs  = '  --blend-to-fireball-footprint'
    else:
        raise Exception("Unknown mode: " + mode)
    args = (('--site %s --yyyymmdd %s --num-threads %d --num-processes %d ' + \
            '--output-folder %s --bundle-length %d %s ') 
            % (run.site, run.yyyymmdd, numThreads, numProcesses, outputFolder,
               options.bundleLength, extraArgs))
    
    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    
    # Submit all the jobs
    jobList = []
    currentFrame = options.startFrame
    for i in range(0, numJobs):
        jobName    = ('%06d_%s' % (currentFrame, baseName) )
        startFrame = currentFrame
        stopFrame  = currentFrame+tasksPerJob # Last frame passed to the tool is not processed
        if (i == numJobs - 1):
            stopFrame = options.stopFrame # Make sure nothing is lost at the end
        thisArgs = (args + ' --start-frame ' + str(startFrame) + \
                    ' --stop-frame ' + str(stopFrame) )
        logPrefix = os.path.join(pbsLogFolder, mode + '_' + jobName)
        logger.info('Submitting job: ' + scriptPath +  ' ' + thisArgs)
        pbs_functions.submitJob(jobName, queueName, maxHours,
                                options.minutesInDevelQueue,
                                GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + ' ' + thisArgs, logPrefix)
        jobList.append(jobName)
        currentFrame += tasksPerJob

    # Wait for conversions to finish
    waitForRunCompletion(baseName, jobList)

# TODO: Move this
def waitForRunCompletion(jobPrefix, jobList):
    '''Sleep until all of the submitted jobs containing the provided job prefix have completed'''

    print("Wait for completion with: " + jobPrefix)
    
    jobsRunning = []
    user = icebridge_common.getUser()
    stillWorking = True
    while stillWorking:

        time.sleep(SLEEP_TIME)
        stillWorking = False

        # Look through the list for jobs with the run's date in the name        
        allJobs = pbs_functions.getActiveJobs(user)
        
        for (job, status) in allJobs:
            if job in jobList:
                # Matching job found so we keep waiting
                stillWorking = True
                # Print a message if this is the first time we saw the job as running
                if (status == 'R') and (job not in jobsRunning):
                    jobsRunning.append(job)
                    print 'Job launched: ' + job

def checkResultsForType(run, options, batchListPath, batchOutputName):

    numNominal  = 0
    numProduced = 0

    (minFrame, maxFrame) = run.getFrameRange()

    with open(batchListPath, 'r') as f:
        for line in f:
            
            outputFolder = getOutputFolderFromBatchCommand(line)
            (begFrame, endFrame) = icebridge_common.getFrameRangeFromBatchFolder(outputFolder)

            if begFrame < options.startFrame:
                continue
            if begFrame >= options.stopFrame:
                continue
            
            if begFrame >= maxFrame:
                # This is necessary. For the last valid frame, there is never a DEM.
                continue
            
            targetPath   = os.path.join(outputFolder, batchOutputName)
            numNominal += 1
            if os.path.exists(targetPath):
                numProduced += 1
            else:
                logger.info('Did not find: ' + targetPath)
                if not os.path.exists(outputFolder):
                    logger.error('Check output folder position in batch log file!')

    return (numNominal, numProduced)
    
def checkResults(run, options, batchListPath):
    '''Return (numNominalDems, numProducedDems, numNominalOrthos, numProducedOrthos, errorCount)
    to help validate our results'''
    
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

    # Produced DEMs
    (numNominalDems, numProducedDems) = checkResultsForType(run, options,
                                                            batchListPath,
                                                            icebridge_common.blendFileName())
    
    # Produced orthos
    (numNominalOrthos, numProducedOrthos) = checkResultsForType(run, options,
                                                                batchListPath,
                                                                icebridge_common.orthoFileName())

    
    return (numNominalDems, numProducedDems, numNominalOrthos, numProducedOrthos, errorCount)
        
def checkRequiredTools():
    '''Verify that we have all the tools we will be calling during the script.'''

    scripts = ['full_processing_script.py',
               'multi_process_command_runner.py',
               'merge_orbitviz.py',
               'process_icebridge_run.py',
               'process_icebridge_batch.py',
               'lvis2kml.py', 'blend_dems.py', 'gen_ortho.py']
    tools  = ['ortho2pinhole', 'camera_footprint', 'bundle_adjust',
              'stereo', 'point2dem', 'mapproject']
    
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

        # Debug option
        parser.add_argument('--minutes-in-devel-queue', dest='minutesInDevelQueue', type=int,
                            default=0,
                            help="If positive, submit to the devel queue for this many minutes.")

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

        parser.add_argument("--no-refetch", action="store_true", 
                            dest="noRefetch", default=False, 
                            help="Do not attempt to refetch from NSIDC.")
        
        parser.add_argument("--no-refetch-index", action="store_true", 
                            dest="noRefetchIndex", default=False, 
                            help="Do not refetch the index from NSIDC.")

        parser.add_argument("--skip-checks", action="store_true", 
                            dest="skipChecks", default=False, 
                            help="Skip checking if files exist. This can be very slow for many files.")

        parser.add_argument("--skip-completed-batches", action="store_true", 
                            dest="failedBatchesOnly", default=False, 
                            help="Don't reprocess completed batches.")

        parser.add_argument("--skip-fetch", action="store_true", dest="skipFetch", default=False, 
                            help="Don't fetch.")
        parser.add_argument("--skip-convert", action="store_true", dest="skipConvert",
                            default=False, 
                            help="Don't convert.")

        parser.add_argument("--skip-archive-cameras", action="store_true",
                            dest="skipArchiveCameras", default=False,
                            help="Skip archiving the cameras.")

        parser.add_argument("--skip-batch-gen", action="store_true",
                            dest="skipBatchGen", default=False,
                            help="Skip generating batches.")
        
        parser.add_argument("--skip-process", action="store_true",
                            dest="skipProcess", default=False, 
                            help="Don't process the batches.")
        
        parser.add_argument("--skip-blend", action="store_true", dest="skipBlend", default=False, 
                            help="Skip blending.")

        parser.add_argument("--skip-ortho-gen", action="store_true", dest="skipOrthoGen",
                            default=False, help="Skip ortho image generation.")

        parser.add_argument("--skip-report", action="store_true", dest="skipReport", default=False, 
                            help="Skip summary report step.")

        parser.add_argument("--skip-archive-aligned-cameras", action="store_true",
                            dest="skipArchiveAlignedCameras", default=False,
                            help="Skip archiving the aligned cameras.")

        parser.add_argument("--skip-archive-orthos", action="store_true",
                            dest="skipArchiveOrthos", default=False,
                            help="Skip archiving the generated ortho images.")

        parser.add_argument("--skip-archive-summary", action="store_true",
                            dest="skipArchiveSummary", default=False,
                            help="Skip archiving the summary.")

        parser.add_argument("--skip-archive-run", action="store_true",
                            dest="skipArchiveRun", default=False,
                            help="Skip archiving the DEMs.")

        parser.add_argument("--skip-cleanup", action="store_true",
                            dest="skipCleanup", default=False, 
                            help="Don't cleanup extra files from a run.")

        parser.add_argument("--skip-email", action="store_true", 
                            dest="skipEmail", default=False, 
                            help="Don't send email.")

        parser.add_argument("--no-nav", action="store_true", dest="noNavFetch",
                            default=False, help="Don't fetch or convert the nav data.")

        parser.add_argument("--simple-cameras", action="store_true", dest="simpleCameras",
                            default=False, help="Don't use ortho images to refine camera models.")
        
        parser.add_argument("--skip-validate", action="store_true", dest="skipValidate",
                            default=False, help="Don't validate the input data.")

        parser.add_argument("--wipe-processed", action="store_true", dest="wipeProcessed",
                            default=False,
                            help="Wipe the processed folder.")

        parser.add_argument("--wipe-all", action="store_true", dest="wipeAll", default=False,
                            help="Wipe completely the directory, including the inputs.")
                          
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

    logger.info("Disabling core dumps.") # these just take a lot of room
    os.system("ulimit -c 0")
    os.system("umask 022") # enforce files be readable by others

    # See how many hours we used so far. I think this counter gets updated once a day.
    (out, err, status) = asp_system_utils.executeCommand("acct_ytd", outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)
    logger.info("Hours used so far:\n" + out + '\n' + err)
  
    # TODO: Uncomment when processing more than one run!
    # Get the list of runs to process
    #logger.info('Reading run lists...')
    #allRuns  = readRunList(ALL_RUN_LIST, options)
    #skipRuns = readRunList(SKIP_RUN_LIST, options)
    #doneRuns = readRunList(COMPLETED_RUN_LIST, options)
    #runList  = getRunsToProcess(allRuns, skipRuns, doneRuns)

    run = run_helper.RunHelper(options.site, options.yyyymmdd, options.unpackDir)

    if True:

        # WARNNING: Below we tweak options.startFrame and options.stopFrame.
        # If a loop over runs is implemeneted, things will break!

        # Narrow the frame range. Note that if we really are at the last
        # existing frame, we increment 1, to make sure we never miss anything.
        (minFrame, maxFrame) = run.getFrameRange()
        if options.startFrame < minFrame:
            options.startFrame = minFrame
        if options.stopFrame >= maxFrame:
            options.stopFrame = maxFrame + 1 # see above
            
        logger.info('Detected frame range: ' + str((options.startFrame, options.stopFrame)))

        # TODO: Put this in a try/except block so it keeps going on error

        # TODO: Prefetch the next run while waiting on this run!

        fullBatchListPath = os.path.join(run.getProcessFolder(), 'batch_commands_log.txt')
        batchListPath = fullBatchListPath

        if not options.skipFetch:
            # Obtain the data for a run if it is not already done
            runFetch(run, options)       

        if not options.skipConvert:                   
            # Run initial camera generation
            runConversion(run, options)

        # Pack up camera folder and store it for later.
        if not options.skipArchiveCameras:
            try:
                archive_functions.packAndSendCameraFolder(run)
            except Exception, e:
                print 'Caught exception sending camera folder'
                logger.exception(e)

        # I see no reason to exit early
        #if options.skipProcess and options.skipBlend and options.skipReport and \
        #       (not options.recomputeBatches) and options.skipArchiveCameras:
        #    logger.info('Quitting early.')
        #    return 0

        if not options.skipBatchGen:
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
            (baseName, jobList) = submitBatchJobs(run, options, batchListPath)

            # Wait for all the jobs to finish
            logger.info('Waiting for job completion of run ' + str(run))
            
            waitForRunCompletion(baseName, jobList)
            logger.info('All jobs finished for run '+str(run))
        
        if not options.skipBlend:
            runJobs(run, 'blend', options)
        
        if not options.skipOrthoGen:
            runJobs(run, 'orthogen', options)

        # TODO: Uncomment when processing multiple runs.
        ## Log the run as completed
        ## - If the run was not processed correctly it will have to be looked at manually
        #addToRunList(COMPLETED_RUN_LIST, run)

        # Pack up the aligned cameras and store them for later
        if not options.skipArchiveAlignedCameras:
            try:
                archive_functions.packAndSendAlignedCameras(run)
            except Exception, e:
                print 'Caught exception sending aligned cameras'
                logger.exception(e)

        if not options.skipReport:
            # Generate a simple report of the results
            (numNominalDems, numProducedDems,
             numNominalOrthos, numProducedOrthos,
             errorCount) = checkResults(run, options, fullBatchListPath)
            resultText = ""
            resultText += ('Created %d out of %d output DEMs. ' % 
                          (numProducedDems, numNominalDems))
            resultText += ('Created %d out of %d output ortho images. ' % 
                          (numProducedOrthos, numNominalOrthos))
            resultText += ('Detected %d errors. ' % 
                          (errorCount))
            logger.info(resultText)

            # Generate a summary folder and send a copy to Lou
            # - Currently the summary folders need to be deleted manually, but they should not
            #   take up much space due to the large amount of compression used.
            summaryFolder = os.path.join(options.summaryFolder, run.name())
            genCmd = ['--yyyymmdd', run.yyyymmdd, '--site', run.site, 
                      '--output-folder', summaryFolder, '--parent-folder', run.parentFolder]

            if options.startFrame != icebridge_common.getSmallestFrame() and \
               options.stopFrame != icebridge_common.getLargestFrame():
                genCmd += ['--start-frame', str(options.startFrame),
                           '--stop-frame', str(options.stopFrame)]
            logger.info("Running generate_flight_summary.py " + " ".join(genCmd))
            generate_flight_summary.main(genCmd)
        else:
            resultText        = 'Summary skipped'
            errorCount        = 0 # Flag values to let the next condition pass
            numNominalDems    = 1
            numProducedDems   = 1
            numNominalOrthos  = 1
            numProducedOrthos = 1
            
        # send data to lunokhod and lfe
        if not options.skipArchiveSummary:
            summaryFolder = os.path.join(options.summaryFolder, run.name())
            archive_functions.packAndSendSummaryFolder(run, summaryFolder)
            

        # Don't pack or clean up the run if it did not generate all the output files.
        # - TODO: Will never produce 100% of outputs. So wiping better be done anyway.
        success = False
        if (numProducedDems == numNominalDems) and \
           (numProducedOrthos == numNominalOrthos) and \
           (errorCount == 0):
            print 'TODO: Automatically send the completed files to lou!'
            if not options.skipArchiveRun:
                archive_functions.packAndSendCompletedRun(run)

            # Pack up the generated ortho images and store them for later
            if not options.skipArchiveOrthos:
                try:
                    archive_functions.packAndSendOrthos(run)
                except Exception, e:
                    print 'Caught exception sending ortho images.'
                    logger.exception(e)

            # Note: Wiping happens later.
            # Note: We count only the produced DEMs and orthos within the frame range.
            
            # TODO: Don't wait on the pack/send operation to finish!
            success = True

        if not options.skipEmail:
            emailAddress = getEmailAddress(icebridge_common.getUser())
            logger.info("Sending email to: " + emailAddress)
            if success:
                sendEmail(emailAddress, 'OIB run passed - ' + str(run), resultText)
            else:
                sendEmail(emailAddress, '"OIB run failed - ' + str(run), resultText)
        

        if options.wipeProcessed:
            processedFolder = run.getProcessFolder()
            logger.info("Will delete: " + processedFolder)
            os.system("rm -rf " + processedFolder)
            
        if options.wipeAll:
            outFolder = run.getFolder()
            logger.info("Will delete: " + outFolder)
            os.system("rm -rf " + outFolder)
            
        #raise Exception('DEBUG - END LOOP')
        
    # End loop through runs
    logger.info('==== pleiades_manager script has finished! ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



