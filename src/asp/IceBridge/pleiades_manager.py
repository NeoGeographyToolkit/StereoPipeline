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
import re, shutil, glob

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
LABEL_PBS_QUEUE = 'normal'

GROUP_ID = 's1827'

g_start_time = -1
g_stop_time  = -1

def start_time():
    global g_start_time, g_stop_time
    g_start_time = time.time()
    
def stop_time(job, logger):
    global g_start_time, g_stop_time
    g_stop_time = time.time()
    wall_s = float(g_stop_time - g_start_time)/3600.0
    logger.info( ("Elapsed time for %s is %g hours." % (job, wall_s) ) )


PFE_NODES = ['san', 'ivy', 'has', 'bro']

#=========================================================================

# 'wes' = Westmere = 12 cores/24 processors, 48 GB mem, SBU 1.0, Launch from mfe1 only!
# 'san' = Sandy bridge = 16 cores,  32 GB mem, SBU 1.82
# 'ivy' = Ivy bridge   = 20 cores,  64 GB mem, SBU 2.52
# 'has' = Haswell      = 24 cores, 128 GB mem, SBU 3.34
# 'bro' = Broadwell    = 28 cores, 128 GB mem, SBU 4.04

def getParallelParams(nodeType, task):
    '''Return (numProcesses, numThreads, tasksPerJob, maxHours) for running a certain task on a certain node type'''

    # Define additional combinations and edit as needed.

    if task == 'camgen':
        if nodeType == 'san': return (8,  3, 300, 3)
        if nodeType == 'ivy': return (10, 3, 400, 3)
        if nodeType == 'bro': return (14, 4, 500, 3)
        if nodeType == 'wes': return (10, 4, 400, 8)
    
    if task == 'dem':
        if nodeType == 'san': return (2, 8, 70,  8)
        if nodeType == 'ivy': return (4, 8, 80,  8)
        if nodeType == 'bro': return (6, 8, 100, 8)
        if nodeType == 'wes': return (3, 8, 75,  8)
    
    if task == 'blend':
        if nodeType == 'san': return (8,  3,  800, 4)
        if nodeType == 'ivy': return (10, 3, 1000, 4)
        if nodeType == 'bro': return (14, 4, 1400, 4) # 200 seems to finish in 10 minutes
        if nodeType == 'wes': return (10, 3,  800, 8) 
    
    if task == 'orthogen':
        if nodeType == 'san': return (8,  2, 350, 6)
        if nodeType == 'ivy': return (10, 2, 400, 5)
        if nodeType == 'bro': return (14, 4, 500, 4)
        if nodeType == 'wes': return (10, 4, 400, 8)

    # TODO: All guesses!
    if task == 'label':
        if nodeType == 'san': return (16, 1, 350, 6)
        if nodeType == 'ivy': return (30, 1, 400, 5)
        if nodeType == 'bro': return (28, 1, 500, 4)
        if nodeType == 'wes': return (12, 1, 400, 8)


    raise Exception('No params defined for node type ' + nodeType + ', task = ' + task)

#=========================================================================

def getLabelTrainingPath(userName):
    '''Path to the OSSP label training file'''

    if userName == 'smcmich1':
        return '/u/smcmich1/repo/OSSP/training_datasets/icebridge_v1_training_data.h5'
    if userName == 'oalexan1':
        raise Exception('Need to set the label training path!')

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
        #print(cmd) # too verbose to print
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


def runFetch(run, options, logger):
    '''Fetch all the data for a run if it is not already available'''

    logger.info("Checking all data is present.")

    # Check if already done
    allIsFetched = False
    if options.skipCheckInputs:
        allIsFetched = True
    else:
        try:
            # Note that the check below is incomplete, it does not check for Fireball
            allIsFetched = run.allSourceDataFetched(options.noNavFetch)
        except Exception, e:
            allIsFetched = False
            logger.warning('Caught error checking fetch status.\n'
                       + str(e))

    # Fetch the archive from lfe, only in the case the directory is not present
    if not options.skipTapeFetch:
        archive_functions.retrieveRunData(run, options.unpackDir, options.useTar,
                                          options.forceTapeFetch, logger)

    pythonPath = asp_system_utils.which('python')
    
    # Fetch whatever is missing directly from NSIDC, and force to have the indices
    # regenerated in this case. Hopefully just a few files are missing.
    # - This is likely to have to fetch the large nav data file(s).
    # Note that we do all conversions as well, sans camera generation. But hopefully the run is clean and all
    # has been done by now. 
    if not options.noRefetch:
        logger.info("Fetch from NSIDC.")
        cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --refetch --stop-after-fetch --start-frame %d --stop-frame %d' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder(), options.startFrame, options.stopFrame))

        if options.noNavFetch:
            cmd += ' --no-nav'
            
        if options.skipValidate or options.parallelValidate:
            # We will do validation in parallel later
            cmd += ' --skip-validate'

        if not options.noRefetchIndex:
            cmd += ' --refetch-index'
            
        logger.info(cmd)
        os.system(cmd)

    # Rename to the newest convention of computing the timestamp.
    # This must happen after all is fetched from tape and we
    # refreshed the indicies from NSIDC.
    run.massRename(options.startFrame, options.stopFrame, logger)
    
    # Don't need to check results, they should be cleaned out in conversion call.
    run.setFlag('fetch_complete')

def runConversion(run, options, conversionAttempt, logger):
    '''Run the conversion tasks for this run on the supercomputer nodes.
       This will also run through the fetch step to make sure we have everything we need.
       Note: This function calls itself!'''

    # Check if already done. If we want to validate in parallel, we must still
    # go through this step, though hopefully it will be very fast then.
    # TODO: Can have a check if all files have been validated (jpeg, ortho, image, etc),
    # and then, if all cameras are also present, this step can be skipped.
    try:
        if run.conversionIsFinished(options.startFrame, options.stopFrame)   and \
           run.checkForImages(options.startFrame, options.stopFrame, logger) and \
           (not options.parallelValidate):
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
        cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --skip-fetch --stop-after-convert --no-lidar-convert --no-ortho-convert --skip-fast-conversions --start-frame %d --stop-frame %d' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder(), options.startFrame, options.stopFrame))
    
        if options.skipValidate or options.parallelValidate:
            cmd += ' --skip-validate'
            
        logger.info(cmd)
        os.system(cmd)    
        logger.info("Finished generating estimated camera files from nav.")
    
    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob, maxHours) = getParallelParams(options.nodeType, 'camgen')
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data.
    # We will also run validation in parallel.
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
    if options.simpleCameras:       # This option greatly decreases the conversion run time
        args += ' --simple-cameras' # - Camera conversion could be local but image conversion still takes time.
        maxHours = 1

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()
    os.system('mkdir -p ' + pbsLogFolder)

    # Submit all the jobs
    currentFrame = options.startFrame
    jobList = []
    for i in range(0, numCamgenJobs):
        jobName    = ('%s%06d%s' % ('C', currentFrame, baseName) ) # C for camera
        startFrame = currentFrame
        stopFrame  = currentFrame+tasksPerJob-1
        if (i == numCamgenJobs - 1):
            stopFrame = options.stopFrame # Make sure nothing is lost at the end

        thisArgs = (args + ' --start-frame ' + str(startFrame)
                         + ' --stop-frame  ' + str(stopFrame ) )
        if i != 0: # Only the first job will convert lidar files
            thisArgs += ' --no-lidar-convert'

        logPrefix = os.path.join(pbsLogFolder, 'convert_' + jobName)
        logger.info('Submitting camera generation job: ' + scriptPath + ' ' + thisArgs)
        pbs_functions.submitJob(jobName, CAMGEN_PBS_QUEUE, maxHours, logger,
                                options.minutesInDevelQueue,
                                GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + " " + thisArgs, logPrefix)
        jobList.append(jobName)
        currentFrame += tasksPerJob

    # Wait for conversions to finish
    pbs_functions.waitForJobCompletion(jobList, logger)

    if options.parallelValidate:

        # Consolidate various validation files done in parallel, to make the subsequent
        # step faster.
        validFilesList = icebridge_common.validFilesList(outputFolder,
                                                         options.startFrame, options.stopFrame)
        validFilesSet = set()
        validFilesSet = icebridge_common.updateValidFilesListFromDisk(validFilesList, validFilesSet)
        allValidFiles = glob.glob(os.path.join(outputFolder,
                                               icebridge_common.validFilesPrefix() + '*'))
        for fileName in allValidFiles:
            validFilesSet = icebridge_common.updateValidFilesListFromDisk(fileName, validFilesSet)
        icebridge_common.writeValidFilesList(validFilesList, validFilesSet)

        # Now we will refetch and reprocess all files that were not
        # valid so far. Hopefully not too many. This must be on a head
        # node to be able to access the network. We don't do any orthoconvert
        # here as that one is too time-consuming
        logger.info("Refetch and process any invalid files.")
        pythonPath = asp_system_utils.which('python')
        cmd = (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + ' --refetch --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --stop-after-convert --no-ortho-convert --start-frame %d --stop-frame %d --num-threads %d --num-processes %d' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder(), options.startFrame, options.stopFrame, numThreads, numProcesses))
        if options.noNavFetch:
            cmd += ' --no-nav'
        logger.info(cmd)
        os.system(cmd)

        # See if there are any new files for which we need to run ortho2pinhole on some node
        logger.info("See for which files to redo ortho.")
        (orthoList, numFrames) = \
                    icebridge_common.orthoListToRerun(validFilesSet, outputFolder,
                                                      options.startFrame, options.stopFrame)
        
        logger.info("Number of cameras to regenerate using ortho2pinhole: " + str(numFrames))
        if numFrames > 0:
            if numFrames > tasksPerJob and conversionAttempt == 0:
                # Too many frames, just rerun all jobs
                conversionAttempt = 1
                logger.info("Re-running conversion with many nodes.")
                runConversion(run, options, conversionAttempt, logger)
                return
            
            logger.info("Re-running conversion with one node.")
            cmd = (icebridge_common.fullPath('full_processing_script.py') + ' --skip-fetch --skip-validate --skip-fast-conversions --stop-after-convert --camera-calibration-folder %s --reference-dem-folder %s --site %s --yyyymmdd %s --output-folder %s --start-frame %d --stop-frame %d --num-threads %d --num-processes %d --frames-file %s' % (options.inputCalFolder, options.refDemFolder, run.site, run.yyyymmdd, run.getFolder(), options.startFrame, options.stopFrame, numThreads, numProcesses, orthoList))
            if options.noNavFetch:
                cmd += ' --no-nav'

            #logger.info(cmd)
            jobList = []
            jobName    = ('%s%06d%s' % ('C', 0, baseName) ) # C for camera
            logPrefix = os.path.join(pbsLogFolder, 'convert_' + jobName)
            logger.info('Submitting camera generation job: ' + cmd)
            pbs_functions.submitJob(jobName, CAMGEN_PBS_QUEUE, maxHours, logger,
                                    options.minutesInDevelQueue,
                                    GROUP_ID,
                                    options.nodeType, '/usr/bin/python2.7',
                                    cmd, logPrefix)
            jobList.append(jobName)
            
            # Wait for conversions to finish
            pbs_functions.waitForJobCompletion(jobList, logger, baseName)

        logger.info("Finished refetching and reprocessing.")

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

def generateBatchList(run, options, listPath, logger):
    '''Generate a list of all the processing batches required for a run'''

    logger.info("Generate batch list: " + listPath)
    
    refDemName = icebridge_common.getReferenceDemName(run.site)
    refDemPath = os.path.join(options.refDemFolder, refDemName)

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob, maxHours) = getParallelParams(options.nodeType, 'dem')

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
    m = re.match('^.*?--output-folder\s+(.*?)\s', batchCommand)
    if not m:
        raise Exception('Failed to extract output folder from: ' + batchCommand)
    return m.group(1)

# TODO: Share code with the other function
def filterBatchJobFile(run, batchListPath, logger):
    '''Make a copy of the batch list file which only contains incomplete batches.'''
    
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
    
def submitBatchJobs(run, options, batchListPath, logger):
    '''Read all the batch jobs required for a run and distribute them across job submissions.
       Returns the common string in the job names.'''

    if not os.path.exists(batchListPath):
        logger.error('Failed to generate batch list file: ' + batchListPath)
        raise Exception('Failed to generate batch list file: ' + batchListPath)

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob, maxHours) = getParallelParams(options.nodeType, 'dem')

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
        jobName    = ('%s%06d%s' % ('D', firstFrame, baseName) ) # D for DEM

        # Specify the range of lines in the file we want this node to execute
        args = ('--command-file-path %s --start-frame %d --stop-frame %d --num-processes %d' % \
                (batchListPath, firstFrame, lastFrame, numProcesses))

        if options.redoFrameList != "":
            args += ' --force-redo-these-frames ' + options.redoFrameList
        
        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting DEM creation job: ' + scriptPath + ' ' + args)

        pbs_functions.submitJob(jobName, BATCH_PBS_QUEUE, maxHours, logger,
                                options.minutesInDevelQueue,
                                GROUP_ID,
                                options.nodeType, '/usr/bin/python2.7',
                                scriptPath + ' ' + args, logPrefix)
        jobList.append(jobName)

    # Waiting on these jobs happens outside this function
    return (baseName, jobList)

def launchJobs(run, mode, options, logger):
    '''Run a blend, ortho gen, or label job.'''
    
    # TODO: Also merge with the logic for creating a camera file.
    
    logger.info('Running task ' + mode + ' for run ' + str(run))
    
    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob, maxHours) = getParallelParams(options.nodeType, mode)
    
    # Split the jobs across multiple nodes using frame ranges
    numFrames = options.stopFrame - options.startFrame + 1
    numJobs   = numFrames / tasksPerJob
    if numJobs < 1:
        numJobs = 1
        
    logger.info( ("Num frames: %d, tasks per job: %d, number of %s jobs: %d" %
                  (numFrames, tasksPerJob, mode, numJobs) ) )
    
    outputFolder = run.getFolder()

    scriptPath = ""
    extraArgs  = ""
    jobTag     = ""
    priority   = 5
    if mode == 'orthogen':
        scriptPath = icebridge_common.fullPath('gen_ortho.py')
        queueName  = ORTHOGEN_PBS_QUEUE
        jobTag     = 'O'
        extraArgs  = ' --bundle-length ' + str(options.bundleLength)
    elif mode == 'blend':
        scriptPath = icebridge_common.fullPath('blend_dems.py')
        queueName  = BLEND_PBS_QUEUE
        jobTag     = 'B'
        extraArgs  = ' --blend-to-fireball-footprint --bundle-length ' + str(options.bundleLength)
    elif mode == 'label':
        scriptPath = icebridge_common.fullPath('label_images.py')
        queueName  = LABEL_PBS_QUEUE
        jobTag     = 'L'
        priority   = 0 # Make these slightly lower priority than the other jobs.
                       # May need to experiment with these numbers.
        extraArgs  = ' --training ' + getLabelTrainingPath(icebridge_common.getUser())
    else:
        raise Exception("Unknown mode: " + mode)
    args = (('--site %s --yyyymmdd %s --num-threads %d --num-processes %d ' + \
            '--output-folder %s %s ') 
            % (run.site, run.yyyymmdd, numThreads, numProcesses, outputFolder, extraArgs))
    
    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Get the location to store the logs    
    pbsLogFolder = run.getPbsLogFolder()

    # Submit all the jobs
    jobNames = []
    jobIds   = []
    currentFrame = options.startFrame
    for i in range(0, numJobs):
        jobName    = ('%s%06d%s' % (jobTag, currentFrame, baseName) ) # 'B'lend or 'O'rtho
        startFrame = currentFrame
        stopFrame  = currentFrame+tasksPerJob # Last frame passed to the tool is not processed
        if (i == numJobs - 1):
            stopFrame = options.stopFrame # Make sure nothing is lost at the end
        thisArgs = (args + ' --start-frame ' + str(startFrame) + ' --stop-frame ' + str(stopFrame) )
        logPrefix = os.path.join(pbsLogFolder, mode + '_' + jobName)
        logger.info('Submitting job: ' + scriptPath +  ' ' + thisArgs)
        jobId = pbs_functions.submitJob(jobName, queueName, maxHours, logger,
                                        options.minutesInDevelQueue, GROUP_ID,
                                        options.nodeType, '/usr/bin/python2.7',
                                        scriptPath + ' ' + thisArgs, logPrefix, priority)

        jobNames.append(jobName)
        jobIds.append(jobName)
        currentFrame += tasksPerJob

    return (baseName, jobNames, jobIds)

def checkResultsForType(run, options, batchListPath, batchOutputName, logger):

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
    
def checkResults(run, options, logger, batchListPath):
    
    logger.info("Checking the results.")

    # TODO: Some existing functionlity below can be integrated better.
    
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
                                                            icebridge_common.blendFileName(),
                                                            logger)
    
    # Produced orthos
    (numNominalOrthos, numProducedOrthos) = checkResultsForType(run, options,
                                                                batchListPath,
                                                                icebridge_common.orthoFileName(),
                                                                logger)

    resultText = ""
    resultText += ('Created %d out of %d output DEMs. ' % 
                   (numProducedDems, numNominalDems))
    resultText += ('Created %d out of %d output ortho images. ' % 
                   (numProducedOrthos, numNominalOrthos))
    resultText += ('Detected %d errors. ' % 
                   (errorCount))

    # Load index of fireball DEMs for comparison
    fireballDems = {}
    try:
        allFireballDems = icebridge_common.getCorrectedFireballDems(run.getFolder())
        for frame in allFireballDems.keys():
            if frame < options.startFrame or frame >= options.stopFrame:
                continue
            fireballDems[frame] = allFireballDems[frame]
    except Exception, e:
        # No fireball
        logger.info(str(e))

    numFireballDems = len(fireballDems.keys())
    
    alignedDems = run.existingFilesDict(icebridge_common.alignFileName(),
                                        options.startFrame, options.stopFrame)
    numAlignedDems = len(alignedDems.keys())
    
    blendedDems = run.existingFilesDict(icebridge_common.blendFileName(),
                                        options.startFrame, options.stopFrame)
    numBlendedDems = len(blendedDems.keys())
    
    orthos = run.existingFilesDict(icebridge_common.orthoFileName(),
                                   options.startFrame, options.stopFrame)
    numOrthos = len(orthos.keys())

    numMissingBlended = 0
    for frame in sorted(alignedDems.keys()):
        if frame not in blendedDems.keys():
            logger.info("Found aligned DEM but no blended DEM for frame: " + str(frame))
            numMissingBlended += 1

    numMissingOrthos = 0
    for frame in sorted(blendedDems.keys()):
        if frame not in orthos.keys():
            logger.info("Found blended DEM but no ortho for frame: " + str(frame))
            numMissingOrthos += 1

    numMissingAligned = 0
    for frame in sorted(fireballDems.keys()):
        if not frame in blendedDems.keys():
            logger.info("Found fireball DEM but no blended DEM for frame: " + str(frame))
            numMissingAligned += 1

    vals = ["Number of aligned DEMs: " + str(numAlignedDems),
            "Number of blended DEMs: " + str(numBlendedDems),
            "Number of ortho images: " + str(numOrthos),
            "Number of fireball DEMs: " + str(numFireballDems),
            "Aligned DEMs without blended DEMs: " + str(numMissingBlended),
            "Blended DEMs without ortho images: " + str(numMissingOrthos),
            "Fireball DEMs with no corresponding blended DEM: " + str(numMissingAligned)
            ]
    
    resultText += "\n" + "\n".join(vals)

    logger.info(resultText)
    
    runWasSuccess = False
    if (numProducedDems == numNominalDems) and \
           (numProducedOrthos == numNominalOrthos) and \
           (errorCount == 0):
        runWasSuccess =  True
        
    return (runWasSuccess, resultText)

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

        parser.add_argument("--skip-check-inputs", action="store_true", 
                            dest="skipCheckInputs", default=False, 
                            help="Skip checking if files exist. This can be very slow for many files.")

        parser.add_argument("--skip-completed-batches", action="store_true", 
                            dest="failedBatchesOnly", default=False, 
                            help="Don't reprocess completed batches.")

        parser.add_argument("--force-redo-these-frames",  dest="redoFrameList", default="",
                          help="For each frame in this file (stored one per line) within the current frame range, delete the batch folder and redo the batch (only applies to batch processing).")

        parser.add_argument("--use-tar", action="store_true", dest="useTar", default=False, 
                            help="Fetch from lfe using tar instead of shift.")

        parser.add_argument("--skip-fetch", action="store_true", dest="skipFetch", default=False, 
                            help="Don't fetch.")

        parser.add_argument("--skip-tape-fetch", action="store_true", dest="skipTapeFetch", default=False, 
                            help="Don't fetch from tape, go directly to NSIDC. With this, one must not skip the convert step, as then subsequent steps will fail.")
        
        parser.add_argument("--force-tape-fetch", action="store_true", dest="forceTapeFetch", default=False, 
                            help="Fetch from tape even if a partial run directory is alrady present.")
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

        parser.add_argument("--skip-check-outputs", action="store_true", dest="skipCheckOutputs",
                            default=False, 
                            help="Skip checking the outputs.")

        parser.add_argument("--skip-report", action="store_true", dest="skipReport",
                            default=False, 
                            help="Skip invoking the flight summary tool.")

        parser.add_argument("--skip-kml", action="store_true", dest="skipKml", default=False, 
                            help="Skip kml gen when doing a flight summary report.")

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

        parser.add_argument("--no-parallel-validate", action="store_false", dest="parallelValidate",
                            default=True, help="Validate in parallel, during conversion.")
        
        parser.add_argument("--wipe-processed", action="store_true", dest="wipeProcessed",
                            default=False,
                            help="Wipe the processed folder.")

        parser.add_argument("--wipe-all", action="store_true", dest="wipeAll", default=False,
                            help="Wipe completely the directory, including the inputs.")

        parser.add_argument("--generate-labels", action="store_true",
                            dest="generateLabels", default=False,
                            help="Run python label creation script on input jpegs.  Only for sea ice flights!")

        parser.add_argument("--archive-labels", action="store_true",
                            dest="archiveLabels", default=False,
                            help="Archive the results from --generate-labels.")
                          
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError, msg:
        parser.error(msg)


    # Check if we are on the right machine
    (host, err, status) = asp_system_utils.executeCommand(['uname', '-n'],
                                                         suppressOutput = True)
    host = host.strip()
    if 'pfe' in host and options.nodeType not in PFE_NODES:
        raise Exception("From machine " + host + " can only launch on: " + " ".join(PFE_NODES)) 
    if 'mfe' in host and options.nodeType != 'wes':
        raise Exception("From machine " + host + " can only launch on: wes")
    
    #ALL_RUN_LIST       = os.path.join(options.baseDir, 'full_run_list.txt')
    #SKIP_RUN_LIST      = os.path.join(options.baseDir, 'run_skip_list.txt')
    #COMPLETED_RUN_LIST = os.path.join(options.baseDir, 'completed_run_list.txt')

    options.logFolder = os.path.join(options.baseDir, 'manager_logs')
    os.system('mkdir -p ' + options.logFolder)

    if options.unpackDir is None:
        options.unpackDir = os.path.join(options.baseDir, 'data')
        
    os.system('mkdir -p ' + options.unpackDir)
    
    # TODO: Uncomment when processing more than one run!
    # Get the list of runs to process
    #logger.info('Reading run lists...')
    #allRuns  = readRunList(ALL_RUN_LIST, options)
    #skipRuns = readRunList(SKIP_RUN_LIST, options)
    #doneRuns = readRunList(COMPLETED_RUN_LIST, options)
    #runList  = getRunsToProcess(allRuns, skipRuns, doneRuns)

    run = run_helper.RunHelper(options.site, options.yyyymmdd, options.unpackDir)
    
    summaryFolder = run.getSummaryFolder()
    
    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(options.logFolder, logLevel,
                                            'pleiades_manager_log_' + str(run))

    checkRequiredTools() # Make sure all the needed tools can be found before we start

    logger.info("Disabling core dumps.") # these just take a lot of room
    os.system("ulimit -c 0")
    os.system("umask 022") # enforce files be readable by others

    # See how many hours we used so far. I think this counter gets updated once a day.
    (out, err, status) = asp_system_utils.executeCommand("acct_ytd", outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)
    logger.info("Hours used so far:\n" + out + '\n' + err)
  
    if True:

        # WARNNING: Below we tweak options.startFrame and options.stopFrame.
        # If a loop over runs is implemeneted, things will break!

        # TODO: Put this in a try/except block so it keeps going on error

        # TODO: Prefetch the next run while waiting on this run!

        if not options.skipFetch:
            # Obtain the data for a run if it is not already done
            start_time()
            runFetch(run, options, logger)       
            stop_time("fetch", logger)

        # This must happen after fetch, otherwise fetch gets confused.
        # Get the location to store the logs    
        pbsLogFolder = run.getPbsLogFolder()
        logger.info("Storing logs in: " + pbsLogFolder)
        os.system('mkdir -p ' + pbsLogFolder)
    
        # Narrow the frame range. Note that if we really are at the last
        # existing frame, we increment 1, to make sure we never miss anything.
        
        (minFrame, maxFrame) = run.getFrameRange()
        if options.startFrame < minFrame:
            options.startFrame = minFrame
        if options.stopFrame >= maxFrame:
            options.stopFrame = maxFrame + 1 # see above
        logger.info('Detected frame range: ' + str((options.startFrame, options.stopFrame)))

        if not options.skipConvert:                   
            # Run initial camera generation
            start_time()
            conversionAttempt = 0
            runConversion(run, options, conversionAttempt, logger)
            stop_time("convert", logger)
            
        # I see no reason to exit early
        #if options.skipProcess and options.skipBlend and options.skipReport and \
        #       (not options.recomputeBatches) and options.skipArchiveCameras:
        #    logger.info('Quitting early.')
        #    return 0

        fullBatchListPath = os.path.join(run.getProcessFolder(), 'batch_commands_log.txt')
        batchListPath = fullBatchListPath

        if not options.skipBatchGen:
            start_time()
            if os.path.exists(fullBatchListPath) and not options.recomputeBatches:
                logger.info('Re-using existing batch list file.')
            else:
                # Run command to generate the list of batch jobs for this run
                logger.info('Fetching batch list for run ' + str(run))
                generateBatchList(run, options, fullBatchListPath, logger)

            if options.failedBatchesOnly:
                logger.info('Assembling batch file with only failed batches...')
                batchListPath = filterBatchJobFile(run, batchListPath, logger)
            stop_time("batch gen", logger)

        if not options.skipProcess:
            start_time()
            # Divide up batches into jobs and submit them to machines.
            logger.info('Submitting jobs for run ' + str(run))
            (baseName, jobList) = submitBatchJobs(run, options, batchListPath, logger)

            # Wait for all the jobs to finish
            logger.info('Waiting for job completion of run ' + str(run))
            
            pbs_functions.waitForJobCompletion(jobList, logger, baseName)
            logger.info('All jobs finished for run '+str(run))
            stop_time("dem creation", logger)

        labelJobNames = None
        if options.generateLabels:
            (baseName, labelJobNames, labelJobIds) = launchJobs(run, 'label', options, logger)
            # Go ahead and launch the other jobs while these are in the queue

        if not options.skipBlend:
            start_time()
            (baseName, jobNames, jobIds) = launchJobs(run, 'blend', options, logger)
            pbs_functions.waitForJobCompletion(jobNames, logger, baseName)
            stop_time("blend", logger)
        
        if not options.skipOrthoGen:
            start_time()
            (baseName, jobNames, jobIds) = launchJobs(run, 'orthogen', options, logger)
            pbs_functions.waitForJobCompletion(jobNames, logger, baseName)
            stop_time("orthogen", logger)

        if labelJobNames: # Now wait for any label jobs to finish.
            pbs_functions.waitForJobCompletion(labelJobNames, logger, baseName)

        # TODO: Uncomment when processing multiple runs.
        ## Log the run as completed
        ## - If the run was not processed correctly it will have to be looked at manually
        #addToRunList(COMPLETED_RUN_LIST, run)

        # Pack up camera folder and store it for later.
        if not options.skipArchiveCameras:
            start_time()
            try:
                archive_functions.packAndSendCameraFolder(run, logger)
            except Exception, e:
                print 'Caught exception sending camera folder'
                logger.exception(e)
            stop_time("archive cameras", logger)

        # Pack up the aligned cameras and store them for later
        if not options.skipArchiveAlignedCameras:
            start_time()
            try:
                archive_functions.packAndSendAlignedCameras(run, logger)
            except Exception, e:
                print 'Caught exception sending aligned cameras'
                logger.exception(e)
            stop_time("archive aligned cameras", logger)

        runWasSuccess = True
        resultText = 'Summary skipped'
        
        if not options.skipCheckOutputs:
            start_time()
            # Generate a simple report of the results
            (runWasSuccess, resultText) = checkResults(run, options, logger, fullBatchListPath)
            stop_time("check outputs", logger)
            
        if not options.skipReport:
            start_time()
            # Generate a summary folder and send a copy to Lou
            os.system('mkdir -p ' + summaryFolder)
            genCmd = ['--yyyymmdd', run.yyyymmdd, '--site', run.site, 
                      '--output-folder', summaryFolder, '--parent-folder', run.parentFolder]

            if ((options.startFrame != icebridge_common.getSmallestFrame()) and
                (options.stopFrame  != icebridge_common.getLargestFrame() ) ):
                genCmd += ['--start-frame', str(options.startFrame),
                           '--stop-frame',  str(options.stopFrame )]
            if options.skipKml:
                genCmd += ['--skip-kml']
                
            logger.info("Running generate_flight_summary.py " + " ".join(genCmd))
            try:
                generate_flight_summary.main(genCmd)
            except Exception, e:
                # Do not let this one ruin the day, if anything we can run it later
                logger.info(str(e))
            stop_time("report", logger)
            
        # send data to lunokhod and lfe
        if not options.skipArchiveSummary:
            start_time()
            archive_functions.packAndSendSummaryFolder(run, summaryFolder, logger)
            stop_time("archive summary", logger)
            
        # Archive the DEMs
        if not options.skipArchiveRun:
            start_time()
            archive_functions.packAndSendCompletedRun(run, logger)
            stop_time("archive dems", logger)
            
        # Pack up the generated ortho images and store them for later
        if not options.skipArchiveOrthos:
            start_time()
            try:
                archive_functions.packAndSendOrthos(run, logger)
            except Exception, e:
                print 'Caught exception sending ortho images.'
                logger.exception(e)
            stop_time("archive orthos", logger)

        # Archive the label files
        if options.archiveLabels:
            start_time()
            archive_functions.packAndSendLabels(run, logger)
            stop_time("archive labels", logger)

        if not options.skipEmail:
            emailAddress = getEmailAddress(icebridge_common.getUser())
            logger.info("Sending email to: " + emailAddress)
            if runWasSuccess:
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
        logger.info('==== pleiades_manager script has finished for run: ' + str(run) + ' ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
