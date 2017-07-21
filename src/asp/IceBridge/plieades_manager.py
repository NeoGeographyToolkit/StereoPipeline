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

import icebridge_common, pbs_functions

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]




# Constants used in this file
PBS_QUEUE = 'devel'
#NODE_TYPE = 'san' # Sandy bridge = 16 core,  32 GB mem, SBU 1.82
NODE_TYPE = 'ivy' # Ivy bridge   = 20 core,  64 GB mem, SBU 2.52
#NODE_TYPE = 'has' # Haswell      = 24 core, 128 GB mem, SBU 3.34
#NODE_TYPE = '???' # Broadwell    = 28 core, 128 GB mem, SBU 4.04
NUM_ORTHO_NODES     = 1
NUM_ORTHO_PROCESSES = 10
NUM_ORTHO_THREADS   = 2
NUM_BATCH_NODES     = 1
NUM_BATCH_THREADS   = 8 # MGM is limited to 8 threads
NUM_BATCH_PROCESSES = 3

# Wait this long between checking for job completion
SLEEP_TIME = 30

ALL_RUN_LIST       = 'full_run_list.txt'
SKIP_RUN_LIST      = 'run_skip_list.txt'
COMPLETED_RUN_LIST = 'completed_run_list.txt'

CALIBRATION_FILE_FOLDER = ''
REFERENCE_DEM_FOLDER    = ''


def getUser():
    '''Return the current user name.'''
    return getpass.getuser()

def readRunList(path):
    '''Reads a list of runs in this format: GR 20110411.
       Output is a list of (site, yyyymmdd) pairs.'''

    runList = []
    with open(path, 'r') as f:
        for line in f:
            if line[0] == '#': # Skip comments
                continue
            parts = line.split() # Site, yyyymmdd
            runList.append(parts)
    return runList

def getRunsToProcess(allRuns, skipRuns, doneRuns):
    '''Go through the run lists to get the list of runs that
       should be processed.'''
       
    runList = []
    for run in allRuns:
        if (run in skipRuns) or (run in doneRuns):
            continue
        runList.append(run)
    return runList

def retrieveRunData(run):
    '''Retrieve the data for the specified run from Lou.'''

    logger = logging.getLogger(__name__)
    logger.info('Retrieving data for run ' + str(run))

    print 'TODO: Retrieve the run data from Lou!'

def runConversion(run):
    '''Run the conversion tasks for this run on the supercomputer nodes'''
    
    # Get the frame range for the data.
    
    raise Exception('DEBUG')
    
    # Split the conversions across multiple nodes using frame ranges
    # - The first job submitted will be the only one that converts the lidar data
    # pleiades_job_runner full_processing_script --skip-fetch --stop-after-convert --no-lidar-convert

    # Wait for conversions to finish


def generateBatchList(run):
    '''Generate a list of all the processing batches required for a run'''

def submitBatchJobs(run, batchListPath):
    '''Read all the batch jobs required for a run and distribute them across job submissions'''
    
    

def waitForRunCompletion(run):
    '''Sleep until all of the submitted jobs for a run have completed'''

    user = getUser()
    stillWorking = True
    while stillWorking:
        os.sleep(SLEEP_TIME)
        stillWorking = False

        # Look through the list for jobs with the run's date in the name        
        jobList = pbs_functions.getActiveJobs(user)
        for job in jobList()
            if run[1] in job:
                stillWorking = True
                break


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

        if len(args) != 2:
            print usage
            return -1
        inputCalFolder = os.path.abspath(args[0])
        refDemFolder   = os.path.abspath(args[1])

    except optparse.OptionError, msg:
        raise Usage(msg)

    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel, 'plieades_manager_log')

    # Get the list of runs to process
    logger.info('Reading run lists...')
    allRuns  = readRunList(ALL_RUN_LIST      )
    skipRuns = readRunList(SKIP_RUN_LIST     )
    doneRuns = readRunList(COMPLETED_RUN_LIST)
    
    runList = getRunsToProcess(allRuns, skipRuns, doneRuns)


    runList = [runList[0]] # DEBUG
    
   
    # Loop through the incomplete runs
    for run in runList:

        # Obtain the data for a run
        logger.info('Retrieving data for run ' + str(run))
        retrieveRunData(run)
        
        raise Exception('DEBUG')
        
        if not conversionIsFinished(run):
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
        waitForRunCompletion(run)
        
        
    # End loop through runs
    logger.info('==== plieades_manager script has finished! ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



