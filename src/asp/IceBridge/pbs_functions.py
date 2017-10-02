import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing
import re, shutil, platform

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

'''Contains functions for working with the PBS job system on the Pleiades supercomputer'''

# Constants
MAX_PBS_NAME_LENGTH = 15

# Wait this long between checking for job completion
SLEEP_TIME = 60

def getNumActiveJobs(user):
  '''Return the current number of active PBS jobs'''
  
  result = os.system('qstat -u '+user+' | wc -l')
  lines = result.split('\n')
  if len(lines) == 0:
      return 0
  # num tasks = num lines - header lines
  return len(lines) - 3


def getActiveJobs(user):
    '''Returns a list of the currently active jobs and their status'''

    # Run qstat command to get a list of all active jobs by this user
    cmd = ['qstat', '-u', user]

    (textOutput, err, status) = \
          asp_system_utils.executeCommand(cmd,
                                          outputPath=None, 
                                          suppressOutput = True,
                                          redo=True, 
                                          noThrow=True, 
                                          numAttempts = 1000,
                                          sleepTime = 60
                                          )
    
    lines = textOutput.split('\n')
    
    # Strip header lines
    NUM_HEADER_LINES = 3
    NAME_INDEX       = 3
    STATUS_INDEX     = 7
    if len(lines) <= NUM_HEADER_LINES:
        return []
    lines = lines[NUM_HEADER_LINES:]

    # Pick out all the job names and put them in a list
    jobs = []
    for line in lines:
        parts = line.split()
        if len(parts) < STATUS_INDEX:
            continue
        name   = parts[NAME_INDEX]
        status = parts[STATUS_INDEX]
        jobs.append((name, status))
    return jobs
    

def checkForJobName(user, name):
  '''Check if the given job name is in the queue'''
  result = os.system('qstat -u '+user+' | grep '+name+' | wc -l')
  return result != ''


def getNumCores(nodeType):
    '''Return the number of cores available for a given node type'''

    if nodeType == 'wes': # Merope
        return 12
    if nodeType == 'san':
        return 16
    if nodeType == 'ivy':
        return 20
    if nodeType == 'has':
        return 24
    if nodeType == 'bro':
        return 28
    raise Exception('Unrecognized node type: ' + nodeType)


def submitJob(jobName, queueName, maxHours, minutesInDevelQueue,
              groupId, nodeType, scriptPath, args, logPrefix):
    '''Submits a job to the PBS system.
    Any such job must invoke icebridge_common.switchWorkDir().'''
    
    if len(queueName) > MAX_PBS_NAME_LENGTH:
        raise Exception('Job name "'+queueName+'" exceeds the maximum length of ' + str(MAX_PBS_NAME_LENGTH))
    
    numCpus = getNumCores(nodeType) # Cores or CPUs?
    
    hourString = '"'+str(maxHours)+':00:00"'
    
    errorsPath = logPrefix + '_errors.log'
    outputPath = logPrefix + '_output.log'
    
    workDir = os.getcwd()

    numAttempts = 1000
    
    # For debugging
    if minutesInDevelQueue > 0:
      queueName = 'devel'
      hourString = '00:' + str(minutesInDevelQueue).zfill(2) + ':00'
      numAttempts = 1 # For some reason qsub errors out in the devel queue
      
    # TODO: Does this need to be wrapped in a shell script?
    # The "-m eb" option sends the user an email when the process begins and when it ends.
    # The -r n ensures the job does not restart if it runs out of memory.

    # Debug the environment
    #for v in os.environ.keys():
    #  print("env is " + v + '=' + os.environ[v])

    # We empty PYTHONSTARTUP and LD_LIBRARY_PATH so that python can function
    # properly on the nodes. 
    command = ('qsub -r n -q %s -N %s -l walltime=%s -W group_list=%s -j oe -e %s -o %s -S /bin/bash -V -C %s -l select=1:ncpus=%d:model=%s  -- /usr/bin/env OIB_WORK_DIR=%s PYTHONPATH=/u/oalexan1/.local/lib/python2.7/site-packages PYTHONSTARTUP="" LD_LIBRARY_PATH="" %s %s' % 
               (queueName, jobName, hourString, groupId, errorsPath, outputPath, workDir, numCpus, nodeType, workDir, scriptPath, args))
    print command

    outputPath = None
    suppressOutput = True
    redo = True
    noThrow = True
    (out, err, status) = \
          asp_system_utils.executeCommand(command, outputPath, 
                                          suppressOutput, redo, 
                                          noThrow, numAttempts, SLEEP_TIME)
      
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
        allJobs = getActiveJobs(user)
        
        for (job, status) in allJobs:
            if job in jobList:
                # Matching job found so we keep waiting
                stillWorking = True
                # Print a message if this is the first time we saw the job as running
                if (status == 'R') and (job not in jobsRunning):
                    jobsRunning.append(job)
                    print 'Job launched: ' + job

