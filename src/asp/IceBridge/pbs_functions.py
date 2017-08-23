
import os, subprocess

'''Contains functions for working with the PBS job system on the Pleiades supercomputer'''

# Constants
MAX_PBS_NAME_LENGTH = 15

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
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    textOutput, err = p.communicate()
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


def submitJob(jobName, queueName, maxHours, groupId, nodeType, scriptPath, args, logPrefix):
    '''Submits a job to the PBS system'''
    
    if len(queueName) > MAX_PBS_NAME_LENGTH:
        raise Exception('Job name "'+queueName+'" exceeds the maximum length of ' + str(MAX_PBS_NAME_LENGTH))
    
    numCpus = getNumCores(nodeType) # Cores or CPUs?
    
    hourString = '"'+str(maxHours)+':00:00"'
    
    errorsPath = logPrefix + '_errors.log'
    outputPath = logPrefix + '_output.log'
    
    workDir = os.getcwd()
    
    # TODO: Does this need to be wrapped in a shell script?
    # The "-m eb" option sends the user an email when the process begins and when it ends.
    # The -r n ensures the job does not restart if it runs out of memory.
    command = ('qsub -r n -q %s -N %s -l walltime=%s -W group_list=%s -j oe -e %s -o %s -S /bin/bash -V -C %s -l select=1:ncpus=%d:model=%s  -- %s %s' % 
               (queueName, jobName, hourString, groupId, errorsPath, outputPath, workDir, numCpus, nodeType, scriptPath, args))
    print command
    os.system(command)
    
    
