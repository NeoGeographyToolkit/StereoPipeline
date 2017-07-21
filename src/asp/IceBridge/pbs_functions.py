
import os

'''Contains functions for working with the PBS job system on the Pleiades supercomputer'''


def getNumActiveJobs(user):
  '''Return the current number of active PBS jobs'''
  
  result = os.system('qstat -u '+user+' | wc -l')
  lines = result.split('\n')
  if len(lines) == 0:
      return 0
  # num tasks = num lines - header lines
  return len(lines) - 3


def getActiveJobs(user):
    '''Returns a list of the currently active jobs'''

    return ['TODO']

def checkForJobName(user, name):
  '''Check if the given job name is in the queue'''
  result = os.system('qstat -u '+user+' | grep '+name+' | wc -l'
  return result != ''


def getNumCores(nodeType):
    '''Return the number of cores available for a given node type'''

    if nodeType == 'san':
        return 16
    if nodeType == 'ivy':
        return 20
    if nodeType == 'has':
        return 24
    if nodeType == 'broadwell': # TODO: What is the code??
        return 28
    raise Exception('Unrecognized node type: ' + nodeType)


def submitJob(jobName, queueName, maxHours, groupId, nodeType, scriptPath, args, logPrefix):
    '''Submits a job to the PBS system'''
    
    MAX_NAME_LENGTH = 15
    if len(queueName) > MAX_NAME_LENGTH:
        raise Exception('Job name "'+queueName+'" exceeds the maximum length of ' + str(MAX_NAME_LENGTH))
    
    numCpus = getNumCores(nodeType) # Cores or CPUs?
    
    hourString = '"'+str(maxHours)+':00:00"'
    
    errorsPath = log_prefix + '_errors.log'
    outputPath = log_prefix + '_output.log'
    
    workDir = os.cwd()
    
    command = ('qsub -q %s -N %s -l walltime=%s -W group_list=%s -j oe -e %s -o %s -S /bin/bash -V -C %s -l select=1:ncpus=%d:model=%s -m eb -- %s %s' % 
               (queueName, jobName, hourString, groupId, errorsPath, outputPath, workDir, numCpus, nodeType, scriptPath, args))
    os.system(command)
