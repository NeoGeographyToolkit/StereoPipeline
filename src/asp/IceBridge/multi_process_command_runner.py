

import os


'''Simple tool to run commands in parallel from an input file.
   First argument is the input file containing the commands to run.
   Second argument is the number of parallel processes to use.
   Third argument (optional) is the starting line.
   Fourth argument (optional) is the stopping line.'''

def runCommand(command):
    '''Run one of the commands from the file'''
    os.system(command)

def main(argsIn):

    # Parse the input commands
    commandFilePath = argsin[0]
    numProcesses    = argsin[1]
    startLine = 0
    stopLine  = None
    if argsIn > 2:
        startLine = int(argsIn[2]) # Start from this line
    if argsIn > 3:
        stopLine  = int(argsIn[3]) # Stop before you reach this line
    
    if not os.path.exists(commandFilePath):
        print 'Error: File ' + commandFilePath + ' does not exist!'
        return -1

    # TODO: Write to a log?

    print 'Starting ortho processing pool with ' + str(numProcesses) +' processes.'
    pool = multiprocessing.Pool(numProcesses)
    taskHandles = []

    # Open the file and loop through all the lines
    # - Count the lines as we go so we only process the desired lines
    with open(commandFilePath, 'r') as f:
        index = 0
        for line in f:
        
            # Check line indices
            if index < startLine:
                continue
            if stopLine and (index >= stopLine):
                break
                          
            # Add the command to the task pool
            taskHandles.append(pool.apply_async(runCommand, (command)))
            index += 1


    # Wait for all the tasks to complete
    logger.info('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, interactive=False)


    # All tasks should be finished, clean up the processing pool
    print 'Cleaning up the processing pool...'
    icebridge_common.stopTaskPool(pool)
    print 'Finished cleaning up the processing pool'

    


# Run main function if called from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
