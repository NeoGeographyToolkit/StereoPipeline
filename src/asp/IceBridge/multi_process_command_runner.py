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

import os, sys, multiprocessing

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools
toolspath = os.path.abspath(basepath + '/../Tools')  # ASP Tools
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, toolspath)

import icebridge_common
import asp_system_utils, asp_alg_utils, asp_geo_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath   + os.pathsep + os.environ["PATH"]

'''Simple tool to run commands in parallel from an input file.
   First argument is the input file containing the commands to run.
   Second argument is the number of parallel processes to use.
   Third argument (optional) is the starting line.
   Fourth argument (optional) is the stopping line (not processed).'''

def runCommand(command):
    '''Run one of the commands from the file'''
    print command
    os.system(command)  

def main(argsIn):

    # Parse the input commands
    commandFilePath = argsIn[0]
    numProcesses    = int(argsIn[1])
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
    print 'Opening command file ' + commandFilePath
    text = ''
    with open(commandFilePath, 'r') as f:
        text = f.read()
    
    index = 0
    for line in text.split('\n'):
    
        # Check line indices
        if index < startLine:
            index += 1
            continue
        if stopLine and (index >= stopLine):
            break
                      
        # Add the command to the task pool
        taskHandles.append(pool.apply_async(runCommand, (line,)))
        index += 1

    # Wait for all the tasks to complete
    print 'Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.'
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, interactive=False)

    # All tasks should be finished, clean up the processing pool
    print 'Cleaning up the processing pool...'
    icebridge_common.stopTaskPool(pool)
    print 'Finished cleaning up the processing pool'

    


# Run main function if called from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
