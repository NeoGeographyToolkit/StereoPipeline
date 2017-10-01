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

import os, sys, argparse, multiprocessing

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

    try:
        usage = '''usage: multi_process_command_runner.py ...'''
                      
        parser = argparse.ArgumentParser(usage=usage)

        # Data selection optios
        parser.add_argument('--start-frame', dest='startFrame', type=int,
                            default=icebridge_common.getSmallestFrame(),
                            help="Frame to start with.  Leave this and stop-frame blank to " + \
                            "process all frames.")
        
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                            default=icebridge_common.getLargestFrame(),
                            help='Frame to stop on. This last one will not be processed.')

        parser.add_argument('--num-processes', dest='numProcesses', type=int,
                            default=-1,
                            help='How many processes to start at the same time.')

        parser.add_argument("--command-file-path",  dest="commandFilePath", default=None,
                          help="The file from where to read the commands to process.")
        
        parser.add_argument("--force-redo-these-frames",  dest="redoFrameList", default="",
                          help="For each frame in this file (stored one per line) within the current frame range, delete the batch folder and redo the batch.")

        options = parser.parse_args(argsIn)
        
    except argparse.ArgumentError, msg:
        parser.error(msg)

    icebridge_common.switchWorkDir()
    
    os.system("ulimit -c 0") # disable core dumps
    os.system("umask 022")   # enforce files be readable by others
    
    if not os.path.exists(options.commandFilePath):
        print 'Error: File ' + options.commandFilePath + ' does not exist!'
        return -1

    # TODO: Write to a log?

    print 'Starting processing pool with ' + str(options.numProcesses) +' processes.'
    pool = multiprocessing.Pool(options.numProcesses)
    taskHandles = []

    framesToDo = set()
    if options.redoFrameList != "" and os.path.exists(options.redoFrameList):
        with open(options.redoFrameList, 'r') as f:
            text = f.read()
        for line in text.split('\n'):
            line = line.strip()
            if line == "":
                continue
            framesToDo.add(int(line))

    # Open the file and loop through all the lines
    # - Count the lines as we go so we only process the desired lines
    print 'Opening command file ' + options.commandFilePath
    text = ''
    with open(options.commandFilePath, 'r') as f:
        text = f.read()
    
    for line in text.split('\n'):
        
        if line == "":
            continue

        (begFrame, endFrame) = icebridge_common.getFrameRangeFromBatchFolder(line)
        
        # Check line indices
        if begFrame >= options.startFrame and begFrame < options.stopFrame:

            if options.redoFrameList != "":
                if begFrame in framesToDo:
                    folderName = icebridge_common.getBatchFolderFromBatchLine(line)
                    if os.path.exists(folderName):
                        print("will wipe " + folderName)
                        cmd = "rm -rf " + folderName
                        print(cmd)
                        try:
                            os.system(cmd)
                        except Exception, e:
                            pass
                    else:
                        print("Could not find " + folderName)
                else:
                    print("Will skip frame: " + str(begFrame))
                    continue
                
            # Add the command to the task pool
            taskHandles.append(pool.apply_async(runCommand, (line,)))

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
