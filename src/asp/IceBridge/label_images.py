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

# Wrapper Script for the OSSP machine learning based labelling tool.

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing, re, glob
import traceback
import os.path as P

# The path to the ASP python files and tools
basepath      = os.path.dirname(os.path.realpath(__file__))  # won't change, unlike syspath
pythonpath    = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath   = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
binpath       = os.path.abspath(basepath + '/../bin')        # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools
toolspath     = os.path.abspath(basepath + '/../Tools')      # ASP Tools

# Prepend to Python path
sys.path.insert(0, basepath)
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)

import icebridge_common
import asp_system_utils, asp_alg_utils, asp_geo_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath      + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath        + os.pathsep + os.environ["PATH"]

# TODO: Should this be run per-frame or as a single multi-frame call?
# - Is it using enough threads?

def label_images(inputFolder, outputFolder, minFrame, maxFrame, trainingPath, numProcesses):
    '''Apply the labeling algorithm to a single image.'''

    # Format the input path

    # Run the label tool

    toolPath = 'python ~/repo/OSSP/ossp_process.py'
    
    NO_SPLITTING = 1 # Plenty of RAM to load these images
    
    cmd = ('%s %s --verbose --output_dir %s --min_frame %d --max_frame %d srgb %s --splits %d --parallel %d' % 
           (toolPath, inputFolder, outputFolder, minFrame, maxFrame, trainingPath, NO_SPLITTING, numProcesses))
    print cmd
    os.system(cmd)
            
def main(argsIn):

    try:
        usage = '''label_image.py <options>'''
                      
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL)")

        parser.add_argument("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        parser.add_argument('--start-frame', dest='startFrame', type=int,
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                          default=icebridge_common.getLargestFrame(),
                          help='Frame to stop on. This frame will also be processed.')
                          
                          
        parser.add_argument("--training",  dest="trainingPath", required=True,
                          help="Path to the training file.")
                          
        parser.add_argument('--num-processes', dest='numProcesses', default=8,
                          type=int, help='The number of simultaneous processes to run.')
        parser.add_argument('--num-threads', dest='numThreads', default=1,
                          type=int, help='IGNORED.')
                         
        options = parser.parse_args(argsIn)

    except argparse.ArgumentError as msg:
        parser.error(msg)

    if not os.path.exists(options.trainingPath):
        print 'Error: Input training file ' + options.trainingPath + ' does not exist!'
        return -1

    # TODO: Everything should use the RunHelper class for this!
    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)

    # Input is raw jpeg files.
    inputFolder = icebridge_common.getJpegFolder(options.outputFolder)

    # Write all tool output to this folder.
    outputFolder = icebridge_common.getLabelFolder(options.outputFolder)
    
    # Do the work
    #label_images(inputFolder, outputFolder, options.startFrame, options.stopFrame, 
    #             options.trainingPath, options.numProcesses)

    # Set up a processing tool to handle the frames, this will be more efficient
    #  than using the built-in mulithreading support.
    pool = multiprocessing.Pool(options.numProcesses)
    taskHandles = []

    for i in range(options.startFrame, options.stopFrame+1):

        # Run on a single frame with one thread.
        #label_images(inputFolder, outputFolder, options.startFrame, options.stopFrame, 
        #             options.trainingPath, options.numProcesses)
        taskHandles.append(pool.apply_async(label_images, (inputFolder, outputFolder, i, i, 
                                                           options.trainingPath, 1)))

    # Wait for all the tasks to complete
    print('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, interactive=False)

    # All tasks should be finished, clean up the processing pool
    icebridge_common.stopTaskPool(pool)
    print('Jobs finished.')


    
# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


