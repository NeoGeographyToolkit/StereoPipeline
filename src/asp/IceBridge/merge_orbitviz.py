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

# Merge multiple output files from orbitviz

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing, re, shutil, time
import os.path as P

# The path to the ASP python files and tools
basepath      = os.path.dirname(os.path.realpath(__file__))  # won't change, unlike syspath
pythonpath    = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath   = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
binpath       = os.path.abspath(basepath + '/../bin')        # for packaged ASP
toolspath     = os.path.abspath(basepath + '/../Tools')      # ASP Tools

# Prepend to Python path
sys.path.insert(0, basepath)
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)


# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath      + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath        + os.pathsep + os.environ["PATH"]


def main(args):

    # Parse input arguments
    
    if len(args) == 2:
        # If only one input file was provided, just copy it to the output file.
        shutil.copyfile(args[1], args[0])
        return 0
    
    if len(args) < 3:
        print 'usage: merge_orbitviz.py <output_file> <input_file_1> <input_file_2> ...'
        return -1

    outputPath = os.path.abspath(args[0])
    if args[1] == '-list':
        # Read from list, this is beneficial if the files are too many
        inputPaths = []
        with open(args[2], 'r') as inputFile:
            for line in inputFile:
                line = line.strip()
                inputPaths.append(line)
    else:
        inputPaths = [os.path.abspath(x) for x in args[1:]]

    # Open the output file
    with open(outputPath, 'w') as outputFile:
        # Copy most of the input file, stopping at the document close tags
        with open(inputPaths[0], 'r') as inputFile:
            for line in inputFile:
                if '</Document>' in line:
                    break
                outputFile.write(line+'\n')
                
        # Copy only the camera positions from the other files
        for inputPath in inputPaths[1:]:
            copying = False
            with open(inputPath, 'r') as inputFile:
                for line in inputFile:
                    if '<Placemark>' in line: # Start copying these lines
                        copying = True
                    if copying:
                        outputFile.write(line+'\n')
                    if '</Placemark>' in line: # Stop copying these lines
                        copying = False
        
        # Close off the output file
        outputFile.write('</Document>\n</kml>')

    print 'Finished writing ' + outputPath

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


