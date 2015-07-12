#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

"""
General system related utilities
"""

import sys, os, re, shutil, subprocess, string, time, errno, multiprocessing

def get_num_cpus():
    """Return the number of CPUs on the current machine."""
    
    import sys
    if sys.version_info < (2, 6, 0):
        num_cpus = 8
    else:
        from multiprocessing import cpu_count
        num_cpus = cpu_count()

    return num_cpus


def checkIfToolExists(toolName):
    """Returns true if the system knows about the utility with this name (it is on the PATH)"""

    # Look for the tool using the 'which' command
    cmd = ['which', toolName]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    translateOut, err = p.communicate()
    

    # Check if that command failed to find the file
    failString = 'no ' + toolName + ' in ('
    if translateOut.find(failString) >= 0:
        raise Exception('Missing requested tool ' + toolName)
    else:
        return True


def getNumNodesInList(nodesListPath):
    """Get number of Pleiades nodes listed in a file"""

    if nodesListPath is None:
        return 1 # local machine

    # Count the number of nodes without repetition
    # (need this for Pleiades).
    nodes     = {}
    num_nodes = 0
    try:
        fileHandle = open(nodesListPath, "r")
        for line in fileHandle:
            if re.match('^\s*$', line): continue # skip empty lines
            matches = re.match('^\s*([^\s]*)', line)
            if matches:
                nodes[matches.group(1)] = 1

        num_nodes = len(nodes)
    except Exception, e: # Fail on exception
        die(e)
    if num_nodes == 0:
        raise Exception('The list of computing nodes is empty')

    return num_nodes



def runInGnuParallel(numParallelProcesses, commandString, argumentFilePath, parallelArgs=[], nodeListPath=None, verbose=False):
    """Use GNU Parallel to spread task across multiple computers and processes"""

    # Make sure GNU parallel is installed
    if not checkIfToolExists('parallel'):
        raise Exception('Need GNU Parallel to distribute the jobs.')

    # Use GNU parallel with given number of processes.
    # - Let output be interspersed, read input series from file
    cmd = ['parallel', '-u', '-a', argumentFilePath]   

    # Add number of processes if specified (default is one job per CPU core)
    if numParallelProcesses is not None:
        cmd += ['-P', str(numParallelProcesses)]

    # Add list of nodes as argument to the parallel tool if it is available
    if nodeListPath is not None: 
        cmd += ['--sshloginfile', nodeListPath]
        
    # Append any additional arguments to parallel
    cmd += parallelArgs

    # Append the actual command we want to call to the GNU Parallel call    
    cmd += [commandString]

    if verbose: # Echo the command line call we are about to make
        print(" ".join(cmd))

    returnCode = subprocess.call(cmd)
    return returnCode
















