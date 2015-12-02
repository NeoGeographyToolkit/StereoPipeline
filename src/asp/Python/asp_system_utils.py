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
import os.path as P

def die(msg, code=-1):
    print >>sys.stderr, msg
    sys.exit(code)

def get_prog_version(prog):
    try:
        p = subprocess.Popen([prog,"--version"], stdout=subprocess.PIPE)
        out, err = p.communicate()
    except:
        raise Exception("Could not find: " + prog)
    if p.returncode != 0:
        raise Exception("Checking " + prog + " version caused errors")

    m = re.match("^.*? ([\d\.]+)", out)
    if not m:
        raise Exception("Could not find " + prog + " version")
    return m.group(1)

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
        raise Exception('Missing required executable "' + toolName + '", please add it to your PATH.')
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

def check_parallel_version():
    # This error will never be reached for users of our packaged final
    # product as that one bundles 'parallel' with it.
    ver = get_prog_version('parallel')
    if ver < '2013':
        die("Expecting a version of GNU parallel from at least 2013.")

def runInGnuParallel(numParallelProcesses, commandString, argumentFilePath, parallelArgs=[], nodeListPath=None, verbose=False):
    """Use GNU Parallel to spread task across multiple computers and processes"""

    # Make sure GNU parallel is installed
    if not checkIfToolExists('parallel'):
        raise Exception('Need GNU Parallel to distribute the jobs.')

    # Ensure our 'parallel' is not out of date
    check_parallel_version()

    # Use GNU parallel with given number of processes.
    # Let output be interspersed, read input series from file
    # Start in the same directory on remote machines. Ensure
    # that vital env variables are copied over.
    cmd = ['parallel',  '--workdir', os.getcwd(), '-u',
           '--env', 'PATH', '--env', 'PYTHONPATH', '--env', 'ISISROOT',
           '--env', 'ISIS3DATA', '-a', argumentFilePath]

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

# When user-exposed ASP executables are installed, they are in
# 'bin'. Otherwise, in dev mode, they are in the same dir as __file__.
# We prefer absolute paths below, in case some intermediate directories
# do not exist.
def bin_path(prog, **kw):
    currpath = kw.get('path', P.dirname(P.abspath(__file__)))
    binpath = os.path.abspath(P.join(currpath, '..', 'bin', prog))
    if not P.isfile(binpath):
        binpath = os.path.abspath(P.join(currpath, '..', 'Tools', prog))
    if not P.isfile(binpath):
        binpath = os.path.abspath(P.join(currpath, prog))
    return binpath

# When hidden ASP executables are installed, they are in
# 'libexec'. Otherwise, in dev mode, they are in the same dir as
# __file__.
def libexec_path(prog, **kw):
    currpath = kw.get('path', P.dirname(P.abspath(__file__)))
    libexecpath = os.path.abspath(P.join(currpath, '..', 'libexec', prog))
    if not P.isfile(libexecpath):
        libexecpath = os.path.abspath(P.join(currpath, '..', 'Tools', prog))
    if not P.isfile(libexecpath):
        libexecpath = os.path.abspath(P.join(currpath, prog))

    if not P.isfile(libexecpath):
        # Could not find prog in libexec either. We will come
        # here only for executables like gdalinfo that will
        # be packages in the release, but are not yet
        # in dev mode. Just print a warning and hope
        # this tool is somewhere in user's path.
        print("Could not find: " + libexecpath)
        libexecpath = which(prog)
        print("Using instead: " + libexecpath)

    return libexecpath

# Find if a program is in the path
def which(program):
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None

# mkdir without throw
def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError:
        pass

# Execute a given command stored in the libexec directory and parse
# its output. The output format is expected to be lines of
# comma-separated values.  The first value on each line becomes the
# output variable name, the other values are read into the array of
# variable values.
def run_and_parse_output(cmd, args, sep, verbose, **kw ):
    libexecpath = libexec_path(cmd)
    call = [libexecpath]
    call.extend(args)

    if verbose:
        print(" ".join(call))

    try:
        p = subprocess.Popen(call, stdout=subprocess.PIPE)
    except OSError, e:
        raise Exception('%s: %s' % (libexecpath, e))
    (stdout, stderr) = p.communicate()

    p.wait()
    if p.returncode != 0:
        print(stdout)
        print(stderr)
        raise Exception('Failed executing: ' + " ".join(call))
    data = {}
    if verbose:
        if stdout is not None: print(stdout)
        if stderr is not None: print(stderr)

    for line in stdout.split('\n'):

        # Print warning messages to stdout
        if re.match("^Warning", line): print(line)

        if sep in line:
            keywords = line.split(sep)
            for index, item in enumerate(keywords):
                # Strip whitespace from ends
                keywords[index] = item.strip(' \t\n\r')
            data[keywords[0]] = keywords[1:]

    return data
