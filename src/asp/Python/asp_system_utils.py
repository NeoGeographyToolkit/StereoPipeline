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
from __future__ import print_function
import sys, os, re, shutil, subprocess, string, time, errno, multiprocessing, signal
import os.path as P
import asp_string_utils, asp_cmd_utils

def die(msg, code=-1):
    '''Exit the program with a message'''
    print(msg, file=sys.stderr)
    sys.exit(code)

def verify_python_version_is_supported():
    '''Verifies that a supported version of Python is being used.'''
    
    if sys.version_info < (2, 6, 0):
        print('\nERROR: Must use Python version >= 2.6.')
        sys.exit(1)

def get_prog_version(prog):
    '''Get the version of a command line program.'''
    try:
        p = subprocess.Popen([prog,"--version"], stdout=subprocess.PIPE, universal_newlines=True)
        out, err = p.communicate()
    except:
        raise Exception("Could not find: " + prog)
    if p.returncode != 0:
        raise Exception("Checking " + prog + " version caused errors")

    # This is a fix for sometimes GNU Parallel printing a warning at the beginning
    for line in out.split("\n"):
        m = re.match("^.*?warning", line, re.IGNORECASE)
        if m: continue
        m = re.match("^.*? ([\d\.]+)", line)
        if not m:
           raise Exception("Could not find " + prog + " version")
        return m.group(1)
    return ""

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
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         universal_newlines=True)
    translateOut, err = p.communicate()

    # Check if that command failed to find the file
    failString = 'no ' + toolName + ' in ('
    if translateOut.find(failString) >= 0:
        raise Exception('Missing required executable "' + toolName + '", please add it to your PATH.')
    else:
        return True

# Find if a program is in the path.
# Some ASP tools like qi2txt can be only in libexec, hence the option lookInLibexec.
# TODO: This logic needs serious cleanup, but while making sure that nothing breaks.
def which(program, lookInLibexec = False):

    if not lookInLibexec:
        checkIfToolExists(program)
    
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        paths = []
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            paths.append(path)

        for path in paths:
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

        if lookInLibexec:
            for path in paths:
                exe_file = os.path.join(os.path.dirname(path), 'libexec', program)
                if is_exe(exe_file):
                    return exe_file
            
    raise Exception('Missing required executable "' + program + '", please add it to your PATH.')

    return None

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
    except Exception as e: # Fail on exception
        die(e)
    if num_nodes == 0:
        raise Exception('The list of computing nodes is empty')

    return num_nodes

def check_parallel_version():
    # This error will never be reached for users of our packaged final
    # product as that one bundles 'parallel' with it.
    ver = get_prog_version('parallel')
    if ver < '20170722':
        die("Expecting a version of GNU parallel >= 20170722.")

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
    cmd = ['parallel',  '--will-cite', '--workdir', os.getcwd(), '-u',
           '--env', 'PATH', '--env', 'PYTHONPATH', '--env', 'ISISROOT',
           '--env', 'ISISDATA', '-a', argumentFilePath]

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
        if libexecpath is None:
            raise Exception('Could not find: ' + prog)
        print("Using instead: " + libexecpath)

    return libexecpath

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
def run_and_parse_output(cmd, args, sep, verbose, return_full_lines = False, **kw ):
    libexecpath = libexec_path(cmd)
    call = [libexecpath]
    call.extend(args)

    if verbose:
        print (asp_string_utils.argListToString(call))

    try:
        p = subprocess.Popen(call, stdout=subprocess.PIPE, universal_newlines=True)
    except OSError as e:
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

    count = 0
    for line in stdout.split('\n'):

        # Print warning messages to stdout
        if re.match("^Warning", line): print(line)

        if return_full_lines:
            data[count] = line # return the entire line
            count += 1
        else:
            if sep in line:
                keywords = line.split(sep)
                for index, item in enumerate(keywords):
                    # Strip whitespace from ends
                    keywords[index] = item.strip(' \t\n\r')
                data[keywords[0]] = keywords[1:]

    return data

def run_with_return_code(cmd, verbose=False):
    # TODO: Wipe this and use instead executeCommand.
    if verbose:
        print (asp_string_utils.argListToString(cmd))

    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
    except OSError as e:
        print('Error: %s: %s' % ( asp_string_utils.argListToString(cmd), e))
        
    (stdout, stderr) = p.communicate()
    p.wait()
    
    if p.returncode != 0 and verbose:
        if stdout is not None: print(stdout)
        if stderr is not None: print(stderr)
        print ('Failed executing: ' + " ".join(cmd))

    return p.returncode

# For timeout
def timeout_alarm_handler(signum, frame):
    raise Exception("Timeout reached!")

# TODO: Improve this function a bit
def executeCommand(cmd,
                   outputPath=None,      # If given, throw if the file is not created.  Don't run if it already exists.
                   suppressOutput=False, # If true, don't print anything!
                   redo=False,           # If true, run even if outputPath already exists.
                   noThrow=False,        # If true, don't throw if output is missing
                   numAttempts = 1,      # How many attempts to use
                   sleepTime = 60,       # How much to sleep between attempts
                   timeout   = -1        # After how long to timeout in seconds
                   ):
    '''Executes a command with multiple options'''

    
    # Initialize outputs
    out    = ""
    status = 0
    err    = ""
    
    if cmd == '': # An empty task
        return (out, err, status)

    # Convert the input to list format if needed
    if not asp_string_utils.isNotString(cmd):
        cmd = asp_string_utils.stringToArgList(cmd)
    
    for attempt in range(numAttempts):
        
        # Run the command if conditions are met
        if redo or (outputPath is None) or (not os.path.exists(outputPath)):

            if not suppressOutput:
                print (asp_string_utils.argListToString(cmd))

            if timeout > 0:
                print("Will enforce timeout of " + str(timeout) + " seconds.")
                signal.signal(signal.SIGALRM, timeout_alarm_handler)
                signal.alarm(timeout)
                    
            try:
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     universal_newlines=True)
                out, err = p.communicate()
                status = p.returncode
                if timeout > 0:
                    signal.alarm(0)  # reset the alarm
            except Exception as e:
                out = ""
                err = ('Error: %s: %s' % (asp_string_utils.argListToString(cmd), e))
                if timeout > 0:
                    # this module is generally not available, so this use is very niche
                    import psutil
                    def kill_proc_tree(pid, including_parent=True):    
                        parent = psutil.Process(pid)
                        for child in parent.children(recursive=True):
                            print("Killing: " + str(child))
                            child.kill()
                        if including_parent:
                            print("Killing: " + str(parent))
                            parent.kill()
                    pid = psutil.Process(p.pid)
                    try:
                        kill_proc_tree(p.pid)
                    except:
                        pass
                    
                status = 1
                if not noThrow:
                    raise Exception(err)

            if out is None: out = ""
            if err is None: err = ""

            if not suppressOutput:
                print (out + '\n' + err)

            if status == 0:
                break

            if numAttempts <= 1:
                break

            if attempt < numAttempts - 1:
                print("attempt: " + str(attempt))
                print("ran: " + asp_string_utils.argListToString(cmd) )
                print("out = " + out)
                print("err = " + err)
                print("status = " + str(status))
                print("Will sleep for " + str(sleepTime) + " seconds")

                time.sleep(sleepTime)
        
        else: # Output file already exists, don't re-run
            out    = ""
            err    = ""
            status = 0

        # Optionally check that the output file was created
        if outputPath and (not os.path.exists(outputPath)) and (not noThrow):
            raise asp_cmd_utils.CmdRunException('Failed to create output file: ' + outputPath)

    return (out, err, status)
    
    
    
