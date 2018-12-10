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

# Program to generate labels for multiple flights, one after the other.

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing
import re, shutil, glob

import os.path as P

# The path to the ASP python files and tools
basepath      = os.path.dirname(os.path.realpath(__file__)) # won't change, unlike syspath
pythonpath    = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath   = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
icebridgepath = os.path.abspath(basepath + '/../IceBridge')  # IceBridge tools

# Prepend to Python path
sys.path.insert(0, basepath)
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)
sys.path.insert(0, icebridgepath)

import icebridge_common, run_helper

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

#=========================================================================
# Functions


def updateLogFile(path, run):
    '''Add an run to the log file if it is not already there'''
    if checkLogFile(date):
        return

    with open(path, 'a') as f:
        f.write(run.name() + '\n')

def checkLogFile(path, run):
    '''Return true if the date is in the file'''
    run_name = run.name()
    with open(path, 'r') as f:
        for line in f:
            if run_name in line:
              return True
    return False


#=========================================================================
# Main

def main(argsIn):

    # Parse the input arguments
    if len(argsIn) < 2:
        print 'Usage: multi_flight_runner <input_flight_list> <finished_flight_list>'
        return 0

    inputFlightLog  = argsIn[0]
    outputFlightLog = argsIn[1]

    # Stop processing early if we build up too many flights!
    MAX_RUNS_RETAINED = 3

    runsRetained = []
    runsDeleted  = []
    with open(inputFlightLog, 'r') as inLog:

        for line in inLog:

            # Make run object and check if we already did this run
            parts = line.split('_')
            run   = RunHelper(parts[0], parts[1])

            if checkLogFile(outputFlightLog, run):
                print 'Skipping already completed run: ' + str(run)
                continue

            print 'Going to process run: ' + str(run)

            runFolder = os.path.join('/nobackup/smcmich1/icebridge/', str(run))

            # Set up the processing command
            # - This will generate all the labels for the current flight and then wipe everything
            #   if all of the label files were generated.

            TOOL_PATH = 'python ~/repo/StereoPipeline/src/asp/IceBridge/pleiades_manager.py'

            cmd = TOOL_PATH + '  --base-dir  /nobackup/smcmich1/icebridge/ --node-type san --camera-calibration-folder /nobackup/smcmich1/icebridge/calib_files/ --reference-dem-folder /nobackup/smcmich1/icebridge/reference_dems/ --bundle-length 2 --simple-cameras --skip-archive-cameras  --skip-archive-aligned-cameras  --skip-archive-orthos  --skip-archive-summary  --skip-archive-run --skip-ortho-gen --skip-check-outputs --skip-report  --skip-process --skip-blend --skip-convert  --skip-validate --generate-labels   --archive-labels --wipe-all'

            cmd += ' --site ' + run.site + ' --yyyymmdd ' + run.yyyymmdd

            print cmd
            os.system(cmd)

            # Whether or not we succeeded, log that we processed the flight.
            updateLogFile(outputFlightLog, run)

            if not os.path.exists(runFolder):
                print 'Run successful, deleted!'
                runsDeleted.append(run.name())
            else:
                print 'Run failed, retained!''
                runsRetained.append(run.name())

            if len(runsRetained) >= MAX_RUNS_RETAINED:
                print 'Too many flights failed, quitting now!' 
                break


    numRuns = len(runsRetained) + len(runsDeleted)
    print '---=== Finished processing ' + str(numRuns) + ' flights! ===---'


    # Send an email with a record of the runs we processed

    emailAddress = getEmailAddress(icebridge_common.getUser())
    print("Sending email to: " + emailAddress)
    subject = 'Finished running batch flight labelling script!'
    body    = '\nThe following runs were retained (some files missing):'
    for r in runsRetained:
        body += r + '\n'
    body    = '\nThe following runs were deleted (all files created):'
    for r in runsDeleted:
        body += r + '\n'

    sendEmail(emailAddress, subject, body)





# Run main function if file used from shell
if __name__ == "__main__":
sys.exit(main(sys.argv[1:]))



