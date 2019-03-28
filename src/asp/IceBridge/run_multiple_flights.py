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

# Script to run pleiades_manager multiple times with fixed arguments to fetch and process data.

import os, sys, argparse, time, threading
import traceback

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


def checkFileForFlight(path, site, date):
    '''Return True if the site/date pair is contained in the file'''
  
    with open(path, 'r') as f:
        for line in f:
            # Parse the line
            parts = line.split()
            if len(parts) != 2:
                print('Illegal input line is skipped: ' + line)
                continue
            thisSite = parts[0]
            thisDate = parts[1]

            if (thisSite == site) and (thisDate == date):
                return True

    return False



def processAndLog(command, logPath, line, deleteFolder=None):
    '''Run the command and log the result'''
    
    print(command)
    #os.system(command)
    time.sleep(2) # DEBUG
    
    # TODO: How to check for an error?
    
    with open(logPath, 'a') as f:
        f.write(line + '\n')

    print('Finished running: ' + command)
    
    if deleteFolder:
        print('Cleaning up folder: ' + deleteFolder)
        cleanup = 'rm -rf ' + deleteFolder
        #print(cleanup)
        #os.system(cleanup)


def main(argsIn):

    try:
        usage = '''label_images.py <options>'''

        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--input-file",  dest="inputFile", required=True,
                          help="Path to a file containing 'SITE DATE' pairs, one per line, to be run.")
        
        parser.add_argument("--success-file",  dest="successFile", required=True,
                          help="Log of completed flights.")
        
        parser.add_argument("--failure-file",  dest="failureFile", required=True,
                          help="Log of failed flights.")

        parser.add_argument("--limit",  dest="limit", default=4,
                          help="Don't process more than this many flights at once.")

        options = parser.parse_args(argsIn)

    except argparse.ArgumentError as msg:
        parser.error(msg)


    # This assumes that we already have the input data archived.
    FETCH_COMMAND = """python ~/repo/StereoPipeline/src/asp/IceBridge/pleiades_manager.py --node-type san --camera-calibration-folder /nobackup/smcmich1/icebridge/calib_files --reference-dem-folder  /nobackup/smcmich1/icebridge/reference_dems/  --skip-convert  --skip-archive-cameras  --skip-batch-gen  --skip-process  --skip-blend  --skip-ortho-gen  --skip-check-outputs  --skip-report  --skip-archive-aligned-cameras  --skip-archive-orthos  --skip-archive-summary  --skip-archive-run  --base-dir /nobackup/smcmich1/icebridge"""
    
    # This assumes that we already have the flight fetched.
    PROCESS_COMMAND = """python ~/repo/StereoPipeline/src/asp/IceBridge/pleiades_manager.py --node-type san --camera-calibration-folder /nobackup/smcmich1/icebridge/calib_files --reference-dem-folder  /nobackup/smcmich1/icebridge/reference_dems/ --skip-check-inputs  --skip-fetch  --skip-convert  --skip-archive-cameras  --skip-batch-gen  --skip-process  --skip-blend  --skip-ortho-gen  --skip-check-outputs  --skip-report  --skip-archive-aligned-cameras  --skip-archive-orthos  --skip-archive-summary  --skip-archive-run  --skip-validate  --base-dir /nobackup/smcmich1/icebridge --generate-labels --archive-labels"""


    # Build up the list of flights to process
    tasks = []
    print(options.inputFile)
    with open(options.inputFile, 'r') as f:
        for line in f:
            # Parse the line
            parts = line.split()
            if len(parts) != 2:
                print('ERROR: Illegal input line is skipped: ' + line)
                continue
            site = parts[0]
            date = parts[1]

            # See if we already processed this flight
            if ( checkFileForFlight(options.successFile, site, date) or
                 checkFileForFlight(options.failureFile, site, date)   ):
                print('This flight was already processed, skipping...')
                continue

            tasks.append((site, date))

    print('Finished creating the flight list.')


    # Loop through all of the flights to process them
    numProcessed  = 0
    lastPair      = None
    fetchThread   = None
    processThread = None
    for (site, date) in tasks:

        idString = (' --site %s --yyyymmdd %s ' % (site, date))

        fetchCmd   = FETCH_COMMAND   + idString
        processCmd = PROCESS_COMMAND + idString

        # Launch the fetch job for this flight
        # TODO: Log to both files
        logLine = site+' '+date
        print('Launching FETCH job for ' + logLine)
        fetchThread = threading.Thread(target=processAndLog, args=(fetchCmd, options.successFile, logLine))
        fetchThread.start()


        # Launch the process job for the previous flight
        if lastPair:
            if processThread:
                print('Waiting on the last processing job to complete...')
                processThread.join()
          
            logLine = lastPair[0]+' '+lastPair[1]
            print('Launching PROCESS job for ' + logLine)
            folder  = '/nobackup/smcmich1/icebridge/data/'+lastPair[0]+'_'+lastPair[1]
            processThread = threading.Thread(target=processAndLog, args=(processCmd, options.successFile, logLine, None))
            processThread.start()

        if fetchThread:
            print('Waiting on the last fetch job to complete...')
            fetchThread.join()

        # This pair was fetched this iteration, will be processed next iteration.
        lastPair = (site, date)

        numProcessed += 1
        if numProcessed >= options.limit:
            print('Hit the limit of processed flights!')
            break


    # Process the data from the last fetch
    if lastPair:
        if processThread:
            print('Waiting on the last processing job to complete...')
            processThread.join()
      
        logLine = lastPair[0]+' '+lastPair[1]
        print('Launching PROCESS job for ' + logLine)
        folder  = '/nobackup/smcmich1/icebridge/data/'+lastPair[0]+'_'+lastPair[1]
        processThread = threading.Thread(target=processAndLog, args=(processCmd, options.successFile, logLine, None))
        processThread.start()

    # Make sure everything is finished
    if fetchThread:
        print('Waiting on the last fetch job to complete...')
        fetchThread.join()
    if processThread:
        print('Waiting on the last processing job to complete...')
        processThread.join()

    print('Jobs finished.')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


