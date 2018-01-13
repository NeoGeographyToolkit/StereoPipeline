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

# Push DEMs and orthos to NSIDC.
# Usage: /usr/bin/python push_to_nsidc.py --yyyymmdd 20140321 --site GR --camera-calibration-folder camera_calib --reference-dem-folder reference_dems --login-info 'user,password server.nsidc.org'

# If there is more than one archive, push the one with the latest
# modification time.

# TODO: How to prevent old copies littering the remote directory?

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

import icebridge_common, archive_functions, run_helper
import asp_system_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

g_start_time = -1
g_stop_time  = -1
def start_time():
    global g_start_time, g_stop_time
    g_start_time = time.time()
def stop_time(job, logger):
    global g_start_time, g_stop_time
    g_stop_time = time.time()
    wall_s = float(g_stop_time - g_start_time)/3600.0
    logger.info( ("Elapsed time for %s is %g hours." % (job, wall_s) ) )
    
def fetchIndices(options, logger):
    '''Fetch the csv indices of available files.'''
    logger.info("Fetch indices from NSIDC.")
    pythonPath = asp_system_utils.which('python')
    cmd = ( (pythonPath + ' ' + icebridge_common.fullPath('full_processing_script.py') + \
             ' --camera-calibration-folder %s --reference-dem-folder %s --site %s ' + \
             '--yyyymmdd %s --stop-after-index-fetch --no-nav ' ) % \
            (options.inputCalFolder, options.refDemFolder, options.site, options.yyyymmdd))
    logger.info(cmd)
    start_time()
    os.system(cmd)
    stop_time("fetch index", logger)
    
def pushByType(run, options, logger, dataType):
               
    # Fetch the ortho index from NSIDC if missing
    outputFolder   = run.getFolder()
    logger.info("Output folder is " + outputFolder)
    os.system("mkdir -p " + outputFolder)
    
    # Current directory. It is important to go from /u to the real dir which is /nobackup...
    unpackDir = os.path.realpath(os.getcwd())
    logger.info("Unpack directory is " + unpackDir)
    
    orthoFolder    = icebridge_common.getOrthoFolder(outputFolder)
    orthoIndexPath = icebridge_common.csvIndexFile(orthoFolder)
    if not os.path.exists(orthoIndexPath):
        fetchIndices(options, logger)
        
    logger.info("Reading ortho index: " + orthoIndexPath)
    (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath)

    # Fetch unarchived folder if missing
    if dataType == 'DEM':
        unarchivedFolder = run.getAssemblyFolder()
    elif dataType == 'ORTHO':
        unarchivedFolder = run.getProcessFolder()
    else:
        raise Exception("Unknown data type: " + dataType)
    logger.info("Unarchived data folder is " + unarchivedFolder)
    
    # Especially for ortho, force-fetch each time, as there is no good way
    # of checking if we fetched well before.
    start_time()
    if not archive_functions.fetchProcessedByType(run, unpackDir, logger, dataType):
        return
    stop_time("fetching archived data by type: " + dataType, logger)

    # Make the output directory at NSIDC
    m = re.match("(\d\d\d\d)(\d\d)(\d\d)", options.yyyymmdd)
    if m:
        outDir = options.site + "_" + m.group(1) + "." + m.group(2) + "." + m.group(3)
    else:
        raise Exception("Could not parse: " + options.yyyymmdd)

    # Keep the output directory locally here
    localDirPath = os.path.join(outputFolder, dataType, outDir)
    os.system("mkdir -p " + localDirPath)

    logger.info("Storing the renamed " + dataType + " files in " + localDirPath)
    logger.info("Directory name at NSIDC: " + outDir)

    # Read the DEMs and orthos, and copy them to outDir according to the final convention
    if dataType == 'DEM':
        dataFiles = icebridge_common.getTifs(unarchivedFolder, prependFolder=True)
    else:
        dataFiles = glob.glob(os.path.join(unarchivedFolder, 'batch_*', 'out-ortho.tif'))
        
    for dataFile in dataFiles:

        # Here we use the convention from archive_functions.py for DEMs and from how we store orthos.
        if dataType == 'DEM':
            m = re.match("^.*?" + unarchivedFolder + "/F_(\d+)_\d+_" + dataType + \
                         "\.tif$", dataFile)
            if not m:
                continue
            frameNumber = int(m.group(1))
        else:
            m = re.match("^.*?" + unarchivedFolder + "/batch_(\d+)_\d+_\d+/" + \
                         "out-ortho.tif$", dataFile)
            if not m:
                continue
            frameNumber = int(m.group(1))
            
        if frameNumber < options.startFrame or frameNumber > options.stopFrame:
            continue

        # For each data file, copy from the ortho its meta info
        if not frameNumber in orthoFrameDict.keys():
            # Bugfix: Ortho fetching failed, try again
            fetchIndices(options, logger)
            logger.info("Re-reading ortho index: " + orthoIndexPath)
            (orthoFrameDict, orthoUrlDict) = icebridge_common.readIndexFile(orthoIndexPath)
            if not frameNumber in orthoFrameDict.keys():
                # This time there is nothing we can do
                raise Exception("Cannot find ortho for frame: " + str(frameNumber))
            
        orthoFile = orthoFrameDict[frameNumber]
        [dateString, timeString] = icebridge_common.parseTimeStamps(orthoFile)

        # It is always possible that the ortho file date will be the next day
        # after the current flight date, if the flight goes after midnight.
        # So it is not unreasonable that options.yyyymmdd != dateString.

        if dataType == 'DEM':
            outFile = ('IODEM3_%s_%s_%05d_DEM.tif' % (dateString, timeString, frameNumber))
        else:
            # TODO: Need to think more of the naming convention.
            outFile = ('IODEM3_%s_%s_%05d_ORTHO.tif' % (dateString, timeString, frameNumber))
            
        cmd = "/bin/cp -fv " + dataFile + " " + os.path.join(localDirPath, outFile)
        logger.info(cmd)
        os.system(cmd)

    # Push the directory to NSIDC
    remoteDirPath = os.path.join(os.path.basename(os.path.dirname(localDirPath)),
                           os.path.basename(localDirPath))
    remoteDirPath = os.path.join('/incoming', 'Ames', remoteDirPath)
    logger.info("Storing at NSIDC in: " + remoteDirPath)

    cmd = 'lftp -e "mirror -P 20 -c -R -vvv --delete --delete-first ' + localDirPath + \
          ' ' + remoteDirPath + ' -i \'\.(tif)$\'; bye\" -u ' + options.loginInfo
    logger.info(cmd)
    start_time()

    (output, err, status) = asp_system_utils.executeCommand(cmd,
                                                            suppressOutput = True)
    #status = os.system(cmd)
    logger.info("LFTP output and error: " + output + ' ' + err)
    logger.info("LFTP status: " + str(status))
    #if status != 0:
    #    raise Exception("Problem pushing")
    
    stop_time("push to NSIDC", logger)

def main(argsIn):

    try:
        usage = '''usage: push_to_nsidc.py <options> '''
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", default="",
                            help="Specify the year, month, and day in one YYYYMMDD string.")

        parser.add_argument("--site",  dest="site", default = "",
                            help="Name of the location of the images (AN, GR, or AL)")
                            
        parser.add_argument("--site_yyyymmdd",  dest="site_yyyymmdd", default = "",
                            help="A value like GR_20150330, which will be split into site and yyyymmdd by this script.")

        parser.add_argument('--start-frame', dest='startFrame', type=int,
                            default=icebridge_common.getSmallestFrame(),
                            help="Frame to start with.  Leave this and stop-frame blank to " + \
                            "process all frames.")
        
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                            default=icebridge_common.getLargestFrame(),
                            help='Frame to stop on.')
        parser.add_argument("--camera-calibration-folder",  dest="inputCalFolder", default=None,
                          help="The folder containing camera calibration.")
        
        parser.add_argument("--reference-dem-folder",  dest="refDemFolder", default=None,
                          help="The folder containing DEMs that created orthoimages.")

        parser.add_argument("--login-info",  dest="loginInfo", default=None,
                            help="user,password destination.nsidc.org.")

        parser.add_argument("--done-file",  dest="doneFile", default=None,
                            help="List of runs that were done by now.")

        options = parser.parse_args(argsIn)

    except argparse.ArgumentError, msg:
        parser.error(msg)

    # parse --site_yyyymmdd. Sometimes it is easier to pass this than
    # to pass separately --site and --yyyymmdd.
    m = re.match('^(\w+)_(\w+)', options.site_yyyymmdd)
    if m:
        options.site = m.group(1)
        options.yyyymmdd = m.group(2)
    else:
        options.site_yyyymmdd = options.site + "_" + options.yyyymmdd
        
    # Read the done file and exit if the current flight is done
    done = set()
    if options.doneFile != "":
        with open(options.doneFile, 'r') as f:
            for val in f:
                val = val.strip()
                done.add(val)
    if options.site_yyyymmdd in done:
        print("Skipping done flight: " + options.site_yyyymmdd)
        return 0
    
    run = run_helper.RunHelper(options.site, options.yyyymmdd, os.getcwd())

    # Set up logging in the run directory. Log outside of the run dir,
    # as that one we will wipe
    logFolder = os.path.abspath(os.path.join(run.getFolder(), '..', 'push_logs'))
    os.system('mkdir -p ' + logFolder)
    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(logFolder, logLevel, "push")
    logger.info("Logging in: " + logFolder)

    # Check the lftp version. On some machines it is too old. 
    (out, err, status) = asp_system_utils.executeCommand(['lftp', '--version'],
                                                         suppressOutput = True)
    m = re.match('^.*?LFTP\s+\|\s+Version\s+4.5', out)
    if not m:
        raise Exception('Expecting LFTP version 4.5.')
    else:
        logger.info("Found an acceptable version of LFTP.")
    
    pushByType(run, options, logger, 'DEM')
    #pushByType(run, options, logger, 'ORTHO') # need to wait for format decision

    # Wipe at the end
    cmd = "rm -rf " + run.getFolder()
    logger.info(cmd)
    os.system(cmd)
    
# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
