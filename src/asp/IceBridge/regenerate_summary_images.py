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

# Program to generate a new version of a SUMMARY tarball with any missing
#  DEM and ORTHO browse images replaced.

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing
import traceback
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

import icebridge_common, pbs_functions, archive_functions, run_helper
import asp_system_utils, generate_flight_summary

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

#=========================================================================
# Parameters

# Constants used in this file

PBS_QUEUE   = 'normal'

GROUP_ID = 's1827'

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


PFE_NODES = ['san', 'ivy', 'has', 'bro']

#=========================================================================

# 'wes' = Westmere = 12 cores/24 processors, 48 GB mem, SBU 1.0, Launch from mfe1 only!
# 'san' = Sandy bridge = 16 cores,  32 GB mem, SBU 1.82
# 'ivy' = Ivy bridge   = 20 cores,  64 GB mem, SBU 2.52
# 'has' = Haswell      = 24 cores, 128 GB mem, SBU 3.34
# 'bro' = Broadwell    = 28 cores, 128 GB mem, SBU 4.04

def getParallelParams(nodeType):
    '''Return (numProcesses, numThreads, tasksPerJob, maxHours) for running a certain task on a certain node type'''

    if nodeType == 'san': return (16, 1, 600, 2)
    if nodeType == 'ivy': return (20, 1, 700, 2)
    if nodeType == 'has': return (24, 1, 800, 2)
    if nodeType == 'bro': return (28, 1, 900, 2)
    if nodeType == 'wes': return (12, 1, 400, 2)

    raise Exception('No params defined for node type ' + nodeType + ', task = ' + task)

#=========================================================================

def getEmailAddress(userName):
    '''Return the email address to use for a user'''
    
    if userName == 'smcmich1':
        return 'scott.t.mcmichael@nasa.gov'
    if userName == 'oalexan1':
        return 'oleg.alexandrov@nasa.gov'

def sendEmail(address, subject, body):
    '''Send a simple email from the command line'''
    # Remove any quotes, as that confuses the command line.
    subject = subject.replace("\"", "")
    body    = body.replace("\"", "")
    try:
        cmd = 'mail -s "' + subject + '" ' + address + ' <<< "' + body + '"'
        #print(cmd) # too verbose to print
        os.system(cmd)
    except Exception as e:
        print("Could not send mail.")
        
#---------------------------------------------------------------------


def submitBatchJobs(commandFileList, options, pbsLogFolder, run, logger):
    '''Read all the batch jobs required for a run and distribute them across job submissions.
       Returns the common string in the job names.'''

    # Retrieve parallel processing parameters
    (numProcesses, numThreads, tasksPerJob, maxHours) = getParallelParams(options.nodeType)

    numBatches = len(commandFileList)
    logger.info( ("Num batches: %d, tasks per job: %d" % (numBatches, tasksPerJob) ) )

    baseName = run.shortName() # SITE + YYMMDD = 8 chars, leaves seven for frame digits.

    # Call the tool which just executes commands from a file
    scriptPath = icebridge_common.fullPath('multi_process_command_runner.py')

    index  = 0
    jobIDs = []
    for commandFile in commandFileList:

        jobName = ('%s%05d%s' % ('RS', index, baseName) )

        # Specify the range of lines in the file we want this node to execute
        args = ('--command-file-path %s --start-frame -1 --stop-frame -1 --num-processes %d' % \
                (commandFile, numProcesses))

        logPrefix = os.path.join(pbsLogFolder, 'batch_' + jobName)
        logger.info('Submitting summary regen job: ' + scriptPath + ' ' + args)

        BATCH_PBS_QUEUE = 'normal'
        jobID = pbs_functions.submitJob(jobName, BATCH_PBS_QUEUE, maxHours, logger,
                                        options.minutesInDevelQueue,
                                        GROUP_ID,
                                        options.nodeType, '~/repo/python_env/bin/python',
                                        scriptPath + ' ' + args, logPrefix)
        jobIDs.append(jobID)
        index += 1

    # Waiting on these jobs happens outside this function
    return (baseName, jobIDs)

def checkRequiredTools():
    '''Verify that we have all the tools we will be calling during the script.'''

    tools  = ['gdal_translate']
    
    for tool in tools:
        asp_system_utils.checkIfToolExists(tool)

def fetchTarball(louPath, localPath):
    '''Retrieve and unpack the desired tarball file from Lou'''

    if os.path.exists(localPath):
        return # Currently not checking that everything is there!

    cmd = 'shiftc --wait -d -r --verify --extract-tar lfe:' + louPath + ' ' + localPath
    print cmd
    os.system(cmd)

    if not os.path.exists(localPath):
        raise Exception('Failed to retrieve tarball: lfe:' + louPath)


def getSummaryFileCommand(inFile, outFile, isOrtho):
   '''Get a one line command to generate a single summary file'''

   if isOrtho:
       cmd = 'gdal_translate -scale -outsize 25% 25% -of jpeg ' + inFile +' '+ outFile
   else:
       # Hillshade then downsample.
       tempPath = outFile + '_temp.tif'
       cmd = 'hillshade ' + inFile +' -o ' + tempPath
       cmd += (' && gdal_translate '+ tempPath +' '+ outFile+
              ' -of GTiff -outsize 40% 40% -b 1 -co "COMPRESS=JPEG"')
       cmd += ' && rm ' + tempPath

   return cmd

def descendIfNeeded(folder):
    '''If the input folder contains only a single folder, return
       that contained folder.  Otherwise return the input folder.'''

    files = os.listdir(folder)
    if (len(files) == 1) and os.path.isdir(os.path.join(folder,files[0])):
        return os.path.join(folder, files[0])
    else:
        return folder

# TODO: Move to a shared file!
def getUnpackedFiles(folder):
    '''Find all the DEM or ORTHO files unpacked from their tarball.'''

    # Dig down through the initial unpack folder
    top_folder = descendIfNeeded(folder)

    # Dig down through a subsequent unpack folder
    possible_directories = ['tarAssembly', 'processed', 'camera', 'summary']
    file_dir   = []
    for d in possible_directories:
        test_dir = os.path.join(top_folder, d)
        if os.path.exists(test_dir):
            file_dir = test_dir
            break
        test_dir = os.path.join(top_folder, d)
        if os.path.exists(test_dir):
            file_dir = test_dir
            break

    # Handle case where each file is buried in a subfolder
    final_files = []
    file_list = os.listdir(file_dir)
    for f in file_list:
        full_path = os.path.join(file_dir, f)
        if os.path.isdir(full_path):
            subfiles = os.listdir(full_path)
            final_files.append(os.path.join(full_path, subfiles[0]))
        else:
            final_files.append(full_path)

    return final_files

def getMissingSummaryFiles(folder, summaryFolder, isOrtho):
    '''Return a list of input/output pairs of missing summary files.'''

    print 'Looking for missing summary files for folder: ' + folder

    inFiles      = getUnpackedFiles(folder)
    summaryFiles = os.listdir(summaryFolder)

    # Handle DEM and ORTHO browse images separately.
    keyword = 'dem'
    if isOrtho:
        keyword = 'ortho'

    # Associate each summary file with its frame number
    summaryFrames = {}
    for c in summaryFiles:
        if keyword not in c:
            continue

        # Inputs like: dem_01058_01060_browse.tif
        parts = os.path.basename(c).split('_')
        frame = int(parts[1])
        summaryFrames[frame] = c

    # See if any input files are missing their summary frame
    missingSummaryList = []
    for f in inFiles:
        # Skip metadata files
        ext = os.path.splitext(f)[1]
        if ext != '.tif':
            continue

        # TODO: This may break later!
        fname = os.path.basename(f)
        if fname[0] == 'F':
            parts      = fname.split('_')
            frameStart = int(parts[1])
            frameEnd   = int(parts[2])
        else: # AN_20141116/processed/batch_05962_05963_2/out-ortho.tif
            parts = os.path.dirname(f).split('_')
            frameStart = int(parts[-3])
            frameEnd   = int(parts[-2])

        try:
            summary = summaryFrames[frameStart]
            continue # Summary file already exists.

        except KeyError:
            # Get the desired summary file name
            if isOrtho:
                summaryName = ('ortho_%05d_%05d_browse.tif' % (frameStart, frameEnd))
            else: # DEM
                summaryName = ('dem_%05d_%05d_browse.tif' % (frameStart, frameEnd))

            summaryPath = os.path.join(summaryFolder, summaryName)
            missingSummaryList.append((f, summaryPath))

    return missingSummaryList

def writeCommandFiles(missingDemFiles, missingOrthoFiles, outputPrefix, chunkSize):
    '''Generate conversion commands for each missing summary file and
       divide them up into files of a fixed length.  Returns the list of
       command files that were written.'''

    if (not missingDemFiles) and (not missingOrthoFiles):
        return []

    # Open the first file
    fileCounter = 0
    cmdCounter  = 0
    fname   = outputPrefix + str(fileCounter) + '.txt'
    handle  = open(fname, 'w')

    commandFiles = [fname]

    # Add all the DEM commands
    for p in missingDemFiles:

        # If the file is too large, start a new file.
        if cmdCounter >= chunkSize:
            fileCounter += 1
            cmdCounter = 0
            handle.close()
            fname   = outputPrefix + str(fileCounter) + '.txt'
            handle  = open(fname, 'w')
            commandFiles.append(fname)

        # Generate command, then write to file.
        cmd = getSummaryFileCommand(p[0], p[1], isOrtho=False)
        handle.write(cmd + '\n')
        cmdCounter += 1
 
    # Add all the ORTHO commands
    for p in missingOrthoFiles:

        if cmdCounter >= chunkSize:
            fileCounter += 1
            cmdCounter = 0
            handle.close()
            fname   = outputPrefix + str(fileCounter) + '.txt'
            handle  = open(fname, 'w')
            commandFiles.append(fname)

        cmd = getSummaryFileCommand(p[0], p[1], isOrtho=True)
        handle.write(cmd + '\n')
        cmdCounter += 1

    handle.close()

    print 'Created ' + str(len(commandFiles)) + ' command files.'

    return commandFiles

def main(argsIn):

    try:
        usage = '''usage: regenerate_summary_images.py <options> '''
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--dem-tarball",  dest="demTarball", default=os.getcwd(),
                            help="Where all the inputs and outputs are stored.")

        parser.add_argument("--ortho-tarball",  dest="orthoTarball", default=None,
                            help="Where to unpack the data.")

        parser.add_argument("--summary-tarball",  dest="summaryTarball", default=None,
                            help="Where to unpack the data.")

        parser.add_argument("--unpack-dir",  dest="unpackDir", default=None,
                            help="Where to unpack the data.")

        parser.add_argument("--node-type",  dest="nodeType", default='san',
                            help="Node type to use (wes[mfe], san, ivy, has, bro)")

        parser.add_argument("--skip-archive-summary", action="store_true",
                            dest="skipArchiveSummary", default=False,
                            help="Skip archiving the summary.")

        # Debug option
        parser.add_argument('--minutes-in-devel-queue', dest='minutesInDevelQueue', type=int,
                            default=0,
                            help="If positive, submit to the devel queue for this many minutes.")

        options = parser.parse_args(argsIn)

    except argparse.ArgumentError as msg:
        parser.error(msg)

    # Check if we are on the right machine
    (host, err, status) = asp_system_utils.executeCommand(['uname', '-n'],
                                                         suppressOutput = True)
    host = host.strip()
    if 'pfe' in host and options.nodeType not in PFE_NODES:
        raise Exception("From machine " + host + " can only launch on: " + " ".join(PFE_NODES)) 
    if 'mfe' in host and options.nodeType != 'wes':
        raise Exception("From machine " + host + " can only launch on: wes")

    # Make sure our paths will work when called from PBS
    options.unpackDir = os.path.abspath(options.unpackDir)

    os.system('mkdir -p ' + options.unpackDir)

    # Figure out the run name
    parts = options.summaryTarball.split('_')
    if 'GR' in parts:
        index = parts.index('GR')
    else:
        index = parts.index('AN')
    site     = parts[index  ]
    yyyymmdd = parts[index+1]

    # TODO: Check folders!
    run = run_helper.RunHelper(site, yyyymmdd, options.unpackDir)

    runFolder = os.path.join(options.unpackDir, str(run))
    os.system('mkdir -p ' + runFolder)

    logFolder = os.path.join(runFolder, 'logs')

    # Set up logging in the run directory
    os.system('mkdir -p ' + logFolder)
    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(logFolder, logLevel,
                                            icebridge_common.manager_log_prefix())
    logger.info("Logging in: " + logFolder)
    
    
    checkRequiredTools() # Make sure all the needed tools can be found before we start

    logger.info("Disabling core dumps.") # these just take a lot of room
    os.system("ulimit -c 0")
    os.system("umask 022") # enforce files be readable by others

    # See how many hours we used so far. I think this counter gets updated once a day.
    (out, err, status) = asp_system_utils.executeCommand("acct_ytd", outputPath = None, 
                                                         suppressOutput = True, redo = True,
                                                         noThrow = True)
    logger.info("Hours used so far:\n" + out + '\n' + err)


    try:
      
        # Fetch and extract the tarball files from Lou
        
        #demTarballName      = os.path.basename(options.demTarball    )
        #orthoTarballName    = os.path.basename(options.orthoTarball  )
        #summaryTarballName  = os.path.basename(options.summaryTarball)
        localDemFolder     = os.path.join(runFolder, 'dems'   )
        localOrthoFolder   = os.path.join(runFolder, 'orthos' )
        localSummaryFolder = os.path.join(runFolder, 'summary')

        print 'Fetching and unpacking tarballs...'
        fetchTarball(options.demTarball,     localDemFolder)
        fetchTarball(options.orthoTarball,   localOrthoFolder)
        fetchTarball(options.summaryTarball, localSummaryFolder)

        # If the summary tarball unpacked to []/summary/summary,
        #  work with the lower level folder from now on.
        localSummaryFolder = descendIfNeeded(localSummaryFolder)

        # Make a list of all input files that are missing their summary file, and
        #  the desired output path for that file.
        missingDemFiles   = getMissingSummaryFiles(localDemFolder,   localSummaryFolder, isOrtho=False)
        missingOrthoFiles = getMissingSummaryFiles(localOrthoFolder, localSummaryFolder, isOrtho=True )

        # Divide this list into chunks and for each chunk generate a file containing all of
        #  the gdal_translate commands that need to be executed.
        print 'Writing command files...'
        commandFileLength = getParallelParams(options.nodeType)[2]
        commandFilePrefix = os.path.join(runFolder, 'convert_commands_')
        print 'Clearing existing command files.'
        os.system('rm ' + commandFilePrefix + '*')
        commandFileList   = writeCommandFiles(missingDemFiles, missingOrthoFiles,
                                              commandFilePrefix, commandFileLength)
        #raise Exception('DEBUG')

        # Get the location to store the logs
        pbsLogFolder = run.getPbsLogFolder()
        logger.info("Storing logs in: " + pbsLogFolder)
        os.system('mkdir -p ' + pbsLogFolder)

        # Call multi_process_command_runner.py through PBS for each chunk.
        start_time()
        (baseName, jobIDs) = submitBatchJobs(commandFileList, options, pbsLogFolder, run, logger)

        # Wait for everything to finish.
        pbs_functions.waitForJobCompletion(jobIDs, logger, baseName)
        stop_time("pbs_jobs", logger)

        # Check that we now have all of the summary files.
        # - Both of these should now be empty.
        newMissingDemFiles   = getMissingSummaryFiles(localDemFolder,   localSummaryFolder, isOrtho=False)
        newMissingOrthoFiles = getMissingSummaryFiles(localOrthoFolder, localSummaryFolder, isOrtho=True )

        resultText = ('After regeneration, missing %d DEM summaries and %d ORTHO summaries' 
                      % (len(newMissingDemFiles), len(newMissingOrthoFiles)))
        logger.info(resultText)

        runWasSuccess = ((not newMissingDemFiles) and (not newMissingOrthoFiles))

        # If successful, create a new tarball and send it to Lou.

        if runWasSuccess and (not options.skipArchiveSummary):
            start_time()
            archive_functions.packAndSendSummaryFolder(run, localSummaryFolder, logger)
            stop_time("archive summary", logger)


    except Exception as e:
        resultText = 'Caught exception: ' + str(e) + '\n' + traceback.format_exc()
        runWasSuccess = False

    # Send a summary email.
    emailAddress = getEmailAddress(icebridge_common.getUser())
    logger.info("Sending email to: " + emailAddress)
    if runWasSuccess:
        sendEmail(emailAddress, 'OIB summary regen passed', resultText)
    else:
        sendEmail(emailAddress, '"OIB summary regen failed', resultText)

    # TODO: Add automated delete command!
    #if options.wipeProcessed:
    #    processedFolder = run.getProcessFolder()
    #    logger.info("Will delete: " + processedFolder)
    #    os.system("rm -rf " + processedFolder)

    logger.info('==== regenerate_summary_images script has finished for run: ' + str(run) + ' ====')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
