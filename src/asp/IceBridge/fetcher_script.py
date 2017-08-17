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

# Fetch all the data for a run and run conversions.
# See sample usage below.

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing, re, shutil, time
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

import asp_system_utils, asp_alg_utils, asp_geo_utils
import icebridge_common, full_processing_script

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath      + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath        + os.pathsep + os.environ["PATH"]

louUser = 'oalexan1' # all data is stored under this user name
    
def workDirs():
    '''When fetching data, return the paths where it is stored temporarily on pfe,
    and for archival, on lou.'''
    currDir = os.getcwd()
    m = re.match("^.*?/" + louUser + "/(.*?)$", currDir)
    if not m:
        raise Exception("Could not match %s in %s " % (louUser, currDir))
    pfePath = '/nobackupp7/' + louUser + '/' + m.group(1) # path on pfe
    lfePath = '/u/'            + louUser + '/' + m.group(1) # path on lfe
    
    return (pfePath, lfePath)
    
def tarAndWipe(options, logger):
    '''Connect to lou from where we can se the files, then tar and wipe the current run.'''

    logger.info("All files were fetched and checks passed. " +
                "Will tar to lou and wipe the dir.")

    # Per https://www.nas.nasa.gov/hecc/support/kb/using-
    # shift-for-local-transfers-and-tar-operations_512.html
    # one can tar and push to lfe at the same time if
    # connecting to lfe first, from where one can see the pfe
    # filesystem. Avoid though using the suggested shift
    # command. It may be faster, but it detaches and is hard
    # to manage. Note: To untar and transfer in one step, one
    # should as well go to lfe first.

    (pfePath, lfePath) = workDirs()
    
    lfeCmd = 'cd ' + pfePath + '; tar cfv ' + lfePath + '/' + \
             options.outputFolder + '.tar ' + options.outputFolder

    cmd = 'ssh ' + louUser + '@lfe "' + lfeCmd + '"'
    logger.info(cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        raise Exception('Failed to tar and copy to lfe.')
    else:
        logger.info('Success tarring and copying to lfe.')

    if options.outputFolder == "" or options.outputFolder[0] == '.':
        raise Exception('Output folder is not as expected. ' +
                        'Not deleting anything just in case.')

    logger.info('Will wipe: ' + options.outputFolder)
    try:
        shutil.rmtree(options.outputFolder)
    except Exception as e:
        # TODO: Can't wipe it as still logging there
        print("Failed to wipe " + options.outputFolder)

    return 0

def startWithLouArchive(options, logger):
    '''Connect to lou, and untar a given archive on pfe.'''

    (pfePath, lfePath) = workDirs()

    # See tarAndWipe() for the logic of how one can work with pfe and lfe
    lfeCmd = 'cd ' + lfePath + '; tar xfv ' + lfePath + '/' + options.outputFolder + '.tar' + \
             ' -C ' + pfePath

    cmd = 'ssh ' + louUser + '@lfe "' + lfeCmd + '"'
    logger.info(cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        raise Exception('Failed to untar lfe archive: ' + options.outputFolder + '.tar')
    else:
        logger.info('Success untarring lfe archive.')

    return 0

def main(argsIn):

    try:
        # Sample usage:
        # python fetcher_script.py \
        #  --yyyymmdd 20091016 --site AN --start-frame 350 --stop-frame 353 --skip-validate
        # An output folder will be crated automatically (with a name like
        # AN_20091016), or its name can be specified via the --output-folder
        # option.
        usage = '''usage: fetcher_script.py <options>'''
                      
        parser = optparse.OptionParser(usage=usage)

        # Run selection
        parser.add_option("--yyyymmdd",  dest="yyyymmdd", default=None,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_option("--site",  dest="site", default=None,
                          help="Name of the location of the images (AN, GR, or AL)")

        # Python treats numbers starting with 0 as being in octal rather than decimal.
        # Ridiculous. So read them as strings and convert to int. 
        parser.add_option('--start-frame', dest='startFrameStr', default=None,
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_option('--stop-frame', dest='stopFrameStr', default=None,
                          help='Frame to stop on.')
        parser.add_option('--max-num-lidar-to-fetch', dest='maxNumLidarToFetch', default=100000000,
                          type='int', help='The maximum number of lidar files to fetch. ' + \
                          'This is used in debugging.')
        parser.add_option("--skip-validate", action="store_true", dest="skipValidate",
                          default=False,
                          help="Skip input data validation.")

        parser.add_option("--refetch-index", action="store_true", dest="refetchIndex",
                          default=False,
                          help="Force refetch of the index file.")
        parser.add_option("--stop-after-index-fetch", action="store_true",
                          dest="stopAfterIndexFetch", default=False,
                          help="Stop after fetching the indices.")

        parser.add_option("--tar-and-wipe", action="store_true", dest="tarAndWipe", default=False,
                          help="After fetching all data and performing all conversions and " + \
                          "validations, make a tarball on lou and wipe the directory. "      + \
                          "Only valid on Pleiades!")
        parser.add_option("--start-with-lou-archive", action="store_true",
                          dest="startWithLouArchive", default=False,
                          help="Untar an existing archive from lou, then continue.")
                          
        (options, args) = parser.parse_args(argsIn)

    except optparse.OptionError, msg:
        raise Usage(msg)

    if options.yyyymmdd is None or options.site is None:
        print("The flight date and site must be specified.")
        return -1

    options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)
    os.system('mkdir -p ' + options.outputFolder)
    
    logLevel = logging.INFO
    logger   = icebridge_common.setUpLogger(options.outputFolder, logLevel,
                                            'icebridge_fetcher_log')

    # Explicitely go from strings to integers, per earlier note.
    if options.startFrameStr is not None:
        startFrame = int(options.startFrameStr)
    else:
        startFrame = icebridge_common.getSmallestFrame()
    if options.stopFrameStr is not None:
        stopFrame  = int(options.stopFrameStr)
    else:
        stopFrame = icebridge_common.getLargestFrame()

    # Unarchive, then continue with fetching
    if options.startWithLouArchive:
        startWithLouArchive(options, logger)

    cmd = (('--yyyymmdd %s --site %s --start-frame %d --stop-frame %d ' +
            '--max-num-lidar-to-fetch %d --stop-after-convert --no-ortho-convert --refetch')
           % (options.yyyymmdd, options.site, startFrame, stopFrame,
              options.maxNumLidarToFetch))
    if options.refetchIndex:
        cmd += ' --refetch-index' # this was not right in older fetched runs
    if options.stopAfterIndexFetch:
        cmd += ' --stop-after-index-fetch' 
    if options.skipValidate:
        cmd += ' --skip-validate'
        
    logger.info("full_processing_script.py " + cmd)
    
    if full_processing_script.main(cmd.split()) < 0:
        return -1

    # Archive after fetching
    if options.tarAndWipe:
        tarAndWipe(options, logger)

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


