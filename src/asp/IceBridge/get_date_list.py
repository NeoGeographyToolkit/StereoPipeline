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

# Compile a list of all the dates with data available

import os, sys, optparse, datetime, subprocess, re


# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, asp_alg_utils, asp_geo_utils, icebridge_common
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = basepath    + os.pathsep + os.environ["PATH"]

#------------------------------------------------------------------------------

# Constants

def fetchAndParseIndexFile(folderUrl, path, parsedPath, fileType):
    '''Retrieve the index file for a folder of data and create
    a parsed version of it that contains frame number / filename pairs.'''

    # Set up the command
    cookiePaths = ' -b ~/.urs_cookies -c ~/.urs_cookies '
    curlOpts    = ' -n -L '
    cmd = 'curl ' + cookiePaths + curlOpts + folderUrl + ' > ' + path

    # Download the file
    print cmd
    p = subprocess.Popen(cmd, shell=True)
    os.waitpid(p.pid, 0)
    
    # Find all the file names in the index file and
    #  dump them to a new index file
    print 'Extracting file name list from index.html file...'
    with open(path, 'r') as f:
        indexText = f.read()
    
    # Extract just the file names
    if fileType == 'image':
        fileList = re.findall(">[0-9_]*.JPG", indexText)
    
    # For each entry that matched the regex, record: the "frame" number, the file name.
    with open(parsedPath, 'w') as f:
        for x in fileList:
            if fileType == 'image':
                f.write(x[12:17] +', '+ x[1:] + '\n')

def main(argsIn):

    # Command line parsing
    try:
        usage  = "usage: get_date_list.py output_path"
        parser = optparse.OptionParser(usage=usage)

        # This call handles all the parallel_mapproject specific options.
        (options, args) = parser.parse_args(argsIn)

        if len(args) != 1:
            print 'Error: Missing output path.'
            print usage
            return -1
        outputPath = os.path.abspath(args[0])

    except optparse.OptionError, msg:
        raise Exception(msg)

    # TODO: Move into a common function!
    # Verify that required files exist
    home = os.path.expanduser("~")
    if not (os.path.exists(home+'/.netrc') and os.path.exists(home+'/.urs_cookies')):
        print 'Missing a required authentication file!  See instructions here:'
        print '    https://nsidc.org/support/faq/what-options-are-available-bulk-downloading-data-https-earthdata-login-enabled'
        return -1
    
    topIndexPath  = outputPath + '_top.csv'
    tempIndexPath = outputPath + '_temp.csv'

    # Get the top level index
    TOP_URL = 'https://n5eil01u.ecs.nsidc.org/ICEBRIDGE_FTP/IODMS0_DMSraw_v01/'
    print 'Fetching top level index from: ' + TOP_URL
    icebridge_common.fetchFile(TOP_URL, topIndexPath)
    
    with open(topIndexPath, 'r') as f:
        topText = f.read()
    
    # Find all the sub folders in this file
    matches     = re.findall(">[0-9_ANGR]*_NASA", topText)
    missionList = [x[1:] for x in matches]
    
    allDates = []
    
    # Loop through the sub folders
    for mission in missionList:
        missionUrl = TOP_URL + mission
        print 'Checking mission at: ' + missionUrl
        icebridge_common.fetchFile(missionUrl, tempIndexPath)

        site = mission[5:7]

        with open(tempIndexPath, 'r') as f:
            missionText = f.read()
        
        matches  = re.findall(">[0-9]*_raw", missionText)
        dateList = [x[1:] for x in matches]
        
        for date in dateList:
            yyyymmdd = date[4:8] + date[0:4]
            allDates.append( (yyyymmdd, site) )

    with open(outputPath, 'w') as f:
        for date in allDates:
            f.write(date[0] +', '+ date[1] +'\n')

    print 'Wrote out ' + str(len(allDates)) + ' dates to file.'
    print 'Finished!'

    # Clean up
    os.remove(topIndexPath)
    os.remove(tempIndexPath)

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



