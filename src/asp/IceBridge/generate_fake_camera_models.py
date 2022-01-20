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

# Simple script to generate fake camera files for selected images.
# - The NSIDC upload script just requires camera file names to get the timestamp.

import os, sys, argparse, datetime, time
import traceback, fetch_icebridge_data, icebridge_common

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


def main(argsIn):

    try:
        usage = '''generate_fake_camera_models.py <options>'''

        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL)")

        parser.add_argument("--work-folder",  dest="workFolder", default=None,
                          help="Temporary download folder.")

        parser.add_argument("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        parser.add_argument('--start-frame', dest='startFrame', type=int,
                          default=icebridge_common.getSmallestFrame(),
                          help="Frame to start with.  Leave this and stop-frame blank to " + \
                          "process all frames.")
        parser.add_argument('--stop-frame', dest='stopFrame', type=int,
                          default=icebridge_common.getLargestFrame(),
                          help='Frame to stop on. This frame will also be processed.')

        options = parser.parse_args(argsIn)

    except argparse.ArgumentError as msg:
        parser.error(msg)

    # Fetch the jpeg files for missing camera files
    fetch_options       = options
    fetch_options.type  = 'jpeg'
    fetch_options.year  = int(options.yyyymmdd[0:4])
    fetch_options.month = int(options.yyyymmdd[4:6])
    fetch_options.day   = int(options.yyyymmdd[6:8])
    fetch_options.skipValidate       = True
    fetch_options.ignoreMissingLidar = True
    fetch_options.maxNumLidarToFetch = 0
    fetch_options.refetchIndex = False
    fetch_options.refetchNav = False
    fetch_options.stopAfterIndexFetch = False
    fetch_options.dryRun = False
    fetch_options.allFrames = False
    fetch_options.frameSkip = 0
    fetch_icebridge_data.doFetch(fetch_options, options.workFolder)

    if not os.path.exists(options.outputFolder):
        os.makedirs(options.outputFolder)

    # For each jpeg file, generate an empty file with the correct file name.

    inputFiles = os.listdir(options.workFolder)
    for f in inputFiles:
        if os.path.splitext(f)[1] != '.JPG':
            continue
        inputPath = os.path.join(options.workFolder, f)
        print inputPath

        # Get image info
        frame = icebridge_common.getFrameNumberFromFilename(inputPath)

        (datestr, timestr) = icebridge_common.getJpegDateTime(inputPath)

        # Pick output name
        outputName = icebridge_common.formFilePrefix(datestr, timestr, frame) + '.tsai'
        outputPath = os.path.join(options.outputFolder, outputName)

        cmd = 'touch ' + outputPath
        print cmd
        os.system(cmd)

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
