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

import os, sys, optparse, subprocess, logging

# TODO: Improve this!
logging.info('DEBUG')
logger = logging.getLogger(__name__)

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, icebridge_common
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = basepath    + os.pathsep + os.environ["PATH"]

#------------------------------------------------------------------------------

# TODO: Fetch the input nav file in the main fetch script

def main(argsIn):

    # Command line parsing
    try:
        usage  = "usage: camera_models_from_nav.py <image_folder> <ortho_folder> <cal_folder> <nav_folder> <output_folder>"
        parser = optparse.OptionParser(usage=usage)

        # This call handles all the parallel_mapproject specific options.
        (options, args) = parser.parse_args(argsIn)

        if len(args) < 4:
            print 'Error: Missing arguments.'
            print usage
            return -1
        imageFolder  = os.path.abspath(args[0])
        orthoFolder  = os.path.abspath(args[1]) # TODO: Replace with options
        calFolder    = os.path.abspath(args[2])
        navFolder    = os.path.abspath(args[3])
        outputFolder = os.path.abspath(args[4])

    except optparse.OptionError, msg:
        raise Exception(msg)

    if not os.path.exists(orthoFolder):
        logger.error('Ortho folder ' + orthoFolder + ' does not exist!')
        return -1

    # Find the nav file
    # - There should only be one or two nav files per flight.
    fileList = os.listdir(navFolder)
    fileList = [x for x in fileList if '.out' in x]
    if len(fileList) > 1:
        logger.error('TODO: Support double nav files!')
        return -1
    navPath = fileList[0]


    # Find a camera file to use.
    # - nav2cam does not support per-frame intrinsic data, so just feed
    #   it any random camera file and let ortho2pinhole insert the 
    #   correct instrinsic data.
    fileList = os.listdir(calFolder)
    fileList = [x for x in fileList if '.tsai' in x]
    if not fileList:
        logger.error('Unable to find any camera files in ' + calFolder)
        return -1
    cameraPath = fileList[0]

    # Get the ortho list
    orthoFiles = icebridge_common.getTifs(orthoFolder)
    logger.info('Found ' + str(len(orthoFiles)) + ' ortho files.')

    # Look up the frame numbers for each ortho file
    infoDict = {}
    for ortho in orthoFiles:
        if ('gray' in ortho) or ('sub' in ortho):
            continue
        frame = icebridge_common.getFrameNumberFromFilename(ortho)
        infoDict[frame] = [ortho, '']

    # Get the image file list
    imageFiles = icebridge_common.getTifs(imageFolder)
    logger.info('Found ' + str(len(imageFiles)) + ' image files.')

    # Update the second part of each dictionary object
    for image in imageFiles:
        if ('gray' in image) or ('sub' in image):
            continue
        frame = icebridge_common.getFrameNumberFromFilename(image)
        if frame not in infoDict:
            raise Exception('Image missing ortho file: ' +image)
        infoDict[frame][1] = image

    os.system('mkdir -p ' + outputFolder)
    orthoListFile = os.path.join(outputFolder, 'ortho_file_list.csv')

    # Open the output file for writing
    if not os.path.exists(orthoListFile):
        with open(orthoListFile, 'w') as outputFile:

            # Loop through frames in order
            for key in sorted(infoDict):
            
                # Write the ortho name and the output camera name to the file
                (ortho, image) = infoDict[key]
                if not image:
                    #raise Exception('Ortho missing image file: ' +ortho)
                    continue
                camera = image.replace('.tif', '.tsai')
                outputFile.write(ortho +', ' + camera + '\n')

    # Check if we already have all of the output camera files.
    haveAllFiles = True
    with open(orthoListFile, 'r') as inputFile:
        for line in inputFile:
            parts   = line.split(',')
            camPath = os.path.join(outputFolder, parts[1])
            if not os.path.exists(camPath):
                haveAllFiles = False
                break

    if haveAllFiles:
        logger.info('Already have all camera models from nav, quitting early!')
        return

    # Convert the nav file from binary to text    
    parsedNavPath = navPath.replace('.out', '.txt')
    cmd = 'sbet2txt.pl -q ' + navPath + ' > ' + parsedNavPath
    logger.info(cmd)
    asp_system_utils.executeCommand(cmd, parsedNavPath, suppressOutput=True, redo=False)

    # Call the C++ tool to generate a camera model for each ortho file
    cmd = ('nav2cam --input-cam %s --nav-file %s --cam-list %s --output-folder %s' 
           % (cameraPath, parsedNavPath, orthoListFile, outputFolder))
    logger.info(cmd)
    os.system(cmd)
    
    # Generate a kml file for the nav camera files
    kmlPath  = os.path.join(outputFolder, 'nav_cameras.kml')
    camFiles = os.listdir(outputFolder)
    camFiles = [x for x in camFiles if '.tsai' in x]
    camString = ' '.join(camFiles)
    logger.info('Generating nav camera kml file: ' + kmlPath)
    cmd = 'orbitviz_pinhole -o ' + kmlPath + ' ' + camString
    asp_system_utils.executeCommand(cmd, kmlPath, suppressOutput=True, redo=False)
    
    logger.info('Finished generating camera models from nav!')
    



if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))



