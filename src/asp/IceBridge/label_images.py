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

# Wrapper Script for the OSSP machine learning based labelling tool.

import os, sys, argparse, datetime, time, subprocess, logging, multiprocessing, re, glob
import traceback
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

import icebridge_common, run_helper
import asp_system_utils, asp_alg_utils, asp_geo_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = toolspath      + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = binpath        + os.pathsep + os.environ["PATH"]



def get_camera(cameraFolder, frame):
    '''Get the label file and the camera file for a given frame'''

    # Get a list of all the input files
#    allImageFiles  = icebridge_common.getTifs(labelFolder)
    allCameraFiles = icebridge_common.getByExtension(cameraFolder, '.tsai')

    # Keep only the images and cameras within the given range
#    imageFile = None
#    for image in allImageFiles:
#        thisFrame = icebridge_common.getFrameNumberFromFilename(image)
#        if thisFrame != frame:
#            continue
#        imageFile = os.path.join(labelFolder, image)
#        break

    cameraFile = None
    for camera in allCameraFiles:
        thisFrame = icebridge_common.getFrameNumberFromFilename(camera)
        if thisFrame != frame:
            continue
        cameraFile = os.path.join(cameraFolder, camera)
        break

    return cameraFile



def label_images(outputFolder, frameNum, trainingPath, site, yyyymmdd, numThreads):
    '''Apply the labeling algorithm to a single image, then map project the result.'''

    print 'Running label for frame: ' + str(frameNum)

    # Get required paths
    inputFolder  = icebridge_common.getJpegFolder      (outputFolder)
    cameraFolder = icebridge_common.getCameraFolder    (outputFolder)
    labelFolder  = icebridge_common.getLabelFolder     (outputFolder)
    orthoFolder  = icebridge_common.getLabelOrthoFolder(outputFolder)

    # Hardcoded paths!!!
    toolPath     = 'python ~/repo/OSSP/ossp_process.py'
    refDemFolder = '/nobackup/smcmich1/icebridge/reference_dems/'

    # Run the label tool
    NO_SPLITTING = 1 # Plenty of RAM to load these images
    ONE_PROCESS  = 1

    # Figure out the label path
    run       = run_helper.RunHelper(site, yyyymmdd)
    labelName = icebridge_common.makeLabelFileName(run, frameNum)
    labelPath = os.path.join(labelFolder, labelName)

    #cmd = ('%s %s --output_dir %s --min_frame %d --max_frame %d srgb %s --splits %d --parallel %d' % 
    #       (toolPath, inputFolder, labelFolder, frameNum, frameNum, trainingPath, NO_SPLITTING, ONE_PROCESS))
    cmd = ('time %s %s --output_dir %s --min_frame %d --max_frame %d srgb %s' %
           (toolPath, inputFolder, labelFolder, frameNum, frameNum, trainingPath))
    print cmd
    if icebridge_common.isValidImage(labelPath):
        print 'Skipping completed file: ' + labelPath
    else:
        os.system(cmd)

    # Also generate the map projected version of the image

    # Figure out the camera and output path
    cameraPath  = get_camera(cameraFolder, frameNum)
    fname       = os.path.basename(labelPath).replace('classified', 'classified_ortho')
    mapProjPath = os.path.join(orthoFolder, fname)

    if not icebridge_common.isValidImage(labelPath):
        print 'ERROR: Failed to generate label file: ' + labelPath
        return

    # Set map projection parameters
    toolPath = 'mapproject'
    isSouth  = (site == 'AN')
    srs      = projString = icebridge_common.getEpsgCode(isSouth, asString=True)
    demPath  = 'WGS84' # Map project on to a flat surface, elevation zero.
    #demPath  = os.path.join(refDemFolder, icebridge_common.getReferenceDemName(site)) # The NSIDC ortho DEM

    # Mapproject
    cmd = ('time %s %s %s %s %s -t nadirpinhole --t_srs %s --threads %d --num-processes 1 --ot Byte --nearest-neighbor' %
           (toolPath, demPath, labelPath, cameraPath, mapProjPath, srs, numThreads))
    print cmd
    if icebridge_common.isValidImage(mapProjPath):
        print 'Skipping existing file: ' + mapProjPath
    else:
        os.system(cmd)  

    if not os.path.exists(mapProjPath):
        print 'ERROR: Failed to generate map projected label file: ' + mapProjPath
        return

def main(argsIn):

    try:
        usage = '''label_images.py <options>'''

        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL)")

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

        parser.add_argument("--training",  dest="trainingPath", required=True,
                          help="Path to the training file.")

        parser.add_argument('--num-processes', dest='numProcesses', default=8,
                          type=int, help='The number of simultaneous processes to run.')
        parser.add_argument('--num-threads', dest='numThreads', default=1,
                          type=int, help='Used for mapproject.')

        options = parser.parse_args(argsIn)

    except argparse.ArgumentError as msg:
        parser.error(msg)

    if not os.path.exists(options.trainingPath):
        print 'Error: Input training file ' + options.trainingPath + ' does not exist!'
        return -1

    # TODO: Everything should use the RunHelper class for this!
    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)


    # Set up a processing tool to handle the frames, this will be more efficient
    #  than using the built-in mulithreading support.
    pool = multiprocessing.Pool(options.numProcesses)
    taskHandles = []

    for i in range(options.startFrame, options.stopFrame+1):

        # Run on a single frame with one thread.
        #label_images(options.outputFolder, i, options.trainingPath, options.site, options.yyyymmdd,  options.numThreads)
        taskHandles.append(pool.apply_async(label_images, (options.outputFolder, 
                                                           i, options.trainingPath, options.site,
                                                           options.yyyymmdd, options.numThreads)))

    # Wait for all the tasks to complete
    print('Finished adding ' + str(len(taskHandles)) + ' tasks to the pool.')
    icebridge_common.waitForTaskCompletionOrKeypress(taskHandles, interactive=False)

    # All tasks should be finished, clean up the processing pool
    icebridge_common.stopTaskPool(pool)
    print('Jobs finished.')


# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


