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

# Top level program to process all of the Icebridge data.
# - This program is not sophisticated enough to handle everything and will need to be
#   superceded by another script.

import os, sys, optparse, datetime, time, subprocess, logging, multiprocessing
import re, shutil, time, getpass, argparse, subprocess32

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

import icebridge_common, pbs_functions, archive_functions, run_helper, lvis2kml
import asp_system_utils, asp_geo_utils, asp_image_utils

asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = basepath       + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = pythonpath     + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = libexecpath    + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = icebridgepath  + os.pathsep + os.environ["PATH"]

# TODO: Move this function!
def convertCoords(x, y, projStringIn, projStringOut):
    '''Convert coordinates from one projection to another'''

    # Using subprocess32 to access the timeout argument which is not always present in subprocess
    cmd = [asp_system_utils.which('gdaltransform'), '-s_srs', projStringIn, '-t_srs', projStringOut]
    #print(" ".join(cmd))
    p = subprocess32.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=False)
    textOutput, err = p.communicate( ('%f %f\n' % (x, y)), timeout=0.1 )
    parts = textOutput.split()
    
    return ( float(parts[0]), float(parts[1]) )

def generateFlightSummary(run, outputFolder, skipKml = False):
    '''Generate a folder containing handy debugging files including output thumbnails'''
    
    # Copy logs to the output folder
    print 'Copying log files...'
    os.system('mkdir -p ' + outputFolder)
    runFolder  = run.getFolder()
    procFolder = run.getProcessFolder()
    packedErrorLog = os.path.join(runFolder, 'packedErrors.log')
    if os.path.exists(packedErrorLog):
        shutil.copy(packedErrorLog, outputFolder)

    if not skipKml:
        # Copy the input camera kml file
        camerasInKmlPath = os.path.join(procFolder, 'cameras_in.kml')
        shutil.copy(camerasInKmlPath, outputFolder)
        
        # Create a merged version of all the bundle adjusted camera files
        # - The tool currently includes cameras more than once if they appear
        #   in multiple bundles.
        print 'Merging output camera kml files...'
        cmd = "find "+procFolder+" -name cameras_out.kml"
        p = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, shell=False)
        textOutput, err = p.communicate()
        camKmlFiles = textOutput.replace('\n', ' ')
        
        outputKml = os.path.join(outputFolder, 'cameras_out.kml')
        scriptPath = asp_system_utils.which('merge_orbitviz.py')
        cmd = scriptPath +' '+ outputKml +' '+ camKmlFiles
        print cmd
        os.system(cmd)
        
        # Generate lidar kml files
        LIDAR_POINT_SKIP = 1527
        lidarFiles = run.getLidarList(prependFolder=True)
        lidarOutputFolder = os.path.join(outputFolder, 'lidar')
        os.system('mkdir -p ' + lidarOutputFolder)
        for f in lidarFiles:
            inputPath = os.path.splitext(f)[0] + '.csv'
            outputPath = os.path.join(lidarOutputFolder, os.path.basename(f)+'.kml')
            args = [inputPath, outputPath, '--skip', str(LIDAR_POINT_SKIP), '--color', 'red']
            if not os.path.exists(outputPath): # Don't recreate these files
                lvis2kml.main(args)
       

    # TODO: Update to blended data!
    # Collect per-batch information
    print 'Consolidating batch information...'
    batchInfoPath   = os.path.join(outputFolder, 'batchInfoSummary.csv')
    failedBatchPath = os.path.join(outputFolder, 'failedBatchList.csv')
    with open(batchInfoPath, 'w') as batchInfoLog, open(failedBatchPath, 'w') as failureLog:
        # Write the header for the batch log file
        batchInfoLog.write('# startFrame, stopFrame, centerLon, centerLat, meanAlt, ' +
                           ' meanLidarDiff, meanInterDiff, meanFireDiff, meanFireLidarDiff\n')
        failureLog.write('# startFrame, stopFrame\n')
        
        demList = run.getOutputDemList()
        for (dem, frames) in demList:

            consolidatedStatsPath = dem.replace('out-align-DEM.tif', 'out-consolidated_stats.txt')
            if os.path.exists(consolidatedStatsPath):
                with open(consolidatedStatsPath, 'r') as f:
                    statsText = f.read()

                # Write info to summary file
                batchInfoLog.write('%d, %d, %s\n' % (frames[0], frames[1], statsText))

                # Keep a list of batches that did not generate an output DEM
                parts = statsText.split(',')
                if (int(parts[0]) == 0) and (int(parts[1]) == 0) and (int(parts[2]) == -999):
                failureLog.write('%d, %d\n' %  (frames[0], frames[1]))  

            else: 

                # TODO: Remove this deprecated part!

                # Get paths to the files of interest
                hillshadePath     = dem.replace('out-align-DEM.tif', 'out-DEM_HILLSHADE_browse.tif')
                lidarDiffPath     = dem.replace('out-align-DEM.tif', 'out-diff.csv')
                interDiffPath     = dem.replace('out-align-DEM.tif', 'out_inter_diff_summary.csv')
                fireDiffPath      = dem.replace('out-align-DEM.tif', 'out_fireball_diff_summary.csv')
                fireLidarDiffPath = dem.replace('out-align-DEM.tif', 'out_fireLidar_diff_summary.csv')

                # Read in the diff results            
                try:
                    lidarDiffResults = icebridge_common.readGeodiffOutput(lidarDiffPath)
                except:
                    lidarDiffResults = {'Mean':-999}
                try:
                    interDiffResults = icebridge_common.readGeodiffOutput(interDiffPath)
                except:
                    interDiffResults = {'Mean':-999}
                try:
                    fireDiffResults  = icebridge_common.readGeodiffOutput(fireDiffPath)
                except:
                    fireDiffResults  = {'Mean':-999}
                try:
                    fireLidarDiffResults = icebridge_common.readGeodiffOutput(fireLidarDiffPath)
                except:
                    fireLidarDiffResults = {'Mean':-999}

                try:
                    # Get DEM stats
                    geoInfo = asp_geo_utils.getImageGeoInfo(dem, getStats=False)
                    stats   = asp_image_utils.getImageStats(dem)[0]
                    meanAlt = stats[2]
                    centerX, centerY = geoInfo['projection_center']

                    # Convert from projected coordinates to lonlat coordinates            
                    isSouth    = ('+lat_0=-90' in geoInfo['proj_string'])
                    projString = icebridge_common.getEpsgCode(isSouth, asString=True)
                    PROJ_STR_WGS84 = 'EPSG:4326'
                    centerLon, centerLat = convertCoords(centerX, centerY, projString, PROJ_STR_WGS84)
                except:
                    centerLon = 0
                    centerLat = 0
                    meanAlt   = -999
                    
                    # Keep a list of batches that failed this step
                    failureLog.write('%d, %d\n' %  (frames[0], frames[1]))
                
                # Write info to summary file
                batchInfoLog.write('%d, %d, %f, %f, %f, %f, %f, %f, %f\n' % 
                                   (frames[0], frames[1], centerLon, centerLat, meanAlt, 
                                    lidarDiffResults['Mean'], interDiffResults    ['Mean'],
                                    fireDiffResults ['Mean'], fireLidarDiffResults['Mean']))
                                    
                # End deprecated code section!
            
            # Make a link to the thumbnail file in our summary folder
            if os.path.exists(hillshadePath):
                thumbName = ('dem_%05d_%05d_browse.tif' % (frames[0], frames[1]))
                thumbPath = os.path.join(outputFolder, thumbName)
                icebridge_common.makeSymLink(hillshadePath, thumbPath, verbose=False)
                
    print 'Finished generating flight summary in folder: ' + outputFolder

# The parent folder is where the runs AN_... and GR_..., etc., are
# stored. Usually it is the current directory.
def main(argsIn):
    '''Parse arguments and call the processing function'''

    try:
        # Sample usage:
        # python generate_flight_summary.py --yyyymmdd 20091016 --site AN 
        usage = '''generate_flight_summary.py <options>'''
                      
        parser = argparse.ArgumentParser(usage=usage)

        parser.add_argument("--yyyymmdd",  dest="yyyymmdd", required=True,
                          help="Specify the year, month, and day in one YYYYMMDD string.")
        
        parser.add_argument("--site",  dest="site", required=True,
                          help="Name of the location of the images (AN, GR, or AL).")

        parser.add_argument("--output-folder",  dest="outputFolder", default=None,
                          help="Name of the output folder. If not specified, " + \
                          "use something like AN_YYYYMMDD.")

        parser.add_argument("--parent-folder",  dest="parentFolder", default=os.getcwd(),
                            help="The folder having all the runs.")

        parser.add_argument("--skip-kml-gen", action="store_true", dest="skipKml", default=False, 
                            help="Skip combining kml files.")
        
        options = parser.parse_args(argsIn)
        
    except argparse.ArgumentError, msg:
        parser.error(msg)
        
    if options.outputFolder is None:
        options.outputFolder = icebridge_common.outputFolder(options.site, options.yyyymmdd)

    run = run_helper.RunHelper(options.site, options.yyyymmdd, options.parentFolder)
    
    generateFlightSummary(run, options.outputFolder, options.skipKml)
    
    return 0

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
