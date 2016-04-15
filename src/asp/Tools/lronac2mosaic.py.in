#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


import os, glob, optparse, re, shutil, subprocess, sys, string, time

libexecpath = os.path.abspath(sys.path[0] + '/../libexec')
sys.path.insert(0, libexecpath) # prepend to Python path
from stereo_utils import get_asp_version

import asp_system_utils
asp_system_utils.verify_python_version_is_supported()

job_pool = [];

# Global output folder variable
outputFolder = ""

def man(option, opt, value, parser):
    print >>sys.stderr, parser.usage
    print >>sys.stderr, '''\
This program operates on LRO (.IMG) files, and performs the
following ISIS 3 operations:
 * Converts to ISIS format (lronac2isis)
 * Attaches SPICE information (spiceinit and spicefit)
 * Performs radiometric calibration (lronaccal)
 * lronacecho?
 * Removes camera distortions from the CCD images (noproj)
 * Performs jitter analysis (lrojitreg)
 * Mosaics individual CCDs into one unified image file (handmos)
 * Normalizes the mosaic (cubenorm)
'''

    sys.exit()

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def add_job( cmd, num_working_threads=4 ):
    if ( len(job_pool) >= num_working_threads):
        job_pool[0].wait();
        job_pool.pop(0);
    print(cmd)
    job_pool.append( subprocess.Popen(cmd, shell=True, env=os.environ) );

def wait_on_all_jobs():
    print("Waiting for jobs to finish")
    while len(job_pool) > 0:
        job_pool[0].wait();
        job_pool.pop(0);

# Go through a list of cubes and sort them into left/right pairs
def build_cube_pairs(cubePaths):
    pairDict = dict();

    for cube in cubePaths:
        print(cube)
        m = re.search('\D*(\d+)(.).*',os.path.basename(cube))
        number     = m.group(1)
        sideLetter = m.group(2)

        if (number not in pairDict):
            pairDict[number] = ['', ''];
        # Store the path in the spot for either the left or right cube
        if (sideLetter == "L"):
            pairDict[number][0] = cube; # Left
        else:
            pairDict[number][1] = cube; # Right
    return pairDict

def read_flatfile( flat ):
    # Fail if the input file is not present
    if not os.path.isfile(flat):
        raise Exception('File ' + flat + ' is missing!')

    averages = [0.0,0.0]

    f = open(flat,'r')
    for line in f:
        if ( line.rfind("Average Sample Offset:") >= 0 ):
            index       = line.rfind("Offset:");
            index_e     = line.rfind("StdDev:");
            crop        = line[index+7:index_e];
            if crop == " NULL ": # Check for null value
                raise Exception('Null sample offset in file ' + flat)
            averages[0] = float(crop);
        elif ( line.rfind("Average Line Offset:") >= 0 ):
            index       = line.rfind("Offset:");
            index_e     = line.rfind("StdDev:");
            crop        = line[index+7:index_e];
            if crop == "   NULL ": # Check for null value
                raise Exception('Null sample offset in file ' + flat)
            averages[1] = float(crop);
        elif ( line.rfind("Using IpFind result only:") >= 0 ):
            index       = line.rfind("only:");
            if (line[index + 7] == 1):
                print("Warning: This result based only on IpFind search.")
    print(str(averages))
    return averages

# Call lronac2isis on each input file, return list of output files.
def lronac2isis( img_files, threads, outputFolder ):
    lronac2isis_cubs = []
    for img in img_files:
        # Expect to end in .IMG, change to end in .cub and move to output folder
        newExtension = os.path.splitext(img)[0] + '.cub'
        cubFilePath  = os.path.join(outputFolder, os.path.basename(newExtension))
        if( os.path.exists(cubFilePath) ):
            print(cubFilePath + ' exists, skipping lronac2isis.')
        else:
            cmd = 'lronac2isis from='+ img +' to='+ cubFilePath
            add_job(cmd, threads)
        lronac2isis_cubs.append( cubFilePath )
    wait_on_all_jobs()
    return lronac2isis_cubs


# Call lronaccal on each input file, return list of output files.
def lronaccal( cub_files, threads, delete=False ):
    lronaccal_cubs = []
    for cub in cub_files:
        # Expect to end in .cub, change to end in .lronaccal.cub
        to_cub = os.path.splitext(cub)[0] + '.lronaccal.cub'
        if( os.path.exists(to_cub) ):
            print(to_cub + ' exists, skipping lronaccal.')
        else:
            cmd = 'lronaccal from='+ cub +' to='+ to_cub
            add_job(cmd, threads)
        lronaccal_cubs.append( to_cub )
    wait_on_all_jobs()

    if( delete ): # Delete all input .cub files and log files
        for cub in cub_files:
          os.remove( cub )
        lronaccal_log_files = glob.glob( os.path.commonprefix(cub_files) + '*.lronaccal.log' )
        for file in lronaccal_log_files:
          os.remove( file )
    return lronaccal_cubs


# Call lronacecho on each input file, return list of output files.
def lronacecho( cub_files, threads, delete=False ):
    lronacecho_cubs = []
    for cub in cub_files:
        # Expect to end in .cub, change to end in .lronaccal.cub
        to_cub = os.path.splitext(cub)[0] + '.lronacecho.cub'
        if( os.path.exists(to_cub) ):
            print(to_cub + ' exists, skipping lronacecho.')
        else:
            cmd = 'lronacecho from='+ cub +' to='+ to_cub
            add_job(cmd, threads)
        lronacecho_cubs.append( to_cub )
    wait_on_all_jobs()

    if( delete ): # Delete all input .cub files and log files
        for cub in cub_files:
          os.remove( cub )
    return lronacecho_cubs


def spice( cub_files, threads):
    for cub in cub_files:
        cmd = 'spiceinit web=false from='+ cub
        add_job(cmd, threads)
    wait_on_all_jobs()
    for cub in cub_files:
        cmd = 'spicefit from='+ cub
        add_job(cmd, threads)
    wait_on_all_jobs()
    return

# Returns true if the .cub LRONAC file has CROSSTRACK_SUMMING = 1
def isFileHalfRes(cubFilePath):

    return False; # It looks like the normal pvl file works so use it in all cases

    f = open(cubFilePath, 'r')
    for line in f:
        if ( line.rfind("CROSSTRACK_SUMMING") >= 0 ):
            index       = line.rfind("=");
            crop        = line[index+2];
            result      = (crop == "2")
    f.close()

    return result;


# Left file is/home/smcmich1 in index 0, right is in index 1
def noproj( file_pairs, threads, delete, fakePvl, outputFolder):
    if fakePvl: # Generate temporary PVL files containing LRONAC definition
                # - We need one for full-res mode, one for half-X-res mode.

       fullResFilePath = os.path.join(outputFolder, 'noprojInstruments_fullRes.pvl')
       if os.path.exists(fullResFilePath):
          print(fullResFilePath + ' exists, using existing file.')
       else: # Need to write the file
          print('Generating LRONAC compatible .pvl file ' + fullResFilePath)
          f = open(fullResFilePath, 'w')
          f.write('Object = IdealInstrumentsSpecifications\n');
          f.write('  UserName     = auto\n');
          f.write('  Created      = 2013-07-18T13:42:00\n');
          f.write('  LastModified = 2013-07-18T13:42:00\n\n');
          f.write('  Group = "LUNAR RECONNAISSANCE ORBITER/NACL"\n');
          f.write('     TransY = 16.8833\n')
          f.write('     ItransS = -2411.9\n')
          f.write('     TransX = 0.6475\n')
          f.write('     ItransL = -92.5\n')
          f.write('     DetectorSamples = 10000\n')
          f.write('  End_Group\n\n')
          f.write('End_Object\n')
          f.write('End')
          f.close()

       halfResFilePath = os.path.join(outputFolder, 'noprojInstruments_halfRes.pvl')
       if os.path.exists(halfResFilePath):
          print(halfResFilePath + ' exists, using existing file.')
       else: # Need to write the file
          print('Generating LRONAC compatible .pvl file ' + halfResFilePath)
          f = open(halfResFilePath, 'w')
          f.write('Object = IdealInstrumentsSpecifications\n');
          f.write('  UserName     = auto\n');
          f.write('  Created      = 2013-07-18T13:42:00\n');
          f.write('  LastModified = 2013-07-18T13:42:00\n\n');
          f.write('  Group = "LUNAR RECONNAISSANCE ORBITER/NACL"\n');
          f.write('     TransY = 16.8833\n')
          f.write('     ItransS = -4823.8\n')     # Halved
          f.write('     TransX = 0.6475\n')
          f.write('     ItransL = -185\n')       # Halved
          f.write('     DetectorSamples = 5000\n') # Halved
          f.write('  End_Group\n\n')
          f.write('End_Object\n')
          f.write('End')
          f.close()

    noproj_pairs = dict();
    for k, v in file_pairs.items():

        noproj_pairs[k] = ['', ''];
        for i in range(2): # Process left and right image
          to_cub = os.path.splitext(v[i])[0] + '.noproj.cub'

          noproj_pairs[k][i] = to_cub; # Add file to output list
          if os.path.exists( to_cub ):
              print(to_cub + ' exists, skipping noproj.')
          else:

              # Generate pvl command if needed
              if fakePvl:

                  fileIsHalfRes = isFileHalfRes(v[0])
                  if fileIsHalfRes:
                      specsLine = ' specs=' + os.path.abspath(halfResFilePath) + ' ';
                  else: # Full resolution
                      specsLine = ' specs=' + os.path.abspath(fullResFilePath) + ' ';
              else: # Use the default file
                  specsLine = '';

              # Multiple noproj threads will create clashing temporary files
              #  so we need to make temporary directories to run each thread in.
              tempDir = 'temp_' + str(k) + '_' + str(i)
              tempDir = os.path.join(outputFolder, tempDir)
              cmd = 'mkdir -p ' + tempDir + ' && ' \
                  + 'cd ' + tempDir + ' && ' \
                  + 'noproj from=' + os.path.abspath(v[i]) \
                  + ' match=' + os.path.abspath(v[0]) \
                  + specsLine \
                  + ' to=' + os.path.abspath(to_cub) + ' && ' \
                  + 'cd .. && rm -rf ' + tempDir

              add_job(cmd, threads)
    wait_on_all_jobs()

    if( delete ): # Clean up input cube files
        for v in file_pairs.values():
           os.remove( v[0] );
           os.remove( v[1] );
#        if fakePvl: # These are not deleted in case this program is running in multiple threads
#           os.remove( halfResFilePath );
#           os.remove( fullResFilePath );
    return noproj_pairs;


def lronacjitreg( noproj_pairs, threads, delete=False ):
    boundsCommands = '--correlator-type 2 --kernel 15 15'
    for k,v in noproj_pairs.items():
        cmd = 'lronacjitreg ' + boundsCommands    \
            + ' --output-log outputLog_'+str(k)+'.txt' \
            + ' '+ v[0] \
            + ' '+ v[1];
        add_job(cmd, threads)
    wait_on_all_jobs()

    # Read in all the shift values from the output text files
    averages = dict()
    for k,v in noproj_pairs.items():
        flat_file = 'outputLog_'+str(k)+'.txt'
        print('Reading log file ' + flat_file)
        averages[k] = read_flatfile( flat_file )
        if delete:
            os.remove( flat_file )

    return averages


def mosaic( noproj_pairs, averages, threads ):
    mosaicList = dict();
    for k,v in noproj_pairs.items():

        # Create mosaic output file
        mosaicPath = os.path.splitext(v[0])[0] + '.mosaic.cub'
        shutil.copy( v[0], mosaicPath ) # Copy the LE image to the output path

        xOffset = -1*averages[k][0] # Sign convention changes here
        yOffset = -1*averages[k][1]

        handmos( v[1], mosaicPath,
                 str( int(round( xOffset )) ),
                 str( int(round( yOffset )) ),
                 threads )
        mosaicList[k] = mosaicPath;

    wait_on_all_jobs()

    return mosaicList


def handmos( fromcub, tocub, outsamp, outline, threads ):
    cmd = 'handmos from='+ fromcub +' mosaic='+ tocub \
            +' outsample = '+ str(outsamp) \
            +' outline = '  + str(outline) \
            +' matchbandbin=FALSE priority=ontop';
    add_job(cmd, threads);
    return


def cubenorm( mosaicList, threads, delete=False ):
    normedList = dict();
    for k,v in mosaicList.items():

        normedPath = os.path.splitext(v)[0] + '.norm.cub'

        cmd = 'cubenorm from='+ v +' to='+ normedPath
        add_job(cmd, threads);

        normedList[k] = normedPath;

    wait_on_all_jobs()

    if( delete ): # Clean up input cube files
        for v in mosaicList.values():
           os.remove(v);

    return normedList

def cropInputs(inputFiles, outputFolder, cropAmount, threads, delete=False):

    outputPaths = []
    for path in inputFiles:
        # Expect to end in .IMG, change to end in .cub and move to output folder
        newExtension = os.path.splitext(path)[0] + '.cropped.cub'
        croppedPath  = os.path.join(outputFolder, os.path.basename(newExtension))
        cmd = 'crop from='+ path +' to='+ croppedPath + ' nlines=' + str(cropAmount)
        add_job(cmd, threads)
        outputPaths.append( croppedPath )

    wait_on_all_jobs()

    if delete:
        for path in inputFiles:
            os.remove(path)

    return outputPaths


#--------------------------------------------------------------------------------

#TODO: Support for file based logging of results

def main():
    try:
        try:
            usage = "usage: lronac2mosaic.py [--help][--manual][--crop][--threads N]" \
                    "[--keep] LRONAC.IMG-files\n  " + get_asp_version()
            parser = optparse.OptionParser(usage=usage)
            parser.set_defaults(delete =True)
            parser.set_defaults(cropAmount=0)
            parser.set_defaults(threads=4)
            parser.set_defaults(fakePvl=True)
            parser.add_option("--manual", action="callback", callback=man,
                              help="Read the manual.")
            parser.add_option("-o", "--output-dir", dest="outputFolder",
                              help="Output folder (default to input folder).",type="string")
            parser.add_option("--stop-at-no-proj", dest="stop_no_proj", action="store_true",
                              help="Process the IMG files only to have SPICE attached.")
            parser.add_option("--resume-at-no-proj", dest="resume_no_proj", action="store_true",
                              help="Pick back up after spiceinit has happened. This was noproj uses your new camera information")
            parser.add_option("-c", "--crop", dest="cropAmount",
                              help="Process only the first N lines of the image.",type="int")
            parser.add_option("-t", "--threads", dest="threads",
                              help="Number of threads to use.",type="int")
            parser.add_option("-k", "--keep", action="store_false",
                              dest="delete",
                              help="Will not delete intermediate files.")
            parser.add_option("--p", dest="fakePvl", action="store_true",
                              help="Don't automatically create a LRONAC pvl file")
            (options, args) = parser.parse_args()

            if not args: parser.error("need .IMG files")

        except optparse.OptionError as msg:
            raise Usage(msg)

        # Make sure only one pair of cubes was passed in
        input_file_pair = build_cube_pairs( args )
        if len(input_file_pair) > 1:
            raise Usage('Input error: Only one pair of input files are allowed!')

        if not options.outputFolder: # Set the output folder equal to the input folder
            options.outputFolder = os.path.dirname(args[0])
        print('Using output folder: ' + options.outputFolder)
        if not os.path.exists(options.outputFolder) and len(options.outputFolder) > 1:
            os.makedirs(options.outputFolder)

        print("Beginning processing.....")

        if not options.resume_no_proj: # If not skipping to later point

            print("lronac2isis") # Per-file operation, returns list of new files
            lronac2isised = lronac2isis( args, options.threads, options.outputFolder )

            print("lronaccal")   # Per-file operation, returns list of new files
            lronaccaled = lronaccal( lronac2isised, options.threads, options.delete )

            print("lronacecho")  # Per-file operation, returns list of new files
            lronacechod = lronacecho( lronaccaled, options.threads, options.delete )

            if (options.cropAmount > 0): # Crop the input files as soon as ISIS calls allow it
                lronacechod = cropInputs(lronacechod, options.outputFolder, options.cropAmount,
                                         options.threads, options.delete)

            print("spice")       # Attach spice info to cubes (adds to existing files)
            spice( lronacechod, options.threads )


        if options.stop_no_proj: # Stop early if requested
            print("Finished")
            return 0

        if options.resume_no_proj: # If resume option was set
            lronacechod = args

        print("build_cube_pairs") # Detected corresponding pairs of cubes
        lronac_file_pairs = build_cube_pairs( lronacechod )

        print("noproj")       # Per-file operation
        noprojed_file_pairs = noproj( lronac_file_pairs, options.threads, options.delete, options.fakePvl, options.outputFolder)

        print("lronacjitreg") # Determines mean shift for each file pair
        averages = lronacjitreg( noprojed_file_pairs, options.threads, options.delete )

        print("mosaic")       # handmos - Use mean shifts to combine the file pairs
        mosaicked = mosaic( noprojed_file_pairs, averages, options.threads )

        # Clean up noproj files
        if( options.delete ):
          for cub in noprojed_file_pairs.values():
              os.remove( cub[0] )
              os.remove( cub[1] )

        # Run a final cubenorm across the image:
        cubenorm( mosaicked, options.threads, options.delete )

        print("Finished")
        return 0

    except Usage as err:
        print >>sys.stderr, err.msg
        return 2

    # To more easily debug this program, comment out this catch block.
    # except Exception, err:
    #     sys.stderr.write( str(err) + '\n' )
    #     return 1


if __name__ == "__main__":
    sys.exit(main())
