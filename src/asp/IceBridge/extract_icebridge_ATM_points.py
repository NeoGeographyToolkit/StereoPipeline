#!/usr/bin/env python

import sys, os, optparse

# The path to the ASP qi2txt file
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
sys.path.insert(0, basepath) # prepend to path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]

import sys
import os, glob, re, shutil, subprocess, string, time, errno, optparse, math

import asp_system_utils, asp_cmd_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]

def extract_qi_points(inputPath):
    '''Extract data from a n Icebridge qi file.
       Use this ASP format code to read: --csv-format 1:lat,2:lon,3:height_above_datum '''

    # Extract the data from the name
    name       = os.path.basename(inputPath)
    start      = name.find('_')
    stop       = name.rfind('_')
    dateString = name[start+1:stop]
    year       = int(dateString[0:4])
    month      = int(dateString[4:6])
    
    # Check if the file is little endian
    endianFlag = ' -L '
    if (year < 2010) or ((year == 2010) and (month < 10)):
        endianFlag = ' ' # Earlier dates are big endian, later dates are little endian.

    outputPath = inputPath.replace('.qi', '.csv')

    lookInLibexec = True
    execPath = asp_system_utils.which('qi2txt', lookInLibexec)
    cmd = execPath + ' -S' + endianFlag + inputPath + ' > ' + outputPath
    os.system(cmd)  
    if os.path.exists(outputPath):
        print 'Wrote file: ' + outputPath
    else:
        print 'Failed to write file ' + outputPath + '!!!'

def extract_hdf5_points(inputPath):
    '''Extract data from an Icebridge hdf5 file.
       Use this ASP format code to read: --csv-format 1:lat,2:lon,3:height_above_datum '''

    outputPath = inputPath.replace('.h5', '.csv')

    tempLat = inputPath + '_lat.txt'
    tempLon = inputPath + '_lon.txt'
    tempAlt = inputPath + '_alt.txt'

    lookInLibexec = True
    execPath = asp_system_utils.which('h5dump', lookInLibexec)
    
    cmd = execPath + ' -o '+tempLat+' -m "%.7f" --noindex  -w 20 --dataset=latitude  ' + inputPath
    os.system(cmd)
    cmd = execPath + ' -o '+tempLon+' -m "%.7f" --noindex  -w 20 --dataset=longitude ' + inputPath
    os.system(cmd)
    cmd = execPath + ' -o '+tempAlt+' -m "%.7f" --noindex  -w 20 --dataset=elevation ' + inputPath
    os.system(cmd)
    
    execPath = asp_system_utils.which('paste', lookInLibexec)
    cmd = execPath + ' ' + tempLat + ' ' + tempLon + ' ' + tempAlt + ' > ' + outputPath
    os.system(cmd)
    
    if os.path.exists(tempLat): os.remove(tempLat)
    if os.path.exists(tempLon): os.remove(tempLon)
    if os.path.exists(tempAlt): os.remove(tempAlt)
    
    if os.path.exists(outputPath):
        print 'Wrote file: ' + outputPath
    else:
        print 'Failed to write file ' + outputPath + '!!!'


def main(argsIn):

    # Use parser that ignores unknown options
    usage  = "usage: " + os.path.basename(__file__) + " <input cloud>"
    parser = asp_cmd_utils.PassThroughOptionParser(usage=usage, epilog="")

    # This call handles all the parallel_mapproject specific options.
    (options, args) = parser.parse_args(argsIn)

    if len(args) < 1:
        parser.print_help()
        parser.error("Missing input ATM cloud.\n" );
        
    # Get the input file and figure out the converter to use
    inputPath = args[0]
    
    ext = os.path.splitext(inputPath)[1]

    if ext == '.h5':
        extract_hdf5_points(inputPath)
        return 0
    if ext == '.qi':
        extract_qi_points(inputPath)
        return 0
        
    print 'Did not recognize input file type!'
    return -1
    
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
