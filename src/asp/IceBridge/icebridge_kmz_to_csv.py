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

'''
Extracts the frame event positions from an Icebridge KMZ file and
writes them out as a bundle_adjust compatible .csv file.  Currently
these files are available at this website: 
http://asapdata.arc.nasa.gov/dms/missions.html
'''

import sys, os
import shutil, string, errno, optparse, glob

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils
asp_system_utils.verify_python_version_is_supported()


def convert(inputPath, outputPath):
    '''Converts the input KML file to a CSV file'''

    fIn  = open(inputPath,  'r')
    fOut = open(outputPath, 'w')
    
    fOut.write('# ID, longitude, latitude, elevation\n')
    for line in fIn:
    
        if '<dd>Frame ID:' in line:
            start    = line.find(':')
            stop     = line.rfind(';')            
            idString = line[start+2:stop]
            
        if '<coordinates>' in line:
            # Extract the coordinate from the string
            start = line.find('>')
            stop  = line.rfind('<')
            s     = line[start+1:stop]
            parts = s.split(',')
            longitude = parts[0]
            latitude  = parts[1]
            elevation = parts[2]
    
            fOut.write('%s, %s, %s, %s\n' % (idString, longitude, latitude, elevation))
            idString = None
        
    fIn.close()
    fOut.close()    

def unpackKmz(inputPath):
    '''Extracts the KML file from the KMZ file'''
    
    # Unzip the KMZ file silently
    cmd = 'unzip -q -o ' + inputPath
    #print cmd
    os.system(cmd)
    
    # Return path to the KML file
    folder    = os.path.dirname(inputPath)
    unzipPath = os.path.join(folder, 'doc.kml')
    if not os.path.exists(unzipPath):
        raise Exception('Failed to unzip KML data!')
    return unzipPath

def main(inputPath, outputPath):
    '''Does all the work!'''

    kmlPath = unpackKmz(inputPath)
    convert(kmlPath, outputPath)

if __name__ == "__main__":

    # Handle input parameters
    if len(sys.argv) != 3:
        print('Wrong number of input arguments provided.')
        print('Usage: icebridge_kmz_to_csv <input kmz file> <output csv file>')
    else:
        sys.exit(main(sys.argv[1], sys.argv[2]))
    
        
        
        
        
        


