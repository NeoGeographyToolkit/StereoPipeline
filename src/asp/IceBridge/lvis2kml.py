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

import sys, os, optparse, sys, string, simplekml

'''
Convert an LVIS lidar data or XML file into a Google Earth compatible KML format.
The raw data produces points, the XML files produce bounding regions.
'''

def readPositions(positionFilePath):
    '''Read in the LVIS data'''

    if not os.path.exists(positionFilePath):
        raise Exception('File ' + positionFilePath + ' is missing!')

    # Is this a border file or a raw data file?
    isBorderFile = ('.xml' in positionFilePath)
    
    isLvisFile = ('.TXT' in positionFilePath)
    
    f = open(positionFilePath, 'r')
    
    pointList = []
    lon = 0 # Only used for border files
    for line in f:

        if isBorderFile:
            if not (('PointLongitude' in line) or ('PointLatitude' in line)):
                continue
            
            # Find the number
            start = line.find('>')+1
            stop  = line.rfind('<')
            num   = float(line[start:stop])
            
            # Record pairs with no elevation
            if 'PointLongitude' in line:
                lon = num
            if 'PointLatitude' in line:
                thisPoint = (lon, num)
                pointList.append(thisPoint)

        else: # Lidar point files (possibly converted to CSV by another tool)
                
            if '#' in line: # Skip lines containing the comment symbol
                continue
            
            strings = line.split()
            
            # Record lot/lat/alt triples
            if isLvisFile:
                lon    = float(strings[3])
                lat    = float(strings[4])
                height = float(strings[5])
            else: # ATM lidar
                lon    = float(strings[1])
                lat    = float(strings[0])
                height = float(strings[2])                
            thisPoint = (lon, lat, height)
            pointList.append(thisPoint)
        
    f.close()
    return pointList


def generatePointKml(pointList, outputPath, pointSkip, name, color):
    """Generates a KML plot for the input points"""
        
    # Initialize kml document
    kml = simplekml.Kml()
    kml.document.name = name
    kml.hint = 'target=earth'
    
    ALT = 2
    
    # Compute the min and max point values
    minHeight = pointList[0][ALT]
    maxHeight = minHeight
    for p in pointList:
        if p[ALT] < minHeight:
            minHeight = p[ALT]
        if p[ALT] > maxHeight:
            maxHeight = p[ALT]
    heightRange = maxHeight - minHeight
    
    print 'min = ' + str(minHeight)
    print 'max = ' + str(maxHeight)
    
    # Plot each point
    counter = 0
    for i in range (0, len(pointList), int(pointSkip)):
    
        (lon, lat, height) = pointList[i]
        point = kml.newpoint(name=str(counter), coords=[(lon, lat, height)],
                              altitudemode= simplekml.AltitudeMode.relativetoground)

        point.extrude = 0
        counter       = counter + 1
        
        point.style.labelstyle.scale    = 0
        point.style.iconstyle.scale     = 0.6
        point.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/open-diamond.png'
        
        # Generate a color based on the error value:  white (low error) <--> red (high error)
        thisHeight = pointList[i][ALT]
        colorVal = int(255.0 - 255.0*(thisHeight - minHeight)/heightRange)
        if (colorVal < 0):
            colorVal = 0
        #print str(pointList[i][ALT]) + ' --> ' + str(colorVal)
        if color == 'blue':
            point.style.iconstyle.color   = simplekml.Color.rgb(colorVal,colorVal,255,255)
        elif color == 'green':
            point.style.iconstyle.color   = simplekml.Color.rgb(colorVal,255,colorVal,255)
        else: # red
            point.style.iconstyle.color   = simplekml.Color.rgb(255,colorVal,colorVal,255)
    
    
    # Save kml document
    kml.save(outputPath)
    return counter
    

def generateBorderKml(pointList, outputPath, name, color):
    """Generates a KML plot of a point border"""
    
    # Initialize kml document
    kml = simplekml.Kml()
    kml.document.name = name
    kml.hint = 'target=earth'

    # Make a polygon with the provided borders    
    poly = kml.newpolygon()
    poly.outerboundaryis.coords = pointList
    poly.outerboundaryis.coords = pointList
    
    # Set style
    if color == 'blue':
        poly.style.linestyle.color = simplekml.Color.blue
    elif color == 'green':
        poly.style.linestyle.color = simplekml.Color.green
    else: # red
        poly.style.linestyle.color = simplekml.Color.red

    # Save kml document
    kml.save(outputPath)
    return len(pointList)


#--------------------------------------------------------------------------------------------
    
def main(argsIn):
    
    try:
        usage = "usage: lvis2kml [options] <inputPath> [outputPath]\n" 
        parser = optparse.OptionParser(usage=usage)
        parser.set_defaults(skip=1)
        parser.set_defaults(name=None)
        parser.set_defaults(color='red')
        parser.add_option("--skip",  dest="skip",  help="Plot only every N'th point (default 1)")
        parser.add_option("--name",  dest="name",  help="KML name")
        parser.add_option("--color", dest="color", help="Draw in red, blue, or green")
        (options, args) = parser.parse_args(argsIn)

    except optparse.OptionError, msg:
        raise Exception(msg)

    if len(args) < 1:
        print 'Missing required input path.'
        print usage
        return -1

    # If output path is not specified, just append .kml to the input path.
    inputPath = args[0]
    if len(args) > 1:
        outputPath = args[1]
    else:
        outputPath = inputPath + '.kml'
        
    if not options.name:
        options.name = os.path.basename(inputPath)
        
    print "Beginning processing....."

    pointList = readPositions(inputPath)
    print 'Loaded ' + str(len(pointList)) +' points.'
    
    if '.xml' in inputPath: # Plot a border polygon
        generateBorderKml(pointList, outputPath, options.name, options.color)
    else: # Plot individual points
        generatePointKml(pointList, outputPath, options.skip, options.name, options.color)

    print "Finished"
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

    
