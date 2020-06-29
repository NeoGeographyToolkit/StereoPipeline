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

"""
   Functions for working with images containing geo metadata.
"""
from __future__ import print_function
import sys, os, glob, re, shutil, string, time, errno, subprocess
import asp_string_utils, asp_image_utils, asp_system_utils

def getGdalInfoTagValue(text, tag):
    """Gets the value of a gdal parameter in a [""] tag or None if it is absent."""

    try:
        lineAfterTag = asp_string_utils.getLineAfterText(text, tag)
        
        # The remaining line should look like this: ",25],
        commaPos   = lineAfterTag.find(',')
        bracketPos = lineAfterTag.find(']')
        # The value is always returned as a string
        return asp_string_utils.convertToFloatIfNumber(lineAfterTag[commaPos+1:bracketPos])
    
    except Exception: # Requested tag was not found
        return None

def convertCoordinate(input_srs_string, output_srs_string, x, y):
    '''Convert a single 2D coordinate between proj.4 coordinate systems.'''

    cmd = [asp_system_utils.which('gdaltransform'), '-s_srs',
           input_srs_string, '-t_srs', output_srs_string]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, encoding = 'utf8')
    input_str = str(x) + " " + str(y)
    out, err = p.communicate(input = input_str)
    parts = out.split()
    return (float(parts[0]), float(parts[1]))

def getLonLatProjString(inputString):
    '''Given a projected proj4 string, get the longlat proj4 string'''
    
    if '+proj=longlat' in inputString:
        return inputString
    
    # The output string is longlat projection plus certain allowed proj4 keys.
    outputString = '+proj=longlat'
    keywords = ['+datum', '+ellps', '+no_defs', '+a', '+b']
   
    parts = inputString.split()
    for p in parts:
        for k in keywords:
            if k in p:
                outputString += ' ' + p
                break
    return outputString


# This can take a while if stats are requested
def getImageGeoInfo(imagePath, getStats=True):
    """Obtains some image geo information from gdalinfo in dictionary format"""
    
    if not os.path.exists(imagePath):
        raise Exception('Error: input file ' + imagePath + ' does not exist!')
    
    outputDict = {}
    
    # Call command line tool silently
    cmd = [asp_system_utils.which('gdalinfo'), imagePath, '-proj4']
    if getStats:
        cmd.append('-stats')
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
    textOutput, err = p.communicate()
    
    # Get the size in pixels
    imageSizeLine = asp_string_utils.getLineAfterText(textOutput, 'Size is ')
    sizeVals      = imageSizeLine.split(',')
    outputDict['image_size'] = (int(sizeVals[0]), int(sizeVals[1]))

    # Get origin location and pixel size    
    originLine    = asp_string_utils.getLineAfterText(textOutput, 'Origin = ')
    pixelSizeLine = asp_string_utils.getLineAfterText(textOutput, 'Pixel Size = ')    
    originVals    = asp_string_utils.getNumbersInParentheses(originLine)
    pixelSizeVals = asp_string_utils.getNumbersInParentheses(pixelSizeLine)
    outputDict['origin']     = originVals
    outputDict['pixel_size'] = pixelSizeVals

    # Get bounding box in projected coordinates
    upperLeftLine  = asp_string_utils.getLineAfterText(textOutput, 'Upper Left')
    lowerRightLine = asp_string_utils.getLineAfterText(textOutput, 'Lower Right')
    (minX, maxY)   = asp_string_utils.getNumbersInParentheses(upperLeftLine)
    (maxX, minY)   = asp_string_utils.getNumbersInParentheses(lowerRightLine)
    outputDict['projection_bounds'] = (minX, maxX, minY, maxY)
    outputDict['projection_center'] = ( (minX+maxX)/2.0, (minY+maxY)/2.0 )

    # Get some proj4 values
    outputDict['standard_parallel_1'] = getGdalInfoTagValue(textOutput, 'standard_parallel_1')
    outputDict['central_meridian']    = getGdalInfoTagValue(textOutput, 'central_meridian')

    # Get the projection type
    projStart = textOutput.find('PROJ.4 string is:')
    nextLine  = textOutput.find("'", projStart)+1
    endLine   = textOutput.find("'", nextLine)
    outputDict['proj_string'] = textOutput[nextLine:endLine]
    outputDict['projection'] = 'UNKNOWN'
    if '+proj=eqc' in textOutput:
        outputDict['projection'] = 'EQUIRECTANGULAR'
    elif ('+proj=ster' in textOutput) or ('+proj=stere' in textOutput):
        outputDict['projection'] = 'POLAR STEREOGRAPHIC'

    outputDict['lonlat_bounds'] = outputDict['projection_bounds']
    if '+proj=longlat' not in outputDict['proj_string']:
        longlatString = getLonLatProjString(outputDict['proj_string'])
        ul = convertCoordinate(outputDict['proj_string'], longlatString, minX, maxY)
        br = convertCoordinate(outputDict['proj_string'], longlatString, maxX, minY)
        outputDict['lonlat_bounds'] = (ul[0], br[0], br[1], ul[1])

    # Extract this variable which ASP inserts into its point cloud files
    try:
        pointOffsetLine = asp_string_utils.getLineAfterText(textOutput, 'POINT_OFFSET=') # Tag name must be synced with C++ code
        offsetValues    = pointOffsetLine.split(' ')
        outputDict['point_offset'] =  (float(offsetValues[0]), float(offsetValues[1]), float(offsetValues[2]))        
    except:
        pass # In most cases this line will not be present

    # TODO: Currently this does not find much information, and there
    #       is another function in image_utils dedicated to returning statistics.
    if getStats:

        # List of dictionaries per band
        outputDict['band_info'] = []
    
        # Populate band information
        band = 1
        while (True): # Loop until we run out of bands
            bandString = 'Band ' + str(band) + ' Block='
            bandLoc = textOutput.find(bandString)
            if bandLoc < 0: # Ran out of bands
                break
        
            # Found the band, read pertinent information
            bandInfo = {}
        
            # Get the type string
            bandLine = asp_string_utils.getLineAfterText(textOutput, bandString)
            typePos  = bandLine.find('Type=')
            commaPos = bandLine.find(',')
            typeName = bandLine[typePos+5:commaPos-1]
            bandInfo['type'] = typeName
        
            outputDict['band_info'] = bandInfo
        
            band = band + 1 # Move on to the next band
        
    return outputDict

def doesImageHaveGeoData(imagePath):
    '''Returns true if a file has geo data associated with it'''
    
    if not os.path.exists(imagePath):
        raise Exception('Image file ' + imagePath + ' not found!')
    
    # Call command line tool silently
    cmd = [asp_system_utils.which('gdalinfo'), imagePath, '-proj4']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
    textOutput, err = p.communicate()
    
    # For now we just do a very simple check
    if "Coordinate System is `'" in textOutput:
        return False
    else:
        return True
    

# This is a very handy function but it requires a C++ tool!
#
#def getGeoTiffBoundingBox(geoTiffPath):
#    """Returns (minLon, maxLon, minLat, maxLat) for a geotiff image"""
#    
#    if not os.path.exists(geoTiffPath):
#        raise Exception('Input file does not exist: ' + geoTiffPath)
#    
#    # Call command line tool silently
#    cmd = ['geoRefTool', '--printBounds', geoTiffPath]
#    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
#    textOutput, err = p.communicate()
#
#    # Check that the call did not fail
#    if (textOutput.find('Failed') >= 0):
#        raise Exception('Error: getGeoTiffBoundingBox failed on input image: ' + geoTiffPath)
#    
#    # Parse the output
#    try:
#        minLat = float( asp_string_utils.getLineAfterText(textOutput, 'Min latitude  =') )
#        maxLat = float( asp_string_utils.getLineAfterText(textOutput, 'Max latitude  =') )
#        minLon = float( asp_string_utils.getLineAfterText(textOutput, 'Min longitude =') )
#        maxLon = float( asp_string_utils.getLineAfterText(textOutput, 'Max longitude =') )
#    except Exception as e:
#        print 'In file: ' + geoTiffPath
#        print 'In text:'
#        print textOutput
#        raise e
#    
#    return (minLon, maxLon, minLat, maxLat)
#

#=========================================================================
# Start ISIS functions

def getProjectedBoundsFromIsisLabel(filePath):
    '''Function to read the projected coordinates bounding box from an ISIS label file'''

    if not os.path.exists(filePath):
        raise Exception('Error, missing label file path!')
    
    # Read all the values!
    minX    = None
    maxY    = None
    pixRes  = None
    numRows = None
    numCols = None
    f = open(filePath, 'r')
    for line in f:
        if ('UpperLeftCornerX' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<')
            if (endPos >= 0):
                minX = float(s[:endPos-1])
            else:
                minX = float(s)
            continue
        if ('UpperLeftCornerY' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<')
            if (endPos >= 0):
                maxY = float(s[:endPos-1])
            else:
                maxY = float(s)
            continue
        if ('PixelResolution' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<')
            if (endPos >= 0):
                pixRes = float(s[:endPos-1])
            else:
                pixRes = float(s)
            continue
        if ('      Samples =' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            numCols = float(s)
            continue
        if ('      Lines   =' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            numRows = float(s)
            continue
        
    f.close()
    if (not minX) or (not maxY) or (not pixRes) or (not numRows) or (not numCols):
        raise Exception('Failed to find projected bounds in file ' + filePath)

    # Compute the other bounds
    maxX = minX + pixRes*numCols
    minY = maxY - pixRes*numRows

    return (minX, maxX, minY, maxY)


def getProjectionFromIsisLabel(filePath):
    '''Function to read the projection type from an ISIS label file'''

    if not os.path.exists(filePath):
        raise Exception('Error, missing label file path!')
    
    f = open(filePath, 'r')
    for line in f:
        if ('MAP_PROJECTION_TYPE          =' in line) or ('ProjectionName     =' in line):
            line = line.replace('"','') # Strip quotes
            projType = asp_string_utils.getLineAfterText(line, '=').strip()
            f.close()
            return projType
    f.close()
    raise Exception('Unable to find projection type in file ' + filePath)


def getBoundingBoxFromIsisLabel(filePath):
    '''Function to read the bounding box from an ISIS label file'''

    if not os.path.exists(filePath):
        raise Exception('Error, missing label file path!')
    
    numFound = 0
    f = open(filePath, 'r')
    for line in f:
        if ('MINIMUM_LATITUDE' in line) or ('MinimumLatitude' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<')
            if (endPos >= 0):
                minLat = float(s[:endPos-1])
            else:
                minLat = float(s)
            numFound = numFound + 1
            continue
        if ('MAXIMUM_LATITUDE' in line) or ('MaximumLatitude' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<')
            if (endPos >= 0):
                maxLat = float(s[:endPos-1])
            else:
                maxLat = float(s)
            numFound = numFound + 1
            continue
        if ('EASTERNMOST_LONGITUDE' in line) or ('MAXIMUM_LONGITUDE' in line)  or ('MaximumLongitude' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<') # Check for unit name
            if (endPos >= 0):
                maxLon = float(s[:endPos-1])
            else:
                maxLon = float(s)
            numFound = numFound + 1
            continue
        if ('WESTERNMOST_LONGITUDE' in line) or ('MINIMUM_LONGITUDE' in line) or ('MinimumLongitude' in line):
            s = asp_string_utils.getLineAfterText(line, '=')
            endPos = s.find('<') # Check for unit name
            if (endPos >= 0):
                minLon = float(s[:endPos-1])
            else:
                minLon = float(s)
            numFound = numFound + 1
            continue
        if numFound == 4:
            break

    f.close()
    if numFound < 4:
        raise Exception('Failed to find lat/lon bounds in file ' + filePath)

    return (minLon, maxLon, minLat, maxLat)


# TODO: Create a real bounding box class or something
def getIsisBoundingBox(cubePath):
    """Returns (minLon, maxLon, minLat, maxLat) for an ISIS compatible object"""
   
    # Get the cube size, then request the positions of the four corners
    cubeSize = getImageSize(cubePath)
    
    # Note that the underlying ISIS tool is one-based
    points  = []
    firstPt =     getPixelLocInCube(cubePath, 1,           1,         )['gdc']
    points.append(getPixelLocInCube(cubePath, cubeSize[0], 1,         )['gdc'])
    points.append(getPixelLocInCube(cubePath, 1,           cubeSize[1])['gdc'])
    points.append(getPixelLocInCube(cubePath, cubeSize[0], cubeSize[1])['gdc'])

    # Go through the four corners and get the bounding box
    minLon = firstPt[0]
    maxLon = firstPt[0]
    minLat = firstPt[1]
    maxLat = firstPt[1]
    
    for p in points:
        if p[0] < minLon:
            minLon = p[0]
        if p[0] > maxLon:
            maxLon = p[0]
        if p[1] < minLat:
            minLat = p[1]
        if p[1] > maxLat:
            maxLat = p[1]

    return (minLon, maxLon, minLat, maxLat)


def getCubeCenterLatitude(cubePath, workDir='tmp'):
    """Calls caminfo on a cube and returns the CenterLatitude value"""

    # Make sure the requested file is present
    if not os.path.exists(cubePath):
        raise Exception('File ' + cubePath + ' does not exist!')

    # Call caminfo (from ISIS) on the input cube to find out the CenterLatitude value
    camInfoOuputPath = workDir + "/camInfoOutput.txt"
    cmd = 'caminfo from=' + cubePath + ' to=' + camInfoOuputPath
    print (cmd)
    os.system(cmd)

    if not os.path.exists(camInfoOuputPath):
        raise Exception('Call to caminfo failed on file ' + cubePath)

    # Read in the output file to extract the CenterLatitude value
    centerLatitude = -9999
    infoFile       = open(camInfoOuputPath, 'r')
    for line in infoFile:
        if (line.find('CenterLatitude') >= 0):
            centerLatitude = asp_string_utils.getNumberAfterEqualSign(line, )
            break
    # Make sure we found the desired value
    if (centerLatitude == -9999) or (isinstance(centerLatitude, basestring)):
        raise Exception("Unable to find CenterLatitude from file " + cubePath)

    # Clean up temporary file
    os.remove(camInfoOuputPath)
    
    return centerLatitude

# End ISIS related functions
#======================================================================


def getImageBoundingBox(filePath):
    """Returns (minLon, maxLon, minLat, maxLat) for a georeferenced image file"""

    extension = os.path.splitext(filePath)[1]
    if asp_image_utils.isIsisFile(filePath):
        return getIsisBoundingBox(filePath)
    else: # Handle all other types
        return getGeoTiffBoundingBox(filePath)
          
    # Any other file types will end up raising some sort of exception
    
    

def build_vrt( fullImageSize, tileLocs, tilePaths, outputPath ):
    """Generates a VRT file from a set of image tiles and their locations in the output image"""

    outputFolder = os.path.dirname(outputPath)

    f = open(outputPath, 'w')
    f.write("<VRTDataset rasterXSize=\"%i\" rasterYSize=\"%i\">\n" % (int(fullImageSize[0]),int(fullImageSize[1])) ) # Write whole image size

    #
    ## If a tile is missing, for example, in the case we
    ## skipped it when it does not intersect user's crop box,
    ## substitute it with a different one, to ensure the mosaic
    ## does not have holes. --> Does this make sense?
    #goodFilename = ""
    #for tile in tiles: # Find the first valid tile (we don't care which one)
    #    directory = settings['out_prefix'][0] + tile.name_str()
    #    filename  = directory + "/" + tile.name_str() + tile_postfix
    #    if os.path.isfile(filename):
    #        goodFilename = filename
    #        break
    #if goodFilename == "":
    #    raise Exception('No tiles were generated')

    
    # Read some metadata from one of the tiles
    gdalInfo = getImageGeoInfo(tilePaths[0])
    
    num_bands = len(gdalInfo['band_info'])
    data_type = gdalInfo['band_info'][0]['type']

    # This special metadata value is only used for ASP stereo point cloud files!    
    if 'point_offset' in gdalInfo:
        f.write("  <Metadata>\n    <MDI key=\"" + 'POINT_OFFSET' + "\">" +
                gdalInfo['point_offset'][0] + "</MDI>\n  </Metadata>\n")
      

    # Write each band
    for b in range( 1, num_bands + 1 ):
        f.write("  <VRTRasterBand dataType=\"%s\" band=\"%i\">\n" % (data_type, b) ) # Write band data type and index

        for tile, tileLoc in zip(tilePaths, tileLocs):
            filename  = tile
            
            imageSize = getImageSize(filename) # Get the image size for this tile

            ## Replace missing tile paths with the good tile we found earlier
            #if not os.path.isfile(filename): filename = goodFilename

            relative = os.path.relpath(filename, outputPath) # Relative path from the output file to the input tile
            f.write("    <SimpleSource>\n")
            f.write("       <SourceFilename relativeToVRT=\"1\">%s</SourceFilename>\n" % relative) # Write relative path
            f.write("       <SourceBand>%i</SourceBand>\n" % b)
            f.write("       <SrcRect xOff=\"%i\" yOff=\"%i\" xSize=\"%i\" ySize=\"%i\"/>\n" % (tileLoc[0], tileLoc[1], imageSize[0], imageSize[1]) ) # Source ROI (entire tile)
            f.write("       <DstRect xOff=\"%i\" yOff=\"%i\" xSize=\"%i\" ySize=\"%i\"/>\n" % (tileLoc[0], tileLoc[1], imageSize[0], imageSize[1]) ) # Output ROI (entire tile)
            f.write("    </SimpleSource>\n")
        f.write("  </VRTRasterBand>\n")
    f.write("</VRTDataset>\n")
    f.close()    

