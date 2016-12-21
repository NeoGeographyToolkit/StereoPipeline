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
   Misc functions implementing algorithms.
"""

import sys, os, re, string, time, math

class BBox:
    def __init__(self, x, y, width, height):
        self.x      = x
        self.y      = y
        self.width  = width
        self.height = height

    def add_collar(self, size):
        '''Expands the BBox in all directions by the given size'''
        self.x      -= size
        self.y      -= size
        self.width  += 2*size
        self.height += 2*size

    def name_str(self):
        return "%i_%i_%i_%i" % ( self.x, self.y, self.width, self.height )

    def crop_str(self):
        '''Return an ASP command line option to specify that a process only
           fill in the pixels from a certain window.'''
        return ["--trans-crop-win",str(self.x),
                str(self.y),str(self.width),str(self.height)]
                
    def __str__(self):
        return 'BBox('+ str(self.x) +', '+ str(self.y) +', '+ str(self.width) +', '+ str(self.height) +')' 

def intersect_boxes(A, B):
    axmin = A.x; axmax = A.x + A.width; aymin = A.y; aymax = A.y + A.height
    bxmin = B.x; bxmax = B.x + B.width; bymin = B.y; bymax = B.y + B.height
    xmin  = max(axmin, bxmin); xmax = min(axmax, bxmax)
    ymin  = max(aymin, bymin); ymax = min(aymax, bymax)
    C     = BBox(0, 0, 0, 0)
    C.x   = xmin; C.width = xmax - xmin
    if (C.width  < 0):
        C.width = 0
    C.y      = ymin; 
    C.height = ymax - ymin
    if (C.height < 0):
        C.height = 0
    return C

def generateTileDir(startX, startY, stopX, stopY):
    """Generate the name of a tile based on its location"""

    tileString = 'tile_' + str(startX) + '_' + str(startY) + '_' + str(stopX) + '_' + str(stopY)
    return tileString

def generateTileName(startX, startY, stopX, stopY):
    """Generate the name of a tile based on its location"""

    return generateTileDir(startX, startY, stopX, stopY) + ".tif"

# Split a segment of of given length in pieces of about given size.
# All pieces are about the same length.
def genSegmentList(length, size, padding):

    if length <= 0: length = 1
    if size   <= 0: size   = 1
    num = int(math.ceil(length/float(size))) # if in doubt, make pieces smaller
    if num <= 0: num = 1

    goodSize = int(math.floor(length/float(num)))
    if goodSize <= 0: goodSize = 1

    # numBig segments will have length goodSize + 1, and the rest will have length goodSize
    numBig = length - goodSize*num

    count = 0
    L = [0]
    for c in range(0, num):
        if count < numBig:
            curLen = goodSize + 1
        else:
            curLen = goodSize
        count = count + 1
        pos = L[-1] + curLen
        L.append(pos)
    
    return L

