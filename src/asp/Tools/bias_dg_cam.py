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

# A script to bias the EPH (positions) and ATT (orientations) fields
# in a Maxar (DigitalGlobe) linescan camera.

import sys, os, re, optparse

try:
    usage = "Usage: bias_dg_cam.py --position-bias 'x y z' --orientation-bias 'x y z w' " + \
            "-i input.xml -o output.xml" 
    
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("--position-bias", dest="position_bias",
                      help="A list of four numbers, in quotes, having the values to add " + \
                      "to positions, in meters.")
    parser.add_option("--orientation-bias", dest="orientation_bias",
                      help="A list of four numbers, in quotes, having the values to " + \
                      "add to quaternions (orientations), in order x, y, z, w.")
    parser.add_option("-i", "--input-camera", dest="in_cam",
                      help="Output camera.", type="string")
    parser.add_option("-o", "--output-camera", dest="out_cam",
                      help="Output camera.", type="string")

    (opt, args) = parser.parse_args()
    
except optparse.OptionError as msg:
    raise Usage(msg)

if opt.in_cam is None or opt.out_cam is None:
    print("Must specify input and output cameras.\n" + usage)
    sys.exit(1)
    
if opt.position_bias is None or opt.orientation_bias is None:
    print("Must specify the position and orientation bias.\n" + usage)
    sys.exit(1)

vals = opt.position_bias.split()
if len(vals) != 3:
    print("Must have three values for the position bias, enclosed in quotes.")
    sys.exit(1)

position_bias = []    
for val in vals:
    position_bias.append(float(val))
    
vals = opt.orientation_bias.split()
if len(vals) != 4:
    print("Must have four values for the orientation bias, enclosed in quotes.")
    sys.exit(1)

orientation_bias = []    
for val in vals:
    orientation_bias.append(float(val))

print("Reading: " + opt.in_cam)
handle = open(opt.in_cam, 'r')
lines = handle.readlines()
handle.close()

for count in range(len(lines)):
    line = lines[count]
    
    m = re.search('^(.*?<EPHEMLIST>)(.*?)(</EPHEMLIST>.*?\n)', line)
    if m:
        ephem = m.group(2)
        vals = ephem.split()
        # Modify values with index 1, 2, 3 (not 0) representing positions
        for it in range(1, 4):
            vals[it] = str(float(vals[it]) + position_bias[it - 1])
        ephem = " ".join(vals)
        lines[count] = m.group(1) + ephem + m.group(3)

    m = re.search('^(.*?<ATTLIST>)(.*?)(</ATTLIST>.*?\n)', line)
    if m:
        att = m.group(2)
        vals = att.split()
        # Modify values with index 1, 2, 3, 4 (not 0), representing quaternions
        for it in range(1, 5):
            vals[it] = str(float(vals[it]) + orientation_bias[it - 1])
        att = " ".join(vals)
        lines[count] = m.group(1) + att + m.group(3)

print("Writing: " + opt.out_cam)
handle = open(opt.out_cam, 'w')
handle.writelines(lines)
handle.close()
