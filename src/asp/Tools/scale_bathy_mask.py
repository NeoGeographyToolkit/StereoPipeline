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
A tool which takes as input a multispectral mask and a PAN image,
and scales and crops the former to agree with the latter. Used
during shallow-water bathymetry. See:
https://stereopipeline.readthedocs.io/en/latest
'''
from __future__ import print_function
import sys, os, subprocess

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../Python')  # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec') # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, asp_image_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]

if len(sys.argv) < 4:
    print("Usage: " + sys.argv[0] + " ms_mask.tif pan_image.tif output_pan_mask.tif")
    sys.exit(1)

ms_mask = sys.argv[1]
pan_image = sys.argv[2]

if not os.path.exists(ms_mask):
    print("Missing file: " + ms_mask)
    sys.exit(1)

if not os.path.exists(pan_image):
    print("Missing file: " + pan_image)
    sys.exit(1)

# It is not clear how much to crop on the left
crop_len = 48
if len(sys.argv) >= 5:
    crop_len = sys.argv[4]
print("Will crop " + str(crop_len) + " pixels on the left after scaling the mask.")

output_pan_mask = sys.argv[3]
tmp_pan_mask = os.path.splitext(output_pan_mask)[0]+'_tmp.tif'

ms_width, ms_height = asp_image_utils.getImageSize(ms_mask)
pan_width, pan_height = asp_image_utils.getImageSize(pan_image)

# gdal_translate with options
gdt = 'gdal_translate -co compress=lzw -co TILED=yes -co INTERLEAVE=BAND ' + \
      '-co BLOCKXSIZE=256 -co BLOCKYSIZE=256 '

# Scale up the mask. Cast to float32.
cmd = gdt + '-outsize 400% 400% -ot float32 ' + ms_mask + " " + tmp_pan_mask

print(cmd)
os.system(cmd)

scaled_width, scaled_height = asp_image_utils.getImageSize(tmp_pan_mask)

# Remove crop_len pixels from the left edge, then adjust the other dimensions
# so that the resulting PAN mask dimensions agree with the PAN image.
cmd = gdt + "-srcwin " + str(crop_len) + " 0 " + str(pan_width) + " " + str(pan_height) + \
      " " + tmp_pan_mask + " " + output_pan_mask
print(cmd)
os.system(cmd)

