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
import sys, os, subprocess, re

# Read the MEANPRODUCTGSD (mean ground sample distance of the delivered
# product, in meters) from a DigitalGlobe/Maxar camera XML file.
def read_mean_product_gsd(xml_file):
    with open(xml_file, 'r') as f:
        text = f.read()
    m = re.search(r'<MEANPRODUCTGSD>\s*([0-9eE.+-]+)\s*</MEANPRODUCTGSD>', text)
    if m is None:
        print("Could not find MEANPRODUCTGSD in: " + xml_file)
        sys.exit(1)
    return float(m.group(1))

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

if len(sys.argv) < 6:
    print("Usage: " + sys.argv[0] + " ms_mask.tif pan_image.tif output_pan_mask.tif " + \
          "ms_camera.xml pan_camera.xml [num_left_cols_crop]")
    sys.exit(1)

ms_mask         = sys.argv[1]
pan_image       = sys.argv[2]
output_pan_mask = sys.argv[3]
ms_xml          = sys.argv[4]
pan_xml         = sys.argv[5]

for f in [ms_mask, pan_image, ms_xml, pan_xml]:
    if not os.path.exists(f):
        print("Missing file: " + f)
        sys.exit(1)

# It is not clear how much to crop on the left
# A value of 50 seems to work better with WV03 and 48 with WV02.
crop_len = 50
if len(sys.argv) >= 7:
    crop_len = sys.argv[6]
print("Will remove " + str(crop_len) + " columns on the left after scaling the mask.")

# The mask scaling assumes the multispectral GSD is 4x the panchromatic GSD.
# This holds by design for DigitalGlobe/Maxar sensors (WorldView, QuickBird,
# GeoEye) at native resolution, and is independent of the off-nadir angle. Read
# both GSDs and assert the ratio is 4, to catch a non-standard product (pan-
# sharpened, reduced-resolution, or custom-GSD). The tolerance is generous, as
# the ratio is 4 by design and MEANPRODUCTGSD is only accurate to a few mm.
ms_gsd  = read_mean_product_gsd(ms_xml)
pan_gsd = read_mean_product_gsd(pan_xml)
ratio   = ms_gsd / pan_gsd
tol     = 0.1
if abs(ratio - 4.0) > tol:
    print("Error: the multispectral-to-panchromatic MEANPRODUCTGSD ratio is " + \
          str(ratio) + " (MS = " + str(ms_gsd) + " m, PAN = " + str(pan_gsd) + \
          " m), but this tool assumes a ratio of 4. A different ratio means a " + \
          "non-standard product (pan-sharpened, reduced-resolution, or custom-" + \
          "GSD). Aborting.")
    sys.exit(1)
print("MS/PAN MEANPRODUCTGSD ratio = " + str(ratio) + " (MS = " + str(ms_gsd) + \
      " m, PAN = " + str(pan_gsd) + " m), within " + str(tol) + " of 4.")
tmp_pan_mask = os.path.splitext(output_pan_mask)[0]+'_tmp.tif'

ms_width, ms_height = asp_image_utils.getImageSize(ms_mask)
pan_width, pan_height = asp_image_utils.getImageSize(pan_image)

# gdal_translate with options
gdt = 'gdal_translate -co compress=lzw -co TILED=yes -co INTERLEAVE=BAND ' + \
      '-co BLOCKXSIZE=256 -co BLOCKYSIZE=256 -co BIGTIFF=YES '

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

# Wipe the temporary file
print("Removing: " + tmp_pan_mask)
os.remove(tmp_pan_mask)
