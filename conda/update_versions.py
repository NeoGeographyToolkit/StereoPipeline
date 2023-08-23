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
Use dependency versions from a conda enviornment .yaml file to update
a recipe/meta.yaml file of a given package. Such an input file can
be created from the given environment with:
conda env export > myenv.yaml
'''

import sys, os, re

if len(sys.argv) < 3:
    print("Usage: " + os.path.basename(sys.argv[0]) + " input.yaml mypackage-feedstock")
    sys.exit(1)
    
inFile = sys.argv[1]
outDir = sys.argv[2]

outFile = outDir + "/recipe/meta.yaml"
if not os.path.exists(outFile):
    print("Cannot open file: " + outFile)
    sys.exit(1)

# parse the versions from the conda env
conda_env = {}
print("Reading: " + inFile)
inHandle = open(inFile, 'r')
lines = inHandle.readlines()
for line in lines:

    # Wipe comments
    m = re.match('^(.*?)\#', line)
    if m:
        line = m.group(1)
        
    # Match the package
    m = re.match('^\s*-\s*(.*?)\s*=+\s*(.*?)(=|\s|$)', line)
    if not m:
        continue
    package = m.group(1)
    version = m.group(2)
    if re.match('^\s*$', package):
        continue # ignore empty lines
    
    conda_env[package] = version
    #print("got ", package, version)

# Update the lines in the output ile
outHandle = open(outFile, 'r')
lines = outHandle.readlines()
for it in range(len(lines)):

    line = lines[it]

    # Ignore comments
    m = re.match('^\#', line)
    if m:
        continue

    # Match the package
    m = re.match('^(\s+-\s*)(.*?)([\s=]+)(.*?)$', line)
    if not m:
        continue
    
    pre = m.group(1)
    package = m.group(2)
    separator = m.group(3).rstrip("\n")
    old_version = m.group(4).rstrip("\n")

    if separator == "":
        # Ensure there's at least one space
        separator = " "
        
    if old_version == "":
        # If there was no version before, don't put one now
        continue
    
    if not package in conda_env:
        continue
    version = conda_env[package]
    if old_version != version:
        if ('[linux]' in old_version) or ('[osx]' in old_version):
            # In this case the user better take a closer look
            print("For package " + package + ", not replacing " +
                  old_version + " with " + version + ", a closer look is suggested.")
        else:
            print("For package " + package + ", replacing version "
                  + old_version + " with " + version)
            lines[it] = pre + package + separator + version + "\n"

# Save the updated lines to disk
print("Updating: " + outFile)
outHandle = open(outFile, "w")
outHandle.writelines(lines)
outHandle.close()
