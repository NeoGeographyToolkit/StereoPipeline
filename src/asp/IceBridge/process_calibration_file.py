#!/usr/bin/env python
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

# Given a calibration file in pdf format from
# https://nsidc.org/data/IODCC0/
# convert it to the tsai format that can be used by ASP.
# Usage:  python ~/projects/StereoPipeline/src/asp/IceBridge/process_calibration_file.py file.pdf

import os, sys, optparse, subprocess, re

# The path to the ASP python files
basepath    = os.path.abspath(sys.path[0])
pythonpath  = os.path.abspath(basepath + '/../IceBridge')  # for dev ASP
pythonpath  = os.path.abspath(basepath + '/../Python')     # for dev ASP
libexecpath = os.path.abspath(basepath + '/../libexec')    # for packaged ASP
sys.path.insert(0, basepath) # prepend to Python path
sys.path.insert(0, pythonpath)
sys.path.insert(0, libexecpath)

import asp_system_utils, asp_alg_utils, asp_geo_utils
asp_system_utils.verify_python_version_is_supported()

# Prepend to system PATH
os.environ["PATH"] = libexecpath + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = basepath    + os.pathsep + os.environ["PATH"]

def main(argsIn):

    try:
        usage = '''usage: process_calibration_file.py file.pdf'''
                      
        parser = optparse.OptionParser(usage=usage)
        (options, args) = parser.parse_args(argsIn)
    except optparse.OptionError, msg:
        raise Usage(msg)


    if len(args) != 1:
        raise Exception(usage)
    
    # The input is a pdf file
    inFile = args[0]
    base, ext = os.path.splitext(inFile)
    if ext != ".pdf":
        raise Exception("The input file must be in pdf format.")
    if not os.path.isfile(inFile):
        raise Exception("Missing input file: " + inFile + ".")

    # Convert from pdf to text
    pdfToTextExec = asp_system_utils.which("pdftotext")
    cmd = pdfToTextExec + ' ' + inFile 
    print(cmd)
    p = subprocess.Popen(cmd, shell=True)
    os.waitpid(p.pid, 0)
    
    txtFile = base + '.txt'
    if not os.path.isfile(txtFile):
        raise Exception("Missing file: " + txtFile + ".")

    tsaiFile = base + '.tsai'
    
    # Read the lines
    lines = []
    fin = open(txtFile, 'r')
    while True:
        line = fin.readline()
        if not line: break
        line = line.strip()
        if line == "":
            continue
        lines.append(line)
    fin.close()
    
    width, height, f, xp, yp, pitch_x, pitch_y, k1, k2, k3, p1, p2, b1, b2 = \
           (-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)

    # Strip the mm
    count = 0
    for line in lines:
        while True:
            m = re.match("^(.*?\d)mm(.*?)$", line)
            if not m:
                break
            line = m.group(1) + m.group(2)
            lines[count] = line
        count += 1
        #print("line is ", line)
     
    # Parse the lines
    count = 0
    for line in lines:

        m = re.match("^.*?Resolution\s*=\s*(\d+)\s*x\s*(\d+)", line) 
        if m and width == -1 and height == -1:
            width = m.group(1)
            height = m.group(2)

        m = re.match("^.*?Pixel width\s*=\s*(.*?),\s*Pixel height\s*=\s*(.*?)\s*$", line)
        if m and pitch_x == -1 and pitch_y == -1:
            pitch_x = m.group(1)
            pitch_y = m.group(2)
            if pitch_x != pitch_y:
            	raise Exception ("Expecting pixel width and height to be same.")

        m = re.match(".*?c\s*=", line)
        if m and f == -1:
            f = lines[count+3]
            
        m = re.match(".*?xp\s*=", line)
        if m and  xp == -1:
            xp = lines[count+1]
            
        m = re.match(".*?yp\s*=", line)
        if m and  yp == -1:
            yp = lines[count+1]
        
        m = re.match(".*?K1\s*=\s*(.*?)\s*$", line)
        if m and  k1 == -1:
            k1 = m.group(1)
            
        m = re.match(".*?K2\s*=\s*(.*?)\s*$", line)
        if m and  k2 == -1:
            k2 = m.group(1)

        m = re.match(".*?K3\s*=\s*(.*?)\s*$", line)
        if m and  k3 == -1:
            k3 = m.group(1)

        m = re.match(".*?P1\s*=\s*(.*?)\s*$", line)
        if m and  p1 == -1:
            p1 = m.group(1)

        m = re.match(".*?P2\s*=\s*(.*?)\s*$", line)
        if m and  p2 == -1:
            p2 = m.group(1)

        m = re.match(".*?B1\s*=\s*(.*?)\s*$", line)
        if m and  b1 == -1:
            b1 = m.group(1)

        m = re.match(".*?B2\s*=\s*(.*?)\s*$", line)
        if m and  b2 == -1:
            b2 = m.group(1)

        count += 1
        
    cu = float(width)  * float(pitch_x) / 2.0
    cv = float(height) * float(pitch_y) / 2.0

    print ("Writing: " + tsaiFile)
    fout = open(tsaiFile, 'w')
    fout.write("VERSION_3\n")
    fout.write("fu = " + f + "\n")
    fout.write("fv = " + f + "\n")
    fout.write("cu = " + repr(cu) + "\n")
    fout.write("cv = " + repr(cv) + "\n")
    fout.write("u_direction = 1  0  0\n")
    fout.write("v_direction = 0  1  0\n")
    fout.write("w_direction = 0  0  1\n")
    fout.write("C = 0 0 0\n")
    fout.write("R = 1 0 0 0 1 0 0 0 1\n")
    fout.write("pitch = " + pitch_x + "\n")
    fout.write("Photometrix\n")
    fout.write("xp = " + xp + "\n")
    fout.write("yp = " + yp + "\n")
    fout.write("k1 = " + k1 + "\n")
    fout.write("k2 = " + k2 + "\n")
    fout.write("k3 = " + k3 + "\n")
    fout.write("p1 = " + p1 + "\n")
    fout.write("p2 = " + p2 + "\n")
    fout.write("b1 = " + b1 + "\n")
    fout.write("b2 = " + b2 + "\n")
    fout.close()
    
    # Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))


