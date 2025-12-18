#!/usr/bin/env python3

# Take a csm linescan json file, and make it work for an image with the
# resolution that is 2x finer. This expects as input the number of columns
# (samples) and rows (lines) in the output image (which are roughly 2x the width
# and height specified in the input camera that is to be modified).

# Usage: scale_linescan.py <in.json> <out.json> <out samples> <out lines>

# This is a research script that is not officially supported.

import sys, json, os, re
import numpy as np

def read_json(file):

    # Read into string
    print("Reading: ", file)
    with open(file, 'r') as f:
        data = f.read()

    # Find first occurrence of open brace. This is needed because the CSM
    # state has some text before the JSON object.
    pos = data.find('{')

    # Do substring from pos to the end, if pos was found
    header = ""
    if pos != -1:
        header = data[:pos] # save the header
        data = data[pos:]
        
    # parse the json from data
    j = json.loads(data)

    # # Print all keys in the json
    # print("will print all keys in the json")
    # # for key in j.keys():
    # #     print(key)
    
    return (header, j)

def multiply(j, key, factor):
    """
    Multiply the values in the list j[key] by factor. j gets modified in place.
    """
    if key in j:
      if isinstance(j[key], list):
        j[key] = [x * factor for x in j[key]]
      elif isinstance(j[key], (int, float)):
        j[key] *= factor
    else:
      raise KeyError(f"Key '{key}' not found or is not a list in the JSON object.")
  
def process_json(j, outSamples, outLines):

    j["m_nSamples"] = outSamples
    j["m_nLines"] = outLines
    
    multiply(j, "m_iTransS", 2.0)
    multiply(j, "m_iTransL", 2.0)
    multiply(j, "m_detectorLineOrigin", 2.0)
    multiply(j, "m_detectorSampleOrigin", 2.0)

    # These need to be divided by 2.0
    multiply(j, "m_intTimes", 0.5)
    multiply(j, "m_gsd", 0.5)
    
def write_json(file, header, j):

    # Convert the json to string
    data = json.dumps(j, indent = 2, sort_keys=True)
    
    # Create the directory having the output file, if it does not exist
    outDir = os.path.dirname(file)
    if not os.path.exists(outDir):
        os.makedirs(outDir)

    # Write the header and the json data to the file
    print("Writing: ", file)
    with open(file, 'w') as f:
        f.write(header + data + "\n")

# Check if we have at least one input file
if len(sys.argv) != 5:
    print("Usage: scale_linescan.py <in.json> <out.json> <out samples> <out lines>")
    sys.exit(1)

# Load the first file
inFile = sys.argv[1]
outFile = sys.argv[2]
outSamples = int(sys.argv[3])
outLines = int(sys.argv[4])

header, j = read_json(inFile)
process_json(j, outSamples, outLines)
write_json(outFile, header, j)

