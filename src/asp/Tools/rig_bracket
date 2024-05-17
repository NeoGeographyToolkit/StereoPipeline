#!/usr/bin/env python

"""
Given a list of ref cam timestamps, one per line, grouped in pairs,
with each pair very close in time, forming a bracket, and a second
list of timestamps for another camera, for each ref cam bracket pick a
timestamp from the second camera list. A timestamp is a float number
in the form <digits>.<digits>. Other data on each line having a timestamp
will be read as well. The output list will have a subset
of lines from the other list.
Can handle correctly the case when both input lists are identical,
when bracketing needs some care, as each timestamp is its own bracket.
"""

import argparse, os, re, sys, rosbag, cv2, sensor_msgs, cv_bridge
from sensor_msgs import point_cloud2
import numpy as np

def read_timestamps(filename):
    """Return a list of timestamps and lines corresponding to them."""

    timestamps_map = {}
    with open(filename, 'r') as f:
        lines = f.readlines()

    for line in lines:
        m = re.match("^.*?(\d+\.\d+).*?\n", line)
        if m:
            timestamp = float(m.group(1))
            timestamps_map[timestamp] = line

    # Return arrays of timestamps and lines, in sorted order of timestamp.
    # Those are easier to parse.
    timestamps = sorted(timestamps_map)
    lines = []
    for t in timestamps:
        lines.append(timestamps_map[t])

    return (timestamps, lines)

parser = argparse.ArgumentParser(description = __doc__,
                                 formatter_class = argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("--ref_list", dest = "ref_list", help = "List of bracketing reference camera timestamps."),
parser.add_argument("--other_list", dest = "other_list", default = "",  
                    help = "List of timestamps for another camera. Pick one of those between each ref cam bracketing timestamps.")
parser.add_argument("--out_list", dest = "out_list", default = "",
                    help = "Output list of timestamps in other camera bracketed by ref cam.")

parser.add_argument("--bracket_len", dest = "bracket_len", type = float, default = -1,
                    help = "Maximum distance in time, in seconds, between two ref cam timestamps forming a bracket.")

args = parser.parse_args()

# Sanity checks
if args.ref_list == "" or args.other_list == "" or args.out_list == "":
    print("Not all arguments were specified.")
    sys.exit(1)
if args.bracket_len <= 0:
    print("The bracket length must be positive.")
    sys.exit(1)
    
(ref_timestamps, ref_lines) = read_timestamps(args.ref_list)
(other_timestamps, other_lines) = read_timestamps(args.other_list)

# Advance simultaneously in both sorted arrays, for effeciency
other_count = 0

found_it = []
for ref_count in range(len(ref_timestamps)):

    ref_count2 = ref_count + 1
    if ref_count == len(ref_timestamps) - 1:
        # Handle the case when we are at the end
        ref_count2 = ref_count

    t1 = ref_timestamps[ref_count]
    t2 = ref_timestamps[ref_count2]
    
    # Search for t such that t1 <= t < t2. When last t1 and t2 are reached,
    # can have t == t2.
    for it in range(other_count, len(other_timestamps)):
        t = other_timestamps[it]
        if t < t1:
            other_count = it + 1 # surely next time need to start searching further
            continue # too early
        if t > t2:
            break # there is no point in continuing
        
        # Now t1 <= t <= t2
        if (t == t1) or (t2 - t1 <= args.bracket_len and t < t2) or \
               (t == t2 and ref_count == len(ref_timestamps) - 1):
            found_it.append(it)
            other_count = it + 1
            break

# Save the lines corresponding to bracketed timestamps
print("Writing: " + args.out_list)
with open(args.out_list, 'w') as f:
    for it in found_it:
        f.write(other_lines[it])
