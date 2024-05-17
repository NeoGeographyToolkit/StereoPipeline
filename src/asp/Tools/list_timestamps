#!/usr/bin/env python

"""
For each message in the bag having a header, add to the output list
the header timestamp (in double-precision seconds since epoch) and the
topic name. Do this for all topics in the bag or for those passed on
input.
"""

import argparse, os, re, sys,rosbag
  
if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--bag", dest = "bag", help="Input bag."),
    parser.add_argument("--list", dest = "list", default="",
                        help="Output list having a timestamp and topic for each message.")
    parser.add_argument("--topics", dest="topics", default = "",  
                        help="Print the timestamps only for these topics.")

    args = parser.parse_args()

    if args.bag == "" or args.list == "":
        print("Must specify the input bag and the output list.")
        sys.exit(1)

    topic_list = None
    if args.topics != "":
        topic_list = args.topics.split()

    print("Listing timestamps in: " + args.bag)
    print("This can be slow for large bags.")
    print("Writing: " + args.list)
    with open(args.list, "w") as fh:
        with rosbag.Bag(args.bag, "r") as bag:
            for topic, msg, t in bag.read_messages(topic_list):
                try:
                    # The timestamp format is consistent with what rig_calibrator uses
                    fh.write("{0:10.7f} {1:s}\n".format(msg.header.stamp.to_sec(), topic))
                except:
                    pass
                
