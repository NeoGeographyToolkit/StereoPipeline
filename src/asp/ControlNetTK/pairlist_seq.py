#!/usr/bin/env python
# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import optparse, sys

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main():
    try:
        try:
            usage = "usage: pairlist_seq.py [--help] <image files>"
            parser = optparse.OptionParser(usage=usage)
            parser.set_defaults(overlap_count=1)
            parser.add_option("-c","--overlap-count", dest="overlap_count",
                              help="The sequential overlap amount to make pairs.",
                              type="int")

            (options,args) = parser.parse_args()

            if not args: parser.error("need input files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        for i in range(0,len(args)-1):
            for j in range(1,options.overlap_count):
                if ( i+j < len(args) ):
                    print args[i]+" "+args[i+j]

    except Usage, err:
        print >>sys.stderr, err.msg
        # print >>sys.stderr, "for help use --help"
        return 2

    # To more easily debug this program, comment out this catch block.
    except Exception, err:
        sys.stderr.write( str(err) + '\n' )
        return 1


if __name__ == "__main__":
    sys.exit(main())
