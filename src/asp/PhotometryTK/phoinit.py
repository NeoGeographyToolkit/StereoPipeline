#!/usr/bin/env python
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import os, optparse, multiprocessing, sys;

class Usage(Exception):
    def __init__(self,msg):
        self.msg = msg

def main():
    try:
        try:
            usage = "phoinit.py [--help] not-sure-yet "
            parser = optparse.OptionParser(usage=usage);

            (options, args) = parser.parse_args()

            if not args: parser.error("missing files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        return 0

    except Usage, err:
        print >>sys.stderr, err.msg
        return 2

    except Exception, err:
        sys.stderr.write( str(err) + '\n' )
        return 1

if __name__ == "__main__":
    sys.exit(main())

