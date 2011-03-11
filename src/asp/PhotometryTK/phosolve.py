#!/usr/bin/env python2.6
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import os, optparse, multiprocessing, sys;
from multiprocessing import Pool

def job_func(cmd):
    os.system(cmd);
    return cmd;

class Usage(Exception):
    def __init__(self,msg):
        self.msg = msg

def main():
    try:
        try:
            usage = "phosolve.py [--help][--threads N][--level N] ptk-url "
            parser = optparse.OptionParser(usage=usage);
            parser.set_defaults(threads=2)
            parser.set_defaults(level=-1)
            parser.set_defaults(iterations=100)
            parser.add_option("-t", "--threads", dest="threads",
                              help="Number of threads to use",type="int")
            parser.add_option("-l", "--level", dest="level",
                              help="Lowest level to process at",type="int")
            parser.add_option("-i", "--iter", dest="iterations",
                              help="Max iterations to perform",type="int")

            (options, args) = parser.parse_args()

            if not args: parser.error("missing files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        pool = Pool(processes=options.threads)

        # Finding URL for Albedo
        albedo_url = args[0][:args[0].rfind("/")]+"_index/Albedo.plate"

        for iteration in range(options.iterations):
            # Perform Albedo
            print " --- ALBEDO ---"
            albedo_cmd = []
            for i in range(2*options.threads):
                cmd = "phoitalbedo "
                if ( options.level > 0 ):
                    cmd = cmd + "-l "+str(options.level)+" "
                cmd = cmd + "-j "+str(i)+" -n "+str(2*options.threads)+" "+args[0]
                albedo_cmd.append( cmd )
            results = [pool.apply_async(job_func, (cmd,)) for cmd in albedo_cmd]
            for result in results:
                result.get()

            # Mipmap up the levels
            print " --- MIPMAP ---"
            os.system("mipmap "+albedo_url)

            # Update the Time Estimate
            time_cmd = []
            print " --- TIME ---"
            for i in range(options.threads):
                cmd = "phoittime "
                if ( options.level > 0 ):
                    cmd = cmd + "-l "+str(options.level-2)+" "
                cmd = cmd + "-j "+str(i)+" -n "+str(options.threads)+" "+args[0]
                time_cmd.append(cmd)
            results = [pool.apply_async(job_func, (cmd,)) for cmd in time_cmd]
            for result in results:
                result.get()

    except Usage, err:
        print >>sys.stderr, err.msg
        return 2

    except Exception, err:
        sys.stderr.write( str(err) + '\n' )
        return 1

if __name__ == "__main__":
    sys.exit(main())

