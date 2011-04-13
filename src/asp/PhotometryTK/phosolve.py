#!/usr/bin/env python2.6
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import os, optparse, multiprocessing, sys, subprocess, shlex;
from multiprocessing import Pool

def job_func(cmd,printStdOut=True):
    #print "Running [", cmd, "]\n"
    args = shlex.split(cmd)
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    print proc.stderr.read()
    if (printStdOut):
        print proc.stdout.read()
    else:
        return proc.stdout.read()

def halt_iterations(initErr, lastErr, currErr):
    # Determine halting
    default_error_delta_threshold = 0.01
    curr_delta = currErr - lastErr
    if ( curr_delta <= default_error_delta_threshold ):
        return true
    else:
        return false

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
            parser.add_option('-n', action='store_true', dest="norm",
                              help="Normalize at the end of this call",default=False)

            (options, args) = parser.parse_args()

            if not args: parser.error("missing files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        pool = Pool(processes=options.threads)

        # Finding URL for Albedo
        albedo_url = args[0][:args[0].rfind("/")]+"_index/Albedo.plate"

        levelArg = ""
        if ( options.level > 0 ):
            levelArg = "-l %d" % (options.level)

        for iteration in range(options.iterations):
            # Perform Albedo
            print " --- ALBEDO ---"
            albedo_cmd = []
            for i in range(2*options.threads):
                cmd = "phoitalbedo %s -j %d -n %d %s" % (levelArg, i, 2*options.threads, args[0])
                albedo_cmd.append( cmd )
            results = [pool.apply_async(job_func, (cmd,)) for cmd in albedo_cmd]
            for result in results:
                result.get()

            # Mipmap up the levels
            # Need to add region arguments based on level
            #
            tiles_per_side = 2**options.level
            region_arg = "0,0:%d,%d@%d" % (tiles_per_side, tiles_per_side, options.level)

            print " --- MIPMAP ---"
            job_func("mipmap --region "+region_arg+" "+albedo_url)

            # Update the error
            error_cmd = []
            print " --- ERROR ---"
            for i in range(options.threads):
                cmd = "phoiterror %s -j %d -n %d %s" % (levelArg, i, options.threads, args[0])
                error_cmd.append(cmd)
            results = [pool.apply_async(job_func, (cmd,)) for cmd in error_cmd]
            for result in results:
                result.get()

            # Run error again with zero jobs, just to output totals for the entire plate
            errvals = job_func("phoiterror -l %d -j 0 -n 0 %s" % (options.level, args[0]), false)
            errvalList = errvals.split()
            initErr = float(errvalList[0])
            lastErr = float(errvalList[1])
            currErr = float(errvalList[2])
            print "ERROR VALS = [%s] [%s] [%s]\n" % (initErr, lastErr, currErr);

            if ( halt_iterations(initErr, lastErr, currErr) ):
                iteration = options.iterations + 1

            # Update the Time Estimate
            time_cmd = []
            print " --- TIME ---"
            for i in range(options.threads):
                cmd = "phoittime %s -j %d -n %d %s" % (levelArg, i, options.threads, args[0])
                time_cmd.append(cmd)
            results = [pool.apply_async(job_func, (cmd,)) for cmd in time_cmd]
            for result in results:
                result.get()

        if ( options.norm ):
            norm_cmd = []
            for i in range(options.threads):
                cmd = "phoitnorm -l %d -j %d -n %d %s" % (options.level, i, options.threads, args[0])
                norm_cmd.append(cmd)
            results = [pool.apply_async(job_func, (cmd,)) for cmd in norm_cmd]
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

