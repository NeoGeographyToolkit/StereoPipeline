#!/usr/bin/env python2.6
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import os, optparse, multiprocessing, sys, subprocess;
from multiprocessing import Pool

def job_func(cmd):
    os.system(cmd)
    return cmd

class Usage(Exception):
    def __init__(self,msg):
        self.msg = msg

def main():
    try:
        try:
            usage = "pmipmap.py [--help][--threads N][--levels_per_job N] plate-url "
            parser = optparse.OptionParser(usage=usage);
            parser.set_defaults(threads=4)
            parser.set_defaults(levels=8)
            parser.set_defaults(iterations=100)
            parser.add_option("-t", "--threads", dest="threads",
                              help="Number of threads to use",type="int")
            parser.add_option("-l", "--levels_per_job", dest="levels",
                              help="Lowest level to process at",type="int")

            (options, args) = parser.parse_args()

            if not args: parser.error("missing files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        pool = Pool(processes=options.threads)

        # First call the server .. see if the plate file still exists
        # and how many levels.
        filename = args[0][args[0].rfind("/")+1:]
        basename = args[0][:args[0].rfind("/")]
        print filename
        print basename
        p = subprocess.Popen("index_client -u "+basename,
                             shell=True,stdout=subprocess.PIPE)
        cmd_return = p.stdout.readlines()
        max_level = 0;
        for line in cmd_return:
            if line.find("Platefile:") >= 0:
                if line.find(filename) >= 0:
                    max_level = int(line[line.find("MaxLevel[")+9:line.rfind("]")])
                    break
        print "Found Max Level: "+str(max_level)

        # Now start solving for the number of jobs.
        for l in range(max_level,0,-(options.levels)):
            length = 2**l;
            job_length = 2**(options.levels)
            cmds = []
            for i in range(0,length,job_length):
                for j in range(0,length,job_length):
                    cmd = "mipmap --region "+str(i)+","+str(j)+":"+str(i+job_length-1)+","+str(j+job_length-1)+"@"+str(l)+" --top-level "+str(l-options.levels)+" "+args[0]
                    cmds.append( cmd )
            results = [pool.apply_async(job_func, (cmd,)) for cmd in cmds]
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


