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


import optparse, sys, subprocess, math, os;
from math import cos, sin;
from multiprocessing import Pool

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def replace_extension( filename, ext ):
    i = filename.rfind('.')
    return filename[0:i+1]+ext

def extract_camera_position_func(cmd):
    p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
    cmd_return = p.stdout.readline().strip();

    x = float(cmd_return.split()[0])
    y = float(cmd_return.split()[1])
    z = float(cmd_return.split()[2])
    length = math.sqrt( x*x + y*y + z*z )

    return [cmd.split()[1], x/length, y/length, z/length]

def main():
    try:
        try:
            usage = "usage: pairlist_degree.py [--help] <image files>"
            parser = optparse.OptionParser(usage=usage)
            parser.set_defaults(angle=6)
            parser.set_defaults(ext="")
            parser.set_defaults(threads=4)
            parser.add_option("-a", "--angle", dest="angle",
                              help="Max degree seperation between images.", type="float")
            parser.add_option("--iextension", dest="ext",
                              help="Output extension to use.", type="string")
            parser.add_option("-t", "--threads", dest="threads",
                              help="Number of threads to use.", type="int")

            (options,args) = parser.parse_args()

            if not args: parser.error("need input files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        isis_position_extract = os.path.realpath(__file__)
        isis_position_extract = isis_position_extract[:isis_position_extract.rfind("/")] + "/../libexec/isis_position_extract"
        vw_position_extract   = os.path.realpath(__file__)
        vw_position_extract   = vw_position_extract[:vw_position_extract.rfind("/")] + "/../libexec/vw_position_extract"

        image_position = dict()
        position_extract_cmds = []
        step_size = 1;

        args.sort() # Make sure the isis_adjust are correctly placed

        # Building position extraction commands
        if ( args[0].rfind(".cub") >= 0 ):
            if ( args[1].rfind(".isis_adjust") >= 0 ):
                step_size = 2;
            sys.stderr.write("Found %d cameras." % (len(args)/step_size))
            for i in range(0,len(args),step_size):
                # Fetching the Lat Long of the center pixel - We could at
                # some point fetch the corners and do something more
                # interesting
                cmd = isis_position_extract+" "+args[i];
                if ( step_size > 1):
                    cmd = cmd+" "+args[i+1]
                position_extract_cmds.append(cmd)
        else:
            sys.stderr.write("Found %d cameras." % len(args))
            for i in args:
                cmd = vw_position_extract+" "+i
                position_extract_cmds.append(cmd);

        # Processing extraction commands
        pool = Pool(processes=options.threads)
        results = [pool.apply_async(extract_camera_position_func, (cmd,)) for cmd in position_extract_cmds]
        for result in results:
            output = result.get()
            image_position[output[0]] = [output[1], output[2], output[3]]

        # Performing second pass to figure out what images appear to
        # be side by side.
        for i in range(0,len(args)-step_size,step_size):
            for j in range(i+step_size,len(args),step_size):

                left = image_position[args[i]]
                righ = image_position[args[j]]

                dot_prod = left[0] * righ[0] + left[1] * righ[1] + left[2] * righ[2]
                angle = math.acos(dot_prod) * 180 / math.pi

                if ( angle < options.angle ):
                    if ""==options.ext:
                        print args[i]+" "+args[j]
                    else:
                        print replace_extension(args[i],options.ext)+" "+replace_extension(args[j],options.ext)

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

    # To more easily debug this program, comment out this catch block.

    # except Exception, err:
    #     sys.stderr.write( str(err) + '\n' )
    #     return 1


if __name__ == "__main__":
    sys.exit(main())
