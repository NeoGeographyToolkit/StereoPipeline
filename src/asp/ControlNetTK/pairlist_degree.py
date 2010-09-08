#!/usr/bin/env python
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__

import optparse, sys, subprocess, math, os;
from math import cos, sin;

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def replace_extension( filename, ext ):
    i = filename.rfind('.')
    return filename[0:i+1]+ext

def angle_diff( lat_s, lat_f, lon_s, lon_f ):
    lon_diff = lon_f-lon_s;
    sin_lon_diff = sin(lon_diff);
    cos_lon_diff = cos(lon_diff);

    cos_lat_f = cos(lat_f)
    sin_lat_f = sin(lat_f)
    cos_lat_s = cos(lat_s)
    sin_lat_s = sin(lat_s)

    top_left = cos_lat_f*sin_lon_diff;
    top_right = cos_lat_s*sin_lat_f-sin_lat_s*cos_lat_f*cos_lon_diff
    top = math.sqrt(top_left*top_left+top_right*top_right)
    bot = sin_lat_s*sin_lat_f+cos_lat_s*cos_lat_f*cos_lon_diff;

    adiff = math.atan2(top,bot)
    return adiff

def angle_diffd( lat_s, lat_f, lon_s, lon_f ):
    return math.degrees(angle_diff(math.radians(lat_s),
                                   math.radians(lat_f),
                                   math.radians(lon_s),
                                   math.radians(lon_f)))

def main():
    try:
        try:
            usage = "usage: pairlist_degree.py [--help] <image files>"
            parser = optparse.OptionParser(usage=usage)
            parser.set_defaults(angle=6)
            parser.set_defaults(ext="")
            parser.add_option("-a", "--angle", dest="angle",
                              help="Max degree seperation between images.", type="float")
            parser.add_option("--iextension", dest="ext",
                              help="Output extension to use.", type="string")

            (options,args) = parser.parse_args()

            if not args: parser.error("need input files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        isis_position_extract = os.path.realpath(__file__)
        isis_position_extract = isis_position_extract[:isis_position_extract.rfind("/")] + "/../libexec/isis_position_extract"

        image_lat = dict()
        image_lon = dict()

        step_size = 1;
        if ( args[1].rfind(".isis_adjust") >= 0 ):
            step_size = 2;
        for i in range(0,len(args),step_size):
            # Fetching the Lat Long of the center pixel - We could at
            # some point fetch the corners and do something more
            # interesting
            latitude = ""
            longitude = ""
            cmd = isis_position_extract+" "+args[i];
            if ( step_size > 1):
                cmd = cmd+" "+args[i+1]
            p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
            cmd_return = p.stdout.readline().strip();

            image_lat[args[i]] = float(cmd_return.split()[1])
            image_lon[args[i]] = float(cmd_return.split()[0])

        # Performing second pass to figure out what images appear to
        # be side by side.
        for i in range(0,len(args)-1):
            for j in range(i+1,len(args)):
                angle = angle_diffd(image_lat[args[i]],
                                    image_lat[args[j]],
                                    image_lon[args[i]],
                                    image_lon[args[j]])
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
    except Exception, err:
        sys.stderr.write( str(err) + '\n' )
        return 1


if __name__ == "__main__":
    sys.exit(main())
