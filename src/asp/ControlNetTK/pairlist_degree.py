#!/usr/bin/env python
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__

import optparse, sys, subprocess, math;
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

        image_lat = dict()
        image_lon = dict()

        for i in range(0,len(args)):
            cmd = "camrange from="+args[i];
            p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
            cmd_return = p.stdout.readlines();
            group = ""
            min_lat = ""
            max_lat = ""
            min_lon = ""
            max_lon = ""
            for line in cmd_return:
                if line.find("Group = ") >= 0:
                    group = line.split()[2]
                if line.find("MinimumLatitude") >= 0 and group == "UniversalGroundRange":
                    min_lat = line.split()[2]
                if line.find("MaximumLatitude") >= 0 and group == "UniversalGroundRange":
                    max_lat = line.split()[2]
                if line.find("MinimumLongitude") >= 0 and group == "UniversalGroundRange":
                    min_lon = line.split()[2]
                if line.find("MaximumLongitude") >= 0 and group == "UniversalGroundRange":
                    max_lon = line.split()[2]
            image_lat[args[i]] = (float(max_lat)-float(min_lat))/2;
            image_lon[args[i]] = (float(max_lon)-float(min_lon))/2;

        # Performing second pass to figure out what images appear to
        # be side by side.
        for i in range(0,len(args)-1):
            for j in range(i+1,len(args)):
                angle = math.degrees(angle_diff(math.radians(image_lat[args[i]]), math.radians(image_lat[args[j]]), math.radians(image_lon[args[i]]), math.radians(image_lon[args[j]])))
                if ( angle < options.angle ):
                    if ""==options.ext:
                        print args[i]+" "+args[j]
                    else:
                        print replace_extension(args[i],options.ext)+" "+replace_extension(args[j],options.ext)

    except Usage, err:
        print >>sys.stderr, err.msg
        # print >>sys.stderr, "for help use --help"
        return 2

    # To more easily debug this program, comment out this catch block.
    #except Exception, err:
    #    sys.stderr.write( str(err) + '\n' )
    #    return 1


if __name__ == "__main__":
    sys.exit(main())
