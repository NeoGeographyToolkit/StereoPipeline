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

        image_lat = dict()
        image_lon = dict()

        for i in range(0,len(args)):
            # First find out size of the image
            cmd = "catlab from="+args[i];
            p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
            cmd_return = p.stdout.readlines();
            group = ""
            samples = ""
            lines = ""
            for line in cmd_return:
                if line.find("Group = ") >= 0:
                    group = line.split()[2]
                if line.find("Samples") >= 0 and group == "Dimensions":
                    samples = line.split()[2]
                if line.find("Lines") >= 0 and group == "Dimensions":
                    lines = line.split()[2]
                    continue;
            center_sample = float(samples)/2;
            center_line =   float(lines)/2;

            # Fetching the Lat Long of the center pixel - We could at
            # some point fetch the corners and do something more
            # interesting
            latitude = ""
            longitude = ""
            cmd = "campt from="+args[i]+" line="+str(center_line)+" sample="+str(center_sample);
            p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
            cmd_return = p.stdout.readlines();
            for line in cmd_return:
                if line.find("PlanetocentricLatitude") >= 0:
                    latitude = line.split()[2]
                if line.find("PositiveWest360Longitude") >= 0:
                    longitude = line.split()[2]

            image_lat[args[i]] = float(latitude)
            image_lon[args[i]] = float(longitude)

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
