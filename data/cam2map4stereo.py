#!/usr/bin/env python
# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import os, glob, optparse, re, shutil, subprocess, sys, string;
from subprocess import CalledProcessError

def man(option, opt, value, parser):
    print >>sys.stderr, parser.usage
    print >>sys.stderr, '''\
This program takes similar arguments as the ISIS3 cam2map program,
but takes two input images.  With no arguments, the program determines
the minimum overlap of the two images, and the worst common resolution,
and then map-projects the two images to this identical area and resolution.

If you want to change the default output file naming scheme, you'll have to
edit the code, but you'll only have to change the mapfile routine.
'''
    sys.exit()

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

class MapExists(Exception):
    def __init__(self, msg):
        self.msg = msg

class ImgInfo:
    def __init__( self ):
        self.resolution = None
        self.minlat = None
        self.maxlat = None
        self.minlon = None
        self.maxlon = None

def mapfile( cube ):
    items = cube.split('.')
    mapname = items[0] + '.map.cub'

    if( os.path.exists(mapname) ):
        items.insert(len(items)-1, 'map')
        mapname = '.'.join(items)

        if( os.path.exists(mapname) ):
            raise MapExists( mapname )

    return mapname
	

def camrange( cube ):
    # run camrange on cube
    tmpfile = 'cam2map4stereo.tmp.txt'
    info = ImgInfo();
    cmd = 'camrange from= '+ cube +' to= '+ tmpfile
    try:
        print 'Running ' + cmd
        subprocess.check_call(cmd, shell=True)

        # extract information
        info.resolution = subprocess.Popen(["getkey", "from= "+ tmpfile, "grpname= PixelResolution", "keyword= Lowest"], stdout=subprocess.PIPE).communicate()[0].strip()
        info.minlat = subprocess.Popen(["getkey", "from= "+ tmpfile, "grpname= UniversalGroundRange", "keyword= MinimumLatitude"], stdout=subprocess.PIPE).communicate()[0].strip()
        info.maxlat = subprocess.Popen(["getkey", "from= "+ tmpfile, "grpname= UniversalGroundRange", "keyword= MaximumLatitude"], stdout=subprocess.PIPE).communicate()[0].strip()
        info.minlon = subprocess.Popen(["getkey", "from= "+ tmpfile, "grpname= UniversalGroundRange", "keyword= MinimumLongitude"], stdout=subprocess.PIPE).communicate()[0].strip()
        info.maxlon = subprocess.Popen(["getkey", "from= "+ tmpfile, "grpname= UniversalGroundRange", "keyword= MaximumLongitude"], stdout=subprocess.PIPE).communicate()[0].strip()

        os.remove( tmpfile )

    except CalledProcessError, e:
        print >>sys.stderr, "Non-zero return code ("+ str(e.returncode) +"): ", e
        raise e
    except OSError, e:
        print >>sys.stderr, "Execution failed:", e
        raise e

    return info 
    

def main():
    try:
        try:
            usage = "usage: cam2map4stereo.py [--help][--manual][--map mapfile][--pixres CAMERA|MAP|MPP|PPD][--resolution float][--interp NN|BI|CC][--lat min:max][--lon min:max] image1.cub image2.cub "
            parser = optparse.OptionParser(usage=usage)
            parser.set_defaults(dryrun=False)
            parser.set_defaults(pixres='MPP')
            parser.add_option("--manual", action="callback", callback=man,
                              help="Read the manual.")
            parser.add_option("-m", "--map", dest="map",
                              help="The mapfile to use for cam2map.")
            parser.add_option("-p", "--pixres", dest="pixres",
                              help="The pixel resolution mode to use for cam2map.")
            parser.add_option("-r", "--resolution", dest="resolution", 
                              help="Resolution of final map for cam2map.")
            parser.add_option("-i", "--interp", dest="interp", 
                              help="Pixel interpolation scheme for cam2map.")
            parser.add_option("-a", "--lat", dest="lat", 
                              help="Latitude range for cam2map.")
            parser.add_option("-o", "--lon", dest="lon", 
                              help="Longitude range for cam2map.")
            parser.add_option("-n", "--dry-run", dest="dryrun", 
                              action="store_true", 
                              help="Make calculations, but print cam2map command, but don't actually run it.")

            (options, args) = parser.parse_args()

            if not args: parser.error("need .IMG files")


        except optparse.OptionError, msg:
            raise Usage(msg)

        image1 = camrange( args[0] )
        image2 = camrange( args[1] )

        mapout = ImgInfo()
        
        mapout.minlat = max( image1.minlat, image2.minlat )
        mapout.maxlat = min( image1.maxlat, image2.maxlat )
        mapout.minlon = max( image1.minlon, image2.minlon )
        mapout.maxlon = min( image1.maxlon, image2.maxlon )
        mapout.resolution = max( image1.resolution, image2.resolution )

        if( options.resolution ): mapout.resolution = options.resolution
        if( options.lat ):
            latrange = options.lat.split(':')
            if latrange[0]: mapout.minlat = latrange[0]
            if latrange[1]: mapout.maxlat = latrange[1]
        if( options.lon ):
            lonrange = options.lon.split(':')
            if lonrange[0]: mapout.minlon = lonrange[0]
            if lonrange[1]: mapout.maxlon = lonrange[1]
        
        # call cam2map with the arguments

        cam2map = ['cam2map', 'from=' + args[0], 'to='+ mapfile( args[0] )]

        if( options.map ):
            cam2map.append( 'map=' + options.map )
        if( options.interp):
            cam2map.append( 'interp=' + options.interp)

        cam2map.append( 'pixres=' + options.pixres )
        cam2map.append( 'resolution=' + mapout.resolution )

        cam2map.append( 'defaultrange=MAP' )
        cam2map.append( 'minlat=' + mapout.minlat )
        cam2map.append( 'maxlat=' + mapout.maxlat )
        cam2map.append( 'minlon=' + mapout.minlon )
        cam2map.append( 'maxlon=' + mapout.maxlon )
    
        # Run for first image 

        # Need to put these together to keep ISIS from calling the GUI
        cam2map_cmd = ' '.join(cam2map)
        if( options.dryrun ):
            print cam2map_cmd
        else:
            subprocess.check_call(cam2map_cmd, shell=True)

        # Run for second image  
        cam2map[1] = 'from=' + args[1]
        cam2map[2] = 'to='+ mapfile( args[1] )
        cam2map_cmd = ' '.join(cam2map)
        if( options.dryrun ):
            print cam2map_cmd
        else:
            subprocess.check_call(cam2map_cmd, shell=True)
        
        print "Finished"
        return 0

    except Usage as err:
        print >>sys.stderr, err.msg
        # print >>sys.stderr, "for help use --help"
        return 2

    except MapExists as e:
        print >>sys.stderr, 'The file '+ e.msg +' already exists, delete first.'
        return 3

    # # To more easily debug this program, comment out this catch block.
    # except Exception, err:
    #     sys.stderr.write( str(err) + '\n' )
    #     return 1

if __name__ == "__main__":
    sys.exit(main())
