#!/usr/bin/env python
#******************************************************************************
#
#  Project:  GDAL
#  Purpose:  Use HTDP to generate PROJ.4 compatible datum grid shift files.
#  Author:   Frank Warmerdam, warmerdam@pobox.com
#
# See also: http://www.ngs.noaa.gov/TOOLS/Htdp/Htdp.shtml
#           http://trac.osgeo.org/proj/wiki/HTDPGrids
#
#******************************************************************************
#  Copyright (c) 2012, Frank Warmerdam <warmerdam@pobox.com>
#
#  Permission is hereby granted, free of charge, to any person obtaining a
#  copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense,
#  and/or sell copies of the Software, and to permit persons to whom the
#  Software is furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included
#  in all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#  DEALINGS IN THE SOFTWARE.
#******************************************************************************

import os
import numpy
import sys

from osgeo import gdal
from osgeo import gdal_array

# Input looks like this:
"""
  PNT_0_0
  LATITUDE     40 00  0.00000 N     40 00  0.02344 N        2.06 mm/yr  north
  LONGITUDE   117 00  0.00000 W    117 00  0.03765 W       -1.22 mm/yr  east
  ELLIP. HT.               0.000              -0.607 m     -1.34 mm/yr  up
  X                 -2221242.768        -2221243.142 m     -0.02 mm/yr
  Y                 -4359434.393        -4359433.159 m      2.64 mm/yr
  Z                  4077985.572         4077985.735 m      0.72 mm/yr

--> WRONG!  The input looks like this:

   49.9999969681  126.9999805726     0.187    "PNT_0_0"

"""

def next_point(fd):

    # Read until we find a valid line
    line = fd.readline().strip()
    while line != '' and line.find('PNT_') == -1:
        print line
        line = fd.readline()

    if line == '':
        return None

    tokens = line.strip().split()
    if len(tokens) != 4:
        raise Exception('Unexpected line: ' + line)

    lat = float(tokens[0])
    lon = float(tokens[1])

    name        = tokens[3].replace('"','')
    name_tokens = name.split('_')

    return (int(name_tokens[1]), int(name_tokens[2]), lat, lon)

    # Bad old code before here!

    # Read LATITUDE line
    line   = fd.readline().strip()
    tokens = line.split()
    print line
    lat_src = float(tokens[1]) + float(tokens[2])/60.0 + float(tokens[3])/3600.0
    lat_dst = float(tokens[5]) + float(tokens[6])/60.0 + float(tokens[7])/3600.0

    # Read LONGITUDE line
    line   = fd.readline().strip()
    tokens = line.split()

    lon_src = float(tokens[1]) + float(tokens[2])/60.0 + float(tokens[3])/3600.0
    lon_dst = float(tokens[5]) + float(tokens[6])/60.0 + float(tokens[7])/3600.0

    return (int(name_tokens[1]),int(name_tokens[2]),lat_src,lon_src,lat_dst,lon_dst)


def read_grid_crs_to_crs(input_filename, output_filename, shape):
    '''Read through the input/output files together to get data for
       the output grid.'''

    fd_in  = open(input_filename )
    fd_out = open(output_filename)
    grid   = numpy.zeros(shape)

    # Read/skip the output file header defining the transformation
    # - The input file does not have a header
    for i in range(5):
      print(fd_out.readline().rstrip())

    points_found = 0

    ptuple_in  = next_point(fd_in )
    ptuple_out = next_point(fd_out)
    while ptuple_out != None:

        lat_src = ptuple_in [2]
        lat_dst = ptuple_out[2]
        lon_src = ptuple_in [3]
        lon_dst = ptuple_out[3]

        lat_diff = lat_dst - lat_src
        lon_diff = lon_dst - lon_src
        #print (lat_diff, lon_diff)

        grid[0,ptuple_in[1],ptuple_in[0]] = lat_diff
        grid[1,ptuple_in[1],ptuple_in[0]] = lon_diff
        points_found = points_found + 1
        ptuple_in    = next_point(fd_in )
        ptuple_out   = next_point(fd_out)

    if points_found < shape[0] * shape[1]:
        print('points found:   ', points_found)
        print('points expected:', shape[1] * shape[2])
        sys.exit(1)

    return grid

###############################################################################
# This function creates a regular grid of lat/long values with one
# "band" for latitude, and one for longitude.
#
def new_create_grid( griddef ):

    lon_start = -1 * griddef[0] # HTDP uses positive-west longitude for some reason!
    lon_end   = -1 * griddef[2]

    lat_start = griddef[1]
    lat_end   = griddef[3]

    lon_steps = int(griddef[4])
    lat_steps = int(griddef[5])

    lat_axis = numpy.linspace(lat_start,lat_end,lat_steps)
    lon_axis = numpy.linspace(lon_start,lon_end,lon_steps)

    lon_list = []
    for i in range(lat_steps):
        lon_list.append(lon_axis)

    lon_band = numpy.vstack(lon_list)

    lat_list = []
    for i in range(lon_steps):
        lat_list.append(lat_axis)

    lat_band = numpy.column_stack(lat_list)

    return numpy.array([lon_band,lat_band])

##############################################################################
# This function writes a grid out in form suitable to use as input to the
# htdp program.
def write_grid(grid, out_filename):
    fd_out = open(out_filename,'w')
    for i in range(grid.shape[2]):
        for j in range(grid.shape[1]):
            fd_out.write('%f %f 0 "PNT_%d_%d"\n' % (grid[1,j,i],grid[0,j,i],i,j))
    fd_out.close()

##############################################################################
# Write the resulting grid out in GeoTIFF format.
def write_gdal_grid(filename, grid, griddef ):

    ps_x = (griddef[2] - griddef[0]) / (griddef[4]-1)
    ps_y = (griddef[3] - griddef[1]) / (griddef[5]-1)
    geotransform = (griddef[0] - ps_x*0.5, ps_x, 0.0,
                    griddef[1] - ps_y*0.5, 0.0, ps_y)

    grid = grid.astype(numpy.float32)
    ds = gdal_array.SaveArray( grid, filename, format='CTable2')
    ds.SetGeoTransform( geotransform )

#############################################################################

def write_control( control_fn, out_grid_fn, in_grid_fn,
                   src_crs_id, src_crs_date,
                   dst_crs_id, dst_crs_date ):

    # start_date, end_date should be something like "2011.0"

    control_template = """
4
%s
%d
%d
2
%s
2
%s
3
%s
0
0
"""

    control_filled = control_template % ( out_grid_fn,
                                          src_crs_id,
                                          dst_crs_id,
                                          src_crs_date,
                                          dst_crs_date,
                                          in_grid_fn )

    open(control_fn,'w').write(control_filled)

#############################################################################
def Usage( brief = 1 ):
    print("""
crs2crs2grid.py
        <src_crs_id> <src_crs_date> <dst_crs_id> <dst_crs_year>
        [-griddef <ul_lon> <ul_lat> <ll_lon> <ll_lat> <lon_count> <lat_count>]
        [-htdp <path_to_exe>] [-wrkdir <dirpath>] [-kwf]
        -o <output_grid_name>

 -griddef: by default the following values for roughly the continental USA
           at a six minute step size are used:
           -127 50 -66 25 251 611
 -kwf: keep working files in the working directory for review.

eg.
 crs2crs2grid.py 29 2002.0 8 2002.0 -o nad83_2002.ct2
 """)

    if brief == 0:
        print("""
The output file will be in CTable2 format suitable for use with PROJ.4
+nadgrids= directive.

Format dates like 2002.0 (for the start of 2002)

CRS Ids
-------
  1...NAD_83(2011) (North America tectonic plate fixed)
  29...NAD_83(CORS96)  (NAD_83(2011) will be used)
  30...NAD_83(2007)    (NAD_83(2011) will be used)
  2...NAD_83(PA11) (Pacific tectonic plate fixed)
  31...NAD_83(PACP00)  (NAD 83(PA11) will be used)
  3...NAD_83(MA11) (Mariana tectonic plate fixed)
  32...NAD_83(MARP00)  (NAD_83(MA11) will be used)

  4...WGS_72                             16...ITRF92
  5...WGS_84(transit) = NAD_83(2011)     17...ITRF93
  6...WGS_84(G730) = ITRF92              18...ITRF94 = ITRF96
  7...WGS_84(G873) = ITRF96              19...ITRF96
  8...WGS_84(G1150) = ITRF2000           20...ITRF97
  9...PNEOS_90 = ITRF90                  21...IGS97 = ITRF97
 10...NEOS_90 = ITRF90                   22...ITRF2000
 11...SIO/MIT_92 = ITRF91                23...IGS00 = ITRF2000
 12...ITRF88                             24...IGb00 = ITRF2000
 13...ITRF89                             25...ITRF2005
 14...ITRF90                             26...IGS05 = ITRF2005
 15...ITRF91                             27...ITRF2008
                                         28...IGS08 = ITRF2008
""")

    sys.exit(1)

#############################################################################
# Main

if __name__ == '__main__':

    # Default GDAL argument parsing.

    argv = gdal.GeneralCmdLineProcessor( sys.argv )
    if argv is None:
        sys.exit( 0 )

    if len(argv) == 1:
        Usage(brief=0)

    # Script argument defaults
    src_crs_id = None
    src_crs_date = None
    dst_crs_id = None
    dst_crs_date = None

    # Decent representation of continental US
    griddef = (-127.0, 50.0, -66.0, 25.0, 611, 251 )

    htdp_path = 'htdp'
    wrkdir = '.'
    kwf = 0
    output_grid_name = None

    # Script argument parsing.

    i = 1
    while  i < len(argv):

        if argv[i] == '-griddef' and i < len(argv)-6:
            griddef = (float(argv[i+1]),
                       float(argv[i+2]),
                       float(argv[i+3]),
                       float(argv[i+4]),
                       float(argv[i+5]),
                       float(argv[i+6]))
            i = i + 6

        elif argv[i] == '-htdp' and i < len(argv)-1:
            htdp_path = argv[i+1]
            i = i + 1

        elif argv[i] == '-kwf':
            kwf = 1

        elif argv[i] == '-wrkdir' and i < len(argv)-1:
            wrkdir = argv[i+1]
            i = i + 1

        elif argv[i] == '-o' and i < len(argv)-1:
            output_grid_name = argv[i+1]
            i = i + 1

        elif argv[i] == '-h' or argv[i] == '--help':
            Usage(brief=0)

        elif argv[i][0] == '-':
            print('Urecognised argument: ' + argv[i])
            Usage()

        elif src_crs_id is None:
            src_crs_id = int(argv[i])

        elif src_crs_date is None:
            src_crs_date = argv[i]

        elif dst_crs_id is None:
            dst_crs_id = int(argv[i])

        elif dst_crs_date is None:
            dst_crs_date = argv[i]

        else:
            print('Urecognised argument: ' + argv[i])
            Usage()

        i = i + 1
        # next argument

    if output_grid_name is None:
        print('Missing output grid name (-o)')
        Usage()

    if dst_crs_date is None:
        print('Source and Destination CRS Ids and Dates are mandatory, '
              'not all provided.')
        Usage()

    # Do a bit of validation of parameters.
    if src_crs_id < 1 or src_crs_id > 32 \
       or dst_crs_id < 1 or dst_crs_id > 32:
        print('Invalid source or destination CRS Id %d and %d.'
              % (src_crs_id, dst_crs_id))
        Usage(brief=0)

    if float(src_crs_date) < 1700.0 or float(src_crs_date) > 2300.0 \
       or float(dst_crs_date) < 1700.0 or float(dst_crs_date) > 2300.0:
        print('Source or destination CRS date seems odd %s and %s.' \
              % (src_crs_date, dst_crs_date))
        Usage(brief=0)

    # Prepare out set of working file names.

    in_grid_fn  = wrkdir + '/crs2crs_input.txt'
    out_grid_fn = wrkdir + '/crs2crs_output.txt'
    control_fn  = wrkdir + '/crs2crs_control.txt'

    # Write out the source grid file.
    grid = new_create_grid( griddef )

    print('Writing htdp input file: ' + in_grid_fn)
    write_grid( grid, in_grid_fn )
    write_control( control_fn, out_grid_fn, in_grid_fn,
                   src_crs_id, src_crs_date,
                   dst_crs_id, dst_crs_date )

    # Run htdp to transform the data.
    try:
      os.unlink( out_grid_fn )
    except:
      pass

    rc = os.system( htdp_path + ' < ' + control_fn )
    if rc != 0:
      print('htdp run failed!')
      sys.exit(1)

    print('htdp run complete.')

    print('Reading htdp output file: ' + out_grid_fn)
    adjustment = read_grid_crs_to_crs(in_grid_fn, out_grid_fn, grid.shape)

    # Convert shifts to radians
    adjustment = adjustment * (3.14159265358979323846 / 180.0)

    # write to output grid file.
    write_gdal_grid( output_grid_name, adjustment, griddef )

    # cleanup working files unless they have been requested to remain.
    if kwf == 0:
      os.unlink( in_grid_fn  )
      os.unlink( out_grid_fn )
      os.unlink( control_fn  )

    print('Processing complete: see ' + output_grid_name)
