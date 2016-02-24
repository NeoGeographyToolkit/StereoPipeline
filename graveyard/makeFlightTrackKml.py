#!/usr/bin/python

## __BEGIN_LICENSE__
## Copyright (C) 2006-2010 United States Government as represented by
## the Administrator of the National Aeronautics and Space Administration
## All Rights Reserved.
## __END_LICENSE__

"""
INFORMATION:
 This utility was created by Matthew Deans, David Lees and Tamar Cohen from NASA Ames Research Center
 initially for use in the Pavilion Lake Research Project.

 You must have python installed to run this utility.
 Invoke as follows:
 
Options:
 -i input; either a single nmea file or a directory containing many nmea files  (for directory no trailing slash)
 -o output: either a single kml file or a directory to hold many kml files (for directory no trailing slash)
 -v vehicle: one of ubcgav or udelgav
 -a for animated.  leave this off for a non-animated kml
 -s skip.  The Gavia nmea files have one row for every second which makes for large kml files.  Right now default is skipping to every 5th line (every 5 seconds).  You can increase this number to make smaller kml files.
 -c color.  This is for the color of the track.  Defaults to yellow for udel and orange for ubc

To convert one file:
 makeFlightTrackKml.py -i /full/path/to/input/file.nmea -o /full/path/to/output/file.kml -v vehicle -a

To convert many files:
 makeFlightTrackKml.py -i /full/path/to/many/input/files -o /full/path/to/where/you/want/outputs -v vehicle -a

"""

# Other standard Python system stuff
import glob
import os
from math import atan2 as atan2
from math import sqrt as sqrt
from copy import copy as copy
import optparse
import datetime

iconurl = ''

# in meters; This is for Pavilion lake.  Info is taken from Google Earth.
top_of_lake_elevation = 804.0

######################################################################
class location:
	def __init__(self):
		self.lat = 0
		self.lon = 0
		self.depth = 0
		self.north = 0
		self.east = 0
		self.elevation = top_of_lake_elevation
		return

######################################################################
def txt_to_deg( text ):
	fields = text.split(" ")
	deg = int( fields[0][1:] )
	min = float( fields[1] )
	deg = deg + min/60
	if (fields[0].find("S")):
		deg = -deg
	if (fields[0].find("W")):
		deg = -deg
	return deg

######################################################################
def is_nan(x):
	return type(x) is float and x != x

# default colors for udel or ubc.  Feel free to add your own.
######################################################################
def trackColorLookup(vehicle):
# Google Earth colors are AABBGGRR
	if 'gav' in vehicle:
		if 'ubc' in vehicle:
			return 'FF0077FF' # Orange
		if 'udel' in vehicle:
			return 'FF00FFFF' # Yellow
		
	# If nothing matches,
	return 'FFFFFFFF' # White

# Read the NMEA file and populate the sub_path points
######################################################################
def readNmeaFile(filename, skip=5):
	print "Reading nmea from file " + filename
	if filename is None:
		return

	num_points = 0
	sub_path = []

	# Open file handle
	f = open(filename,"r" )
	try:
		nav = location()
		old_east = 0
		old_north = 0
		cumulativeDistance = 0
		dist = 0
		lines = f.readlines()
		numlines = len(lines)
		for lineIndex in range(0, numlines, skip):  # skip by 5s
			line = lines[lineIndex]
			fields = line.split(',')

			# In these flight files, we have:
			# Northing, Easting, Lat, Lon, Morphology, Time
			if (len(fields)>5):
				month, day, year = fields[3].split("/")
				nav.date = '%s-%s-%s' % (year,month,day)
				nav.time = fields[4]
				hours = int(fields[4].split(':')[0])
				minutes = int(fields[4].split(':')[1])
				fseconds = float(fields[4].split(':')[2])
				seconds = int(fseconds)
				if seconds > 59:
					seconds = 59
				nav.timestamp = datetime.datetime(int(year), int(month), int(day), hours, minutes, seconds)

				nextLineIndex = lineIndex + skip
				if (nextLineIndex >= numlines):
					nextLineIndex = numlines - 1
				nextline = lines[nextLineIndex]
				nextfields = nextline.split(',')
				nexthours = int(nextfields[4].split(':')[0])
				nextminutes = int(nextfields[4].split(':')[1])
				fnextseconds = float(nextfields[4].split(':')[2])
				nextseconds = int(fnextseconds)
				if nextseconds > 59:
					nextseconds = 59
				nav.endtimestamp = datetime.datetime(int(year), int(month), int(day), nexthours, nextminutes, nextseconds)
					
				nav.north = float( fields[1][1:] )
				nav.east = float( fields[0][1:] )
				dx = nav.north - old_north
				dy = nav.east - old_east
				dist = sqrt(dx*dx+dy*dy)
				if (dist<10):
					cumulativeDistance += dist
				
				#if (abs(dx>0)):
				if (1):
					heading = atan2( dy, dx ) * 180 / 3.14159263
					if (heading<0):
						heading = heading + 360
					nav.heading = int( ( (heading)/45 ) + 0.5 )
					if (nav.heading==8):
						nav.heading = 0
				old_east = nav.east
				old_north = nav.north
				nav.lat = float( fields[5] )
				nav.lon = float(fields[6].split('*')[0])
				
				#depth, people!
				depth = fields[2][2:]
				floatDepth = float(depth)
				nav.elevation = top_of_lake_elevation + floatDepth
				if (not (is_nan(nav.lon))):
					sub_path.append( copy(nav) )
					num_points += 1

	finally:
		f.close()

	print 'Cumulative distance: %f' % cumulativeDistance
	print "got %d points" % (num_points)
	return sub_path

# Actually write the kml file
######################################################################
def writeKml(sub_path, filename, flightName, trackColor="FFFFFFFF", iconurl = None, animated = 'False'):
	print "Saving KML to " + filename

	FILE = open( filename, "w" )
	FILE.write( "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" )
	FILE.write( "<kml xmlns=\"http://earth.google.com/kml/2.2\">\n" )
	FILE.write( "<Document>\n" )
	if animated:
		FILE.write( "    <name>%s Animated Flight Track</name>\n" % flightName )
	else:
		FILE.write( "    <name>%s Flight Track</name>\n" % flightName )

	FILE.write( "    <Style>\n" )
	FILE.write( "        <ListStyle>\n" )
	FILE.write( "            <listItemType>checkHideChildren</listItemType>\n" )
	FILE.write( "" )
	FILE.write( "        </ListStyle>\n" )
	FILE.write( "    </Style>\n" )
	FILE.write( "    <Style id=\"dwnone\">\n" )
	FILE.write( "        <IconStyle>\n" )
	FILE.write( "            <scale>0.4</scale>\n" )
	FILE.write( "        </IconStyle>\n" )
	FILE.write( "    </Style>\n" )
	# This makes the styles for the icon pointing different directions
	if (iconurl):
		headings = [270,315,0,45,90,135,180,225]
		for index, heading in enumerate(headings):
			FILE.write( "    <Style id=\"dw%s\">\n" % index )
			FILE.write( "        <IconStyle>\n" )
			FILE.write( "            <scale>1</scale>\n" )
			FILE.write( "            <heading>%f</heading>\n" % heading )
			FILE.write( "            <Icon>\n" )
			FILE.write( "                <href>%s</href>\n" % (iconurl) )
			FILE.write( "            </Icon>\n" )
			FILE.write( "        </IconStyle>\n" )
			FILE.write( "    </Style>\n" )
	
	FILE.write( "    <Folder>\n" )
	FILE.write( "        <name>Trajectory</name>\n" )
	FILE.write( "        <open>1</open>\n" )
	FILE.write( "        <Placemark>\n" )
	FILE.write( "            <name>Trajectory</name>\n" )
	FILE.write( "            <Style>\n" )
	FILE.write( "                <LineStyle>\n" )
	FILE.write( "                    <color>%s</color>\n" % trackColor )
	FILE.write( "                    <width>3.0</width>\n" )
	FILE.write( "                </LineStyle>\n" )
	FILE.write( "            </Style>\n" )
	FILE.write( "            <LineString>\n" )
	FILE.write( "                <tessellate>1</tessellate>\n" )
	FILE.write( "                <coordinates>\n" )
	if (len(sub_path) > 0):
		print "Found sub_path %s points " % len(sub_path)
		for point in sub_path:
			FILE.write( str(point.lon) + "," + str(point.lat) + "," + str(point.elevation) + " " )
	else:
		print "NO SUB PATH POINTS FOR TRAJECTORY"
	FILE.write( "                </coordinates>\n" )
	FILE.write( "            </LineString>\n" )
	FILE.write( "        </Placemark>\n" )
	FILE.write( "    </Folder>\n" )

	if animated:
		FILE.write( "    <Folder>\n" )
		FILE.write( '      <visibility>1</visibility>\n')
		FILE.write( "        <name>TimeStamps</name>\n" )
		for point in sub_path:
			FILE.write( "        <Placemark>\n" )
			FILE.write( "            <TimeSpan>\n" )
			begin = point.timestamp
			FILE.write( "                <begin>%04d-%02d-%02dT%02d:%02d:%02dZ</begin>\n" % 
			            (begin.year, begin.month, begin.day, 
			             begin.hour, begin.minute, begin.second) )
			end = point.endtimestamp
			FILE.write( "                <end>%04d-%02d-%02dT%02d:%02d:%02dZ</end>\n" % 
			            (end.year, end.month, end.day, 
			             end.hour, end.minute, end.second) )
			FILE.write( "            </TimeSpan>\n" )
			FILE.write( "            <styleUrl>#dw%d</styleUrl>\n" % (point.heading) )
			FILE.write( "            <gx:balloonVisibility>1</gx:balloonVisibility>\n" )
			FILE.write( "            <Point>\n" )
			FILE.write( "                <coordinates>%f,%f,%f\n" % (point.lon,point.lat,point.elevation) )
			FILE.write( "                </coordinates>\n" )
			FILE.write( "            </Point>\n" )
			FILE.write( "        </Placemark>\n" )
		FILE.write( "    </Folder>\n" )

	# Close document
	FILE.write( "</Document>\n" )
	FILE.write( "</kml>\n" )
	FILE.close()
	return

######################################################################
# Main:
def main():
	parser = optparse.OptionParser('usage: %prog')
	parser.add_option('-i','--input',help='nmea input file OR directory containing nmea files (no slash at end of directory)')
	parser.add_option('-o','--output',help='output file OR directory for output files (no slash at end of directory)')
	parser.add_option('-v','--vehicle',help='vehicle (ubcgav,udelgav)')
	parser.add_option('-a','--animated',action="store_true",dest="animated",default=False,help='make animated with timestamps')
	parser.add_option('-s','--skip',help='skip to reading every nth line.  5 is default. 1 is min.')
	parser.add_option('-c','--color',help='color for the track.  AABBGGRR')
	opts,args = parser.parse_args()
	
	print 'input is %s' % opts.input
	print 'output is %s' % opts.output
	print 'vehicle is %s' % opts.vehicle
	print 'animated is %s' % opts.animated
	print 'skip is %s' % opts.skip
	print 'color is %s' % opts.color
	
	#Make sure the input exists
	input = opts.input
	if not os.path.exists(input):
		print 'Cannot find file %s' % input
		exit(-1)

	#Make sure the output is defined
	output = ''
	if (opts.output):
		output = opts.output
	else:
		print "OUTFILE REQUIRED."
		exit()

	#set up the skipping; defaults to 5.		
	skip = 5		
	if (opts.skip):
		skip = opts.skip
		if (skip < 1):
			skip = 1
			
	# Icon URL is derived from the vehicle argument and or flight number
	if ('gav' in opts.vehicle):
		if ( 'del' in opts.vehicle):
			iconurl = 'http://supercritical.civil.ubc.ca/pavilion/Icons/GaviaModel_UDel.png'
		if ('ubc' in opts.vehicle):
			iconurl = 'http://supercritical.civil.ubc.ca/pavilion/Icons/GaviaModel_UBC.png'
	else:
		print "Vehicle not specified as a gavia: no icon!"
		
	# passed in color overrides the default color
	if (opts.color):
		color = opts.color
	else:
		color = trackColorLookup(opts.vehicle)

	nmeaInputs = []
	if (os.path.basename(input)[-5:] == '.nmea'):
		nmeaInputs.append(input)
	else:
		globme = input + "/*.nmea"
		nmeaInputs = glob.glob(globme)
		
	singleOutput = None
	if (os.path.basename(output)[-4:] == '.kml'):
		singleOutput = output

	#do the work
	for nmeaFilename in nmeaInputs:		
		flightName = os.path.basename(nmeaFilename).split(".")[0]
		
		sub_path = readNmeaFile(nmeaFilename, skip)
				
		if (singleOutput):
			outfile = singleOutput
		else:
			outfile = output + "/" + flightName
			if (opts.animated):
				outfile = outfile + ".atrack"
			outfile = outfile + ".kml"
			
		if opts.animated:
			writeKml(sub_path, outfile, flightName, color, iconurl, opts.animated)
		else:
			writeKml(sub_path, outfile, flightName, color, iconurl, False)
	
		print '%s finished.' % flightName
if __name__ == '__main__':
    main()
