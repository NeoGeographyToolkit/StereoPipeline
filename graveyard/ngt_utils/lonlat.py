## __BEGIN_LICENSE__
## Copyright (C) 2006-2010 United States Government as represented by
## the Administrator of the National Aeronautics and Space Administration
## All Rights Reserved.
## __END_LICENSE__

import sys
import math
import numpy

def w2e_deg(lon):
    '''Convert positive-west longitude to positive-east in the range [-180,180).'''
    lon = -lon
    if lon >= 180:
        lon -= 360
    if lon <= -180:
        lon += 360
    return lon

def pm180(lon):
    '''Convert latitudes to the range [-180,180).'''
    if lon >= 180:
        lon -= 360
    return lon

def lonlat_to_vec(lon, lat=None):
    if lat is None:
        lon, lat = lon
    x = math.cos(lat*math.pi/180) * math.cos(lon*math.pi/180)
    y = math.cos(lat*math.pi/180) * math.sin(lon*math.pi/180)
    z = math.sin(lat*math.pi/180)
    return numpy.array((x,y,z))

def vec_to_lonlat(vec):
    x, y, z = vec
    lat = 180/math.pi*math.atan2(z,math.sqrt(x**2+y**2))
    lon = 180/math.pi*math.atan2(y,x)
    return numpy.array((lon,lat))

def vec_norm(vec):
    return math.sqrt(numpy.dot(vec,vec))

def heading(pt1,pt2):
    '''Compute the heading from point 1 to point 2, in expressed as
    (lon,lat) in degrees.'''
    lon1,lat1 = pt1[0:2]
    lon2,lat2 = pt2[0:2]
    x1 = math.cos(lat1*math.pi/180) * math.cos(lon1*math.pi/180)
    y1 = math.cos(lat1*math.pi/180) * math.sin(lon1*math.pi/180)
    z1 = math.sin(lat1*math.pi/180)
    x2 = math.cos(lat2*math.pi/180) * math.cos(lon2*math.pi/180)
    y2 = math.cos(lat2*math.pi/180) * math.sin(lon2*math.pi/180)
    z2 = math.sin(lat2*math.pi/180)
    x1 = x2 + 0.0001 * (x1-x2)
    y1 = y2 + 0.0001 * (y1-y2)
    z1 = z2 + 0.0001 * (z1-z2)
    lat1 = 180/math.pi*math.atan2(z1,math.sqrt(x1*x1+y1*y1))
    lon1 = 180/math.pi*math.atan2(y1,x1)
    return 90-180/math.pi*math.atan2(lat2-lat1,(lon2-lon1)*math.cos(lat2*math.pi/180))

def interpolate_lonlat(ll1,ll2,n):
    '''Returns a list of points interpolating between two (lon,lat) points
    along a great circle.  If the third argument is an integer, this
    function returns that many points, uniformly spaced.  If the third
    argument is a list, the elements are interpreted as blend
    parameters, with 0 representing the first point and 1 representing
    the second point.  Passing in blend parameters outside the range
    [0,1] will result in extrapolation.'''
    if n.__class__ == list:
        alphas = n
    else:
        alphas = [float(i)/(n-1) for i in range(n)]
    lon1 = math.pi*ll1[0]/180
    lat1 = math.pi*ll1[1]/180
    lon2 = math.pi*ll2[0]/180
    lat2 = math.pi*ll2[1]/180
    v1 = numpy.array([math.cos(lon1)*math.cos(lat1), math.sin(lon1)*math.cos(lat1), math.sin(lat1)])
    assert abs(numpy.dot(v1,v1)-1) < 1e-8
    v2 = numpy.array([math.cos(lon2)*math.cos(lat2), math.sin(lon2)*math.cos(lat2), math.sin(lat2)])
    assert abs(numpy.dot(v2,v2)-1) < 1e-8
    axis = numpy.cross(v1,v2)
    assert abs(numpy.dot(v1,axis)) < 1e-8
    assert abs(numpy.dot(v2,axis)) < 1e-8
    axis = axis / math.sqrt(numpy.dot(axis,axis))
    assert abs(numpy.dot(v1,axis)) < 1e-8
    assert abs(numpy.dot(v2,axis)) < 1e-8
    assert abs(numpy.dot(axis,axis)-1) < 1e-8
    angle = math.acos(numpy.dot(v1,v2))
    ovec = numpy.cross(v1,axis)
    assert abs(numpy.dot(v1,ovec)) < 1e-8
    assert abs(numpy.dot(axis,ovec)) < 1e-8
    assert abs(numpy.dot(ovec,ovec)-1) < 1e-8
    M = numpy.array([v1,axis,ovec])
    assert numpy.allclose(numpy.dot(M,M.T),numpy.array([[1,0,0],[0,1,0],[0,0,1]]))
    values = []
    for alpha in alphas:
        c = math.cos(angle*alpha)
        s = math.sin(angle*alpha)
        R = numpy.array([[c,0,s],[0,1,0],[-s,0,c]])
        v = numpy.dot( numpy.dot( numpy.dot(M.T,R), M ), v1 )
        lon = 180/math.pi*math.atan2(v[1],v[0])
        lat = 180/math.pi*math.atan2(v[2],math.sqrt(v[0]**2+v[1]**2))
        values.append( (lon,lat) )
    return values

def dispatch_cmd(context, argv):
    '''A helper for building command-line tools.'''
    commands = {}
    for name, value in context.iteritems():
        if name.startswith('cmd_') and callable(value):
            commands[name[4:]] = value
    if len(argv) < 2 or argv[1] not in commands:
        if context['__doc__']:
            print context['__doc__'] + '\n'
        print 'Usage: ' + sys.argv[0] + ' <command>' + '\n'
        print 'Commands: '
        for command, func in commands.iteritems():
            print '  %s : %s' % (command, func.__doc__)
        sys.exit(1)
    commands[argv[1]](*argv[2:])
