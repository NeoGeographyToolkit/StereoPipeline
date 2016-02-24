## __BEGIN_LICENSE__
## Copyright (C) 2006-2010 United States Government as represented by
## the Administrator of the National Aeronautics and Space Administration
## All Rights Reserved.
## __END_LICENSE__

import math
import lonlat

MARS_MAJOR_RADIUS = 3396200.0
MARS_MINOR_RADIUS = 3376200.0
MARS_G2C_FACTOR = (MARS_MINOR_RADIUS/MARS_MAJOR_RADIUS)*(MARS_MINOR_RADIUS/MARS_MAJOR_RADIUS)

def mars_g2c_deg(lat):
    '''Convert a mars latitude in degrees from 'ographic to 'ocentric coordinates.'''
    return math.atan( MARS_G2C_FACTOR * math.tan(lat*math.pi/180) )*180/math.pi

def mars_w2e_deg(lon):
    '''Convert positive-west longitude to positive-east in the range [-180,180).'''
    return lonlat.w2e_deg(lon)

def mars_pm180(lon):
    '''Convert latitudes to the range [-180,180).'''
    return lonlat.pl180(lon)


