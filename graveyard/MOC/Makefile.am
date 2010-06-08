# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

include_HEADERS = Ephemeris.h MOLA.h MOLAReader.h Metadata.h  \
		  StereoSessionMOC.h

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

includedir = $(prefix)/include/asp/Sessions/MOC

include $(top_srcdir)/config/rules.mak