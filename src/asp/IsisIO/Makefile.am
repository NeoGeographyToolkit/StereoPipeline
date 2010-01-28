# __BEGIN_LICENSE__
# Copyright (C) 2006-2009 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_ISISIO

include_HEADERS = BaseEquation.h Equation.h PolyEquation.h            \
		  DiskImageResourceIsis.h IsisAdjustCameraModel.h     \
		  IsisCameraModel.h RPNEquation.h

libaspIsisIO_la_SOURCES = DiskImageResourceIsis.cc Equation.cc        \
		  IsisAdjustCameraModel.cc IsisCameraModel.cc         \
		  PolyEquation.cc RPNEquation.cc

libaspIsisIO_la_LIBADD = @MODULE_ISISIO_LIBS@

lib_LTLIBRARIES = libaspIsisIO.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = . tests

includedir = $(prefix)/include/asp/IsisIO

include $(top_srcdir)/config/rules.mak