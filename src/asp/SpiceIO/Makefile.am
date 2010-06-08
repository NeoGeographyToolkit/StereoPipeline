# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_SPICEIO

include_HEADERS = SpiceUtilities.h TabulatedDataReader.h

libaspSpiceIO_la_SOURCES = SpiceUtilities.cc TabulatedDataReader.cc

libaspSpiceIO_la_LIBADD = @MODULE_SPICEIO_LIBS@

lib_LTLIBRARIES = libaspSpiceIO.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = .

includedir = $(prefix)/include/asp/SpiceIO

include $(top_srcdir)/config/rules.mak