# __BEGIN_LICENSE__
# Copyright (C) 2006-2009 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_PHOTOMETRYTK

protocol_headers = ProjectFile.pb.h
protocol_sources = ProjectFile.pb.cc

BUILT_SOURCES          = $(protocol_sources)
nodist_include_HEADERS = $(protocol_headers)

include_HEADERS = ProjectFileIO.h ProjectService.h RemoteProjectFile.h
libaspPhotometryTK_la_SOURCES = $(protocol_sources) ProjectService.cc  \
                  RemoteProjectFile.cc

libaspPhotometryTK_la_LIBADD = @MODULE_PHOTOMETRYTK_LIBS@
lib_LTLIBRARIES = libaspPhotometryTK.la

if ENABLE_EXCEPTIONS

phodrg2plate_SOURCES  = phodrg2plate.cc
phodrg2plate_LDADD    = @MODULE_PHOTOMETRYTK_LIBS@

phoinitalbedo_SOURCES = phoinitalbedo.cc
phoinitalbedo_LDADD   = @MODULE_PHOTOMETRYTK_LIBS@

phoinitfile_SOURCES   = phoinitfile.cc
phoinitfile_LDADD     = @MODULE_PHOTOMETRYTK_LIBS@

phoinitet_SOURCES     = phoinitet.cc
phoinitet_LDADD       = @MODULE_PHOTOMETRYTK_LIBS@ libaspPhotometryTK.la

phoitalbedo_SOURCES   = phoitalbedo.cc
phoitalbedo_LDADD     = @MODULE_PHOTOMETRYTK_LIBS@

phoitdem_SOURCES      = phoitdem.cc
phoitdem_LDADD        = @MODULE_PHOTOMETRYTK_LIBS@

phoiterror_SOURCES    = phoiterror.cc
phoiterror_LDADD      = @MODULE_PHOTOMETRYTK_LIBS@

phoitreflect_SOURCES  = phoitreflect.cc
phoitreflect_LDADD    = @MODULE_PHOTOMETRYTK_LIBS@

phoittime_SOURCES     = phoittime.cc
phoittime_LDADD       = @MODULE_PHOTOMETRYTK_LIBS@

ptk_server_SOURCES    = ptk_server.cc
ptk_server_LDADD      = @MODULE_PHOTOMETRYTK_LIBS@ libaspPhotometryTK.la

bin_PROGRAMS = phodrg2plate phoinitalbedo phoinitfile phoitalbedo \
               phoitdem phoiterror phoitreflect phoittime         \
	       ptk_server phoinitet

endif


endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@

bin_SCRIPTS = phoinit.py phosolve.py

SUBDIRS = . tests
CLEANFILES = $(protocol_headers) $(protocol_sources)

includedir = $(prefix)/include/asp/PhotometryTK

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/thirdparty/protobuf.mak