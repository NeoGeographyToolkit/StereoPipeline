# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
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

include_HEADERS = ProjectFileIO.h ProjectService.h RemoteProjectFile.h \
                  AlbedoAccumulators.h ImageAccumulators.h             \
                  ErrorAccumulators.h TimeAccumulators.h               \
                  ExtremePixvalAccumulator.h
libaspPhotometryTK_la_SOURCES = $(protocol_sources) ProjectService.cc  \
                  RemoteProjectFile.cc

libaspPhotometryTK_la_LIBADD = @MODULE_PHOTOMETRYTK_LIBS@
lib_LTLIBRARIES = libaspPhotometryTK.la

if ENABLE_EXCEPTIONS

PTK_LOCAL_LIBS = @MODULE_PHOTOMETRYTK_LIBS@ libaspPhotometryTK.la

phodrg2plate_SOURCES  = phodrg2plate.cc
phodrg2plate_LDADD    = $(PTK_LOCAL_LIBS)

phoinitfile_SOURCES   = phoinitfile.cc
phoinitfile_LDADD     = $(PTK_LOCAL_LIBS)

phoinitet_SOURCES     = phoinitet.cc
phoinitet_LDADD       = $(PTK_LOCAL_LIBS)

phoitalbedo_SOURCES   = phoitalbedo.cc
phoitalbedo_LDADD     = $(PTK_LOCAL_LIBS)

phoitdem_SOURCES      = phoitdem.cc
phoitdem_LDADD        = $(PTK_LOCAL_LIBS)

phoiterror_SOURCES    = phoiterror.cc
phoiterror_LDADD      = $(PTK_LOCAL_LIBS)

phoitreflect_SOURCES  = phoitreflect.cc
phoitreflect_LDADD    = $(PTK_LOCAL_LIBS)

phoittime_SOURCES     = phoittime.cc
phoittime_LDADD       = $(PTK_LOCAL_LIBS)

ptk_server_SOURCES    = ptk_server.cc
ptk_server_LDADD      = $(PTK_LOCAL_LIBS)

mipmap_SOURCES        = mipmap.cc
mipmap_LDADD          = $(PTK_LOCAL_LIBS)

shadow_mask_SOURCES   = shadow_mask.cc
shadow_mask_LDADD     = $(PTK_LOCAL_LIBS)

phoitnorm_SOURCES     = phoitnorm.cc
phoitnorm_LDADD       = $(PTK_LOCAL_LIBS)

bin_PROGRAMS = phodrg2plate phoinitfile phoitalbedo        \
               phoitdem phoiterror phoitreflect phoittime  \
               ptk_server phoinitet mipmap shadow_mask phoitnorm

endif


endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@

bin_SCRIPTS = phoinit.py phosolve.py pmipmap.py

SUBDIRS = . tests
CLEANFILES = $(protocol_headers) $(protocol_sources)

includedir = $(prefix)/include/asp/PhotometryTK

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/thirdparty/protobuf.mak