# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_CONTROLNETTK

include_HEADERS = Equalization.h

#libaspControlNetTK_la_LIBADD = @MODULE_PHOTOMETRYTK_LIBS@
#lib_LTLIBRARIES = libaspControlNetTK.la

if ENABLE_EXCEPTIONS

CNETTK_LOCAL_LIBS = @MODULE_CONTROLNETTK_LIBS@

reduce_match_SOURCES = reduce_match.cc
reduce_match_LDADD   = $(CNETTK_LOCAL_LIBS)

cnet_build_SOURCES = cnet_build.cc
cnet_build_LDADD   = $(CNETTK_LOCAL_LIBS)

cnet_convert_SOURCES = cnet_convert.cc
cnet_convert_LDADD   = $(CNETTK_LOCAL_LIBS)

cnet_merge_SOURCES   = cnet_merge.cc
cnet_merge_LDADD     = $(CNETTK_LOCAL_LIBS)

isis_position_extract_SOURCES = isis_position_extract.cc
isis_position_extract_LDADD   = $(CNETTK_LOCAL_LIBS)

bin_PROGRAMS = reduce_match cnet_build cnet_convert cnet_merge
libexec_PROGRAMS = isis_position_extract

endif

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@

bin_SCRIPTS = pairlist_all.py pairlist_seq.py pairlist_degree.py

includedir = $(prefix)/include/asp/ControlNetTK

include $(top_srcdir)/config/rules.mak