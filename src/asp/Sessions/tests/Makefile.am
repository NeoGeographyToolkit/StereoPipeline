# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_SESSIONS

TestStereoSessionDG_SOURCES = TestStereoSessionDG.cxx

TESTS = TestStereoSessionDG

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_SESSIONS_LIBS@

check_PROGRAMS = $(TESTS)

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
