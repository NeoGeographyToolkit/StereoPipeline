# __BEGIN_LICENSE__
# Copyright (C) 2006-2009 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_CORE

TestErodeView_SOURCES       = TestErodeView.cxx

TESTS = TestErodeView

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_CORE_LIBS@

check_PROGRAMS = $(TESTS)

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
