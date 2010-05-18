# __BEGIN_LICENSE__
# Copyright (C) 2006-2009 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_PHOTOMETRYTK

TestProjectFileIO_SOURCES      = TestProjectFileIO.cxx
TestProjectService_SOURCES     = TestProjectService.cxx
TestAlbedoAccumulators_SOURCES = TestAlbedoAccumulators.cxx
TestTimeAccumulators_SOURCES   = TestTimeAccumulators.cxx
TestErrorAccumulators_SOURCES  = TestErrorAccumulators.cxx
TestFullIteration_SOURCES      = TestFullIteration.cxx

TESTS = TestProjectFileIO TestProjectService TestAlbedoAccumulators   \
        TestTimeAccumulators TestErrorAccumulators TestFullIteration

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_PHOTOMETRYTK_LIBS@

check_PROGRAMS = $(TESTS)

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am