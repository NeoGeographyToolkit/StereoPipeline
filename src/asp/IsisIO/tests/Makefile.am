# __BEGIN_LICENSE__
# Copyright (C) 2006-2009 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_ISISIO

TestIsisCameraModel_SOURCES       = TestIsisCameraModel.cxx
TestEphemerisEquations_SOURCES    = TestEphemerisEquations.cxx
TestIsisAdjustCameraModel_SOURCES = TestIsisAdjustCameraModel.cxx

TESTS = TestIsisCameraModel TestEphemerisEquations TestIsisAdjustCameraModel

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_ISISIO_LIBS@

check_PROGRAMS = $(TESTS)
#CLEANFILES = log.txt

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
