# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

if MAKE_MODULE_CORE

TestBlobIndexThreaded_SOURCES  = TestBlobIndexThreaded.cxx
TestErodeView_SOURCES          = TestErodeView.cxx
TestGaussianClustering_SOURCES = TestGaussianClustering.cxx
TestInterestPointMatching_SOURCES = TestInterestPointMatching.cxx
TestThreadedEdgeMask_SOURCES   = TestThreadedEdgeMask.cxx


TESTS = TestErodeView TestBlobIndexThreaded TestThreadedEdgeMask \
        TestGaussianClustering TestInterestPointMatching

endif

########################################################################
# general
########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@ @PKG_CORE_LIBS@

check_PROGRAMS = $(TESTS)
EXTRA_DIST = ThreadTest1.tif ThreadTest2.tif ThreadTest3.tif

include $(top_srcdir)/config/rules.mak
include $(top_srcdir)/config/tests.am
