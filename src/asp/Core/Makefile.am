# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_CORE

if HAVE_PKG_VW_BUNDLEADJUSTMENT

ba_headers = BundleAdjustUtils.h
ba_sources = BundleAdjustUtils.cc

endif

include_HEADERS = BlobIndexThreaded.h StereoSettings.h SparseView.h      \
                  InpaintView.h MedianFilter.h OrthoRasterizer.h         \
                  SoftwareRenderer.h ErodeView.h $(ba_headers) Macros.h  \
                  Common.h ThreadedEdgeMask.h

libaspCore_la_SOURCES = BlobIndexThreaded.cc SoftwareRenderer.cc         \
                  StereoSettings.cc $(ba_sources)

libaspCore_la_LIBADD = @MODULE_CORE_LIBS@

lib_LTLIBRARIES = libaspCore.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = . tests

includedir = $(prefix)/include/asp/Core

include $(top_srcdir)/config/rules.mak
