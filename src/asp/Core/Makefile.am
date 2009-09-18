#########################################################################
# sources
#########################################################################

if MAKE_MODULE_CORE

include_HEADERS = BaseEquation.h BlobIndexThreaded.h CameraAdjust.h      \
		  ControlNetworkLoader.h DiskImageResourceDDD.h          \
		  Equation.h InpaintView.h MedianFilter.h                \
		  OrthoRasterizer.h PolyEquation.h RPNEquation.h         \
		  SoftwareRenderer.h SparseView.h StereoSettings.h

libaspCore_la_SOURCES = BlobIndexThreaded.cc BundleAdjustUtils.cc        \
		  ControlNetworkLoader.cc DiskImageResourceDDD.cc        \
		  Equation.cc PolyEquation.cc RPNEquation.cc             \
		  SoftwareRenderer.cc StereoSettings.cc

if HAVE_PKG_OPENSCENEGRAPH
  include_HEADERS += nff_terrain.h
  libaspCore_la_SOURCES += nff_terrain.cc
endif

libaspCore_la_LIBADD = @MODULE_CORE_LIBS@

lib_LTLIBRARIES = libaspCore.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = .

includedir = $(prefix)/include/asp/Core

include $(top_srcdir)/config/rules.mak