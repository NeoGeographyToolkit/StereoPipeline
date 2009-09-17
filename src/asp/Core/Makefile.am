#########################################################################
# sources
#########################################################################

if MAKE_MODULE_CORE

include_HEADERS = BaseEquation.h BlobIndexThreaded.h BundleAdjustUtils.h \
		  CameraAdjust.h ControlNetworkLoader.h			 \
		  DiskImageResourceDDD.h DiskImageResourceIsis.h	 \
		  Equation.h InpaintView.h IsisAdjustCameraModel.h	 \
		  IsisCameraModel.h OrthoRasterizer.h PolyEquation.h	 \
		  RPNEquation.h SoftwareRenderer.h SparseView.h		 \
		  SpiceUtilities.h StereoSettings.h			 \
		  TabulatedDataReader.h nff_terrain.h MedianFilter.h

libaspCore_la_SOURCES =	 BlobIndexThreaded.cc BundleAdjustUtils.cc	 \
		  ControlNetworkLoader.cc DiskImageResourceDDD.cc	 \
		  DiskImageResourceIsis.cc Equation.cc			 \
		  IsisAdjustCameraModel.cc IsisCameraModel.cc		 \
		  PolyEquation.cc RPNEquation.cc SoftwareRenderer.cc	 \
		  SpiceUtilities.cc StereoSettings.cc			 \
		  TabulatedDataReader.cc nff_terrain.cc

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