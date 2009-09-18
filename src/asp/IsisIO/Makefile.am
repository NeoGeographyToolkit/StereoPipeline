#########################################################################
# sources
#########################################################################

if MAKE_MODULE_ISISIO

include_HEADERS = DiskImageResourceIsis.h IsisAdjustCameraModel.h     \
		  IsisCameraModel.h

libaspIsisIO_la_SOURCES = DiskImageResourceIsis.cc                    \
		  IsisAdjustCameraModel.cc IsisCameraModel.cc

libaspIsisIO_la_LIBADD = @MODULE_ISISIO_LIBS@

lib_LTLIBRARIES = libaspIsisIO.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = .

includedir = $(prefix)/include/asp/IsisIO

include $(top_srcdir)/config/rules.mak