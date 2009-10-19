
#########################################################################
# sources
#########################################################################

if MAKE_MODULE_SESSIONS

include_HEADERS = StereoSession.h

libaspSessions_la_SOURCES = StereoSession.cc \
		  Keypoint/StereoSessionKeypoint.cc \
		  Pinhole/StereoSessionPinhole.cc RMAX/RMAX.cc		\
		  RMAX/StereoSessionRmax.cc

libaspSessions_la_LIBADD = @MODULE_SESSIONS_LIBS@

lib_LTLIBRARIES = libaspSessions.la

SUBDIRS = . Keypoint Pinhole RMAX

if HAVE_PKG_ISIS
SUBDIRS += ISIS
libaspSessions_la_SOURCES += ISIS/StereoSessionIsis.cc
endif


endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

includedir = $(prefix)/include/asp/Sessions

include $(top_srcdir)/config/rules.mak
