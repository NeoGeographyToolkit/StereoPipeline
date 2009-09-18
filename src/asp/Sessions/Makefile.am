
#########################################################################
# sources
#########################################################################

if MAKE_MODULE_SESSIONS

include_HEADERS = StereoSession.h

libaspSessions_la_SOURCES = StereoSession.cc HRSC/HRSC.cc		\
		  HRSC/StereoSessionHRSC.cc ISIS/StereoSessionIsis.cc	\
		  Keypoint/StereoSessionKeypoint.cc MOC/Ephemeris.cc	\
		  MOC/MOLA.cc MOC/MOLAReader.cc MOC/Metadata.cc		\
		  MOC/StereoSessionMOC.cc MRO/CTXEphemeris.cc		\
		  MRO/CTXMetadata.cc MRO/StereoSessionCTX.cc		\
		  Pinhole/StereoSessionPinhole.cc RMAX/RMAX.cc		\
		  RMAX/StereoSessionRmax.cc

libaspSessions_la_LIBADD = @MODULE_SESSIONS_LIBS@

lib_LTLIBRARIES = libaspSessions.la

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ -version-info @LIBTOOL_VERSION@

SUBDIRS = . HRSC ISIS Keypoint MOC MRO Pinhole RMAX

includedir = $(prefix)/include/asp/Sessions

include $(top_srcdir)/config/rules.mak