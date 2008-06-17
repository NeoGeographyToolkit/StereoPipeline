# __BEGIN_LICENSE__
#
# Copyright (C) 2006 United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration
# (NASA).  All Rights Reserved.
# 
# This software is distributed under the NASA Open Source Agreement
# (NOSA), version 1.3.  The NOSA has been approved by the Open Source
# Initiative.  See the file COPYING at the top of the distribution
# directory tree for the complete NOSA document.
# 
# THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
# KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
# LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
# SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
# A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
# THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
# DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
#
# __END_LICENSE__

########################################################################
# sources
########################################################################
#include $(top_srcdir)/config/autotroll.mak

if HAVE_PKG_ISIS
isis_sources = Isis/StereoSessionIsis.cc Isis/IsisCameraModel.cc	\
               Isis/DiskImageResourceIsis.cc

endif

ctx_sources = MRO/StereoSessionCTX.cc MRO/DiskImageResourceDDD.cc \
			MRO/CTXEphemeris.cc MRO/CTXMetadata.cc

hrsc_sources = HRSC/StereoSessionHRSC.cc HRSC/HRSC.cc

apollo_sources = apollo/StereoSessionApolloMetric.cc

rmax_sources = RMAX/StereoSessionRmax.cc RMAX/RMAX.cc

moc_sources = MOC/StereoSessionMOC.cc MOC/Ephemeris.cc MOLAReader.cc	\
	  	 MOC/Metadata.cc MOC/MOLA.cc

generic_stereo_sources = file_lib.cc StereoSession.cc	\
	 	  	 StereoSessionKeypoint.cc	\
	 	  	 StereoSessionPinhole.cc	\
	 	  	 TabulatedDataReader.cc KML.cc	\
	 	  	 BundleAdjustUtils.cc
bin_PROGRAMS = 

if MAKE_APP_STEREO
  bin_PROGRAMS += stereo
  stereo_SOURCES = $(moc_sources) $(ctx_sources) $(hrsc_sources)	\
	 	   $(apollo_sources) $(generic_stereo_sources)		\
	 	   $(isis_sources) $(rmax_sources) SpiceUtilities.cc	\
	 	   stereo.cc
  stereo_LDFLAGS = $(APP_STEREO_LIBS) $(ASP_LDFLAGS)
endif

if MAKE_APP_BUNDLEADJUST
  bin_PROGRAMS += bundle_adjust
  bundle_adjust_SOURCES = $(moc_sources) $(ctx_sources)			\
	 	          $(hrsc_sources) $(apollo_sources)		\
	 	          $(generic_stereo_sources) $(isis_sources)	\
	 	          $(rmax_sources) SpiceUtilities.cc		\
	 	          bundle_adjust.cc
  bundle_adjust_LDFLAGS = $(APP_BUNDLEADJUST_LIBS) $(ASP_LDFLAGS)
endif

if MAKE_APP_ORBITVIZ
  bin_PROGRAMS += orbitviz
  orbitviz_SOURCES = $(moc_sources) $(ctx_sources) $(hrsc_sources)	\
	 	   $(apollo_sources) $(generic_stereo_sources)		\
	 	   $(isis_sources) SpiceUtilities.cc orbitviz.cc
  orbitviz_LDFLAGS = $(APP_ORBITVIZ_LIBS) $(ASP_LDFLAGS)
endif

#  bin_PROGRAMS += aspgui
#  aspgui_SOURCES = aspgui.cc gui/GlWidget.cc
#  aspgui_CXXFLAGS = $(QT_CXXFLAGS) $(QT_INCPATH)
#  aspgui_LDFLAGS = $(QT_LDFLAGS) $(QT_LIBS) $(APP_ASPGUI_LIBS)
#endif

if MAKE_APP_DISPARITYDEBUG
  bin_PROGRAMS += disparitydebug
  disparitydebug_SOURCES = disparitydebug.cc 
  disparitydebug_LDFLAGS = $(APP_DISPARITYDEBUG_LIBS)  $(ASP_LDFLAGS)
endif

if MAKE_APP_POINT2MESH
  bin_PROGRAMS += point2mesh
  point2mesh_SOURCES = point2mesh.cc nff_terrain.cc
  point2mesh_LDFLAGS = $(APP_POINT2MESH_LIBS)  $(ASP_LDFLAGS)
endif

if MAKE_APP_POINT2DEM
  bin_PROGRAMS += point2dem
  point2dem_SOURCES = point2dem.cc DEM.cc SoftwareRenderer.cc
  point2dem_LDFLAGS = $(APP_POINT2DEM_LIBS)  $(ASP_LDFLAGS)
endif


if MAKE_APP_NURBS
  bin_PROGRAMS += nurbs
  nurbs_SOURCES = nurbs.cc
  nurbs_LDFLAGS = $(APP_NURBS_LIBS) $(ASP_LDFLAGS)
endif

if MAKE_APP_CTXIMAGE
  bin_PROGRAMS += ctximage
  ctximage_SOURCES = ctximage.cc MRO/DiskImageResourceDDD.cc
  ctximage_LDFLAGS = $(APP_CTXIMAGE_LIBS) $(ASP_LDFLAGS)
endif

if MAKE_APP_RMAX2CAHVOR
  bin_PROGRAMS += rmax2cahvor
  rmax2cahvor_SOURCES = rmax2cahvor.cc RMAX/RMAX.cc
  rmax2cahvor_LDFLAGS = $(APP_RMAX2CAHVOR_LIBS) $(ASP_LDFLAGS)
endif

if MAKE_APP_RMAXADJUST
  bin_PROGRAMS += rmax_adjust
  rmax_adjust_SOURCES = rmax_adjust.cc RMAX/RMAX.cc BundleAdjustUtils.cc
  rmax_adjust_LDFLAGS = $(APP_RMAXADJUST_LIBS) $(ASP_LDFLAGS)
endif

if MAKE_APP_BUNDLEVIS
  bin_PROGRAMS += bundlevis
  bundlevis_SOURCES = bundlevis.cc
  bundlevis_LDFLAGS = $(APP_BUNDLEVIS_LIBS) $(ASP_LDFLAGS)
endif

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@ 

##############################################################################
#                             Tests	                                     #
##############################################################################

include $(top_srcdir)/config/rules.mak




