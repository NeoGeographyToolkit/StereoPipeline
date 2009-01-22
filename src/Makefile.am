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
SUBDIRS = tests

bin_PROGRAMS=

ctx_code    = MRO/StereoSessionCTX.cc MRO/DiskImageResourceDDD.cc MRO/CTXEphemeris.cc MRO/CTXMetadata.cc
rmax_code   = RMAX/StereoSessionRmax.cc RMAX/RMAX.cc
bundle_code = BundleAdjustUtils.cc
stereo_code = HRSC/StereoSessionHRSC.cc HRSC/HRSC.cc MOC/StereoSessionMOC.cc    \
              MOC/Ephemeris.cc MOLAReader.cc MOC/Metadata.cc MOC/MOLA.cc        \
              StereoSession.cc StereoSessionKeypoint.cc StereoSessionPinhole.cc \
              TabulatedDataReader.cc KML.cc StereoSettings.cc SpiceUtilities.cc  \
              $(ctx_code) $(rmax_code) $(bundle_code)

if HAVE_PKG_ISIS
isis_code = Isis/StereoSessionIsis.cc Isis/IsisCameraModel.cc Isis/DiskImageResourceIsis.cc Isis/IsisAdjustCameraModel.cc
stereo_code += $(isis_code)
endif


if MAKE_APP_STEREO
  bin_PROGRAMS += stereo
  stereo_SOURCES = stereo.cc $(stereo_code)
  stereo_LDADD = $(APP_STEREO_LIBS)
endif

if MAKE_APP_STEREOGUI
  bin_PROGRAMS += stereo_gui
  stereo_gui_SOURCES = stereo_gui.cc					\
		   gui/MainWindow.cc gui/MainWindow.moc.cc		\
		   gui/MainWindow.h gui/InputWidget.cc			\
		   gui/InputWidget.moc.cc gui/InputWidget.h		\
		   gui/AlignmentWidget.cc gui/AlignmentWidget.moc.cc	\
		   gui/AlignmentWidget.h gui/PreviewGLWidget.cc		\
		   gui/PreviewGLWidget.moc.cc gui/PreviewGLWidget.h	\
		   gui/StereoGuiSession.cc gui/StereoGuiSession.moc.cc	\
		   gui/StereoGuiSession.h gui/ProgressBar.moc.cc	\
		   gui/ProgressBar.h gui/ProgressBar.cc $(stereo_code)
  stereo_gui_LDADD = $(APP_STEREOGUI_LIBS)

BUILT_SOURCES = gui/MainWindow.moc.cc
endif

if MAKE_APP_BUNDLEADJUST
  bin_PROGRAMS += bundle_adjust
  bundle_adjust_SOURCES = bundle_adjust.cc $(stereo_code)
  bundle_adjust_LDADD = $(APP_BUNDLEADJUST_LIBS)
endif

if MAKE_APP_ORBITVIZ
  bin_PROGRAMS += orbitviz
  orbitviz_SOURCES = orbitviz.cc $(stereo_code)
  orbitviz_LDADD = $(APP_ORBITVIZ_LIBS)
endif

if MAKE_APP_DISPARITYDEBUG
  bin_PROGRAMS += disparitydebug
  disparitydebug_SOURCES = disparitydebug.cc
  disparitydebug_LDADD = $(APP_DISPARITYDEBUG_LIBS)
endif

if MAKE_APP_POINT2MESH
  bin_PROGRAMS += point2mesh
  point2mesh_SOURCES = point2mesh.cc nff_terrain.cc
  point2mesh_LDADD = $(APP_POINT2MESH_LIBS)
endif

if MAKE_APP_POINT2DEM
  bin_PROGRAMS += point2dem
  point2dem_SOURCES = point2dem.cc DEM.cc SoftwareRenderer.cc
  point2dem_LDADD = $(APP_POINT2DEM_LIBS)
endif


if MAKE_APP_NURBS
  bin_PROGRAMS += nurbs
  nurbs_SOURCES = nurbs.cc
  nurbs_LDADD = $(APP_NURBS_LIBS)
endif

if MAKE_APP_CTXIMAGE
  bin_PROGRAMS += ctximage
  ctximage_SOURCES = ctximage.cc $(ctx_code)
  ctximage_LDADD = $(APP_CTXIMAGE_LIBS)
endif

if MAKE_APP_RMAX2CAHVOR
  bin_PROGRAMS += rmax2cahvor
  rmax2cahvor_SOURCES = rmax2cahvor.cc $(rmax_code)
  rmax2cahvor_LDADD = $(APP_RMAX2CAHVOR_LIBS)
endif

if MAKE_APP_RMAXADJUST
  bin_PROGRAMS += rmax_adjust
  rmax_adjust_SOURCES = rmax_adjust.cc $(rmax_code) $(bundle_code)
  rmax_adjust_LDADD = $(APP_RMAXADJUST_LIBS)
endif

if MAKE_APP_ISISADJUST
  bin_PROGRAMS += isis_adjust
  isis_adjust_SOURCES = isis_adjust.cc $(bundle_code) $(isis_code)
  isis_adjust_LDADD = $(APP_ISISADJUST_LIBS)
endif

if MAKE_APP_BUNDLEVIS
  bin_PROGRAMS += bundlevis
  bundlevis_SOURCES = bundlevis.cc
  bundlevis_LDADD = $(APP_BUNDLEVIS_LIBS)
endif

if MAKE_APP_POINT2MESH2
  bin_PROGRAMS += point2mesh2
  point2mesh2_SOURCES = point2mesh2.cc
  point2mesh2_LDADD = $(APP_POINT2MESH2_LIBS)
endif

if MAKE_APP_ORTHOPROJECT
  bin_PROGRAMS += orthoproject
  orthoproject_SOURCES = orthoproject.cc $(stereo_code)
  orthoproject_LDADD = $(APP_ORTHOPROJECT_LIBS)
endif

if MAKE_APP_RESULTS
  bin_PROGRAMS +=results
  results_SOURCES = results.cc
  results_LDADD = $(APP_RESULTS_LIBS)
endif

if MAKE_APP_CUDATEST
  bin_PROGRAMS += cuda_test
  cuda_test_SOURCES = cuda_code.cu cuda_test.cc
  cuda_test_LDADD = $(APP_CUDATEST_LIBS)
endif

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

##############################################################################
#                             Tests					     #
##############################################################################

include $(top_srcdir)/config/rules.mak
