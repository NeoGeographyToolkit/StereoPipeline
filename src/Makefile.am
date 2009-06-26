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
EXTRA_DIST=

bin_PROGRAMS=

rmax_code   = \
	RMAX/RMAX.cc \
	RMAX/RMAX.h \
	RMAX/StereoSessionRmax.cc \
	RMAX/StereoSessionRmax.h

bundle_code = \
	BundleAdjustUtils.cc \
	BundleAdjustUtils.h \
	ControlNetworkLoader.cc \
	ControlNetworkLoader.h

stereo_code = \
	KML.cc \
	KML.h \
	MedianFilter.h \
	MOLAReader.cc \
	MOLAReader.h \
	stereo.h \
	StereoSession.cc \
	StereoSession.h \
	StereoSessionKeypoint.cc \
	StereoSessionKeypoint.h \
	StereoSessionPinhole.cc \
	StereoSessionPinhole.h \
	StereoSettings.cc \
	StereoSettings.h \
	SurfaceNURBS.h \
	TabulatedDataReader.cc \
	TabulatedDataReader.h \
	Outliers.cc \
	Outliers.h \
	$(rmax_code) \
	$(bundle_code)

ctx_code = \
	MRO/DiskImageResourceDDD.cc \
	MRO/DiskImageResourceDDD.h

if HAVE_PKG_ISIS
eqn_code = \
	Isis/BaseEquation.h \
	Isis/Equation.cc \
	Isis/Equation.h \
	Isis/PolyEquation.cc \
	Isis/PolyEquation.h \
	Isis/RPNEquation.cc \
	Isis/RPNEquation.h

isis_code = \
	Isis/DiskImageResourceIsis.cc \
	Isis/DiskImageResourceIsis.h \
	Isis/IsisAdjustCameraModel.cc \
	Isis/IsisAdjustCameraModel.h \
	Isis/IsisCameraModel.cc \
	Isis/IsisCameraModel.h \
	$(eqn_code)

stereo_code += $(isis_code) Isis/StereoSessionIsis.cc Isis/StereoSessionIsis.h
endif

if HAVE_PKG_SPICE
spice_code  = SpiceUtilities.cc SpiceUtilities.h

ctx_code    += \
	MRO/CTXEphemeris.cc \
	MRO/CTXEphemeris.h \
	MRO/CTXMetadata.cc \
	MRO/CTXMetadata.h \
	MRO/StereoSessionCTX.cc \
	MRO/StereoSessionCTX.h

moc_code    = \
	MOC/Ephemeris.cc \
	MOC/Ephemeris.h \
	MOC/Metadata.cc \
	MOC/Metadata.h \
	MOC/MOLA.cc \
	MOC/MOLA.h \
	MOC/StereoSessionMOC.cc \
	MOC/StereoSessionMOC.h

hrsc_code = \
	HRSC/ExtoriExtrinsics.h \
	HRSC/HRSC.cc \
	HRSC/HRSC.h \
	HRSC/StereoSessionHRSC.cc \
	HRSC/StereoSessionHRSC.h
endif

stereo_code += $(spice_code) $(ctx_code) $(moc_code) $(hrsc_code)

if MAKE_APP_STEREO
  bin_PROGRAMS += stereo
  stereo_SOURCES = stereo.cc $(stereo_code)
  stereo_LDADD = $(APP_STEREO_LIBS)
endif

EXTRA_DIST+=gui/PreviewGLWidget.frag

if MAKE_APP_STEREOGUI
  bin_PROGRAMS += stereo_gui

  stereo_gui_SOURCES = \
	gui/AlignmentWidget.cc \
	gui/AlignmentWidget.h \
	gui/CorrelationWidget.cc \
	gui/CorrelationWidget.h \
	gui/CostFunctionWidget.cc \
	gui/CostFunctionWidget.h \
	gui/InputWidget.cc \
	gui/InputWidget.h \
	gui/KyleStereo.h \
	gui/MainWindow.cc \
	gui/MainWindow.h \
	gui/Noise.h \
	gui/PreprocessWidget.cc \
	gui/PreprocessWidget.h \
	gui/PreviewGLWidget.cc \
	gui/PreviewGLWidget.h \
	gui/QCompatFormLayout.h \
	gui/SearchWindowWidget.cc \
	gui/SearchWindowWidget.h \
	gui/StereoGuiSession.cc \
	gui/StereoGuiSession.h \
	$(stereo_code) \
	stereo_gui.cc

  nodist_stereo_gui_SOURCES = \
	gui/AlignmentWidget.moc.cc \
	gui/InputWidget.moc.cc \
	gui/MainWindow.moc.cc \
	gui/PreviewGLWidget.moc.cc \
	gui/StereoGuiSession.moc.cc

  stereo_gui_LDADD = $(APP_STEREOGUI_LIBS)

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
  point2mesh_SOURCES = point2mesh.cc nff_terrain.cc nff_terrain.h
  point2mesh_LDADD = $(APP_POINT2MESH_LIBS)
endif

if MAKE_APP_POINT2DEM
  bin_PROGRAMS += point2dem
  point2dem_SOURCES = point2dem.cc DEM.cc DEM.h SoftwareRenderer.cc SoftwareRenderer.h OrthoRasterizer.h
  point2dem_LDADD = $(APP_POINT2DEM_LIBS)
endif


if MAKE_APP_NURBS
  bin_PROGRAMS += nurbs
  nurbs_SOURCES = nurbs.cc SurfaceNURBS.h
  nurbs_LDADD = $(APP_NURBS_LIBS)
endif

if MAKE_APP_CTXIMAGE
  bin_PROGRAMS += ctximage
  ctximage_SOURCES = ctximage.cc MRO/DiskImageResourceDDD.cc MRO/DiskImageResourceDDD.h
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
  bundlevis_SOURCES = bundlevis.cc bundlevis.h
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
