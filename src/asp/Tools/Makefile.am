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

bin_PROGRAMS =

if MAKE_APP_STEREO
  bin_PROGRAMS += stereo
  stereo_SOURCES = stereo.cc
  stereo_LDADD = $(APP_STEREO_LIBS) @PKG_CORE_LIBS@ @PKG_SESSIONS_LIBS@

if HAVE_PKG_ISIS
  stereo_LDADD += @PKG_ISISIO_LIBS@
endif
if HAVE_PKG_SPICE
  stereo_LDADD += @PKG_SPICEIO_LIBS@
endif
endif

if MAKE_APP_BUNDLEADJUST
  bin_PROGRAMS += bundle_adjust
  bundle_adjust_SOURCES = bundle_adjust.cc
  bundle_adjust_LDADD = $(APP_BUNDLEADJUST_LIBS) @PKG_CORE_LIBS@ @PKG_SESSIONS_LIBS@
if HAVE_PKG_ISIS
  bundle_adjust_LDADD += @PKG_ISISIO_LIBS@
endif
if HAVE_PKG_SPICE
  bundle_adjust_LDADD += @PKG_SPICEIO_LIBS@
endif
endif

if MAKE_APP_ORBITVIZ
  bin_PROGRAMS += orbitviz
  orbitviz_SOURCES = orbitviz.cc
  orbitviz_LDADD = $(APP_ORBITVIZ_LIBS) @PKG_CORE_LIBS@ @PKG_SESSIONS_LIBS@
if HAVE_PKG_ISIS
  orbitviz_LDADD += @PKG_ISISIO_LIBS@
endif
if HAVE_PKG_SPICE
  orbitviz_LDADD += @PKG_SPICEIO_LIBS@
endif
endif

if MAKE_APP_DISPARITYDEBUG
  bin_PROGRAMS += disparitydebug
  disparitydebug_SOURCES = disparitydebug.cc
  disparitydebug_LDADD = $(APP_DISPARITYDEBUG_LIBS)
endif

if MAKE_APP_POINT2MESH
  bin_PROGRAMS += point2mesh
  point2mesh_SOURCES = point2mesh.cc
  point2mesh_LDADD = $(APP_POINT2MESH_LIBS) @PKG_CORE_LIBS@
endif

if MAKE_APP_POINT2DEM
  bin_PROGRAMS += point2dem
  point2dem_SOURCES = point2dem.cc
  point2dem_LDADD = $(APP_POINT2DEM_LIBS) @PKG_CORE_LIBS@
endif

if MAKE_APP_CTXIMAGE
  bin_PROGRAMS += ctximage
  ctximage_SOURCES = ctximage.cc
  ctximage_LDADD = $(APP_CTXIMAGE_LIBS) @PKG_CORE_LIBS@
endif

if MAKE_APP_RMAX2CAHVOR
  bin_PROGRAMS += rmax2cahvor
  rmax2cahvor_SOURCES = rmax2cahvor.cc
  rmax2cahvor_LDADD = $(APP_RMAX2CAHVOR_LIBS) @PKG_CORE_LIBS@ @PKG_SESSIONS_LIBS@
endif

if MAKE_APP_RMAXADJUST
  bin_PROGRAMS += rmax_adjust
  rmax_adjust_SOURCES = rmax_adjust.cc
  rmax_adjust_LDADD = $(APP_RMAXADJUST_LIBS) @PKG_CORE_LIBS@ @PKG_SESSIONS_LIBS@
endif

if MAKE_APP_ISISADJUST
  bin_PROGRAMS += isis_adjust
  isis_adjust_SOURCES = isis_adjust.cc
  isis_adjust_LDADD = $(APP_ISISADJUST_LIBS) @PKG_CORE_LIBS@ @PKG_ISISIO_LIBS@
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
  orthoproject_SOURCES = orthoproject.cc
  orthoproject_LDADD = $(APP_ORTHOPROJECT_LIBS) @PKG_CORE_LIBS@ @PKG_SESSIONS_LIBS@
endif

if MAKE_APP_RESULTS
  bin_PROGRAMS +=results
  results_SOURCES = results.cc
  results_LDADD = $(APP_RESULTS_LIBS) @PKG_ISISIO_LIBS@
endif

if MAKE_APP_RECONSTRUCT
  bin_PROGRAMS += reconstruct
  reconstruct_SOURCES = reconstruct.cc
  reconstruct_LDADD = $(APP_RECONSTRUCT_LIBS)
endif

##############################################################################
#                             Tests                                          #
##############################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

include $(top_srcdir)/config/rules.mak
