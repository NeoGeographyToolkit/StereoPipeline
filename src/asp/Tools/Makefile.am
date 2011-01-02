# __BEGIN_LICENSE__
# Copyright (C) 2006-2010 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# sources
########################################################################

bin_PROGRAMS =

if MAKE_APP_STEREO
  bin_PROGRAMS += stereo_corr stereo_fltr stereo_pprc stereo_rfne stereo_tri
  stereo_corr_SOURCES = stereo_corr.cc
  stereo_corr_LDADD   = $(APP_STEREO_LIBS)
  stereo_fltr_SOURCES = stereo_fltr.cc
  stereo_fltr_LDADD   = $(APP_STEREO_LIBS)
  stereo_pprc_SOURCES = stereo_pprc.cc
  stereo_pprc_LDADD   = $(APP_STEREO_LIBS)
  stereo_rfne_SOURCES = stereo_rfne.cc
  stereo_rfne_LDADD   = $(APP_STEREO_LIBS)
  stereo_tri_SOURCES  = stereo_tri.cc
  stereo_tri_LDADD    = $(APP_STEREO_LIBS)
endif

if MAKE_APP_BUNDLEADJUST
  bin_PROGRAMS += bundle_adjust
  bundle_adjust_SOURCES = bundle_adjust.cc bundle_adjust.h
  bundle_adjust_LDADD = $(APP_BUNDLEADJUST_LIBS)
endif

if MAKE_APP_ORBITVIZ
  bin_PROGRAMS += orbitviz
  orbitviz_SOURCES = orbitviz.cc
  orbitviz_LDADD = $(APP_ORBITVIZ_LIBS)
endif

if MAKE_APP_DISPARITYDEBUG
  bin_PROGRAMS += disparitydebug
  disparitydebug_SOURCES = disparitydebug.cc
  disparitydebug_LDADD = $(APP_DISPARITYDEBUG_LIBS)
endif

if MAKE_APP_POINT2DEM
  bin_PROGRAMS += point2dem
  point2dem_SOURCES = point2dem.cc
  point2dem_LDADD = $(APP_POINT2DEM_LIBS)
endif

if MAKE_APP_RMAX2CAHVOR
  bin_PROGRAMS += rmax2cahvor
  rmax2cahvor_SOURCES = rmax2cahvor.cc
  rmax2cahvor_LDADD = $(APP_RMAX2CAHVOR_LIBS)
endif

if MAKE_APP_RMAXADJUST
  bin_PROGRAMS += rmax_adjust
  rmax_adjust_SOURCES = rmax_adjust.cc
  rmax_adjust_LDADD = $(APP_RMAXADJUST_LIBS)
endif

if MAKE_APP_ISISADJUST
  bin_PROGRAMS += isis_adjust
  isis_adjust_SOURCES = isis_adjust.cc isis_adjust.h
  isis_adjust_LDADD = $(APP_ISISADJUST_LIBS)
endif

if MAKE_APP_BUNDLEVIS
  bin_PROGRAMS += bundlevis
  bundlevis_SOURCES = bundlevis.cc bundlevis.h
  bundlevis_LDADD = $(APP_BUNDLEVIS_LIBS)
endif

if MAKE_APP_POINT2MESH
  bin_PROGRAMS += point2mesh
  point2mesh_SOURCES = point2mesh.cc
  point2mesh_LDADD = $(APP_POINT2MESH_LIBS)
endif

if MAKE_APP_ORTHOPROJECT
  bin_PROGRAMS += orthoproject
  orthoproject_SOURCES = orthoproject.cc
  orthoproject_LDADD = $(APP_ORTHOPROJECT_LIBS)
endif

if MAKE_APP_RESULTS
  bin_PROGRAMS +=results
  results_SOURCES = results.cc
  results_LDADD = $(APP_RESULTS_LIBS)
endif

if MAKE_APP_RECONSTRUCT
  bin_PROGRAMS += reconstruct
  reconstruct_SOURCES = reconstruct.cc
  reconstruct_LDADD = $(APP_RECONSTRUCT_LIBS)
endif

if MAKE_APP_ALIGNDEM
  bin_PROGRAMS += aligndem
  aligndem_SOURCES = aligndem.cc
  aligndem_LDADD = $(APP_ALIGNDEM_LIBS)
endif

if MAKE_APP_GEODIFF
  bin_PROGRAMS += geodiff
  geodiff_SOURCES = geodiff.cc
  geodiff_LDADD = $(APP_GEODIFF_LIBS)
endif

if MAKE_APP_DEMPROFILE
  bin_PROGRAMS += dem_profile
  dem_profile_SOURCES = dem_profile.cc
  dem_profile_LDADD = $(APP_DEMPROFILE_LIBS)
endif

if MAKE_APP_PLATEORTHOPROJECT
  bin_PROGRAMS += plateorthoproject
  plateorthoproject_SOURCES = plateorthoproject.cc
  plateorthoproject_LDADD = $(APP_PLATEORTHOPROJECT_LIBS)
endif

if MAKE_APP_HSVMERGE
  bin_PROGRAMS += hsv_merge
  hsv_merge_SOURCES = hsv_merge.cc
  hsv_merge_LDADD = $(APP_HSVMERGE_LIBS)
endif

if MAKE_APP_ISISADJUSTWRITE
  bin_PROGRAMS += isis_adjust_write
  isis_adjust_write_SOURCES = isis_adjust_write.cc
  isis_adjust_write_LDADD   = $(APP_ISISADJUSTWRITE_LIBS)
endif

if MAKE_APP_ISISADJUSTCAMERAERR
  bin_PROGRAMS += isis_adjust_camera_err
  isis_adjust_camera_err_SOURCES = isis_adjust_camera_err.cc
  isis_adjust_camera_err_LDADD   = $(APP_ISISADJUSTCAMERAERR_LIBS)
endif

if MAKE_APP_ISISADJUSTCNETCLIP
  bin_PROGRAMS += isis_adjust_cnet_clip
  isis_adjust_cnet_clip_SOURCES = isis_adjust_cnet_clip.cc
  isis_adjust_cnet_clip_LDADD   = $(APP_ISISADJUSTCNETCLIP_LIBS)
endif

##############################################################################
#                             Tests                                          #
##############################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

bin_SCRIPTS = cam2map4stereo.py hiedr2mosaic.py stereo

SUBDIRS = .

includedir = $(prefix)/include/asp/Tools

include $(top_srcdir)/config/rules.mak
