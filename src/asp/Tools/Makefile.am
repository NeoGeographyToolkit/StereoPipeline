# __BEGIN_LICENSE__
#  Copyright (c) 2009-2013, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__


########################################################################
# sources
########################################################################

bin_PROGRAMS =
bin_SCRIPTS =
libexec_SCRIPTS =
libexec_PROGRAMS = # Auxiliary C++ executables

if MAKE_APP_STEREO
  bin_SCRIPTS      += stereo parallel_stereo sparse_disp dg_mosaic
  libexec_SCRIPTS  += stereo_utils.py
  bin_PROGRAMS     += stereo_corr stereo_fltr stereo_pprc stereo_rfne stereo_blend
  libexec_PROGRAMS += stereo_parse
  stereo_corr_LDADD       = $(APP_STEREO_LIBS)
  stereo_corr_SOURCES     = stereo_corr.cc stereo.cc
  stereo_fltr_LDADD       = $(APP_STEREO_LIBS)
  stereo_fltr_SOURCES     = stereo_fltr.cc stereo.cc
  stereo_parse_LDADD      = $(APP_STEREO_LIBS)
  stereo_parse_SOURCES    = stereo_parse.cc stereo.cc
  stereo_pprc_LDADD       = $(APP_STEREO_LIBS)
  stereo_pprc_SOURCES     = stereo_pprc.cc stereo.cc
  stereo_rfne_LDADD       = $(APP_STEREO_LIBS)
  stereo_rfne_SOURCES     = stereo_rfne.cc stereo.cc
  stereo_blend_LDADD      = $(APP_STEREO_LIBS)
  stereo_blend_SOURCES    = stereo_blend.cc stereo.cc
  # bin_PROGRAMS += extract_camera_positions
  # extract_camera_positions_SOURCES = extract_camera_positions.cc
  # extract_camera_positions_LDADD   = $(APP_STEREO_LIBS)
endif

# Need this separate since it also depends on CERES
if MAKE_APP_STEREO_TRI
  bin_PROGRAMS        += stereo_tri
  stereo_tri_LDADD     = $(APP_STEREO_TRI_LIBS)
  stereo_tri_SOURCES   = stereo_tri.cc stereo.cc jitter_adjust.h jitter_adjust.cc \
                         ccd_adjust.h ccd_adjust.cc
endif

# The stereo_gui app is separate as it also depends on Qt
if MAKE_APP_STEREO_GUI
  bin_PROGRAMS += stereo_gui
  stereo_gui_SOURCES = stereo_gui.cc stereo.cc
  stereo_gui_LDADD = $(APP_STEREO_GUI_LIBS)
endif

if MAKE_APP_BUNDLE_ADJUST
  bin_PROGRAMS     += bundle_adjust 
  libexec_PROGRAMS += opencv_imagelist_creator
  libexec_PROGRAMS += opencv_calibrate 
  bin_SCRIPTS      += camera_calibrate camera_solve
  bundle_adjust_SOURCES            = bundle_adjust.cc bundle_adjust.h
  opencv_imagelist_creator_SOURCES = opencv_imagelist_creator.cpp
  opencv_calibrate_SOURCES         = opencv_calibrate.cpp
  bundle_adjust_LDADD            = $(APP_BUNDLE_ADJUST_LIBS)
  opencv_imagelist_creator_LDADD = $(APP_BUNDLE_ADJUST_LIBS)
  opencv_calibrate_LDADD         = $(APP_BUNDLE_ADJUST_LIBS)
endif

if MAKE_APP_SFS
  bin_PROGRAMS += sfs
  bin_SCRIPTS  += parallel_sfs
  sfs_SOURCES = sfs.cc
  sfs_LDADD = $(APP_SFS_LIBS)
endif

if MAKE_APP_DEM_MOSAIC
  bin_PROGRAMS += dem_mosaic
  dem_mosaic_SOURCES = dem_mosaic.cc
  dem_mosaic_LDADD   = $(APP_DEM_MOSAIC_LIBS)
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

if MAKE_APP_POINT2MESH
  bin_PROGRAMS += point2mesh
  point2mesh_SOURCES = point2mesh.cc
  point2mesh_LDADD = $(APP_POINT2MESH_LIBS)
endif

if MAKE_APP_POINT2LAS
  bin_PROGRAMS += point2las
  point2las_SOURCES = point2las.cc
  point2las_LDADD = $(APP_POINT2LAS_LIBS)
endif

if MAKE_APP_DEM_GEOID
  bin_PROGRAMS += dem_geoid
  dem_geoid_SOURCES = dem_geoid.cc
  dem_geoid_LDFLAGS = $(PKG_GEOID_LDFLAGS) $(AM_LDFLAGS)
  dem_geoid_LDADD = $(APP_DEM_GEOID_LIBS)
endif

if MAKE_APP_BUNDLEVIS
  bin_PROGRAMS += bundlevis
  bundlevis_SOURCES = bundlevis.cc bundlevis.h
  bundlevis_LDADD = $(APP_BUNDLEVIS_LIBS)
endif

if MAKE_APP_PC_ALIGN
  bin_PROGRAMS += pc_align
  pc_align_SOURCES = pc_align.cc
  pc_align_CPPFLAGS = $(AM_CPPFLAGS) $(OPENMPFLAGS)
  pc_align_LDFLAGS = $(AM_LDFLAGS) $(OPENMPFLAGS)
  pc_align_LDADD = $(APP_PC_ALIGN_LIBS)
endif

if MAKE_APP_PC_MERGE
  bin_PROGRAMS += pc_merge
  pc_merge_SOURCES = pc_merge.cc
  pc_merge_LDADD = $(APP_PC_MERGE_LIBS)
endif

if MAKE_APP_WV_CORRECT
  bin_PROGRAMS += wv_correct
  wv_correct_SOURCES = wv_correct.cc
  wv_correct_LDADD = $(APP_WV_CORRECT_LIBS)
endif

if MAKE_APP_GEODIFF
  bin_PROGRAMS += geodiff
  geodiff_SOURCES = geodiff.cc
  geodiff_LDADD = $(APP_GEODIFF_LIBS)
endif

if MAKE_APP_HSVMERGE
  bin_PROGRAMS += hsv_merge
  hsv_merge_SOURCES = hsv_merge.cc
  hsv_merge_LDADD = $(APP_HSVMERGE_LIBS)
endif

if MAKE_APP_MER2CAMERA
  bin_PROGRAMS += mer2camera
  mer2camera_SOURCES = mer2camera.cc
  mer2camera_LDADD   = $(APP_MER2CAMERA_LIBS)
endif

if MAKE_APP_MAPPROJECT
  libexec_PROGRAMS += mapproject_single
  bin_SCRIPTS  += mapproject
  mapproject_single_SOURCES = mapproject_single.cc
  mapproject_single_LDADD   = $(APP_MAPPROJECT_LIBS)
endif

if MAKE_APP_CAMERA_FOOTPRINT
  bin_PROGRAMS += camera_footprint
  camera_footprint_SOURCES = camera_footprint.cc
  camera_footprint_LDADD   = $(APP_CAMERA_FOOTPRINT_LIBS)
endif

if MAKE_APP_LRONACJITREG
  bin_PROGRAMS += lronacjitreg # not in libexec as to get a script wrapper
  lronacjitreg_LDADD = $(APP_LRONACJITREG_LIBS)
  lronacjitreg_SOURCES = lronacjitreg.cc
  bin_SCRIPTS  += lronac2mosaic.py
endif

if MAKE_APP_IMAGE_CALC
  bin_PROGRAMS += image_calc
  image_calc_SOURCES = image_calc.cc
  image_calc_LDADD   = $(APP_IMAGE_CALC_LIBS)
endif

if MAKE_APP_PANSHARP
  bin_PROGRAMS += pansharp
  pansharp_SOURCES = pansharp.cc
  pansharp_LDADD   = $(APP_PANSHARP_LIBS)
endif

if MAKE_APP_DATUM_CONVERT
  bin_PROGRAMS += datum_convert
  datum_convert_SOURCES = datum_convert.cc
  datum_convert_LDADD   = $(APP_DATUM_CONVERT_LIBS)
endif

if MAKE_APP_ASTER2ASP
  bin_PROGRAMS += aster2asp
  aster2asp_SOURCES = aster2asp.cc
  aster2asp_LDADD = $(APP_ASTER2ASP_LIBS)
endif

if MAKE_APP_CAM2RPC
  bin_PROGRAMS += cam2rpc
  cam2rpc_SOURCES = cam2rpc.cc
  cam2rpc_LDADD = $(APP_CAM2RPC_LIBS)
endif


# Auxiliary tools in libexec
##############################################################################

if MAKE_APP_RPC_GEN
  libexec_PROGRAMS += rpc_gen
  rpc_gen_SOURCES = rpc_gen.cc
  rpc_gen_LDADD   = $(APP_RPC_GEN_LIBS)

  # Just add this here, it has no extra dependencies.
  bin_PROGRAMS += add_spot_rpc
  add_spot_rpc_SOURCES = add_spot_rpc.cc
  add_spot_rpc_LDADD   = $(APP_ADD_SPOT_RPC_LIBS)  
endif

if MAKE_APP_TIF_MOSAIC
  libexec_PROGRAMS += tif_mosaic
  tif_mosaic_SOURCES = tif_mosaic.cc
  tif_mosaic_LDADD   = $(APP_TIF_MOSAIC_LIBS)
endif

# Scripts
##############################################################################

bin_SCRIPTS += time_trials cam2map4stereo.py hiedr2mosaic.py
all_scripts = $(bin_SCRIPTS)
all_scripts += $(libexec_SCRIPTS)
CLEANFILES = $(all_scripts)

# $@ = target file name
# $< = name of first prerequisite
# %  = matches any non-empty substring
$(all_scripts): %: %.in
	sed 's/\[@\]ASP_VERSION\[@\]/$(VERSION)/g' < $(srcdir)/$< > $@
	chmod +x $@


##############################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

SUBDIRS = .

includedir = $(prefix)/include/asp/Tools

include $(top_srcdir)/config/rules.mak
