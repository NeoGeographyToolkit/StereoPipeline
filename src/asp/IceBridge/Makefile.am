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

if MAKE_APP_ICEBRIDGE
  bin_PROGRAMS     += correct_icebridge_l3_dem nav2cam
  libexec_PROGRAMS += qi2txt
  bin_SCRIPTS      += icebridge_kmz_to_csv.py lvis2kml.py extract_icebridge_ATM_points.py \
			process_icebridge_run.py fetch_icebridge_data.py icebridge_common.py

  correct_icebridge_l3_dem_SOURCES = correct_icebridge_l3_dem.cc
  correct_icebridge_l3_dem_LDADD = $(APP_ICEBRIDGE_LIBS)

  qi2txt_SOURCES = qi2txt.cc

  nav2cam_SOURCES = nav2cam.cc
  nav2cam_LDADD = $(APP_ICEBRIDGE_LIBS)
endif

if MAKE_APP_ORTHO2PINHOLE
  bin_PROGRAMS     += ortho2pinhole
  ortho2pinhole_SOURCES = ortho2pinhole.cc
  ortho2pinhole_LDADD = $(APP_ORTHO2PINHOLE_LIBS)
endif

# Scripts
##############################################################################

bin_SCRIPTS += 
all_scripts = $(bin_SCRIPTS)
all_scripts += $(libexec_SCRIPTS)
CLEANFILES = $(all_scripts)


##############################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS  = @ASP_LDFLAGS@

SUBDIRS = .

includedir = $(prefix)/include/asp/IceBridge

include $(top_srcdir)/config/rules.mak
