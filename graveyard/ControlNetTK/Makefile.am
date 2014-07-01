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


#########################################################################
# sources
#########################################################################

if MAKE_MODULE_CONTROLNETTK

include_HEADERS = Equalization.h

#libaspControlNetTK_la_LIBADD = @MODULE_PHOTOMETRYTK_LIBS@
#lib_LTLIBRARIES = libaspControlNetTK.la

if ENABLE_EXCEPTIONS

CNETTK_LOCAL_LIBS = @MODULE_CONTROLNETTK_LIBS@

reduce_match_SOURCES = reduce_match.cc
reduce_match_LDADD   = $(CNETTK_LOCAL_LIBS)

cnet_build_SOURCES = cnet_build.cc
cnet_build_LDADD   = $(CNETTK_LOCAL_LIBS)

cnet_convert_SOURCES = cnet_convert.cc
cnet_convert_LDADD   = $(CNETTK_LOCAL_LIBS)

cnet_merge_SOURCES   = cnet_merge.cc
cnet_merge_LDADD     = $(CNETTK_LOCAL_LIBS)

isis_position_extract_SOURCES = isis_position_extract.cc
isis_position_extract_LDADD   = $(CNETTK_LOCAL_LIBS)

vw_position_extract_SOURCES = vw_position_extract.cc
vw_position_extract_LDADD   = $(CNETTK_LOCAL_LIBS)

render_gcp_SOURCES = render_gcp.cc
render_gcp_LDADD   = $(CNETTK_LOCAL_LIBS)

bin_PROGRAMS = reduce_match cnet_build cnet_convert cnet_merge render_gcp
libexec_PROGRAMS = isis_position_extract vw_position_extract

endif

endif

#########################################################################
# general
#########################################################################

AM_CPPFLAGS = @ASP_CPPFLAGS@
AM_LDFLAGS = @ASP_LDFLAGS@

bin_SCRIPTS = pairlist_all.py pairlist_seq.py pairlist_degree.py

includedir = $(prefix)/include/asp/ControlNetTK

include $(top_srcdir)/config/rules.mak